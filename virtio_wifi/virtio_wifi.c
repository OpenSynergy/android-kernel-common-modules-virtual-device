// SPDX-License-Identifier: GPL-2.0+

/*
 * A virtio mac80211 WLAN driver that intendes to facilitates
 * Wifi TX/RX between guest system and QEMU emulator.
 */

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_types.h>
#include <net/mac80211.h>

#include "virtio_wifi.h"

#define VIRTIO_WIFI_RX_PADDING (NET_IP_ALIGN + NET_SKB_PAD)
#define IEEE80211_HEADER_MIN_LEN 24
#define IEEE80211_HEADER_LEN 32

#define CHAN2G(_freq) { \
	.band = NL80211_BAND_2GHZ, \
	.center_freq = (_freq), \
	.hw_value = (_freq), \
	.max_power = 20, \
}

#define CHAN5G(_freq) { \
	.band = NL80211_BAND_5GHZ, \
	.center_freq = (_freq), \
	.hw_value = (_freq), .max_power = 20, \
}

static const int napi_weight = NAPI_POLL_WEIGHT;

static const struct ieee80211_channel virtio_wifi_channels_2ghz[] = {
	CHAN2G(2412), /* Channel 1 */
	CHAN2G(2417), /* Channel 2 */
	CHAN2G(2422), /* Channel 3 */
	CHAN2G(2427), /* Channel 4 */
	CHAN2G(2432), /* Channel 5 */
	CHAN2G(2437), /* Channel 6 */
	CHAN2G(2442), /* Channel 7 */
	CHAN2G(2447), /* Channel 8 */
	CHAN2G(2452), /* Channel 9 */
	CHAN2G(2457), /* Channel 10 */
	CHAN2G(2462), /* Channel 11 */
	CHAN2G(2467), /* Channel 12 */
	CHAN2G(2472), /* Channel 13 */
	CHAN2G(2484), /* Channel 14 */
};

static const struct ieee80211_channel virtio_wifi_channels_5ghz[] = {
	CHAN5G(5180), /* Channel 36 */
};

static const struct ieee80211_rate virtio_wifi_rates[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60 },
	{ .bitrate = 90 },
	{ .bitrate = 120 },
	{ .bitrate = 180 },
	{ .bitrate = 240 },
	{ .bitrate = 360 },
	{ .bitrate = 480 },
	{ .bitrate = 540 }
};

struct virtio_wifi_config {
	/* The config defining mac address */
	__u8 mac[ETH_ALEN];
	/* Default maximum transmit unit advice */
	__u16 mtu;

} __packed;

/* Internal representation of a send virtqueue */
struct send_queue {
	/* Virtqueue associated with this send _queue */
	struct virtqueue *vq;

	/* TX: fragments + linear part + virtio header */
	struct scatterlist sg[MAX_SKB_FRAGS + 2];

	struct napi_struct napi;
};

/* Internal representation of a receive virtqueue */
struct receive_queue {
	/* Virtqueue associated with this receive_queue */
	struct virtqueue *vq;

	/* RX: fragments + linear part + virtio header */
	struct scatterlist sg[MAX_SKB_FRAGS + 2];

	/* Page frag for packet buffer allocation. */
	struct page_frag alloc_frag;

	struct napi_struct napi;
};

struct virtio_wifi_info {
	struct virtio_device *vdev;
	struct ieee80211_hw *hw;
	struct virtqueue *cvq;
	struct send_queue *sq_sta;
	struct send_queue *sq_p2p;
	struct receive_queue *rq_sta;
	struct net_device napi_dev;
	unsigned int status;
	int hdr_len;
	int max_queue_pairs;
	unsigned int rx_filter;
	struct mutex mutex;
	struct ieee80211_channel channels_2ghz[
			ARRAY_SIZE(virtio_wifi_channels_2ghz)];
	struct ieee80211_channel channels_5ghz[
			ARRAY_SIZE(virtio_wifi_channels_5ghz)];
	struct ieee80211_rate rates[
			ARRAY_SIZE(virtio_wifi_rates)];
	struct ieee80211_supported_band bands[NUM_NL80211_BANDS];
	struct ieee80211_channel *channel;
	bool started;
	bool associated;
	/* Work struct for refilling if we run low on memory. */
	struct delayed_work refill_work;
	struct hrtimer beacon_timer;
	/* beacon interval in us */
	u64 beacon_int;
};

static inline u64 virtio_wifi_get_tsf_raw(void)
{
	return ktime_to_us(ktime_get_boottime());
}

static void virtqueue_napi_schedule(struct napi_struct *napi,
					struct virtqueue *vq)
{
	if (napi_schedule_prep(napi)) {
		virtqueue_disable_cb(vq);
		__napi_schedule(napi);
	}
}

static void virtqueue_napi_complete(struct napi_struct *napi,
					struct virtqueue *vq,
					int processed)
{
	int opaque;

	opaque = virtqueue_enable_cb_prepare(vq);
	if (napi_complete_done(napi, processed)) {
		if (unlikely(virtqueue_poll(vq, opaque)))
			virtqueue_napi_schedule(napi, vq);
	} else
		virtqueue_disable_cb(vq);
}

static void virtio_wifi_napi_enable(struct virtqueue *vq,
					struct napi_struct *napi)
{
	napi_enable(napi);

	/* If all buffers were filled by other side before we napi_enabled, we
	 * won't get another interrupt, so process any outstanding packets now.
	 * Call local_bh_enable after to trigger softIRQ processing.
	 */
	local_bh_disable();
	virtqueue_napi_schedule(napi, vq);
	local_bh_enable();
}

static void free_old_xmit_skbs(struct send_queue *sq)
{
	struct sk_buff *skb = NULL;
	unsigned int len;

	while ((skb = virtqueue_get_buf(sq->vq, &len)) != NULL)
		dev_consume_skb_any(skb);
}

static int xmit_skb(struct send_queue *sq, struct sk_buff *skb)
{
	int num_sg;

	sg_init_table(sq->sg, skb_shinfo(skb)->nr_frags + 1);
	num_sg = skb_to_sgvec(skb, sq->sg, 0, skb->len);
	if (unlikely(num_sg < 0))
		return num_sg;

	return virtqueue_add_outbuf(sq->vq, sq->sg, num_sg, skb, GFP_ATOMIC);
}

static void virtio_wifi_tx(struct ieee80211_hw *hw,
			   struct ieee80211_tx_control *control,
			   struct sk_buff *skb)
{
	struct virtio_wifi_info *vi = hw->priv;
	struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
	struct ieee80211_channel *channel;
	int err;
	struct send_queue *sq;
	bool kick = !netdev_xmit_more();

	channel = vi->channel;
	if (WARN_ON(skb->len < 10) || !vi->started) {
		ieee80211_free_txskb(hw, skb);
		return;
	}

	if (WARN(!channel, "TX w/o channel - queue = %d\n", txi->hw_queue)) {
		ieee80211_free_txskb(hw, skb);
		return;
	}

	if (ieee80211_hw_check(hw, SUPPORTS_RC_TABLE))
		ieee80211_get_tx_rates(txi->control.vif, control->sta, skb,
				   txi->control.rates,
				   ARRAY_SIZE(txi->control.rates));

	if (txi->control.vif->type == NL80211_IFTYPE_STATION)
		sq = vi->sq_sta;
	else
		sq = vi->sq_p2p;
	free_old_xmit_skbs(sq);
	err = xmit_skb(sq, skb);
	if (unlikely(err)) {
		dev_kfree_skb_any(skb);
		return;
	}
	ieee80211_tx_info_clear_status(txi);
	/* frame was transmitted at most favorable rate at first attempt */
	txi->control.rates[0].count = 1;
	txi->control.rates[1].idx = -1;
	// Always assume that tx is acked.
	txi->flags |= IEEE80211_TX_STAT_ACK;
	ieee80211_tx_status_noskb(hw, control->sta, txi);
	skb_orphan(skb);
	nf_reset_ct(skb);

	/*
	 * If running out of space, stop queue to avoid getting packets that we
	 * are then unable to transmit.
	 */
	if (sq->vq->num_free < 2 + MAX_SKB_FRAGS)
		ieee80211_stop_queues(hw);

	if (kick)
		virtqueue_kick(sq->vq);
}

/*
 * Returns false if we couldn't fill entirely (OOM).
 *
 * Normally run in the receive path, but can also be run from virtio_wifi_start
 * before we're receiving packets, or from refill_work which is
 * careful to disable receiving (using napi_disable).
 */
static bool try_fill_recv(struct virtio_wifi_info *vi,
			  struct receive_queue *rq,
			  gfp_t gfp)
{
	int err, len;
	bool oom;
	char *buf;
	struct page_frag *alloc_frag;

	do {
		alloc_frag = &rq->alloc_frag;
		len = VIRTIO_WIFI_RX_PADDING + IEEE80211_MAX_FRAME_LEN;

		len = SKB_DATA_ALIGN(len) + SKB_DATA_ALIGN(
				sizeof(struct skb_shared_info));
		if (unlikely(!skb_page_frag_refill(len, alloc_frag, gfp))) {
			err = -ENOMEM;
			return false;
		}
		buf = (char *)page_address(alloc_frag->page) +
				alloc_frag->offset;
		get_page(alloc_frag->page);
		alloc_frag->offset += len;
		sg_init_one(rq->sg, buf + VIRTIO_WIFI_RX_PADDING,
				vi->hdr_len + IEEE80211_MAX_FRAME_LEN);
		err = virtqueue_add_inbuf(rq->vq, rq->sg, 1, buf, gfp);
		if (err < 0)
			put_page(virt_to_head_page(buf));
		if (err)
			return false;
	} while (rq->vq->num_free);
	virtqueue_kick(rq->vq);
	oom = err == -ENOMEM;

	return !oom;
}

static int virtio_wifi_start(struct ieee80211_hw *hw)
{
	struct virtio_wifi_info *vi = hw->priv;

	mutex_lock(&vi->mutex);
	vi->started = true;
	mutex_unlock(&vi->mutex);

	if (!try_fill_recv(vi, vi->rq_sta, GFP_KERNEL))
		schedule_delayed_work(&vi->refill_work, 0);

	virtio_wifi_napi_enable(vi->rq_sta->vq, &vi->rq_sta->napi);
	return 0;
}

static void virtio_wifi_stop(struct ieee80211_hw *hw)
{
	struct virtio_wifi_info *vi = hw->priv;

	mutex_lock(&vi->mutex);
	vi->started = false;
	mutex_unlock(&vi->mutex);
	cancel_delayed_work_sync(&vi->refill_work);
	napi_disable(&vi->rq_sta->napi);
	hrtimer_cancel(&vi->beacon_timer);
}

static int virtio_wifi_add_interface(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif)
{
	struct virtio_wifi_info *vi = hw->priv;
	int ret = 0;

	mutex_lock(&vi->mutex);
	vif->cab_queue = 0;
	vif->hw_queue[IEEE80211_AC_VO] = 0;
	vif->hw_queue[IEEE80211_AC_VI] = 1;
	vif->hw_queue[IEEE80211_AC_BE] = 2;
	vif->hw_queue[IEEE80211_AC_BK] = 3;
	mutex_unlock(&vi->mutex);
	return ret;
}

static int virtio_wifi_change_interface(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					enum nl80211_iftype newtype,
					bool newp2p)
{
	/*
	 * interface may change from non-AP to AP in
	 * which case this needs to be set up again
	 */
	vif->cab_queue = 0;

	return 0;
}

static int virtio_wifi_hwconfig(struct ieee80211_hw *hw, u32 changed)
{
	// TODO (wdu@) Implement channel switch based on hw->conf
	struct virtio_wifi_info *vi = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;

	mutex_lock(&vi->mutex);
	vi->channel = conf->chandef.chan;
	mutex_unlock(&vi->mutex);
	if (!vi->started || !vi->beacon_int)
		hrtimer_cancel(&vi->beacon_timer);
	else if (!hrtimer_is_queued(&vi->beacon_timer)) {
		u64 tsf = virtio_wifi_get_tsf_raw();
		u32 bcn_int = vi->beacon_int;
		u64 until_tbtt = bcn_int - do_div(tsf, bcn_int);

		hrtimer_start(&vi->beacon_timer, ns_to_ktime(until_tbtt * 1000),
				HRTIMER_MODE_REL);
	}

	return 0;
}

static void virtio_wifi_bss_info_changed(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 struct ieee80211_bss_conf *info,
					 u32 changed)
{
	struct virtio_wifi_info *vi = hw->priv;

	if (changed & BSS_CHANGED_ASSOC) {
		mutex_lock(&vi->mutex);
		if (info->assoc)
			vi->associated = true;
		else
			vi->associated = false;
		mutex_unlock(&vi->mutex);
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED) {
		if (vi->started && !hrtimer_is_queued(&vi->beacon_timer) &&
			info->enable_beacon) {
			u64 tsf, until_tbtt;
			u32 bcn_int;

			vi->beacon_int = info->beacon_int * 1024;
			tsf = virtio_wifi_get_tsf_raw();
			bcn_int = vi->beacon_int;
			until_tbtt = bcn_int - do_div(tsf, bcn_int);
			hrtimer_start(&vi->beacon_timer, ns_to_ktime(
					until_tbtt * 1000), HRTIMER_MODE_REL);
		}
	}
}

static void virtio_wifi_remove_interface(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif)
{
	// Un-implemented
}

static void virtio_wifi_configure_filter(struct ieee80211_hw *hw,
					 unsigned int changed_flags,
					 unsigned int *total_flags,
					 u64 multicast)
{
	struct virtio_wifi_info *vi = hw->priv;

	vi->rx_filter = 0;
	if (*total_flags & FIF_ALLMULTI)
		vi->rx_filter |= FIF_ALLMULTI;
	*total_flags = vi->rx_filter;
}

static void virtio_wifi_flush(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  u32 queues,
				  bool drop)
{
	// Un-implemented
}

static int virtio_wifi_ampdu_action(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					struct ieee80211_ampdu_params *params)
{
	struct ieee80211_sta *sta = params->sta;
	enum ieee80211_ampdu_mlme_action action = params->action;
	u16 tid = params->tid;

	switch (action) {
	case IEEE80211_AMPDU_TX_START:
		ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
	break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
	break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
	break;
	case IEEE80211_AMPDU_RX_START:
	case IEEE80211_AMPDU_RX_STOP:
	break;
	default:
	return -EOPNOTSUPP;
	}
	return 0;
}

static const struct ieee80211_ops virtio_wifi_ops = {
	.start = virtio_wifi_start,
	.stop = virtio_wifi_stop,
	.tx = virtio_wifi_tx,
	.add_interface = virtio_wifi_add_interface,
	.change_interface = virtio_wifi_change_interface,
	.remove_interface = virtio_wifi_remove_interface,
	.config = virtio_wifi_hwconfig,
	.bss_info_changed = virtio_wifi_bss_info_changed,
	.configure_filter = virtio_wifi_configure_filter,
	.flush = virtio_wifi_flush,
	.ampdu_action = virtio_wifi_ampdu_action,
};

static void virtio_wifi_refill_work(struct work_struct *work)
{
	struct virtio_wifi_info *vi =
		container_of(work, struct virtio_wifi_info, refill_work.work);
	bool still_empty;

	napi_disable(&vi->rq_sta->napi);
	still_empty = !try_fill_recv(vi, vi->rq_sta, GFP_KERNEL);
	virtio_wifi_napi_enable(vi->rq_sta->vq, &vi->rq_sta->napi);
	/* In theory, this can happen: if we don't get any buffers in
	 * we will *never* try to fill again.
	 */
	if (still_empty)
		schedule_delayed_work(&vi->refill_work, HZ / 2);
}

static int receive_buf(struct virtio_wifi_info *vi,
			   struct receive_queue *rq,
			   void *buf,
			   unsigned int len)
{
	struct ieee80211_rx_status rx_status;
	struct ieee80211_hdr *hdr;
	unsigned int header_offset = VIRTIO_WIFI_RX_PADDING;
	unsigned int headroom = vi->hdr_len + header_offset;
	unsigned int buflen = SKB_DATA_ALIGN(IEEE80211_MAX_FRAME_LEN);
	struct page *page = virt_to_head_page(buf);
	struct sk_buff *skb = build_skb(buf, buflen);

	if (!skb) {
		put_page(page);
		return -1;
	}

	skb_reserve(skb, headroom);
	skb_put(skb, len);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->protocol = htons(ETH_P_802_2);
	hdr = (void *)skb->data;

	if (skb->len < IEEE80211_HEADER_MIN_LEN) {
		put_page(page);
		return 0;
	}

	if (skb->len >= IEEE80211_HEADER_LEN &&
			ieee80211_is_probe_resp(hdr->frame_control)) {
		struct ieee80211_mgmt *mgmt =
				(struct ieee80211_mgmt *)skb->data;

		mgmt->u.probe_resp.timestamp =
				cpu_to_le64(virtio_wifi_get_tsf_raw());
	}
	memset(&rx_status, 0, sizeof(rx_status));
	rx_status.flag |= RX_FLAG_MACTIME_END;
	rx_status.mactime = virtio_wifi_get_tsf_raw();
	rx_status.freq = vi->channel->center_freq;
	rx_status.band = vi->channel->band;
	rx_status.bw = RATE_INFO_BW_20;
	rx_status.signal = -50;
	memcpy(IEEE80211_SKB_RXCB(skb), &rx_status, sizeof(rx_status));
	ieee80211_rx(vi->hw, skb);
	return len;
}

static int virtio_wifi_receive(struct receive_queue *rq, int budget)
{
	struct virtio_wifi_info *vi = rq->vq->vdev->priv;
	unsigned int len, received = 0;
	void *buf;

	while (received < budget &&
	   (buf = virtqueue_get_buf(rq->vq, &len)) != NULL) {
		if (vi->started)
			receive_buf(vi, rq, buf, len);
		received++;
	}
	if (rq->vq->num_free > virtqueue_get_vring_size(rq->vq) / 2) {
		if (!try_fill_recv(vi, rq, GFP_ATOMIC))
			schedule_delayed_work(&vi->refill_work, 0);
	}
	return received;
}

static int virtio_wifi_poll(struct napi_struct *napi, int budget)
{
	struct receive_queue *rq = container_of(napi,
			struct receive_queue, napi);
	struct virtio_wifi_info *vi = rq->vq->vdev->priv;
	unsigned int received;

	if (vi->sq_sta->vq->num_free >= 2 + MAX_SKB_FRAGS &&
			vi->sq_p2p->vq->num_free >= 2 + MAX_SKB_FRAGS)
		ieee80211_wake_queues(vi->hw);
	received = virtio_wifi_receive(rq, budget);

	/* Out of packets? */
	if (received < budget)
		virtqueue_napi_complete(napi, rq->vq, received);
	return received;
}

static void virtio_wifi_rx_completed(struct virtqueue *vq)
{
	struct virtio_wifi_info *vi = vq->vdev->priv;
	struct receive_queue *rq = vi->rq_sta;

	virtqueue_napi_schedule(&rq->napi, vq);
}

static int virtio_wifi_init_vqs(struct virtio_wifi_info *vi)
{
	int err;
	const int kTotalVqs = 4;
	struct virtqueue *vqs[kTotalVqs];
	const char * const names[] = { "rx_sta", "tx_sta", "cvq", "tx_p2p" };
	vq_callback_t *callbacks[] = {
		virtio_wifi_rx_completed,
		NULL,
		NULL,
		NULL,
	};
	vi->rq_sta = kzalloc(sizeof(*vi->rq_sta), GFP_KERNEL);
	if (!vi->rq_sta)
		goto err_rq;
	vi->sq_sta = kzalloc(sizeof(*vi->sq_sta), GFP_KERNEL);
	if (!vi->sq_sta)
		goto err_sq;
	vi->sq_p2p = kzalloc(sizeof(*vi->sq_p2p), GFP_KERNEL);
	if (!vi->sq_p2p)
		goto err_sq_p2p;

	netif_napi_add(&vi->napi_dev, &vi->rq_sta->napi, virtio_wifi_poll,
		   napi_weight);
	sg_init_table(vi->rq_sta->sg, ARRAY_SIZE(vi->rq_sta->sg));
	sg_init_table(vi->sq_sta->sg, ARRAY_SIZE(vi->sq_sta->sg));
	sg_init_table(vi->sq_p2p->sg, ARRAY_SIZE(vi->sq_p2p->sg));

	err = vi->vdev->config->find_vqs(vi->vdev, kTotalVqs,
			vqs, callbacks, names, NULL, NULL);
	if (!err) {
		vi->rq_sta->vq = vqs[0];
		vi->sq_sta->vq = vqs[1];
		vi->cvq = vqs[2];
		vi->sq_p2p->vq = vqs[3];
		return 0;
	}
err_sq_p2p:
	kfree(vi->sq_p2p);
err_sq:
	kfree(vi->sq_sta);
err_rq:
	kfree(vi->rq_sta);
	return -ENOMEM;
}

static void virtio_wifi_del_vqs(struct virtio_wifi_info *vi)
{
	struct virtio_device *vdev = vi->vdev;

	vdev->config->del_vqs(vdev);

	kfree(vi->rq_sta);
	kfree(vi->sq_sta);
	kfree(vi->sq_p2p);
}

static void virtio_wifi_remove(struct virtio_device *vdev)
{
	struct virtio_wifi_info *vi = vdev->priv;

	vi->vdev->config->reset(vi->vdev);
	virtio_wifi_del_vqs(vi);
	cancel_delayed_work_sync(&vi->refill_work);
	ieee80211_free_hw(vi->hw);
}

static int virtio_wifi_init_modes(struct virtio_wifi_info *vi)
{
	enum nl80211_band band;

	memcpy(vi->channels_2ghz, virtio_wifi_channels_2ghz,
		sizeof(virtio_wifi_channels_2ghz));
	memcpy(vi->channels_5ghz, virtio_wifi_channels_5ghz,
		sizeof(virtio_wifi_channels_5ghz));
	memcpy(vi->rates, virtio_wifi_rates, sizeof(virtio_wifi_rates));

	for (band = NL80211_BAND_2GHZ; band < NUM_NL80211_BANDS; band++) {
		struct ieee80211_supported_band *sband = &vi->bands[band];

		switch (band) {
		case NL80211_BAND_2GHZ:
			sband->channels =
				(struct ieee80211_channel *)vi->channels_2ghz;
			sband->n_channels = ARRAY_SIZE(vi->channels_2ghz);
			sband->bitrates = (struct ieee80211_rate *)vi->rates;
			sband->n_bitrates = ARRAY_SIZE(vi->rates);
			break;
		case NL80211_BAND_5GHZ:
			sband->channels =
				(struct ieee80211_channel *)vi->channels_5ghz;
			sband->n_channels = ARRAY_SIZE(vi->channels_5ghz);
			sband->bitrates =
				(struct ieee80211_rate *)vi->rates + 4;
			sband->n_bitrates = ARRAY_SIZE(vi->rates) - 4;
			break;
		default:
			continue;
		}
		sband->ht_cap.ht_supported = true;
		sband->ht_cap.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
				IEEE80211_HT_CAP_GRN_FLD |
				IEEE80211_HT_CAP_SGI_20 |
				IEEE80211_HT_CAP_SGI_40 |
				IEEE80211_HT_CAP_DSSSCCK40;
		sband->ht_cap.ampdu_factor = 0x3;
		sband->ht_cap.ampdu_density = 0x6;
		memset(&sband->ht_cap.mcs, 0, sizeof(sband->ht_cap.mcs));
		sband->ht_cap.mcs.rx_mask[0] = 0xff;
		sband->ht_cap.mcs.rx_mask[1] = 0xff;
		sband->ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
		vi->hw->wiphy->bands[band] = sband;
	}
	return 0;
}

static void virtio_wifi_beacon_tx(void *arg, u8 *mac, struct ieee80211_vif *vif)
{
	struct virtio_wifi_info *vi = arg;
	struct send_queue *sq = vi->sq_p2p;
	struct ieee80211_hw *hw = vi->hw;
	struct ieee80211_tx_info *info;
	struct ieee80211_mgmt *mgmt;
	struct sk_buff *skb;
	int err;

	if (vif->type != NL80211_IFTYPE_AP &&
			vif->type != NL80211_IFTYPE_MESH_POINT &&
			vif->type != NL80211_IFTYPE_ADHOC)
		return;

	skb = ieee80211_beacon_get(hw, vif);
	if (skb == NULL)
		return;
	info = IEEE80211_SKB_CB(skb);
	if (ieee80211_hw_check(hw, SUPPORTS_RC_TABLE))
		ieee80211_get_tx_rates(vif, NULL, skb, info->control.rates,
					ARRAY_SIZE(info->control.rates));
	mgmt = (struct ieee80211_mgmt *)skb->data;
	/* fake header transmission time */
	mgmt->u.beacon.timestamp = virtio_wifi_get_tsf_raw();
	err = xmit_skb(sq, skb);
	if (unlikely(err)) {
		dev_kfree_skb_any(skb);
		return;
	}
	skb_orphan(skb);
	nf_reset_ct(skb);

	if (vif->csa_active && ieee80211_csa_is_complete(vif))
		ieee80211_csa_finish(vif);
}

static enum hrtimer_restart virtio_wifi_beacon(struct hrtimer *timer)
{
	struct virtio_wifi_info *vi =
		container_of(timer, struct virtio_wifi_info, beacon_timer);
	struct ieee80211_hw *hw = vi->hw;
	u64 bcn_int = vi->beacon_int;
	ktime_t next_bcn;

	if (!vi->started)
		goto out;

	ieee80211_iterate_active_interfaces_atomic(
			hw, IEEE80211_IFACE_ITER_NORMAL,
			virtio_wifi_beacon_tx, vi);
	next_bcn = ktime_add(hrtimer_get_expires(timer),
		ns_to_ktime(bcn_int * 5000));
	hrtimer_start(&vi->beacon_timer, next_bcn, HRTIMER_MODE_ABS);
out:
	return HRTIMER_NORESTART;
}

static const struct ieee80211_iface_limit virtio_wifi_if_limit[] = {
	{ .max = 1, .types = BIT(NL80211_IFTYPE_ADHOC) },
	{ .max = 2048,
	  .types = BIT(NL80211_IFTYPE_STATION) |
				BIT(NL80211_IFTYPE_P2P_CLIENT) |
				BIT(NL80211_IFTYPE_AP) |
				BIT(NL80211_IFTYPE_P2P_GO) },
	/* must be last, see hwsim_if_comb */
	{ .max = 1, .types = BIT(NL80211_IFTYPE_P2P_DEVICE) }
};

static const struct ieee80211_iface_combination virtio_wifi_ifcomb[] = {
	{
		.limits = virtio_wifi_if_limit,
		.n_limits = ARRAY_SIZE(virtio_wifi_if_limit),
		.max_interfaces = 2048,
		.num_different_channels = 1,
		.radar_detect_widths =
			BIT(NL80211_CHAN_WIDTH_20_NOHT) |
			BIT(NL80211_CHAN_WIDTH_20) |
			BIT(NL80211_CHAN_WIDTH_40) |
			BIT(NL80211_CHAN_WIDTH_80) |
			BIT(NL80211_CHAN_WIDTH_160),
	},
};

static int virtio_wifi_probe(struct virtio_device *vdev)
{
	int err;
	u8 macaddr[ETH_ALEN];
	struct ieee80211_hw *hw;
	struct virtio_wifi_info *vi;

	hw = ieee80211_alloc_hw(sizeof(*vi), &virtio_wifi_ops);
	err = 1;
	if (!hw)
		goto err_init_ieee80211;

	vi = hw->priv;
	vdev->priv = vi;
	vi->vdev = vdev;
	vi->hdr_len = 0;
	vi->hw = hw;
	mutex_init(&vi->mutex);
	init_dummy_netdev(&vi->napi_dev);
	INIT_DELAYED_WORK(&vi->refill_work, virtio_wifi_refill_work);
	netif_set_real_num_tx_queues(&vi->napi_dev, 1);
	netif_set_real_num_rx_queues(&vi->napi_dev, 1);
	err = virtio_wifi_init_vqs(vi);
	if (err)
		goto err_init_vq;

	SET_IEEE80211_DEV(hw, &vdev->dev);
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, CONNECTION_MONITOR);

	hw->wiphy->interface_modes =
		BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP) |
		BIT(NL80211_IFTYPE_P2P_CLIENT) | BIT(NL80211_IFTYPE_P2P_GO) |
		BIT(NL80211_IFTYPE_P2P_DEVICE) | BIT(NL80211_IFTYPE_ADHOC);
	hw->queues = 5;
	hw->offchannel_tx_hw_queue = 4;
	virtio_wifi_init_modes(vi);
	hw->wiphy->iface_combinations = virtio_wifi_ifcomb;
	hw->wiphy->n_iface_combinations = ARRAY_SIZE(virtio_wifi_ifcomb);

	virtio_cread_bytes(vdev, offsetof(struct virtio_wifi_config, mac),
			macaddr, ETH_ALEN);

	SET_IEEE80211_PERM_ADDR(hw, macaddr);

	hrtimer_init(&vi->beacon_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_SOFT);
	vi->beacon_timer.function = virtio_wifi_beacon;
	hw->wiphy->flags |= WIPHY_FLAG_SUPPORTS_TDLS |
			WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL | WIPHY_FLAG_AP_UAPSD |
			WIPHY_FLAG_HAS_CHANNEL_SWITCH;
	hw->wiphy->features |= NL80211_FEATURE_ACTIVE_MONITOR |
			   NL80211_FEATURE_AP_MODE_CHAN_WIDTH_CHANGE |
			   NL80211_FEATURE_STATIC_SMPS |
			   NL80211_FEATURE_DYNAMIC_SMPS;

	err = ieee80211_register_hw(hw);
	if (err) {
		pr_debug("virtio wifi: could not register device\n");
		goto err_register_hw;
	}
	virtio_device_ready(vdev);
	return 0;

err_register_hw:
	ieee80211_free_hw(hw);
err_init_vq:
	cancel_delayed_work_sync(&vi->refill_work);
	virtio_wifi_del_vqs(vi);
	kfree(vi);
err_init_ieee80211:
	return err;
}

static void virtio_wifi_config_changed(struct virtio_device *vdev)
{
}

static int virtio_wifi_validate(struct virtio_device *vdev)
{
	return 0;
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_MAC80211_WLAN, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {
	/* none */
};

static struct virtio_driver virtio_wifi_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.validate = virtio_wifi_validate,
	.probe = virtio_wifi_probe,
	.remove = virtio_wifi_remove,
	.config_changed = virtio_wifi_config_changed,
};

static __init int virtio_wifi_driver_init(void)
{
	return register_virtio_driver(&virtio_wifi_driver);
}
module_init(virtio_wifi_driver_init);

static __exit void virtio_wifi_driver_exit(void)
{
	unregister_virtio_driver(&virtio_wifi_driver);
}
module_exit(virtio_wifi_driver_exit);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("A virtio MAC80211 WLAN driver for emulated wifi on Emulator.");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
