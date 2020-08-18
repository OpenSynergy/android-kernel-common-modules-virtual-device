/* Put all options we expect gki_defconfig to provide us here */

#ifndef CONFIG_BRIDGE
#error CONFIG_BRIDGE is required
#endif

#ifndef CONFIG_VETH
#error CONFIG_VETH is required
#endif

#ifndef CONFIG_CFG80211
#error CONFIG_CFG80211 is required
#endif

#ifndef CONFIG_NAMESPACES
#error CONFIG_NAMESPACES is required
#endif

#ifndef CONFIG_PCI
#error CONFIG_PCI is required
#endif

#ifndef CONFIG_COMPAT
#error CONFIG_COMPAT is required
#endif

#ifdef CONFIG_BLK_DEV_MD
#error CONFIG_BLK_DEV_MD is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_CPUFREQ_DUMMY
#error CONFIG_CPUFREQ_DUMMY is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_DRM_VIRTIO_GPU
#error CONFIG_DRM_VIRTIO_GPU is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_HW_RANDOM_VIRTIO
#error CONFIG_HW_RANDOM_VIRTIO is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_MAC80211_HWSIM
#error CONFIG_MAC80211_HWSIM is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_RTC_DRV_TEST
#error CONFIG_RTC_DRV_TEST is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_SDCARD_FS
#error CONFIG_SDCARD_FS is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_SND_HDA_CODEC_REALTEK
#error CONFIG_SND_HDA_CODEC_REALTEK is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_SND_HDA_INTEL
#error CONFIG_SND_HDA_INTEL is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_TEST_STACKINIT
#error CONFIG_TEST_STACKINIT is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_TEST_MEMINIT
#error CONFIG_TEST_MEMINIT is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_USB_DUMMY_HCD
#error CONFIG_USB_DUMMY_HCD is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_BLK
#error CONFIG_VIRTIO_BLK is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_CONSOLE
#error CONFIG_VIRTIO_CONSOLE is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_INPUT
#error CONFIG_VIRTIO_INPUT is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_MMIO
#error CONFIG_VIRTIO_MMIO is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_NET
#error CONFIG_VIRTIO_NET is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_PCI
#error CONFIG_VIRTIO_PCI is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_VIRTIO_PMEM
#error CONFIG_VIRTIO_PMEM is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_INCREMENTAL_FS
#error CONFIG_INCREMENTAL_FS is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_ZRAM
#error CONFIG_ZRAM is a module in goldfish_defconfig.fragment
#endif

#ifdef CONFIG_ZSMALLOC
#error CONFIG_ZSMALLOC is a module in goldfish_defconfig.fragment
#endif
