/* SPDX-License-Identifier: GPL-2.0 */

#ifndef UAPI_GOLDFISH_SYNC_H
#define UAPI_GOLDFISH_SYNC_H

#include <linux/types.h>

#define GOLDFISH_SYNC_DEVICE_NAME "goldfish_sync"

struct goldfish_sync_ioctl_info {
	__u64 host_glsync_handle_in;
	__u64 host_syncthread_handle_in;
	__s32 fence_fd_out;
};

/* There is an ioctl associated with goldfish sync driver.
 * Make it conflict with ioctls that are not likely to be used
 * in the emulator.
 *
 * '@'	00-0F	linux/radeonfb.h		conflict!
 * '@'	00-0F	drivers/video/aty/aty128fb.c	conflict!
 */
#define GOLDFISH_SYNC_IOC_MAGIC	'@'

#define GOLDFISH_SYNC_IOC_QUEUE_WORK	\
	_IOWR(GOLDFISH_SYNC_IOC_MAGIC, 0, struct goldfish_sync_ioctl_info)

#endif /* UAPI_GOLDFISH_SYNC_H */
