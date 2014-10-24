/*
 * USBCAM abstraction library for USB webcam drivers
 *
 * Copyright (c) 2007 Sam Revitch <samr7 cs washington edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * This header file is private and internal to usbcam.
 * DO NOT INCLUDE THIS HEADER FILE IN MINIDRIVERS.  USE usbcam.h INSTEAD.
 */

#if !defined(__USBCAM_PRIV_H__)
#define __USBCAM_PRIV_H__

#include "usbcam.h"

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0) /*3pei*/
#include <linux/smp_lock.h>
#endif
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
#include <linux/videodev.h>
#else
#include "libv4l1-videodev.h"
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#include <media/video-buf.h>
#else
#include <media/videobuf-dma-sg.h>
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define usb_endpoint_xfer_bulk(EPD) \
	(((EPD)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == \
	 USB_ENDPOINT_XFER_BULK)
#define usb_endpoint_xfer_int(EPD) \
	(((EPD)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == \
	 USB_ENDPOINT_XFER_INT)
#define usb_endpoint_xfer_isoc(EPD) \
	(((EPD)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == \
	 USB_ENDPOINT_XFER_ISOC)
#define usb_endpoint_dir_in(EPD) \
	(((EPD)->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define usb_autopm_get_interface(X) 0
#define usb_autopm_put_interface(X)
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
#if !defined(videobuf_queue_pci_init)
#define videobuf_queue_pci_init videobuf_queue_sg_init
#endif
#endif

/*
 * When creating a minidriver that is not part of the core kernel, it
 * may be desired to use a specific version of usbcam.  In this case,
 * usbcam's symbols should be available only to the minidriver with
 * which it is linked, and its symbols should not be exported.
 */
#define USBCAM_EXPORT_SYMBOL(X) EXPORT_SYMBOL(X)

#define assert usbcam_assert

#define usbcam_drvname(MDP) ((MDP)->um_owner->name)

#if defined(CONFIG_USB_USBCAM_DEBUG)
#define usbcam_dbgm(MD, SUBSYS, FMT, ARG...) do {			\
	if ((MD)->um_debug &&						\
	    *(MD)->um_debug & (1UL << USBCAM_DBG_ ## SUBSYS))		\
		printk(KERN_DEBUG "%s: " FMT "\n",			\
		       (MD)->um_modname, ## ARG);			\
} while (0)
#else
#define usbcam_dbgm(MD, SUBSYS, FMT, ARG...)
#endif  /* defined(CONFIG_USB_USBCAM_DEBUG) */

#define usbcam_minidrv_op_present(UDP, CB) 				\
	((UDP)->ud_minidrv->um_ops->CB ? 1 : 0)
#define usbcam_minidrv_op(UDP, CB, ARGS...)				\
	((UDP)->ud_minidrv->um_ops->CB((UDP), ## ARGS))


/*
 * Private data structure definitions
 */

/*
 * This structure represents a registered minidriver
 */
struct usbcam_minidrv {
	struct kref			um_kref;
	struct module			*um_owner;
	const char			*um_modname;
	int				*um_debug;
	int				um_version;
	int				um_dev_count;
	struct list_head		um_dev_list;
	int				um_dev_privsize;
	struct usb_driver		um_usbdrv;
	struct mutex			um_lock;
	const struct usbcam_dev_ops	*um_ops;
	struct video_device		um_videodev_template;
	struct v4l2_file_operations		um_v4l_fops;
	const int			*um_video_nr_array;
	int				um_video_nr_array_len;
};

/*
 * The frame structure is generally managed by the video-buf module
 * and represents some chunk of memory that the video4linux client
 * requested as a frame buffer.  It might be vmalloc()'d, or it might
 * be mapped from a user address space.  In either case, usbcam
 * guarantees a contiguous kernel mapping accessible to the minidriver.
 *
 * The primary reason to use video-buf in usbcam is for its
 * implementation of buffer mapping methods and "zero-copy" kernel-user
 * data movement.  The V4L2 API is quite rich, and it's much easier to
 * use video-buf than to create a private full-featured implementation,
 * and much more desirable to use video-buf than to limp along with a
 * substandard implementation.  The video-buf module isn't specifically
 * used for DMA functionality, as most USB devices, with the possible
 * exception of those employing bulk transfers, are unsuitable for
 * direct frame buffer DMA.
 *
 * Minidrivers can access details of the current frame using
 * usbcam_curframe_get(), and can signal completion of the current
 * frame with usbcam_curframe_complete().  It is up to the minidriver
 * to fill in the frame buffer.
 */
struct usbcam_frame {
	struct videobuf_buffer	vbb;
	struct list_head	cap_links;
	void			*vmap_base;
	void			*vmap_sof;
};

/*
 * This structure represents an open file handle and the frame
 * buffers associated with that client
 */
struct usbcam_fh {
	struct usbcam_dev		*ufh_dev;
	int				ufh_flags;
	struct videobuf_queue		ufh_vbq;
};

#define USBCAM_FH_USE_FIXED_FB		0x00000001

extern void usbcam_work_stop(struct usbcam_dev *udp);
extern void usbcam_capture_stop(struct usbcam_dev *udp);
extern void usbcam_ctrl_releaseall(struct usbcam_dev *udp);

/*
 * The below maybe_stop function allows usbcam to ensure that any kernel
 * threads it creates are completely stopped and not executing any
 * usbcam code at module unload time.
 *
 * This is ugly but it remains unclear how better to do it.
 */
static inline void usbcam_work_maybe_stop(struct usbcam_dev *udp)
{
	if (!udp->ud_work_refs)
		usbcam_work_stop(udp);
}

/*
 * Only stop capturing if no frames are queued.
 *
 * We allow and encourage the minidriver to continue capturing in the
 * last requested format, and have it stop autonomously when it receives
 * its first data for the next frame but finds no frame available.  This
 * expedites the process for situations such as S_FMT which cannot
 * tolerate capture being in progress.
 */
static inline void usbcam_capture_stop_nondestructive(struct usbcam_dev *udp)
{
	if (udp->ud_capturing && list_empty(&udp->ud_frame_cap_queue))
		usbcam_capture_stop(udp);
}

extern struct videobuf_queue_ops usbcam_videobuf_qops;
extern struct v4l2_file_operations usbcam_v4l_fops_template;
extern struct video_device usbcam_videodev_template;

#endif  /* !defined(__USBCAM_PRIV_H__) */
