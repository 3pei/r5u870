/*
 * USBCAM abstraction library for USB webcam drivers
 *
 * Copyright (c) 2007 Sam Revitch <samr7 cs washington edu>
 *
 * This driver is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this driver; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * This is a usbcam skeleton driver intended to be a starting point for
 * minidriver development.
 */

#define USBCAM_DEBUG_DEFAULT 0xffff

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include "usbcam.h"


#define USBCAM_DBG_SKEL_INIT	(USBCAM_DBGBIT_MD_START + 0)
#define USBCAM_DBG_SKEL_FRAME	(USBCAM_DBGBIT_MD_START + 1)
#define USBCAM_DBG_SKEL_MDINTF	(USBCAM_DBGBIT_MD_START + 2)
#define USBCAM_DBG_SKEL_CTRL	(USBCAM_DBGBIT_MD_START + 3)


#define skel_info(RV, FMT...) usbcam_info((RV)->cs_parent, FMT)
#define skel_err(RV, FMT...) usbcam_err((RV)->cs_parent, FMT)
#define skel_dbg(RV, SUBSYS, FMT...) usbcam_dbg((RV)->cs_parent, SUBSYS, FMT)

#define SKEL_VERSION KERNEL_VERSION(1,0,0)
#define SKEL_VERSION_EXTRA ""


/*
 * struct skel_dev is the per-device private structure for this
 * minidriver.  We store all the details of the current state of the
 * camera in here, other than those explicitly shared with usbcam.
 */
struct skel_dev {
	struct usbcam_dev *cs_parent;
};

#define udp_skel(UDP) ((struct skel_dev *)((UDP)->ud_minidrv_data))

static int skel_usbcam_init(struct usbcam_dev *udp,
			    const struct usb_device_id *devid)
{
	struct skel_dev *cdp = 	udp_skel(udp);
	cdp->cs_parent = udp;

	/* Probe/initialize the device */

	return 0;
}

static void skel_usbcam_disconnect(struct usbcam_dev *udp)
{
}

static int skel_usbcam_try_format(struct usbcam_dev *udp,
				  struct v4l2_pix_format *f)
{
	return -EINVAL;
}

static int skel_usbcam_set_format(struct usbcam_dev *udp,
				  struct v4l2_pix_format *f)
{
	return -EINVAL;
}

static int skel_usbcam_cap_start(struct usbcam_dev *udp)
{
	return -EINVAL;
}

static void skel_usbcam_cap_stop(struct usbcam_dev *udp)
{
}


static struct usbcam_dev_ops skel_usbcam_dev_ops = {
	.init		= skel_usbcam_init,
	.disconnect	= skel_usbcam_disconnect,
	.try_format	= skel_usbcam_try_format,
	.set_format	= skel_usbcam_set_format,
	.cap_start	= skel_usbcam_cap_start,
	.cap_stop	= skel_usbcam_cap_stop,
};

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0xfff0, 0xfff0), .driver_info = 0 },
	{ }
};

DEFINE_USBCAM_MINIDRV_MODULE(SKEL_VERSION,
			     SKEL_VERSION_EXTRA,
			     &skel_usbcam_dev_ops,
			     sizeof(struct skel_dev),
			     id_table)

MODULE_DESCRIPTION("Driver for Skeleton Webcam");
MODULE_AUTHOR("You <you yourdomain com>");
MODULE_LICENSE("GPL");
