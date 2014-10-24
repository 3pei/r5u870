/*
 * USBCAM abstraction library for USB webcam drivers
 *
 * Copyright (C) 2007 Sam Revitch <samr7 cs washington edu>
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

#include "usbcam_priv.h"
#include <media/v4l2-device.h>
/*
 * This file contains:
 * - Minidriver registration / deregistration handlers.
 * - Device and minidriver reference counting functions
 * - USB subsystem callouts
 */

/*
 * Reference Counting Notes
 *
 * Each usbcam_minidrv gets:
 * - One reference for being in the registered state
 * - One reference for each outstanding usbcam_dev
 *
 * Each usbcam_dev gets:
 * - One reference for having its V4L minor registered and not released
 * - One reference for having its underlying USB device not disconnected
 * - One reference for each open file handle
 */

static void usbcam_minidrv_release(struct kref *kref)
{
	struct usbcam_minidrv *minidrv =
		container_of(kref, struct usbcam_minidrv, um_kref);

	assert(!minidrv->um_dev_count);
	usbcam_dbgm(minidrv, DEV_STATE, "%s: destroying minidrvier",
		    __FUNCTION__);
	kfree(minidrv);
}


static void usbcam_dev_free(struct kref *kref)
{
	struct usbcam_dev *udp =
		container_of(kref, struct usbcam_dev, ud_kref);

	if (udp->ud_work_refs) {
		usbcam_lock(udp);
		usbcam_warn(udp, "%s: work queue has %d leaked refs",
			    __FUNCTION__, udp->ud_work_refs);
		while (udp->ud_work_refs)
			usbcam_work_unref(udp);
		usbcam_unlock(udp);
	}

	usbcam_work_maybe_stop(udp);
	assert(!udp->ud_work_thread);

	usb_put_intf(udp->ud_intf);
	udp->ud_intf = NULL;

	usb_put_dev(udp->ud_dev);
	udp->ud_dev = NULL;

	mutex_lock(&udp->ud_minidrv->um_lock);

	assert(!list_empty(&udp->ud_drv_links));
	assert(udp->ud_minidrv->um_dev_count > 0);

	list_del_init(&udp->ud_drv_links);
	udp->ud_minidrv->um_dev_count--;

	mutex_unlock(&udp->ud_minidrv->um_lock);

	kref_put(&udp->ud_minidrv->um_kref, usbcam_minidrv_release);
    if(udp->ud_vdev.v4l2_dev) {     /* 3pei */
        kfree(udp->ud_vdev.v4l2_dev);
        udp->ud_vdev.v4l2_dev = NULL;
        }
	kfree(udp);
}

static void usbcam_dev_release(struct kref *kref)
{
	struct usbcam_dev *udp =
		container_of(kref, struct usbcam_dev, ud_kref);

	usbcam_dbg(udp, DEV_STATE, "%s: destroying device", __FUNCTION__);

	if (usbcam_minidrv_op_present(udp, release)) {
		usbcam_lock(udp);
		usbcam_ctrl_releaseall(udp);
		usbcam_minidrv_op(udp, release);
		usbcam_unlock(udp);
	}
	usbcam_dev_free(kref);
}

void usbcam_put(struct usbcam_dev *udp)
{
	kref_put(&udp->ud_kref, usbcam_dev_release);
}
USBCAM_EXPORT_SYMBOL(usbcam_put);

/*
 * V4L2 videodev callout implementations
 */

static void usbcam_videodev_release(struct video_device *vfd)
{
	struct usbcam_dev *udp = container_of(vfd, struct usbcam_dev, ud_vdev);

	usbcam_lock(udp);

	assert(!udp->ud_videodev_released);
	udp->ud_videodev_released = 1;

	usbcam_unlock(udp);
	usbcam_put(udp);
}

/*
 * 3pei: mocks
 */
static void v4l2_release(struct v4l2_device *v4l2dev) {
    // 3pei: we have nothing, so we don't need release anything at current.
    printk(KERN_WARNING "**3PEI: !!!!!!! FIXME! not implemented! %s", __FUNCTION__);
}

static void v4l2_notify(struct v4l2_subdev *sd, unsigned int notification, void *arg) {
    // 3pei: we don't have sub devices, so who called us?
    printk(KERN_WARNING "**3PEI: !!!!!!! FIXME! not implemented! %s", __FUNCTION__);
}

/*
 * USB subsystem operation implementations
 */

struct usbcam_claimed_interface {
	struct list_head	ui_links;
	struct usb_interface	*ui_intf;
};


static int usbcam_usb_probe(struct usb_interface *intf,
			    const struct usb_device_id *devid)
{
	struct usb_driver *drvp;
	struct usbcam_dev *udp = NULL, *udpx;
	usbcam_minidrv_t *minidrv;
	struct usb_device *dev;
	struct list_head *listp;
	struct usbcam_claimed_interface *cip;
    struct v4l2_device *v4l2dev;
	int minidrv_init_failed = 0;
	int res, i;
    int registered = 0;

	/* Locate the mini-driver */
	dev = interface_to_usbdev(intf);
	drvp = to_usb_driver(intf->dev.driver);
	minidrv = container_of(drvp, usbcam_minidrv_t, um_usbdrv);
    
    /* 3pei */
    v4l2dev = (struct v4l2_device *) kzalloc(sizeof(*v4l2dev), GFP_KERNEL);
    if(!v4l2dev) { return -ENOMEM; }
    INIT_LIST_HEAD(&v4l2dev->subdevs);
    mutex_init(&v4l2dev->ioctl_lock);
    spin_lock_init(&v4l2dev->lock);
    kref_init(&v4l2dev->ref);
    v4l2dev->release = v4l2_release;
    v4l2dev->dev = NULL;
    //v4l2dev->mdev = NULL;
    v4l2_prio_init(&v4l2dev->prio);
    v4l2dev->ctrl_handler = NULL;
    v4l2dev->notify = v4l2_notify;

	/* Allocate and initialize a device structure */
	udp = (struct usbcam_dev *) kzalloc(sizeof(*udp) + minidrv->um_dev_privsize, GFP_KERNEL);

	if (!udp) { return -ENOMEM; }

	INIT_LIST_HEAD(&udp->ud_ctrl_list);
	mutex_init(&udp->ud_open_lock);
	mutex_init(&udp->ud_lock);
	spin_lock_init(&udp->ud_work_lock);
	INIT_LIST_HEAD(&udp->ud_work_queue);
	udp->ud_minidrv = minidrv;
	udp->ud_dev = usb_get_dev(dev);
	udp->ud_intf = usb_get_intf(intf);
	udp->ud_debug = minidrv->um_debug;
	INIT_LIST_HEAD(&udp->ud_drv_links);
	kref_init(&udp->ud_kref);

	INIT_LIST_HEAD(&udp->ud_interface_list);
	INIT_LIST_HEAD(&udp->ud_frame_cap_queue);

	if (minidrv->um_dev_privsize)
		udp->ud_minidrv_data = &udp[1];

	/* Set up the video4linux structure */
	udp->ud_vdev = minidrv->um_videodev_template;
	udp->ud_vdev.release = usbcam_videodev_release;
    /* sanpei */
    udp->ud_vdev.v4l2_dev = v4l2dev;

	/* Add the device to the minidriver's list of active devices */
	usbcam_lock(udp);

	mutex_lock(&minidrv->um_lock);

	/* Inefficiently find an unused ID in the device list */
	i = 0;
	udpx = NULL;
	list_for_each(listp, &minidrv->um_dev_list) {
		udpx = list_entry(listp, struct usbcam_dev, ud_drv_links);
		if (udpx->ud_minidrv_id < 0) {
			udpx = NULL;
			continue;
		}
		if (udpx->ud_minidrv_id != i)
			break;
		udpx = NULL;
		i++;
	}

	udp->ud_minidrv_id = i;
	if (udpx) {
		list_add_tail(&udp->ud_drv_links, &udpx->ud_drv_links);
	} else {
		list_add_tail(&udp->ud_drv_links, &minidrv->um_dev_list);
	}

	minidrv->um_dev_count++;
	kref_get(&minidrv->um_kref);

	snprintf(udp->ud_dev_name, sizeof(udp->ud_dev_name), "%s-%d", usbcam_drvname(udp->ud_minidrv), udp->ud_minidrv_id);
	snprintf(udp->ud_vdev.name, sizeof(udp->ud_vdev.name), "%s USB Camera #%d", usbcam_drvname(minidrv), udp->ud_minidrv_id + 1);
    snprintf(v4l2dev->name, sizeof(v4l2dev->name), "%s USBCAM V4L2 #%d", usbcam_drvname(minidrv), udp->ud_minidrv_id + 2);


	mutex_unlock(&minidrv->um_lock);

	/* Invoke the minidriver initialization callout */
	udp->ud_initializing = 1;
	res = usbcam_minidrv_op(udp, init, devid);
	udp->ud_initializing = 0;
	if (res) {
		usbcam_dbg(udp, DEV_STATE, "minidriver init failed: %d", res);
		minidrv_init_failed = 1;
		goto out_nodisconn;
	}

	/* Complain if the device isn't filled out correctly */
	if (!udp->ud_format.width || !udp->ud_format.height) {
		usbcam_warn(udp, "minidriver did not set default size");
		res = -EINVAL;
		goto out;
	}
	if (!udp->ud_format.pixelformat) {
		usbcam_warn(udp, "minidriver did not set default pixelformat");
		res = -EINVAL;
		goto out;
	}

	usb_set_intfdata(intf, udp);
	usbcam_unlock(udp);

	/*
	 * Register the device with video4linux
	 *
	 * BUG: video_register_device() may or may not call back
	 * into usbcam_videodev_release(), depending on how it fails,
	 * and if it does call back, its callback may be latent.
	 *
	 * We will assume no callback on failure.
	 */

	if (udp->ud_vdev.minor != -1) {
		/* Minidriver has indicated its preference for a minor */
		res = video_register_device(&udp->ud_vdev, VFL_TYPE_GRABBER, -1);
		if (!res) {
			usbcam_err(udp, "%s: video_register_device failed (minor != -1)", __FUNCTION__);
			goto video_registered; 
		}
        registered = 1;
	}

	for (i = 0; i < minidrv->um_video_nr_array_len; i++) {
		res = video_register_device(&udp->ud_vdev, VFL_TYPE_GRABBER, minidrv->um_video_nr_array[i]);
		if (!res) {
			usbcam_err(udp, "%s: video_register_device failed (minidrv->um_video_nr_array[%d]=%d)", __FUNCTION__, i, minidrv->um_video_nr_array[i]);
			goto video_registered;
			}
	}

    res = video_register_device(&udp->ud_vdev, VFL_TYPE_GRABBER, -1);

	if (res) {
		usbcam_err(udp, "%s: video_register_device failed", __FUNCTION__);
		usbcam_lock(udp);
		assert(!udp->ud_videodev_released);
        udp->ud_vdev.v4l2_dev = NULL;
        kfree(v4l2dev);
		udp->ud_videodev_released = 1;
		goto out;
	}

video_registered:
	usbcam_get(udp);
	/*
	 * There should now be at least two references on udp:
	 * One for the primary USB interface in the non-disconnected state
	 * One for the videodev stuff
	 * One for each additional claimed interface
	 */

	usbcam_info(udp, "registered as video%d", udp->ud_vdev.minor);

	usbcam_work_maybe_stop(udp);
	return 0;

out:
	assert(!udp->ud_disconnected);
	udp->ud_disconnected = 1;
	if (usbcam_minidrv_op_present(udp, disconnect))
		usbcam_minidrv_op(udp, disconnect);

out_nodisconn:
	while (!list_empty(&udp->ud_interface_list)) {
		cip = list_entry(udp->ud_interface_list.next,
				 struct usbcam_claimed_interface,
				 ui_links);
		list_del_init(&cip->ui_links);
		usb_set_intfdata(cip->ui_intf, NULL);
		usb_driver_release_interface(&minidrv->um_usbdrv,
					     cip->ui_intf);
		usb_put_intf(cip->ui_intf);
		kfree(cip);
		usbcam_put(udp);
	}

	usbcam_unlock(udp);

	if (minidrv_init_failed)
		kref_put(&udp->ud_kref, usbcam_dev_free);
	else
		usbcam_put(udp);
	return res;
}

static void usbcam_usb_disconnect(struct usb_interface *intf)
{
	struct usbcam_dev *udp = (struct usbcam_dev *) usb_get_intfdata(intf);
	struct usbcam_claimed_interface *iterp, *cip;
	int put_intf = 0;
	int put_udp = 0;

	if (!udp)
		return;

	usbcam_lock(udp);
	if (!udp->ud_disconnected) {
		udp->ud_disconnected = 1;
		usbcam_unlock(udp);

		usbcam_dbg(udp, DEV_STATE, "disconnected");
		video_unregister_device(&udp->ud_vdev);

		usbcam_dbg(udp, DEV_STATE, "unregistered from video%d",
			   udp->ud_vdev.minor);

		usbcam_lock(udp);
		if (usbcam_minidrv_op_present(udp, disconnect))
			usbcam_minidrv_op(udp, disconnect);

		usbcam_capture_stop(udp);
	}

	if (intf == udp->ud_intf) {
		assert(!udp->ud_disconnected_primary);
		udp->ud_disconnected_primary = 1;
		put_udp = 1;

	} else {
		cip = NULL;
		list_for_each_entry(iterp, &udp->ud_interface_list, ui_links) {
			if (iterp->ui_intf == intf) {
				cip = iterp;
				break;
			}
		}

		if (cip) {
			list_del_init(&cip->ui_links);
			kfree(cip);
			put_intf = 1;
			put_udp = 1;
		} else {
			usbcam_err(udp, "interface %p is not claimed", intf);
		}
	}

	usb_set_intfdata(intf, NULL);
	usbcam_unlock(udp);

	usbcam_work_maybe_stop(udp);

	if (put_intf)
		usb_put_intf(intf);
	if (put_udp)
		usbcam_put(udp);
}

#if defined(CONFIG_PM)
static int usbcam_usb_suspend(struct usb_interface *intf, pm_message_t msg)
{
	struct usbcam_dev *udp = (struct usbcam_dev *) usb_get_intfdata(intf);
	int relock = 0, res = 0;
	if (!udp) {
		printk(KERN_WARNING "%s: no associated device\n",
		       __FUNCTION__);
		return 0;
	}

	usbcam_lock(udp);
	if ((intf != udp->ud_intf) || udp->ud_suspended) {
		/* Do nothing */
	} else if (usbcam_minidrv_op_present(udp, suspend)) {
		usbcam_dbg(udp, DEV_STATE, "invoking minidriver suspend");
		udp->ud_suspended = 1;
		if (udp->ud_minidrv->um_ops->unlocked_pm) {
			usbcam_unlock(udp);
			relock = 1;
		}

		res = usbcam_minidrv_op(udp, suspend, msg);

		if (relock)
			usbcam_lock(udp);
		if (res)
			udp->ud_suspended = 0;
	} else {
		usbcam_dbg(udp, DEV_STATE, "no minidriver suspend method");
		udp->ud_suspended = 1;
	}

	usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_usb_resume(struct usb_interface *intf)
{
	struct usbcam_dev *udp = (struct usbcam_dev *) usb_get_intfdata(intf);
	int relock = 0, res = 0;
	if (!udp) {
		printk(KERN_WARNING "%s: no associated device\n",
		       __FUNCTION__);
		return 0;
	}

	usbcam_lock(udp);
	if ((intf != udp->ud_intf) || !udp->ud_suspended) {
		/* Nothing to do! */
	} else if (usbcam_minidrv_op_present(udp, resume)) {
		usbcam_dbg(udp, DEV_STATE, "invoking minidriver resume");
		if (udp->ud_minidrv->um_ops->unlocked_pm) {
			usbcam_unlock(udp);
			relock = 1;
		}
		res = usbcam_minidrv_op(udp, resume);
		if (relock)
			usbcam_lock(udp);
	} else
		usbcam_dbg(udp, DEV_STATE, "no minidriver resume method");

	if (!res)
		udp->ud_suspended = 0;

	usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}
#endif  /* defined(CONFIG_PM) */


static const struct usb_driver usbcam_usb_driver_template = {
	.name		= "usbcam minidriver",
	.probe		= usbcam_usb_probe,
	.disconnect	= usbcam_usb_disconnect,
#if defined(CONFIG_PM)
	.suspend	= usbcam_usb_suspend,
	.resume		= usbcam_usb_resume,
#endif
};

/*
 * Minidriver registration/unregistration
 */

int usbcam_register_mod(usbcam_minidrv_t **driverpp,
			int minidrv_version, const char *minidrv_verx,
			const struct usbcam_dev_ops *ops,
			const int dev_priv_size,
			const struct usb_device_id *id_table,
			const int *video_nrs, int video_nrs_len,
			int *debug, struct module *md, const char *modname)
{
	usbcam_minidrv_t *minidrv;
	int res;

	printk(KERN_INFO "usbcam: registering driver %s %d.%d.%d%s\n",
	       modname,
	       (minidrv_version >> 16) & 0xff,
	       (minidrv_version >> 8) & 0xff,
	       minidrv_version & 0xff,
	       minidrv_verx ? minidrv_verx : "");

	minidrv = (usbcam_minidrv_t *) kzalloc(sizeof(*minidrv), GFP_KERNEL);
	if (!minidrv) {
		printk(KERN_ERR "%s: Failed to allocate usbcam_minidrv_t", __FUNCTION__);
		return -ENOMEM;
	}

	kref_init(&minidrv->um_kref);
	minidrv->um_owner = md;
	minidrv->um_modname = modname;
	minidrv->um_version = minidrv_version;
	minidrv->um_debug = debug;
	minidrv->um_dev_privsize = dev_priv_size;
	INIT_LIST_HEAD(&minidrv->um_dev_list);
	mutex_init(&minidrv->um_lock);
	minidrv->um_video_nr_array = video_nrs;
	minidrv->um_video_nr_array_len = video_nrs_len;

	minidrv->um_ops = ops;

	minidrv->um_usbdrv = usbcam_usb_driver_template;
	minidrv->um_usbdrv.name = usbcam_drvname(minidrv);
	minidrv->um_usbdrv.id_table = id_table;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
	minidrv->um_usbdrv.supports_autosuspend =
		minidrv->um_ops->supports_autosuspend;
#endif

	/*
	 * We have a separate fops per minidriver structure so that
	 * module reference counting works without egregious hacks.
	 */
	minidrv->um_v4l_fops = usbcam_v4l_fops_template;
	minidrv->um_v4l_fops.owner = minidrv->um_owner;

	minidrv->um_videodev_template = usbcam_videodev_template;
	minidrv->um_videodev_template.fops = &minidrv->um_v4l_fops;

	*driverpp = minidrv;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21)
	res = usb_register_driver(&minidrv->um_usbdrv, minidrv->um_owner,
				  minidrv->um_modname);
#else
	res = usb_register_driver(&minidrv->um_usbdrv, minidrv->um_owner);
#endif
	if (res) {
		kref_put(&minidrv->um_kref, usbcam_minidrv_release);
		*driverpp = NULL;
	}

	usbcam_dbgm(minidrv, DEV_STATE, "registered minidriver");

	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_register_mod);

void usbcam_unregister(usbcam_minidrv_t *minidrv)
{
	usbcam_dbgm(minidrv, DEV_STATE, "unregistering minidriver");

	usb_deregister(&minidrv->um_usbdrv);

	if (minidrv->um_dev_count) {
		/*
		 * This can happen if minidrivers unregister prior to
		 * module_exit(), but is usually bad.
		 */
		printk(KERN_ERR "%s: %d \"%s\" devices remain",
		    __FUNCTION__, minidrv->um_dev_count,
		    usbcam_drvname(minidrv));
	}

	kref_put(&minidrv->um_kref, usbcam_minidrv_release);
}
USBCAM_EXPORT_SYMBOL(usbcam_unregister);


int usbcam_claim_interface(struct usbcam_dev *udp, int ifnum)
{
	struct usb_interface *intf;
	struct usbcam_claimed_interface *cip;
	int res;

	usbcam_chklock(udp);

	if (!udp->ud_initializing) {
		usbcam_warn(udp, "%s may only be called from minidriver init",
			    __FUNCTION__);
		return -EINVAL;
	}

	intf = usb_ifnum_to_if(udp->ud_dev, ifnum);
	if (!intf) {
		usbcam_warn(udp, "%s: interface %d does not exist",
			    __FUNCTION__, ifnum);
		return -ENODEV;
	}

	res = usb_driver_claim_interface(&udp->ud_minidrv->um_usbdrv,
					 intf, NULL);

	if (!res) {
		cip = kmalloc(sizeof(*cip), GFP_KERNEL);
		if (!cip) {
			usb_driver_release_interface(&udp->ud_minidrv->
						     um_usbdrv, intf);
			return -ENOMEM;
		}

		INIT_LIST_HEAD(&cip->ui_links);
		cip->ui_intf = usb_get_intf(intf);
		usb_set_intfdata(intf, udp);
		usbcam_get(udp);
		list_add_tail(&cip->ui_links, &udp->ud_interface_list);
	}

	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_claim_interface);


/*
 * Work queue implementation
 */

static DECLARE_WAIT_QUEUE_HEAD(usbcam_work_idle_wait);

static int usbcam_work_thread(void *arg)
{
	struct usbcam_dev *udp = (struct usbcam_dev *) arg;
	struct usbcam_workitem *wip;
	sigset_t wakesigs;
	unsigned long flags;
	usbcam_workfunc_t fn;
	int res;

	current->flags |= PF_NOFREEZE;
	set_user_nice(current, -5);

	sigemptyset(&wakesigs);
	sigaddset(&wakesigs, SIGUSR1);

	while (1) {
		/* Wait for something to appear on the work queue */
		spin_lock_irqsave(&udp->ud_work_lock, flags);
		udp->ud_work_lockwait = 0;
		if (list_empty(&udp->ud_work_queue)) {
			if (kthread_should_stop()) {
				spin_unlock_irqrestore(&udp->ud_work_lock,
						       flags);
				break;
			}

			set_current_state(TASK_INTERRUPTIBLE);
			wake_up_all(&usbcam_work_idle_wait);
			spin_unlock_irqrestore(&udp->ud_work_lock, flags);
			schedule();
			spin_lock_irqsave(&udp->ud_work_lock, flags);
		}
		udp->ud_work_lockwait = 1;
		spin_unlock_irqrestore(&udp->ud_work_lock, flags);

		/* Enable the mutex wait cancelation signal */
		sigprocmask(SIG_UNBLOCK, &wakesigs, NULL);

		/* Re-check the queue, wait if it's still nonempty */
		res = -EINTR;
		if (!list_empty(&udp->ud_work_queue))
			res = mutex_lock_interruptible(&udp->ud_lock);

		/* Disable the mutex wait cancelation signal */
		sigprocmask(SIG_BLOCK, &wakesigs, NULL);
		flush_signals(current);

		if (res)
			continue;

		wip = NULL;
		spin_lock_irqsave(&udp->ud_work_lock, flags);
		udp->ud_work_lockwait = 0;
		if (!list_empty(&udp->ud_work_queue)) {
			wip = container_of(udp->ud_work_queue.next,
					   struct usbcam_workitem,
					   uw_links);
			list_del_init(&wip->uw_links);
		}

		spin_unlock_irqrestore(&udp->ud_work_lock, flags);

		if (wip) {
			fn = wip->uw_func;
			fn(wip);
		}

		usbcam_unlock(udp);
	}

	return 0;
}

void usbcam_work_init(struct usbcam_dev *udp, struct usbcam_workitem *wip,
		      usbcam_workfunc_t func)
{
	INIT_LIST_HEAD(&wip->uw_links);
	wip->uw_dev = udp;
	wip->uw_func = func;
}
USBCAM_EXPORT_SYMBOL(usbcam_work_init);

int usbcam_work_queue(struct usbcam_workitem *wip)
{
	struct usbcam_dev *udp = wip->uw_dev;
	unsigned long flags;
	int res;

	assert(udp != NULL);

	spin_lock_irqsave(&udp->ud_work_lock, flags);
	if (!list_empty(&wip->uw_links)) {
		res = -EALREADY;
		assert(wip->uw_dev == udp);
	} else if (udp->ud_work_refs) {
		res = 0;
		wip->uw_dev = udp;
		list_add_tail(&wip->uw_links, &udp->ud_work_queue);
		if (udp->ud_work_queue.next == &wip->uw_links)
			wake_up_process(udp->ud_work_thread);
	} else {
		res = -EBUSY;
	}
	spin_unlock_irqrestore(&udp->ud_work_lock, flags);

	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_work_queue);

int usbcam_work_cancel(struct usbcam_workitem *wip)
{
	struct usbcam_dev *udp = wip->uw_dev;
	unsigned long flags;
	int res, wakeit = 0;

	assert(udp != NULL);
	usbcam_chklock(udp);

	res = -ENOENT;
	spin_lock_irqsave(&udp->ud_work_lock, flags);
	if (!list_empty(&wip->uw_links)) {
		res = 0;
		assert(wip->uw_dev == udp);
		if ((udp->ud_work_queue.next == &wip->uw_links) &&
		    udp->ud_work_lockwait)
			wakeit = 1;
		list_del_init(&wip->uw_links);
		if (wakeit)
			force_sig(SIGUSR1, udp->ud_work_thread);
	}
	spin_unlock_irqrestore(&udp->ud_work_lock, flags);

	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_work_cancel);

int usbcam_work_ref(struct usbcam_dev *udp)
{
	struct task_struct *kt_new;
	unsigned long flags;

	usbcam_chklock(udp);

	/*
	 * We adjust this value under the spinlock to synchronize with
	 * usbcam_work_queue().
	 */
	spin_lock_irqsave(&udp->ud_work_lock, flags);
	udp->ud_work_refs++;
	spin_unlock_irqrestore(&udp->ud_work_lock, flags);

	if (!udp->ud_work_thread) {
		kt_new = kthread_create(usbcam_work_thread, udp,
					udp->ud_dev_name);
		if (!kt_new) {
			usbcam_err(udp, "%s: could not create worker thread",
				   __FUNCTION__);
			return -ENOMEM;
		}

		spin_lock_irqsave(&udp->ud_work_lock, flags);
		udp->ud_work_thread = kt_new;
		spin_unlock_irqrestore(&udp->ud_work_lock, flags);
	}

	return 0;
}
USBCAM_EXPORT_SYMBOL(usbcam_work_ref);

void usbcam_work_unref(struct usbcam_dev *udp)
{
	unsigned long flags;

	usbcam_chklock(udp);

	if (!udp->ud_work_refs) {
		usbcam_warn(udp, "%s: work queue has zero refs", __FUNCTION__);
		return;
	}

	spin_lock_irqsave(&udp->ud_work_lock, flags);
	udp->ud_work_refs--;
	spin_unlock_irqrestore(&udp->ud_work_lock, flags);
}
USBCAM_EXPORT_SYMBOL(usbcam_work_unref);

void usbcam_work_runqueue(struct usbcam_dev *udp)
{
	struct usbcam_workitem *wip;
	unsigned long flags;
	usbcam_workfunc_t fn;

	usbcam_chklock(udp);

	spin_lock_irqsave(&udp->ud_work_lock, flags);
	while (!list_empty(&udp->ud_work_queue)) {
		wip = container_of(udp->ud_work_queue.next,
				   struct usbcam_workitem,
				   uw_links);
		list_del_init(&wip->uw_links);
		spin_unlock_irqrestore(&udp->ud_work_lock, flags);

		fn = wip->uw_func;
		fn(wip);

		spin_lock_irqsave(&udp->ud_work_lock, flags);
	}
	spin_unlock_irqrestore(&udp->ud_work_lock, flags);
}
USBCAM_EXPORT_SYMBOL(usbcam_work_runqueue);


void usbcam_work_stop(struct usbcam_dev *udp)
{
	struct task_struct *kt_stop = NULL;
	unsigned long flags;

	usbcam_lock(udp);

	if (!udp->ud_work_refs) {
		/* Prevent further tasks from being queued */
		spin_lock_irqsave(&udp->ud_work_lock, flags);
		kt_stop = udp->ud_work_thread;
		udp->ud_work_thread = NULL;
		spin_unlock_irqrestore(&udp->ud_work_lock, flags);
	}

	usbcam_unlock(udp);

	if (kt_stop) {
		/*
		 * Wait for the queue to empty out, then stop the
		 * thread.  It might be easier to just call
		 * usbcam_work_flush() and execute the remaining
		 * tasks synchronously in the current thread.
		 */
		wait_event(usbcam_work_idle_wait,
			   list_empty(&udp->ud_work_queue));
		kthread_stop(kt_stop);
	}
}


static void usbcam_delayedwork_timeout(unsigned long data)
{
	struct usbcam_delayedwork *dwp = (struct usbcam_delayedwork *) data;
	int res;
	res = usbcam_work_queue(&dwp->dw_work);
	if (res)
		usbcam_warn(dwp->dw_work.uw_dev,
			    "delayed work item submit failed: %d", res);
}

void usbcam_delayedwork_init(struct usbcam_dev *udp,
			     struct usbcam_delayedwork *dwp,
			     usbcam_workfunc_t func)
{
	usbcam_work_init(udp, &dwp->dw_work, func);
	setup_timer(&dwp->dw_timer,
		    usbcam_delayedwork_timeout,
		    (unsigned long) dwp);
}
USBCAM_EXPORT_SYMBOL(usbcam_delayedwork_init);

void usbcam_delayedwork_queue(struct usbcam_delayedwork *dwp,
			      unsigned int timeout_ms)
{
	dwp->dw_timer.expires = jiffies + ((timeout_ms * HZ) / 1000);
	add_timer(&dwp->dw_timer);
}	
USBCAM_EXPORT_SYMBOL(usbcam_delayedwork_queue);

int usbcam_delayedwork_cancel(struct usbcam_delayedwork *dwp)
{
	if (timer_pending(&dwp->dw_timer) && del_timer_sync(&dwp->dw_timer))
		return 0;
	return usbcam_work_cancel(&dwp->dw_work);
}
USBCAM_EXPORT_SYMBOL(usbcam_delayedwork_cancel);


/*
 * Control related stuff
 */

struct usbcam_ctrl *usbcam_ctrl_find(struct usbcam_dev *udp, u32 ctlid)
{
	struct usbcam_ctrl *ctrlp;

	usbcam_chklock(udp);
	list_for_each_entry(ctrlp, &udp->ud_ctrl_list, uc_links) {
		if (ctrlp->uc_v4l.id == ctlid)
			return ctrlp;
	}
	return NULL;
}
USBCAM_EXPORT_SYMBOL(usbcam_ctrl_find);

int usbcam_ctrl_add(struct usbcam_dev *udp, struct usbcam_ctrl *ctrlp)
{
	int errors = 0;
	struct usbcam_ctrl *xctrlp;

	usbcam_chklock(udp);

	/* Verify that the ID isn't already registered */
	xctrlp = usbcam_ctrl_find(udp, ctrlp->uc_v4l.id);
	if (xctrlp) {
		usbcam_warn(udp, "control \"%s\" id=%d already defined",
			    ctrlp->uc_v4l.name, ctrlp->uc_v4l.id);
		errors++;
	}

	/* Check minimum, maximum, step, and default */
	switch (ctrlp->uc_v4l.type) {
	case V4L2_CTRL_TYPE_INTEGER:
		if (ctrlp->uc_v4l.minimum > ctrlp->uc_v4l.maximum) {
			usbcam_warn(udp, "control \"%s\" has "
				    "minimum > maximum",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		break;

	case V4L2_CTRL_TYPE_BOOLEAN:
		break;

	case V4L2_CTRL_TYPE_MENU:
		if (ctrlp->uc_v4l.minimum) {
			usbcam_warn(udp, "control \"%s\" is MENU and has "
				    "minimum != 0",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		if (!ctrlp->uc_v4l.maximum) {
			usbcam_warn(udp, "control \"%s\" is MENU and has "
				    "maximum == 0",
				    ctrlp->uc_v4l.name);
				errors++;
			}
		if (!ctrlp->uc_menu_names) {
			usbcam_warn(udp, "control \"%s\" is MENU and has "
				    "NULL menu_names",
				    ctrlp->uc_v4l.name);
			errors++;
			break;
		}
		break;

	case V4L2_CTRL_TYPE_BUTTON:
		if (ctrlp->uc_v4l.minimum) {
			usbcam_warn(udp, "control \"%s\" is BUTTON "
				    "and has minimum != 0",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		if (ctrlp->uc_v4l.maximum) {
			usbcam_warn(udp, "control \"%s\" is BUTTON "
				    "and has maximum != 0",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		if (ctrlp->uc_v4l.step) {
			usbcam_warn(udp, "control \"%s\" is BUTTON "
				    "and has step != 0",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		break;

	default:
		usbcam_warn(udp, "control \"%s\" is of "
			    "invalid type %d",
			    ctrlp->uc_v4l.name,
			    ctrlp->uc_v4l.type);
		errors++;
	}

	/* Check the range */
	if (ctrlp->uc_v4l.type == V4L2_CTRL_TYPE_BOOLEAN) {
		if ((ctrlp->uc_v4l.default_value != 0) &&
		    (ctrlp->uc_v4l.default_value != 1)) {
			usbcam_warn(udp, "control \"%s\" is BOOLEAN "
				    "and default value is %d",
				    ctrlp->uc_v4l.name,
				    ctrlp->uc_v4l.default_value);
			errors++;
		}
	}

	else if ((ctrlp->uc_v4l.default_value <
		  ctrlp->uc_v4l.minimum) ||
		 (ctrlp->uc_v4l.default_value >
		  ctrlp->uc_v4l.maximum)) {
		usbcam_warn(udp, "control \"%s\" default out of range",
			    ctrlp->uc_v4l.name);
		errors++;
	}

	/* Check the get_fn callout */
	if (ctrlp->uc_v4l.type == V4L2_CTRL_TYPE_BUTTON) {
		if (ctrlp->get_fn) {
			usbcam_warn(udp, "control \"%s\" is BUTTON "
				    "and has a get_fn callout",
				    ctrlp->uc_v4l.name);
			errors++;
		}
		if (!ctrlp->set_fn) {
			usbcam_warn(udp, "control \"%s\" is BUTTON "
				    "and has no set_fn callout",
				    ctrlp->uc_v4l.name);
			errors++;
		}
	}

	if (errors)
		return -EINVAL;

	list_add_tail(&ctrlp->uc_links, &udp->ud_ctrl_list);
	return 0;
}
USBCAM_EXPORT_SYMBOL(usbcam_ctrl_add);

struct usbcam_ctrl *usbcam_ctrl_alloc(size_t real_size)
{
	struct usbcam_ctrl *ctrlp;

	if (!real_size)
		real_size = sizeof(*ctrlp);
	if (real_size < sizeof(*ctrlp)) {
		printk(KERN_WARNING "Minidriver set control size %zd, "
		       "must be at least %zd\n", real_size, sizeof(*ctrlp));
		return NULL;
	}

	ctrlp = kzalloc(real_size, GFP_KERNEL);
	if (!ctrlp)
		return NULL;

	INIT_LIST_HEAD(&ctrlp->uc_links);
	return ctrlp;
}
USBCAM_EXPORT_SYMBOL(usbcam_ctrl_alloc);

struct usbcam_ctrl *usbcam_ctrl_add_tmpl(struct usbcam_dev *udp,
					 const struct usbcam_ctrl *tmplp,
					 size_t real_size)
{
	struct usbcam_ctrl *ctrlp;

	ctrlp = usbcam_ctrl_alloc(real_size);
	if (!ctrlp)
		return NULL;

	memcpy(ctrlp, tmplp, real_size);
	INIT_LIST_HEAD(&ctrlp->uc_links);

	if (usbcam_ctrl_add(udp, ctrlp)) {
		usbcam_ctrl_free(ctrlp);
		return NULL;
	}

	return ctrlp;
}
USBCAM_EXPORT_SYMBOL(usbcam_ctrl_add_tmpl);

void usbcam_ctrl_releaseall(struct usbcam_dev *udp)
{
	struct usbcam_ctrl *ctrlp;
	while (!list_empty(&udp->ud_ctrl_list)) {
		ctrlp = list_entry(udp->ud_ctrl_list.next,
				   struct usbcam_ctrl,
				   uc_links);
		list_del_init(&ctrlp->uc_links);
		if (ctrlp->release_fn)
			ctrlp->release_fn(udp, ctrlp);
		kfree(ctrlp);
	}
}


MODULE_DESCRIPTION("Abstraction Library for USB Webcam Drivers");
MODULE_AUTHOR("Sam Revitch <samr7 cs washington edu>");
MODULE_LICENSE("GPL");
