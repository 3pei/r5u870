/*
 * USBCAM abstraction library for USB webcam drivers
 * Version 0.11.3
 *
 * Copyright (c) 2007 Sam Revitch <samr7 cs washington edu>
 * Copyright (c) 2008 Alexander Hixon <hixon.alexander@mediati.org>
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
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
#include "media/v4l2-ioctl.h"
#endif

#include <linux/printk.h>
#define log_error(msg) printk(KERN_ERR "**3PEI: %s, %i -> %s.\n", __FILE__, __LINE__, msg)

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
/*3pei: the power manager for usb interfaces is transparent, bypass these operations. */
#define usb_autopm_get_interface(x) (0)
#define usb_autopm_put_interface(x) (0)
#endif
/*
 * This file contains the file_operations implementation
 *
 * TODO LIST:
 * - Add debug tracing to more ioctl paths
 */

/* HELPER FOR V4L1 COMPAT OPS */

const static unsigned int palette2pixelformat[] = {
	[VIDEO_PALETTE_GREY]    = V4L2_PIX_FMT_GREY,
	[VIDEO_PALETTE_RGB555]  = V4L2_PIX_FMT_RGB555,
	[VIDEO_PALETTE_RGB565]  = V4L2_PIX_FMT_RGB565,
	[VIDEO_PALETTE_RGB24]   = V4L2_PIX_FMT_BGR24,
	[VIDEO_PALETTE_RGB32]   = V4L2_PIX_FMT_BGR32,
	/* yuv packed pixel */
	[VIDEO_PALETTE_YUYV]    = V4L2_PIX_FMT_YUYV,
	[VIDEO_PALETTE_YUV422]  = V4L2_PIX_FMT_YUYV,
	[VIDEO_PALETTE_UYVY]    = V4L2_PIX_FMT_UYVY,
	/* yuv planar */
	[VIDEO_PALETTE_YUV410P] = V4L2_PIX_FMT_YUV410,
	[VIDEO_PALETTE_YUV420]  = V4L2_PIX_FMT_YUV420,
	[VIDEO_PALETTE_YUV420P] = V4L2_PIX_FMT_YUV420,
	[VIDEO_PALETTE_YUV411P] = V4L2_PIX_FMT_YUV411P,
	[VIDEO_PALETTE_YUV422P] = V4L2_PIX_FMT_YUV422P,
};
/* 3pei : not used at current
const static char *v4l_ioctl_names[] = {
	"UNKNOWN",
	"VIDIOCGCAP",
	"VIDIOCGCHAN",
	"VIDIOCSCHAN",
	"VIDIOCGTUNER",
	"VIDIOCSTUNER",
	"VIDIOCGPICT",
	"VIDIOCSPICT",
	"VIDIOCCAPTURE",
	"VIDIOCGWIN",
	"VIDIOCSWIN",
	"VIDIOCGFBUF",
	"VIDIOCSFBUF",
	"VIDIOCKEY",
	"VIDIOCGFREQ",
	"VIDIOCSFREQ",
	"VIDIOCGAUDIO",
	"VIDIOCSAUDIO",
	"VIDIOCSYNC",
	"VIDIOCMCAPTURE",
	"VIDIOCGMBUF",
	"VIDIOCGUNIT",
	"VIDIOCGCAPTURE",
	"VIDIOCSCAPTURE",
	"VIDIOCSPLAYMODE",
	"VIDIOCSWRITEMODE",
	"VIDIOCGPLAYINFO",
	"VIDIOCSMICROCODE",
	"VIDIOCGVBIFMT",
	"VIDIOCSVBIFMT",
}; 

static unsigned int __pure
palette_to_pixelformat(unsigned int palette)
{
	if (palette < ARRAY_SIZE(palette2pixelformat))
		return palette2pixelformat[palette];
	else
		return 0;
}

static int poll_one(struct file *file)
{
	int retval = 1;
	poll_table *table;
	struct poll_wqueues pwq;

	poll_initwait(&pwq);
	table = &pwq.pt;
	for (;;) {
		int mask;
		set_current_state(TASK_INTERRUPTIBLE);
		mask = file->f_op->poll(file, table);
		if (mask & POLLIN)
			break;
		table = NULL;
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		schedule();
	}
	set_current_state(TASK_RUNNING);
	poll_freewait(&pwq);
	return retval;
}*/

/*
 * V4L file_operations callout implementations
 */

static int usbcam_v4l_open(struct file *filp)
{    
    struct usbcam_dev *udp;
	struct usbcam_fh *ufp;
	int autopm_ref = 0;
	int work_ref = 0;
	int res = 0;
    printk(KERN_NOTICE"**3PEI: usbcam_v4lopen(...) enter\n");

	/* The usbcam_dev is referenced by the videodev at this point */
	udp = container_of(video_devdata(filp), struct usbcam_dev, ud_vdev);

	ufp = (struct usbcam_fh *) kzalloc(sizeof(*ufp), GFP_KERNEL);
	if (!ufp) {
        log_error("!ufp");
		return -ENOMEM;
    }

	ufp->ufh_dev = udp;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	videobuf_queue_init(&ufp->ufh_vbq,
			    &usbcam_videobuf_qops,
			    NULL,
			    NULL,
			    V4L2_BUF_TYPE_VIDEO_CAPTURE,
			    V4L2_FIELD_INTERLACED,
			    sizeof(struct usbcam_frame), ufp);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37) /*3pei*/
	videobuf_queue_pci_init(&ufp->ufh_vbq,
			    &usbcam_videobuf_qops,
			    NULL,
			    &udp->slock,
			    V4L2_BUF_TYPE_VIDEO_CAPTURE,
			    V4L2_FIELD_INTERLACED,
			    sizeof(struct usbcam_frame), ufp);
#else
	videobuf_queue_pci_init(&ufp->ufh_vbq,
			    &usbcam_videobuf_qops,
			    NULL,
			    &udp->slock,
			    V4L2_BUF_TYPE_VIDEO_CAPTURE,
			    V4L2_FIELD_INTERLACED,
			    sizeof(struct usbcam_frame), ufp, udp->ud_vdev.lock);
#endif

	mutex_lock(&udp->ud_open_lock);

	if (!udp->ud_user_refs) {
		res = usb_autopm_get_interface(udp->ud_intf);
        if(res) {
		    printk(KERN_WARNING "%s(..) usb_autopm_get_interface(udp->ud_intf) == %d", __FUNCTION__, res);		
		    if(res == -EAGAIN && udp->ud_intf->dev.power.disable_depth) {
		        printk(KERN_WARNING "%s(..) udp->ud_intf->dev.power.disable_depth == %d, runtime pm disabled?", __FUNCTION__, udp->ud_intf->dev.power.disable_depth);
		        res = 0;        
		    } else {
		        log_error("don't know what happend, return with error.");
		        goto bail_nolock;
		    }
        }
		autopm_ref = 1;
	}

	usbcam_lock(udp);

	if (udp->ud_disconnected) {
        log_error("udp->ud_disconnected");
		res = -ENODEV;
		goto bail;
	}

	if (!udp->ud_user_refs) {
		if (!udp->ud_minidrv->um_ops->no_workref_on_open) {
			res = usbcam_work_ref(udp);
			if (res) {
                log_error("usbcam_work_ref(udp) failed");
				goto bail;
            }
			work_ref = 1;
		}

		if (usbcam_minidrv_op_present(udp, open)) {
			res = usbcam_minidrv_op(udp, open);
			if (res) {
                log_error("usbcam_minidrv_op(udp, open) failed");
				if (work_ref) {
					usbcam_work_unref(udp);
                }
				assert(!udp->ud_user_refs);	
				goto bail;
			}
		}

		/* Transfer the autopm reference */
		assert(autopm_ref);
		autopm_ref = 0;
	}

	udp->ud_user_refs++;
	filp->private_data = ufp;
	usbcam_get(udp);

bail:
	usbcam_unlock(udp);
bail_nolock:
	mutex_unlock(&udp->ud_open_lock);
	if (res) {
		kfree(ufp);
    }
	if (autopm_ref) { usb_autopm_put_interface(udp->ud_intf); }
	usbcam_work_maybe_stop(udp);
    printk(KERN_NOTICE"**3PEI: usbcam_v4lopen(...) exit with %d", res);
	return res;
}

static int usbcam_v4l_release(struct file *filp)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) filp->private_data;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int autopm_ref = 0;

	videobuf_mmap_free(&ufp->ufh_vbq);

	mutex_lock(&udp->ud_open_lock);
	usbcam_lock(udp);

	assert(udp->ud_user_refs);
	if (udp->ud_excl_owner == filp)
		udp->ud_excl_owner = NULL;
	kfree(ufp);
	filp->private_data = NULL;

	if (!--udp->ud_user_refs) {
		usbcam_capture_stop(udp);

		if (usbcam_minidrv_op_present(udp, close))
			usbcam_minidrv_op(udp, close);
		if (!udp->ud_minidrv->um_ops->no_workref_on_open)
			usbcam_work_unref(udp);
		autopm_ref = 1;
	}

	usbcam_unlock(udp);
	mutex_unlock(&udp->ud_open_lock);
	if (autopm_ref)
		usb_autopm_put_interface(udp->ud_intf);
	usbcam_work_maybe_stop(udp);
	usbcam_put(udp);
	return 0;
}

static ssize_t usbcam_v4l_read(struct file *filp, char __user *data,
			       size_t count, loff_t *ppos)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) filp->private_data;
	ssize_t res;

	res = videobuf_read_one(&ufp->ufh_vbq, data, count, ppos,
				(filp->f_flags & O_NONBLOCK) ? 1 : 0);
	usbcam_work_maybe_stop(ufp->ufh_dev);
	return res;
}

static unsigned int usbcam_v4l_poll(struct file *filp,
				    struct poll_table_struct *wait)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) filp->private_data;

	return videobuf_poll_stream(filp, &ufp->ufh_vbq, wait);
}

static int usbcam_v4l_mmap(struct file *filp, struct vm_area_struct * vma)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) filp->private_data;

	return videobuf_mmap_mapper(&ufp->ufh_vbq, vma);
}
/*
static int usbcam_v4l_vidiocgmbuf(struct file *filp, void *fh, struct video_mbuf *p)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	struct v4l2_requestbuffers req;
	unsigned int i;
	int res;

	// * APPBUG: motion keeps the first mmap, yet requests
	// * larger capture sizes.	
	usbcam_lock(udp);
	ufp->ufh_flags |= USBCAM_FH_USE_FIXED_FB;
	usbcam_unlock(udp);

	req.type = ufp->ufh_vbq.type;
	req.count = 2;
	req.memory = V4L2_MEMORY_MMAP;
	res = videobuf_reqbufs(&ufp->ufh_vbq, &req);
	if (res == -EBUSY)
	{
		usbcam_dbg(udp, IOCTL_BUF, "VIDIOCGMBUF reqbufs failed: device was busy"
                    			   " - closing and trying again."); 
		
		res = videobuf_streamoff(&ufp->ufh_vbq);
		if (res < 0)
		{
			usbcam_dbg(udp, IOCTL_BUF, "VIDIOCGMBUF reqbufs failed:"
            			        	   "couldn't free previous buffer.");
			return -EBUSY;
		}
		else
		{
			// we freed previous reqbuf OK.
			usbcam_lock(udp);
			ufp->ufh_flags |= USBCAM_FH_USE_FIXED_FB;
			usbcam_unlock(udp);

			req.type = ufp->ufh_vbq.type;
			req.count = 2;
			req.memory = V4L2_MEMORY_MMAP;
			res = videobuf_reqbufs(&ufp->ufh_vbq, &req);
		}
	}
	else if (res < 0) {
		usbcam_dbg(udp, IOCTL_BUF, "VIDIOCGMBUF reqbufs failed: %d", res);
		return res;
	}

	p->frames = req.count;
	p->size = 0;
	for (i = 0; i < p->frames; i++) {
		p->offsets[i] = ufp->ufh_vbq.bufs[i]->boff;
		p->size += ufp->ufh_vbq.bufs[i]->bsize;
	}

	usbcam_dbg(udp, IOCTL_BUF, "VIDIOCGMBUF frames=%d size=%d", p->frames, p->size);
	return 0;
}*/

static void usbcam_dbg_v4l2_buffer_res(struct usbcam_dev *udp, int res,
				       void *arg, const char *prefix)
{
	struct v4l2_buffer *b __attribute__((unused)) =
		(struct v4l2_buffer *) arg;

	if (res) {
		usbcam_dbg(udp, IOCTL_BUF, "%s res:%d", prefix, res);
		return;
	}

	usbcam_dbg(udp, IOCTL_BUF, "%s out: index=%d type=%d bytesused=%d "
		   "flags=0x%x field=%d memory=%d m=0x%lx length=%d",
		   prefix, b->index, b->type, b->bytesused,
		   b->flags, b->field, b->memory, b->m.userptr, b->length);
}

static void usbcam_dbg_v4l2_pix_format(struct usbcam_dev *udp,
				       struct v4l2_pix_format *f,
				       const char *prefix)
{
	__u32 pixfmt = f->pixelformat;
	if (!pixfmt)
		pixfmt = 0x3f3f3f3f;
	usbcam_dbg(udp, IOCTL_FMT, "%s wid=%d hgt=%d fmt=%.4s field=%d "
		   "bpl=%d size=%d cs=%d", prefix,
		   f->width, f->height, (char *) &pixfmt, f->field,
		   f->bytesperline, f->sizeimage, f->colorspace);
}

static void usbcam_dbg_v4l2_pix_format_res(struct usbcam_dev *udp, int res,
					   struct v4l2_pix_format *f,
					   const char *prefix)
{
	if (res) {
		usbcam_dbg(udp, IOCTL_FMT, "%s %d", prefix, res);
		return;
	}
	usbcam_dbg_v4l2_pix_format(udp, f, prefix);
}

static int usbcam_v4l_vidioc_reqbufs(struct file *filp, void *fh,
				     struct v4l2_requestbuffers *r)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	/* APPBUG: disable USE_FIXED_FB if we enter this path */
	usbcam_lock(udp);
	ufp->ufh_flags &= ~(USBCAM_FH_USE_FIXED_FB);
	usbcam_unlock(udp);

	usbcam_dbg(udp, IOCTL_BUF,
		   "VIDIOC_REQBUFS count=%d type=%d memory=%d",
		   r->count, r->type, r->memory);
	res = videobuf_reqbufs(&ufp->ufh_vbq, r);
	usbcam_dbg(udp, IOCTL_BUF, "REQBUFS result=%d", res);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_querybuf(struct file *filp, void *fh,
				      struct v4l2_buffer *b)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	usbcam_dbg(udp, IOCTL_BUF,
		   "VIDIOC_QUERYBUF in: index=%d type=%d",
		   b->index, b->type);
	res = videobuf_querybuf(&ufp->ufh_vbq, b);
	usbcam_dbg_v4l2_buffer_res(udp, res, b, "VIDIOC_QUERYBUF");
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_qbuf(struct file *filp, void *fh,
				  struct v4l2_buffer *b)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	/*
	 * APPBUG: ptlib / Ekiga has an issue with zeroing the
	 * flags field before calling QBUF.
	 *
	 * Minidriver support for fast input switching is
	 * unavailable for the time being.
	 */
	b->flags = 0;

	usbcam_dbg(udp, IOCTL_BUF, "VIDIOC_QBUF in: index=%d type=%d",
		   b->index, b->type);
	res = videobuf_qbuf(&ufp->ufh_vbq, b);
	usbcam_dbg_v4l2_buffer_res(udp, res, b, "VIDIOC_QBUF");
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_dqbuf(struct file *filp, void *fh,
				   struct v4l2_buffer *b)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	res = videobuf_dqbuf(&ufp->ufh_vbq, b,
			     (filp->f_flags & O_NONBLOCK) ? 1 : 0);
	usbcam_dbg_v4l2_buffer_res(udp, res, b, "VIDIOC_DQBUF");
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_streamon(struct file *filp, void *fh,
				      enum v4l2_buf_type f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	if (f != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		usbcam_dbg(udp, IOCTL_BUF,
			   "VIDIOC_STREAMON: invalid buf type %d", f);
		return -EINVAL;
	}
	if (!udp->ud_excl_owner) {
		usbcam_lock(udp);
		if (!udp->ud_excl_owner)
			udp->ud_excl_owner = filp;
		usbcam_unlock(udp);
	}
	if (udp->ud_excl_owner != filp) {
		usbcam_dbg(udp, IOCTL_BUF,
			   "VIDIOC_STREAMON: not exclusive owner");
		return -EBUSY;
	}
	res = videobuf_streamon(&ufp->ufh_vbq);
	usbcam_dbg(udp, IOCTL_BUF, "VIDIOC_STREAMON: res:%d", res);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_streamoff(struct file *filp, void *fh,
				       enum v4l2_buf_type f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	if (f != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		usbcam_dbg(udp, IOCTL_BUF,
			   "VIDIOC_STREAMOFF: invalid buf type %d", f);
		return -EINVAL;
	}
	res = videobuf_streamoff(&ufp->ufh_vbq);
	usbcam_dbg(udp, IOCTL_BUF, "VIDIOC_STREAMOFF: res:%d", res);
	usbcam_work_maybe_stop(udp);
	return res;
}


/* DEFAULT CAPABILITIES / DEVICE NAME / DRIVER NAME / BUS INFO */
static int usbcam_v4l_vidioc_querycap(struct file *filp, void *fh,
				      struct v4l2_capability *cap)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;

	usbcam_lock(udp);
	strlcpy(cap->driver,
		usbcam_drvname(udp->ud_minidrv),
		sizeof(cap->driver));
	strlcpy(cap->card, udp->ud_vdev.name, sizeof(cap->card));
    /*
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "usb:%s", udp->ud_dev->dev.bus_id);
         */
	cap->version = udp->ud_minidrv->um_version;
	cap->capabilities = (V4L2_CAP_VIDEO_CAPTURE |
			     V4L2_CAP_READWRITE |
			     V4L2_CAP_STREAMING);
	usbcam_unlock(udp);
	return 0;
}

/* DEFAULT FORMAT HANDLING - USE MINDIRIVER CALLOUTS */
static int usbcam_v4l_vidioc_enum_fmt_cap(struct file *filp, void *fh,
					  struct v4l2_fmtdesc *f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	struct usbcam_pix_fmt *pf;
	int res;

	usbcam_lock(udp);

	res = -EINVAL;
	if (f->index >= udp->ud_fmt_array_len)
		goto enum_fmt_done;

	res = 0;
	pf = (struct usbcam_pix_fmt *)
		&(((u8 *) udp->ud_fmt_array)
		  [f->index * udp->ud_fmt_array_elem_size]);
	f->flags = pf->flags;
	f->pixelformat = pf->pixelformat;
	strlcpy(f->description,
		pf->description,
		sizeof(f->description));

enum_fmt_done:
	usbcam_unlock(udp);
	return res;
}

static int usbcam_v4l_vidioc_g_fmt_cap(struct file *filp, void *fh,
				       struct v4l2_format *f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;

	usbcam_lock(udp);
	f->fmt.pix = udp->ud_format;
	usbcam_unlock(udp);
	usbcam_dbg_v4l2_pix_format(udp, &f->fmt.pix, "VIDIOC_G_FMT: res:");
	return 0;
}

static int usbcam_v4l_vidioc_s_fmt_cap(struct file *filp, void *fh,
				       struct v4l2_format *f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	usbcam_lock(udp);

	usbcam_dbg_v4l2_pix_format(udp, &f->fmt.pix,
				   "VIDIOC_S_FMT: param:");

	if (udp->ud_disconnected) {
		usbcam_dbg(udp, IOCTL_FMT,
			   "VIDIOC_S_FMT: device disconnected");
		res = -EIO;
		goto s_fmt_done;
	}
	if (!udp->ud_excl_owner)
		udp->ud_excl_owner = filp;
	if (!memcmp(&f->fmt.pix, &udp->ud_format, sizeof(f->fmt.pix))) {
		usbcam_dbg(udp, IOCTL_FMT, "VIDIOC_S_FMT: nothing to do");
		res = 0;
		goto s_fmt_done;
	}
	if (!usbcam_minidrv_op_present(udp, set_format)) {
		usbcam_dbg(udp, IOCTL_FMT, "VIDIOC_S_FMT: no minidriver op");
		res = -EINVAL;
		goto s_fmt_done;
	}
	if (udp->ud_excl_owner != filp) {
		usbcam_dbg(udp, IOCTL_FMT,
			   "VIDIOC_S_FMT: not exclusive owner");
		res = -EBUSY;
		goto s_fmt_done;
	}
	usbcam_capture_stop_nondestructive(udp);
	if (udp->ud_capturing) {
		usbcam_dbg(udp, IOCTL_FMT,
			   "VIDIOC_S_FMT: capture in progress");
		res = -EBUSY;
		goto s_fmt_done;
	}
	res = usbcam_minidrv_op(udp, set_format, &f->fmt.pix);
	usbcam_dbg_v4l2_pix_format_res(udp, res, &f->fmt.pix,
				       "VIDIOC_S_FMT: res:");

s_fmt_done:
	usbcam_unlock(udp);
	return res;
}

static int usbcam_v4l_vidioc_try_fmt_cap(struct file *filp, void *fh,
					 struct v4l2_format *f)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	int res;

	usbcam_lock(udp);

	usbcam_dbg_v4l2_pix_format(udp, &f->fmt.pix,
				   "VIDIOC_TRY_FMT: param:");

	if (udp->ud_disconnected) {
		usbcam_dbg(udp, IOCTL_FMT,
			   "VIDIOC_TRY_FMT: device disconnected");
		res = -EIO;
		goto try_fmt_done;
	}
	if (!usbcam_minidrv_op_present(udp, try_format)) {
		usbcam_dbg(udp, IOCTL_FMT,
			   "VIDIOC_TRY_FMT: no minidriver op");
		res = -EINVAL;
		goto try_fmt_done;
	}

	res = usbcam_minidrv_op(udp, try_format, &f->fmt.pix);
	usbcam_dbg_v4l2_pix_format_res(udp, res, &f->fmt.pix,
				       "VIDIOC_TRY_FMT: res:");

try_fmt_done:
	usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}

/* DEFAULT CONTROL HANDLING - USE MINIDRIVER ARRAY / CALLOUTS */
static int usbcam_v4l_vidioc_queryctrl(struct file *filp, void *fh,
				       struct v4l2_queryctrl *a)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	const struct usbcam_ctrl *ctrlp, *resp;
	int droplock;
	u32 targ;
	int higher_ids = 0, res = -EINVAL;

	usbcam_lock(udp);
	droplock = 1;
	targ = a->id;

	resp = NULL;

	if (targ & V4L2_CTRL_FLAG_NEXT_CTRL) {
		/*
		 * Find the control with the least ID greater than or
		 * equal to a->id
		 */
		targ &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		list_for_each_entry(ctrlp, &udp->ud_ctrl_list, uc_links) {
			if (ctrlp->uc_v4l.id <= targ) {
				if (!resp || (ctrlp->uc_v4l.id <
					      resp->uc_v4l.id))
					resp = ctrlp;
			}
		}

	} else {
		/* Find an exact match */
		list_for_each_entry(ctrlp, &udp->ud_ctrl_list, uc_links) {
			if (ctrlp->uc_v4l.id == targ) {
				resp = ctrlp;
				break;
			}
			else if ((ctrlp->uc_v4l.id >=
				  V4L2_CID_PRIVATE_BASE) &&
				 (ctrlp->uc_v4l.id <
				  (V4L2_CID_PRIVATE_BASE + 1024)) &&
				 (targ >= V4L2_CID_PRIVATE_BASE) &&
				 (targ < ctrlp->uc_v4l.id)) {
				higher_ids = 1;
			}
		}

		if (!resp && higher_ids) {
			/*
			 * Deal with the private control enumeration
			 * nonsense that the CTRL_FLAG_NEXT_CTRL thing
			 * fixes.
			 */
			memset(a, 0, sizeof(*a));
			a->id = targ;
			a->type = V4L2_CTRL_TYPE_INTEGER;
			strlcpy(a->name, "Disabled", sizeof(a->name));
			a->flags = V4L2_CTRL_FLAG_DISABLED;
			res = 0;
		}
	}

	if (resp) {
		*a = resp->uc_v4l;
		res = 0;

		/* Fill in required values for certain types */
		switch (a->type) {
		case V4L2_CTRL_TYPE_BOOLEAN:
			a->minimum = 0;
			a->maximum = 1;
			a->step = 1;
			break;
		case V4L2_CTRL_TYPE_MENU:
			a->step = 1;
			break;
		default:
			break;
		}

		/*
		 * If a query function was provided, call it to
		 * postprocess the response structure, e.g. to set
		 * flags.
		 */
		if (resp->query_fn) {
			if (udp->ud_minidrv->um_ops->unlocked_ctrl) {
				usbcam_unlock(udp);
				droplock = 0;
			}
			res = resp->query_fn(udp, resp, a);
		}
	}

	if (droplock)
		usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_g_ctrl(struct file *filp, void *fh,
				    struct v4l2_control *a)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	const struct usbcam_ctrl *resp;
	struct v4l2_ext_control ec;
	int droplock;
	int res;

	usbcam_lock(udp);
	droplock = 1;

	if (udp->ud_disconnected) {
		usbcam_unlock(udp);
		return -EIO;
	}

	resp = usbcam_ctrl_find(udp, a->id);
	if (!resp ||
	    (resp->uc_v4l.type == V4L2_CTRL_TYPE_BUTTON) ||
	    !resp->get_fn)
		res = -EINVAL;
	else {
		if (udp->ud_minidrv->um_ops->unlocked_ctrl) {
			usbcam_unlock(udp);
			droplock = 0;
		}
		memset(&ec, 0, sizeof(ec));
		ec.id = a->id;
		ec.value = a->value;
		res = resp->get_fn(udp, resp, &ec);
		memset(a, 0, sizeof(*a));
		a->id = ec.id;
		a->value = ec.value;
	}

	if (droplock)
		usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_s_ctrl(struct file *filp, void *fh,
				    struct v4l2_control *a)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	const struct usbcam_ctrl *resp;
	struct v4l2_ext_control ec;
	int droplock;
	int res;

	usbcam_lock(udp);
	droplock = 1;

	if (udp->ud_disconnected) {
		usbcam_unlock(udp);
		return -EIO;
	}

	resp = usbcam_ctrl_find(udp, a->id);
	if (!resp) {
		res = -EINVAL;
	} else if (!resp->set_fn) {
		/* Read-only control */
		res = -EBUSY;
	} else if ((resp->uc_v4l.type != V4L2_CTRL_TYPE_BUTTON) &&
		   ((a->value < resp->uc_v4l.minimum) ||
		    (a->value > resp->uc_v4l.maximum))) {
		res = -ERANGE;
	} else {
		if (udp->ud_minidrv->um_ops->unlocked_ctrl) {
			usbcam_unlock(udp);
			droplock = 0;
		}
		memset(&ec, 0, sizeof(ec));
		ec.id = a->id;
		ec.value = a->value;
		res = resp->set_fn(udp, resp, &ec);
		memset(a, 0, sizeof(*a));
		a->id = ec.id;
		a->value = ec.value;
	}

	if (droplock)
		usbcam_unlock(udp);
	usbcam_work_maybe_stop(udp);
	return res;
}

static int usbcam_v4l_vidioc_querymenu(struct file *filp, void *fh,
				       struct v4l2_querymenu *a)
{
	struct usbcam_fh *ufp = (struct usbcam_fh *) fh;
	struct usbcam_dev *udp = ufp->ufh_dev;
	const struct usbcam_ctrl *resp;
	int res;

	usbcam_lock(udp);

	resp = usbcam_ctrl_find(udp, a->id);

	if (!resp ||
	    (resp->uc_v4l.type != V4L2_CTRL_TYPE_MENU) ||
	    (a->index > resp->uc_v4l.maximum)) {
		res = -EINVAL;
		goto querymenu_done;
	}

	strlcpy(a->name,
		resp->uc_menu_names[a->index],
		sizeof(a->name));
	res = 0;

querymenu_done:
	usbcam_unlock(udp);
	return res;
}

/* DEFAULT INPUT HANDLING -- There is one input called "Camera" */
static int usbcam_v4l_vidioc_enum_input(struct file *filp, void *fh,
					struct v4l2_input *inp)
{
	const struct v4l2_input dfl_input = {
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
	};

	if (inp->index > 0)
		return -EINVAL;
	*inp = dfl_input;
	return 0;
}

static int usbcam_v4l_vidioc_g_input(struct file *filp, void *fh,
				     unsigned int *i)
{
	*i = 0;
	return 0;
}

static int usbcam_v4l_vidioc_s_input(struct file *filp, void *fh,
				     unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

/* Intercept calls to minidriver V4L handler thing for compat calls. */
/* 3pei: not used at current
static long usbcam_v4l_int_ioctl(struct file *filp,
				unsigned int cmd, void *arg)
{
    struct usbcam_fh        *ufp = (struct usbcam_fh *) filp->private_data;
	struct usbcam_dev       *udp = ufp->ufh_dev;
	
	struct v4l2_buffer      buf2;
	enum v4l2_buf_type      captype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int                     err = 0;
	
	
	if (cmd == VIDIOCGCAP) {
	    struct video_capability *cap = (struct video_capability *) arg;
		
		usbcam_lock(udp);
		
		strlcpy(cap->name, udp->ud_vdev.name, sizeof(cap->name));
		cap->audios = 0;
		cap->channels = 1;	// only one input source, the camera 
		
		cap->maxwidth = udp->ud_format.width;
		cap->maxheight = udp->ud_format.height;
		
		// * We lie, here. These values normally return 640x480, which is
		// * actually the maximum, not the minimum. Minimum is usually
		// * 160x120. It's sort of useful to lie since lots of software
		// * just stick with the minimum - we want higher res for the
		// * user where possible.
		
		cap->minwidth = udp->ud_format.width;
		cap->minheight = udp->ud_format.height;
		
		usbcam_unlock(udp);
		return 0;
	}
	else if (cmd == VIDIOCGCHAN) {
	    struct video_channel *chan = (struct video_channel *) arg;
	    
	    usbcam_lock(udp);
	    
	    chan->channel = 0;
	    strlcpy(chan->name, udp->ud_vdev.name, sizeof(chan->name));
	    chan->tuners = 0;
	    chan->type = VIDEO_TYPE_CAMERA;
	    
	    usbcam_unlock(udp);
	    return 0;
	}
	else if (cmd == VIDIOCSCHAN) {
	    struct video_channel *chan = (struct video_channel *) arg;
	    
	    if (chan->norm != 0)
		    return -EINVAL;
	    return 0;
	}
	else if (cmd == VIDIOCGAUDIO) {
	    return -ENOIOCTLCMD;
	}
	else if (cmd == VIDIOCGTUNER) {
	    return -ENOIOCTLCMD;
	}
	else if (cmd == VIDIOCGPICT) {
	    return -ENOIOCTLCMD;
	}
	else if (cmd == VIDIOCGWIN) {
	    struct video_window *win = (struct video_window *) arg;
	    
	    usbcam_lock(udp);
	    
	    win->x = 0;
	    win->y = 0;
	    win->width = udp->ud_format.width;
	    win->height = udp->ud_format.height;
	    win->chromakey = 0;
	    win->clips = NULL;
	    win->clipcount = 0;
	    win->flags = 0;
	    
	    usbcam_unlock(udp);
	    return 0;
	}
	else if (cmd == VIDIOCGFBUF) {
		struct video_buffer *buf = (struct video_buffer *) arg;
		
		usbcam_lock(udp);
		
		buf->base = NULL;	// no physical frame buffer access 
		buf->height = udp->ud_format.height;
		buf->width = udp->ud_format.width;
		
		
		// * graciously stolen from drivers/media/video/v4l1-compat.c
		// * and modified slightly.
		 
		switch (udp->ud_format.pixelformat) {
		case V4L2_PIX_FMT_RGB332:
			buf->depth = 8;
			break;
		case V4L2_PIX_FMT_RGB555:
			buf->depth = 15;
			break;
		case V4L2_PIX_FMT_RGB565:
			buf->depth = 16;
			break;
		case V4L2_PIX_FMT_BGR24:
			buf->depth = 24;
			break;
		case V4L2_PIX_FMT_BGR32:
			buf->depth = 32;
			break;
		default:
			buf->depth = 0;
		}
		
		if (udp->ud_format.bytesperline) {
			buf->bytesperline = udp->ud_format.bytesperline;
			
			// typically comes out at 16 bit depth as non-rgb 
			if (!buf->depth && buf->width)
				buf->depth   = ((udp->ud_format.bytesperline<<3)
						  + (buf->width-1) )
						  /buf->width;
		} else {
			buf->bytesperline =
				(buf->width * buf->depth + 7) & 7;
			buf->bytesperline >>= 3;
		}
		
		usbcam_unlock(udp);
		return 0;
	}
	else if (cmd == VIDIOCGMBUF) {
	    struct video_mbuf *mbuf = (struct video_mbuf *) arg;
	    return usbcam_v4l_vidiocgmbuf(filp, filp->private_data, mbuf);
	}
	else if (cmd == VIDIOCSFBUF) {
	    usbcam_warn(udp, "VIDIOCSFBUF called.");
	    return -ENOIOCTLCMD;
	}
	else if (cmd == VIDIOCSWIN) {
	    return -ENOIOCTLCMD;
	}
	else if (cmd == VIDIOCMCAPTURE) {
	    struct v4l2_format      *fmt2  = NULL;
	    struct video_mmap	     *mm    = arg;

		fmt2 = kzalloc(sizeof(*fmt2),GFP_KERNEL);
		memset(&buf2,0,sizeof(buf2));

		fmt2->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		err = usbcam_v4l_vidioc_g_fmt_cap(filp, filp->private_data, fmt2);
		if (err < 0) {
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCMCAPTURE / VIDIOC_G_FMT: %d\n",err);
			return err;
		}
		if (mm->width   != fmt2->fmt.pix.width  ||
		    mm->height  != fmt2->fmt.pix.height ||
		    palette_to_pixelformat(mm->format) !=
		    fmt2->fmt.pix.pixelformat)
		{// New capture format...  
			fmt2->fmt.pix.width = mm->width;
			fmt2->fmt.pix.height = mm->height;
			fmt2->fmt.pix.pixelformat =
				palette_to_pixelformat(mm->format);
			fmt2->fmt.pix.field = V4L2_FIELD_ANY;
			fmt2->fmt.pix.bytesperline = 0;
			err = usbcam_v4l_vidioc_s_fmt_cap(filp, filp->private_data, fmt2);
			if (err < 0) {
				usbcam_dbg(udp, IOCTL_MISC, "VIDIOCMCAPTURE / VIDIOC_S_FMT: %d\n",err);
				return err;
			}
		}
		buf2.index = mm->frame;
		buf2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		err = usbcam_v4l_vidioc_querybuf(filp, filp->private_data, &buf2);
		if (err < 0) {
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCMCAPTURE / VIDIOC_QUERYBUF: %d\n",err);
			return err;
		}
		err = usbcam_v4l_vidioc_qbuf(filp, filp->private_data, &buf2);
		if (err < 0) {
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCMCAPTURE / VIDIOC_QBUF: %d\n",err);
			return err;
		}
		err = usbcam_v4l_vidioc_streamon(filp, filp->private_data, captype);
		if (err < 0)
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCMCAPTURE / VIDIOC_STREAMON: %d\n",err);
		return 0;
	}
	else if (cmd == VIDIOCSYNC) {
	    int			*i = arg;

		memset(&buf2,0,sizeof(buf2));
		buf2.index = *i;
		buf2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		err = usbcam_v4l_vidioc_querybuf(filp, filp->private_data, &buf2);
		if (err < 0) {
			//  No such buffer 
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCSYNC / VIDIOC_QUERYBUF: %d\n",err);
			return err;
		}
		if (!(buf2.flags & V4L2_BUF_FLAG_MAPPED)) {
			// Buffer is not mapped  
			err = -EINVAL;
			return err;
		}

		// make sure capture actually runs so we don't block forever 
		err = usbcam_v4l_vidioc_streamon(filp, filp->private_data, captype);
		if (err < 0) {
			usbcam_dbg(udp, IOCTL_MISC, "VIDIOCSYNC / VIDIOC_STREAMON: %d\n",err);
			return err;
		}

		//  Loop as long as the buffer is queued, but not done  
		while ((buf2.flags &
			(V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE))
		       == V4L2_BUF_FLAG_QUEUED)
		{
			err = poll_one(filp);
			if (err < 0 ||	// error or sleep was interrupted  
			    err == 0)	// timeout? Shouldn't occur.  
				return err;
			err = usbcam_v4l_vidioc_querybuf(filp, filp->private_data, &buf2);
			if (err < 0)
				usbcam_dbg(udp, IOCTL_MISC, "VIDIOCSYNC / VIDIOC_QUERYBUF: %d\n",err);
		}
		if (!(buf2.flags & V4L2_BUF_FLAG_DONE)) // not done
			return err;
		do {
		    err = usbcam_v4l_vidioc_dqbuf(filp, filp->private_data, &buf2);
			if (err < 0)
				usbcam_dbg(udp, IOCTL_MISC, "VIDIOCSYNC / VIDIOC_DQBUF: %d\n",err);
		} while (err == 0 && buf2.index != *i);
		return err;
	}
	else {
	    usbcam_warn(udp, "usbcam_v4l_int_ioctl called without valid ioctl");
	    return -ENOIOCTLCMD;
	}
}
*/
static long usbcam_v4l_ioctl (struct file *file,
	       unsigned int cmd, unsigned long arg)
{
    //struct usbcam_fh        *ufp = (struct usbcam_fh *) file->private_data;
	//struct usbcam_dev       *udp = ufp->ufh_dev;
	//usbcam_dbg(udp, IOCTL_MISC, "received V4L ioctl: %d\n", cmd);
	usbcam_dbg(((struct usbcam_fh *) file->private_data)->ufh_dev, IOCTL_MISC, "received V4L ioctl: %d\n", cmd);
	
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	if (_IOC_TYPE(cmd) == 'v')
	{
		// run our own internal ioctl handler for these V4L compat ioctl.
		return video_usercopy(file, cmd, arg, usbcam_v4l_int_ioctl);
	}
#endif
	
	return video_ioctl2(file, cmd, arg);
}

/*
 * The template file_operations structure
 *
 * Each usbcam_minidrv_t contains its own copy of this, which
 * is associated with the video4linux device created for that
 * minidriver.
 *
 * In general, copies will differ only in the .owner field, which
 * will refer to the minidriver module, not usbcam.
 */

struct v4l2_file_operations usbcam_v4l_fops_template = {
	.owner		= THIS_MODULE,
	.open		= usbcam_v4l_open,
	.release	= usbcam_v4l_release,
	.read		= usbcam_v4l_read,
	.poll		= usbcam_v4l_poll,
	.mmap		= usbcam_v4l_mmap,
	/*.ioctl		= video_ioctl2,*/
	.ioctl		= usbcam_v4l_ioctl,
#ifdef CONFIG_COMPAT && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	.compat_ioctl	= v4l_compat_ioctl32,
#endif
};


/*
 * The template video_device structure
 *
 * Each usbcam_dev contains its own copy of this.  The minidriver is
 * free to install its own handlers for each interface, although it
 * should take care not to screw up the frame buffer handling.
 *
 * This gets installed via video_register_device() from usb_usbcam_probe().
 */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26)
static const struct v4l2_ioctl_ops this_cam_ops = {
	.vidioc_querycap	= usbcam_v4l_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= usbcam_v4l_vidioc_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap	= usbcam_v4l_vidioc_g_fmt_cap,
	.vidioc_s_fmt_vid_cap	= usbcam_v4l_vidioc_s_fmt_cap,
	.vidioc_try_fmt_vid_cap	= usbcam_v4l_vidioc_try_fmt_cap,
	.vidioc_reqbufs		= usbcam_v4l_vidioc_reqbufs,
	.vidioc_querybuf	= usbcam_v4l_vidioc_querybuf,
	.vidioc_qbuf		= usbcam_v4l_vidioc_qbuf,
	.vidioc_dqbuf		= usbcam_v4l_vidioc_dqbuf,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38) /* 3pei: for CONFIG_VIDEO_V4L1_COMPAT */
	.vidiocgmbuf		= usbcam_v4l_vidiocgmbuf,
#endif
	.vidioc_enum_input	= usbcam_v4l_vidioc_enum_input,
	.vidioc_streamon	= usbcam_v4l_vidioc_streamon,
	.vidioc_streamoff	= usbcam_v4l_vidioc_streamoff,
	.vidioc_g_input		= usbcam_v4l_vidioc_g_input,
	.vidioc_s_input		= usbcam_v4l_vidioc_s_input,
	.vidioc_queryctrl	= usbcam_v4l_vidioc_queryctrl,
	.vidioc_g_ctrl		= usbcam_v4l_vidioc_g_ctrl,
	.vidioc_s_ctrl		= usbcam_v4l_vidioc_s_ctrl,
	.vidioc_querymenu	= usbcam_v4l_vidioc_querymenu,
};

struct video_device usbcam_videodev_template = {
	.name			= "usbcam-unknown",
	.vfl_type		= VFL_TYPE_GRABBER,
	.minor			= -1,
	.fops                   = &usbcam_v4l_fops_template,
	.ioctl_ops		= &this_cam_ops,

};
#else
struct video_device usbcam_videodev_template = {
	.name			= "usbcam-unknown",
	.type			= VFL_TYPE_GRABBER,
	.minor			= -1,

	.vidioc_querycap	= usbcam_v4l_vidioc_querycap,
	.vidioc_enum_fmt_cap	= usbcam_v4l_vidioc_enum_fmt_cap,
	.vidioc_g_fmt_cap	= usbcam_v4l_vidioc_g_fmt_cap,
	.vidioc_s_fmt_cap	= usbcam_v4l_vidioc_s_fmt_cap,
	.vidioc_try_fmt_cap	= usbcam_v4l_vidioc_try_fmt_cap,
	.vidioc_reqbufs		= usbcam_v4l_vidioc_reqbufs,
	.vidioc_querybuf	= usbcam_v4l_vidioc_querybuf,
	.vidioc_qbuf		= usbcam_v4l_vidioc_qbuf,
	.vidioc_dqbuf		= usbcam_v4l_vidioc_dqbuf,
	.vidiocgmbuf		= usbcam_v4l_vidiocgmbuf,
	.vidioc_enum_input	= usbcam_v4l_vidioc_enum_input,
	.vidioc_streamon	= usbcam_v4l_vidioc_streamon,
	.vidioc_streamoff	= usbcam_v4l_vidioc_streamoff,
	.vidioc_g_input		= usbcam_v4l_vidioc_g_input,
	.vidioc_s_input		= usbcam_v4l_vidioc_s_input,
	.vidioc_queryctrl	= usbcam_v4l_vidioc_queryctrl,
	.vidioc_g_ctrl		= usbcam_v4l_vidioc_g_ctrl,
	.vidioc_s_ctrl		= usbcam_v4l_vidioc_s_ctrl,
	.vidioc_querymenu	= usbcam_v4l_vidioc_querymenu,
};
#endif
