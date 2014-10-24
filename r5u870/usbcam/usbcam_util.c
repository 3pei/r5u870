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

#include "usbcam_priv.h"

/*
 * This file contains utility routines, including:
 * - usbcam_choose_altsetting() for selecting int/isoc altsettings
 * - The usbcam_urbstream accessory module
 * - usbcam_hexdump() buffer dumping utility function
 *
 * TODO:
 * - Find some way to alert minidrivers of URB shortages
 */

/*
 * Traverse the alternate setting list and find one that provides
 * the least bandwidth that satisfies the minimum requirement.
 */
int usbcam_choose_altsetting(struct usbcam_dev *udp, int ifnum,
			     int pipe, int bytes_per_sec_min,
			     int pkt_min, int pkt_max,
			     int *altsetting_nr)
{
	struct usb_interface *intf;
	const struct usb_host_interface *aintf;
	const struct usb_endpoint_descriptor *epd = NULL;
	int i, j;

	int wmp, bw;
	int best_alt = -1, best_alt_bw = 0;

	usbcam_chklock(udp);

	if (udp->ud_disconnected) {
		usbcam_warn(udp, "%s: device is disconnected", __FUNCTION__);
		return -ENODEV;
	}

	if (ifnum < 0)
		ifnum = udp->ud_intf->cur_altsetting->desc.bInterfaceNumber;

	intf = usb_ifnum_to_if(udp->ud_dev, ifnum);
	if (!intf) {
		usbcam_warn(udp, "%s: interface %d does not exist",
			    __FUNCTION__, ifnum);
		return -ENODEV;
	}

	if ((bytes_per_sec_min >= 0) &&
	    !usb_pipeisoc(pipe) && !usb_pipeint(pipe)) {
		usbcam_warn(udp, "%s: minidriver specified bytes_per_sec_min "
			    "on non-iso non-int pipe", __FUNCTION__);
	}

	for (i = 0; i < intf->num_altsetting; i++) {

		aintf = &intf->altsetting[i];
		for (j = 0; j < aintf->desc.bNumEndpoints; j++) {
			epd = &aintf->endpoint[j].desc;
			if ((epd->bEndpointAddress &
			     USB_ENDPOINT_NUMBER_MASK) ==
			    usb_pipeendpoint(pipe))
				break;
		}

		if (j == aintf->desc.bNumEndpoints) {
			/* Desired endpoint not present in this descriptor */
			usbcam_dbg(udp, ALTSETTING,
				   "altsetting %d has no EP%d",
				   i, usb_pipeendpoint(pipe));
			continue;
		}

		if (((usb_pipetype(pipe) == PIPE_ISOCHRONOUS) &&
		     !usb_endpoint_xfer_isoc(epd)) ||
		    ((usb_pipetype(pipe) == PIPE_INTERRUPT) &&
		     !usb_endpoint_xfer_int(epd)) ||
		    (usb_pipein(pipe) && !usb_endpoint_dir_in(epd)) ||
		    (!usb_pipein(pipe) && usb_endpoint_dir_in(epd))) {
			/* Something is horribly wrong */
			usbcam_dbg(udp, ALTSETTING,
				   "altsetting %d has unexpected EP%d",
				   i, usb_pipeendpoint(pipe));
			continue;
		}

		bw = 0;
		wmp = le16_to_cpu(epd->wMaxPacketSize);

		/* Bandwidth only applies to iso & int pipes */
		if (usb_pipeisoc(pipe) || usb_pipeint(pipe)) {
			if (udp->ud_dev->speed == USB_SPEED_HIGH) {
				/* 8 uframes per regular frame */
				bw = 8000;

				/* high bandwidth endpoint? */
				wmp = ((wmp & 0x7ff) *
				       (((wmp >> 11) & 0x3) + 1));
			} else {
				bw = 1000;
				wmp &= 0x7ff;
			}

			bw *= wmp;

			/* Divide by interval / frame skippage */
			bw = bw / (1 << (epd->bInterval - 1));

			usbcam_dbg(udp, ALTSETTING,
				   "altsetting %d provides %d B/s bandwidth",
				   i, bw);

			/* Check the bandwidth */
			if (bw < bytes_per_sec_min)
				continue;

		} else
			wmp &= 0x7ff;

		/* Check the packet size */
		if (((pkt_min >= 0) && (wmp < pkt_min)) ||
		    ((pkt_max >= 0) && (wmp > pkt_max)))
			continue;

		if ((best_alt < 0) || (bw < best_alt_bw)) {
			best_alt = i;
			best_alt_bw = bw;
		}
	}

	if (best_alt == -1)
		return -ENODEV;

	*altsetting_nr = best_alt;
	return 0;
}
USBCAM_EXPORT_SYMBOL(usbcam_choose_altsetting);


/*
 * DMA buffer helper routines
 */
static int usbcam_urb_allocbuf(struct urb *urbp, size_t nbytes)
{
/* 3pei */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)		
	urbp->transfer_buffer = usb_alloc_coherent(urbp->dev,
						 nbytes,
						 GFP_KERNEL,
						 &urbp->transfer_dma);
#else
	urbp->transfer_buffer = usb_buffer_alloc(urbp->dev,
						 nbytes,
						 GFP_KERNEL,
						 &urbp->transfer_dma);
#endif
	if (!urbp->transfer_buffer)
		return -ENOMEM;

	urbp->transfer_buffer_length = nbytes;
	urbp->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	return 0;
}

static inline void usbcam_urb_freebuf(struct urb *urbp)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)		
	usb_free_coherent(urbp->dev,
			urbp->transfer_buffer_length,
			urbp->transfer_buffer,
			urbp->transfer_dma);
#else
	usb_buffer_free(urbp->dev,
			urbp->transfer_buffer_length,
			urbp->transfer_buffer,
			urbp->transfer_dma);
#endif
}

struct urb *usbcam_urb_alloc(struct usbcam_dev *udp, int pipe,
			     int pktsize, int npkts, int alloc_buf)
{
	struct urb *urbp;
	int nbytes, i;

	if (npkts) {
		if (!usb_pipeisoc(pipe)) {
			usbcam_err(udp, "npkts=%d but !pipeisoc", npkts);
			return NULL;
		}
	} else if (usb_pipeisoc(pipe)) {
		usbcam_err(udp, "npkts=0 but pipeisoc");
		return NULL;
	}


	urbp = usb_alloc_urb(npkts, GFP_KERNEL);
	if (!urbp)
		return NULL;

	urbp->dev = udp->ud_dev;
	urbp->pipe = pipe;
	urbp->number_of_packets = npkts;
	urbp->transfer_buffer = NULL;
	urbp->transfer_buffer_length = 0;

	if (alloc_buf) {
		nbytes = pktsize;
		if (npkts)
			nbytes *= npkts;

		if (usbcam_urb_allocbuf(urbp, nbytes)) {
			usb_free_urb(urbp);
			return NULL;
		}
	}

	if (npkts)
		urbp->transfer_flags |= URB_ISO_ASAP;

	for (i = 0; i < npkts; i++) {
		urbp->iso_frame_desc[i].offset = (i * pktsize);
		urbp->iso_frame_desc[i].length = pktsize;
		urbp->iso_frame_desc[i].actual_length = 0;
		urbp->iso_frame_desc[i].status = 0;
	}

	return urbp;
}
USBCAM_EXPORT_SYMBOL(usbcam_urb_alloc);

void usbcam_urb_free(struct urb *urbp, int free_buf)
{
	if (free_buf)
		usbcam_urb_freebuf(urbp);
	usb_free_urb(urbp);
}
USBCAM_EXPORT_SYMBOL(usbcam_urb_free);


/*
 * Streaming request helper functions
 * This depends on the other usbcam stuff, but they don't depend on it,
 * and it should be considered an extension sub-library.
 */

/* Default parameters for config_iso and config_bulk */
#define USBCAM_DFL_ISO_REQS		8
#define USBCAM_DFL_ISO_URB_PKTS		32

#define USBCAM_DFL_BULK_REQS		8


/*
 * This structure represents a set of USB requests - URBs and buffers
 */
struct usbcam_urbinfo {
	struct list_head		ib_links;
	struct usbcam_urbstream		*ib_urbstream;
	struct task_struct		**ib_worker;
	struct usbcam_workitem		ib_workitem;
	struct timer_list		ib_timeout;
	unsigned short			ib_nurbs;
	unsigned short			ib_cururb;
	struct urb			*ib_urbs[0];
};

/* usp->us_lock must be held on entry */
static void usbcam_urbstream_resubmit(struct usbcam_urbstream *usp,
				      struct usbcam_urbinfo *ibp)
{
	int res;

	if (!ibp->ib_cururb) {
		list_del(&ibp->ib_links);
		list_add(&ibp->ib_links, &usp->us_active_list);
		usp->us_active_count++;
	}

	res = usb_submit_urb(ibp->ib_urbs[ibp->ib_cururb], GFP_ATOMIC);
	if (res == -EL2NSYNC)
		res = usb_submit_urb(ibp->ib_urbs[ibp->ib_cururb], GFP_ATOMIC);

	if (res) {
 		usbcam_dbg(usp->us_dev, URBSTREAM,
			   "urbstream[%d] resubmit %p/%d failed: %d",
			   usp->us_endpoint, ibp, ibp->ib_cururb, res);
		usp->us_resubmit_err = res;

		ibp->ib_cururb = 0;
		list_del(&ibp->ib_links);
		list_add(&ibp->ib_links, &usp->us_unused_list);
		if (!--usp->us_active_count)
			complete(&usp->us_active_empty);

		(void) usbcam_work_queue(&usp->us_error_workitem);
	}

	else if (usp->us_timeout_ticks) {
		ibp->ib_timeout.expires = jiffies + usp->us_timeout_ticks;
		add_timer(&ibp->ib_timeout);
	}
}

/* usp->us_lock NOT held on entry */
static int usbcam_urbstream_submit_unused(struct usbcam_urbstream *usp)
{
	struct usbcam_urbinfo *ibp = NULL;
	unsigned long flags;
	int res;

	spin_lock_irqsave(&usp->us_lock, flags);

	if (!list_empty(&usp->us_unused_list)) {
		ibp = list_entry(usp->us_unused_list.next,
				 struct usbcam_urbinfo, ib_links);
		list_del(&ibp->ib_links);
		list_add_tail(&ibp->ib_links, &usp->us_active_list);
		usp->us_active_count++;
		ibp->ib_cururb = 0;
	}

	spin_unlock_irqrestore(&usp->us_lock, flags);

	if (!ibp)
		return -ENOENT;

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] urb submit %p/%d",
		   usp->us_endpoint, ibp, ibp->ib_cururb);
	res = usb_submit_urb(ibp->ib_urbs[ibp->ib_cururb], GFP_KERNEL);
	if (res == -EL2NSYNC)
		res = usb_submit_urb(ibp->ib_urbs[ibp->ib_cururb], GFP_KERNEL);

	if (!res && usp->us_timeout_ticks) {
		ibp->ib_timeout.expires = jiffies + usp->us_timeout_ticks;
		add_timer(&ibp->ib_timeout);
	}

	if (res) {
		usbcam_dbg(usp->us_dev, URBSTREAM, "%s: URB submit failed: %d",
			   __FUNCTION__, res);
		spin_lock_irqsave(&usp->us_lock, flags);
		list_del(&ibp->ib_links);
		list_add_tail(&ibp->ib_links, &usp->us_unused_list);
		assert(usp->us_active_count > 0);
		usp->us_active_count--;
		spin_unlock_irqrestore(&usp->us_lock, flags);
	}

	return res;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void usbcam_urbstream_urb_complete(struct urb *urb)
#else
static void usbcam_urbstream_urb_complete(struct urb *urb,
					  struct pt_regs *unused)
#endif
{
	struct usbcam_urbinfo *ibp = (struct usbcam_urbinfo *) urb->context;
	struct usbcam_urbstream *usp = ibp->ib_urbstream;
	unsigned long flags;
	int timer_fired = 0, res;

	assert(ibp->ib_urbs[ibp->ib_cururb] == urb);

	if (usp->us_timeout_ticks)
		timer_fired = !del_timer_sync(&ibp->ib_timeout);

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] urb complete: %p/%d",
		   usp->us_endpoint, ibp, ibp->ib_cururb);

	spin_lock_irqsave(&usp->us_lock, flags);

	if (list_empty(&ibp->ib_links)) {
		/* We are being singled out for cancelation, do nothing */
		usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] "
			   "request canceled, ignoring", usp->us_endpoint);
		goto done;
	}

	ibp->ib_cururb++;
	if (!urb->status) {
		if (ibp->ib_cururb != ibp->ib_nurbs) {
			usbcam_urbstream_resubmit(usp, ibp);
			goto done;
		}
	}

	/* Move to the done queue, submit a new URB, wake */
	list_del(&ibp->ib_links);
	list_add_tail(&ibp->ib_links, &usp->us_complete_list);
	assert(usp->us_active_count > 0);
	usp->us_active_count--;

	if ((urb->status == -ECONNRESET) && timer_fired) {
		usbcam_dbg(usp->us_dev, URBSTREAM,
			   "urbstream[%d] urb %p/%d timed out",
			   usp->us_endpoint, ibp, ibp->ib_cururb);
		urb->status = -ETIMEDOUT;
	}

	res = usbcam_work_queue(&ibp->ib_workitem);
	if (res) {
		assert(res == -EBUSY);
		list_del(&ibp->ib_links);
		list_add_tail(&ibp->ib_links, &usp->us_unused_list);
		goto done;
	}

	if (usp->us_resubmit_err) {
		usbcam_dbg(usp->us_dev, URBSTREAM,
			   "urbstream[%d] not resubmitting, pending error",
			   usp->us_endpoint);
		goto done;
	}

	/* Is the active count sufficient? */
	if (usp->us_active_count >= usp->us_active_goal)
		goto done;

	/* Should we automatically submit an URB from the unused list? */
	if (usp->us_streaming) {
		if (list_empty(&usp->us_unused_list)) {
			usbcam_dbg(usp->us_dev, URBSTREAM,
				   "urbstream[%d] not resubmitting, "
				   "URBs exhausted",
				   usp->us_endpoint);
			goto done;
		}

		ibp = list_entry(usp->us_unused_list.next,
				 struct usbcam_urbinfo, ib_links);

		ibp->ib_cururb = 0;
		usbcam_urbstream_resubmit(usp, ibp);
	}

done:
	if (!usp->us_active_count)
		complete(&usp->us_active_empty);
	spin_unlock_irqrestore(&usp->us_lock, flags);
}

static void usbcam_urbstream_urb_timeout(unsigned long data)
{
	struct usbcam_urbinfo *ibp = (struct usbcam_urbinfo *) data;
	usb_unlink_urb(ibp->ib_urbs[ibp->ib_cururb]);
}

static void usbcam_urbstream_req_done(struct usbcam_urbinfo *ibp)
{
	struct usbcam_urbstream *usp;
	unsigned long flags;

	usp = ibp->ib_urbstream;
	spin_lock_irqsave(&usp->us_lock, flags);

	if (usp->us_streaming &&
	    (usp->us_active_count < usp->us_active_goal)) {
		/* Try to limp along with underflows */
		ibp->ib_cururb = 0;
		usbcam_urbstream_resubmit(usp, ibp);

	} else {
		list_del(&ibp->ib_links);
		list_add(&ibp->ib_links, &usp->us_unused_list);
	}

	spin_unlock_irqrestore(&usp->us_lock, flags);
}

static void usbcam_urbstream_freereq(struct usbcam_urbinfo *ibp)
{
	int i;
	assert(list_empty(&ibp->ib_links));
	for (i = 0; i < ibp->ib_nurbs; i++)
		usbcam_urb_free(ibp->ib_urbs[i], 1);
	kfree(ibp);
}

static struct usbcam_urbinfo *
usbcam_urbstream_allocreq(struct usbcam_urbstream *usp, int pipe, int ival,
			  int pktlen, int npkts, int alloc)
{
	struct usbcam_urbinfo *ibp;
	int sizeremain, nurbs, cpktlen;
	int i;

	usbcam_chklock(usp->us_dev);

	sizeremain = 0;
	nurbs = 1;
	if (usb_pipebulk(pipe)) {
		sizeremain = npkts;
		npkts = 0;
		nurbs = (sizeremain + pktlen - 1) / pktlen;
	}

	ibp = kzalloc(sizeof(*ibp) + (nurbs * sizeof(struct urb *)),
		      GFP_KERNEL);
	if (!ibp)
		return NULL;

	INIT_LIST_HEAD(&ibp->ib_links);
	ibp->ib_urbstream = usp;
	ibp->ib_nurbs = nurbs;

	for (i = 0; i < nurbs; i++) {
		cpktlen = pktlen;
		if (sizeremain && (cpktlen > sizeremain))
			cpktlen = sizeremain;
		ibp->ib_urbs[i] = usbcam_urb_alloc(usp->us_dev, pipe, cpktlen,
						   npkts, alloc);

		if (!ibp->ib_urbs[i])
			break;

		ibp->ib_urbs[i]->complete = usbcam_urbstream_urb_complete;
		ibp->ib_urbs[i]->interval = ival;
		ibp->ib_urbs[i]->context = ibp;


		if (sizeremain)
			sizeremain -= cpktlen;
	}

	if (i < nurbs) {
		while (i--)
			usbcam_urb_free(ibp->ib_urbs[i], alloc);
		kfree(ibp);
		return NULL;
	}

	setup_timer(&ibp->ib_timeout,
		    usbcam_urbstream_urb_timeout,
		    (unsigned long) ibp);

	return ibp;
}

static void usbcam_urbstream_freereqs(struct list_head *head)
{
	struct usbcam_urbinfo *ibp;
	while (!list_empty(head)) {
		ibp = list_entry(head->next, struct usbcam_urbinfo, ib_links);
		list_del_init(&ibp->ib_links);
		usbcam_urbstream_freereq(ibp);
	}
}

static void usbcam_urbstream_error(struct usbcam_workitem *work)
{
	struct usbcam_urbstream *usp = container_of(work,
						     struct usbcam_urbstream,
						     us_error_workitem);
	unsigned long flags;
	int sts;

	spin_lock_irqsave(&usp->us_lock, flags);
	sts = usp->us_resubmit_err;
	usp->us_resubmit_err = 0;
	spin_unlock_irqrestore(&usp->us_lock, flags);

	if (sts && usp->us_ops && usp->us_ops->submit_error)
		usp->us_ops->submit_error(usp->us_dev, usp, sts);
}

void usbcam_urbstream_init(struct usbcam_urbstream *usp,
			   struct usbcam_dev *udp, int ep)
{
	memset(usp, 0, sizeof(*usp));
	usp->us_dev = udp;
	usp->us_endpoint = ep;
	spin_lock_init(&usp->us_lock);
	INIT_LIST_HEAD(&usp->us_unused_list);
	INIT_LIST_HEAD(&usp->us_active_list);
	INIT_LIST_HEAD(&usp->us_complete_list);
	init_completion(&usp->us_active_empty);
	usbcam_work_init(usp->us_dev,
			 &usp->us_error_workitem,
			 usbcam_urbstream_error);
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_init);

void usbcam_urbstream_stop(struct usbcam_urbstream *usp, int wait)
{
	unsigned long flags;
	struct usbcam_urbinfo *ibp, *prev;
	int res;

	usbcam_chklock(usp->us_dev);

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] stopping",
		   usp->us_endpoint);
	spin_lock_irqsave(&usp->us_lock, flags);

	if (usp->us_streaming)
		usp->us_streaming = 0;

	if (!wait) {
		/*
		 * Cancel all in-flight requests without waiting
		 */
		while (!list_empty(&usp->us_active_list)) {
			ibp = list_entry(usp->us_active_list.prev,
					 struct usbcam_urbinfo, ib_links);
			list_del_init(&ibp->ib_links);
			assert(usp->us_active_count > 0);
			usp->us_active_count--;
			spin_unlock_irqrestore(&usp->us_lock, flags);
			usb_kill_urb(ibp->ib_urbs[ibp->ib_cururb]);
			spin_lock_irqsave(&usp->us_lock, flags);
			list_add(&ibp->ib_links, &usp->us_unused_list);
		}
	} else {
		/*
		 * Wait for all in-flight request groups to complete
		 * Do so while holding the device mutex, which is a bit ugly
		 */
		while (!list_empty(&usp->us_active_list)) {
			init_completion(&usp->us_active_empty);
			spin_unlock_irqrestore(&usp->us_lock, flags);
			wait_for_completion(&usp->us_active_empty);
			spin_lock_irqsave(&usp->us_lock, flags);
		}
	}


	/* Cancel all completed request groups with queued work items */
	list_for_each_entry_safe(ibp, prev,
				 &usp->us_complete_list,
				 ib_links) {

		res = usbcam_work_cancel(&ibp->ib_workitem);

		if (!ibp->ib_worker)
			assert(!res);
		else {
			assert(res);
			assert(*ibp->ib_worker == current);
			*ibp->ib_worker = NULL;
			ibp->ib_worker = NULL;
		}

		list_del(&ibp->ib_links);
		list_add_tail(&ibp->ib_links, &usp->us_unused_list);
	}

	/* Cancel the error work item */
	(void) usbcam_work_cancel(&usp->us_error_workitem);

	/* Clear the resubmission error code */
	usp->us_resubmit_err = 0;

	spin_unlock_irqrestore(&usp->us_lock, flags);
	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] stopped",
		   usp->us_endpoint);
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_stop);

int usbcam_urbstream_start(struct usbcam_urbstream *usp)
{
	struct usbcam_dev *udp;
	unsigned long flags;
	int submitted = 0, res;

	udp = usp->us_dev;

	usbcam_chklock(udp);

	spin_lock_irqsave(&usp->us_lock, flags);
	if (usp->us_streaming) {
		spin_unlock_irqrestore(&usp->us_lock, flags);
		usbcam_warn(udp, "%s: urbstream[%d] already streaming",
			    __FUNCTION__, usp->us_endpoint);
		return -EEXIST;
	}

	if (list_empty(&usp->us_unused_list)) {
		usbcam_warn(udp, "%s urbstream[%d] no unused URBs",
			    __FUNCTION__, usp->us_endpoint);
		return -ENOENT;
	}

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] starting",
		   usp->us_endpoint);
	usp->us_streaming = 1;

	spin_unlock_irqrestore(&usp->us_lock, flags);

	/* Submit initial URB(s) */
	while (1) {
		res = usbcam_urbstream_submit_unused(usp);
		assert(res != -ENOENT);
		if (res) {
			usbcam_urbstream_stop(usp, 0);
			return res;
		}

		if (++submitted == usp->us_active_goal)
			break;
	}

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] started",
		   usp->us_endpoint);
	return 0;

}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_start);

int usbcam_urbstream_submit_one(struct usbcam_urbstream *usp)
{
	usbcam_chklock(usp->us_dev);
	assert(!usp->us_streaming);
	return usbcam_urbstream_submit_unused(usp);
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_submit_one);

void usbcam_urbstream_cleanup(struct usbcam_urbstream *usp)
{
	usbcam_urbstream_stop(usp, 0);
	usbcam_urbstream_freereqs(&usp->us_unused_list);
	assert(list_empty(&usp->us_active_list));
	assert(list_empty(&usp->us_complete_list));
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_cleanup);


static void usbcam_urbstream_iso_process(struct usbcam_workitem *work)
{
	struct usbcam_urbinfo *ibp = container_of(work, struct usbcam_urbinfo,
						 ib_workitem);
	struct usbcam_urbstream *usp = ibp->ib_urbstream;
	struct task_struct *me = current;
	struct urb *urbp;
	int i;

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] processing %p",
		   usp->us_endpoint, ibp);

	assert(!ibp->ib_worker);
	ibp->ib_worker = &me;

	urbp = ibp->ib_urbs[0];
	for (i = 0; i < urbp->number_of_packets; i++) {
		char *buf = (((char *) urbp->transfer_buffer) +
			     urbp->iso_frame_desc[i].offset);
		int len = urbp->iso_frame_desc[i].actual_length;
		int status = urbp->iso_frame_desc[i].status;

		urbp->iso_frame_desc[i].actual_length = 0;
		urbp->iso_frame_desc[i].status = 0;

		usp->us_ops->packet_done(usp->us_dev,
					 usp, buf, len, status);
		if (!me)
			return;
	}

	assert(ibp->ib_worker == &me);
	ibp->ib_worker = NULL;

	usbcam_urbstream_req_done(ibp);
}

static int usbcam_urbstream_allocreqs_isorcv(struct usbcam_urbstream *usp,
					     struct list_head *head, int count,
					     int ival, int pktcount,
					     int pktlen)
{
	struct usbcam_urbinfo *ibp;
	struct list_head new_bufs;
	int pipe;

	pipe = usb_rcvisocpipe(usp->us_dev->ud_dev, usp->us_endpoint);

	INIT_LIST_HEAD(&new_bufs);

	while (count--) {
		ibp = usbcam_urbstream_allocreq(usp, pipe, ival,
						pktlen, pktcount, 1);
		if (!ibp) {
			usbcam_urbstream_freereqs(&new_bufs);
			return -ENOMEM;
		}

		usbcam_work_init(usp->us_dev,
				 &ibp->ib_workitem,
				 usbcam_urbstream_iso_process);
		list_add_tail(&ibp->ib_links, &new_bufs);
	}

	list_splice(&new_bufs, head);
	return 0;
}

int usbcam_urbstream_config_iso(struct usbcam_urbstream *usp,
				const struct usbcam_urbstream_ops *ops,
				int pktcount, int nreqs, int interval,
				int pktlen)
{
	int res;

	usbcam_urbstream_cleanup(usp);

	usp->us_active_goal = 2;
	usp->us_ops = ops;

	if (!interval) {
		/* FIXME: find the appropriate interval for the endpoint */
		return -EINVAL;
	}

	if (!pktlen) {
		/* Choose a packet length based on the current altsetting */
		pktlen = usb_maxpacket(usp->us_dev->ud_dev,
				       usb_rcvisocpipe(usp->us_dev->ud_dev,
						       usp->us_endpoint), 0);
		if (!pktlen) {
			usbcam_dbg(usp->us_dev, URBSTREAM,
				   "urbstream[%d]: current altsetting has "
				   "maxpacket=0", usp->us_endpoint);
			return -EINVAL;
		}
		if (usp->us_dev->ud_dev->speed == USB_SPEED_HIGH)
			pktlen = (pktlen & 0x7ff) *
				(((pktlen >> 11) & 0x3) + 1);
		else
			pktlen &= 0x7ff;

		usbcam_dbg(usp->us_dev, URBSTREAM,
			   "urbstream[%d] using pktlen %d",
			   usp->us_endpoint, pktlen);
	}

	if (!pktcount)
		pktcount = USBCAM_DFL_ISO_URB_PKTS;
	if (!nreqs)
		nreqs = USBCAM_DFL_ISO_REQS;
	if (nreqs < usp->us_active_goal) {
		usbcam_warn(usp->us_dev, "%s urbstream[%d]: at least %d reqs "
			    "are required", __FUNCTION__, usp->us_endpoint,
			    usp->us_active_goal);
		nreqs = usp->us_active_goal;
	}

	usp->us_timeout_ticks = 0;

	res = usbcam_urbstream_allocreqs_isorcv(usp, &usp->us_unused_list,
						nreqs, interval, pktcount,
						pktlen);
	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_config_iso);


static void usbcam_urbstream_bulk_process(struct usbcam_workitem *work)
{
	struct usbcam_urbinfo *ibp = container_of(work, struct usbcam_urbinfo,
						 ib_workitem);
	struct usbcam_urbstream *usp = ibp->ib_urbstream;
	struct task_struct *me = current;
	struct urb *urbp;
	int len, status;
	int i;

	usbcam_dbg(usp->us_dev, URBSTREAM, "urbstream[%d] processing %p",
		   usp->us_endpoint, ibp);

	assert(!ibp->ib_worker);
	ibp->ib_worker = &me;

	for (i = 0; i < ibp->ib_cururb; i++) {
		urbp = ibp->ib_urbs[i];
		len = urbp->actual_length;
		status = urbp->status;

		urbp->actual_length = 0;
		urbp->status = 0;

		usp->us_ops->packet_done(usp->us_dev, usp,
					 urbp->transfer_buffer, len, status);
		if (!me)
			return;
	}

	assert(ibp->ib_worker == &me);
	ibp->ib_worker = NULL;

	usbcam_urbstream_req_done(ibp);
}

static int usbcam_urbstream_allocreqs_bulkrcv(struct usbcam_urbstream *usp,
					      struct list_head *head,
					      int count, int reqlen,
					      int maxpkt)
{
	struct usbcam_urbinfo *ibp;
	struct list_head new_bufs;
	int pipe;

	pipe = usb_rcvbulkpipe(usp->us_dev->ud_dev, usp->us_endpoint);

	INIT_LIST_HEAD(&new_bufs);

	while (count--) {
		ibp = usbcam_urbstream_allocreq(usp, pipe, 0,
						maxpkt, reqlen, 1);
		if (!ibp) {
			usbcam_urbstream_freereqs(&new_bufs);
			return -ENOMEM;
		}

		usbcam_work_init(usp->us_dev,
				 &ibp->ib_workitem,
				 usbcam_urbstream_bulk_process);
		list_add_tail(&ibp->ib_links, &new_bufs);
	}

	list_splice(&new_bufs, head);
	return 0;
}

int usbcam_urbstream_config_bulk(struct usbcam_urbstream *usp,
				 const struct usbcam_urbstream_ops *ops,
				 int nreqs, int reqlen, int maxpkt,
				 int timeout_ms)
{
	int res;

	usbcam_urbstream_cleanup(usp);

	usp->us_active_goal = 1;
	usp->us_ops = ops;

	if (!maxpkt)
		maxpkt = 64 * 1024;
	if (reqlen < 0) {
		usbcam_dbg(usp->us_dev, URBSTREAM,
			   "urbstream[%d]: packet length must be >=0",
			   usp->us_endpoint);
		return -EINVAL;
	}

	if (!nreqs)
		nreqs = USBCAM_DFL_BULK_REQS;
	if (nreqs < usp->us_active_goal) {
		usbcam_warn(usp->us_dev, "%s urbstream[%d]: at least %d URBs "
			    "are required", __FUNCTION__, usp->us_endpoint,
			usp->us_active_goal);
		nreqs = usp->us_active_goal;
	}

	usp->us_timeout_ticks = (timeout_ms * HZ) / 1000;

	res = usbcam_urbstream_allocreqs_bulkrcv(usp, &usp->us_unused_list,
						 nreqs, reqlen, maxpkt);
	return res;
}
USBCAM_EXPORT_SYMBOL(usbcam_urbstream_config_bulk);


#ifdef usbcam_hexdump
#undef usbcam_hexdump
#endif
#define dumpable_char(X) (((X) >= ' ') && ((X) <= '~'))
void usbcam_hexdump(struct usbcam_dev *udp, const u8 *buf, size_t len)
{
	const int bpl = 16;
	const int cend_max = ((bpl * 4) + 1);
	static const char n2x[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
				      '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
	char x, outbuf[cend_max + 1];
	unsigned int cstart, cend, xpos, cpos, offset;

	offset = 0;
	cstart = (3 * bpl) + 1;
	cend = cstart + bpl;
	outbuf[cend] = '\0';
	goto beginning;

	while (len) {
		x = *buf++;
		outbuf[xpos++] = n2x[(x >> 4) & 0xf];
		outbuf[xpos++] = n2x[x & 0xf];
		outbuf[cpos+cstart] = dumpable_char(x) ? x : '.';
		cpos++;
		xpos++;
		len--;

		if (!len || (cpos == bpl)) {
			printk(KERN_DEBUG "%s: %08x %s\n",
			       udp->ud_dev_name, offset, outbuf);
			offset += bpl;

		beginning:
			memset(outbuf, ' ', cend);
			cpos = 0;
			xpos = 0;
		}
	}
}
USBCAM_EXPORT_SYMBOL(usbcam_hexdump);
