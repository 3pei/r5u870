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
 * This file contains videobuf frame buffer interface routines, and
 * current frame access functions.
 */

/*
 * APPBUG: Some applications expect VIDIOCGMBUF to provide a buffer
 * large enough to accommodate whatever image format they choose in the
 * future.  We enable fixed size buffer mode from VIDIOCGMBUF, and
 * disable it from VIDIOC_REQBUFS.
 */
static int fixed_fbsize = 1024 * 1024;
module_param(fixed_fbsize, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(fixed_fbsize, "Size in bytes of fixed-length framebuffers");

/*
 * naresh <cyan_00391@yahoo.co.in>
 * Define the various codes to permit compilation of this on 
 * 2.6.25 and higher kernels, we use gcc preprocessors for backward
 * compatibility
 * CREDITS: Thanks to stefano.brivio for his patch.
 *          Refer bug @ http://bugs.mediati.org/r5u870/issue2
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
 #define STATE_ACTIVE      VIDEOBUF_ACTIVE
 #define STATE_DONE        VIDEOBUF_DONE
 #define STATE_ERROR       VIDEOBUF_ERROR
 #define STATE_NEEDS_INIT  VIDEOBUF_NEEDS_INIT
 #define STATE_PREPARED    VIDEOBUF_PREPARED
#endif

/*
 * Frame capture handling helpers follow
 */

static inline struct usbcam_frame *
usbcam_capture_curframe(struct usbcam_dev *udp)
{
	return list_empty(&udp->ud_frame_cap_queue)
		? NULL
		: list_entry(udp->ud_frame_cap_queue.next,
			     struct usbcam_frame, cap_links);
}

static void usbcam_capture_abortall(struct usbcam_dev *udp)
{
	struct usbcam_frame *framep;

	/* Abort all frames on the capture queue */
	while (1) {
		framep = usbcam_capture_curframe(udp);
		if (!framep)
			break;
		usbcam_dbg(udp, VIDEOBUF, "completing frame %d STATE_ERROR",
			   framep->vbb.i);
		list_del_init(&framep->cap_links);
		framep->vbb.state = STATE_ERROR;
		wake_up_all(&framep->vbb.done);
	}
}

static inline void usbcam_capture_complete_frame(struct usbcam_dev *udp,
						 struct usbcam_frame *framep,
						 int is_error)
{
	usbcam_chklock(udp);
	usbcam_dbg(udp, VIDEOBUF, "completing frame %d/%p %s", framep->vbb.i,
		   framep, is_error ? "STATE_ERROR" : "STATE_DONE");
	list_del_init(&framep->cap_links);
	framep->vbb.state = is_error ? STATE_ERROR : STATE_DONE;
	wake_up_all(&framep->vbb.done);
}

static int usbcam_capture_start(struct usbcam_dev *udp)
{
	int res;

	if (udp->ud_capturing) {
		usbcam_warn(udp, "%s: already capturing", __FUNCTION__);
		return 0;
	}

	if (list_empty(&udp->ud_frame_cap_queue)) {
		usbcam_warn(udp, "%s: no frames queued to capture",
			    __FUNCTION__);
		return -ENOENT;
	}

	if (udp->ud_disconnected) {
		/*
		 * We can't let any frames through if the device has
		 * been disconnected
		 */
		usbcam_capture_abortall(udp);
		return -ENODEV;
	}

	usbcam_dbg(udp, CAPTURE, "invoking minidriver cap_start");

	res = usbcam_minidrv_op(udp, cap_start);
	if (res) {
		usbcam_dbg(udp, CAPTURE,
			   "%s: could not start capture for %s: %d",
			   __FUNCTION__, usbcam_drvname(udp->ud_minidrv), res);

		if (udp->ud_capturing) {
			usbcam_warn(udp,
				    "%s: minidriver left ud_capturing set\n",
				    __FUNCTION__);
		}

		usbcam_capture_abortall(udp);
		return res;
	}

	if (!udp->ud_capturing && usbcam_capture_curframe(udp)) {
		usbcam_warn(udp, "%s: minidriver failed to set ud_capturing!",
			    __FUNCTION__);
	} else {
		usbcam_dbg(udp, CAPTURE, "minidriver capture started");
	}

	return 0;
}

void usbcam_capture_stop(struct usbcam_dev *udp)
{
	if (udp->ud_capturing) {
		usbcam_dbg(udp, CAPTURE, "invoking minidriver cap_stop");
		usbcam_minidrv_op(udp, cap_stop);

		if (udp->ud_capturing) {
			usbcam_warn(udp, "%s: minidriver failed to clear "
				    "ud_capturing!", __FUNCTION__);
		} else {
			usbcam_dbg(udp, CAPTURE, "minidriver capture stopped");
		}
	}
}

static inline struct videobuf_dmabuf* usbframe_get_dmabuf(struct videobuf_buffer *buf)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
    return &buf->dma;
#else
    return videobuf_to_dma(buf);
#endif
}

/*
 * External APIs for minidriver access to the frame queue
 */

int usbcam_curframe_get(struct usbcam_dev *udp, struct usbcam_curframe *cf)
{
	struct usbcam_frame *framep = usbcam_capture_curframe(udp);
	struct videobuf_dmabuf *dma;

	usbcam_chklock(udp);

	if (!framep)
		return -ENOENT;
		
	dma = usbframe_get_dmabuf(&framep->vbb);

	cf->uf_base = (u8 *) (framep->vmap_sof
			      ? framep->vmap_sof
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36) /*3pei*/
			      : dma->vmalloc);
#else
                  : dma->vaddr);
#endif
	cf->uf_size = framep->vbb.size;
	cf->uf_field = framep->vbb.field;
	memset(&cf->uf_timestamp, 0, sizeof(cf->uf_timestamp));

	return 0;
}
USBCAM_EXPORT_SYMBOL(usbcam_curframe_get);

void usbcam_curframe_complete_detail(struct usbcam_dev *udp,
				     struct usbcam_curframe *cf)
{
	struct usbcam_frame *framep;

	usbcam_chklock(udp);

	framep = usbcam_capture_curframe(udp);
	if (!framep) {
		usbcam_warn(udp, "%s: no current frame!", __FUNCTION__);
		return;
	}

	if (framep->vbb.state != STATE_ACTIVE) {
		usbcam_err(udp, "%s: current frame is in unexpected state %d",
			   __FUNCTION__, framep->vbb.state);
	}

	if (cf) {
		framep->vbb.size = cf->uf_size;
		framep->vbb.field = cf->uf_field;
		framep->vbb.ts = cf->uf_timestamp;

		if (framep->vbb.bsize < cf->uf_size) {
			usbcam_warn(udp, "%s: minidriver supplied "
				    "excessive size %zu",
				    __FUNCTION__, cf->uf_size);
			framep->vbb.size = framep->vbb.bsize;
		}
	}

	usbcam_capture_complete_frame(udp, framep, 0);
}
USBCAM_EXPORT_SYMBOL(usbcam_curframe_complete_detail);

void usbcam_curframe_abortall(struct usbcam_dev *udp)
{
	usbcam_dbg(udp, CAPTURE, "minidriver aborting all frames");

	usbcam_chklock(udp);

	if (udp->ud_capturing) {
		usbcam_warn(udp, "%s: minidriver left ud_capturing set",
			    __FUNCTION__);
	}
	usbcam_capture_abortall(udp);
}
USBCAM_EXPORT_SYMBOL(usbcam_curframe_abortall);


/*
 * Test pattern code
 */

void usbcam_curframe_fill(struct usbcam_dev *udp, size_t offset,
			  const void *pattern, int patlen, int nrecs)
{
	struct usbcam_curframe cf;
	size_t end;

	usbcam_chklock(udp);

	if (usbcam_curframe_get(udp, &cf)) {
		usbcam_warn(udp, "%s: no current frame", __FUNCTION__);
		return;
	}

	end = offset + (patlen * nrecs);
	if (end > cf.uf_size) {
		usbcam_warn(udp, "%s(offs=%zu patlen=%d nrecs=%d) would "
			    "write past end of %zu byte framebuffer",
			    __FUNCTION__,
			    offset, patlen, nrecs, cf.uf_size);
		if (offset > cf.uf_size)
			nrecs = 0;
		else
			nrecs = (cf.uf_size - offset) / patlen;
	}
	else if (end > udp->ud_format.sizeimage)
		usbcam_warn(udp, "%s(offs=%zu patlen=%d nrecs=%d) writing "
			    "beyond %u-byte sizeimage", __FUNCTION__,
			    offset, patlen, nrecs, udp->ud_format.sizeimage);

	if (!nrecs)
		return;

	if (patlen == 1) {
		memset(cf.uf_base + offset, *(char *)pattern, nrecs);
		return;
	}

	while (nrecs--) {
		memcpy(cf.uf_base + offset, pattern, patlen);
		offset += patlen;
	}
}
USBCAM_EXPORT_SYMBOL(usbcam_curframe_fill);

void usbcam_curframe_fill_lines(struct usbcam_dev *udp,
				const char *pat, int patlen,
				int pixperpat)
{
	int line, nrecperline, stride;
	size_t offset = 0;

	nrecperline = (udp->ud_format.width + pixperpat - 1) / pixperpat;
	stride = udp->ud_format.bytesperline
		? udp->ud_format.bytesperline
		: (nrecperline * patlen);

	if ((patlen * nrecperline) == stride) {
		usbcam_curframe_fill(udp, 0, pat, patlen,
				     nrecperline * udp->ud_format.height);
		return;
	}

	for (line = 0; line < udp->ud_format.height; line++) {
		usbcam_curframe_fill(udp, offset, pat, patlen, nrecperline);
		offset += stride;
	}
}

void usbcam_curframe_fill_interleaved(struct usbcam_dev *udp,
				      const char *pat_even,
				      const char *pat_odd,
				      int patlen, int pixperpat)
{
	int line, nrecperline, stride;
	size_t offset = 0;

	nrecperline = (udp->ud_format.width + pixperpat - 1) / pixperpat;
	stride = udp->ud_format.bytesperline
		? udp->ud_format.bytesperline
		: (nrecperline * patlen);

	for (line = 0; line < udp->ud_format.height; line++) {
		usbcam_curframe_fill(udp, offset,
				     (line & 1) ? pat_odd : pat_even,
				     patlen, nrecperline);
		offset += stride;
	}
}

void usbcam_curframe_fill_planar(struct usbcam_dev *udp,
				 const char *pat0, int pat0len, int pixperpat0,
				 const char *pat1, int pat1len, int pixperpat1,
				 const char *pat2, int pat2len, int pixperpat2)
{
	int nrecperline;
	size_t offset = 0;

	if (pat0 && pat0len) {
		nrecperline = ((udp->ud_format.width + pixperpat0 - 1) /
			       pixperpat0);
		usbcam_curframe_fill(udp, offset, pat0, pat0len,
				     nrecperline * udp->ud_format.height);
		offset += (nrecperline * udp->ud_format.height * pat0len);
	}
	if (pat1 && pat1len) {
		nrecperline = ((udp->ud_format.width + pixperpat1 - 1) /
			       pixperpat1);
		usbcam_curframe_fill(udp, offset, pat1, pat1len,
				     nrecperline * udp->ud_format.height);
		offset += (nrecperline * udp->ud_format.height * pat1len);
	}
	if (pat2 && pat2len) {
		nrecperline = ((udp->ud_format.width + pixperpat2 - 1) /
			       pixperpat2);
		usbcam_curframe_fill(udp, offset, pat2, pat2len,
				     nrecperline * udp->ud_format.height);
		offset += (nrecperline * udp->ud_format.height * pat2len);
	}
}

/*
 * The goal is to be able to come up with a solid blue image in all
 * basic uncompressed formats.  No JPEG or compressed formats, at least
 * not yet.
 */
int usbcam_curframe_testpattern(struct usbcam_dev *udp)
{
	usbcam_chklock(udp);

#define DO_FILL(PIXPERPAT, TP...) {					\
	const char tp[] = {TP};						\
	usbcam_curframe_fill_lines(udp, tp, sizeof(tp),	PIXPERPAT);	\
}
	switch (udp->ud_format.pixelformat) {
	case V4L2_PIX_FMT_RGB332:
		DO_FILL(1, 0x03)
		return 0;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
		DO_FILL(1, 0x1f, 0x00)
		return 0;
	case V4L2_PIX_FMT_RGB555X:
	case V4L2_PIX_FMT_RGB565X:
		DO_FILL(1, 0x00, 0x1f)
		return 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	case V4L2_PIX_FMT_RGB444:
		DO_FILL(1, 0x00, 0x0f)
		return 0;
#endif
	case V4L2_PIX_FMT_BGR24:
		DO_FILL(1, 0xff, 0x00, 0x00);
		return 0;
	case V4L2_PIX_FMT_RGB24:
		DO_FILL(1, 0x00, 0x00, 0xff);
		return 0;
	case V4L2_PIX_FMT_BGR32:
		DO_FILL(1, 0xff, 0x00, 0x00, 0x00);
		return 0;
	case V4L2_PIX_FMT_RGB32:
		DO_FILL(1, 0x00, 0x00, 0xff, 0x00);
		return 0;
	case V4L2_PIX_FMT_GREY:
		DO_FILL(1, 0x1f);
		return 0;
	case V4L2_PIX_FMT_YUYV:
		DO_FILL(2, 0x29, 0xf0, 0x29, 0x6e);
		return 0;
	case V4L2_PIX_FMT_UYVY:
		DO_FILL(2, 0xf0, 0x29, 0x6e, 0x29);
		return 0;
	case V4L2_PIX_FMT_YYUV:
		DO_FILL(2, 0x29, 0x29, 0xf0, 0x6e);
		return 0;
	case V4L2_PIX_FMT_Y41P:
		DO_FILL(8,
			0xf0, 0x29, 0x6e, 0x29, 0xf0, 0x29, 0x6e, 0x29,
			0x29, 0x29, 0x29, 0x29);
		return 0;
#undef DO_FILL

	case V4L2_PIX_FMT_SBGGR8: {
		const char tp0[] = { 0xff, 0x00 }, tp1[] = { 0x00, 0x00 };
		usbcam_curframe_fill_interleaved(udp, tp0, tp1, 2, 2);
		return 0;
	}
	case V4L2_PIX_FMT_YVU410: {
		const char tp0[] = {0x29}, tp1[] = {0x6e}, tp2[] = {0xf0};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 16,
					    tp2, sizeof(tp2), 16);
		return 0;
	}
	case V4L2_PIX_FMT_YUV410: {
		const char tp0[] = {0x29}, tp1[] = {0xf0}, tp2[] = {0x6e};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 16,
					    tp2, sizeof(tp2), 16);
		return 0;
	}
	case V4L2_PIX_FMT_YVU420: {
		const char tp0[] = {0x29}, tp1[] = {0x6e}, tp2[] = {0xf0};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 4,
					    tp2, sizeof(tp2), 4);
		return 0;
	}
	case V4L2_PIX_FMT_YUV411P: 
	case V4L2_PIX_FMT_YUV420: {
		const char tp0[] = {0x29}, tp1[] = {0xf0}, tp2[] = {0x6e};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 4,
					    tp2, sizeof(tp2), 4);
		return 0;
	}
	case V4L2_PIX_FMT_YUV422P: {
		const char tp0[] = {0x29}, tp1[] = {0xf0}, tp2[] = {0x6e};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 2,
					    tp2, sizeof(tp2), 2);
		return 0;
	}
	case V4L2_PIX_FMT_NV12: {
		const char tp0[] = {0x29}, tp1[] = {0xf0, 0x6e};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 4,
					    NULL, 0, 0);
		return 0;
	}
	case V4L2_PIX_FMT_NV21: {
		const char tp0[] = {0x29}, tp1[] = {0x6e, 0xf0};
		usbcam_curframe_fill_planar(udp, tp0, sizeof(tp0), 1,
					    tp1, sizeof(tp1), 4,
					    NULL, 0, 0);
		return 0;
	}
	}

	return -EINVAL;
}
USBCAM_EXPORT_SYMBOL(usbcam_curframe_testpattern);


/*
 * video-buf interfaces for managing frame buffers
 * The device mutex is not held on entry to _any_ of these functions.
 */

static int usbcam_videobuf_setup(struct videobuf_queue *vq,
				 unsigned int *count, unsigned int *size)
{
	struct usbcam_fh *ufp = container_of(vq, struct usbcam_fh, ufh_vbq);
	struct usbcam_dev *udp = ufp->ufh_dev;

	usbcam_lock(udp);

	/* APPBUG: possibly request larger buffers than necessary */
	if ((ufp->ufh_flags & USBCAM_FH_USE_FIXED_FB) &&
	    (fixed_fbsize > udp->ud_format.sizeimage))
		*size = fixed_fbsize;
	else
		*size = udp->ud_format.sizeimage;

	if (!*count)
		*count = 2;

	usbcam_dbg(udp, VIDEOBUF, "videobuf setup: size=%u count=%u",
		   *size, *count);

	usbcam_unlock(udp);
	return 0;
}

static void usbcam_videobuf_free(struct videobuf_queue *vq,
				 struct usbcam_frame *framep)
{
	struct videobuf_dmabuf *dma = usbframe_get_dmabuf(&framep->vbb);
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)  /*3pei*/
    videobuf_waiton(&framep->vbb, 0, 0);
	videobuf_dma_unmap(vq, dma); /* should it be vq->dev? typo or interface changed?*/
	#else
	videobuf_waiton(vq, &framep->vbb, 0, 0);
	videobuf_dma_unmap(vq->dev, dma); 
	#endif
	videobuf_dma_free(dma);
	if (framep->vbb.state != STATE_NEEDS_INIT) {
		if (framep->vmap_base) {
			vunmap(framep->vmap_base);
			framep->vmap_base = NULL;
			framep->vmap_sof = NULL;
		}
		assert(list_empty(&framep->cap_links));
		framep->vbb.state = STATE_NEEDS_INIT;
	}
}

static int usbcam_videobuf_prepare(struct videobuf_queue *vq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct usbcam_fh *ufp = container_of(vq, struct usbcam_fh, ufh_vbq);
	struct usbcam_dev *udp = ufp->ufh_dev;
	struct usbcam_frame *framep =
		container_of(vb, struct usbcam_frame, vbb);
	struct videobuf_dmabuf *dma = usbframe_get_dmabuf(&framep->vbb);
	int res;

	framep->vbb.size = udp->ud_format.sizeimage;
	if (framep->vbb.baddr && (framep->vbb.bsize < framep->vbb.size)) {
		usbcam_warn(udp, "process %s requested capture of a frame "
			    "larger than its", current->comm);
		usbcam_warn(udp, "allocated frame buffer, fix it!");
		return -EINVAL;
	}

	if (framep->vbb.state == STATE_NEEDS_INIT) {
		/*
		 * This is the place where we initialize the rest of
		 * the usbcam_frame structure.
		 */
		INIT_LIST_HEAD(&framep->cap_links);
		framep->vmap_base = NULL;
		framep->vmap_sof = NULL;

		usbcam_dbg(udp, VIDEOBUF,
			   "preparing frame %d/%p", framep->vbb.i, framep);

		/* We also lock down the memory that was allocated for it */
		res = videobuf_iolock(vq, &framep->vbb, NULL);
		if (res)
			goto fail;

		/* If there's no kernel mapping, we must create one */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36) /*3pei*/
		if (!dma->vmalloc) {
#else
		if (!dma->vaddr) {
#endif
			framep->vmap_base = vmap(dma->pages,
						 dma->nr_pages,
						 VM_MAP,
						 PAGE_KERNEL);
			if (!framep->vmap_base) {
				res = -ENOMEM;
				goto fail;
			}

			framep->vmap_sof =
				((char *)framep->vmap_base) +
				dma->offset;
		}
	}

	framep->vbb.field = field;
	framep->vbb.state = STATE_PREPARED;
	return 0;

fail:
	usbcam_videobuf_free(vq, framep);
	return res;
}

static void usbcam_videobuf_queue(struct videobuf_queue *vq,
				  struct videobuf_buffer *vb)
{
	struct usbcam_fh *ufp = container_of(vq, struct usbcam_fh, ufh_vbq);
	struct usbcam_dev *udp = ufp->ufh_dev;
	struct usbcam_frame *framep =
		container_of(vb, struct usbcam_frame, vbb);
	int was_empty = 0;

	assert(framep->vbb.state != STATE_NEEDS_INIT);

	usbcam_lock(udp);

	if (list_empty(&udp->ud_frame_cap_queue))
		was_empty = 1;

	usbcam_dbg(udp, VIDEOBUF, "queueing frame %d/%p",
		   framep->vbb.i, framep);

	/*
	 * We always set buffers to STATE_ACTIVE to prevent them from
	 * being manipulated / dequeued by the videobuf code.
	 */
	list_add_tail(&framep->cap_links, &udp->ud_frame_cap_queue);
	framep->vbb.state = STATE_ACTIVE;

	if (was_empty && !udp->ud_capturing)
		(void) usbcam_capture_start(udp);

	usbcam_unlock(udp);
}

static void usbcam_videobuf_release(struct videobuf_queue *vq,
				    struct videobuf_buffer *vb)
{
	struct usbcam_fh *ufp = container_of(vq, struct usbcam_fh, ufh_vbq);
	struct usbcam_dev *udp = ufp->ufh_dev;
	struct usbcam_frame *framep =
		container_of(vb, struct usbcam_frame, vbb);
	int stopped_capture = 0;

	usbcam_lock(udp);

	if ((framep->vbb.state != STATE_NEEDS_INIT) &&
	    !list_empty(&framep->cap_links)) {

		usbcam_dbg(udp, VIDEOBUF,
			   "aborting frame %d/%p", framep->vbb.i, framep);

		/*
		 * An active frame is being shot down here, most
		 * likely by videobuf_queue_cancel.
		 */
		assert(framep->vbb.state == STATE_ACTIVE);

		if (udp->ud_capturing &&
		    !list_empty(&udp->ud_frame_cap_queue) &&
		    (framep == usbcam_capture_curframe(udp))) {
			/*
			 * The current frame has been user-aborted.
			 * We will stop capturing.  The minidriver may complete
			 * it with an error, or may leave it alone, in which
			 * case we will complete it with an error.
			 */
			usbcam_dbg(udp, VIDEOBUF,
				   "current frame aborted, stopping capture");

			usbcam_capture_stop(udp);
			stopped_capture = 1;
		}

		if (!list_empty(&framep->cap_links))
			usbcam_capture_complete_frame(udp, framep, 1);

		/*
		 * Ideally, if we stopped capturing, and there are frames
		 * still in the queue, we would restart.
		 *
		 * In reality, we only take this code path if all frames
		 * from the owning file handle are aborted, and restarting
		 * would pointlessly slow down this process.
		 */
	}

	usbcam_unlock(udp);
	usbcam_videobuf_free(vq, framep);
}

struct videobuf_queue_ops usbcam_videobuf_qops = {
	.buf_setup	= usbcam_videobuf_setup,
	.buf_prepare	= usbcam_videobuf_prepare,
	.buf_queue	= usbcam_videobuf_queue,
	.buf_release	= usbcam_videobuf_release,
};
