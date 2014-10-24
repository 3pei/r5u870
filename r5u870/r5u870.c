/*
 * Driver for Ricoh R5U870-based Custom OEM Webcams
 * Copyright (c) 2007 Sam Revitch <samr7@cs.washington.edu>
 * Copyright (c) 2008 Alexander Hixon <hixon.alexander@mediati.org>
 *
 * Check out README for additional credits.
 * Version 0.11.3
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
 * This driver supports certain custom OEM webcams based on the Ricoh
 * R5U870 / Micro Vision M25U870 controller chip.  These tend to be
 * built in to certain models of laptop computers and claim USB vendor
 * ID 0x05CA (Ricoh).
 *
 * All Ricoh webcams support common microcode upload, reset, and query
 * version commands.
 *
 * Each distinct piece of hardware requires a different microcode blob,
 * which is uploaded to the device the first time the device is probed
 * after power-on.  The microcode blob is tailored to the attached
 * image sensor and the desired type of USB interface.  Some microcode
 * blobs are created to work with the Ricoh proprietary USB interface.
 * Others behave as UVC devices, but use the common Ricoh vendor
 * commands to set certain controls.  Some devices, in particular the
 * 1810 and 1870 HP Pavilion Webcam devices, appear to be identical in
 * hardware, and differ only by USB descriptor tables, and the
 * microcode blobs implement the different USB personalities.
 *
 * This driver supports basic UVC descriptor table parsing to determine
 * available controls and resolutions, and enough UVC commands to
 * support the UVC interface style of the Ricoh webcams.
 */

#define USBCAM_DEBUG_DEFAULT 0

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/firmware.h>
#include <linux/dmi.h>
#include <linux/ctype.h>

#include "usbcam/usbcam.h"


#define USBCAM_DBG_R5U_INIT	(USBCAM_DBGBIT_MD_START + 0)
#define USBCAM_DBG_R5U_FRAME	(USBCAM_DBGBIT_MD_START + 1)
#define USBCAM_DBG_R5U_MDINTF	(USBCAM_DBGBIT_MD_START + 2)
#define USBCAM_DBG_R5U_CTRL	(USBCAM_DBGBIT_MD_START + 3)


#define r5u_info(RV, FMT...) usbcam_info((RV)->vh_parent, FMT)
#define r5u_err(RV, FMT...) usbcam_err((RV)->vh_parent, FMT)
#define r5u_dbg(RV, SUBSYS, FMT...) usbcam_dbg((RV)->vh_parent, SUBSYS, FMT)

#define R5U870_VERSION KERNEL_VERSION(0,11,3)
#define R5U870_VERSION_EXTRA ""


struct r5u870_resolution {
	int rw_width;
	int rw_height;
	int rw_reqbw;
	int rw_interval;
	int rw_frameidx;
};

struct r5u870_pix_fmt {
	struct usbcam_pix_fmt base;
	int rp_formatidx;
	const struct r5u870_resolution *rp_restbl;
	int rp_restbl_alloc;
};

struct r5u870_ctrl {
	struct usbcam_ctrl base;
	int reg;
	u8 unit;
	u8 info;
	u8 size;
	u8 is_auto;
	int auto_offset;
	int val_offset;
	int value;
};

struct r5u870_model {
	char				*rm_name;
	const char			*rm_ucode_file;
	const struct r5u870_resolution	*rm_res;
	const int			*rm_wdm_ctrlids;
	const struct r5u870_pix_fmt	*rm_pixfmts;
	int				rm_npixfmts;
	u16				rm_ucode_version;
	unsigned int			rm_uvc : 1,
					rm_no_ctrl_reload : 1,
					rm_no_first_auto_suppress : 1;
};

struct r5u870_ctx {
	struct usbcam_dev		*vh_parent;
	struct usbcam_urbstream		vh_iso;

	const struct r5u870_model	*vh_model;
	const struct r5u870_pix_fmt	*vh_pixfmts;
	int				vh_npixfmts;

	int				vh_dyn_pixfmts : 1,
					vh_configured : 1,
					vh_ctrl_reg_enable : 1,
					vh_ctrl_sync : 1,
					vh_ctrl_auto_suppress : 1;

	const struct r5u870_pix_fmt	*vh_fmt;
	const struct r5u870_resolution	*vh_res;

	int				vh_timeout;
	int				vh_firstframe;
	int				vh_emptypkts;
	int				vh_frame_accum;

	int				vh_framebuf_offset;

	int				vh_ctrl_ifnum;
	int				vh_iso_ep;
	int				vh_iso_ifnum;
	int				vh_iso_minpacket;
	int				vh_act_altsetting;

	int (*vh_set_fmt)(struct r5u870_ctx *,
			  const struct r5u870_pix_fmt *fmtp,
			  const struct r5u870_resolution *resp);
	int (*vh_cap_stop)(struct r5u870_ctx *);
	int (*vh_decide_pkt)(struct r5u870_ctx *, int st, int len,
				 const u8 *pktdata, int *start);

	/* Auto settings */
	int vh_auto_wb;
	int vh_aec;
	int vh_agc;
};

#define udp_r5u870(UDP) ((struct r5u870_ctx *)((UDP)->ud_minidrv_data))
#define r5u870_dev(RV) ((RV)->vh_parent->ud_dev)

/*
 * USB control message with kmalloc'd buffer helper
 */

static int r5u870_control_msg(struct r5u870_ctx *vhp, int write, int class,
				  u8 request, u16 value, u16 index, void *data,
				  u16 size)
{
	char *dbuf = NULL;
	int res;

	if (size) {
		dbuf = kmalloc(size, GFP_KERNEL);
		if (!dbuf)
			return -ENOMEM;
		if (write)
			memcpy(dbuf, data, size);
	}

	res = usb_control_msg(r5u870_dev(vhp),
				  write
				  ? usb_sndctrlpipe(r5u870_dev(vhp), 0)
				  : usb_rcvctrlpipe(r5u870_dev(vhp), 0),
				  request,
				  (write ? USB_DIR_OUT : USB_DIR_IN) |
				  (class
				   ? (USB_TYPE_CLASS | USB_RECIP_INTERFACE)
				   : (USB_TYPE_VENDOR | USB_RECIP_DEVICE)),
				  value, index, dbuf, size,
				  vhp->vh_timeout);

	if (dbuf) {
		if (!write)
			memcpy(data, dbuf, size);
		kfree(dbuf);
	}
	return res;
}


/*
 * Controls
 *
 * For this driver, there is only one get function, which retrieves the
 * value for the control stored in the r5u870_ctx structure.  Each style
 * of control -- WDM/UVC -- have separate set functions.
 *
 * There are query functions for setting the disabled flag on manual
 * white balance controls when auto white balance is enabled.
 */

enum {
	V4L2_CID_R5U870_SHARPNESS = (V4L2_CID_PRIVATE_BASE + 0),
	V4L2_CID_R5U870_GREEN_BALANCE,
	V4L2_CID_R5U870_AUTOEXPOSURE,
	V4L2_CID_R5U870_POWERLINE,
	V4L2_CID_R5U870_BACKLIGHT,
	V4L2_CID_R5U870_PRIVACY,
	V4L2_CID_R5U870_NIGHT_MODE,
};

static int r5u870_get_ctrl(struct usbcam_dev *udp,
			   const struct usbcam_ctrl *basep,
			   struct v4l2_ext_control *c)
{
	struct r5u870_ctrl *ctrlp = container_of(basep, struct r5u870_ctrl,
						 base);
	c->value = ctrlp->value;
	return 0;
}

static int r5u870_query_ctrl(struct usbcam_dev *udp,
				 const struct usbcam_ctrl *basep,
				 struct v4l2_queryctrl *c)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	struct r5u870_ctrl *ctrlp = container_of(basep, struct r5u870_ctrl,
						base);
	int auto_ctrl = 0;

	if (ctrlp->auto_offset) {
		auto_ctrl = *(int *) (((char *) vhp) + ctrlp->auto_offset);
		if (auto_ctrl)
			c->flags |= V4L2_CTRL_FLAG_INACTIVE;
	}
	return 0;
}

static int r5u870_set_controls(struct r5u870_ctx *vhp, int dflt)
{
	const struct r5u870_ctrl *ctrlp;
	struct v4l2_ext_control cv;
	int res;

	res = 0;
	list_for_each_entry(ctrlp, &vhp->vh_parent->ud_ctrl_list,
				base.uc_links) {
		cv.id = ctrlp->base.uc_v4l.id;

		if (dflt)
			cv.value = ctrlp->base.uc_v4l.default_value;
		else
			cv.value = ctrlp->value;

		res = ctrlp->base.set_fn(vhp->vh_parent,
					 &ctrlp->base,
					 &cv);
		if (res)
			break;
	}

	if (vhp->vh_ctrl_reg_enable &&
		!vhp->vh_ctrl_auto_suppress &&
		vhp->vh_model->rm_no_first_auto_suppress)
		vhp->vh_ctrl_auto_suppress = 1;

	return res;
}


/*
 * Microcode management and device initialization functions follow
 */

static int r5u870_set_gen_reg(struct r5u870_ctx *vhp,
				  int cmd, int reg, int val)
{
	int res;
	res = r5u870_control_msg(vhp, 1, 0, cmd, val, reg, NULL, 0);
	if (res < 0) {
		r5u_err(vhp, "set_gen_reg %04x/%04x/%04x failed: %d",
			cmd, reg, val, res);
	    // XXX: Only commented for debugging. UNCOMMENT BEFORE RELEASE!
        /*3pei*/
		return res;
	}
	return 0;
}

/*
 * Send the power-up init sequence, in case it is needed.
 */
static int r5u870_microcode_upload(struct r5u870_ctx *vhp)
{
	const struct firmware *fws;
	char *pgbuf;
	const u8 *dptr;
	int tolerance = 3;
	int i, rem, addr, len, res = 0;

	pgbuf = (char *) kmalloc(64, GFP_KERNEL);
	if (!pgbuf)
		return -ENOMEM;

	r5u_dbg(vhp, R5U_INIT, "loading microcode file \"%s\"",
		vhp->vh_model->rm_ucode_file);

	res = request_firmware(&fws,
				   vhp->vh_model->rm_ucode_file,
				   &vhp->vh_parent->ud_dev->dev);

	if (res) {
		r5u_err(vhp, "Microcode file \"%s\" is missing",
			vhp->vh_model->rm_ucode_file);
		r5u_err(vhp, "Please see http://wiki.mediati.org/r5u870/Microcode");
		kfree(pgbuf);
		return res;
	}

	i = 0;
	dptr = fws->data;
	rem = fws->size;
	while (rem) {
		if (rem < 3) {
			r5u_err(vhp, "Microcode file msg %d is incomplete", i);
			res = -EINVAL;
			break;
		}

		len = dptr[0];
		addr = dptr[1] | (dptr[2] << 8);
		dptr += 3;
		rem -= 3;

		if ((rem < len) || (len > 64)) {
			r5u_err(vhp, "Microcode file msg %d has bad length %d",
				i, len);
			res = -EINVAL;
			break;
		}

		/*
		 * The USB stack has issues with the initseq data if
		 * initseq points into the vmalloc arena.  This is
		 * the case for microcode embedded in a module, or
		 * data loaded by request_firmware().
		 *
		 * As a workaround, we memcpy() into a kmalloc page.
		 */
		memcpy(pgbuf, dptr, len);
		dptr += len;
		rem -= len;

	retry:
		/* TODO: Maybe make this use r5u870_control_msg or similar? */
		res = usb_control_msg(r5u870_dev(vhp),
					  usb_sndctrlpipe(r5u870_dev(vhp), 0),
					  0xa0,
					  USB_DIR_OUT | USB_TYPE_VENDOR |
					  USB_RECIP_DEVICE,
					  addr, 0, pgbuf, len, vhp->vh_timeout);

		if (res < 0) {
			if (tolerance--)
				goto retry;
			r5u_err(vhp, "command a0[%d] failed: %d",
				i, res);
			break;
		}
		if (res != len) {
			r5u_err(vhp, "command a0[%d] failed: %d (exp %d)",
				i, res, len);
			res = -EIO;
			break;
		}

		i++;
	}

	release_firmware(fws);
	kfree(pgbuf);
	return res;
}

static int r5u870_microcode_get_state(struct r5u870_ctx *vhp)
{
	char buf[1];
	int res;
	r5u_dbg(vhp, R5U_INIT, "requesting microcode state");
	res = r5u870_control_msg(vhp, 0, 0, 0xa4, 0, 0, buf, 1);
	if ((res != 1) || ((buf[0] != 0) && (buf[0] != 1))) {
		r5u_err(vhp, "command 0xa4 failed: %d", res);
		return res < 0 ? res : -EIO;
	}

	r5u_dbg(vhp, R5U_INIT, "camera reports %s microcode state",
		buf[0] ? "positive" : "negative");

	return (buf[0] == 0) ? -ENOENT : 0;
}

static int r5u870_microcode_get_ver(struct r5u870_ctx *vhp, int *verp)
{
	char buf[2];
	int res;

	r5u_dbg(vhp, R5U_INIT, "requesting microcode version");
	res = r5u870_control_msg(vhp, 0, 0, 0xc3, 0, 0x0e, buf, 2);
	if (res != 2) {
		r5u_err(vhp, "command 0xa3 failed: %d", res);
		return res < 0 ? res : -EIO;
	}

	res = le16_to_cpup((__le16 *) buf);
	r5u_dbg(vhp, R5U_INIT, "camera reports version %04x", res);
	*verp = res;
	return 0;
}

static int r5u870_microcode_enable(struct r5u870_ctx *vhp)
{
	char buf[1];
	int res;

	r5u_dbg(vhp, R5U_INIT, "enabling microcode");
	buf[0] = 1;
	res = r5u870_control_msg(vhp, 1, 0, 0xa1, 0, 0, buf, 1);
	if (res != 1) {
		r5u_err(vhp, "command 0xa1 failed: %d", res);
		return res < 0 ? res : -EIO;
	}

	return 0;
}

static int r5u870_microcode_reset(struct r5u870_ctx *vhp)
{
	int res;
	r5u_dbg(vhp, R5U_INIT, "sending microcode reset command");
	msleep(100);		/* The Windows driver waits 1sec */
	res = r5u870_set_gen_reg(vhp, 0xa6, 0, 0);
	if (!res)
		msleep(200);
	return res;
}

/*
 * Initialize the camera after it is detected.
 */
static int r5u870_dev_init(struct r5u870_ctx *vhp)
{
	int mcver;
	int res;

	if (!vhp->vh_model->rm_ucode_file)
		return 0;

	res = r5u870_microcode_get_state(vhp);
	if (res && (res != -ENOENT))
		return res;

	if (!res) {
		res = r5u870_microcode_get_ver(vhp, &mcver);
		if (res)
			return res;

		if (mcver != vhp->vh_model->rm_ucode_version) {
			res = r5u870_microcode_reset(vhp);
			if (res)	
				return res;
			res = -ENOENT;
		}
	}


	if (res == -ENOENT) {
		res = r5u870_microcode_upload(vhp);
		if (res < 0)
			return res;

		res = r5u870_microcode_enable(vhp);
		if (res)
			return res;

		res = r5u870_microcode_get_ver(vhp, &mcver);
		if (res)
			return res;
	}

	if (mcver != vhp->vh_model->rm_ucode_version)
		r5u_err(vhp, "Unexpected microcode version "
			"(exp:%04x got:%04x)",
			vhp->vh_model->rm_ucode_version, mcver);

	/* Halt capture in case it's running (broken driver?) */
	res = vhp->vh_cap_stop(vhp);
	if (res < 0)
		return res;

	return 0;
}

/*
 * WDM Device Registers are listed below.
 *
 * To set: use r5u870_set_reg_wdm().
 *
 * No information is given about how to retrieve values from these
 * registers.
 *
 * Note that some register IDs are overloaded.  The Sony cam drivers
 * seem to use 0x36 for backlight compensation and 0x37 for gain,
 * whereas the HP drivers use 0x36 for color enable and 0x37 for frame
 * rate.
 */
enum {
	/* Brightness: [0,127] D:63 */
	R5U870_REG_BRIGHTNESS = 0x02,

	/* Contrast: [0,127] D:63 */
	R5U870_REG_CONTRAST = 0x04,

	/* Hue: [-180,180] D:0 (16-bit 2's complement) */
	R5U870_REG_HUE = 0x34,

	/* Saturation: [0,127] D:63 */
	R5U870_REG_SATURATION = 0x05,

	/* Sharpness: [0,127] D:63 */
	R5U870_REG_SHARPNESS = 0x03,

	/* Gamma correction: [1,500] D:100 */
	R5U870_REG_GAMMA = 0x35,

	/* Registers with unknown usage */
	R5U870_REG_COLOR_ENABLE = 0x36,

	/* White balance: [0,127] D:63 */
	R5U870_REG_WHITE_BALANCE = 0x1,

	/* Frame Rate: D:30 */
	R5U870_REG_FRAME_RATE = 0x37,

	/* Registers with unknown usage */
	R5U870_REG_BRIGHTNESS_EX = 0x20,
	R5U870_REG_CONTRAST_EX = 0x21,
	R5U870_REG_HUE_EX = 0x22,
	R5U870_REG_SATURATION_EX = 0x23,
	R5U870_REG_SHARPNESS_EX = 0x24,
	R5U870_REG_GAMMA_EX = 0x25,

	/* White balance red value: [0,255] D:127 */
	R5U870_REG_WB_RED_EX = 0x26,

	/* White balance green value: [0,255] D:127 */
	R5U870_REG_WB_GREEN_EX = 0x27,

	/* White balance blue value: [0,255] D:127 */
	R5U870_REG_WB_BLUE_EX = 0x28,

	/* Auto white balance: [0,1] D:1 */
	R5U870_REG_WB_AUTO_EX = 0x29,

	/* Exposure Control: [0,255] D:255 */
	R5U870_REG_EXPOSURE_EX = 0x2a,

	/* Auto Exposure Control: [0,1] D:1 */
	R5U870_REG_AEC_EX = 0x2b,

	/* Gain: [0,127] D:63 */
	R5U870_REG_GAIN_EX = 0x2c,

	/* Auto Gain: [0,1] D:0 */
	R5U870_REG_AGC_EX = 0x2d,

	/* Light source flicker compensation: see R5U870_POWERLINE */
	R5U870_REG_POWERLINE_EX = 0x2e,

	/* Registers with unknown usage */
	R5U870_REG_SCENE_EX = 0x2f,

	/* Vertical flip: [0,1] D:0 */
	R5U870_REG_VFLIP_EX = 0x30,

	/* Horizontal flip: [0,1] D:0 */
	R5U870_REG_HFLIP_EX = 0x31,

	/* Blank image: [0,1] D:0 */
	R5U870_REG_PRIVACY_EX = 0x32,

	/* Night mode: [0,1] D:0 */
	R5U870_REG_NIGHT_MODE_EX = 0x33,

	/* Backlight compensation: [0,500] D:1 */
	R5U870_REG_BACKLIGHT_COMP = 0x36,

	/* Registers with unknown usage */
	R5U870_REG_GAIN = 0x37,

	/* Backlight compensation for 1834 device: [0,2] D:1 */
	R5U870_REG_BACKLIGHT_COMP_2 = 0x39,


	/* Values for R5U870_REG_POWERLINE flicker compensation */
	R5U870_POWERLINE_OFF = 0,
	R5U870_POWERLINE_50HZ = 1,
	R5U870_POWERLINE_60HZ = 2,

	/* Number of empty packets between frames */
	R5U870_EMPTYPKT_FRAME_DELIM = 10,

	/* Number of empty packets before declaring the device dead (.5sec) */
	R5U870_EMPTYPKT_GIVE_UP = 4000,
};

static int r5u870_set_reg_wdm(struct r5u870_ctx *vhp, int reg, int val)
{
	return r5u870_set_gen_reg(vhp, 0xc2, reg, val);
}

/*
 * Set the frame size and data format.
 * Do not call this function with the isochronous stream active.
 */
static int r5u870_set_fmt_wdm(struct r5u870_ctx *vhp,
				  const struct r5u870_pix_fmt *fmtp,
				  const struct r5u870_resolution *resp)
{
	int res;

	msleep(1);
	res = r5u870_set_gen_reg(vhp, 0xc5, 2, fmtp->rp_formatidx);
	if (res)
		return res;
	msleep(1);
	res = r5u870_set_gen_reg(vhp, 0xc5, 0, resp->rw_width);
	if (res)
		return res;
	msleep(1);
	res = r5u870_set_gen_reg(vhp, 0xc5, 1, resp->rw_height);
	if (res)
		return res;
	msleep(5);
	return 0;
}


/*
 * Turn frame grabbing on or off (WDM).
 * This will also turn on or off the LED.
 */
static int r5u870_set_cap_state_wdm(struct r5u870_ctx *vhp, int val)
{
	return r5u870_set_gen_reg(vhp, 0xc4, val, 0);
}

static int r5u870_cap_stop_wdm(struct r5u870_ctx *vhp)
{
	return r5u870_set_cap_state_wdm(vhp, 0);
}

/*
 * r5u870_decide_pkt_wdm
 *
 *	Based on the size of an isochronous data packet, this function
 * decides whether to copy the packet into the frame buffer and possibly
 * complete the frame, or to discard both the packet and the frame.
 *
 * Returns:
 * 	0		Frame is done
 *	-EAGAIN		Append packet to frame, frame is not done
 *	-EPIPE		Discard frame and packet
 *	-EIO		The device is nonresponsive, abort
 */
static int r5u870_decide_pkt_wdm(struct r5u870_ctx *vhp, int pktstatus,
				 int pktlen, const u8 *pktdata, int *start)
{
	int ret = -EAGAIN;

	*start = 0;

	if (pktstatus) {
		/* Abort current frame */
		r5u_dbg(vhp, R5U_FRAME, "frame abort: packet status %d",
			pktstatus);
		vhp->vh_frame_accum = -1;
		vhp->vh_emptypkts = 0;
		ret = -EPIPE;

	} else if (!pktlen) {
		if (++vhp->vh_emptypkts == R5U870_EMPTYPKT_FRAME_DELIM) {
			if (vhp->vh_frame_accum == -1) {
				/* Frame was previously aborted */
				ret = -EPIPE;
			} else if (vhp->vh_frame_accum ==
				   vhp->vh_parent->ud_format.sizeimage) {
				/* Complete frame */
				ret = 0;
			} else {
				/* Not enough data in frame sequence */
				r5u_dbg(vhp, R5U_FRAME, "frame abort: "
					"Frame seq too short (exp:%d got:%d)",
					vhp->vh_parent->ud_format.sizeimage,
					vhp->vh_frame_accum);
				ret = -EPIPE;
			}

			if (!vhp->vh_firstframe) {
				/* Always reject the first frame */
				vhp->vh_firstframe = 1;
				vhp->vh_frame_accum = -1;
				ret = -EPIPE;
			} else {
				vhp->vh_frame_accum = 0;
			}
		}

		else if (vhp->vh_emptypkts >= R5U870_EMPTYPKT_GIVE_UP) {
			r5u_dbg(vhp, R5U_FRAME, "%d empty packets, giving up",
				vhp->vh_emptypkts);
			ret = -EIO;
		}

	} else {
		vhp->vh_emptypkts = 0;
		if (vhp->vh_frame_accum == -1) {
			/* Frame was previously aborted */
			ret = -EPIPE;
		} else if ((vhp->vh_frame_accum + pktlen) <=
				vhp->vh_parent->ud_format.sizeimage) {
			/* Append this data */
			vhp->vh_frame_accum += pktlen;
		} else {
			/* Oversized frame, abort */
			r5u_dbg(vhp, R5U_FRAME, "frame abort: "
				"Frame seq too long");
			vhp->vh_frame_accum = -1;
			ret = -EPIPE;
		}
	}

	return ret;
}

static int r5u870_set_manual_ctrls_wdm(struct r5u870_ctx *vhp, int auto_offset)
{
	const struct r5u870_ctrl *ctrlp;
	int val, res;

	res = 0;
	list_for_each_entry(ctrlp, &vhp->vh_parent->ud_ctrl_list,
				base.uc_links) {
		if (ctrlp->auto_offset != auto_offset)
			continue;
		if (!vhp->vh_ctrl_reg_enable) {
			vhp->vh_ctrl_sync = 0;
			continue;
		}

		val = ctrlp->value;

		r5u_dbg(vhp, R5U_CTRL, "control %s/wdm %02x <= %d [manual]",
			ctrlp->base.uc_v4l.name, ctrlp->reg, val);
		res = r5u870_set_reg_wdm(vhp, ctrlp->reg, val);
		if (res)
			break;
	}
	return res;
}

static int r5u870_set_ctrl_wdm(struct usbcam_dev *udp,
				   const struct usbcam_ctrl *basep,
				   const struct v4l2_ext_control *c)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	struct r5u870_ctrl *ctrlp = container_of(basep, struct r5u870_ctrl,
						 base);
	int res = 0;
	int auto_ctrl = 0;

	if (ctrlp->auto_offset)
		auto_ctrl = *(int *) (((char *) vhp) + ctrlp->auto_offset);

	if (auto_ctrl && vhp->vh_ctrl_auto_suppress) {
		r5u_dbg(vhp, R5U_CTRL, "control %s <= %d [auto suppress]",
			ctrlp->base.uc_v4l.name, c->value);

	} else if (!vhp->vh_ctrl_reg_enable) {
		r5u_dbg(vhp, R5U_CTRL, "control %s <= %d [capture off]",
			ctrlp->base.uc_v4l.name, c->value);
		vhp->vh_ctrl_sync = 0;

	} else {
		r5u_dbg(vhp, R5U_CTRL, "control %s/wdm %02x <= %d",
			ctrlp->base.uc_v4l.name, ctrlp->reg, c->value);
		res = r5u870_set_reg_wdm(vhp, ctrlp->reg, c->value);
		if (res)
			return res;
	}

	ctrlp->value = c->value;

	if (ctrlp->val_offset)
		*(int *) (((char *) vhp) + ctrlp->val_offset) = c->value;

	if (ctrlp->is_auto && !c->value)
		res = r5u870_set_manual_ctrls_wdm(vhp, ctrlp->val_offset);

	return res;
}

/*
 * WDM control templates follow
 *
 * Each device has an array of integer control IDs, which refer to an
 * element in the control template array.  When the device is detected,
 * we build the control array out of the element list and the templates,
 * by r5u870_wdm_add_ctrls().
 */

enum {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_SHARPNESS,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_GAMMA,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_500,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_500_DEF1,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_X1834,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_AUTO_EXPOSURE,
	R5U870_WDM_CTRL_EXPOSURE,
	R5U870_WDM_CTRL_AUTO_GAIN,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_VFLIP_DEFAULTON,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,

	R5U870_WDM_CTRL_LAST = 0xffff,
};

/* TODO: Merge these into V4L API. */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#define V4L2_CID_GREEN_BALANCE		(V4L2_CID_BASE+24)
#define V4L2_CID_EXPOSURE_AUTO		(V4L2_CID_BASE+25)
#define V4L2_CID_POWER_LINE_FREQUENCY	(V4L2_CID_BASE+26)
#define V4L2_CID_BACKLIGHT_COMPENSATION	(V4L2_CID_BASE+27)
#define V4L2_CID_PRIVACY		(V4L2_CID_BASE+28)
#define V4L2_CID_NIGHT_MODE		(V4L2_CID_BASE+29)
#define V4L2_CID_SHARPNESS		(V4L2_CID_BASE+30)
#define V4L2_CID_LASTP1			(V4L2_CID_BASE+31) /* last CID + 1 */
#else
#define V4L2_CID_GREEN_BALANCE		(V4L2_CID_BASE+31)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
#define V4L2_CID_PRIVACY		(V4L2_CID_BASE+32)
#define V4L2_CID_LASTP1			(V4L2_CID_BASE+34) /* last CID + 1 */
#endif
#define V4L2_CID_NIGHT_MODE		(V4L2_CID_BASE+33)
#endif

/* 2007-09-09 TJ Ensure names are identical to uvcvideo,
 * so user applications aren't confused by differing results for UVC queries */
static const char *r5u870_powerline_names[] = { "Disabled", "50 Hz", "60 Hz" };

/* TODO: Use our own internal control IDs, instead of crap unmerged ones. */
static struct r5u870_ctrl r5u870_wdm_ctrls[] = {

	[R5U870_WDM_CTRL_BRIGHTNESS] = {
		.base = { .uc_v4l = { .id = V4L2_CID_BRIGHTNESS,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Brightness",
					  .minimum = 0,
					  .maximum = 127,
					  .step = 1,
					  .default_value = 63,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_BRIGHTNESS,
	},
	[R5U870_WDM_CTRL_CONTRAST] = { 
		.base = { .uc_v4l = { .id = V4L2_CID_CONTRAST,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Contrast",
					  .minimum = 0,
					  .maximum = 127,
					  .step = 1,
					  .default_value = 63,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_CONTRAST,
	},
	[R5U870_WDM_CTRL_SATURATION] = {
		.base = { .uc_v4l = { .id = V4L2_CID_SATURATION,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Saturation",
					  .minimum = 0,
					  .maximum = 127,
					  .step = 1,
					  .default_value = 63,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_SATURATION,
	},
	[R5U870_WDM_CTRL_SHARPNESS] = {
		.base = { .uc_v4l = { .id = V4L2_CID_SHARPNESS,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Sharpness",
					  .minimum = 0,
					  .maximum = 127,
					  .step = 1,
					  .default_value = 63,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_SHARPNESS,
	},
	[R5U870_WDM_CTRL_HUE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_HUE,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Hue",
					  .minimum = -180,
					  .maximum = 180,
					  .step = 1,
					  .default_value = 0,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_HUE,
	},
	[R5U870_WDM_CTRL_GAMMA] = {
		.base = { .uc_v4l = { .id = V4L2_CID_GAMMA,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Gamma",
					  .minimum = 0,
					  .maximum = 500,
					  .step = 1,
					  .default_value = 100,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_GAMMA,
	},
	[R5U870_WDM_CTRL_BACKLIGHT_COMP_500] = {
		.base = { .uc_v4l = { .id = V4L2_CID_BACKLIGHT_COMPENSATION,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Backlight Compensation",
					  .minimum = 0,
					  .maximum = 500,
					  .step = 1,
					  .default_value = 250,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_BACKLIGHT_COMP,
	},
	[R5U870_WDM_CTRL_BACKLIGHT_COMP_500_DEF1] = {
		.base = { .uc_v4l = { .id = V4L2_CID_BACKLIGHT_COMPENSATION,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Backlight Compensation",
					  .minimum = 0,
					  .maximum = 500,
					  .step = 1,
					  .default_value = 1,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_BACKLIGHT_COMP,
	},
	[R5U870_WDM_CTRL_BACKLIGHT_COMP_X1834] = {
		.base = { .uc_v4l = { .id = V4L2_CID_BACKLIGHT_COMPENSATION,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Backlight Compensation",
					  .minimum = 0,
					  .maximum = 2,
					  .step = 1,
					  .default_value = 1,
					  .flags = V4L2_CTRL_FLAG_SLIDER },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_BACKLIGHT_COMP_2,
	},
	[R5U870_WDM_CTRL_WB_RED] = {
		.base = { .uc_v4l = { .id = V4L2_CID_RED_BALANCE,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "White Balance Red",
					  .minimum = 0,
					  .maximum = 255,
					  .step = 1,
					  .default_value = 127,
					  .flags = 0 },
			  .query_fn = r5u870_query_ctrl,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_WB_RED_EX,
		.auto_offset = offsetof(struct r5u870_ctx, vh_auto_wb)
	},
	[R5U870_WDM_CTRL_WB_GREEN] = {
		.base = { .uc_v4l = { .id = V4L2_CID_GREEN_BALANCE,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "White Balance Green",
					  .minimum = 0,
					  .maximum = 255,
					  .step = 1,
					  .default_value = 127,
					  .flags = 0 },
			  .query_fn = r5u870_query_ctrl,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_WB_GREEN_EX,
		.auto_offset = offsetof(struct r5u870_ctx, vh_auto_wb)
	},
	[R5U870_WDM_CTRL_WB_BLUE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_BLUE_BALANCE,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "White Balance Blue",
					  .minimum = 0,
					  .maximum = 255,
					  .step = 1,
					  .default_value = 127,
					  .flags = 0 },
			  .query_fn = r5u870_query_ctrl,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_WB_BLUE_EX,
		.auto_offset = offsetof(struct r5u870_ctx, vh_auto_wb)
	},
	[R5U870_WDM_CTRL_WB_AUTO] = {
		.base = { .uc_v4l = { .id = V4L2_CID_AUTO_WHITE_BALANCE,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "Auto White Balance",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 1,
					  .flags = V4L2_CTRL_FLAG_UPDATE },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_WB_AUTO_EX,
		.is_auto = 1,
		.val_offset = offsetof(struct r5u870_ctx, vh_auto_wb)
	},
	[R5U870_WDM_CTRL_AUTO_EXPOSURE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_EXPOSURE_AUTO,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "Auto Exposure Control",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 1,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_AEC_EX,
		.is_auto = 1,
		.val_offset = offsetof(struct r5u870_ctx, vh_aec)
	},
	[R5U870_WDM_CTRL_EXPOSURE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_EXPOSURE,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Exposure",
					  .minimum = 0,
					  .maximum = 511,
					  .step = 1,
					  .default_value = 255,
					  .flags = 0 },
			  .query_fn = r5u870_query_ctrl,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_EXPOSURE_EX,
		.auto_offset = offsetof(struct r5u870_ctx, vh_aec)
	},
	[R5U870_WDM_CTRL_AUTO_GAIN] = {
		.base = { .uc_v4l = { .id = V4L2_CID_AUTOGAIN,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "Auto Gain Control",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 1,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_AGC_EX,
		.is_auto = 1,
		.val_offset = offsetof(struct r5u870_ctx, vh_agc)
	},
	[R5U870_WDM_CTRL_GAIN] = {
		.base = { .uc_v4l = { .id = V4L2_CID_GAIN,
					  .type = V4L2_CTRL_TYPE_INTEGER,
					  .name = "Gain",
					  .minimum = 0,
					  .maximum = 127,
					  .step = 1,
					  .default_value = 63,
					  .flags = 0 },
			  .query_fn = r5u870_query_ctrl,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_GAIN_EX,
		.auto_offset = offsetof(struct r5u870_ctx, vh_agc)
	},
	[R5U870_WDM_CTRL_POWERLINE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_POWER_LINE_FREQUENCY,
					  .type = V4L2_CTRL_TYPE_MENU,
					  .name = "Power Line Frequency",
					  .minimum = 0,
					  .maximum = 2,
					  .step = 1,
					  .default_value = 0,
					  .flags = 0 },
			  .uc_menu_names = r5u870_powerline_names,
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_POWERLINE_EX,
	},
	[R5U870_WDM_CTRL_VFLIP] = {
		.base = { .uc_v4l = { .id = V4L2_CID_VFLIP,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "V-Flip",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 0,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_VFLIP_EX,
	},
	[R5U870_WDM_CTRL_VFLIP_DEFAULTON] = {
		.base = { .uc_v4l = { .id = V4L2_CID_VFLIP,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "V-Flip",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 1,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_VFLIP_EX,
	},
	[R5U870_WDM_CTRL_HFLIP] = {
		.base = { .uc_v4l = { .id = V4L2_CID_HFLIP,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "H-Flip",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 0,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_HFLIP_EX,
	},
	[R5U870_WDM_CTRL_PRIVACY] = {
		.base = { .uc_v4l = { .id = V4L2_CID_PRIVACY,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "Privacy",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 0,
					  .flags = 0 },
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_PRIVACY_EX,
	},
	[R5U870_WDM_CTRL_NIGHTMODE] = {
		.base = { .uc_v4l = { .id = V4L2_CID_NIGHT_MODE,
					  .type = V4L2_CTRL_TYPE_BOOLEAN,
					  .name = "Night Mode",
					  .minimum = 0,
					  .maximum = 1,
					  .step = 1,
					  .default_value = 0,
					  .flags = 0 },
			  /*  2007-09-09 TJ Add missing IOCTL query */
			  .query_fn = r5u870_query_ctrl, 
			  .get_fn = r5u870_get_ctrl,
			  .set_fn = r5u870_set_ctrl_wdm },
		.reg = R5U870_REG_NIGHT_MODE_EX,
	},
};

static int r5u870_wdm_add_ctrls(struct r5u870_ctx *vhp, const int *ctrlarray)
{
	struct r5u870_ctrl *ncp;
	int i;

	for (i = 0; ctrlarray[i] != R5U870_WDM_CTRL_LAST; i++) {
		ncp = (struct r5u870_ctrl *)
			usbcam_ctrl_add_tmpl(vhp->vh_parent,
					 &r5u870_wdm_ctrls[ctrlarray[i]].base,
						 sizeof(*ncp));
		if (!ncp)
			return -ENOMEM;
	}
	r5u_dbg(vhp, R5U_CTRL, "Added %d WDM controls", i);
	return 0;
}



/*
 * UVC related code follows
 */

enum {
	UVC_SC_VIDEOCONTROL = 1,
	UVC_SC_VIDEOSTREAMING = 1,

	UVC_VC_HEADER = 1,
	UVC_VC_INPUT_TERMINAL = 2,
	UVC_VC_OUTPUT_TERMINAL = 3,
	UVC_VC_SELECTOR_UNIT = 4,
	UVC_VC_PROCESSING_UNIT = 5,
	UVC_VC_EXTENSION_UNIT = 6,

	UVC_VC_REQUEST_ERROR_CODE_CONTROL = 0x02,

	UVC_VS_INPUT_HEADER = 0x01,
	UVC_VS_FORMAT_UNCOMPRESSED = 0x04,
	UVC_VS_FRAME_UNCOMPRESSED = 0x05,

	UVC_SET_CUR = 0x01,
	UVC_GET_CUR = 0x81,
	UVC_GET_MIN = 0x82,
	UVC_GET_MAX = 0x83,
	UVC_GET_RES = 0x84,
	UVC_GET_LEN = 0x85,
	UVC_GET_INFO = 0x86,
	UVC_GET_DEF = 0x87,

	UVC_PU_BACKLIGHT_COMPENSATION_CONTROL = 0x01,
	UVC_PU_BRIGHTNESS_CONTROL = 0x02,
	UVC_PU_CONTRAST_CONTROL = 0x03,
	UVC_PU_POWER_LINE_FREQUENCY_CONTROL = 0x05,
	UVC_PU_HUE_CONTROL = 0x06,
	UVC_PU_SATURATION_CONTROL = 0x07,
	UVC_PU_SHARPNESS_CONTROL = 0x08,
	UVC_PU_GAMMA_CONTROL = 0x09,

	UVC_VS_PROBE_CONTROL = 0x01,
	UVC_VS_COMMIT_CONTROL = 0x02,

};

static int r5u870_uvc_req(struct r5u870_ctx *vhp, int cmd,
			  u8 valhi, u8 vallow, u8 idxhi, u8 idxlow,
			  u8 *buf, int len)
{
	int out, res, stres;
	int tries = 5;
	u8 stbuf[1];

	out = (cmd == UVC_SET_CUR) ? 1 : 0;

retry:
	/* TODO: Base our other retry control message off this one. */
	res = r5u870_control_msg(vhp, out, 1, cmd,
				 (valhi << 8) | vallow, (idxhi << 8) | idxlow,
				 buf, len);

	if (res != -EPIPE)
		//r5u_err(vhp, "res != -EPIPE.");
		goto complete;

	stres = r5u870_control_msg(vhp, 0, 1, UVC_GET_CUR,
				   UVC_VC_REQUEST_ERROR_CODE_CONTROL << 8,
				   vhp->vh_ctrl_ifnum,
				   stbuf, sizeof(stbuf));

	if (((stres == -EPIPE) && --tries) ||
		((stres == 1) && (stbuf[0] == 1) && --tries)) {
		msleep(5);
		r5u_err(vhp, "uvc_req: retrying - EPIPE/stres error.");
		goto retry;
	}

	if (stres != 1) {
		r5u_err(vhp, "uvc_req: status req failed: %d", stres);
		goto complete;

	} else {
		r5u_err(vhp, "uvc_req: status %d", stbuf[0]);
	}

complete:
	if (res < 0) {
		r5u_err(vhp, "uvc_req %02x/%02x%02x/%02x%02x failed: %d",
			cmd, valhi, vallow, idxhi, idxlow, res);
	}

	return res;
}

static int r5u870_set_fmt_uvc(struct r5u870_ctx *vhp,
				  const struct r5u870_pix_fmt *fmtp,
				  const struct r5u870_resolution *resp)
{
	unsigned char buf[26];
	int res;

	memset(buf, 0, sizeof(buf));

	buf[2] = fmtp->rp_formatidx;
	buf[3] = resp->rw_frameidx;
	*(__le32 *) &buf[4] = cpu_to_le32(resp->rw_interval);

	r5u_dbg(vhp, R5U_FRAME,
		"set_format: fmtidx:%d frameidx:%d %dx%d ival:%d",
		fmtp->rp_formatidx, resp->rw_frameidx,
		resp->rw_width, resp->rw_height, resp->rw_interval);

	res = r5u870_uvc_req(vhp, UVC_SET_CUR, UVC_VS_PROBE_CONTROL, 0,
				 0, vhp->vh_iso_ifnum, buf, sizeof(buf));
	if (res != sizeof(buf)) {
		r5u_err(vhp, "%s: probe_control set_cur1: short write %d",
			__FUNCTION__, res);

		return -EIO;
	}

	res = r5u870_uvc_req(vhp, UVC_GET_CUR, UVC_VS_PROBE_CONTROL, 0,
				 0, vhp->vh_iso_ifnum, buf, sizeof(buf));
	if (res != sizeof(buf)) {
		r5u_err(vhp, "%s: probe_control get_cur: short read %d",
			__FUNCTION__, res);
		return -EIO;
	}

	if (buf[2] != fmtp->rp_formatidx) {
		r5u_err(vhp, "%s: probe_control get_cur: got fmt %d",
			__FUNCTION__, buf[2]);
		return -EIO;
	}

	if (buf[3] != resp->rw_frameidx) {
		r5u_err(vhp, "%s: probe_control get_cur: got frame idx %d",
			__FUNCTION__, buf[3]);
		return -EIO;
	}

	res = r5u870_uvc_req(vhp, UVC_SET_CUR, UVC_VS_COMMIT_CONTROL, 0,
				 0, vhp->vh_iso_ifnum, buf, sizeof(buf));
	if (res != sizeof(buf)) {
		r5u_err(vhp, "%s: commit_control set_cur2: short write %d",
			__FUNCTION__, res);
		return -EIO;
	}

	vhp->vh_iso_minpacket = le32_to_cpup((__le32 *) &buf[22]);

	return 0;
}

static int r5u870_cap_stop_uvc(struct r5u870_ctx *vhp)
{
	/* UVC capture is controlled by changing the altsetting */
	return 0;
}

static int r5u870_decide_pkt_uvc(struct r5u870_ctx *vhp, int pktstatus,
				 int pktlen, const u8 *pktdata, int *start)
{
	if (!pktlen) {
		vhp->vh_emptypkts++;
		if (vhp->vh_emptypkts >= R5U870_EMPTYPKT_GIVE_UP) {
			r5u_err(vhp, "capture abort: too many empty pkts");
			return -EIO;
		}
		if (vhp->vh_frame_accum < 0)
			return -EPIPE;
		return -EAGAIN;
	}

	vhp->vh_emptypkts = 0;
	if (vhp->vh_frame_accum < 0) {
		if (pktdata[1] & 2)
			vhp->vh_frame_accum = 0;
		return -EPIPE;
	}

	if ((pktdata[0] < 2) || (pktdata[0] > pktlen)) {
		r5u_err(vhp, "capture abort: hdrlen=%d pktlen=%d",
			pktdata[0], pktlen);
		return -EIO;
	}
	vhp->vh_frame_accum += pktlen - pktdata[0];
	if (vhp->vh_frame_accum > vhp->vh_parent->ud_format.sizeimage) {
		r5u_err(vhp, "frame abort: accum=%d", vhp->vh_frame_accum);
		vhp->vh_frame_accum = -1;
		return -EPIPE;
	}

	*start = pktdata[0];
	if (pktdata[1] & 2) {
		if (vhp->vh_frame_accum <
			vhp->vh_parent->ud_format.sizeimage) {
			r5u_err(vhp, "warning: short frame (exp:%d got:%d)",
				vhp->vh_parent->ud_format.sizeimage,
				vhp->vh_frame_accum);
		}
		vhp->vh_frame_accum = 0;
		return 0;
	}
	return -EAGAIN;
}


/*
 * Known video format GUIDs and V4L pixel format translations
 *
 * Only uncompressed formats are supported, as Ricoh webcams are too
 * minimal to do anything else.
 */

static const struct r5u870_uvc_fmtinfo {
	const char	*fi_name;
	int		fi_v4l_id;
	u8		fi_guid[16];

} r5u870_uvc_fmts[] = {
	{ .fi_name = "YUY2 4:2:2",
	  .fi_v4l_id = V4L2_PIX_FMT_YUYV,
	  .fi_guid = { 0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00,
			   0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71 } },
	{ }
};

static int r5u870_uvc_add_resolution(struct r5u870_ctx *vhp,
					 struct r5u870_pix_fmt *fmtp,
					 int width, int height, int reqbw,
					 int frameidx, int interval)
{
	int i;
	struct r5u870_resolution *resp;

	if (!width || !height) {
		r5u_dbg(vhp, R5U_INIT, "invalid frame descriptor %d at %dx%d",
			frameidx, width, height);
		return -EINVAL;
	}

	resp = NULL;
	for (i = 0; i < fmtp->rp_restbl_alloc; i++) {
		if (!fmtp->rp_restbl[i].rw_width) {
			resp = (struct r5u870_resolution *)
				&fmtp->rp_restbl[i];
			break;
		}
	}

	if (!resp) {
		r5u_dbg(vhp, R5U_INIT,
			"no space for frame descriptor %d at %dx%d",
			frameidx, width, height);
		return 0;
	}

	resp->rw_width = width;
	resp->rw_height = height;
	resp->rw_frameidx = frameidx;
	resp->rw_reqbw = reqbw;
	resp->rw_interval = interval;

	r5u_dbg(vhp, R5U_INIT, "Found resolution %d: %dx%d ival %d (%d B/s)",
		frameidx, width, height, interval, reqbw);

	return 0;
}

static int r5u870_uvc_add_fmt(struct r5u870_ctx *vhp, const u8 *guid,
				  int fmtidx, int nresolutions,
				  struct r5u870_pix_fmt **new_fmt)
{
	const struct r5u870_uvc_fmtinfo *fip, *fmtarray = r5u870_uvc_fmts;
	struct r5u870_pix_fmt *nfp, *fmtp;
	int i;
	
	fip = NULL;
	for (i = 0; fmtarray[i].fi_name != NULL; i++) {
		if (!memcmp(fmtarray[i].fi_guid, guid, 16)) {
			fip = &fmtarray[i];
			break;
		}
	}

	if (fip == NULL) {
		r5u_dbg(vhp, R5U_INIT, "unknown format "
			"%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-"
			"%02x%02x%02x%02x%02x%02x",
			guid[0], guid[1], guid[2], guid[3], guid[4], guid[5],
			guid[6], guid[7], guid[8], guid[9], guid[10],
			guid[11], guid[12], guid[13], guid[14], guid[15]);
		return -ENOENT;
	}

	nfp = (struct r5u870_pix_fmt *)
		kmalloc((1 + vhp->vh_npixfmts) * sizeof(*nfp), GFP_KERNEL);
	if (!nfp)
		return -ENOMEM;
	if (vhp->vh_npixfmts)
		memcpy(nfp, vhp->vh_pixfmts, vhp->vh_npixfmts * sizeof(*nfp));

	fmtp = &nfp[vhp->vh_npixfmts];
	memset(fmtp, 0, sizeof(*fmtp));
	strlcpy(fmtp->base.description,
		fip->fi_name,
		sizeof(fmtp->base.description));
	fmtp->base.pixelformat = fip->fi_v4l_id;
	fmtp->rp_formatidx = fmtidx;
	fmtp->rp_restbl = (struct r5u870_resolution *)
		kmalloc((1 + nresolutions) * sizeof(*fmtp->rp_restbl),
			GFP_KERNEL);
	if (!fmtp->rp_restbl) {
		kfree(nfp);
		return -ENOMEM;
	}

	memset((char *)fmtp->rp_restbl, 0,
		   (1 + nresolutions) * sizeof(*fmtp->rp_restbl));
	fmtp->rp_restbl_alloc = nresolutions;

	if (vhp->vh_npixfmts && vhp->vh_dyn_pixfmts)
		kfree(vhp->vh_pixfmts);
	vhp->vh_pixfmts = nfp;
	vhp->vh_npixfmts++;

	if (new_fmt)
		(*new_fmt) = fmtp;

	r5u_dbg(vhp, R5U_INIT, "Found format %d: %c%c%c%c (%d frames)",
		fmtidx,
		fmtp->base.pixelformat & 0xff,
		(fmtp->base.pixelformat >> 8) & 0xff,
		(fmtp->base.pixelformat >> 16) & 0xff,
		(fmtp->base.pixelformat >> 24) & 0xff,
		nresolutions);

	return 0;
}

static int r5u870_uvc_parse_vs(struct r5u870_ctx *vhp, int ifnum)
{
	struct usb_interface *intf;
	struct usb_host_interface *aintf;
	struct r5u870_pix_fmt *curfmt = NULL;
	u8 *desc;
	int dlen, rlen;
	int wid, hgt, reqbw, ival;
	int res;

	intf = usb_ifnum_to_if(r5u870_dev(vhp), ifnum);
	if (!intf)
		return -EINVAL;

	aintf = intf->cur_altsetting;

	for (desc = aintf->extra, rlen = aintf->extralen;
		 rlen > 2;
		 rlen -= desc[0], desc += desc[0]) {

		dlen = desc[0];
		if (dlen < 2)
			return -EINVAL;
		if (desc[1] != USB_DT_CS_INTERFACE)
			continue;
		if (dlen < 3)
			return -EINVAL;

		switch (desc[2]) {
		case UVC_VS_INPUT_HEADER:
			if (dlen < 7) {
				r5u_err(vhp, "VS_INPUT_HEADER too short "
					"at %d bytes", dlen);
				return -EINVAL;
			}
			vhp->vh_iso_ep = desc[6] & 0xf;
			break;

		case UVC_VS_FORMAT_UNCOMPRESSED:
			if (dlen < 21) {
				r5u_err(vhp, "VS_FORMAT_UNCOMP too short "
					"at %d bytes", dlen);
				break;
			}
			res = r5u870_uvc_add_fmt(vhp, &desc[5], desc[3],
						 desc[4], &curfmt);
			if (res) {
				if (res != -ENOENT)
					return res;
				curfmt = NULL;
			}
			break;

		case UVC_VS_FRAME_UNCOMPRESSED:
			if (dlen < 26) {
				r5u_err(vhp, "VS_FRAME_UNCOMP too short "
					"at %d bytes", dlen);
				break;
			}
			if (!curfmt) {
				r5u_dbg(vhp, R5U_INIT, "VS_FRAME_UNCOMP "
					"not following VS_FORMAT_UNCOMP");
				break;
			}

			wid = desc[5] | (desc[6] << 8);
			hgt = desc[7] | (desc[8] << 8);
			reqbw = desc[13] | (desc[14] << 8) |
				(desc[15] << 16) | (desc[16] << 24);
			reqbw = (reqbw + 7) / 8;
			ival = le32_to_cpup((__le32 *) &desc[21]);

			res = r5u870_uvc_add_resolution(vhp, curfmt, wid, hgt,
							reqbw, desc[3], ival);
			if (res)
				return res;
			break;
		}
	}

	return 0;
}


/*
 * Known UVC controls for processing units
 *
 * Not all types of UVC controls are supported.  We stick to simple
 * controls with one or two byte values, and don't do anything very
 * complicated.
 *
 * We don't support camera unit controls, or multiple processing units
 * with instances of the same control.  We also don't check for
 * redefined control IDs, and let usbcam do this for us.
 */

struct r5u870_uvc_ctrlinfo {
	const char	*ci_name;
	u32		ci_v4l_id;
	int		ci_v4l_type;
	int		ci_v4l_flags;
	u8		ci_reg;
	u8		ci_size;
	u8		ci_bm_index;
	int		ci_min, ci_max, ci_def;
	u8		ci_min_force, ci_max_force, ci_def_force;
	const char	**ci_menu_names;
	int		ci_val_offset;
};

static struct r5u870_uvc_ctrlinfo r5u870_uvc_proc_ctrls[] = {
	{ .ci_name = "Brightness",
	  .ci_v4l_id = V4L2_CID_BRIGHTNESS,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_BRIGHTNESS_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 0 },
	{ .ci_name = "Contrast",
	  .ci_v4l_id = V4L2_CID_CONTRAST,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_CONTRAST_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 1 },
	{ .ci_name = "Hue",
	  .ci_v4l_id = V4L2_CID_HUE,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_HUE_CONTROL,
	  .ci_size = 2,
	  .ci_min = -180, .ci_min_force = 1,
	  .ci_max = 180, .ci_max_force = 1,
	  .ci_def = 0, .ci_def_force = 1,
	  .ci_bm_index = 2 },
	{ .ci_name = "Saturation",
	  .ci_v4l_id = V4L2_CID_SATURATION,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_SATURATION_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 3 },
	{ .ci_name = "Sharpness",
	  .ci_v4l_id = V4L2_CID_SHARPNESS,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_SHARPNESS_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 4 },
	{ .ci_name = "Gamma",
	  .ci_v4l_id = V4L2_CID_GAMMA,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_GAMMA_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 5 },
	{ .ci_name = "Backlight Compensation",
	  .ci_v4l_id = V4L2_CID_BACKLIGHT_COMPENSATION,
	  .ci_v4l_type = V4L2_CTRL_TYPE_INTEGER,
	  .ci_v4l_flags = V4L2_CTRL_FLAG_SLIDER,
	  .ci_reg = UVC_PU_BACKLIGHT_COMPENSATION_CONTROL,
	  .ci_size = 2,
	  .ci_bm_index = 8 },
	{ .ci_name = "Power Line Frequency",
	  .ci_v4l_id = V4L2_CID_POWER_LINE_FREQUENCY,
	  .ci_v4l_type = V4L2_CTRL_TYPE_MENU,
	  .ci_reg = UVC_PU_POWER_LINE_FREQUENCY_CONTROL,
	  .ci_size = 1,
	  .ci_bm_index = 10,
	  .ci_min = 0, .ci_min_force = 1,
	  .ci_max = 2, .ci_max_force = 1,
	  .ci_menu_names = r5u870_powerline_names },
	{ }
};

static int r5u870_uvc_ctrl_req(struct r5u870_ctx *vhp,
				   const struct r5u870_ctrl *ctrlp,
				   int req, int *value)
{
	u8 buf[4];
	int size, i, val, res;

	size = ctrlp->size;
	if (req == UVC_GET_INFO)
		size = 1;

	if (size > sizeof(buf)) {
		r5u_err(vhp, "Control ID %d is too large, %d bytes",
			ctrlp->reg, size);
		return -EINVAL;
	}

	memset(buf, 0, sizeof(buf));

	if (req != UVC_SET_CUR) {
		res = r5u870_uvc_req(vhp, req, ctrlp->reg, 0,
					 ctrlp->unit, vhp->vh_ctrl_ifnum,
					 buf, size);
		if (res < 0)
			return res;
		if (res != size) {
			r5u_err(vhp, "short read for UVC control %d",
				ctrlp->reg);
			return -EIO;
		}

		val = 0;
		switch (size) {
		case 1:
			val = buf[0];
			break;
		case 2:
			val = le16_to_cpu(*(__le16 *) buf);
			break;
		case 4:
			val = le32_to_cpu(*(__le32 *) buf);
			break;
		default:
			return -EINVAL;
		}

		*value = val;
		return 0;
	}

	val = *value;
	for (i = 0; i < size; i++, val >>= 8)
		buf[i] = val & 0xff;

	res = r5u870_uvc_req(vhp, UVC_SET_CUR, ctrlp->reg, 0,
				 ctrlp->unit, vhp->vh_ctrl_ifnum,
				 buf, size);
	if (res < 0)
		return res;
	if (res != size) {
		r5u_err(vhp, "short write for UVC control %d",
			ctrlp->reg);
		return -EIO;
	}

	return 0;
}

static int r5u870_set_ctrl_uvc(struct usbcam_dev *udp,
				   const struct usbcam_ctrl *basep,
				   const struct v4l2_ext_control *c)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	struct r5u870_ctrl *ctrlp = container_of(basep, struct r5u870_ctrl,
						base);
	int val;
	int res = 0;

	if (!(ctrlp->info & 2)) {
		r5u_dbg(vhp, R5U_CTRL, "control %s <= %d [set not supported]",
			ctrlp->base.uc_v4l.name, c->value);
		return -EINVAL;
	}

	r5u_dbg(vhp, R5U_CTRL, "control %s/uvc %02x <= %d",
		ctrlp->base.uc_v4l.name, ctrlp->reg, c->value);

	val = c->value;
	res = r5u870_uvc_ctrl_req(vhp, ctrlp, UVC_SET_CUR, &val);
	if (res)
		return res;

	ctrlp->value = c->value;

	return res;
}

/*
 * Initialize a r5u870_ctrl structure based on a UVC descriptor array,
 * and a bit within a supported control bitmap.
 */
static int r5u870_uvc_init_ctrl(struct r5u870_ctx *vhp,
				int unit,
				const struct r5u870_uvc_ctrlinfo *ci_array,
				int bit_offset)
{
	const struct r5u870_uvc_ctrlinfo *cip = NULL;
	struct r5u870_ctrl *ctrlp;
	int i, res, val;

	for (i = 0; ci_array[i].ci_name != NULL; i++) {
		if (ci_array[i].ci_bm_index == bit_offset) {
			cip = &ci_array[i];
			break;
		}
	}

	if (!cip)
		return -ENOENT;

	ctrlp = (struct r5u870_ctrl *) usbcam_ctrl_alloc(sizeof(*ctrlp));
	if (!ctrlp)
		return -ENOMEM;

	ctrlp->base.uc_v4l.id = cip->ci_v4l_id;
	ctrlp->base.uc_v4l.type = cip->ci_v4l_type;
	strlcpy(ctrlp->base.uc_v4l.name,
		cip->ci_name,
		sizeof(ctrlp->base.uc_v4l.name));
	ctrlp->base.uc_v4l.flags = cip->ci_v4l_flags;
	ctrlp->base.uc_v4l.step = 1;
	ctrlp->base.uc_menu_names = cip->ci_menu_names;
	ctrlp->base.get_fn = r5u870_get_ctrl;
	ctrlp->base.set_fn = r5u870_set_ctrl_uvc;
	ctrlp->base.query_fn = r5u870_query_ctrl;
	ctrlp->reg = cip->ci_reg;
	ctrlp->unit = unit;
	ctrlp->size = cip->ci_size;
	ctrlp->is_auto = 0;
	ctrlp->auto_offset = 0;

	res = r5u870_uvc_ctrl_req(vhp, ctrlp, UVC_GET_INFO, &val);
	if (res)
		goto failed;
	ctrlp->info = val;

	if (cip->ci_min_force) {
		ctrlp->base.uc_v4l.minimum = cip->ci_min;
	} else {
		res = r5u870_uvc_ctrl_req(vhp, ctrlp, UVC_GET_MIN, &val);
		if (res < 0)
			goto failed;
		ctrlp->base.uc_v4l.minimum = val;
	}
	if (cip->ci_max_force) {
		ctrlp->base.uc_v4l.maximum = cip->ci_max;
	} else {
		res = r5u870_uvc_ctrl_req(vhp, ctrlp, UVC_GET_MAX, &val);
		if (res < 0)
			goto failed;
		ctrlp->base.uc_v4l.maximum = val;
	}

	if (cip->ci_def_force) {
		ctrlp->base.uc_v4l.default_value = cip->ci_def;
	} else {
		res = r5u870_uvc_ctrl_req(vhp, ctrlp, UVC_GET_DEF, &val);
		if (res)
			goto failed;
			return res;
		ctrlp->base.uc_v4l.default_value = val;
	}

	res = usbcam_ctrl_add(vhp->vh_parent, &ctrlp->base);
	if (res)
		goto failed;

	r5u_dbg(vhp, R5U_INIT, "Found UVC control %s [%d,%d] def:%d info:%02x",
		ctrlp->base.uc_v4l.name, ctrlp->base.uc_v4l.minimum,
		ctrlp->base.uc_v4l.maximum, ctrlp->base.uc_v4l.default_value,
		ctrlp->info);
	return 0;

failed:
	usbcam_ctrl_free(ctrlp);
	return res;
}

static int r5u870_uvc_add_ctrls(struct r5u870_ctx *vhp, int unit,
				const struct r5u870_uvc_ctrlinfo *ci_array,
				const u8 *bits, int nbits)
{
	int i, res;

	for (i = 0; i < nbits; i++) {
		if (!test_bit(i, (const unsigned long *) bits))
			continue;
		res = r5u870_uvc_init_ctrl(vhp, unit, ci_array, i);
		if (res == -ENOENT) {
			r5u_dbg(vhp, R5U_INIT,
				"unit %d ctrl %d not recognized", unit, i);
		}
		else if (res)
			return res;
	}

	return 0;
}

static int r5u870_uvc_parse_vc(struct r5u870_ctx *vhp)
{
	struct usb_host_interface *aintf =
		vhp->vh_parent->ud_intf->cur_altsetting;
	u8 *desc;
	int dlen, rlen, count;
	int i, res;

	vhp->vh_ctrl_ifnum = aintf->desc.bInterfaceNumber;

	for (desc = aintf->extra, rlen = aintf->extralen;
		 rlen > 2;
		 rlen -= desc[0], desc += desc[0]) {

		dlen = desc[0];
		if (dlen < 2)
			return -EINVAL;
		if (desc[1] != USB_DT_CS_INTERFACE)
			continue;
		if (dlen < 3)
			return -EINVAL;

		switch (desc[2]) {
		case UVC_VC_HEADER:
			count = (dlen < 12) ? 0 : desc[11];
			if (dlen < (12 + count)) {
				r5u_err(vhp, "VC_HEADER too short at %d bytes",
					dlen);
				return -EINVAL;
			}

			for (i = 0; i < count; i++) {
				res = usbcam_claim_interface(vhp->vh_parent,
								 desc[12 + i]);
				if (res)
					r5u_err(vhp, "interface %d already "
						"claimed", desc[12 + i]);
				vhp->vh_iso_ifnum = desc[12 + i];

				res = r5u870_uvc_parse_vs(vhp, desc[12 + i]);
				if (res)
					return res;
			}
			break;

		case UVC_VC_PROCESSING_UNIT:
			count = (dlen < 8) ? 0 : desc[7];
			if (dlen < (8 + count)) {
				r5u_err(vhp, "VC_PROCESSING_UNIT too short "
					"at %d bytes", dlen);
				return -EINVAL;
			}

			res = r5u870_uvc_add_ctrls(vhp, desc[3],
						   r5u870_uvc_proc_ctrls,
						   &desc[8], desc[7] * 8);
			if (res)
				return res;
			break;
		}
	}

	return 0;
}


static void r5u870_do_stop(struct r5u870_ctx *vhp)
{
	if (vhp->vh_parent->ud_capturing) {
		vhp->vh_parent->ud_capturing = 0;
		vhp->vh_ctrl_reg_enable = 0;

		usbcam_urbstream_stop(&vhp->vh_iso, 0);
		usbcam_curframe_abortall(vhp->vh_parent);

		if (!vhp->vh_parent->ud_disconnected) {
			usb_set_interface(r5u870_dev(vhp),
					  vhp->vh_iso_ifnum,
					  0);
			vhp->vh_cap_stop(vhp);
		}

		usbcam_urbstream_cleanup(&vhp->vh_iso);
	}
}

/*
 * As long as we are requested to capture, we keep the iso stream running.
 * If we are explicitly requested to stop, we halt.
 * If we run out of frame buffers to capture into, we halt.
 */
static void r5u870_iso_packet_done(struct usbcam_dev *udp,
				   struct usbcam_urbstream *usp,
				   const uint8_t *pktdata,
				   int pktlen, int pktstatus)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	struct usbcam_curframe cf;
	int res, start = 0;

	res = vhp->vh_decide_pkt(vhp, pktstatus, pktlen,
				 (u8 *) pktdata, &start);
	switch (res) {
	case -EPIPE:
		if (vhp->vh_framebuf_offset) {
			vhp->vh_framebuf_offset = 0;
			usbcam_curframe_testpattern(udp);
			usbcam_curframe_complete(udp);
		}
		break;

	case 0:
	case -EAGAIN:
		if (pktlen) {
			if (usbcam_curframe_get(udp, &cf)) {
				/*
				 * We have data, but there is no frame buffer
				 * queued to accept it, so we stop.
				 */
				r5u870_do_stop(vhp);
				break;
			}

			BUG_ON(pktlen - start + vhp->vh_framebuf_offset >
				   cf.uf_size);

			/*
			 * This is our one and only memcpy.
			 * It's kind of hard to get around doing this, as
			 * we cannot predict into which isochronous
			 * packets the camera will choose to return data.
			 */
			memcpy(cf.uf_base + vhp->vh_framebuf_offset,
				   pktdata + start,
				   pktlen - start);
			vhp->vh_framebuf_offset += (pktlen - start);
		}

		if (!res) {
			vhp->vh_framebuf_offset = 0;
			usbcam_curframe_complete(udp);
		}
		break;

	case -EIO:
		r5u870_do_stop(vhp);
		break;

	default:
		BUG();
	}
}

static void r5u870_iso_submit_error(struct usbcam_dev *udp,
					struct usbcam_urbstream *usp, int status)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	r5u_dbg(vhp, R5U_FRAME, "iso submit error: %d", status);
	r5u870_do_stop(vhp);
}

static struct usbcam_urbstream_ops r5u870_urb_data_ops = {
	.packet_done	= r5u870_iso_packet_done,
	.submit_error	= r5u870_iso_submit_error,
};


static void r5u870_usbcam_release(struct usbcam_dev *udp)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	int i;

	for (i = 0; i < vhp->vh_npixfmts; i++) {
		if (vhp->vh_pixfmts[i].rp_restbl_alloc)
			kfree(vhp->vh_pixfmts[i].rp_restbl);
	}
}


static const struct r5u870_model *r5u870_find_model(int driver_info);
static int r5u870_usbcam_init(struct usbcam_dev *udp,
				  const struct usb_device_id *devid)
{
	struct r5u870_ctx *vhp;
	int model_info;
	int res;

	model_info = devid->driver_info;

	/* Initialize the private structure, don't use r5u_dbg() before this */
	vhp = udp_r5u870(udp);
	memset(vhp, 0, sizeof(*vhp));
	vhp->vh_parent = udp;
	vhp->vh_timeout = 1000;

	vhp->vh_ctrl_ifnum = -1;
	vhp->vh_iso_ifnum = -1;
	vhp->vh_iso_minpacket = -1;

	vhp->vh_model = r5u870_find_model(model_info);

	if (!vhp->vh_model) {
		r5u_err(vhp, "no suitable model descriptor for %04x:%04x",
			le16_to_cpu(r5u870_dev(vhp)->descriptor.idVendor),
			le16_to_cpu(r5u870_dev(vhp)->descriptor.idProduct));
		return -ENODEV;
	}

	vhp->vh_pixfmts = vhp->vh_model->rm_pixfmts;
	vhp->vh_npixfmts = vhp->vh_model->rm_npixfmts;

	if (vhp->vh_model->rm_uvc) {
		vhp->vh_set_fmt = r5u870_set_fmt_uvc;
		vhp->vh_cap_stop = r5u870_cap_stop_uvc;
		vhp->vh_decide_pkt = r5u870_decide_pkt_uvc;
	} else {
		vhp->vh_set_fmt = r5u870_set_fmt_wdm;
		vhp->vh_cap_stop = r5u870_cap_stop_wdm;
		vhp->vh_decide_pkt = r5u870_decide_pkt_wdm;
	}

	r5u_info(vhp, "Detected %s", vhp->vh_model->rm_name);
	snprintf(vhp->vh_parent->ud_vdev.name,
		 sizeof(vhp->vh_parent->ud_vdev.name),
		 "%s #%d",
		 vhp->vh_model->rm_name, vhp->vh_parent->ud_minidrv_id + 1);

	res = r5u870_dev_init(vhp);
	if (res < 0) {
		r5u_err(vhp, "initialization failed: %d", res);
		goto out_failed;
	}

	if (vhp->vh_model->rm_uvc) {
		/*
		 * This appears to be a UVC VideoControl interface.
		 * Claim all of the associated VideoStreaming interfaces
		 * and configure the required endpoints and unit IDs
		 */
		res = r5u870_uvc_parse_vc(vhp);
		if (res < 0) {
			r5u_err(vhp, "UVC setup failed: %d", res);
			goto out_failed;
		}

	} else {
		/* We are looking at a proprietary Ricoh interface */
		vhp->vh_iso_ifnum =
			udp->ud_intf->altsetting->desc.bInterfaceNumber;
		vhp->vh_iso_ep = 6;
	}

	if (vhp->vh_model->rm_wdm_ctrlids) {
		res = r5u870_wdm_add_ctrls(vhp, vhp->vh_model->rm_wdm_ctrlids);
		if (res < 0) {
			r5u_err(vhp, "Vendor control setup failed: %d", res);
			goto out_failed;
		}
	}

	/* Configure the usbcam pixel format and control arrays */
	if (!vhp->vh_npixfmts) {
		r5u_err(vhp, "No pixel formats detected");
		res = -ENODEV;
		goto out_failed;
	}

	udp->ud_fmt_array = &vhp->vh_pixfmts[0].base;
	udp->ud_fmt_array_elem_size = sizeof(vhp->vh_pixfmts[0]);
	udp->ud_fmt_array_len = vhp->vh_npixfmts;

	if (!vhp->vh_model->rm_no_first_auto_suppress)
		vhp->vh_ctrl_auto_suppress = 1;

	usbcam_urbstream_init(&vhp->vh_iso, udp, vhp->vh_iso_ep);

	/* Set the default format */
	vhp->vh_fmt = &vhp->vh_pixfmts[0];
	vhp->vh_res = &vhp->vh_fmt->rp_restbl[0];
	udp->ud_format.width = vhp->vh_res->rw_width;
	udp->ud_format.height = vhp->vh_res->rw_height;
	udp->ud_format.pixelformat = vhp->vh_fmt->base.pixelformat;
	udp->ud_format.field = V4L2_FIELD_INTERLACED;
	udp->ud_format.bytesperline = udp->ud_format.width * 2;
	udp->ud_format.sizeimage = (udp->ud_format.width *
					udp->ud_format.height * 2);
	udp->ud_format.colorspace = V4L2_COLORSPACE_SMPTE170M;

	/* Configure default values for all controls */
	res = r5u870_set_controls(vhp, 1);
	if (res < 0) {
		r5u_err(vhp, "set defaults failed: %d", res);
		goto out_failed;
	}

	return 0;

out_failed:
	r5u870_usbcam_release(udp);
	return res;
}

static void r5u870_usbcam_disconnect(struct usbcam_dev *udp)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	r5u870_do_stop(vhp);
}


/*
 * The power management stuff doesn't quite work yet, so we don't
 * yet set the supports_autosuspend field in the ops structure.
 */

static int r5u870_usbcam_suspend(struct usbcam_dev *udp, pm_message_t msg)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	r5u870_do_stop(vhp);
	return 0;
}

static int r5u870_usbcam_resume(struct usbcam_dev *udp)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	int res;

	res = r5u870_dev_init(vhp);
	if (res < 0) {
		r5u_err(vhp, "dev reinitialization failed: %d", res);
		return res;
	}

	vhp->vh_configured = 0;

	return 0;
}

static int r5u870_try_format(struct usbcam_dev *udp, struct v4l2_pix_format *f,
				 const struct r5u870_pix_fmt **fmt_out,
				 const struct r5u870_resolution **res_out)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	const struct r5u870_pix_fmt *fmt;
	const struct r5u870_resolution *res, *restbl;
	int i;

	fmt = NULL;
	for (i = 0; i < vhp->vh_npixfmts; i++) {
		if (vhp->vh_pixfmts[i].base.pixelformat == f->pixelformat) {
			fmt = &vhp->vh_pixfmts[i];
			break;
		}
	}
	if (fmt == NULL)
		return -EINVAL;

	restbl = fmt->rp_restbl;
	if (!restbl || !restbl[0].rw_width) {
		r5u_err(vhp, "invalid resolution table");
		return -EINVAL;
	}

	/* Find the most acceptable resolution */
	res = NULL;
	for (i = 0; restbl[i].rw_width > 0; i++) {
		if (!res) {
			res = &restbl[i];
		}
		else if (res->rw_width > f->width) {
			if (restbl[i].rw_width < res->rw_width)
				res = &restbl[i];
		}
		else if (res->rw_height > f->height) {
			if ((restbl[i].rw_width <= f->width) &&
				(restbl[i].rw_height < res->rw_height))
				res = &restbl[i];
		}
		else if ((restbl[i].rw_width <= f->width) &&
			 (restbl[i].rw_height <= f->height) &&
			 ((restbl[i].rw_width > res->rw_width) ||
			  ((restbl[i].rw_width == res->rw_width) &&
			   (restbl[i].rw_height > res->rw_height)))) {
			res = &restbl[i];
		}
	}

	if ((f->width > 1) && (f->height > 1)) {
		r5u_dbg(vhp, R5U_INIT, "pix_fmt width: %d height: %d", f->width, f->height);
		r5u_dbg(vhp, R5U_INIT, "res width: %d height %d", res->rw_width, res->rw_height);

		if (((res->rw_width > f->width) || (res->rw_height > f->height))) {
			r5u_dbg(vhp, R5U_INIT, "Bad size request. Returning -EINVAL.");
			return -EINVAL;
		}
	}

	memset(f, 0, sizeof(*f));
	f->width = res->rw_width;
	f->height = res->rw_height;
	f->pixelformat = fmt->base.pixelformat;
	f->bytesperline = f->width * 2;
	f->sizeimage = f->width * f->height * 2;
	f->field = V4L2_FIELD_INTERLACED;
	f->colorspace = V4L2_COLORSPACE_SMPTE170M;

	r5u_dbg(vhp, R5U_INIT, "Settled on pixel format: %d (%s)", fmt->base.pixelformat, fmt->base.description);

	if (fmt_out)
		(*fmt_out) = fmt;
	if (res_out)
		(*res_out) = res;

	return 0;
}

static int r5u870_usbcam_try_format(struct usbcam_dev *udp,
					struct v4l2_pix_format *f)
{
	return r5u870_try_format(udp, f, NULL, NULL);
}

static int r5u870_usbcam_set_format(struct usbcam_dev *udp,
					struct v4l2_pix_format *f)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	const struct r5u870_pix_fmt *fmt_out;
	const struct r5u870_resolution *res_out;
	int res;

	res = r5u870_try_format(udp, f, &fmt_out, &res_out);
	if (res)
		return res;

	if ((udp->ud_format.width != f->width) ||
		(udp->ud_format.height != f->height) ||
		(udp->ud_format.pixelformat != f->pixelformat) ||
		(udp->ud_format.sizeimage != f->sizeimage))
		vhp->vh_configured = 0;

	udp->ud_format = *f;
	vhp->vh_fmt = fmt_out;
	vhp->vh_res = res_out;
	return 0;
}

static void r5u870_usbcam_cap_stop(struct usbcam_dev *udp)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	r5u870_do_stop(vhp);
}

static int r5u870_config_iso_ep(struct r5u870_ctx *vhp)
{
	int res;

	if (!vhp->vh_configured) {
		vhp->vh_ctrl_sync = 0;

		res = usbcam_choose_altsetting(vhp->vh_parent,
						   vhp->vh_iso_ifnum,
						   usb_rcvisocpipe(r5u870_dev(vhp),
								   vhp->vh_iso_ep),
						   vhp->vh_res->rw_reqbw,
						   vhp->vh_iso_minpacket, -1,
						   &vhp->vh_act_altsetting);
		if (res) {
			r5u_err(vhp, "need %d B/s, no altsetting provides",
				vhp->vh_res->rw_reqbw);
			return res;
		}

		r5u_dbg(vhp, R5U_FRAME, "using altsetting %d",
			vhp->vh_act_altsetting);
	}

	res = usb_set_interface(r5u870_dev(vhp), vhp->vh_iso_ifnum,
				vhp->vh_act_altsetting);
	if (res) {
		r5u_err(vhp, "could not set altsetting: %d", res);
		return res;
	}

	return 0;
}

static int r5u870_usbcam_cap_start(struct usbcam_dev *udp)
{
	struct r5u870_ctx *vhp = udp_r5u870(udp);
	int res;

	if (vhp->vh_parent->ud_capturing)
		return 0;

	if (!vhp->vh_model->rm_uvc) {
		res = r5u870_config_iso_ep(vhp);
		if (res)
			return res;
	}

	vhp->vh_ctrl_reg_enable = 1;

	if (!vhp->vh_configured) {
		r5u_dbg(vhp, R5U_MDINTF, "setting initial control values");
		res = r5u870_set_controls(vhp, 0);
		if (res)
			goto out_set_idle;

		res = vhp->vh_set_fmt(vhp, vhp->vh_fmt, vhp->vh_res);
		if (res) {
			r5u_err(vhp, "could not configure capture: %d", res);
			goto out_set_idle;
		}

		if (vhp->vh_model->rm_no_ctrl_reload)
			vhp->vh_ctrl_sync = 1;
	}

	if (vhp->vh_model->rm_uvc) {
		res = r5u870_config_iso_ep(vhp);
		if (res)	
			goto out_set_idle;
	}

	vhp->vh_configured = 1;

	res = usbcam_urbstream_config_iso(&vhp->vh_iso,
					  &r5u870_urb_data_ops,
					  0, 0, 1, 0);
	if (res < 0) {
		r5u_err(vhp, "urbstream init failed: %d", res);
		goto out_set_idle;
	}

	udp->ud_capturing = 1;

	if (!vhp->vh_model->rm_uvc) {
		res = r5u870_set_cap_state_wdm(vhp, 1);
		if (res)
			goto out_cleanup_urbstream;
	}

	if (!vhp->vh_ctrl_sync) {
		r5u_dbg(vhp, R5U_MDINTF, "reloading control values");

		/* Reload the control values after changing res/format */
		res = r5u870_set_controls(vhp, 0);
		if (res) {
			r5u_err(vhp, "could not load control values: %d", res);
			goto out_stop_capture;
		}	

		vhp->vh_ctrl_sync = 1;
	}

	if (res)
		goto out_failed;

	r5u_dbg(vhp, R5U_MDINTF, "starting capture");

	vhp->vh_firstframe = 0;
	vhp->vh_frame_accum = -1;
	vhp->vh_framebuf_offset = 0;
	vhp->vh_emptypkts = R5U870_EMPTYPKT_FRAME_DELIM - 1;

	res = usbcam_urbstream_start(&vhp->vh_iso);
	if (res)
		goto out_stop_capture;

	return 0;

out_stop_capture:
	(void) vhp->vh_cap_stop(vhp);
out_cleanup_urbstream:
	usbcam_urbstream_cleanup(&vhp->vh_iso);
out_set_idle:
	(void) usb_set_interface(r5u870_dev(vhp), vhp->vh_iso_ifnum, 0);
out_failed:
	vhp->vh_ctrl_reg_enable = 0;
	vhp->vh_parent->ud_capturing = 0;
	return res;
}

static struct usbcam_dev_ops r5u870_usbcam_dev_ops = {
	.init		= r5u870_usbcam_init,
	.disconnect	= r5u870_usbcam_disconnect,
	.release	= r5u870_usbcam_release,
	.suspend	= r5u870_usbcam_suspend,
	.resume		= r5u870_usbcam_resume,	
	.try_format	= r5u870_usbcam_try_format,
	.set_format	= r5u870_usbcam_set_format,
	.cap_start	= r5u870_usbcam_cap_start,
	.cap_stop	= r5u870_usbcam_cap_stop,

	/* .supports_autosuspend = 1, */
};


/*
 * Per-device hard coded vendor control lists follow
 */

static const int r5u870_1830_ctrls[] = {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_SHARPNESS,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_GAMMA,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_500,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1832_ctrls[] = {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_500_DEF1,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1833_ctrls[] = {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_SHARPNESS,
	R5U870_WDM_CTRL_GAMMA,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_500_DEF1,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1834_ctrls[] = {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_SHARPNESS,
	R5U870_WDM_CTRL_GAMMA,
	R5U870_WDM_CTRL_BACKLIGHT_COMP_X1834,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_AUTO_EXPOSURE,
	R5U870_WDM_CTRL_EXPOSURE,
	R5U870_WDM_CTRL_AUTO_GAIN,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1870_ctrls[] = {
	R5U870_WDM_CTRL_BRIGHTNESS,
	R5U870_WDM_CTRL_CONTRAST,
	R5U870_WDM_CTRL_HUE,
	R5U870_WDM_CTRL_SATURATION,
	R5U870_WDM_CTRL_SHARPNESS,
	R5U870_WDM_CTRL_GAMMA,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_AUTO_EXPOSURE,
	R5U870_WDM_CTRL_EXPOSURE,
	R5U870_WDM_CTRL_AUTO_GAIN,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_POWERLINE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};

/*
 * Even the UVC models do not express all of their controls in the UVC
 * descriptor tables, and get sets of hard-coded vendor controls
 */

// FIXME: This device still has a bunch of unknown control IDs.
static const int r5u870_1812_ctrls[] = {
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1835_ctrls[] = {
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_1836_ctrls[] = {
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_AUTO_EXPOSURE,
	R5U870_WDM_CTRL_EXPOSURE,
	R5U870_WDM_CTRL_AUTO_GAIN,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_1837_ctrls[] = {
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_AUTO_EXPOSURE,
	R5U870_WDM_CTRL_EXPOSURE,
	R5U870_WDM_CTRL_AUTO_GAIN,
	R5U870_WDM_CTRL_GAIN,
	R5U870_WDM_CTRL_VFLIP_DEFAULTON,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_183a_ctrls[] = {
 	R5U870_WDM_CTRL_WB_RED, 
 	R5U870_WDM_CTRL_WB_GREEN,
 	R5U870_WDM_CTRL_WB_BLUE, 
 	R5U870_WDM_CTRL_WB_AUTO, 
 	R5U870_WDM_CTRL_VFLIP, 
 	R5U870_WDM_CTRL_HFLIP, 
 	R5U870_WDM_CTRL_PRIVACY,
 	R5U870_WDM_CTRL_NIGHTMODE, 
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_183b_ctrls[] = {
	/* TODO: Maybe there are more of these? I don't actually have a webcam
	   to test against the different WDM controls. */
 	R5U870_WDM_CTRL_WB_RED, 
 	R5U870_WDM_CTRL_WB_GREEN,
 	R5U870_WDM_CTRL_WB_BLUE, 
 	R5U870_WDM_CTRL_WB_AUTO, 
 	R5U870_WDM_CTRL_VFLIP, 
 	R5U870_WDM_CTRL_HFLIP, 
 	R5U870_WDM_CTRL_PRIVACY,
 	R5U870_WDM_CTRL_NIGHTMODE, 
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_183e_ctrls[] = {
	/* TODO: Maybe there are more of these? I don't actually have a webcam
	   to test against the different WDM controls. */
 	R5U870_WDM_CTRL_WB_RED,
 	R5U870_WDM_CTRL_WB_GREEN,
 	R5U870_WDM_CTRL_WB_BLUE,
 	R5U870_WDM_CTRL_WB_AUTO,
 	R5U870_WDM_CTRL_VFLIP,
 	R5U870_WDM_CTRL_HFLIP,
 	R5U870_WDM_CTRL_PRIVACY,
 	R5U870_WDM_CTRL_NIGHTMODE,
	R5U870_WDM_CTRL_LAST,
};
static const int r5u870_1810_1839_ctrls[] = {
	/* TODO: Maybe there are more of these? I don't actually have a webcam
   	   to test against the different WDM controls. */
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_LAST,
};

static const int r5u870_1841_ctrls[] = {
	/* TODO: Maybe there are more of these? I don't actually have a webcam
	   to test against the different WDM controls. */
	R5U870_WDM_CTRL_WB_RED,
	R5U870_WDM_CTRL_WB_GREEN,
	R5U870_WDM_CTRL_WB_BLUE,
	R5U870_WDM_CTRL_WB_AUTO,
	R5U870_WDM_CTRL_VFLIP,
	R5U870_WDM_CTRL_HFLIP,
	R5U870_WDM_CTRL_PRIVACY,
	R5U870_WDM_CTRL_LAST,
};


/*
 * Standard resolution table for non-UVC cameras,
 * as UVC camera report back as to what resolutions
 * they support.
 * The Sony VGP-VCC2 Windows driver supports:
 *	160x120, 176x144, 320x240, 352x288, 640x480
 * The HP driver also supports 1280x1024
 */
static const struct r5u870_resolution r5u870_vga_wdm_res[] = {
	{  160,  120,  1152000 },
	{  176,  144,  1520640 },
	{  320,  240,  4608000 },
	{  352,  288,  6082560 },
	{  640,  480, 18432000 },
	{ }
};
static const struct r5u870_resolution r5u870_sxga_wdm_res[] = {
	{  160,  120,  1152000 },
	{  176,  144,  1520640 },
	{  320,  240,  4608000 },
	{  352,  288,  6082560 },
	{  640,  480, 18432000 },
	{ 1280, 1024, 19660800 },
	{ }
};
static struct r5u870_pix_fmt r5u870_vga_wdm_pixfmts[] = {
	{ .base = { .description = "YUY2 4:2:2",
			.pixelformat = V4L2_PIX_FMT_YUYV,
			.flags = 0 },
	  .rp_formatidx = 0,
	  .rp_restbl = r5u870_vga_wdm_res },

	{ .base = { .description = "UYVY 4:2:2",
			.pixelformat = V4L2_PIX_FMT_UYVY,
			.flags = 0 },
	  .rp_formatidx = 1,
	  .rp_restbl = r5u870_vga_wdm_res },
};
static struct r5u870_pix_fmt r5u870_sxga_wdm_pixfmts[] = {
	{ .base = { .description = "YUY2 4:2:2",
			.pixelformat = V4L2_PIX_FMT_YUYV,
			.flags = 0 },
	  .rp_formatidx = 0,
	  .rp_restbl = r5u870_sxga_wdm_res },

	{ .base = { .description = "UYVY 4:2:2",
			.pixelformat = V4L2_PIX_FMT_UYVY,
			.flags = 0 },
	  .rp_formatidx = 1,
	  .rp_restbl = r5u870_sxga_wdm_res },
};

enum {
	R5U870_DI_INVALID,
	R5U870_DI_VGP_VCC2_SZ,
	R5U870_DI_VGP_VCC3,
	R5U870_DI_VGP_VCC2_AR1,
	R5U870_DI_VGP_VCC2_AR2,
	R5U870_DI_VGP_VCC5,
	R5U870_DI_VGP_VCC4,
	R5U870_DI_VGP_VCC4_VFLIP,
	R5U870_DI_VGP_VCC6,
	R5U870_DI_VGP_VCC7,
	R5U870_DI_VGP_VCC8,
	R5U870_DI_VGP_VCC9,
	R5U870_DI_HP_WEBCAM1K,
	R5U870_DI_HP_PAVWC_WDM,
	R5U870_DI_HP_PAVWC_UVC,
	R5U870_DI_HP_PAVWC_UVC_NOFW,
	R5U870_DI_GENERIC_UVC,
	R5U870_DI_FUJITSU,
};

static const struct r5u870_model r5u870_models[] = {
	[R5U870_DI_VGP_VCC2_SZ] = {
		.rm_name = "Sony VGP-VCC2 (VAIO SZ)",
		.rm_ucode_file = "r5u870_1830.fw",
		.rm_ucode_version = 0x0100,
		.rm_wdm_ctrlids = r5u870_1830_ctrls,
		.rm_pixfmts = r5u870_vga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_vga_wdm_pixfmts),
	},
	[R5U870_DI_VGP_VCC3] = {
		.rm_name = "Sony VGP-VCC3",
		.rm_ucode_file = "r5u870_1832.fw",
		.rm_ucode_version = 0x0100,
		.rm_wdm_ctrlids = r5u870_1832_ctrls,
		.rm_pixfmts = r5u870_vga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_vga_wdm_pixfmts),
		.rm_no_ctrl_reload = 1,
	},
	[R5U870_DI_VGP_VCC2_AR1] = {
		.rm_name = "Sony VGP-VCC2 (VAIO AR1)",
		.rm_ucode_file = "r5u870_1833.fw",
		.rm_ucode_version = 0x0100,
		.rm_wdm_ctrlids = r5u870_1833_ctrls,
		.rm_pixfmts = r5u870_vga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_vga_wdm_pixfmts),
	},
	[R5U870_DI_VGP_VCC2_AR2] = {
		.rm_name = "Sony VGP-VCC2 (VAIO AR)",
		.rm_ucode_file = "r5u870_1834.fw",
		.rm_ucode_version = 0x0111,
		.rm_wdm_ctrlids = r5u870_1834_ctrls,
		.rm_pixfmts = r5u870_vga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_vga_wdm_pixfmts),
	},
	[R5U870_DI_VGP_VCC5] = {
		.rm_name = "Sony VGP-VCC5",
		.rm_ucode_file = "r5u870_1835.fw",
		.rm_ucode_version = 0x0107,
		.rm_wdm_ctrlids = r5u870_1835_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC4] = {
		.rm_name = "Sony VGP-VCC4",
		.rm_ucode_file = "r5u870_1836.fw",
		.rm_ucode_version = 0x0115,
		.rm_wdm_ctrlids = r5u870_1810_1836_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC4_VFLIP] = {
		.rm_name = "Sony VGP-VCC4",
		.rm_ucode_file = "r5u870_1836.fw",
		.rm_ucode_version = 0x0115,
		.rm_wdm_ctrlids = r5u870_1810_1837_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC6] = {
		.rm_name = "Sony VGP-VCC6",
		.rm_ucode_file = "r5u870_1839.fw",
		.rm_ucode_version = 0x0113,
		.rm_wdm_ctrlids = r5u870_1810_1839_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC7] = {
		.rm_name = "Sony VGP-VCC7",
		.rm_ucode_file = "r5u870_183a.fw",
		.rm_ucode_version = 0x0111,
		.rm_wdm_ctrlids = r5u870_1810_183a_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC8] = {
		.rm_name = "Sony VGP-VCC8",
		.rm_ucode_file = "r5u870_183b.fw",
		.rm_ucode_version = 0x0131,
		.rm_wdm_ctrlids = r5u870_1810_183b_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_VGP_VCC9] = {
		.rm_name = "Sony VGP-VCC9",
		.rm_ucode_file = "r5u870_183e.fw",
		.rm_ucode_version = 0x0100,
		.rm_wdm_ctrlids = r5u870_1810_183e_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_FUJITSU] = {
		.rm_name = "Fujitsu F01",
		.rm_ucode_file = "r5u870_1841.fw",
		.rm_ucode_version = 0x0103,
		.rm_wdm_ctrlids = r5u870_1841_ctrls,
		.rm_uvc = 1,
	},
	[R5U870_DI_HP_WEBCAM1K] = {
		.rm_name = "HP Webcam 1000",
		.rm_ucode_file = "r5u870_1870_1.fw",
		.rm_ucode_version = 0x0100,
		.rm_wdm_ctrlids = r5u870_1870_ctrls,
		.rm_pixfmts = r5u870_vga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_vga_wdm_pixfmts),
	},
	[R5U870_DI_HP_PAVWC_WDM] = {
		.rm_name = "HP Pavilion Webcam (WDM)",
		.rm_ucode_file = "r5u870_1870.fw",
		.rm_ucode_version = 0x0112,
		.rm_wdm_ctrlids = r5u870_1870_ctrls,
		.rm_pixfmts = r5u870_sxga_wdm_pixfmts,
		.rm_npixfmts = ARRAY_SIZE(r5u870_sxga_wdm_pixfmts),
		.rm_no_ctrl_reload = 1,
	},
	[R5U870_DI_HP_PAVWC_UVC] = {
		.rm_name = "HP Pavilion Webcam (UVC)",
		.rm_ucode_file = "r5u870_1810.fw",
		.rm_ucode_version = 0x0115,
		.rm_wdm_ctrlids = r5u870_1810_1836_ctrls,
		.rm_uvc = 1,
		.rm_no_ctrl_reload = 1,
	},
	[R5U870_DI_HP_PAVWC_UVC_NOFW] = {
		.rm_name = "HP Pavilion Webcam (UVC - NO FW)",
		.rm_wdm_ctrlids = r5u870_1812_ctrls,
		.rm_uvc = 1,
		.rm_no_ctrl_reload = 1,
	},
	[R5U870_DI_GENERIC_UVC] = {
		.rm_name = "Generic UVC Webcam",
		.rm_uvc = 1,
	},
};

/*
 * Someone clever at HP decided to use 05ca:1870 for two distinct devices.
 * The Pavilion dv1xxx machines all seem to have the less common of the
 * two.  There is no known, working method to distinguish the devices
 * using USB commands only.  We resort to reading the model number out
 * of DMI.
 */
static int dv1000 = 2;
module_param(dv1000, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(dv1000, "HP dv1000 detect mode (0=no,1=yes,2=DMI)");

static int r5u870_check_hp_dv1000(void)
{
	const char *prod_name;
	if (!dv1000)
		return 0;
	if (dv1000 == 1)
		return 1;
	prod_name = dmi_get_system_info(DMI_PRODUCT_NAME);
	if (!prod_name)
		printk(KERN_INFO "r5u870: No DMI model found\n");
	else {
		printk(KERN_INFO "r5u870: Found DMI model: \"%s\"\n",
			   prod_name);
		if (!strncmp(prod_name, "HP Pavilion dv1000", 18) &&
			!isdigit(prod_name[18]))
			return 1;
	}
	return 0;
}

static const struct r5u870_model *r5u870_find_model(int driver_info)
{
	if (driver_info == R5U870_DI_HP_PAVWC_WDM) {
		if (r5u870_check_hp_dv1000())
			driver_info = R5U870_DI_HP_WEBCAM1K;
	}
	if ((driver_info <= R5U870_DI_INVALID) ||
		(driver_info >= ARRAY_SIZE(r5u870_models)))
		return NULL;
	if (!r5u870_models[driver_info].rm_name)
		return NULL;
	return &r5u870_models[driver_info];
}


#define R5U870_DEVICE_UVC(VID, PID, DINFO)				\
	.match_flags		= USB_DEVICE_ID_MATCH_DEVICE		\
				| USB_DEVICE_ID_MATCH_INT_INFO,		\
	.idVendor		= (VID),				\
	.idProduct		= (PID),				\
	.bInterfaceClass	= USB_CLASS_VIDEO,			\
	.bInterfaceSubClass	= 1,					\
	.bInterfaceProtocol	= 0,					\
	.driver_info		= (DINFO)

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x05CA, 0x1830), .driver_info = R5U870_DI_VGP_VCC2_SZ },
	{ USB_DEVICE(0x05CA, 0x1832), .driver_info = R5U870_DI_VGP_VCC3 },
	{ USB_DEVICE(0x05CA, 0x1833), .driver_info = R5U870_DI_VGP_VCC2_AR1 },
	{ USB_DEVICE(0x05CA, 0x1834), .driver_info = R5U870_DI_VGP_VCC2_AR2 },
	{ USB_DEVICE(0x05CA, 0x1870), .driver_info = R5U870_DI_HP_PAVWC_WDM },

	{ R5U870_DEVICE_UVC(0x05CA, 0x1810, R5U870_DI_HP_PAVWC_UVC) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x1812, R5U870_DI_HP_PAVWC_UVC_NOFW) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x1835, R5U870_DI_VGP_VCC5) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x1836, R5U870_DI_VGP_VCC4) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x1837, R5U870_DI_VGP_VCC4_VFLIP) },
	/* 0x1838 does not appear to have ever been released */
	{ R5U870_DEVICE_UVC(0x05CA, 0x1839, R5U870_DI_VGP_VCC6) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x183a, R5U870_DI_VGP_VCC7) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x183b, R5U870_DI_VGP_VCC8) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x183e, R5U870_DI_VGP_VCC9) },
	{ R5U870_DEVICE_UVC(0x05CA, 0x1841, R5U870_DI_FUJITSU) },
	{ },
};


DEFINE_USBCAM_MINIDRV_MODULE(R5U870_VERSION, R5U870_VERSION_EXTRA,
				 &r5u870_usbcam_dev_ops,
				 sizeof(struct r5u870_ctx),
				 id_table)

MODULE_DEVICE_TABLE(usb, id_table);
MODULE_DESCRIPTION("Driver for Ricoh R5U870-based Webcams");
MODULE_AUTHOR("Sam Revitch <samr7@cs.washington.edu>");
MODULE_LICENSE("GPL");
