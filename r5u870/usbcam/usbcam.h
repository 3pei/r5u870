/*
 * USBCAM abstraction library for USB webcam drivers
 * Version 0.11.1
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
 * This library is intended to ease the process of creating drivers
 * for simpler USB webcam devices.  It handles most V4L interactions, and
 * all USB driver entry points.  It provides a minidriver callback API
 * for handling most common tasks.
 */

#ifndef __USBCAM_H__
#define	__USBCAM_H__

#ifdef __KERNEL__
#include<linux/init.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/version.h>
/* 3pei */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
#include <linux/videodev.h>
#else
#include "libv4l1-videodev.h"
#endif
#include <media/v4l2-common.h>

/* The actual per-minidriver structure is opaque */
typedef struct usbcam_minidrv usbcam_minidrv_t;
struct usbcam_dev;
struct usbcam_curframe;


/*
 * Log file and debug infrastructure
 */
#define usbcam_info(UDP, FMT, ARG...) do {				\
	printk(KERN_INFO "%s: " FMT "\n",				\
	       (UDP)->ud_dev_name, ## ARG);				\
} while (0)
#define usbcam_warn(UDP, FMT, ARG...) do {				\
	printk(KERN_WARNING "%s: " FMT "\n",				\
	       (UDP)->ud_dev_name, ## ARG);				\
} while (0)
#define usbcam_err(UDP, FMT, ARG...) do {				\
	printk(KERN_ERR "%s: " FMT "\n",				\
	       (UDP)->ud_dev_name, ## ARG);				\
} while (0)


#if defined(CONFIG_USB_USBCAM_DEBUG)
#define usbcam_dbg(UDP, SUBSYS, FMT, ARG...) do {			\
	if ((UDP)->ud_debug)                            \
		printk(KERN_DEBUG "%s: " FMT "\n",			\
		       (UDP)->ud_dev_name, ## ARG);			\
} while (0)
#define usbcam_assert(expr)						\
do {									\
	if (!(expr)) {							\
		printk(KERN_ERR "%s:%d: assertion \"" # expr "\" "	\
		       "failed", __FILE__, __LINE__);			\
		dump_stack();						\
	}								\
} while (0)
extern void usbcam_hexdump(struct usbcam_dev *udp, const u8 *buf, size_t len);

#else
#define usbcam_dbg(UDP, SUBSYS, FMT, ARG...)
#define usbcam_assert(expr)
#define usbcam_hexdump(udp, buf, len)
#endif

/*
 * Debug subsystem bit values
 *
 * The usbcam_dev.ud_debug debug subsystem mask is a pointer to a
 * per-minidriver integer, which is usually a module parameter and can
 * be manipulated via sysfs.
 */
enum {
	USBCAM_DBG_VIDEOBUF = 0,
	USBCAM_DBG_CAPTURE,
	USBCAM_DBG_IOCTL_BUF,
	USBCAM_DBG_IOCTL_FMT,
	USBCAM_DBG_IOCTL_MISC,
	USBCAM_DBG_DEV_STATE,
	USBCAM_DBG_ALTSETTING,
	USBCAM_DBG_URBSTREAM,

	/* First bit available to minidrivers */
	USBCAM_DBGBIT_MD_START = 8,
};



/*
 * The usbcam_pix_fmt structure is used to describe a pixel format natively
 * supported by the driver to V4L clients.  A minidriver may set the
 * ud_fmt_array member of usbcam_dev to point to an array of these
 * structures, and the array will be used to service the ENUMFMT ioctl,
 * as long as the minidriver does not intercept that ioctl.
 */
struct usbcam_pix_fmt {
	char description[32];
	unsigned int pixelformat;
	unsigned int flags;
};


/*
 * usbcam_dev: The per-device structure representing:
 *	(1) A USB device/interface
 *	(2) A registered V4L device
 *
 * Commented fields are of interest to minidrivers.
 * Uncommented fields should be considered opaque.
 */

struct usbcam_dev {
	/*
	 * ud_vdev is the video4linux device structure.
	 * The minidriver may be interested in a few fields:
	 *	ud_vdev.name: The device name displayed by applications
	 */
	struct video_device		ud_vdev;

	/*
	 * ud_dev, ud_intf: The associated USB device/primary interface
	 * These members are read-only to all except when set during
	 * usbcam_dev structure initialization.
	 */
	struct usb_device		*ud_dev;
	struct usb_interface		*ud_intf;

	usbcam_minidrv_t		*ud_minidrv;

	/*
	 * ud_minidrv_data: Minidriver private data
	 * During structure initialization, if a minidriver structure
	 * size was specified to usbcam_register(), that many bytes of
	 * memory are allocated, the allocation is assigned here, and
	 * the original allocation is automatically freed with the
	 * usbcam_dev structure.  Otherwise this member is initialized
	 * to NULL.
	 * The minidriver may set whatever policies it wants with how
	 * this field is managed.
	 */
	void				*ud_minidrv_data;

	struct list_head		ud_drv_links;

	/*
	 * ud_minidrv_id: The device's unique number among all devices
	 * belonging to the minidriver.  Set prior to the minidriver's
	 * init handler being called.  Read-only at all other times.
	 */
	int				ud_minidrv_id;

	/*
	 * ud_lock: the mutex protecting most of this structure and all
	 * minidriver entry points.
	 */
	struct mutex			ud_lock;

	/*
	 * ud_format: Currently configured size/format
	 *	Protected by ud_lock
	 *	Modified only by minidriver
	 *	Examined by ioctl handler and framebuffer allocator
	 */
	struct v4l2_pix_format		ud_format;

	/*
	 * ud_fmt_array, ud_fmt_array_elem_size, ud_fmt_array_len:
	 * Supported pixel formats for enumeration
	 *	Protected by ud_lock
	 *	Modified only by minidriver, usually set by init callout
	 *	Examined by default ioctl handler for ENUMFMT
	 */
	const struct usbcam_pix_fmt	*ud_fmt_array;
	int				ud_fmt_array_elem_size;
	int				ud_fmt_array_len;

	struct list_head		ud_ctrl_list;

	/*
	 * ud_capturing: Minidriver capture-in-progress flag
	 *	Protected by ud_lock
	 *	Modified only by minidriver, e.g. cap_start, cap_stop
	 *	Examined by framebuffer interface and S_FMT ioctl handler
	 */
	unsigned int			ud_capturing : 1,

	/*
	 * ud_disconnected: Set if the underlying USB device has been
	 * disconnected, just prior to invoking the minidriver disconnect
	 * callout, if one is provided.  This field should be considered
	 * read-only to minidrivers.
	 */
					ud_disconnected : 1,

					ud_initializing : 1,
					ud_suspended : 1,
					ud_disconnected_primary : 1,
					ud_videodev_released : 1;

	struct kref			ud_kref;
	struct list_head		ud_interface_list;

	struct list_head		ud_frame_cap_queue;

	int				*ud_debug;

	int				ud_work_refs;
	spinlock_t			ud_work_lock;
	int				ud_work_lockwait;
	struct task_struct		*ud_work_thread;
	struct list_head		ud_work_queue;

	struct mutex			ud_open_lock;
	int				ud_user_refs;
	unsigned int			ud_autopm_ref : 1;
	struct file			*ud_excl_owner;

        // EMS add a lock to use (otherwise BUG occurs during videobuf init)
        spinlock_t                      slock;

	/*
	 * ud_dev_name: Name of device as used for worker thread names and
	 * debug messages.  The minidriver may set this field in its init
	 * callout, but should consider it read-only after that point.
	 */
	char				ud_dev_name[32];
};


/*
 * Per-device reference counting helpers
 *
 * When the last reference is released, the minidriver's release
 * callout will be invoked.
 *
 * usbcam_get() may be called from any context.
 * usbcam_put() can sleep, and may not be called while holding the device
 * lock (see below).
 */
#define usbcam_get(UDP) kref_get(&(UDP)->ud_kref)
extern void usbcam_put(struct usbcam_dev *udp);


/*
 * Per-Device locking helpers
 *
 * The minidriver callouts, which are described below in usbcam_dev_ops
 * and usbcam_urbstream_ops, are all invoked with the device lock held.
 * Also, all usbcam work queue callouts are invoked with the device lock
 * held.
 *
 * Minidrivers that must be entered through paths other than those
 * described above may need to examine or update data structures
 * protected by the device lock, and may do so using the lock
 * acquire/release macros.  For example, minidrivers that use
 * procfs/sysfs file operations, or completion callouts unsuitable for
 * the usbcam work queue will need this.
 *
 * Minidrivers may release and reacquire the lock inside of certain
 * usbcam minidriver callouts (see 
 */
#define usbcam_lock(UDP) mutex_lock(&(UDP)->ud_lock)
#define usbcam_unlock(UDP) mutex_unlock(&(UDP)->ud_lock)
#define usbcam_chklock(UDP) usbcam_assert(mutex_is_locked(&(UDP)->ud_lock))


/*
 * MINIDRIVER CALLOUTS
 *
 * Callouts invoked at various stages of the lifetime of a usbcam_dev
 * device.
 *
 * REQUIRED: init, cap_start, cap_stop.
 *
 * All other callouts are optional.
 *
 * By default, all callouts are invoked with the device lock held.
 *
 * The device lock is not intended to be held for long periods of time.
 * Some operations may run for a long time, and may need to sleep
 * waiting for an operation to complete on the device.  If these
 * operations need to be able to run at the same time as capture, the
 * minidriver must ensure that it does not hold the device lock while
 * waiting for such long operations to complete.
 *
 * To support operating without the device lock, there are flags
 * in the ops structure to selectively disable this behavior for the
 * ioctl callout, control callouts, and power management callouts.
 * The minidriver will be responsible for acquiring and releasing the
 * device lock, and re-verifying the device state upon reacquisition.
 *
 * Certain callouts are explicitly restricted from releasing and
 * reacquiring the device lock.  These include:
 *
 * disconnect, open, close, try_format, set_format, cap_start, cap_stop,
 * testpattern
 * 
 */
struct usbcam_dev_ops {
	int	unlocked_ioctl : 1,
		unlocked_ctrl : 1,
		unlocked_pm : 1,
		supports_autosuspend : 1,
		no_workref_on_open : 1;

	/* init: invoked when a matching USB device is discovered */
	int (*init)(struct usbcam_dev *udp, const struct usb_device_id *);

	/* suspend/resume: invoked from power management paths */
	int (*suspend)(struct usbcam_dev *udp, pm_message_t message);
	int (*resume)(struct usbcam_dev *udp);

	/* disconnect: invoked when a device has been disconnected */
	void (*disconnect)(struct usbcam_dev *udp);

	/* release: invoked when a usbcam_dev is about to be freed */
	void (*release)(struct usbcam_dev *udp);

	/* open: invoked on first user open of the device */
	int (*open)(struct usbcam_dev *udp);

	/* close: invoked on last user close of the device */
	void (*close)(struct usbcam_dev *udp);

	/* try_format: invoked to negotiate capture format */
	int (*try_format)(struct usbcam_dev *udp,
			  struct v4l2_pix_format *fmt_in_out);

	/* set_format: invoked when the capture format is being set */
	int (*set_format)(struct usbcam_dev *udp,
			  struct v4l2_pix_format *fmt_in_out);

	/* ioctl: ioctl call interception for most commands */
	int (*ioctl)(struct usbcam_dev *udp, int cmd, void *arg);

	/* cap_start: invoked when capture should be initiated */
	int (*cap_start)(struct usbcam_dev *udp);

	/* cap_stop: invoked when capture should be halted */
	void (*cap_stop)(struct usbcam_dev *udp);
};


/*
 * Minidrivers may register themselves using usbcam_register_mod(),
 * although the recommended method is to use the usbcam_register()
 * macro instead.
 *
 * The driver should keep a usbcam_minidrv structure pointer as a
 * handle to the usbcam registration, as it will need it later when it
 * needs to unregister itself.  usbcam_register_mod() accepts a pointer
 * to this structure, and fills it in on success.
 *
 * The minidriver must report a primary version number in KERNEL_VERSION
 * format, which is used by the default VIDIOC_QUERYCAP ioctl handler.
 * The minidriver may optionally report an extra version string.
 *
 * usbcam_register_mod() will cause usbcam to register a new USB driver
 * with the given device ID table, so that it may be called when
 * matching devices are discovered.
 *
 * When a matching device is detected, usbcam will:
 *	-> Allocate a usbcam_dev structure, including a minidriver-specific
 *	   portion of a given size, attached to ud_minidrv_data
 *	-> Invoke the ->init() callout for the minidriver
 *	-> If successful, register a new video4linux device and
 *	   start accepting V4L API requests on it
 *
 * usbcam_register() depends on variables defined by the
 * DEFINE_USBCAM_MODPARAMS macro to function correctly.  This defines
 * a set of static variables and marks them as module parameters.  These
 * include:
 *	-> video_nr: Favored video4linux minor numbers
 *	-> debug: A pointer to the debug level variable
 */

extern int usbcam_register_mod(usbcam_minidrv_t **driverpp,
			       int mdrv_version, const char *mdrv_verx,
			       const struct usbcam_dev_ops *ops,
			       const int dev_priv_size,
			       const struct usb_device_id *id_table,
			       const int *video_nrs, int video_nrs_len,
			       int *debug, struct module *md,
			       const char *modname);

/*
 * usbcam_unregister() will remove a minidriver registration with the
 * USB subsystem, and will prepare the minidriver structure to be
 * freed.  It is safe to call this API in paths other than minidriver
 * module unload, as long as the minidriver is registered.
 */
extern void usbcam_unregister(usbcam_minidrv_t *driverp);

#define DEFINE_USBCAM_MODPARAMS_BASE					\
	static int video_nr_len = 0;					\
	static int video_nr[8];						\
	module_param_array(video_nr, int, &video_nr_len, S_IRUGO|S_IWUSR);    \
	MODULE_PARM_DESC(video_nr, "=n[,n...] Force /dev/videoX IDs");

#if defined(CONFIG_USB_USBCAM_DEBUG)
#ifndef USBCAM_DEBUG_DEFAULT
#define USBCAM_DEBUG_DEFAULT 0x0020
#endif
#define DEFINE_USBCAM_MODPARAMS 					\
	DEFINE_USBCAM_MODPARAMS_BASE					\
	static int debug = USBCAM_DEBUG_DEFAULT;			\
	module_param(debug, int, S_IRUGO|S_IWUSR);		      	\
	MODULE_PARM_DESC(debug, "Enable debug trace messages");
#define usbcam_register(DRVPP, MDV, VERX, CB, STRUCTSIZE, IDTBL)	\
	usbcam_register_mod((DRVPP), (MDV), (VERX), (CB), (STRUCTSIZE),	\
			    (IDTBL), video_nr, video_nr_len, &debug,	\
			    THIS_MODULE, KBUILD_MODNAME)
#else
#define DEFINE_USBCAM_MODPARAMS 					\
	DEFINE_USBCAM_MODPARAMS_BASE
#define usbcam_register(DRVPP, MDV, VERX, CB, STRUCTSIZE, IDTBL)	\
	usbcam_register_mod((DRVPP), (MDV), (VERX), (CB), (STRUCTSIZE),	\
			    (IDTBL), video_nr, video_nr_len, NULL,	\
			    THIS_MODULE, KBUILD_MODNAME)
#endif


/*
 * For minidrivers that do not need to do anything other than register
 * and unregister themselves as usbcam minidrivers when their modules
 * are loaded and unloaded, the DEFINE_USBCAM_MINIDRV_MODULE macro
 * is offered.
 */
#define DEFINE_USBCAM_MINIDRV_MODULE(VER, VERX, OPS, STRUCTSIZE, IDTBL)	\
	DEFINE_USBCAM_MODPARAMS						\
	static usbcam_minidrv_t *usbcam_minidrv_driver;			\
	static int __init usbcam_minidrv_init(void)			\
	{								\
		return usbcam_register(&usbcam_minidrv_driver,		\
				       (VER), (VERX), (OPS),		\
				       (STRUCTSIZE), (IDTBL));		\
	}								\
	static void __exit usbcam_minidrv_exit(void)			\
	{								\
		usbcam_unregister(usbcam_minidrv_driver);		\
	}								\
	module_init(usbcam_minidrv_init);				\
	module_exit(usbcam_minidrv_exit);


/*
 * Function for claiming additional interfaces
 *
 * This may only be called from the minidriver init callout
 *
 * For devices with multiple interfaces that pertain to a single driver,
 * a separate usbcam_dev would ordinarily be created for each interface,
 * and the init callout would be invoked.  With this function,
 * additional pertinent interfaces can be bound to the base usbcam_dev.
 *
 * Additional claimed interfaces will be automatically released at
 * disconnect time, or in the event of a failure from the init callout.
 *
 * If the subject interface is already claimed by another USB driver,
 * this function will fail with -EBUSY.
 */
extern int usbcam_claim_interface(struct usbcam_dev *udp, int ifnum);


/*
 * Alternate setting choosing helper function
 *
 * This function assists in choosing an alternate setting based on
 * overall bandwidth and packet size requirements for a given pipe.
 */
extern int usbcam_choose_altsetting(struct usbcam_dev *udp, int ifnum,
				    int pipe, int bytes_per_sec_min,
				    int pkt_min, int pkt_max,
				    int *altsetting_nr);


/*
 * CONTROL MANAGEMENT
 *
 * The control structure is interpreted by the usbcam ioctl handler to
 * answer QUERYCTRL/G_CTRL/S_CTRL/QUERYMENU requests.  A minidriver may
 * define template controls, and add instances of them per device using
 * usbcam_ctrl_add().  Template controls may include additional fields,
 * as long as struct usbcam_ctrl is the first member.
 *
 * This mode of handling control-related ioctls is only used if the
 * minidriver does not intercept and handle the appropriate ioctl
 * commands in its ioctl callout.
 */
struct usbcam_ctrl {
	struct list_head	uc_links;
	struct v4l2_queryctrl	uc_v4l;

	const char 		**uc_menu_names;

	/* Retrieves details about the control dynamically, may be NULL */
	int (*query_fn)(struct usbcam_dev *, const struct usbcam_ctrl *,
			struct v4l2_queryctrl *);

	/* Retrieves the current value of the control */
	int (*get_fn)(struct usbcam_dev *, const struct usbcam_ctrl *,
		      struct v4l2_ext_control *);

	/* If set_fn is not set, the control is considered read-only. */
	int (*set_fn)(struct usbcam_dev *, const struct usbcam_ctrl *,
		      const struct v4l2_ext_control *);

	/* Called when a control is being freed, not normally needed */
	void (*release_fn)(struct usbcam_dev *, struct usbcam_ctrl *);
};

extern struct usbcam_ctrl *usbcam_ctrl_find(struct usbcam_dev *udp, u32 ctlid);

/* Recommended API: Add a control based on a template */
extern struct usbcam_ctrl *usbcam_ctrl_add_tmpl(struct usbcam_dev *udp,
						const struct usbcam_ctrl *tplp,
						size_t real_size);

/* Less recommended APIs: allocate, configure, try to add, free on failure */
extern struct usbcam_ctrl *usbcam_ctrl_alloc(size_t real_size);
#define usbcam_ctrl_free(CTRLP) kfree(CTRLP)
extern int usbcam_ctrl_add(struct usbcam_dev *udp,
			   struct usbcam_ctrl *ctrlp);


/*
 * CURRENT FRAME BUFFER ACCESS
 *
 * Minidrivers need to get the address to which the current frame
 * buffer is mapped, and the allocated size of the frame buffer in
 * order to do their job.  The below functions facilitate that.
 *
 * usbcam_curframe_complete() will cause the current frame buffer to
 * be returned to the client application with a successful status.
 * The next queued frame buffer will become the current frame buffer.
 *
 * usbcam_curframe_abortall() will cause all pending frame buffers
 * to be aborted and returned to the client application with -EIO.
 */
struct usbcam_curframe {
	u8			*uf_base;
	size_t			uf_size;
	enum v4l2_field		uf_field;
	struct timeval		uf_timestamp;
};

extern int usbcam_curframe_get(struct usbcam_dev *udp,
			       struct usbcam_curframe *cf);
extern void usbcam_curframe_complete_detail(struct usbcam_dev *udp,
					    struct usbcam_curframe *cf);
#define usbcam_curframe_complete(UDP) \
	usbcam_curframe_complete_detail(UDP, NULL)

extern void usbcam_curframe_abortall(struct usbcam_dev *udp);

/*
 * FRAME FILLERS AND TEST PATTERNS
 *
 * usbcam_curframe_fill() is a glorified memset: it fills the current
 * frame buffer, starting at offs, with nrecs instances of the repeating
 * byte sequence pattern/patlen.
 *
 * It does range checking and will issue printk warnings if the frame
 * buffer or the ud_format.sizeimage is overshot.
 */
extern void usbcam_curframe_fill(struct usbcam_dev *udp, size_t offset,
				 const void *pattern, int patlen, int nrecs);

/*
 * usbcam_curframe_testpattern() attempts to fill the current frame
 * buffer with a blue test pattern.  It paves over any partial frame
 * data.  If you wish to display part of an incomplete or interrupted
 * frame, don't use this function.  It is used by default to fill in
 * frames that are completed with is_error = 1.
 *
 * This function understands many formats, but there will always be
 * exceptions.  To keep the testpattern mechanism consistent in this
 * situation, this function will always defer to the 'testpattern'
 * function in the minidriver's usbcam_dev_ops, if one is given,
 * before attempting to fill the frame itself.
 *
 * This function is aware of bytesperline and padded line formats.
 */
extern int usbcam_curframe_testpattern(struct usbcam_dev *udp);


/*
 * WORK ITEMS AND THE PER-DEVICE WORKER THREAD
 *
 * So long as a usbcam video4linux device is open, there is a worker
 * thread available to execute deferred minidriver tasks.  The worker
 * thread becomes available prior to the open callout issued to the
 * minidriver, and the last close of the device will block after the
 * minidriver close callout finishes, waiting for the work queue to
 * drain.
 *
 * A work item may be queued from any context, including interrupt
 * contexts and URB completion callouts.
 *
 * Work item callouts are invoked in the context of the worker thread
 * with the device mutex held.  When a work item is queued, when the
 * worker thread is next idle, it will wait for the device mutex, and
 * will start the work item once it acquires the device mutex.  Until
 * that point, the work item may be canceled and dequeued using the
 * usbcam_work_cancel() API, which may only be called while holding
 * the device mutex.
 *
 * It was decided not to use the core kernel work queue
 * implementation for the usbcam work queue because:
 * 1. Work item cancelation is seen as a useful feature
 * 2. Multithreading is not seen as a useful feature
 */

struct usbcam_workitem;
typedef void (*usbcam_workfunc_t)(struct usbcam_workitem *work);

struct usbcam_workitem {
	struct list_head	uw_links;
	struct usbcam_dev	*uw_dev;
	usbcam_workfunc_t	uw_func;
};

extern void usbcam_work_init(struct usbcam_dev *udp,
			     struct usbcam_workitem *wip,
			     usbcam_workfunc_t func);
extern int usbcam_work_queue(struct usbcam_workitem *wip);
extern int usbcam_work_cancel(struct usbcam_workitem *wip);

struct usbcam_delayedwork {
	struct usbcam_workitem	dw_work;
	struct timer_list	dw_timer;
};

extern void usbcam_delayedwork_init(struct usbcam_dev *udp,
				    struct usbcam_delayedwork *dwp,
				    usbcam_workfunc_t func);
extern void usbcam_delayedwork_queue(struct usbcam_delayedwork *dwp,
				     unsigned int timeout_ms);
extern int usbcam_delayedwork_cancel(struct usbcam_delayedwork *dwp);


/*
 * usbcam_work_ref() and usbcam_work_unref() are for managing the start
 * and stop of the worker thread.
 */
extern int usbcam_work_ref(struct usbcam_dev *udp);
extern void usbcam_work_unref(struct usbcam_dev *udp);

/*
 * usbcam_work_runqueue() will run work queue items in the current context
 * until the work queue becomes empty.
 */
extern void usbcam_work_runqueue(struct usbcam_dev *udp);


/*
 * Streaming URB helper structure
 *
 * Minidrivers create any number of these, assign appropriate endpoints,
 * and invoke start.  Completed packets are automatically processed in the
 * worker thread.
 *
 * This infrastructure may only be used when a device is opened, as the
 * worker thread is shut down at other times.
 */

extern struct urb *usbcam_urb_alloc(struct usbcam_dev *udp, int pipe,
				    int pktsize, int npkts, int alloc_buf);
extern void usbcam_urb_free(struct urb *urbp, int free_buf);


struct usbcam_urbstream;

struct usbcam_urbstream_ops {
	void (*packet_done)(struct usbcam_dev *, struct usbcam_urbstream *,
			    const u8 *pktdata, int pktlen, int pktstatus);
	void (*submit_error)(struct usbcam_dev *, struct usbcam_urbstream *,
			     int status);
};

struct usbcam_urbstream {
	struct usbcam_dev			*us_dev;
	char					us_endpoint;
	char					us_streaming;
	char					us_active_goal;
	char					us_active_count;
	const struct usbcam_urbstream_ops	*us_ops;
	spinlock_t				us_lock;
	int					us_timeout_ticks;
	int					us_resubmit_err;
	struct list_head			us_unused_list;
	struct list_head			us_active_list;
	struct list_head			us_complete_list;
	struct completion			us_active_empty;
	struct usbcam_workitem			us_error_workitem;
};

extern void usbcam_urbstream_init(struct usbcam_urbstream *usp,
				  struct usbcam_dev *udp, int ep);

extern int usbcam_urbstream_start(struct usbcam_urbstream *);
extern void usbcam_urbstream_stop(struct usbcam_urbstream *, int wait);
extern void usbcam_urbstream_cleanup(struct usbcam_urbstream *usp);
extern int usbcam_urbstream_submit_one(struct usbcam_urbstream *usp);

extern int usbcam_urbstream_config_iso(struct usbcam_urbstream *usp,
				       const struct usbcam_urbstream_ops *ops,
				       int pktcount, int nreqs, int interval,
				       int pktlen);

extern int usbcam_urbstream_config_bulk(struct usbcam_urbstream *usp,
					const struct usbcam_urbstream_ops *ops,
					int nreqs, int reqlen, int maxpkt,
					int timeout_ms);


/*
 * Backward compatibility crap for slightly older 2.6 series kernels
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#if !defined(V4L2_CTRL_FLAG_NEXT_CTRL)
#define V4L2_CTRL_FLAG_NEXT_CTRL 0
#endif
#if !defined(V4L2_CTRL_FLAG_SLIDER)
#define V4L2_CTRL_FLAG_SLIDER 0
#endif
#if !defined(V4L2_CTRL_FLAG_INACTIVE)
#define V4L2_CTRL_FLAG_INACTIVE 0
#endif
#if !defined(V4L2_CTRL_FLAG_UPDATE)
#define V4L2_CTRL_FLAG_UPDATE 0
#endif
#endif  /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) */
#endif /* __KERNEL__ */


#endif /* __USBCAM_H__ */
