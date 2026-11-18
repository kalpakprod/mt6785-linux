// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 GNSS bridge over the CONNSYS STP transport.
 *
 * Copyright (C) 2026 The poco-m5s port
 *
 * The MT6631 GNSS engine is a task on the CONNSYS MCU, so GNSS data arrives
 * the same way Bluetooth does: as STP frames, here of type 2 (GPS_TASK_INDX).
 * Unlike Bluetooth there is no upstream protocol stack to plug into - the
 * vendor userspace daemon talks to a character device - so this driver offers
 * the same two devices the vendor driver does:
 *
 *   /dev/stpgps   the STP type-2 channel. open() switches the GNSS function on
 *                 through WMT, release() switches it off; read()/write() carry
 *                 raw NMEA and PMTK. The vendor ioctl numbers are kept as they
 *                 are, because the daemon issues them by number.
 *   /dev/gps_emi  a placeholder for the EMI window the vendor driver exposes.
 *                 On MT6785 the GNSS engine reaches its memory through the
 *                 CONNSYS carve-out, so the file only has to open and mmap.
 *
 * The ioctl contract is copied from the vendor driver
 * (connectivity/gps/gps_stp/stp_chrdev_gps.c): the ones the daemon actually
 * calls are 7 (IC HW version), 8 (IC FW version), 16 (GPS LNA pin) and 20
 * (block until the channel closes); the rest are answered the way the vendor
 * code answers them.
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/sprintf.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "osal_typedef.h"
#include "wmt_exp.h"
#include "stp_exp.h"

#define STPGPS_DEV_NAME		"stpgps"
#define GPS_EMI_DEV_NAME	"gps_emi"
#define GPS_PWR_DEV_NAME	"gps_pwr"

/* Vendor ioctl numbers, unchanged: the daemon sends the numbers, not names. */
#define COMBO_IOC_GPS_HWVER		6
#define COMBO_IOC_GPS_IC_HW_VERSION	7
#define COMBO_IOC_GPS_IC_FW_VERSION	8
#define COMBO_IOC_D1_EFUSE_GET		9
#define COMBO_IOC_RTC_FLAG		10
#define COMBO_IOC_CO_CLOCK_FLAG		11
#define COMBO_IOC_TRIGGER_WMT_ASSERT	12
#define COMBO_IOC_WMT_STATUS		13
#define COMBO_IOC_TAKE_GPS_WAKELOCK	14
#define COMBO_IOC_GIVE_GPS_WAKELOCK	15
#define COMBO_IOC_GET_GPS_LNA_PIN	16
#define COMBO_IOC_GPS_FWCTL		17
#define COMBO_IOC_GPS_HW_SUSPEND	18
#define COMBO_IOC_GPS_HW_RESUME		19
#define COMBO_IOC_GPS_LISTEN_RST_EVT	20
#define COMBO_IOC_GPS_GET_MD_STATUS	21

/*
 * The vendor EMI stub hands out a window inside the CONNSYS carve-out, not a
 * separately allocated buffer: `gGpsEmiPhyBase = gConEmiPhyBase +
 * GPS_EMI_BASE_ADDR_OFFSET` (gps_emi.c, mtk_wcn_consys_gps_memory_reserve()).
 * It must be real memory the chip can reach, so it is mapped out of the
 * carveout rather than allocated here.
 *
 * What the daemon does with that window is narrower than it looks: on this SoC
 * the GNSS image and the boot commands reach the chip over the STP channel
 * (measured on the same SoC: 39721 B out before the first NMEA byte, and the
 * begonia port got that far with a plain vmalloc stub), while the window
 * carries the daemon's own data exchanges. The window is still given its real
 * place in the carveout, because the vendor's ioctl 4 hands its physical base
 * to the daemon and the chip may be asked to fetch from it.
 *
 * The offset for MT6785 is not in the vendor file - gps_emi.c has no
 * CONFIG_MACH_MT6785 branch at all, only MT6765/61/68 (2 MiB + 1.5 MiB +
 * 0x1000), MT6779/6873/6853 (3 MiB + 0x10000), MT6771/75/58 (1 MiB) and
 * MT6873. So the window is anchored on what this port's own carveout says:
 * the conn EMI region is 4 MiB (consys_reserved, DT size 0x400000) and the
 * WMT patch images already live in its first 1.5 MiB
 * (soc1_0_ram_mcu @0x0, soc1_0_ram_bt @0x80000, soc1_0_ram_wifi @0x140000,
 * the last one 0x30444 long).
 *
 * Placing GPS after the Wi-Fi image - at 0x1A0000, 1.5 MiB in - keeps it
 * inside the 4 MiB carveout with 2.5 MiB to spare and clear of every offset
 * the WMT patch download uses on this SoC, which is the same relative
 * position the MT6765 family uses (2 MiB + 1.5 MiB). The window is 1.5 MiB,
 * the size the vendor allocates for the bigger SoCs, and it is checked
 * against the real carveout at probe so a wrong offset fails loudly instead
 * of handing the daemon memory the chip cannot reach.
 */
#define GPS_EMI_BASE_ADDR_OFFSET	(0x1A0000)
#define GPS_EMI_SIZE			(SZ_1M + SZ_1M / 2)

/* One STP frame holds one NMEA sentence or one PMTK command. */
#define STPGPS_FRAME_SIZE	1024

/*
 * How much of the first frames of each direction reaches the log. The daemon's
 * boot exchange is short and its framing is undocumented: a session that dies
 * after one 116-byte frame and a 20-byte answer cannot be explained from byte
 * counters alone, so the first frames of each direction are printed raw, in
 * full - 168 bytes covers the largest GPS STP frame the vendor's daemon sends.
 */
#define STPGPS_DUMP_FRAMES	4
#define STPGPS_DUMP_BYTES	168

/*
 * The vendor driver tracks the chip's reset from the WMT notification
 * (gps_cdev_rst_cb) and reports it through COMBO_IOC_WMT_STATUS as -888 while
 * the chip is resetting and -889 once it came back, so the daemon knows it has
 * to start a new session instead of waiting for answers that will never come
 * (stp_chrdev_gps.c, GPS_unlocked_ioctl case COMBO_IOC_WMT_STATUS).
 */
#define STPGPS_RST_NONE		0
#define STPGPS_RST_START	1
#define STPGPS_RST_DONE		2

struct stpgps {
	struct miscdevice gps_misc;
	struct miscdevice emi_misc;
	struct miscdevice pwr_misc;

	/* Guards opened/rx_list: read() and the RX path both take it. */
	struct mutex lock;
	wait_queue_head_t read_wait;
	wait_queue_head_t close_wait;

	bool opened;
	bool gps_on;

	/*
	 * Set by remove(), read under the same wait as the queue: a reader must
	 * wake up and see end of file when the module goes away, and it must not
	 * depend on kthread_should_stop() to do it - that predicate belongs to
	 * the thread which called kthread_run() and dereferences a NULL in every
	 * other task (the oops lead note 23 opens with).
	 */
	bool shutting_down;

	/* RX queue: one STP frame per entry. */
	struct list_head rx_list;

	/* Chip reset, as reported by WMT: STPGPS_RST_*. */
	unsigned int rst_state;

	/*
	 * What the channel carried in this session. The daemon uploads the GNSS DSP
	 * image and its command stream before the chip answers anything, so these
	 * numbers are the evidence that the upload happened at all: on the same SoC
	 * the working session measured 39721 B out before the first NMEA byte in.
	 */
	u64 tx_bytes;
	u64 rx_bytes;
	unsigned int tx_frames;
	unsigned int rx_frames;

	/* GPS wake lock, COMBO_IOC_TAKE/GIVE_GPS_WAKELOCK and open/close. */
	struct wakeup_source *ws;
	bool ws_held;

	/* Frames already dumped this session, per direction. */
	unsigned int tx_dumped;
	unsigned int rx_dumped;

	/*
	 * STP hands frames over in its own RX thread, which must not sleep in a
	 * callback: the callback only schedules this work, and the work moves
	 * the frames into rx_list and wakes the readers. Draining here and not in
	 * read() is also what makes poll() honest - a frame waiting in STP's queue
	 * is invisible to a reader that asked whether the device is readable.
	 */
	struct work_struct rx_work;

	void *emi_buf;
	phys_addr_t emi_phys;
	size_t emi_size;
};

static struct stpgps *stpgps_dev;

/* ------------------------------------------------------------- STP bridge */

static void stpgps_rx_cb(void)
{
	struct stpgps *gps = READ_ONCE(stpgps_dev);

	if (gps)
		schedule_work(&gps->rx_work);
}

/* The GPS wake lock, held while the daemon runs a session. */
static void stpgps_hold_wake_lock(struct stpgps *gps, bool hold)
{
	if (hold && !gps->ws_held) {
		__pm_stay_awake(gps->ws);
		gps->ws_held = true;
	} else if (!hold && gps->ws_held) {
		__pm_relax(gps->ws);
		gps->ws_held = false;
	}
}

/*
 * WMT tells us when the conn MCU resets the GPS task, and the vendor daemon
 * watches for exactly this: after a reset neither PMTK nor NMEA will arrive on
 * the old session, so it has to close and open again. Without this the daemon
 * waited forever.
 */
static void stpgps_wmt_cb(ENUM_WMTDRV_TYPE_T src, ENUM_WMTDRV_TYPE_T dst,
			  ENUM_WMTMSG_TYPE_T type, void *buf, unsigned int sz)
{
	struct stpgps *gps = READ_ONCE(stpgps_dev);
	ENUM_WMTRSTMSG_TYPE_T msg;

	if (!gps || !buf || sz < sizeof(msg))
		return;
	memcpy(&msg, buf, sizeof(msg));

	if (src != WMTDRV_TYPE_WMT || dst != WMTDRV_TYPE_GPS ||
	    type != WMTMSG_TYPE_RESET)
		return;

	switch (msg) {
	case WMTRSTMSG_RESET_START:
		gps->rst_state = STPGPS_RST_START;
		break;
	case WMTRSTMSG_RESET_END:
	case WMTRSTMSG_RESET_END_FAIL:
		gps->rst_state = STPGPS_RST_DONE;
		break;
	default:
		return;
	}

	dev_info(gps->gps_misc.this_device, "conn MCU reported a GPS task reset (%s)\n",
		 gps->rst_state == STPGPS_RST_START ? "start" : "end");

	/* Wake the daemon's status poll and the ioctl that waits for a reset. */
	wake_up_interruptible(&gps->read_wait);
	wake_up_interruptible(&gps->close_wait);
}

static void stpgps_dump_frame(struct stpgps *gps, bool tx, const u8 *buf,
			      size_t len);

/* Takes the frames STP holds for the GNSS task and puts them on our queue. */
static int stpgps_drain(struct stpgps *gps)
{
	int total = 0;

	while (!mtk_wcn_stp_is_rxqueue_empty(GPS_TASK_INDX)) {
		struct sk_buff *skb;
		int cnt;

		skb = alloc_skb(STPGPS_FRAME_SIZE, GFP_KERNEL);
		if (!skb)
			break;

		cnt = mtk_wcn_stp_receive_data(skb->data, STPGPS_FRAME_SIZE,
					       GPS_TASK_INDX);
		if (cnt <= 0) {
			kfree_skb(skb);
			break;
		}

		skb_put(skb, cnt);
		stpgps_dump_frame(gps, false, skb->data, cnt);
		mutex_lock(&gps->lock);
		list_add_tail(&skb->list, &gps->rx_list);
		gps->rx_bytes += cnt;
		gps->rx_frames++;
		mutex_unlock(&gps->lock);
		total += cnt;
	}

	return total;
}

/*
 * One line per frame, for the first frames of a session: hex, because that is
 * what the vendor's framing is read from. `dir` is "tx" or "rx".
 */
static void stpgps_dump_frame(struct stpgps *gps, bool tx, const u8 *buf,
			      size_t len)
{
	unsigned int *count = tx ? &gps->tx_dumped : &gps->rx_dumped;
	const char *dir = tx ? "tx" : "rx";
	char hex[3 * STPGPS_DUMP_BYTES + 1];
	size_t i, n = min(len, (size_t)STPGPS_DUMP_BYTES);

	if (*count >= STPGPS_DUMP_FRAMES)
		return;
	(*count)++;

	for (i = 0; i < n; i++)
		snprintf(hex + 3 * i, 4, "%02x ", buf[i]);
	hex[3 * n] = '\0';

	if (n < len)
		dev_info(gps->gps_misc.this_device,
			 "GNSS %s frame %zu B (first %zu): %s...\n", dir, len, n, hex);
	else
		dev_info(gps->gps_misc.this_device,
			 "GNSS %s frame %zu B: %s\n", dir, len, hex);
}

static void stpgps_rx_work(struct work_struct *work)
{
	struct stpgps *gps = container_of(work, struct stpgps, rx_work);

	stpgps_drain(gps);

	if (!list_empty_careful(&gps->rx_list) || READ_ONCE(gps->shutting_down))
		wake_up_interruptible(&gps->read_wait);
}

/* One frame for the reader, or NULL when the queue is empty. */
static struct sk_buff *stpgps_take_frame(struct stpgps *gps)
{
	struct sk_buff *skb;

	mutex_lock(&gps->lock);
	skb = list_first_entry_or_null(&gps->rx_list, struct sk_buff, list);
	if (skb)
		list_del(&skb->list);
	mutex_unlock(&gps->lock);

	return skb;
}

/* ------------------------------------------------------------ char device */

static int stpgps_open(struct inode *inode, struct file *file)
{
	struct stpgps *gps = container_of(file->private_data, struct stpgps,
					  gps_misc);

	mutex_lock(&gps->lock);
	if (gps->opened) {
		mutex_unlock(&gps->lock);
		return -EBUSY;
	}
	gps->opened = true;
	mutex_unlock(&gps->lock);

	/*
	 * FUNC_CTRL(GPS on) returns MTK_WCN_BOOL: TRUE (1) is success, FALSE (0)
	 * is failure. The errno-style comparison this replaces treated a
	 * successful power-on as an error.
	 */
	if (mtk_wcn_wmt_func_on(WMTDRV_TYPE_GPS) != MTK_WCN_BOOL_TRUE) {
		gps->opened = false;
		return -EIO;
	}

	gps->gps_on = true;
	gps->rst_state = STPGPS_RST_NONE;
	gps->tx_dumped = 0;
	gps->rx_dumped = 0;
	file->private_data = gps;

	/* A reset of the GPS task ends the session: the daemon has to know. */
	mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_GPS, stpgps_wmt_cb);
	stpgps_hold_wake_lock(gps, true);

	/*
	 * The vendor's GPS_open() refuses to proceed unless the STP transport is
	 * ready: "STP is not ready, Cannot open GPS Devices" -> -ENODEV. Ours
	 * skipped that, so a daemon could open the device on a WMT that never
	 * finished coming up and then read nothing forever, with no error to
	 * show for it. Closing again restores the function state.
	 */
	if (!mtk_wcn_stp_is_ready()) {
		mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_GPS, NULL);
		stpgps_hold_wake_lock(gps, false);
		mtk_wcn_wmt_func_off(WMTDRV_TYPE_GPS);
		gps->gps_on = false;
		gps->opened = false;
		file->private_data = NULL;
		return -ENODEV;
	}

	return 0;
}

static int stpgps_release(struct inode *inode, struct file *file)
{
	struct stpgps *gps = file->private_data;

	if (!gps)
		return 0;

	if (gps->gps_on) {
		mtk_wcn_wmt_func_off(WMTDRV_TYPE_GPS);
		gps->gps_on = false;
	}
	mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_GPS, NULL);
	stpgps_hold_wake_lock(gps, false);

	/*
	 * The session's traffic, in one line, because it is the only honest
	 * measurement of whether the host uploaded anything: a silent channel
	 * reports no traffic at all, a working one tens of kilobytes out before the
	 * first NMEA byte (39721 B measured on the same SoC by the begonia port).
	 * A quiet channel stays at dev_dbg, so that a reader which reopens on every
	 * silence timeout does not fill the log.
	 */
	if (gps->tx_bytes || gps->rx_bytes)
		dev_info(gps->gps_misc.this_device,
			 "GNSS channel closed: tx %llu B in %u frames, rx %llu B in %u frames\n",
			 gps->tx_bytes, gps->tx_frames, gps->rx_bytes, gps->rx_frames);
	else
		dev_dbg(gps->gps_misc.this_device,
			"GNSS channel closed with no traffic\n");

	mutex_lock(&gps->lock);
	gps->opened = false;
	gps->tx_bytes = 0;
	gps->rx_bytes = 0;
	gps->tx_frames = 0;
	gps->rx_frames = 0;
	mutex_unlock(&gps->lock);

	/*
	 * Wake a reader blocked in the ioctl that waits for the close, and any
	 * process sitting in read(): the channel is going away.
	 */
	wake_up_interruptible(&gps->close_wait);
	wake_up_interruptible(&gps->read_wait);

	return 0;
}

/*
 * read() hands the daemon one STP frame at a time, the way the vendor's
 * GPS_read() does: block until the RX path has something, return it, and give
 * -EAGAIN instead of blocking when the daemon asked for non-blocking.
 *
 * What it must never do is decide whether to sleep on kthread_should_stop():
 * that predicate only means anything inside the thread kthread_run() created
 * (it reads current->worker_private), so calling it from the reading process
 * dereferenced NULL and took the machine through an oops - "Unable to handle
 * kernel NULL pointer dereference at virtual address 0, pc kthread_should_stop"
 * on a plain user-space read of /dev/stpgps (lead note 23). End of file comes
 * from the module going away instead, and only from there.
 */
static ssize_t stpgps_read(struct file *file, char __user *buf, size_t count,
			   loff_t *ppos)
{
	struct stpgps *gps = file->private_data;
	struct sk_buff *skb;
	size_t len;
	int err;

	if (!gps->gps_on)
		return -ENODEV;

	if (!count)
		return 0;

	for (;;) {
		skb = stpgps_take_frame(gps);
		if (skb) {
			len = min(count, skb->len);
			if (copy_to_user(buf, skb->data, len))
				len = -EFAULT;
			kfree_skb(skb);
			return len;
		}

		if (READ_ONCE(gps->shutting_down))
			return 0;

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		err = wait_event_interruptible(gps->read_wait,
					       READ_ONCE(gps->shutting_down) ||
					       !list_empty(&gps->rx_list));
		if (err)
			return err;
	}
}

static ssize_t stpgps_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct stpgps *gps = file->private_data;
	u8 *kbuf;
	ssize_t err;

	if (!gps->gps_on)
		return -ENODEV;

	if (count > STPGPS_FRAME_SIZE)
		return -EMSGSIZE;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	err = mtk_wcn_stp_send_data(kbuf, count, GPS_TASK_INDX);
	if (err > 0)
		stpgps_dump_frame(gps, true, kbuf, err);
	kfree(kbuf);

	if (err < 0)
		return err;

	/*
	 * Return what STP actually took, not what was asked for: the daemon uploads
	 * the DSP image in 512-byte frames and has to know when the window was full
	 * instead of believing the whole frame went out. The vendor's GPS_write()
	 * answers -ENOSPC for exactly that case, and 0 is the only value here that
	 * means "nothing accepted".
	 */
	if (err == 0) {
		dev_dbg(gps->gps_misc.this_device,
			"TX window full, %zu bytes refused\n", count);
		return -ENOSPC;
	}

	mutex_lock(&gps->lock);
	gps->tx_bytes += err;
	gps->tx_frames++;
	mutex_unlock(&gps->lock);

	return err;
}

static __poll_t stpgps_poll(struct file *file, poll_table *wait)
{
	struct stpgps *gps = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &gps->read_wait, wait);

	mutex_lock(&gps->lock);
	if (!list_empty(&gps->rx_list))
		mask |= EPOLLIN | EPOLLRDNORM;
	mutex_unlock(&gps->lock);

	if (READ_ONCE(gps->shutting_down))
		mask |= EPOLLHUP;

	return mask;
}

/*
 * COMBO_IOC_GPS_FWCTL - the daemon's synchronous request/response channel to
 * the GPS task, and the path the vendor driver uses to push the GNSS patch:
 * GPS_fwctl() in stp_chrdev_gps.c copies the request out of the daemon's
 * struct, calls mtk_wmt_gps_mcu_ctrl() and copies the answer back. That call is
 * a WMT op (WMT_OPID_GPS_MCU_CTRL), which wraps the payload in the fixed
 * command "01 32 00 00" and takes the GPS channel synchronously - not the
 * asynchronous type-2 data path write() uses.
 *
 * Answering -EFAULT here (as this driver did up to h9) means the daemon has no
 * way to talk to the task it is booting, whatever else the driver gets right.
 */
struct stpgps_fwctl_data {
	u32 size;
	u32 tx_len;
	u32 rx_max;
	u32 pad;
	u64 p_tx_buf;
	u64 p_rx_buf;
	u64 p_rx_len;
	u64 p_reserved;
};

#define STPGPS_FWCTL_BUF_MAX	128

static long stpgps_fwctl(struct stpgps *gps, unsigned long arg)
{
	struct stpgps_fwctl_data ctl;
	u8 tx_buf[STPGPS_FWCTL_BUF_MAX];
	u8 rx_buf[STPGPS_FWCTL_BUF_MAX];
	u32 rx_len = 0;
	u32 to_user;
	int status;

	if (copy_from_user(&ctl, (void __user *)arg, sizeof(ctl)))
		return -EFAULT;

	if (ctl.size < sizeof(ctl) || !ctl.tx_len ||
	    ctl.tx_len > STPGPS_FWCTL_BUF_MAX || !ctl.rx_max)
		return -EFAULT;

	if (copy_from_user(tx_buf, (void __user *)(uintptr_t)ctl.p_tx_buf,
			   ctl.tx_len))
		return -EFAULT;

	/* The vendor clears this flag when the chip resets the GPS task. */
	if (gps->rst_state == STPGPS_RST_START)
		return -EFAULT;

	status = mtk_wmt_gps_mcu_ctrl(tx_buf, ctl.tx_len, rx_buf,
				      STPGPS_FWCTL_BUF_MAX, &rx_len);
	if (status || rx_len < 2 || rx_len > STPGPS_FWCTL_BUF_MAX) {
		dev_dbg(gps->gps_misc.this_device,
			"fwctl failed: status %d, tx %u bytes, rx %u bytes\n",
			status, ctl.tx_len, rx_len);
		return -EFAULT;
	}

	to_user = min(ctl.rx_max, rx_len);
	if (copy_to_user((void __user *)(uintptr_t)ctl.p_rx_buf, rx_buf, to_user) ||
	    copy_to_user((void __user *)(uintptr_t)ctl.p_rx_len, &rx_len,
			 sizeof(rx_len)))
		return -EFAULT;

	dev_dbg(gps->gps_misc.this_device,
		"fwctl: tx %u bytes ([0]=%02x), rx %u bytes ([0]=%02x [1]=%02x)\n",
		ctl.tx_len, tx_buf[0], rx_len, rx_buf[0], rx_buf[1]);

	return 0;
}

static long stpgps_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	struct stpgps *gps = file->private_data;
	u32 value;

	if (!gps->gps_on)
		return -ENODEV;

	switch (cmd) {
	case COMBO_IOC_GPS_IC_HW_VERSION:
		value = mtk_wcn_wmt_ic_info_get(0);	/* WMTCHIN_HWVER */
		return copy_to_user((void __user *)arg, &value, sizeof(value)) ?
			-EFAULT : 0;

	case COMBO_IOC_GPS_IC_FW_VERSION:
		value = mtk_wcn_wmt_ic_info_get(1);	/* WMTCHIN_FWVER */
		return copy_to_user((void __user *)arg, &value, sizeof(value)) ?
			-EFAULT : 0;

	case COMBO_IOC_GET_GPS_LNA_PIN:
		value = mtk_wmt_get_gps_lna_pin_num();
		return copy_to_user((void __user *)arg, &value, sizeof(value)) ?
			-EFAULT : 0;

	case COMBO_IOC_GPS_LISTEN_RST_EVT:
		/*
		 * The vendor daemon blocks here until the channel goes away or the
		 * chip resets the GPS task - a reset means the session is over too.
		 */
		return wait_event_interruptible(gps->close_wait,
						!gps->opened ||
						gps->rst_state != STPGPS_RST_NONE);

	case COMBO_IOC_D1_EFUSE_GET:
		/*
		 * The vendor's MT6785 build has no branch for this read and logs
		 * "Read Efuse not supported in this platform", leaving the answer at
		 * 0. Answering -EFAULT here made the daemon see a failure where the
		 * vendor sees none.
		 */
		dev_dbg(gps->gps_misc.this_device,
			"D1 efuse not readable on MT6785, as the vendor driver says\n");
		return 0;

	case COMBO_IOC_RTC_FLAG:
		/* rtc_GPS_low_power_detected(): the vendor answers 0 (not low). */
		return 0;

	case COMBO_IOC_CO_CLOCK_FLAG:
		return mtk_wcn_wmt_co_clock_flag_get();

	case COMBO_IOC_TRIGGER_WMT_ASSERT:
		if (mtk_wcn_wmt_assert(WMTDRV_TYPE_GPS, (UINT32)arg) ==
		    MTK_WCN_BOOL_TRUE)
			return 0;
		dev_warn(gps->gps_misc.this_device,
			 "host-triggered FW assert failed\n");
		return -EBUSY;

	case COMBO_IOC_WMT_STATUS:
		/* -888 resetting, -889 reset finished, 0 normal (vendor values). */
		if (gps->rst_state == STPGPS_RST_START)
			return -888;
		if (gps->rst_state == STPGPS_RST_DONE)
			return -889;
		return 0;

	case COMBO_IOC_TAKE_GPS_WAKELOCK:
		stpgps_hold_wake_lock(gps, true);
		return gps->ws_held ? 0 : -EAGAIN;

	case COMBO_IOC_GIVE_GPS_WAKELOCK:
		stpgps_hold_wake_lock(gps, false);
		return gps->ws_held ? -EAGAIN : 0;

	case COMBO_IOC_GPS_FWCTL:
		return stpgps_fwctl(gps, arg);

	case 0:			/* enable/disable STP: WMT owns that */
	case 1:			/* raw data from GPS dev */
	case COMBO_IOC_GPS_HWVER:
	case COMBO_IOC_GPS_HW_SUSPEND:
	case COMBO_IOC_GPS_HW_RESUME:
	case COMBO_IOC_GPS_GET_MD_STATUS:
		/*
		 * 0 and 1: the daemon must not drive STP itself (-EINVAL).
		 * 6 (raw hw version): compiled out on this platform (-EFAULT).
		 * 18/19 (hw suspend/resume): no daemon on this SoC uses them.
		 * 21 (MD status): the vendor needs a physical MD2GPS register address,
		 * has none on MT6785 and answers -EFAULT.
		 */
		return (cmd == 0 || cmd == 1) ? -EINVAL : -EFAULT;

	default:
		return -EINVAL;
	}
}

static const struct file_operations stpgps_fops = {
	.owner = THIS_MODULE,
	.open = stpgps_open,
	.release = stpgps_release,
	.read = stpgps_read,
	.write = stpgps_write,
	.poll = stpgps_poll,
	.unlocked_ioctl = stpgps_ioctl,
};

static int gps_emi_open(struct inode *inode, struct file *file)
{
	struct stpgps *gps = container_of(file->private_data, struct stpgps,
					  emi_misc);

	file->private_data = gps;

	return 0;
}

static int gps_emi_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct stpgps *gps = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	int err;

	if (size > gps->emi_size)
		return -EINVAL;

	/*
	 * Real carveout memory, mapped straight to userspace: the MNL daemon
	 * writes the DSP image here and the conn MCU fetches it over EMI, so a
	 * private buffer would look right and never reach the chip.
	 */
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	err = remap_pfn_range(vma, vma->vm_start,
			      gps->emi_phys >> PAGE_SHIFT, size,
			      vma->vm_page_prot);
	if (err)
		return err;

	return 0;
}

/*
 * The MNL daemon's ioctl numbers on /dev/gps_emi, from the vendor
 * gps_emi.c. 1 is the one on the path to NMEA: it makes the driver reserve
 * and map the GPS window (there `mtk_wcn_consys_gps_emi_init()`), and the
 * daemon calls it before writing the DSP image. 2 and 3 are the NVRAM
 * helpers the daemon uses to keep its calibration file in sync with the
 * window; on this port the window is ordinary carveout memory the daemon
 * can read and write directly, so they only have to succeed. 4 returns the
 * window's physical address, which the daemon passes to the chip.
 */
#define IOCTL_EMI_MEMORY_INIT		1
#define IOCTL_MNL_NVRAM_FILE_TO_MEM	2
#define IOCTL_MNL_NVRAM_MEM_TO_FILE	3
#define IOCTL_ADC_CAPTURE_ADDR_GET	4

static long gps_emi_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct stpgps *gps = file->private_data;
	u32 addr = (u32)gps->emi_phys;

	switch (cmd) {
	case IOCTL_EMI_MEMORY_INIT:
		/* The window is reserved at probe; this is the daemon's cue to
		 * start writing, and answering 1 is what the vendor returns on
		 * success. */
		dev_dbg(gps->gps_misc.this_device,
			"GPS EMI window ready: phys 0x%llx size 0x%zx\n",
			(unsigned long long)gps->emi_phys, gps->emi_size);
		return 1;

	case IOCTL_MNL_NVRAM_FILE_TO_MEM:
	case IOCTL_MNL_NVRAM_MEM_TO_FILE:
		/* Directly addressable memory: no copy needed either way. */
		return 0;

	case IOCTL_ADC_CAPTURE_ADDR_GET:
		return copy_to_user((u32 __user *)arg, &addr, sizeof(addr)) ?
			-EFAULT : 0;

	default:
		/* The vendor driver returns 0 for unknown commands here. */
		return 0;
	}
}

static const struct file_operations gps_emi_fops = {
	.owner = THIS_MODULE,
	.open = gps_emi_open,
	.mmap = gps_emi_mmap,
	.unlocked_ioctl = gps_emi_ioctl,
};

/* ------------------------------------------------------------ power node */

/*
 * /dev/gps_pwr.
 *
 * mnld opens it (O_RDWR|O_NONBLOCK) for the power hints of the standalone GPS
 * designs: on those, this node switches the chip's rails through the PMIC. This
 * board has no such switch - the GNSS task lives on the CONNSYS MCU and is
 * powered by FUNC_CTRL(GPS on) through WMT, which is what /dev/stpgps does - but
 * the daemon asks for the node anyway and its power thread retries forever when
 * it is missing. So the node exists and answers "done" to every request, which
 * is the truth for this board: there is nothing left for it to switch.
 */
static int gps_pwr_open(struct inode *inode, struct file *file)
{
	struct stpgps *gps = container_of(file->private_data, struct stpgps,
					  pwr_misc);

	file->private_data = gps;

	return 0;
}

static long gps_pwr_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct stpgps *gps = file->private_data;

	dev_dbg(gps->gps_misc.this_device,
		"gps_pwr: cmd %u answered 0 (the GPS rails are switched by FUNC_CTRL)\n",
		cmd);

	return 0;
}

static const struct file_operations gps_pwr_fops = {
	.owner = THIS_MODULE,
	.open = gps_pwr_open,
	.unlocked_ioctl = gps_pwr_ioctl,
};

/* ---------------------------------------------------------------- driver */

static int stpgps_probe(struct platform_device *pdev)
{
	struct stpgps *gps;
	int err;

	gps = devm_kzalloc(&pdev->dev, sizeof(*gps), GFP_KERNEL);
	if (!gps)
		return -ENOMEM;

	mutex_init(&gps->lock);
	init_waitqueue_head(&gps->read_wait);
	init_waitqueue_head(&gps->close_wait);
	INIT_LIST_HEAD(&gps->rx_list);
	INIT_WORK(&gps->rx_work, stpgps_rx_work);

	/*
	 * Carve the GPS window out of the CONNSYS EMI region itself, the way
	 * stock does (gGpsEmiPhyBase = gConEmiPhyBase + GPS_EMI_BASE_ADDR_OFFSET),
	 * so the conn MCU can fetch the DSP image the daemon writes there.
	 * The DT's consys node carries the region through memory-region; the
	 * WMT module resolved it before us, so read it back from the same
	 * reservation rather than allocating fresh memory the chip cannot see.
	 */
	{
		struct device_node *consys;
		struct resource res;

		consys = of_find_compatible_node(NULL, NULL,
						 "mediatek,mt6785-consys");
		if (!consys) {
			dev_err(&pdev->dev, "cannot find the consys node for the EMI window\n");
			return -ENODEV;
		}
		/*
		 * Index 0 of the node's "memory-region" property, not the "_byname"
		 * helper: that one searches "memory-region-names"
		 * (drivers/of/of_reserved_mem.c, of_reserved_mem_region_to_resource_byname()),
		 * and the consys node has a bare "memory-region" with no names
		 * companion - so it could only ever return -EINVAL, which is the
		 * "cannot resolve the consys EMI region (-22)" that killed this probe
		 * on every boot up to lead/comb6. This is the same lookup the WMT
		 * module already does correctly: of_parse_phandle() index 0 plus
		 * of_reserved_mem_lookup() (common_main/platform/mtk_wcn_consys_hw.c,
		 * wmt_allocate_connsys_emi()).
		 */
		err = of_reserved_mem_region_to_resource(consys, 0, &res);
		of_node_put(consys);
		if (err) {
			dev_err(&pdev->dev, "cannot resolve the consys EMI region (%d)\n",
				err);
			return err;
		}

		gps->emi_phys = res.start + GPS_EMI_BASE_ADDR_OFFSET;
		gps->emi_size = GPS_EMI_SIZE;
		if (gps->emi_phys + gps->emi_size > res.end + 1) {
			dev_err(&pdev->dev,
				"GPS EMI window 0x%llx+0x%zx does not fit the carveout (end 0x%llx)\n",
				(unsigned long long)gps->emi_phys, gps->emi_size,
				(unsigned long long)res.end);
			return -ERANGE;
		}

		/*
		 * ioremap, not memremap: the carveout is declared no-map, so it is
		 * not in the linear map and the vendor maps it the same way
		 * (ioremap() in mtk_wcn_consys_gps_emi_init()).
		 */
		gps->emi_buf = ioremap(gps->emi_phys, gps->emi_size);
		if (!gps->emi_buf) {
			dev_err(&pdev->dev, "cannot map the GPS EMI window\n");
			return -ENOMEM;
		}
	}

	/*
	 * The daemon's GPS wake lock. Registered here, after every early return of
	 * this probe, so a failed probe cannot leave a wakeup source behind.
	 */
	gps->ws = wakeup_source_register(&pdev->dev, "stpgps");
	if (!gps->ws)
		return -ENOMEM;

	gps->gps_misc.minor = MISC_DYNAMIC_MINOR;
	gps->gps_misc.name = STPGPS_DEV_NAME;
	gps->gps_misc.fops = &stpgps_fops;
	gps->gps_misc.parent = &pdev->dev;

	gps->emi_misc.minor = MISC_DYNAMIC_MINOR;
	gps->emi_misc.name = GPS_EMI_DEV_NAME;
	gps->emi_misc.fops = &gps_emi_fops;
	gps->emi_misc.parent = &pdev->dev;

	err = misc_register(&gps->gps_misc);
	if (err) {
		dev_err(&pdev->dev, "cannot create /dev/%s (%d)\n",
			STPGPS_DEV_NAME, err);
		goto err_free;
	}

	gps->pwr_misc.minor = MISC_DYNAMIC_MINOR;
	gps->pwr_misc.name = GPS_PWR_DEV_NAME;
	gps->pwr_misc.fops = &gps_pwr_fops;
	gps->pwr_misc.parent = &pdev->dev;

	err = misc_register(&gps->pwr_misc);
	if (err) {
		dev_err(&pdev->dev, "cannot create /dev/%s (%d)\n",
			GPS_PWR_DEV_NAME, err);
		goto err_unreg_gps;
	}

	err = misc_register(&gps->emi_misc);
	if (err) {
		dev_err(&pdev->dev, "cannot create /dev/%s (%d)\n",
			GPS_EMI_DEV_NAME, err);
		goto err_unreg_pwr;
	}

	err = mtk_wcn_stp_register_event_cb(GPS_TASK_INDX, stpgps_rx_cb);
	if (err) {
		dev_err(&pdev->dev, "cannot register the STP RX callback (%d)\n",
			err);
		goto err_unreg_emi;
	}

	platform_set_drvdata(pdev, gps);
	WRITE_ONCE(stpgps_dev, gps);

	dev_info(&pdev->dev,
		 "GNSS bridge ready: /dev/%s, /dev/%s, /dev/%s, EMI window 0x%llx+0x%zx\n",
		 STPGPS_DEV_NAME, GPS_EMI_DEV_NAME, GPS_PWR_DEV_NAME,
		 (unsigned long long)gps->emi_phys, gps->emi_size);

	return 0;

err_unreg_emi:
	misc_deregister(&gps->emi_misc);
err_unreg_pwr:
	misc_deregister(&gps->pwr_misc);
err_unreg_gps:
	misc_deregister(&gps->gps_misc);
err_free:
	wakeup_source_unregister(gps->ws);
	iounmap(gps->emi_buf);

	return err;
}

static void stpgps_remove(struct platform_device *pdev)
{
	struct stpgps *gps = platform_get_drvdata(pdev);
	struct sk_buff *skb, *tmp;

	/*
	 * No new frames first, then whatever the RX work still has in flight,
	 * and only then end of file for a reader that is still in read().
	 */
	WRITE_ONCE(stpgps_dev, NULL);
	mtk_wcn_stp_register_event_cb(GPS_TASK_INDX, NULL);
	cancel_work_sync(&gps->rx_work);
	stpgps_hold_wake_lock(gps, false);
	wakeup_source_unregister(gps->ws);

	WRITE_ONCE(gps->shutting_down, true);
	wake_up_interruptible(&gps->read_wait);
	wake_up_interruptible(&gps->close_wait);

	misc_deregister(&gps->emi_misc);
	misc_deregister(&gps->pwr_misc);
	misc_deregister(&gps->gps_misc);

	mutex_lock(&gps->lock);
	list_for_each_entry_safe(skb, tmp, &gps->rx_list, list) {
		list_del(&skb->list);
		kfree_skb(skb);
	}
	mutex_unlock(&gps->lock);

	iounmap(gps->emi_buf);
}

static const struct of_device_id stpgps_of_match[] = {
	{ .compatible = "mediatek,mt6785-stpgps" },
	{ }
};
MODULE_DEVICE_TABLE(of, stpgps_of_match);

static struct platform_driver stpgps_driver = {
	.probe = stpgps_probe,
	.remove = stpgps_remove,
	.driver = {
		.name = STPGPS_DEV_NAME,
		.of_match_table = stpgps_of_match,
	},
};
module_platform_driver(stpgps_driver);

MODULE_AUTHOR("The poco-m5s port");
MODULE_DESCRIPTION("MediaTek MT6785 GNSS bridge over the CONNSYS STP transport");
MODULE_LICENSE("GPL");
