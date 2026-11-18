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
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "osal_typedef.h"
#include "wmt_exp.h"
#include "stp_exp.h"

#define STPGPS_DEV_NAME		"stpgps"
#define GPS_EMI_DEV_NAME	"gps_emi"

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
 * The MNL daemon then writes its DSP image into that window through
 * /dev/gps_emi and the conn MCU fetches it from there, which is why this
 * window must be real memory the chip can reach and not a private vmalloc
 * buffer.
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

struct stpgps {
	struct miscdevice gps_misc;
	struct miscdevice emi_misc;

	/* Guards opened/rx_list: read() and the RX callback both take it. */
	struct mutex lock;
	wait_queue_head_t read_wait;
	wait_queue_head_t close_wait;

	bool opened;
	bool gps_on;

	/* RX queue: one STP frame per entry. */
	struct list_head rx_list;
	unsigned int rx_len;

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
		wake_up_interruptible(&gps->read_wait);
}

/* Called from the RX thread; takes one STP frame off the queue. */
static ssize_t stpgps_drain(void)
{
	struct stpgps *gps = stpgps_dev;
	int total = 0;

	while (!mtk_wcn_stp_is_rxqueue_empty(GPS_TASK_INDX)) {
		struct sk_buff *skb;
		int cnt;

		skb = alloc_skb(STPGPS_FRAME_SIZE, GFP_ATOMIC);
		if (!skb)
			break;

		cnt = mtk_wcn_stp_receive_data(skb->data, STPGPS_FRAME_SIZE,
					       GPS_TASK_INDX);
		if (cnt <= 0) {
			kfree_skb(skb);
			break;
		}

		skb_put(skb, cnt);
		mutex_lock(&gps->lock);
		list_add_tail(&skb->list, &gps->rx_list);
		gps->rx_len += cnt;
		mutex_unlock(&gps->lock);
		total += cnt;
	}

	return total;
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
	file->private_data = gps;

	/*
	 * The vendor's GPS_open() refuses to proceed unless the STP transport is
	 * ready: "STP is not ready, Cannot open GPS Devices" -> -ENODEV. Ours
	 * skipped that, so a daemon could open the device on a WMT that never
	 * finished coming up and then read nothing forever, with no error to
	 * show for it. Closing again restores the function state.
	 */
	if (!mtk_wcn_stp_is_ready()) {
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

	mutex_lock(&gps->lock);
	gps->opened = false;
	mutex_unlock(&gps->lock);

	/* Wake a reader blocked in the ioctl that waits for the close. */
	wake_up_interruptible(&gps->close_wait);

	return 0;
}

static ssize_t stpgps_read(struct file *file, char __user *buf, size_t count,
			   loff_t *ppos)
{
	struct stpgps *gps = file->private_data;
	struct sk_buff *skb;
	size_t len;
	int err;

	if (!gps->gps_on)
		return -ENODEV;

	err = wait_event_interruptible(gps->read_wait,
				       kthread_should_stop() ||
				       !mtk_wcn_stp_is_rxqueue_empty(GPS_TASK_INDX));
	if (err)
		return err;

	stpgps_drain();

	mutex_lock(&gps->lock);
	skb = list_first_entry_or_null(&gps->rx_list, struct sk_buff, list);
	if (!skb) {
		mutex_unlock(&gps->lock);
		return 0;
	}
	list_del(&skb->list);

	len = min(count, skb->len);
	if (copy_to_user(buf, skb->data, len))
		err = -EFAULT;
	else
		err = len;

	mutex_unlock(&gps->lock);
	kfree_skb(skb);

	return err;
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
	kfree(kbuf);

	return err < 0 ? err : count;
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

	return mask;
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
		/* The vendor daemon blocks here until the channel goes away. */
		return wait_event_interruptible(gps->close_wait, !gps->opened);

	case 0:			/* enable/disable STP: WMT owns that */
	case 1:			/* raw data from GPS dev */
	case COMBO_IOC_GPS_HWVER:
	case COMBO_IOC_D1_EFUSE_GET:
	case COMBO_IOC_RTC_FLAG:
	case COMBO_IOC_CO_CLOCK_FLAG:
	case COMBO_IOC_TRIGGER_WMT_ASSERT:
	case COMBO_IOC_WMT_STATUS:
	case COMBO_IOC_TAKE_GPS_WAKELOCK:
	case COMBO_IOC_GIVE_GPS_WAKELOCK:
	case COMBO_IOC_GPS_FWCTL:
	case COMBO_IOC_GPS_HW_SUSPEND:
	case COMBO_IOC_GPS_HW_RESUME:
	case COMBO_IOC_GPS_GET_MD_STATUS:
		/*
		 * The vendor driver answers -EINVAL for the first two and
		 * -EFAULT for the unimplemented rest; none of them is on the
		 * path to NMEA output.
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

	err = misc_register(&gps->emi_misc);
	if (err) {
		dev_err(&pdev->dev, "cannot create /dev/%s (%d)\n",
			GPS_EMI_DEV_NAME, err);
		goto err_unreg_gps;
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
		 "GNSS bridge ready: /dev/%s, /dev/%s, EMI window 0x%llx+0x%zx\n",
		 STPGPS_DEV_NAME, GPS_EMI_DEV_NAME,
		 (unsigned long long)gps->emi_phys, gps->emi_size);

	return 0;

err_unreg_emi:
	misc_deregister(&gps->emi_misc);
err_unreg_gps:
	misc_deregister(&gps->gps_misc);
err_free:
	iounmap(gps->emi_buf);

	return err;
}

static void stpgps_remove(struct platform_device *pdev)
{
	struct stpgps *gps = platform_get_drvdata(pdev);

	WRITE_ONCE(stpgps_dev, NULL);
	mtk_wcn_stp_register_event_cb(GPS_TASK_INDX, NULL);

	misc_deregister(&gps->emi_misc);
	misc_deregister(&gps->gps_misc);
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
