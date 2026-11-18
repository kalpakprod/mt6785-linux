// SPDX-License-Identifier: GPL-2.0
/*
 * wmt_wifi_trigger.c - minimal /dev/wmtWifi WiFi on/off control node for the
 * begonia (MT6785) vendor WMT/connsys forward-port.
 *
 * The stock /dev/wmtWifi node is created by the vendor wlan adaptor
 * (wmt_chrdev_wifi.c), which is not present in any available source tree and is
 * therefore not part of this forward-port. This is a faithful, minimal
 * reconstruction of that node's WiFi power control: a write of '1' / '0' drives
 * mtk_wcn_wmt_func_on/off(WMTDRV_TYPE_WIFI) - the same exported WMT entry the
 * stock node uses, and exactly what the working stock oracle does
 * (`echo 1 > /dev/wmtWifi`). func_on(WIFI) powers connsys, arms the WFSYS, and
 * invokes the gen4m-registered wlan probe -> wlan0. Compiled INTO wmt_drv so it
 * links the export directly (no cross-module symbol-version concerns).
 *
 * RUNTIME-UNPROVEN: compile/link verified only. Validate on hardware (the exact
 * func_on ordering / return semantics are confirmed from wmt_exp.c source, but
 * the end-to-end bring-up has not been exercised on a device).
 */
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/printk.h>

#include "osal_typedef.h"
#include "wmt_exp.h"

/* begonia fwport: register the BTIF HIF with the WMT core (wmt_dev.c) before the
 * first func_on - replaces the userspace launcher's WMT_IOCTL_SET_STP_MODE. */
int wmt_dev_set_hif_btif(void);
struct work_struct;
static void wmt_late_wlan_load(struct work_struct *work);

static ssize_t wmt_wifi_trigger_write(struct file *filp, const char __user *buf,
				      size_t count, loff_t *ppos)
{
	char c = 0;

	if (count == 0)
		return 0;
	if (get_user(c, buf))
		return -EFAULT;

	if (c == '1' || c == 1) {
		/* register BTIF as the STP transport first (else stp_init has no
		 * hif info); idempotent after the first call. */
		wmt_dev_set_hif_btif();
		if (mtk_wcn_wmt_func_on(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_FALSE) {
			pr_err("[wmtWifi] func_on(WIFI) failed\n");
			return -EIO;
		}
		pr_info("[wmtWifi] WIFI function ON\n");
	} else if (c == '0' || c == 0) {
		if (mtk_wcn_wmt_func_off(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_FALSE) {
			pr_err("[wmtWifi] func_off(WIFI) failed\n");
			return -EIO;
		}
		pr_info("[wmtWifi] WIFI function OFF\n");
	}
	return count;
}

static const struct file_operations wmt_wifi_trigger_fops = {
	.owner = THIS_MODULE,
	.write = wmt_wifi_trigger_write,
	.llseek = noop_llseek,
};

static struct miscdevice wmt_wifi_trigger_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "wmtWifi",
	.fops = &wmt_wifi_trigger_fops,
	.mode = 0660,
};

int wmt_wifi_trigger_init(void)
{
	int ret = misc_register(&wmt_wifi_trigger_dev);

	if (ret)
		pr_err("[wmtWifi] misc_register failed (%d)\n", ret);
	else
		pr_info("[wmtWifi] /dev/wmtWifi ready (begonia fwport trigger)\n");
	return ret;
}

void wmt_wifi_trigger_exit(void)
{
	misc_deregister(&wmt_wifi_trigger_dev);
}
