// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 factory addresses from a copied nvdata tree.
 *
 * Copyright (C) 2026 The poco-m5s port
 *
 * The Wi-Fi MAC and the Bluetooth address of this phone live in the nvdata
 * partition, which stock Android reads through MTK's nvram service. Nothing on
 * mainline reads that partition, and it must not be read live either: a wrong
 * write there destroys the RF calibration. The port therefore uses a read-only
 * *copy* that the owner takes once from the phone:
 *
 *   /lib/firmware/rosemary/nvdata/APCFG/APRDEB/WIFI
 *   /lib/firmware/rosemary/nvdata/APCFG/APRDEB/BT_Addr
 *
 * Both files are MTK NVRAM records: a 12-byte header followed by the payload.
 * The address sits at payload offset 0 in the byte order the network stack
 * expects. The fourth octet of the Wi-Fi record is the per-unit part of the
 * MAC, so no two phones share it.
 *
 * Sizes, per the begonia port which read the same records: the BT record is
 * 440 bytes in total, the Wi-Fi record is shorter than that and starts with
 * the same header.
 *
 * This module loads early and publishes the two addresses to the drivers that
 * need them; when the files are absent - the normal case until the owner
 * copies them - both consumers keep their previous behaviour (a random Wi-Fi
 * MAC from gen4m and the firmware default BD_ADDR, which the Bluetooth driver
 * then re-applies).
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "nvdata_mac.h"

#define NVDATA_DIR		"rosemary/nvdata/APCFG/APRDEB"
#define NVDATA_WIFI_FILE	NVDATA_DIR "/WIFI"
#define NVDATA_BT_FILE		NVDATA_DIR "/BT_Addr"

/* MTK NVRAM record header, 12 bytes; the payload follows it. */
#define NVRAM_HEADER_SIZE	12

/* Enough for the largest record either file holds (BT is 440 bytes). */
#define NVRAM_RECORD_MAX	512

static u8 mtk_nvdata_wifi_mac[ETH_ALEN];
static u8 mtk_nvdata_bt_addr[ETH_ALEN];
static bool mtk_nvdata_wifi_valid;
static bool mtk_nvdata_bt_valid;
static DEFINE_MUTEX(mtk_nvdata_lock);

bool mtk_nvdata_get_wifi_mac(u8 mac[ETH_ALEN])
{
	bool found;

	mutex_lock(&mtk_nvdata_lock);
	found = mtk_nvdata_wifi_valid;
	if (found)
		ether_addr_copy(mac, mtk_nvdata_wifi_mac);
	mutex_unlock(&mtk_nvdata_lock);

	return found;
}
EXPORT_SYMBOL_GPL(mtk_nvdata_get_wifi_mac);

bool mtk_nvdata_get_bt_addr(u8 addr[ETH_ALEN])
{
	bool found;

	mutex_lock(&mtk_nvdata_lock);
	found = mtk_nvdata_bt_valid;
	if (found)
		ether_addr_copy(addr, mtk_nvdata_bt_addr);
	mutex_unlock(&mtk_nvdata_lock);

	return found;
}
EXPORT_SYMBOL_GPL(mtk_nvdata_get_bt_addr);

/* Reject an all-zero or broadcast record: those are erased or default data. */
static bool nvdata_addr_is_valid(const u8 *addr)
{
	static const u8 zero[ETH_ALEN];
	static const u8 bcast[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	return !ether_addr_equal(addr, zero) && !ether_addr_equal(addr, bcast);
}

static int nvdata_load_one(const char *name, u8 addr[ETH_ALEN])
{
	const struct firmware *fw;
	int err;

	err = request_firmware(&fw, name, NULL);
	if (err) {
		pr_info("nvdata: %s not available (%d), keeping the default address\n",
			name, err);
		return err;
	}

	if (fw->size < NVRAM_HEADER_SIZE + ETH_ALEN) {
		pr_warn("nvdata: %s is too short (%zu bytes)\n", name, fw->size);
		err = -EINVAL;
		goto out;
	}

	ether_addr_copy(addr, fw->data + NVRAM_HEADER_SIZE);

	if (!nvdata_addr_is_valid(addr)) {
		pr_warn("nvdata: %s holds no usable address (%pM), ignoring it\n",
			name, addr);
		err = -EINVAL;
		goto out;
	}

	pr_info("nvdata: %s gives %pM\n", name, addr);

out:
	release_firmware(fw);

	return err;
}

static int mtk_nvdata_probe(struct platform_device *pdev)
{
	u8 addr[ETH_ALEN];

	if (!nvdata_load_one(NVDATA_WIFI_FILE, addr)) {
		mutex_lock(&mtk_nvdata_lock);
		ether_addr_copy(mtk_nvdata_wifi_mac, addr);
		mtk_nvdata_wifi_valid = true;
		mutex_unlock(&mtk_nvdata_lock);
	}

	if (!nvdata_load_one(NVDATA_BT_FILE, addr)) {
		mutex_lock(&mtk_nvdata_lock);
		ether_addr_copy(mtk_nvdata_bt_addr, addr);
		mtk_nvdata_bt_valid = true;
		mutex_unlock(&mtk_nvdata_lock);
	}

	return 0;
}

static void mtk_nvdata_remove(struct platform_device *pdev)
{
	mutex_lock(&mtk_nvdata_lock);
	mtk_nvdata_wifi_valid = false;
	mtk_nvdata_bt_valid = false;
	mutex_unlock(&mtk_nvdata_lock);
}

static const struct of_device_id mtk_nvdata_of_match[] = {
	{ .compatible = "mediatek,mt6785-nvdata" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_nvdata_of_match);

static struct platform_driver mtk_nvdata_driver = {
	.probe = mtk_nvdata_probe,
	.remove = mtk_nvdata_remove,
	.driver = {
		.name = "mtk-nvdata",
		.of_match_table = mtk_nvdata_of_match,
	},
};
module_platform_driver(mtk_nvdata_driver);

MODULE_AUTHOR("The poco-m5s port");
MODULE_DESCRIPTION("MediaTek MT6785 factory addresses from a copied nvdata tree");
MODULE_LICENSE("GPL");
