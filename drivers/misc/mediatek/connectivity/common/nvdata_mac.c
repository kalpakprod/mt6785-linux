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
 * This phone's own records hold no address at all: the WIFI record's MAC field
 * is 00:00:00:00:00:00 and the BT_Addr record's 440 payload bytes contain no
 * unicast six-byte run (measured on the copy in this repository, 18.11). So
 * the module also accepts an explicit, one-line override that survives boots:
 *
 *   /lib/firmware/rosemary/wifi_mac     e.g. "xx:xx:xx:xx:xx:xx"
 *   /lib/firmware/rosemary/bt_addr      e.g. "07:11:22:33:44:55"
 *
 * either as that 17-character text or as six raw bytes. The override wins over
 * the nvdata record, the record wins over nothing at all, and when neither is
 * usable the consumers keep their previous behaviour (gen4m derives a Wi-Fi
 * MAC per boot, the Bluetooth driver re-applies what the controller reports).
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "nvdata_mac.h"

#define NVDATA_DIR		"rosemary/nvdata/APCFG/APRDEB"
#define NVDATA_WIFI_FILE	NVDATA_DIR "/WIFI"
#define NVDATA_BT_FILE		NVDATA_DIR "/BT_Addr"

/* One-line overrides, kept outside the nvdata tree on purpose: they are the
 * port's own files, not a copy of the phone's factory data. */
#define OVERRIDE_WIFI_FILE	"rosemary/wifi_mac"
#define OVERRIDE_BT_FILE	"rosemary/bt_addr"

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

/*
 * Reject what is not a usable unicast address: all-zero and broadcast records
 * are erased or default data, and a multicast one is not an address any radio
 * may adopt. The last case is not hypothetical: this phone's BT_Addr record
 * yields xx:xx:xx:xx:xx:xx (byte 0 odd), which the driver used to apply to the
 * controller before this check existed (lead note 22).
 */
static bool nvdata_addr_is_valid(const u8 *addr)
{
	static const u8 zero[ETH_ALEN];
	static const u8 bcast[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	if (ether_addr_equal(addr, zero) || ether_addr_equal(addr, bcast))
		return false;

	return !is_multicast_ether_addr(addr);
}

/*
 * Parse an address from one of the override files: seventeen characters of
 * text ("xx:xx:xx:xx:xx:xx", the shape the owner reads off a router or off
 * Android's settings), or six raw bytes. Anything else is refused loudly,
 * because a silently ignored override would look like the feature not working.
 */
static int nvdata_parse_override(const char *name, const struct firmware *fw,
				 u8 addr[ETH_ALEN])
{
	u8 parsed[ETH_ALEN];

	if (fw->size >= 17 && fw->data[2] == ':' && fw->data[5] == ':') {
		if (sscanf((const char *)fw->data,
			   "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
			   &parsed[0], &parsed[1], &parsed[2],
			   &parsed[3], &parsed[4], &parsed[5]) != ETH_ALEN) {
			pr_warn("nvdata: %s is not an address in aa:bb:cc:dd:ee:ff form, ignoring it\n",
				name);
			return -EINVAL;
		}
	} else if (fw->size >= ETH_ALEN) {
		ether_addr_copy(parsed, fw->data);
	} else {
		pr_warn("nvdata: %s is %zu bytes, too short for an address\n",
			name, fw->size);
		return -EINVAL;
	}

	if (!nvdata_addr_is_valid(parsed)) {
		pr_warn("nvdata: %s holds no usable address (%pM), ignoring it\n",
			name, parsed);
		return -EINVAL;
	}

	ether_addr_copy(addr, parsed);
	return 0;
}

/* An override, if it exists, wins over the nvdata record. */
static bool nvdata_load_override(const char *name, u8 addr[ETH_ALEN])
{
	const struct firmware *fw;
	int err;

	err = request_firmware(&fw, name, NULL);
	if (err)
		return false;

	err = nvdata_parse_override(name, fw, addr);
	release_firmware(fw);
	if (!err)
		pr_info("nvdata: address taken from the override %s\n", name);

	return err == 0;
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

static void nvdata_take(bool is_wifi, const u8 addr[ETH_ALEN])
{
	mutex_lock(&mtk_nvdata_lock);
	if (is_wifi) {
		ether_addr_copy(mtk_nvdata_wifi_mac, addr);
		mtk_nvdata_wifi_valid = true;
	} else {
		ether_addr_copy(mtk_nvdata_bt_addr, addr);
		mtk_nvdata_bt_valid = true;
	}
	mutex_unlock(&mtk_nvdata_lock);
}

static int mtk_nvdata_probe(struct platform_device *pdev)
{
	u8 addr[ETH_ALEN];

	/* Wi-Fi: the override file first, then the factory record. */
	if (nvdata_load_override(OVERRIDE_WIFI_FILE, addr) ||
	    !nvdata_load_one(NVDATA_WIFI_FILE, addr))
		nvdata_take(true, addr);

	/* Bluetooth: the same order. */
	if (nvdata_load_override(OVERRIDE_BT_FILE, addr) ||
	    !nvdata_load_one(NVDATA_BT_FILE, addr))
		nvdata_take(false, addr);

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
