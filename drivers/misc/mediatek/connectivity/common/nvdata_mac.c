// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 Wi-Fi MAC and Bluetooth address for the port.
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
 * The two records have the layouts the vendor's own structures give them, not
 * the offsets guessed in h5 (which read six bytes too far into each and found
 * 00:00:00:00:00:00 and xx:xx:xx:xx:xx:xx - lead note 23):
 *
 *   APCFG/APRDEB/WIFI     2050 bytes of file = struct WIFI_CFG_PARAM_STRUCT
 *                         (2048 bytes, gen4m/include/CFG_Wifi_File.h, whose
 *                         own compile-time asserts pin it) plus two trailing
 *                         bytes the nvram layer keeps. Two u16 version words
 *                         open the struct, so aucMacAddress[6] is at byte 4.
 *                         This phone's copy holds xx:xx:xx:xx:xx:xx.
 *   APCFG/APRDEB/BT_Addr  440 bytes of file, and it begins with the BD_ADDR:
 *                         xx:xx:xx:xx:xx:xx - the Wi-Fi address minus one, the
 *                         way the factory writes the pair. Byte 12 is where the
 *                         h5 guess read, and it holds xx:xx:xx:xx:xx:xx, a
 *                         multicast address no radio may adopt.
 *
 * So, in order of what wins:
 *
 *   1. an explicit one-line override, for the owner's real addresses or to
 *      pin one by hand: /lib/firmware/rosemary/wifi_mac and .../bt_addr,
 *      either 17 characters of text ("xx:xx:xx:xx:xx:xx") or six raw bytes;
 *   2. the factory record from the nvdata copy above;
 *   3. a stable address derived from the device serial, with the
 *      locally-administered bit set and the multicast bit clear, so a unit
 *      without a usable record keeps one address across boots instead of
 *      gen4m's 00:08:22 + timer, which changed every boot.
 *
 * Nothing here writes to the nvdata partition: a wrong write destroys the RF
 * calibration.
 */

#include <crypto/sha2.h>

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/types.h>

#include "nvdata_mac.h"

#define NVDATA_DIR		"rosemary/nvdata/APCFG/APRDEB"
#define NVDATA_WIFI_FILE	NVDATA_DIR "/WIFI"
#define NVDATA_BT_FILE		NVDATA_DIR "/BT_Addr"

/* One-line overrides, kept outside the nvdata tree on purpose: they are the
 * port's own files, not a copy of the phone's factory data. */
#define OVERRIDE_WIFI_FILE	"rosemary/wifi_mac"
#define OVERRIDE_BT_FILE	"rosemary/bt_addr"

/*
 * Where each record keeps its address (see the comment at the top): the WIFI
 * record's aucMacAddress after two version words, the BT_Addr record's BD_ADDR
 * at the very start.
 */
#define WIFI_MAC_OFFSET		4
#define BT_ADDR_OFFSET		0

/* "androidboot.serialno=" is what the loader passes this phone at every boot. */
#define SERIALNO_PROP		"androidboot.serialno="
#define SERIAL_MAX		64

/* Domain separator for the derived addresses: role, then the serial. */
#define DERIVE_PREFIX		"poco-m5s/rosemary/"

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
 * may adopt. That last case is what made h5's wrong offset visible rather than
 * silent: read at byte 12, this phone's BT_Addr record yields xx:xx:xx:xx:xx:xx,
 * whose odd first octet is the multicast bit, and the controller took it as the
 * BD_ADDR until this check existed (lead note 22). Read where the record really
 * keeps it, the same file yields xx:xx:xx:xx:xx:xx and passes.
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

/*
 * The factory record: `name` is the file in the nvdata copy, `offset` where its
 * structure keeps the address.
 */
static int nvdata_load_record(const char *name, unsigned int offset,
			      u8 addr[ETH_ALEN])
{
	const struct firmware *fw;
	int err;

	err = request_firmware(&fw, name, NULL);
	if (err) {
		pr_info("nvdata: %s not available (%d)\n", name, err);
		return err;
	}

	if (fw->size < offset + ETH_ALEN) {
		pr_warn("nvdata: %s is %zu bytes, too short for an address at byte %u\n",
			name, fw->size, offset);
		err = -EINVAL;
		goto out;
	}

	ether_addr_copy(addr, fw->data + offset);

	if (!nvdata_addr_is_valid(addr)) {
		pr_warn("nvdata: %s holds no usable address at byte %u (%pM), ignoring it\n",
			name, offset, addr);
		err = -EINVAL;
		goto out;
	}

	pr_info("nvdata: %s gives %pM (byte %u)\n", name, addr, offset);

out:
	release_firmware(fw);

	return err;
}

/*
 * The loader's view of this phone's serial, from /chosen/bootargs - the same
 * string the kernel prints as "androidboot.serialno=" in its command line, and
 * the one thing this board keeps per unit that the kernel can read without
 * touching a partition. Returns false when the bootloader did not pass one.
 */
static bool nvdata_serial(char *out, size_t len)
{
	struct device_node *chosen;
	const char *prop, *bootargs, *p, *end;
	int prop_len;
	size_t n;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return false;

	prop = of_get_property(chosen, "bootargs", &prop_len);
	of_node_put(chosen);
	if (!prop || prop_len <= 0)
		return false;

	/*
	 * The kernel's own command line comes out of this property, so it ends
	 * with a NUL in practice - but the length is what makes the search below
	 * safe, and the loader is not ours to trust.
	 */
	bootargs = prop;
	n = strnlen(bootargs, prop_len);

	p = strnstr(bootargs, SERIALNO_PROP, n);
	if (!p)
		return false;

	p += strlen(SERIALNO_PROP);
	end = memchr(p, ' ', n - (size_t)(p - bootargs));
	n = end ? (size_t)(end - p) : n - (size_t)(p - bootargs);

	if (!n || n >= len)
		return false;

	memcpy(out, p, n);
	out[n] = '\0';

	return true;
}

/*
 * A stable address for a unit whose nvdata copy holds nothing usable. The
 * serial is hashed with SHA-256 under a role prefix, so the address is the
 * same on every boot and on every reinstall of this image, two roles never
 * collide, and the serial itself cannot be recovered from it. The two bits
 * that make it a legal address are set the standard way: the multicast bit
 * cleared, the locally-administered bit set.
 */
static bool nvdata_derive(const char *role, u8 addr[ETH_ALEN])
{
	char serial[SERIAL_MAX];
	char seed[SERIAL_MAX + sizeof(DERIVE_PREFIX) + sizeof("bluetooth")];
	u8 digest[SHA256_DIGEST_SIZE];
	int len;

	if (!nvdata_serial(serial, sizeof(serial))) {
		pr_warn("nvdata: no %s address and no androidboot.serialno to derive one from\n",
			role);
		return false;
	}

	len = scnprintf(seed, sizeof(seed), DERIVE_PREFIX "%s:%s", role, serial);
	sha256(seed, len, digest);

	memcpy(addr, digest, ETH_ALEN);
	addr[0] = (addr[0] & 0xfc) | 0x02;

	pr_info("nvdata: no factory %s address, using the stable one derived from the device serial: %pM\n",
		role, addr);

	return true;
}

static void nvdata_take(bool is_wifi, const u8 addr[ETH_ALEN]);

/* One role, in the order the override, the record and the derivation win. */
static void nvdata_resolve(bool is_wifi, const char *override,
			   const char *record, unsigned int offset)
{
	u8 addr[ETH_ALEN];

	if (nvdata_load_override(override, addr) ||
	    !nvdata_load_record(record, offset, addr) ||
	    nvdata_derive(is_wifi ? "wifi" : "bluetooth", addr))
		nvdata_take(is_wifi, addr);
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
	nvdata_resolve(true, OVERRIDE_WIFI_FILE, NVDATA_WIFI_FILE,
		       WIFI_MAC_OFFSET);
	nvdata_resolve(false, OVERRIDE_BT_FILE, NVDATA_BT_FILE,
		       BT_ADDR_OFFSET);

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
MODULE_DESCRIPTION("MediaTek MT6785 Wi-Fi MAC and Bluetooth address");
MODULE_LICENSE("GPL");
