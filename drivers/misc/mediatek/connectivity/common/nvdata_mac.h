/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 factory addresses taken from a copied nvdata tree.
 *
 * The records themselves are read once, from /lib/firmware, by nvdata_mac.c;
 * this header is what the Wi-Fi and Bluetooth drivers include.
 */
#ifndef _MTK_NVDATA_MAC_H
#define _MTK_NVDATA_MAC_H

#include <linux/if_ether.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_MTK_NVDATA_MAC)
bool mtk_nvdata_get_wifi_mac(u8 mac[ETH_ALEN]);
bool mtk_nvdata_get_bt_addr(u8 addr[ETH_ALEN]);
#else
static inline bool mtk_nvdata_get_wifi_mac(u8 mac[ETH_ALEN])
{
	return false;
}

static inline bool mtk_nvdata_get_bt_addr(u8 addr[ETH_ALEN])
{
	return false;
}
#endif

#endif /* _MTK_NVDATA_MAC_H */
