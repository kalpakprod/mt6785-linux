/* SPDX-License-Identifier: GPL-2.0 */
/* compat: mtk_lpae.h (begonia fwport). begonia runs without the BSP 4G-LPAE
 * remap; enable_4G() returns 0 (faithful to the !CONFIG_ARM_LPAE vendor path),
 * so the CROSS_OVERFLOW / MAPPING_DRAM macros are no-ops. */
#ifndef _COMPAT_MTK_LPAE_H
#define _COMPAT_MTK_LPAE_H
static inline unsigned int enable_4G(void) { return 0; }
#define MT_OVERFLOW_ADDR_START 0
#define CROSS_OVERFLOW_ADDR_TRANSFER(phy_addr, size, ret) do { (ret) = 0; } while (0)
#define MAPPING_DRAM_ACCESS_ADDR(phy_addr) do { } while (0)
#endif
