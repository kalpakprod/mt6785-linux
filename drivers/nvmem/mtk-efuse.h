/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek eFuse cell decoding shared between the eFuse driver and the driver that
 * exposes the bootloader's copy of the block.
 */

#ifndef __MTK_EFUSE_H
#define __MTK_EFUSE_H

#include <linux/bits.h>
#include <linux/nvmem-provider.h>
#include <linux/string.h>

/*
 * The MT6785 GPU eFuse is not a small bin number but the chip's segment code, a whole
 * byte whose encoding changed between eFuse revisions. The vendor's MT6785 gpufreq
 * driver decodes it twice - byte to segment (mtk_gpufreq_core.c:3392-3406), segment to
 * the fastest OPP it may use (:3033-3042) - and the four segments differ exactly in
 * that top OPP:
 *
 *	MT6785U	900 MHz	(vendor OPP index 0)  efuse 0xE0, 0x07
 *	MT6785T	806 MHz	(vendor OPP index 15) everything else, including 0x00
 *	MT6785	720 MHz	(vendor OPP index 21) efuse 0xC0, 0x03, 0x20, 0x04
 *	MT6783	570 MHz	(vendor OPP index 30) efuse 0x80, 0x01, 0x40, 0x02
 *
 * Turning the byte into a bin index and then into BIT(index) gives the GPU's
 * opp-supported-hw cells a mask they can be tested against, which is the same shape
 * mt8186 uses for its three speed bins.
 */
#define MTK_MT6785_GPU_BIN_6785U	0
#define MTK_MT6785_GPU_BIN_6785		1
#define MTK_MT6785_GPU_BIN_6785T	2
#define MTK_MT6785_GPU_BIN_6783		3

static inline int mtk_mt6785_efuse_gpu_speedbin_pp(void *context, const char *id,
						   int index, unsigned int offset,
						   void *data, size_t bytes)
{
	u8 *val = data;

	switch (val[0]) {
	case 0xe0:
	case 0x07:
		val[0] = BIT(MTK_MT6785_GPU_BIN_6785U);
		break;
	case 0xc0:
	case 0x03:
	case 0x20:
	case 0x04:
		val[0] = BIT(MTK_MT6785_GPU_BIN_6785);
		break;
	case 0x80:
	case 0x01:
	case 0x40:
	case 0x02:
		val[0] = BIT(MTK_MT6785_GPU_BIN_6783);
		break;
	default:
		val[0] = BIT(MTK_MT6785_GPU_BIN_6785T);
		break;
	}

	return 0;
}

static inline void mtk_mt6785_efuse_fixup_dt_cell_info(struct nvmem_device *nvmem,
						       struct nvmem_cell_info *cell)
{
	size_t sz = strlen(cell->name);

	if (strncmp(cell->name, "gpu-speedbin", min(sz, strlen("gpu-speedbin"))) == 0)
		cell->read_post_process = mtk_mt6785_efuse_gpu_speedbin_pp;
}

#endif /* __MTK_EFUSE_H */
