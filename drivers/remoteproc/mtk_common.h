/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __RPROC_MTK_COMMON_H
#define __RPROC_MTK_COMMON_H

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_scp.h>

#define MT8183_SW_RSTN			0x0
#define MT8183_SW_RSTN_BIT		BIT(0)
#define MT8183_SCP_TO_HOST		0x1C
/* the core's own state, for a boot that never answers (vendor scp_reg.h:11-40) */
#define MT8183_SCP_DEBUG_SP		0xA8
#define MT8183_SCP_DEBUG_LR		0xAC
#define MT8183_SCP_DEBUG_PSP		0xB0
#define MT8183_SCP_DEBUG_PC		0xB4
#define MT8183_SCP_SLEEP_STATUS		0x114
/* "AP side write 1 to clear SCP to SPM reg" (vendor scp_ipi.c:75-79) */
#define MT8183_SCP_TO_SPM		0x20
#define MT8183_SCP_BUS_RESOURCE		0x08
#define MT8183_SCP_IPC_INT_BIT		BIT(0)
#define MT8183_SCP_WDT_INT_BIT		BIT(8)
#define MT8183_HOST_TO_SCP		0x28
#define MT8183_HOST_IPC_INT_BIT		BIT(0)
#define MT8183_WDT_CFG			0x84
#define MT8183_SCP_CLK_SW_SEL		0x4000
#define MT8183_SCP_CLK_DIV_SEL		0x4024
#define MT8183_SCP_SRAM_PDN		0x402C
#define MT8183_SCP_L1_SRAM_PD		0x4080
#define MT8183_SCP_TCM_TAIL_SRAM_PD	0x4094

#define MT8183_SCP_CACHE_SEL(x)		(0x14000 + (x) * 0x3000)
#define MT8183_SCP_CACHE_CON		MT8183_SCP_CACHE_SEL(0)
#define MT8183_SCP_DCACHE_CON		MT8183_SCP_CACHE_SEL(1)
#define MT8183_SCP_CACHESIZE_8KB	BIT(8)
#define MT8183_SCP_CACHE_CON_WAYEN	BIT(10)

#define MT8186_SCP_L1_SRAM_PD_P1	0x40B0
#define MT8186_SCP_L1_SRAM_PD_p2	0x40B4

#define MT8192_L2TCM_SRAM_PD_0		0x10C0
#define MT8192_L2TCM_SRAM_PD_1		0x10C4
#define MT8192_L2TCM_SRAM_PD_2		0x10C8
#define MT8192_L1TCM_SRAM_PDN		0x102C
#define MT8192_CPU0_SRAM_PD		0x1080

#define MT8192_SCP2APMCU_IPC_SET	0x4080
#define MT8192_SCP2APMCU_IPC_CLR	0x4084
#define MT8192_SCP_IPC_INT_BIT		BIT(0)
#define MT8192_SCP2SPM_IPC_CLR		0x4094
#define MT8192_GIPC_IN_SET		0x4098
#define MT8192_HOST_IPC_INT_BIT		BIT(0)
#define MT8195_CORE1_HOST_IPC_INT_BIT	BIT(4)

#define MT8192_CORE0_SW_RSTN_CLR	0x10000
#define MT8192_CORE0_SW_RSTN_SET	0x10004
#define MT8192_CORE0_MEM_ATT_PREDEF	0x10008
#define MT8192_CORE0_WDT_IRQ		0x10030
#define MT8192_CORE0_WDT_CFG		0x10034

#define MT8195_SYS_STATUS		0x4004
#define MT8195_CORE0_WDT		BIT(16)
#define MT8195_CORE1_WDT		BIT(17)

#define MT8195_L1TCM_SRAM_PDN_RESERVED_RSI_BITS		GENMASK(7, 4)

#define MT8195_CPU1_SRAM_PD			0x1084
#define MT8195_SSHUB2APMCU_IPC_SET		0x4088
#define MT8195_SSHUB2APMCU_IPC_CLR		0x408C
#define MT8195_CORE1_SW_RSTN_CLR		0x20000
#define MT8195_CORE1_SW_RSTN_SET		0x20004
#define MT8195_CORE1_MEM_ATT_PREDEF		0x20008
#define MT8195_CORE1_WDT_IRQ			0x20030
#define MT8195_CORE1_WDT_CFG			0x20034

#define MT8195_SEC_CTRL				0x85000
#define MT8195_CORE_OFFSET_ENABLE_D		BIT(13)
#define MT8195_CORE_OFFSET_ENABLE_I		BIT(12)
/*
 * The SCP subsystem's module reset and sleep protection, in infracfg (0x10001000) - the vendor's
 * scpsys node's first window (vendor mt6785.dts:636-644) and, in mainline, the infracfg syscon.
 * Offsets from the vendor scp_reg.h:96-112; bits 30/31 of the sleep protect are SCP->infra and
 * SCP->audio, bits 3 and 10 of the module reset are SCP and SCP-secure.
 */
#define MT6785_INFRACFG_MODULE_RESET_SET	0x140
#define MT6785_INFRACFG_MODULE_RESET_CLR	0x144
#define MT6785_INFRACFG_MODULE_RESET_STATUS	0x148
#define MT6785_INFRACFG_SLP_PROT_SET		0x220
/* a free cfg scratch word (GENERAL_REG0, cfg + 0x50, scp_reg.h:39-42) and the value put in it */
#define MT6785_SCP_PROBE_REG		0x50
#define MT6785_SCP_PROBE_VALUE		0x5a5a1234
/* the two candidates for a core that loses its bus, and the probe's timing (scp_reg.h) */
#define MT6785_SPM_ULPOSC_CON		0x440
/* topckgen's clock-config register the vendor touches for the pwrap oscillator gate: CK_CFG_6 */
#define MT6785_TOP_CK_CFG_6		0x80
/* the SCP's own mux: select bits 16-18, power-down bit 23 (clk-mt6785-top.c:473) */
#define MT6785_TOP_CK_CFG_0		0x20
/* the reserved DRAM pool the SCP may use, and where its logger ring can live */
#define MT6785_SCP_DRAM_PHYS		0x50000000
#define MT6785_SCP_WDT_REG		0x84
#define MT6785_SCP_WDT_CTRL		0x88
#define MT6785_SCP_WDT_SP		0xb8
#define MT6785_SCP_WDT_LR		0xbc
#define MT6785_SCP_WDT_PSP		0xc0
#define MT6785_SCP_WDT_PC		0xc4
#define MT6785_SCP_CPU_SLEEP_STATUS	0x114
#define MT6785_SCP_SLEEP_DEBUG_REG	0x28	/* in the clock control window */
#define MT6785_SCP_CLK_BLOCK		0x4000
/* the SCP's own clock block, offsets from the vendor's scp_reg.h:84-89 */
#define MT6785_SCP_CLK_SW_SEL		0x0
#define MT6785_SCP_CLK_ENABLE		0x4
#define MT6785_SCP_CLK_CTRL_24		0x24
#define MT6785_SCP_PROBE_STEP_US	100000
#define MT6785_SCP_PROBE_POLLS		20
#define MT6785_SCP_PROBE_REPORT		1
/* the AP's awake handshake with the SCP, through the SPM (scp_reg.h:114-120) */
#define MT6785_INFRACFG_IRQ_SET			0xB14
#define MT6785_INFRACFG_IRQ_CLEAR		0xB18
#define MT6785_INFRACFG_SLP_PROT_STAT		0x224
#define MT6785_SCP_RESET_BIT			BIT(3)
#define MT6785_SCP_SEC_RESET_BIT		BIT(10)
#define MT6785_SCP_TO_INFRA_BIT			BIT(30)
#define MT6785_SCP_TO_AUDIO_BIT			BIT(31)
#define MT6785_SCP_SYS_RESET_TIMEOUT_US		1000000

/* the vendor's SCP clock-gate registers inside topckgen (clk-mt6785.c:91-92) */
#define MT6785_SCP_CLK_CFG_0			0x200
#define MT6785_SCP_CLK_CFG_1			0x210

#define MT8195_L2TCM_OFFSET_RANGE_0_LOW		0x850b0
#define MT8195_L2TCM_OFFSET_RANGE_0_HIGH	0x850b4
#define MT8195_L2TCM_OFFSET			0x850d0

#define SCP_FW_VER_LEN			32

struct scp_run {
	u32 signaled;
	s8 fw_ver[SCP_FW_VER_LEN];
	u32 dec_capability;
	u32 enc_capability;
	wait_queue_head_t wq;
};

struct scp_ipi_desc {
	/* For protecting handler. */
	struct mutex lock;
	scp_ipi_handler_t handler;
	void *priv;
};

struct mtk_scp;

struct mtk_scp_sizes_data {
	size_t max_dram_size;
	size_t ipi_share_buffer_size;
};

struct mtk_scp_of_data {
	int (*scp_clk_get)(struct mtk_scp *scp);
	int (*scp_before_load)(struct mtk_scp *scp);
	int (*scp_after_load)(struct mtk_scp *scp,
			      const struct firmware *fw);
	void (*scp_irq_handler)(struct mtk_scp *scp);
	void (*scp_reset_assert)(struct mtk_scp *scp);
	void (*scp_reset_deassert)(struct mtk_scp *scp);
	void (*scp_stop)(struct mtk_scp *scp);
	/* a boot that times out is otherwise silent: let the SoC report where the core is */
	void (*scp_dump_regs)(struct mtk_scp *scp);
	/* ack an IPI the way the firmware expects, where it differs from the default */
	void (*scp_ipi_ack)(struct mtk_scp *scp);
	/* SoC clock gates that are not modelled as clocks and have to be opened before loading */
	int (*scp_clocks_on)(struct mtk_scp *scp);
	/*
	 * Leave the block untouched when a boot fails, so its state can still be read
	 * afterwards: the dump is taken first, and a following /dev/mem read then shows
	 * what the hardware really holds instead of what a cleanup wrote.
	 */
	bool scp_keep_state_on_error;
	/*
	 * The loader on this SoC brings the SCP up itself, so a block that answers is a
	 * running subsystem: it is adopted instead of being reset and reloaded.
	 */
	bool scp_maybe_preloaded;
	/* the SoC's loader leaves its loader and region info in the SRAM, which is worth keeping */
	bool scp_use_lk_images;
	/* the firmware file is the vendor's MTK image (records), not an ELF */
	bool scp_mtk_image;
	void *(*scp_da_to_va)(struct mtk_scp *scp, u64 da, size_t len);

	u32 host_to_scp_reg;
	u32 host_to_scp_int_bit;

	size_t ipi_buf_offset;
	/* offset of the payload inside a share object; 0 means the default 8 */
	u32 ipi_buf_data_offset;
	/*
	 * IPI id the firmware uses to report that it is up and running; 0 is the
	 * SCP_IPI_INIT of the MT8183/MT8192 firmware family, and the MT6785
	 * sensor-hub firmware uses its own numbering (12, IPI_SCP_A_READY).
	 */
	u32 ipi_init_id;
	const struct mtk_scp_sizes_data *scp_sizes;
};

#define MTK_SCP_MAX_PLL			6

struct mtk_scp_of_cluster {
	void __iomem *reg_base;
	/* the size of the window reg_base maps, for the bring-up's bounds checks */
	resource_size_t cfg_size;
	/*
	 * The clock sources the SCP's firmware may switch to once it runs its own DVFS: the
	 * vendor lists the crystal and six PLL taps as the SCP mux's parents and enables them
	 * from the AP side (vendor mt6785.dts:655-669, scp_dvfs.c:627-643), so they are held
	 * before the release and dropped after the ready IPI.
	 */
	struct clk *pll[MTK_SCP_MAX_PLL];
	unsigned int pll_num;
	void __iomem *l1tcm_base;
	size_t l1tcm_size;
	phys_addr_t l1tcm_phys;
	struct list_head mtk_scp_list;
	/* Prevent concurrent operations of this structure and L2TCM power control. */
	struct mutex cluster_lock;
	u32 l2tcm_refcnt;
};

struct mtk_scp {
	struct device *dev;
	struct rproc *rproc;
	struct clk *clk;
	/* the SPM block, for the oscillator state the PCM firmware owns */
	struct regmap *spm_regmap;
	/* the loaded image, to tell the firmware's own strings from its DRAM log */
	const u8 *image;
	size_t image_size;
	/* topckgen, for the frequency meter the firmware's calibration drives */
	struct regmap *topckgen_regmap;
	void __iomem *sram_base;
	size_t sram_size;
	phys_addr_t sram_phys;

	const struct mtk_scp_of_data *data;

	struct mtk_share_obj __iomem *recv_buf;
	struct mtk_share_obj __iomem *send_buf;
	struct scp_run run;
	/* To prevent multiple ipi_send run concurrently. */
	struct mutex send_lock;
	struct scp_ipi_desc ipi_desc[SCP_IPI_MAX];
	bool ipi_id_ack[SCP_IPI_MAX];
	wait_queue_head_t ack_wq;
	u8 *share_buf;

	void *cpu_addr;
	dma_addr_t dma_addr;

	/* the loader started this SCP and the driver adopted it instead of loading its own image */
	bool preloaded;
	/* LK's own loader and region information were still in the SRAM and are being used */
	bool lk_images;
	/* LK's copies in its reserved DRAM region were used as the image to load */
	bool lk_staged;
	/* where the parsed image's dram record was staged, relative to the carve-out */
	u32 mtk_dram_ofs;
	/* SCP_A_TO_HOST interrupts (GIC SPI 201): the only readiness signal trusted here */
	u32 to_host_irqs;
	/* the SCP's L1 cache control block (two instances 0x3000 apart) */
	void __iomem *l1c_base;
	/* LK's reserved region: its mapping, physical base and size */
	void __iomem *lk_base;
	phys_addr_t lk_phys;
	resource_size_t lk_size;

	struct rproc_subdev *rpmsg_subdev;

	struct list_head elem;
	struct mtk_scp_of_cluster *cluster;
};

/**
 * struct mtk_share_obj - SRAM buffer shared with AP and SCP
 *
 * @id:		IPI id
 * @len:	share buffer length
 * @share_buf:	share buffer data
 */
struct mtk_share_obj {
	u32 id;
	u32 len;
	u8 *share_buf;
};

void scp_memcpy_aligned(void __iomem *dst, const void *src, unsigned int len);
void scp_ipi_lock(struct mtk_scp *scp, u32 id);
void scp_ipi_unlock(struct mtk_scp *scp, u32 id);

#endif
