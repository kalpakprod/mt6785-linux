// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2019 MediaTek Inc.

#include <asm/barrier.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/unaligned.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_scp.h>
#include <linux/rpmsg/mtk_rpmsg.h>
#include <linux/string.h>

#include "mtk_common.h"
#include "remoteproc_internal.h"

#define SECTION_NAME_IPI_BUFFER ".ipi_buffer"

/**
 * scp_get() - get a reference to SCP.
 *
 * @pdev:	the platform device of the module requesting SCP platform
 *		device for using SCP API.
 *
 * Return: Return NULL if failed.  otherwise reference to SCP.
 **/
struct mtk_scp *scp_get(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *scp_node;
	struct platform_device *scp_pdev;
	struct mtk_scp *scp;

	scp_node = of_parse_phandle(dev->of_node, "mediatek,scp", 0);
	if (!scp_node) {
		dev_err(dev, "can't get SCP node\n");
		return NULL;
	}

	scp_pdev = of_find_device_by_node(scp_node);
	of_node_put(scp_node);

	if (WARN_ON(!scp_pdev)) {
		dev_err(dev, "SCP pdev failed\n");
		return NULL;
	}

	scp = platform_get_drvdata(scp_pdev);
	if (!scp) {
		put_device(&scp_pdev->dev);
		return NULL;
	}

	return scp;
}
EXPORT_SYMBOL_GPL(scp_get);

/**
 * scp_put() - "free" the SCP
 *
 * @scp:	mtk_scp structure from scp_get().
 **/
void scp_put(struct mtk_scp *scp)
{
	put_device(scp->dev);
}
EXPORT_SYMBOL_GPL(scp_put);

static void scp_wdt_handler(struct mtk_scp *scp, u32 scp_to_host)
{
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;
	struct mtk_scp *scp_node;

	dev_err(scp->dev, "SCP watchdog timeout! 0x%x", scp_to_host);

	/* report watchdog timeout to all cores */
	list_for_each_entry(scp_node, &scp_cluster->mtk_scp_list, elem)
		rproc_report_crash(scp_node->rproc, RPROC_WATCHDOG);
}

static void scp_init_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_scp *scp = priv;
	struct scp_run *run = data;

	/*
	 * The MT8183/MT8192 family answers with a full struct scp_run, whose first
	 * word says whether the firmware really is ready. MT6785's sensor-hub
	 * firmware answers on its own id (see ipi_init_id) with a single word: the
	 * size of the image it booted (vendor scp_helper.c:493-497 reads exactly
	 * that). A payload shorter than the struct is therefore not a truncated
	 * message, it is the whole "I am up" signal, and there is nothing to copy
	 * out of it.
	 */
	if (len >= sizeof(*run)) {
		scp->run.signaled = run->signaled;
		strscpy(scp->run.fw_ver, run->fw_ver, SCP_FW_VER_LEN);
		scp->run.dec_capability = run->dec_capability;
		scp->run.enc_capability = run->enc_capability;
	} else {
		scp->run.signaled = len > 0;
	}

	wake_up_interruptible(&scp->run.wq);
}

static void scp_ipi_handler(struct mtk_scp *scp)
{
	struct mtk_share_obj __iomem *rcv_obj = scp->recv_buf;
	struct scp_ipi_desc *ipi_desc = scp->ipi_desc;
	scp_ipi_handler_t handler;
	u32 id = readl(&rcv_obj->id);
	u32 len = readl(&rcv_obj->len);
	const struct mtk_scp_sizes_data *scp_sizes;

	scp_sizes = scp->data->scp_sizes;
	if (len > scp_sizes->ipi_share_buffer_size) {
		dev_err(scp->dev, "ipi message too long (len %d, max %zd)", len,
			scp_sizes->ipi_share_buffer_size);
		return;
	}
	if (id >= SCP_IPI_MAX) {
		dev_err(scp->dev, "No such ipi id = %d\n", id);
		return;
	}

	scp_ipi_lock(scp, id);
	handler = ipi_desc[id].handler;
	if (!handler) {
		dev_err(scp->dev, "No handler for ipi id = %d\n", id);
		scp_ipi_unlock(scp, id);
		return;
	}

	memcpy_fromio(scp->share_buf,
		      (void __iomem *)rcv_obj +
		      (scp->data->ipi_buf_data_offset ?: sizeof(u32) * 2), len);
	memset(&scp->share_buf[len], 0, scp_sizes->ipi_share_buffer_size - len);
	handler(scp->share_buf, len, ipi_desc[id].priv);
	scp_ipi_unlock(scp, id);

	if (scp->data->scp_ipi_ack)
		scp->data->scp_ipi_ack(scp);

	scp->ipi_id_ack[id] = true;
	wake_up(&scp->ack_wq);
}

static int scp_elf_read_ipi_buf_addr(struct mtk_scp *scp,
				     const struct firmware *fw,
				     size_t *offset);

static int scp_ipi_init(struct mtk_scp *scp, const struct firmware *fw)
{
	int ret;
	size_t buf_sz, offset;
	size_t share_buf_offset;
	const struct mtk_scp_sizes_data *scp_sizes;

	/* read the ipi buf addr from FW itself first */
	ret = scp_elf_read_ipi_buf_addr(scp, fw, &offset);
	if (ret) {
		/* use default ipi buf addr if the FW doesn't have it */
		offset = scp->data->ipi_buf_offset;
		if (!offset)
			return ret;
	}
	dev_info(scp->dev, "IPI buf addr %#010zx\n", offset);

	/* Make sure IPI buffer fits in the L2TCM range assigned to this core */
	buf_sz = sizeof(*scp->recv_buf) + sizeof(*scp->send_buf);

	if (scp->sram_size < buf_sz + offset) {
		dev_err(scp->dev, "IPI buffer does not fit in SRAM.\n");
		return -EOVERFLOW;
	}

	scp_sizes = scp->data->scp_sizes;
	scp->recv_buf = (struct mtk_share_obj __iomem *)
			(scp->sram_base + offset);
	share_buf_offset = sizeof(scp->recv_buf->id)
		+ (scp->data->ipi_buf_data_offset ?: sizeof(u32) * 2)
		+ scp_sizes->ipi_share_buffer_size;
	scp->send_buf = (struct mtk_share_obj __iomem *)
			(scp->sram_base + offset + share_buf_offset);
	memset_io(scp->recv_buf, 0, share_buf_offset);
	memset_io(scp->send_buf, 0, share_buf_offset);

	return 0;
}

static void mt8183_scp_reset_assert(struct mtk_scp *scp)
{
	u32 val;

	val = readl(scp->cluster->reg_base + MT8183_SW_RSTN);
	val &= ~MT8183_SW_RSTN_BIT;
	writel(val, scp->cluster->reg_base + MT8183_SW_RSTN);
}

static void mt8183_scp_reset_deassert(struct mtk_scp *scp)
{
	u32 val;

	val = readl(scp->cluster->reg_base + MT8183_SW_RSTN);
	val |= MT8183_SW_RSTN_BIT;
	writel(val, scp->cluster->reg_base + MT8183_SW_RSTN);
}

static void mt8192_scp_reset_assert(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8192_CORE0_SW_RSTN_SET);
}

static void mt8192_scp_reset_deassert(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8192_CORE0_SW_RSTN_CLR);
}

static void mt8195_scp_c1_reset_assert(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8195_CORE1_SW_RSTN_SET);
}

static void mt8195_scp_c1_reset_deassert(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8195_CORE1_SW_RSTN_CLR);
}

static void mt8183_scp_irq_handler(struct mtk_scp *scp)
{
	u32 scp_to_host;

	scp_to_host = readl(scp->cluster->reg_base + MT8183_SCP_TO_HOST);
	if (scp_to_host & MT8183_SCP_IPC_INT_BIT)
		scp_ipi_handler(scp);
	else
		scp_wdt_handler(scp, scp_to_host);

	/* SCP won't send another interrupt until we set SCP_TO_HOST to 0. */
	writel(MT8183_SCP_IPC_INT_BIT | MT8183_SCP_WDT_INT_BIT,
	       scp->cluster->reg_base + MT8183_SCP_TO_HOST);
}

static void mt8192_scp_irq_handler(struct mtk_scp *scp)
{
	u32 scp_to_host;

	scp_to_host = readl(scp->cluster->reg_base + MT8192_SCP2APMCU_IPC_SET);

	if (scp_to_host & MT8192_SCP_IPC_INT_BIT) {
		scp_ipi_handler(scp);

		/*
		 * SCP won't send another interrupt until we clear
		 * MT8192_SCP2APMCU_IPC.
		 */
		writel(MT8192_SCP_IPC_INT_BIT,
		       scp->cluster->reg_base + MT8192_SCP2APMCU_IPC_CLR);
	} else {
		scp_wdt_handler(scp, scp_to_host);
		writel(1, scp->cluster->reg_base + MT8192_CORE0_WDT_IRQ);
	}
}

static void mt8195_scp_irq_handler(struct mtk_scp *scp)
{
	u32 scp_to_host;

	scp_to_host = readl(scp->cluster->reg_base + MT8192_SCP2APMCU_IPC_SET);

	if (scp_to_host & MT8192_SCP_IPC_INT_BIT) {
		scp_ipi_handler(scp);
	} else {
		u32 reason = readl(scp->cluster->reg_base + MT8195_SYS_STATUS);

		if (reason & MT8195_CORE0_WDT)
			writel(1, scp->cluster->reg_base + MT8192_CORE0_WDT_IRQ);

		if (reason & MT8195_CORE1_WDT)
			writel(1, scp->cluster->reg_base + MT8195_CORE1_WDT_IRQ);

		scp_wdt_handler(scp, reason);
	}

	writel(scp_to_host, scp->cluster->reg_base + MT8192_SCP2APMCU_IPC_CLR);
}

static void mt8195_scp_c1_irq_handler(struct mtk_scp *scp)
{
	u32 scp_to_host;

	scp_to_host = readl(scp->cluster->reg_base + MT8195_SSHUB2APMCU_IPC_SET);

	if (scp_to_host & MT8192_SCP_IPC_INT_BIT)
		scp_ipi_handler(scp);

	writel(scp_to_host, scp->cluster->reg_base + MT8195_SSHUB2APMCU_IPC_CLR);
}

static irqreturn_t scp_irq_handler(int irq, void *priv)
{
	struct mtk_scp *scp = priv;
	int ret;

	ret = clk_enable(scp->clk);
	if (ret) {
		dev_err(scp->dev, "failed to enable clocks\n");
		return IRQ_NONE;
	}

	scp->data->scp_irq_handler(scp);

	clk_disable(scp->clk);

	return IRQ_HANDLED;
}

static int scp_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void __iomem *ptr;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
			phdr->p_type, da, memsz, filesz);

		if (phdr->p_type != PT_LOAD)
			continue;
		if (!filesz)
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = (void __iomem *)rproc_da_to_va(rproc, da, memsz, NULL);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		scp_memcpy_aligned(ptr, elf_data + phdr->p_offset, filesz);
	}

	return ret;
}

static int scp_elf_read_ipi_buf_addr(struct mtk_scp *scp,
				     const struct firmware *fw,
				     size_t *offset)
{
	struct elf32_hdr *ehdr;
	struct elf32_shdr *shdr, *shdr_strtab;
	int i;
	const u8 *elf_data = fw->data;
	const char *strtab;

	ehdr = (struct elf32_hdr *)elf_data;
	shdr = (struct elf32_shdr *)(elf_data + ehdr->e_shoff);
	shdr_strtab = shdr + ehdr->e_shstrndx;
	strtab = (const char *)(elf_data + shdr_strtab->sh_offset);

	for (i = 0; i < ehdr->e_shnum; i++, shdr++) {
		if (strcmp(strtab + shdr->sh_name,
			   SECTION_NAME_IPI_BUFFER) == 0) {
			*offset = shdr->sh_addr;
			return 0;
		}
	}

	return -ENOENT;
}

static int mt8183_scp_clk_get(struct mtk_scp *scp)
{
	struct device *dev = scp->dev;
	int ret = 0;

	scp->clk = devm_clk_get(dev, "main");
	if (IS_ERR(scp->clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(scp->clk);
	}

	return ret;
}

static int mt8192_scp_clk_get(struct mtk_scp *scp)
{
	return mt8183_scp_clk_get(scp);
}

static int mt8195_scp_clk_get(struct mtk_scp *scp)
{
	scp->clk = NULL;

	return 0;
}

/* MT6785 SCP: the same generation as MT8183, with a staged firmware image */
#define MT6785_SCP_REGION_INFO_OFFSET	0x400
#define MT6785_SCP_LOADER_SLOT		0x2000
#define MT6785_SCP_DRAM_OFS		0x200000
/* the L1 cache configuration LK puts in the region info it leaves (pl_lk.txt:2409-2410) */
#define MT6785_SCP_L1C_CON		0x10d
#define MT6785_SCP_IPI_BUF_OFS		0x5c0
#define MT6785_SCP_IPI_BUF_DATA_OFS	16
#define MT6785_SCP_IPI_BUF_SIZE		272
/* the firmware's "ready" IPI: IPI_SCP_A_READY, vendor scp_cm4.h:17-40 */
#define MT6785_SCP_IPI_READY		12
#define MT6785_SCP_DRAM_SIZE		0x400000

/*
 * The vendor's region info, which the loader reads from TCM + 0x400
 * (scp_helper.h:18 and the struct at :84-105 of the 4.19 SCP driver).
 */
struct mt6785_scp_region_info {
	u32 ap_loader_start;
	u32 ap_loader_size;
	u32 ap_firmware_start;
	u32 ap_firmware_size;
	u32 ap_dram_start;
	u32 ap_dram_size;
	u32 ap_dram_backup_start;
	u32 struct_size;
	u32 scp_log_thru_ap_uart;
	u32 task_context_ptr;
	u32 il1c_con;
	u32 dl1c_con;
	u32 scpctl;
	u32 ap_params_start;
};

static int mt6785_scp_clocks_on(struct mtk_scp *scp);
static int mt6785_scp_subsys_reset(struct mtk_scp *scp);
static int mt6785_scp_slp_prot_clear(struct mtk_scp *scp);
/*
 * True when the SCP is already up. On MT6785 the loader starts it from the scp_a partition, so its
 * clock block holds SCP_CLK_SW_SEL's programmed value (0x201 on this phone, measured on a kernel
 * without any SCP driver) and the region information at TCM + 0x400 is the loader's, not ours.
 * Resetting and reloading a subsystem in that state is what audio20 did - and the block stopped
 * answering the AP as soon as the core ran (every register read 0x1e1, the stale last value).
 */
static struct regmap *mt6785_infracfg(struct mtk_scp *scp);

/*
 * "The SCP is already running" takes more than a readable clock block: audio22 adopted a core
 * that audio20 had just stopped, because cfg + 0x4000 still read 0x201 (the loader's clock
 * select) while sw_rstn was 0 and the TCM was empty. The vendor's own test is the core's state,
 * not the clock: reset_scp() reads SCP_SLEEP_STATUS_REG and GPR_CM4_A_REBOOT before touching the
 * core (scp_helper.c:588-610). Here the equivalent pair is the release bit this driver writes
 * and the vector table the core would fetch: released, and a stack pointer that lives inside
 * the TCM.
 */
static bool mt6785_scp_is_up(struct mtk_scp *scp)
{
	u32 rstn = readl(scp->cluster->reg_base + MT8183_SW_RSTN);
	u32 sp = readl(scp->sram_base);
	u32 pc = readl(scp->sram_base + 4);

	return (rstn & BIT(0)) && sp && sp < scp->sram_size && pc && pc < scp->sram_size;
}

static void mt6785_scp_state(struct mtk_scp *scp, const char *when);

static int mt6785_scp_before_load(struct mtk_scp *scp)
{
	struct mtk_scp_of_cluster *cluster = scp->cluster;
	u32 val;
	int ret;

	/* without these the whole SCP window reads back as zeros (audio16) */
	ret = mt6785_scp_clocks_on(scp);
	if (ret)
		return ret;

	/*
	 * The subsystem's module reset, and the SRAM switched on one block at a time, in the
	 * vendor's order (scp_awake.c:222-283). The per-block ramp matters: scp_enable_sram()
	 * writes 0x7fffffff, 0x3fffffff, ... 0 - "enable 1 block per time" - not one 0.
	 */
	ret = mt6785_scp_subsys_reset(scp);
	if (ret)
		return ret;

	/* Clear any stale SCP to host interrupt */
	writel(MT8183_SCP_IPC_INT_BIT, cluster->reg_base + MT8183_SCP_TO_HOST);

	/*
	 * No write to the SCP clock block at cfg + 0x4000 here, and that is deliberate:
	 * mt8183's before_load writes CLK_SW_SEL there, but the vendor's MT6785 driver only
	 * ever *reads* it (scp_excep.c:227 and :317, both dumps; the block is ioremapped for
	 * those dumps at scp_helper.c:1764-1769) - the SCP's clock is chosen by topckgen's
	 * mux, which the clock framework owns. Writing 0 into an undocumented mux select is
	 * how a block ends up reading as zeros from the AP, which is what audio17 measured.
	 */

	/*
	 * The SRAM: 0xffffffff is "all banks powered down" (the base kernel reads exactly that
	 * at cfg + 0x402c/0x4080/0x4094), so the PD bits are active high. The vendor's
	 * scp_enable_sram() releases the banks one at a time and only then the L1/TCM-tail bits
	 * (scp_awake.c:196-214).
	 */
	dev_info(scp->dev, "sram pdn before: %#x %#x %#x\n",
		 readl(cluster->reg_base + MT8183_SCP_SRAM_PDN),
		 readl(cluster->reg_base + MT8183_SCP_L1_SRAM_PD),
		 readl(cluster->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD));

	for (val = 0xffffffff; val; val >>= 1)
		writel(val, cluster->reg_base + MT8183_SCP_SRAM_PDN);
	writel(0x0, cluster->reg_base + MT8183_SCP_SRAM_PDN);
	writel(0x0, cluster->reg_base + MT8183_SCP_L1_SRAM_PD);
	writel(0x0, cluster->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD);

	/*
	 * LK loads the loader into the TCM and writes the region information before Linux runs
	 * (pl_lk.txt:2305-2418: the loader image, the firmware staged at 0x9fc00800 and the
	 * region info). If the SRAM kept it, that state is the vendor's own: keep it and only
	 * release the core (reset_scp(), scp_helper.c:569-620), instead of loading our image
	 * over LK's. The test is the region information itself - a loader size inside the TCM
	 * and non-zero firmware and DRAM pointers.
	 */
	if (scp->data->scp_use_lk_images) {
		struct mt6785_scp_region_info lk;
		int i;

		memcpy_fromio(&lk, scp->sram_base + MT6785_SCP_REGION_INFO_OFFSET,
			      sizeof(lk));
		dev_info(scp->dev, "LK region info: loader %#x+%#x firmware %#x+%#x dram %#x+%#x l1c %#x/%#x\n",
			 lk.ap_loader_start, lk.ap_loader_size,
			 lk.ap_firmware_start, lk.ap_firmware_size,
			 lk.ap_dram_start, lk.ap_dram_size, lk.il1c_con, lk.dl1c_con);
		scp->lk_images = lk.ap_loader_size && lk.ap_loader_size <= 0x2000 &&
				 lk.ap_firmware_start && lk.ap_firmware_size &&
				 lk.ap_dram_start;
		for (i = 0; scp->lk_images && i < MIN(lk.ap_loader_size, 0x1000) / 4; i++)
			if (!readl(scp->sram_base + 4 * i))
				scp->lk_images = false;
		if (scp->lk_images)
			dev_info(scp->dev, "LK's loader and region information survived: keeping them, releasing the core only\n");
	}

	dev_info(scp->dev, "sram pdn after: %#x %#x %#x, clk_sw %#x, tcm[0] %#x\n",
		 readl(cluster->reg_base + MT8183_SCP_SRAM_PDN),
		 readl(cluster->reg_base + MT8183_SCP_L1_SRAM_PD),
		 readl(cluster->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD),
		 readl(cluster->reg_base + MT8183_SCP_CLK_SW_SEL),
		 readl(scp->sram_base));

	return 0;
}

/* the filesz of the ELF32 segment loaded at @paddr, or 0 */
static u32 mt6785_elf_segment_size(const struct firmware *fw, u32 paddr)
{
	const u8 *buf = fw->data;
	u16 phnum, phentsize;
	u32 phoff;
	int i;

	if (fw->size < 52 || memcmp(buf, "\177ELF", 4) || buf[4] != 1)
		return 0;

	phoff = get_unaligned_le32(buf + 28);
	phnum = get_unaligned_le16(buf + 44);
	phentsize = get_unaligned_le16(buf + 42);
	if (phoff > fw->size || phentsize != 32)
		return 0;

	for (i = 0; i < phnum; i++) {
		const u8 *ph = buf + phoff + i * phentsize;

		if (ph + 32 > buf + fw->size)
			return 0;
		if (get_unaligned_le32(ph) != 1)	/* PT_LOAD */
			continue;
		if (get_unaligned_le32(ph + 12) == paddr)	/* p_paddr */
			return get_unaligned_le32(ph + 16);	/* p_filesz */
	}

	return 0;
}

/* the bytes the repacker staged above `base`: the record's tail and its DRAM part */
static u32 mt6785_elf_dram_size(const struct firmware *fw, phys_addr_t base)
{
	const u8 *buf = fw->data;
	u16 phnum, phentsize;
	u32 phoff, total = 0;
	int i;

	if (fw->size < 52 || memcmp(buf, "\177ELF", 4) || buf[4] != 1)
		return 0;

	phoff = get_unaligned_le32(buf + 28);
	phnum = get_unaligned_le16(buf + 44);
	phentsize = get_unaligned_le16(buf + 42);
	if (phoff > fw->size || phentsize != 32)
		return 0;

	for (i = 0; i < phnum; i++) {
		const u8 *ph = buf + phoff + i * phentsize;

		if (ph + 32 > buf + fw->size)
			return 0;
		if (get_unaligned_le32(ph) != 1)	/* PT_LOAD */
			continue;
		if (get_unaligned_le32(ph + 12) < base)	/* p_paddr */
			continue;
		total += get_unaligned_le32(ph + 16);	/* p_filesz */
	}

	return total;
}

/*
 * A boot that never answers leaves nothing behind: the image loaded, the registers
 * were writable, and then two seconds of silence. The core keeps its own state in
 * the SCP block (vendor scp_reg.h:11-40, dumped by scp_A_dump_regs() there), and
 * it distinguishes the two cases that matter: a PC/SP that never moved means the
 * core is not executing at all (clock, reset), a PC inside or just past the image
 * means it started and faulted (image layout, wrong load base).
 */
/*
 * After an IPI the firmware has to be told that the AP is done with it, or the
 * core stays in its IPI-busy idle state and stops sending more (vendor comment in
 * scp_ipi.c:64-79, which then writes 1 to SCP_TO_SPM). The default upstream path
 * relies on clearing SCP_TO_HOST alone, which this handler already does; MT6785's
 * firmware also wants this word.
 */
static void mt6785_scp_ipi_ack(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8183_SCP_TO_SPM);
}

static void mt6785_scp_dump_regs(struct mtk_scp *scp)
{
	void __iomem *cfg = scp->cluster->reg_base;
	u32 rstn, tcm0, tcm1;

	dev_err(scp->dev, "core PC %#x LR %#x SP %#x PSP %#x\n",
		readl(cfg + MT8183_SCP_DEBUG_PC), readl(cfg + MT8183_SCP_DEBUG_LR),
		readl(cfg + MT8183_SCP_DEBUG_SP), readl(cfg + MT8183_SCP_DEBUG_PSP));
	dev_err(scp->dev, "sleep %#x to_host %#x gipc_in %#x\n",
		readl(cfg + MT8183_SCP_SLEEP_STATUS),
		readl(cfg + MT8183_SCP_TO_HOST), readl(cfg + MT8183_HOST_TO_SCP));

	/*
	 * The two questions the core's own state cannot answer by itself: is the block out of
	 * reset, and did the loader reach TCM at all. All-zero PC/LR/SP/PSP means either "the core
	 * never ran" or "it ran and read a zero vector table", and these two reads separate them:
	 * SW_RSTN bit 0 is what this driver writes to release the core, and TCM word 0 is the
	 * vector's initial stack pointer (0x538 for the vendor's loader record at TCM 0 - while a
	 * zero there means the ELF segment never landed or the TCM is not powered).
	 */
	rstn = readl(cfg + MT8183_SW_RSTN);
	tcm0 = readl(scp->sram_base);
	tcm1 = readl(scp->sram_base + 4);
	/*
	 * scp_reset_deassert() writes bit 0 of SW_RSTN to 1, so 1 is the released state: a 0 there
	 * means the write did not land, and together with zero TCM words it means the whole window
	 * reads back as zeros - the SCP's clock gates, not the core, are then the fault.
	 */
	dev_err(scp->dev, "sw_rstn %#x (bit0=1 released, 0 = write never landed)\n",
		rstn);
	dev_err(scp->dev, "TCM[0] %#x TCM[4] %#x\n", tcm0, tcm1);
	if (!(tcm0 | tcm1))
		dev_err(scp->dev, "TCM holds no vector table: load or TCM power is the fault, not the core\n");
	else if (tcm0 >= scp->sram_size)
		dev_err(scp->dev, "TCM[0] = %#x is not a stack pointer inside the TCM\n", tcm0);
}

static int mt6785_scp_after_load(struct mtk_scp *scp,
				 const struct firmware *fw)
{
	struct mt6785_scp_region_info ri = { 0 };
	u32 fw_size, dram_size;
	int ret;

	/*
	 * The loader copies ap_firmware_size bytes from ap_firmware_start to a fixed
	 * *core address*, 0x800, and then takes the stack pointer and the reset vector
	 * from the first two words it copied (read out of the loader itself, at TCM +
	 * 0x1e0: `ldr r0, =0x800` after fetching ap_firmware_start/size from the region
	 * info). So what the repacker stages at dma_addr + LOADER_SLOT has to begin with
	 * the firmware's own vector table, and its size is whatever TCM can hold from
	 * 0x800 - the rest of a longer record, and the record's DRAM part, are staged
	 * above dma_addr + DRAM_OFS and announced in ap_dram_start/size.
	 */
	fw_size = mt6785_elf_segment_size(fw, scp->dma_addr + MT6785_SCP_LOADER_SLOT);
	dram_size = mt6785_elf_dram_size(fw, scp->dma_addr + MT6785_SCP_DRAM_OFS);
	if (!fw_size || !dram_size) {
		dev_err(scp->dev, "firmware lacks the staged segments\n");
		return -EINVAL;
	}

	ri.ap_loader_start = scp->dma_addr;
	ri.ap_loader_size = MT6785_SCP_LOADER_SLOT;
	ri.ap_firmware_start = scp->dma_addr + MT6785_SCP_LOADER_SLOT;
	ri.ap_firmware_size = fw_size;
	ri.ap_dram_start = scp->dma_addr + MT6785_SCP_DRAM_OFS;
	ri.ap_dram_size = dram_size;
	/*
	 * The two cache configurations LK writes into the region info it leaves behind
	 * (pl_lk.txt:2409-2410: Il1c_con 0x10d, Dl1c_con 0x10d). They are part of the contract
	 * with the loader, and leaving them zero - as this struct was - tells the firmware its
	 * caches are configured in a way nobody chose.
	 */
	ri.il1c_con = MT6785_SCP_L1C_CON;
	ri.dl1c_con = MT6785_SCP_L1C_CON;
	ri.struct_size = sizeof(ri);

	memcpy_toio(scp->sram_base + MT6785_SCP_REGION_INFO_OFFSET, &ri,
		    sizeof(ri));
	/* make sure the loader sees the region info before it is released */
	wmb();

	dev_dbg(scp->dev, "staged loader %#x firmware %#x dram %#x at %pa\n",
		ri.ap_loader_size, fw_size, dram_size, &scp->dma_addr);

	mt6785_scp_state(scp, "after the ELF copy and the region info");

	/* the images are in place, so the buses need no protection any more */
	ret = mt6785_scp_slp_prot_clear(scp);
	if (!ret)
		mt6785_scp_state(scp, "after the sleep-protect release");

	return ret;
}

static int mt8183_scp_before_load(struct mtk_scp *scp)
{
	/* Clear SCP to host interrupt */
	writel(MT8183_SCP_IPC_INT_BIT, scp->cluster->reg_base + MT8183_SCP_TO_HOST);

	/* Reset clocks before loading FW */
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_CLK_SW_SEL);
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_CLK_DIV_SEL);

	/* Initialize TCM before loading FW. */
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_L1_SRAM_PD);
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD);

	/* Turn on the power of SCP's SRAM before using it. */
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_SRAM_PDN);

	/*
	 * Set I-cache and D-cache size before loading SCP FW.
	 * SCP SRAM logical address may change when cache size setting differs.
	 */
	writel(MT8183_SCP_CACHE_CON_WAYEN | MT8183_SCP_CACHESIZE_8KB,
	       scp->cluster->reg_base + MT8183_SCP_CACHE_CON);
	writel(MT8183_SCP_CACHESIZE_8KB, scp->cluster->reg_base + MT8183_SCP_DCACHE_CON);

	return 0;
}

static void scp_sram_power_on(void __iomem *addr, u32 reserved_mask)
{
	int i;

	for (i = 31; i >= 0; i--)
		writel(GENMASK(i, 0) & ~reserved_mask, addr);
	writel(0, addr);
}

static void scp_sram_power_off(void __iomem *addr, u32 reserved_mask)
{
	int i;

	writel(0, addr);
	for (i = 0; i < 32; i++)
		writel(GENMASK(i, 0) & ~reserved_mask, addr);
}

static int mt8186_scp_before_load(struct mtk_scp *scp)
{
	/* Clear SCP to host interrupt */
	writel(MT8183_SCP_IPC_INT_BIT, scp->cluster->reg_base + MT8183_SCP_TO_HOST);

	/* Reset clocks before loading FW */
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_CLK_SW_SEL);
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_CLK_DIV_SEL);

	/* Turn on the power of SCP's SRAM before using it. Enable 1 block per time*/
	scp_sram_power_on(scp->cluster->reg_base + MT8183_SCP_SRAM_PDN, 0);

	/* Initialize TCM before loading FW. */
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_L1_SRAM_PD);
	writel(0x0, scp->cluster->reg_base + MT8183_SCP_TCM_TAIL_SRAM_PD);
	writel(0x0, scp->cluster->reg_base + MT8186_SCP_L1_SRAM_PD_P1);
	writel(0x0, scp->cluster->reg_base + MT8186_SCP_L1_SRAM_PD_p2);

	/*
	 * Set I-cache and D-cache size before loading SCP FW.
	 * SCP SRAM logical address may change when cache size setting differs.
	 */
	writel(MT8183_SCP_CACHE_CON_WAYEN | MT8183_SCP_CACHESIZE_8KB,
	       scp->cluster->reg_base + MT8183_SCP_CACHE_CON);
	writel(MT8183_SCP_CACHESIZE_8KB, scp->cluster->reg_base + MT8183_SCP_DCACHE_CON);

	return 0;
}

static int mt8188_scp_l2tcm_on(struct mtk_scp *scp)
{
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	mutex_lock(&scp_cluster->cluster_lock);

	if (scp_cluster->l2tcm_refcnt == 0) {
		/* clear SPM interrupt, SCP2SPM_IPC_CLR */
		writel(0xff, scp->cluster->reg_base + MT8192_SCP2SPM_IPC_CLR);

		/* Power on L2TCM */
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN, 0);
	}

	scp_cluster->l2tcm_refcnt += 1;

	mutex_unlock(&scp_cluster->cluster_lock);

	return 0;
}

static int mt8188_scp_before_load(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8192_CORE0_SW_RSTN_SET);

	mt8188_scp_l2tcm_on(scp);

	scp_sram_power_on(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* enable MPU for all memory regions */
	writel(0xff, scp->cluster->reg_base + MT8192_CORE0_MEM_ATT_PREDEF);

	return 0;
}

static int mt8188_scp_c1_before_load(struct mtk_scp *scp)
{
	u32 sec_ctrl;
	struct mtk_scp *scp_c0;
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	scp->data->scp_reset_assert(scp);

	mt8188_scp_l2tcm_on(scp);

	scp_sram_power_on(scp->cluster->reg_base + MT8195_CPU1_SRAM_PD, 0);

	/* enable MPU for all memory regions */
	writel(0xff, scp->cluster->reg_base + MT8195_CORE1_MEM_ATT_PREDEF);

	/*
	 * The L2TCM_OFFSET_RANGE and L2TCM_OFFSET shift the destination address
	 * on SRAM when SCP core 1 accesses SRAM.
	 *
	 * This configuration solves booting the SCP core 0 and core 1 from
	 * different SRAM address because core 0 and core 1 both boot from
	 * the head of SRAM by default. this must be configured before boot SCP core 1.
	 *
	 * The value of L2TCM_OFFSET_RANGE is from the viewpoint of SCP core 1.
	 * When SCP core 1 issues address within the range (L2TCM_OFFSET_RANGE),
	 * the address will be added with a fixed offset (L2TCM_OFFSET) on the bus.
	 * The shift action is tranparent to software.
	 */
	writel(0, scp->cluster->reg_base + MT8195_L2TCM_OFFSET_RANGE_0_LOW);
	writel(scp->sram_size, scp->cluster->reg_base + MT8195_L2TCM_OFFSET_RANGE_0_HIGH);

	scp_c0 = list_first_entry(&scp_cluster->mtk_scp_list, struct mtk_scp, elem);
	writel(scp->sram_phys - scp_c0->sram_phys, scp->cluster->reg_base + MT8195_L2TCM_OFFSET);

	/* enable SRAM offset when fetching instruction and data */
	sec_ctrl = readl(scp->cluster->reg_base + MT8195_SEC_CTRL);
	sec_ctrl |= MT8195_CORE_OFFSET_ENABLE_I | MT8195_CORE_OFFSET_ENABLE_D;
	writel(sec_ctrl, scp->cluster->reg_base + MT8195_SEC_CTRL);

	return 0;
}

static int mt8192_scp_before_load(struct mtk_scp *scp)
{
	/* clear SPM interrupt, SCP2SPM_IPC_CLR */
	writel(0xff, scp->cluster->reg_base + MT8192_SCP2SPM_IPC_CLR);

	writel(1, scp->cluster->reg_base + MT8192_CORE0_SW_RSTN_SET);

	/* enable SRAM clock */
	scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
	scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
	scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
	scp_sram_power_on(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN, 0);
	scp_sram_power_on(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* enable MPU for all memory regions */
	writel(0xff, scp->cluster->reg_base + MT8192_CORE0_MEM_ATT_PREDEF);

	return 0;
}

static int mt8195_scp_l2tcm_on(struct mtk_scp *scp)
{
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	mutex_lock(&scp_cluster->cluster_lock);

	if (scp_cluster->l2tcm_refcnt == 0) {
		/* clear SPM interrupt, SCP2SPM_IPC_CLR */
		writel(0xff, scp->cluster->reg_base + MT8192_SCP2SPM_IPC_CLR);

		/* Power on L2TCM */
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
		scp_sram_power_on(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN,
				  MT8195_L1TCM_SRAM_PDN_RESERVED_RSI_BITS);
	}

	scp_cluster->l2tcm_refcnt += 1;

	mutex_unlock(&scp_cluster->cluster_lock);

	return 0;
}

static int mt8195_scp_before_load(struct mtk_scp *scp)
{
	writel(1, scp->cluster->reg_base + MT8192_CORE0_SW_RSTN_SET);

	mt8195_scp_l2tcm_on(scp);

	scp_sram_power_on(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* enable MPU for all memory regions */
	writel(0xff, scp->cluster->reg_base + MT8192_CORE0_MEM_ATT_PREDEF);

	return 0;
}

static int mt8195_scp_c1_before_load(struct mtk_scp *scp)
{
	u32 sec_ctrl;
	struct mtk_scp *scp_c0;
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	scp->data->scp_reset_assert(scp);

	mt8195_scp_l2tcm_on(scp);

	scp_sram_power_on(scp->cluster->reg_base + MT8195_CPU1_SRAM_PD, 0);

	/* enable MPU for all memory regions */
	writel(0xff, scp->cluster->reg_base + MT8195_CORE1_MEM_ATT_PREDEF);

	/*
	 * The L2TCM_OFFSET_RANGE and L2TCM_OFFSET shift the destination address
	 * on SRAM when SCP core 1 accesses SRAM.
	 *
	 * This configuration solves booting the SCP core 0 and core 1 from
	 * different SRAM address because core 0 and core 1 both boot from
	 * the head of SRAM by default. this must be configured before boot SCP core 1.
	 *
	 * The value of L2TCM_OFFSET_RANGE is from the viewpoint of SCP core 1.
	 * When SCP core 1 issues address within the range (L2TCM_OFFSET_RANGE),
	 * the address will be added with a fixed offset (L2TCM_OFFSET) on the bus.
	 * The shift action is tranparent to software.
	 */
	writel(0, scp->cluster->reg_base + MT8195_L2TCM_OFFSET_RANGE_0_LOW);
	writel(scp->sram_size, scp->cluster->reg_base + MT8195_L2TCM_OFFSET_RANGE_0_HIGH);

	scp_c0 = list_first_entry(&scp_cluster->mtk_scp_list, struct mtk_scp, elem);
	writel(scp->sram_phys - scp_c0->sram_phys, scp->cluster->reg_base + MT8195_L2TCM_OFFSET);

	/* enable SRAM offset when fetching instruction and data */
	sec_ctrl = readl(scp->cluster->reg_base + MT8195_SEC_CTRL);
	sec_ctrl |= MT8195_CORE_OFFSET_ENABLE_I | MT8195_CORE_OFFSET_ENABLE_D;
	writel(sec_ctrl, scp->cluster->reg_base + MT8195_SEC_CTRL);

	return 0;
}

static int scp_load(struct rproc *rproc, const struct firmware *fw)
{
	struct mtk_scp *scp = rproc->priv;
	struct device *dev = scp->dev;
	int ret;

	ret = clk_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		return ret;
	}

	if (scp->data->scp_use_lk_images) {
		/* the SRAM has to be powered before it can be looked at */
		ret = scp->data->scp_before_load(scp);
		if (ret)
			goto leave;
		if (scp->lk_images) {
			dev_info(dev, "using the loader and region information LK left in the SRAM\n");
			ret = 0;
			goto leave;
		}
	}

	/* Hold SCP in reset while loading FW. */
	scp->data->scp_reset_assert(scp);

	ret = scp->data->scp_before_load(scp);
	if (ret < 0)
		goto leave;

	ret = scp_elf_load_segments(rproc, fw);
	if (ret)
		goto leave;

	if (scp->data->scp_after_load)
		ret = scp->data->scp_after_load(scp, fw);
leave:
	clk_disable(scp->clk);

	return ret;
}

/*
 * The core was booted by the loader (RPROC_DETACHED): there is nothing to load and nothing to
 * release, only the IPI side to have ready - and it already is, registered in scp_rproc_init().
 * The firmware's own ready IPI was sent when the loader started it, so the run state is marked
 * here rather than waited for.
 */
static int mtk_scp_attach(struct rproc *rproc)
{
	struct mtk_scp *scp = rproc->priv;

	dev_info(scp->dev, "attached to the SCP the loader started\n");
	scp->run.signaled = true;
	return 0;
}

static int scp_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct mtk_scp *scp = rproc->priv;
	struct device *dev = scp->dev;
	int ret;

	ret = clk_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		return ret;
	}

	ret = scp_ipi_init(scp, fw);
	clk_disable(scp->clk);
	return ret;
}

static int scp_start(struct rproc *rproc)
{
	struct mtk_scp *scp = rproc->priv;
	struct device *dev = scp->dev;
	struct scp_run *run = &scp->run;
	int ret;

	ret = clk_enable(scp->clk);
	if (ret) {
		dev_err(dev, "failed to enable clocks\n");
		return ret;
	}

	run->signaled = false;

	scp->data->scp_reset_deassert(scp);

	if (scp->data->scp_dump_regs == mt6785_scp_dump_regs) {
		/* the steps have to be told apart: this is right after the release write */
		mt6785_scp_state(scp, "right after the core-release write");
		mt6785_scp_state(scp, "a moment later");
	}

	ret = wait_event_interruptible_timeout(
					run->wq,
					run->signaled,
					msecs_to_jiffies(2000));

	if (ret == 0) {
		dev_err(dev, "wait SCP initialization timeout!\n");
		if (scp->data->scp_dump_regs)
			scp->data->scp_dump_regs(scp);
		ret = -ETIME;
		goto stop;
	}
	if (ret == -ERESTARTSYS) {
		dev_err(dev, "wait SCP interrupted by a signal!\n");
		goto stop;
	}

	clk_disable(scp->clk);
	if (run->fw_ver[0])
		dev_info(dev, "SCP is ready. FW version %s\n", run->fw_ver);
	else
		dev_info(dev, "SCP is ready.\n");

	return 0;

stop:
	if (!scp->data->scp_keep_state_on_error) {
		scp->data->scp_reset_assert(scp);
	} else {
		dev_info(dev, "leaving the SCP block untouched for inspection\n");
		mt6785_scp_state(scp, "at the end of the failed start");
	}
	clk_disable(scp->clk);
	return ret;
}

static void *mt8183_scp_da_to_va(struct mtk_scp *scp, u64 da, size_t len)
{
	int offset;
	const struct mtk_scp_sizes_data *scp_sizes;

	scp_sizes = scp->data->scp_sizes;
	if (da < scp->sram_size) {
		offset = da;
		if (offset >= 0 && (offset + len) <= scp->sram_size)
			return (void __force *)scp->sram_base + offset;
	} else if (scp_sizes->max_dram_size) {
		offset = da - scp->dma_addr;
		if (offset >= 0 && (offset + len) <= scp_sizes->max_dram_size)
			return scp->cpu_addr + offset;
	}

	return NULL;
}

static void *mt8192_scp_da_to_va(struct mtk_scp *scp, u64 da, size_t len)
{
	int offset;
	const struct mtk_scp_sizes_data *scp_sizes;

	scp_sizes = scp->data->scp_sizes;
	if (da >= scp->sram_phys &&
	    (da + len) <= scp->sram_phys + scp->sram_size) {
		offset = da - scp->sram_phys;
		return (void __force *)scp->sram_base + offset;
	}

	/* optional memory region */
	if (scp->cluster->l1tcm_size &&
	    da >= scp->cluster->l1tcm_phys &&
	    (da + len) <= scp->cluster->l1tcm_phys + scp->cluster->l1tcm_size) {
		offset = da - scp->cluster->l1tcm_phys;
		return (void __force *)scp->cluster->l1tcm_base + offset;
	}

	/* optional memory region */
	if (scp_sizes->max_dram_size &&
	    da >= scp->dma_addr &&
	    (da + len) <= scp->dma_addr + scp_sizes->max_dram_size) {
		offset = da - scp->dma_addr;
		return scp->cpu_addr + offset;
	}

	return NULL;
}

static void *scp_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct mtk_scp *scp = rproc->priv;

	return scp->data->scp_da_to_va(scp, da, len);
}

/*
 * Stopping an SCP the loader started means stopping a firmware this driver does not own, so the
 * subsystem is cycled the way the vendor's recovery does it (scp_sys_full_reset(), scp_awake.c:
 * 222-283): the module reset in infracfg is taken and released again, and the core is held in
 * reset through the AP-side bit. A later rproc_boot then loads the image from the firmware file,
 * which is what a restart after a crash has to do.
 */
static void mt8183_scp_stop(struct mtk_scp *scp);

static void mt6785_scp_stop_full(struct mtk_scp *scp)
{
	struct regmap *infracfg = mt6785_infracfg(scp);
	u32 val;

	mt8183_scp_stop(scp);

	if (IS_ERR(infracfg))
		return;

	/* hold the subsystem in reset, then release the hold so a reload starts clean */
	regmap_read(infracfg, MT6785_INFRACFG_MODULE_RESET_SET, &val);
	regmap_write(infracfg, MT6785_INFRACFG_MODULE_RESET_SET,
		     val | MT6785_SCP_RESET_BIT);
	usleep_range(50, 60);
	regmap_read(infracfg, MT6785_INFRACFG_MODULE_RESET_CLR, &val);
	regmap_write(infracfg, MT6785_INFRACFG_MODULE_RESET_CLR,
		     val | MT6785_SCP_RESET_BIT);

	dev_info(scp->dev, "SCP stopped and its subsystem reset left released for a reload\n");
}

static void mt8183_scp_stop(struct mtk_scp *scp)
{
	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8183_WDT_CFG);
}

static void mt8188_scp_l2tcm_off(struct mtk_scp *scp)
{
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	mutex_lock(&scp_cluster->cluster_lock);

	if (scp_cluster->l2tcm_refcnt > 0)
		scp_cluster->l2tcm_refcnt -= 1;

	if (scp_cluster->l2tcm_refcnt == 0) {
		/* Power off L2TCM */
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN, 0);
	}

	mutex_unlock(&scp_cluster->cluster_lock);
}

static void mt8188_scp_stop(struct mtk_scp *scp)
{
	mt8188_scp_l2tcm_off(scp);

	scp_sram_power_off(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8192_CORE0_WDT_CFG);
}

static void mt8188_scp_c1_stop(struct mtk_scp *scp)
{
	mt8188_scp_l2tcm_off(scp);

	/* Power off CPU SRAM */
	scp_sram_power_off(scp->cluster->reg_base + MT8195_CPU1_SRAM_PD, 0);

	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8195_CORE1_WDT_CFG);
}

static void mt8192_scp_stop(struct mtk_scp *scp)
{
	/* Disable SRAM clock */
	scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
	scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
	scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
	scp_sram_power_off(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN, 0);
	scp_sram_power_off(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8192_CORE0_WDT_CFG);
}

static void mt8195_scp_l2tcm_off(struct mtk_scp *scp)
{
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;

	mutex_lock(&scp_cluster->cluster_lock);

	if (scp_cluster->l2tcm_refcnt > 0)
		scp_cluster->l2tcm_refcnt -= 1;

	if (scp_cluster->l2tcm_refcnt == 0) {
		/* Power off L2TCM */
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_0, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_1, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L2TCM_SRAM_PD_2, 0);
		scp_sram_power_off(scp->cluster->reg_base + MT8192_L1TCM_SRAM_PDN,
				   MT8195_L1TCM_SRAM_PDN_RESERVED_RSI_BITS);
	}

	mutex_unlock(&scp_cluster->cluster_lock);
}

static void mt8195_scp_stop(struct mtk_scp *scp)
{
	mt8195_scp_l2tcm_off(scp);

	scp_sram_power_off(scp->cluster->reg_base + MT8192_CPU0_SRAM_PD, 0);

	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8192_CORE0_WDT_CFG);
}

static void mt8195_scp_c1_stop(struct mtk_scp *scp)
{
	mt8195_scp_l2tcm_off(scp);

	/* Power off CPU SRAM */
	scp_sram_power_off(scp->cluster->reg_base + MT8195_CPU1_SRAM_PD, 0);

	/* Disable SCP watchdog */
	writel(0, scp->cluster->reg_base + MT8195_CORE1_WDT_CFG);
}

static int scp_stop(struct rproc *rproc)
{
	struct mtk_scp *scp = rproc->priv;
	int ret;

	ret = clk_enable(scp->clk);
	if (ret) {
		dev_err(scp->dev, "failed to enable clocks\n");
		return ret;
	}

	scp->data->scp_reset_assert(scp);
	scp->data->scp_stop(scp);
	clk_disable(scp->clk);

	return 0;
}

static int scp_prepare(struct rproc *rproc)
{
	struct mtk_scp *scp = rproc->priv;

	return clk_prepare(scp->clk);
}

static int scp_unprepare(struct rproc *rproc)
{
	struct mtk_scp *scp = rproc->priv;

	clk_unprepare(scp->clk);
	return 0;
}

static const struct rproc_ops scp_ops = {
	.prepare	= scp_prepare,
	.unprepare	= scp_unprepare,
	.start		= scp_start,
	.stop		= scp_stop,
	.attach		= mtk_scp_attach,
	.load		= scp_load,
	.da_to_va	= scp_da_to_va,
	.parse_fw	= scp_parse_fw,
	.sanity_check	= rproc_elf_sanity_check,
};

/**
 * scp_get_device() - get device struct of SCP
 *
 * @scp:	mtk_scp structure
 **/
struct device *scp_get_device(struct mtk_scp *scp)
{
	return scp->dev;
}
EXPORT_SYMBOL_GPL(scp_get_device);

/**
 * scp_get_rproc() - get rproc struct of SCP
 *
 * @scp:	mtk_scp structure
 **/
struct rproc *scp_get_rproc(struct mtk_scp *scp)
{
	return scp->rproc;
}
EXPORT_SYMBOL_GPL(scp_get_rproc);

/**
 * scp_get_vdec_hw_capa() - get video decoder hardware capability
 *
 * @scp:	mtk_scp structure
 *
 * Return: video decoder hardware capability
 **/
unsigned int scp_get_vdec_hw_capa(struct mtk_scp *scp)
{
	return scp->run.dec_capability;
}
EXPORT_SYMBOL_GPL(scp_get_vdec_hw_capa);

/**
 * scp_get_venc_hw_capa() - get video encoder hardware capability
 *
 * @scp:	mtk_scp structure
 *
 * Return: video encoder hardware capability
 **/
unsigned int scp_get_venc_hw_capa(struct mtk_scp *scp)
{
	return scp->run.enc_capability;
}
EXPORT_SYMBOL_GPL(scp_get_venc_hw_capa);

/**
 * scp_mapping_dm_addr() - Mapping SRAM/DRAM to kernel virtual address
 *
 * @scp:	mtk_scp structure
 * @mem_addr:	SCP views memory address
 *
 * Mapping the SCP's SRAM address /
 * DMEM (Data Extended Memory) memory address /
 * Working buffer memory address to
 * kernel virtual address.
 *
 * Return: Return ERR_PTR(-EINVAL) if mapping failed,
 * otherwise the mapped kernel virtual address
 **/
void *scp_mapping_dm_addr(struct mtk_scp *scp, u32 mem_addr)
{
	void *ptr;

	ptr = scp_da_to_va(scp->rproc, mem_addr, 0, NULL);
	if (!ptr)
		return ERR_PTR(-EINVAL);

	return ptr;
}
EXPORT_SYMBOL_GPL(scp_mapping_dm_addr);

static int scp_map_memory_region(struct mtk_scp *scp)
{
	int ret;
	const struct mtk_scp_sizes_data *scp_sizes;

	ret = of_reserved_mem_device_init(scp->dev);

	/* reserved memory is optional. */
	if (ret == -ENODEV) {
		dev_info(scp->dev, "skipping reserved memory initialization.");
		return 0;
	}

	if (ret) {
		dev_err(scp->dev, "failed to assign memory-region: %d\n", ret);
		return -ENOMEM;
	}

	/* Reserved SCP code size */
	scp_sizes = scp->data->scp_sizes;
	scp->cpu_addr = dma_alloc_coherent(scp->dev, scp_sizes->max_dram_size,
					   &scp->dma_addr, GFP_KERNEL);
	if (!scp->cpu_addr)
		return -ENOMEM;

	return 0;
}

static void scp_unmap_memory_region(struct mtk_scp *scp)
{
	const struct mtk_scp_sizes_data *scp_sizes;

	scp_sizes = scp->data->scp_sizes;
	if (scp_sizes->max_dram_size == 0)
		return;

	dma_free_coherent(scp->dev, scp_sizes->max_dram_size, scp->cpu_addr,
			  scp->dma_addr);
	of_reserved_mem_device_release(scp->dev);
}

static int scp_register_ipi(struct platform_device *pdev, u32 id,
			    ipi_handler_t handler, void *priv)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	return scp_ipi_register(scp, id, handler, priv);
}

static void scp_unregister_ipi(struct platform_device *pdev, u32 id)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	scp_ipi_unregister(scp, id);
}

static int scp_send_ipi(struct platform_device *pdev, u32 id, const void *buf,
			unsigned int len, unsigned int wait)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);

	return scp_ipi_send(scp, id, buf, len, wait);
}

static struct mtk_rpmsg_info mtk_scp_rpmsg_info = {
	.send_ipi = scp_send_ipi,
	.register_ipi = scp_register_ipi,
	.unregister_ipi = scp_unregister_ipi,
	.ns_ipi_id = SCP_IPI_NS_SERVICE,
};

static void scp_add_rpmsg_subdev(struct mtk_scp *scp)
{
	scp->rpmsg_subdev =
		mtk_rpmsg_create_rproc_subdev(to_platform_device(scp->dev),
					      &mtk_scp_rpmsg_info);
	if (scp->rpmsg_subdev)
		rproc_add_subdev(scp->rproc, scp->rpmsg_subdev);
}

static void scp_remove_rpmsg_subdev(struct mtk_scp *scp)
{
	if (scp->rpmsg_subdev) {
		rproc_remove_subdev(scp->rproc, scp->rpmsg_subdev);
		mtk_rpmsg_destroy_rproc_subdev(scp->rpmsg_subdev);
		scp->rpmsg_subdev = NULL;
	}
}

/**
 * scp_get_default_fw_path() - Get default SCP firmware path
 * @dev:     SCP Device
 * @core_id: SCP Core number
 *
 * This function generates a path based on the following format:
 *     mediatek/(soc_model)/scp(_cX).img; for multi-core or
 *     mediatek/(soc_model)/scp.img for single core SCP HW
 *
 * Return: A devm allocated string containing the full path to
 *         a SCP firmware or an error pointer
 */
static const char *scp_get_default_fw_path(struct device *dev, int core_id)
{
	struct device_node *np = core_id < 0 ? dev->of_node : dev->parent->of_node;
	const char *compatible, *soc;
	char scp_fw_file[7];
	int ret;

	/* Use only the first compatible string */
	ret = of_property_read_string_index(np, "compatible", 0, &compatible);
	if (ret)
		return ERR_PTR(ret);

	/* If the compatible string's length is implausible bail out early */
	if (strlen(compatible) < strlen("mediatek,mtXXXX-scp"))
		return ERR_PTR(-EINVAL);

	/* If the compatible string starts with "mediatek,mt" assume that it's ok */
	if (!str_has_prefix(compatible, "mediatek,mt"))
		return ERR_PTR(-EINVAL);

	if (core_id >= 0)
		ret = snprintf(scp_fw_file, sizeof(scp_fw_file), "scp_c%d", core_id);
	else
		ret = snprintf(scp_fw_file, sizeof(scp_fw_file), "scp");
	if (ret >= sizeof(scp_fw_file))
		return ERR_PTR(-ENAMETOOLONG);

	/* Not using strchr here, as strlen of a const gets optimized by compiler */
	soc = &compatible[strlen("mediatek,")];

	return devm_kasprintf(dev, GFP_KERNEL, "mediatek/%.*s/%s.img",
			      (int)strlen("mtXXXX"), soc, scp_fw_file);
}

static struct mtk_scp *scp_rproc_init(struct platform_device *pdev,
				      struct mtk_scp_of_cluster *scp_cluster,
				      const struct mtk_scp_of_data *of_data,
				      int core_id)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_scp *scp;
	struct rproc *rproc;
	struct resource *res;
	const char *fw_name;
	int ret, i;
	const struct mtk_scp_sizes_data *scp_sizes;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret) {
		fw_name = scp_get_default_fw_path(dev, core_id);
		if (IS_ERR(fw_name)) {
			dev_err(dev, "Cannot get firmware path: %ld\n", PTR_ERR(fw_name));
			return ERR_CAST(fw_name);
		}
	}

	rproc = devm_rproc_alloc(dev, np->name, &scp_ops, fw_name, sizeof(*scp));
	if (!rproc) {
		dev_err(dev, "unable to allocate remoteproc\n");
		return ERR_PTR(-ENOMEM);
	}

	scp = rproc->priv;
	scp->rproc = rproc;
	scp->dev = dev;
	scp->data = of_data;
	scp->cluster = scp_cluster;
	platform_set_drvdata(pdev, scp);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	scp->sram_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(scp->sram_base)) {
		dev_err(dev, "Failed to parse and map sram memory\n");
		return ERR_CAST(scp->sram_base);
	}

	scp->sram_size = resource_size(res);
	scp->sram_phys = res->start;

	ret = scp->data->scp_clk_get(scp);
	if (ret)
		return ERR_PTR(ret);

	ret = scp_map_memory_region(scp);
	if (ret)
		return ERR_PTR(ret);

	mutex_init(&scp->send_lock);
	for (i = 0; i < SCP_IPI_MAX; i++)
		mutex_init(&scp->ipi_desc[i].lock);

	/* register SCP initialization IPI */
	ret = scp_ipi_register(scp, scp->data->ipi_init_id,
			       scp_init_ipi_handler, scp);
	if (ret) {
		dev_err(dev, "Failed to register the init IPI (%u)\n",
			scp->data->ipi_init_id);
		goto release_dev_mem;
	}

	scp_sizes = scp->data->scp_sizes;
	scp->share_buf = kzalloc(scp_sizes->ipi_share_buffer_size, GFP_KERNEL);
	if (!scp->share_buf) {
		dev_err(dev, "Failed to allocate IPI share buffer\n");
		ret = -ENOMEM;
		goto release_dev_mem;
	}

	init_waitqueue_head(&scp->run.wq);
	init_waitqueue_head(&scp->ack_wq);

	scp_add_rpmsg_subdev(scp);

	ret = devm_request_threaded_irq(dev, platform_get_irq(pdev, 0), NULL,
					scp_irq_handler, IRQF_ONESHOT,
					pdev->name, scp);

	if (ret)
		goto remove_subdev;

	return scp;

remove_subdev:
	scp_remove_rpmsg_subdev(scp);
	scp_ipi_unregister(scp, SCP_IPI_INIT);
	kfree(scp->share_buf);
	scp->share_buf = NULL;
release_dev_mem:
	scp_unmap_memory_region(scp);
	for (i = 0; i < SCP_IPI_MAX; i++)
		mutex_destroy(&scp->ipi_desc[i].lock);
	mutex_destroy(&scp->send_lock);

	return ERR_PTR(ret);
}

static void scp_free(struct mtk_scp *scp)
{
	int i;

	scp_remove_rpmsg_subdev(scp);
	scp_ipi_unregister(scp, SCP_IPI_INIT);
	kfree(scp->share_buf);
	scp->share_buf = NULL;
	scp_unmap_memory_region(scp);
	for (i = 0; i < SCP_IPI_MAX; i++)
		mutex_destroy(&scp->ipi_desc[i].lock);
	mutex_destroy(&scp->send_lock);
}

static int scp_add_single_core(struct platform_device *pdev,
			       struct mtk_scp_of_cluster *scp_cluster)
{
	struct device *dev = &pdev->dev;
	struct list_head *scp_list = &scp_cluster->mtk_scp_list;
	struct mtk_scp *scp;
	int ret;

	scp = scp_rproc_init(pdev, scp_cluster, of_device_get_match_data(dev), -1);
	if (IS_ERR(scp))
		return PTR_ERR(scp);

	if (scp->data->scp_maybe_preloaded) {
		/*
		 * The loader on this SoC loads the images and may or may not release the core.
		 * A released core with a vector table in the TCM is a running firmware, and
		 * remoteproc's own mechanism for that is RPROC_DETACHED plus an .attach op:
		 * rproc_boot() then attaches instead of loading our image over it. Anything
		 * else - and audio22's adopted-then-stopped core was anything else - takes
		 * the staging path.
		 */
		ret = clk_enable(scp->clk);
		if (ret) {
			scp_free(scp);
			return ret;
		}
		scp->preloaded = mt6785_scp_is_up(scp);
		if (scp->preloaded)
			scp->rproc->state = RPROC_DETACHED;
		dev_info(dev, "%s (sw_rstn %#x, TCM %#x/%#x)\n",
			 scp->preloaded ? "the loader left the SCP running: attaching" :
					  "the loader left the SCP stopped: staging our image",
			 readl(scp->cluster->reg_base + MT8183_SW_RSTN),
			 readl(scp->sram_base), readl(scp->sram_base + 4));
		clk_disable(scp->clk);
	}

	ret = rproc_add(scp->rproc);
	if (ret) {
		dev_err(dev, "Failed to add rproc\n");
		scp_free(scp);
		return ret;
	}

	list_add_tail(&scp->elem, scp_list);

	return 0;
}

static int scp_add_multi_core(struct platform_device *pdev,
			      struct mtk_scp_of_cluster *scp_cluster)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev_of_node(dev);
	struct platform_device *cpdev;
	struct list_head *scp_list = &scp_cluster->mtk_scp_list;
	const struct mtk_scp_of_data **cluster_of_data;
	struct mtk_scp *scp, *temp;
	int core_id = 0;
	int ret;

	cluster_of_data = (const struct mtk_scp_of_data **)of_device_get_match_data(dev);

	for_each_available_child_of_node_scoped(np, child) {
		if (!cluster_of_data[core_id]) {
			ret = -EINVAL;
			dev_err(dev, "Not support core %d\n", core_id);
			goto init_fail;
		}

		cpdev = of_find_device_by_node(child);
		if (!cpdev) {
			ret = -ENODEV;
			dev_err(dev, "Not found platform device for core %d\n", core_id);
			goto init_fail;
		}

		scp = scp_rproc_init(cpdev, scp_cluster, cluster_of_data[core_id], core_id);
		put_device(&cpdev->dev);
		if (IS_ERR(scp)) {
			ret = PTR_ERR(scp);
			dev_err(dev, "Failed to initialize core %d rproc\n", core_id);
			goto init_fail;
		}

		ret = rproc_add(scp->rproc);
		if (ret) {
			dev_err(dev, "Failed to add rproc of core %d\n", core_id);
			scp_free(scp);
			goto init_fail;
		}

		list_add_tail(&scp->elem, scp_list);
		core_id++;
	}

	/*
	 * Here we are setting the platform device for @pdev to the last @scp that was
	 * created, which is needed because (1) scp_rproc_init() is calling
	 * platform_set_drvdata() on the child platform devices and (2) we need a handle to
	 * the cluster list in scp_remove().
	 */
	platform_set_drvdata(pdev, scp);

	return 0;

init_fail:
	list_for_each_entry_safe_reverse(scp, temp, scp_list, elem) {
		list_del(&scp->elem);
		rproc_del(scp->rproc);
		scp_free(scp);
	}

	return ret;
}

static bool scp_is_single_core(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev_of_node(dev);
	struct device_node *child;
	int num_cores = 0;

	for_each_child_of_node(np, child)
		if (of_device_is_compatible(child, "mediatek,scp-core"))
			num_cores++;

	return num_cores < 2;
}

static int scp_cluster_init(struct platform_device *pdev, struct mtk_scp_of_cluster *scp_cluster)
{
	int ret;

	if (scp_is_single_core(pdev))
		ret = scp_add_single_core(pdev, scp_cluster);
	else
		ret = scp_add_multi_core(pdev, scp_cluster);

	return ret;
}

static const struct of_device_id scp_core_match[] = {
	{ .compatible = "mediatek,scp-core" },
	{}
};

static int scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_scp_of_cluster *scp_cluster;
	struct resource *res;
	int ret;

	scp_cluster = devm_kzalloc(dev, sizeof(*scp_cluster), GFP_KERNEL);
	if (!scp_cluster)
		return -ENOMEM;

	scp_cluster->reg_base = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(scp_cluster->reg_base))
		return dev_err_probe(dev, PTR_ERR(scp_cluster->reg_base),
				     "Failed to parse and map cfg memory\n");

	/* l1tcm is an optional memory region */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "l1tcm");
	if (res) {
		scp_cluster->l1tcm_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(scp_cluster->l1tcm_base))
			return dev_err_probe(dev, PTR_ERR(scp_cluster->l1tcm_base),
					     "Failed to map l1tcm memory\n");

		scp_cluster->l1tcm_size = resource_size(res);
		scp_cluster->l1tcm_phys = res->start;
	}

	INIT_LIST_HEAD(&scp_cluster->mtk_scp_list);
	mutex_init(&scp_cluster->cluster_lock);

	ret = of_platform_populate(dev_of_node(dev), scp_core_match, NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to populate platform devices\n");

	ret = scp_cluster_init(pdev, scp_cluster);
	if (ret) {
		of_platform_depopulate(dev);
		return ret;
	}

	return 0;
}

static void scp_remove(struct platform_device *pdev)
{
	struct mtk_scp *scp = platform_get_drvdata(pdev);
	struct mtk_scp_of_cluster *scp_cluster = scp->cluster;
	struct mtk_scp *temp;

	list_for_each_entry_safe_reverse(scp, temp, &scp_cluster->mtk_scp_list, elem) {
		list_del(&scp->elem);
		rproc_del(scp->rproc);
		scp_free(scp);
	}
	of_platform_depopulate(&pdev->dev);
	mutex_destroy(&scp_cluster->cluster_lock);
}

static const struct mtk_scp_sizes_data default_scp_sizes = {
	.max_dram_size = 0x500000,
	.ipi_share_buffer_size = 288,
};

static const struct mtk_scp_sizes_data mt8188_scp_sizes = {
	.max_dram_size = 0x800000,
	.ipi_share_buffer_size = 600,
};

static const struct mtk_scp_sizes_data mt8188_scp_c1_sizes = {
	.max_dram_size = 0xA00000,
	.ipi_share_buffer_size = 600,
};

static const struct mtk_scp_sizes_data mt8195_scp_sizes = {
	.max_dram_size = 0x800000,
	.ipi_share_buffer_size = 288,
};

static const struct mtk_scp_sizes_data mt6785_scp_sizes = {
	.max_dram_size = MT6785_SCP_DRAM_SIZE,
	.ipi_share_buffer_size = MT6785_SCP_IPI_BUF_SIZE,
};

/*
 * The SCP subsystem's clock gates, as the vendor's clock driver sets them at probe:
 *
 *	clk_writel(CLK_SCP_CFG_0, clk_readl(CLK_SCP_CFG_0) | 0x3FF);
 *	clk_writel(CLK_SCP_CFG_1, clk_readl(CLK_SCP_CFG_1) | 0x11);
 *
 *	(vendor clk-mt6785.c:3144-3145, MTK's own comment there is "Need Confirm";
 *	 CLK_SCP_CFG_0 = topckgen + 0x200, CLK_SCP_CFG_1 = topckgen + 0x210, :91-92)
 *
 * Nothing in mainline models these twelve bits, and with them clear the SCP's register window and
 * its TCM read back as zeros however much the driver writes there - which is what audio16's dump
 * showed (every cfg register zero, SW_RSTN included, after a write of 1 into it). So MT6785 sets
 * them once, in before_load, exactly as the vendor's clock driver does at boot.
 */
static int mt6785_scp_clk_get(struct mtk_scp *scp)
{
	struct clk *infra;
	int ret;

	ret = mt8183_scp_clk_get(scp);
	if (ret)
		return ret;

	/*
	 * The SCP subsystem's own bus gate ("infra_scp", CLK_INFRA_SCP, infracfg gate 4).
	 * Nothing else in this kernel consumes it, and while it is closed every access to the
	 * SCP's register window and its TCM reads back as zero - the load "succeeds" and the
	 * core then fetches zeros. It stays enabled for the life of the system: it is the bus
	 * the SCP lives on, and gating it between the phases the remoteproc core drives would
	 * only add a failure mode.
	 */
	infra = devm_clk_get_optional(scp->dev, "infra");
	if (IS_ERR(infra))
		return dev_err_probe(scp->dev, PTR_ERR(infra), "Failed to get the infra clock\n");
	if (infra) {
		ret = clk_prepare_enable(infra);
		if (ret)
			return ret;
	}

	return 0;
}

/* the infracfg regmap, from the phandle the DT gives */
static struct regmap *mt6785_infracfg(struct mtk_scp *scp)
{
	return syscon_regmap_lookup_by_phandle(scp->dev->of_node,
					       "mediatek,infracfg");
}

/* infracfg bit set, then wait for the status bit, as the vendor does (scp_awake.c:232-283) */
static int mt6785_infracfg_reset(struct mtk_scp *scp, const char *what,
				 u32 set_reg, u32 stat_reg, u32 bits)
{
	struct regmap *infracfg = mt6785_infracfg(scp);
	u32 val;
	int ret;

	if (IS_ERR(infracfg))
		return PTR_ERR(infracfg);

	regmap_read(infracfg, set_reg, &val);
	dev_info(scp->dev, "%s: %#x = %#x, setting %#x\n", what, set_reg, val, bits);
	regmap_write(infracfg, set_reg, val | bits);
	/* regmap writes are posted on this bus: the status read below must see this one first */
	wmb();

	ret = regmap_read_poll_timeout(infracfg, stat_reg, val,
				       (val & bits) == bits, 10,
				       MT6785_SCP_SYS_RESET_TIMEOUT_US);
	if (ret)
		dev_err(scp->dev, "%s: timeout (status %#x)\n", what, val);
	else
		dev_info(scp->dev, "%s: status %#x = %#x (acknowledged)\n",
			 what, stat_reg, val);

	return ret;
}

/*
 * The SCP subsystem's own module reset and its sleep protection, in the order the vendor's
 * scp_sys_full_reset() uses (scp_awake.c:222-300): protect the infra and audio buses, hold the SCP
 * subsystem in reset and release it again, then switch the SRAM on one block at a time. The port
 * only had the AP-side SW_RSTN in the cfg window, which is a different reset (the vendor's own
 * reset_scp() writes it, scp_helper.c:596 and :617, but only on a subsystem that is already up).
 */
static int mt6785_scp_subsys_reset(struct mtk_scp *scp)
{
	struct regmap *infracfg = mt6785_infracfg(scp);
	u32 status, clk_sw, val;
	int ret;

	if (IS_ERR(infracfg))
		return PTR_ERR(infracfg);

	/*
	 * Whether the subsystem needs its reset cycled at all is decided by its own clock block:
	 * on the base kernel (no driver) cfg + 0x4000 reads 0x201, i.e. the block is alive and only
	 * the SRAM is down. audio18 left the whole block reading zero after this sequence had run,
	 * which is what an *asserted* reset looks like - and the likely reason is the secure reset
	 * bit below, which the non-secure world can set but not clear.
	 */
	clk_sw = readl(scp->cluster->reg_base + MT8183_SCP_CLK_SW_SEL);
	regmap_read(infracfg, MT6785_INFRACFG_MODULE_RESET_STATUS, &status);
	dev_info(scp->dev, "state: clk_sw %#x reset_status %#x\n", clk_sw, status);

	if (clk_sw && clk_sw != ~0U) {
		dev_info(scp->dev, "the SCP clock block reads alive: no module reset\n");
		return 0;
	}

	/*
	 * The block is dead, so cycle the subsystem's reset the way the vendor's full reset does -
	 * but with the SCP bit only (bit 3). The vendor also sets SCP_SEC_RESET_BIT (bit 10,
	 * scp_awake.c:268-283): writing it is possible from here, clearing it is the secure world's
	 * privilege, and leaving it asserted holds the whole block unreadable from the AP.
	 */
	ret = mt6785_infracfg_reset(scp, "sleep protect", MT6785_INFRACFG_SLP_PROT_SET,
				    MT6785_INFRACFG_SLP_PROT_STAT,
				    MT6785_SCP_TO_INFRA_BIT | MT6785_SCP_TO_AUDIO_BIT);
	if (ret)
		return ret;

	ret = mt6785_infracfg_reset(scp, "subsys reset",
				    MT6785_INFRACFG_MODULE_RESET_SET,
				    MT6785_INFRACFG_MODULE_RESET_STATUS,
				    MT6785_SCP_RESET_BIT);
	if (ret)
		return ret;

	/* "finish the shutdown process by clearing reset bit" (scp_awake.c:276-281) */
	regmap_read(infracfg, MT6785_INFRACFG_MODULE_RESET_CLR, &val);
	regmap_write(infracfg, MT6785_INFRACFG_MODULE_RESET_CLR, val | MT6785_SCP_RESET_BIT);
	/* and the SRAM writes that follow must not overtake it */
	wmb();
	usleep_range(50, 60);
	regmap_read(infracfg, MT6785_INFRACFG_MODULE_RESET_STATUS, &status);
	dev_info(scp->dev, "after the reset cycle: reset_status %#x\n", status);

	return 0;
}

/* "disable sleep protection" once the images are in place (vendor scp_awake.c:285-296) */
static int mt6785_scp_slp_prot_clear(struct mtk_scp *scp)
{
	struct regmap *infracfg = mt6785_infracfg(scp);
	u32 val;

	if (IS_ERR(infracfg))
		return PTR_ERR(infracfg);

	regmap_read(infracfg, MT6785_INFRACFG_SLP_PROT_SET, &val);
	regmap_write(infracfg, MT6785_INFRACFG_SLP_PROT_SET,
		     val & ~(MT6785_SCP_TO_INFRA_BIT | MT6785_SCP_TO_AUDIO_BIT));
	/* the protection must be gone before the core is released */
	wmb();

	return 0;
}

/* one line of block state, for the steps that must be told apart */
static void mt6785_scp_state(struct mtk_scp *scp, const char *when)
{
	dev_info(scp->dev, "%s: clk_sw %#x sw_rstn %#x tcm[0] %#x tcm[4] %#x\n",
		 when, readl(scp->cluster->reg_base + MT8183_SCP_CLK_SW_SEL),
		 readl(scp->cluster->reg_base + MT8183_SW_RSTN),
		 readl(scp->sram_base), readl(scp->sram_base + 4));
}

static int mt6785_scp_clocks_on(struct mtk_scp *scp)
{
	struct regmap *topckgen;
	u32 val;

	topckgen = syscon_regmap_lookup_by_phandle(scp->dev->of_node,
						   "mediatek,topckgen");
	if (IS_ERR(topckgen)) {
		dev_err(scp->dev, "no topckgen phandle: %ld\n", PTR_ERR(topckgen));
		return PTR_ERR(topckgen);
	}

	regmap_read(topckgen, MT6785_SCP_CLK_CFG_0, &val);
	regmap_write(topckgen, MT6785_SCP_CLK_CFG_0, val | 0x3ff);
	regmap_read(topckgen, MT6785_SCP_CLK_CFG_1, &val);
	regmap_write(topckgen, MT6785_SCP_CLK_CFG_1, val | 0x11);

	return 0;
}

/*
 * MT6785 does need a clock, and d5b9b1de1 got this wrong: the vendor's own
 * scp@10500000 node lists no clocks, but the SCP's clock is driven from the
 * *scp_dvfs* block through the clock framework, and the vendor enables it
 * immediately before releasing the core from reset - "request pll clock before
 * turn on scp" (scp_helper.c:583), which is scp_pll_ctrl_set(PLL_ENABLE,
 * CLK_26M) (scp_dvfs.c:597-655): clk_prepare_enable(topckgen's SCP mux) and then
 * clk_set_parent(that mux, clk26m). Mainline has the same object as "top_scp"
 * (CLK_TOP_SCP, drivers/clk/mediatek/clk-mt6785-top.c:473, gate bit 2, parent
 * list starting at "clk26m"), so the port takes it as "main" and lets the clock
 * framework parent it to the 26 MHz oscillator from the device tree. Without
 * this the TCM loads and nothing else happens, which is what audio13 measured.
 *
 * The init IPI is the other half: this firmware reports "ready" on IPI 12,
 * IPI_SCP_A_READY (vendor scp_cm4.h:17-40, registered at scp_helper.c:2015-2016),
 * not on the MT8183 family's SCP_IPI_INIT = 0.
 */
static const struct mtk_scp_of_data mt6785_of_data = {
	.scp_clk_get = mt6785_scp_clk_get,
	.scp_before_load = mt6785_scp_before_load,
	.scp_after_load = mt6785_scp_after_load,
	.scp_irq_handler = mt8183_scp_irq_handler,
	.scp_reset_assert = mt8183_scp_reset_assert,
	.scp_reset_deassert = mt8183_scp_reset_deassert,
	.scp_stop = mt6785_scp_stop_full,
	.scp_dump_regs = mt6785_scp_dump_regs,
	.scp_keep_state_on_error = true,
	.scp_maybe_preloaded = true,
	.scp_use_lk_images = true,
	.scp_ipi_ack = mt6785_scp_ipi_ack,
	.scp_clocks_on = mt6785_scp_clocks_on,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8183_HOST_TO_SCP,
	.host_to_scp_int_bit = MT8183_HOST_IPC_INT_BIT,
	.ipi_buf_offset = MT6785_SCP_IPI_BUF_OFS,
	.ipi_buf_data_offset = MT6785_SCP_IPI_BUF_DATA_OFS,
	.ipi_init_id = MT6785_SCP_IPI_READY,
	.scp_sizes = &mt6785_scp_sizes,
};

static const struct mtk_scp_of_data mt8183_of_data = {
	.scp_clk_get = mt8183_scp_clk_get,
	.scp_before_load = mt8183_scp_before_load,
	.scp_irq_handler = mt8183_scp_irq_handler,
	.scp_reset_assert = mt8183_scp_reset_assert,
	.scp_reset_deassert = mt8183_scp_reset_deassert,
	.scp_stop = mt8183_scp_stop,
	.scp_da_to_va = mt8183_scp_da_to_va,
	.host_to_scp_reg = MT8183_HOST_TO_SCP,
	.host_to_scp_int_bit = MT8183_HOST_IPC_INT_BIT,
	.ipi_buf_offset = 0x7bdb0,
	.scp_sizes = &default_scp_sizes,
};

static const struct mtk_scp_of_data mt8186_of_data = {
	.scp_clk_get = mt8195_scp_clk_get,
	.scp_before_load = mt8186_scp_before_load,
	.scp_irq_handler = mt8183_scp_irq_handler,
	.scp_reset_assert = mt8183_scp_reset_assert,
	.scp_reset_deassert = mt8183_scp_reset_deassert,
	.scp_stop = mt8183_scp_stop,
	.scp_da_to_va = mt8183_scp_da_to_va,
	.host_to_scp_reg = MT8183_HOST_TO_SCP,
	.host_to_scp_int_bit = MT8183_HOST_IPC_INT_BIT,
	.ipi_buf_offset = 0x3bdb0,
	.scp_sizes = &default_scp_sizes,
};

static const struct mtk_scp_of_data mt8188_of_data = {
	.scp_clk_get = mt8195_scp_clk_get,
	.scp_before_load = mt8188_scp_before_load,
	.scp_irq_handler = mt8195_scp_irq_handler,
	.scp_reset_assert = mt8192_scp_reset_assert,
	.scp_reset_deassert = mt8192_scp_reset_deassert,
	.scp_stop = mt8188_scp_stop,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8192_GIPC_IN_SET,
	.host_to_scp_int_bit = MT8192_HOST_IPC_INT_BIT,
	.scp_sizes = &mt8188_scp_sizes,
};

static const struct mtk_scp_of_data mt8188_of_data_c1 = {
	.scp_clk_get = mt8195_scp_clk_get,
	.scp_before_load = mt8188_scp_c1_before_load,
	.scp_irq_handler = mt8195_scp_c1_irq_handler,
	.scp_reset_assert = mt8195_scp_c1_reset_assert,
	.scp_reset_deassert = mt8195_scp_c1_reset_deassert,
	.scp_stop = mt8188_scp_c1_stop,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8192_GIPC_IN_SET,
	.host_to_scp_int_bit = MT8195_CORE1_HOST_IPC_INT_BIT,
	.scp_sizes = &mt8188_scp_c1_sizes,
};

static const struct mtk_scp_of_data mt8192_of_data = {
	.scp_clk_get = mt8192_scp_clk_get,
	.scp_before_load = mt8192_scp_before_load,
	.scp_irq_handler = mt8192_scp_irq_handler,
	.scp_reset_assert = mt8192_scp_reset_assert,
	.scp_reset_deassert = mt8192_scp_reset_deassert,
	.scp_stop = mt8192_scp_stop,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8192_GIPC_IN_SET,
	.host_to_scp_int_bit = MT8192_HOST_IPC_INT_BIT,
	.scp_sizes = &default_scp_sizes,
};

static const struct mtk_scp_of_data mt8195_of_data = {
	.scp_clk_get = mt8195_scp_clk_get,
	.scp_before_load = mt8195_scp_before_load,
	.scp_irq_handler = mt8195_scp_irq_handler,
	.scp_reset_assert = mt8192_scp_reset_assert,
	.scp_reset_deassert = mt8192_scp_reset_deassert,
	.scp_stop = mt8195_scp_stop,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8192_GIPC_IN_SET,
	.host_to_scp_int_bit = MT8192_HOST_IPC_INT_BIT,
	.scp_sizes = &mt8195_scp_sizes,
};

static const struct mtk_scp_of_data mt8195_of_data_c1 = {
	.scp_clk_get = mt8195_scp_clk_get,
	.scp_before_load = mt8195_scp_c1_before_load,
	.scp_irq_handler = mt8195_scp_c1_irq_handler,
	.scp_reset_assert = mt8195_scp_c1_reset_assert,
	.scp_reset_deassert = mt8195_scp_c1_reset_deassert,
	.scp_stop = mt8195_scp_c1_stop,
	.scp_da_to_va = mt8192_scp_da_to_va,
	.host_to_scp_reg = MT8192_GIPC_IN_SET,
	.host_to_scp_int_bit = MT8195_CORE1_HOST_IPC_INT_BIT,
	.scp_sizes = &default_scp_sizes,
};

static const struct mtk_scp_of_data *mt8188_of_data_cores[] = {
	&mt8188_of_data,
	&mt8188_of_data_c1,
	NULL
};

static const struct mtk_scp_of_data *mt8195_of_data_cores[] = {
	&mt8195_of_data,
	&mt8195_of_data_c1,
	NULL
};

static const struct of_device_id mtk_scp_of_match[] = {
	{ .compatible = "mediatek,mt8183-scp", .data = &mt8183_of_data },
	{ .compatible = "mediatek,mt6785-scp", .data = &mt6785_of_data },
	{ .compatible = "mediatek,mt8186-scp", .data = &mt8186_of_data },
	{ .compatible = "mediatek,mt8188-scp", .data = &mt8188_of_data },
	{ .compatible = "mediatek,mt8188-scp-dual", .data = &mt8188_of_data_cores },
	{ .compatible = "mediatek,mt8192-scp", .data = &mt8192_of_data },
	{ .compatible = "mediatek,mt8195-scp", .data = &mt8195_of_data },
	{ .compatible = "mediatek,mt8195-scp-dual", .data = &mt8195_of_data_cores },
	{},
};
MODULE_DEVICE_TABLE(of, mtk_scp_of_match);

static int __maybe_unused scp_suspend(struct device *dev)
{
	struct mtk_scp *scp = dev_get_drvdata(dev);
	struct rproc *rproc = scp->rproc;

	/*
	 * Only unprepare if the SCP is running and holding the clock.
	 *
	 * Note: `scp_ops` doesn't implement .attach() callback, hence
	 * `rproc->state` can never be RPROC_ATTACHED.  Otherwise, it
	 * should also be checked here.
	 */
	if (rproc->state == RPROC_RUNNING)
		clk_unprepare(scp->clk);
	return 0;
}

static int __maybe_unused scp_resume(struct device *dev)
{
	struct mtk_scp *scp = dev_get_drvdata(dev);
	struct rproc *rproc = scp->rproc;

	/*
	 * Only prepare if the SCP was running and holding the clock.
	 *
	 * Note: `scp_ops` doesn't implement .attach() callback, hence
	 * `rproc->state` can never be RPROC_ATTACHED.  Otherwise, it
	 * should also be checked here.
	 */
	if (rproc->state == RPROC_RUNNING)
		return clk_prepare(scp->clk);
	return 0;
}

static const struct dev_pm_ops scp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(scp_suspend, scp_resume)
};

static struct platform_driver mtk_scp_driver = {
	.probe = scp_probe,
	.remove = scp_remove,
	.driver = {
		.name = "mtk-scp",
		.of_match_table = mtk_scp_of_match,
		.pm = &scp_pm_ops,
	},
};

module_platform_driver(mtk_scp_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SCP control driver");
