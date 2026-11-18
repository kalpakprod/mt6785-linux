// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2019 MediaTek Inc.

#include <asm/barrier.h>
#include <linux/arm-smccc.h>
#include <linux/io.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/clk.h>
#include <linux/crc32.h>
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
#include <linux/regulator/consumer.h>

#include "mtk_common.h"
#include "remoteproc_internal.h"

struct mtk_scp;
static u32 mt6785_scp_crc_region(struct mtk_scp *scp, u32 ofs, size_t len);

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

	/*
	 * SCP_A_TO_HOST (GIC SPI 201) is the only readiness signal this driver trusts: the core
	 * owns the bus once released, so a register read is not evidence of anything. Counting the
	 * interrupts makes a trial say whether the firmware ever spoke.
	 */
	scp->to_host_irqs++;
	if (scp->to_host_irqs <= 4)
		dev_info(scp->dev, "SCP_A_TO_HOST interrupt #%u (to_host %#x)\n",
			 scp->to_host_irqs, scp_to_host);

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

	/* the SCP clock's other possible parents, if the DT names them */
	{
		static const char *const names[MTK_SCP_MAX_PLL] = {
			"scp_pll_1", "scp_pll_2", "scp_pll_3",
			"scp_pll_4", "scp_pll_5", "scp_pll_6",
		};
		unsigned int i;

		for (i = 0; i < MTK_SCP_MAX_PLL; i++) {
			scp->cluster->pll[i] = devm_clk_get_optional(dev, names[i]);
			if (IS_ERR(scp->cluster->pll[i])) {
				scp->cluster->pll[i] = NULL;
				continue;
			}
			if (scp->cluster->pll[i])
				scp->cluster->pll_num = i + 1;
		}
	}

	scp->topckgen_regmap =
		syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,topckgen-meter");
	if (IS_ERR(scp->topckgen_regmap)) {
		dev_info(dev, "no topckgen phandle for the meter: unreported\n");
		scp->topckgen_regmap = NULL;
	}

	scp->spm_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,spm");
	if (IS_ERR(scp->spm_regmap)) {
		dev_info(dev, "no SPM phandle: the oscillator state will not be reported\n");
		scp->spm_regmap = NULL;
	}

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
/* LK's own L1C_CON value and the distance between the I and D instances (pl_lk.txt:2390-2393) */
#define MT6785_SCP_L1C_CON_INIT		0x100
/* crc32 of the firmware part as the vendor image carries it, measured in trials 35 and 36 */
#define MT6785_DRAM_COPY_CRC		0x3e3c38ba
/* crc32 of a run of zeros: what a cleared range reads as */
#define MT6785_ZERO_CRC			0x00000000
#define MT6785_SCP_L1C_STRIDE		0x3000

/*
 * LK's own layout inside its "SCP-reserved" region (pl_lk.txt:2305-2306, :2406-2411): the
 * loader image at its start (its vector table in the first words), the staged firmware 0x800
 * bytes in, the DRAM part 1 MiB in, and the backup copy 2 MiB in.
 */
#define MT6785_LK_LOADER_SP		0x0
#define MT6785_LK_LOADER_PC		0x4
#define MT6785_LK_FIRMWARE_OFS		0x800
#define MT6785_LK_DRAM_OFS		0x100000
/* LK's own sizes for that layout (pl_lk.txt:2311-2313, :2406-2411) */
#define MT6785_SCP_LK_REGION_MIN	0x300000
/* the TCM part of the vendor's scp record, and where its tail sits in the carve-out */
#define MT6785_SCP_FIRMWARE_TCM_MAX	0xbb800
#define MT6785_SCP_REST_OFS		0x1c0000
#define MT6785_LK_LOADER_SIZE		0x438
#define MT6785_LK_FIRMWARE_SIZE		0xbb800
#define MT6785_LK_DRAM_SIZE		0x59968
/* the vendor's reboot handshake word: cfg + 0x64, ready-to-reboot = 0x34 (scp_reg.h:44-46) */
#define MT6785_SCP_GPR_CM4_A_REBOOT	0x64
#define MT6785_SCP_CM4_READY_TO_REBOOT	0x34
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
static void mt6785_scp_mpu_probe(struct mtk_scp *scp);
static void mt6785_scp_dram_probe(struct mtk_scp *scp);
static void mt6785_scp_dram_probe_check(struct mtk_scp *scp, const char *when);
static void mt6785_scp_clock_state(struct mtk_scp *scp, const char *when);
static int mt6785_scp_subsys_reset(struct mtk_scp *scp);
static int mt6785_scp_slp_prot_set(struct mtk_scp *scp);
static int mt6785_scp_slp_prot_clear(struct mtk_scp *scp);
static int mt6785_scp_awake(struct mtk_scp *scp, bool lock);
static void mt6785_scp_log_wdt_sleep(struct mtk_scp *scp, const char *when);
static void mt6785_scp_log_ulposc(struct mtk_scp *scp, const char *when);
static void mt6785_scp_log_fmeter(struct mtk_scp *scp, const char *when);
static void mt6785_scp_scan_dram_text(struct mtk_scp *scp);
static void mt6785_scp_log_ring_probe(struct mtk_scp *scp, const char *when);
static void mt6785_scp_diff_region(struct mtk_scp *scp);

/*
 * Where the region's bytes came from: the requested image, and the offsets of the firmware and
 * dram records inside it. Filled in when the image is staged.
 */
static struct {
	const u8 *img;
	size_t fw_ofs, fw_len, dram_ofs, dram_len;
} mt6785_region_ref;
static void mt6785_scp_log_spm_handshake(struct mtk_scp *scp, const char *when);
static void mt6785_scp_log_sspm(struct mtk_scp *scp);
static void mt6785_scp_log_rails(struct mtk_scp *scp);
static void mt6785_scp_vol_lv(struct mtk_scp *scp);
/*
 * True when the SCP is already up. On MT6785 the loader starts it from the scp_a partition, so its
 * clock block holds SCP_CLK_SW_SEL's programmed value (0x201 on this phone, measured on a kernel
 * without any SCP driver) and the region information at TCM + 0x400 is the loader's, not ours.
 * Resetting and reloading a subsystem in that state is what audio20 did - and the block stopped
 * answering the AP as soon as the core ran (every register read 0x1e1, the stale last value).
 */
static struct regmap *mt6785_infracfg(struct mtk_scp *scp);
static int mt6785_scp_check_windows(struct mtk_scp *scp);
static int mt6785_scp_stage_mtk_image(struct mtk_scp *scp,
				      const struct firmware *fw);
static void mt6785_scp_probe_lk_images(struct mtk_scp *scp);
static void mt6785_scp_map_windows(struct platform_device *pdev,
				   struct mtk_scp *scp);
static void mt6785_scp_log_dram_state(struct mtk_scp *scp, const char *when);
static void mt6785_scp_log_pages(struct mtk_scp *scp, const char *when);
static void mt6785_scp_hexdump(struct mtk_scp *scp, u32 ofs, size_t len,
			       const char *label);
static void mt6785_scp_scan_text(struct mtk_scp *scp);
static void mt6785_scp_log_tcm(struct mtk_scp *scp, const char *when);
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

	/*
	 * LK sets up the two L1 cache instances before it writes the region information:
	 * L1C_CON = 0x100 for IL1C at 0x105d4000 and DL1C at 0x105d7000, and then reads
	 * 105d4000 to report "l1c init status" (pl_lk.txt:2390-2393). The instances are 0x3000
	 * apart (vendor scp_l1c.h:19-21).
	 */
	if (scp->l1c_base) {
		writel(MT6785_SCP_L1C_CON_INIT, scp->l1c_base);
		writel(MT6785_SCP_L1C_CON_INIT, scp->l1c_base + MT6785_SCP_L1C_STRIDE);
		/*
		 * LK's "l1c init status: 0x0" is its own function's return value (0 = success),
		 * not this register - that line is not in our sources, only in LK's log, and the
		 * register it writes is 0x100. So this prints the register and says so.
		 */
		dev_info(scp->dev, "l1c L1C_CON readback: %#x (LK wrote %#x here)\n",
			 readl(scp->l1c_base), MT6785_SCP_L1C_CON_INIT);
	}

	/* a wrong offset has to fail the probe, not the boot (audio25) */
	ret = mt6785_scp_check_windows(scp);
	if (ret)
		return ret;

	/*
	 * A word with a known value in the cfg block, written before the release and read
	 * back right after it: the post-mortem's 0x1 is the bus timing out, so this tells
	 * whether the bus went dark at the release or later. GENERAL_REG0 (cfg + 0x50) is a
	 * free scratch word in the vendor's map (scp_reg.h:39-42), unused by this driver.
	 */
	writel(MT6785_SCP_PROBE_VALUE, cluster->reg_base + MT6785_SCP_PROBE_REG);
	dev_info(scp->dev, "probe word written before the release: %#x reads %#x\n",
		 MT6785_SCP_PROBE_VALUE,
		 readl(cluster->reg_base + MT6785_SCP_PROBE_REG));
	mt6785_scp_log_wdt_sleep(scp, "before the release");
	mt6785_scp_log_ulposc(scp, "before the release");
	mt6785_scp_log_fmeter(scp, "before the release");
	mt6785_scp_log_spm_handshake(scp, "before the release");
	mt6785_scp_log_sspm(scp);
	mt6785_scp_log_rails(scp);

	/* without these the whole SCP window reads back as zeros (audio16) */
	ret = mt6785_scp_clocks_on(scp);
	if (ret)
		return ret;

	mt6785_scp_mpu_probe(scp);

	/*
	 * The SCP's bus protection, around the bring-up this driver performs itself:
	 * INFRA_SLP_PROT_SET bits 30/31 (SCP_TO_INFRA_BIT | SCP_TO_AUDIO_BIT) hold the SCP's
	 * paths to the infra and audio busses while the SRAM is powered up and the images are
	 * put in place. That is the vendor's full-reset path - the one piece of its driver that
	 * does this work at all (scp_awake.c:222-305): protection up, reset cycled, SRAM on,
	 * loader and region information copied, protection down (:285-296). Stock's normal boot
	 * is LK's next door and never enters it, so the citation is "the path that does our job",
	 * not "every boot". This driver does that work on every start, but took the protection
	 * only inside mt6785_scp_subsys_reset()'s dead-block branch - and the state we find
	 * (clk_sw 0x201, the loader's own clock select) returns from that function before the
	 * write, so the work ran unprotected. The acknowledgment polled here is the block's own
	 * (vendor :232-244), so a timeout is a fact about this SoC, not a reason to stop.
	 */
	(void)mt6785_scp_slp_prot_set(scp);

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
	 * No cache-geometry write here, and the reason is the SoC: mt8183's CACHE_CON and
	 * DCACHE_CON live at 0x14000/0x17000 of its register block (MT8183_SCP_CACHE_SEL(),
	 * mt_common.h), which MT6785 does not have - writing them there is what took audio25
	 * down with a paging fault. MT6785 hands the cache geometry to the firmware in the
	 * region information instead, Il1c_con and Dl1c_con 0x10d, exactly as LK does
	 * (pl_lk.txt:2409-2410), and the vendor's own MT6785 driver has no cache register at
	 * all (no definition in its scp_reg.h, no write in scp_helper.c).
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

/*
 * The vendor's firmware file is not an ELF: it is the MTK image LK parses - a list of records,
 * each with a 512-byte header (magic 0x58881688, size, name[32], load address, mode, then the
 * extension header: magic 0x58891689, header size, header version, image type, list-end,
 * alignment, 64-bit size and address) and its payload (LK's parser: ccci_util_lib_load_img.c:
 * 1216-1260 in the vendor tree; the same layout the repacker in this port's tools reads).
 * LK loads "tinysys-loader-CM4_A" (1080 bytes) at TCM 0, stages "tinysys-scp-CM4_A" for the
 * loader to copy to core 0x800 and "tinysys-scp-CM4_A_dram" as the firmware's DRAM part
 * (pl_lk.txt:2311-2313, :2406-2411).
 */
#define MTK_IMG_MAGIC		0x58881688
#define MTK_IMG_EXT_MAGIC	0x58891689
#define MTK_IMG_HDR_SIZE	0x200
#define MTK_IMG_NAME_SIZE	32
#define MTK_IMG_ALIGN		16

struct mtk_img_header {
	__le32 magic;
	__le32 dsize;
	char name[MTK_IMG_NAME_SIZE];
	__le32 maddr;
	__le32 mode;
	__le32 ext_magic;
	__le32 hdr_size;
	__le32 hdr_version;
	__le32 img_type;
	__le32 img_list_end;
	__le32 align_size;
	__le32 dsize_extend;
	__le32 maddr_extend;
};

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

	/*
	 * One probe word, not the whole register set: the core's state words and the status
	 * words all read as the same timeout value (audio37's journal: 13 ms per access), so
	 * only the word this driver wrote with a known value can say anything.
	 */
	dev_err(scp->dev, "post-mortem: reading the probe word at cfg + %#x\n",
		MT6785_SCP_PROBE_REG);
	dev_err(scp->dev, "post-mortem: probe word %#x (expected %#x)\n",
		readl(cfg + MT6785_SCP_PROBE_REG), MT6785_SCP_PROBE_VALUE);
	mt6785_scp_scan_dram_text(scp);
	mt6785_scp_diff_region(scp);
	mt6785_scp_log_ring_probe(scp, "at the timeout");
	mt6785_scp_log_dram_state(scp, "at the timeout");
	mt6785_scp_log_pages(scp, "at the timeout");
	mt6785_scp_hexdump(scp, MT6785_LK_DRAM_OFS, MT6785_LK_DRAM_SIZE,
			   "dram part at the timeout");
	mt6785_scp_hexdump(scp, MT6785_LK_DRAM_OFS * 2, MT6785_LK_DRAM_SIZE,
			   "backup at the timeout");
	mt6785_scp_scan_text(scp);

	dev_err(scp->dev, "SCP_A_TO_HOST interrupts seen: %u\n", scp->to_host_irqs);

	/*
	 * The two questions the core's own state cannot answer by itself: is the block out of
	 * reset, and did the loader reach TCM at all. All-zero PC/LR/SP/PSP means either "the core
	 * never ran" or "it ran and read a zero vector table", and these two reads separate them:
	 * SW_RSTN bit 0 is what this driver writes to release the core, and TCM word 0 is the
	 * vector's initial stack pointer (0x538 for the vendor's loader record at TCM 0 - while a
	 * zero there means the ELF segment never landed or the TCM is not powered).
	 */
	/* nothing more: every other cfg or SRAM word is the bus timeout's 0x1 */
}

static int mt6785_scp_after_load(struct mtk_scp *scp,
				 const struct firmware *fw)
{
	struct mt6785_scp_region_info ri = { 0 };
	u32 fw_size, dram_size;
	int ret;

	/* kept for the post-mortem's DRAM text scan, to tell the image's strings apart */
	scp->image = (u8 *)fw->data;
	scp->image_size = fw->size;

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
	/* the remaining fields LK prints as zero for this board (pl_lk.txt:2411-2414) */
	ri.scp_log_thru_ap_uart = 0;
	ri.scpctl = 0;
	ri.ap_params_start = 0;
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

	/* Hold SCP in reset while loading FW. */
	scp->data->scp_reset_assert(scp);

	ret = scp->data->scp_before_load(scp);
	if (ret < 0)
		goto leave;

	/*
	 * LK's own images need no ELF load and no region-info rewrite: the loader copy and the
	 * region information were written in before_load(), and the loader pulls the firmware from
	 * the DRAM region LK prepared.
	 */
	/*
	 * MT6785's firmware is the vendor image LK parses, not an ELF: stage the records into the
	 * carve-out the way LK stages them into its own region, and let the loader do the rest.
	 */
	if (scp->data->scp_mtk_image) {
		ret = mt6785_scp_stage_mtk_image(scp, fw);
		if (ret)
			goto leave;
		ret = 0;
		goto leave;
	}

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

	/*
	 * The IPI buffer address is read out of the firmware's ELF; the vendor's MTK image has no
	 * ELF, so the SoC data's own offset is what the client uses.
	 */
	if (!scp->data->scp_mtk_image)
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

	if (scp->data->scp_dump_regs == mt6785_scp_dump_regs) {
		mt6785_scp_log_dram_state(scp, "before the release");
		mt6785_scp_log_pages(scp, "before");
		mt6785_scp_log_tcm(scp, "before the release (core held in reset)");
		mt6785_scp_hexdump(scp, MT6785_LK_DRAM_OFS, MT6785_LK_DRAM_SIZE,
				   "dram part before the release");
	}

	if (scp->data->scp_dump_regs == mt6785_scp_dump_regs)
		dev_info(dev, "reboot handshake word before release: %#x (LK/vendor ready value %#x)\n",
			 readl(scp->cluster->reg_base + MT6785_SCP_GPR_CM4_A_REBOOT),
			 MT6785_SCP_CM4_READY_TO_REBOOT);

	/*
	 * What the stock bring-up does around the release, and where this driver stands:
	 *
	 *  hold the SCP mux's 26 MHz PLL across the reset   scp_helper.c:588       yes (clk)
	 *  release the reset bit                            scp_helper.c:619       yes
	 *  wait for the ready IPI                           scp_helper.c:621-640   yes
	 *  release that PLL once the IPI has arrived        scp_helper.c:350       yes
	 *  request MAINPLL|CK_26M|AXI_BUS from the SPM      scp_dvfs.c:289-292     no write
	 *
	 * The last line is the one deviation this driver had on the vendor's boot path, and
	 * NOTE 27's table is why it is gone: the vendor delivers that request by SMC, one call
	 * per field (spm/mt6785/mtk_spm_resource_req_console.c:44-70,
	 * SMC_CALL(PWR_CTRL_ARGS, id, PW_REG_SPM_F26M_REQ/INFRA_REQ/VRF18_REQ, 1)), and releases
	 * it at the ready IPI; stock reads SPM_SRC_REQ (0xb8) as 0x0 with the SCP running. An
	 * earlier version of this driver wrote bits 4|3|1 into that register and held them
	 * through the boot, which is neither the delivery nor the state of the reference, and the
	 * SMC's id - the idle model the SCP maps to - is not spelled out in any source we hold.
	 * So nothing is written: the register is reported with the rest of the SPM block.
	 */
	mt6785_scp_vol_lv(scp);

	/*
	 * The protection has to be down before the core is released, and this is the one place
	 * that covers every load path. mt6785_scp_after_load() releases it for the ELF firmware,
	 * but the vendor container (scp-ext.img) is staged by mt6785_scp_stage_mtk_image() and
	 * never reaches after_load() - which is what comb7 showed: the block stayed alive and
	 * answered the AP while the core never ran (clk_sw 0x201, clk_en 0x0, TCM[0] 0x538
	 * unchanged for two seconds, still -62), because the release happened with
	 * INFRA_SLP_PROT bits 30/31 still set and the core's path to its own firmware held.
	 * The vendor's order is protection down, then release (scp_awake.c:285-296, then
	 * scp_helper.c:617); a second clear on the ELF path is a no-op.
	 */
	mt6785_scp_slp_prot_clear(scp);

	/* the loader's place is taken by the copy probe (NOTE 32), see its comment */
	mt6785_scp_dram_probe(scp);

	scp->data->scp_reset_deassert(scp);

	/*
	 * Two reads right after the release, before the core can have gone to sleep: the
	 * probe word this driver wrote before the release, and the vendor's own reboot word.
	 * A probe word that still holds its pattern means the bus answered at that moment;
	 * the 0x1 the post-mortem reads is the bus waiting out its timeout.
	 */
	mt6785_scp_state(scp, "right after the release");
	dev_info(dev, "probe word after the release: %#x (expected %#x), sw_rstn %#x\n",
		 readl(scp->cluster->reg_base + MT6785_SCP_PROBE_REG),
		 MT6785_SCP_PROBE_VALUE,
		 readl(scp->cluster->reg_base + MT8183_SW_RSTN));

	/*
	 * One read back, immediately: the core has just been released and cannot have taken the
	 * bus yet, so this is the last honest look at the register the write went to.
	 */
	if (scp->data->scp_dump_regs == mt6785_scp_dump_regs)
		dev_info(dev, "released the core: sw_rstn reads %#x (bit0 set is released)\n",
			 readl(scp->cluster->reg_base + MT8183_SW_RSTN));

	/*
	 * Where the core is. cfg + 0xa8..0xc4 are the CM4's SP, LR, PSP and PC as the SCP's own
	 * debug block latches them (scp_reg.h:22-30), the same four words the timeout path has
	 * always printed *before* a release - 0xfffffffe there is the reset value. Two samples,
	 * not a sweep: one immediately and one after 100 ms, the smallest disturbance that can
	 * say whether the core ever started and where it stopped. The addresses are 0x800-based
	 * (the loader copies the firmware to core 0x800 and jumps there), so the image can be
	 * disassembled at whatever PC shows.
	 */
	mt6785_scp_log_wdt_sleep(scp, "just after the release");
	/*
	 * The copy's progress, sampled while it runs: the loop publishes its word index every 256
	 * words, so a series of samples says whether the transfer moves and where it stops. The
	 * deltas sum to a second, which is short compared with the two seconds the ready IPI gets.
	 */
	{
		static const unsigned int deltas_ms[] = { 1, 4, 15, 30, 50, 900 };
		static const char * const when[] = { "1 ms", "5 ms", "20 ms", "50 ms", "100 ms", "1 s" };
		unsigned int i;

		mt6785_scp_dram_probe_check(scp, "at the release");
		for (i = 0; i < ARRAY_SIZE(deltas_ms); i++) {
			msleep(deltas_ms[i]);
			mt6785_scp_dram_probe_check(scp, when[i]);
		}
		mt6785_scp_log_wdt_sleep(scp, "1 s after the release");
	}

	/*
	 * Nothing else reads the SCP's registers from here on: readiness is the ready IPI over
	 * SCP_A_TO_HOST, counted in the interrupt handler.
	 */

	ret = wait_event_interruptible_timeout(
					run->wq,
					run->signaled,
					msecs_to_jiffies(2000));

	if (ret == 0) {
		dev_err(dev, "wait SCP initialization timeout!\n");

		/*
		 * The post-mortem is the first access to the block since the release, and it goes
		 * under the vendor's awake request - which can only be taken now: its
		 * acknowledgment is the firmware's own ISR (scp_awake.c:82-96), and
		 * scp_awake_lock() refuses to send the request at all while is_scp_ready() is 0
		 * (:68-72). comb11 is that rule measured from the other side: the request written
		 * at the release sat unacknowledged for its whole second while the firmware was
		 * still booting.
		 */
		mt6785_scp_awake(scp, true);
		/*
		 * When the block stopped answering: the probe word, once every 100 ms, together
		 * with the TCM's first word - the loader's vector table, then the firmware's -
		 * and the clock block's own state, which is what names the moment the firmware
		 * switched its clock tree and what it switched to.
		 */
		{
			void __iomem *cfg = scp->cluster->reg_base;
			void __iomem *clk = cfg + MT6785_SCP_CLK_BLOCK;
			unsigned int i;
			u32 val = MT6785_SCP_PROBE_VALUE, tcm = 0;
			s64 start = ktime_get();

			for (i = 0; i < MT6785_SCP_PROBE_POLLS; i++) {
				val = readl(cfg + MT6785_SCP_PROBE_REG);
				tcm = readl(scp->sram_base);
				if (val != MT6785_SCP_PROBE_VALUE) {
					dev_err(scp->dev, "post-mortem: probe word %#x after %lld us (TCM[0] %#x)\n",
						val, ktime_us_delta(ktime_get(), start), tcm);
					break;
				}
				if (!(i % MT6785_SCP_PROBE_REPORT)) {
					u32 ms = i * MT6785_SCP_PROBE_STEP_US / 1000;
					u32 clk_sw = readl(clk + MT6785_SCP_CLK_SW_SEL);
					u32 clk_en = readl(clk + MT6785_SCP_CLK_ENABLE);

					dev_info(scp->dev, "post-mortem %u ms: clk_sw %#x clk_en %#x ctrl24 %#x\n",
						 ms, clk_sw, clk_en,
						 readl(clk + MT6785_SCP_CLK_CTRL_24));
					mt6785_scp_log_ulposc(scp, "post-mortem");
					mt6785_scp_log_fmeter(scp, "post-mortem");
					mt6785_scp_log_spm_handshake(scp, "post-mortem");
					dev_info(scp->dev, "post-mortem %u ms: sleep_dbg %#x cpu_sleep %#x wdt %#x/%#x TCM[0] %#x\n",
						 ms, readl(clk + MT6785_SCP_SLEEP_DEBUG_REG),
						 readl(cfg + MT6785_SCP_CPU_SLEEP_STATUS),
						 readl(cfg + MT6785_SCP_WDT_REG),
						 readl(cfg + MT6785_SCP_WDT_CTRL), tcm);
				}
				/*
				 * 100 ms between samples, and this runs in process context, so
				 * fsleep() is the right primitive for a delay this long.
				 */
				fsleep(MT6785_SCP_PROBE_STEP_US);
			}
			if (i == MT6785_SCP_PROBE_POLLS)
				dev_info(scp->dev, "post-mortem: the block answered for the whole %u ms\n",
					 MT6785_SCP_PROBE_POLLS * MT6785_SCP_PROBE_STEP_US / 1000);
		}

		mt6785_scp_log_wdt_sleep(scp, "at the timeout");
		mt6785_scp_dram_probe_check(scp, "at the timeout");
		mt6785_scp_clock_state(scp, "at the timeout");
		if (scp->data->scp_dump_regs)
			scp->data->scp_dump_regs(scp);
		mt6785_scp_awake(scp, false);
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

/*
 * The vendor's SCP firmware is not an ELF but the MTK record list the loader parses, so the ELF
 * sanity check the remoteproc core runs by default rejects it ("Image is corrupted (bad magic)",
 * trial 28). This is the same walk mt6785_scp_stage_mtk_image() does, kept separate so the check
 * happens before anything is written: every record's magic and extension header have to be right
 * and its payload has to fit inside the file.
 */
static int mtk_img_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	const u8 *buf = fw->data;
	size_t off = 0, records = 0;

	while (off + MTK_IMG_HDR_SIZE <= fw->size) {
		const struct mtk_img_header *h = (const void *)(buf + off);
		u32 dsize = le32_to_cpu(h->dsize);
		size_t next;

		if (le32_to_cpu(h->magic) != MTK_IMG_MAGIC)
			return -EINVAL;
		if (le32_to_cpu(h->ext_magic) != MTK_IMG_EXT_MAGIC ||
		    le32_to_cpu(h->hdr_size) != MTK_IMG_HDR_SIZE)
			return -EINVAL;
		if (off + MTK_IMG_HDR_SIZE + dsize > fw->size)
			return -EINVAL;

		records++;
		next = round_up(off + MTK_IMG_HDR_SIZE + dsize, MTK_IMG_ALIGN);
		if (next <= off)
			return -EINVAL;
		off = next;

		if (le32_to_cpu(h->img_list_end))
			break;
	}

	return records ? 0 : -EINVAL;
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

/*
 * MT6785 loads the vendor's record list, so it needs its own ops: no ELF sanity check, no ELF
 * resource table and no ELF entry point (imx_dsp_rproc and qcom's mdt loaders build their ops the
 * same way - the ELF hooks simply are not in the table).
 */
static const struct rproc_ops scp_mtk_ops = {
	.prepare	= scp_prepare,
	.unprepare	= scp_unprepare,
	.start		= scp_start,
	.stop		= scp_stop,
	.attach		= mtk_scp_attach,
	.load		= scp_load,
	.da_to_va	= scp_da_to_va,
	.parse_fw	= scp_parse_fw,
	.sanity_check	= mtk_img_sanity_check,
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

	if (ret)
		return dev_err_probe(scp->dev, ret,
				     "failed to assign memory-region\n");

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

	rproc = devm_rproc_alloc(dev, np->name,
				 of_data->scp_mtk_image ? &scp_mtk_ops : &scp_ops,
				 fw_name, sizeof(*scp));
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
	if (ret) {
		/*
		 * Nothing below has been set up yet - no IPI locks, no share buffer - so the
		 * teardown at release_dev_mem (which destroys those locks) must not run: it
		 * would destroy uninitialised mutexes.
		 */
		of_reserved_mem_device_release(scp->dev);
		return ERR_PTR(ret);
	}

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

	mt6785_scp_map_windows(pdev, scp);
	mt6785_scp_probe_lk_images(scp);

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
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (res)
		scp_cluster->cfg_size = resource_size(res);

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

/*
 * The AP's awake request to the SCP, the vendor's own handshake (scp_awake.c:79-100, unlock at
 * :162-178): write 0xA0 | the request bit to INFRA_IRQ_SET (scpsys + 0xB14, i.e. infracfg +
 * 0xB14 in this tree) and wait for that bit to clear, which is the acknowledgment; the status is
 * then cleared through INFRA_IRQ_CLEAR (+0xB18). Without a running SPM there is no partner for
 * this handshake, which is why it can time out on this phone.
 */
static int mt6785_scp_awake(struct mtk_scp *scp, bool lock)
{
	struct regmap *infracfg = mt6785_infracfg(scp);
	u32 val = 0, want = 0xA0 | (lock ? BIT(0) : BIT(1));
	int ret;

	if (IS_ERR(infracfg))
		return PTR_ERR(infracfg);

	regmap_write(infracfg, MT6785_INFRACFG_IRQ_SET, want);
	dev_info(scp->dev, "%s: INFRA_IRQ_SET <- %#x, waiting for the ack\n",
		 lock ? "awake lock" : "awake unlock", want);

	ret = regmap_read_poll_timeout(infracfg, MT6785_INFRACFG_IRQ_SET, val,
				       !(val & 0x0f & (lock ? BIT(0) : BIT(1))), 10,
				       MT6785_SCP_SYS_RESET_TIMEOUT_US);
	if (ret)
		dev_err(scp->dev, "%s: request bit still set after %u us (INFRA_IRQ_SET %#x): not acknowledged\n",
			lock ? "awake lock" : "awake unlock",
			MT6785_SCP_SYS_RESET_TIMEOUT_US, val);
	else
		dev_info(scp->dev, "%s: acknowledged (%#x)\n",
			 lock ? "awake lock" : "awake unlock", val);

	regmap_write(infracfg, MT6785_INFRACFG_IRQ_CLEAR, want);

	return ret;
}

/*
 * The two candidates for a core that runs and then loses its bus, read from the registers that
 * tell them apart (scp_reg.h:31, :55-58, :69, :86 and scp_dvfs.h:33-37):
 *
 *  - the watchdog: cfg + 0x84 is SCP_A_WDT_REG. The firmware writes 0xfffff into it (its own
 *    literal at image offset 0x29a28, stored at 0x29a20), and the vendor kernel writes 0x8000000f
 *    there only to force a reset (scp_helper.c:801); it never kicks it and has no handler for the
 *    WDT interrupt, so a WDT reset reaches the AP as silence. cfg + 0xb8..0xc4 hold the core's
 *    SP/LR/PSP/PC as the watchdog captured them (scp_excep.c:230-233).
 *  - sleep: cfg + 0x114 is SCP_CPU_SLEEP_STATUS, and clkctrl + 0x28 is SCP_A_SLEEP_DEBUG_REG,
 *    whose values the vendor decodes as 1 idle, 2 entering sleep, 4 in sleep, 8 entering active,
 *    16 active (scp_dvfs.h:33-37, decoded in scp_dvfs.c:682-694).
 */
static void mt6785_scp_log_wdt_sleep(struct mtk_scp *scp, const char *when)
{
	void __iomem *cfg = scp->cluster->reg_base;
	u32 state;

	dev_info(scp->dev, "%s: WDT %#x, captured core SP %#x LR %#x PSP %#x PC %#x\n",
		 when, readl(cfg + MT6785_SCP_WDT_REG), readl(cfg + MT6785_SCP_WDT_SP),
		 readl(cfg + MT6785_SCP_WDT_LR), readl(cfg + MT6785_SCP_WDT_PSP),
		 readl(cfg + MT6785_SCP_WDT_PC));
	dev_info(scp->dev, "%s: watchdog control %#x (1 = enabled, set by the firmware at image 0x29a60)\n",
		 when, readl(cfg + MT6785_SCP_WDT_CTRL));
	state = readl(cfg + MT6785_SCP_CLK_BLOCK + MT6785_SCP_SLEEP_DEBUG_REG);
	dev_info(scp->dev, "%s: sleep debug %#x (%s), cpu sleep status %#x\n", when, state,
		 state == 1 ? "idle" : state == 2 ? "entering sleep" : state == 4 ? "in sleep" :
		 state == 8 ? "entering active" : state == 16 ? "active" : "no known state",
		 readl(cfg + MT6785_SCP_CPU_SLEEP_STATUS));
}

/*
 * The SPM's own oscillator control, the register trial 43 read live as 0x0 with the SCP already
 * dark: SPM + 0x440, whose bits the vendor names ULPOSC_EN (bit 0), ULPOSC_RST (1), ULPOSC_CG_EN
 * (2) and ULPOSC_CLK_SEL (3) (mtk_spm_reg.h:1858-1862 in the MT6785 vendor tree). The vendor's AP
 * kernel only ever reads it - as the idle check's "VCORE_ULPOSC on" condition, bit 0
 * (mtk_idle_cond_check.c:1124) - and writes it nowhere in that tree, and the SPM resource requests
 * have no ULPOSC bit at all (MAINPLL, DRAM, CK_26M, AXI_BUS, CPU; mtk_spm_resource_req.h:12-20).
 * So whoever turns it on is the SPM's own PCM firmware, and this driver can only report it.
 */
static void mt6785_scp_log_ulposc(struct mtk_scp *scp, const char *when)
{
	static const u16 spm_ofs[] = {
		0xb8, 0xbc, 0x110, 0x4ac, 0x4c0, 0x4f8,
		0x114, 0x118, 0x11c, 0x120, 0x124,
	};
	struct regmap *spm = scp->spm_regmap;
	u32 spm_val[ARRAY_SIZE(spm_ofs)];
	unsigned int i;
	u32 val;

	if (IS_ERR_OR_NULL(spm))
		return;

	if (regmap_read(spm, MT6785_SPM_ULPOSC_CON, &val))
		return;

	dev_info(scp->dev, "%s: SPM ULPOSC_CON %#x (en %u, rst %u, cg %u, sel %u)\n",
		 when, val, val & BIT(0) ? 1 : 0, val & BIT(1) ? 1 : 0,
		 val & BIT(2) ? 1 : 0, val & BIT(3) ? 1 : 0);

	/*
	 * Everything the SPM holds about requests and about the SCP's own channel to it, from the
	 * vendor's MT6785 register map (base 0x10006000):
	 *
	 *  - SPM_SRC_REQ (0xb8) holds one request bit per master; the SCP's is bit 13
	 *    (REG_SPM_SCP_MAILBOX_REQ_LSB, mtk_spm_reg.h:834), and the SPM's request-from-SCP
	 *    baseline is bit 1 (REG_SPM_F26M_REQ, :825);
	 *  - SPM_SRC_MASK (0xbc) decides which of the SCP's source requests are *allowed* through:
	 *    bits 5-10 are SCP_SRCCLKENA_MASK_B, SCP_INFRA_REQ_MASK_B, SCP_APSRC_REQ_MASK_B,
	 *    SCP_VRF18_REQ_MASK_B, SCP_DDR_EN_MASK_B and SCP_DDR_EN2_MASK_B (:876-881);
	 *  - SRC_REQ_STA_0..4 (0x114-0x124) are the latched source-request states (:89-93);
	 *  - PCM_REG13_DATA (0x110, :88) is the PCM block's own data register;
	 *  - SPM_SCP_CON (0x4ac), SCP_SPM_MAILBOX (0x4c0) and SCP_VCORE_LEVEL (0x4f8) are the
	 *    SCP's channel and its voltage level as the SPM sees them (:295, :300, :314).
	 *
	 * Read only: this says which request arrives and which mask blocks it.
	 */
	for (i = 0; i < ARRAY_SIZE(spm_ofs); i++)
		if (regmap_read(spm, spm_ofs[i], &spm_val[i]))
			return;

	dev_info(scp->dev,
		 "%s: SPM src_req %#x src_mask %#x pcm_r13 %#x scp_con %#x scp_mbox %#x scp_vcore %#x\n",
		 when, spm_val[0], spm_val[1], spm_val[2], spm_val[3], spm_val[4], spm_val[5]);
	dev_info(scp->dev, "%s: SPM src_req_sta %#x %#x %#x %#x %#x\n", when,
		 spm_val[6], spm_val[7], spm_val[8], spm_val[9], spm_val[10]);
}

/*
 * The oscillator's control registers as the firmware itself addresses them: its calibration
 * routine loads 0xA0000220, 0xA000010C, 0xA0000110 and 0xA0000224 from its literal pool (image
 * 0x2794c-0x2795c), starts a frequency meter by setting 0x1000 and then 0x10 there, waits for the
 * meter to finish and reads a 16-bit count (image 0x2790a-0x27936), then prints "ULPOSC%d is
 * calibrated (%ld MHz)" or "ULPOSC%d is not calibrated" (strings at 0x4b1ed, 0x4b20f).
 *
 * The SCP reaches 0x10000000 through its own remap of 0xA0000000, and topckgen is what lives
 * there. The four offsets are the vendor's own frequency meter on this SoC: CLK_DBG_CFG (0x10c),
 * CLK_MISC_CFG_0 (0x110), CLK26CALI_0 (0x220) and CLK26CALI_1 (0x224) (clkdbg-mt6785.c:353-356),
 * and the vendor drives them exactly as the firmware does - CLK26CALI_0 <- 0x1000, then 0x1010,
 * wait while bit 0x10 is set, then read CLK26CALI_1 & 0xffff and scale by 26000/1024 kHz
 * (clkdbg-mt6785.c:471-499). So the firmware measures its own oscillator with the AP's meter, and
 * this reports the four registers - read only.
 */
static void mt6785_scp_log_fmeter(struct mtk_scp *scp, const char *when)
{
	static const u16 ofs[] = { 0x10c, 0x110, 0x220, 0x224 };
	struct regmap *topckgen = scp->topckgen_regmap;
	u32 meter[4], ck_cfg_0, ck_cfg_6;
	unsigned int i;

	if (IS_ERR_OR_NULL(topckgen))
		return;

	for (i = 0; i < ARRAY_SIZE(ofs); i++)
		if (regmap_read(topckgen, ofs[i], &meter[i]))
			return;
	if (regmap_read(topckgen, MT6785_TOP_CK_CFG_6, &ck_cfg_6))
		return;
	if (regmap_read(topckgen, MT6785_TOP_CK_CFG_0, &ck_cfg_0))
		return;

	dev_info(scp->dev, "%s: meter dbg %#x misc %#x cali0 %#x cali1 %#x\n",
		 when, meter[0], meter[1], meter[2], meter[3]);
	/*
	 * CK_CFG_0 (0x20) holds the SCP's own mux - select bits 16-18, power-down bit 23
	 * (clk-mt6785-top.c:473) - so it decides whether the SCP mux is on and what it selects.
	 * CK_CFG_6 (0x80) holds four others: msdc30_1 (0-2, pdn 7), top_audio (8-9, pdn 15),
	 * top_audio_intbus (16-17, pdn 23) and pwrap_ulposc (24-26, pdn 31) (:503-508) - none of
	 * them the SCP.
	 */
	dev_info(scp->dev, "%s: top ck_cfg_0 %#x (scp sel %u, scp pdn %u), ck_cfg_6 %#x\n",
		 when, ck_cfg_0, (ck_cfg_0 >> 16) & 0x7, (ck_cfg_0 >> 23) & 0x1, ck_cfg_6);
}

/* a firmware log line: it holds a space and a lowercase letter */
static bool mt6785_scp_log_like(const char *s, size_t n)
{
	bool space = false, lower = false;
	size_t i;

	for (i = 0; i < n; i++) {
		space |= s[i] == ' ';
		lower |= s[i] >= 'a' && s[i] <= 'z';
	}

	return space && lower;
}

/*
 * The SCP's own request to the SPM, as its firmware uses it: the request word is cfg + 0x20
 * (SCP_TO_SPM_REG, vendor scp_reg.h:17) and the firmware writes 1 there and then polls bit 0 of
 * cfg + 0x1c (image 0x2c2a6-0x2c2ae and 0x2c328-0x2c32c). Reporting both shows whether a request
 * was ever made and whether an acknowledgment ever came back.
 */
static void mt6785_scp_log_spm_handshake(struct mtk_scp *scp, const char *when)
{
	void __iomem *cfg = scp->cluster->reg_base;

	dev_info(scp->dev, "%s: SCP->SPM request %#x, status %#x\n", when,
		 readl(cfg + MT6785_SCP_TO_SPM_REG), readl(cfg + MT6785_SCP_SPM_STATUS));
}

/*
 * The SSPM's platform handshake, read from its reserved buffer (0x7f8c0000, 1 MiB: LK's own
 * mblock_alloc_range line and "SSPM: set reserved mem status=okay", pl_lk.txt:533, :2730).
 *
 * LK starts the SSPM itself - "[BLDR] SSPM Start without ddr reserved mode" after loading
 * tinysys-sspm (pl_lk.txt:540-548) - so unlike the SPM it needs no kick. What it does need is the
 * AP's platform init: the kernel writes the magic 0x504C5401 (PLT_INIT) at the start of that buffer
 * and again at its last word, fills the control block and sends PLT_INIT over the IPI channel
 * IPIS_C_PLATFORM, waiting for the acknowledgment (sspm/v2/sspm_plt.c:163-175 and :198-212). The
 * idle and screen-off decisions - where this SoC's ULPOSC state is a condition - are handed to the
 * SSPM through IPI_ID_SPM_SUSPEND (mt6785/mtk_sspm.c:27-49).
 *
 * Read only: the two magic words say whether any AP-side init ever happened, which no mainline
 * driver does on this SoC.
 */
static void mt6785_scp_log_sspm(struct mtk_scp *scp)
{
	void __iomem *head, *tail;
	u32 magic_head, magic_tail, ctl_magic, ctl_size;

	head = ioremap(MT6785_SSPM_RESERVED_PHYS, SZ_4K);
	if (!head) {
		dev_info(scp->dev, "SSPM buffer not mappable, nothing to report\n");
		return;
	}
	tail = ioremap(MT6785_SSPM_RESERVED_PHYS + MT6785_SSPM_RESERVED_SIZE - SZ_4K, SZ_4K);
	magic_head = readl(head);
	ctl_magic = readl(head + 0x10);
	ctl_size = readl(head + 0x14);
	magic_tail = tail ? readl(tail + SZ_4K - 4) : 0;

	dev_info(scp->dev,
		 "SSPM buffer: head %#x (PLT_INIT is %#x), tail %#x, ctrl magic %#x size %u\n",
		 magic_head, MT6785_SSPM_PLT_INIT, magic_tail, ctl_magic, ctl_size);
	if (tail)
		iounmap(tail);
	iounmap(head);
}

/*
 * The SCP's own two rails, as the regulator framework sees them. The stock kernel brings them up
 * once at boot in mt_pmic_sshub_init (vendor scp/mt6785/scp_dvfs.c:919-946): VCORE_SSHUB 575 mV and
 * VSRAM_OTHERS_SSHUB 800 mV, both enabled, RG_VR_SSHUB_MODE 0
 * (vendor pmic/mt6359/v1/pmic.c:117-147).
 * They are already configured by the board DT; this only reports what the framework holds, so a
 * rail that is still off at the release is visible in the log.
 */
static void mt6785_scp_log_rails(struct mtk_scp *scp)
{
	static const char *const names[] = { "vcore_sshub", "vsram_others_sshub" };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct regulator *r = regulator_get_optional(scp->dev, names[i]);
		int uv, en;

		if (IS_ERR(r)) {
			dev_info(scp->dev, "rail %s: not described (%ld)\n",
				 names[i], PTR_ERR(r));
			continue;
		}
		uv = regulator_get_voltage(r);
		en = regulator_is_enabled(r);
		dev_info(scp->dev, "rail %s: %d uV, %s\n", names[i], uv,
			 en > 0 ? "enabled" : en == 0 ? "disabled" : "unknown");
		regulator_put(r);
	}
}

/*
 * The SCP's own voltage-level word, which its firmware hands to the SPM: SCP_SCP2SPM_VOL_LV
 * (vendor scp_reg.h:51), the SCP's register 0xC0094 as the vendor comments it. The vendor writes it
 * from its DVFS path as one of three steps before a frequency change
 * (scp_dvfs.c:426-462 scp_vcore_request): set the PMIC's VCORE, ask DVFSRC through
 * mtk_pm_qos_update_request(&dvfsrc_scp_vcore_req, ...), and store the OPP's spm_opp
 * here - 0x0 for the lowest OPP, 0x1 for 0.625 V, 0x12 for 0.7 V, 0x28 for 0.8 V
 * (:457-462).
 *
 * This writes the lowest OPP's value, exactly as the vendor's own table has it for 110/196 MHz
 * (vendor mt6785.dts, dvfs-opp rows: <575000 800000 0xff 0x0 0x0 110 1>), and reads it
 * back; the SPM side of the same channel, SCP_VCORE_LEVEL (SPM 0x4f8), is reported with
 * the rest of the SPM block, so a value that never arrives there says the channel is not
 * being read by the PCM.
 */
static void mt6785_scp_vol_lv(struct mtk_scp *scp)
{
	void __iomem *cfg = scp->cluster->reg_base;

	writel(MT6785_SCP_VOL_LV_OPP0, cfg + MT6785_SCP_VOL_LV);
	dev_info(scp->dev, "SCP voltage level %#x written, reads back %#x\n",
		 MT6785_SCP_VOL_LV_OPP0, readl(cfg + MT6785_SCP_VOL_LV));
}

/* is this run of characters already in the loaded image? */
static bool mt6785_scp_in_image(const u8 *raw, size_t raw_len, const char *s, size_t n)
{
	size_t i;

	if (!raw || raw_len < n)
		return false;

	for (i = 0; i + n <= raw_len; i++)
		if (!memcmp(raw + i, s, n))
			return true;

	return false;
}

/*
 * The firmware's own log, after the timeout: its logger is handed a DRAM address and writes a ring
 * there (the vendor's scp_logger.c), so DRAM is where its last words are - and DRAM is ordinary
 * memory, never the SRAM of a core that owns it. The scan is bounded: only the two regions this
 * driver already maps, only printable runs of 12 characters or more, at most 32 of them, and each
 * one is compared against the loaded image so that the image's own strings are not repeated.
 */
/*
 * Where the firmware's own logger ring could live, and whether anything is in it.
 *
 * The vendor's logger is `SCP_A_LOGGER_MEM_ID` of the scp_share pool, 0x180000 bytes
 * (vendor mt6785.dts' scp_mem_tbl), and the pool on this phone is LK's "SCP-reserved"
 * carve-out at 0x9fc00000 with 3 MiB (pl_lk.txt:1806). The exact offset inside it depends on
 * which of the table's entries are enabled (vow is the first and this build may not have it),
 * so both plausible starts are looked at: after the 1 MiB sensor block (0x100000) and after
 * sensor + flp (0x139200), plus the start of the pool itself.
 *
 * Nothing here writes: three memremaps of RAM and a few words each, at the timeout.
 */
/*
 * The SCP's own clock at the timeout, and the six parents the firmware may switch to.
 *
 * comb23 read CK_CFG_0 as 0x20502, i.e. select 2: the firmware reprogrammed its own mux, which
 * means it ran far enough to do it. Select 2 of scp_parents[] (clk-mt6785-top.c:142-150) is
 * mainpll_d2_d4, the parent our bring-up enables as "SCP clock parent 2"; the vendor's OPP
 * table asks for 165 MHz on mux 2 (vendor mt6785.dts dvfs-opp row 3). This prints the select,
 * the power-down bit, the rate the clock framework reports for the selected parent and all six
 * parents' rates at that moment - a parent that silently went away would show as a rate of 0.
 */
static void mt6785_scp_clock_state(struct mtk_scp *scp, const char *when)
{
	void __iomem *clk = scp->cluster->reg_base + MT6785_SCP_CLK_BLOCK;
	unsigned int i, sel = 0;
	u32 cfg0 = 0;
	struct regmap *topckgen;

	topckgen = syscon_regmap_lookup_by_phandle(scp->dev->of_node, "mediatek,topckgen");
	if (!IS_ERR(topckgen))
		regmap_read(topckgen, MT6785_TOP_CK_CFG_0, &cfg0);

	dev_info(scp->dev, "%s: CK_CFG_0 %#x (scp sel %u, pdn %u), clk_sw %#x clk_en %#x ctrl24 %#x\n",
		 when, cfg0, (cfg0 >> 16) & 0x7, (cfg0 >> 23) & 0x1,
		 readl(clk + MT6785_SCP_CLK_SW_SEL),
		 readl(clk + MT6785_SCP_CLK_ENABLE),
		 readl(clk + MT6785_SCP_CLK_CTRL_24));

	for (i = 0; i < scp->cluster->pll_num; i++) {
		if (!scp->cluster->pll[i])
			continue;
		dev_info(scp->dev, "%s: SCP parent %u rate %lu Hz%s\n", when, i + 1,
			 clk_get_rate(scp->cluster->pll[i]),
			 ((cfg0 >> 16) & 0x7) == i + 1 ? "  <- the mux selects this one" : "");
	}
	dev_info(scp->dev, "%s: the mux rate itself is %lu Hz\n", when, clk_get_rate(scp->clk));
}

static void mt6785_scp_log_ring_probe(struct mtk_scp *scp, const char *when)
{
	static const unsigned int ofs[] = { 0x0, 0x100000, 0x139200 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ofs); i++) {
		unsigned int w, nz = 0;
		u32 first[8];
		void *m = memremap(MT6785_LK_REGION_PHYS + ofs[i], 0x100, MEMREMAP_WB);

		if (!m)
			continue;
		for (w = 0; w < ARRAY_SIZE(first); w++) {
			first[w] = readl(m + w * 4);
			if (first[w])
				nz++;
		}
		dev_info(scp->dev, "%s: pool+%#x: %u of %zu words non-zero, first: %#x %#x %#x %#x %#x %#x %#x %#x\n",
			 when, ofs[i], nz, ARRAY_SIZE(first), first[0], first[1], first[2],
			 first[3], first[4], first[5], first[6], first[7]);
		memunmap(m);
	}
}

/*
 * What has been written over the staged image by the time the start fails.
 *
 * comb22 showed the three slices changing while the firmware ran (firmware crc 0x3e3c38ba ->
 * 0x5b052a57, dram 0x8797730b -> 0x2278a4e1, backup -> 0x06dad136) and the region's text coming
 * back garbled. There is no remap register behind this: the vendor kernel only ioremaps
 * ap_dram_start (scp_helper.c:1726-1731) and h16 measured that the core reaches 0x9fc00800
 * directly, so a wrong SCP->DRAM mapping cannot be the cause - something the firmware ran
 * (its own copy, its crash path, a log or dump write) lands on the image.
 *
 * This walks the three slices word by word against the image records they were staged from and
 * prints each run of differing words with its region offset, its length and the first pair of
 * values, which is what names the range and its shape (a dump header, text, or noise).
 */
static void mt6785_scp_diff_region(struct mtk_scp *scp)
{
	struct slice { size_t region_ofs, len, img_ofs; const char *name; };
	struct slice slices[2];
	u8 *buf;
	size_t total = MT6785_SCP_FIRMWARE_TCM_MAX + MT6785_LK_DRAM_OFS;
	unsigned int s;
	size_t total_runs = 0;

	if (!mt6785_region_ref.img || !mt6785_region_ref.fw_len) {
		dev_err(scp->dev, "diff: no staged reference kept, nothing to compare\n");
		return;
	}

	slices[0].region_ofs = MT6785_LK_FIRMWARE_OFS;
	slices[0].len = mt6785_region_ref.fw_len;
	slices[0].img_ofs = mt6785_region_ref.fw_ofs;
	slices[0].name = "firmware";
	slices[1].region_ofs = MT6785_LK_DRAM_OFS;
	slices[1].len = mt6785_region_ref.dram_len;
	slices[1].img_ofs = mt6785_region_ref.dram_ofs;
	slices[1].name = "dram part";

	/*
	 * kvzalloc: 1.8 MiB does not come from the slab for an allocation that has to succeed on
	 * the failure path - an earlier version used kzalloc and returned silently when it could
	 * not, which is why one trial printed no report at all.
	 */
	/* the four areas' checksums as they stand now, beside the ones printed before the release */
	dev_err(scp->dev, "diff: crc now - loader %#010x, firmware %#010x, dram %#010x, backup %#010x\n",
		mt6785_scp_crc_region(scp, 0, MT6785_SCP_PROBE_OFS),
		mt6785_scp_crc_region(scp, MT6785_LK_FIRMWARE_OFS, MT6785_SCP_FIRMWARE_TCM_MAX),
		mt6785_scp_crc_region(scp, MT6785_LK_DRAM_OFS, MT6785_LK_DRAM_SIZE),
		mt6785_scp_crc_region(scp, MT6785_LK_DRAM_OFS * 2, MT6785_LK_DRAM_SIZE));

	buf = kvzalloc(total, GFP_KERNEL);
	if (!buf) {
		dev_err(scp->dev, "diff: no memory for the %zu-byte comparison\n", total);
		return;
	}

	for (s = 0; s < ARRAY_SIZE(slices); s++) {
		const u8 *img = mt6785_region_ref.img + slices[s].img_ofs;
		size_t i, n = slices[s].len, runs = 0;

		memcpy_fromio(buf, scp->lk_base + slices[s].region_ofs, n);
		for (i = 0; i + 4 <= n && runs < 8; ) {
			size_t start, words = 0;

			if (!memcmp(buf + i, img + i, 4)) {
				i += 4;
				continue;
			}
			start = i;
			while (i + 4 <= n && memcmp(buf + i, img + i, 4)) {
				i += 4;
				words++;
			}
			runs++;
			dev_err(scp->dev, "%s+%#zx (%zu bytes, %zu word(s)): region %08x %08x %08x %08x, image %08x %08x %08x %08x\n",
				slices[s].name, start, words * 4, words,
				get_unaligned_le32(buf + start),
				get_unaligned_le32(buf + start + 4),
				get_unaligned_le32(buf + start + 8),
				get_unaligned_le32(buf + start + 12),
				get_unaligned_le32(img + start),
				get_unaligned_le32(img + start + 4),
				get_unaligned_le32(img + start + 8),
				get_unaligned_le32(img + start + 12));
		}
		dev_err(scp->dev, "%s: %zu byte(s) compared, %zu run(s) of damage%s\n",
			slices[s].name, n, runs, runs == 8 ? " (first eight shown)" : "");
		total_runs += runs;
	}
	kfree(buf);
	dev_err(scp->dev, "diff: %zu range(s) of damage in the staged slices%s\n",
		total_runs, total_runs ? "" : " - nothing wrote over the image in this run");
}

static void mt6785_scp_scan_dram_text(struct mtk_scp *scp)
{
	/* DRAM only: the loader region the driver already maps, and the reserved pool */
	struct { void __iomem *base; size_t size; const char *what; bool mapped; } regions[] = {
		{ scp->lk_base, SZ_1M, "loader region", true },
		{ NULL, 4 * SZ_1M, "scp-memory", false },
	};
	u8 *buf;
	const u8 *raw = scp->image;
	size_t raw_len = scp->image_size;
	unsigned int r, found = 0;
	const u8 *p;
	size_t ofs;
	bool same;

	buf = kzalloc(SZ_1M, GFP_KERNEL);
	if (!buf)
		return;

	regions[1].base = ioremap(MT6785_SCP_DRAM_PHYS, regions[1].size);
	if (!regions[1].base)
		regions[1].size = 0;

	for (r = 0; r < ARRAY_SIZE(regions); r++) {
		if (!regions[r].base || !regions[r].size)
			continue;
		memcpy_fromio(buf, regions[r].base, SZ_1M);
		for (p = buf; p < buf + SZ_1M - 12 && found < 32; p++) {
			size_t n = 0;

			while (p + n < buf + SZ_1M && p[n] >= 0x20 && p[n] < 0x7f)
				n++;
			if (n >= 12) {
				const char *str = (const char *)p;

				if (!regions[r].mapped && !mt6785_scp_log_like(str, n))
					continue;
				/*
				 * Whether a run is image data or text written at run time has to be
				 * decided at the same offset: a string that sits in the image at the same
				 * place is the image's, whatever the search over the whole file says
				 * (the region may hold a different build of the same image than the one
				 * this boot requested, which is what made the earlier scan unreadable).
				 */
				ofs = (size_t)(p - buf);
				same = ofs + n <= raw_len && raw &&
				       !memcmp(raw + ofs, str, n);
				if (!same) {
					dev_err(scp->dev, "%s+%#zx: %.*s%s\n",
						regions[r].what, ofs, (int)n, str,
						mt6785_scp_in_image(raw, raw_len, str, n) ?
						"   [in the image elsewhere]" :
						"   [written at run time]");
					found++;
				}
				p += n;
			}
		}
	}
	dev_err(scp->dev, "DRAM text scan: %u line(s) differ from the image at their own offset\n",
		found);
	if (regions[1].base)
		iounmap(regions[1].base);
	kfree(buf);
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

/*
 * "enable sleep protection": hold the SCP's infra and audio paths while it is brought up. The
 * vendor's own order is protection up, then the SRAM and the images, then the protection down
 * (scp_awake.c:224-244 and :285-296), and it takes it on every boot, not only on a recovery.
 */
static int mt6785_scp_slp_prot_set(struct mtk_scp *scp)
{
	return mt6785_infracfg_reset(scp, "sleep protect",
				      MT6785_INFRACFG_SLP_PROT_SET,
				      MT6785_INFRACFG_SLP_PROT_STAT,
				      MT6785_SCP_TO_INFRA_BIT |
				      MT6785_SCP_TO_AUDIO_BIT);
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

	if (regmap_read(infracfg, MT6785_INFRACFG_SLP_PROT_SET, &val))
		return 0;
	dev_info(scp->dev, "sleep protect released: %#x = %#x (bits %#x must be 0)\n",
		 MT6785_INFRACFG_SLP_PROT_SET, val,
		 MT6785_SCP_TO_INFRA_BIT | MT6785_SCP_TO_AUDIO_BIT);

	return 0;
}

/*
 * LK loads the SCP itself and leaves its work in a region of its own: "SCP-reserved" at
 * 0x9fc00000 with 3 MiB (pl_lk.txt:2305-2306), the loader image at ap_loader_start (its own
 * record, 0x438 bytes), the firmware staged at ap_firmware_start 0x9fc00800 with 0xbb800
 * bytes, the DRAM part at 0x9fd00000 with 0x59968, and the region information it wrote into
 * the SRAM (pl_lk.txt:2406-2414). That region is reserved in this board's device tree, so the
 * copies are still there when this driver probes - and they can be read without touching the
 * SCP at all, which is what this does: it validates and reports them, so a trial says whether
 * the loader's work survived instead of inferring it from SCP register reads that a running
 * core may not answer.
 */
/*
 * A checksum of one of the images in LK's region. The AP cannot read the SCP's memory once the
 * core owns the bus, but DRAM is plain memory: the loader and the firmware write into their DRAM
 * and backup areas when they run, so a checksum taken before the release and again at a timeout
 * says whether anything executed at all.
 */
static u32 mt6785_scp_crc_region(struct mtk_scp *scp, u32 ofs, size_t len)
{
	u8 *buf;
	u32 crc = ~0;
	size_t done = 0, chunk = SZ_4K;

	if (!scp->lk_base || ofs + len > scp->lk_size)
		return 0;

	buf = kmalloc(chunk, GFP_KERNEL);
	if (!buf)
		return 0;

	while (done < len) {
		size_t n = min(chunk, len - done);

		memcpy_fromio(buf, scp->lk_base + ofs + done, n);
		crc = crc32_le(crc, buf, n);
		done += n;
	}

	kfree(buf);
	return ~crc;
}

/* the three images' checksums, in one line each caller can compare with the next */
static void mt6785_scp_log_dram_state(struct mtk_scp *scp, const char *when)
{
	if (!scp->lk_base)
		return;

	dev_info(scp->dev, "dram %s: firmware %#x crc %#010x, dram %#x crc %#010x, backup crc %#010x\n",
		 when, MT6785_SCP_FIRMWARE_TCM_MAX,
		 mt6785_scp_crc_region(scp, MT6785_LK_FIRMWARE_OFS,
				       MT6785_SCP_FIRMWARE_TCM_MAX),
		 MT6785_LK_DRAM_SIZE,
		 mt6785_scp_crc_region(scp, MT6785_LK_DRAM_OFS, MT6785_LK_DRAM_SIZE),
		 mt6785_scp_crc_region(scp, MT6785_LK_DRAM_OFS * 2, MT6785_LK_DRAM_SIZE));
}

/* a checksum of the TCM range the firmware was staged into, and its non-zero pages */
static void mt6785_scp_log_tcm(struct mtk_scp *scp, const char *when)
{
	u8 *buf;
	u32 crc = ~0, page, first = 0, total = MT6785_SCP_FIRMWARE_TCM_MAX / SZ_4K;
	bool in_range = false;
	size_t done;

	if (!scp->sram_base)
		return;

	buf = kmalloc(SZ_4K, GFP_KERNEL);
	if (!buf)
		return;

	for (done = 0; done < MT6785_SCP_FIRMWARE_TCM_MAX; done += SZ_4K) {
		size_t n = min((size_t)SZ_4K, MT6785_SCP_FIRMWARE_TCM_MAX - done);
		bool nz = false;
		size_t i;

		memcpy_fromio(buf, scp->sram_base + done, n);
		crc = crc32_le(crc, buf, n);
		for (i = 0; i < n; i++)
			if (buf[i]) {
				nz = true;
				break;
			}
		page = done / SZ_4K;
		if (nz && !in_range) {
			first = page;
			in_range = true;
		} else if (!nz && in_range) {
			dev_info(scp->dev, "  %s TCM pages %#x..%#x hold data\n", when,
				 first * SZ_4K, page * SZ_4K - 1);
			in_range = false;
		}
	}
	if (in_range)
		dev_info(scp->dev, "  %s TCM pages %#x..%#x hold data\n", when,
			 first * SZ_4K, total * SZ_4K - 1);

	dev_err(scp->dev, "TCM %s: [0x0..%#x] crc %#010x, first words %#x/%#x\n",
		when, MT6785_SCP_FIRMWARE_TCM_MAX, ~crc,
		readl(scp->sram_base), readl(scp->sram_base + 4));

	kfree(buf);
}

/* one zone's first and last 256 bytes, hex, 32 bytes a line */
static void mt6785_scp_hexdump(struct mtk_scp *scp, u32 ofs, size_t len,
			       const char *label)
{
	u8 buf[32];
	size_t at, end;

	if (!scp->lk_base || ofs + len > scp->lk_size)
		return;

	dev_info(scp->dev, "%s: first 0x%zx bytes at %#x\n", label, (size_t)256, ofs);
	for (at = 0; at < 256 && at < len; at += sizeof(buf)) {
		size_t n = min(sizeof(buf), len - at);

		memcpy_fromio(buf, scp->lk_base + ofs + at, n);
		dev_info(scp->dev, "  %04zx: %*ph\n", at, (int)n, buf);
	}

	end = len > 256 ? len - 256 : 0;
	dev_info(scp->dev, "%s: last 0x100 bytes ending at %#zx\n", label, ofs + len);
	for (at = end; at < len; at += sizeof(buf)) {
		size_t n = min(sizeof(buf), len - at);

		memcpy_fromio(buf, scp->lk_base + ofs + at, n);
		dev_info(scp->dev, "  %04zx: %*ph\n", at, (int)n, buf);
	}
}

/* which 4 KiB pages of the region hold anything, as ranges */
static void mt6785_scp_log_pages(struct mtk_scp *scp, const char *when)
{
	u32 page, first = 0, total = scp->lk_size / SZ_4K;
	bool in_range = false, any = false;
	u8 buf[16];
	size_t i, n;
	int dumped = 0;

	if (!scp->lk_base)
		return;

	for (page = 0; page < total; page++) {
		bool nz = false;

		for (i = 0; i < SZ_4K && !nz; i += sizeof(buf)) {
			memcpy_fromio(buf, scp->lk_base + page * SZ_4K + i, sizeof(buf));
			for (n = 0; n < sizeof(buf); n++)
				if (buf[n]) {
					nz = true;
					break;
				}
		}

		if (nz && !in_range) {
			first = page;
			in_range = true;
			any = true;
		} else if (!nz && in_range) {
			dev_info(scp->dev, "  %s non-zero pages %#x..%#x\n", when,
				 first * SZ_4K, page * SZ_4K - 1);
			if (++dumped > 40)
				break;
			in_range = false;
		}
	}
	if (in_range)
		dev_info(scp->dev, "  %s non-zero pages %#x..%#x\n", when,
			 first * SZ_4K, total * SZ_4K - 1);
	if (!any)
		dev_info(scp->dev, "  %s the whole region is zeros\n", when);
}

/*
 * Any printable text in the region: the firmware keeps a log ring in its DRAM, and if it is in
 * this region its text says where the firmware stopped (vendor scp_logger.c runs the AP-side
 * reader, the ring itself lives in the SCP's DRAM).
 */
static void mt6785_scp_scan_text(struct mtk_scp *scp)
{
	u8 *buf;
	size_t len = scp->lk_size, i, start = 0, run = 0;
	int shown = 0;

	if (!scp->lk_base)
		return;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;
	memcpy_fromio(buf, scp->lk_base, len);

	for (i = 0; i < len; i++) {
		bool printable = buf[i] >= 0x20 && buf[i] < 0x7f;

		if (printable) {
			if (!run)
				start = i;
			run++;
			continue;
		}
		if (run >= 24 && shown < 12) {
			dev_info(scp->dev, "  text at %#zx: %.*s\n", start,
				 (int)min(run, (size_t)96), buf + start);
			shown++;
		}
		run = 0;
	}
	if (!shown)
		dev_info(scp->dev, "  no printable text in the region\n");

	kfree(buf);
}

/* the named windows this SoC's bring-up uses beyond sram and cfg */
static void mt6785_scp_map_windows(struct platform_device *pdev, struct mtk_scp *scp)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "l1c");
	if (res)
		scp->l1c_base = devm_ioremap_resource(&pdev->dev, res);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lk-images");
	if (!res)
		return;
	scp->lk_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!scp->lk_base)
		return;
	scp->lk_phys = res->start;
	scp->lk_size = resource_size(res);
}

static void mt6785_scp_probe_lk_images(struct mtk_scp *scp)
{
	u32 sp, pc;

	if (!scp->lk_base) {
		dev_info(scp->dev, "no loader region named: nothing to probe\n");
		return;
	}

	sp = readl(scp->lk_base + MT6785_LK_LOADER_SP);
	pc = readl(scp->lk_base + MT6785_LK_LOADER_PC);
	dev_info(scp->dev, "loader region %#llx+%#zx: initial SP %#x reset vector %#x%s\n",
		 (u64)scp->lk_phys, (size_t)scp->lk_size, sp, pc,
		 (sp && sp < 0x10000 && pc && pc < 0x10000) ?
		 " (looks like the loader's vector table)" : " (not a vector table)");
	dev_info(scp->dev, "loader copies: firmware at %#llx, dram at %#llx\n",
		 (u64)scp->lk_phys + MT6785_LK_FIRMWARE_OFS,
		 (u64)scp->lk_phys + MT6785_LK_DRAM_OFS);
}

/*
 * Find one record of the MTK image by name and return its payload, in the order LK walks them
 * (a 512-byte header, the payload, then the next header 16-byte aligned after it).
 */
static const void *mtk_img_find(const struct firmware *fw, const char *want,
				size_t *len)
{
	const u8 *buf = fw->data;
	size_t off = 0;

	while (off + MTK_IMG_HDR_SIZE <= fw->size) {
		const struct mtk_img_header *h = (const void *)(buf + off);
		u32 dsize = le32_to_cpu(h->dsize);
		size_t next;

		if (le32_to_cpu(h->magic) != MTK_IMG_MAGIC)
			return NULL;
		if (le32_to_cpu(h->ext_magic) != MTK_IMG_EXT_MAGIC ||
		    le32_to_cpu(h->hdr_size) != MTK_IMG_HDR_SIZE)
			return NULL;
		if (off + MTK_IMG_HDR_SIZE + dsize > fw->size)
			return NULL;

		if (!strncmp(h->name, want, MTK_IMG_NAME_SIZE)) {
			*len = dsize;
			return buf + off + MTK_IMG_HDR_SIZE;
		}

		next = off + MTK_IMG_HDR_SIZE + dsize;
		next = round_up(next, MTK_IMG_ALIGN);
		if (next <= off)
			return NULL;
		off = next;

		if (le32_to_cpu(h->img_list_end))
			break;
	}

	return NULL;
}

/*
 * Stage the vendor's own image the way LK does it, into this port's carve-out: the loader into
 * the TCM, the scp record where the loader pulls it from (it copies ap_firmware_size bytes to
 * core 0x800 itself, so only the part TCM can hold is announced - LK's own number for this
 * record is 0xbb800, pl_lk.txt:2407), the record's tail and the dram record in the carve-out,
 * and the region information pointing at all of it with LK's cache configuration.
 *
 * The carve-out is 0x50000000 with 4 MiB, this port's own reserved-memory node: the vendor's
 * 0x9fc00000 cannot be relied on because a kernel without that reservation (the base kernel
 * before a kexec trial) hands it to the allocator.
 */
static int mt6785_scp_stage_mtk_image(struct mtk_scp *scp, const struct firmware *fw)
{
	struct mt6785_scp_region_info ri = { 0 };
	size_t loader_len = 0, scp_len = 0, dram_len = 0;
	const void *loader, *scp_img, *dram;
	u32 fw_len, rest_len;
	/* the images go where the SCP master may read them: LK's own region */
	dma_addr_t base = scp->lk_phys;

	loader = mtk_img_find(fw, "tinysys-loader-CM4_A", &loader_len);
	scp_img = mtk_img_find(fw, "tinysys-scp-CM4_A", &scp_len);
	dram = mtk_img_find(fw, "tinysys-scp-CM4_A_dram", &dram_len);
	if (!loader || !scp_img || !dram) {
		dev_err(scp->dev, "not an MTK SCP image: loader %d scp %d dram %d\n",
			!!loader, !!scp_img, !!dram);
		return -EINVAL;
	}
	if (loader_len > MT6785_SCP_LOADER_SLOT ||
	    MT6785_LK_DRAM_OFS + dram_len > scp->lk_size) {
		dev_err(scp->dev, "image does not fit: loader %#zx dram %#zx\n",
			loader_len, dram_len);
		return -EINVAL;
	}

	/*
	 * LK's own layout, address for address (pl_lk.txt:2406-2411): the firmware's TCM part at
	 * +0x800 with 0xbb800 bytes - the loader copies exactly that to core 0x800, so anything
	 * beyond it is not staged at all - and the dram record at +0x100000. The backup slot stays
	 * free for the vendor's own copy.
	 */
	fw_len = min_t(size_t, scp_len, MT6785_SCP_FIRMWARE_TCM_MAX);
	rest_len = 0;
	scp->mtk_dram_ofs = MT6785_LK_DRAM_OFS;

	memcpy_toio(scp->sram_base, loader, loader_len);
	/* LK's own copy puts the initial stack pointer and the reset vector here (0x538, 0x1e1) */
	dev_info(scp->dev, "loader in the TCM: SP %#x reset vector %#x, %#zx bytes from the firmware\n",
		 readl(scp->sram_base), readl(scp->sram_base + 4), loader_len);
	if (!scp->lk_base) {
		dev_err(scp->dev, "no region to stage the firmware into\n");
		return -ENODEV;
	}
	memcpy_toio(scp->lk_base + MT6785_LK_FIRMWARE_OFS, scp_img, fw_len);
	/*
	 * Keep where those bytes came from, so the post-mortem can compare the region with
	 * the image word by word and name the ranges something wrote over (see
	 * mt6785_scp_diff_region()).
	 */
	mt6785_region_ref.img = scp->image;
	mt6785_region_ref.fw_ofs = (const u8 *)scp_img - (const u8 *)scp->image;
	mt6785_region_ref.fw_len = fw_len;
	mt6785_region_ref.dram_ofs = (const u8 *)dram - (const u8 *)scp->image;
	mt6785_region_ref.dram_len = dram_len;
	memcpy_toio(scp->lk_base + scp->mtk_dram_ofs, dram, dram_len);
	/* the loader is staged in that region as well, where LK puts it */
	memcpy_toio(scp->lk_base, loader, loader_len);

	ri.ap_loader_start = base;
	ri.ap_loader_size = MT6785_SCP_LOADER_SLOT;
	ri.ap_firmware_start = base + MT6785_LK_FIRMWARE_OFS;
	ri.ap_firmware_size = fw_len;
	ri.ap_dram_start = base + scp->mtk_dram_ofs;
	ri.ap_dram_size = dram_len;
	ri.ap_dram_backup_start = base + MT6785_LK_DRAM_OFS * 2;
	ri.il1c_con = MT6785_SCP_L1C_CON;
	ri.dl1c_con = MT6785_SCP_L1C_CON;
	/* the remaining fields LK prints as zero for this board (pl_lk.txt:2411-2414) */
	ri.scp_log_thru_ap_uart = 0;
	ri.scpctl = 0;
	ri.ap_params_start = 0;
	ri.struct_size = sizeof(ri);
	memcpy_toio(scp->sram_base + MT6785_SCP_REGION_INFO_OFFSET, &ri, sizeof(ri));
	/* the loader must see the region info before the core is released */
	wmb();

	dev_info(scp->dev, "staged: loader %#zx to the TCM, firmware %#x at %#llx, dram %#zx at %#llx, l1c %#x/%#x\n",
		 loader_len, fw_len, (u64)ri.ap_firmware_start, dram_len,
		 (u64)ri.ap_dram_start, ri.il1c_con, ri.dl1c_con);

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

/*
 * Every register this driver touches has to be inside a window the device tree gives it: the
 * MT6785 bring-up writes cfg + 0x0, + 0x1c, + 0x28, + 0x114 and the SCP clock block at
 * + 0x4000..+ 0x40a0, and reads the TCM. audio25 proved why this is not paranoia - a write at
 * an offset this SoC does not have (mt8183's cache registers at 0x14000) is a paging fault
 * inside an interrupt-free probe path, i.e. a boot that ends in pstore.
 */
static int mt6785_scp_check_windows(struct mtk_scp *scp)
{
	static const struct {
		const char *what;
		resource_size_t off;
		resource_size_t size;
		bool in_sram;
	} used[] = {
		{ "sw_rstn", 0x0, 0x4, false },
		{ "to_host", 0x1c, 0x4, false },
		{ "gipc_in", 0x28, 0x4, false },
		{ "sleep status", 0x114, 0x4, false },
		{ "sram pdn", 0x402c, 0x4, false },
		{ "l1 sram pd", 0x4080, 0x4, false },
		{ "tcm tail sram pd", 0x4094, 0x4, false },
		{ "region info", MT6785_SCP_REGION_INFO_OFFSET, 0x40, true },
		{ "vector table", 0x0, 0x8, true },
	};
	resource_size_t cfg_size = scp->cluster->cfg_size;
	resource_size_t sram_size = scp->sram_size;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(used); i++) {
		resource_size_t limit = used[i].in_sram ? sram_size : cfg_size;

		if (used[i].off + used[i].size > limit) {
			dev_err(scp->dev, "%s at %#x is outside the %s window (%#x bytes)\n",
				used[i].what, (u32)used[i].off,
				used[i].in_sram ? "sram" : "cfg", (u32)limit);
			return -ERANGE;
		}
	}

	return 0;
}

/*
 * The EMI MPU's view of the SCP's DRAM window, read through the secure monitor, and LK's own
 * permission words put back over it.
 *
 * Why this matters: the firmware's first act after the loader copy is to read its own DRAM
 * (ap_dram_start, 0x9fd00000) and its log ring; if the MPU forbids the SCP's domain there, the
 * core stalls on that first access and never reaches its ready IPI, which is what this phone
 * shows. The reference is LK's own programming of exactly this window, printed by LK itself:
 *
 *   LK emi_mpu_set_protection start:9fc00000 end:9fefffff region=6
 *      apc1=b6db6d apc0=b6d16c      (pl_lk.txt:1915)
 *
 * and decoded with the 16-domain layout of emi/submodule/mpu_v1.h:43-57 it allows the AP reads
 * (d0 = 4), the SCP domain (d3 = 0) and three further domains (d8-d10 = 0) while forbidding the
 * rest - i.e. the reference already lets the SCP reach its window.
 *
 * So nothing is invented here: the region is read back and compared with those exact numbers,
 * and then set again to the same values (the vendor's own call shape, mpu_v1.c:222-223). If the
 * secure monitor does not implement the call, its return value says so and the boot continues.
 */
static u32 mt6785_scp_mpu_read(u32 type, u32 region, u32 group)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MT6785_EMIMPU_CONTROL, MT6785_EMIMPU_READ, type, region, group,
		      0, 0, 0, &res);

	return res.a0;
}

/*
 * Every region the MPU holds, so the answer to "is the read call real" comes from the probe
 * itself: LK programs at least a dozen regions (pl_lk.txt:1915-2206), and they must show up here.
 * A sweep that prints nothing at all means the read call reports nothing on this phone - and then
 * the set call next to it is a no-op to be judged the same way.
 */
static void mt6785_scp_mpu_dump(struct mtk_scp *scp, const char *when)
{
	unsigned int r, shown = 0;

	for (r = 0; r < MT6785_EMIMPU_REGION_NUM; r++) {
		u32 sa = mt6785_scp_mpu_read(MT6785_EMIMPU_READ_SA, r, 0);
		u32 ea = mt6785_scp_mpu_read(MT6785_EMIMPU_READ_EA, r, 0);
		u32 apc0 = mt6785_scp_mpu_read(MT6785_EMIMPU_READ_APC, r, 0);
		u32 apc1 = mt6785_scp_mpu_read(MT6785_EMIMPU_READ_APC, r, 1);

		if (!sa && !ea && !apc0 && !apc1)
			continue;
		shown++;
		dev_info(scp->dev, "%s: MPU region %u = %#x..%#x apc0 %#x apc1 %#x\n",
			 when, r, sa, ea, apc0, apc1);
	}
	dev_info(scp->dev, "%s: %u of %u MPU regions carry a value\n",
		 when, shown, MT6785_EMIMPU_REGION_NUM);
}

static void mt6785_scp_mpu_probe(struct mtk_scp *scp)
{
	struct arm_smccc_res res;
	const u32 lk_apc[2] = { MT6785_EMIMPU_SCP_APC0, MT6785_EMIMPU_SCP_APC1 };
	u32 sa, ea, apc[2];
	int g;

	mt6785_scp_mpu_dump(scp, "before the MPU set");

	arm_smccc_smc(MT6785_EMIMPU_CONTROL, MT6785_EMIMPU_READ,
		      MT6785_EMIMPU_READ_SA, MT6785_EMIMPU_SCP_REGION, 0, 0, 0, 0, &res);
	sa = res.a0;
	arm_smccc_smc(MT6785_EMIMPU_CONTROL, MT6785_EMIMPU_READ,
		      MT6785_EMIMPU_READ_EA, MT6785_EMIMPU_SCP_REGION, 0, 0, 0, 0, &res);
	ea = res.a0;
	for (g = 0; g < ARRAY_SIZE(apc); g++) {
		arm_smccc_smc(MT6785_EMIMPU_CONTROL, MT6785_EMIMPU_READ,
			      MT6785_EMIMPU_READ_APC, MT6785_EMIMPU_SCP_REGION, g,
			      0, 0, 0, &res);
		apc[g] = res.a0;
	}

	dev_info(scp->dev, "EMI MPU region %u now: raw start %#x end %#x apc0 %#x apc1 %#x (LK's: %#x %#x, apc0 %#x apc1 %#x)\n",
		 MT6785_EMIMPU_SCP_REGION, sa, ea, apc[0], apc[1],
		 MT6785_EMIMPU_SCP_START >> MT6785_EMIMPU_ALIGN_BITS,
		 MT6785_EMIMPU_SCP_END >> MT6785_EMIMPU_ALIGN_BITS,
		 lk_apc[0], lk_apc[1]);
	dev_info(scp->dev, "EMI MPU region %u: d0 %u, d3 %u, d8 %u, d9 %u, d10 %u (0 = allowed, 5 = forbidden)\n",
		 MT6785_EMIMPU_SCP_REGION, (apc[0] >> 0) & 0x7, (apc[0] >> 9) & 0x7,
		 (apc[1] >> 0) & 0x7, (apc[1] >> 3) & 0x7, (apc[1] >> 6) & 0x7);

	if (apc[0] == lk_apc[0] && apc[1] == lk_apc[1] &&
	    sa == (MT6785_EMIMPU_SCP_START >> MT6785_EMIMPU_ALIGN_BITS) &&
	    ea == (MT6785_EMIMPU_SCP_END >> MT6785_EMIMPU_ALIGN_BITS))
		dev_info(scp->dev, "EMI MPU region %u still holds LK's own words: the SCP's domain is allowed\n",
			 MT6785_EMIMPU_SCP_REGION);

	/* put the reference back: one call per domain group, group 1 first, as the vendor does */
	for (g = ARRAY_SIZE(apc) - 1; g >= 0; g--) {
		arm_smccc_smc(MT6785_EMIMPU_CONTROL, MT6785_EMIMPU_SET,
			      (MT6785_EMIMPU_SCP_REGION << 24) |
			      (MT6785_EMIMPU_SCP_START >> MT6785_EMIMPU_ALIGN_BITS),
			      (g << 24) |
			      (MT6785_EMIMPU_SCP_END >> MT6785_EMIMPU_ALIGN_BITS),
			      lk_apc[g], 0, 0, 0, &res);
		dev_info(scp->dev, "EMI MPU set region %u group %d to %#x: returned %ld\n",
			 MT6785_EMIMPU_SCP_REGION, g, lk_apc[g], (long)res.a0);
	}

	mt6785_scp_mpu_dump(scp, "after the MPU set");
}

/*
 * The loader's own job, done by hand, at the loader's own reset vector.
 *
 * h15 proved the core executes our code from the TCM, and h16 proved it reads DRAM at
 * 0x9fc00800 (`9fc00800 -> 0xb0180`, the same word the AP sees). So what is left of the loader
 * is its copy: the firmware's 0xbb800 bytes from 0x9fc00800 into the TCM at core 0x800, and then
 * the jump to 0x801 (the loader's own literals are exactly 0x800 for the destination). This
 * program does that itself, 56 bytes, and writes a progress word to TCM 0x610 so the AP can see
 * how far it got:
 *
 *   1 = about to copy      2 = copy finished      3 = about to jump to the firmware
 *
 *   1e0: ldr r5,[pc] -> 0x610 (progress), r7 -> 0x620 (word index), movs r3,#1, str
 *   1e6: ldr r0 -> 0x9fc00800, r1 -> 0x800, r2 -> 0x2ee00 words, r6 -> 0
 *   1f0: loop: ldr r3,[r0]; str r3,[r1]; adds r0,#4; adds r1,#4; adds r6,#1
 *   1fa:       lsls r3,r6,#24; bne skip; str r6,[r7]   (every 256 words the index is published)
 *   200: skip: subs r2,#1; bne loop
 *   204: movs r3,#2; str r3,[r5]   ldr r0 -> 0x801   movs r3,#3; str r3,[r5]; bx r0
 *
 * Hand-encoded Thumb16, no assembler: ldr rX,[pc] = 0x48xx|X, ldr rX,[rY] = 0x68|Y<<3|X,
 * str rX,[rY] = 0x60|Y<<3|X, movs rX,#imm = 0x20|X<<8|imm, adds rX,#4 = 0x30|X<<8|4,
 * subs rX,#1 = 0x38|X<<8|1, bne = 0xd1xx, bx = 0x4700|X.
 *
 * If the copy is what the real loader dies in, 0x610 stays at 1 and nothing else moves; if the
 * copy finishes and the jump is taken, the firmware is executing from TCM 0x800 and its own log
 * buffer becomes the next place to look (the post-mortem's DRAM text scan already covers it).
 *
 * Test scaffolding for one trial, not for upstream: it replaces the loader, so nothing the real
 * loader would do happens while it is in place, and deleting its two calls is the revert.
 */
static const u8 mt6785_scp_probe_code[] = {
	0x0b, 0x4d, 0x01, 0x23, 0x2b, 0x60, 0x0b, 0x4f, 0x00, 0x26, 0x0b, 0x48,
	0x0b, 0x49, 0x0c, 0x4a, 0x03, 0x68, 0x0b, 0x60, 0x04, 0x30, 0x04, 0x31,
	0x01, 0x36, 0x33, 0x06, 0x00, 0xd1, 0x3e, 0x60, 0x01, 0x3a, 0xf5, 0xd1,
	0x02, 0x23, 0x2b, 0x60, 0x06, 0x48, 0x03, 0x23, 0x2b, 0x60, 0x00, 0x47,
	0x10, 0x06, 0x00, 0x00, 0x20, 0x06, 0x00, 0x00, 0x00, 0x08, 0xc0, 0x9f,
	0x00, 0x08, 0x00, 0x00, 0x00, 0xee, 0x02, 0x00, 0x01, 0x08, 0x00, 0x00,
};

#define MT6785_SCP_PROBE_OFS	0x1e0
#define MT6785_SCP_PROBE_PROGRESS	0x610
/* the copy's word index, published every 256 words so the AP can watch the transfer move */
#define MT6785_SCP_PROBE_INDEX		0x620
#define MT6785_SCP_PROBE_COPY_STARTED	1
#define MT6785_SCP_PROBE_COPY_DONE	2
#define MT6785_SCP_PROBE_JUMPING	3
/* where the copy lands, and the word it should hold: the first word of the firmware record */
#define MT6785_SCP_PROBE_DEST	0x800

static void mt6785_scp_dram_probe(struct mtk_scp *scp)
{
	u32 sp = readl(scp->sram_base);
	u32 vec = readl(scp->sram_base + 4);

	writel(0, scp->sram_base + MT6785_SCP_PROBE_PROGRESS);
	writel(0, scp->sram_base + MT6785_SCP_PROBE_INDEX);
	/* the destination is spelled out so a stale word from a previous boot cannot be mistaken
	 * for a copy that happened
	 */
	writel(0, scp->sram_base + MT6785_SCP_PROBE_DEST);
	memcpy_toio(scp->sram_base + MT6785_SCP_PROBE_OFS, mt6785_scp_probe_code,
		    sizeof(mt6785_scp_probe_code));
	wmb();

	dev_info(scp->dev, "copy probe: %zu bytes at TCM %#x, vector table kept (SP %#x vector %#x), source %#x -> TCM %#x, %u words\n",
		 sizeof(mt6785_scp_probe_code), MT6785_SCP_PROBE_OFS, sp, vec,
		 MT6785_LK_REGION_PHYS + MT6785_LK_FIRMWARE_OFS, MT6785_SCP_PROBE_DEST,
		 MT6785_SCP_FIRMWARE_TCM_MAX / 4);
}

static void mt6785_scp_dram_probe_check(struct mtk_scp *scp, const char *when)
{
	void __iomem *cfg = scp->cluster->reg_base;
	void __iomem *clk = cfg + MT6785_SCP_CLK_BLOCK;
	u32 progress = readl(scp->sram_base + MT6785_SCP_PROBE_PROGRESS);
	u32 idx = readl(scp->sram_base + MT6785_SCP_PROBE_INDEX);
	u32 dst = readl(scp->sram_base + MT6785_SCP_PROBE_DEST);
	u32 src = readl(scp->lk_base + MT6785_LK_FIRMWARE_OFS);

	dev_info(scp->dev, "copy probe %s: progress %u (%s), index %u words = %u bytes = source %#x, TCM %#x = %#x, the AP reads %#x there%s\n",
		 when, progress,
		 progress == MT6785_SCP_PROBE_JUMPING ? "jumped to the firmware" :
		 progress == MT6785_SCP_PROBE_COPY_DONE ? "copy finished" :
		 progress == MT6785_SCP_PROBE_COPY_STARTED ? "still copying (or died in the loop)" :
		 "nothing",
		 idx, idx * 4, MT6785_LK_REGION_PHYS + MT6785_LK_FIRMWARE_OFS + idx * 4,
		 MT6785_SCP_PROBE_DEST, dst, src,
		 dst && dst == src ? " (the copy is in the TCM)" : "");
	if (progress == MT6785_SCP_PROBE_COPY_STARTED && !idx && !dst)
		dev_info(scp->dev, "copy probe %s: the loop has not reached its first 256-word boundary\n",
			 when);
	dev_info(scp->dev, "copy probe %s: clk_sw %#x clk_en %#x ctrl24 %#x\n", when,
		 readl(clk + MT6785_SCP_CLK_SW_SEL),
		 readl(clk + MT6785_SCP_CLK_ENABLE),
		 readl(clk + MT6785_SCP_CLK_CTRL_24));
}

static int mt6785_scp_clocks_on(struct mtk_scp *scp)
{
	struct regmap *topckgen;
	unsigned int i;
	u32 val;

	/*
	 * The parents the firmware's own DVFS may switch to, held for the whole bring-up: the
	 * vendor enables the SCP's mux clock and its PLL taps from the AP side before the
	 * release and drops them once the ready IPI has arrived (mt6785.dts:655-669,
	 * scp_dvfs.c:588 for the enable, :350 for the release).
	 */
	for (i = 0; i < scp->cluster->pll_num; i++) {
		int err = clk_prepare_enable(scp->cluster->pll[i]);

		dev_info(scp->dev, "SCP clock parent %u: %s, %lu Hz\n", i + 1,
			 err ? "failed" : "enabled",
			 err ? 0UL : clk_get_rate(scp->cluster->pll[i]));
	}

	/*
	 * The parent stock runs the SCP's mux on. With the SCP running, stock's CK_CFG_0
	 * (0x10000020) reads 0x80008302, i.e. select bits 16-18 = 2
	 * (poco-m5s/evidence/stock-session-2026-11/scp-up.txt), and select 2 of scp_parents[]
	 * is mainpll_d2_d4 at 136.5 MHz (clk-mt6785-top.c:142-150, mux at :473). Our device tree
	 * pins the mux to clk26m, and the trial reads 0x502 (select 0): the core would run 5.25
	 * times slower than the reference, which stretches every timing loop the firmware has -
	 * its watchdog kick interval and the oscillator calibration whose arithmetic audio41
	 * already matched to 26 MHz. The vendor takes this parent from the AP's clock framework
	 * before the release (scp_dvfs.c:627-643), so this does the same.
	 */
	if (scp->cluster->pll_num > 1) {
		int err = clk_set_parent(scp->clk, scp->cluster->pll[1]);

		dev_info(scp->dev, "SCP clock: parent 2 (mainpll_d2_d4) %s, %lu Hz\n",
			 err ? "refused" : "set", err ? 0UL : clk_get_rate(scp->clk));
	}

	topckgen = syscon_regmap_lookup_by_phandle(scp->dev->of_node,
						   "mediatek,topckgen");
	if (IS_ERR(topckgen)) {
		dev_err(scp->dev, "no topckgen phandle: %ld\n", PTR_ERR(topckgen));
		return PTR_ERR(topckgen);
	}

	regmap_read(topckgen, MT6785_SCP_CLK_CFG_0, &val);
	dev_info(scp->dev, "CK_CFG_0 %#x: scp select %u, power-down %u\n", val,
		 (val >> 16) & 0x7, (val >> 23) & 0x1);
	regmap_write(topckgen, MT6785_SCP_CLK_CFG_0, val | 0x3ff);
	regmap_read(topckgen, MT6785_SCP_CLK_CFG_0, &val);
	dev_info(scp->dev, "CK_CFG_0 %#x after the gate enable: scp select %u\n", val,
		 (val >> 16) & 0x7);
	regmap_read(topckgen, MT6785_SCP_CLK_CFG_1, &val);
	regmap_write(topckgen, MT6785_SCP_CLK_CFG_1, val | 0x11);

	/*
	 * The word DIFF-2026-11-18 reports at 0x10000110 (stock 0x7f00, this board 0x7f10) is
	 * reported and never written: it is not the meter's configuration but the gate bank of
	 * mainline's MT6785 top gates (clk-mt6785-top.c:568-572, sta/clr/set = 0x110), and its
	 * bits 4 and 5 are the armpll_div_pll1/2 gates (:579-580). h6 cleared bit 4 there in the
	 * belief that the vendor's clkdbg _CKGEN(0x110) named this register, and comb10/comb10a
	 * hung silently ~0.3 s later: the ARMPLL divider's clock went away under the CPU. A read
	 * of the same address is harmless and has been in every timeline tick since audio48, so
	 * exactly that - a read and one line - is what stays here.
	 */
	if (!regmap_read(topckgen, MT6785_TOP_CK_REG_110, &val))
		dev_info(scp->dev, "topckgen %#x (the armpll-div gate bank; not written) %#x, stock %#x\n",
			 MT6785_TOP_CK_REG_110, val, MT6785_TOP_CK_REG_110_STOCK);

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
	.scp_mtk_image = true,
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
