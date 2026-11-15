// SPDX-License-Identifier: GPL-2.0
/*
 * MT6785 SCP (tinysys, Cortex-M4) bring-up for mainline on the Poco M5s.
 *
 * Almost every motion and environment sensor of this phone sits on the SCP's
 * own I2C buses, not on the AP's: the SCP is a Cortex-M4 inside the SoC that
 * runs firmware from the phone's `scp` partition.  Linux therefore has to get
 * that core running before a single sensor is visible.
 *
 * The image does not come from Linux: the phone's bootloader (LK) reads the
 * device tree node with compatible "mediatek,scp", and when its status is
 * okay it reserves 0x9fc00000+0x300000 as "SCP-reserved", loads the
 * tinysys-loader-CM4_A and tinysys-scp-CM4_A records of the scp partition into
 * SCP SRAM and that carriage-out, fills the region-info struct at SRAM+0x400
 * and leaves the core held (poco-m5s/evidence/try-noresv1/pl_lk.txt, and the
 * stock device tree's mblock-23-SCP-reserved).  This driver does what the
 * vendor kernel does with that: reads the region info back, checks that the
 * image is really there, and releases the core (vendor
 * drivers/misc/mediatek/scp/cm4/v01/scp_helper.c: scp_region_info_init(),
 * params_to_scp(), reset_scp(SCP_ALL_ENABLE)).
 *
 * Register map and sequences from the vendor driver of this SoC,
 * drivers/misc/mediatek/scp/cm4/scp_reg.h, scp_awake.c and v01/scp_helper.c of
 * android_kernel_xiaomi_mt6785 (scratch/native/4-modem/src/los-mt6785); the
 * structure of the driver (probe, DT parsing, debugfs) follows the working
 * MT6885 mainline port in cezanne-linux-port/linux/drivers/scp/scp_cezanne.c.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/types.h>

#include "mtk_scp_cm4.h"

/*
 * The image carriage-out, as the bootloader names it: LK reserves
 * 0x9fc00000+0x300000 and hands it over as a reserved-memory node with
 * compatible "mediatek,SCP-reserved" (stock DT mblock-23-SCP-reserved).
 */
#define SCP_CM4_IMAGE_COMPATIBLE	"mediatek,SCP-reserved"

/* an SCP CM4 image and its DRAM reach 0x80000000..0xbfffffff (vendor check) */
#define SCP_CM4_DRAM_WINDOW_START	0x80000000
#define SCP_CM4_DRAM_WINDOW_END		0xc0000000

/* scp_reg.h, MT6785 (CM4) */
#define SCP_SW_RSTN		0x0000	/* cfg: bit 0, 0 = held, 1 = released */
#define SCP_SW_RSTN_BIT		BIT(0)
#define SCP_A_TO_HOST_REG	0x001c
#define SCP_IRQ_SCP2HOST	BIT(0)
#define SCP_IRQ_WDT		BIT(8)
#define SCP_A_WDT_REG		0x0084
#define SCP_A_DEBUG_SP_REG	0x00a8
#define SCP_A_DEBUG_LR_REG	0x00ac
#define SCP_A_DEBUG_PSP_REG	0x00b0
#define SCP_A_DEBUG_PC_REG	0x00b4
#define SCP_A_GENERAL_REG0	0x0050
#define SCP_SLEEP_STATUS_REG	0x0114

#define SCP_READY_TIMEOUT_MS	2000

/* the one SCP of this SoC; the IPI layer and its clients reach it here */
struct scp_cm4 *scp_cm4_scp;

/*
 * The region info the bootloader leaves at SCP SRAM+0x400.  Field order from
 * vendor scp/cm4/scp_helper.h (struct scp_region_info_st); the commit of the
 * bootloader's log in poco-m5s/device-dump-2026-11-12/partitions/expdb.img
 * shows the values it writes.
 */
struct scp_region_info_cm4 {
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

static void scp_cm4_dump_status(struct scp_cm4 *scp, const char *why)
{
	dev_err(scp->dev,
		"%s: to-host %#x, wdt %#x, sleep %#x, pc %#x lr %#x sp %#x psp %#x, gregs %#x %#x\n",
		why, readl(scp->cfg + SCP_A_TO_HOST_REG),
		readl(scp->cfg + SCP_A_WDT_REG),
		readl(scp->cfg + SCP_SLEEP_STATUS_REG),
		readl(scp->cfg + SCP_A_DEBUG_PC_REG),
		readl(scp->cfg + SCP_A_DEBUG_LR_REG),
		readl(scp->cfg + SCP_A_DEBUG_SP_REG),
		readl(scp->cfg + SCP_A_DEBUG_PSP_REG),
		readl(scp->cfg + SCP_A_GENERAL_REG0),
		readl(scp->cfg + SCP_A_GENERAL_REG0 + 4));
}

/* the firmware announces itself with its image size on IPI_SCP_A_READY */
static void scp_cm4_ready_ipi(int id, void *data, unsigned int len)
{
	struct scp_cm4 *scp = scp_cm4_scp;
	u32 payload = len >= sizeof(u32) ? *(u32 *)data : 0;

	scp->image_size_reported = payload;
	if (payload != scp->sram_size)
		dev_err(scp->dev, "image size mismatch: AP %#zx, SCP %#x\n",
			scp->sram_size, payload);

	if (!scp->ready_seen) {
		scp->ready_seen = true;
		dev_info(scp->dev, "SCP ready\n");
		complete(&scp->ready);
	}

	/* the chain is blocking, clients may sleep in it */
	schedule_work(&scp->ready_work);
}

static void scp_cm4_ready_ws(struct work_struct *ws)
{
	struct scp_cm4 *scp = container_of(ws, struct scp_cm4, ready_work);

	blocking_notifier_call_chain(&scp->notifier, SCP_EVENT_READY, NULL);
}

static void scp_cm4_err_ipi(int id, void *data, unsigned int len)
{
	struct scp_cm4 *scp = scp_cm4_scp;

	dev_err(scp->dev, "SCP error report (ipi %d): %*phN\n", id,
		min_t(unsigned int, len, 32), data);
}

static irqreturn_t scp_cm4_irq(int irq, void *data)
{
	struct scp_cm4 *scp = data;
	u32 status = readl(scp->cfg + SCP_A_TO_HOST_REG);

	if (status & SCP_IRQ_WDT) {
		dev_err(scp->dev, "SCP watchdog\n");
		scp_cm4_dump_status(scp, "watchdog");
	}

	if (status & SCP_IRQ_SCP2HOST)
		scp_cm4_ipi_isr(scp);

	writel(status & (SCP_IRQ_SCP2HOST | SCP_IRQ_WDT),
	       scp->cfg + SCP_A_TO_HOST_REG);

	return IRQ_HANDLED;
}

/* find the image carriage-out by the name the bootloader gives it */
static int scp_cm4_find_image_mem(struct scp_cm4 *scp)
{
	struct device_node *node;
	struct reserved_mem *rmem;

	node = of_find_compatible_node(NULL, NULL,
				       SCP_CM4_IMAGE_COMPATIBLE);
	if (!node) {
		dev_err(scp->dev,
			"no reserved-memory node %s: the bootloader did not reserve the image\n",
			SCP_CM4_IMAGE_COMPATIBLE);
		return -ENODEV;
	}
	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem)
		return dev_err_probe(scp->dev, -ENODEV,
				     "cannot look up the image carriage-out\n");

	scp->dram_phys = rmem->base;
	scp->dram_size = rmem->size;
	scp->dram = devm_memremap(scp->dev, rmem->base, rmem->size,
				  MEMREMAP_WC);
	if (IS_ERR(scp->dram))
		return dev_err_probe(scp->dev, PTR_ERR(scp->dram),
				     "cannot map carriage-out %pa+%#zx\n",
				     &scp->dram_phys, scp->dram_size);

	return 0;
}

/*
 * Read back what the bootloader staged and check it really is a Cortex-M4
 * image: a CM4 starts from a vector table whose first word is the initial
 * stack pointer and whose second word is the reset vector (Thumb, so bit 0
 * set).  An empty or stale region info means the bootloader skipped the SCP.
 */
static int scp_cm4_check_staged(struct scp_cm4 *scp)
{
	struct scp_region_info_cm4 ri;
	u32 loader[2], firmware[2];

	memcpy_fromio(&ri, scp->sram + SCP_CM4_REGION_INFO_OFFSET, sizeof(ri));

	if (!ri.ap_loader_start || !ri.ap_loader_size ||
	    !ri.ap_firmware_start || !ri.ap_firmware_size) {
		dev_err(scp->dev, "SCP not loaded by the bootloader\n");
		return -ENODEV;
	}

	if (ri.ap_loader_start < SCP_CM4_DRAM_WINDOW_START ||
	    ri.ap_loader_start >= SCP_CM4_DRAM_WINDOW_END ||
	    ri.ap_firmware_start < SCP_CM4_DRAM_WINDOW_START ||
	    ri.ap_firmware_start >= SCP_CM4_DRAM_WINDOW_END ||
	    ri.ap_firmware_size > scp->sram_size) {
		dev_err(scp->dev,
			"SCP region info out of range: loader %#x+%#x, firmware %#x+%#x\n",
			ri.ap_loader_start, ri.ap_loader_size,
			ri.ap_firmware_start, ri.ap_firmware_size);
		return -EINVAL;
	}

	/* the loader and the image are staged in the carriage-out */
	if (ri.ap_loader_start < scp->dram_phys ||
	    ri.ap_loader_start + ri.ap_loader_size > scp->dram_phys + scp->dram_size ||
	    ri.ap_firmware_start < scp->dram_phys ||
	    ri.ap_firmware_start + ri.ap_firmware_size > scp->dram_phys + scp->dram_size) {
		dev_err(scp->dev, "SCP image lies outside %pa+%#zx\n",
			&scp->dram_phys, scp->dram_size);
		return -EINVAL;
	}

	memcpy_fromio(loader, scp->sram, sizeof(loader));
	memcpy_fromio(firmware, scp->dram + ri.ap_firmware_start -
		      scp->dram_phys, sizeof(firmware));

	if (loader[0] > scp->sram_size || (loader[0] & 3) ||
	    !(loader[1] & 1) || loader[1] >= ri.ap_loader_size) {
		dev_err(scp->dev,
			"no CM4 loader in SRAM: sp %#x reset %#x\n",
			loader[0], loader[1]);
		return -EINVAL;
	}
	if (firmware[0] > scp->sram_size || (firmware[0] & 3) ||
	    !(firmware[1] & 1) || firmware[1] >= ri.ap_firmware_size) {
		dev_err(scp->dev,
			"no CM4 image at %#x: sp %#x reset %#x\n",
			ri.ap_firmware_start, firmware[0], firmware[1]);
		return -EINVAL;
	}

	scp->dram_fw_phys = ri.ap_firmware_start;
	scp->dram_fw_size = ri.ap_firmware_size;
	scp->dram_data_phys = ri.ap_dram_start;
	scp->dram_data_size = ri.ap_dram_size;
	scp->dram_backup_phys = ri.ap_dram_backup_start;

	dev_info(scp->dev,
		 "staged by the bootloader: loader %#x+%#x, image %#x+%#x, dram %#x+%#x\n",
		 ri.ap_loader_start, ri.ap_loader_size, ri.ap_firmware_start,
		 ri.ap_firmware_size, ri.ap_dram_start, ri.ap_dram_size);

	return 0;
}

/*
 * params_to_scp() of the vendor driver passes the SCP_DRV_PARAMS block to the
 * firmware, and only when CFG_SENSOR_PARAMS_TO_SCP_SUPPORT is built in (it is
 * not, in the stock kernel of this phone).  The block has size 0 in this
 * SoC's scp_mem_tbl, so the call degenerates to the same no-op here; it is
 * kept as a named step so the difference is visible.
 */
static void scp_cm4_params_to_scp(struct scp_cm4 *scp)
{
	size_t size = scp->mblock[SCP_DRV_PARAMS_MEM_ID].size;

	if (!size)
		dev_dbg(scp->dev, "no SCP params block, nothing to pass\n");
}

static int scp_cm4_get_clocks(struct scp_cm4 *scp)
{
	struct device *dev = scp->dev;

	scp->clk_main = devm_clk_get_optional_enabled(dev, "main");
	if (IS_ERR(scp->clk_main))
		return dev_err_probe(dev, PTR_ERR(scp->clk_main), "main clock\n");

	scp->clk_infra = devm_clk_get_optional_enabled(dev, "infra");
	if (IS_ERR(scp->clk_infra))
		return dev_err_probe(dev, PTR_ERR(scp->clk_infra),
				     "infra clock\n");

	return 0;
}

/*
 * reset_scp(SCP_ALL_ENABLE) of the vendor driver: the core is already held by
 * the bootloader, so there is nothing to assert, only the release.
 */
static void scp_cm4_release(struct scp_cm4 *scp)
{
	writel(SCP_SW_RSTN_BIT, scp->cfg + SCP_SW_RSTN);
	dsb(sy);

	if (!wait_for_completion_timeout(&scp->ready,
					 msecs_to_jiffies(SCP_READY_TIMEOUT_MS)))
		scp_cm4_dump_status(scp, "SCP ready timeout");
}

static int scp_cm4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct scp_cm4 *scp;
	int ret, irq;

	scp = devm_kzalloc(dev, sizeof(*scp), GFP_KERNEL);
	if (!scp)
		return -ENOMEM;
	scp->dev = dev;
	init_completion(&scp->ready);
	INIT_WORK(&scp->ready_work, scp_cm4_ready_ws);
	platform_set_drvdata(pdev, scp);
	scp_cm4_scp = scp;

	scp->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(scp->sram))
		return PTR_ERR(scp->sram);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	scp->sram_size = resource_size(res);

	scp->cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(scp->cfg))
		return PTR_ERR(scp->cfg);

	scp->clkctrl = devm_platform_ioremap_resource_byname(pdev, "clkctrl");
	if (IS_ERR(scp->clkctrl))
		return PTR_ERR(scp->clkctrl);

	/* the L1 cache block is not used by the CM4 firmware */
	scp->l1cctrl = devm_platform_ioremap_resource_byname(pdev, "l1cctrl");
	if (IS_ERR(scp->l1cctrl))
		scp->l1cctrl = NULL;

	ret = scp_cm4_get_clocks(scp);
	if (ret)
		return ret;

	ret = scp_cm4_find_image_mem(scp);
	if (ret)
		return ret;

	ret = scp_cm4_ipi_init(scp);
	if (ret)
		return ret;
	scp_cm4_ipi_registration(IPI_SCP_A_READY, scp_cm4_ready_ipi,
				 "scp-A-ready");
	scp_cm4_ipi_registration(IPI_SCP_ERROR_INFO, scp_cm4_err_ipi,
				 "scp-A-error");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	/* the ready handshake and every IPI come in over this line */
	ret = devm_request_irq(dev, irq, scp_cm4_irq, 0, dev_name(dev), scp);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq %d\n", irq);

	ret = scp_cm4_check_staged(scp);
	if (ret)
		return ret;
	scp_cm4_params_to_scp(scp);

	/*
	 * An IPI may already be pending (the bootloader can leave the core
	 * running): let the ordinary path take it instead of discarding it,
	 * or the ready handshake would be lost.
	 */
	if (readl(scp->cfg + SCP_A_TO_HOST_REG))
		scp_cm4_irq(irq, scp);
	scp_cm4_ipi_reset(scp);

	dev_info(dev, "releasing the core, waiting %d ms for the ready ipi\n",
		 SCP_READY_TIMEOUT_MS);
	scp_cm4_release(scp);

	scp->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_bool("ready", 0444, scp->debugfs, &scp->ready_seen);
	debugfs_create_u32("image-size-reported", 0444, scp->debugfs,
			   &scp->image_size_reported);

	return 0;
}

static void scp_cm4_remove(struct platform_device *pdev)
{
	struct scp_cm4 *scp = platform_get_drvdata(pdev);

	debugfs_remove_recursive(scp->debugfs);
	writel(0, scp->cfg + SCP_SW_RSTN);
	scp->ready_seen = false;
	cancel_work_sync(&scp->ready_work);
}

static const struct of_device_id scp_cm4_of_match[] = {
	{ .compatible = "mediatek,mt6785-scp" },
	{ }
};
MODULE_DEVICE_TABLE(of, scp_cm4_of_match);

static struct platform_driver scp_cm4_driver = {
	.probe = scp_cm4_probe,
	.remove = scp_cm4_remove,
	.driver = {
		.name = "mtk_scp_cm4",
		.of_match_table = scp_cm4_of_match,
		/* the firmware and the ipi state cannot be torn down and rebuilt */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(scp_cm4_driver);

MODULE_DESCRIPTION("Mediatek MT6785 SCP (Cortex-M4) boot control and IPI");
MODULE_LICENSE("GPL");
