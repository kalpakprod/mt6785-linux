// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CSI-2 receiver (SENINF).
 *
 * The receiver takes the D-PHY lanes of a camera sensor and hands the packets to the camera
 * muxes that the capture engines (CAMSV) read from. This driver only brings the block up -
 * clocks and power - and exposes the counters it keeps, which is the first thing a board can
 * check while the sensor side is still being brought up: if the sensor streams, the packet
 * counter moves.
 *
 * Register offsets are the vendor's, from its own register dump in the 4.19 drop
 * (drivers/misc/mediatek/imgsensor/src/common/v1_1/seninf.c, seninf_dump_reg()):
 *
 *   0x0200  SENINFn_CTRL                     (:211)
 *   0x0210  SENINFn_CSI2_CTRL                (:212)
 *   0x0A00  SENINFn_CSI2_EN                  (:219)
 *   0x0AC8  SENINFn_CSI2_PACKET_CNT_STATUS   (:220, cleared by writing all ones at :223)
 *   0x0ADC  second counter of the receiver   (:222)
 *   0x0D18  SENINFn_MUX_IRQ_STATUS           (:231, cleared the same way at :235)
 *   0x0D30  SENINFn_MUX_SIZE                 (:232)
 *
 * The block windows are the vendor device tree's (seninf1..5 at 0x1a040000 + 0x1000*n), the
 * clock gates and muxes this tree's clock driver already has (CLK_CAM_SENINF,
 * CLK_TOP_SENINF0..2), and the CAM power domain, which larb6 uses as well.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#define SENINF_CTRL			0x0200
#define SENINF_CSI2_CTRL		0x0210
#define SENINF_CSI2_EN			0x0A00
#define SENINF_CSI2_PACKET_CNT		0x0AC8
#define SENINF_CSI2_ERR_CNT		0x0ADC
#define SENINF_MUX_IRQ_STATUS		0x0D18
#define SENINF_MUX_SIZE			0x0D30

/**
 * struct mtk_seninf - one receiver instance
 * @dev: the platform device
 * @base: the block's registers
 * @clks: the receiver's clock gate and its top-level mux
 */
struct mtk_seninf {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[2];
	struct dentry *debugfs;
};

static int mtk_seninf_runtime_resume(struct device *dev)
{
	struct mtk_seninf *seninf = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ARRAY_SIZE(seninf->clks), seninf->clks);
}

static int mtk_seninf_runtime_suspend(struct device *dev)
{
	struct mtk_seninf *seninf = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(seninf->clks), seninf->clks);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mtk_seninf_pm_ops, mtk_seninf_runtime_suspend,
				 mtk_seninf_runtime_resume, NULL);

static int mtk_seninf_packet_cnt_show(struct seq_file *s, void *unused)
{
	struct mtk_seninf *seninf = s->private;
	u32 val;

	pm_runtime_get_sync(seninf->dev);
	val = readl(seninf->base + SENINF_CSI2_PACKET_CNT);
	pm_runtime_put_autosuspend(seninf->dev);

	seq_printf(s, "%u\n", val);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_seninf_packet_cnt);

static int mtk_seninf_err_cnt_show(struct seq_file *s, void *unused)
{
	struct mtk_seninf *seninf = s->private;
	u32 val;

	pm_runtime_get_sync(seninf->dev);
	val = readl(seninf->base + SENINF_CSI2_ERR_CNT);
	pm_runtime_put_autosuspend(seninf->dev);

	seq_printf(s, "%u\n", val);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_seninf_err_cnt);

static int mtk_seninf_clear_write(void *data, u64 val)
{
	struct mtk_seninf *seninf = data;

	if (!val)
		return 0;

	pm_runtime_get_sync(seninf->dev);
	writel(~0U, seninf->base + SENINF_CSI2_PACKET_CNT);
	writel(~0U, seninf->base + SENINF_CSI2_ERR_CNT);
	writel(~0U, seninf->base + SENINF_MUX_IRQ_STATUS);
	pm_runtime_put_autosuspend(seninf->dev);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mtk_seninf_clear_fops, NULL, mtk_seninf_clear_write, "%llu\n");

static int mtk_seninf_regs_show(struct seq_file *s, void *unused)
{
	static const struct {
		const char *name;
		u32 off;
	} regs[] = {
		{ "SENINF_CTRL", SENINF_CTRL },
		{ "SENINF_CSI2_CTRL", SENINF_CSI2_CTRL },
		{ "SENINF_CSI2_EN", SENINF_CSI2_EN },
		{ "SENINF_CSI2_PACKET_CNT", SENINF_CSI2_PACKET_CNT },
		{ "SENINF_CSI2_ERR_CNT", SENINF_CSI2_ERR_CNT },
		{ "SENINF_MUX_IRQ_STATUS", SENINF_MUX_IRQ_STATUS },
		{ "SENINF_MUX_SIZE", SENINF_MUX_SIZE },
	};
	struct mtk_seninf *seninf = s->private;
	unsigned int i;

	pm_runtime_get_sync(seninf->dev);
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		seq_printf(s, "%-24s 0x%08x\n", regs[i].name,
			   readl(seninf->base + regs[i].off));
	pm_runtime_put_autosuspend(seninf->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_seninf_regs);

static void mtk_seninf_debugfs_init(struct mtk_seninf *seninf)
{
	seninf->debugfs = debugfs_create_dir(dev_name(seninf->dev), NULL);

	debugfs_create_file("packet_cnt", 0444, seninf->debugfs, seninf,
			    &mtk_seninf_packet_cnt_fops);
	debugfs_create_file("err_cnt", 0444, seninf->debugfs, seninf,
			    &mtk_seninf_err_cnt_fops);
	debugfs_create_file("regs", 0444, seninf->debugfs, seninf,
			    &mtk_seninf_regs_fops);
	debugfs_create_file("clear", 0200, seninf->debugfs, seninf,
			    &mtk_seninf_clear_fops);
}

static int mtk_seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_seninf *seninf;
	int ret;

	seninf = devm_kzalloc(dev, sizeof(*seninf), GFP_KERNEL);
	if (!seninf)
		return -ENOMEM;

	seninf->dev = dev;
	seninf->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(seninf->base))
		return PTR_ERR(seninf->base);

	seninf->clks[0].id = "seninf";
	seninf->clks[1].id = "mux";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(seninf->clks), seninf->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the receiver clocks\n");

	platform_set_drvdata(pdev, seninf);
	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/* Bring the block up once so that its registers are readable, then let it idle. */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power the receiver on\n");

	mtk_seninf_debugfs_init(seninf);

	dev_info(dev, "receiver on, CSI2_EN 0x%08x, packet counter %u\n",
		 readl(seninf->base + SENINF_CSI2_EN),
		 readl(seninf->base + SENINF_CSI2_PACKET_CNT));

	pm_runtime_put_autosuspend(dev);

	return 0;
}

static const struct of_device_id mtk_seninf_of_ids[] = {
	{ .compatible = "mediatek,mt6785-seninf" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_ids);

static struct platform_driver mtk_seninf_driver = {
	.probe = mtk_seninf_probe,
	.driver = {
		.name = "mtk-seninf",
		.of_match_table = mtk_seninf_of_ids,
		.pm = pm_ptr(&mtk_seninf_pm_ops),
	},
};
module_platform_driver(mtk_seninf_driver);

MODULE_DESCRIPTION("MediaTek CSI-2 receiver (SENINF)");
MODULE_LICENSE("GPL");
