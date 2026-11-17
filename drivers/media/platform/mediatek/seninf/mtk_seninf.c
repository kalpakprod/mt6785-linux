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
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/phy/phy.h>
#include <linux/seq_file.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#include "../cam/mtk_cam_core.h"

#define SENINF_CTRL			0x0200
#define SENINF_CSI2_CTRL		0x0210
#define SENINF_CSI2_EN			0x0A00
#define SENINF_CSI2_PACKET_CNT		0x0AC8
#define SENINF_CSI2_ERR_CNT		0x0ADC
#define SENINF_MUX_IRQ_STATUS		0x0D18
#define SENINF_MUX_SIZE			0x0D30

/*
 * CSI2_CTRL packing. What the vendor drop gives is the register's *address* and nothing else:
 * `git grep SENINF_CSI2_CTRL` and `git grep set_csi_mipi` over the MT6785 4.19 tree return no
 * hit at all - its receiver code is clock plumbing (`imgsensor/src/mt6785/seninf/seninf_clk.c`)
 * plus the register dump in `imgsensor/src/common/v1_1/seninf.c` (`CSI2_CTRL` at 0x0210, :212;
 * `CSI2_EN` at 0x0A00, :219). The field layout below is therefore the one the camera research
 * documents for this register generation from the ChromeOS isp_50 driver (§2.2): lane count in
 * bits 3:0 as 0x10 | ((1 << lanes) - 1), data type in bits 21:16. It is the part of this driver
 * that still wants the stock-phone replay tables, and the register dump below exists so the
 * phone can show what the closed HAL would have written.
 */
#define SENINF_CSI2_CTRL_LANES(lanes)	(0x10 | ((1 << (lanes)) - 1))
#define SENINF_CSI2_CTRL_DT(dt)		((dt) << 16)
#define SENINF_CSI2_EN_ON		BIT(0)

/*
 * The top-level registers that tie the analog ports to the CSI-2 receivers, and the values the
 * ChromeOS mt8183 driver writes for a four-data-lane D-PHY camera on CSI0
 * (scratch/native/5-cameras/tmp/cros/mtk_seninf.c:348-356, its SENINF_TOP_PHY_SENINF_CTL_CSI0
 * case: `(0x7fffc8fe & read) | 0x80001100`; the CSI1/CSI2 cases use 0x80000200). Without this the
 * receiver instance we enable is not connected to any analog port, which is what "packets 0,
 * errors 0" on every port looks like.
 */
#define SENINF_TOP_PHY_SENINF_CTL_CSI0	0x001C

/*
 * Which receiver instance the streaming sensor is wired to: the board routes its sensors through
 * a MIPI switch onto one port, and which one cannot be read from any source we have. A module
 * parameter lets one boot try every port: 0 = instance 1 (base + 0x0), 1 and 2 the next ones.
 */
static int csi_port;
module_param(csi_port, int, 0444);
MODULE_PARM_DESC(csi_port, "receiver instance to program and read, 0-based");
#define SENINF_TOP_PHY_CSI0_4D1C	0x80001100
#define SENINF_TOP_PHY_CSI0_MASK	0x7fffc8fe

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
	struct delayed_work late_counters;
	/* The board's MIPI switch select line, when the sensors share one receiver port. */
	struct gpio_desc *mipi_sel;
	/* The CSI-2 RX PHY: nobody else turns the D-PHY receiver on, and without it a port never
	 * sees an LP->HS transition - which is what every port measuring zero packets and zero
	 * errors means. The upstream driver for this block family is phy-mtk-mipi-csi-0-5. */
	struct phy *phy[3];
	/* The block's physical base, so logs can name absolute addresses. */
	phys_addr_t base_pa;
	struct dentry *debugfs;

	struct v4l2_subdev sd;
	struct media_pad pads[3];
	struct mtk_cam_core *cam;
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
		/* The routing the HAL would program: top mux, cam mux and the cam mux checkers. */
		{ "SENINF_TOP_MUX_CTRL0", 0x0010 },
		{ "SENINF_TOP_MUX_CTRL1", 0x0014 },
		{ "SENINF_CAM_MUX_CTRL1", 0x0404 },
		{ "SENINF_CAM_MUX_CTRL2", 0x0408 },
		{ "SENINF_CAM_MUX_IRQ_EN", 0x04A0 },
		{ "SENINF_CAM_MUX_IRQ_STATUS", 0x04A8 },
		{ "SENINF_CAM_MUX0_CHK_RES", 0x0508 },
		{ "SENINF_MUX_CTRL0", 0x0D00 },
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

static void mtk_seninf_late_counters(struct work_struct *work)
{
	struct mtk_seninf *seninf =
		container_of(to_delayed_work(work), struct mtk_seninf, late_counters);

	unsigned int port;

	dev_info(seninf->dev, "lane state: DGB_PORT 0x%08x LNRD_FSM 0x%08x (0x%08llx)\n",
		 readl(seninf->base + 0x0a1c), readl(seninf->base + 0x0a2c),
		 (unsigned long long)seninf->base_pa + 0x0a1c);

	/* This instance first, then one line per receiver port of the block. */
	dev_info(seninf->dev,
		 "500 ms in: CSI2_EN 0x%08x, packet cnt %u, err cnt %u, INTSTA 0x%08x\n",
		 readl(seninf->base + SENINF_CSI2_EN),
		 readl(seninf->base + SENINF_CSI2_PACKET_CNT),
		 readl(seninf->base + SENINF_CSI2_ERR_CNT),
		 readl(seninf->base + 0x0ac8));

	/*
	 * Measured, not guessed: the per-index port assignment lives in the closed HAL, so the port
	 * the streaming sensor really arrives on is whichever of these moves. One line per receiver
	 * instance (5 x 0x1000 from this node's base): its CSI-2 control, enable, packet and error
	 * counters and status word.
	 */
	for (port = 0; port < 5; port++) {
		void __iomem *p = seninf->base + 0x1000 * port;

		dev_info(seninf->dev,
			 "port %u (0x%08llx): CSI2_CTRL 0x%08x, CSI2_EN 0x%08x, pkt %u, err %u, INTSTA 0x%08x\n",
			 port + 1,
			 (unsigned long long)seninf->base_pa + 0x1000 * port,
			 readl(p + SENINF_CSI2_CTRL), readl(p + SENINF_CSI2_EN),
			 readl(p + SENINF_CSI2_PACKET_CNT), readl(p + SENINF_CSI2_ERR_CNT),
			 readl(p + 0x0ac8));
	}

}

static int mtk_seninf_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mtk_seninf *seninf = container_of(sd, struct mtk_seninf, sd);
	int ret;

	if (!enable) {
		/* Stop the upstream first, then the receiver, the reverse of the start order. */
		{
			struct media_pad *remote =
				media_pad_remote_pad_unique(&seninf->sd.entity.pads[0]);

			if (!IS_ERR(remote)) {
				struct v4l2_subdev *src =
					media_entity_to_v4l2_subdev(remote->entity);

				dev_info(seninf->dev, "s_stream upstream %s: %d\n", src->name,
					 v4l2_subdev_call(src, video, s_stream, 0));
			}
		}

		cancel_delayed_work_sync(&seninf->late_counters);
		writel(0, seninf->base + SENINF_CSI2_EN);
			if (seninf->phy[csi_port])
			phy_power_off(seninf->phy[csi_port]);
		pm_runtime_put_autosuspend(seninf->dev);
		dev_info(seninf->dev, "stream off: packet cnt %u, err cnt %u\n",
			 readl(seninf->base + SENINF_CSI2_PACKET_CNT),
			 readl(seninf->base + SENINF_CSI2_ERR_CNT));
		return 0;
	}

	/*
	 * The receiver has to stay powered and clocked for as long as the stream runs: without this
	 * reference its runtime PM suspends it seconds into streaming - the phone measured exactly
	 * that (`cam_seninf` enable 0, `top_seninf0` off, the device "suspended" 3 s into a stream) -
	 * and every write and read below lands in a dead window.
	 */
	ret = pm_runtime_resume_and_get(seninf->dev);
	if (ret)
		return ret;

	/*
	 * The TOP block read-only, named by the ChromeOS mt8183 map
	 * (scratch/native/5-cameras/tmp/cros/mtk_seninf_reg.h:136-144 and :162, :271), so the layout
	 * of this SoC can be read off the phone instead of assumed: which of these registers exist,
	 * and what the bootloader left in them.
	 */
	dev_info(seninf->dev, "TOP: CTRL %08x CMODEL %08x MUX_CTRL %08x CAM_MUX %08x\n",
		 readl(seninf->base + 0x0000), readl(seninf->base + 0x0004),
		 readl(seninf->base + 0x0008), readl(seninf->base + 0x0010));
	dev_info(seninf->dev, "TOP: N3D_A %08x N3D_B %08x PHY_CSI0 %08x PHY_CSI1 %08x PHY_CSI2 %08x\n",
		 readl(seninf->base + 0x0014), readl(seninf->base + 0x0018),
		 readl(seninf->base + 0x001c), readl(seninf->base + 0x0020),
		 readl(seninf->base + 0x0024));
	{
		unsigned int inst;

		for (inst = 0; inst < 5; inst++) {
			void __iomem *i = seninf->base + 0x1000 * inst;

			dev_info(seninf->dev,
				 "inst %u: CTRL %08x CTRL_EXT %08x CSI2_CTRL %08x CSI2_EN %08x MUX_CTRL %08x MUX_EXT %08x\n",
				 inst + 1, readl(i + 0x0200), readl(i + 0x0204),
				 readl(i + SENINF_CSI2_CTRL), readl(i + SENINF_CSI2_EN),
				 readl(i + 0x0d00), readl(i + 0x0d3c));
		}
	}

	/*
	 * Only the writes disp62 saw hold are kept, at the cezanne layout this receiver answers on:
	 * 0xa00 keeps the lane enables (0xf, and nothing else - SENINF1_CSI2_CTL on cezanne),
	 * 0xa08 keeps 0x500 of the settle word, 0xa10 keeps 0x71f, 0xa18 keeps 0xa. The MT8183
	 * writes that read back zero (0x200 pad_sel and the 0x80 pulse, 0x204, 0xa74, 0xb20, 0xa40,
	 * 0xb10, 0x1c) are dropped: they are another receiver's field map.
	 */
	{
		void __iomem *in = seninf->base + 0x1000 * csi_port;
		struct { u32 off; u32 val; const char *what; } seq[] = {
			{ 0x0a00, 0x0000000f, "CSI2 lane enables (sticky)" },
			{ 0x0a08, 0x00000500, "CSI2 settle (sticky part 0x500)" },
			{ 0x0a10, 0x0000071f, "CSI2 interrupt enable (sticky)" },
			{ 0x0a18, 0x0000000a, "CSI2 debug select (sticky)" },
			{ 0x0d00, 0x00000001, "seninf mux enable" },
			{ 0x0d04, 0x002b0000, "seninf mux data type RAW10" },
			{ 0x0400, 0x00000000, "cam mux ctrl0" },
			{ 0x0404, 0x00000001, "cam mux ctrl1" },
			{ 0x0408, 0x00000000, "cam mux ctrl2" },
			{ 0x040c, 0x00000000, "cam mux ctrl3" },
			{ 0x0410, 0x00000008, "cam mux enable, mux 3 (CAMSV0)" },
		};
		unsigned int i;

		dev_info(seninf->dev, "programming receiver instance %d at 0x%08llx\n",
			 csi_port + 1, (unsigned long long)seninf->base_pa + 0x1000 * csi_port);

		for (i = 0; i < ARRAY_SIZE(seq); i++) {
			writel(seq[i].val, in + seq[i].off);
			dev_info(seninf->dev, "prog %s: 0x%08llx = 0x%08x -> 0x%08x\n",
				 seq[i].what,
				 (unsigned long long)seninf->base_pa + 0x1000 * csi_port +
				 seq[i].off,
				 seq[i].val, readl(in + seq[i].off));
		}
	}


	/* The receiver's D-PHY first: without it no port can see anything on the wire. */
	dev_info(seninf->dev, "analog port %d (%s), instance %d\n", csi_port,
		 seninf->phy[csi_port] ? "present" : "absent", csi_port + 1);
	if (seninf->phy[csi_port]) {
		ret = phy_set_mode_ext(seninf->phy[csi_port], PHY_MODE_MIPI_DPHY, 4);
		dev_info(seninf->dev, "phy set mode: %d\n", ret);
		{
			ret = phy_power_on(seninf->phy[csi_port]);
			dev_info(seninf->dev, "csi rx phy %d on: %d\n", csi_port, ret);
		}
	}

	/*
	 * Lane count from the sensor's bus configuration, data type 0x2b (RAW10) for the Bayer
	 * sensors this port carries. The values are written once per stream start; the receiver's
	 * own mux programming (which engine reads which port) follows with the replay tables.
	 */
	writel(SENINF_CSI2_CTRL_LANES(4) | SENINF_CSI2_CTRL_DT(0x2b),
	       seninf->base + SENINF_CSI2_CTRL);
	writel(SENINF_CSI2_EN_ON, seninf->base + SENINF_CSI2_EN);

	dev_info(seninf->dev,
		 "stream on: CSI2_CTRL 0x%08x, CSI2_EN 0x%08x, packet cnt %u, err cnt %u\n",
		 readl(seninf->base + SENINF_CSI2_CTRL),
		 readl(seninf->base + SENINF_CSI2_EN),
		 readl(seninf->base + SENINF_CSI2_PACKET_CNT),
		 readl(seninf->base + SENINF_CSI2_ERR_CNT));

	/*
	 * The routing at the moment the stream starts: the top mux decides which receiver
	 * feeds which mux, the cam mux decides which engine reads it, and the per-instance mux
	 * register shows what this receiver is attached to. Packets counted after a mux that is
	 * not routed give zero, so the numbers belong next to each other. The analog MIPI block
	 * (0x11c80000) is outside this node window and is not mapped here yet.
	 */
	dev_info(seninf->dev,
		 "routing: TOP_MUX0 0x%08x, TOP_MUX1 0x%08x, CAM_MUX1 0x%08x, CAM_MUX2 0x%08x, MUX0 0x%08x\n",
		 readl(seninf->base + 0x0010), readl(seninf->base + 0x0014),
		 readl(seninf->base + 0x0404), readl(seninf->base + 0x0408),
		 readl(seninf->base + 0x0d00));

	/*
	 * The receiver is programmed; now the sensor that feeds it has to transmit. Each entity in
	 * the chain starts the one upstream of it, so the cascade stops at the sensor.
	 */
	{
		struct media_pad *remote =
			media_pad_remote_pad_unique(&seninf->sd.entity.pads[0]);
		int src_ret;

		if (IS_ERR(remote)) {
			dev_info(seninf->dev, "no upstream sensor on the sink pad: %ld\n",
				 PTR_ERR(remote));
		} else {
			struct v4l2_subdev *src =
				media_entity_to_v4l2_subdev(remote->entity);

			src_ret = v4l2_subdev_call(src, video, s_stream, 1);
			dev_info(seninf->dev, "s_stream upstream %s: %d\n", src->name,
				 src_ret);
		}
	}

	/*
	 * The reference taken at the top of this function is held until s_stream(0): releasing it here
	 * is what let the device autosuspend 100 ms into a live stream, and it left the put at stop
	 * without a matching get (the "Runtime PM usage count underflow" the phone logged).
	 */
	dev_info(seninf->dev, "stream on holds the receiver: pm usage %d\n",
		 pm_runtime_active(seninf->dev));

	/*
	 * The switch position belongs to the sensor that is streaming: the state is read from that
	 * sensor's own node (`mipi-sel-level`, 0 for the MAIN2 side, 1 for the SUB side) and then
	 * driven before the sensor is started, with the value logged.
	 */
	{
		struct media_pad *remote =
			media_pad_remote_pad_unique(&seninf->sd.entity.pads[0]);

		if (!IS_ERR(remote) && seninf->mipi_sel) {
			struct v4l2_subdev *src =
				media_entity_to_v4l2_subdev(remote->entity);
			u32 level = 0;

			if (src->dev && src->dev->of_node)
				of_property_read_u32(src->dev->of_node, "mipi-sel-level",
						     &level);

			gpiod_set_value_cansleep(seninf->mipi_sel, level);
			dev_info(seninf->dev, "mipi switch: %s -> %u, reads %d\n",
				 src->name, level,
				 gpiod_get_value_cansleep(seninf->mipi_sel));
		}
	}

	/* And once more half a second later, when the sensor has really been transmitting. */
	schedule_delayed_work(&seninf->late_counters, msecs_to_jiffies(500));

	return 0;
}

static int mtk_seninf_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = 1920;
	fmt->format.height = 1080;
	fmt->format.field = V4L2_FIELD_NONE;

	return 0;
}

static const struct v4l2_subdev_video_ops mtk_seninf_video_ops = {
	.s_stream = mtk_seninf_s_stream,
};

static const struct v4l2_subdev_pad_ops mtk_seninf_pad_ops = {
	.get_fmt = mtk_seninf_get_fmt,
};

static const struct v4l2_subdev_ops mtk_seninf_subdev_ops = {
	.video = &mtk_seninf_video_ops,
	.pad = &mtk_seninf_pad_ops,
};

static int mtk_seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_seninf *seninf;
	int ret;
	unsigned int i;

	seninf = devm_kzalloc(dev, sizeof(*seninf), GFP_KERNEL);
	if (!seninf)
		return -ENOMEM;

	seninf->dev = dev;
	seninf->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(seninf->base))
		return PTR_ERR(seninf->base);

	/* Every register log names this base plus the offset, never a bare "+0x..." . */
	seninf->base_pa = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start;

	seninf->clks[0].id = "seninf";
	seninf->clks[1].id = "mux";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(seninf->clks), seninf->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the receiver clocks\n");

	/*
	 * Some boards route two sensors onto one receiver port through a switch: the vendor's
	 * platform sequence drives this line per sensor index (SUB high, MAIN2 low,
	 * imgsensor_cfg_table.c:101-145), and on rosemary it is GPIO21 (stock DT: the pinctrl states
	 * camera1_mipisel_off/on at gpio21). Optional, so a board without one still works.
	 */
	seninf->mipi_sel = devm_gpiod_get_optional(dev, "mipi-sel", GPIOD_OUT_LOW);

	/* Every analog port the device tree describes, so csi_port can pick one. */
	for (i = 0; i < ARRAY_SIZE(seninf->phy); i++) {
		static const char * const names[] = { "csi-rx0", "csi-rx1", "csi-rx2" };
		struct phy *p = devm_phy_optional_get(dev, names[i]);

		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			continue;
		}
		seninf->phy[i] = p;
	}
	if (IS_ERR(seninf->phy))
		return dev_err_probe(dev, PTR_ERR(seninf->phy),
				     "failed to get the CSI-2 RX phy\n");

	platform_set_drvdata(pdev, seninf);
	INIT_DELAYED_WORK(&seninf->late_counters, mtk_seninf_late_counters);
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

	/* The receiver is a subdevice in the pipeline: sink pad from the sensor, source to CAMSV. */
	v4l2_subdev_init(&seninf->sd, &mtk_seninf_subdev_ops);
	seninf->sd.dev = dev;
	seninf->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	seninf->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	strscpy(seninf->sd.name, dev_name(dev), sizeof(seninf->sd.name));
	seninf->pads[0].flags = MEDIA_PAD_FL_SINK;
	seninf->pads[1].flags = MEDIA_PAD_FL_SINK;
	seninf->pads[2].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&seninf->sd.entity, 3, seninf->pads);
	if (ret)
		goto err_debugfs;

	seninf->cam = mtk_cam_core_get(dev);
	if (seninf->cam) {
		/* The pipeline's async notifier collects the entities and makes the links. */
		ret = v4l2_async_register_subdev(&seninf->sd);
		if (ret)
			goto err_entity;
	}

	dev_info(dev, "receiver on, CSI2_EN 0x%08x, packet counter %u\n",
		 readl(seninf->base + SENINF_CSI2_EN),
		 readl(seninf->base + SENINF_CSI2_PACKET_CNT));

	pm_runtime_put_autosuspend(dev);

	return 0;

err_entity:
	media_entity_cleanup(&seninf->sd.entity);
err_debugfs:
	debugfs_remove_recursive(seninf->debugfs);
	pm_runtime_put_autosuspend(dev);

	return ret;
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
