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
#include <linux/property.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/phy/phy.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#include "../cam/mtk_cam_core.h"

#define SENINF_CTRL			0x0200
#define SENINF_CSI2_CTRL		0x0210
#define SENINF_CSI2_EN			0x0A00
/*
 * The two counters, named as this phone's own vendor driver names them
 * (los-mt6785, drivers/misc/mediatek/imgsensor/src/common/v1_1/seninf.c:218-223):
 * 0x0AC8 is the CSI-2 interrupt status, cleared by writing ones, and 0x0ADC is the
 * packet counter. This driver had them the other way round, so every "packet cnt 0"
 * reading in the trials was the interrupt-status word, and the real counter was never
 * looked at.
 */
#define SENINF_CSI2_IRQ_STATUS		0x0AC8
#define SENINF_CSI2_PACKET_CNT		0x0ADC
/* the receiver's own interrupt status and the lane/frame state words */
#define SENINF_CSI2_INT_STATUS		0x0A14
#define SENINF_CSI2_LNRD_FSM		0x0A2C
#define SENINF_CSI2_FRAME_LINE_NUM	0x0A30
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
/*
 * 0x0210 is NOT a control register on this SoC: the stock HAL's receiver sequence never writes it
 * (hal_csirx_seninf_csi2_setting, libcam.halsensor.so 0x326f8-0x3280c), the phone reads 0 there on
 * every instance, and the write this driver used to make there was never observed to stick - which
 * is why the lane count and data type live in the 0x0A00 bank and the old CHROMEOS-derived macros
 * for 0x0210 are gone. The offset stays for the register dumps, so a future trial can keep proving
 * it is unused.
 */

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
static int csi_port = 2;
module_param(csi_port, int, 0644);

/* Written by the sweep: the receiver the driver serves, and the parameter's own value. */
struct mtk_seninf;
static struct mtk_seninf *seninf_sweep;
static int sweep_state;

/*
 * The board's MIPI switch level, forced instead of taken from the sensor node's mipi-sel-level: -1
 * keeps the device tree's value. The front and rear 8 MP sensors share CSI port 2 through this
 * switch (vendor imgsensor_cfg_table.c:101-145 drives it high for the front one and not at all for
 * the rear 8 MP one), so streaming the front sensor needs the other level - and if neither sensor
 * produces a packet with the level forced each way, the fault is not the switch.
 */
static int mipi_sel_force = -1;
module_param(mipi_sel_force, int, 0644);

/*
 * Program every receiver the HAL's port table lists (SENINF1, SENINF3 and SENINF5, one per CSI
 * port) and every analog port with them, instead of only the one csi_port names. The vendor table
 * says MAIN2 (the IMX355) is on port 2, and three trials have taken that as given along with the
 * switch levels; if the board really routes it elsewhere, one scan says which instance counts.
 */
/*
 * When a stream produces no packet, try the other sensor on this receiver once, by itself, with the
 * switch at that sensor's level - the one combination no trial has run yet. On by default here because
 * the question it answers (is any sensor on this port alive?) blocks everything else.
 */
static bool probe_sensors = true;
module_param(probe_sensors, bool, 0644);
MODULE_PARM_DESC(probe_sensors,
		 "if a stream sees no packet after 500 ms, start the other sensor on this receiver once");

static bool scan_ports = true;
module_param(scan_ports, bool, 0644);
MODULE_PARM_DESC(scan_ports,
		 "program all three CSI ports and receivers, not only the selected one");

/*
 * Which sensor sink pad this stream starts: -1 keeps the first one (pad 0, the camera the port was
 * built for), 1 starts the sensor on the second pad instead. The board's front OV13B10 and rear
 * IMX355 share CSI port 2 through the MIPI switch, so starting the other one - with the switch level
 * forced - is the cheapest way to ask whether any sensor on this port produces a packet at all.
 */
static int sensor_pad = -1;
module_param(sensor_pad, int, 0644);
MODULE_PARM_DESC(sensor_pad,
		 "sink pad whose sensor this stream starts: -1 = pad 0, or a pad number");

MODULE_PARM_DESC(mipi_sel_force,
		 "force the MIPI switch level at stream on: -1 = device tree, 0 or 1 = force");
MODULE_PARM_DESC(csi_port,
		 "HAL CSI port to program (0 = PORT_0, 1 = PORT_1, 2 = PORT_2, the default: the stock HAL puts imx355 and ov13b10 there); read at every stream on");

/*
 * The stock HAL's own CSI table (SeninfDrvImp::getCSIInfo, libcam.halsensor.so 0x31d10, table at
 * 0xf5168): port 0 -> receiver index 0, port 1 -> index 2, port 2 -> index 4, i.e. SENINF1, SENINF3
 * and SENINF5, whose instance bases are +0x0000, +0x2000 and +0x4000. The analog half follows the
 * port number: PORT_0 on CSI0 (0x11c80000), PORT_1 on CSI1 (0x11c82000), PORT_2 on CSI2
 * (0x11c84000).
 */
static const struct {
	unsigned int seninf_index;
	unsigned int phy_index;
} csi_ports[] = {
	{ 0, 0 },	/* PORT_0: SENINF1, CSI0 */
	{ 2, 1 },	/* PORT_1: SENINF3, CSI1 */
	{ 4, 2 },	/* PORT_2: SENINF5, CSI2 */
};
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
	phys_addr_t inst_pa;
	/* The camsys gate window at 0x1a000000 (the device tree's clock-controller@1a000000): its
	 * CG_CON word says which camera gates are open, which no clock API call shows. */
	void __iomem *camsys;
	/* Read-only views for the diagnostics: topckgen, infra and the pin controller's first window. */
	void __iomem *topck;
	void __iomem *infra;
	void __iomem *pio;
	/* The receiver instance this stream is on; every per-instance access uses it. */
	void __iomem *inst;
	/* Its zero-based index, 0..4, so late work can find the top route again. */
	unsigned int cur_idx;
	/* The block's physical base, so logs can name absolute addresses. */
	phys_addr_t base_pa;
	struct dentry *debugfs;

	struct v4l2_subdev sd;
	struct media_pad pads[3];
	struct mtk_cam_core *cam;
};

/* The upstream pad of this stream: the sink pad the parameter names, or the first one. */
static struct media_pad *mtk_seninf_upstream(struct mtk_seninf *seninf)
{
	unsigned int pad = sensor_pad < 0 ? 0 : sensor_pad;

	if (pad >= 2)
		return ERR_PTR(-EINVAL);

	return media_pad_remote_pad_unique(&seninf->sd.entity.pads[pad]);
}

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

/* The instance in use, or the block's first instance before the first stream. */
static void __iomem *seninf_inst(struct mtk_seninf *seninf)
{
	return seninf->inst ? seninf->inst : seninf->base;
}

static int mtk_seninf_packet_cnt_show(struct seq_file *s, void *unused)
{
	struct mtk_seninf *seninf = s->private;
	u32 val;

	pm_runtime_get_sync(seninf->dev);
	val = readl(seninf_inst(seninf) + SENINF_CSI2_PACKET_CNT);
	pm_runtime_put_autosuspend(seninf->dev);

	seq_printf(s, "%u\n", val);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_seninf_packet_cnt);

static int mtk_seninf_int_status_show(struct seq_file *s, void *unused)
{
	struct mtk_seninf *seninf = s->private;

	pm_runtime_get_sync(seninf->dev);
	seq_printf(s, "int_status 0x%08x irq_status 0x%08x lnrd_fsm 0x%08x frame_line 0x%08x\n",
		   readl(seninf_inst(seninf) + SENINF_CSI2_INT_STATUS),
		   readl(seninf_inst(seninf) + SENINF_CSI2_IRQ_STATUS),
		   readl(seninf_inst(seninf) + SENINF_CSI2_LNRD_FSM),
		   readl(seninf_inst(seninf) + SENINF_CSI2_FRAME_LINE_NUM));
	pm_runtime_put_autosuspend(seninf->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_seninf_int_status);

static int mtk_seninf_clear_write(void *data, u64 val)
{
	struct mtk_seninf *seninf = data;

	if (!val)
		return 0;

	pm_runtime_get_sync(seninf->dev);
	writel(~0U, seninf_inst(seninf) + SENINF_CSI2_IRQ_STATUS);
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
		{ "SENINF_CSI2_IRQ_STATUS", SENINF_CSI2_IRQ_STATUS },
		{ "SENINF_CSI2_INT_STATUS", SENINF_CSI2_INT_STATUS },
		{ "SENINF_CSI2_LNRD_FSM", SENINF_CSI2_LNRD_FSM },
		{ "SENINF_CSI2_FRAME_LINE_NUM", SENINF_CSI2_FRAME_LINE_NUM },
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
	debugfs_create_file("int_status", 0444, seninf->debugfs, seninf,
			    &mtk_seninf_int_status_fops);
	debugfs_create_file("regs", 0444, seninf->debugfs, seninf,
			    &mtk_seninf_regs_fops);
	debugfs_create_file("clear", 0200, seninf->debugfs, seninf,
			    &mtk_seninf_clear_fops);
}

/*
 * The clock and power side of a stream, printed at stream on and again half a second in: the two
 * clocks this driver holds (the camsys SENINF gate and the top-level SENINF mux), their rates, and
 * the camsys CG_CON word. A mux parked on the 26 MHz crystal, or a gate bit still closed while the
 * driver thinks it enabled it, would look exactly like "no packets" from the receiver side.
 */
static void mtk_seninf_clock_state(struct mtk_seninf *seninf, const char *when)
{
	unsigned long gate = seninf->clks[0].clk ? clk_get_rate(seninf->clks[0].clk) : 0;
	unsigned long mux = seninf->clks[1].clk ? clk_get_rate(seninf->clks[1].clk) : 0;

	dev_info(seninf->dev, "%s: %s %lu Hz, %s %lu Hz, camsys CG_CON 0x%08x CG_SET 0x%08x\n",
		 when, seninf->clks[0].id, gate, seninf->clks[1].id, mux,
		 seninf->camsys ? readl(seninf->camsys) : 0,
		 seninf->camsys ? readl(seninf->camsys + 0x4) : 0);
}

/*
 * The stock dump's SENINF words, and a way to replay them.
 *
 * Source: poco-m5s/evidence/stock-session-2026-11/camera-streaming.txt, taken on the phone running
 * stock MIUI with its camera app in the foreground (the stock kernel's REGDUMP module reads
 * /dev/mem). Every word below is a *configuration* word by classification - the status words of the
 * same dump (instance CTRL/CTRL_EXT, the counters at +0xac8/+0xadc, the cam-mux IRQ status at
 * 0x04a8) are deliberately left out, because they carry what happened, not what was set.
 *
 * seninf.stock_words=1 writes them over ours just before the sensor streams, and prints our own
 * values for the whole stock address list in the same REGDUMP format, so the two files can be
 * diffed word by word.
 */
static bool stock_words;
module_param(stock_words, bool, 0644);
MODULE_PARM_DESC(stock_words,
		 "write the stock dump's seninf configuration words before the sensor starts");

static const struct {
	u32 off;
	u32 val;
	const char *what;
} seninf_stock_words[] = {
	{ 0x0008, 0x32100000, "top mux" },
	{ 0x0010, 0x03020000, "top cam mux ctrl" },
	{ 0x0014, 0x00000504, "top mux ctrl 1 (n3d a)" },
	{ 0x0400, 0x03020000, "cam mux ctrl 0" },
	{ 0x0410, 0x00000012, "cam mux enable: mux 1 and 4" },
	{ 0x04a0, 0x8000030c, "cam mux irq enable" },
	{ 0x4a10, 0x00000006, "instance 5 csi2 +0x10" },
	{ 0x4d04, 0x001f0200, "instance 5 mux inten" },
};

/* Every address the stock dump holds for this window, in its order. */
static const u32 seninf_stock_dump[] = {
	0x0000, 0x0008, 0x0010, 0x0014, 0x0018, 0x001c, 0x0020, 0x0024,
	0x0400, 0x0404, 0x0408, 0x0410, 0x04a0, 0x04a4, 0x04a8, 0x0504,
	0x4000, 0x4004, 0x4008, 0x4010, 0x4200, 0x4204, 0x4210,
	0x4a00, 0x4a08, 0x4a10, 0x4a18, 0x4a1c, 0x4a2c, 0x4ac8, 0x4adc,
	0x4d00, 0x4d04, 0x4d3c,
};

static void mtk_seninf_stock_apply(struct mtk_seninf *seninf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(seninf_stock_words); i++) {
		u32 off = seninf_stock_words[i].off;

		writel(seninf_stock_words[i].val, seninf->base + off);
		dev_info(seninf->dev, "stock %s: +0x%04x = 0x%08x -> 0x%08x\n",
			 seninf_stock_words[i].what, off, seninf_stock_words[i].val,
			 readl(seninf->base + off));
	}
}

static void mtk_seninf_stock_dump_words(struct mtk_seninf *seninf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(seninf_stock_dump); i++)
		dev_info(seninf->dev, "REGDUMP camera-streaming 0x%08x 0x%08x\n",
			 (u32)seninf->base_pa + seninf_stock_dump[i],
			 readl(seninf->base + seninf_stock_dump[i]));
}

/*
 * Which of the three top-route registers this receiver uses, from the HAL's masks
 * (hal_csirx_seninf_top_setting, libcam.halsensor.so 0x32910-0x3292c: 0x60 for indices 0, 3
 * and 4; 0x64 for 1, 5 and 6; 0x68 for the rest).
 */
static u32 mtk_seninf_top_offset(unsigned int idx)
{
	if (idx == 0 || idx == 3 || idx == 4)
		return 0x60;
	if (idx == 1 || idx == 5 || idx == 6)
		return 0x64;
	return 0x68;
}

/*
 * Three axes the lead's ranked list asks for, all as runtime parameters so one boot prints one table.
 *
 *  topmux_all  the SENINF top mux word (0x0008) that says which receiver feeds which mux: our normal
 *              path has never written it, and if no mux is sourced from the enabled receiver, the
 *              CSI-2 block gets no input at all (counters 0, errors 0, no interrupt - exactly what
 *              ten trials show). With this set, mux n is fed by receiver n and every mux is enabled.
 *  csi2_enable the HAL sets bit 0 of the CSI-2 bank's +0x04 only in its "mode 2" path
 *              (hal_csirx_seninf_csi2_setting, libcam.halsensor.so :0x32770-0x327cc); our driver has
 *              only ever cleared it. Mode 2 is most plausibly the "CSI-2 feeds the capture engines"
 *              path, which is ours.
 *  gates       read-only dump of the topckgen, infra and pin-controller windows: the CSI-PHY
 *              reference clock has no gate in this tree at all, and if it lives in one of those
 *              windows under a name we never mapped, the analogue block would be clock-less while
 *              still reporting "ready". Nothing here writes those windows.
 */
static bool topmux_all;
module_param(topmux_all, bool, 0644);
MODULE_PARM_DESC(topmux_all,
		 "feed SENINF mux n from receiver n and enable every mux, before the sensor starts");

static bool csi2_enable;
module_param(csi2_enable, bool, 0644);
MODULE_PARM_DESC(csi2_enable,
		 "set bit 0 of the CSI-2 bank's +0x04 (the HAL's mode-2 enable) instead of clearing it");

static bool gates;
module_param(gates, bool, 0644);
MODULE_PARM_DESC(gates,
		 "print the topckgen, infra and pin-controller windows read-only at stream on");

/*
 * Which receiver instance reads the sensor: -1 keeps the HAL table's pairing (port 0 -> receiver 1,
 * port 1 -> 3, port 2 -> 5), 0..4 forces one. The lead's list needs the *cross* of analogue ports and
 * receivers, because the board may route a port to a receiver the table does not pair it with, and
 * that combination has never been programmed.
 */
static int rx_index = -1;
module_param(rx_index, int, 0644);
MODULE_PARM_DESC(rx_index,
		 "receiver to program: -1 = the port's own (HAL pairing), 0..4 = force");

/*
 * The state of the camera pins, decoded through this tree's own pin map rather than guessed:
 * drivers/pinctrl/mediatek/pinctrl-mt6785.c:24-36 gives DIR at 0x000, DOUT at 0x100, DIN at 0x200
 * and MODE at 0x300 with a 0x10 stride per sixteen pins and one bit (four for MODE) per pin, so
 * pin 21 is bit 5 of the pins-16..31 words. MTK's DIR convention: 1 = input, 0 = output.
 */
static void mtk_seninf_pin_state(struct mtk_seninf *seninf, const char *when)
{
	static const unsigned int pins[] = { 21, 22, 118, 124, 125, 195 };
	unsigned int i;

	if (!seninf->pio) {
		dev_warn(seninf->dev, "CAMGATES %s: the pin controller window is unmapped\n", when);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(pins); i++) {
		unsigned int p = pins[i];
		unsigned int bank = (p / 16) * 0x10;
		unsigned int bit = p % 16;
		u32 dir = readl(seninf->pio + 0x000 + bank);
		u32 dout = readl(seninf->pio + 0x100 + bank);
		u32 din = readl(seninf->pio + 0x200 + bank);
		u32 mode = readl(seninf->pio + 0x300 + bank);

		dev_info(seninf->dev,
			 "CAMGATES %s pin %u: mode %u %s dout %u din %u\n", when, p,
			 (mode >> (bit * 4)) & 0xf,
			 ((dir >> bit) & 1) ? "in" : "out",
			 (dout >> bit) & 1, (din >> bit) & 1);
	}
}

static void mtk_seninf_window(struct mtk_seninf *seninf, const char *what,
			      void __iomem *w, phys_addr_t pa, unsigned int words)
{
	unsigned int i, j;

	if (!w)
		return;
	for (i = 0; i < words; i += 8) {
		char line[160];
		int n = 0;

		n += scnprintf(line + n, sizeof(line) - n, "%s 0x%08x", what, (u32)pa + 4 * i);
		for (j = 0; j < 8 && i + j < words; j++)
			n += scnprintf(line + n, sizeof(line) - n, " %08x", readl(w + 4 * (i + j)));
		dev_info(seninf->dev, "%s\n", line);
	}
}

/*
 * The sweep: one flash, one table. Written through /sys/module/mtk_seninf/parameters/sweep while a stream
 * runs, it walks every port the HAL's table names with every lane mask this receiver accepts, and prints
 * one line per combination - port, lanes, mask, and what the receiver counted. That is the whole of what
 * is left: the sensor is proven to produce frames (DIAG frames counting), the clocks and gates are alive,
 * so either the port is not the one the board routes (the driver’s own assumption) or the lane count /
 * mask is not what the sensor sends.
 *
 * Ports other than the streaming one are powered on for their row of the table and powered off after it,
 * so a row can only be attributed to the port it names; the streaming port is left configured as the
 * stream expects.
 */
static const u32 mtk_seninf_sweep_masks[] = { 0x1, 0x3, 0x7, 0xf };
static const unsigned int mtk_seninf_sweep_lanes[] = { 1, 2, 3, 4 };

static void mtk_seninf_sweep(void)
{
	unsigned int p, m, s, sels;
	unsigned int live = clamp(csi_port, 0, 2);
	int switch_keep = 0;

	if (!seninf_sweep)
		return;
	if (!seninf_sweep->inst) {
		dev_info(seninf_sweep->dev, "sweep: no stream is running, nothing to sweep\n");
		return;
	}

	/*
	 * The board's MIPI switch is the one thing between the sensors and the analog blocks that a
	 * register sweep cannot see, so it is an axis of the table too: it has no enable pin in any
	 * source we have (the only switch_enable/switch_disable pinctrl states in the live device tree
	 * belong to the USB redriver, node redrv_*, :6301), only the select line on GPIO21, and here
	 * both of its positions are walked with every port and lane mask.
	 */
	sels = seninf_sweep->mipi_sel ? 2 : 1;
	if (seninf_sweep->mipi_sel) {
		switch_keep = gpiod_get_value_cansleep(seninf_sweep->mipi_sel);
		dev_info(seninf_sweep->dev, "sweep: mipi switch starts at %d, walking %u position(s)\n",
			 switch_keep, sels);
	}

	for (s = 0; s < sels; s++) {
		if (seninf_sweep->mipi_sel) {
			gpiod_set_value_cansleep(seninf_sweep->mipi_sel, s);
			msleep(20);
			dev_info(seninf_sweep->dev, "sweep: mipi switch %d (reads %d)\n", s,
				 gpiod_get_value_cansleep(seninf_sweep->mipi_sel));
		}

		for (p = 0; p < ARRAY_SIZE(csi_ports); p++) {
			unsigned int idx = csi_ports[p].seninf_index;
			unsigned int ph = csi_ports[p].phy_index;
			void __iomem *in = seninf_sweep->base + 0x1000 * idx;
			u32 top = mtk_seninf_top_offset(idx);

			if (seninf_sweep->phy[ph] && p != live) {
				if (phy_set_mode_ext(seninf_sweep->phy[ph],
						     PHY_MODE_MIPI_DPHY, 4) ||
				    phy_power_on(seninf_sweep->phy[ph]))
					continue;
			}

			writel((readl(seninf_sweep->base + top) & ~0x2u) | 0x1u,
			       seninf_sweep->base + top);

			for (m = 0; m < ARRAY_SIZE(mtk_seninf_sweep_masks); m++) {
				writel(mtk_seninf_sweep_masks[m], in + 0x0a00);
				msleep(120);
				dev_info(seninf_sweep->dev,
					 "sweep: sel %u port %u (%s, receiver %u) lanes %u mask 0x%x -> pkt %u irq 0x%08x int 0x%08x fsm 0x%08x\n",
					 s, p,
					 seninf_sweep->phy[ph] ?
						dev_name(&seninf_sweep->phy[ph]->dev) : "none",
					 idx + 1, mtk_seninf_sweep_lanes[m],
					 mtk_seninf_sweep_masks[m],
					 readl(in + SENINF_CSI2_PACKET_CNT),
					 readl(in + SENINF_CSI2_IRQ_STATUS),
					 readl(in + SENINF_CSI2_INT_STATUS),
					 readl(in + SENINF_CSI2_LNRD_FSM));
			}

			if (seninf_sweep->phy[ph] && p != live)
				phy_power_off(seninf_sweep->phy[ph]);
		}
	}

	/* Put the switch and the streaming port back the way the stream left them. */
	if (seninf_sweep->mipi_sel) {
		gpiod_set_value_cansleep(seninf_sweep->mipi_sel, switch_keep);
		dev_info(seninf_sweep->dev, "sweep: mipi switch back to %d\n", switch_keep);
	}
	{
		unsigned int idx = csi_ports[live].seninf_index;
		void __iomem *in = seninf_sweep->base + 0x1000 * idx;

		writel(0xf, in + 0x0a00);
		dev_info(seninf_sweep->dev, "sweep: done, port %u back to mask 0xf\n", live);
	}
}

static int mtk_seninf_sweep_set(const char *val, const struct kernel_param *kp)
{
	int ret = kstrtoint(val, 0, &sweep_state);

	if (ret)
		return ret;
	if (sweep_state)
		mtk_seninf_sweep();

	return 0;
}

static const struct kernel_param_ops mtk_seninf_sweep_ops = {
	.set = mtk_seninf_sweep_set,
	.get = param_get_int,
};

module_param_cb(sweep, &mtk_seninf_sweep_ops, &sweep_state, 0644);
MODULE_PARM_DESC(sweep,
		 "write 1 during a stream to sweep every port with every lane mask once");

/*
 * The other sensor on this receiver, with the switch at its own level, started by the kernel because no
 * packet came from the first one. The board puts the front OV13B10 and the rear IMX355 on the same CSI
 * port through the MIPI switch, and both are linked to this subdev (sink pads 0 and 1); until now only
 * one of the two has ever been streamed on this port. Runs once, from the 500 ms work, and leaves the
 * switch and the sensors as it found them.
 */
static void mtk_seninf_probe_other_sensor(struct mtk_seninf *seninf)
{
	unsigned int cur_pad = sensor_pad < 0 ? 0 : sensor_pad;
	unsigned int other_pad = cur_pad ? 0 : 1;
	struct media_pad *cur, *other;
	struct v4l2_subdev *cur_src, *other_src;
	void __iomem *in = seninf_inst(seninf);
	u32 level = 0, keep;

	cur = media_pad_remote_pad_unique(&seninf->sd.entity.pads[cur_pad]);
	other = media_pad_remote_pad_unique(&seninf->sd.entity.pads[other_pad]);
	if (IS_ERR(cur) || IS_ERR(other))
		return;

	cur_src = media_entity_to_v4l2_subdev(cur->entity);
	other_src = media_entity_to_v4l2_subdev(other->entity);

	if (other_src->dev && other_src->dev->of_node)
		of_property_read_u32(other_src->dev->of_node, "mipi-sel-level", &level);
	if (mipi_sel_force >= 0)
		level = mipi_sel_force;

	keep = seninf->mipi_sel ? gpiod_get_value_cansleep(seninf->mipi_sel) : 0;
	dev_info(seninf->dev, "probe: stopping %s, switch %u -> %u, starting %s on pad %u\n",
		 cur_src->name, keep, level, other_src->name, other_pad);

	dev_info(seninf->dev, "probe: %s stop %d\n", cur_src->name,
		 v4l2_subdev_call(cur_src, video, s_stream, 0));
	if (seninf->mipi_sel)
		gpiod_set_value_cansleep(seninf->mipi_sel, level);
	dev_info(seninf->dev, "probe: %s start %d\n", other_src->name,
		 v4l2_subdev_call(other_src, video, s_stream, 1));

	msleep(300);
	dev_info(seninf->dev, "probe: after 300 ms with %s: pkt %u irq 0x%08x int 0x%08x fsm 0x%08x\n",
		 other_src->name, readl(in + SENINF_CSI2_PACKET_CNT),
		 readl(in + SENINF_CSI2_IRQ_STATUS), readl(in + SENINF_CSI2_INT_STATUS),
		 readl(in + SENINF_CSI2_LNRD_FSM));

	/* Put everything back the way the stream found it. */
	v4l2_subdev_call(other_src, video, s_stream, 0);
	if (seninf->mipi_sel) {
		gpiod_set_value_cansleep(seninf->mipi_sel, keep);
		dev_info(seninf->dev, "probe: switch back to %u, %s restarted %d\n", keep,
			 cur_src->name, v4l2_subdev_call(cur_src, video, s_stream, 1));
	}
}

static void mtk_seninf_late_counters(struct work_struct *work)
{
	struct mtk_seninf *seninf =
		container_of(to_delayed_work(work), struct mtk_seninf, late_counters);

	unsigned int port;

	dev_info(seninf->dev, "lane state: DGB_PORT 0x%08x LNRD_FSM 0x%08x (0x%08llx)\n",
		 readl(seninf_inst(seninf) + 0x0a1c), readl(seninf_inst(seninf) + 0x0a2c),
		 (unsigned long long)seninf->inst_pa + 0x0a1c);

	/* This instance first, then one line per receiver port of the block. */
	dev_info(seninf->dev,
		 "500 ms in: CSI2_EN 0x%08x, pkt %u, irq_status 0x%08x, int_status 0x%08x, frame_line 0x%08x\n",
		 readl(seninf_inst(seninf) + SENINF_CSI2_EN),
		 readl(seninf_inst(seninf) + SENINF_CSI2_PACKET_CNT),
		 readl(seninf_inst(seninf) + SENINF_CSI2_IRQ_STATUS),
		 readl(seninf_inst(seninf) + SENINF_CSI2_INT_STATUS),
		 readl(seninf_inst(seninf) + SENINF_CSI2_FRAME_LINE_NUM));

	/*
	 * The top route and the CSI-2 bank once more, now that the sensor is really sending: a word
	 * that is not what s_stream wrote says something cleared it, and the top registers were
	 * never read on the phone before this trial.
	 */
	dev_info(seninf->dev, "500 ms in: top +0x60 %08x +0x64 %08x +0x68 %08x\n",
		 readl(seninf->base + 0x60), readl(seninf->base + 0x64),
		 readl(seninf->base + 0x68));
	mtk_seninf_clock_state(seninf, "500 ms in clocks");

	dev_info(seninf->dev, "500 ms in: mask %08x +0x04 %08x +0x08 %08x +0x10 %08x +0x18 %08x +0xc0 %08x\n",
		 readl(seninf_inst(seninf) + 0x0a00), readl(seninf_inst(seninf) + 0x0a04),
		 readl(seninf_inst(seninf) + 0x0a08), readl(seninf_inst(seninf) + 0x0a10),
		 readl(seninf_inst(seninf) + 0x0a18), readl(seninf_inst(seninf) + 0x0ac0));

	/*
	 * Measured, not guessed: the per-index port assignment lives in the closed HAL, so the port
	 * the streaming sensor really arrives on is whichever of these moves. One line per receiver
	 * instance (5 x 0x1000 from this node's base): its CSI-2 control, enable, packet and error
	 * counters and status word.
	 */
	for (port = 0; port < 5; port++) {
		void __iomem *p = seninf->base + 0x1000 * port;

		dev_info(seninf->dev,
			 "instance %u (0x%08llx): CSI2_CTRL 0x%08x, mask 0x%08x, pkt %u, irq 0x%08x, int 0x%08x\n",
			 port + 1,
			 (unsigned long long)seninf->base_pa + 0x1000 * port,
			 readl(p + SENINF_CSI2_CTRL), readl(p + SENINF_CSI2_EN),
			 readl(p + SENINF_CSI2_PACKET_CNT),
			 readl(p + SENINF_CSI2_IRQ_STATUS),
			 readl(p + SENINF_CSI2_INT_STATUS));
	}

	/*
	 * Still nothing after half a second. Two things can be tried from the kernel without a new
	 * flash, and each one leaves its verdict in the counters:
	 *
	 *  1. Program the receiver again now that the sensor is certainly sending, in case the
	 *     sensor's own start is what upsets the route (the HAL programs the receiver first and
	 *     starts the sensor afterwards, like this driver - so the reverse order is the case this
	 *     rules out, and a repeated write is harmless either way).
	 *  2. Move the board's MIPI switch to its other position and look again. Nothing in any
	 *     source says which level connects which sensor on this board; the vendor's power
	 *     sequence only shows that the front sensor's sequence drives it high and the rear 8 MP
	 *     sensor's sequence does not drive it at all. If the level this driver set isolates the
	 *     IMX355, the packet counter moves when it is flipped.
	 */
	if (!readl(seninf_inst(seninf) + SENINF_CSI2_PACKET_CNT)) {
		void __iomem *in = seninf_inst(seninf);
		u32 top = mtk_seninf_top_offset(seninf->cur_idx);
		u32 mask = readl(in + 0x0a00);
		int cur, flipped;

		writel(mask, in + 0x0a00);
		writel((readl(seninf->base + top) & ~0x2u) | 0x1u, seninf->base + top);
		msleep(80);
		dev_info(seninf->dev,
			 "stage 1, route re-asserted after the sensor started: pkt %u int 0x%08x FSM 0x%08x\n",
			 readl(in + SENINF_CSI2_PACKET_CNT),
			 readl(in + SENINF_CSI2_INT_STATUS),
			 readl(in + SENINF_CSI2_LNRD_FSM));

		if (readl(in + SENINF_CSI2_PACKET_CNT) || !seninf->mipi_sel)
			return;

		cur = gpiod_get_value_cansleep(seninf->mipi_sel);
		flipped = !cur;
		gpiod_set_value_cansleep(seninf->mipi_sel, flipped);
		msleep(80);
		dev_info(seninf->dev,
			 "stage 2, mipi switch %d -> %d: pkt %u int 0x%08x FSM 0x%08x\n",
			 cur, flipped, readl(in + SENINF_CSI2_PACKET_CNT),
			 readl(in + SENINF_CSI2_INT_STATUS),
			 readl(in + SENINF_CSI2_LNRD_FSM));

		if (readl(in + SENINF_CSI2_PACKET_CNT))
			return;

		gpiod_set_value_cansleep(seninf->mipi_sel, cur);
		msleep(80);
		dev_info(seninf->dev,
			 "stage 3, mipi switch back to %d: pkt %u int 0x%08x FSM 0x%08x\n",
			 cur, readl(in + SENINF_CSI2_PACKET_CNT),
			 readl(in + SENINF_CSI2_INT_STATUS),
			 readl(in + SENINF_CSI2_LNRD_FSM));
	}

	if (probe_sensors)
		mtk_seninf_probe_other_sensor(seninf);

}

/*
 * Data lanes the sensor in front of this receiver wants, from its own endpoint. Four when the
 * graph does not say - that is the IMX355's and the OV13B10's count, and the one the HAL's
 * lane-mask table needs (0xF).
 */
/*
 * Runtime override for the receiver's lane mask, so one boot can try 4, 2 and 1 lanes without a
 * rebuild: 0 keeps the device tree's data-lanes (four on this board), 1..4 force the mask 0x1..0xf.
 * The matching sensor-side value is imx355.sensor_lanes, which changes its 0x0114 lane mode.
 */
static int lanes_override;
module_param(lanes_override, int, 0644);
MODULE_PARM_DESC(lanes_override,
		 "force the receiver's data lanes: 0 = device tree, 1..4 = that many");

static unsigned int mtk_seninf_lanes(struct mtk_seninf *seninf)
{
	struct media_pad *remote;
	struct v4l2_subdev *src;
	struct fwnode_handle *ep;
	u32 lanes[4];
	int n;

	if (lanes_override >= 1 && lanes_override <= 4)
		return lanes_override;

	remote = mtk_seninf_upstream(seninf);
	if (IS_ERR(remote))
		return 4;
	src = media_entity_to_v4l2_subdev(remote->entity);
	if (!src->dev || !src->dev->fwnode)
		return 4;

	ep = fwnode_graph_get_endpoint_by_id(src->dev->fwnode, 0, 0,
					     0);
	if (!ep)
		return 4;

	n = fwnode_property_count_u32(ep, "data-lanes");
	if (n <= 0 || n > (int)ARRAY_SIZE(lanes) ||
	    fwnode_property_read_u32_array(ep, "data-lanes", lanes, n)) {
		fwnode_handle_put(ep);
		return 4;
	}
	fwnode_handle_put(ep);

	return n;
}

static int mtk_seninf_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mtk_seninf *seninf = container_of(sd, struct mtk_seninf, sd);
	int port = clamp(csi_port, 0, 2);
	unsigned int seninf_idx = csi_ports[port].seninf_index;
	unsigned int phy_idx = csi_ports[port].phy_index;

	/* The receiver may be forced independently of the analogue port, for the cross sweep. */
	if (rx_index >= 0 && rx_index <= 4)
		seninf_idx = rx_index;
	/* Every per-instance access goes through this base: the sensor is on SENINF5, not SENINF1. */
	void __iomem *inst = seninf->base + 0x1000 * seninf_idx;

	seninf->inst = inst;
	seninf->cur_idx = seninf_idx;
	seninf->inst_pa = seninf->base_pa + 0x1000 * seninf_idx;
	int ret;

	if (!enable) {
		/* Stop the upstream first, then the receiver, the reverse of the start order. */
		{
			struct media_pad *remote = mtk_seninf_upstream(seninf);

			if (!IS_ERR(remote)) {
				struct v4l2_subdev *src =
					media_entity_to_v4l2_subdev(remote->entity);

				dev_info(seninf->dev, "s_stream upstream %s: %d\n", src->name,
					 v4l2_subdev_call(src, video, s_stream, 0));
			}
		}

		cancel_delayed_work_sync(&seninf->late_counters);
		writel(0, inst + SENINF_CSI2_EN);
		if (scan_ports) {
			unsigned int i;

			for (i = 0; i < ARRAY_SIZE(csi_ports); i++)
				if (seninf->phy[csi_ports[i].phy_index] &&
				    csi_ports[i].phy_index != phy_idx)
					phy_power_off(seninf->phy[csi_ports[i].phy_index]);
		}
		if (seninf->phy[phy_idx])
			phy_power_off(seninf->phy[phy_idx]);
		pm_runtime_put_autosuspend(seninf->dev);
		dev_info(seninf->dev, "stream off: packet cnt %u, err cnt %u\n",
			 readl(seninf->base + SENINF_CSI2_PACKET_CNT));
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
		void __iomem *in = seninf->base + 0x1000 * seninf_idx;
		struct { u32 off; u32 val; const char *what; } seq[] = {
			{ 0x0a00, 0x0000000f, "CSI2 lane enables (sticky)" },
			{ 0x0a08, 0x00000500, "CSI2 settle (sticky part 0x500)" },
			{ 0x0a10, 0x0000071f, "CSI2 interrupt enable (sticky)" },
			{ 0x0a18, 0x0000000a, "CSI2 debug select (sticky)" },
			{ 0x0d00, 0x00000001, "seninf mux enable" },
			{ 0x0d04, 0x002b0000, "seninf mux data type RAW10" },
		};
		unsigned int i;

		dev_info(seninf->dev, "programming receiver instance %d at 0x%08llx\n",
			 port + 1, (unsigned long long)seninf->base_pa + 0x1000 * seninf_idx);

		/*
		 * The camera multiplexer is not part of a receiver instance. The stock HAL keeps its
		 * base in the object at this+0x180, and init() builds it as the seninf block's own
		 * mmap base plus 0x400 (libcam.halsensor.so init 0x2e7fc-0x2e814: x11 is [this+0x38],
		 * the 19-page seninf mapping, and x8 = x11 + 0x400). That is why the same values read
		 * back zero at instance 5's 0x1a0444xx while the block's top window holds live values.
		 *
		 * enableCamMUX (:0x2fd00) does, for camera mux n:
		 *   [0x400+0x10] |= 1 << n     -> 0x1a040410
		 *   [0x400+0xa0] |= 3 << 2n    -> 0x1a0404a0
		 *   [0x400+0xa8]  = 3 << 2n    -> 0x1a0404a8
		 * and setSeninfCamMuxSrc (:0x30f40) puts the mux index in bits 0..3 of [0x400+0x0] and
		 * the virtual channel in [0x400+0x104].
		 */
		/*
		 * No write to the seninf top window here. The stock HAL never touches 0x1a04001c/0x20/0x24:
		 * its port selection is the per-instance mux (setSeninfMuxCtrl, instance + 0x0..0x0c) and
		 * the top mux registers at 0x1a040010 / 0x1a040014 (setSeninfTopMuxCtrl :0x30b98-0x30c34,
		 * base [this+0x38] = the seninf block itself). The 0x1a040024 write that came before this
		 * was MT8183's value for a register this SoC reads back as zero, so it is dropped.
		 */
		dev_info(seninf->dev, "top window: 0x1a040010 0x%08x, 0x1a040014 0x%08x, 0x1a040024 0x%08x\n",
			 readl(seninf->base + 0x10), readl(seninf->base + 0x14),
			 readl(seninf->base + 0x24));

		{
			void __iomem *cmux = seninf->base + 0x400;
			unsigned int n = 3;	/* CAMSV0's camera mux */

			writel(readl(cmux + 0x10) | (1 << n), cmux + 0x10);
			writel(readl(cmux + 0xa0) | (3 << (2 * n)), cmux + 0xa0);
			writel(3 << (2 * n), cmux + 0xa8);
			writel((readl(cmux) & ~0xf) | (n & 0xf), cmux);

			dev_info(seninf->dev,
				 "cam mux 0x%08llx: ctrl 0x%08x, en 0x%08x, src 0x%08x, sel 0x%08x\n",
				 (unsigned long long)seninf->base_pa + 0x400,
				 readl(cmux), readl(cmux + 0x10), readl(cmux + 0xa0),
				 readl(cmux + 0xa8));
		}

		for (i = 0; i < ARRAY_SIZE(seq); i++) {
			writel(seq[i].val, in + seq[i].off);
			dev_info(seninf->dev, "prog %s: 0x%08llx = 0x%08x -> 0x%08x\n",
				 seq[i].what,
				 (unsigned long long)seninf->base_pa + 0x1000 * seninf_idx +
				 seq[i].off,
				 seq[i].val, readl(in + seq[i].off));
		}
	}


	/* The receiver's D-PHY first: without it no port can see anything on the wire. */
	dev_info(seninf->dev, "analog port %d (%s), instance %d\n", port,
		 seninf->phy[phy_idx] ? "present" : "absent", port + 1);
	if (seninf->phy[phy_idx]) {
		ret = phy_set_mode_ext(seninf->phy[phy_idx], PHY_MODE_MIPI_DPHY, 4);
		dev_info(seninf->dev, "phy set mode: %d\n", ret);
		{
			ret = phy_power_on(seninf->phy[phy_idx]);
			dev_info(seninf->dev, "csi rx phy %d on: %d\n", port, ret);
		}
	}

	/*
	 * The receiver's own CSI-2 programming, exactly as the stock HAL does it for this SoC
	 * (SeninfDrvImp::hal_csirx_seninf_csi2_setting, libcam.halsensor.so 0x326f8-0x3280c, from
	 * scratch/native/5-cameras/vendor-img/):
	 *
	 *   0xA10 bit 8 = 1                                     (:0x32764-0x3276c)
	 *   0xA08 bits 10:8 cleared; only the HAL's csirx mode 2 also sets bit 1 and the lane
	 *       bits                                            (:0x32770-0x327a4)
	 *   0xA04 bit 0 = 1 in that mode, 0 otherwise           (:0x327a8-0x327cc)
	 *   0xAC0 bit 31 = 1                                    (:0x327d0-0x327d8)
	 *   0xA00 = lane mask, 0x1/0x3/0x7/0xF for 1..4 lanes, written last
	 *                                                       (:0x327dc-0x3280c)
	 *
	 * Two things this driver did before are why this is a rewrite: it put a lane count
	 * into 0x0A00 and then overwrote it with the constant 1 (SENINF_CSI2_EN_ON), so a
	 * four-lane sensor faced a one-lane receiver; and its write to 0x0210 never stuck,
	 * the phone read 0x11 back where 0x2b001f had been written. Every word is printed
	 * with what the silicon answers, so a trial that still sees no packet names the
	 * write that was refused.
	 */
	{
		unsigned int lanes = mtk_seninf_lanes(seninf);
		struct { u32 off; u32 val; const char *what; } seq[] = {
			{ 0x0a10, readl(inst + 0x0a10) | 0x100, "CSI2 +0x10 bit 8" },
			/*
			 * The HAL's mode-2 branch also sets bit 1 here
			 * (hal_csirx_seninf_csi2_setting, libcam.halsensor.so :0x32784-:0x32794), and mode 2 is
			 * the C-PHY path the OV64B40 needs - that is what csi2_enable now selects, together with
			 * bit 0 of +0x04 below.
			 */
			{ 0x0a08, csi2_enable ? ((readl(inst + 0x0a08) & ~0x700) | 0x2) :
				(readl(inst + 0x0a08) & ~0x700),
			  csi2_enable ? "CSI2 +0x08 mode-2 bit 1 set" :
					"CSI2 +0x08 mode bits cleared" },
			{ 0x0a04, csi2_enable ? (readl(inst + 0x0a04) | 1) :
				(readl(inst + 0x0a04) & ~1),
			  csi2_enable ? "CSI2 +0x04 mode-2 enable ON" :
					"CSI2 +0x04 (mode-2 enable off)" },
			{ 0x0ac0, readl(inst + 0x0ac0) | 0x80000000, "CSI2 +0xc0 bit 31" },
			{ 0x0a00, (1u << lanes) - 1, "CSI2 lane mask" },
			{ 0x0a18, 0x8000001a, "CSI2 debug port = packet counter" },
		};
		unsigned int i;

		dev_info(seninf->dev, "csi2: instance %u at 0x%08llx, %u data lanes\n",
			 seninf_idx + 1, (unsigned long long)seninf->inst_pa, lanes);
		for (i = 0; i < ARRAY_SIZE(seq); i++) {
			writel(seq[i].val, inst + seq[i].off);
			dev_info(seninf->dev, "csi2 %s: +0x%03x = 0x%08x -> 0x%08x\n",
				 seq[i].what, seq[i].off, seq[i].val,
				 readl(inst + seq[i].off));
		}
	}

	/*
	 * The route from the analog port into this receiver. Only the HAL knows it:
	 * SeninfDrvImp::hal_csirx_seninf_top_setting (libcam.halsensor.so 0x328a0) picks one of
	 * three registers by receiver index - 0x60 for indices 0, 3 and 4, 0x64 for 1, 5 and 6,
	 * 0x68 for the rest (:0x32910-0x3292c, test masks 0x19 and 0x62) - and its normal path
	 * writes "clear bit 1, set bit 0" into it (:0x3294c-0x32960). SENINF5 is index 4, so it
	 * is 0x60 here. The routing block is [this+0x38], the SENINF window itself (:0x328a8).
	 * Nothing in this port ever wrote those three registers, and they were not in the 18.11
	 * stock dump's address list either, which is why they were never suspected.
	 */
	{
		u32 top_off = mtk_seninf_top_offset(seninf_idx);
		u32 before, want;

		before = readl(seninf->base + top_off);
		want = (before & ~0x2u) | 0x1u;
		writel(want, seninf->base + top_off);

		dev_info(seninf->dev,
			 "top route +0x%02x: 0x%08x -> 0x%08x (want 0x%08x), +0x60 %08x +0x64 %08x +0x68 %08x\n",
			 top_off, before, readl(seninf->base + top_off), want,
			 readl(seninf->base + 0x60), readl(seninf->base + 0x64),
			 readl(seninf->base + 0x68));
	}

	/* The top mux word and every mux enabled, plus the read-only window dumps. */
	if (topmux_all) {
		unsigned int inst_i;

		writel(0x76543210, seninf->base + 0x0008);
		for (inst_i = 0; inst_i < 5; inst_i++) {
			void __iomem *i2c = seninf->base + 0x1000 * inst_i;

			writel(readl(i2c + 0x0d00) | 0x1, i2c + 0x0d00);
			writel(0x002b0000, i2c + 0x0d04);
		}
		dev_info(seninf->dev, "topmux: 0x0008 0x%08x (mux n <- receiver n), five muxes enabled\n",
			 readl(seninf->base + 0x0008));
	}
	dev_info(seninf->dev, "CAMGATES gates=%d topmux_all=%d csi2_enable=%d rx_index=%d\n",
		 gates, topmux_all, csi2_enable, rx_index);
	if (gates) {
		/* The pins first, so this line cannot be lost to a failed window map. */
		dev_info(seninf->dev,
			 "CAMGATES windows: topck %s infra %s pio %s, mipi-sel gpiolib dir/val %d/%d\n",
			 seninf->topck ? "mapped" : "UNMAPPED",
			 seninf->infra ? "mapped" : "UNMAPPED",
			 seninf->pio ? "mapped" : "UNMAPPED",
			 seninf->mipi_sel ? gpiod_get_direction(seninf->mipi_sel) : -1,
			 seninf->mipi_sel ? gpiod_get_value_cansleep(seninf->mipi_sel) : -1);
		mtk_seninf_pin_state(seninf, "stream");
		mtk_seninf_window(seninf, "topck", seninf->topck, 0x10000000, 0x80);
		mtk_seninf_window(seninf, "infra", seninf->infra, 0x10001000, 0x80);
		mtk_seninf_window(seninf, "pio", seninf->pio, 0x10005000, 0x80);
	}

	if (stock_words) {
		mtk_seninf_stock_apply(seninf);
		mtk_seninf_stock_dump_words(seninf);
	}

	mtk_seninf_clock_state(seninf, "stream on clocks");

	/*
	 * The scan: every receiver of the HAL's port table with its analog port and its lane mask, so
	 * the per-instance line 500 ms later says which one the sensor really arrives on.
	 */
	if (scan_ports) {
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(csi_ports); i++) {
			unsigned int idx = csi_ports[i].seninf_index;
			unsigned int ph = csi_ports[i].phy_index;
			void __iomem *in = seninf->base + 0x1000 * idx;
			u32 top = mtk_seninf_top_offset(idx);
			int ret2;

			if (seninf->phy[ph] && ph != phy_idx) {
				ret2 = phy_set_mode_ext(seninf->phy[ph], PHY_MODE_MIPI_DPHY, 4);
				if (!ret2)
					phy_power_on(seninf->phy[ph]);
			}
			writel(0xf, in + 0x0a00);
			writel((readl(seninf->base + top) & ~0x2u) | 0x1u,
			       seninf->base + top);
			dev_info(seninf->dev,
				 "scan: port %u instance %u mask 0x%08x top +0x%02x 0x%08x pkt %u\n",
				 i, idx + 1, readl(in + 0x0a00), top,
				 readl(seninf->base + top),
				 readl(in + SENINF_CSI2_PACKET_CNT));
		}
	}

	dev_info(seninf->dev,
		 "stream on (instance %u, 0x%08llx): mask 0x%08x, +0x04 0x%08x, pkt %u, int 0x%08x, dbg 0x%08x\n",
		 seninf_idx + 1, (unsigned long long)seninf->inst_pa,
		 readl(inst + SENINF_CSI2_EN), readl(inst + 0x0a04),
		 readl(inst + SENINF_CSI2_PACKET_CNT),
		 readl(inst + SENINF_CSI2_INT_STATUS), readl(inst + 0x0a1c));

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
		struct media_pad *remote = mtk_seninf_upstream(seninf);
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
		struct media_pad *remote = mtk_seninf_upstream(seninf);

		if (!IS_ERR(remote) && seninf->mipi_sel) {
			struct v4l2_subdev *src =
				media_entity_to_v4l2_subdev(remote->entity);
			u32 level = 0;

			if (src->dev && src->dev->of_node)
				of_property_read_u32(src->dev->of_node, "mipi-sel-level",
						     &level);
			if (mipi_sel_force >= 0)
				level = mipi_sel_force;

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

	/* The camsys window, from the device tree's clock-controller@1a000000. Read only. */
	seninf->camsys = devm_ioremap(dev, 0x1a000000, 0x1000);
	if (!seninf->camsys)
		dev_warn(dev, "cannot map the camsys window, gate state will not be logged\n");

	/* Read-only: the clock parents and gates, and the pin controller's own registers. */
	seninf->topck = devm_ioremap(dev, 0x10000000, 0x1000);
	seninf->infra = devm_ioremap(dev, 0x10001000, 0x1000);
	seninf->pio = devm_ioremap(dev, 0x10005000, 0x1000);

	/*
	 * The MIPI-switch select pad's state BEFORE this driver touches it: the vendor's own imgsensor
	 * driver never drives it on this platform (imgsensor_platform.h:9 is '#undef MIPI_SWITCH', and
	 * every cam_mipi_switch_* use sits under that ifdef), so whatever the bootloader left is the
	 * state the stock driver runs in, and our driving the pin is a change nothing in the vendor
	 * tree explains - hence this reading of the pad before the driver touches it.
	 */
	mtk_seninf_pin_state(seninf, "before");

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

	/* Which analog block each port index is, so no log can be read against the wrong window. */
	for (i = 0; i < ARRAY_SIZE(seninf->phy); i++)
		dev_info(dev, "port %u -> %s (receiver %u)\n", i,
			 seninf->phy[i] ? dev_name(&seninf->phy[i]->dev) : "no phy",
			 csi_ports[i].seninf_index + 1);
	if (IS_ERR(seninf->phy))
		return dev_err_probe(dev, PTR_ERR(seninf->phy),
				     "failed to get the CSI-2 RX phy\n");

	platform_set_drvdata(pdev, seninf);
	seninf_sweep = seninf;
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
		 readl(seninf_inst(seninf) + SENINF_CSI2_EN),
		 readl(seninf_inst(seninf) + SENINF_CSI2_PACKET_CNT));

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
