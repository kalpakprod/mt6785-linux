// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek SENINF CSI-2 receiver for MT6785 (Poco M5s, rosemary).
 *
 * SENINF is the first stage of the MediaTek camera front end and, besides
 * CAMSV, the only one that can be ours: it receives CSI-2 from the sensors,
 * applies the per-receiver MUX and routes the result through a global "cam
 * mux" either to the closed ISP (cam mux 0..2, its CAM_A/B/C) or to a CAMSV
 * capture engine (cam mux 3 and up).
 *
 * Adapted from Google's ChromeOS 5.10 tree,
 * drivers/media/platform/mtk-isp/isp_50/seninf/ (mtk_seninf.c,
 * mtk_seninf_reg.h, mtk_seninf_def.h), from
 * chromium.googlesource.com/chromiumos/third_party/kernel. That tree was
 * written for this generation of the block: its register header is annotated
 * 0x1a040000..0x1a047d40 for the digital part and 0x11c80000..0x11c850ac for
 * the analog one, which are this phone's windows (live device tree
 * seninf1_csi2@1a040000 and mipi_rx_ana_csi0a@11c80000; study
 * poco-m5s/research/native/5-cameras.md 2.2). The register names and the
 * values written below are that file's; the addresses, lanes and routing come
 * from our device tree and the study.
 *
 * Addressing, all cited in the commit message: digital block 0x1a040000 with
 * one 0x1000 block per receiver (receivers 1..6 at +0x0000..+0x5000), the top
 * registers at the base, the analog receiver block at 0x11c80000.
 *
 * The register snapshot of study 4.4 does not exist yet: it needs the stock
 * phone streaming each module under Android, which is a phone action for the
 * lead. Until it does, the per-sensor replay tables are an empty list in
 * mtk-seninf-mt6785-replay.h and the driver programs SENINF from the fields
 * the reference tree knows; the snapshot's job is to add what the vendor HAL
 * writes that this code does not.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "mtk-seninf-mt6785-replay.h"

/* top registers, at the base of the digital block */
#define SENINF_TOP_CTRL			0x0000
#define SENINF_TOP_MUX_CTRL		0x0008
#define SENINF_TOP_CAM_MUX_CTRL		0x0010
#define SENINF_TOP_PHY_SENINF_CTL_CSI0	0x001c

/* per-receiver block: 0x1000 bytes each, receiver n (1-based) at (n - 1) */
#define SENINF_RX(n)			(0x1000 * ((n) - 1))
#define SENINF_CTRL			0x0200
#define   SENINF_CTRL_EN		BIT(0)
#define SENINF_CTRL_EXT			0x0204
#define   SENINF_CTRL_EXT_VAL		0x40
#define SENINF_TM_STP			0x0614
#define SENINF_CSI2_CTL			0x0a00
#define   CSI2_CTL_LANES		GENMASK(4, 0)
#define   CSI2_CTL_HEADER_ORDER		BIT(16)
#define SENINF_CSI2_LNRD_TIMING		0x0a08
#define   CSI2_LNRD_SETTLE		GENMASK(15, 8)
#define SENINF_CSI2_DPCM		0x0a0c
#define SENINF_CSI2_INT_EN		0x0a10
#define SENINF_CSI2_INT_STATUS		0x0a14
#define SENINF_CSI2_DGB_SEL		0x0a18
#define SENINF_CSI2_SPARE0		0x0a20
#define SENINF_CSI2_HS_TRAIL		0x0a40
#define SENINF_CSI2_RESYNC_MERGE_CTL	0x0a74
#define SENINF_CSI2_LINE_COUNT		0x0adc
#define SENINF_CSI2_MODE		0x0ae8
#define SENINF_CSI2_DPHY_SYNC		0x0b20
#define SENINF_MUX_CTRL			0x0d00
#define   MUX_CTRL_EN			BIT(31)
#define   MUX_CTRL_PIXEL_SEL		BIT(8)
#define   MUX_CTRL_HS_POL		BIT(10)
#define   MUX_CTRL_VS_POL		BIT(9)
#define SENINF_MUX_INTEN		0x0d04
#define SENINF_MUX_SPARE		0x0d2c
#define SENINF_MUX_CTRL_EXT		0x0d3c
#define   MUX_EXT_PIXEL_SEL		BIT(4)

/* cam mux: one control byte each, one enable bit each, one option word each */
#define SENINF_CAM_MUX_CTRL(n)		(0x0400 + 4 * ((n) / 4))
#define   CAM_MUX_CTRL_SHIFT(n)		(8 * ((n) % 4))
#define SENINF_CAM_MUX_EN		0x0410
#define SENINF_CAM_MUX_OPT(n)		(0x0420 + 4 * (n))
#define SENINF_CAM_MUX_CHK_CTL0(n)	(0x0500 + 0x10 * (n))
#define SENINF_CAM_MUX_CHK_CTL1(n)	(0x0504 + 0x10 * (n))

/* values the reference tree uses for a CSI-2 sensor at one pixel per clock */
#define SENINF_SETTLE_DELAY		0x15
#define SENINF_HS_TRAIL_PARAMETER	0x08
#define SENINF_TIMESTAMP_STEP		0x67
#define SENINF_DGB_SEL_PACKET_COUNTER	0x8000001a
#define SENINF_DPHY_SYNC_VALUE		0x1dff00

/*
 * Cam mux numbers. 0..2 are the ISP's CAM_A/B/C; the capture engines follow,
 * so cam mux 3 is CAMSV0. That mapping is from the cezanne port, which ran the
 * same routing block and routed "cam mux 3 -> CAMSV0" (its README, quoted in
 * study 3.1); the phone check for it is in the report.
 */
#define SENINF_CAM_MUX_FIRST_CAMSV	3

#define SENINF_NUM_SENSOR_PADS		5
#define SENINF_NUM_CAM_MUX_PADS		3
#define SENINF_NUM_PADS \
	(SENINF_NUM_SENSOR_PADS + SENINF_NUM_CAM_MUX_PADS)

/**
 * struct seninf_port_cfg - one sensor input, as described by its endpoint
 * @lanes: MIPI data lanes
 * @cphy: the sensor is C-PHY rather than D-PHY
 * @receiver: the SENINF receiver it reaches, 1-based
 */
struct seninf_port_cfg {
	unsigned int lanes;
	bool cphy;
	unsigned int receiver;
};

/**
 * struct seninf_dev - the SENINF block
 * @dev: the platform device
 * @base: digital registers
 * @ana: analog receiver registers
 * @clks: the block's clocks
 * @sd: the media entity sensors feed and cam muxes leave
 * @pads: sensor pads, then cam-mux pads
 * @port: the selected sensor pad
 * @cam_mux: the selected cam-mux pad
 * @cfg: per-pad lane, type and receiver
 * @lock: serialises power and routing
 * @powered: the block is running
 * @debugfs: register view, which exposes the line counter
 * @nf: async notifier that binds the sensors
 */
struct seninf_dev {
	struct device *dev;
	void __iomem *base;
	void __iomem *ana;
	struct clk_bulk_data *clks;
	int num_clks;

	struct v4l2_subdev sd;
	struct media_pad pads[SENINF_NUM_PADS];
	struct v4l2_device v4l2_dev;
	struct media_device mdev;

	unsigned int port;
	unsigned int cam_mux;
	struct seninf_port_cfg cfg[SENINF_NUM_SENSOR_PADS];

	struct mutex lock;
	bool powered;

	struct dentry *debugfs;
	struct v4l2_async_notifier nf;
};

static inline struct seninf_dev *to_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct seninf_dev, sd);
}

static void seninf_rmw(void __iomem *base, u32 reg, u32 mask, u32 val)
{
	writel((readl(base + reg) & ~mask) | (mask & val), base + reg);
}

/* Replay the optional per-receiver table, analog block first (study 3.1). */
static void seninf_replay(struct seninf_dev *seninf, unsigned int receiver)
{
	const struct seninf_replay *table = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(seninf_replay_tables); i++) {
		if (seninf_replay_tables[i].receiver == receiver) {
			table = &seninf_replay_tables[i];
			break;
		}
	}
	if (!table)
		return;

	for (i = 0; i < table->n_ana; i++)
		writel(table->ana[i].val, seninf->ana + table->ana[i].ofs);
	for (i = 0; i < table->n_dig; i++)
		writel(table->dig[i].val, seninf->base + table->dig[i].ofs);
}

/*
 * Bring the selected receiver up and route it to its cam mux. The order and
 * the values follow the reference's mtk_seninf_set_csi_mipi() and
 * mtk_seninf_set_mux(): control register, CSI-2 lane count and timing, then
 * the per-receiver mux, then the cam mux.
 */
static void seninf_power_on(struct seninf_dev *seninf)
{
	const struct seninf_port_cfg *cfg = &seninf->cfg[seninf->port];
	void __iomem *rx = seninf->base + SENINF_RX(cfg->receiver);
	unsigned int mux = seninf->port + 1;
	unsigned int cam_mux = seninf->cam_mux + SENINF_CAM_MUX_FIRST_CAMSV;

	writel(SENINF_TIMESTAMP_STEP, rx + SENINF_TM_STP);

	seninf_rmw(rx, SENINF_CTRL, SENINF_CTRL_EN, SENINF_CTRL_EN);
	seninf_rmw(rx, SENINF_CTRL_EXT, GENMASK(7, 0), SENINF_CTRL_EXT_VAL);

	/* settle delay, then the lane count in the low bits plus header order */
	seninf_rmw(rx, SENINF_CSI2_LNRD_TIMING, CSI2_LNRD_SETTLE,
		   SENINF_SETTLE_DELAY << 8);
	writel((readl(rx + SENINF_CSI2_CTL) & ~CSI2_CTL_LANES) |
	       ((BIT(cfg->lanes) - 1) << 1) | CSI2_CTL_HEADER_ORDER | 0x10,
	       rx + SENINF_CSI2_CTL);
	seninf_rmw(rx, SENINF_CSI2_RESYNC_MERGE_CTL, GENMASK(10, 0), 0x3);
	writel(0, rx + SENINF_CSI2_MODE);
	writel(SENINF_DPHY_SYNC_VALUE, rx + SENINF_CSI2_DPHY_SYNC);
	seninf_rmw(rx, SENINF_CSI2_SPARE0, BIT(0), 0);
	seninf_rmw(rx, SENINF_CSI2_CTL, BIT(25), BIT(25));
	seninf_rmw(rx, SENINF_CSI2_HS_TRAIL, GENMASK(7, 0),
		   SENINF_HS_TRAIL_PARAMETER);

	/* packet counter on the debug port, and every interrupt enabled */
	writel(SENINF_DGB_SEL_PACKET_COUNTER, rx + SENINF_CSI2_DGB_SEL);
	writel(0xffffffff, rx + SENINF_CSI2_INT_EN);
	writel(0xffffffff, rx + SENINF_CSI2_INT_STATUS);

	/* per-receiver mux: source is this receiver, one pixel per clock */
	seninf_rmw(seninf->base, SENINF_TOP_MUX_CTRL, GENMASK(3, 0) << 0,
		   cfg->receiver - 1);
	seninf_rmw(seninf->base, SENINF_RX(mux) + SENINF_MUX_CTRL,
		   GENMASK(19, 16), (cfg->receiver - 1) << 16);
	seninf_rmw(seninf->base, SENINF_RX(mux) + SENINF_MUX_CTRL,
		   MUX_CTRL_PIXEL_SEL, 0);
	seninf_rmw(seninf->base, SENINF_RX(mux) + SENINF_MUX_CTRL_EXT,
		   MUX_EXT_PIXEL_SEL, 0);
	writel(0x001f0008, seninf->base + SENINF_RX(mux) + SENINF_MUX_INTEN);
	seninf_rmw(seninf->base, SENINF_RX(mux) + SENINF_MUX_CTRL,
		   MUX_CTRL_HS_POL | MUX_CTRL_VS_POL, 0);
	seninf_rmw(seninf->base, SENINF_RX(mux) + SENINF_MUX_CTRL,
		   MUX_CTRL_EN, MUX_CTRL_EN);

	/* cam mux: take this receiver's mux output, and enable the mux */
	seninf_rmw(seninf->base, SENINF_CAM_MUX_CTRL(cam_mux),
		   GENMASK(7, 0) << CAM_MUX_CTRL_SHIFT(cam_mux),
		   mux << CAM_MUX_CTRL_SHIFT(cam_mux));
	writel(0, seninf->base + SENINF_CAM_MUX_OPT(cam_mux));
	writel(0, seninf->base + SENINF_CAM_MUX_CHK_CTL0(cam_mux));
	seninf_rmw(seninf->base, SENINF_CAM_MUX_EN, 0, BIT(cam_mux));
	writel(0, seninf->base + SENINF_TOP_CAM_MUX_CTRL);

	seninf_replay(seninf, cfg->receiver);
}

static void seninf_power_off(struct seninf_dev *seninf)
{
	const struct seninf_port_cfg *cfg = &seninf->cfg[seninf->port];
	void __iomem *rx = seninf->base + SENINF_RX(cfg->receiver);
	unsigned int cam_mux = seninf->cam_mux + SENINF_CAM_MUX_FIRST_CAMSV;

	writel(0, rx + SENINF_CSI2_INT_EN);
	seninf_rmw(rx, SENINF_CSI2_CTL, CSI2_CTL_LANES, 0);
	seninf_rmw(seninf->base, SENINF_RX(seninf->port + 1) + SENINF_MUX_CTRL,
		   MUX_CTRL_EN, 0);
	seninf_rmw(seninf->base, SENINF_CAM_MUX_EN, BIT(cam_mux), 0);
}

static int seninf_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct seninf_dev *seninf = to_seninf(sd);
	int ret = 0;

	mutex_lock(&seninf->lock);
	if (enable) {
		if (seninf->powered) {
			ret = -EBUSY;
			goto out;
		}
		ret = pm_runtime_resume_and_get(seninf->dev);
		if (ret)
			goto out;
		ret = clk_bulk_prepare_enable(seninf->num_clks, seninf->clks);
		if (ret) {
			pm_runtime_put(seninf->dev);
			goto out;
		}
		seninf_power_on(seninf);
		seninf->powered = true;
	} else if (seninf->powered) {
		seninf_power_off(seninf);
		clk_bulk_disable_unprepare(seninf->num_clks, seninf->clks);
		pm_runtime_put(seninf->dev);
		seninf->powered = false;
	}
out:
	mutex_unlock(&seninf->lock);
	return ret;
}

/* -------------------------------------------------------------------------
 * Formats
 */

static const struct v4l2_mbus_framefmt seninf_default_fmt = {
	.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	.width = 1600,
	.height = 1200,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.xfer_func = V4L2_XFER_FUNC_NONE,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_FULL_RANGE,
};

static int seninf_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *mf;
	unsigned int i;

	for (i = 0; i < SENINF_NUM_PADS; i++) {
		mf = v4l2_subdev_state_get_format(state, i);
		if (mf)
			*mf = seninf_default_fmt;
	}
	return 0;
}

/* both pad directions carry the sensor's format unchanged */
static int seninf_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf;

	if (format->pad >= SENINF_NUM_PADS)
		return -EINVAL;

	mf = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!mf)
		return -EINVAL;

	format->format.code = format->format.code ? : seninf_default_fmt.code;
	format->format.field = V4L2_FIELD_NONE;
	*mf = format->format;
	return 0;
}

static int seninf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	static const u32 codes[] = {
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_Y10_1X10,
	};

	if (code->index >= ARRAY_SIZE(codes))
		return -EINVAL;
	code->code = codes[code->index];
	return 0;
}

static const struct v4l2_subdev_pad_ops seninf_pad_ops = {
	.enum_mbus_code = seninf_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = seninf_set_fmt,
};

static const struct v4l2_subdev_video_ops seninf_video_ops = {
	.s_stream = seninf_s_stream,
};

static const struct v4l2_subdev_internal_ops seninf_internal_ops = {
	.init_state = seninf_init_state,
};

static const struct v4l2_subdev_ops seninf_subdev_ops = {
	.pad = &seninf_pad_ops,
	.video = &seninf_video_ops,
};

/*
 * Routing. A sink link selects which sensor pad is active (and therefore which
 * receiver); a source link selects which cam mux the frames leave on. Both are
 * the reference's rule: the link enabled last wins.
 */
static int seninf_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct seninf_dev *seninf = to_seninf(sd);

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	if (local->flags & MEDIA_PAD_FL_SINK) {
		if (local->index >= SENINF_NUM_SENSOR_PADS)
			return -EINVAL;
		seninf->port = local->index;
		dev_dbg(seninf->dev, "sensor pad %u selected\n", local->index);
	} else {
		unsigned int mux = local->index - SENINF_NUM_SENSOR_PADS;

		if (mux >= SENINF_NUM_CAM_MUX_PADS)
			return -EINVAL;
		seninf->cam_mux = mux;
		dev_dbg(seninf->dev, "cam mux %u selected\n", mux);
	}
	return 0;
}

static const struct media_entity_operations seninf_entity_ops = {
	.link_setup = seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/* -------------------------------------------------------------------------
 * Sensor binding
 */

struct seninf_async {
	struct v4l2_async_connection asc;
	unsigned int pad;
};

static int seninf_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asc)
{
	struct seninf_dev *seninf =
		container_of(notifier, struct seninf_dev, nf);
	struct seninf_async *sa = container_of(asc, struct seninf_async, asc);
	int ret;

	dev_dbg(seninf->dev, "%s bound to pad %u\n", sd->name, sa->pad);
	ret = media_create_pad_link(&sd->entity, 0, &seninf->sd.entity,
				    sa->pad, 0);
	if (ret)
		dev_err(seninf->dev, "failed to link %s\n", sd->entity.name);
	return ret;
}

static int seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct seninf_dev *seninf =
		container_of(notifier, struct seninf_dev, nf);

	/* every sensor is bound and linked: the graph is complete */
	return media_device_register(&seninf->mdev);
}

static const struct v4l2_async_notifier_operations seninf_notifier_ops = {
	.bound = seninf_notifier_bound,
	.complete = seninf_notifier_complete,
};

static int seninf_parse_endpoint(struct seninf_dev *seninf,
				 struct fwnode_handle *ep, unsigned int pad)
{
	struct v4l2_fwnode_endpoint vep = { };
	struct seninf_async *sa;
	int ret;

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		return ret;

	switch (vep.bus_type) {
	case V4L2_MBUS_CSI2_CPHY:
		seninf->cfg[pad].cphy = true;
		seninf->cfg[pad].lanes = vep.bus.mipi_csi2.num_data_lanes;
		dev_warn(seninf->dev,
			 "pad %u is C-PHY; this driver programs D-PHY only\n",
			 pad);
		break;
	case V4L2_MBUS_CSI2_DPHY:
		seninf->cfg[pad].lanes = vep.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		return -EINVAL;
	}

	/* the receiver is the pad's index plus one, unless stated otherwise */
	seninf->cfg[pad].receiver = pad + 1;

	sa = v4l2_async_nf_add_fwnode_remote(&seninf->nf, ep,
					     struct seninf_async);
	if (IS_ERR(sa))
		return PTR_ERR(sa);
	sa->pad = pad;
	return 0;
}

/*
 * Only the sink pads of the graph feed receivers: the source pads are the cam
 * muxes, which lead away from SENINF and are selected by link_setup() instead.
 * A sink endpoint sits in port@N, and N is the pad number that the sensor
 * links and the media graph use, so the port's reg is the pad.
 */
static int seninf_parse_dt(struct seninf_dev *seninf)
{
	struct fwnode_handle *ep = NULL;
	int ret = 0;

	while ((ep = fwnode_graph_get_next_endpoint(dev_fwnode(seninf->dev), ep))) {
		struct fwnode_handle *port = fwnode_get_parent(ep);
		u32 reg = 0;
		int err;

		fwnode_property_read_u32(port, "reg", &reg);
		fwnode_handle_put(port);

		if (reg < SENINF_NUM_SENSOR_PADS) {
			err = seninf_parse_endpoint(seninf, ep, reg);
			if (err && !ret)
				ret = dev_err_probe(seninf->dev, err,
						    "endpoint %u\n", reg);
		}
		fwnode_handle_put(ep);
	}
	return ret;
}

/* -------------------------------------------------------------------------
 * Debugfs
 */

/*
 * The line counter is the one register that proves the receiver is locked to
 * the sensor: it moves while the sensor streams and stops when it stops (the
 * cezanne port read its own offset the same way, study 3.1).
 */
static int seninf_line_count_show(struct seq_file *s, void *unused)
{
	struct seninf_dev *seninf = s->private;
	unsigned int receiver = seninf->cfg[seninf->port].receiver;

	seq_printf(s, "receiver %u line count 0x%08x\n", receiver,
		   readl(seninf->base + SENINF_RX(receiver) +
			 SENINF_CSI2_LINE_COUNT));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(seninf_line_count);

/* -------------------------------------------------------------------------
 * Probe
 */

static int seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct seninf_dev *seninf;
	struct v4l2_subdev *sd;
	unsigned int i;
	int ret;

	seninf = devm_kzalloc(dev, sizeof(*seninf), GFP_KERNEL);
	if (!seninf)
		return -ENOMEM;

	seninf->dev = dev;
	mutex_init(&seninf->lock);

	seninf->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(seninf->base))
		return PTR_ERR(seninf->base);

	seninf->ana = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(seninf->ana))
		return PTR_ERR(seninf->ana);

	ret = devm_clk_bulk_get_all(dev, &seninf->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	seninf->num_clks = ret;

	sd = &seninf->sd;
	v4l2_subdev_init(sd, &seninf_subdev_ops);
	sd->dev = dev;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &seninf_entity_ops;
	sd->internal_ops = &seninf_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, dev_name(dev), sizeof(sd->name));

	for (i = 0; i < SENINF_NUM_PADS; i++)
		seninf->pads[i].flags = i < SENINF_NUM_SENSOR_PADS ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, SENINF_NUM_PADS, seninf->pads);
	if (ret)
		return dev_err_probe(dev, ret, "pads init failed\n");

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init failed\n");
		goto err_media;
	}

	/*
	 * SENINF owns the media device of the camera pipeline and registers it
	 * once a sensor is bound; the sensor and the CAMSV video node join it
	 * through the v4l2_device that v4l2_async_register_subdev_sensor() and
	 * the capture driver find by walking the graph.
	 */
	ret = v4l2_device_register(dev, &seninf->v4l2_dev);
	if (ret) {
		dev_err_probe(dev, ret, "v4l2 device register failed\n");
		goto err_cleanup;
	}
	sd->v4l2_dev = &seninf->v4l2_dev;

	seninf->mdev.dev = dev;
	seninf->mdev.hw_revision = 0;
	strscpy(seninf->mdev.model, "mtk-seninf", sizeof(seninf->mdev.model));
	media_device_init(&seninf->mdev);
	seninf->v4l2_dev.mdev = &seninf->mdev;

	v4l2_async_subdev_nf_init(&seninf->nf, sd);
	seninf->nf.ops = &seninf_notifier_ops;

	ret = seninf_parse_dt(seninf);
	if (ret)
		goto err_nf;

	pm_runtime_enable(dev);

	/*
	 * SENINF is a bridge, not a sensor: it registers its own notifier for
	 * the sensors that feed it, so it is registered as a plain subdev.
	 * v4l2_async_register_subdev_sensor() would create a second notifier
	 * for its own fwnode, which this node does not have.
	 */
	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev register failed\n");
		goto err_pm;
	}

	ret = v4l2_async_nf_register(&seninf->nf);
	if (ret) {
		dev_err_probe(dev, ret, "notifier register failed\n");
		goto err_subdev;
	}

	seninf->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("line-count", 0444, seninf->debugfs, seninf,
			    &seninf_line_count_fops);

	dev_info(dev, "SENINF at %pR\n", &pdev->resource[0]);
	return 0;

err_subdev:
	v4l2_async_unregister_subdev(sd);
err_pm:
	pm_runtime_disable(dev);
err_nf:
	v4l2_async_nf_cleanup(&seninf->nf);
	media_device_unregister(&seninf->mdev);
	media_device_cleanup(&seninf->mdev);
	v4l2_device_unregister(&seninf->v4l2_dev);
err_cleanup:
	v4l2_subdev_cleanup(sd);
err_media:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void seninf_remove(struct platform_device *pdev)
{
	struct seninf_dev *seninf = platform_get_drvdata(pdev);

	debugfs_remove_recursive(seninf->debugfs);
	v4l2_async_nf_unregister(&seninf->nf);
	v4l2_async_nf_cleanup(&seninf->nf);
	v4l2_async_unregister_subdev(&seninf->sd);
	pm_runtime_disable(seninf->dev);
	media_device_unregister(&seninf->mdev);
	media_device_cleanup(&seninf->mdev);
	v4l2_device_unregister(&seninf->v4l2_dev);
	v4l2_subdev_cleanup(&seninf->sd);
	media_entity_cleanup(&seninf->sd.entity);
}

static const struct of_device_id seninf_of_match[] = {
	{ .compatible = "mediatek,mt6785-seninf" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, seninf_of_match);

static struct platform_driver seninf_driver = {
	.probe = seninf_probe,
	.remove = seninf_remove,
	.driver = {
		.name = "mtk-seninf",
		.of_match_table = seninf_of_match,
	},
};
module_platform_driver(seninf_driver);

MODULE_DESCRIPTION("MediaTek MT6785 SENINF CSI-2 receiver");
MODULE_LICENSE("GPL");
