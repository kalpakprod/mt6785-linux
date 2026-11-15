// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the GalaxyCore GC02M1 image sensor.
 *
 * The Poco M5s (rosemary) carries it as its 2 MP macro camera, on the camera
 * I2C bus at 7-bit address 0x37 with a single MIPI CSI-2 lane (vendor driver
 * drivers/misc/mediatek/imgsensor/src/mt6785/gc02m1ofilm_mipi_raw/
 * gc02m1ofilm_mipi_raw_Sensor.c of the Xiaomi MT6785 Android 4.19 tree:
 * .i2c_addr_table {0x6e, 0x20, 0xff} in 8-bit form, .mipi_lane_num
 * SENSOR_MIPI_1_LANE, 1600x1200 at 30 fps).
 *
 * Modelled on drivers/media/i2c/gc05a2.c from the same vendor family; the
 * register sequence below is the vendor's init table, extracted by
 * poco-m5s/research/native/5-cameras/extract_tables.py into
 * tables/gc02m1ofilm_tables.h (200 writes) and copied from there.
 *
 * The sensor is mono-adjacent in use (the board can fit either this part or
 * the OV02B1B as the depth module, study section 1) but reports a Bayer
 * pattern, so the Bayer formats are offered and the software ISP decides.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GC02M1_STREAMING_REG		CCI_REG8(0x0100)
#define GC02M1_STREAMING_MASK		BIT(0)

#define GC02M1_REG_CHIP_ID		CCI_REG16(0xf0)
#define GC02M1_CHIP_ID			0x02e0

#define GC02M1_EXPOSURE_REG		CCI_REG16(0x0202)
#define GC02M1_EXPOSURE_MIN		4
#define GC02M1_EXPOSURE_STEP		1
#define GC02M1_EXPOSURE_MARGIN		8

#define GC02M1_AGAIN_REG		CCI_REG16(0x0204)
#define GC02M1_AGAIN_MIN		1024
#define GC02M1_AGAIN_MAX		(1024 * 16)
#define GC02M1_AGAIN_STEP		1

#define GC02M1_VBLANK_REG		CCI_REG16(0x0340)
#define GC02M1_VBLANK_MAX		0x3fff

#define GC02M1_HBLANK_REG		CCI_REG16(0x0342)

#define GC02M1_TEST_PATTERN_REG		CCI_REG8(0x0080)
#define GC02M1_TEST_PATTERN_MASK	GENMASK(1, 0)
#define GC02M1_TEST_PATTERN_EN		0x40

#define GC02M1_NATIVE_WIDTH		1600
#define GC02M1_NATIVE_HEIGHT		1200

#define GC02M1_DEFAULT_CLK_FREQ		(24 * HZ_PER_MHZ)
#define GC02M1_LINK_FREQ		(336 * HZ_PER_MHZ)
#define GC02M1_DATA_LANES		1
#define GC02M1_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10

#define GC02M1_REG_END			0xffff
#define GC02M1_REG_DELAY		0xfffe

static const char *const gc02m1_supply_names[] = {
	"avdd",
	"dvdd",
	"dovdd",
};

static const s64 gc02m1_link_freq_menu[] = {
	GC02M1_LINK_FREQ,
};

static const char *const gc02m1_test_pattern_menu[] = {
	"Disabled",
	"Color bars",
	"Grayscale",
	"Checkerboard",
};

struct gc02m1_mode {
	u32 width;
	u32 height;
	u32 hts;	/* horizontal timing size, in pixels */
	u32 vts_def;	/* default vertical timing size, in lines */
	u32 vts_min;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

struct gc02m1 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc02m1_supply_names)];
	struct gpio_desc *reset_gpio;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	const struct gc02m1_mode *cur_mode;
	bool streaming;
};

/* Register sequence, from tables/gc02m1ofilm_tables.h (vendor init table). */
static const struct cci_reg_sequence gc02m1_init_regs[] = {
	{ CCI_REG8(0xfc), 0x01 }, { CCI_REG8(0xf4), 0x41 }, { CCI_REG8(0xf5), 0xc0 },
	{ CCI_REG8(0xf6), 0x44 }, { CCI_REG8(0xf8), 0x38 }, { CCI_REG8(0xf9), 0x82 },
	{ CCI_REG8(0xfa), 0x00 }, { CCI_REG8(0xfd), 0x80 }, { CCI_REG8(0xfc), 0x81 },
	{ CCI_REG8(0xfe), 0x03 }, { CCI_REG8(0x1), 0x0b }, { CCI_REG8(0xf7), 0x01 },
	{ CCI_REG8(0xfc), 0x80 }, { CCI_REG8(0xfc), 0x80 }, { CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x8e }, { CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0x87), 0x09 },
	{ CCI_REG8(0xee), 0x72 }, { CCI_REG8(0xfe), 0x01 }, { CCI_REG8(0x8c), 0x90 },
	{ CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0x90), 0x00 }, { CCI_REG8(0x3), 0x04 },
	{ CCI_REG8(0x4), 0x7d }, { CCI_REG8(0x41), 0x04 }, { CCI_REG8(0x42), 0xf4 },
	{ CCI_REG8(0x5), 0x04 }, { CCI_REG8(0x6), 0x48 }, { CCI_REG8(0x7), 0x00 },
	{ CCI_REG8(0x8), 0x18 }, { CCI_REG8(0x9d), 0x18 }, { CCI_REG8(0x9), 0x00 },
	{ CCI_REG8(0xa), 0x02 }, { CCI_REG8(0xd), 0x04 }, { CCI_REG8(0xe), 0xbc },
	{ CCI_REG8(0x17), 0x80 }, { CCI_REG8(0x19), 0x04 }, { CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x56), 0x20 }, { CCI_REG8(0x5b), 0x00 }, { CCI_REG8(0x5e), 0x01 },
	{ CCI_REG8(0x21), 0x3c }, { CCI_REG8(0x44), 0x20 }, { CCI_REG8(0xcc), 0x01 },
	{ CCI_REG8(0x1a), 0x04 }, { CCI_REG8(0x1f), 0x11 }, { CCI_REG8(0x27), 0x30 },
	{ CCI_REG8(0x2b), 0x00 }, { CCI_REG8(0x33), 0x00 }, { CCI_REG8(0x53), 0x90 },
	{ CCI_REG8(0xe6), 0x50 }, { CCI_REG8(0x39), 0x07 }, { CCI_REG8(0x43), 0x04 },
	{ CCI_REG8(0x46), 0x2a }, { CCI_REG8(0x7c), 0xa0 }, { CCI_REG8(0xd0), 0xbe },
	{ CCI_REG8(0xd1), 0x60 }, { CCI_REG8(0xd2), 0x40 }, { CCI_REG8(0xd3), 0xf3 },
	{ CCI_REG8(0xde), 0x1d }, { CCI_REG8(0xcd), 0x05 }, { CCI_REG8(0xce), 0x6f },
	{ CCI_REG8(0xfc), 0x88 }, { CCI_REG8(0xfe), 0x10 }, { CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e }, { CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 }, { CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x04 }, { CCI_REG8(0xe0), 0x01 }, { CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x01 }, { CCI_REG8(0x53), 0x44 }, { CCI_REG8(0x87), 0x53 },
	{ CCI_REG8(0x89), 0x03 }, { CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0xb0), 0x74 },
	{ CCI_REG8(0xb1), 0x04 }, { CCI_REG8(0xb2), 0x00 }, { CCI_REG8(0xb6), 0x00 },
	{ CCI_REG8(0xfe), 0x04 }, { CCI_REG8(0xd8), 0x00 }, { CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x60 }, { CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x2a }, { CCI_REG8(0xc0), 0x80 }, { CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0x40 }, { CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0x90 }, { CCI_REG8(0xc0), 0x19 },
	{ CCI_REG8(0xc0), 0xc0 }, { CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0xd0 },
	{ CCI_REG8(0xc0), 0x2f }, { CCI_REG8(0xc0), 0xe0 }, { CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 }, { CCI_REG8(0xc0), 0x39 }, { CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0x20 }, { CCI_REG8(0xc0), 0x04 },
	{ CCI_REG8(0xc0), 0x20 }, { CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x0f }, { CCI_REG8(0xc0), 0x40 }, { CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 }, { CCI_REG8(0xc0), 0x1a }, { CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0x20 }, { CCI_REG8(0xc0), 0x25 },
	{ CCI_REG8(0xc0), 0x80 }, { CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x2c }, { CCI_REG8(0xc0), 0xa0 }, { CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 }, { CCI_REG8(0xc0), 0x32 }, { CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0x20 }, { CCI_REG8(0xc0), 0x38 },
	{ CCI_REG8(0xc0), 0xe0 }, { CCI_REG8(0xc0), 0x01 }, { CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x3c }, { CCI_REG8(0xc0), 0x00 }, { CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0xa0 }, { CCI_REG8(0xc0), 0x40 }, { CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x02 }, { CCI_REG8(0xc0), 0x18 }, { CCI_REG8(0xc0), 0x5c },
	{ CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0x9f), 0x10 }, { CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x26), 0x20 }, { CCI_REG8(0xfe), 0x01 }, { CCI_REG8(0x40), 0x22 },
	{ CCI_REG8(0x46), 0x7f }, { CCI_REG8(0x49), 0x0f }, { CCI_REG8(0x4a), 0xf0 },
	{ CCI_REG8(0xfe), 0x04 }, { CCI_REG8(0x14), 0x80 }, { CCI_REG8(0x15), 0x80 },
	{ CCI_REG8(0x16), 0x80 }, { CCI_REG8(0x17), 0x80 }, { CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x41), 0x20 }, { CCI_REG8(0x4c), 0x00 }, { CCI_REG8(0x4d), 0x0c },
	{ CCI_REG8(0x44), 0x08 }, { CCI_REG8(0x48), 0x03 }, { CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x90), 0x01 }, { CCI_REG8(0x91), 0x00 }, { CCI_REG8(0x92), 0x06 },
	{ CCI_REG8(0x93), 0x00 }, { CCI_REG8(0x94), 0x06 }, { CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xb0 }, { CCI_REG8(0x97), 0x06 }, { CCI_REG8(0x98), 0x40 },
	{ CCI_REG8(0xfe), 0x03 }, { CCI_REG8(0x1), 0x23 }, { CCI_REG8(0x3), 0xce },
	{ CCI_REG8(0x4), 0x48 }, { CCI_REG8(0x15), 0x00 }, { CCI_REG8(0x21), 0x10 },
	{ CCI_REG8(0x22), 0x05 }, { CCI_REG8(0x23), 0x20 }, { CCI_REG8(0x25), 0x20 },
	{ CCI_REG8(0x26), 0x08 }, { CCI_REG8(0x29), 0x06 }, { CCI_REG8(0x2a), 0x0a },
	{ CCI_REG8(0x2b), 0x08 }, { CCI_REG8(0xfe), 0x01 }, { CCI_REG8(0x8c), 0x10 },
	{ CCI_REG8(0xfe), 0x00 }, { CCI_REG8(0x3e), 0x00 },
};

static const struct gc02m1_mode gc02m1_modes[] = {
	{
		.width = GC02M1_NATIVE_WIDTH,
		.height = GC02M1_NATIVE_HEIGHT,
		/* 84 MHz pixel clock / 30 fps / 1200 lines = 2333 pixels */
		.hts = 2192,
		.vts_def = 1276,
		.vts_min = 1276,
	},
};

static inline struct gc02m1 *to_gc02m1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc02m1, sd);
}

static u64 gc02m1_link_freq_to_pixel_rate(u64 link_freq, u8 lanes)
{
	/* two bits per clock cycle per lane, ten bits per pixel */
	return div_u64(link_freq * 2 * lanes, 10);
}

static int gc02m1_power_on(struct gc02m1 *gc02m1)
{
	struct device *dev = gc02m1->dev;
	int ret;

	ret = clk_prepare_enable(gc02m1->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(gc02m1_supplies),
				    gc02m1->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		clk_disable_unprepare(gc02m1->xclk);
		return ret;
	}

	/* the vendor power table releases reset after the rails are up */
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(gc02m1->reset_gpio, 0);
	usleep_range(5000, 6000);

	return 0;
}

static void gc02m1_power_off(struct gc02m1 *gc02m1)
{
	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(gc02m1_supplies), gc02m1->supplies);
	clk_disable_unprepare(gc02m1->xclk);
}

static int gc02m1_write_regs(struct gc02m1 *gc02m1,
			     const struct cci_reg_sequence *regs,
			     unsigned int n_regs)
{
	int err = 0;

	cci_multi_reg_write(gc02m1->regmap, regs, n_regs, &err);
	if (err)
		dev_err(gc02m1->dev, "failed to write the register table: %d\n",
			err);
	return err;
}

static int gc02m1_identify(struct gc02m1 *gc02m1)
{
	u64 id = 0;
	int ret;

	ret = cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID, &id, NULL);
	if (ret)
		return ret;

	if (id != GC02M1_CHIP_ID) {
		dev_err(gc02m1->dev, "chip id 0x%llx, expected 0x%x\n", id,
			GC02M1_CHIP_ID);
		return -ENODEV;
	}
	return 0;
}

static int gc02m1_start_streaming(struct gc02m1 *gc02m1)
{
	const struct gc02m1_mode *mode = gc02m1->cur_mode;
	int ret;

	ret = gc02m1_write_regs(gc02m1, gc02m1_init_regs,
				ARRAY_SIZE(gc02m1_init_regs));
	if (ret)
		return ret;

	ret = cci_write(gc02m1->regmap, GC02M1_VBLANK_REG, mode->vts_def,
			NULL);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(gc02m1->sd.ctrl_handler);
	if (ret)
		return ret;

	return cci_write(gc02m1->regmap, GC02M1_STREAMING_REG,
			 GC02M1_STREAMING_MASK, NULL);
}

static int gc02m1_stop_streaming(struct gc02m1 *gc02m1)
{
	return cci_write(gc02m1->regmap, GC02M1_STREAMING_REG, 0, NULL);
}

static int gc02m1_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	if (enable == gc02m1->streaming)
		return 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(gc02m1->dev);
		if (ret)
			return ret;
		ret = gc02m1_start_streaming(gc02m1);
		if (ret) {
			pm_runtime_put(gc02m1->dev);
			return ret;
		}
	} else {
		gc02m1_stop_streaming(gc02m1);
		pm_runtime_put(gc02m1->dev);
	}

	gc02m1->streaming = enable;
	return 0;
}

static int gc02m1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m1 *gc02m1 = container_of(ctrl->handler, struct gc02m1,
					     ctrls);
	int ret;

	if (!gc02m1->streaming && ctrl->id != V4L2_CID_TEST_PATTERN)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(gc02m1->regmap, GC02M1_EXPOSURE_REG, ctrl->val,
				NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(gc02m1->regmap, GC02M1_AGAIN_REG, ctrl->val,
				NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(gc02m1->regmap, GC02M1_VBLANK_REG,
				ctrl->val + gc02m1->cur_mode->height, NULL);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(gc02m1->regmap, GC02M1_HBLANK_REG, ctrl->val,
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_update_bits(gc02m1->regmap, GC02M1_TEST_PATTERN_REG,
				      GC02M1_TEST_PATTERN_MASK,
				      ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret)
		dev_err(gc02m1->dev, "control 0x%x: %d\n", ctrl->id, ret);
	return ret;
}

static const struct v4l2_ctrl_ops gc02m1_ctrl_ops = {
	.s_ctrl = gc02m1_set_ctrl,
};

static int gc02m1_init_controls(struct gc02m1 *gc02m1)
{
	struct v4l2_ctrl_handler *handler = &gc02m1->ctrls;
	const struct gc02m1_mode *mode = gc02m1->cur_mode;
	u32 exposure_max;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;

	gc02m1->link_freq =
		v4l2_ctrl_new_int_menu(handler, &gc02m1_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       gc02m1_link_freq_menu);
	if (gc02m1->link_freq)
		gc02m1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = gc02m1_link_freq_to_pixel_rate(GC02M1_LINK_FREQ,
						    GC02M1_DATA_LANES);
	gc02m1->pixel_rate = v4l2_ctrl_new_std(handler, &gc02m1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 0,
					       pixel_rate, 1, pixel_rate);
	if (gc02m1->pixel_rate)
		gc02m1->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc02m1->hblank = v4l2_ctrl_new_std(handler, &gc02m1_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->hts - mode->width,
					   mode->hts - mode->width, 1,
					   mode->hts - mode->width);
	if (gc02m1->hblank)
		gc02m1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc02m1->vblank = v4l2_ctrl_new_std(handler, &gc02m1_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vts_min - mode->height,
					   GC02M1_VBLANK_MAX - mode->height, 1,
					   mode->vts_def - mode->height);

	exposure_max = mode->vts_def - GC02M1_EXPOSURE_MARGIN;
	gc02m1->exposure = v4l2_ctrl_new_std(handler, &gc02m1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC02M1_EXPOSURE_MIN, exposure_max,
					     GC02M1_EXPOSURE_STEP,
					     exposure_max);

	v4l2_ctrl_new_std(handler, &gc02m1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC02M1_AGAIN_MIN, GC02M1_AGAIN_MAX,
			  GC02M1_AGAIN_STEP, GC02M1_AGAIN_MIN);

	v4l2_ctrl_new_std_menu_items(handler, &gc02m1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc02m1_test_pattern_menu) - 1,
				     0, 0, gc02m1_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	gc02m1->sd.ctrl_handler = handler;
	return 0;
}

static int gc02m1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_state_get_format(state, 0);
	if (fmt) {
		fmt->code = GC02M1_MBUS_CODE;
		fmt->width = gc02m1_modes[0].width;
		fmt->height = gc02m1_modes[0].height;
		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = V4L2_COLORSPACE_RAW;
	}

	crop = v4l2_subdev_state_get_crop(state, 0);
	if (crop) {
		crop->width = gc02m1_modes[0].width;
		crop->height = gc02m1_modes[0].height;
	}
	return 0;
}

static int gc02m1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = GC02M1_MBUS_CODE;
	return 0;
}

static int gc02m1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;
	fse->min_width = gc02m1_modes[0].width;
	fse->max_width = gc02m1_modes[0].width;
	fse->min_height = gc02m1_modes[0].height;
	fse->max_height = gc02m1_modes[0].height;
	return 0;
}

static int gc02m1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *v4l2_subdev_state_get_crop(state, 0);
	return 0;
}

static int gc02m1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);
	if (!format)
		return -EINVAL;

	if (fmt->format.code != GC02M1_MBUS_CODE)
		fmt->format.code = GC02M1_MBUS_CODE;
	fmt->format.width = gc02m1_modes[0].width;
	fmt->format.height = gc02m1_modes[0].height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	*format = fmt->format;
	return 0;
}

static const struct v4l2_subdev_pad_ops gc02m1_pad_ops = {
	.enum_mbus_code = gc02m1_enum_mbus_code,
	.enum_frame_size = gc02m1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc02m1_set_pad_format,
	.get_selection = gc02m1_get_selection,
};

static const struct v4l2_subdev_video_ops gc02m1_video_ops = {
	.s_stream = gc02m1_set_stream,
};

static const struct v4l2_subdev_ops gc02m1_subdev_ops = {
	.pad = &gc02m1_pad_ops,
	.video = &gc02m1_video_ops,
};

static const struct v4l2_subdev_internal_ops gc02m1_internal_ops = {
	.init_state = gc02m1_init_state,
};

static const struct media_entity_operations gc02m1_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int gc02m1_get_pm_resources(struct gc02m1 *gc02m1)
{
	struct device *dev = gc02m1->dev;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(gc02m1_supply_names); i++)
		gc02m1->supplies[i].supply = gc02m1_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc02m1_supplies),
				      gc02m1->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	gc02m1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc02m1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc02m1->reset_gpio),
				     "failed to get reset gpio\n");

	gc02m1->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc02m1->xclk))
		return dev_err_probe(dev, PTR_ERR(gc02m1->xclk),
				     "failed to get xclk\n");

	return 0;
}

static int gc02m1_check_hwcfg(struct gc02m1 *gc02m1)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct device *dev = gc02m1->dev;
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!fwnode)
		return dev_err_probe(dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return ret;

	if (ep.bus.mipi_csi2.num_data_lanes != GC02M1_DATA_LANES)
		return dev_err_probe(dev, -EINVAL, "%u data lanes, expected %u\n",
				     ep.bus.mipi_csi2.num_data_lanes,
				     GC02M1_DATA_LANES);

	return 0;
}

static int gc02m1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct gc02m1 *gc02m1;
	int ret;

	gc02m1 = devm_kzalloc(dev, sizeof(*gc02m1), GFP_KERNEL);
	if (!gc02m1)
		return -ENOMEM;
	gc02m1->dev = dev;

	ret = gc02m1_check_hwcfg(gc02m1);
	if (ret)
		return ret;

	ret = gc02m1_get_pm_resources(gc02m1);
	if (ret)
		return ret;

	gc02m1->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc02m1->regmap))
		return dev_err_probe(dev, PTR_ERR(gc02m1->regmap),
				     "failed to init CCI\n");

	gc02m1->cur_mode = &gc02m1_modes[0];

	sd = &gc02m1->sd;
	v4l2_i2c_subdev_init(sd, client, &gc02m1_subdev_ops);
	sd->internal_ops = &gc02m1_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &gc02m1_entity_ops;

	gc02m1->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &gc02m1->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init media entity\n");
		return ret;
	}

	ret = gc02m1_init_controls(gc02m1);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init controls\n");
		goto err_entity;
	}

	sd->state_lock = gc02m1->ctrls.lock;
	ret = v4l2_subdev_init_finalize(sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to finalize subdev\n");
		goto err_ctrls;
	}

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	ret = gc02m1_identify(gc02m1);
	if (ret)
		dev_warn(dev, "no chip id: %d (power state is the runtime PM's)\n",
			 ret);

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	return 0;

err_pm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(sd);
err_ctrls:
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void gc02m1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(gc02m1->dev);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
	media_entity_cleanup(&sd->entity);
}

static int gc02m1_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	gc02m1_power_off(to_gc02m1(sd));
	return 0;
}

static int gc02m1_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	return gc02m1_power_on(to_gc02m1(sd));
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc02m1_pm_ops, gc02m1_runtime_suspend,
				 gc02m1_runtime_resume, NULL);

static const struct of_device_id gc02m1_of_match[] = {
	{ .compatible = "galaxycore,gc02m1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc02m1_of_match);

static const struct i2c_device_id gc02m1_id_table[] = {
	{ "gc02m1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, gc02m1_id_table);

static struct i2c_driver gc02m1_driver = {
	.driver = {
		.name = "gc02m1",
		.pm = pm_ptr(&gc02m1_pm_ops),
		.of_match_table = gc02m1_of_match,
	},
	.probe = gc02m1_probe,
	.remove = gc02m1_remove,
	.id_table = gc02m1_id_table,
};
module_i2c_driver(gc02m1_driver);

MODULE_DESCRIPTION("GalaxyCore GC02M1 image sensor driver");
MODULE_LICENSE("GPL");
