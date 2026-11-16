// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) Kamil Gołda <kamil.golda@protonmail.com>
// Copyright (c) Yedaya Katsman <yedaya.ka@gmail.com>
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) The Linux Foundation. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct s6e8fc0_ctx;

/*
 * One S6E8FC0 panel: everything that differs between the modules this controller
 * drives. The controller itself (test keys, 0xb0 offset register, brightness over
 * 0x51) is common, so the board-specific parts are the mode, the power-on
 * sequence, the rails and the DSI flags.
 */
struct s6e8fc0_panel_desc {
	const char *name;
	const struct drm_display_mode *mode;
	int (*on)(struct s6e8fc0_ctx *ctx);
	int (*off)(struct s6e8fc0_ctx *ctx);
	void (*reset)(struct s6e8fc0_ctx *ctx);
	const struct regulator_bulk_data *supplies;
	const unsigned int *rail_delay_us;
	unsigned int num_supplies;
	unsigned long mode_flags;
	unsigned int max_brightness;
	unsigned int default_brightness;
};

struct s6e8fc0_ctx {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct s6e8fc0_panel_desc *desc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
};

static inline
struct s6e8fc0_ctx *to_s6e8fc0_ctx(struct drm_panel *panel)
{
	return container_of_const(panel, struct s6e8fc0_ctx, panel);
}

#define s6e8fc0_test_key_on_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e8fc0_test_key_off_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e8fc0_test_key_on_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define s6e8fc0_test_key_off_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)

/* ------------------------------------------------------------------ Mi A3 */

static const struct regulator_bulk_data s6e8fc0_m1906f9_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vci" },
};

static void s6e8fc0_m1906f9_reset(struct s6e8fc0_ctx *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int s6e8fc0_m1906f9_on(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	s6e8fc0_test_key_on_lvl3(&dsi_ctx);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04, 0xed);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed,
				     0xe4, 0x08, 0x96, 0xa4, 0x2a, 0x72, 0xe2,
				     0xca, 0x00);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);
	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	s6e8fc0_test_key_on_lvl3(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x01, 0x81, 0x04);
	s6e8fc0_test_key_off_lvl2(&dsi_ctx);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int s6e8fc0_m1906f9_off(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static const struct drm_display_mode s6e8fc0_m1906f9_mode = {
	.clock = (720 + 350 + 40 + 294) * (1560 + 17 + 2 + 5) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 350,
	.hsync_end = 720 + 350 + 40,
	.htotal = 720 + 350 + 40 + 294,
	.vdisplay = 1560,
	.vsync_start = 1560 + 17,
	.vsync_end = 1560 + 17 + 2,
	.vtotal = 1560 + 17 + 2 + 5,
	.width_mm = 65,
	.height_mm = 140,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct s6e8fc0_panel_desc s6e8fc0_m1906f9_desc = {
	.name = "s6e8fc0-m1906f9",
	.mode = &s6e8fc0_m1906f9_mode,
	.on = s6e8fc0_m1906f9_on,
	.off = s6e8fc0_m1906f9_off,
	.reset = s6e8fc0_m1906f9_reset,
	.supplies = s6e8fc0_m1906f9_supplies,
	.num_supplies = ARRAY_SIZE(s6e8fc0_m1906f9_supplies),
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.max_brightness = 1023,
	.default_brightness = 512,
};

/* ----------------------------------------- POCO M5s / Redmi Note 10S (k7) */

/*
 * The k7 38 0c 0a module: 1080x2400, same S6E8FC0 controller, DSI video mode
 * with sync pulses, 4 lanes, RGB888, no DSC.
 *
 * Two rails, both gated by GPIOs on the board and modelled as fixed regulators:
 * vci is VDDI 1.8 V (GPIO 14), vdd is AVDD/ELVDD 3.0 V (GPIO 127).
 *
 * The sequence is the vendor LCM driver's, verbatim:
 *   drivers/misc/mediatek/lcm/dsi_panel_k7_38_0c_0a_fhdp_video/
 * in Youffx/kernel_xiaomi_rosemary @ 634d6ac5d96e, branch 23.2-lineage, and the
 * same table is carried by two independent Xiaomi drivers, begonia's MTK-DRM
 * panel-samsung-s6e8fc01.c (same controller, same porches; only 0x80 differs,
 * 0x01 there and 0x80 here) and the Qualcomm DT table
 * dsi_panel_k7_38_0c_0a_fhdp_video.dtsi of the Redmi Note 10 sunny/mojito trees.
 *
 * The mode is the loader's rather than the vendor C's: LK recomputes the porches
 * against its PLL to land on exactly 60 Hz and those are the values proven to
 * scan out on this phone (poco-m5s/evidence/try-force1/pl_lk.txt:1460-1465).
 */
static const struct regulator_bulk_data s6e8fc0_k7_38_0c_0a_supplies[] = {
	{ .supply = "vci" },	/* VDDI 1.8 V, GPIO 14 */
	{ .supply = "vdd" },	/* AVDD/ELVDD 3.0 V, GPIO 127 */
};

/*
 * Vendor lcm_init(): GPIO 14 (VDDI) high, then GPIO 127 (AVDD+ELVDD) high 1 ms
 * later; power-off the other way round in lcm_suspend_power(). Both rails are
 * GPIO-gated fixed regulators on this board, so the delay is all the sequencing
 * there is.
 */
static const unsigned int s6e8fc0_k7_38_0c_0a_rail_delays[] = { 1000, 0 };

static void s6e8fc0_k7_38_0c_0a_reset(struct s6e8fc0_ctx *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int s6e8fc0_k7_38_0c_0a_on(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	s6e8fc0_test_key_on_lvl3(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03, 0xed);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed,
				     0x40, 0x04, 0x08, 0xa8, 0x84, 0x4a, 0x73,
				     0x02, 0x0a);
	s6e8fc0_test_key_on_lvl3(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x01, 0xcd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x93);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);

	/* OFC setting; the vendor comment says 91.8 MHz */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf,
				     0x09, 0x30, 0x95, 0x43, 0x34, 0x05, 0x00,
				     0x27, 0x00, 0x2e, 0x4f, 0x7a, 0x77, 0x10,
				     0x3d, 0x73, 0x00, 0xff, 0x01, 0x8b, 0x08);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);

	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x11, 0x4b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x06, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x93);

	/* CRC / sRGB and P3 gamma tables */
	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x80, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1,
				     0xa7, 0x02, 0x05, 0x34, 0xcf, 0x14, 0x09,
				     0x04, 0xae, 0x46, 0xeb, 0xd0, 0xc7, 0x06,
				     0xc1, 0xe6, 0xe2, 0x1d, 0xff, 0xfa, 0xe1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x16, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1,
				     0xd9, 0x00, 0x00, 0x06, 0xcf, 0x02, 0x09,
				     0x03, 0xc6, 0x10, 0xf4, 0xd9, 0xfd, 0x00,
				     0xe0, 0xe8, 0xe1, 0x03, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x55, 0xb1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x80);
	s6e8fc0_test_key_off_lvl2(&dsi_ctx);

	/* TE on, vsync only */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int s6e8fc0_k7_38_0c_0a_off(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

/*
 * Frame 1164 x 2432 at 60.0 Hz: the vendor LCM's porches (hfp 48, hsa 14,
 * hbp 22, vfp 21, vsa 2, vbp 9), clock 169850 kHz. The loader logs hfp 43 /
 * vfp 37 because LK recomputes them for its own PLL.
 */
static const struct drm_display_mode s6e8fc0_k7_38_0c_0a_mode = {
	.clock = (1080 + 48 + 14 + 22) * (2400 + 21 + 2 + 9) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 48,
	.hsync_end = 1080 + 48 + 14,
	.htotal = 1080 + 48 + 14 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 21,
	.vsync_end = 2400 + 21 + 2,
	.vtotal = 2400 + 21 + 2 + 9,
	.width_mm = 67,
	.height_mm = 149,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct s6e8fc0_panel_desc s6e8fc0_k7_38_0c_0a_desc = {
	.name = "s6e8fc0-k7-38-0c-0a",
	.mode = &s6e8fc0_k7_38_0c_0a_mode,
	.on = s6e8fc0_k7_38_0c_0a_on,
	.off = s6e8fc0_k7_38_0c_0a_off,
	.reset = s6e8fc0_k7_38_0c_0a_reset,
	.supplies = s6e8fc0_k7_38_0c_0a_supplies,
	.rail_delay_us = s6e8fc0_k7_38_0c_0a_rail_delays,
	.num_supplies = ARRAY_SIZE(s6e8fc0_k7_38_0c_0a_supplies),
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.max_brightness = 2047,
	.default_brightness = 1023,
};

/* ------------------------------------------------------------ drm plumbing */

static int s6e8fc0_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);

	return drm_connector_helper_get_modes_fixed(connector, ctx->desc->mode);
}

/*
 * The rails go up one at a time, in the order the description lists them, with the
 * settle time the board needs after each - regulator_bulk_enable() switches the
 * whole array on back to back, which cannot express "VDDI 1 ms before AVDD".
 * Power-down walks the same list backwards, which is the order the panel datasheet
 * and the vendor LCM use (AVDD/ELVDD first, then VDDI).
 */
static int s6e8fc0_power_on(struct s6e8fc0_ctx *ctx)
{
	const struct s6e8fc0_panel_desc *desc = ctx->desc;
	unsigned int i;
	int ret;

	for (i = 0; i < desc->num_supplies; i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret < 0) {
			dev_err(&ctx->dsi->dev, "Failed to enable %s: %d\n",
				desc->supplies[i].supply, ret);
			goto err_disable;
		}

		if (desc->rail_delay_us && desc->rail_delay_us[i])
			usleep_range(desc->rail_delay_us[i],
				     desc->rail_delay_us[i] + 1000);
	}

	return 0;

err_disable:
	while (i--)
		regulator_disable(ctx->supplies[i].consumer);

	return ret;
}

static void s6e8fc0_power_off(struct s6e8fc0_ctx *ctx)
{
	int i = ctx->desc->num_supplies;

	while (i--)
		regulator_disable(ctx->supplies[i].consumer);
}

static int s6e8fc0_prepare(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = s6e8fc0_power_on(ctx);
	if (ret < 0)
		return ret;

	ctx->desc->reset(ctx);

	ret = ctx->desc->on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		s6e8fc0_power_off(ctx);
		return ret;
	}

	return 0;
}

static int s6e8fc0_unprepare(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ctx->desc->off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	s6e8fc0_power_off(ctx);

	return 0;
}

static const struct drm_panel_funcs s6e8fc0_panel_funcs = {
	.prepare = s6e8fc0_prepare,
	.unprepare = s6e8fc0_unprepare,
	.get_modes = s6e8fc0_get_modes,
};

static int s6e8fc0_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int s6e8fc0_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops s6e8fc0_bl_ops = {
	.update_status = s6e8fc0_bl_update_status,
	.get_brightness = s6e8fc0_bl_get_brightness,
};

static struct backlight_device *s6e8fc0_create_backlight(struct s6e8fc0_ctx *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = ctx->desc->default_brightness,
		.max_brightness = ctx->desc->max_brightness,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, ctx->dsi,
					      &s6e8fc0_bl_ops, &props);
}

static int s6e8fc0_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e8fc0_ctx *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e8fc0_ctx, panel,
				   &s6e8fc0_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ret = devm_regulator_bulk_get_const(dev, ctx->desc->num_supplies,
					    ctx->desc->supplies, &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = ctx->desc->mode_flags;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = s6e8fc0_create_backlight(ctx);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void s6e8fc0_remove(struct mipi_dsi_device *dsi)
{
	struct s6e8fc0_ctx *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_s6e8fc0_of_match[] = {
	{ .compatible = "samsung,s6e8fc0-m1906f9",
	  .data = &s6e8fc0_m1906f9_desc },
	{ .compatible = "samsung,s6e8fc0-k7-38-0c-0a",
	  .data = &s6e8fc0_k7_38_0c_0a_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_s6e8fc0_of_match);

static struct mipi_dsi_driver s6e8fc0_driver = {
	.probe = s6e8fc0_probe,
	.remove = s6e8fc0_remove,
	.driver = {
		.name = "panel-samsung-s6e8fc0-m1906f9",
		.of_match_table = samsung_s6e8fc0_of_match,
	},
};
module_mipi_dsi_driver(s6e8fc0_driver);

MODULE_AUTHOR("Kamil Gołda <kamil.golda@protonmail.com>");
MODULE_AUTHOR("Yedaya Katsman <yedaya.ka@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung S6E8FC0 DSI panels");
MODULE_LICENSE("GPL");
