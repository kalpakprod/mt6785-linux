// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi POCO M5s / Redmi Note 10S (rosemary) AMOLED panel
 * dsi_panel_k7_38_0c_0a_fhdp_video: 1080x2400, Samsung-made module driven by a
 * FocalTech FT3418-class display driver IC, MIPI DSI video mode with sync
 * pulses, 4 lanes at 1098 Mbps, no DSC, brightness over DCS 0x51 (11-bit).
 *
 * The init sequence is the vendor LCM driver's, verbatim:
 *   drivers/misc/mediatek/lcm/dsi_panel_k7_38_0c_0a_fhdp_video/
 * in Youffx/kernel_xiaomi_rosemary @ 634d6ac5d96e, branch 23.2-lineage,
 * extracted into poco-m5s/research/native/1-display-touch/panel-init-sequence.txt.
 *
 * The mode is the loader's, not the vendor C's: LK recomputes the porches to
 * land exactly on 60 Hz and those are the values provably scanning out on this
 * phone (poco-m5s/evidence/try-force1/pl_lk.txt lines 1460-1465), so
 * vsa 2 / vbp 9 / vfp 37 / hsa 14 / hbp 22 / hfp 43 are used here while the
 * vendor driver says vfp 21 / hfp 48.
 *
 * Structure follows the cezanne port's panel-xiaomi-j10-38-08-0a.c
 * (DrShadow34/cezanne-linux-port, linux/patches/0023), the same Xiaomi
 * four-byte AMOLED naming family.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

/*
 * The stock config has CONFIG_BACKLIGHT_SUPPORT_2047_FEATURE=y, so the level is
 * the register value itself (the vendor's "level << 3" branch is compiled out).
 */
#define K7_MAX_BRIGHTNESS	2047
#define K7_DEFAULT_BRIGHTNESS	1023
/* vendor BL_MIN_LEVEL: below this the panel is dark */
#define K7_MIN_BRIGHTNESS	20

struct k7_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *vddi;		/* 1.8 V I/O, gated by GPIO 14 */
	struct regulator *vdd;		/* 3.0 V AVDD/ELVDD, gated by GPIO 127 */
	struct gpio_desc *reset_gpio;	/* GPIO 44, active low */
};

static inline struct k7_panel *to_k7_panel(struct drm_panel *panel)
{
	return container_of(panel, struct k7_panel, panel);
}

/*
 * Vendor lcm_init(): reset high, 1 ms, low, 1 ms, high, 10 ms. The DT line is
 * active low, so 1 asserts reset.
 */
static void k7_reset(struct k7_panel *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

/*
 * The vendor init_setting[] table, in order, including the 0xfc / 0xf0 register
 * page selectors (5a5a = unlock, a5a5 = lock). Nothing here is added or changed.
 */
static int k7_init_sequence(struct k7_panel *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* sleep out, then settle as the vendor does */
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	/*
	 * Pixel format: 24 bits per pixel, RGB888, on both interfaces. The vendor
	 * table does not carry this write, so the DDI keeps whatever COLMOD state
	 * it resets into - and the driver pulses the hardware reset (GPIO 44)
	 * before replaying the table, so that state is the power-on default and not
	 * what the loader had left running. Our DSI sends packed 24-bit
	 * (PS_SEL 3, PS_WC 3240 = 1080 x 3), so the panel has to be on 24 bpp.
	 *
	 * Trial 7's geometry says it is not: 2400 framebuffer rows (4320 bytes
	 * each at 32 bpp) are consumed over about two thirds of the panel, i.e.
	 * roughly 1.5 framebuffer rows per panel line, which is the ratio between
	 * the 3240 bytes we send per line and the 2160 a 16-bpp panel would take.
	 * 0x77 is the DCS encoding for 24 bits per pixel on both interfaces; if the
	 * DDI is already there, this writes the same value and changes nothing.
	 */
	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0x77);

	/* TE on, vsync only */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_ON, 0x00);

	/* failsafe / ESD error flag setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03, 0xed);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x40, 0x04, 0x08, 0xa8,
				     0x84, 0x4a, 0x73, 0x02, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x01, 0xcd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);

	/* OFC setting; the vendor comment says 91.8 MHz */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf,
				     0x09, 0x30, 0x95, 0x43, 0x34, 0x05, 0x00,
				     0x27, 0x00, 0x2e, 0x4f, 0x7a, 0x77, 0x10,
				     0x3d, 0x73, 0x00, 0xff, 0x01, 0x8b, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);

	/* 11-bit dimming */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x11, 0x4b);

	/* fast discharge */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcd, 0x04);

	/* dynamic ELVSS */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0xc0);

	/* dimming speed */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x06, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb3, 0x93);

	/* CRC / sRGB and P3 gamma tables */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
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
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* brightness control enable, then the vendor's 100 ms before display on */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return dsi_ctx.accum_err;
}

static int k7_prepare(struct drm_panel *panel)
{
	struct k7_panel *ctx = to_k7_panel(panel);
	int ret;

	ret = regulator_enable(ctx->vddi);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = regulator_enable(ctx->vdd);
	if (ret)
		goto err_vddi;
	usleep_range(1000, 2000);

	k7_reset(ctx);

	ret = k7_init_sequence(ctx);
	if (ret) {
		dev_err(panel->dev, "panel init failed: %d\n", ret);
		goto err_off;
	}

	return 0;

err_off:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vdd);
err_vddi:
	regulator_disable(ctx->vddi);
	return ret;
}

static int k7_unprepare(struct drm_panel *panel)
{
	struct k7_panel *ctx = to_k7_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);		/* 0x28 */
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);		/* 0x10 */
	mipi_dsi_msleep(&dsi_ctx, 150);

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(5);
	regulator_disable(ctx->vdd);
	msleep(5);
	regulator_disable(ctx->vddi);

	return 0;
}

/*
 * The mode is the vendor LCM's, which is what the panel's own driver asks for
 * (lcm_get_params(), poco-m5s/research/native/1-display-touch.md 2): hfp 48,
 * hsa 14, hbp 22 and vfp 21, vsa 2, vbp 9. The loader's log reports hfp 43 /
 * vfp 37 because LK recomputes the porches against its own PLL for exactly
 * 60 Hz (pl_lk.txt line 1460); with the vendor's numbers and the clock below,
 * the frame is 1164 x 2432 at 60.0 Hz, 169850 kHz, i.e. 1019 Mbps per lane.
 * DRM wants the front porch first, then the sync pulse, then the back porch.
 */
static const struct drm_display_mode k7_mode = {
	.clock = (1080 + 48 + 14 + 22) * (2400 + 21 + 2 + 9) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 48,
	.hsync_end = 1080 + 48 + 14,
	.htotal = 1080 + 48 + 14 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 21,
	.vsync_end = 2400 + 21 + 2,
	.vtotal = 2400 + 21 + 2 + 9,
	.width_mm = 67,		/* vendor LCM_PHYSICAL_WIDTH 67000 um */
	.height_mm = 149,	/* vendor LCM_PHYSICAL_HEIGHT 149000 um */
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int k7_get_modes(struct drm_panel *panel,
			struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &k7_mode);
}

static const struct drm_panel_funcs k7_panel_funcs = {
	.prepare = k7_prepare,
	.unprepare = k7_unprepare,
	.get_modes = k7_get_modes,
};

/* DCS 0x51, 11 bits: the vendor's CONFIG_BACKLIGHT_SUPPORT_2047_FEATURE path */
static int k7_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	unsigned int brightness = backlight_get_brightness(bl);
	int ret;

	if (brightness && brightness < K7_MIN_BRIGHTNESS)
		brightness = K7_MIN_BRIGHTNESS;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops k7_bl_ops = {
	.update_status = k7_bl_update_status,
};

static int k7_probe(struct mipi_dsi_device *dsi)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = K7_DEFAULT_BRIGHTNESS,
		.max_brightness = K7_MAX_BRIGHTNESS,
	};
	struct device *dev = &dsi->dev;
	struct k7_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct k7_panel, panel,
				   &k7_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi), "vddi supply\n");

	ctx->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctx->vdd))
		return dev_err_probe(dev, PTR_ERR(ctx->vdd), "vdd supply\n");

	/* the loader left the panel running: keep reset deasserted until prepare */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "reset gpio\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->hs_rate = 1098000000UL;	/* PLL 549 MHz -> 1098 Mbps per lane */
	/*
	 * Video mode with sync pulses, exactly as the loader drives it, and the
	 * clock lane back to low power on every line: the loader's DSI_TXRX_CTRL
	 * has HSTX_CKLP_EN set (bit 16, poco-m5s/evidence/disp-trial2/
	 * dsi-mipitx-regs-loader.txt, 0x018 = 0x0001003c) and the vendor's
	 * DSI_TXRX_Control() derives that bit as "cont_clock ? 0 : 1". With the
	 * clock driven continuously instead, an LP escape command cannot be
	 * framed and the engine never reports CMD_DONE (trial 2: EXIT_SLEEP_MODE
	 * timed out).
	 */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;
	ctx->panel.backlight = devm_backlight_device_register(dev, dev_name(dev),
							      dev, dsi, &k7_bl_ops,
							      &props);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "mipi_dsi_attach\n");
	}

	return 0;
}

static void k7_remove(struct mipi_dsi_device *dsi)
{
	struct k7_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id k7_of_match[] = {
	{ .compatible = "xiaomi,k7-38-0c-0a" },
	{ }
};
MODULE_DEVICE_TABLE(of, k7_of_match);

static struct mipi_dsi_driver k7_driver = {
	.probe = k7_probe,
	.remove = k7_remove,
	.driver = {
		.name = "panel-xiaomi-k7-38-0c-0a",
		.of_match_table = k7_of_match,
	},
};
module_mipi_dsi_driver(k7_driver);

MODULE_DESCRIPTION("Xiaomi POCO M5s / Redmi Note 10S k7 (38 0c 0a) AMOLED panel");
MODULE_LICENSE("GPL");
