// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the OmniVision OV64B40 image sensor.
 *
 * The Poco M5s (rosemary) carries it as its main camera, on the camera I2C bus
 * at 7-bit 0x10, as a 3-trio C-PHY part (vendor driver
 * drivers/misc/mediatek/imgsensor/src/mt6785/ov64b40semco_mipi_raw/
 * ov64b40semco_mipi_raw.c of the Xiaomi MT6785 Android 4.19 tree:
 * .i2c_addr_table {0x20, 0x46, 0xff} in 8-bit form, .mipi_sensor_type
 * MIPI_CPHY, .mipi_lane_num SENSOR_MIPI_3_LANE, .grabwindow 4624x3472,
 * .mipi_pixel_rate 570 MHz, output RAW_4CELL_HW_BAYER_B).
 *
 * Structure and control set follow mainline drivers/media/i2c/ov64a40.c, the
 * sibling part of the same family (chip id 0x566441 against this one's
 * 0x566442, and that is the exact difference the identification check below
 * looks for). The register sequences are the vendor's, extracted by
 * poco-m5s/research/native/5-cameras/extract_tables.py into
 * tables/ov64b40semco_tables.h and copied from there in the vendor's order:
 * the 938-write init table and the 160-write capture table for 4624x3472.
 *
 * What this driver does NOT do: stream. The part is C-PHY and neither the PHY
 * driver nor the SENINF driver can program C-PHY yet (see the camera report),
 * so the driver refuses to start a stream with a message that says exactly
 * that, instead of starting one that cannot work. Everything else - power,
 * clock, reset, identification, formats, controls, the register tables - is
 * complete, and the stream path is one call away once C-PHY exists.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV64B40_REG_CHIP_ID		CCI_REG24(0x300a)
#define OV64B40_CHIP_ID			0x566442

#define OV64B40_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OV64B40_MODE_STANDBY		0x00
#define OV64B40_MODE_STREAMING		0x01

#define OV64B40_REG_SOFTWARE_RST	CCI_REG8(0x0103)
#define OV64B40_SOFTWARE_RST		0x01

#define OV64B40_REG_EXPOSURE		CCI_REG16(0x3501)
#define OV64B40_EXPOSURE_MIN		4

#define OV64B40_REG_AGAIN		CCI_REG16(0x3508)
#define OV64B40_AGAIN_MIN		1024
#define OV64B40_AGAIN_MAX		(1024 * 16)

#define OV64B40_REG_VTS			CCI_REG16(0x380e)
#define OV64B40_VTS_MAX			0xffff

#define OV64B40_REG_HTS			CCI_REG16(0x380c)

#define OV64B40_NATIVE_WIDTH		4624
#define OV64B40_NATIVE_HEIGHT		3472
#define OV64B40_DEFAULT_CLK_FREQ	(24 * 1000 * 1000)
#define OV64B40_DATA_LANES		3
#define OV64B40_LINK_FREQ		(570 * 1000 * 1000)
#define OV64B40_MBUS_CODE		MEDIA_BUS_FMT_SBGGR10_1X10

static const char *const ov64b40_supply_names[] = {
	"avdd",
	"dvdd",
	"dovdd",
};

static const s64 ov64b40_link_freq_menu[] = {
	OV64B40_LINK_FREQ,
};

struct ov64b40_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u32 vts_min;
	u32 link_freq_index;
};

struct ov64b40 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov64b40_supply_names)];
	struct gpio_desc *reset_gpio;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	const struct ov64b40_mode *cur_mode;
	bool streaming;
	bool cphy;	/* the endpoint describes a C-PHY bus */
};


/* 938 writes: the vendor's sensor_init sequence, in its order */
static const struct cci_reg_sequence ov64b40_init_regs[] = {
	{ CCI_REG8(0x103), 0x01 }, { CCI_REG8(0x102), 0x01 }, { CCI_REG8(0x102), 0x01 }, { CCI_REG8(0x301), 0x48 }, { CCI_REG8(0x303), 0x03 },
	{ CCI_REG8(0x304), 0x01 }, { CCI_REG8(0x305), 0x63 }, { CCI_REG8(0x306), 0x03 }, { CCI_REG8(0x307), 0x00 }, { CCI_REG8(0x309), 0x50 },
	{ CCI_REG8(0x316), 0x73 }, { CCI_REG8(0x320), 0x02 }, { CCI_REG8(0x321), 0x03 }, { CCI_REG8(0x323), 0x03 }, { CCI_REG8(0x324), 0x01 },
	{ CCI_REG8(0x325), 0x80 }, { CCI_REG8(0x326), 0xc3 }, { CCI_REG8(0x327), 0x03 }, { CCI_REG8(0x329), 0x00 }, { CCI_REG8(0x32c), 0x00 },
	{ CCI_REG8(0x32d), 0x01 }, { CCI_REG8(0x32e), 0x06 }, { CCI_REG8(0x32f), 0xe1 }, { CCI_REG8(0x343), 0x03 }, { CCI_REG8(0x344), 0x01 },
	{ CCI_REG8(0x345), 0x09 }, { CCI_REG8(0x350), 0x00 }, { CCI_REG8(0x360), 0x01 }, { CCI_REG8(0x361), 0x00 }, { CCI_REG8(0x3002), 0x00 },
	{ CCI_REG8(0x3008), 0x00 }, { CCI_REG8(0x3012), 0x31 }, { CCI_REG8(0x3015), 0x00 }, { CCI_REG8(0x301a), 0xb0 }, { CCI_REG8(0x301e), 0x88 },
	{ CCI_REG8(0x3025), 0x89 }, { CCI_REG8(0x3218), 0xa1 }, { CCI_REG8(0x3220), 0x12 }, { CCI_REG8(0x3221), 0x28 }, { CCI_REG8(0x3400), 0x04 },
	{ CCI_REG8(0x3406), 0x10 }, { CCI_REG8(0x340c), 0x00 }, { CCI_REG8(0x340d), 0x00 }, { CCI_REG8(0x340e), 0x60 }, { CCI_REG8(0x3422), 0x00 },
	{ CCI_REG8(0x3423), 0x00 }, { CCI_REG8(0x3424), 0x15 }, { CCI_REG8(0x3425), 0x40 }, { CCI_REG8(0x3426), 0x10 }, { CCI_REG8(0x3427), 0x10 },
	{ CCI_REG8(0x3428), 0x00 }, { CCI_REG8(0x3429), 0x00 }, { CCI_REG8(0x3421), 0x08 }, { CCI_REG8(0x3500), 0x00 }, { CCI_REG8(0x3501), 0x0f },
	{ CCI_REG8(0x3502), 0xec }, { CCI_REG8(0x3504), 0x0c }, { CCI_REG8(0x3508), 0x08 }, { CCI_REG8(0x3509), 0x00 }, { CCI_REG8(0x350a), 0x01 },
	{ CCI_REG8(0x350b), 0x00 }, { CCI_REG8(0x350c), 0x00 }, { CCI_REG8(0x3540), 0x00 }, { CCI_REG8(0x3541), 0x00 }, { CCI_REG8(0x3542), 0x20 },
	{ CCI_REG8(0x3544), 0x08 }, { CCI_REG8(0x3548), 0x01 }, { CCI_REG8(0x3549), 0x00 }, { CCI_REG8(0x354a), 0x01 }, { CCI_REG8(0x354b), 0x00 },
	{ CCI_REG8(0x3580), 0x00 }, { CCI_REG8(0x3581), 0x00 }, { CCI_REG8(0x3582), 0x10 }, { CCI_REG8(0x3584), 0x08 }, { CCI_REG8(0x3588), 0x01 },
	{ CCI_REG8(0x3589), 0x00 }, { CCI_REG8(0x358a), 0x01 }, { CCI_REG8(0x358b), 0x00 }, { CCI_REG8(0x3605), 0xf9 }, { CCI_REG8(0x3608), 0x8a },
	{ CCI_REG8(0x360a), 0x5d }, { CCI_REG8(0x360b), 0x50 }, { CCI_REG8(0x360c), 0x92 }, { CCI_REG8(0x360d), 0x05 }, { CCI_REG8(0x360e), 0x08 },
	{ CCI_REG8(0x3618), 0x80 }, { CCI_REG8(0x361e), 0x80 }, { CCI_REG8(0x3622), 0x0a }, { CCI_REG8(0x3623), 0x08 }, { CCI_REG8(0x3624), 0x18 },
	{ CCI_REG8(0x3628), 0x99 }, { CCI_REG8(0x362b), 0x77 }, { CCI_REG8(0x362d), 0x0a }, { CCI_REG8(0x3659), 0x48 }, { CCI_REG8(0x365a), 0x2b },
	{ CCI_REG8(0x365b), 0x0a }, { CCI_REG8(0x3684), 0x87 }, { CCI_REG8(0x3685), 0x00 }, { CCI_REG8(0x3688), 0x02 }, { CCI_REG8(0x3689), 0x88 },
	{ CCI_REG8(0x368a), 0x2e }, { CCI_REG8(0x368d), 0x00 }, { CCI_REG8(0x368e), 0x78 }, { CCI_REG8(0x368f), 0x00 }, { CCI_REG8(0x3694), 0x7f },
	{ CCI_REG8(0x3695), 0x00 }, { CCI_REG8(0x3696), 0x51 }, { CCI_REG8(0x3699), 0x19 }, { CCI_REG8(0x369a), 0x00 }, { CCI_REG8(0x369f), 0x20 },
	{ CCI_REG8(0x36a4), 0x00 }, { CCI_REG8(0x36a6), 0x00 }, { CCI_REG8(0x36a7), 0x00 }, { CCI_REG8(0x36a8), 0x00 }, { CCI_REG8(0x36aa), 0x12 },
	{ CCI_REG8(0x36ab), 0x28 }, { CCI_REG8(0x36b0), 0x12 }, { CCI_REG8(0x36b1), 0x28 }, { CCI_REG8(0x36b2), 0x12 }, { CCI_REG8(0x36b3), 0x28 },
	{ CCI_REG8(0x36b4), 0x12 }, { CCI_REG8(0x36b5), 0x28 }, { CCI_REG8(0x36b6), 0x12 }, { CCI_REG8(0x36b7), 0x28 }, { CCI_REG8(0x36b8), 0x12 },
	{ CCI_REG8(0x36b9), 0x28 }, { CCI_REG8(0x36ba), 0x12 }, { CCI_REG8(0x36bb), 0x28 }, { CCI_REG8(0x36bc), 0x12 }, { CCI_REG8(0x36bd), 0x28 },
	{ CCI_REG8(0x36be), 0x12 }, { CCI_REG8(0x36bf), 0x28 }, { CCI_REG8(0x36c0), 0x12 }, { CCI_REG8(0x36c1), 0x28 }, { CCI_REG8(0x3700), 0x2c },
	{ CCI_REG8(0x3701), 0x40 }, { CCI_REG8(0x3702), 0x4b }, { CCI_REG8(0x3703), 0x40 }, { CCI_REG8(0x3706), 0x3a }, { CCI_REG8(0x3708), 0x59 },
	{ CCI_REG8(0x3709), 0xf8 }, { CCI_REG8(0x370b), 0x82 }, { CCI_REG8(0x3711), 0x00 }, { CCI_REG8(0x3712), 0x50 }, { CCI_REG8(0x3713), 0x00 },
	{ CCI_REG8(0x3714), 0x67 }, { CCI_REG8(0x3716), 0x40 }, { CCI_REG8(0x3717), 0x82 }, { CCI_REG8(0x371d), 0x02 }, { CCI_REG8(0x371e), 0x12 },
	{ CCI_REG8(0x371f), 0x02 }, { CCI_REG8(0x3720), 0x08 }, { CCI_REG8(0x3721), 0x02 }, { CCI_REG8(0x3723), 0x00 }, { CCI_REG8(0x3724), 0x0d },
	{ CCI_REG8(0x3725), 0x22 }, { CCI_REG8(0x372d), 0x00 }, { CCI_REG8(0x3730), 0x14 }, { CCI_REG8(0x3731), 0x14 }, { CCI_REG8(0x3732), 0x07 },
	{ CCI_REG8(0x3734), 0x59 }, { CCI_REG8(0x3736), 0x0c }, { CCI_REG8(0x3738), 0x01 }, { CCI_REG8(0x3739), 0x18 }, { CCI_REG8(0x373b), 0x05 },
	{ CCI_REG8(0x3741), 0x48 }, { CCI_REG8(0x3743), 0xef }, { CCI_REG8(0x3744), 0x0f }, { CCI_REG8(0x3745), 0xff }, { CCI_REG8(0x3746), 0x00 },
	{ CCI_REG8(0x3748), 0x01 }, { CCI_REG8(0x3749), 0x18 }, { CCI_REG8(0x3751), 0x49 }, { CCI_REG8(0x3753), 0xff }, { CCI_REG8(0x3754), 0x00 },
	{ CCI_REG8(0x3755), 0x40 }, { CCI_REG8(0x3758), 0x88 }, { CCI_REG8(0x3759), 0x88 }, { CCI_REG8(0x375a), 0x88 }, { CCI_REG8(0x375b), 0x88 },
	{ CCI_REG8(0x375c), 0x88 }, { CCI_REG8(0x375e), 0x00 }, { CCI_REG8(0x375f), 0x02 }, { CCI_REG8(0x3760), 0x08 }, { CCI_REG8(0x3761), 0x10 },
	{ CCI_REG8(0x3762), 0x08 }, { CCI_REG8(0x3763), 0x08 }, { CCI_REG8(0x3764), 0x08 }, { CCI_REG8(0x3765), 0x10 }, { CCI_REG8(0x3766), 0x18 },
	{ CCI_REG8(0x3767), 0x10 }, { CCI_REG8(0x3768), 0x00 }, { CCI_REG8(0x3769), 0x08 }, { CCI_REG8(0x376a), 0x10 }, { CCI_REG8(0x376b), 0x00 },
	{ CCI_REG8(0x376c), 0x28 }, { CCI_REG8(0x3770), 0x19 }, { CCI_REG8(0x3771), 0x01 }, { CCI_REG8(0x3772), 0x01 }, { CCI_REG8(0x3773), 0x01 },
	{ CCI_REG8(0x3774), 0x1a }, { CCI_REG8(0x3775), 0x01 }, { CCI_REG8(0x3776), 0x01 }, { CCI_REG8(0x3777), 0x01 }, { CCI_REG8(0x3780), 0xc8 },
	{ CCI_REG8(0x3791), 0x38 }, { CCI_REG8(0x3793), 0x3a }, { CCI_REG8(0x3795), 0x3a }, { CCI_REG8(0x3797), 0x80 }, { CCI_REG8(0x379c), 0x14 },
	{ CCI_REG8(0x379d), 0x14 }, { CCI_REG8(0x3799), 0x82 }, { CCI_REG8(0x379b), 0x82 }, { CCI_REG8(0x37a0), 0x07 }, { CCI_REG8(0x37a1), 0x99 },
	{ CCI_REG8(0x37a2), 0x11 }, { CCI_REG8(0x37af), 0x00 }, { CCI_REG8(0x37f7), 0x00 }, { CCI_REG8(0x37f8), 0x5a }, { CCI_REG8(0x37ff), 0x18 },
	{ CCI_REG8(0x3800), 0x00 }, { CCI_REG8(0x3801), 0x00 }, { CCI_REG8(0x3802), 0x00 }, { CCI_REG8(0x3803), 0x00 }, { CCI_REG8(0x3804), 0x24 },
	{ CCI_REG8(0x3805), 0x3f }, { CCI_REG8(0x3806), 0x1b }, { CCI_REG8(0x3807), 0x3f }, { CCI_REG8(0x3808), 0x12 }, { CCI_REG8(0x3809), 0x10 },
	{ CCI_REG8(0x380a), 0x0d }, { CCI_REG8(0x380b), 0x90 }, { CCI_REG8(0x380c), 0x03 }, { CCI_REG8(0x380d), 0xa8 }, { CCI_REG8(0x380e), 0x10 },
	{ CCI_REG8(0x380f), 0x0c }, { CCI_REG8(0x3810), 0x00 }, { CCI_REG8(0x3811), 0x09 }, { CCI_REG8(0x3812), 0x00 }, { CCI_REG8(0x3813), 0x08 },
	{ CCI_REG8(0x3814), 0x11 }, { CCI_REG8(0x3815), 0x11 }, { CCI_REG8(0x3820), 0x42 }, { CCI_REG8(0x3821), 0x04 }, { CCI_REG8(0x3822), 0x10 },
	{ CCI_REG8(0x3823), 0x04 }, { CCI_REG8(0x3828), 0x0f }, { CCI_REG8(0x382a), 0x80 }, { CCI_REG8(0x382e), 0x41 }, { CCI_REG8(0x3830), 0x0b },
	{ CCI_REG8(0x3837), 0x0c }, { CCI_REG8(0x383e), 0x00 }, { CCI_REG8(0x383f), 0x0c }, { CCI_REG8(0x3840), 0x00 }, { CCI_REG8(0x3847), 0x00 },
	{ CCI_REG8(0x384a), 0x00 }, { CCI_REG8(0x384c), 0x03 }, { CCI_REG8(0x384d), 0xa8 }, { CCI_REG8(0x3856), 0x00 }, { CCI_REG8(0x3858), 0x00 },
	{ CCI_REG8(0x3859), 0x00 }, { CCI_REG8(0x3888), 0x00 }, { CCI_REG8(0x3889), 0x60 }, { CCI_REG8(0x388a), 0x00 }, { CCI_REG8(0x388b), 0x18 },
	{ CCI_REG8(0x388c), 0x23 }, { CCI_REG8(0x388d), 0x80 }, { CCI_REG8(0x388e), 0x0d }, { CCI_REG8(0x388f), 0x70 }, { CCI_REG8(0x3894), 0x00 },
	{ CCI_REG8(0x3900), 0x0c }, { CCI_REG8(0x3901), 0x06 }, { CCI_REG8(0x3902), 0xdf }, { CCI_REG8(0x3903), 0xfb }, { CCI_REG8(0x3904), 0xff },
	{ CCI_REG8(0x3906), 0x00 }, { CCI_REG8(0x3907), 0x00 }, { CCI_REG8(0x3908), 0x57 }, { CCI_REG8(0x3909), 0xde }, { CCI_REG8(0x390c), 0x3f },
	{ CCI_REG8(0x390e), 0x3d }, { CCI_REG8(0x390f), 0x54 }, { CCI_REG8(0x3911), 0x3c }, { CCI_REG8(0x3912), 0x03 }, { CCI_REG8(0x3913), 0x41 },
	{ CCI_REG8(0x3914), 0x40 }, { CCI_REG8(0x3919), 0x26 }, { CCI_REG8(0x391b), 0xa2 }, { CCI_REG8(0x391c), 0x69 }, { CCI_REG8(0x3920), 0x88 },
	{ CCI_REG8(0x3921), 0x00 }, { CCI_REG8(0x3924), 0x84 }, { CCI_REG8(0x3925), 0x0f }, { CCI_REG8(0x3926), 0xfe }, { CCI_REG8(0x3930), 0x03 },
	{ CCI_REG8(0x3932), 0x03 }, { CCI_REG8(0x3933), 0x01 }, { CCI_REG8(0x3935), 0x01 }, { CCI_REG8(0x393a), 0x03 }, { CCI_REG8(0x393c), 0x03 },
	{ CCI_REG8(0x393d), 0x01 }, { CCI_REG8(0x393f), 0x01 }, { CCI_REG8(0x3945), 0x00 }, { CCI_REG8(0x3946), 0x00 }, { CCI_REG8(0x3949), 0x01 },
	{ CCI_REG8(0x394b), 0x00 }, { CCI_REG8(0x394c), 0x16 }, { CCI_REG8(0x3953), 0x0c }, { CCI_REG8(0x3954), 0x0c }, { CCI_REG8(0x3955), 0x00 },
	{ CCI_REG8(0x3956), 0x00 }, { CCI_REG8(0x3968), 0x00 }, { CCI_REG8(0x396a), 0x00 }, { CCI_REG8(0x3973), 0x02 }, { CCI_REG8(0x3977), 0x04 },
	{ CCI_REG8(0x3978), 0x02 }, { CCI_REG8(0x3979), 0x04 }, { CCI_REG8(0x397d), 0x04 }, { CCI_REG8(0x3981), 0x04 }, { CCI_REG8(0x3983), 0x04 },
	{ CCI_REG8(0x3987), 0x04 }, { CCI_REG8(0x3989), 0x04 }, { CCI_REG8(0x3997), 0x00 }, { CCI_REG8(0x399b), 0x4b }, { CCI_REG8(0x399c), 0x00 },
	{ CCI_REG8(0x39a2), 0x02 }, { CCI_REG8(0x39af), 0x01 }, { CCI_REG8(0x39b7), 0x01 }, { CCI_REG8(0x39bf), 0x01 }, { CCI_REG8(0x39cb), 0x00 },
	{ CCI_REG8(0x39cd), 0x18 }, { CCI_REG8(0x39ce), 0x48 }, { CCI_REG8(0x39cf), 0x48 }, { CCI_REG8(0x39d0), 0x48 }, { CCI_REG8(0x39d2), 0x1b },
	{ CCI_REG8(0x39d3), 0x4b }, { CCI_REG8(0x39d4), 0x4b }, { CCI_REG8(0x39d5), 0x4b }, { CCI_REG8(0x39d7), 0x01 }, { CCI_REG8(0x39dc), 0x00 },
	{ CCI_REG8(0x39e2), 0x20 }, { CCI_REG8(0x39e4), 0x20 }, { CCI_REG8(0x39e6), 0x20 }, { CCI_REG8(0x39e8), 0x20 }, { CCI_REG8(0x39eb), 0x01 },
	{ CCI_REG8(0x39fd), 0x01 }, { CCI_REG8(0x39f5), 0x04 }, { CCI_REG8(0x39f6), 0x1a }, { CCI_REG8(0x39f7), 0x00 }, { CCI_REG8(0x39f8), 0x00 },
	{ CCI_REG8(0x39f9), 0x00 }, { CCI_REG8(0x39ff), 0x00 }, { CCI_REG8(0x3a01), 0x03 }, { CCI_REG8(0x3a03), 0x00 }, { CCI_REG8(0x3a05), 0x05 },
	{ CCI_REG8(0x3a07), 0x07 }, { CCI_REG8(0x3a17), 0x04 }, { CCI_REG8(0x3a2d), 0x00 }, { CCI_REG8(0x3a2e), 0x00 }, { CCI_REG8(0x3a2f), 0x20 },
	{ CCI_REG8(0x3a30), 0x20 }, { CCI_REG8(0x3a31), 0x20 }, { CCI_REG8(0x3a32), 0x20 }, { CCI_REG8(0x3a33), 0x20 }, { CCI_REG8(0x3a34), 0x20 },
	{ CCI_REG8(0x3a35), 0x55 }, { CCI_REG8(0x3a43), 0x1e }, { CCI_REG8(0x3a44), 0x0f }, { CCI_REG8(0x3a45), 0x07 }, { CCI_REG8(0x3a46), 0x03 },
	{ CCI_REG8(0x3a4b), 0x08 }, { CCI_REG8(0x3a4c), 0x08 }, { CCI_REG8(0x3a4d), 0x08 }, { CCI_REG8(0x3a4e), 0x08 }, { CCI_REG8(0x3a4f), 0x08 },
	{ CCI_REG8(0x3a50), 0x08 }, { CCI_REG8(0x3a51), 0x08 }, { CCI_REG8(0x3a52), 0x08 }, { CCI_REG8(0x3a53), 0x88 }, { CCI_REG8(0x3a54), 0x88 },
	{ CCI_REG8(0x3a55), 0x88 }, { CCI_REG8(0x3a56), 0x88 }, { CCI_REG8(0x3a57), 0x15 }, { CCI_REG8(0x3a58), 0x00 }, { CCI_REG8(0x3a59), 0x00 },
	{ CCI_REG8(0x3a5a), 0x01 }, { CCI_REG8(0x3a5f), 0x3f }, { CCI_REG8(0x3a60), 0x3f }, { CCI_REG8(0x3a61), 0x3f }, { CCI_REG8(0x3a62), 0x3f },
	{ CCI_REG8(0x3a63), 0x3f }, { CCI_REG8(0x3a64), 0x3f }, { CCI_REG8(0x3a65), 0x3f }, { CCI_REG8(0x3a66), 0x3f }, { CCI_REG8(0x3a67), 0x1f },
	{ CCI_REG8(0x3a68), 0x1f }, { CCI_REG8(0x3a69), 0x1f }, { CCI_REG8(0x3a6a), 0x1f }, { CCI_REG8(0x3a6b), 0x1f }, { CCI_REG8(0x3a6c), 0x1f },
	{ CCI_REG8(0x3a6d), 0x1f }, { CCI_REG8(0x3a6e), 0x1f }, { CCI_REG8(0x3a73), 0x13 }, { CCI_REG8(0x3a74), 0x10 }, { CCI_REG8(0x3a75), 0x0b },
	{ CCI_REG8(0x3a76), 0x00 }, { CCI_REG8(0x3a77), 0x13 }, { CCI_REG8(0x3a78), 0x10 }, { CCI_REG8(0x3a79), 0x0b }, { CCI_REG8(0x3a7a), 0x00 },
	{ CCI_REG8(0x3a7d), 0x60 }, { CCI_REG8(0x3d85), 0x0b }, { CCI_REG8(0x3d86), 0x12 }, { CCI_REG8(0x3d87), 0x28 }, { CCI_REG8(0x3d8c), 0x07 },
	{ CCI_REG8(0x3d8d), 0xb0 }, { CCI_REG8(0x3daa), 0x70 }, { CCI_REG8(0x3dab), 0x10 }, { CCI_REG8(0x3dac), 0x76 }, { CCI_REG8(0x3dad), 0xa0 },
	{ CCI_REG8(0x3dae), 0x77 }, { CCI_REG8(0x3daf), 0x1f }, { CCI_REG8(0x3f00), 0x10 }, { CCI_REG8(0x4009), 0x02 }, { CCI_REG8(0x400e), 0xc6 },
	{ CCI_REG8(0x400f), 0x00 }, { CCI_REG8(0x4010), 0xe8 }, { CCI_REG8(0x4011), 0x01 }, { CCI_REG8(0x4012), 0x0c }, { CCI_REG8(0x4015), 0x00 },
	{ CCI_REG8(0x4016), 0x0f }, { CCI_REG8(0x4017), 0x00 }, { CCI_REG8(0x4018), 0x07 }, { CCI_REG8(0x401a), 0x40 }, { CCI_REG8(0x401b), 0x04 },
	{ CCI_REG8(0x401e), 0x01 }, { CCI_REG8(0x401f), 0x00 }, { CCI_REG8(0x4020), 0x04 }, { CCI_REG8(0x4021), 0x00 }, { CCI_REG8(0x4022), 0x04 },
	{ CCI_REG8(0x4023), 0x00 }, { CCI_REG8(0x4024), 0x04 }, { CCI_REG8(0x4025), 0x00 }, { CCI_REG8(0x4026), 0x04 }, { CCI_REG8(0x4027), 0x00 },
	{ CCI_REG8(0x4028), 0x01 }, { CCI_REG8(0x4029), 0x08 }, { CCI_REG8(0x40c2), 0x12 }, { CCI_REG8(0x40c3), 0x28 }, { CCI_REG8(0x4288), 0xa7 },
	{ CCI_REG8(0x4289), 0x20 }, { CCI_REG8(0x428c), 0x00 }, { CCI_REG8(0x428d), 0x80 }, { CCI_REG8(0x428e), 0x04 }, { CCI_REG8(0x4500), 0x08 },
	{ CCI_REG8(0x4502), 0x00 }, { CCI_REG8(0x4504), 0x80 }, { CCI_REG8(0x4506), 0x01 }, { CCI_REG8(0x4509), 0x05 }, { CCI_REG8(0x450c), 0x00 },
	{ CCI_REG8(0x450d), 0x20 }, { CCI_REG8(0x450e), 0x00 }, { CCI_REG8(0x450f), 0x00 }, { CCI_REG8(0x4510), 0x00 }, { CCI_REG8(0x4523), 0x00 },
	{ CCI_REG8(0x4526), 0x00 }, { CCI_REG8(0x4540), 0x12 }, { CCI_REG8(0x4541), 0x28 }, { CCI_REG8(0x4542), 0x00 }, { CCI_REG8(0x4543), 0x00 },
	{ CCI_REG8(0x4544), 0x00 }, { CCI_REG8(0x4545), 0x00 }, { CCI_REG8(0x4546), 0x00 }, { CCI_REG8(0x4547), 0x10 }, { CCI_REG8(0x4580), 0x01 },
	{ CCI_REG8(0x4589), 0x12 }, { CCI_REG8(0x458a), 0x28 }, { CCI_REG8(0x45c0), 0x61 }, { CCI_REG8(0x45c9), 0x12 }, { CCI_REG8(0x45ca), 0x28 },
	{ CCI_REG8(0x45cb), 0x30 }, { CCI_REG8(0x4602), 0x00 }, { CCI_REG8(0x4603), 0x15 }, { CCI_REG8(0x4606), 0x12 }, { CCI_REG8(0x4607), 0x28 },
	{ CCI_REG8(0x4609), 0x20 }, { CCI_REG8(0x460b), 0x03 }, { CCI_REG8(0x4640), 0x00 }, { CCI_REG8(0x4641), 0x47 }, { CCI_REG8(0x4643), 0x0c },
	{ CCI_REG8(0x4644), 0x40 }, { CCI_REG8(0x4645), 0xb3 }, { CCI_REG8(0x4648), 0x12 }, { CCI_REG8(0x4649), 0x28 }, { CCI_REG8(0x464c), 0x01 },
	{ CCI_REG8(0x464a), 0x00 }, { CCI_REG8(0x464b), 0x30 }, { CCI_REG8(0x4800), 0x64 }, { CCI_REG8(0x4802), 0x02 }, { CCI_REG8(0x480b), 0x10 },
	{ CCI_REG8(0x480c), 0x80 }, { CCI_REG8(0x480e), 0x04 }, { CCI_REG8(0x480f), 0x32 }, { CCI_REG8(0x481b), 0x12 }, { CCI_REG8(0x481f), 0x26 },
	{ CCI_REG8(0x4833), 0x20 }, { CCI_REG8(0x4837), 0x08 }, { CCI_REG8(0x484b), 0x27 }, { CCI_REG8(0x4850), 0x42 }, { CCI_REG8(0x4853), 0x04 },
	{ CCI_REG8(0x4854), 0x08 }, { CCI_REG8(0x4860), 0x01 }, { CCI_REG8(0x4861), 0xef }, { CCI_REG8(0x4862), 0x18 }, { CCI_REG8(0x4883), 0x24 },
	{ CCI_REG8(0x4885), 0x30 }, { CCI_REG8(0x4888), 0x80 }, { CCI_REG8(0x4889), 0x01 }, { CCI_REG8(0x488a), 0x00 }, { CCI_REG8(0x4909), 0x02 },
	{ CCI_REG8(0x4910), 0xe8 }, { CCI_REG8(0x4911), 0x01 }, { CCI_REG8(0x4912), 0x0c }, { CCI_REG8(0x4915), 0x00 }, { CCI_REG8(0x4916), 0x0f },
	{ CCI_REG8(0x4917), 0x00 }, { CCI_REG8(0x4918), 0x07 }, { CCI_REG8(0x491a), 0x40 }, { CCI_REG8(0x491b), 0x04 }, { CCI_REG8(0x491e), 0x01 },
	{ CCI_REG8(0x491f), 0x00 }, { CCI_REG8(0x4920), 0x04 }, { CCI_REG8(0x4921), 0x00 }, { CCI_REG8(0x4922), 0x04 }, { CCI_REG8(0x4923), 0x00 },
	{ CCI_REG8(0x4924), 0x04 }, { CCI_REG8(0x4925), 0x00 }, { CCI_REG8(0x4926), 0x04 }, { CCI_REG8(0x4927), 0x00 }, { CCI_REG8(0x4929), 0x08 },
	{ CCI_REG8(0x49c2), 0x12 }, { CCI_REG8(0x49c3), 0x28 }, { CCI_REG8(0x4a09), 0x02 }, { CCI_REG8(0x4a10), 0xe8 }, { CCI_REG8(0x4a11), 0x01 },
	{ CCI_REG8(0x4a12), 0x0c }, { CCI_REG8(0x4a15), 0x00 }, { CCI_REG8(0x4a16), 0x0f }, { CCI_REG8(0x4a17), 0x00 }, { CCI_REG8(0x4a18), 0x07 },
	{ CCI_REG8(0x4a1a), 0x40 }, { CCI_REG8(0x4a1b), 0x04 }, { CCI_REG8(0x4a1e), 0x01 }, { CCI_REG8(0x4a1f), 0x00 }, { CCI_REG8(0x4a20), 0x04 },
	{ CCI_REG8(0x4a21), 0x00 }, { CCI_REG8(0x4a22), 0x04 }, { CCI_REG8(0x4a23), 0x00 }, { CCI_REG8(0x4a24), 0x04 }, { CCI_REG8(0x4a25), 0x00 },
	{ CCI_REG8(0x4a26), 0x04 }, { CCI_REG8(0x4a27), 0x00 }, { CCI_REG8(0x4a29), 0x08 }, { CCI_REG8(0x4ac2), 0x12 }, { CCI_REG8(0x4ac3), 0x28 },
	{ CCI_REG8(0x4d00), 0x04 }, { CCI_REG8(0x4d01), 0xf5 }, { CCI_REG8(0x4d02), 0xb8 }, { CCI_REG8(0x4d03), 0x76 }, { CCI_REG8(0x4d04), 0x04 },
	{ CCI_REG8(0x4d05), 0xcc }, { CCI_REG8(0x5000), 0xf7 }, { CCI_REG8(0x5001), 0x01 }, { CCI_REG8(0x5002), 0xf7 }, { CCI_REG8(0x5003), 0x00 },
	{ CCI_REG8(0x5004), 0x80 }, { CCI_REG8(0x5005), 0x42 }, { CCI_REG8(0x5006), 0x04 }, { CCI_REG8(0x5007), 0x02 }, { CCI_REG8(0x5015), 0x20 },
	{ CCI_REG8(0x5053), 0x04 }, { CCI_REG8(0x5060), 0x00 }, { CCI_REG8(0x5061), 0x30 }, { CCI_REG8(0x5062), 0x00 }, { CCI_REG8(0x5063), 0x30 },
	{ CCI_REG8(0x5064), 0x23 }, { CCI_REG8(0x5065), 0xe0 }, { CCI_REG8(0x5066), 0x1a }, { CCI_REG8(0x5067), 0xe0 }, { CCI_REG8(0x5068), 0x02 },
	{ CCI_REG8(0x5069), 0x10 }, { CCI_REG8(0x506a), 0x10 }, { CCI_REG8(0x506b), 0x04 }, { CCI_REG8(0x506c), 0x04 }, { CCI_REG8(0x506d), 0x0c },
	{ CCI_REG8(0x506e), 0x0c }, { CCI_REG8(0x506f), 0x04 }, { CCI_REG8(0x5070), 0x04 }, { CCI_REG8(0x5071), 0x0c }, { CCI_REG8(0x5072), 0x0c },
	{ CCI_REG8(0x5073), 0x01 }, { CCI_REG8(0x5074), 0x01 }, { CCI_REG8(0x5075), 0xaa }, { CCI_REG8(0x5083), 0x00 }, { CCI_REG8(0x50c1), 0x00 },
	{ CCI_REG8(0x5110), 0x70 }, { CCI_REG8(0x5111), 0x10 }, { CCI_REG8(0x5112), 0x77 }, { CCI_REG8(0x5113), 0x1f }, { CCI_REG8(0x5114), 0x01 },
	{ CCI_REG8(0x5180), 0xc1 }, { CCI_REG8(0x518a), 0x04 }, { CCI_REG8(0x51b0), 0x00 }, { CCI_REG8(0x51d0), 0xc2 }, { CCI_REG8(0x51d1), 0x68 },
	{ CCI_REG8(0x51d2), 0xff }, { CCI_REG8(0x51d3), 0x1c }, { CCI_REG8(0x51d8), 0x08 }, { CCI_REG8(0x51d9), 0x10 }, { CCI_REG8(0x51da), 0x02 },
	{ CCI_REG8(0x51db), 0x02 }, { CCI_REG8(0x51dc), 0x06 }, { CCI_REG8(0x51dd), 0x06 }, { CCI_REG8(0x51de), 0x02 }, { CCI_REG8(0x51df), 0x06 },
	{ CCI_REG8(0x51e0), 0x0a }, { CCI_REG8(0x51e1), 0x0e }, { CCI_REG8(0x51e2), 0x00 }, { CCI_REG8(0x51e3), 0x00 }, { CCI_REG8(0x51ee), 0x01 },
	{ CCI_REG8(0x51ef), 0x02 }, { CCI_REG8(0x51f0), 0x04 }, { CCI_REG8(0x51f1), 0xaa }, { CCI_REG8(0x5200), 0x12 }, { CCI_REG8(0x5201), 0x20 },
	{ CCI_REG8(0x5202), 0x0d }, { CCI_REG8(0x5203), 0xa0 }, { CCI_REG8(0x5205), 0x10 }, { CCI_REG8(0x5207), 0x10 }, { CCI_REG8(0x5208), 0x12 },
	{ CCI_REG8(0x5209), 0x00 }, { CCI_REG8(0x520a), 0x0d }, { CCI_REG8(0x520b), 0x80 }, { CCI_REG8(0x520d), 0x08 }, { CCI_REG8(0x5250), 0x06 },
	{ CCI_REG8(0x5251), 0x00 }, { CCI_REG8(0x525b), 0x00 }, { CCI_REG8(0x525d), 0x00 }, { CCI_REG8(0x527a), 0x00 }, { CCI_REG8(0x527b), 0x38 },
	{ CCI_REG8(0x527c), 0x00 }, { CCI_REG8(0x527d), 0x4b }, { CCI_REG8(0x5286), 0x1b }, { CCI_REG8(0x5287), 0x40 }, { CCI_REG8(0x5298), 0x00 },
	{ CCI_REG8(0x5299), 0x50 }, { CCI_REG8(0x529a), 0x00 }, { CCI_REG8(0x529b), 0x50 }, { CCI_REG8(0x529c), 0x00 }, { CCI_REG8(0x529d), 0x50 },
	{ CCI_REG8(0x529e), 0x00 }, { CCI_REG8(0x529f), 0x50 }, { CCI_REG8(0x52a0), 0x00 }, { CCI_REG8(0x52a1), 0x50 }, { CCI_REG8(0x52a2), 0x00 },
	{ CCI_REG8(0x52a3), 0x50 }, { CCI_REG8(0x52a4), 0x00 }, { CCI_REG8(0x52a5), 0x50 }, { CCI_REG8(0x52a6), 0x00 }, { CCI_REG8(0x52a7), 0x50 },
	{ CCI_REG8(0x52a8), 0x00 }, { CCI_REG8(0x52a9), 0x50 }, { CCI_REG8(0x52aa), 0x00 }, { CCI_REG8(0x52ab), 0x50 }, { CCI_REG8(0x52ac), 0x00 },
	{ CCI_REG8(0x52ad), 0x50 }, { CCI_REG8(0x52ae), 0x00 }, { CCI_REG8(0x52af), 0x50 }, { CCI_REG8(0x52d0), 0x20 }, { CCI_REG8(0x52d1), 0x20 },
	{ CCI_REG8(0x52d2), 0x20 }, { CCI_REG8(0x52d3), 0x20 }, { CCI_REG8(0x52d4), 0x20 }, { CCI_REG8(0x52d5), 0x20 }, { CCI_REG8(0x52d6), 0x20 },
	{ CCI_REG8(0x52d7), 0x20 }, { CCI_REG8(0x52d8), 0x20 }, { CCI_REG8(0x52d9), 0x20 }, { CCI_REG8(0x52da), 0x20 }, { CCI_REG8(0x52db), 0x20 },
	{ CCI_REG8(0x52dc), 0x20 }, { CCI_REG8(0x52dd), 0x20 }, { CCI_REG8(0x52de), 0x20 }, { CCI_REG8(0x52df), 0x20 }, { CCI_REG8(0x52f0), 0x08 },
	{ CCI_REG8(0x52f1), 0x07 }, { CCI_REG8(0x52f2), 0x06 }, { CCI_REG8(0x52f3), 0x05 }, { CCI_REG8(0x52f4), 0x0a }, { CCI_REG8(0x52f5), 0x02 },
	{ CCI_REG8(0x52f6), 0x01 }, { CCI_REG8(0x52f7), 0x09 }, { CCI_REG8(0x52f8), 0x0c }, { CCI_REG8(0x52f9), 0x04 }, { CCI_REG8(0x52fa), 0x03 },
	{ CCI_REG8(0x52fb), 0x0b }, { CCI_REG8(0x52fc), 0x10 }, { CCI_REG8(0x52fd), 0x0f }, { CCI_REG8(0x52fe), 0x0e }, { CCI_REG8(0x52ff), 0x0d },
	{ CCI_REG8(0x5300), 0x00 }, { CCI_REG8(0x5301), 0x00 }, { CCI_REG8(0x5302), 0x00 }, { CCI_REG8(0x5303), 0x00 }, { CCI_REG8(0x5304), 0x00 },
	{ CCI_REG8(0x5305), 0x00 }, { CCI_REG8(0x5306), 0x00 }, { CCI_REG8(0x5307), 0x00 }, { CCI_REG8(0x5308), 0x00 }, { CCI_REG8(0x5309), 0x00 },
	{ CCI_REG8(0x530a), 0x00 }, { CCI_REG8(0x530b), 0x00 }, { CCI_REG8(0x530c), 0x00 }, { CCI_REG8(0x530d), 0x00 }, { CCI_REG8(0x530e), 0x00 },
	{ CCI_REG8(0x530f), 0x00 }, { CCI_REG8(0x5310), 0x03 }, { CCI_REG8(0x5311), 0xe8 }, { CCI_REG8(0x5312), 0x00 }, { CCI_REG8(0x5331), 0x02 },
	{ CCI_REG8(0x5332), 0x42 }, { CCI_REG8(0x5333), 0x24 }, { CCI_REG8(0x5353), 0x09 }, { CCI_REG8(0x5354), 0x00 }, { CCI_REG8(0x53c1), 0x00 },
	{ CCI_REG8(0x5414), 0x01 }, { CCI_REG8(0x5480), 0xc1 }, { CCI_REG8(0x548a), 0x04 }, { CCI_REG8(0x54b0), 0x10 }, { CCI_REG8(0x54d1), 0x68 },
	{ CCI_REG8(0x54d2), 0xff }, { CCI_REG8(0x54d3), 0x1c }, { CCI_REG8(0x54ee), 0x01 }, { CCI_REG8(0x54ef), 0x02 }, { CCI_REG8(0x54f0), 0x04 },
	{ CCI_REG8(0x5510), 0x03 }, { CCI_REG8(0x5511), 0xe8 }, { CCI_REG8(0x5550), 0x06 }, { CCI_REG8(0x557a), 0x00 }, { CCI_REG8(0x557b), 0x38 },
	{ CCI_REG8(0x557c), 0x00 }, { CCI_REG8(0x557d), 0x4b }, { CCI_REG8(0x5598), 0x00 }, { CCI_REG8(0x5599), 0x50 }, { CCI_REG8(0x559a), 0x00 },
	{ CCI_REG8(0x559b), 0x50 }, { CCI_REG8(0x559c), 0x00 }, { CCI_REG8(0x559d), 0x50 }, { CCI_REG8(0x559e), 0x00 }, { CCI_REG8(0x559f), 0x50 },
	{ CCI_REG8(0x55a0), 0x00 }, { CCI_REG8(0x55a1), 0x50 }, { CCI_REG8(0x55a2), 0x00 }, { CCI_REG8(0x55a3), 0x50 }, { CCI_REG8(0x55a4), 0x00 },
	{ CCI_REG8(0x55a5), 0x50 }, { CCI_REG8(0x55a6), 0x00 }, { CCI_REG8(0x55a7), 0x50 }, { CCI_REG8(0x55a8), 0x00 }, { CCI_REG8(0x55a9), 0x50 },
	{ CCI_REG8(0x55aa), 0x00 }, { CCI_REG8(0x55ab), 0x50 }, { CCI_REG8(0x55ac), 0x00 }, { CCI_REG8(0x55ad), 0x50 }, { CCI_REG8(0x55ae), 0x00 },
	{ CCI_REG8(0x55af), 0x50 }, { CCI_REG8(0x55d0), 0x20 }, { CCI_REG8(0x55d1), 0x20 }, { CCI_REG8(0x55d2), 0x20 }, { CCI_REG8(0x55d3), 0x20 },
	{ CCI_REG8(0x55d4), 0x20 }, { CCI_REG8(0x55d5), 0x20 }, { CCI_REG8(0x55d6), 0x20 }, { CCI_REG8(0x55d7), 0x20 }, { CCI_REG8(0x55d8), 0x20 },
	{ CCI_REG8(0x55d9), 0x20 }, { CCI_REG8(0x55da), 0x20 }, { CCI_REG8(0x55db), 0x20 }, { CCI_REG8(0x55dc), 0x20 }, { CCI_REG8(0x55dd), 0x20 },
	{ CCI_REG8(0x55de), 0x20 }, { CCI_REG8(0x55df), 0x20 }, { CCI_REG8(0x55f0), 0x01 }, { CCI_REG8(0x55f1), 0x02 }, { CCI_REG8(0x55f2), 0x03 },
	{ CCI_REG8(0x55f3), 0x04 }, { CCI_REG8(0x55f4), 0x05 }, { CCI_REG8(0x55f5), 0x06 }, { CCI_REG8(0x55f6), 0x07 }, { CCI_REG8(0x55f7), 0x08 },
	{ CCI_REG8(0x55f8), 0x09 }, { CCI_REG8(0x55f9), 0x0a }, { CCI_REG8(0x55fa), 0x0b }, { CCI_REG8(0x55fb), 0x0c }, { CCI_REG8(0x55fc), 0x0d },
	{ CCI_REG8(0x55fd), 0x0e }, { CCI_REG8(0x55fe), 0x0f }, { CCI_REG8(0x55ff), 0x10 }, { CCI_REG8(0x5600), 0x11 }, { CCI_REG8(0x5601), 0x12 },
	{ CCI_REG8(0x5602), 0x13 }, { CCI_REG8(0x5603), 0x14 }, { CCI_REG8(0x5604), 0x15 }, { CCI_REG8(0x5605), 0x16 }, { CCI_REG8(0x5606), 0x17 },
	{ CCI_REG8(0x5607), 0x18 }, { CCI_REG8(0x5608), 0x19 }, { CCI_REG8(0x5609), 0x1a }, { CCI_REG8(0x560a), 0x1b }, { CCI_REG8(0x560b), 0x1c },
	{ CCI_REG8(0x560c), 0x1d }, { CCI_REG8(0x560d), 0x1e }, { CCI_REG8(0x560e), 0x1f }, { CCI_REG8(0x560f), 0x20 }, { CCI_REG8(0x5631), 0x02 },
	{ CCI_REG8(0x5632), 0x42 }, { CCI_REG8(0x5633), 0x24 }, { CCI_REG8(0x5653), 0x09 }, { CCI_REG8(0x5654), 0x00 }, { CCI_REG8(0x56c1), 0x00 },
	{ CCI_REG8(0x5714), 0x01 }, { CCI_REG8(0x5780), 0xc1 }, { CCI_REG8(0x578a), 0x04 }, { CCI_REG8(0x57b0), 0x10 }, { CCI_REG8(0x57d1), 0x68 },
	{ CCI_REG8(0x57d2), 0xff }, { CCI_REG8(0x57d3), 0x1c }, { CCI_REG8(0x57ee), 0x01 }, { CCI_REG8(0x57ef), 0x02 }, { CCI_REG8(0x57f0), 0x04 },
	{ CCI_REG8(0x5810), 0x03 }, { CCI_REG8(0x5811), 0xe8 }, { CCI_REG8(0x5850), 0x06 }, { CCI_REG8(0x587a), 0x00 }, { CCI_REG8(0x587b), 0x38 },
	{ CCI_REG8(0x587c), 0x00 }, { CCI_REG8(0x587d), 0x4b }, { CCI_REG8(0x5898), 0x00 }, { CCI_REG8(0x5899), 0x50 }, { CCI_REG8(0x589a), 0x00 },
	{ CCI_REG8(0x589b), 0x50 }, { CCI_REG8(0x589c), 0x00 }, { CCI_REG8(0x589d), 0x50 }, { CCI_REG8(0x589e), 0x00 }, { CCI_REG8(0x589f), 0x50 },
	{ CCI_REG8(0x58a0), 0x00 }, { CCI_REG8(0x58a1), 0x50 }, { CCI_REG8(0x58a2), 0x00 }, { CCI_REG8(0x58a3), 0x50 }, { CCI_REG8(0x58a4), 0x00 },
	{ CCI_REG8(0x58a5), 0x50 }, { CCI_REG8(0x58a6), 0x00 }, { CCI_REG8(0x58a7), 0x50 }, { CCI_REG8(0x58a8), 0x00 }, { CCI_REG8(0x58a9), 0x50 },
	{ CCI_REG8(0x58aa), 0x00 }, { CCI_REG8(0x58ab), 0x50 }, { CCI_REG8(0x58ac), 0x00 }, { CCI_REG8(0x58ad), 0x50 }, { CCI_REG8(0x58ae), 0x00 },
	{ CCI_REG8(0x58af), 0x50 }, { CCI_REG8(0x58d0), 0x20 }, { CCI_REG8(0x58d1), 0x20 }, { CCI_REG8(0x58d2), 0x20 }, { CCI_REG8(0x58d3), 0x20 },
	{ CCI_REG8(0x58d4), 0x20 }, { CCI_REG8(0x58d5), 0x20 }, { CCI_REG8(0x58d6), 0x20 }, { CCI_REG8(0x58d7), 0x20 }, { CCI_REG8(0x58d8), 0x20 },
	{ CCI_REG8(0x58d9), 0x20 }, { CCI_REG8(0x58da), 0x20 }, { CCI_REG8(0x58db), 0x20 }, { CCI_REG8(0x58dc), 0x20 }, { CCI_REG8(0x58dd), 0x20 },
	{ CCI_REG8(0x58de), 0x20 }, { CCI_REG8(0x58df), 0x20 }, { CCI_REG8(0x58f0), 0x01 }, { CCI_REG8(0x58f1), 0x02 }, { CCI_REG8(0x58f2), 0x03 },
	{ CCI_REG8(0x58f3), 0x04 }, { CCI_REG8(0x58f4), 0x05 }, { CCI_REG8(0x58f5), 0x06 }, { CCI_REG8(0x58f6), 0x07 }, { CCI_REG8(0x58f7), 0x08 },
	{ CCI_REG8(0x58f8), 0x09 }, { CCI_REG8(0x58f9), 0x0a }, { CCI_REG8(0x58fa), 0x0b }, { CCI_REG8(0x58fb), 0x0c }, { CCI_REG8(0x58fc), 0x0d },
	{ CCI_REG8(0x58fd), 0x0e }, { CCI_REG8(0x58fe), 0x0f }, { CCI_REG8(0x58ff), 0x10 }, { CCI_REG8(0x5900), 0x11 }, { CCI_REG8(0x5901), 0x12 },
	{ CCI_REG8(0x5902), 0x13 }, { CCI_REG8(0x5903), 0x14 }, { CCI_REG8(0x5904), 0x15 }, { CCI_REG8(0x5905), 0x16 }, { CCI_REG8(0x5906), 0x17 },
	{ CCI_REG8(0x5907), 0x18 }, { CCI_REG8(0x5908), 0x19 }, { CCI_REG8(0x5909), 0x1a }, { CCI_REG8(0x590a), 0x1b }, { CCI_REG8(0x590b), 0x1c },
	{ CCI_REG8(0x590c), 0x1d }, { CCI_REG8(0x590d), 0x1e }, { CCI_REG8(0x590e), 0x1f }, { CCI_REG8(0x590f), 0x20 }, { CCI_REG8(0x5931), 0x02 },
	{ CCI_REG8(0x5932), 0x42 }, { CCI_REG8(0x5933), 0x24 }, { CCI_REG8(0x5953), 0x09 }, { CCI_REG8(0x5954), 0x00 }, { CCI_REG8(0x5980), 0x3a },
	{ CCI_REG8(0x5989), 0x84 }, { CCI_REG8(0x59c3), 0x04 }, { CCI_REG8(0x59c4), 0x24 }, { CCI_REG8(0x59c5), 0x40 }, { CCI_REG8(0x59c6), 0x1b },
	{ CCI_REG8(0x59c7), 0x40 }, { CCI_REG8(0x5a02), 0x0f }, { CCI_REG8(0x5f00), 0x29 }, { CCI_REG8(0x5f02), 0x04 }, { CCI_REG8(0x5f2d), 0x28 },
	{ CCI_REG8(0x5f2e), 0x28 }, { CCI_REG8(0x6524), 0x00 }, { CCI_REG8(0x6525), 0x00 },};

/* 160 writes: the vendor's capture settings for 4624x3472 */
static const struct cci_reg_sequence ov64b40_mode_4624x3472_regs[] = {
	{ CCI_REG8(0x325), 0x80 }, { CCI_REG8(0x326), 0xc3 }, { CCI_REG8(0x327), 0x03 }, { CCI_REG8(0x32f), 0xe1 }, { CCI_REG8(0x344), 0x01 },
	{ CCI_REG8(0x345), 0x09 }, { CCI_REG8(0x3501), 0x0f }, { CCI_REG8(0x3502), 0xec }, { CCI_REG8(0x3608), 0x8a }, { CCI_REG8(0x360b), 0x50 },
	{ CCI_REG8(0x360d), 0x05 }, { CCI_REG8(0x3622), 0x0a }, { CCI_REG8(0x3623), 0x08 }, { CCI_REG8(0x3659), 0x48 }, { CCI_REG8(0x3684), 0x87 },
	{ CCI_REG8(0x3701), 0x40 }, { CCI_REG8(0x3702), 0x4b }, { CCI_REG8(0x3709), 0xf8 }, { CCI_REG8(0x3712), 0x50 }, { CCI_REG8(0x3714), 0x67 },
	{ CCI_REG8(0x3723), 0x00 }, { CCI_REG8(0x3724), 0x19 }, { CCI_REG8(0x3770), 0x19 }, { CCI_REG8(0x3774), 0x1a }, { CCI_REG8(0x3791), 0x38 },
	{ CCI_REG8(0x3797), 0x80 }, { CCI_REG8(0x379c), 0x14 }, { CCI_REG8(0x379d), 0x14 }, { CCI_REG8(0x3800), 0x00 }, { CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 }, { CCI_REG8(0x3803), 0x00 }, { CCI_REG8(0x3804), 0x24 }, { CCI_REG8(0x3805), 0x3f }, { CCI_REG8(0x3806), 0x1b },
	{ CCI_REG8(0x3807), 0x3f }, { CCI_REG8(0x3808), 0x12 }, { CCI_REG8(0x3809), 0x10 }, { CCI_REG8(0x380a), 0x0d }, { CCI_REG8(0x380b), 0x90 },
	{ CCI_REG8(0x380c), 0x03 }, { CCI_REG8(0x380d), 0xa8 }, { CCI_REG8(0x380e), 0x10 }, { CCI_REG8(0x380f), 0x06 }, { CCI_REG8(0x3811), 0x09 },
	{ CCI_REG8(0x3813), 0x08 }, { CCI_REG8(0x3815), 0x11 }, { CCI_REG8(0x3820), 0x42 }, { CCI_REG8(0x3821), 0x04 }, { CCI_REG8(0x3822), 0x10 },
	{ CCI_REG8(0x3830), 0x0b }, { CCI_REG8(0x3837), 0x0c }, { CCI_REG8(0x3889), 0x60 }, { CCI_REG8(0x388b), 0x18 }, { CCI_REG8(0x388c), 0x23 },
	{ CCI_REG8(0x388d), 0x80 }, { CCI_REG8(0x388e), 0x0d }, { CCI_REG8(0x388f), 0x70 }, { CCI_REG8(0x3909), 0xde }, { CCI_REG8(0x390f), 0x54 },
	{ CCI_REG8(0x3920), 0x88 }, { CCI_REG8(0x3921), 0x00 }, { CCI_REG8(0x3924), 0x84 }, { CCI_REG8(0x3925), 0x0f }, { CCI_REG8(0x3926), 0xfe },
	{ CCI_REG8(0x3973), 0x02 }, { CCI_REG8(0x3978), 0x02 }, { CCI_REG8(0x399b), 0x4b }, { CCI_REG8(0x39af), 0x01 }, { CCI_REG8(0x39b7), 0x01 },
	{ CCI_REG8(0x39bf), 0x01 }, { CCI_REG8(0x39cd), 0x18 }, { CCI_REG8(0x39ce), 0x48 }, { CCI_REG8(0x39cf), 0x48 }, { CCI_REG8(0x39d0), 0x48 },
	{ CCI_REG8(0x39d2), 0x1b }, { CCI_REG8(0x39d3), 0x4b }, { CCI_REG8(0x39d4), 0x4b }, { CCI_REG8(0x39d5), 0x4b }, { CCI_REG8(0x39fd), 0x01 },
	{ CCI_REG8(0x39f5), 0x04 }, { CCI_REG8(0x39f6), 0x1a }, { CCI_REG8(0x39f7), 0x00 }, { CCI_REG8(0x39f8), 0x00 }, { CCI_REG8(0x39f9), 0x00 },
	{ CCI_REG8(0x39ff), 0x00 }, { CCI_REG8(0x3a01), 0x03 }, { CCI_REG8(0x3a03), 0x00 }, { CCI_REG8(0x3a05), 0x05 }, { CCI_REG8(0x3a2d), 0x00 },
	{ CCI_REG8(0x3a2e), 0x00 }, { CCI_REG8(0x3a2f), 0x20 }, { CCI_REG8(0x3a30), 0x20 }, { CCI_REG8(0x3a4b), 0x08 }, { CCI_REG8(0x3a4c), 0x08 },
	{ CCI_REG8(0x3a4d), 0x08 }, { CCI_REG8(0x3a4e), 0x08 }, { CCI_REG8(0x3a4f), 0x08 }, { CCI_REG8(0x3a50), 0x08 }, { CCI_REG8(0x3a51), 0x08 },
	{ CCI_REG8(0x3a52), 0x08 }, { CCI_REG8(0x3a57), 0x15 }, { CCI_REG8(0x3a58), 0x00 }, { CCI_REG8(0x3a59), 0x00 }, { CCI_REG8(0x3a5a), 0x01 },
	{ CCI_REG8(0x3a73), 0x13 }, { CCI_REG8(0x3a74), 0x10 }, { CCI_REG8(0x3a75), 0x0b }, { CCI_REG8(0x3a76), 0x00 }, { CCI_REG8(0x3a77), 0x13 },
	{ CCI_REG8(0x3a78), 0x10 }, { CCI_REG8(0x3a79), 0x0b }, { CCI_REG8(0x3a7a), 0x00 }, { CCI_REG8(0x3a7d), 0x60 }, { CCI_REG8(0x4016), 0x0f },
	{ CCI_REG8(0x4018), 0x07 }, { CCI_REG8(0x401f), 0x00 }, { CCI_REG8(0x45c0), 0x61 }, { CCI_REG8(0x45cb), 0x30 }, { CCI_REG8(0x4641), 0x47 },
	{ CCI_REG8(0x4643), 0x0c }, { CCI_REG8(0x480e), 0x04 }, { CCI_REG8(0x4916), 0x0f }, { CCI_REG8(0x4918), 0x07 }, { CCI_REG8(0x491f), 0x00 },
	{ CCI_REG8(0x4a16), 0x0f }, { CCI_REG8(0x4a18), 0x07 }, { CCI_REG8(0x4a1f), 0x00 }, { CCI_REG8(0x5000), 0xf7 }, { CCI_REG8(0x5001), 0x01 },
	{ CCI_REG8(0x5002), 0xf7 }, { CCI_REG8(0x5068), 0x02 }, { CCI_REG8(0x51b0), 0x00 }, { CCI_REG8(0x51d2), 0xff }, { CCI_REG8(0x51d9), 0x10 },
	{ CCI_REG8(0x51de), 0x02 }, { CCI_REG8(0x51df), 0x06 }, { CCI_REG8(0x51e0), 0x0a }, { CCI_REG8(0x51e1), 0x0e }, { CCI_REG8(0x5202), 0x0d },
	{ CCI_REG8(0x5203), 0xa0 }, { CCI_REG8(0x5205), 0x10 }, { CCI_REG8(0x5207), 0x10 }, { CCI_REG8(0x5208), 0x12 }, { CCI_REG8(0x5209), 0x00 },
	{ CCI_REG8(0x520a), 0x0d }, { CCI_REG8(0x520b), 0x80 }, { CCI_REG8(0x520d), 0x08 }, { CCI_REG8(0x5250), 0x06 }, { CCI_REG8(0x5331), 0x02 },
	{ CCI_REG8(0x5332), 0x42 }, { CCI_REG8(0x5333), 0x24 }, { CCI_REG8(0x54d2), 0xff }, { CCI_REG8(0x5550), 0x06 }, { CCI_REG8(0x57d2), 0xff },
	{ CCI_REG8(0x5850), 0x06 }, { CCI_REG8(0x5180), 0xc1 }, { CCI_REG8(0x5480), 0xc1 }, { CCI_REG8(0x5780), 0xc1 }, { CCI_REG8(0x3046), 0x01 },};

/*
 * 4624x3472 at 15 fps, from the vendor's own timing: pixel clock 115.2 MHz,
 * line length 936, frame length 4102.
 */
static const struct ov64b40_mode ov64b40_modes[] = {
	{
		.width = OV64B40_NATIVE_WIDTH,
		.height = OV64B40_NATIVE_HEIGHT,
		.hts = 936,
		.vts_def = 4102,
		.vts_min = 4102,
		.link_freq_index = 0,
	},
};

static inline struct ov64b40 *to_ov64b40(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov64b40, sd);
}

static int ov64b40_power_on(struct ov64b40 *ov64b40)
{
	struct device *dev = ov64b40->dev;
	int ret;

	ret = clk_prepare_enable(ov64b40->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(ov64b40_supplies),
				    ov64b40->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		clk_disable_unprepare(ov64b40->xclk);
		return ret;
	}

	/* vendor power-up: rails, clock, then reset released after 5 ms */
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ov64b40->reset_gpio, 0);
	usleep_range(5000, 6000);

	return 0;
}

static void ov64b40_power_off(struct ov64b40 *ov64b40)
{
	gpiod_set_value_cansleep(ov64b40->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ov64b40_supplies), ov64b40->supplies);
	clk_disable_unprepare(ov64b40->xclk);
}

static int ov64b40_write_regs(struct ov64b40 *ov64b40,
			      const struct cci_reg_sequence *regs,
			      unsigned int n_regs)
{
	int err = 0;

	cci_multi_reg_write(ov64b40->regmap, regs, n_regs, &err);
	if (err)
		dev_err(ov64b40->dev, "register table failed: %d\n", err);
	return err;
}

static int ov64b40_identify(struct ov64b40 *ov64b40)
{
	u64 id = 0;
	int ret;

	ret = cci_read(ov64b40->regmap, OV64B40_REG_CHIP_ID, &id, NULL);
	if (ret)
		return ret;
	if (id != OV64B40_CHIP_ID) {
		dev_err(ov64b40->dev, "chip id 0x%llx, expected 0x%x\n", id,
			OV64B40_CHIP_ID);
		return -ENODEV;
	}
	dev_info(ov64b40->dev, "OV64B40 chip id 0x%llx\n", id);
	return 0;
}

/*
 * The board wires this part as C-PHY (three trios), which neither the CSI PHY
 * driver nor SENINF can program yet, so a stream on such a bus is refused with
 * a message that names the reason. On a D-PHY bus the vendor sequence runs:
 * init table, then the mode table, then streaming.
 */
static int ov64b40_start_streaming(struct ov64b40 *ov64b40)
{
	int ret;

	if (ov64b40->cphy) {
		dev_err(ov64b40->dev,
			"cannot stream: C-PHY is not supported by the CSI PHY or SENINF yet\n");
		return -EOPNOTSUPP;
	}

	ret = ov64b40_write_regs(ov64b40, ov64b40_init_regs,
				 ARRAY_SIZE(ov64b40_init_regs));
	if (ret)
		return ret;
	ret = ov64b40_write_regs(ov64b40, ov64b40_mode_4624x3472_regs,
				 ARRAY_SIZE(ov64b40_mode_4624x3472_regs));
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(ov64b40->sd.ctrl_handler);
	if (ret)
		return ret;

	return cci_write(ov64b40->regmap, OV64B40_REG_MODE_SELECT,
			 OV64B40_MODE_STREAMING, NULL);
}

static int ov64b40_stop_streaming(struct ov64b40 *ov64b40)
{
	return cci_write(ov64b40->regmap, OV64B40_REG_MODE_SELECT,
			 OV64B40_MODE_STANDBY, NULL);
}

static int ov64b40_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov64b40 *ov64b40 = to_ov64b40(sd);
	int ret;

	if (enable == ov64b40->streaming)
		return 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(ov64b40->dev);
		if (ret)
			return ret;
		ret = ov64b40_start_streaming(ov64b40);
		if (ret) {
			pm_runtime_put(ov64b40->dev);
			return ret;
		}
	} else {
		ov64b40_stop_streaming(ov64b40);
		pm_runtime_put(ov64b40->dev);
	}

	ov64b40->streaming = enable;
	return 0;
}

static int ov64b40_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov64b40 *ov64b40 = container_of(ctrl->handler, struct ov64b40,
					       ctrls);
	int ret;

	if (!ov64b40->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(ov64b40->regmap, OV64B40_REG_EXPOSURE, ctrl->val,
				NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(ov64b40->regmap, OV64B40_REG_AGAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(ov64b40->regmap, OV64B40_REG_VTS,
				ctrl->val + ov64b40->cur_mode->height, NULL);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(ov64b40->regmap, OV64B40_REG_HTS, ctrl->val,
				NULL);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret)
		dev_err(ov64b40->dev, "control 0x%x: %d\n", ctrl->id, ret);
	return ret;
}

static const struct v4l2_ctrl_ops ov64b40_ctrl_ops = {
	.s_ctrl = ov64b40_set_ctrl,
};

static int ov64b40_init_controls(struct ov64b40 *ov64b40)
{
	struct v4l2_ctrl_handler *handler = &ov64b40->ctrls;
	const struct ov64b40_mode *mode = ov64b40->cur_mode;
	u32 exposure_max;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;

	ov64b40->link_freq =
		v4l2_ctrl_new_int_menu(handler, &ov64b40_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       ov64b40_link_freq_menu);
	if (ov64b40->link_freq)
		ov64b40->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* pixel_rate = link_freq * 2 (DDR) * trios / 10 bits per pixel */
	pixel_rate = div_u64((u64)OV64B40_LINK_FREQ * 2 * OV64B40_DATA_LANES, 10);
	ov64b40->pixel_rate =
		v4l2_ctrl_new_std(handler, &ov64b40_ctrl_ops, V4L2_CID_PIXEL_RATE,
				  0, pixel_rate, 1, pixel_rate);
	if (ov64b40->pixel_rate)
		ov64b40->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov64b40->hblank = v4l2_ctrl_new_std(handler, &ov64b40_ctrl_ops,
					    V4L2_CID_HBLANK,
					    mode->hts - mode->width,
					    mode->hts - mode->width, 1,
					    mode->hts - mode->width);
	if (ov64b40->hblank)
		ov64b40->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov64b40->vblank = v4l2_ctrl_new_std(handler, &ov64b40_ctrl_ops,
					    V4L2_CID_VBLANK,
					    mode->vts_min - mode->height,
					    OV64B40_VTS_MAX - mode->height, 1,
					    mode->vts_def - mode->height);

	exposure_max = mode->vts_def - 8;
	ov64b40->exposure = v4l2_ctrl_new_std(handler, &ov64b40_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OV64B40_EXPOSURE_MIN, exposure_max,
					      1, exposure_max);

	v4l2_ctrl_new_std(handler, &ov64b40_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV64B40_AGAIN_MIN, OV64B40_AGAIN_MAX, 1,
			  OV64B40_AGAIN_MIN);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	ov64b40->sd.ctrl_handler = handler;
	return 0;
}

static int ov64b40_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_state_get_format(state, 0);
	if (fmt) {
		fmt->code = OV64B40_MBUS_CODE;
		fmt->width = ov64b40_modes[0].width;
		fmt->height = ov64b40_modes[0].height;
		fmt->field = V4L2_FIELD_NONE;
		fmt->colorspace = V4L2_COLORSPACE_RAW;
	}

	crop = v4l2_subdev_state_get_crop(state, 0);
	if (crop) {
		crop->width = ov64b40_modes[0].width;
		crop->height = ov64b40_modes[0].height;
	}
	return 0;
}

static int ov64b40_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = OV64B40_MBUS_CODE;
	return 0;
}

static int ov64b40_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;
	fse->min_width = ov64b40_modes[0].width;
	fse->max_width = ov64b40_modes[0].width;
	fse->min_height = ov64b40_modes[0].height;
	fse->max_height = ov64b40_modes[0].height;
	return 0;
}

static int ov64b40_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	sel->r = *v4l2_subdev_state_get_crop(state, 0);
	return 0;
}

static int ov64b40_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);
	if (!format)
		return -EINVAL;

	if (fmt->format.code != OV64B40_MBUS_CODE)
		fmt->format.code = OV64B40_MBUS_CODE;
	fmt->format.width = ov64b40_modes[0].width;
	fmt->format.height = ov64b40_modes[0].height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	*format = fmt->format;
	return 0;
}

static const struct v4l2_subdev_pad_ops ov64b40_pad_ops = {
	.enum_mbus_code = ov64b40_enum_mbus_code,
	.enum_frame_size = ov64b40_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov64b40_set_pad_format,
	.get_selection = ov64b40_get_selection,
};

static const struct v4l2_subdev_video_ops ov64b40_video_ops = {
	.s_stream = ov64b40_set_stream,
};

static const struct v4l2_subdev_ops ov64b40_subdev_ops = {
	.pad = &ov64b40_pad_ops,
	.video = &ov64b40_video_ops,
};

static const struct v4l2_subdev_internal_ops ov64b40_internal_ops = {
	.init_state = ov64b40_init_state,
};

static const struct media_entity_operations ov64b40_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov64b40_get_pm_resources(struct ov64b40 *ov64b40)
{
	struct device *dev = ov64b40->dev;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ov64b40_supply_names); i++)
		ov64b40->supplies[i].supply = ov64b40_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ov64b40_supplies),
				      ov64b40->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ov64b40->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ov64b40->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ov64b40->reset_gpio),
				     "failed to get reset gpio\n");

	ov64b40->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(ov64b40->xclk))
		return dev_err_probe(dev, PTR_ERR(ov64b40->xclk),
				     "failed to get xclk\n");

	return 0;
}

static int ov64b40_check_hwcfg(struct ov64b40 *ov64b40)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = 0 };
	struct device *dev = ov64b40->dev;
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!fwnode)
		return dev_err_probe(dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return ret;

	/* C-PHY is the board's wiring; D-PHY is accepted for a plain CSI-2 host */
	ov64b40->cphy = ep.bus_type == V4L2_MBUS_CSI2_CPHY;

	if (ep.bus_type != V4L2_MBUS_CSI2_CPHY &&
	    ep.bus_type != V4L2_MBUS_CSI2_DPHY)
		return dev_err_probe(dev, -EINVAL, "not a CSI-2 bus\n");

	if (ep.bus.mipi_csi2.num_data_lanes != OV64B40_DATA_LANES)
		dev_warn(dev, "%u data lanes, the vendor describes %u\n",
			 ep.bus.mipi_csi2.num_data_lanes, OV64B40_DATA_LANES);

	return 0;
}

static int ov64b40_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct ov64b40 *ov64b40;
	int ret;

	ov64b40 = devm_kzalloc(dev, sizeof(*ov64b40), GFP_KERNEL);
	if (!ov64b40)
		return -ENOMEM;
	ov64b40->dev = dev;

	ret = ov64b40_check_hwcfg(ov64b40);
	if (ret)
		return ret;

	ret = ov64b40_get_pm_resources(ov64b40);
	if (ret)
		return ret;

	ov64b40->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov64b40->regmap))
		return dev_err_probe(dev, PTR_ERR(ov64b40->regmap),
				     "failed to init CCI\n");

	ov64b40->cur_mode = &ov64b40_modes[0];

	sd = &ov64b40->sd;
	v4l2_i2c_subdev_init(sd, client, &ov64b40_subdev_ops);
	sd->internal_ops = &ov64b40_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ov64b40_entity_ops;

	ov64b40->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &ov64b40->pad);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init media entity\n");

	ret = ov64b40_init_controls(ov64b40);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init controls\n");
		goto err_entity;
	}

	sd->state_lock = ov64b40->ctrls.lock;
	ret = v4l2_subdev_init_finalize(sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to finalize subdev\n");
		goto err_ctrls;
	}

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	/* identify on a powered part, then let runtime PM put it back down */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto err_pm;
	ret = ov64b40_identify(ov64b40);
	pm_runtime_put(dev);
	if (ret)
		goto err_pm;

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
	v4l2_ctrl_handler_free(&ov64b40->ctrls);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void ov64b40_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov64b40 *ov64b40 = to_ov64b40(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(ov64b40->dev);
	v4l2_subdev_cleanup(sd);
	v4l2_ctrl_handler_free(&ov64b40->ctrls);
	media_entity_cleanup(&sd->entity);
}

static int ov64b40_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	ov64b40_power_off(to_ov64b40(sd));
	return 0;
}

static int ov64b40_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	return ov64b40_power_on(to_ov64b40(sd));
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov64b40_pm_ops, ov64b40_runtime_suspend,
				 ov64b40_runtime_resume, NULL);

static const struct of_device_id ov64b40_of_match[] = {
	{ .compatible = "ovti,ov64b40" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov64b40_of_match);

static const struct i2c_device_id ov64b40_id_table[] = {
	{ "ov64b40" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ov64b40_id_table);

static struct i2c_driver ov64b40_driver = {
	.driver = {
		.name = "ov64b40",
		.pm = pm_ptr(&ov64b40_pm_ops),
		.of_match_table = ov64b40_of_match,
	},
	.probe = ov64b40_probe,
	.remove = ov64b40_remove,
	.id_table = ov64b40_id_table,
};
module_i2c_driver(ov64b40_driver);

MODULE_DESCRIPTION("OmniVision OV64B40 image sensor driver");
MODULE_LICENSE("GPL");
