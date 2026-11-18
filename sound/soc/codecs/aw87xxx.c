// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW87xxx smart speaker amplifier driver
 *
 * Copyright (c) 2020 AWINIC Technology CO., LTD
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * Mainline-shaped driver for the Awinic AW87xxx family of Class-D speaker
 * amplifiers (AW87339, AW87359, AW87369, AW87519, AW87559). The part takes its
 * audio in analog from the codec and is programmed over I2C with a per-scene
 * register profile: the phone's own /vendor/firmware holds one profile per
 * scene (off, music, voice, fm, rcv) for each part and channel.
 *
 * Ported from the vendor driver sound/soc/codecs/aw87xxx/aw87xxx.c (V0.1.15) and
 * its firmware container parser aw_bin_parse.c of LineageOS'
 * android_kernel_xiaomi_mt6785 (scratch/native/4-modem/src/los-mt6785):
 *  - the part is detected from register 0x00, as the vendor driver does;
 *  - the profile file name is built the way the vendor driver builds it,
 *    aw87xxx_pid_<id>_<scene>_<pa-channel>.bin;
 *  - the scene is switched from ASoC DAPM events (power up: music, power down:
 *    off) instead of the vendor's procfs/ioctl interface, so the amplifier is
 *    part of the sound card's DAPM graph and switches on and off with the
 *    output path. The vendor's monitor (aw87xxx_monitor.c) and its DSP helper
 *    (awinic_dsp.c) are not carried over: the monitor only re-reads the
 *    temperature-compensation table on a timer and the DSP helper is for
 *    parts with a DSP, which none of the five above has.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define AW87XXX_REG_CHIPID		0x00
#define AW87XXX_CHIPID_39		0x39
#define AW87XXX_CHIPID_59		0x59
#define AW87XXX_CHIPID_69		0x69
#define AW87XXX_CHIPID_5A		0x5a

/* Awinic's firmware container, from the vendor aw_bin_parse.h */
#define AW87XXX_BIN_HEADER_LEN		60
#define AW87XXX_BIN_HEADER_VERSION	0x01000000
#define AW87XXX_BIN_DATA_TYPE_REG	0x00000000
#define AW87XXX_BIN_DATA_VERSION	0x00000001
#define AW87XXX_CFG_NAME_MAX		64

enum aw87xxx_mode {
	AW87XXX_MODE_OFF = 0,
	AW87XXX_MODE_MUSIC,
	AW87XXX_MODE_VOICE,
	AW87XXX_MODE_FM,
	AW87XXX_MODE_RCV,
	AW87XXX_MODE_MAX,
};

static const char * const aw87xxx_mode_name[AW87XXX_MODE_MAX] = {
	[AW87XXX_MODE_OFF] = "off",
	[AW87XXX_MODE_MUSIC] = "music",
	[AW87XXX_MODE_VOICE] = "voice",
	[AW87XXX_MODE_FM] = "fm",
	[AW87XXX_MODE_RCV] = "rcv",
};

/**
 * struct aw87xxx - one amplifier instance
 * @dev: the I2C device
 * @client: its I2C client
 * @reset_gpio: the part's reset pin, if it has one on this board
 * @chipid: the value read from register 0x00
 * @channel: the pa-channel property, which the profile names carry
 * @mode: the scene currently programmed
 * @lock: serialises register and reset access
 */
struct aw87xxx {
	struct device *dev;
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	u8 chipid;
	u32 channel;
	enum aw87xxx_mode mode;
	struct mutex lock; /* serialises register and reset access */
};

static const char *aw87xxx_part_name(u8 chipid)
{
	switch (chipid) {
	case AW87XXX_CHIPID_39:
		return "aw87339";
	case AW87XXX_CHIPID_59:
		return "aw87359/aw87519";
	case AW87XXX_CHIPID_69:
		return "aw87369";
	case AW87XXX_CHIPID_5A:
		return "aw87559";
	default:
		return NULL;
	}
}

static u32 aw87xxx_le32(const u8 *p)
{
	return (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0];
}

/**
 * aw87xxx_fw_is_container() - tell the two profile formats apart
 * @data: the file
 * @len: its length
 *
 * Awinic ships profile files in two shapes: the container with the 60-byte
 * header this driver parses, and, for older parts, a bare list of
 * (register, value) bytes. The vendor parser tells them apart the same way: a
 * container's first four bytes are the little-endian sum of everything that
 * follows.
 */
static bool aw87xxx_fw_is_container(const u8 *data, size_t len)
{
	u32 sum = 0;
	unsigned int i;

	if (len < sizeof(u32))
		return false;

	for (i = sizeof(u32); i < len; i++)
		sum += data[i];

	return sum == aw87xxx_le32(data);
}

/**
 * aw87xxx_load_profile() - program one scene into the amplifier
 * @priv: the amplifier
 * @mode: the scene to program
 *
 * The file is Awinic's container: a 60-byte header, then a 32-bit register
 * count, then that many (register, value) byte pairs. Only the register-file
 * type this phone's firmware uses is accepted (register and value one byte
 * wide), exactly as the vendor parser does.
 */
static int aw87xxx_write_pairs(struct aw87xxx *priv, const char *name,
			       const u8 *pairs, unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = i2c_smbus_write_byte_data(priv->client, pairs[2 * i],
						pairs[2 * i + 1]);
		if (ret < 0) {
			dev_warn(priv->dev, "%s: register 0x%02x failed: %d\n",
				 name, pairs[2 * i], ret);
			return ret;
		}
	}

	return 0;
}

static int aw87xxx_load_profile(struct aw87xxx *priv, enum aw87xxx_mode mode)
{
	char name[AW87XXX_CFG_NAME_MAX];
	const struct firmware *fw;
	const u8 *data, *pairs;
	u32 check_sum, header_ver, data_type, data_ver, data_len, reg_len;
	u32 val_len, reg_num, sum = 0;
	unsigned int i;
	int ret;

	snprintf(name, sizeof(name), "aw87xxx_pid_%02x_%s_%u.bin", priv->chipid,
		 aw87xxx_mode_name[mode], priv->channel);

	ret = request_firmware(&fw, name, priv->dev);
	if (ret) {
		/* A board without the profile files simply keeps the defaults. */
		dev_warn(priv->dev, "no profile %s: %d\n", name, ret);
		return ret;
	}

	data = fw->data;
	ret = 0;

	if (!aw87xxx_fw_is_container(data, fw->size)) {
		/*
		 * Bare (register, value) list, which is what the files of the
		 * AW87359/AW87519 on this board are.
		 */
		if (fw->size < 2 || fw->size % 2) {
			dev_warn(priv->dev, "%s: %zu bytes, not a register list\n",
				 name, fw->size);
			goto out;
		}
		ret = aw87xxx_write_pairs(priv, name, data, fw->size / 2);
		goto out;
	}

	if (fw->size < AW87XXX_BIN_HEADER_LEN) {
		dev_warn(priv->dev, "%s: shorter than its header\n", name);
		goto out;
	}

	check_sum = aw87xxx_le32(data);
	header_ver = aw87xxx_le32(data + 4);
	data_type = aw87xxx_le32(data + 8);
	data_ver = aw87xxx_le32(data + 12);
	data_len = aw87xxx_le32(data + 16);
	reg_len = aw87xxx_le32(data + 32);
	val_len = aw87xxx_le32(data + 36);

	if (header_ver != AW87XXX_BIN_HEADER_VERSION ||
	    data_type != AW87XXX_BIN_DATA_TYPE_REG ||
	    data_ver != AW87XXX_BIN_DATA_VERSION ||
	    reg_len != 1 || val_len != 1) {
		dev_warn(priv->dev, "%s: unsupported container\n", name);
		goto out;
	}

	if (AW87XXX_BIN_HEADER_LEN + data_len > fw->size ||
	    data_len < sizeof(u32)) {
		dev_warn(priv->dev, "%s: bad length %u\n", name, data_len);
		goto out;
	}

	for (i = 4; i < AW87XXX_BIN_HEADER_LEN + data_len; i++)
		sum += data[i];
	if (sum != check_sum) {
		dev_warn(priv->dev, "%s: checksum 0x%08x, computed 0x%08x\n",
			 name, check_sum, sum);
		goto out;
	}

	pairs = data + AW87XXX_BIN_HEADER_LEN;
	reg_num = aw87xxx_le32(pairs);
	if (reg_num != (data_len - sizeof(u32)) / 2) {
		dev_warn(priv->dev, "%s: %u registers in %u bytes\n", name,
			 reg_num, data_len);
		goto out;
	}

	ret = aw87xxx_write_pairs(priv, name, pairs + sizeof(u32), reg_num);

out:
	release_firmware(fw);

	if (!ret) {
		priv->mode = mode;
		dev_dbg(priv->dev, "%s: programmed\n", name);
	}

	return ret;
}

static void aw87xxx_reset(struct aw87xxx *priv)
{
	if (!priv->reset_gpio)
		return;

	/* The part resets while its reset pin is low: assert, wait, release. */
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(1000, 3000);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(1000, 3000);
}

static int aw87xxx_amp_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct aw87xxx *priv = snd_soc_component_get_drvdata(component);

	mutex_lock(&priv->lock);

	/*
	 * A profile that cannot be programmed is reported by the loader and
	 * leaves the part at its power-on defaults; it must not fail the
	 * sound-card path itself.
	 */
	if (event & SND_SOC_DAPM_POST_PMU) {
		aw87xxx_reset(priv);
		aw87xxx_load_profile(priv, AW87XXX_MODE_MUSIC);
	} else if (event & SND_SOC_DAPM_PRE_PMD) {
		aw87xxx_load_profile(priv, AW87XXX_MODE_OFF);
	}

	mutex_unlock(&priv->lock);

	return 0;
}

static const struct snd_soc_dapm_widget aw87xxx_dapm_widgets[] = {
	/*
	 * The amplifier is a sink in the card's audio path: the machine driver
	 * routes the codec's output into it, and the widget's power events load
	 * the matching scene. The DT's sound-name-prefix makes the name unique
	 * when a board has two of them.
	 */
	SND_SOC_DAPM_SPK("Amp", aw87xxx_amp_event),
};

static const struct snd_soc_component_driver aw87xxx_component_driver = {
	.dapm_widgets = aw87xxx_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw87xxx_dapm_widgets),
	.endianness = 1,
};

static int aw87xxx_probe(struct i2c_client *client)
{
	struct aw87xxx *priv;
	int ret, chipid;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &client->dev;
	priv->client = client;
	mutex_init(&priv->lock);

	ret = device_property_read_u32(priv->dev, "awinic,pa-channel",
				       &priv->channel);
	if (ret)
		priv->channel = 0;

	priv->reset_gpio = devm_gpiod_get_optional(priv->dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(priv->dev, PTR_ERR(priv->reset_gpio),
				     "cannot get the reset gpio\n");

	i2c_set_clientdata(client, priv);

	chipid = i2c_smbus_read_byte_data(client, AW87XXX_REG_CHIPID);
	if (chipid < 0)
		return dev_err_probe(priv->dev, chipid,
				     "cannot read the chip id\n");

	if (!aw87xxx_part_name(chipid))
		return dev_err_probe(priv->dev, -ENODEV,
				     "unknown chip id 0x%02x\n", chipid);

	priv->chipid = chipid;
	priv->mode = AW87XXX_MODE_MAX;

	dev_info(priv->dev, "%s at 0x%02x, pa-channel %u\n",
		 aw87xxx_part_name(chipid), client->addr, priv->channel);

	/*
	 * Leave the part in its off scene until a path powers it up. A missing
	 * or unreadable profile is not a reason to fail: the amplifier works
	 * with its power-on defaults, and the sound card must still come up.
	 */
	aw87xxx_reset(priv);
	aw87xxx_load_profile(priv, AW87XXX_MODE_OFF);

	return devm_snd_soc_register_component(priv->dev,
					       &aw87xxx_component_driver,
					       NULL, 0);
}

static const struct of_device_id aw87xxx_of_match[] = {
	{ .compatible = "awinic,aw87xxx" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw87xxx_of_match);

static const struct i2c_device_id aw87xxx_i2c_id[] = {
	{ "aw87xxx" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw87xxx_i2c_id);

static struct i2c_driver aw87xxx_driver = {
	.driver = {
		.name = "aw87xxx",
		.of_match_table = aw87xxx_of_match,
	},
	.probe = aw87xxx_probe,
	.id_table = aw87xxx_i2c_id,
};
module_i2c_driver(aw87xxx_driver);

MODULE_AUTHOR("Bruce <zhaolei@awinic.com>");
MODULE_AUTHOR("AWINIC Technology CO., LTD");
MODULE_DESCRIPTION("Awinic AW87xxx smart speaker amplifier driver");
MODULE_LICENSE("GPL");
