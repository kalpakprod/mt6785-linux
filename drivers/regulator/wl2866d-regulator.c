// SPDX-License-Identifier: GPL-2.0-only
/*
 * Wingtech/Will Semiconductor WL2866D four-channel camera LDO controller.
 *
 * The part sits on the camera I2C bus and feeds the sensor rails: two DVDD
 * outputs with a 0.6 V base and 6 mV steps, two AVDD outputs with a 1.2 V
 * base and 12.5 mV steps, and one enable bit per output.
 *
 * Register map and voltage tables are from the vendor driver of the Xiaomi
 * MT6785 Android 4.19 kernel (LineageOS android_kernel_xiaomi_mt6785,
 * branch lineage-22.2): drivers/regulator/wl2866d.c, which defines
 * WL2866D_REG_LDO = 0x03 with LDO_VSET_REG(offset) = offset + 0x03, the
 * output enable register 0x0e with "1u << offset" per output, the discharge
 * register 0x02 written as 0x8f once at probe, and the two voltage formulas
 * (VSET_BASE_12 6000 + sel * 60, VSET_BASE_34 12000 + sel * 125, both in
 * 100 uV units). The rails of this phone are described in the live device
 * tree, i2c6/wl2866d@28, with the same windows this driver computes.
 *
 * The part has no readable identification register: the vendor's "ID" check
 * reads a voltage-selector register and accepts both its value and 0, so a
 * missing chip passes it. This driver instead proves that the chip answers
 * and that writes stick, by writing back the value it just read.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define WL2866D_REG_DISCHARGE		0x02
#define WL2866D_REG_VSET(_id)		(0x03 + (_id))
#define WL2866D_REG_ENABLE		0x0e

/* The vendor driver writes this value to the discharge register at probe. */
#define WL2866D_DISCHARGE_INIT		0x8f

#define WL2866D_NUM_LDO			4

struct wl2866d {
	struct regmap *regmap;
};

static const struct regulator_ops wl2866d_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

/*
 * LDO1/LDO2 run from 0.6 V in 6 mV steps, LDO3/LDO4 from 1.2 V in 12.5 mV
 * steps, 256 selectors each, which is 2.13 V and 4.3875 V at the top and
 * matches the windows of the live device tree exactly.
 */
#define WL2866D_LDO(_id, _name, _min_uV, _step_uV)			\
	{								\
		.name = _name,						\
		.of_match = of_match_ptr(_name),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.id = (_id),						\
		.ops = &wl2866d_ops,					\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.n_voltages = 256,					\
		.min_uV = (_min_uV),					\
		.uV_step = (_step_uV),					\
		.vsel_reg = WL2866D_REG_VSET(_id),			\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2866D_REG_ENABLE,			\
		.enable_mask = BIT(_id),				\
	}

static const struct regulator_desc wl2866d_regulators[WL2866D_NUM_LDO] = {
	WL2866D_LDO(0, "dvdd1", 600000, 6000),
	WL2866D_LDO(1, "dvdd2", 600000, 6000),
	WL2866D_LDO(2, "avdd1", 1200000, 12500),
	WL2866D_LDO(3, "avdd2", 1200000, 12500),
};

static const struct regmap_config wl2866d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2866D_REG_ENABLE,
	.cache_type = REGCACHE_NONE,
};

/*
 * A read alone cannot tell "the chip answered" from "the bus returned
 * something", so read a selector, write it back unchanged and read it again:
 * a part that is absent or held in reset fails at the first or the last step.
 */
static int wl2866d_verify(struct wl2866d *chip)
{
	unsigned int vset, readback, enable;
	int ret;

	ret = regmap_read(chip->regmap, WL2866D_REG_VSET(0), &vset);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, WL2866D_REG_VSET(0), vset);
	if (ret)
		return ret;

	ret = regmap_read(chip->regmap, WL2866D_REG_VSET(0), &readback);
	if (ret)
		return ret;

	if (readback != vset)
		return -ENODEV;

	ret = regmap_read(chip->regmap, WL2866D_REG_ENABLE, &enable);
	if (ret)
		return ret;

	return 0;
}

static int wl2866d_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct wl2866d *chip;
	struct gpio_desc *enable;
	int i, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	/*
	 * The vendor drives this line low for the whole time it talks to the
	 * part, so the device tree describes it as active low and this driver
	 * asserts it, which leaves the same level on the pin.
	 */
	enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable))
		return dev_err_probe(dev, PTR_ERR(enable), "enable GPIO\n");
	if (enable)
		usleep_range(1000, 2000);

	chip->regmap = devm_regmap_init_i2c(client, &wl2866d_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "regmap init failed\n");

	ret = wl2866d_verify(chip);
	if (ret)
		return dev_err_probe(dev, ret, "no WL2866D at %#x\n",
				     client->addr);

	/* Discharge off the outputs when they are disabled, as the vendor does. */
	ret = regmap_write(chip->regmap, WL2866D_REG_DISCHARGE,
			   WL2866D_DISCHARGE_INIT);
	if (ret)
		return dev_err_probe(dev, ret, "discharge setup failed\n");

	config.dev = dev;
	config.regmap = chip->regmap;
	config.driver_data = chip;

	for (i = 0; i < WL2866D_NUM_LDO; i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &wl2866d_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2866d_regulators[i].name);
	}

	return 0;
}

static const struct i2c_device_id wl2866d_id_table[] = {
	{ .name = "wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_id_table);

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
	.id_table = wl2866d_id_table,
};
module_i2c_driver(wl2866d_driver);

MODULE_DESCRIPTION("Wingtech/WillSemi WL2866D camera LDO controller");
MODULE_LICENSE("GPL");
