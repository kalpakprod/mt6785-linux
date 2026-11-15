// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3418 incell AMOLED touchscreen controller.
 *
 * Found on the Xiaomi POCO M5s / Redmi Note 10S (rosemary) behind the
 * "dsi_panel_k7_38_0c_0a_fhdp_video" panel, on I2C0 at address 0x38.
 *
 * The register map and the report format come from the vendor driver
 * drivers/input/touchscreen/mediatek/ft3418_i2c/ in
 * Youffx/kernel_xiaomi_rosemary @ 634d6ac5d96e, branch 23.2-lineage
 * (focaltech_core.c, focaltech_core.h, focaltech_common.h,
 * focaltech_config.h), extracted into
 * poco-m5s/research/native/1-display-touch/touch-protocol-ft3418.txt.
 *
 * The FT3418 is not an FT5x06-family part: it reports through the incell FTS
 * frame ("write 0x01 to 0x00, read 63 bytes from 0x01") and identifies as
 * 0x54/0x52, so edt-ft5x06.c cannot drive it. Only the identify, poll and
 * report plumbing is shared with that driver.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

/* Registers, all from the vendor's FTS_REG_* list */
#define FT3418_REG_FRAME		0x00	/* write 0x01 to arm a frame read */
#define FT3418_FRAME_START		0x01	/* frame data starts here */
#define FT3418_REG_CHIP_ID2		0x9f
#define FT3418_REG_CHIP_ID		0xa3
#define FT3418_REG_POWER_MODE		0xa5

#define FT3418_CHIP_ID_HI		0x54
#define FT3418_CHIP_ID_LO		0x52

#define FT3418_MAX_POINTS		10
#define FT3418_POINT_LEN		6
#define FT3418_FRAME_LEN		(FT3418_MAX_POINTS * FT3418_POINT_LEN + 3)
#define FT3418_POINT_NUM(buf)		((buf)[2] & 0x0f)

#define FT3418_ID_RETRIES		5
#define FT3418_ID_INTERVAL_MS		200

#define FT3418_POWER_SLEEP		0x03

#define FT3418_EVENT_DOWN		0
#define FT3418_EVENT_UP			1
#define FT3418_EVENT_CONTACT		2
#define FT3418_EVENT_RESERVED		3

struct ft3418_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct regulator *iovdd;
	struct touchscreen_properties prop;
};

static int ft3418_reg_read(struct i2c_client *client, u8 reg, void *buf,
			   size_t len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int ft3418_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	u8 buf[2] = { reg, value };
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return ret == sizeof(buf) ? 0 : -EIO;
}

static int ft3418_read_frame(struct i2c_client *client, u8 *buf)
{
	u8 cmd[2] = { FT3418_REG_FRAME, FT3418_FRAME_START };
	int ret;

	ret = i2c_master_send(client, cmd, sizeof(cmd));
	if (ret < 0)
		return ret;
	if (ret != sizeof(cmd))
		return -EIO;

	return ft3418_reg_read(client, FT3418_FRAME_START, buf,
			       FT3418_FRAME_LEN);
}

static void ft3418_reset(struct ft3418_data *ts)
{
	/* reset is active low in DT, so 1 pulls the line down */
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	/* the vendor waits for the IC to come back before talking to it */
	msleep(300);
}

static int ft3418_read_id(struct ft3418_data *ts, u8 *id_hi, u8 *id_lo)
{
	struct i2c_client *client = ts->client;
	struct device *dev = &client->dev;
	int i, ret;

	for (i = 0; i < FT3418_ID_RETRIES; i++) {
		ret = ft3418_reg_read(client, FT3418_REG_CHIP_ID, id_hi, 1);
		if (ret)
			return ret;

		ret = ft3418_reg_read(client, FT3418_REG_CHIP_ID2, id_lo, 1);
		if (ret)
			return ret;

		if (*id_hi == FT3418_CHIP_ID_HI && *id_lo == FT3418_CHIP_ID_LO)
			return 0;

		dev_dbg(dev, "id 0x%02x%02x not ready, retrying\n", *id_hi,
			*id_lo);
		msleep(FT3418_ID_INTERVAL_MS);
	}

	dev_err(dev, "no answer, id 0x%02x%02x, expected 0x%02x%02x\n",
		*id_hi, *id_lo, FT3418_CHIP_ID_HI, FT3418_CHIP_ID_LO);

	return -ENODEV;
}

static void ft3418_report(struct ft3418_data *ts, const u8 *buf)
{
	struct input_dev *input = ts->input;
	unsigned int point_num = FT3418_POINT_NUM(buf);
	unsigned int i;

	/*
	 * An incell part answers point_num 0x0f with an 0xff payload when the
	 * IC is in a bad state; the vendor releases every finger and recovers.
	 * Recovering needs process context, so only the release is done here.
	 */
	if (point_num == 0x0f)
		point_num = 0;

	if (point_num > FT3418_MAX_POINTS) {
		dev_dbg_ratelimited(&ts->client->dev,
				    "invalid point_num %u\n", point_num);
		point_num = 0;
	}

	for (i = 0; i < point_num; i++) {
		const u8 *p = &buf[3 + FT3418_POINT_LEN * i];
		unsigned int event = p[0] >> 6;
		unsigned int id = p[2] >> 4;
		unsigned int x = ((p[0] & 0x0f) << 8) | p[1];
		unsigned int y = ((p[2] & 0x0f) << 8) | p[3];
		unsigned int weight = p[4];
		unsigned int area = p[5] >> 4;

		if (event == FT3418_EVENT_RESERVED || id >= FT3418_MAX_POINTS)
			break;

		input_mt_slot(input, id);

		if (event == FT3418_EVENT_UP) {
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
			continue;
		}

		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(input, &ts->prop, x, y, true);
		input_report_abs(input, ABS_MT_PRESSURE, weight);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, area);
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

static irqreturn_t ft3418_irq(int irq, void *data)
{
	struct ft3418_data *ts = data;
	u8 buf[FT3418_FRAME_LEN];
	int ret;

	ret = ft3418_read_frame(ts->client, buf);
	if (ret) {
		dev_dbg_ratelimited(&ts->client->dev, "frame read failed: %d\n",
				    ret);
		return IRQ_HANDLED;
	}

	ft3418_report(ts, buf);

	return IRQ_HANDLED;
}

static int ft3418_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ft3418_data *ts;
	struct input_dev *input;
	unsigned long irq_flags;
	u8 id_hi = 0, id_lo = 0;
	int error;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/*
	 * The 3.0 V rail is the PMIC's LDO2 (the live DT's "vtouch-supply") and
	 * the 1.8 V I/O rail is not described on this board, so both are taken
	 * optionally: when the board names neither, the rails stay as the
	 * loader left them.
	 */
	ts->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		if (PTR_ERR(ts->vdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->vdd), "vdd supply\n");
		ts->vdd = NULL;
	}

	ts->iovdd = devm_regulator_get_optional(dev, "iovdd");
	if (IS_ERR(ts->iovdd)) {
		if (PTR_ERR(ts->iovdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ts->iovdd), "iovdd supply\n");
		ts->iovdd = NULL;
	}

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio), "reset gpio\n");

	if (ts->iovdd) {
		error = regulator_enable(ts->iovdd);
		if (error)
			return dev_err_probe(dev, error, "cannot enable iovdd\n");
	}

	if (ts->vdd) {
		error = regulator_enable(ts->vdd);
		if (error)
			goto err_iovdd;
	}

	ft3418_reset(ts);

	error = ft3418_read_id(ts, &id_hi, &id_lo);
	if (error)
		goto err_regulators;

	dev_info(dev, "FT3418 ready, id 0x%02x%02x\n", id_hi, id_lo);

	input = devm_input_allocate_device(dev);
	if (!input) {
		error = -ENOMEM;
		goto err_regulators;
	}

	ts->input = input;
	input->name = "ft3418_ts";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 1079, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 2399, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	error = input_mt_init_slots(input, FT3418_MAX_POINTS, INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "cannot init MT slots: %d\n", error);
		goto err_regulators;
	}

	irq_flags = irq_get_trigger_type(client->irq);
	if (irq_flags == IRQF_TRIGGER_NONE)
		irq_flags = IRQF_TRIGGER_FALLING;

	error = devm_request_threaded_irq(dev, client->irq, NULL, ft3418_irq,
					  irq_flags | IRQF_ONESHOT, client->name,
					  ts);
	if (error) {
		dev_err(dev, "cannot request irq %d: %d\n", client->irq, error);
		goto err_regulators;
	}

	error = input_register_device(input);
	if (error)
		goto err_regulators;

	return 0;

err_regulators:
	if (ts->vdd)
		regulator_disable(ts->vdd);
err_iovdd:
	if (ts->iovdd)
		regulator_disable(ts->iovdd);
	return error;
}

static void ft3418_remove(struct i2c_client *client)
{
	struct ft3418_data *ts = i2c_get_clientdata(client);

	disable_irq(client->irq);
	if (ts->vdd)
		regulator_disable(ts->vdd);
	if (ts->iovdd)
		regulator_disable(ts->iovdd);
}

static int ft3418_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);
	ft3418_reg_write(client, FT3418_REG_POWER_MODE, FT3418_POWER_SLEEP);

	return 0;
}

static int ft3418_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft3418_data *ts = i2c_get_clientdata(client);
	u8 id_hi, id_lo;

	ft3418_reset(ts);
	if (ft3418_read_id(ts, &id_hi, &id_lo))
		dev_err(dev, "not answering after resume\n");
	enable_irq(client->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ft3418_pm_ops, ft3418_suspend, ft3418_resume);

static const struct i2c_device_id ft3418_id[] = {
	{ "ft3418" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft3418_id);

static const struct of_device_id ft3418_of_match[] = {
	{ .compatible = "focaltech,ft3418" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft3418_of_match);

static struct i2c_driver ft3418_driver = {
	.driver = {
		.name = "focaltech-ft3418",
		.of_match_table = ft3418_of_match,
		.pm = pm_sleep_ptr(&ft3418_pm_ops),
	},
	.probe = ft3418_probe,
	.remove = ft3418_remove,
	.id_table = ft3418_id,
};
module_i2c_driver(ft3418_driver);

MODULE_DESCRIPTION("FocalTech FT3418 touchscreen");
MODULE_LICENSE("GPL");
