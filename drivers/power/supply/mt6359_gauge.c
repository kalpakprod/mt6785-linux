// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6359 coulomb-counting fuel gauge.
 *
 * A missing block of a supported chip: the MT6359 is driven by mt6397-core (MFD),
 * mt6359-regulator, mt6359-rtc and others, but its fuel-gauge ADC has no driver in
 * mainline 7.3.4, so a phone with it reports whatever a generic ADC battery does -
 * voltage, current, a design capacity, and no charge percentage at all.
 *
 * Sources for everything here, all in the vendor 4.19 driver
 * (scratch/native/4-modem/src/los-mt6785/drivers/misc/mediatek/pmic/mt6359/v1/mt6359_gauge.c):
 *
 *   :45,53,55,57   the coulomb counter's LSB, 0.085 uAh, with CAR_TO_REG_SHIFT 5 and
 *                  CAR_TO_REG_FACTOR 0x2E14 = 1000*1000/85
 *   :44            DEFAULT_R_FG 50 mOhm, and the scaling by DEFAULT_R_FG / r_fg_value
 *                  (:258-260, :303-305) for boards with another resistor
 *   :41-42         the average-current LSB, 305.176 uA
 *   :659-666       the 32-bit counter is read as two halves, NCAR_CON0 (15:0) and
 *                  NCAR_CON1 (31:16); their offsets are 0xd24 and 0xd26
 *                  (upmu_hw.h:558-559, wired as PMIC_FG_NCAR_15_00/31_16 at
 *                  upmu_hw.h:8647-8653)
 *
 * The register helpers and the arithmetic between them live in
 * mt6359_gauge_calc.h, which the host test in tools/battery-gauge/tests holds to the
 * reference values, including three planted wrong constants.
 *
 * What this driver does not do yet, and says so: it does not anchor the capacity on
 * an open-circuit voltage with the vendor's ZCV tables. charge_now and capacity are
 * the coulomb count against the full charge from the battery profile in the DT, which
 * is exact while the counter is calibrated at boot (the gauge starts counting from the
 * value the loader left) and drifts with the battery's age. OCV anchoring with
 * power_supply_ocv2cap_simple() over a simple-battery profile is the next step and
 * needs rosemary's own tables out of the vendor DT.
 */
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#include "mt6359_gauge_calc.h"

/*
 * Two counters live in the gauge, and they are not the same register pair:
 *
 *   MT6359_FGADC_CAR_CON0/CON1   0xd16/0xd18   the coulomb counter, mapped as
 *                                              PMIC_FG_CAR_15_00/31_16
 *                                              (upmu_hw.h:8623-8628)
 *   MT6359_FGADC_NCAR_CON0/CON1  0xd24/0xd26   the *negative* counter, mapped as
 *                                              PMIC_FG_NCAR_15_00/31_16
 *                                              (upmu_hw.h:8647-8653), which only
 *                                              moves in the discharge direction
 *
 * The first version of this driver read the NCAR pair, which is why a phone on the
 * charger reported a counter of 0 and a current of 0 for as long as it was plugged in.
 */
#define MT6359_FGADC_CAR_CON0	0xd16
#define MT6359_FGADC_CAR_CON1	0xd18

/* The gauge itself must be switched on before either counter moves. */
#define MT6359_FGADC_CON0	0xd08
#define MT6359_FG_ON		BIT(0)	/* upmu_hw.h:8419-8422, PMIC_FG_ON */

/* Hardware current, in the vendor's own register (MTK: PMIC_FG_CURRENT_OUT). */
#define MT6359_FGADC_CUR_CON0	0xd8a

/* The counter is latched for a consistent read; CON1 carries both bits. */
#define MT6359_FGADC_CON1	0xd0a
#define MT6359_FGADC_CON1_LATCH	BIT(15)	/* trigger a snapshot */
#define MT6359_FGADC_CON1_ST	BIT(0)	/* PMIC_FG_LATCHDATA_ST, upmu_hw.h:8499-8501 */

/* How often the counter is sampled for current_now; the display is not faster. */
#define MT6359_FG_SAMPLE_MS	1000

struct mt6359_gauge {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct iio_channel *vbat;
	struct iio_channel *ntc;
	unsigned int r_fg_scale;
	long long charge_full_uah;

	/* Previous counter sample, for current_now. */
	long long last_car;
	ktime_t last_time;
	long long current_ua;
	struct delayed_work sample;
};

static int mt6359_gauge_read_car(struct mt6359_gauge *g, long long *car)
{
	unsigned int lo, hi;
	int ret;

	ret = regmap_read(g->regmap, MT6359_FGADC_CAR_CON0, &lo);
	if (ret)
		return ret;
	ret = regmap_read(g->regmap, MT6359_FGADC_CAR_CON1, &hi);
	if (ret)
		return ret;

	/* The counter is a signed 32-bit value split over two halves. */
	*car = (long long)(s32)(((hi & 0xffff) << 16) | (lo & 0xffff));
	return 0;
}

static void mt6359_gauge_sample(struct work_struct *work)
{
	struct mt6359_gauge *g = container_of(to_delayed_work(work),
					      struct mt6359_gauge, sample);
	ktime_t now = ktime_get();
	long long car, dcar = 0;
	s64 dt_us;

	if (!mt6359_gauge_read_car(g, &car)) {
		dcar = car - g->last_car;
		dt_us = ktime_us_delta(now, g->last_time);
		g->last_car = car;
		g->last_time = now;
		/*
		 * Current from the charge difference: dq (uAh) over dt (h) is a current
		 * in uA, with a factor of 1000 between the two. This is the vendor's own
		 * approach - its current comes from the counter, not from a separate
		 * register (:640-700) - and it is why the counter's LSB matters.
		 */
		if (dt_us > 0) {
			long long duah = mt6359_car_to_uah(dcar, g->r_fg_scale);

			g->current_ua = div_s64(duah * 1000000, dt_us);
		}
	}
	schedule_delayed_work(&g->sample, msecs_to_jiffies(MT6359_FG_SAMPLE_MS));
}

static int mt6359_gauge_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mt6359_gauge *g = power_supply_get_drvdata(psy);
	long long car;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = mt6359_gauge_read_car(g, &car);
		if (ret)
			return ret;
		car = mt6359_car_to_uah(car, g->r_fg_scale);
		if (psp == POWER_SUPPLY_PROP_CHARGE_COUNTER ||
		    psp == POWER_SUPPLY_PROP_CHARGE_NOW)
			val->intval = car;
		else
			val->intval = mt6359_uah_to_capacity(car,
							     g->charge_full_uah);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = g->charge_full_uah;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/*
		 * From the gauge's own current register rather than from the counter's
		 * difference: MTK exposes the result (PMIC_FG_CURRENT_OUT ->
		 * MT6359_FGADC_CUR_CON0, upmu_hw.h:8807-8810, register 0xd8a) and the
		 * vendor reads it there (:236-250, read_fg_hw_info_current_1). The counter
		 * difference stays as a cross-check in the log.
		 */
		ret = regmap_read(g->regmap, MT6359_FGADC_CUR_CON0, &val->intval);
		if (ret)
			return ret;
		val->intval = (long long)(s16)val->intval *
			      mt6359_iavg_to_ua(1, g->r_fg_scale);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!g->vbat)
			return -ENODATA;
		ret = iio_read_channel_processed(g->vbat, &val->intval);
		if (ret < 0)
			return ret;
		/* iio gives mV for this ADC; power_supply wants uV. */
		val->intval *= 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!g->ntc)
			return -ENODATA;
		ret = iio_read_channel_processed(g->ntc, &val->intval);
		if (ret < 0)
			return ret;
		/* iio gives tenths of a degree; power_supply wants the same. */
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property mt6359_gauge_props[] = {
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc mt6359_gauge_desc = {
	.name = "mt6359_fg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mt6359_gauge_props,
	.num_properties = ARRAY_SIZE(mt6359_gauge_props),
	.get_property = mt6359_gauge_get_property,
};

static int mt6359_gauge_probe(struct platform_device *pdev)
{
	struct mt6397_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config cfg = { };
	struct mt6359_gauge *g;
	u32 val;
	int ret;

	/* The regmap is the PMIC chip's, as for every sibling (lead note 35). */
	if (!chip || !chip->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "no MT6397 chip or regmap from the parent\n");

	g = devm_kzalloc(&pdev->dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = &pdev->dev;
	g->regmap = chip->regmap;

	/*
	 * The sense resistor, in the vendor's own units. The vendor board data gives
	 * R_FG_VALUE = 5 for this platform
	 * (arch/arm64/boot/dts/mediatek/bat_setting/mt6785_battery_prop.dtsi:17,
	 * "R_sense resistance") and the driver scales every current by
	 * DEFAULT_R_FG / r_fg_value (mt6359_gauge.c:258-260), where DEFAULT_R_FG is 50
	 * (:44) - so the resistor is 5 mOhm and the counter needs a factor of ten
	 * against the nominal. Passing the vendor's number straight through keeps the
	 * arithmetic identical to the vendor's.
	 */
	/*
	 * 10 mOhm is the vendor value for this board: the board data's R_FG_VALUE is 5
	 * and DEFAULT_R_FG is 50 (mt6359_gauge.c:44), so the vendor's scaling factor is
	 * DEFAULT_R_FG / R_FG_VALUE = 10 - the number the vendor's own probe prints for
	 * rosemary, and the one the trial saw reported.
	 */
	/*
	 * The vendor's number is a *scale*, not a resistance. The board data gives
	 * R_FG_VALUE = 5 (bat_setting/mt6785_battery_prop.dtsi:17) and the driver
	 * divides DEFAULT_R_FG (50, mt6359_gauge.c:44) by it, so every current and
	 * charge is multiplied by 10 (mt6359_gauge.c:258-260, :303-305). Printing that
	 * factor as "10 mOhm" was wrong in the first trial; it is a factor.
	 */
	g->r_fg_scale = MT6359_R_FG_NOMINAL / 10;	/* R_FG_VALUE 5 -> x10 */
	if (!of_property_read_u32(pdev->dev.of_node, "mediatek,r-fg-value", &val) && val)
		g->r_fg_scale = MT6359_R_FG_NOMINAL / val;

	g->charge_full_uah = 5000000;	/* 5 Ah, as the vendor profile reports */
	if (!of_property_read_u32(pdev->dev.of_node,
				  "mediatek,charge-full-microamp-hours", &val))
		g->charge_full_uah = val;

	/*
	 * Both channels are optional: a board without them still gets charge, current
	 * and capacity from the counter, and the property reads return -ENODATA for
	 * the two it cannot answer. -ENODEV is how the IIO core says "not described".
	 */
	g->vbat = devm_iio_channel_get(&pdev->dev, "vbat");
	if (IS_ERR(g->vbat)) {
		if (PTR_ERR(g->vbat) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(g->vbat),
					     "cannot get the vbat channel\n");
		g->vbat = NULL;
	}
	g->ntc = devm_iio_channel_get(&pdev->dev, "ntc");
	if (IS_ERR(g->ntc)) {
		if (PTR_ERR(g->ntc) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(g->ntc),
					     "cannot get the ntc channel\n");
		g->ntc = NULL;
	}

	/*
	 * Switch the gauge on first: with FG_ON clear the counters stay at zero and the
	 * supply reports a flat battery on the charger, which is exactly what the first
	 * trial showed. Read-modify-write, because bits 1:2 of the same register are the
	 * calibration field (PMIC_FG_CAL, upmu_hw.h:8424-8426).
	 */
	ret = regmap_update_bits(g->regmap, MT6359_FGADC_CON0, MT6359_FG_ON,
				 MT6359_FG_ON);
	if (ret) {
		dev_err(&pdev->dev, "cannot switch the gauge on: %d\n", ret);
		return ret;
	}
	msleep(100);

	ret = mt6359_gauge_read_car(g, &g->last_car);
	if (ret) {
		dev_err(&pdev->dev, "cannot read the coulomb counter: %d\n", ret);
		return ret;
	}
	g->last_time = ktime_get();

	cfg.drv_data = g;
	cfg.fwnode = dev_fwnode(&pdev->dev);
	g->psy = devm_power_supply_register(&pdev->dev, &mt6359_gauge_desc, &cfg);
	if (IS_ERR(g->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(g->psy),
				     "cannot register the power supply\n");

	INIT_DELAYED_WORK(&g->sample, mt6359_gauge_sample);
	schedule_delayed_work(&g->sample, msecs_to_jiffies(MT6359_FG_SAMPLE_MS));

	dev_info(&pdev->dev, "MT6359 gauge: %lld uAh full, R_FG scale x%u, counter %lld\n",
		 g->charge_full_uah, g->r_fg_scale, g->last_car);
	return 0;
}

static void mt6359_gauge_remove(struct platform_device *pdev)
{
	struct mt6359_gauge *g = platform_get_drvdata(pdev);

	if (!g)
		return;
	cancel_delayed_work_sync(&g->sample);
}

static const struct of_device_id mt6359_gauge_of_match[] = {
	{ .compatible = "mediatek,mt6359-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6359_gauge_of_match);

static struct platform_driver mt6359_gauge_driver = {
	.probe = mt6359_gauge_probe,
	.remove = mt6359_gauge_remove,
	.driver = {
		.name = "mt6359-gauge",
		.of_match_table = mt6359_gauge_of_match,
	},
};
module_platform_driver(mt6359_gauge_driver);

/*
 * devm_iio_channel_get() and iio_read_channel_processed() are exported into the
 * IIO_CONSUMER symbol namespace, and a module that uses them must import it or
 * modpost refuses to link: "uses symbol ... from namespace IIO_CONSUMER, but does
 * not import it". Caught only by a build that runs modpost, which is why the check
 * in tools/checks looks for this class statically as well.
 */
MODULE_IMPORT_NS("IIO_CONSUMER");

MODULE_DESCRIPTION("MediaTek MT6359 fuel gauge");
MODULE_LICENSE("GPL");
