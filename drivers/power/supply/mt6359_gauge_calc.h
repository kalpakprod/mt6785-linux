/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pure conversion arithmetic of the MT6359 fuel gauge, kept out of the driver so
 * the host can test it without a phone.
 *
 * Every constant is the vendor's, with its source:
 *   - the coulomb counter unit, CHARGE_LSB = 0.085 uAh, and the register
 *     conversion factor 1000*1000/85 = 0x2E14 with a 5-bit shift:
 *     mt6359_gauge.c:53,55,57 (CAR_TO_REG_SHIFT, CAR_TO_REG_FACTOR, UNIT_FGCAR),
 *     mt6359_gauge.c:45 (UNIT_FGCAR_ZCV 85, "CHARGE_LSB = 0.085 uAh");
 *   - the current sense resistor, 50 mOhm nominal, with every current scaled by
 *     DEFAULT_R_FG / r_fg_value: mt6359_gauge.c:44,258-260,303-305;
 *   - the average-current LSB, 305.176 uA: mt6359_gauge.c:41-42 (UNIT_FG_IAVG).
 *
 * A wrong constant here produces a plausible, wrong percentage - which is why these
 * functions have a host test that fails when one of them is changed.
 */
#ifndef __MT6359_GAUGE_CALC_H__
#define __MT6359_GAUGE_CALC_H__

/* Charge of one coulomb-counter LSB, in uAh: 0.085. */
#ifndef MT6359_CHARGE_LSB_NUM
#define MT6359_CHARGE_LSB_NUM	85
#endif
#define MT6359_CHARGE_LSB_DIV	1000

/* Nominal sense resistor, in mOhm (vendor DEFAULT_R_FG). */
#ifndef MT6359_R_FG_NOMINAL
#define MT6359_R_FG_NOMINAL	50
#endif

/*
 * The current path's constant, UNIT_FGCURRENT = 610352 (mt6359_gauge.c:31). The
 * vendor divides it by 100000, multiplies by DEFAULT_R_FG / r_fg_value
 * (:258-260) and then by car_tune_value / 1000 (:264-266), and the result is in
 * units of 0.1 mA. Those two corrections cancel on this board - 50/5 = 10 and
 * 100/1000 = 0.1 - so one counter step is UNIT_FGCURRENT / 1000 = 610.352 uA.
 */
#ifndef MT6359_UNIT_FGCURRENT
#define MT6359_UNIT_FGCURRENT	610352
#endif

/* Average-current LSB in uA * 1000: 305176 means 305.176 uA (the IAVG path). */
#ifndef MT6359_UNIT_FG_IAVG
#define MT6359_UNIT_FG_IAVG	305176
#endif

/*
 * Raw coulomb counter -> charge in uAh. The counter counts LSBs of
 * MT6359_CHARGE_LSB_NUM/1000 uAh, and the sense resistor scales it: a board with a
 * 10 mOhm resistor counts ten times faster for the same charge.
 */
static inline long long mt6359_car_to_uah(long long car, unsigned int r_fg_mohm)
{
	long long uah;

	if (r_fg_mohm == 0)
		r_fg_mohm = MT6359_R_FG_NOMINAL;
	uah = car * MT6359_CHARGE_LSB_NUM;
	uah = (uah * MT6359_R_FG_NOMINAL) / (MT6359_CHARGE_LSB_DIV * r_fg_mohm);
	return uah;
}

/*
 * A current-register value -> uA, the vendor's own arithmetic
 * (mt6359_gauge.c:250-266): raw * UNIT_FGCURRENT / 100000,
 * * DEFAULT_R_FG / r_fg_value, * car_tune_value / 1000, result in 0.1 mA.
 */
static inline long long mt6359_current_to_ua(long long raw, unsigned int r_fg_value,
					     unsigned int car_tune_value)
{
	long long v;

	if (r_fg_value == 0)
		r_fg_value = MT6359_R_FG_NOMINAL;
	v = (raw * MT6359_UNIT_FGCURRENT) / 100000;
	v = (v * MT6359_R_FG_NOMINAL) / r_fg_value;
	v = (v * car_tune_value) / 1000;
	/* The result is in 0.1 mA: -> uA. */
	return v * 100;
}

/*
 * Battery NTC. The MT6359 AUXADC's BAT_TEMP channel (index 1,
 * MT6359_AUXADC_BAT_TEMP, include/dt-bindings/iio/adc/mediatek,mt6359-auxadc.h:8)
 * is the NTC in a divider against RBAT_PULL_UP_R, and the vendor profile pairs it with
 * RBAT_PULL_UP_VOLT 2800 mV and the 10k-beta table whose 25 degC point is 100000 ohm
 * (drivers/misc/mediatek/include/mt-plat/mt6785/include/mach/mtk_battery_table.h:
 * :205 RBAT_PULL_UP_R 100000, :208 RBAT_PULL_UP_VOLT 2800, :213-237 the 23-point
 * Fg_Temperature_Table from -40 to +70 degC). RAW is a 12-bit code.
 */
/* Kept usable outside the kernel: the host test builds this header directly. */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#endif
#ifndef ENODATA
#define ENODATA		61
#endif

#ifndef MT6359_NTC_PULL_UP_OHM
#define MT6359_NTC_PULL_UP_OHM	100000
#endif
#define MT6359_NTC_ADC_MAX	4095

struct mt6359_ntc_point {
	int temp_c;
	long ohm;
};

static const struct mt6359_ntc_point mt6359_ntc_table[] = {
	{ -40, 4251000 }, { -35, 3005000 }, { -30, 2149000 }, { -25, 1554000 },
	{ -20, 1135000 }, { -15, 837800 },  { -10, 624100 },  { -5, 469100 },
	{ 0, 355600 },    { 5, 271800 },    { 10, 209400 },   { 15, 162500 },
	{ 20, 127000 },   { 25, 100000 },   { 30, 79230 },    { 35, 63180 },
	{ 40, 50680 },    { 45, 40900 },    { 50, 33190 },    { 55, 27090 },
	{ 60, 22220 },    { 65, 18320 },    { 70, 15180 },
};

/*
 * A raw BAT_TEMP code -> tenths of a degree Celsius, interpolating the vendor table.
 * The divider is the vendor's: R_NTC = R_pull_up * raw / (full scale - raw).
 */
static inline int mt6359_ntc_raw_to_temp(unsigned int raw)
{
	long ohm;
	size_t i;

	if (raw == 0 || raw >= MT6359_NTC_ADC_MAX)
		return -ENODATA;
	ohm = (MT6359_NTC_PULL_UP_OHM * (long)raw) / (MT6359_NTC_ADC_MAX - (long)raw);

	if (ohm >= mt6359_ntc_table[0].ohm)
		return mt6359_ntc_table[0].temp_c * 10;
	for (i = 1; i < ARRAY_SIZE(mt6359_ntc_table); i++) {
		const struct mt6359_ntc_point *hi = &mt6359_ntc_table[i - 1];
		const struct mt6359_ntc_point *lo = &mt6359_ntc_table[i];

		if (ohm <= hi->ohm && ohm >= lo->ohm) {
			long span = hi->ohm - lo->ohm;
			int dt = (int)((hi->ohm - ohm) * (lo->temp_c - hi->temp_c) / span);

			return hi->temp_c * 10 + dt * 10;
		}
	}
	return mt6359_ntc_table[ARRAY_SIZE(mt6359_ntc_table) - 1].temp_c * 10;
}

/* Charge in uAh -> capacity in percent, clamped to 0..100. */
static inline int mt6359_uah_to_capacity(long long uah, long long full_uah)
{
	long long pct;

	if (full_uah <= 0)
		return 0;
	pct = (uah * 100) / full_uah;
	if (pct < 0)
		return 0;
	if (pct > 100)
		return 100;
	return (int)pct;
}

#endif /* __MT6359_GAUGE_CALC_H__ */
