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
