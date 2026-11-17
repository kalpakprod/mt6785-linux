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

/* Average-current LSB in uA * 1000: 305176 means 305.176 uA. */
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

/* Average current register -> uA. */
static inline long long mt6359_iavg_to_ua(long long iavg, unsigned int r_fg_mohm)
{
	if (r_fg_mohm == 0)
		r_fg_mohm = MT6359_R_FG_NOMINAL;
	return (iavg * MT6359_UNIT_FG_IAVG) / 1000;
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
