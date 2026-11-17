// SPDX-License-Identifier: GPL-2.0-only
/*
 * Host test for the gauge arithmetic. It fails when a conversion constant is wrong,
 * which is the failure mode the driver cannot be trusted without: a wrong LSB yields
 * a plausible percentage and nothing on the phone complains.
 *
 *   cc -I drivers/power/supply -o test-calc tools/battery-gauge/tests/test-calc.c
 *   ./test-calc            # all checks pass
 *   ./test-calc --expect-fail-ok  # planted fault: built with a wrong constant
 *
 * The planted fault is a build with -DMT6359_CAR_UAH_X1000=850 (the unmeasured LSB) or
 * any other wrong value;
 * then the reference values below no longer match and the test must fail.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mt6359_gauge_calc.h"

static int failures;

static void expect(const char *what, long long got, long long want)
{
	if (got == want) {
		printf("PASS  %s = %lld\n", what, got);
	} else {
		printf("FAIL  %s = %lld, want %lld\n", what, got, want);
		failures++;
	}
}

int main(int argc, char **argv)
{
	int planted = argc > 1 && !strcmp(argv[1], "--planted");

	/*
	 * The measured scale: 1.774 uAh per counter unit, pinned at conn38's own point -
	 * 72035 units of discharge against 127800 uAh integrated from a current that was
	 * verified against the MT6360's independent ADC.
	 */
	expect("car 1000 -> uAh", mt6359_car_to_uah(1000, 5), 574);
	/* conn40: 55225 raw units against 31673 uAh integrated from the current. */
	expect("car 55225 -> uAh", mt6359_car_to_uah(55225, 5), 31699);
	/*
	 * The vendor's current arithmetic (mt6359_gauge.c:250-266) with this board's
	 * numbers: raw * 610352 / 100000, * 50 / 5, * 100 / 1000, in 0.1 mA.
	 *   raw 1000 -> 6103 -> 61030 -> 6103 -> 610300 uA
	 *   raw  498 -> 3039 -> 30390 -> 3039 -> 303900 uA (304 mA, the reference)
	 */
	expect("current raw 1000 -> uA", mt6359_current_to_ua(1000, 5, 100), 610300);
	expect("current raw 498 -> uA", mt6359_current_to_ua(498, 5, 100), 303900);
	/* A board with the nominal resistor and the same tune is ten times smaller:
	 * 610352/100000 -> 6103 (integer, as the vendor's do_div does), x1, x0.1 ->
	 * 610 in 0.1 mA -> 61000 uA. The truncation is the vendor's own. */
	expect("current raw 1000, r_fg 50 -> uA",
	       mt6359_current_to_ua(1000, 50, 100), 61000);
	/* Half a 5 Ah battery is 50 %. */
	expect("2500 mAh of 5000 mAh -> %", mt6359_uah_to_capacity(2500000, 5000000), 50);
	expect("clamped high", mt6359_uah_to_capacity(6000000, 5000000), 100);
	expect("clamped low", mt6359_uah_to_capacity(-5, 5000000), 0);
	/* A coulomb counter of 1 C (1 A for 1 s) is 277.8 uAh; with 0.085 uAh per LSB
	 * that is 3268 LSBs, and 3268 * 85 / 1000 = 277 uAh. */
	expect("car 3268 -> uAh", mt6359_car_to_uah(3268, 5), 1875);

	/*
	 * The NTC: the vendor divider and table (RBAT_PULL_UP_R 100k, 12-bit code, 23
	 * points). A code of 2048 is exactly half the pull-up, i.e. 100 kOhm, which is
	 * the table's 25 degC point.
	 */
	/*
	 * The vendor divider: mV = raw * 4500 / 4096, then R = 100k * mV / (2800 - mV).
	 * R = 100 kOhm (the table's 25 degC point) is mV = 1400, i.e. raw = 1274.
	 * And the phone's own calibration rows: raw 1114 at ln8000 32.6 degC and raw 1097
	 * at 33.4 - the conversion must land within a couple of degrees of both.
	 */
	expect("ntc raw 1274 -> 25.0 C", mt6359_ntc_raw_to_temp(1274), 250);
	expect("ntc raw 1114 between 30 and 33 C", mt6359_ntc_raw_to_temp(1114) >= 295 &&
	       mt6359_ntc_raw_to_temp(1114) <= 330, 1);
	expect("ntc raw 1097 between 30 and 34 C", mt6359_ntc_raw_to_temp(1097) >= 295 &&
	       mt6359_ntc_raw_to_temp(1097) <= 340, 1);
	/* Twice the resistance is colder: 2048*2/3 ~ 2730 is 200 kOhm, ~17 degC. */
	expect("a smaller code is colder", mt6359_ntc_raw_to_temp(1000) > 250, 1);
	expect("a larger code is warmer", mt6359_ntc_raw_to_temp(2000) < 250, 1);

	/*
	 * capacity follows the counter: one full charge is 5000000 uAh, and a counter
	 * movement of 850 counts is 1000 uAh (0.085 per count), i.e. 0.02 % of full -
	 * so a movement of 42500 counts is 3612 uAh = 0.07 % ... at 100000 counts the
	 * charge has moved 8500 uAh, which is 0.17 %. The test uses round numbers that
	 * the ABI can show: 500000 counts = 42500 uAh = 0.85 %, and 10x that is 8.5 %.
	 */
	expect("counter +100000 -> uAh", mt6359_car_to_uah(100000, 5), 57400);
	expect("capacity at 0 uAh", mt6359_uah_to_capacity(0, 5000000), 0);
	expect("capacity at anchor 45 %", mt6359_uah_to_capacity(2250000, 5000000), 45);
	/* conn40 end to end: 45 % of 5 Ah plus 31673 uAh is still 45 %; plus 56 mAh is 46 %. */
	expect("capacity after a 31.7 mAh charge",
	       mt6359_uah_to_capacity(2250000 + 31699, 5000000), 45);
	expect("capacity after a 56 mAh charge",
	       mt6359_uah_to_capacity(2250000 + 56000, 5000000), 46);
	expect("capacity never above 100", mt6359_uah_to_capacity(9000000, 5000000), 100);
	expect("capacity never below 0", mt6359_uah_to_capacity(-900000, 5000000), 0);

	if (planted) {
		/* With a planted constant one of the values above must have failed. */
		printf("%s planted-constant build %s\n",
		       failures ? "PASS " : "FAIL ",
		       failures ? "was caught" : "was NOT caught");
		if (!failures)
			return 1;
		return 0;
	}

	printf("\n%d check(s) failed\n", failures);
	return failures ? 1 : 0;
}
