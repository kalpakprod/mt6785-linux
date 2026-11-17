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
 * The planted fault is a build with -DMT6359_CHARGE_LSB_NUM=86 (or any wrong value);
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

	/* 1000 LSBs of charge at the nominal 50 mOhm: 85 uAh. */
	expect("car 1000 -> uAh", mt6359_car_to_uah(1000, 50), 85);
	/* The same counter on a 10 mOhm board counts five times more charge. */
	expect("car 1000 at 10 mOhm -> uAh", mt6359_car_to_uah(1000, 10), 425);
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
	expect("car 3268 -> uAh", mt6359_car_to_uah(3268, 50), 277);

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
