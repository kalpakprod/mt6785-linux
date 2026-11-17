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
	/* 1000 mA average current = 1000 * 305176 / 1000 = 305176 uA. */
	expect("iavg 1000 -> uA", mt6359_iavg_to_ua(1000, 50), 305176);
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
