#!/bin/sh
# Host tests for the MT6359 gauge arithmetic, both ways.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
kernel=$(cd "$here/../../.." && pwd)
inc=$kernel/drivers/power/supply
cc=${CC:-cc}
out=$(mktemp -d "${TMPDIR:-/tmp}/gauge-test.XXXXXX")
trap 'rm -rf "$out"' EXIT

fails=0
echo "== the committed constants"
if $cc -I "$inc" -o "$out/t" "$here/test-calc.c" && "$out/t" >/dev/null; then
	echo "PASS  the conversion constants reproduce the reference values"
else
	echo "FAIL  the conversion constants do not reproduce the reference values"
	fails=$((fails + 1))
fi

echo
echo "== the NTC conversion is pinned by exact values, not by a planted constant"
if $cc "-DMT6359_NTC_PULL_UP_OHM=200000" -I "$inc" -o "$out/tn" "$here/test-calc.c" &&
   ! "$out/tn" >/dev/null 2>&1; then
	echo "PASS  a doubled pull-up fails the exact NTC points"
else
	echo "FAIL  a doubled pull-up was not caught by the exact NTC points"
	fails=$((fails + 1))
fi

echo
echo "== planted fault: one wrong constant must fail"
for bad in "-DMT6359_CAR_UAH_X1000=850" "-DMT6359_CAR_UAH_X1000=1775" \
	   "-DMT6359_UNIT_FGCURRENT=305176"; do
	if $cc $bad -I "$inc" -o "$out/tb" "$here/test-calc.c" &&
	   ! "$out/tb" >/dev/null 2>&1; then
		echo "PASS  planted fault '$bad' caught"
	else
		echo "FAIL  planted fault '$bad' was not caught"
		fails=$((fails + 1))
	fi
done

echo
echo "$fails check(s) failed"
exit "$fails"
