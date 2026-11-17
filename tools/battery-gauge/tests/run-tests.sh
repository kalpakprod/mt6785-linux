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
echo "== planted fault: one wrong constant must fail"
for bad in "-DMT6359_CHARGE_LSB_NUM=86" "-DMT6359_R_FG_NOMINAL=51" "-DMT6359_UNIT_FG_IAVG=305000"; do
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
