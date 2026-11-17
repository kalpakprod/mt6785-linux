#!/bin/sh
# check-mfd-cells.sh — every PMIC child in the built DTB must have an MFD cell.
#
#     ./check-mfd-cells.sh [path/to/board.dtb]
#
# Why this exists: the MT6397 MFD core builds the PMIC's children from its own
# mfd_cell table (drivers/mfd/mt6397-core.c), not from arbitrary DT children. A node
# with a compatible and a driver behind it still never probes when its cell is
# missing — that is what happened to the MT6359 clock buffer on conn15: the DT node
# and clk-mt6359-clkbuf.ko were both there, the platform device was not, and st-nci
# deferred with "failed to get the reference clock". The host tests of the time
# checked command lines and userspace scripts, so nothing could see it.
#
# The check is deliberately narrow: it compares the compatibles of the PMIC's DT
# children against the of_compatible strings in the cell table of the same source
# file. It cannot know about cells without of_compatible (the regulator container is
# one) and it does not try.
#
#     --self-test  plants the fault on a copy of the source and requires a failure
set -u

here=$(cd "$(dirname "$0")" && pwd)
kernel=${KERNEL:-$(cd "$here/../.." && pwd)}
dtb=${1:-}
self_test=0
if [ "${1:-}" = --self-test ]; then
	self_test=1
	dtb=${2:-}
fi
core=${MFD_CORE:-$kernel/drivers/mfd/mt6397-core.c}
[ -f "$core" ] || { echo "check: no $core" >&2; exit 2; }

fails=0
# Compatibles of the PMIC children as the DTB has them (the node under pwrap).
dt_children() {
	if [ -z "$dtb" ] || [ ! -f "$dtb" ] || ! command -v fdtget >/dev/null; then
		return 0
	fi
	for node in $(fdtget -l "$dtb" /soc/pwrap@1000d000/pmic 2>/dev/null); do
		compat=$(fdtget "$dtb" "/soc/pwrap@1000d000/pmic/$node" compatible 2>/dev/null |
			 grep '^mediatek,' | tr ' ' '|')
		[ -n "$compat" ] && echo "$node $compat"
	done
}

dt_list=$(dt_children | sort -u)
if [ -z "$dt_list" ] && [ -n "$dtb" ]; then
	echo "check: no PMIC children found in $dtb (path changed?)" >&2
	fails=$((fails + 1))
fi

echo "== DT children versus MFD cells"
# The loop runs in this shell, not in a pipe: its failures have to count.
dt_tmp=$(mktemp "${TMPDIR:-/tmp}/mfd-dt.XXXXXX")
trap 'rm -f "$dt_tmp"' EXIT
printf '%s\n' "$dt_list" > "$dt_tmp"
while IFS= read -r line; do
	node=${line%% *}
	compat=${line#* }
	matched=""
	for c in $(printf '%s' "$compat" | tr '|' ' '); do
		if grep -q "of_compatible = \"$c\"" "$core"; then
			matched=$c
			break
		fi
	done
	if [ -n "$matched" ]; then
		echo "PASS  $node ($compat) has a cell for $matched"
		continue
	fi
	compat=$(printf '%s' "$compat" | tr '|' ' ' | awk '{print $1}')
	# The regulator container is the cell of the same name; its driver looks the
	# child up by node name, so it needs no of_compatible.
	if printf '%s' "$compat" | grep -q -- '-regulator$' &&
	   grep -q "name = \"mt[0-9]*-regulator\"" "$core"; then
		echo "PASS  $compat is served by the regulator cell (name match)"
		continue
	fi
	# A child with no driver anywhere in the tree is somebody else's business: say
	# so, but do not fail on it. A child WITH a driver and no cell never probes -
	# that is the MT6359 clock-buffer failure this check exists for.
	driver=$(grep -rl "\"$compat\"" drivers/ sound/ 2>/dev/null | head -1)
	if [ -z "$driver" ]; then
		echo "INFO  $compat has no cell and no driver in this tree (another area's work)"
	else
		echo "FAIL  $compat is in the DT, has a driver ($driver), and has no cell: it never probes"
		fails=$((fails + 1))
	fi
done < "$dt_tmp"
[ -n "$dt_list" ] || echo "      (no DTB given; skipping the DT side)"

if [ "$self_test" = 1 ]; then
	echo
	echo "== self-test: the MT6359 clock-buffer cell is dropped from a copy of the source"
	work=$(mktemp -d "${TMPDIR:-/tmp}/mfd-cells-test.XXXXXX")
	trap 'rm -rf "$work"' EXIT
	sed '/of_compatible = "mediatek,mt6359-clkbuf"/d' "$core" > "$work/core-without-cell.c"
	# Use the real DTB when the caller gave one, otherwise the tree's own names.
	target=${dtb:-}
	if MFD_CORE="$work/core-without-cell.c" "$0" $target >/dev/null 2>&1; then
		echo "FAIL  planted fault 'clock-buffer cell removed' was not caught"
		fails=$((fails + 1))
	else
		echo "PASS  planted fault 'clock-buffer cell removed' caught"
	fi
fi

echo
echo "$fails check(s) failed"
exit "$fails"
