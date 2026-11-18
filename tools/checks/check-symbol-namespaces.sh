#!/bin/sh
# check-symbol-namespaces.sh — a module that uses a namespaced export must import it.
#
#   ./check-symbol-namespaces.sh [file.c ...]
#
# modpost enforces this at link time ("mt6359_gauge.ko: module uses symbol
# devm_iio_channel_get from namespace IIO_CONSUMER, but does not import it"), but an
# out-of-tree compile here cannot run modpost against the kit's symbol table, so the
# class was missed and cost a build. This check does it statically: for every
# EXPORT_SYMBOL_NS(name, NS) in the tree, if a file calls name( and does not carry
# MODULE_IMPORT_NS("NS"), it fails.
#
#   --self-test  plants the fault on a copy and requires the failure
set -u
here=$(cd "$(dirname "$0")" && pwd)
kernel=$(cd "$here/../.." && pwd)
self_test=0
[ "${1:-}" = --self-test ] && self_test=1 && shift
files=$*
if [ -z "$files" ]; then
	for f in "$kernel"/drivers/power/supply/mt63*.c; do
		[ -f "$f" ] && files="$files $f"
	done
fi

fails=0
echo "== namespaced exports used without MODULE_IMPORT_NS"
for f in $files; do
	[ -f "$f" ] || continue
	# name<TAB>NS for every namespaced export in the tree
	# name<TAB>NS for every namespaced export in the tree. awk rather than a
	# regular expression: the namespaces are quoted, and the GPL variant has its
	# own macro name.
	grep -rh "EXPORT_SYMBOL_NS" "$kernel/include" "$kernel/kernel" \
		"$kernel/drivers" "$kernel/fs" "$kernel/net" 2>/dev/null |
		sed 's/.*EXPORT_SYMBOL_NS[A-Z_]*(//; s/).*//; s/"//g' |
		awk -F',[[:space:]]*' '{ if (NF >= 2) print $1, $2 }' |
		sort -u > /tmp/ns-exports.$$
	while read -r sym ns; do
		[ -n "$sym" ] || continue
		grep -q "[^_a-zA-Z0-9]$sym(" "$f" 2>/dev/null || continue
		if grep -q "MODULE_IMPORT_NS(\"$ns\")" "$f"; then
			continue
		fi
		echo "FAIL  $(basename "$f"): uses $sym from namespace $ns without MODULE_IMPORT_NS(\"$ns\")"
		fails=$((fails + 1))
	done < /tmp/ns-exports.$$
	rm -f /tmp/ns-exports.$$
	if [ "$fails" = 0 ]; then
		:
	fi
done
[ "$fails" = 0 ] && echo "PASS  every namespaced export used is imported"

if [ "$self_test" = 1 ]; then
	echo
	echo "== self-test: the IIO_CONSUMER import removed from a copy"
	work=$(mktemp -d "${TMPDIR:-/tmp}/ns-check.XXXXXX")
	planted=$kernel/drivers/power/supply/mt6359_gauge.c
	for f in $files; do
		[ -f "$f" ] && grep -q 'MODULE_IMPORT_NS("IIO_CONSUMER")' "$f" && { planted=$f; break; }
	done
	sed 's/^MODULE_IMPORT_NS("IIO_CONSUMER");$//' "$planted" > "$work/no-import.c"
	# The planted file keeps the calls; give it a name the real check will scan.
	if "$0" "$work/no-import.c" >/dev/null 2>&1; then
		echo "FAIL  planted fault 'namespace import removed' was not caught"
		fails=$((fails + 1))
	else
		echo "PASS  planted fault 'namespace import removed' caught"
	fi
	rm -rf "$work"
fi

echo
echo "$fails check(s) failed"
exit "$fails"
