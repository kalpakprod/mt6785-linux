#!/bin/sh
# Tests for crash-init's decision logic, run natively against the same source.
#
#     ./run-tests.sh
#
# The capture kernel's mounts, chroot and reboot are compiled out with
# -DCRASH_TEST, so the binary can be run as a normal user on a fake root: the
# dumpers become small scripts this test plants, and each scenario asks whether
# crash-init picked the right one and said so. Every scenario is a scenario
# because the code can be wrong in that direction - a planted fault that must be
# caught, not a happy path repeated.
set -eu

here=$(cd "$(dirname "$0")/.." && pwd)
cc=${CC:-cc}
work=$(mktemp -d "${TMPDIR:-/tmp}/crash-capture-test.XXXXXX")
trap 'rm -rf "$work"' EXIT

echo "building crash-init for the host (-DCRASH_TEST)"
$cc -DCRASH_TEST -O1 -Wall -Wextra -Werror -o "$work/crash-init" "$here/crash-init.c"

fails=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; fails=$((fails + 1)); }

# One scenario: a fake root with a fake /proc/vmcore, a userdata directory and
# the dumpers the scenario wants. crash-init resolves every path under
# CRASH_TEST_ROOT and writes the dump into <root>/var/lib/poco/crash.
run_case() {
	name=$1
	mode=${2:-}
	root=$work/$name
	mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt"
	[ "$mode" = novmcore ] || printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
	printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' \
		> "$root/proc/cmdline"
	# Discard crash-init's own chatter; the assertions look at the files.
	CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
}

# --- 1. the first dumper exists and works
root=$work/good
mkdir -p "$root/usr/bin"
printf '#!/bin/sh\necho "dmesg from the crashed kernel"\n' > "$root/usr/bin/vmcore-dmesg"
chmod +x "$root/usr/bin/vmcore-dmesg"
run_case good
if ls "$root"/var/lib/poco/crash/*.txt >/dev/null 2>&1 &&
   grep -q "dmesg from the crashed kernel" "$root"/var/lib/poco/crash/*.txt; then
	pass "first dumper used when it works"
else
	fail "first dumper used when it works"
fi

# --- 2. planted fault: the first dumper is there but exits 1
root=$work/exits
mkdir -p "$root/usr/bin"
printf '#!/bin/sh\nexit 1\n' > "$root/usr/bin/vmcore-dmesg"
chmod +x "$root/usr/bin/vmcore-dmesg"
printf '#!/bin/sh\necho "second dump"\n' > "$root/usr/bin/makedumpfile"
chmod +x "$root/usr/bin/makedumpfile"
run_case exits
if grep -q "second dump" "$root"/var/lib/poco/crash/*.txt 2>/dev/null; then
	pass "a dumper that fails is passed over for the next one"
else
	fail "a dumper that fails is passed over for the next one"
fi

# --- 3. planted fault: the dumper exits 0 but writes nothing
root=$work/empty
mkdir -p "$root/usr/bin"
printf '#!/bin/sh\nexit 0\n' > "$root/usr/bin/vmcore-dmesg"
chmod +x "$root/usr/bin/vmcore-dmesg"
printf '#!/bin/sh\necho "fallback after empty"\n' > "$root/usr/bin/makedumpfile"
chmod +x "$root/usr/bin/makedumpfile"
run_case empty
if grep -q "fallback after empty" "$root"/var/lib/poco/crash/*.txt 2>/dev/null; then
	pass "an empty dump counts as a failure"
else
	fail "an empty dump counts as a failure"
fi

# --- 4. planted fault: no dumper at all -> the notes fallback, and it says so
root=$work/none
run_case none
if grep -q "no vmcore-dmesg and no makedumpfile" "$root"/var/lib/poco/crash/*.txt 2>/dev/null &&
   grep -q "^method=notes-fallback" "$root"/var/lib/poco-crash-method.tmp 2>/dev/null; then
	pass "no dumper leaves the notes fallback plus a marker"
else
	fail "no dumper leaves the notes fallback plus a marker"
fi

# --- 5. no /proc/vmcore: a non-crash boot must not claim a capture
root=$work/novmcore
run_case novmcore novmcore
if ls "$root"/var/lib/poco/crash/*.txt >/dev/null 2>&1; then
	fail "a boot without /proc/vmcore writes no dump"
else
	pass "a boot without /proc/vmcore writes no dump"
fi

echo
echo "$fails case(s) failed"
exit "$fails"
