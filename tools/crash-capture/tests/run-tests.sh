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
	# No /proc/cmdline yet: the simulated proc mount creates it, which is what the
	# ordering test below depends on.
	# The capture kernel mounts the phone filesystem before it writes anything; in
	# this test build the attempts are simulated, so a case has to plant the one
	# that succeeds. ext4 read-write is what a real capture boot does.
	[ "$mode" = nomount ] || : > "$root/mount-ok.ext4.rw"
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

# --- 6. the capture kernel's own log is written after the mount
root=$work/kmsg
mkdir -p "$root/dev"
printf 'kernel: Booting Linux on physical CPU 0x0\nkernel: crashkernel reserved\n' > "$root/dev/kmsg"
run_case kmsg
if grep -q "capture kernel log, after mounting" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null &&
   grep -q "Booting Linux on physical CPU" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "the capture kernel's own log is saved after the mount"
else
	fail "the capture kernel's own log is saved after the mount"
fi

# --- 7. planted fault: a self-test boot (no vmcore) must still leave its log and
#        must not pretend a crashed kernel was read
root=$work/selftest
mkdir -p "$root/dev"
printf 'kernel: selftest boot line\n' > "$root/dev/kmsg"
run_case selftest novmcore
if ls "$root"/var/lib/poco/crash/*-selftest.kmsg >/dev/null 2>&1 &&
   grep -q "selftest boot line" "$root"/var/lib/poco/crash/*-selftest.kmsg &&
   ! ls "$root"/var/lib/poco/crash/*.txt >/dev/null 2>&1; then
	pass "a self-test boot leaves its own log and no fake dump"
else
	fail "a self-test boot leaves its own log and no fake dump"
fi

# --- 8. PARTLABEL resolution: the partition appears only after a few polls
#        (UFS probes asynchronously in the capture kernel, which has no udev)
root=$work/partlabel
mkdir -p "$root/sys/class/block" "$root/dev" "$root/proc" "$root/var/lib/poco/crash" "$root/mnt"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'poco.crashdev=PARTLABEL=userdata poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
printf 'kernel: waiting for userdata\n' > "$root/dev/kmsg"
(
	usleep 250000 2>/dev/null || sleep 0.25
	mkdir -p "$root/sys/class/block/mmcblk0p42"
	printf 'MAJOR=179\nMINOR=42\nDEVNAME=mmcblk0p42\nPARTNAME=userdata\n' \
		> "$root/sys/class/block/mmcblk0p42/uevent"
) &
waiter=$!
CRASH_TEST_ROOT=$root CRASH_TEST_TRIES=40 CRASH_TEST_POLL_US=100000 \
	"$work/crash-init" >"$root/log.txt" 2>&1 || true
wait $waiter 2>/dev/null || true
if grep -q "PARTLABEL=userdata resolved to /dev/mmcblk0p42 after" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "a PARTLABEL that appears late is found and logged"
else
	fail "a PARTLABEL that appears late is found and logged"
fi

# --- 9. planted fault: the partition never appears -> resolution fails, nothing
#        is written as a dump, and the reason is in the kernel log
root=$work/nopartlabel
mkdir -p "$root/sys/class/block" "$root/dev" "$root/proc" "$root/var/lib/poco/crash" "$root/mnt"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'poco.crashdev=PARTLABEL=userdata poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
printf 'kernel: no userdata here\n' > "$root/dev/kmsg"
CRASH_TEST_ROOT=$root CRASH_TEST_TRIES=2 CRASH_TEST_POLL_US=1000 \
	"$work/crash-init" >"$root/log.txt" 2>&1 || true
if ! ls "$root"/var/lib/poco/crash/*.txt >/dev/null 2>&1 &&
   grep -q "PARTLABEL=userdata never appeared" "$root/dev/kmsg"; then
	pass "a PARTLABEL that never appears is reported, not guessed"
else
	fail "a PARTLABEL that never appears is reported, not guessed"
fi

# --- 10. mount fallback: f2fs does not match, ext4 read-write must be tried next
root=$work/mountfallback
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.rw"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if [ "$(head -1 "$root/mount-attempts.txt" 2>/dev/null)" = "f2fs rw" ] &&
   grep -q "^ext4 rw$" "$root/mount-attempts.txt" 2>/dev/null &&
   grep -q "mounted .* as ext4" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "a filesystem that does not match is passed over for ext4 read-write"
else
	fail "a filesystem that does not match is passed over for ext4 read-write"
fi

# --- 11. planted fault: no read-write mount works -> the read-only pass is tried
root=$work/mountro
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.ro"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if grep -q "^ext4 ro$" "$root/mount-attempts.txt" 2>/dev/null &&
   grep -q "mounted .* as ext4 read-only" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "an unmatched read-write attempt falls back to read-only and says so"
else
	fail "an unmatched read-write attempt falls back to read-only and says so"
fi

# --- 12. planted fault: nothing mounts -> no dump, and every attempt is recorded
root=$work/nomount
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if [ "$(wc -l < "$root/mount-attempts.txt" 2>/dev/null)" = 8 ] &&
   ! ls "$root"/var/lib/poco/crash/*.txt >/dev/null 2>&1; then
	pass "with no filesystem at all all eight attempts are recorded and nothing is faked"
else
	fail "with no filesystem at all all eight attempts are recorded and nothing is faked"
fi

# --- 13. hold mode: poco.crashhold=1 must not reboot, and must say so
root=$work/hold
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash poco.crashhold=1\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.rw"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if [ -f "$root/crashhold.txt" ] &&
   grep -q "hold=1" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "hold mode blocks for the watchdog instead of rebooting"
else
	fail "hold mode blocks for the watchdog instead of rebooting"
fi

# --- 14. planted fault: the same boot without the flag must take the reboot path
root=$work/nohold
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.rw"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if [ ! -f "$root/crashhold.txt" ] &&
   grep -q "hold=0" "$root"/var/lib/poco/crash/*.kmsg 2>/dev/null; then
	pass "without the flag the capture still takes the reboot path"
else
	fail "without the flag the capture still takes the reboot path"
fi

# --- 15. planted fault: the old order (read the command line before mounting /proc)
#        must fail, because a fresh initramfs has no /proc/cmdline at that point
root=$work/oldorder
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
# The directory is deliberately not the compiled-in default: with the old order the
# template is never read, so nothing may appear in it.
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash-old-order\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.rw"
CRASH_OLD_ORDER=1 CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if ! ls "$root"/var/lib/poco/crash-old-order/* >/dev/null 2>&1; then
	pass "reading the command line before mounting /proc leaves nothing, as it must"
else
	fail "reading the command line before mounting /proc leaves nothing, as it must"
fi

# --- 16. the panic's pstore copy is saved from inside the capture kernel: the copy
#        cannot survive the PSCI reboot, so this is the only chance to read it
root=$work/pstore
mkdir -p "$root/proc" "$root/var/lib/poco/crash" "$root/mnt" "$root/dev" \
	 "$root/sys/fs/pstore"
printf 'ELF fake vmcore\n' > "$root/proc/vmcore"
printf 'kernel: boot\n' > "$root/dev/kmsg"
printf 'Panic#1 Part1\nkernel: the crashed log line\n' > "$root/sys/fs/pstore/dmesg-ramoops-0"
printf 'poco.crashdev=/dev/fake poco.crashdir=/var/lib/poco/crash\n' > "$root/cmdline-template"
: > "$root/mount-ok.ext4.rw"
CRASH_TEST_ROOT=$root "$work/crash-init" >"$root/log.txt" 2>&1 || true
if ls "$root"/var/lib/poco/crash/pstore-dmesg-ramoops-0-* >/dev/null 2>&1 &&
   grep -q "the crashed log line" "$root"/var/lib/poco/crash/pstore-dmesg-ramoops-0-*; then
	pass "the panic's pstore record is saved before the reboot"
else
	fail "the panic's pstore record is saved before the reboot"
fi

echo
echo "$fails case(s) failed"
exit "$fails"
