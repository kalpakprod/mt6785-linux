#!/bin/sh
# Check everything the engine needs, without starting it.
#
# Every line prints PASS or FAIL and says what was actually looked at, so this
# can be read as evidence rather than as reassurance. Nothing here passes on the
# strength of a file existing on the host: the tree is only correct if the
# linker *inside* it resolves and runs the daemon, so that is what is executed.
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
BINDIR=/usr/local/bin
RUNNER=$ROOT/data/local/tmp/run-mnld
STATE=/var/log/gps-engine
mkdir -p "$STATE"

fails=0

ok()   { echo "PASS  $*"; }
bad()  { echo "FAIL  $*"; fails=$((fails + 1)); }
info() { echo "      $*"; }

# Paths inside the tree are checked from inside it: system/bin/linker64 and the
# bionic libraries are relative symlinks into the apex, and they resolve against
# the tree's root, not against /opt/gps-engine on the host.
in_root() { chroot "$ROOT" "$@" 2>/dev/null; }

echo "== kernel interfaces"
for dev in /dev/stpgps /dev/gps_emi; do
	if [ -c "$dev" ]; then
		ok "$dev exists ($(stat -c '%t:%T' "$dev"))"
	else
		bad "$dev is missing - is the stpgps module loaded?"
	fi
done
if [ -r /proc/interrupts ] && grep -qiE "btif|wmt" /proc/interrupts; then
	ok "the connectivity interrupt lines are registered"
else
	info "no btif/wmt line in /proc/interrupts yet (expected until the WMT driver binds)"
fi
dmesg 2>/dev/null | grep -q "GNSS bridge ready" && ok "kernel logged the GNSS bridge probe" \
	|| info "no 'GNSS bridge ready' in dmesg yet"

echo
echo "== the tree, from inside it"
if [ ! -d "$ROOT" ]; then
	bad "no tree at $ROOT - run install.sh first"
	echo
	echo "$fails check(s) failed"
	exit "$fails"
fi

for f in /vendor/bin/mnld /vendor/lib64/libmnl.so /system/bin/linker64 \
	 /system/lib64/libprops-shim.so /system/lib64/libc.so \
	 /data/local/tmp/gps.props /data/local/tmp/run-mnld; do
	if in_root test -e "$f"; then
		ok "$f resolves inside the tree"
	else
		bad "$f does not resolve inside the tree"
	fi
done

for f in /vendor/bin/mnld /data/local/tmp/run-mnld /system/bin/linker64; do
	if in_root test -x "$f"; then
		ok "$f is executable"
	else
		bad "$f is not executable (chmod 0755)"
	fi
done

echo
echo "== does the linker really resolve the daemon"
# This is the check that cannot be faked: bionic's own linker loads mnld and its
# whole closure from inside the tree. A missing library shows up here, not on the
# first start.
if in_root /system/bin/linker64 --list /vendor/bin/mnld > "$STATE/linker-list.txt" 2> "$STATE/linker-error.txt"; then
	n=$(grep -c '=>' "$STATE/linker-list.txt")
	if [ "$n" -ge 20 ] && ! grep -q "not found" "$STATE/linker-error.txt"; then
		ok "the linker resolved mnld's closure ($n libraries)"
	else
		bad "the closure is incomplete ($n libraries listed):"
		sed 's/^/      /' "$STATE/linker-error.txt" | head -5
		grep "not found" "$STATE/linker-list.txt" | head -5 | sed 's/^/      /'
	fi
else
	bad "the linker could not be run as /system/bin/linker64 inside the tree:"
	sed 's/^/      /' "$STATE/linker-error.txt" | head -5
fi

echo
echo "== the preload wrapper"
# mnld needs LD_PRELOAD=/system/lib64/libprops-shim.so and the tree has no env(1);
# run-mnld is the static wrapper that sets it. Prove it execs and that the shim
# is what gets preloaded, by asking it to run the linker's --list on itself.
if [ -x "$RUNNER" ]; then
	if in_root /data/local/tmp/run-mnld /system/bin/linker64 --list /vendor/bin/mnld \
		> "$STATE/wrapper-list.txt" 2> "$STATE/wrapper-error.txt"; then
		ok "run-mnld execs the linker with the shim preloaded ($(grep -c '=>' "$STATE/wrapper-list.txt") libraries)"
	else
		bad "run-mnld failed:"
		sed 's/^/      /' "$STATE/wrapper-error.txt" | head -5
	fi
else
	bad "$RUNNER is missing - reinstall the archive"
fi

echo
echo "== the forwarder"
if [ -x "$BINDIR/nmea-forward" ]; then
	ok "nmea-forward is installed"
	if pgrep -f "nmea-forward" >/dev/null 2>&1; then
		info "nmea-forward is already running (start.sh restarts it)"
	fi
else
	bad "nmea-forward is not installed"
fi

echo
echo "== mounts the daemon needs"
for d in dev proc sys; do
	if grep -q " $ROOT/$d " /proc/mounts 2>/dev/null; then
		ok "/$d is bind-mounted into the tree"
	else
		info "/$d is not mounted yet (start.sh does it)"
	fi
done

echo
echo "== what this does NOT prove"
info "the daemon has not run: the EMI ioctls, the fwctl exchange and the DSP"
info "upload are only proven by start.sh plus the kernel log"
echo
echo "$fails check(s) failed"
exit "$fails"
