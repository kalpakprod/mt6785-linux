#!/bin/sh
# Check everything the engine needs, without starting it.
#
# Every line prints PASS or FAIL and says what was actually looked at, so this
# can be read as evidence rather than as reassurance.
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
BINDIR=/usr/local/bin
fails=0
STATE=/var/log/gps-engine
mkdir -p "$STATE"

ok()   { echo "PASS  $*"; }
bad()  { echo "FAIL  $*"; fails=$((fails + 1)); }
info() { echo "      $*"; }

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
if command -v dmesg >/dev/null 2>&1; then
	dmesg | grep -q "GNSS bridge ready" && ok "kernel logged the GNSS bridge probe" \
		|| info "no 'GNSS bridge ready' in dmesg yet"
fi

echo
echo "== the engine tree"
missing=0
for f in vendor/bin/mnld vendor/lib64/libmnl.so system/bin/linker64 \
	 system/lib64/libprops-shim.so data/local/tmp/gps.props; do
	[ -f "$ROOT/$f" ] || { bad "missing $ROOT/$f"; missing=1; }
done
[ "$missing" = 0 ] && ok "every file the runtime needs is in place"

# The real dependency check: bionic's linker resolves the whole closure itself
# and refuses to start when a library is absent. --list prints what it would
# load, which is exactly the question.
if [ -x "$ROOT/system/bin/linker64" ] && [ -f "$ROOT/vendor/bin/mnld" ]; then
	if chroot "$ROOT" /system/bin/linker64 --list /vendor/bin/mnld >"$STATE/linker-list.txt" 2>&1; then
		ok "the linker resolved mnld's whole closure ($(wc -l <"$STATE/linker-list.txt") entries)"
	else
		bad "the linker could not resolve mnld:"
		sed 's/^/      /' "$STATE/linker-list.txt" | head -5
	fi
	rm -f "$STATE/linker-list.txt"
fi

echo
echo "== helpers"
if [ -x "$BINDIR/nmea-forward" ]; then
	ok "nmea-forward is installed"
	if timeout 1 "$BINDIR/nmea-forward" >/dev/null 2>&1; then
		info "nmea-forward exited at once - the socket name may be taken"
	elif [ $? = 124 ]; then
		ok "nmea-forward bound mtk_mnl2hal and waited (as it should)"
	fi
else
	bad "nmea-forward is not installed"
fi
if [ -x /usr/local/bin/stpgps-bridge ]; then
	ok "stpgps-bridge is installed (publishes NMEA to gpsd)"
else
	info "stpgps-bridge is not installed - the NMEA will only be in the log"
fi

echo
echo "== what this does NOT prove"
info "the daemon has not run: the DSP upload, the STP channel and the ioctls"
info "it needs are untested until start.sh is used deliberately."
echo
echo "$fails check(s) failed"
exit "$fails"
