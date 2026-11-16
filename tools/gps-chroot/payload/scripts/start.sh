#!/bin/sh
# Start the engine: the vendor daemon inside its tree, and the reader that offers
# what it says to gpsd.
#
# Order matters. nmea-forward binds mtk_mnl2hal first, because mnld blocks in
# sendto until somebody listens there; the forwarder also greets the daemon on
# mtk_hal2mnl so it knows a HAL is present.
set -eu

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
STATE=/var/log/gps-engine
mkdir -p "$STATE"

[ -c /dev/stpgps ] || { echo "start: /dev/stpgps is missing" >&2; exit 1; }

# The daemon needs the kernel's device nodes and the two pseudo filesystems.
grep -q " $ROOT/dev " /proc/mounts 2>/dev/null || mount --bind /dev "$ROOT/dev"
grep -q " $ROOT/proc " /proc/mounts 2>/dev/null || mount --bind /proc "$ROOT/proc"
grep -q " $ROOT/sys " /proc/mounts 2>/dev/null || mount --bind /sys "$ROOT/sys"

if pgrep -f "nmea-forward" >/dev/null 2>&1; then
	echo "start: nmea-forward is already running"
else
	nohup /usr/local/bin/nmea-forward >>"$STATE/nmea.log" 2>&1 &
	echo "start: nmea-forward pid $!"
	sleep 1
fi

if grep -q " $ROOT/dev " /proc/mounts 2>/dev/null && [ ! -c "$ROOT/dev/stpgps" ]; then
	echo "start: $ROOT/dev/stpgps does not exist - the bind mount hides it?" >&2
fi

echo "start: launching mnld"
nohup chroot "$ROOT" \
	env LD_PRELOAD=/system/lib64/libprops-shim.so \
	/vendor/bin/mnld >>"$STATE/mnld.log" 2>&1 &
echo "start: mnld pid $!"
echo
echo "watch it with:   tail -f $STATE/mnld.log $STATE/nmea.log"
echo "kernel side:     dmesg | grep -iE 'FUNC_CTRL|OPID\\(3\\) type\\(2\\)|stpgps'"
echo "stop it with:    $PREFIX/stop.sh"
