#!/bin/sh
# Start the engine: the vendor daemon inside its tree, and the reader that offers
# what it says to gpsd.
#
# Order matters. nmea-forward binds mtk_mnl2hal first, because mnld blocks in
# sendto until somebody listens there; the forwarder also greets the daemon on
# mtk_hal2mnl so it knows a HAL is present. mnld then starts, opens the kernel
# devices and uploads its DSP image.
#
# Running it twice is safe: whatever the previous run left is stopped first
# (processes and mounts), and the logs of this run go to their own files, so an
# old run's errors can never be read as this run's.
set -eu

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
STATE=/var/log/gps-engine
RUNNER=$ROOT/data/local/tmp/run-mnld
. "$PREFIX/lib.sh"

say() { echo "start: $*"; }

[ "$(id -u)" = 0 ] || { echo "start: run as root" >&2; exit 1; }
[ -x "$PREFIX/stop.sh" ] || { echo "start: $PREFIX/stop.sh is missing - run install.sh first" >&2; exit 1; }
[ -e "$ROOT/vendor/bin/mnld" ] || { echo "start: the engine tree is missing ($ROOT/vendor/bin/mnld)" >&2; exit 1; }
[ -x "$RUNNER" ] || { echo "start: $RUNNER is missing or not executable - reinstall the archive" >&2; exit 1; }

if [ ! -c /dev/stpgps ]; then
	echo "start: /dev/stpgps is missing - is the stpgps module loaded?" >&2
	exit 1
fi

# The connsys MCU has to be booted by Wi-Fi before the GPS function can be
# switched on. A GPS-first FUNC_CTRL makes the WMT core's power-on sequence poll
# for an MCU that never boots ("connector MCU did not boot: 0x18002600=0x0",
# "get hwcode (chip id) fail"), and after that failure every later attempt fails
# too, until the phone is rebooted. With Wi-Fi up first the same call answers
# "OPID(3) type(2) ok". See REPORT.md, LEAD-NOTE 30, for the code path.
#
# So: bring Wi-Fi on first - once per boot, and only if nobody did it already.
if [ "${GPS_ENGINE_SKIP_WIFI:-0}" != 1 ] && [ ! -d /sys/class/net/wlan0 ]; then
	echo "start: bringing Wi-Fi up first - the connsys MCU boots on Wi-Fi, not on GPS"
	modprobe wmt_drv >/dev/null 2>&1 || true
	modprobe wlan_gen4m >/dev/null 2>&1 || true
	if [ -w /dev/wmtWifi ]; then
		printf 1 > /dev/wmtWifi 2>/dev/null || true
	fi
	i=0
	while [ $i -lt 100 ]; do
		[ -d /sys/class/net/wlan0 ] && break
		i=$((i + 1))
		sleep 0.1
	done
	if [ -d /sys/class/net/wlan0 ]; then
		echo "start: wlan0 is up, the connsys is booted"
	else
		echo "start: wlan0 did not appear; the GPS function will probably fail" >&2
		echo "       (bring Wi-Fi up with conn-up.sh wifi, then run start.sh again)" >&2
	fi
fi

mkdir -p "$STATE"
stamp=$(date +%Y%m%d-%H%M%S)
MNLD_LOG=$STATE/mnld-$stamp.log
NMEA_LOG=$STATE/nmea-$stamp.log
ln -sf "$MNLD_LOG" "$STATE/mnld.log"
ln -sf "$NMEA_LOG" "$STATE/nmea.log"

# Whatever the previous run left: processes first (stop.sh takes the mounts down
# too), then a sweep of our own in case stop.sh could not.
"$PREFIX/stop.sh" >/dev/null 2>&1 || true
i=0
while [ $i -lt 30 ]; do
	if ! pgrep -f "nmea-forward" >/dev/null 2>&1 &&
	   ! pgrep -f "vendor/bin/mnld" >/dev/null 2>&1; then
		break
	fi
	i=$((i + 1))
	sleep 0.1
done
if pgrep -f "nmea-forward" >/dev/null 2>&1; then
	echo "start: an old nmea-forward is still running and holds mtk_mnl2hal" >&2
	exit 1
fi
if ! unmount_tree; then
	echo "start: old mounts are in the way; not starting" >&2
	exit 1
fi

# The daemon needs the kernel's device nodes and the two pseudo filesystems.
for d in dev proc sys; do
	mount --bind "/$d" "$ROOT/$d" ||
		{ echo "start: cannot bind-mount /$d into the tree" >&2; exit 1; }
done
if [ ! -c "$ROOT/dev/stpgps" ]; then
	echo "start: $ROOT/dev/stpgps does not exist - the bind mount hides it?" >&2
	"$PREFIX/stop.sh" --keep-mounted >/dev/null 2>&1 || true
	exit 1
fi

# The forwarder first: mnld blocks in sendto until somebody listens.
setsid /usr/local/bin/nmea-forward >>"$NMEA_LOG" 2>&1 < /dev/null &
sleep 1
if ! pgrep -f "/usr/local/bin/nmea-forward" >/dev/null 2>&1; then
	sed -n '1,5p' "$NMEA_LOG" >&2
	echo "start: nmea-forward did not stay up (its log is above)" >&2
	exit 1
fi
say "nmea-forward is up, mtk_mnl2hal bound (log $NMEA_LOG)"

# The daemon, through the static wrapper that sets LD_PRELOAD: `chroot` itself
# cannot carry the variable (the host's loader would try to preload a bionic
# library), and the tree has no env(1).
setsid chroot "$ROOT" /data/local/tmp/run-mnld /vendor/bin/mnld \
	>>"$MNLD_LOG" 2>&1 < /dev/null &
sleep 2
mnld=$(pgrep -f "vendor/bin/mnld" | head -1)
if [ -n "$mnld" ]; then
	say "mnld is up, pid $mnld (log $MNLD_LOG)"
else
	sed -n '1,10p' "$MNLD_LOG" >&2
	echo "start: mnld did not stay up (its log is above)" >&2
	"$PREFIX/stop.sh" || true
	exit 1
fi

echo
echo "this run's logs:  $MNLD_LOG"
echo "                  $NMEA_LOG"
echo "watch it with:    tail -f $MNLD_LOG $NMEA_LOG"
echo "kernel side:      dmesg | grep -iE 'FUNC_CTRL|stpgps|fwctl|GNSS channel'"
echo "stop it with:     $PREFIX/stop.sh"
