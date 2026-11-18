#!/bin/sh
# Start the engine: the vendor daemon inside its tree, and the reader that offers
# what it says to gpsd.
#
# Order matters. nmea-forward binds mtk_mnl2hal first, because mnld blocks in
# sendto until somebody listens there; the forwarder also greets the daemon on
# mtk_hal2mnl so it knows a HAL is present. mnld then starts, opens the kernel
# devices and uploads its DSP image.
#
# Running it twice is safe: whatever the previous run left behind is stopped
# first, and the abstract sockets it held are waited for, so a restart never
# dies with "bind: Address in use".
set -eu

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
STATE=/var/log/gps-engine
RUNNER=$ROOT/data/local/tmp/run-mnld
mkdir -p "$STATE"

say() { echo "start: $*"; }

[ "$(id -u)" = 0 ] || { echo "start: run as root" >&2; exit 1; }
[ -x "$PREFIX/stop.sh" ] || { echo "start: $PREFIX/stop.sh is missing - run install.sh first" >&2; exit 1; }
[ -e "$ROOT/vendor/bin/mnld" ] || { echo "start: the engine tree is missing ($ROOT/vendor/bin/mnld)" >&2; exit 1; }
[ -x "$RUNNER" ] || { echo "start: $RUNNER is missing or not executable - reinstall the archive" >&2; exit 1; }

if [ ! -c /dev/stpgps ]; then
	echo "start: /dev/stpgps is missing - is the stpgps module loaded?" >&2
	exit 1
fi

# Stop what a previous run left, and wait for the sockets to fall away. The
# forwarder binds an abstract socket, which lives as long as its process.
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

# The daemon needs the kernel's device nodes and the two pseudo filesystems.
for d in dev proc sys; do
	grep -q " $ROOT/$d " /proc/mounts 2>/dev/null || mount --bind "/$d" "$ROOT/$d" ||
		{ echo "start: cannot bind-mount /$d into the tree" >&2; exit 1; }
done

if [ ! -c "$ROOT/dev/stpgps" ]; then
	echo "start: $ROOT/dev/stpgps does not exist - the bind mount hides it?" >&2
	exit 1
fi

# The forwarder first: mnld blocks in sendto until somebody listens.
: > "$STATE/nmea.log"
setsid /usr/local/bin/nmea-forward >>"$STATE/nmea.log" 2>&1 < /dev/null &
fwd=$!
sleep 1
if ! kill -0 "$fwd" 2>/dev/null; then
	sed -n '1,5p' "$STATE/nmea.log" >&2
	echo "start: nmea-forward exited at once" >&2
	exit 1
fi
say "nmea-forward pid $fwd (mtk_mnl2hal bound)"

# The daemon, through the static wrapper that sets LD_PRELOAD: `chroot` itself
# cannot carry the variable (the host's loader would try to preload a bionic
# library), and the tree has no env(1).
: > "$STATE/mnld.log"
setsid chroot "$ROOT" /data/local/tmp/run-mnld /vendor/bin/mnld \
	>>"$STATE/mnld.log" 2>&1 < /dev/null &
sleep 2
# setsid may or may not fork, so ask for the process rather than trust $!.
mnld=$(pgrep -f "vendor/bin/mnld" | head -1)
if [ -n "$mnld" ]; then
	say "mnld pid $mnld"
else
	sed -n '1,10p' "$STATE/mnld.log" >&2
	echo "start: mnld exited at once (its log is above)" >&2
	exit 1
fi

echo
echo "watch it with:   tail -f $STATE/mnld.log $STATE/nmea.log"
echo "kernel side:     dmesg | grep -iE 'FUNC_CTRL|stpgps|fwctl|GNSS channel'"
echo "stop it with:    $PREFIX/stop.sh"
