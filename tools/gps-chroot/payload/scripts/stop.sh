#!/bin/sh
# Stop the engine and leave the kernel device closed, which powers the GNSS
# function off through the driver's release().
#
# Safe to run when nothing is running: start.sh calls it first, so a restart
# never trips over the previous run's processes or its abstract socket.
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root

stopped=0

# The daemon runs inside the chroot, so its cmdline is /vendor/bin/mnld; the
# wrapper and the chroot parent are matched too.
for pat in "vendor/bin/mnld" "data/local/tmp/run-mnld" "chroot $ROOT"; do
	if pkill -f "$pat" >/dev/null 2>&1; then
		stopped=1
	fi
done
[ "$stopped" = 1 ] && echo "stop: mnld stopped" || echo "stop: mnld was not running"

if pkill -f "/usr/local/bin/nmea-forward" >/dev/null 2>&1; then
	echo "stop: nmea-forward stopped"
else
	echo "stop: nmea-forward was not running"
fi

# Wait for the abstract sockets to fall away with their processes, which is what
# makes the next start's bind succeed.
i=0
while [ $i -lt 30 ]; do
	if ! pgrep -f "vendor/bin/mnld" >/dev/null 2>&1 &&
	   ! pgrep -f "nmea-forward" >/dev/null 2>&1; then
		break
	fi
	i=$((i + 1))
	sleep 0.1
done

# Leave the mounts in place: they are harmless and re-mounting every run is
# noise. Unmount only when asked.
if [ "${1:-}" = "--unmount" ]; then
	for d in dev proc sys; do
		mountpoint -q "$ROOT/$d" && umount "$ROOT/$d" && echo "stop: unmounted $d"
	done
fi

echo "stop: /dev/stpgps should now be closed; a second session without a connsys"
echo "      power cycle returns EIO, so if the next start fails, power-cycle."
