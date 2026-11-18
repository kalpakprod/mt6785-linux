#!/bin/sh
# Stop the engine: the daemon, the forwarder, and the mounts start.sh put inside
# the tree. Leaving the mounts behind is what made a later install.sh delete
# across a live /proc, so taking them down is part of stopping, not an option.
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
. "$PREFIX/lib.sh"

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

# Wait for the processes to fall away with their sockets, so the next start's
# bind succeeds.
i=0
while [ $i -lt 30 ]; do
	if ! pgrep -f "vendor/bin/mnld" >/dev/null 2>&1 &&
	   ! pgrep -f "nmea-forward" >/dev/null 2>&1; then
		break
	fi
	i=$((i + 1))
	sleep 0.1
done

if [ "${1:-}" = "--keep-mounted" ]; then
	echo "stop: mounts left in place (--keep-mounted)"
	exit 0
fi

# The engine is gone, so its mounts go too. Lazy where a process still holds
# one; the helper reports anything that survives.
if unmount_tree; then
	echo "stop: /dev, /proc and /sys are no longer mounted in the tree"
else
	echo "stop: some mounts could not be taken down (see above)" >&2
	exit 1
fi

echo "stop: /dev/stpgps is closed again; a second session without a connsys"
echo "      power cycle returns EIO, so if the next start fails, power-cycle."
