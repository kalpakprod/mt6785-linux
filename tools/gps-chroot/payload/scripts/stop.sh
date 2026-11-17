#!/bin/sh
# Stop the engine and leave the kernel device closed, which powers the GNSS
# function off through the driver's release().
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root

pkill -f "chroot $ROOT" >/dev/null 2>&1 && echo "stop: mnld stopped" || echo "stop: mnld was not running"
pkill -f "/usr/local/bin/nmea-forward" >/dev/null 2>&1 && echo "stop: nmea-forward stopped" \
	|| echo "stop: nmea-forward was not running"

# Leave the mounts in place: they are harmless and re-mounting every run is
# noise. Unmount only when asked.
if [ "${1:-}" = "--unmount" ]; then
	for d in dev proc sys; do
		mountpoint -q "$ROOT/$d" && umount "$ROOT/$d" && echo "stop: unmounted $d"
	done
fi

echo "stop: /dev/stpgps should now be closed; a second session without a connsys"
echo "      power cycle returns EIO, so if the next start fails, power-cycle."
