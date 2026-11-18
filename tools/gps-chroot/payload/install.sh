#!/bin/sh
# Install the vendor GNSS engine and its environment. Runs on the phone, as
# root, from the unpacked tarball:
#
#     sudo sh install.sh
#
# It only places files and checks them. It does not start the engine: starting
# is a separate, explicit command, because what happens next is a vendor binary
# talking to the chip.
set -eu

PREFIX=/opt/gps-engine
BINDIR=/usr/local/bin
STATE=/var/log/gps-engine
here=$(cd "$(dirname "$0")" && pwd)

die() { echo "install: $*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "run as root"
case "$(uname -m)" in aarch64) ;; *) die "this tree is aarch64 only" ;; esac
[ -d "$here/root" ] || die "root/ is missing - is this the whole tarball?"

mkdir -p "$PREFIX" "$BINDIR" "$STATE"
echo "installing the tree into $PREFIX"
rm -rf "$PREFIX/root"
cp -a "$here/root" "$PREFIX/root"

# The engine expects Android's shape: /data writable, /dev, /proc, /sys mounted.
mkdir -p "$PREFIX/root/data/vendor/gps" "$PREFIX/root/data/vendor/log/gps" \
	 "$PREFIX/root/data/agps_supl" "$PREFIX/root/data/local/tmp" \
	 "$PREFIX/root/dev" "$PREFIX/root/proc" "$PREFIX/root/sys" \
	 "$PREFIX/root/tmp"
chmod 1777 "$PREFIX/root/tmp"

echo "installing the two helpers"
install -m 0755 "$here/bin/nmea-forward" "$BINDIR/nmea-forward"
install -m 0644 "$here/lib/libprops-shim.so" \
	"$PREFIX/root/system/lib64/libprops-shim.so"
install -m 0644 "$here/etc/gps.props" "$PREFIX/root/data/local/tmp/gps.props"
install -m 0755 "$here/scripts/start.sh" "$PREFIX/start.sh"
install -m 0755 "$here/scripts/stop.sh" "$PREFIX/stop.sh"
install -m 0755 "$here/scripts/check.sh" "$PREFIX/check.sh"

# Optional: a unit that runs the same start.sh. Installed disabled on purpose.
if [ -d /etc/systemd/system ]; then
	sed "s|@PREFIX@|$PREFIX|g" "$here/gps-engine.service" \
		> /etc/systemd/system/gps-engine.service
	echo "unit written to /etc/systemd/system/gps-engine.service (not enabled)"
fi

echo
echo "files placed. Now the checks:"
echo
exec sh "$PREFIX/check.sh"
