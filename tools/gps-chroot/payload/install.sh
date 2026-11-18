#!/bin/sh
# Install the vendor GNSS engine and its environment. Runs on the phone, as
# root, from the unpacked tarball:
#
#     sudo sh install.sh
#
# It only places files and checks them. It does not start the engine: starting
# is a separate, explicit command, because what happens next is a vendor binary
# talking to the chip.
#
# Everything it touches is inside PREFIX. It never chowns, chmods or removes
# anything outside /opt/gps-engine and /usr/local/bin, so extracting the tarball
# into a home directory (or /root) and running it from there cannot disturb that
# directory's permissions.
set -eu

PREFIX=/opt/gps-engine
BINDIR=/usr/local/bin
STATE=/var/log/gps-engine
here=$(cd "$(dirname "$0")" && pwd)

die() { echo "install: $*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "run as root"
case "$(uname -m)" in aarch64) ;; *) die "this tree is aarch64 only" ;; esac
[ -d "$here/root" ] || die "root/ is missing - is this the whole tarball?"
[ -f "$here/bin/nmea-forward" ] || die "bin/nmea-forward is missing"
[ -f "$here/lib/libprops-shim.so" ] || die "lib/libprops-shim.so is missing"
[ -f "$here/bin/run-mnld" ] || die "bin/run-mnld is missing"

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

# Modes, explicitly: everything the engine executes inside the tree has to be
# executable there, whatever mode the archive happened to carry (an mnld at 0644
# is the "chroot: Permission denied" that a first trial hits).
chmod 0755 "$PREFIX/root/vendor/bin/mnld" "$PREFIX/root/vendor/bin/mtk_agpsd"
chmod 0755 "$PREFIX/root/apex/com.android.runtime/bin/"* 2>/dev/null || true
# Any other ELF executable in the tree gets its execute bit too; libraries stay 0644.
for f in $(find "$PREFIX/root" -type f -name '*.so' -prune -o -type f -print); do
	case "$(head -c 4 "$f" 2>/dev/null | od -An -tx1 | tr -d ' \n')" in
	7f454c46) chmod a+x "$f" ;;
	esac
done

echo "installing the helpers"
install -m 0755 "$here/bin/nmea-forward" "$BINDIR/nmea-forward"
install -m 0755 "$here/bin/run-mnld" "$PREFIX/root/data/local/tmp/run-mnld"
install -m 0644 "$here/lib/libprops-shim.so" \
	"$PREFIX/root/system/lib64/libprops-shim.so"
install -m 0644 "$here/etc/gps.props" "$PREFIX/root/data/local/tmp/gps.props"
install -m 0755 "$here/scripts/start.sh" "$PREFIX/start.sh"
install -m 0755 "$here/scripts/stop.sh" "$PREFIX/stop.sh"
install -m 0755 "$here/scripts/check.sh" "$PREFIX/check.sh"

# The unit needs the same wrapper; keep the shipped one in step with the tree.
if [ -d /etc/systemd/system ]; then
	sed "s|@PREFIX@|$PREFIX|g" "$here/gps-engine.service" \
		> /etc/systemd/system/gps-engine.service
	echo "unit written to /etc/systemd/system/gps-engine.service (not enabled)"
fi

echo
echo "files placed. Now the checks:"
echo
exec sh "$PREFIX/check.sh"
