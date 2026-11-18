#!/bin/sh
# Build the one archive the phone needs.
#
# On the host:  ./deploy.sh [--super <super.img>] [--out <dir>]
# On the phone: sudo sh install.sh      (one command, then it prints the checks)
#
# Nothing here talks to the phone. The archive is assembled from the phone's own
# super image (see assemble.py) plus two small helpers built with zig.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
super=${SUPER:-/path/to/project/poco-m5s/backup-2026-11-14/super.img}
out=${OUT:-$here/out}
zig=${ZIG:-zig}

while [ $# -gt 0 ]; do
	case "$1" in
		--super) super=$2; shift 2 ;;
		--out) out=$2; shift 2 ;;
		-h|--help) sed -n '2,9p' "$0"; exit 0 ;;
		*) echo "deploy: unknown option $1" >&2; exit 2 ;;
	esac
done

[ -f "$super" ] || { echo "deploy: no super image at $super" >&2; exit 1; }
command -v "$zig" >/dev/null || { echo "deploy: zig is not installed" >&2; exit 1; }

work=$out/work
stage=$out/stage
root=$work/root

mkdir -p "$out"
echo "1/4 extracting the engine from $super"
if [ ! -d "$root/vendor/bin" ] || [ ! -f "$root/system/bin/linker64" ]; then
	"$here/assemble.py" --super "$super" --out "$work" > "$out/extract.log"
	echo "    $(tail -1 "$out/extract.log")"
else
	echo "    reusing $root"
fi

echo "2/4 building the helpers"
rm -rf "$stage"
mkdir -p "$stage/bin" "$stage/lib" "$stage/etc" "$stage/scripts"
"$zig" cc -target aarch64-linux-musl -O2 -Wall -Wextra -Werror -static -s \
	-o "$stage/bin/nmea-forward" "$here/payload/src/nmea-forward.c"
"$zig" cc -target aarch64-linux-musl -nostdlib -shared -fPIC -O2 \
	-Wall -Wextra -Werror -o "$stage/lib/libprops-shim.so" \
	"$here/payload/src/props-shim.c"
# Static, because it has to run inside the chroot where the only loader is
# bionic's: it sets LD_PRELOAD for the daemon, which chroot(1) itself cannot do.
"$zig" cc -target aarch64-linux-musl -O2 -Wall -Wextra -Werror -static -s \
	-o "$stage/bin/run-mnld" "$here/payload/src/run-mnld.c"

echo "3/4 staging the tree and the scripts"
cp -a "$root" "$stage/root"

# Modes in the archive, not left to the kernel's umask on extraction: every
# executable the engine runs has to arrive executable. debugfs restores the
# inode mode for most files but not for all of them (mnld arrived as 0644 and
# chroot answered "Permission denied"), so the rule is applied here as well.
for f in "$stage/root/vendor/bin/"* "$stage/bin/"*; do
	if [ -f "$f" ]; then chmod 0755 "$f"; fi
done
for f in $(find "$stage/root" -type f ! -name '*.so' ! -name '*.xml' ! -name '*.props' \
		! -name '*.txt' ! -name '*.sh' ! -name '*.conf'); do
	header=$(head -c 4 "$f" 2>/dev/null | od -An -tx1 | tr -d ' \n')
	if [ "$header" = 7f454c46 ]; then chmod 0755 "$f"; fi
done
chmod 0644 "$stage/root/system/lib64/libprops-shim.so" 2>/dev/null || true
cp "$here/payload/gps.props" "$stage/etc/gps.props"
cp "$here/payload/gps-engine.service" "$stage/gps-engine.service"
cp "$here/payload/install.sh" "$stage/install.sh"
cp "$here/payload/scripts/"*.sh "$stage/scripts/"

echo "3/4b verifying the tree the way check.sh will on the phone"
[ -L "$stage/root/system/bin/linker64" ] || { echo "deploy: system/bin/linker64 is not a symlink" >&2; exit 1; }
qemu=${QEMU:-qemu-aarch64-static}
if command -v "$qemu" >/dev/null; then
	# bionic's own linker, resolving mnld's closure inside the tree. It is the only check
	# that cannot be faked: a missing library, or a runtime library the linker cannot find
	# without an ld.config, fails here rather than on the phone.
	run="$stage/root/apex/com.android.runtime/bin/linker64"
	if ! "$qemu" -L "$stage/root" "$run" --list "$stage/root/vendor/bin/mnld" \
		> "$out/linker-list.txt" 2> "$out/linker-error.txt"; then
		sed -n '1,5p' "$out/linker-error.txt" >&2
		echo "deploy: the runtime closure does not resolve" >&2
		exit 1
	fi
	if grep -q "not found" "$out/linker-error.txt"; then
		grep "not found" "$out/linker-error.txt" | head -5 >&2
		echo "deploy: the runtime closure does not resolve" >&2
		exit 1
	fi
	echo "    $(grep -c '=>' "$out/linker-list.txt") libraries resolved ($out/linker-list.txt)"
else
	echo "    $qemu is not installed: the closure check is left to check.sh on the phone"
fi

echo "4/4 packing"
name=gps-engine-$(date +%Y%m%d-%H%M%S).tar.gz
tar -C "$stage" -czf "$out/$name" .
sha256sum "$out/$name" > "$out/$name.sha256"
cat "$out/$name.sha256"
echo
echo "archive: $out/$name  ($(du -h "$out/$name" | cut -f1))"
echo "on the phone, one command:"
echo "    tar -xzf $name && sudo sh install.sh"
