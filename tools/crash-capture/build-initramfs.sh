#!/bin/sh
# Build the kdump capture initramfs for the phone.
#
#     ./build-initramfs.sh [--out DIR]
#
# It contains one static aarch64 binary (crash-init) and nothing else: no shell,
# no busybox, no libraries. That is deliberate - the capture kernel runs after a
# crash, and every extra piece is another thing that can be missing.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
zig=${ZIG:-zig}
out=${OUT:-$here/out}

while [ $# -gt 0 ]; do
	case "$1" in
	--out) out=$2; shift 2 ;;
	-h|--help) sed -n '2,9p' "$0"; exit 0 ;;
	*) echo "build-initramfs: unknown option $1" >&2; exit 2 ;;
	esac
done

command -v "$zig" >/dev/null || { echo "build-initramfs: no zig" >&2; exit 1; }

stage=$out/initramfs
rm -rf "$stage"
mkdir -p "$stage/dev" "$stage/proc" "$stage/sys" "$stage/mnt" "$stage/tmp"

echo "building crash-init (aarch64, static, no libc dependency beyond musl)"
"$zig" cc -target aarch64-linux-musl -O2 -Wall -Wextra -Werror -static -s \
	-o "$stage/init" "$here/crash-init.c"

chmod 0755 "$stage/init"

iso=$(date -u +%Y%m%dT%H%M%SZ)
archive=$out/crash-initramfs-$iso.cpio.gz
(cd "$stage" && find . -print0 | cpio --null -o --format=newc 2>/dev/null | gzip -9n) > "$archive"
sha256sum "$archive" | tee "$archive.sha256"
ls -l "$archive" | awk '{print "archive:", $9, "(" $5 " bytes)"}'
echo
echo "load it on the phone with:"
echo "  kexec -p /boot/Image-capture --initrd=$archive \\"
echo "      --command-line=\"poco.crashdev=/dev/disk/by-partlabel/userdata poco.crashdir=/var/lib/poco/crash\""
