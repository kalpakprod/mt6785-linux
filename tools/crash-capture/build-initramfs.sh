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

# vmcore-dmesg: the crashed kernel's log is the one thing this initramfs could not
# produce on the first real capture ("no vmcore-dmesg and no makedumpfile on this
# root"), because the phone has no kexec-tools. A static aarch64 build of
# kexec-tools' vmcore-dmesg goes into the archive and crash-init runs it first.
# It is not built here: kexec-tools' source is not in this repository and this
# environment has no network. Build it once and drop it in bin/:
#
#   curl -LO https://cdn.kernel.org/pub/linux/utils/kernel/kexec/kexec-tools-2.0.32.tar.xz
#   tar xf kexec-tools-2.0.32.tar.xz
#   ./configure --host=aarch64-linux-musl CC="zig cc -target aarch64-linux-musl" \
#       --without-xen --without-zlib --without-lzma --without-lzo --without-booke
#   make -C kexec-tools-2.0.32/purgatory -j
#   make -C kexec-tools-2.0.32 vmcore-dmesg/vmcore-dmesg
#   cp kexec-tools-2.0.32/vmcore-dmesg/vmcore-dmesg \
#      tools/crash-capture/bin/vmcore-dmesg
#
# The archive must carry it: the check below fails without it unless
# --allow-missing-vmcore-dmesg is passed for a diagnostic build.
allow_missing=0
for a in "$@"; do
	[ "$a" = --allow-missing-vmcore-dmesg ] && allow_missing=1
done
vmd=$here/bin/vmcore-dmesg

stage=$out/initramfs
rm -rf "$stage"
mkdir -p "$stage/dev" "$stage/proc" "$stage/sys" "$stage/mnt" "$stage/tmp"

echo "building crash-init (aarch64, static, no libc dependency beyond musl)"
"$zig" cc -target aarch64-linux-musl -O2 -Wall -Wextra -Werror -static -s \
	-o "$stage/init" "$here/crash-init.c"

chmod 0755 "$stage/init"

if [ -f "$vmd" ]; then
	install -m 0755 "$vmd" "$stage/vmcore-dmesg"
	echo "included vmcore-dmesg: $(file -b "$vmd" | cut -c1-60)"
elif [ "$allow_missing" = 1 ]; then
	echo "WARNING: no bin/vmcore-dmesg - this archive cannot save the crashed kernel's dmesg"
else
	echo "build-initramfs: bin/vmcore-dmesg is missing and crash-init needs it." >&2
	echo "  The crashed kernel's dmesg is the point of the capture; see the recipe above." >&2
	echo "  Pass --allow-missing-vmcore-dmesg only for a diagnostic build." >&2
	exit 1
fi

iso=$(date -u +%Y%m%dT%H%M%SZ)
archive=$out/crash-initramfs-$iso.cpio.gz
(cd "$stage" && find . -print0 | cpio --null -o --format=newc 2>/dev/null | gzip -9n) > "$archive"
# The archive itself is checked: the check fails when vmcore-dmesg is not inside.
if tar -tzf "$archive" 2>/dev/null | grep -qE '^\./vmcore-dmesg$|^vmcore-dmesg$'; then
	echo "check: vmcore-dmesg is in the archive"
elif [ "$allow_missing" = 1 ]; then
	echo "check: vmcore-dmesg is NOT in the archive (allowed for this build)"
else
	echo "check: vmcore-dmesg is not in the archive" >&2
	exit 1
fi
sha256sum "$archive" | tee "$archive.sha256"
ls -l "$archive" | awk '{print "archive:", $9, "(" $5 " bytes)"}'
echo
echo "load it on the phone with:"
echo "  kexec -p /boot/Image-capture --initrd=$archive \\"
echo "      --command-line=\"poco.crashdev=PARTLABEL=userdata poco.crashdir=/var/lib/poco/crash\""
echo
echo "PARTLABEL=, not /dev/disk/by-partlabel/...: the capture kernel has a bare"
echo "devtmpfs and no udev, so those symlinks do not exist there."
