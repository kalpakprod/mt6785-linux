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
#
# It is not built here - kexec-tools' source is not in this repository - so build it
# once and drop it in bin/ (recipe as used, kexec-tools 2.0.32 from
# https://cdn.kernel.org/pub/linux/utils/kernel/kexec/kexec-tools-2.0.32.tar.xz,
# checked against kernel.org's sha256sums: 8f81422a...):
#
#   tar xf kexec-tools-2.0.32.tar.xz && cd kexec-tools-2.0.32
#   zig cc -target aarch64-linux-musl -O2 -static -s -D_GNU_SOURCE \
#       -Iutil_lib/include -Iinclude \
#       vmcore-dmesg/vmcore-dmesg.c util_lib/elf_info.c -o vmcore-dmesg
#   cp vmcore-dmesg <this tree>/tools/crash-capture/bin/vmcore-dmesg
#
# Those two source files are the whole tool: no configure, no purgatory, no other
# objects. The binary is not committed (bin/ is ignored); the build needs it.
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
# The archive itself is checked, and the archive is a compressed newc cpio:
# tar cannot read it (a false "not in the archive" the first version printed).
archive_has_vmcore_dmesg() {
	zcat "$1" 2>/dev/null | cpio -t 2>/dev/null |
		grep -qE '^\./vmcore-dmesg$|^vmcore-dmesg$'
}
if archive_has_vmcore_dmesg "$archive"; then
	echo "check: vmcore-dmesg is in the archive"
elif [ "$allow_missing" = 1 ]; then
	echo "check: vmcore-dmesg is NOT in the archive (allowed for this build)"
else
	echo "check: vmcore-dmesg is not in the archive" >&2
	exit 1
fi
# --self-test plants both ways the check can be wrong: no binary at all (the build
# must refuse) and an archive without the member (the check must fail).
if [ "${SELF_TEST:-0}" = 1 ]; then
	echo
	echo "== self-test"
	work=$(mktemp -d "${TMPDIR:-/tmp}/initramfs-selftest.XXXXXX")
	# (a) an archive without the member
	(
		cd "$stage" && rm -f vmcore-dmesg &&
		find . -print0 | cpio --null -o --format=newc 2>/dev/null |
			gzip -9n > "$work/no-member.cpio.gz"
	)
	listed=$(zcat "$work/no-member.cpio.gz" 2>/dev/null | cpio -t 2>/dev/null | grep -c .)
	if [ "$listed" -lt 2 ]; then
		echo "FAIL  the planted archive is not a readable newc cpio ($listed entries)"
	elif archive_has_vmcore_dmesg "$work/no-member.cpio.gz"; then
		echo "FAIL  planted fault 'member missing from the archive' was not caught"
	else
		echo "PASS  planted fault 'member missing from the archive' caught"
	fi
	# (b) no binary to include: the refusal path
	if [ -f "$vmd" ]; then
		mv "$vmd" "$work/vmcore-dmesg.saved"
		if ZIG="$zig" OUT="$work/out2" "$0" >/dev/null 2>&1; then
			echo "FAIL  planted fault 'no bin/vmcore-dmesg' was not caught"
		else
			echo "PASS  planted fault 'no bin/vmcore-dmesg' caught"
		fi
		mv "$work/vmcore-dmesg.saved" "$vmd"
	else
		echo "INFO  no bin/vmcore-dmesg to plant the second fault with"
	fi
	rm -rf "$work"
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
