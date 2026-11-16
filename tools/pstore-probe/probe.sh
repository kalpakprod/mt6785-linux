#!/bin/sh
# pstore-probe - answer one question on the phone: is the pstore region
# protected against our writes, or are we writing somewhere else?
#
# Run as root on the phone:
#
#     sudo sh probe.sh [--region 0x4d010000] [--size 0xe0000]
#
# It does two things:
#   1. writes one byte through /dev/mem into a scratch offset of the pstore
#      region (the top of the region, far above pstore's ring) and reads it
#      back. If the byte does not come back, the memory refuses our writes and
#      the ramoops mapping is innocent; if it does, the fault is in the kernel's
#      own mapping of the region.
#   2. dumps the SoC's EMI MPU block (0x10226000 by the loader's device tree)
#      and scans it for page-aligned start/end pairs that bracket the region,
#      printing their offsets. The register layout is decoded by eye or with the
#      vendor driver: this script does not pretend to know the stride.
#
# Nothing else is touched, no service is stopped, and the one byte written is
# above everything pstore uses.
set -eu

REGION=0x4d010000
SIZE=0xe0000
MPU=0x10226000
# The loader's own device tree gives the EMI MPU block 0x1000 bytes, while the
# same block is declared 0x2000 on mt6795 upstream and 0x1000 on mt6739 in the
# trees in this repo. Dump the larger size: it costs one page read and it cannot
# hide a region behind the end of the smaller window.
MPU_SIZE=0x2000
OUT=/var/log/pstore-probe

MODE=all
while [ $# -gt 0 ]; do
	case "$1" in
	--retention-write) MODE=retention-write; shift ;;
	--retention-check) MODE=retention-check; shift ;;
	--expdb) MODE=expdb; shift ;;
	--region) REGION=$2; shift 2 ;;
	--size) SIZE=$2; shift 2 ;;
	--mpu) MPU=$2; shift 2 ;;
	--mpu-size) MPU_SIZE=$2; shift 2 ;;
	-h|--help) sed -n '2,25p' "$0"; exit 0 ;;
	*) echo "probe: unknown option $1" >&2; exit 2 ;;
	esac
done

[ "$(id -u)" = 0 ] || { echo "probe: run as root" >&2; exit 1; }
[ -e /dev/mem ] || { echo "probe: no /dev/mem (CONFIG_DEVMEM?)" >&2; exit 1; }

mkdir -p "$OUT"
scratch=$((REGION + SIZE - 16))
echo "region   $REGION .. $((REGION + SIZE - 1))"
echo "scratch  $scratch (one byte, above everything pstore uses)"
echo "mpu      $MPU .. $((MPU + MPU_SIZE - 1))"

# Retention modes: write a marker before a reset and look for it after. The
# question they answer is whether this phone keeps DRAM across a reset at all -
# the measured answer so far is no, and then ramoops cannot work however it is
# configured, and the crash log has to live in flash (see REPORT.md).
if [ "$MODE" = retention-write ]; then
	off1=$((REGION + SIZE - 16))
	off2=$((REGION + SIZE - 32))
	printf '\x9e\xfe\xe7\xff' | dd of=/dev/mem bs=1 seek=$off1 count=4 conv=notrunc 2>/dev/null
	printf '\x5a\x5a\xa5\xa5' | dd of=/dev/mem bs=1 seek=$off2 count=4 conv=notrunc 2>/dev/null
	echo "markers written at $off1 (0x9efee7ff) and $off2 (0x5a5aa5a5)"
	echo "now reboot (clean or watchdog) and run: sh probe.sh --retention-check"
	exit 0
fi

if [ "$MODE" = retention-check ]; then
	off1=$((REGION + SIZE - 16))
	off2=$((REGION + SIZE - 32))
	v1=$(dd if=/dev/mem bs=4 skip=$((off1 / 4)) count=1 2>/dev/null | od -An -tx4 | tr -d ' \n')
	v2=$(dd if=/dev/mem bs=4 skip=$((off2 / 4)) count=1 2>/dev/null | od -An -tx4 | tr -d ' \n')
	echo "at $off1: $v1 (want 9efee7ff)"
	echo "at $off2: $v2 (want 5a5aa5a5)"
	if [ "$v1" = 9efee7ff ] || [ "$v2" = 5a5aa5a5 ]; then
		echo "VERDICT DRAM was retained across the reset: ramoops can work."
		echo "        If /sys/fs/pstore is still stale, the fault is the restore order"
		echo "        (something rewriting the region at boot), not the reset."
	else
		echo "VERDICT DRAM was lost across the reset: ramoops cannot survive it,"
		echo "        whatever its mapping or its DT says. The log has to be in flash"
		echo "        before the reset, or the reset has to stop clearing DRAM."
	fi
	exit 0
fi

if [ "$MODE" = expdb ]; then
	part=/dev/disk/by-partlabel/expdb
	[ -e "$part" ] || { echo "probe: no $part" >&2; exit 1; }
	dd if="$part" of="$OUT/expdb-head.bin" bs=64k count=4 2>/dev/null || true
	echo "expdb head written to $OUT/expdb-head.bin"
	grep -abo "SYS_PSTORE_RAW" "$OUT/expdb-head.bin" 2>/dev/null | head -3 || \
		echo "no SYS_PSTORE_RAW in the first 256 KiB (the loader may keep it deeper)"
	echo "the section ids the loader uses are in poco-m5s/evidence/try-force1/expdb.strings"
	exit 0
fi

# One byte, written twice with a different value, read back after each write.
# Two values are needed: a single read could match by luck on stale data.
before=$(dd if=/dev/mem bs=1 skip="$scratch" count=1 2>/dev/null | od -An -tu1 | tr -d ' \n')
printf '\x5a' | dd of=/dev/mem bs=1 seek="$scratch" count=1 conv=notrunc 2>/dev/null
after1=$(dd if=/dev/mem bs=1 skip="$scratch" count=1 2>/dev/null | od -An -tu1 | tr -d ' \n')
printf '\xa5' | dd of=/dev/mem bs=1 seek="$scratch" count=1 conv=notrunc 2>/dev/null
after2=$(dd if=/dev/mem bs=1 skip="$scratch" count=1 2>/dev/null | od -An -tu1 | tr -d ' \n')

echo
echo "was $before, wrote 90 then 165, read back $after1 then $after2"
if [ "$after1" = 90 ] && [ "$after2" = 165 ]; then
	echo "VERDICT the region accepts our writes: the pstore problem is NOT protection -"
	echo "        look at the kernel side (ramoops mapping, mem_type, DT node)."
else
	echo "VERDICT our writes do not land in that memory: it is protected (or aliased),"
	echo "        and no ramoops setting can fix it until the protection is lifted."
fi

echo
echo "dumping the EMI MPU block to $OUT/mpu.bin ($((MPU_SIZE / 1024)) KiB)"
dd if=/dev/mem of="$OUT/mpu.bin" bs=1024 skip=$((MPU / 1024)) count=$((MPU_SIZE / 1024)) 2>/dev/null || true
ls -l "$OUT/mpu.bin" 2>/dev/null || echo "probe: could not read $MPU"

python3 - "$OUT/mpu.bin" "$REGION" <<'PY' 2>/dev/null || echo "probe: no python3, decode by eye from mpu.bin"
import struct
import sys

path, region = sys.argv[1], int(sys.argv[2], 0)
data = open(path, "rb").read()

print("\npage-aligned 32-bit values in the block that bracket 0x%08x:" % region)
hits = 0
for stride in (4,):
    for off in range(0, len(data) - 8, stride):
        start, = struct.unpack_from(">I", data, off)
        end, = struct.unpack_from(">I", data, off + 8) if off + 12 <= len(data) else (0,)
        if start == 0 or (start & 0xfff) or start > region:
            continue
        if end and end != 0xffffffff and end < region:
            continue
        print("  offset 0x%03x: start 0x%08x end 0x%08x  words %s" %
              (off, start, end,
               " ".join("%08x" % w for w in struct.unpack_from(">4I", data, off))))
        hits += 1
if not hits:
    print("  none - the layout is not a simple start/end pair at 4-byte stride;")
    print("  the raw block is in %s, decode it with the vendor emi_mpu layout." % path)
PY

echo
echo "After a fix: reboot, then 'dmesg | grep -i persistent_ram' - the console"
echo "zone's start must move away from 0xce24, and 'echo hello > /dev/pmsg0'"
echo "must reappear in /sys/fs/pstore/pmsg-ramoops-0 after the same reboot."
