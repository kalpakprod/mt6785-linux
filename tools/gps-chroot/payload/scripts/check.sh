#!/bin/sh
# Check everything the engine needs, without starting it.
#
# Every line prints PASS or FAIL and says what was actually looked at, so this
# can be read as evidence rather than as reassurance.
#
# Two rules that a previous version broke:
#   * a path inside the tree is tested by resolving it *on the host* against
#     $ROOT - the tree has no shell and no coreutils, so `chroot $ROOT test ...`
#     cannot work and answered false for every file;
#   * a path is only PASS when it resolves to a real file: system/bin/linker64
#     and the bionic libraries are symlinks, some of them absolute (into /apex),
#     and an absolute link must be resolved against $ROOT as well.
set -u

PREFIX=/opt/gps-engine
ROOT=$PREFIX/root
BINDIR=/usr/local/bin
RUNNER=$ROOT/data/local/tmp/run-mnld
STATE=/var/log/gps-engine
. "$PREFIX/lib.sh"

fails=0

ok()   { echo "PASS  $*"; }
bad()  { echo "FAIL  $*"; fails=$((fails + 1)); }
info() { echo "      $*"; }

# normalize <guest path> - textual ".." resolution, the way the kernel does it
# inside the chroot (no symlinks followed, just the dots).
normalize() {
	printf '%s\n' "$1" | awk -F/ '
		{ n = 0
		  for (i = 1; i <= NF; i++) {
			  if ($i == "" || $i == ".") continue
			  if ($i == "..") { if (n > 0) n--; continue }
			  s[++n] = $i
		  }
		  out = ""
		  for (i = 1; i <= n; i++) out = out "/" s[i]
		  print (out == "" ? "/" : out)
		}'
}

# guest_path <path inside the tree> - the real path on the host, symlinks and
# dots resolved *as the chroot would*: the tree's links are written for a root at
# $ROOT (system/bin/linker64 -> ../../../apex/... is /apex inside the chroot),
# so they must not be resolved against the host's filesystem, where that path has
# one more component and lands outside the tree.
guest_path() {
	g=$1
	i=0
	while [ $i -lt 10 ]; do
		i=$((i + 1))
		p="$ROOT$g"
		[ -L "$p" ] || { echo "$p"; return 0; }
		t=$(readlink "$p") || return 1
		case "$t" in
		/*) g=$t ;;
		*)  g=$(normalize "$(dirname "$g")/$t") ;;
		esac
	done
	return 1
}

# resolves <path inside the tree> - true when it ends at an existing file.
resolves() {
	p=$(guest_path "$1") || return 1
	[ -e "$p" ]
}

# executable <path inside the tree>
executable() {
	p=$(guest_path "$1") || return 1
	[ -f "$p" ] && [ -x "$p" ]
}

echo "== kernel interfaces"
for dev in /dev/stpgps /dev/gps_emi; do
	if [ -c "$dev" ]; then
		ok "$dev exists ($(stat -c '%t:%T' "$dev"))"
	else
		bad "$dev is missing - is the stpgps module loaded?"
	fi
done
if [ -r /proc/interrupts ] && grep -qiE "btif|wmt" /proc/interrupts; then
	ok "the connectivity interrupt lines are registered"
else
	info "no btif/wmt line in /proc/interrupts yet (expected until the WMT driver binds)"
fi
dmesg 2>/dev/null | grep -q "GNSS bridge ready" && ok "kernel logged the GNSS bridge probe" \
	|| info "no 'GNSS bridge ready' in dmesg yet"

echo
echo "== the tree"
if [ ! -d "$ROOT" ]; then
	bad "no tree at $ROOT - run install.sh first"
	echo
	echo "$fails check(s) failed"
	exit "$fails"
fi

for f in /vendor/bin/mnld /vendor/lib64/libmnl.so /system/bin/linker64 \
	 /system/lib64/libprops-shim.so /system/lib64/libc.so \
	 /apex/com.android.runtime/bin/linker64 \
	 /data/local/tmp/gps.props /data/local/tmp/run-mnld; do
	if resolves "$f"; then
		ok "$f resolves ($(resolves "$f" && readlink -f "$ROOT$f" 2>/dev/null || echo "$ROOT$f"))"
	else
		bad "$f does not resolve"
	fi
done

for f in /vendor/bin/mnld /data/local/tmp/run-mnld /system/bin/linker64 \
	 /apex/com.android.runtime/bin/linker64; do
	if executable "$f"; then
		ok "$f is executable"
	else
		bad "$f is not executable (needs 0755)"
	fi
done

echo
echo "== does the linker really resolve the daemon"
# The one check that cannot be faked: bionic's own linker loads mnld and its
# whole closure from inside the tree. A missing library shows up here, not on the
# first start.
if chroot "$ROOT" /system/bin/linker64 --list /vendor/bin/mnld \
	> "$STATE/linker-list.txt" 2> "$STATE/linker-error.txt"; then
	n=$(grep -c '=>' "$STATE/linker-list.txt")
	if [ "$n" -ge 20 ] && ! grep -q "not found" "$STATE/linker-error.txt"; then
		ok "the linker resolved mnld's closure ($n libraries)"
	else
		bad "the closure is incomplete ($n libraries listed)"
		sed 's/^/      /' "$STATE/linker-error.txt" | head -5
	fi
else
	bad "the linker could not be run inside the tree:"
	sed 's/^/      /' "$STATE/linker-error.txt" | head -5
fi

echo
echo "== the preload wrapper"
# mnld needs LD_PRELOAD=/system/lib64/libprops-shim.so and the tree has no env(1);
# run-mnld is the static wrapper that sets it for the child it starts.
if [ -x "$RUNNER" ]; then
	if chroot "$ROOT" /data/local/tmp/run-mnld /system/bin/linker64 --list /vendor/bin/mnld \
		> "$STATE/wrapper-list.txt" 2> "$STATE/wrapper-error.txt"; then
		ok "run-mnld execs the linker with the shim preloaded ($(grep -c '=>' "$STATE/wrapper-list.txt") entries)"
	else
		bad "run-mnld failed:"
		sed 's/^/      /' "$STATE/wrapper-error.txt" | head -5
	fi
else
	bad "$RUNNER is missing - reinstall the archive"
fi

echo
echo "== the forwarder and the mounts"
if [ -x "$BINDIR/nmea-forward" ]; then
	ok "nmea-forward is installed"
else
	bad "nmea-forward is not installed"
fi
m=$(tree_mounts)
if [ -z "$m" ]; then
	info "nothing is mounted in the tree right now (start.sh mounts /dev, /proc, /sys)"
else
	ok "mounted in the tree: $(echo "$m" | tr '\n' ' ')"
fi

echo
echo "== what this does NOT prove"
info "the daemon has not run: the EMI ioctls, the fwctl exchange and the DSP"
info "upload are only proven by start.sh plus the kernel log"
echo
echo "$fails check(s) failed"
exit "$fails"
