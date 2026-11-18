#!/bin/sh
# Test the mount safety of lib.sh, which install.sh, start.sh and stop.sh share.
#
# The failure this exists for: start.sh bind-mounts /dev, /proc and /sys into the
# engine's chroot, and a later install.sh replaced the tree with `rm -rf` while
# those mounts were still in place - a recursive delete walking into the live
# /proc and /dev. The rule under test: the tree is only deleted when nothing is
# mounted in it, and the delete itself must not cross into another filesystem.
#
# It runs in a private mount namespace (unshare -m), so nothing it does is
# visible outside, and it never touches /opt/gps-engine or the real devices.
#
#     ./test-mounts.sh          # quiet
#     ./test-mounts.sh -v       # show every step
set -u

here=$(cd "$(dirname "$0")" && pwd)
verbose=0
[ "${1:-}" = "-v" ] && verbose=1
fails=0

ok()  { echo "PASS  $*"; }
bad() { echo "FAIL  $*"; fails=$((fails + 1)); }
run() { [ "$verbose" = 1 ] && echo "      \$ $*"; "$@"; }

command -v unshare >/dev/null || { echo "test: unshare is missing" >&2; exit 1; }
if ! unshare -rm true 2>/dev/null; then
	echo "test: no unprivileged mount namespace here (unshare -rm failed)" >&2
	exit 1
fi

# The whole test body runs in its own mount namespace.
unshare -rm /bin/sh -s "$here" "$verbose" <<'BODY'
set -u
here=$1
verbose=$2
work=$(mktemp -d "${TMPDIR:-/tmp}/gps-engine-mounts.XXXXXX") || exit 1
trap 'umount -l "$work/prefix/root/dev" 2>/dev/null; umount -l "$work/prefix/root/proc" 2>/dev/null;
      umount -l "$work/prefix/root/sys" 2>/dev/null; rm -rf "$work"' EXIT

PREFIX=$work/prefix
ROOT=$PREFIX/root
export PREFIX ROOT
mkdir -p "$ROOT/dev" "$ROOT/proc" "$ROOT/sys" "$ROOT/vendor/bin" "$work/outside"
echo "engine file" > "$ROOT/vendor/bin/mnld"
echo "outside file" > "$work/outside/keep-me"

fails=0
ok()  { echo "PASS  $*"; }
bad() { echo "FAIL  $*"; fails=$((fails + 1)); }

# A mount in the tree whose contents must survive a refused delete.
mount -t tmpfs -o size=1m none "$ROOT/dev" || { echo "FAIL  cannot mount tmpfs"; exit 1; }
mkdir -p "$ROOT/dev/sub"
echo "live node" > "$ROOT/dev/sub/keep-node"
mount -t tmpfs -o size=1m none "$ROOT/proc"
mount -t tmpfs -o size=1m none "$ROOT/sys"

. "$here/payload/scripts/lib.sh"

[ "$(tree_mounts | wc -l)" = 3 ] && ok "tree_mounts finds the three mounts" \
	|| bad "tree_mounts finds $(tree_mounts | wc -l) mounts, expected 3"

# 1. The guard: with mounts in place, the delete must refuse and change nothing.
if require_unmounted >/dev/null 2>&1; then
	bad "require_unmounted accepted a tree with live mounts"
else
	ok "require_unmounted refuses a tree with live mounts"
fi
if remove_tree >/dev/null 2>&1; then
	bad "remove_tree deleted a tree with live mounts"
else
	ok "remove_tree refuses a tree with live mounts"
fi
[ -f "$ROOT/dev/sub/keep-node" ] && [ -f "$ROOT/vendor/bin/mnld" ] \
	&& ok "the mounted contents and the tree survived the refused delete" \
	|| bad "the refused delete damaged the tree"

# 2. unmount_tree takes them all down, deepest first, and says so.
if unmount_tree; then
	ok "unmount_tree removed all three mounts"
else
	bad "unmount_tree left something mounted"
fi
[ -z "$(tree_mounts)" ] && ok "nothing is mounted under the tree any more" \
	|| bad "mounts remain: $(tree_mounts | tr '\n' ' ')"

# 3. Only now may the tree go, and it must not cross into another filesystem.
if remove_tree; then
	ok "remove_tree removed the tree once it was clean"
else
	bad "remove_tree failed on a clean tree"
fi
[ -f "$work/outside/keep-me" ] && ok "nothing outside the tree was touched" \
	|| bad "something outside the tree was deleted"

# 4. The lazy path: a mount a process still holds must come down with `umount -l`.
mkdir -p "$ROOT/sys"
mount -t tmpfs -o size=1m none "$ROOT/sys"
( cd "$ROOT/sys" && sleep 3 ) &
holder=$!
sleep 0.3
if unmount_tree; then
	ok "unmount_tree took down a mount a process was sitting in (lazy)"
else
	bad "unmount_tree could not take down a mount held by a process"
fi
kill "$holder" 2>/dev/null

echo
echo "$fails check(s) failed"
exit "$fails"
BODY
