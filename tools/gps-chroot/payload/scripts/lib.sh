#!/bin/sh
# Shared helpers: where the tree lives, and how to take it apart without ever
# walking through a live mount.
#
# Why this file exists: the daemon needs /dev, /proc and /sys bind-mounted into
# its chroot. A later `rm -rf $ROOT` would walk into the *live* /proc and /dev
# through those mounts - the kernel refuses most of it with "Operation not
# permitted", but a recursive delete through a live mount is exactly the kind of
# thing that eats real device nodes. So: unmount first, verify nothing is left
# mounted under the tree, and only then touch the files, with
# `rm -rf --one-file-system` as the second guard.

PREFIX=${PREFIX:-/opt/gps-engine}
ROOT=${ROOT:-$PREFIX/root}

# Mount points at or under $ROOT, deepest first (so nested mounts go first).
tree_mounts() {
	[ -r /proc/mounts ] || return 0
	awk -v root="$ROOT" '
		$2 == root || index($2, root "/") == 1 { print $2 }
	' /proc/mounts | sort -r
}

# Take every mount under the tree down: normal first, then lazy for the ones a
# running process still holds. Returns non-zero if anything is still mounted.
unmount_tree() {
	m=$(tree_mounts)
	[ -z "$m" ] && return 0
	for p in $m; do
		umount "$p" 2>/dev/null || true
	done
	for p in $m; do
		umount -l "$p" 2>/dev/null || true
	done
	i=0
	while [ $i -lt 30 ]; do
		[ -z "$(tree_mounts)" ] && break
		i=$((i + 1))
		sleep 0.1
	done
	m=$(tree_mounts)
	[ -z "$m" ] && return 0
	echo "still mounted under $ROOT:" >&2
	echo "$m" | sed 's/^/  /' >&2
	return 1
}

# Refuse to go on while anything is mounted under the tree.
require_unmounted() {
	m=$(tree_mounts)
	[ -z "$m" ] && return 0
	echo "refusing: these mount points are still active under $ROOT:" >&2
	echo "$m" | sed 's/^/  /' >&2
	echo "unmount them first (stop.sh does it, or: umount -l <path>)" >&2
	return 1
}

# Delete the tree, but never across a mount and never into another filesystem.
remove_tree() {
	require_unmounted || return 1
	rm -rf --one-file-system "$ROOT" || return 1
	[ -e "$ROOT" ] && return 1
	return 0
}

# Replace the installed tree with a fresh copy: unmount, verify, delete, copy.
# This is the whole dangerous sequence in one place, so that install.sh cannot
# forget a step and the test can drive exactly the same code.
install_tree() {
	src=${1:?install_tree: no source tree}
	[ -d "$src" ] || return 1
	if [ -d "$ROOT" ]; then
		unmount_tree || return 1
		remove_tree || return 1
	fi
	mkdir -p "$(dirname "$ROOT")" || return 1
	cp -a "$src" "$ROOT" || return 1
	return 0
}
