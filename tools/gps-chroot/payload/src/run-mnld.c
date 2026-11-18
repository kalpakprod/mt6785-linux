// SPDX-License-Identifier: GPL-2.0-only
/*
 * run-mnld - start an Android binary inside the chroot with the property shim
 * preloaded.
 *
 * Why this exists: mnld is a bionic binary that must run with
 * LD_PRELOAD=/system/lib64/libprops-shim.so, and there is no `env` inside the
 * tree to set it (the tree is the engine's closure, not a shell). Putting
 * LD_PRELOAD on the host's `chroot` command line does not work either: the host
 * chroot binary inherits it too, and the host's loader refuses to preload a
 * bionic library ("cannot be preloaded"). This wrapper is the missing link: it
 * is static, so it runs inside the chroot with no loader of its own, sets the
 * variable in the environment of the process it starts, and execs.
 *
 *     chroot ROOT /data/local/tmp/run-mnld /vendor/bin/mnld [args]
 *
 * Built for aarch64-linux-musl, statically (deploy.sh does it).
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SHIM "/system/lib64/libprops-shim.so"

int main(int argc, char **argv)
{
	const char *preload = getenv("GPS_ENGINE_PRELOAD");

	if (argc < 2) {
		fprintf(stderr, "usage: %s PROGRAM [ARGS...]\n", argv[0]);
		return 2;
	}
	if (preload && !*preload)
		preload = NULL;
	if (!preload)
		preload = SHIM;

	/* Keep whatever the caller set, but make sure ours is in front. */
	if (access(preload, R_OK) != 0) {
		fprintf(stderr, "%s: cannot read %s: %s\n", argv[0], preload,
			strerror(errno));
		return 1;
	}
	if (setenv("LD_PRELOAD", preload, 1) != 0) {
		fprintf(stderr, "%s: setenv: %s\n", argv[0], strerror(errno));
		return 1;
	}

	execv(argv[1], argv + 1);
	fprintf(stderr, "%s: exec %s: %s\n", argv[0], argv[1], strerror(errno));
	return 1;
}
