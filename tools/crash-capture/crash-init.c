// SPDX-License-Identifier: GPL-2.0-only
/*
 * crash-init - the capture kernel's init for the pocophone kdump setup.
 *
 * A hung or panicking trial kernel is not saved by ramoops: this phone comes
 * back from every reset with DRAM at its power-up state (REPORT.md, "pstore on
 * rosemary, part 2"). The log therefore has to be taken out of memory by a
 * second kernel that is already loaded with `kexec -p` and runs without a
 * reset, so the crashed kernel's memory is still there.
 *
 * This program is that second kernel's init. It:
 *   1. mounts /proc, /sys, /dev,
 *   2. finds the userdata filesystem (device and directory from the kernel
 *      command line: poco.crashdev= / poco.crashdir=),
 *   3. pulls the crashed kernel's log out of /proc/vmcore - using the phone's
 *      own vmcore-dmesg or makedumpfile through a chroot, because a full vmcore
 *      is gigabytes and this phone has no room for that,
 *   4. falls back to recording the ELF notes when neither tool is there, so a
 *      capture always leaves evidence of what was wrong,
 *   5. syncs, writes a marker saying which path was used, and reboots.
 *
 * It is a static binary with no shell and no busybox: one file, one job.
 *
 * Built twice: for the phone (aarch64, in the initramfs) and for the host with
 * -DCRASH_TEST, where mounts, chroot and reboot are no-ops and every path is
 * resolved under $CRASH_TEST_ROOT - which is what lets the test script plant a
 * broken dumper and check that the fallback happens.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define MAX_CMDLINE 4096

static const char *crashdev = "/dev/disk/by-partlabel/userdata";
static const char *crashdir = "/var/lib/poco/crash";
#ifndef CRASH_TEST
static const char *mountpoint = "/mnt";
#endif

#ifdef CRASH_TEST
#define MOUNT_DIR getenv("CRASH_TEST_ROOT") ? getenv("CRASH_TEST_ROOT") : "."
#else
#define MOUNT_DIR ""
#endif

/* Every fixed path goes through here, so the test build can resolve it inside
 * the fake root instead of the real filesystem. */
static const char *root_path(const char *path, char *buf, size_t len)
{
	snprintf(buf, len, "%s%s", MOUNT_DIR, path);
	return buf;
}

static void logmsg(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

/* Command line: poco.crashdev=<device> poco.crashdir=<dir on the fs>. */
static void read_cmdline(void)
{
	char buf[MAX_CMDLINE], path[512];
	char *tok, *save = NULL;
	ssize_t n;
	int fd;

	fd = open(root_path("/proc/cmdline", path, sizeof(path)), O_RDONLY);
	if (fd < 0)
		return;
	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return;
	buf[n] = '\0';

	for (tok = strtok_r(buf, " \n", &save); tok;
	     tok = strtok_r(NULL, " \n", &save)) {
		if (!strncmp(tok, "poco.crashdev=", 14))
			crashdev = strdup(tok + 14);
		else if (!strncmp(tok, "poco.crashdir=", 14))
			crashdir = strdup(tok + 14);
	}
}

/*
 * Save this kernel's own log. The capture kernel has no console on this phone
 * (no serial header, no display driver worth the risk), so if it dies after the
 * filesystem is mounted and before it writes the dump, the only way to know why
 * is to have written its own log first. Called twice: right after the mount and
 * again just before the reboot, appending the second time.
 */
static void dump_kmsg(const char *path, const char *tag)
{
	char kmsg[512], buf[4096];
	FILE *in, *out;
	ssize_t n;
	int fd;

	fd = open(root_path("/dev/kmsg", kmsg, sizeof(kmsg)), O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		logmsg("crash-init: no /dev/kmsg (%s)", strerror(errno));
		return;
	}
	out = fopen(path, "a");
	if (!out) {
		close(fd);
		logmsg("crash-init: cannot write %s: %s", path, strerror(errno));
		return;
	}
	fprintf(out, "===== capture kernel log, %s =====\n", tag);
	in = fdopen(fd, "r");
	if (in) {
		/* /dev/kmsg is read as records, one per line; non-blocking, so this
		 * stops at EAGAIN once everything printed so far has been read. */
		while ((n = read(fd, buf, sizeof(buf) - 1)) > 0) {
			buf[n] = '\0';
			fputs(buf, out);
		}
		fclose(in);
	} else {
		close(fd);
	}
	fclose(out);
}

static int mkdir_p(const char *path)
{
	char tmp[512];
	char *p;
	size_t len;

	snprintf(tmp, sizeof(tmp), "%s", path);
	len = strlen(tmp);
	if (len && tmp[len - 1] == '/')
		tmp[len - 1] = '\0';
	for (p = tmp + 1; *p; p++) {
		if (*p != '/')
			continue;
		*p = '\0';
		if (mkdir(tmp, 0755) && errno != EEXIST)
			return -1;
		*p = '/';
	}
	if (mkdir(tmp, 0755) && errno != EEXIST)
		return -1;
	return 0;
}

#ifndef CRASH_TEST
static void do_mount(const char *src, const char *tgt, const char *type,
		     unsigned long flags)
{
	mkdir_p(tgt);
	if (mount(src, tgt, type, flags, NULL))
		logmsg("crash-init: mount %s -> %s (%s): %s", src, tgt, type,
		       strerror(errno));
}

static int bind_dir(const char *src, const char *tgt)
{
	mkdir_p(tgt);
	return mount(src, tgt, NULL, MS_BIND, NULL);
}
#endif

/*
 * The log path. /proc/vmcore is the crashed kernel's memory; the phone's own
 * tools parse it. A full vmcore is out of the question (gigabytes of DRAM), so
 * only the dmesg dump is asked for.
 */
struct dumper {
	const char *name;
	const char *path;
	char *const argv[6];
};

static const struct dumper dumpers[] = {
	{ "vmcore-dmesg", "/usr/bin/vmcore-dmesg",
	  { "vmcore-dmesg", "/proc/vmcore", NULL } },
	{ "vmcore-dmesg-sbin", "/usr/sbin/vmcore-dmesg",
	  { "vmcore-dmesg", "/proc/vmcore", NULL } },
	{ "makedumpfile", "/usr/bin/makedumpfile",
	  { "makedumpfile", "--dump-dmesg", "/proc/vmcore", "-", NULL } },
};

static int file_exists(const char *path)
{
	struct stat st;

	return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static long file_size(const char *path)
{
	struct stat st;

	return stat(path, &st) == 0 ? (long)st.st_size : -1;
}

/*
 * Run one dumper. In the capture kernel this chroots into the phone's
 * filesystem, because the tools and their libraries live there; the test build
 * runs the script directly under $CRASH_TEST_ROOT.
 */
static int run_dumper(const struct dumper *d, const char *outpath)
{
#ifndef CRASH_TEST
	char real[512];
#endif
	pid_t pid;
	int status = 0;
	int fd;

	fd = open(outpath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0)
		return -1;

	pid = fork();
	if (pid < 0) {
		close(fd);
		return -1;
	}
	if (pid == 0) {
		dup2(fd, STDOUT_FILENO);
		dup2(fd, STDERR_FILENO);
#ifndef CRASH_TEST
		snprintf(real, sizeof(real), "%s%s", mountpoint, d->path);
		if (chroot(mountpoint))
			_exit(126);
		execv(real, d->argv);
		_exit(127);
#else
		{
			const char *root = MOUNT_DIR;
			char p[512];

			snprintf(p, sizeof(p), "%s%s", root, d->path);
			execv(p, d->argv);
			_exit(127);
		}
#endif
	}
	close(fd);
	if (waitpid(pid, &status, 0) < 0)
		return -1;
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return -1;
	/* An empty dump is a failed dump: a tool that exits 0 saying nothing
	 * must not look like a captured crash. */
	return file_size(outpath) > 0 ? 0 : -1;
}

/* Fallback: the ELF notes of the vmcore, which name the release and the
 * offsets; small, and enough to tell a later reader what was there. */
static int write_notes_fallback(const char *outpath)
{
	char vmcore[512];
	FILE *in = fopen(root_path("/proc/vmcore", vmcore, sizeof(vmcore)), "rb");
	FILE *out;
	unsigned char hdr[64];
	size_t n;

	if (!in)
		return -1;
	out = fopen(outpath, "w");
	if (!out) {
		fclose(in);
		return -1;
	}
	fprintf(out, "crash-init: no vmcore-dmesg and no makedumpfile on this root.\n"
		     "crash-init: a full vmcore was not saved (size). The first 64 bytes\n"
		     "crash-init: of the ELF header follow; the container starts with the\n"
		     "crash-init: crashed kernel's own boot line.\n\n");
	n = fread(hdr, 1, sizeof(hdr), in);
	for (size_t i = 0; i < n; i++)
		fprintf(out, "%02x%s", hdr[i], (i % 16 == 15) ? "\n" : " ");
	fprintf(out, "\n");
	fclose(in);
	fclose(out);
	return file_size(outpath) > 0 ? 0 : -1;
}

int main(int argc, char **argv)
{
	char stamp[32], outdir[512], marker[900];
	const char *used = "notes";
	time_t now = time(NULL);
	struct tm tm;
	int i, done = 0, selftest = 0;
	long total;
	FILE *f;

	(void)argc;
	(void)argv;

	umask(0);
	read_cmdline();

#ifndef CRASH_TEST
	do_mount("proc", "/proc", "proc", 0);
	do_mount("sysfs", "/sys", "sysfs", 0);
	do_mount("devtmpfs", "/dev", "devtmpfs", 0);
#endif

	{
		char vmcore[512];

		if (!file_exists(root_path("/proc/vmcore", vmcore,
					   sizeof(vmcore)))) {
			selftest = 1;
			logmsg("crash-init: no /proc/vmcore - self-test boot");
		}
	}

#ifndef CRASH_TEST
	mkdir_p(mountpoint);
	for (i = 0; i < 4; i++) {
		/*
		 * f2fs, ext4, btrfs, xfs in that order. ext4 is mounted with
		 * "noload": the crashed kernel may have had writeback in flight, and
		 * replaying an unfinished journal from a second kernel that knows
		 * nothing of the first one's in-memory state is how a kdump run turns
		 * one crash into a corrupted filesystem. Writes still work; the
		 * journal is simply not replayed here and the next normal boot does
		 * what it always does.
		 */
		static const char *types[] = { "f2fs", "ext4", "btrfs", "xfs" };
		static const char *opts[] = { NULL, "noload", NULL, NULL };

		if (!mount(crashdev, mountpoint, types[i], 0, opts[i])) {
			logmsg("crash-init: mounted %s as %s%s%s", crashdev,
			       types[i], opts[i] ? " with " : "",
			       opts[i] ? opts[i] : "");
			break;
		}
	}
	if (i == 4) {
		logmsg("crash-init: cannot mount %s: %s", crashdev,
		       strerror(errno));
		goto out;
	}
	/* The tools need the phone's own /proc (for /proc/vmcore), its /sys and
	 * its /dev inside the chroot. */
	bind_dir("/proc", "/tmp/crash-root-proc");
#else
	mkdir_p("/mnt");
#endif

#ifdef CRASH_TEST
	/* In the test build the "phone filesystem" is the fake root itself. */
	snprintf(outdir, sizeof(outdir), "%s%s", MOUNT_DIR, crashdir);
#else
	snprintf(outdir, sizeof(outdir), "%s%s", mountpoint, crashdir);
#endif
	mkdir_p(outdir);
	gmtime_r(&now, &tm);
	strftime(stamp, sizeof(stamp), "%Y%m%dT%H%M%SZ", &tm);
	snprintf(marker, sizeof(marker), "%s/%s.txt", outdir, stamp);

	/*
	 * First thing after the mount: save our own log. Whatever happens next -
	 * the dumpers, a driver that resets shared hardware, a reboot that hangs -
	 * this file is the only account of it.
	 */
	{
		char kmsgfile[sizeof(marker) + 8];

		snprintf(kmsgfile, sizeof(kmsgfile), "%s/%s%s.kmsg", outdir, stamp,
			 selftest ? "-selftest" : "");
		dump_kmsg(kmsgfile, "after mounting the phone filesystem");
		if (selftest)
			logmsg("crash-init: self-test, log in %s", kmsgfile);
	}

	/* A self-test boot has no crashed kernel to read: the log above is the
	 * whole point, so go straight to the reboot. */
	if (selftest)
		goto out;

	/* Try each dumper; the first that produces a non-empty file wins. */
	for (i = 0; i < (int)(sizeof(dumpers) / sizeof(dumpers[0])); i++) {
		char candidate[sizeof(marker) + 8];

		snprintf(candidate, sizeof(candidate), "%s.tmp", marker);
		if (run_dumper(&dumpers[i], candidate) == 0) {
			rename(candidate, marker);
			used = dumpers[i].name;
			done = 1;
			logmsg("crash-init: dumped with %s", dumpers[i].name);
			break;
		}
		unlink(candidate);
	}

	if (!done) {
		/* write_notes_fallback() opens the path it is given; the notes go to
		 * the same ".tmp" name the dumpers used, then get renamed. */
		char notepath[sizeof(marker) + 8];

		snprintf(notepath, sizeof(notepath), "%s.tmp", marker);
		if (write_notes_fallback(notepath) == 0) {
			rename(notepath, marker);
			used = "notes-fallback";
			done = 1;
		}
	}

	/* One line next to the dump, so a later reader knows how it was made. */
	{
		char mpath[512];

		f = fopen(root_path("/var/lib/poco-crash-method.tmp", mpath,
				    sizeof(mpath)), "w");
	}
	if (f) {
		fprintf(f, "method=%s file=%s\n", used, marker);
		fclose(f);
	}
	if (done) {
		char method[sizeof(outdir) + 16];

		snprintf(method, sizeof(method), "%s/method.txt", outdir);
		f = fopen(method, "a");
		if (f) {
			fprintf(f, "%s %s\n", stamp, used);
			fclose(f);
		}
	}

	total = done ? file_size(marker) : -1;
	logmsg("crash-init: %s (%ld bytes)", done ? marker : "no dump", total);

out:
	/* The second half of the log, if the filesystem is mounted at all. */
	if (*crashdir) {
		char kmsgfile[sizeof(marker) + 8];

		if (!stamp[0])
			snprintf(stamp, sizeof(stamp), "boot");
#ifdef CRASH_TEST
		snprintf(kmsgfile, sizeof(kmsgfile), "%s%s/%s%s.kmsg", MOUNT_DIR,
			 crashdir, stamp, selftest ? "-selftest" : "");
#else
		snprintf(kmsgfile, sizeof(kmsgfile), "/mnt%s/%s%s.kmsg", crashdir,
			 stamp, selftest ? "-selftest" : "");
#endif
		dump_kmsg(kmsgfile, "before reboot");
	}

	/*
	 * Leave the filesystem clean: sync, unmount, sync again. The capture kernel
	 * reboots without a shutdown, and an unclean userdata would be the second
	 * piece of damage a crash does.
	 */
	sync();
#ifndef CRASH_TEST
	if (umount(mountpoint))
		logmsg("crash-init: umount %s: %s", mountpoint, strerror(errno));
	sync();
#endif
	sleep(1);
#ifndef CRASH_TEST
	reboot(RB_AUTOBOOT);
	/* If the reboot is refused something is very wrong; never fall through
	 * into the capture kernel's shell-less void. */
	for (;;)
		sleep(3600);
#endif
	return done ? 0 : 1;
}
