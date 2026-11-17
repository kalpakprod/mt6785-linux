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
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
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

/*
 * poco.crashhold=1: do not reboot when done. The hardware watchdog that the
 * previous kernel armed keeps running across kexec (watchdog.stop_on_reboot=0) and
 * nothing in this initramfs pets it, so the SoC resets through the WDT about its
 * timeout later - the one reset path MTK builds a DDR reserve mode for. If that
 * reset keeps DRAM, this kernel's own console (loglevel 8, ramoops parameters on
 * its command line) is in console-ramoops-0 on the next boot, which is the whole
 * point: it is how a capture kernel that dies gets a voice.
 *
 * Diagnostic only. A real capture must reboot when it is done, and
 * crash-kexec-load.sh does not pass this.
 */
static int crashhold;

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
		else if (!strncmp(tok, "poco.crashhold=", 15))
			crashhold = atoi(tok + 15);
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

/*
 * Resolve the device to mount.
 *
 * A capture kernel has a bare devtmpfs: /dev/<DEVNAME> exists for every block
 * device the kernel found, but /dev/disk/by-partlabel is made by udev and there is
 * no udev here. That is why three capture runs wrote nothing - the mount of
 * "/dev/disk/by-partlabel/userdata" failed, so no file could be created, and the
 * ~12 s round trip was just the reboot after that. The argument is therefore a
 * PARTLABEL= name that is resolved through sysfs, plus a plain /dev path for
 * anyone who prefers to give one.
 *
 * UFS probes asynchronously, so the partition can appear after the initramfs
 * starts: poll for up to 30 s and write every attempt to /dev/kmsg, which
 * crash-init later saves next to the dump.
 */
#define PARTLABEL_MAX_TRIES	300	/* 300 x 100 ms = 30 s */
#define PARTLABEL_FAIL_MS	100

static void kmsg_line(const char *fmt, ...)
{
	char buf[512];
	char path[512];
	va_list ap;
	int fd, n;

	va_start(ap, fmt);
	n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;

	/* O_APPEND: each write to /dev/kmsg is a record anyway, and it keeps the
	 * test build from overwriting a planted log line. */
	fd = open(root_path("/dev/kmsg", path, sizeof(path)),
		  O_WRONLY | O_APPEND | O_NONBLOCK);
	if (fd < 0)
		return;
	write(fd, buf, (size_t)n);
	close(fd);
}

static int read_uevent_field(const char *dir, const char *field, char *out,
			     size_t len)
{
	char path[512], line[256];
	FILE *f;
	int found = 0;

	snprintf(path, sizeof(path), "%s/uevent", dir);
	f = fopen(path, "r");
	if (!f)
		return 0;
	while (fgets(line, sizeof(line), f)) {
		size_t fl = strlen(field);

		if (strncmp(line, field, fl) || line[fl] != '=')
			continue;
		snprintf(out, len, "%s", line + fl + 1);
		out[strcspn(out, "\n")] = '\0';
		found = 1;
		break;
	}
	fclose(f);
	return found;
}

/*
 * PARTLABEL=<name> -> /dev/<DEVNAME>, by scanning /sys/class/block. Returns 0 and
 * fills outpath on success, -1 when the partition never appeared, and 1 when the
 * argument was a plain path and was copied through.
 */
static int resolve_dev(const char *arg, char *outpath, size_t len)
{
	char blockdir[512], dir[1024];
	int max_tries = PARTLABEL_MAX_TRIES;
	long poll_us = PARTLABEL_FAIL_MS * 1000;
	int tries, ret = -1;

#ifdef CRASH_TEST
	/* The tests cannot wait 30 s per case; the bounds are overridable there. */
	if (getenv("CRASH_TEST_TRIES"))
		max_tries = atoi(getenv("CRASH_TEST_TRIES"));
	if (getenv("CRASH_TEST_POLL_US"))
		poll_us = atol(getenv("CRASH_TEST_POLL_US"));
#endif

	if (strncmp(arg, "PARTLABEL=", 10)) {
		snprintf(outpath, len, "%s", arg);
		return 1;
	}

	for (tries = 1; tries <= max_tries; tries++) {
		DIR *d;
		struct dirent *de;

		snprintf(blockdir, sizeof(blockdir), "%s/sys/class/block",
			 MOUNT_DIR);
		d = opendir(blockdir);
		if (d) {
			while ((de = readdir(d))) {
				char name[128], devname[128], majors[16], minors[16];

				if (de->d_name[0] == '.')
					continue;
				snprintf(dir, sizeof(dir), "%s/%s", blockdir,
					 de->d_name);
				if (!read_uevent_field(dir, "PARTNAME", name,
						       sizeof(name)))
					continue;
				if (strcmp(name, arg + 10))
					continue;
				if (!read_uevent_field(dir, "DEVNAME", devname,
						       sizeof(devname)))
					continue;
				snprintf(outpath, len, "/dev/%s", devname);
				/* devtmpfs has the node; if not, make it from the
				 * major/minor the same uevent carries. */
				if (access(outpath, F_OK) != 0 &&
				    read_uevent_field(dir, "MAJOR", majors,
						      sizeof(majors)) &&
				    read_uevent_field(dir, "MINOR", minors,
						      sizeof(minors))) {
					char real[600];

					snprintf(real, sizeof(real), "%s%s",
						 MOUNT_DIR, outpath);
					mknod(real, S_IFBLK | 0600,
					      makedev(atoi(majors), atoi(minors)));
				}
				kmsg_line("crash-init: %s resolved to %s after %d tries\n",
					  arg, outpath, tries);
				closedir(d);
				return 0;
			}
			closedir(d);
		}
		if (tries == 1 || tries % 50 == 0)
			kmsg_line("crash-init: %s not in /sys/class/block yet (try %d)\n",
				  arg, tries);
		usleep((useconds_t)poll_us);
	}
	kmsg_line("crash-init: %s never appeared - is the driver built in?\n", arg);
	return ret;
}

/*
 * Mount the phone's filesystem.
 *
 * ext4 is tried WITHOUT noload. That looks unsafe and is not: a filesystem whose
 * journal needs recovery cannot be mounted read-write with noload at all - ext4
 * refuses with -EINVAL ("required journal recovery suppressed and not mounted
 * read-only", fs/ext4/super.c:5552-5558) - and replaying the journal is exactly
 * what every ordinary boot does after a crash. Refusing it was why the capture
 * kernel mounted nothing: after a kexec whose shutdown could not remount root
 * read-only, userdata always needs recovery.
 *
 * The read-only pass afterwards is the safety net the lead asked for: if nothing
 * mounts read-write, the dump cannot be written, but the attempt is still logged
 * so the next run says which filesystem refused and why.
 */
static int try_mount(const char *type, unsigned long flags, const char *data)
{
#ifndef CRASH_TEST
	if (!mount(crashdev, mountpoint, type, flags, data)) {
		kmsg_line("crash-init: mounted %s as %s%s%s\n", crashdev, type,
			  (flags & MS_RDONLY) ? " read-only" : "",
			  data ? " with options" : "");
		return 0;
	}
	kmsg_line("crash-init: mount %s as %s%s failed: %s\n", crashdev, type,
		  (flags & MS_RDONLY) ? " read-only" : "", strerror(errno));
	return -1;
#else
	(void)data;
	/* The tests simulate the mounts: an attempt succeeds when the test planted
	 * <root>/mount-ok.<type>.<rw|ro>. Every attempt is appended to a log the
	 * test asserts on, because the order of the attempts is the point. */
	{
		char ok[256], log[64], path[1024];
		FILE *f;
		int found = 0;

		snprintf(ok, sizeof(ok), "/mount-ok.%s.%s", type,
			 (flags & MS_RDONLY) ? "ro" : "rw");
		snprintf(log, sizeof(log), "/mount-attempts.txt");
		snprintf(path, sizeof(path), "%s%s", MOUNT_DIR, log);
		f = fopen(path, "a");
		if (f) {
			fprintf(f, "%s %s\n", type, (flags & MS_RDONLY) ? "ro" : "rw");
			fclose(f);
		}
		snprintf(path, sizeof(path), "%s%s", MOUNT_DIR, ok);
		found = access(path, F_OK) == 0;
		if (found)
			kmsg_line("crash-init: mounted %s as %s%s\n", crashdev, type,
				  (flags & MS_RDONLY) ? " read-only" : "");
		return found ? 0 : -1;
	}
#endif
}

static int mount_userdata(void)
{
	static const struct {
		const char *type;
		unsigned long flags;
	} tries[] = {
		{ "f2fs", 0 },
		{ "ext4", 0 },
		{ "btrfs", 0 },
		{ "xfs", 0 },
		{ "f2fs", MS_RDONLY },
		{ "ext4", MS_RDONLY },
		{ "btrfs", MS_RDONLY },
		{ "xfs", MS_RDONLY },
	};
	unsigned int i;

	for (i = 0; i < sizeof(tries) / sizeof(tries[0]); i++) {
		if (!try_mount(tries[i].type, tries[i].flags, NULL))
			return 1;
	}
	return 0;
}

#ifdef CRASH_TEST
/*
 * In the test build a mount is simulated by copying the planted
 * <root>/cmdline-template over <root>/proc/cmdline: before this runs there is no
 * /proc/cmdline in the fake root, which is the whole point of the ordering test.
 */
static void simulate_mount(const char *fs)
{
	char tmpl[512], dst[512];
	FILE *in, *out;
	char buf[4096];
	size_t n;

	if (strcmp(fs, "proc"))
		return;
	snprintf(tmpl, sizeof(tmpl), "%s/cmdline-template", MOUNT_DIR);
	snprintf(dst, sizeof(dst), "%s%s", MOUNT_DIR, "/proc/cmdline");
	in = fopen(tmpl, "r");
	if (!in)
		return;
	out = fopen(dst, "w");
	if (!out) {
		fclose(in);
		return;
	}
	while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
		fwrite(buf, 1, n, out);
	fclose(in);
	fclose(out);
}
#endif

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
	long total = -1;
	FILE *f;

	(void)argc;
	(void)argv;

	umask(0);

	/*
	 * Mount first, read the command line second. This is a fresh initramfs: until
	 * /proc is mounted there is no /proc/cmdline at all, so reading it first left
	 * crashdev, crashdir, selftest and crashhold unset - the boot was then taken as
	 * "no /proc/vmcore, nothing to do", crash-init rebooted at once through PSCI,
	 * and that is why every capture run left no file and no record. The live-phone
	 * run of this program passed only because /proc/cmdline already existed there,
	 * and so did every host test.
	 */
#ifdef CRASH_TEST
	/* The simulated mount is what makes the command line appear, exactly as a real
	 * procfs mount does in the initramfs. CRASH_OLD_ORDER reproduces the ordering bug
	 * so the test can show it fails. */
	if (getenv("CRASH_OLD_ORDER")) {
		read_cmdline();		/* the bug: nothing is there yet */
		simulate_mount("proc");
	} else {
		simulate_mount("proc");
		read_cmdline();
	}
#else
	do_mount("proc", "/proc", "proc", 0);
	do_mount("sysfs", "/sys", "sysfs", 0);
	do_mount("devtmpfs", "/dev", "devtmpfs", 0);
	read_cmdline();
#endif

	{
		char vmcore[512];

		if (!file_exists(root_path("/proc/vmcore", vmcore,
					   sizeof(vmcore)))) {
			selftest = 1;
			logmsg("crash-init: no /proc/vmcore - self-test boot");
		}
	}

	/*
	 * The device is resolved in both builds: the tests plant a /sys tree with a
	 * partition that appears late (or never) and check what crash-init does, and
	 * the capture kernel needs the same code because it has no udev.
	 */
	{
		char devpath[256];
		int r = resolve_dev(crashdev, devpath, sizeof(devpath));

		if (r < 0) {
			logmsg("crash-init: cannot resolve %s: %s", crashdev,
			       strerror(errno));
			goto out;
		}
		crashdev = strdup(devpath);
		if (!crashdev)
			goto out;
		logmsg("crash-init: userdata device is %s", crashdev);
	}

#ifndef CRASH_TEST
	mkdir_p(mountpoint);
#endif
	if (!mount_userdata()) {
		logmsg("crash-init: no filesystem on %s could be mounted", crashdev);
		kmsg_line("crash-init: no filesystem on %s could be mounted\n", crashdev);
		goto out;
	}
#ifdef CRASH_TEST
	mkdir_p("/mnt");
#endif
#ifndef CRASH_TEST
	/*
	 * The dumpers run chrooted into the phone's filesystem and read
	 * /proc/vmcore there, so the three pseudo filesystems have to be visible
	 * inside it. A failed bind is logged, not fatal: the notes fallback still
	 * writes something.
	 */
	{
		char inside[600];

		snprintf(inside, sizeof(inside), "%s/proc", mountpoint);
		if (bind_dir("/proc", inside))
			logmsg("crash-init: bind /proc: %s", strerror(errno));
		snprintf(inside, sizeof(inside), "%s/dev", mountpoint);
		if (bind_dir("/dev", inside))
			logmsg("crash-init: bind /dev: %s", strerror(errno));
		snprintf(inside, sizeof(inside), "%s/sys", mountpoint);
		if (bind_dir("/sys", inside))
			logmsg("crash-init: bind /sys: %s", strerror(errno));
	}
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
	/* The summary first, then the final dump of the log: the file is written
	 * from /dev/kmsg, so a line added after it would be lost. */
	kmsg_line("crash-init: %s (%ld bytes), hold=%d\n",
		  done ? marker : "no dump", total, crashhold);

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
	if (crashhold) {
		kmsg_line("crash-init: holding for the hardware watchdog; no reboot from here\n");
		logmsg("crash-init: holding for the hardware watchdog");
#ifdef CRASH_TEST
		{
			char hp[512];
			FILE *hf = fopen(root_path("/crashhold.txt", hp, sizeof(hp)), "w");

			if (hf) {
				fprintf(hf, "hold\n");
				fclose(hf);
			}
		}
		return done ? 0 : 1;
#else
		for (;;)
			pause();
#endif
	}

#ifndef CRASH_TEST
	reboot(RB_AUTOBOOT);
	/* If the reboot is refused something is very wrong; never fall through
	 * into the capture kernel's shell-less void. */
	for (;;)
		sleep(3600);
#endif
	return done ? 0 : 1;
}
