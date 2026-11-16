// SPDX-License-Identifier: GPL-2.0-only
/*
 * Host test for stpgps-bridge.
 *
 * There is no STP device on a build host, so the device side is a pty pair: the
 * harness holds the master and gives the bridge the slave path as --dev, which
 * is byte-for-byte the same interface the bridge uses on the phone (read() per
 * STP frame). The bridge publishes its own pty, and the harness reads that,
 * exactly as gpsd will.
 *
 * Three scenarios, and the third one is the reason the first two mean anything:
 *   1. one sentence per frame            -> the published stream must match
 *   2. one sentence split across frames  -> must still match
 *   3. two sentences glued into one frame-> must match; with a bridge built
 *      with -DSTPGPS_NAIVE_FRAMING it must NOT match (see run-tests.sh)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define BUF 8192

static const char *f1 = "$PMTK010,001*2E\r\n";
static const char *f2 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const char *f3 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

static long long now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void msleep(long ms) { usleep((useconds_t)ms * 1000); }

static int read_all(int fd, char *out, size_t cap, long budget_ms)
{
	size_t used = 0;
	long long deadline = now_ms() + budget_ms;

	while (used + 1 < cap && now_ms() < deadline) {
		ssize_t n;
		fd_set rs;
		struct timeval tv = { 0, 50000 };

		FD_ZERO(&rs);
		FD_SET(fd, &rs);
		if (select(fd + 1, &rs, NULL, NULL, &tv) <= 0)
			continue;
		n = read(fd, out + used, cap - used - 1);
		if (n == 0)
			break;
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				continue;
			if (errno == EINTR)
				continue;
			break;
		}
		used += (size_t)n;
		if (used + 1 >= cap)
			break;
	}
	out[used] = '\0';
	return (int)used;
}

/*
 * Compare as sentences. A tty may add or drop a carriage return and may insert
 * an empty line, and none of that is what this test is about: what matters is
 * that every sentence the engine produced comes out once, in order, unmodified.
 */
static int same_lines(const char *want, const char *got)
{
	const char *w = want, *g = got;

	for (;;) {
		const char *we, *ge;
		size_t wl, gl;

		while (*w == '\n' || *w == '\r')
			w++;
		while (*g == '\n' || *g == '\r')
			g++;
		if (!*w || !*g)
			return !*w && !*g;
		we = strpbrk(w, "\r\n");
		ge = strpbrk(g, "\r\n");
		wl = we ? (size_t)(we - w) : strlen(w);
		gl = ge ? (size_t)(ge - g) : strlen(g);
		if (wl != gl || memcmp(w, g, wl) != 0)
			return 0;
		w += wl;
		g += gl;
	}
}

/* Ask nicely, then make sure: a bridge that ignored the request would hang the
 * harness in waitpid instead of failing the scenario. */
static void stop_child(pid_t pid, int *status)
{
	long long deadline = now_ms() + 800;

	kill(pid, SIGTERM);
	while (now_ms() < deadline) {
		if (waitpid(pid, status, WNOHANG) == pid)
			return;
		msleep(20);
	}
	kill(pid, SIGKILL);
	waitpid(pid, status, 0);
}

/*
 * The device side is a pty, and a pty has a line discipline: without raw mode the
 * first half of a sentence would sit in the tty line buffer until the newline
 * arrived, and a "split frame" would never actually reach the bridge. A real
 * /dev/stpgps is a character device with no line discipline, so raw mode is what
 * makes this fake behave like the thing it stands in for.
 */
static int mkpty(char *slave, size_t cap, int *master_out, int *slave_fd_out)
{
	int m = posix_openpt(O_RDWR | O_NOCTTY);
	char *s;
	int sfd;
	struct termios tio;

	if (m < 0)
		return -1;
	if (grantpt(m) || unlockpt(m)) {
		close(m);
		return -1;
	}
	s = ptsname(m);
	if (!s) {
		close(m);
		return -1;
	}
	snprintf(slave, cap, "%s", s);
	sfd = open(slave, O_RDWR | O_NOCTTY);
	if (sfd < 0) {
		close(m);
		return -1;
	}
	if (tcgetattr(sfd, &tio) == 0) {
		cfmakeraw(&tio);
		tcsetattr(sfd, TCSANOW, &tio);
	}
	*master_out = m;
	*slave_fd_out = sfd;
	return 0;
}

struct scenario {
	const char *name;
	const char *frames[8];
	int nframes;
	int glue_last_two;
	int split_middle;
	int capture;
};

/*
 * Decode a capture file back into bytes: sheet lines start with '#', every other
 * line is "<ms> <hex pairs>". What comes out must equal the stream the device
 * was given, which is the only property the capture has to have to be trusted
 * as evidence of what the engine sent.
 */
static size_t decode_capture(const char *path, char *out, size_t cap)
{
	FILE *f = fopen(path, "r");
	char line[4096];
	size_t len = 0;

	if (!f)
		return 0;
	while (fgets(line, sizeof(line), f)) {
		char *p;

		if (line[0] == '#')
			continue;
		p = strchr(line, ' ');
		if (!p)
			continue;
		while (*p) {
			while (*p == ' ')
				p++;
			if (!isxdigit((unsigned char)p[0]) ||
			    !isxdigit((unsigned char)p[1]))
				break;
			if (len + 1 >= cap)
				break;
			out[len++] = (char)strtol((char[]){ p[0], p[1], 0 },
						  NULL, 16);
			p += 2;
		}
	}
	fclose(f);
	out[len] = '\0';
	return len;
}

static int run_one(const struct scenario *sc, const char *bridge,
		   int expect_pass)
{
	char dev_slave[256], link[256], got[BUF], want[BUF];
	char capture[256], captured[BUF];
	char maxl[16];
	int dev_master = -1, dev_slave_fd = -1, out_fd, status, i, ok;
	pid_t pid;

	snprintf(link, sizeof(link), "/tmp/stpgps-bridge-test-%d.nmea",
		 (int)getpid());
	unlink(link);
	snprintf(capture, sizeof(capture), "/tmp/stpgps-bridge-test-%d.capture",
		 (int)getpid());
	if (sc->capture)
		unlink(capture);

	if (mkpty(dev_slave, sizeof(dev_slave), &dev_master, &dev_slave_fd) != 0) {
		fprintf(stderr, "FAIL %s: cannot create the device pty\n", sc->name);
		return 1;
	}

	snprintf(maxl, sizeof(maxl), "%d", sc->nframes + 8);

	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "FAIL %s: fork\n", sc->name);
		close(dev_master);
		close(dev_slave_fd);
		return 1;
	}
	if (pid == 0) {
		/* No shell in between: the pid we kill must be the bridge, or a
		 * leftover keeps the pty open and the harness never returns. */
		if (sc->capture)
			execl(bridge, bridge, "--dev", dev_slave, "--pty-link",
			      link, "--silence-ms", "400", "--max-lines", maxl,
			      "--quiet", "--capture", capture,
			      "--capture-bytes", "4096", (char *)NULL);
		execl(bridge, bridge, "--dev", dev_slave, "--pty-link", link,
		      "--silence-ms", "400", "--max-lines", maxl, "--quiet",
		      (char *)NULL);
		_exit(127);
	}

	/* Wait for the symlink, then open the published pty as gpsd would. */
	{
		long long deadline = now_ms() + 3000;

		while (now_ms() < deadline) {
			struct stat st;

			if (lstat(link, &st) == 0)
				break;
			msleep(20);
		}
	}
	out_fd = open(link, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (out_fd < 0) {
		fprintf(stderr, "FAIL %s: cannot open the published pty: %s\n",
			sc->name, strerror(errno));
		stop_child(pid, NULL);
		close(dev_master);
		return 1;
	}

	/* Let the bridge reach its read loop before any frame is written: a frame
	 * pushed into the pty before open() is not what any scenario is about. */
	msleep(300);

	/* Build the expected output, then replay the frames. */
	want[0] = '\0';
	for (i = 0; i < sc->nframes; i++)
		strncat(want, sc->frames[i], sizeof(want) - strlen(want) - 1);

	for (i = 0; i < sc->nframes; i++) {
		if (sc->glue_last_two && i == sc->nframes - 2) {
			size_t l1 = strlen(sc->frames[i]);

			write(dev_master, sc->frames[i], l1);
			write(dev_master, sc->frames[i + 1],
			      strlen(sc->frames[i + 1]));
			break;
		}
		if (sc->split_middle && i == 0) {
			size_t l = strlen(sc->frames[i]);

			write(dev_master, sc->frames[i], l / 2);
			/* Longer than one read cycle, so the two halves really do
			 * arrive as two frames - the moved boundary this scenario
			 * is built to exercise. */
			msleep(200);
			write(dev_master, sc->frames[i] + l / 2, l - l / 2);
			continue;
		}
		write(dev_master, sc->frames[i], strlen(sc->frames[i]));
		msleep(20);
	}

	read_all(out_fd, got, sizeof(got), 1500);
	ok = same_lines(want, got);

	stop_child(pid, &status);
	if (ok && sc->capture) {
		size_t clen = decode_capture(capture, captured, sizeof(captured));

		/* The capture is the raw stream, byte for byte: it is what a phone
		 * log will be read as. */
		ok = clen == strlen(want) && memcmp(captured, want, clen) == 0;
		if (!ok)
			fprintf(stderr,
				"  capture: %zu bytes, stream: %zu bytes\n",
				clen, strlen(want));
	}
	close(out_fd);
	close(dev_master);
	close(dev_slave_fd);
	unlink(link);
	if (sc->capture)
		unlink(capture);

	if (ok == expect_pass) {
		printf("PASS %s%s\n", sc->name,
		       expect_pass ? "" : " (fault detected, as required)");
		return 0;
	}
	printf("FAIL %s: expected %s\n", sc->name,
	       expect_pass ? "the published stream to match the engine's"
			   : "the broken framing to be caught");
	printf("  want: %s\n  got : %s\n", want, got);
	return 1;
}

int main(int argc, char **argv)
{
	const char *bridge = argc > 1 ? argv[1] : "./stpgps-bridge";
	int failures = 0, faults = argc > 2 && strcmp(argv[2], "--fault") == 0;

	struct scenario one = {
		.name = "one-sentence-per-frame",
		.frames = { f1, f2, f3 },
		.nframes = 3,
	};
	struct scenario split = {
		.name = "sentence-split-across-frames",
		.frames = { f2, f3 },
		.nframes = 2,
		.split_middle = 1,
	};
	struct scenario glue = {
		.name = "two-sentences-glued-in-one-frame",
		.frames = { f1, f2 },
		.nframes = 2,
		.glue_last_two = 1,
	};
	struct scenario cap = {
		.name = "capture-file-equals-the-stream",
		.frames = { f1, f2 },
		.nframes = 2,
		.capture = 1,
	};

	if (!faults) {
		failures += run_one(&one, bridge, 1);
		failures += run_one(&split, bridge, 1);
		failures += run_one(&glue, bridge, 1);
		failures += run_one(&cap, bridge, 1);
	} else {
		/*
		 * The planted fault is a moved frame boundary: one sentence arrives
		 * as two frames. A bridge that trusts read() boundaries emits two
		 * fragments instead of the sentence, so this scenario must fail -
		 * that is what proves the passing runs above are not vacuous.
		 */
		failures += run_one(&split, bridge, 0);
		/* Same idea for the capture: the naive build loses a byte, so the
		 * decode must not match the stream and the harness must say so. */
		{
			struct scenario capf = {
				.name = "capture-file-equals-the-stream",
				.frames = { f1, f2 },
				.nframes = 2,
				.capture = 1,
			};

			failures += run_one(&capf, bridge, 0);
		}
	}

	if (failures) {
		printf("%d scenario(s) did not behave as required\n", failures);
		return 1;
	}
	return 0;
}
