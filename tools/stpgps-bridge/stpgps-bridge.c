// SPDX-License-Identifier: GPL-2.0-only
/*
 * stpgps-bridge - republish the MT6785 GNSS engine's NMEA to a pty for gpsd.
 *
 * Route A of REPORT.md: the engine is a task on the CONNSYS MCU, its channel is
 * /dev/stpgps (STP task 2, one STP frame per read), and userspace has to own that
 * device and give gpsd something it can read. This program does exactly that and
 * nothing else: it opens the device, copies frames out, splits them into NMEA
 * lines, writes them to the master side of a pty, and publishes the slave path.
 *
 * It deliberately contains no engine protocol of its own. The start sequence
 * (PMTK or binary) could not be proven from the sources available in the repo -
 * see README.md, "What is proven and what is missing" - so this program does not
 * guess it. It can replay a sequence given on the command line (--send), which is
 * how the sequence will be added the moment it is proven.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "republish.h"

#define RX_CHUNK 1024

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static long long now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* Write the slave path of the publishing pty as a stable symlink path. */
static int publish_link(const char *slave, const char *link)
{
	char tmp[512];
	int n;

	n = snprintf(tmp, sizeof(tmp), "%s.tmp", link);
	if (n < 0 || (size_t)n >= sizeof(tmp))
		return -1;
	unlink(tmp);
	if (symlink(slave, tmp) != 0)
		return -1;
	if (rename(tmp, link) != 0) {
		unlink(tmp);
		return -1;
	}
	return 0;
}

/*
 * The bridge keeps the slave end open for its whole life. Without that, a write
 * to the master before the consumer (gpsd) has opened the slave is discarded by
 * the tty layer, and the first sentences of a session vanish exactly when the
 * consumer is slowest to attach. holding both ends is what socat does too.
 */
static int make_pty(char **slave_out, int *slave_fd_out)
{
	int master;
	char *slave;

	master = posix_openpt(O_RDWR | O_NOCTTY);
	if (master < 0)
		return -1;
	if (grantpt(master) != 0 || unlockpt(master) != 0) {
		close(master);
		return -1;
	}
	slave = ptsname(master);
	if (!slave) {
		close(master);
		return -1;
	}
	*slave_out = strdup(slave);
	if (!*slave_out) {
		close(master);
		return -1;
	}
	/* The engine's sentences arrive at 1 Hz; disabling echo and line
	 * discipline keeps the bytes gpsd sees identical to the engine's. */
	{
		struct termios tio;
		int sfd = open(slave, O_RDWR | O_NOCTTY);

		if (sfd < 0) {
			close(master);
			free(*slave_out);
			*slave_out = NULL;
			return -1;
		}
		if (tcgetattr(sfd, &tio) == 0) {
			cfmakeraw(&tio);
			tcsetattr(sfd, TCSANOW, &tio);
		}
		*slave_fd_out = sfd;
	}
	return master;
}

static void emit_line(const char *line, size_t len, void *ctx)
{
	int out = *(int *)ctx;

	/* A closed consumer must not kill the bridge: gpsd may restart and
	 * re-open the pty. */
	if (write(out, line, len) < 0) {
	}
}

/*
 * Send a fixed byte sequence to the device. Until the engine's start sequence is
 * proven this is the only way to inject one, and it is also what the host tests
 * use for the reverse direction.
 */
static int send_bytes(int fd, const char *spec)
{
	char buf[256];
	size_t len = 0;
	const char *p = spec;

	while (*p && len < sizeof(buf)) {
		if (*p == '\\' && p[1]) {
			p++;
			if (*p == 'n') {
				buf[len++] = '\n';
			} else if (*p == 'r') {
				buf[len++] = '\r';
			} else if (*p == 't') {
				buf[len++] = '\t';
			} else if (*p == '\\') {
				buf[len++] = '\\';
			} else {
				buf[len++] = *p;
			}
			p++;
		} else {
			buf[len++] = *p++;
		}
	}
	if (write(fd, buf, len) != (ssize_t)len)
		return -1;
	return 0;
}

/*
 * Capture the first bytes the engine sends, in a form a human can read and a
 * script can decode. Stage 5 of the phone plan needs exactly one answer - what
 * the engine says first, PMTK010 or binary - and until now that needed a
 * hexdump piped from a device that also has to stay owned by the bridge. One
 * session now yields both the capture and the running bridge.
 */
static void capture_open(const char *path, const char *dev, long bytes)
{
	FILE *f = fopen(path, "w");

	if (!f) {
		fprintf(stderr, "stpgps-bridge: capture %s: %s\n", path,
			strerror(errno));
		exit(1);
	}
	fprintf(f, "# stpgps-bridge capture of %s, first %ld bytes\n", dev, bytes);
	fprintf(f, "# <ms since start> <bytes, hex>\n");
	fclose(f);
}

static void capture_chunk(const char *path, long long ms,
			  const unsigned char *buf, size_t len)
{
	FILE *f = fopen(path, "a");
	size_t off = 0;

	if (!f)
		return;
#ifdef STPGPS_NAIVE_FRAMING
	/* The planted fault for the capture test: lose the first byte. */
	if (len > 0) {
		buf++;
		len--;
	}
#endif
	while (off < len) {
		size_t i, n = len - off > 16 ? 16 : len - off;

		fprintf(f, "%lld ", ms);
		for (i = 0; i < n; i++)
			fprintf(f, " %02x", buf[off + i]);
		fprintf(f, "\n");
		off += n;
	}
	fclose(f);
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		"  --dev PATH        STP GNSS device, or - for standard input\n"
		"                    (default /dev/stpgps)\n"
		"  --pty-link PATH   symlink to the published pty slave\n"
		"                    (default /run/stpgps/nmea)\n"
		"  --silence-ms N    close and reopen after N ms without data\n"
		"                    (default 1000)\n"
		"  --send STR        bytes to send once after open, C escapes ok\n"
		"  --once            exit after the first silence timeout\n"
		"  --max-lines N     exit after N republished lines\n"
		"  --capture PATH    write the first bytes the engine sends, hex\n"
		"  --capture-bytes N how many bytes to capture (default 256)\n"
		"  --quiet           no status lines on stderr\n"
		"  --help            this text\n",
		prog);
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "dev", required_argument, NULL, 'd' },
		{ "pty-link", required_argument, NULL, 'l' },
		{ "silence-ms", required_argument, NULL, 's' },
		{ "send", required_argument, NULL, 'S' },
		{ "once", no_argument, NULL, '1' },
		{ "max-lines", required_argument, NULL, 'm' },
		{ "capture", required_argument, NULL, 'c' },
		{ "capture-bytes", required_argument, NULL, 'C' },
		{ "quiet", no_argument, NULL, 'q' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	const char *dev_path = "/dev/stpgps";
	long long start_ms;
	const char *pty_link = "/run/stpgps/nmea";
	const char *send_spec = NULL;
	const char *capture_path = NULL;
	long silence_ms = 1000;
	long max_lines = -1;
	long capture_left = 0;
	long fail_delay_ms = 200;
	int once = 0, quiet = 0;
	struct sigaction sa;
	struct repub repub;
	char *slave = NULL;
	int master, slave_fd = -1, dev_fd, opt;
	char rx[RX_CHUNK];

	while ((opt = getopt_long(argc, argv, "", opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			dev_path = optarg;
			break;
		case 'l':
			pty_link = optarg;
			break;
		case 's':
			silence_ms = strtol(optarg, NULL, 10);
			break;
		case 'S':
			send_spec = optarg;
			break;
		case '1':
			once = 1;
			break;
		case 'm':
			max_lines = strtol(optarg, NULL, 10);
			break;
		case 'c':
			capture_path = optarg;
			break;
		case 'C':
			capture_left = strtol(optarg, NULL, 10);
			break;
		case 'q':
			quiet = 1;
			break;
		case 'h':
			usage(argv[0]);
			return 0;
		default:
			usage(argv[0]);
			return 2;
		}
	}
	if (silence_ms <= 0)
		silence_ms = 1000;

	/*
	 * Signals may arrive blocked: the mask survives fork/exec, and a service
	 * manager or a test harness that starts us from a shell with SIGTERM
	 * blocked would otherwise leave us running after the stop request.
	 */
	{
		sigset_t unblock;

		sigemptyset(&unblock);
		sigaddset(&unblock, SIGINT);
		sigaddset(&unblock, SIGTERM);
		sigprocmask(SIG_UNBLOCK, &unblock, NULL);
	}
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	start_ms = now_ms();
	master = make_pty(&slave, &slave_fd);
	if (master < 0) {
		fprintf(stderr, "stpgps-bridge: cannot create a pty: %s\n",
			strerror(errno));
		return 1;
	}
	if (publish_link(slave, pty_link) != 0 && !quiet)
		fprintf(stderr, "stpgps-bridge: no symlink at %s: %s\n",
			pty_link, strerror(errno));
	if (!quiet)
		fprintf(stderr, "stpgps-bridge: NMEA on %s (link %s)\n",
			slave, pty_link);

	repub_init(&repub);

	if (capture_path) {
		if (capture_left <= 0)
			capture_left = 256;
		capture_open(capture_path, dev_path, capture_left);
	}

	dev_fd = -1;
	if (strcmp(dev_path, "-") == 0) {
		/*
		 * Standard input as the source. The engine on this chip does not
		 * speak until the host has uploaded its DSP image (see README), so
		 * the NMEA may well come from a program that drives the device -
		 * mnld behind a HAL forwarder, a capture replay - and this program
		 * should still be the thing that publishes it to gpsd, unchanged.
		 */
		dev_fd = STDIN_FILENO;
		send_spec = NULL;
	}
	if (dev_fd == STDIN_FILENO && !quiet)
		fprintf(stderr, "stpgps-bridge: reading standard input\n");

	fail_delay_ms = 200;
	while (!g_stop) {
		long long last_rx;
		int opened = 0;

		if (dev_fd < 0) {
			dev_fd = open(dev_path, O_RDWR | O_NOCTTY);
			if (dev_fd < 0) {
				if (!quiet)
					fprintf(stderr,
						"stpgps-bridge: open %s: %s, retrying in %ld ms\n",
						dev_path, strerror(errno),
						fail_delay_ms);
				usleep((useconds_t)fail_delay_ms * 1000);
				/* A second GNSS session without a connsys power cycle
				 * returns EIO, so hammering the device is pointless and
				 * noisy: back off to five seconds per attempt. */
				fail_delay_ms *= 2;
				if (fail_delay_ms > 5000)
					fail_delay_ms = 5000;
				continue;
			}
			fail_delay_ms = 200;
			opened = 1;
			repub_init(&repub);
			if (!quiet)
				fprintf(stderr, "stpgps-bridge: %s open\n",
					dev_path);
			if (send_spec) {
				if (send_bytes(dev_fd, send_spec) != 0) {
					fprintf(stderr,
						"stpgps-bridge: send failed: %s\n",
						strerror(errno));
					close(dev_fd);
					dev_fd = -1;
					continue;
				}
				send_spec = NULL; /* once per open */
			}
		}

		last_rx = now_ms();
		while (!g_stop) {
			struct pollfd pfd = { .fd = dev_fd, .events = POLLIN };
			int pr = poll(&pfd, 1, (int)silence_ms);

			if (pr < 0) {
				if (errno == EINTR)
					continue;
				break;
			}
			if (pr == 0) {
				/* Silence is what tells us the engine is gone, but a
				 * source like standard input is silent by nature; there
				 * is nothing to reopen, so keep waiting. */
				if (dev_fd == STDIN_FILENO)
					continue;
				break; /* silence: reopen below */
			}

			/*
			 * Read before acting on the error flags: a pipe whose writer
			 * has gone reports POLLIN together with POLLHUP, and breaking
			 * on the HUP first throws away the bytes that are still
			 * there - two-sentence captures lost exactly that way.
			 */
			if (pfd.revents & POLLIN) {
				ssize_t n = read(dev_fd, rx, sizeof(rx));

				if (n < 0) {
					if (errno == EINTR)
						continue;
					break;
				}
				if (n == 0)
					break;
				last_rx = now_ms();
				if (capture_path && capture_left > 0) {
					size_t take = (size_t)n;

					if ((long)take > capture_left)
						take = (size_t)capture_left;
					capture_chunk(capture_path,
						      now_ms() - start_ms,
						      (const unsigned char *)rx,
						      take);
					capture_left -= (long)take;
					if (capture_left == 0)
						fprintf(stderr,
							"stpgps-bridge: capture complete, %s\n",
							capture_path);
				}
				repub_feed(&repub, rx, (size_t)n, emit_line,
					   &master);
				if (max_lines >= 0 &&
				    (long)repub.lines >= max_lines)
					g_stop = 1;
			} else if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
				break;
			}
		}

		if (g_stop)
			break;
		if (dev_fd == STDIN_FILENO) {
			/*
			 * The source ended, the channel must not: closing the master
			 * throws away whatever the consumer has not read yet, and a
			 * forwarder upstream of us restarting must not make gpsd's
			 * device disappear. Stay up and let a signal be the stop.
			 */
			if (!quiet)
				fprintf(stderr,
					"stpgps-bridge: standard input closed, keeping the pty open\n");
			dev_fd = -1;
			if (once)
				break;
			while (!g_stop)
				pause();
			break;
		}
		/* Silence, EOF or error: the engine is gone. Drop the channel so
		 * the kernel's release() runs FUNC_CTRL(GPS off), then reopen. */
		if (!quiet)
			fprintf(stderr,
				"stpgps-bridge: %s silent %lld ms, reopening\n",
				dev_path, (long long)(now_ms() - last_rx));
		close(dev_fd);
		dev_fd = -1;
		if (opened && once)
			break;
	}

	if (dev_fd >= 0 && dev_fd != STDIN_FILENO)
		close(dev_fd);
	close(master);
	if (slave_fd >= 0)
		close(slave_fd);
	unlink(pty_link);
	free(slave);

	if (!quiet)
		fprintf(stderr,
			"stpgps-bridge: done, %llu lines, %llu framing drops\n",
			repub.lines, repub.dropped);
	return 0;
}
