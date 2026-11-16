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

static void usage(const char *prog)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		"  --dev PATH        STP GNSS device (default /dev/stpgps)\n"
		"  --pty-link PATH   symlink to the published pty slave\n"
		"                    (default /run/stpgps/nmea)\n"
		"  --silence-ms N    close and reopen after N ms without data\n"
		"                    (default 1000)\n"
		"  --send STR        bytes to send once after open, C escapes ok\n"
		"  --once            exit after the first silence timeout\n"
		"  --max-lines N     exit after N republished lines\n"
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
		{ "quiet", no_argument, NULL, 'q' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	const char *dev_path = "/dev/stpgps";
	const char *pty_link = "/run/stpgps/nmea";
	const char *send_spec = NULL;
	long silence_ms = 1000;
	long max_lines = -1;
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

	dev_fd = -1;
	while (!g_stop) {
		long long last_rx;
		int opened = 0;

		if (dev_fd < 0) {
			dev_fd = open(dev_path, O_RDWR | O_NOCTTY);
			if (dev_fd < 0) {
				if (!quiet)
					fprintf(stderr,
						"stpgps-bridge: open %s: %s\n",
						dev_path, strerror(errno));
				usleep(200 * 1000);
				continue;
			}
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
			if (pr == 0)
				break; /* silence: reopen below */

			if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
				break;

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
				repub_feed(&repub, rx, (size_t)n, emit_line,
					   &master);
				if (max_lines >= 0 &&
				    (long)repub.lines >= max_lines)
					g_stop = 1;
			}
		}

		if (g_stop)
			break;
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

	if (dev_fd >= 0)
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
