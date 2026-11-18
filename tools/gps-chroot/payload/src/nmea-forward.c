/*
 * nmea-forward - receive what mnld sends to the GNSS HAL and print the NMEA.
 *
 * mnld talks to the framework over abstract UNIX datagram sockets: it takes
 * commands on "mtk_hal2mnl" and answers on "mtk_mnl2hal" (begonia-notes.md:437).
 * Abstract sockets live in the kernel's namespace, not on a filesystem, so this
 * program can run outside the chroot the daemon runs in - which is the point:
 * the engine stays in its own Android-shaped tree and the output leaves it
 * through the kernel.
 *
 * The protocol is MediaTek's own and only partly known: integer command codes
 * (201 ready, 308 version string) followed by payloads whose layout is not
 * documented anywhere in this repo. NMEA is plain ASCII, so instead of
 * pretending to know the framing this program extracts the printable runs that
 * look like sentences and prints them one per line. That is enough for gpsd,
 * and it cannot invent a sentence that was not there.
 *
 * The first thing it sends is the HAL's own greeting, because mnld blocks in
 * sendto until somebody listens on mtk_mnl2hal - a receive-only program would
 * look like a hanging daemon.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <stdint.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define SOCK_TO_MNL "mtk_hal2mnl"
#define SOCK_FROM_MNL "mtk_mnl2hal"

/* HAL -> mnld commands, from begonia's HAL emulator (begonia-notes.md:437). */
#define CMD_INIT 101
#define CMD_START 102

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

/*
 * Fill in an abstract address.
 *
 * Two details decide whether the peer can find it, and both were wrong in the
 * first version of this program:
 *
 *   - the name starts at sun_path[1]; sun_path[0] is the NUL that *makes* the
 *     address abstract. Writing the name at sun_path[0] overwrote that NUL, so
 *     the kernel read it as a filesystem path: the first run left a socket file
 *     in the working directory and every later run answered EADDRINUSE - which
 *     is exactly the "bind mtk_mnl2hal: Address in use" that started this.
 *   - the length must be what the daemon uses: bionic pads the name to the whole
 *     108-byte sun_path, so the abstract name includes the trailing zeros. An
 *     address of 13 bytes is a *different* name for the kernel.
 */
static void abstract_addr(struct sockaddr_un *addr, const char *name)
{
	memset(addr, 0, sizeof(*addr));
	addr->sun_family = AF_UNIX;
	addr->sun_path[0] = '\0';
	if (strlen(name) >= sizeof(addr->sun_path) - 1) {
		fprintf(stderr, "nmea-forward: socket name too long: %s\n", name);
		exit(2);
	}
	memcpy(addr->sun_path + 1, name, strlen(name));
}

static int bind_abstract(const char *name)
{
	struct sockaddr_un addr;
	int fd = socket(AF_UNIX, SOCK_DGRAM, 0);

	if (fd < 0)
		return -1;
	abstract_addr(&addr, name);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		int err = errno;

		close(fd);
		errno = err;
		return -1;
	}
	return fd;
}

static int send_abstract(int fd, const char *name)
{
	struct sockaddr_un addr;
	int32_t cmd[2] = { 1, 0 };

	abstract_addr(&addr, name);

	/* Two datagrams, each a little-endian "1, command" pair: init then
	 * start, which is what a HAL does when the framework comes up. */
	cmd[1] = CMD_INIT;
	if (sendto(fd, cmd, sizeof(cmd), 0, (struct sockaddr *)&addr,
		   sizeof(addr)) < 0)
		return -1;
	cmd[1] = CMD_START;
	if (sendto(fd, cmd, sizeof(cmd), 0, (struct sockaddr *)&addr,
		   sizeof(addr)) < 0)
		return -1;
	return 0;
}

/*
 * Pull out the printable, sentence-shaped runs of a datagram. A sentence starts
 * with '$' or '!' and ends before the first byte that cannot belong to one.
 */
static void emit_sentences(const unsigned char *buf, size_t len)
{
	size_t i = 0;

	while (i < len) {
		size_t start;

		while (i < len && buf[i] != '$' && buf[i] != '!')
			i++;
		if (i >= len)
			return;
		start = i;
		while (i < len && (buf[i] == '\r' || buf[i] == '\n' ||
				   (buf[i] >= 0x20 && buf[i] < 0x7f)))
			i++;
		/* A sentence needs at least "$X," and a checksum to be usable. */
		if (i - start >= 8 && memchr(buf + start, ',', i - start) &&
		    memchr(buf + start, '*', i - start)) {
			fwrite(buf + start, 1, i - start, stdout);
			fputc('\n', stdout);
			fflush(stdout);
		}
	}
}

int main(int argc, char **argv)
{
	int rx, tx;
	unsigned char buf[8192];

	(void)argc;
	(void)argv;

	signal(SIGPIPE, SIG_IGN);
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	rx = bind_abstract(SOCK_FROM_MNL);
	if (rx < 0) {
		int err = errno;

		/* Say exactly which address was tried and who holds it. With the
		 * abstract marker in place, EADDRINUSE means another process has
		 * this name; a stale filesystem socket was the old bug and cannot
		 * happen any more. */
		fprintf(stderr, "nmea-forward: bind @%s (%zu-byte abstract name): %s\n",
			SOCK_FROM_MNL, sizeof(struct sockaddr_un), strerror(err));
		if (err == EADDRINUSE) {
			fprintf(stderr, "nmea-forward: another process holds it: %s\n",
				"stop it (stop.sh) or check `ss -xa | grep mtk_mnl2hal`");
		}
		return 1;
	}
	if (getenv("GPS_ENGINE_VERBOSE"))
		fprintf(stderr, "nmea-forward: bound @%s (%zu bytes)\n",
			SOCK_FROM_MNL, sizeof(struct sockaddr_un));
	tx = socket(AF_UNIX, SOCK_DGRAM, 0);
	if (tx < 0) {
		fprintf(stderr, "nmea-forward: socket: %s\n", strerror(errno));
		return 1;
	}

	while (!g_stop) {
		struct pollfd pfd = { .fd = rx, .events = POLLIN };
		int pr = poll(&pfd, 1, 1000);

		if (pr < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (pr == 0) {
			/* Nothing from the daemon yet: keep telling it we are
			 * here, because it retries sendto and gives up. */
			send_abstract(tx, SOCK_TO_MNL);
			continue;
		}
		if (pfd.revents & POLLIN) {
			ssize_t n = recv(rx, buf, sizeof(buf), 0);

			if (n > 0)
				emit_sentences(buf, (size_t)n);
		}
	}

	close(rx);
	close(tx);
	return 0;
}
