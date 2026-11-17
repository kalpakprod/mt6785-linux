/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * NMEA framing for the GNSS bridge.
 *
 * The kernel side (/dev/stpgps) hands userspace one STP frame per read() and
 * every frame carries one NMEA sentence or one PMTK acknowledgement, but that
 * guarantee is the driver's, not the transport's: a frame can arrive split
 * across two reads, and two sentences can be glued into one read when the
 * transport coalesces. Everything in here therefore works on a byte stream
 * and never assumes a read boundary equals a sentence boundary.
 *
 * Deliberately free of syscalls so the same code is exercised by the host
 * tests and by the aarch64 build.
 */
#ifndef STPGPS_REPUBLISH_H
#define STPGPS_REPUBLISH_H

#include <stddef.h>

#define REPUB_BUF_SIZE 4096

typedef void (*repub_emit_fn)(const char *line, size_t len, void *ctx);

struct repub {
	char buf[REPUB_BUF_SIZE];
	size_t len;
	unsigned long long lines;
	unsigned long long dropped;
};

void repub_init(struct repub *r);

/*
 * Feed arbitrary bytes. Every complete line (terminated by '\n') is passed to
 * emit(), including the terminator, so the consumer sees the stream as the
 * engine produced it. A trailing partial line stays buffered for the next
 * call. Returns the number of lines emitted.
 */
int repub_feed(struct repub *r, const char *data, size_t n,
	       repub_emit_fn emit, void *ctx);

/* Non-zero when a partial line is waiting for the rest of its bytes. */
int repub_pending(const struct repub *r);

#endif /* STPGPS_REPUBLISH_H */
