// SPDX-License-Identifier: GPL-2.0-only
/*
 * NMEA framing for the GNSS bridge. See republish.h for why the read boundary
 * is not trusted.
 */
#include "republish.h"

#include <string.h>

void repub_init(struct repub *r)
{
	r->len = 0;
	r->lines = 0;
	r->dropped = 0;
}

int repub_pending(const struct repub *r)
{
	return r->len != 0;
}

static void repub_drop_prefix(struct repub *r, size_t n)
{
	if (n >= r->len) {
		r->len = 0;
		return;
	}
	memmove(r->buf, r->buf + n, r->len - n);
	r->len -= n;
}

int repub_feed(struct repub *r, const char *data, size_t n,
	       repub_emit_fn emit, void *ctx)
{
	size_t off = 0;
	int emitted = 0;

#ifdef STPGPS_NAIVE_FRAMING
	/*
	 * Deliberately wrong, and only compiled for the fault test. This is the
	 * mistake a hand-written bridge actually makes: it treats each read() as
	 * a complete line and throws away the tail that has no terminator yet.
	 * Nothing is reordered and no byte is corrupted, so only a test that can
	 * move a frame boundary catches it - which is exactly the planted fault.
	 */
	{
		size_t i;
		size_t cut = 0;

		for (i = 0; i < n; i++) {
			if (data[i] == '\n')
				cut = i + 1;
		}
		if (cut == 0) {
			r->dropped++;
			return 0;
		}
		emit(data, cut, ctx);
		r->lines++;
		return 1;
	}
#endif

	while (off < n) {
		size_t space = REPUB_BUF_SIZE - r->len;
		size_t chunk = n - off;
		char *nl;
		size_t line_len;

		if (chunk > space) {
			/* The line is longer than any NMEA sentence; keep the
			 * buffer bounded and count the loss instead of
			 * silently truncating into a malformed sentence. */
			chunk = space;
			r->dropped++;
		}

		memcpy(r->buf + r->len, data + off, chunk);
		r->len += chunk;
		off += chunk;

		/* Emit every complete line this chunk completed. */
		while ((nl = memchr(r->buf, '\n', r->len)) != NULL) {
			line_len = (size_t)(nl - r->buf) + 1;
			emit(r->buf, line_len, ctx);
			r->lines++;
			emitted++;
			repub_drop_prefix(r, line_len);
		}

		if (r->len == REPUB_BUF_SIZE) {
			/* No terminator in a full buffer: a framing loss. Drop
			 * what is there so the next sentence can resynchronise
			 * instead of being appended to garbage. */
			r->dropped++;
			r->len = 0;
		}
	}

	return emitted;
}
