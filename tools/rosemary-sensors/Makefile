# SPDX-License-Identifier: GPL-2.0
#
# rosemary-sensor-cal, the factory calibration replay of the Poco M5s sensor
# hub.  Not part of the kernel build: install it with "make install".

PREFIX ?= /usr/local
BINDIR ?= $(PREFIX)/bin
UNITDIR ?= /usr/local/lib/systemd/system

CFLAGS ?= -O2 -g -Wall -Wextra
CFLAGS += -std=gnu11

all: rosemary-sensor-cal

rosemary-sensor-cal: rosemary-sensor-cal.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

install: rosemary-sensor-cal
	install -Dm755 rosemary-sensor-cal $(DESTDIR)$(BINDIR)/rosemary-sensor-cal
	install -Dm644 rosemary-sensor-cal.service \
		$(DESTDIR)$(UNITDIR)/rosemary-sensor-cal.service

clean:
	rm -f rosemary-sensor-cal

.PHONY: all install clean
