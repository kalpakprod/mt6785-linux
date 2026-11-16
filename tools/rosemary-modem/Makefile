# SPDX-License-Identifier: MIT
#
# Userspace for the Poco M5s modem (kernel driver rosemary-ccci).
#
# Not built here: the host has no aarch64 cross compiler and no target
# libraries, and the lead builds these on the phone. `cc` on the phone with
# these flags produces the binary; there is nothing to configure.
#
#   PREFIX=/usr/local make install

CC	?= cc
CFLAGS	?= -O2 -Wall -Wextra
PREFIX	?= /usr/local
DESTDIR	?=

BIN	= rosemary-modemd
SRC	= rosemary-modemd.c

all: $(BIN)

$(BIN): $(SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $(SRC)

# The NVRAM import, the unit and the udev rule are installed as they are.
install: $(BIN)
	install -Dm755 $(BIN) "$(DESTDIR)$(PREFIX)/bin/$(BIN)"
	install -Dm755 rosemary-modem-nvram-import \
		"$(DESTDIR)$(PREFIX)/bin/rosemary-modem-nvram-import"
	install -Dm644 rosemary-modem.service \
		"$(DESTDIR)/usr/lib/systemd/system/rosemary-modem.service"
	install -Dm644 78-rosemary-modem.rules \
		"$(DESTDIR)/usr/lib/udev/rules.d/78-rosemary-modem.rules"
	install -Dm644 rosemary-modem-properties \
		"$(DESTDIR)/usr/share/rosemary-modem/properties"

clean:
	rm -f $(BIN)

.PHONY: all install clean
