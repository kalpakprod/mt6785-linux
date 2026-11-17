# stpgps-bridge - republish the MT6785 GNSS NMEA to a pty for gpsd
#
# Host build is for the tests; the phone build is static aarch64 with zig cc,
# which needs no sysroot for this program (libc only, no threads).

CC          ?= cc
ZIG         ?= zig
CFLAGS      ?= -O2 -Wall -Wextra -Werror
TARGET_ARCH ?= aarch64-linux-musl

all: stpgps-bridge

stpgps-bridge: stpgps-bridge.o republish.o
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c republish.h
	$(CC) $(CFLAGS) -c -o $@ $<

# The deliberately naive framing, used only to prove the test can fail. It gets
# its own objects: a target-specific CFLAGS would not rebuild the shared ones,
# and the "fault" binary would silently be the correct one.
stpgps-bridge-naive: stpgps-bridge-naive.o republish-naive.o
	$(CC) $(CFLAGS) -o $@ $^

stpgps-bridge-naive.o: stpgps-bridge.c republish.h
	$(CC) $(CFLAGS) -DSTPGPS_NAIVE_FRAMING -c -o $@ $<

republish-naive.o: republish.c republish.h
	$(CC) $(CFLAGS) -DSTPGPS_NAIVE_FRAMING -c -o $@ $<

test-bridge: test-bridge.c
	$(CC) $(CFLAGS) -o $@ test-bridge.c

test: stpgps-bridge test-bridge
	./test-bridge ./stpgps-bridge

# The planted fault must be *caught*: the naive bridge is asked to satisfy the
# correct expectation and the harness has to reject it.
test-fault: stpgps-bridge-naive test-bridge
	./test-bridge ./stpgps-bridge-naive --fault

# Same thing seen from the other side, for a human reading the log: the harness
# run against the naive bridge must exit non-zero.
show-fault-fails: stpgps-bridge-naive test-bridge
	@if ./test-bridge ./stpgps-bridge-naive; then \
		echo "UNEXPECTED: the broken bridge passed"; exit 1; \
	else \
		echo "EXPECTED: the broken bridge failed the test"; fi

# Phone binary: static, so it runs on a system with no musl and no glibc.
phone: stpgps-bridge.c republish.c republish.h
	$(ZIG) cc -target $(TARGET_ARCH) -O2 -Wall -Wextra -Werror -s \
		-static -o stpgps-bridge-$(TARGET_ARCH) \
		stpgps-bridge.c republish.c

clean:
	rm -f *.o stpgps-bridge stpgps-bridge-naive test-bridge \
		stpgps-bridge-$(TARGET_ARCH)

.PHONY: all test test-fault show-fault-fails phone clean
