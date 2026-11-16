# stpgps-bridge

Republishes the MT6785 (Poco M5s `rosemary`) GNSS engine's NMEA from
`/dev/stpgps` to a pty, so that `gpsd` can read it. Route A of
`scratch/wt/connectivity/REPORT.md`, "Userspace plan for NMEA in gpsd".

C, libc only, no threads. The phone build is static:

    make phone        # zig cc -target aarch64-linux-musl -static
    make test         # host: five scenarios, all must pass
    make test-fault   # host: the planted fault must be caught
    make show-fault-fails   # host: the same, from a reader's point of view

## What the program does

Two ways to give it a source: a character device (the normal case) or standard input (`--dev -`, for NMEA
produced by another program - see "What that means" below).

    stpgps-bridge --dev /dev/stpgps --pty-link /run/stpgps/nmea
    mnld-forward | stpgps-bridge --dev - --pty-link /run/stpgps/nmea

1. Creates a pty, keeps **both** ends open (a write to the master whose slave
   nobody has opened yet is discarded by the tty layer, so the first sentences
   of a session would vanish while `gpsd` is still starting), and publishes the
   slave path as a symlink.
2. Opens `/dev/stpgps`, which is `FUNC_CTRL(GPS on)` on the kernel side. With
   `--dev -` it reads standard input instead: silence never reopens it, and
   when the source ends the pty stays up (a forwarder restarting must not make
   gpsd's device vanish) until a signal stops the program.
3. Reads frames, splits them on `\n` and writes the sentences unchanged to the
   pty. It never assumes one `read()` is one sentence: a sentence may arrive
   split across two reads and two sentences may arrive in one read.
4. If no byte arrives for `--silence-ms` (default 1000), it closes the device -
   which runs the kernel's `FUNC_CTRL(GPS off)` - and reopens. A second GNSS
   session without a connsys power cycle returns `EIO`, so a failed open backs
   off from 200 ms to five seconds per attempt instead of hammering the
   device.
5. `--send 'STR'` writes a byte sequence once after each open; this is how a
   proven start sequence will be injected without touching the code.
6. `--capture PATH --capture-bytes N` records the first N bytes the engine sends
   as `<ms> <hex bytes>` lines. One session then answers the open question
   ("does the engine say `$PMTK010,001` first, or something binary?") while the
   bridge keeps serving gpsd. The file is plain text, so it can be pasted into a
   report as it is, and the host test decodes it back and requires it to equal
   the stream byte for byte.

## What is proven, and from where

Every claim below was read in this repo; the file:line is the source.

* **The channel is STP task 2 and open() powers the engine.** In the vendor tree,
  `gps_stp/stp_chrdev_gps.c`, `GPS_open()`: `mtk_wcn_wmt_func_on(WMTDRV_TYPE_GPS)` at `:1081` (failure is
  `-ENODEV`), then `mtk_wcn_stp_is_ready()` at `:1092` - when it is false the call returns `-ENODEV` without
  touching the channel - then `mtk_wcn_stp_register_event_cb(GPS_TASK_INDX, GPS_event_cb)` at `:1100`.
  Our kernel driver does the same.

* **The daemon exists and runs on this phone.** `poco-m5s/device-dump-2026-11-13/stock/getprop-filtered.txt:216`
  `[init.svc.mnld]: [running]`, `:1040` `ro.vendor.mtk_gps_support=1`, `:1171` `vendor.gps.clock.type=21`,
  `:1172` `vendor.gps.gps.version=0x6785`.

* **This phone does not build the legacy EMI device.** `proc-config.gz` (same dump) reports
  `# CONFIG_MTK_GPS_EMI is not set`, and `gps_stp/gps_emi.c` is wrapped in `#ifdef CONFIG_MTK_GPS_EMI`,
  so that file is not compiled on the phone at all.

* **The modern stack's `/dev/gps_emi` is a stub.** `data_link/linux/gps_dl_emi.c:36-37` define the two
  ioctls, the device is created at `:187`/`:209`, and each ioctl handler only logs. So on stock that device
  neither reserves memory nor returns an address.

* **NMEA sentences are ASCII and newline-terminated** - standard NMEA 0183.

And the three facts this note asked for, which **are** answered - not by a file in the dump, but by the
project's own worked port on this chip family (`scratch/native/2-connectivity/logs/begonia-notes.md`, a
Poco X3 NFC / begonia, HCR `0x6632`, the same CONNSYS GPS block as `0x6785`):

1. **Where the DSP image comes from.** Not from a file: `begonia-notes.md:425` - "There is no separate GPS
   firmware in /vendor/firmware (only patch_mcu, ram_{mcu,bt,wifi}, fm)". The image lives inside the closed
   `libmnl.so` that `mnld` loads, and `mnld` **uploads ~40 KB into the chip over this same channel**
   (`:442`: `tx 39721 B`, ~7-28 KB of answers). Our own dump agrees: `vendor-firmware.sha256` has 106
   entries and not one of them is GPS, and the WMT patch download logs `There is no need to download (2)
   type patch!` for GPS in every trial.
2. **The start sequence.** The chip answers only after that upload: `begonia-notes.md:441` - "without
   firmware the chip is silent to PMTK (rx 0 B) -> the host uploads the DSP"; `:424` - during 8 s of
   `FUNC_CTRL(GPS on)` with no upload, "NOT A SINGLE frame of type 2 came over STP". So the sequence is not
   a set of PMTK commands we could send: it is the vendor's DSP upload protocol, 40 KB of it, spoken by
   `libmnl`.
3. **What comes back first.** With the DSP loaded: `$PMTK010,001`, then NMEA at 1 Hz, GGA/RMC/GSA and no GSV
   (`begonia-notes.md:750`).

### What that means for this program

`stpgps-bridge` cannot make the engine speak, and it never could: on this chip the host uploads the engine's
DSP, and that upload is inside a closed vendor library. The bridge's job is the part that is provable and
still needed - owning the character device, keeping the channel alive, and giving gpsd a stable pty - and it
now also reads its source from standard input (`--dev -`) so that the NMEA produced by whatever does the
upload is published the same way. The three routes and their real costs:

| route | what it needs | cost |
|---|---|---|
| (a) stock `mnld` + `libmnl` in a small chroot, as begonia did | the phone's own `/vendor` files, our `/dev/stpgps` and `/dev/gps_emi`, and a GNSS HAL forwarder that turns `mnld`'s socket output into NMEA | proven on hardware once already (`begonia-notes.md:747-750`); the forwarder is a small program, and this bridge publishes its output |
| (b) reverse `libmnl`'s upload and write our own engine interface | reversing ~1.2 MB of `.text` | weeks; begonia's notes say an open engine is impossible without it |
| (c) accept that this milestone ends at "the channel works" | nothing | GPS produces no position, by decision |

Route (a) is the only one that has already produced NMEA on this hardware, and the bridge is what makes its
output usable by gpsd. Two facts from that trial shape the bridge's design and are honoured here: a second
session in a row without a connsys power cycle returns `EIO` (`begonia-notes.md:752`), so the reopen path
backs off to five seconds instead of hammering the device; and the engine's DSP upload goes through
`/dev/gps_emi` ioctls 1 and 4 (`:442`), which is exactly the ioctl set our kernel driver implements.

## Host tests, and how they can fail

`test-bridge.c` gives the bridge a pty pair as `--dev` - the same interface the kernel driver offers, one
`read()` per STP frame - and reads the bridge's own pty exactly as `gpsd` will.

| scenario | input | requirement |
|---|---|---|
| `one-sentence-per-frame` | three sentences, one per write | all three republished, in order, unchanged |
| `sentence-split-across-frames` | one sentence cut in half, 200 ms apart | still republished as one sentence |
| `two-sentences-glued-in-one-frame` | two sentences in one write | republished as two sentences |
| `capture-file-equals-the-stream` | two sentences, capture on | the capture decodes back to the same bytes |
| `stdin-source` | two sentences on `--dev -` | published the same way, and the pty outlives the source |

Both pty ends are put in raw mode by the harness; without that, a pty's line discipline would hold the
first half of a sentence until its newline and the "split frame" scenario would silently test nothing.
(The harness itself taught this: before raw mode, the split scenario passed against a bridge that
discards partial lines.)

`make show-fault-fails` builds the same program with `-DSTPGPS_NAIVE_FRAMING`, which treats each read as a
complete line, throws away any unterminated tail - the mistake a hand-written bridge actually makes - and
loses the first captured byte, and requires the harness to reject both:

    FAIL sentence-split-across-frames: expected the published stream to match the engine's
    EXPECTED: the broken bridge failed the test

## Installing on the phone

    install -Dm755 stpgps-bridge-aarch64-linux-musl /usr/local/bin/stpgps-bridge
    install -Dm644 stpgps-bridge.service /etc/systemd/system/stpgps-bridge.service
    install -Dm644 stpgps-bridge-gpsd.conf \
        /etc/systemd/system/gpsd.service.d/stpgps-bridge.conf
    systemctl daemon-reload
    systemctl enable --now stpgps-bridge
    systemctl restart gpsd

The binary is static, so nothing else has to be installed and no library version
on the phone matters. `/run/stpgps/nmea` appears once the bridge is up; `cgps`
or `gpsmon` shows whether gpsd found it.

To answer "what does the engine say first" in the same session:

    stpgps-bridge --dev /dev/stpgps --pty-link /run/stpgps/nmea
    mnld-forward | stpgps-bridge --dev - --pty-link /run/stpgps/nmea \
        --capture /run/stpgps/first-bytes.txt --capture-bytes 256

## Files

    stpgps-bridge.c   device or stdin source, pty publication, reopen loop, --send
    republish.c/.h    byte stream -> NMEA lines, no syscalls, unit-testable
    test-bridge.c     pty-pair harness, five scenarios plus the fault mode
    stpgps-bridge.service   systemd unit for the phone
    stpgps-bridge-gpsd.conf gpsd drop-in that points gpsd at the pty
    Makefile          host build, zig static aarch64 build, tests
