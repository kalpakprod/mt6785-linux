# stpgps-bridge

Republishes the MT6785 (Poco M5s `rosemary`) GNSS engine's NMEA from
`/dev/stpgps` to a pty, so that `gpsd` can read it. Route A of
`scratch/wt/connectivity/REPORT.md`, "Userspace plan for NMEA in gpsd".

C, libc only, no threads. The phone build is static:

    make phone        # zig cc -target aarch64-linux-musl -static
    make test         # host: three pty scenarios, all must pass
    make test-fault   # host: the planted fault must be caught
    make show-fault-fails   # host: the same, from a reader's point of view

## What the program does

    stpgps-bridge --dev /dev/stpgps --pty-link /run/stpgps/nmea

1. Creates a pty, keeps **both** ends open (a write to the master whose slave
   nobody has opened yet is discarded by the tty layer, so the first sentences
   of a session would vanish while `gpsd` is still starting), and publishes the
   slave path as a symlink.
2. Opens `/dev/stpgps`, which is `FUNC_CTRL(GPS on)` on the kernel side.
3. Reads frames, splits them on `\n` and writes the sentences unchanged to the
   pty. It never assumes one `read()` is one sentence: a sentence may arrive
   split across two reads and two sentences may arrive in one read.
4. If no byte arrives for `--silence-ms` (default 1000), it closes the device -
   which runs the kernel's `FUNC_CTRL(GPS off)` - and reopens, because a second
   GNSS session without a connsys power cycle is known to return `EIO`.
5. `--send 'STR'` writes a byte sequence once after each open; this is how a
   proven start sequence will be injected without touching the code.

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

* **A DSP image load into the EMI window is named but disabled.**
  `gps_stp/gps_emi.c:249` uses `pFullPatchName = "MNL.bin"` and copies it into the window - inside the
  `#if 0` block that spans `:232-284`. So the idea exists in vendor code and is switched off in this drop.

* **NMEA sentences are ASCII and newline-terminated** - standard NMEA 0183, and begonia's port notes
  (`poco-m5s/research/native/2-connectivity.md` §4.5) record this engine answering `$PMTK010,001` and
  emitting GGA/RMC/GSA at 1 Hz.

## What is missing, and why the program stops before it

The note that asked for this program said: if the start sequence cannot be proven, stop at step 3 and
write what is missing. It cannot be proven from this repo, and here is precisely what is absent:

1. **The DSP image.** No `MNL.bin`, no GPS firmware of any kind, is in the repo: the stock dump carries
   `vendor-firmware.sha256` (106 entries) and none of them is GPS (`grep -iE 'gps|gnss|mnl'` → 0 hits);
   the WMT patch set this port already downloads contains only `soc1_0_ram_{bt,wifi,mcu}` and logs
   `There is no need to download (2) type patch!` for GPS in every trial. Begonia's port reached NMEA
   without loading anything itself, which suggests the engine image arrives through a path we have not
   read. **Needed:** the file list and the contents of `/vendor/etc/gps/`, `/vendor/lib*/libmnl*.so` and
   `/vendor/bin/mnld` from the phone, or the vendor HAL source.
2. **The start sequence.** Without `mnld` or the HAL source there is no way to tell PMTK from a binary
   blob, and guessing it would violate the project rule against trial-and-error on the phone.
   **Needed:** `strings /vendor/bin/mnld` (or `libmnl.so`) filtered for `$PMTK`, plus the GPS config in
   `/vendor/etc/gps/` (`gps.conf`, `mnl.prop`).
3. **The engine's first bytes.** The same sources would show whether the first output is `$PMTK010,001`
   (boot notification) or something binary. **Needed:** a single capture from the phone of `cat /dev/stpgps`
   with the engine powered on, hex-dumped.

Until those exist, the bridge does everything that is proven and nothing that is guessed: it opens the
device, republishes whatever the engine emits, survives silence, and can inject a sequence given on the
command line the moment one is proven.

## Host tests, and how they can fail

`test-bridge.c` gives the bridge a pty pair as `--dev` - the same interface the kernel driver offers, one
`read()` per STP frame - and reads the bridge's own pty exactly as `gpsd` will.

| scenario | input | requirement |
|---|---|---|
| `one-sentence-per-frame` | three sentences, one per write | all three republished, in order, unchanged |
| `sentence-split-across-frames` | one sentence cut in half, 200 ms apart | still republished as one sentence |
| `two-sentences-glued-in-one-frame` | two sentences in one write | republished as two sentences |

Both pty ends are put in raw mode by the harness; without that, a pty's line discipline would hold the
first half of a sentence until its newline and the "split frame" scenario would silently test nothing.
(The harness itself taught this: before raw mode, the split scenario passed against a bridge that
discards partial lines.)

`make show-fault-fails` builds the same program with `-DSTPGPS_NAIVE_FRAMING`, which treats each read as a
complete line and throws away any unterminated tail - the mistake a hand-written bridge actually makes -
and requires the harness to reject it:

    FAIL sentence-split-across-frames: expected the published stream to match the engine's
    EXPECTED: the broken bridge failed the test

## Files

    stpgps-bridge.c   device handling, pty publication, reopen loop, --send
    republish.c/.h    byte stream -> NMEA lines, no syscalls, unit-testable
    test-bridge.c     pty-pair harness, three scenarios plus the fault mode
    Makefile          host build, zig static aarch64 build, tests
