# gps-chroot — the vendor GNSS engine in a small environment

Route (a) of the GPS plan in `scratch/wt/connectivity/REPORT.md`: the MT6785 GPS engine is not
something the kernel can start. The chip stays silent until the host uploads its DSP image, and that
image lives inside the closed `libmnl.so` that `mnld` loads (`scratch/native/2-connectivity/logs/
begonia-notes.md:425`, `:441`, `:442`; `$PMTK010,001` and NMEA at 1 Hz only appear after it:
`:750`). So the engine has to be run as what it is: Android vendor binaries, on top of the kernel
interfaces we already wrote.

Everything in this directory works on the host. Nothing here runs a vendor binary, and nothing reads
or writes the phone.

## The file set

`./assemble.py --super <super.img> --out <dir>` reads the dynamic-partition metadata of the phone's
own `super.img`, carves the four partitions it needs, extracts the vendor files and resolves the
ELF dependency closure. `filelist.txt` is its output: 45 files, 24.3 MiB, every `NEEDED` among them
resolved (245 edges, zero missing). Nothing in it is guessed.

| group | files | what it is |
|---|---|---|
| engine | `mnld`, `libmnl.so`, `libviagpsrpc.so`, `mtk_agpsd` | the daemon, the closed GNSS engine with the DSP images inside it, the RPC helper for the modem side, the A-GPS daemon |
| vendor libraries | 28 files from `/lib64` (34 vendor rows minus the 6 engine/config files) | `libc++.so`, `libcrypto`/`libssl`, `libcurl`, `libnvram`, `libDR`, the HIDL sensor glue, and their own dependencies |
| framework libraries | 7 files from `/system` and `/system_ext` | `liblog`, `libbinder_ndk`, `libapexsupport`, `libvndksupport`, `libandroid_net`, ... |
| bionic | 4 files: `libc.so`, `libm.so`, `libdl.so`, `libdl_android.so` | taken from `/system/apex/com.android.runtime.apex` inside the system image, i.e. the same runtime the phone boots |
| configuration | `/etc/MNL_Config.xml`, `/etc/gnss/agps_profiles_conf2.xml` | GNSS mode (GP+GL+GA+BD) and the A-GPS profiles |

The interpreter is `/system/bin/linker64` (`readelf -l mnld`), which on the phone is a symlink into
the runtime apex - the tree keeps that shape.

## The tree

`assemble.py` lays the files out exactly where the binaries look for them. The environment is a
directory, not an image, so it can be inspected and changed with ordinary tools:

    root/
      system/bin/linker64 -> /apex/com.android.runtime/bin/linker64
      system/lib64/{liblog,libbinder_ndk,...}.so          symlinks to what lives in the apex
      apex/com.android.runtime/{bin/linker64,lib64/bionic/*.so}
      vendor/bin/{mnld,mtk_agpsd}
      vendor/lib64/*.so
      vendor/etc/{MNL_Config.xml,gnss/*}
      data/vendor/gps/          assistance data and logs (EPO.DAT, MTKNAV.DAT, gps.conf, mnl.prop)
      data/vendor/log/gps/
      data/agps_supl/           the two UNIX sockets mnld and mtk_agpsd share
      dev/  proc/  sys/         bind mounts from the host

`mnld` writes `gpsdebug.log`, `ADC.txt` and the EPO/MTKNAV files into `/data/vendor/gps`, and the
daemon's own strings name all of them, so the directory has to be writable by the same user the
daemon runs as.

## What it needs from the framework, and how little that is

Not a property service. `mnld` reads 16 properties and nothing here needs the Android framework to
answer them; a property area file (`/dev/__properties__`, the bionic `prop_area` format) is enough,
which is how begonia ran it (`begonia-notes.md:747-750`). The keys and the values this phone reports:

| property | value on this phone |
|---|---|
| `persist.vendor.connsys.chipid` | `0x6785` |
| `vendor.gps.gps.version` | `0x6785` |
| `vendor.gps.clock.type` | `21` |
| `vendor.connsys.adie.chipid` | `0x0000` |
| `vendor.connsys.driver.ready` | `yes` |
| `vendor.service.nvram_init` | `Ready` |
| `ro.build.type` / `ro.debuggable` | `user` / `0` |
| `ro.vendor.mtk_log_hide_gps` | `0` |

and the debug keys (`debug.mnl`, `debug.debug_nmea`, `vendor.debug.gps.*`, `vendor.gpsdbglog.*`)
which only turn logging on. Without `persist.vendor.connsys.chipid` mnld reports "chip is unknown,
version 0x0" and exits; without `vendor.service.nvram_init=Ready` it waits for NVRAM and exits
(both observed on begonia).

Not the GNSS HAL either. `mnld` talks to the framework over two abstract UNIX sockets,
`mtk_hal2mnl` (commands 101 init, 102 start, 103 stop, 104 cleanup; `int32 LE` code) and
`mtk_mnl2hal` (answers 201 ready, 308 version string, and the NMEA/status stream),
`begonia-notes.md:437`. A listener on the second socket is what receives the sentences, and it is
also what stops `mnld` from blocking in `sendto`: the forwarder is a small reader, and
`stpgps-bridge --dev -` publishes whatever it prints to gpsd unchanged.

So the whole environment is: a directory, a property-area file, two sockets, one bind-mounted `/dev`,
and something that reads `mtk_mnl2hal`.

## What it needs from our kernel

* `/dev/stpgps` — `open()` powers the GNSS function (`FUNC_CTRL(GPS on)`, which the vendor does
  before it checks the STP channel), `read()` returns STP task-2 frames (the DSP upload and all
  later traffic ride this channel), `release()` powers it off. Ioctls the daemon calls, from the
  vendor driver `gps_stp/stp_chrdev_gps.c:64-79`: 6, 7, 8 (HW/FW version), 9 (D1 efuse), 10 (RTC
  flag), 11 (co-clock), 12 (trigger assert), 13 (WMT status), 14/15 (wake lock), 16 (LNA pin),
  17 (FW control), 18/19 (suspend/resume), 20 (listen for reset, blocks until close), 21 (MD status).
  Our driver (`drivers/gnss/stpgps.c`) answers every one of them the way the vendor driver does:
  7/8 from the chip, 9 with 0 (the vendor logs "Read Efuse not supported in this platform" and
  answers 0 on MT6785), 10 with 0, 11 from `mtk_wcn_wmt_co_clock_flag_get()`, 12 through
  `mtk_wcn_wmt_assert()`, 13 with the vendor's -888/-889/0 reset states (fed by the WMT reset
  callback), 14/15 through a real wakeup source, 16 from the pin number, 17 through
  `mtk_wmt_gps_mcu_ctrl()`, 20 which wakes on a close **or** a chip reset, and 18/19/21 as -EFAULT
  like the vendor's unimplemented ones on this platform. That work is branches
  `area/connectivity-h6` (read/write path, no oops), `-h9` (the ioctl set and the byte counters) and
  `-h10` (fwctl); before it, the daemon's own status polls were answered with failures.
* `/dev/gps_emi` — ioctl 1 (the daemon's "window is ready", answered 1), 2/3 (NVRAM copies, 0),
  4 (physical address of the window), plus `mmap`. Our driver has all four; begonia observed
  `mnld` calling exactly 1 and 4 (`begonia-notes.md:442`).
* Optional nodes the daemon probes and the port does not create: `/dev/gps_pwr`, `/dev/gps2scp`,
  `/dev/ttygserial`, `/dev/ttyGS3`. On begonia the engine ran with none of them, so their absence
  is expected rather than a fault - but it is an expectation, not a measurement on our phone, and
  the first trial has to confirm it in `mnld`'s log.
* A connsys power cycle between sessions. A second session in a row returns `EIO`
  (`begonia-notes.md:752`), which is kernel-side work: a GPS off that actually power-cycles the
  chip, instead of only dropping the function.

## The sequence that produces the first byte, and what the bridge cannot do

The chip says nothing until four things have happened, and three of them are the daemon's:

| # | who | what | on our side |
|---|---|---|---|
| a | `mnld` | `open("/dev/stpgps")` | `stpgps_open()` = `FUNC_CTRL(GPS on)`; the type-2 STP channel exists |
| b | `mnld` | open `/dev/gps_emi`, `ioctl(1)` (window ready), `ioctl(4)` (physical base), `mmap` | answered and mapped out of the real CONNSYS carveout |
| c | `mnld` | **the GNSS patch/DSP download**: `ioctl(17)` = `COMBO_IOC_GPS_FWCTL` → `mtk_wmt_gps_mcu_ctrl()` (a WMT op wrapping the payload in `01 32 00 00`), plus the image and command stream written to `/dev/stpgps` as type-2 frames | fwctl implemented; `write()` returns what STP took, `-ENOSPC` when it took nothing, and the kernel reports the totals on close (`GNSS channel closed: tx ... B`) |
| d | `mnld` | PMTK out, NMEA back at 1 Hz, handed to the HAL on `mtk_mnl2hal` | frames land in `rx_list` from the RX work, so `read()`/`poll()` see them; `nmea-forward` plays the HAL |

**`stpgps-bridge` alone does (a) and (d) only.** It opens the channel and reads it; it never asks the
kernel for fwctl and never writes a frame, so on its own it can only ever report silence - which is
exactly what every trial without the engine showed. The bytes of (c) are not in any file this system
can load:

* the stock kernel module `gps_drv.ko` (this phone's own, `scratch/stock-dump/vendor-ko/`) imports
  `mtk_wmt_gps_mcu_ctrl`, `mtk_wmt_gps_suspend_ctrl` and the STP calls, and **no `request_firmware`** -
  the kernel loads no GPS firmware on this SoC;
* the phone's 106 vendor firmware files (`device-dump-2026-11-13/stock/vendor-firmware.sha256`) hold
  none: `soc1_0_patch_mcu_2a_1_hdr.bin`, `soc1_0_ram_{mcu,bt,wifi}_2a_1_hdr.bin`,
  `WIFI_RAM_CODE_soc1_0_2a_1.bin`, `mt6631_fm_v1_patch.bin`, touch/amp/SMART-PA blobs - and nothing
  named gps, gnss, mnl, arc or bee;
* the engine is inside `libmnl.so`: `_BootDSP_UART_V2()`, `BootDSP: start,%d`,
  `code_frag_num,dsp_frag_size`, `COMD4`, `fgPatchApx`, a per-chip table carrying `MT6768_E1 DSP`
  (this SoC's family), and no patch file name anywhere in its strings. `ARC.BIN`, `BEE.BIN` and
  `STDTESTIF.BIN` in its data are entries of its own module-dispatch table, and no such file exists
  under `/vendor`.

So there is nothing to copy from a stock partition: the missing piece is the engine itself, and
`deploy.sh` below is how it gets onto the phone.

## The order the connsys needs: Wi-Fi first, then GPS

Measured on the phone (REPORT.md, LEAD-NOTE 30): if the GPS function is the first consys user after a
boot, `FUNC_CTRL(GPS on)` fails and takes the chip with it -

```
wmt_cmd_test cmd len err ...
WMT-CORE: get hwcode (chip id) fail
consys_hw_reset_bit_set: connector MCU did not boot: 0x18002600=0x0 after 11 polls
Read CONSYS version id(0x00000000)
connsys clock check fail 0x18007000(0x0)
```

- and every later attempt fails too until the phone reboots. With Wi-Fi up first the same call answers
`OPID(3) type(2) ok`.

The code path says why this cannot be a GPS-path bug: `opfunc_func_on()` (`wmt_core.c:1255`) powers the
chip through the *shared* `opfunc_pwr_on()` before it looks at the function type at all, and the power-on
sequence (`wmt_core_ctrl(WMT_CTRL_HW_PWR_ON)` → `mtk_wcn_consys_hw_reg_ctrl()`) is the same for both. The
only difference for Wi-Fi is that `opfunc_func_on()` hands the rest to `WMT_OPID_WLAN_PROBE`, i.e. to
`wlan_gen4m`'s own probe, after the chip is up. GPS's extra steps (`wmt_func_gps_on()`: PALDO/VCN28 and
the LNA) are gated on `co_clock_flag`, which is 0 on this board, so they are skipped. The failure above
happens *inside* the shared power-on (the MCU does not boot), which means the state of the chip at that
moment is wrong, not the GPS function's code - and the kernel's own headless MCU patch step
(`wmt_dev_srh_patch_headless()`, which publishes `soc1_0_patch_mcu_2a_1_hdr.bin` because this port has
no `stp_uart_launcher`) is the step that has to have completed for the MCU to boot at all.

`start.sh` therefore brings Wi-Fi up first when nothing has yet (`printf 1 > /dev/wmtWifi`, wait for
`wlan0`), and only then starts the daemon. `GPS_ENGINE_SKIP_WIFI=1` skips that, for a trial that wants
the failure.

## Reading mnld's own log

`mnld` reports through Android's `liblog`, and this environment has no `logd`: the lines went nowhere.
The preloaded shim now also defines `__android_log_print`, `__android_log_vprint`, `__android_log_write`,
`__android_log_buf_write` and `__android_log_assert`, and writes them to stderr, which `start.sh` sends
to `/var/log/gps-engine/mnld-<stamp>.log`. The formatter covers `%s %c %d %i %u %x %X %p` with the
`hh/h/l/ll/z` modifiers; anything else is printed literally rather than guessed at, and a bad format
string cannot take the daemon down.

On the kernel side the first frames of each direction are dumped raw, so a short exchange can be read
without a protocol decoder:

```
GNSS tx frame 116 B (first 48): aa 0f 32 ...
GNSS rx frame 20 B: ...
GNSS channel closed: tx 116 B in 1 frames, rx 20 B in 1 frames
```

## Deploying

    ./deploy.sh                 # host: extracts, builds the helpers, writes out/gps-engine-*.tar.gz

On the phone, one command:

    tar -xzf gps-engine-*.tar.gz && sudo sh install.sh

`install.sh` places the tree in `/opt/gps-engine`, the forwarder in
`/usr/local/bin/nmea-forward`, the property shim inside the tree, and then runs `check.sh`, which
prints PASS or FAIL for every prerequisite:

* the two device nodes exist,
* the tree holds every file the runtime needs,
* **bionic's own linker resolves mnld's whole closure inside the tree** (`linker64 --list`), which is
  the only check that cannot be faked,
* `nmea-forward` binds `mtk_mnl2hal` and waits.

It deliberately does not start the daemon. `start.sh` does that, and it is where a vendor binary
first runs against the chip:

    /opt/gps-engine/start.sh      # forwarder + chroot mnld, logs in /var/log/gps-engine
    tail -f /var/log/gps-engine/mnld.log
    /opt/gps-engine/stop.sh

The helpers are built with zig for aarch64: `nmea-forward` is static musl, and `libprops-shim.so` is
built with `-nostdlib` on purpose - its only external symbols are `open`, `read`, `close` and
`strlen`, which the bionic process already has, so preloading it cannot drag a second C library into
an Android binary.

## Reproducing

    ./assemble.py --super ~/poco-m5s/backup-2026-11-14/super.img --out /path/to/tree

Needs `debugfs` (e2fsprogs), `dd`, `python3`, `readelf` and nothing else; the partitions are read
from the image, never from the phone. `filelist.txt` is written next to this file and is what a
review reads.
