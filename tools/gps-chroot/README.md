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
  Our driver (`drivers/gnss/stpgps.c`) answers 7, 8, 16 in full and blocks in 20; the rest return
  `-EINVAL`/`-EFAULT` the way the vendor's unimplemented ones do. **This is the gap to watch in the
  first trial**: begonia's daemon called 9, 10 and 13 as well and the vendor driver answers them
  with values, so a negative return there may or may not be tolerated - it will be visible in
  `mnld`'s log and is the first thing to look at if the DSP upload starts and then stops.
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

## Reproducing

    ./assemble.py --super ~/poco-m5s/backup-2026-11-14/super.img --out /path/to/tree

Needs `debugfs` (e2fsprogs), `dd`, `python3`, `readelf` and nothing else; the partitions are read
from the image, never from the phone. `filelist.txt` is written next to this file and is what a
review reads.
