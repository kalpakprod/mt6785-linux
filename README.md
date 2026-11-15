# mt6785-linux

Mainline Linux **7.3.4** for phones on the **MediaTek MT6785** (Helio G90T / Helio G95).

The chip support (clocks, pinctrl, power domains, display pipeline, audio, Wi-Fi/Bluetooth/GPS,
modem, camera blocks) is written for the SoC and kept apart from the board files, so any MT6785
phone can take it. The board we test on is the **Xiaomi POCO M5s / Redmi Note 10S (`rosemary`)**.
The Redmi Note 8 Pro (`begonia`) device trees from the base are carried along.

Phones on this chip include the Xiaomi Redmi Note 8 Pro (begonia), Redmi Note 10S and POCO M5s
(rosemary), Realme 6, Realme 7 and Infinix Note 10 Pro. If you own one, the SoC part should already
apply; a board file for your phone is what is left.

## Status on rosemary (POCO M5s)

"Works" means checked on the phone, with the evidence named. Anything not checked on the phone is
not "Works".

| Feature | State | How it was checked |
|---|---|---|
| Boot, flashing (`boot_a` via fastboot) | Works | kernel in `boot_a`, boots unattended |
| USB networking | Works | SSH over USB gadget Ethernet |
| Internal storage (UFS) | Works | root filesystem on UFS |
| Battery / charging | Partial | MT6360 charger controlled (inhibit, input limit), held at 45-50 %; gauge reports voltage only |
| Volume keys, power key | Works | evtest: GPIO volume-down, PMIC home key = volume-up |
| Haptics (AW8622) | Works | felt on the phone; strength scaling still open |
| Camera flash (MT6360 FLED) | Partial | channel 1 lights; channel 2 not seen yet |
| Screen (k7 AMOLED, DSI) | In progress | pipeline binds, first DCS command times out |
| Touchscreen (FT3418) | In progress | input device registers; waits for the screen |
| Audio | In progress | sound card registers; no playback checked yet |
| Cameras | In progress | camsv video nodes and sensor drivers probe; no image yet |
| Wi-Fi (gen4m) | In progress | firmware download and adapter start reached |
| Bluetooth | In progress | WMT core starts; BT power-on fails |
| Modem (calls, SMS, data) | In progress | modem device probes; boot fails |
| Sensors (via SCP) | In progress | SCP bring-up |
| SD card, IR, GPS, NFC, 3D (Mali-G76/Panfrost), FDE, USB OTG | Untested | |

## Branches

- `main` - the area branches merged for the last test kernel (display and fingerprint are tested
  on their own branches until they pass); an area branch may be ahead of `main`.
- `kit` - Linux 7.3.4 plus the MT6785 SoC base.
- `area/*` - one branch per subsystem (display, audio, connectivity, modem, camera, sensors,
  fingerprint, power); each commit says why it is there.

## Building

```sh
cp configs/mt6785-rosemary.config .config
make ARCH=arm64 LLVM=1 olddefconfig
make ARCH=arm64 LLVM=1 -j"$(nproc)" Image dtbs modules
```

`configs/mt6785-rosemary.config` is the exact configuration of the last test kernel.

Device tree: `arch/arm64/boot/dts/mediatek/mt6785-xiaomi-rosemary.dtb`. Vendor firmware is not in
this repository; it comes from the phone's own vendor partition.

## Where it comes from

- SoC base: Dinolek's [mt6785-mainline](https://github.com/Dinolek/mt6785-mainline) (7.1),
  forward-ported to 7.3.4.
- Hardware facts: the phone's own device tree and the Xiaomi / LineageOS MT6785 kernel sources;
  each commit names its source.
- Most driver commits are written by AI coding agents (authors `poco-*-worker`) under review: every
  commit is read, built and tested on the phone before it lands here.

## Licence

GPL-2.0, as the Linux kernel. See `COPYING`.
