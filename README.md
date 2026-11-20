# mt6785-linux

Mainline Linux **7.3.4** for phones on the **MediaTek MT6785** (Helio G90T / Helio G95).

The chip support (clocks, pinctrl, power domains, display pipeline, audio, Wi-Fi/Bluetooth/GPS,
modem, camera blocks) is written for the SoC and kept apart from the board files, so any MT6785
phone can take it. The board we test on is the **Xiaomi POCO M5s / Redmi Note 10S (`rosemary`)**.
The Redmi Note 8 Pro (`begonia`) device trees from the base are carried along.

Phones on this chip (44 models by one spec database; send a correction if yours is missing):

- Xiaomi: Redmi Note 8 Pro (begonia), Redmi Note 10S and POCO M5s (rosemary), Redmi Note 11 SE (India)
- Realme: 6, 6i, 6S, 7, 8, Narzo, Narzo 20 Pro, Narzo 30
- Infinix: Zero 8, Zero 8i, Note 10 Pro (and NFC), Zero X, Zero X Neo, Zero X Pro
- Tecno: Camon 16 Premier, Camon 17 Pro, Phantom X
- Motorola Moto G60S, vivo Y73, Micromax In note 2, Gionee M15, HiSense Infinity H50
- BLU G90 Pro, G91 Pro, G91 Max; Doogee S96 Pro, S96 GT, S97 Pro; Ulefone Power Armor 13;
  Umidigi S5 Pro, Bison GT, Bison GT2, A15T; iiiF150 R2022

If you own one, the SoC part should already apply; a board file (device tree) for your phone is what is
left. Issues and pull requests with board files are welcome.

## Status on rosemary (POCO M5s)

"Works" means checked on the phone, with the evidence named. Anything not checked on the phone is
not "Works".

| Feature | State | How it was checked |
|---|---|---|
| Boot, flashing (`boot_a` via fastboot) | Works | kernel in `boot_a`, boots unattended |
| USB networking | Works | SSH over USB gadget Ethernet |
| Internal storage (UFS) | Works | root filesystem on UFS |
| Battery / charging | Partial | MT6360 charger controlled (inhibit, input limit), held at 45-50 %; the percent the gauge reports is not trusted yet |
| Volume keys, power key | Works | evtest: GPIO volume-down, PMIC home key = volume-up |
| Haptics (AW8622) | Works | felt on the phone; strength scaling still open |
| Camera flash (MT6360 FLED) | Partial | channel 1 lights; channel 2 not seen yet |
| Screen (S6E8FC0 AMOLED, DSI) | Works | 35 of 35 bootloader boots with picture, webcam photo of each; a rare dark-screen fault (2 of ~70 boots before v0.2.0) is not shown gone |
| Brightness | Works | levels checked on the phone |
| Touchscreen (FT3418) | Works | touches reported on the phone |
| Audio | In progress | sound card registers; the audio DSP firmware stalls, no playback |
| Cameras | Partial | 8 MP IMX355: raw RAW10 frames in memory, colour bars decode correctly; 64 MP OV64B40: chip id read, no frames; no image processing, no focus |
| Wi-Fi (gen4m) | Works | WPA3 client: 34 of 34 reboots connect and get an address, 32 of 34 reach the internet at once (two single misses, cause not known); Wi-Fi off and on works; one chip start failed in 18 |
| Wi-Fi hotspot | Works | a laptop joins (WPA2), gets an address and reaches the internet through the phone: 41 Mbit/s down with the USB uplink; sharing the phone's own Wi-Fi connection runs at only ~7 Mbit/s |
| Bluetooth | Partial | scan finds devices; connections not checked |
| GPS | Partial | the engine outputs NMEA; no position fix yet |
| 3D (Mali-G76, Panfrost) | Works | Panfrost renders on the phone |
| Modem (calls, SMS, data) | In progress | the modem starts and waits for its file service; calls, SMS and data not checked |
| Sensors (via SCP) | In progress | SCP bring-up |
| Fingerprint | In progress | not working yet |
| SD card, IR, NFC, FDE, USB OTG | Untested | |

## Branches

- `main` - the kernel that runs on the test phone: every subsystem merged, with this README and the build
  configuration; each release tags a state of `main`.
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
- The code is written with AI coding agents and published by the maintainer: every commit carries `Assisted-by: LLM`
  (the kernel's rule, `Documentation/process/coding-assistants.rst`), and each commit is read, built and tested on
  the phone before it lands here.

## Licence

GPL-2.0, as the Linux kernel. See `COPYING`.
