# ST21NFCD on rosemary: extend st-nci, and what stands in the way

Research only. No driver is written here until the lead decides; this file is the
evidence and the plan, with file:line for everything that is claimed.

## What the chip is

From the phone's own files, not from a parts page:

* the ST HAL runs on stock: `init.svc.vendor.st_nfc_hal_service: running`
  (`poco-m5s/device-dump-2026-11-12/07-sensors-radios.txt:93`);
* the NCI HAL module is `nfc_nci.st21nfc` and the firmware is
  `st21nfc_fw7.bin` (`/vendor/etc/libnfc-hal-st.conf` from the vendor image;
  `vendor-firmware.sha256` lists the file), loaded by
  `android.hardware.nfc-service.st`;
* the bus is `i2c3@1100f000` at address `0x08`, 400 kHz, open-drain; reset is
  GPIO32 (`nfc-gpio-v2`, `gpio-rst`), the interrupt is GPIO31 as EINT
  level-high (`device-tree.dts:4105-4110`, `:7604-7620`; vendor
  `rosemary/cust.dtsi:105-109`, `:305-309`).

## What the chip speaks, from its own HAL binary

`strings` on the vendor `nfc_nci.st21nfc.default.so` shows how the vendor stack
brings the chip up:

    -> Loader Mode NCI_CORE_RESET_NTF received after HW Reset
    -> Router Mode NCI_CORE_RESET_NTF received after HW Reset
    -> Loader V3 Mode NCI_CORE_RESET_NTF received after HW Reset
    ---> Firmware update needed from 0x%08X to 0x%08X
    UpdateHandlerST54L / mHalFDState = HAL_FD_ST54L_STATE_SEND_RAW_APDU
    %s - CORE_RESET_NTF : after core_reset_cmd

So the controller's ROM answers in **NCI**, announces by `CORE_RESET_NTF` whether
it came up in loader or router mode, and the HAL drives a **host-side firmware
update** in loader mode when the version check says so. In router mode it is an
ordinary NCI controller. That is the same family as what `drivers/nfc/st-nci`
drives - not a different transport and not a different framing.

## What st-nci already speaks

* NCI core, via `st_nci_ops` (`drivers/nfc/st-nci/core.c:95-111`) registered with
  `nci_allocate_device()`/`nci_register_device()` (`:133-149`).
* ST proprietary core op: `NCI_GID_PROPRIETARY` + `ST_NCI_CORE_PROP`
  (`core.c:88-93`).
* HCI-over-NCI, including the chip-initiated firmware update commands
  `ST_NCI_HCI_DM_{GETDATA,PUTDATA,LOAD,GETINFO,FWUPD_START,FWUPD_STOP}`
  (`vendor_cmds.c:16-21`) and the `fw_download_in_progress` state machine
  (`:161-185`).
* eSE: `discover_se`/`enable_se`/`disable_se`/`se_io` (`se.c`, wired at
  `core.c:104-108`).
* Phy (I2C): reset pulse low 10-15 ms then high 80-85 ms
  (`i2c.c:45-53`), IRQ enable/disable, NCI length-framed reads (`:104-140`).
  **No clock handling anywhere** (`grep clk_ drivers/nfc/st-nci/` finds nothing).

## The three real differences

1. **Reference clock.** This is not a crystal on the board: the 26 MHz comes out
   of the PMIC's clock buffer, XO_NFC, which the vendor kernel drives as a PMIC
   register vote - `PMIC_XO_NFC_VOTE_ADDR/MASK/SHIFT` in
   `poco-lunaris-kernel/drivers/misc/mediatek/include/mt-plat/mt6785/include/mach/upmu_hw.h:4915-4918`
   and `PMIC_XO_NFC_VOTE` in the register enum, with `mt6382_nfc` (EINT + srclk,
   `device-tree.dts:6671-6678`) as the vendor glue. Upstream 7.3.4 has **no
   provider for it**: the MT6359 node carries auxadc/codec/regulator/rtc only
   (`arch/arm64/boot/dts/mediatek/mt6359.dtsi:8-23`), and nothing in
   `drivers/clk/mediatek` or `drivers/mfd` mentions a clock buffer.
2. **Loader-mode firmware download** is host-driven in the vendor stack (the HAL
   strings above). st-nci has only the chip-initiated HCI DM path. For a chip
   that is already in router mode this is a userspace concern, not a kernel one.
3. **eSE/UICC flags** in the DT (`ese-present`, `uicc-present`,
   `st,st-nci.yaml`). The vendor config suggests an eSE
   (`OFF_HOST_ESE_PIPE_ID=0x5E` in `libnfc-hal-st.conf`), but nothing in the
   dumps proves it, so the flags are a measurement, not an assumption.

## Verdict: extend st-nci in C

The chip is NCI-compatible with the driver family, so a new (Rust) driver is not
justified by the evidence. The diff plan, smallest to largest:

1. `Documentation/devicetree/bindings/net/nfc/st,st-nci.yaml`: add
   `st,st21nfcd-i2c` to the compatible enum, add `clocks`/`clock-names`
   (`maxItems: 1`).
2. `drivers/nfc/st-nci/i2c.c`: add the compatible to `of_st_nci_i2c_match`
   (`:272-276`) and enable the reference clock before the first reset pulse,
   exactly as `s3fwrn5/i2c.c:177` does it:
   `phy->clk = devm_clk_get_optional_enabled(&client->dev, NULL);`
   (`st_nci_i2c_phy` gains the `struct clk *clk`, released with
   `devm_clk_get_optional_enabled`'s devm lifetime).
3. A clock provider for the MT6359 clock buffer: a small driver on the PMIC node
   (its regmap) exposing XO_NFC as a gate, with the vote address/mask for this
   PMIC. Without it the chip never gets its clock and nothing above matters -
   this is the actual work, and it is also useful for other consumers.
4. Board DT: the `nfc@8` node on i2c3 with `interrupts` GPIO31 level-high,
   `reset-gpios` GPIO32 ACTIVE_HIGH, `clocks = <&clkbuf XO_NFC>`, and the
   eSE/UICC flags once measured.

## What remains unknown, and how it would be settled

ST's own kernel driver and HAL sources were the sources this note wanted; the
network was unavailable in this environment (two fetches timed out), so the
statement above rests on the phone's HAL binary, the vendor kernel headers and
the upstream driver. What would settle the rest is one experiment on the phone,
after the clock provider exists: bind st-nci to the chip with the new compatible
and look for `CORE_RESET_NTF` in dmesg - the router-mode string means the driver
speaks its protocol, loader mode means a firmware download has to happen first.
Without the clock the chip stays silent, which is why the order is 3, then 1-2,
then 4.
