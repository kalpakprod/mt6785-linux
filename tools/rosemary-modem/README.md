# rosemary-modem on the Poco M5s: what this directory holds and what it needs

The kernel module `rosemary_ccci` (steps 1-4 of the modem card) is only half of
the modem. The modem's NVRAM file service, its RPC service and its boot are
driven from userspace, and the MTK-specific parts of ModemManager have to know
that this modem is not a USB one. Everything a phone needs for that lives here.

Ported from the cezanne port (github.com/DrShadow34/cezanne-linux-port):
`gnome-apps/cezanne-modem/` and
`software-patches/modemmanager/0003-plugins-new-mtk-ccmni-plugin-for-MediaTek-SoC-modems.patch`.
Its protocol code is not rewritten, because the FS and RPC protocols belong to
the modem, not to the SoC; what changed is every board-specific value, and each
changed line in the sources below says which row of the delta table
(`poco-m5s/research/native/4-modem/3-userspace.md` §4) it comes from.

> **Owner rule, 17.11: the SIM is Russian and on Magticom it roams, so every call, message and
> data byte costs money.** The driver and every test bring the modem up with the **radio off
> (`AT+CFUN=0`)**, and nothing we run may send `AT+CFUN=1`, any `AT+COPS` form other than the
> read-only `AT+COPS?`, `AT+CGACT`, `ATD`, `AT+CMGS`/`+CMGW`, or anything else that registers,
> attaches or opens a data context. ModemManager raises the radio by itself when it initialises a
> modem, so it must stay away from the port (`ID_MM_DEVICE_IGNORE`) until the owner says otherwise.
> Checks are the read-only ones: `AT`, `ATI`, `AT+CGMR`, `AT+CPIN?`, `AT+CSQ`, `AT+CFUN?`,
> `AT+CREG?`, `AT+CEREG?`, `AT+COPS?`, `AT+CGDCONT?`.

## Files

| File | What it is |
|---|---|
| `rosemary-modemd.c` | the daemon: NVRAM file service (CCCI 14/15), RPC service (32/33), MD boot, AT init, then handover to ModemManager. `-v` logs each file request with its path, flags and result |
| `rosemary-modem-nvram-import` | copies `nvdata nvcfg protect1 protect2` to `dd` copies, `e2fsck -fy` on the copy, mounts it read-only and copies the tree out — the live partitions are never mounted |
| `modem-check.sh` | a read-only phone check: driver state, the WWAN ports, the network interfaces and their `md_interface`, then `AT`, `ATI`, `AT+CGMR`, `AT+CPIN?`, `AT+CSQ`, `AT+CFUN?` (see the radio rule below), `AT+CREG?`, `AT+CEREG?`, `AT+COPS?` and `AT+CGDCONT?` as queries only. Every AT command has a timeout, every line is timestamped, and IMEI/IMSI/ICCID are masked before anything is printed. `tests/` holds a fake AT modem on a pty and the test that runs the check against it |
| `rosemary-modem-datalink` | creates the modem's packet-data network interface (`ip link add … type wwan`, the WWAN framework's own way). One interface serves every context: the plugin moves it to the ccmni id each context gets. Run by the unit's `ExecStartPost` |
| `rosemary-modem-properties` | the properties the modem asks for, in the stock phone's own values: `key=value`, one per line, generated from the stock phone's property dump. Keys that identify one phone (chip id, CPU id, model number) are left out: the modem did not ask for any of them in the checked boots, and the service names every key it cannot answer in its log. Installed as `/usr/share/rosemary-modem/properties` |
| `rosemary-modem.service` | systemd unit for the daemon |
| `78-rosemary-modem.rules` | udev rules: the AT port is primary AT for ModemManager, and nothing is handed over before the SIM is up |
| `modemmanager-mtk-ccmni/` | the ModemManager plugin and its packaging: `0003-plugins-new-mtk-ccmni-plugin.patch` (plugin `mtk-ccmni`: AT on `wwan<instance>at0`, data on the DPMAIF netdevs, `+EIF` → `/sys/class/net/<link>/md_interface`, address from `+CGPADDR`, SIM hot-swap from `+ESIMS`), the `+EIF` parser as its own testable file with a unit test, and a `PKGBUILD` for Arch ARM that builds ModemManager 1.24.2 with the plugin |
| `Makefile` | `cc` only, no cross compiler: the lead builds it on the phone |

## What the lead has to do on the phone

1. Copy the firmware configuration once, from the stock vendor image, verbatim:
   `/vendor/etc/mdota` (139 `MTK_OPOTA_SBPID_<n>.mcfopota` + `MTK_OPOTA_GENERAL.mcfopota`
   + `MTK_MD_OTA_CONFIG.ini`, 84 bytes each) and `/vendor/etc/firmware/md`,
   `/vendor/etc/firmware/modem` into `/usr/lib/firmware/mediatek/rosemary/`
   (`mdota/`, `md/`, `modem/`). These are the drives `T:`, `R:` and `W:` of the
   file service; without them the modem's file service cannot answer and it
   never reaches the second handshake.
2. `rosemary-modem-nvram-import` once, with the stock partitions present.
3. Build and install: `make && sudo make install`.
4. Build the variant ModemManager with the plugin: `cd modemmanager-mtk-ccmni &&
   makepkg -si` (it provides and conflicts `modemmanager`, so it replaces the distro
   package). Then `sudo udevadm control --reload` and `systemctl restart ModemManager`.
5. `systemctl enable --now rosemary-modem`, then the phone checks in the card.

## Two things this directory deliberately does not do

- **Call audio** (`rosemary-speechd`). The speech-control protocol is the
  modem's, but the ids are MT6297-era and were reverse-engineered from MIUI's
  audio HAL, and they belong to the HAL rather than to the modem. On this phone
  the HAL that would confirm them is not in the vendor partition at all (it is
  AOSP's stub there), and the AFE side needs the audio area's `mt6785` ASoC
  driver, which does not exist yet. Writing the ids from another phone's HAL
  would be a guess; the card puts call audio outside this area's steps and so
  does this directory. See `5-port-plan.md` §2b.
- **A second modem, MD3, CCIF4/CCIF5, UDC, C2K, MTEE.** None of them exist on
  this phone: only the `md1img` partition is loaded, `ro.vendor.mtk_md3_support=0`,
  `mtee_support` is 0, and the vendor's own port table for this generation has
  no C2K or UDC nodes.
