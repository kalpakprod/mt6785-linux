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

## Files

| File | What it is |
|---|---|
| `rosemary-modemd.c` | the daemon: NVRAM file service (CCCI 14/15), RPC service (32/33), MD boot, AT init, then handover to ModemManager. `-v` logs each file request with its path, flags and result |
| `rosemary-modem-nvram-import` | copies `nvdata nvcfg protect1 protect2` to `dd` copies, `e2fsck -fy` on the copy, mounts it read-only and copies the tree out — the live partitions are never mounted |
| `rosemary-modem-datalink` | creates the modem's packet-data network interface (`ip link add … type wwan`, the WWAN framework's own way). One interface serves every context: the plugin moves it to the ccmni id each context gets. Run by the unit's `ExecStartPost` |
| `rosemary-modem-properties` | the properties the modem asks for, in the stock phone's own values: `key=value`, one per line, generated from `device-dump-2026-11-13/stock/getprop-filtered.txt`. Installed as `/usr/share/rosemary-modem/properties` |
| `rosemary-modem.service` | systemd unit for the daemon |
| `78-rosemary-modem.rules` | udev rules: the AT port is primary AT for ModemManager, and nothing is handed over before the SIM is up |
| `0003-plugins-new-mtk-ccmni-plugin.patch` | ModemManager plugin `mtk-ccmni`: AT on `wwan0at0`, data on the DPMAIF netdevs, `+EIF` → `/sys/class/net/<link>/md_interface`, address from `+CGPADDR`, SIM hot-swap from `+ESIMS`. The only change from cezanne's is the udev driver string |
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
4. Apply `0003-plugins-new-mtk-ccmni-plugin.patch` to the ModemManager source and
   install the patched build; `sudo udevadm control --reload`.
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
