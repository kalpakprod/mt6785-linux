# Stock HAL receiver sequence: raw disassembly (note 190)

Taken from `/path/to/project/scratch/native/5-cameras/vendor-img/libcam.halsensor.so`
(Android 11 stock `vendor_b`), `llvm-objdump -d --no-show-raw-insn`, one file per function:

| file | symbol | range |
|---|---|---|
| `phyA_init.txt` | `SeninfDrvImp::hal_csirx_phyA_init` | 0x32208-0x32520 |
| `phyA_power_on.txt` | `hal_csirx_phyA_power_on` | 0x32a40-0x32c08 |
| `phyA_setting.txt` | `hal_csirx_phyA_setting` | 0x32c08-0x32ed8 |
| `dphy_init.txt` | `hal_csirx_dphy_init` | 0x32078-0x32198 |
| `dphy_setting.txt` | `hal_csirx_dphy_setting` | 0x32ed8-0x33108 |
| `seninf_setting.txt` | `hal_csirx_seninf_setting` | 0x32870-0x328a0 |
| `seninf_top_setting.txt` | `hal_csirx_seninf_top_setting` | 0x328a0-0x32a40 |
| `seninf_csi2_setting.txt` | `hal_csirx_seninf_csi2_setting` | 0x326f8-0x32870 |
| `setSeninfCsiMipi.txt` | `setSeninfCsiMipi` | 0x325e8-0x326f8 |
| `setSeninfTopMuxCtrl.txt` | `setSeninfTopMuxCtrl` | 0x30b90-0x30c60 |
| `setSeninfMuxCtrl.txt` | `setSeninfMuxCtrl` | 0x31638-0x316e0 |
| `setSeninfCamMuxSrc.txt` | `setSeninfCamMuxSrc` | 0x30f40-0x31000 |
| `enableCamMUX.txt` | `enableCamMUX` | 0x2fd00-0x2fd78 |

## What the disassembly says about the bases

The binary holds **no physical address**: a scan of the whole file for the little-endian words
`0x001a040000`, `0x001a048000`, `0x0011c80000`, `0x0011c82000` returns zero hits, and `llvm-nm -D` shows
`mmap`/`open` on the kernel device, not on `/dev/mem`. So the register bases are handed to the HAL by the
kernel driver (one mmap page per receiver instance), not baked in - which means the port-2 base is
`0x1a040000 + 0x1000 * 4` by the kernel's own per-instance layout, and the analog half `0x11c84000`
(the vendor device tree's `mipi_rx_ana_csi2a@11c84000`).

The sequence functions write through one base pointer per instance held in the object (`this+0x150`,
`+0x160`, `+0x168`, `+0x170`, `+0x178` select the instance by the port enum), and do read-modify-write
with masks; the offsets visible in `seninf_csi2_setting` are `+0x0`, `+0x4`, `+0x8`, `+0x10`, `+0xc0`.

## Status

The register writes are extracted and stored here verbatim. The **field-level decode** of every offset
(which bit is which, and which of them correspond to the writes our driver already makes) and the
resulting replacement of our start sequence are **not** done; see REPORT.md, note 190.
