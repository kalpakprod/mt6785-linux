# Stock HAL receiver registers: per-instance vs top-level (note 193)

Source: `libcam.halsensor.so` (stock `vendor_b`), disassembly in this directory. All bases are
members of the `SeninfDrvImp` object; `init()` (0x2e580) builds them from one 19-page `mmap` of the
kernel's seninf device (`[this+0x38]`, ioctl `0xC0187328` returns offset+size) plus fixed byte
offsets taken from `.rodata` (0x11990/0x119a0/0x11a70/0x11a90/0x118d0/0x11900/0x11ae0/0x11af0 =
0x2000,0x3000,...,0xb000 and 0x200,0x600,0xa00 + 0x1000*n).

## Per-instance (base = seninf block + 0x1000 * instance)

| register | offset | access | function (binary address) |
|---|---|---|---|
| instance base | `+0x1000..+0xb000` | object members `this+0x150..0x180` | `init` 0x2e6d4-0x2e810 |
| CSI2 control | `+0x00` / `+0x04` / `+0x08` / `+0x10` / `+0xc0` | RMW | `hal_csirx_seninf_csi2_setting` 0x326f8 |
| mux control | `+0x00`/`+0x04`/`+0x08`/`+0x0c` | RMW | `setSeninfMuxCtrl` 0x31638 |

## Top-level (base = seninf block itself)

| register | absolute (MT6785) | access | function (address) |
|---|---|---|---|
| camera mux control | `0x1a040400` | bits 0..3 = mux index | `setSeninfCamMuxSrc` 0x30f84-0x30f90 |
| camera mux VC | `0x1a040504` | low 16 / high 16 | `setSeninfCamMuxSrc` 0x30f98-0x30fbc |
| camera mux enable | `0x1a040410` | `\|= 1 << n` | `enableCamMUX` 0x2fd28-0x2fd48 |
| camera mux select | `0x1a0404a0` | `\|= 3 << 2n` | `enableCamMUX` 0x2fd4c-0x2fd54 |
| camera mux select (write) | `0x1a0404a8` | `= 3 << 2n` | `enableCamMUX` 0x2fd58 |
| top mux control | via `setSeninfTopMuxCtrl` 0x30b90 | — | top-level |

The camera-mux base is `[this+0x180]`, and `init` sets it to the seninf mmap base **+ 0x400**
(0x2e7fc: `add x8, x11, #0x400`; 0x2e814: `str x8, [x20, #0x180]`). Instance bases are
`mmap base + 0x1000 * n` (0x2000..0xb000 in `.rodata`).

## Consequence for the driver

The camera-mux writes my driver made at `0x1a0444xx` (instance 5 + 0x400) belong at `0x1a0404xx`
(the block's own top window): that is exactly why comb1 read them back as zero while the same log's
top dump showed live `CAM_MUX 0x03020100` / `CAM_MUX1 0x07060504`. `dedcb515b`'s statement that the
binary holds no physical address stands - the bases come from the kernel mapping - and it does not
hide this piece: the offsets are in `init`, not in a constant.
