# The analog half and the top window, from the stock HAL (note 203)

## 1. Does the HAL clear `0x1a040024` for port 2?

**It never writes it.** `hal_csirx_phyA_init` (0x32208) writes through `x10`, which is the *analog*
base (the 3-page mmap at `[this+0x40]`), at offsets `+0x0`, `+0x4`, `+0x18`, `+0x1c`, `+0x20`,
`+0x24`, `+0x40`, `+0x80` — those are the MIPI RX analog registers `ANA00/ANA04/ANA18/ANA1C/ANA20/ANA24/ANA40/WRAPPER80`,
not the seninf top block. The port selection is done elsewhere:

- per-instance mux: `setSeninfMuxCtrl` writes `instance + 0x0/0x4/0x8/0xc`;
- top mux: `setSeninfTopMuxCtrl` (0x30b90) uses base `[this+0x38]` (the seninf block) at `+0x10`
  and `+0x14` (0x30bc4-0x30c34).

So `0x80001100 & 0x7fffc8fe` written at `0x1a040024` is MT8183's value for a register this SoC
reads back as **zero**: not "cleared by the HAL", but never used by it. The write is dropped.

## 2. Which bits the stock sequence sets in the analog block

From `hal_csirx_phyA_init` (read-modify-write, one line per field):

| register | offset | mask | value | meaning (per the CrOS field names) |
|---|---|---|---|---|
| ANA00 | `+0x00` | `0xfffffff8` | `0x4` | lane 0 data/clock mode |
| ANA00 | `+0x00` | `0xffffff8f` | `0x40` | lane 1 mode |
| ANA00 | `+0x00` | `0xfffff8ff` | `0x400` | lane 2 mode |
| ANA00 | `+0x00` | `0xffff8fff` | `0x4000` | lane 3 mode |
| ANA00 | `+0x00` | `0xfff0ffff` | `0x80000` | lane 4 mode |
| ANA18 | `+0x18` | `0xffffffcf` | `0x10` | L0 EQ/termination |
| ANA18 | `+0x18` | `0xffcfffff` | `0x100000` | L0 second field |
| ANA1C | `+0x1c` | `0xffffffcf` | `0x10` | L1 |
| ANA20 | `+0x20` | (same pattern) | | L2 |
| ANA24 | `+0x24` | | | reserve / timing |
| ANA40 | `+0x40` | | `0x90` | (in the vendor: `mtk_phy_set_bits(ANA40, 0x90)`) |
| WRAPPER80 | `+0x80` | | reset mode | |

**What we read back after power-on on port 2: `A 0x00801a4e`, `B 0x0080124e`.** Decoding the A half:
bits 1, 2, 3, 6, 9, 11, 12 and 23 are set. The vendor's per-lane mode fields expect `0x4` (bit 2) for
lane 0, `0x40` (bit 6) for lane 1, `0x400` (bit **10**) for lane 2, `0x4000` (bit **14**) for lane 3,
`0x80000` (bit **19**) for lane 4. Bits 10, 14 and 19 are **absent** in our read-back, and bit 23 is
set although the vendor sequence above does not set it. So the lanes above lane 1 are very likely
**not in D-PHY mode** - which is exactly the shape of "receiver enabled, LNRD_FSM 0". This is the
next thing to fix, and it needs the rest of `phyA_init`/`phyA_setting` decoded field by field
(offsets 0x24, 0x40, 0x80 and the B half), not a guess.

## 3. Is the sensor itself streaming?

Not answerable from the kernel side here: the check is `0x0100` (mode_select) and the CSI-2 clock
lane state on the sensor, read over I2C while the stream runs:

```
i2cget -y -f 4 0x1a 0x0100 w   # mode_select: 1 = streaming
i2cget -y -f 4 0x1a 0x0110 w   # PLL / lane config
i2cget -y -f 4 0x1a 0x3406 w   # vendor "CSI-2 clock lane" state on this sensor family
```

If `0x0100` is 0 the sensor is not streaming and the whole receiver side is a red herring.
