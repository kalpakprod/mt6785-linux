// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MIPI CSI v0.5 driver
 *
 * Copyright (c) 2023, MediaTek Inc.
 * Copyright (c) 2023, BayLibre Inc.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-csi-0-5-rx-reg.h"

#define CSIXB_OFFSET		0x1000

/* Debug overrides, both off by default; see the comment where they are applied. */
static int cphy = -1;
module_param(cphy, int, 0644);
MODULE_PARM_DESC(cphy, "force the C-PHY select bit: -1 leave alone, 0 = D-PHY, 1 = C-PHY");

static bool ckmode_always;
module_param(ckmode_always, bool, 0644);
MODULE_PARM_DESC(ckmode_always,
		 "set the per-lane clock-mode enables and clear their selects at power on");

struct mtk_mipi_cdphy_port {
	struct device *dev;
	void __iomem *base;
	struct phy *phy;
	u32 type;
	u32 mode;
	u32 num_lanes;
	/* One late read of the block, half a second after it is powered on. */
	struct delayed_work status_work;
};

enum PHY_TYPE {
	DPHY = 0,
	CPHY,
	CDPHY,
};

static void mtk_phy_csi_cdphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);
}

static void mtk_phy_csi_dphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);
}

/*
 * DIAG (not for upstream): the block's first eight registers of both halves, before and after the
 * programming, so a log shows whether the writes landed in the halves the driver believes it has.
 * Off by default; enable with `file phy-mtk-mipi-csi-0-5.c +p` in dynamic debug.
 */
/*
 * The analog fields the stock HAL sets, extracted from its own disassembly
 * (libcam.halsensor.so: hal_csirx_phyA_init 0x32208-0x32520; the other four sequence functions add
 * no field these do not carry). One line per field, mask and value as the HAL writes them.
 *
 * Mainline's header names some of these bits: offset 0x4 == ANA04, where bit 2 is RG_CSIXA_BG_LPF_EN,
 * bit 6 RG_CSIXA_DPHY_L0_CKSEL, bit 9 RG_CSIXA_DPHY_L1_CKSEL, bits 11/12 RG_CSIXA_DPHY_L2_CKMODE_EN
 * and _CKSEL (phy-mtk-mipi-csi-0-5-rx-reg.h:17-27). Bits 10, 14 and 19 of the same register, which the
 * stock HAL sets and this driver did not, are not named in that header - and they are exactly the ones
 * missing in what this phone reads back after power-on (A 0x00801a4e). Offsets 0x8, 0xc, 0x18-0x20,
 * 0x30, 0x34 and 0x3c are the per-lane timing/equaliser fields, likewise unnamed in mainline.
 */
static const struct {
	u32 off;
	u32 mask;
	u32 val;
} mtk_mipi_vendor_ana[] = {
	{ 0x4, 0xfffffff8, 0x4 },	/* phyA_init */
	{ 0x4, 0xffffff8f, 0x40 },	/* phyA_init */
	{ 0x4, 0xfffff8ff, 0x400 },	/* phyA_init */
	{ 0x4, 0xffff8fff, 0x4000 },	/* phyA_init */
	{ 0x4, 0xfff0ffff, 0x80000 },	/* phyA_init */
	{ 0x18, 0xffffffcf, 0x10 },	/* phyA_init */
	{ 0x18, 0xffcfffff, 0x100000 },	/* phyA_init */
	{ 0x1c, 0xffffffcf, 0x10 },	/* phyA_init */
	{ 0x1c, 0xffcfffff, 0x100000 },	/* phyA_init */
	{ 0x20, 0xffffffcf, 0x10 },	/* phyA_init */
	{ 0x20, 0xffcfffff, 0x100000 },	/* phyA_init */
	{ 0x18, 0xffffff3f, 0x40 },	/* phyA_init */
	{ 0x18, 0xff3fffff, 0x400000 },	/* phyA_init */
	{ 0x1c, 0xffffff3f, 0x40 },	/* phyA_init */
	{ 0x1c, 0xff3fffff, 0x400000 },	/* phyA_init */
	{ 0x20, 0xffffff3f, 0x40 },	/* phyA_init */
	{ 0x20, 0xff3fffff, 0x400000 },	/* phyA_init */
	{ 0x20, 0xfffff0ff, 0x40000 },	/* phyA_init */
	{ 0x18, 0xfffff0ff, 0x300 },	/* phyA_init */
	{ 0x18, 0xf0ffffff, 0x3000000 },	/* phyA_init */
	{ 0x1c, 0xfffff0ff, 0x300 },	/* phyA_init */
	{ 0x1c, 0xf0ffffff, 0x3000000 },	/* phyA_init */
	{ 0x20, 0xfffff0ff, 0x300 },	/* phyA_init */
	{ 0x20, 0xf0ffffff, 0x3000000 },	/* phyA_init */
	{ 0x8, 0xffffffe0, 0x10 },	/* phyA_init */
	{ 0x8, 0xffffe0ff, 0x1000 },	/* phyA_init */
	{ 0x8, 0xffe0ffff, 0x100000 },	/* phyA_init */
	{ 0x8, 0xe0ffffff, 0x10000000 },	/* phyA_init */
	{ 0xc, 0xffffffe0, 0x10 },	/* phyA_init */
	{ 0xc, 0xffffe0ff, 0x1000 },	/* phyA_init */
	{ 0x30, 0xffffffe0, 0x8 },	/* phyA_init */
	{ 0x30, 0xffffe0ff, 0x800 },	/* phyA_init */
	{ 0x30, 0xffe0ffff, 0x80000 },	/* phyA_init */
	{ 0x3c, 0xffffffe0, 0x8 },	/* phyA_init */
	{ 0x3c, 0xffffe0ff, 0x800 },	/* phyA_init */
	{ 0x3c, 0xffe0ffff, 0x80000 },	/* phyA_init */
	{ 0x34, 0xe0ffffff, 0x1 },	/* phyA_init */
	{ 0x30, 0xe0ffffff, 0x1000000 },	/* phyA_init */
	{ 0x3c, 0xe0ffffff, 0x1000000 },	/* phyA_init */
};

static void mtk_mipi_phy_vendor_ana(struct device *dev, void __iomem *base,
				    const char *half)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_mipi_vendor_ana); i++) {
		u32 off = mtk_mipi_vendor_ana[i].off;
		u32 before = readl(base + off);
		u32 want = (before & mtk_mipi_vendor_ana[i].mask) | mtk_mipi_vendor_ana[i].val;

		writel(want, base + off);

		dev_info(dev, "ana %s +0x%02x: 0x%08x -> 0x%08x (want 0x%08x)%s\n",
			 half, off, before, readl(base + off), want,
			 readl(base + off) == want ? "" : "  MISMATCH");
	}
}

static void mtk_mipi_phy_dump(struct device *dev, void __iomem *base, const char *when)
{
	dev_info(dev, "DIAG ana %s: A 0x00 %08x %08x %08x %08x | B 0x00 %08x %08x %08x %08x\n",
		when,
		readl(base + 0x00), readl(base + 0x04), readl(base + 0x08), readl(base + 0x0c),
		readl(base + CSIXB_OFFSET + 0x00), readl(base + CSIXB_OFFSET + 0x04),
		readl(base + CSIXB_OFFSET + 0x08), readl(base + CSIXB_OFFSET + 0x0c));
}

/*
 * The analog lane receivers and the ready flag, from the stock HAL's own power-on for one half
 * (SeninfDrvImp::hal_csirx_phyA_power_on, libcam.halsensor.so 0x32a40, read from
 * scratch/native/5-cameras/vendor-img/), in its order:
 *
 *   :0x32a68-0x32aac  clear bits 0 and 16 of 0x18, 0x1c and 0x20   (lane receivers off)
 *   :0x32ab0-0x32ac4  clear bits 2 and 3 of 0x00                   (bias and LPF off)
 *   :0x32ac8          usleep 200
 *   :0x32ad0-0x32adc  set bit 3 of 0x00, usleep 30
 *   :0x32ae4-0x32af0  set bit 2 of 0x00, usleep 1
 *   :0x32af8-0x32b40  set bits 0 and 16 of 0x18, 0x1c and 0x20     (lane receivers on)
 *   :0x32b48-0x32ba0  poll 0x48 for (value & 0x29) != 0, 101 tries 30 us apart
 *
 * The driver brings the bias up in that order already, but it never switches a lane receiver on
 * and never reads the ready flag, so a powered block can still sit with silent lanes - which is
 * what the phone measured: LNRD_FSM 0, packets 0, errors 0, no interrupt.
 */
static void mtk_mipi_phy_lanes(struct device *dev, void __iomem *base, bool on)
{
	unsigned int half, lane, i;
	u32 a = 0, b = 0;

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;

		for (lane = 0; lane < 3; lane++) {
			u32 val = readl(h + 0x18 + 4 * lane);

			val = on ? (val | 0x10001) : (val & ~0x10001);
			writel(val, h + 0x18 + 4 * lane);
		}
	}

	if (!on) {
		dev_info(dev, "lanes off: A +0x18 %08x, B +0x18 %08x\n",
			 readl(base + 0x18), readl(base + CSIXB_OFFSET + 0x18));
		return;
	}

	for (i = 0; i < 101; i++) {
		a = readl(base + 0x48);
		b = readl(base + CSIXB_OFFSET + 0x48);
		if ((a & 0x29) && (b & 0x29))
			break;
		usleep_range(30, 60);
	}

	dev_info(dev, "lanes on: A +0x18 %08x +0x48 %08x, B +0x18 %08x +0x48 %08x, %u tries\n",
		 readl(base + 0x18), a, readl(base + CSIXB_OFFSET + 0x18), b, i);
}

/*
 * The block's own state, half a second after it was powered on and - because this runs from
 * power_on - while the sensor is transmitting. The vendor reads the analog state through a debug
 * selector: hal_csirx_ana_get_dbg_value (libcam.halsensor.so 0x33a20) writes the selector into
 * bits 15:8 of 0x80 of each half and reads 0x84. Selector 0x18 is the first of the four per-lane
 * codes its dphy counterpart builds (hal_csirx_dphy_get_dbg_value, :0x337f0-:0x3380c packs
 * 0x03020100, i.e. one byte per lane, and steps the base by 0x18181818 per selector). Reads only.
 */
static void mtk_mipi_phy_status_work(struct work_struct *work)
{
	struct mtk_mipi_cdphy_port *port =
		container_of(to_delayed_work(work), struct mtk_mipi_cdphy_port, status_work);
	void __iomem *base = port->base;
	unsigned int sel, half;

	dev_info(port->dev, "status: num_lanes %u type %u\n", port->num_lanes, port->type);

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;

		dev_info(port->dev,
			 "status %s: 0x00 %08x 0x04 %08x 0x18 %08x 0x1c %08x 0x20 %08x 0x24 %08x 0x40 %08x 0x48 %08x 0x80 %08x 0xa8 %08x\n",
			 half ? "B" : "A", readl(h + 0x00), readl(h + 0x04),
			 readl(h + 0x18), readl(h + 0x1c), readl(h + 0x20),
			 readl(h + 0x24), readl(h + 0x40), readl(h + 0x48),
			 readl(h + 0x80), readl(h + 0xa8));
	}

	for (sel = 0x18; sel <= 0x1b; sel++) {
		u32 a, b;

		writel((readl(base + 0x80) & ~0xff00u) | (sel << 8), base + 0x80);
		writel((readl(base + CSIXB_OFFSET + 0x80) & ~0xff00u) | (sel << 8),
		       base + CSIXB_OFFSET + 0x80);
		a = readl(base + 0x84);
		b = readl(base + CSIXB_OFFSET + 0x84);
		dev_info(port->dev, "status dbg sel 0x%02x: A 0x%08x B 0x%08x\n", sel, a, b);
	}
}

/*
 * The per-lane clock assignment of the HAL's own D-PHY path, which nothing in this driver carries.
 * hal_csirx_dphy_setting (libcam.halsensor.so 0x32ed8) branches on the lane count and, for three
 * lanes or more, sets bits 8, 9, 10 and 11 of 0x00 (:0x32f74-0x32fa0) and the fields bits 10:8 = 1,
 * bits 2:0 = 2, bits 13:12 = 3 and bits 22:20 = 4 of 0x04 (:0x32f28-0x32f70). The values this
 * driver writes there come from the MT8365 field map instead, and the ChromeOS 4D1C word it applies
 * to bits 5..12 of 0x00 is that SoC's lane order, not this one's.
 *
 * Bit 0 of 0x00 is part of the same HAL step and is left out here on purpose: the in-tree header
 * names it RG_CSI0A_CPHY_EN (phy-mtk-mipi-csi-0-5-rx-reg.h:18), so writing it would switch the port
 * to C-PHY and no D-PHY frame could arrive. That one bit is the open question, not a guess to flash.
 */
static void mtk_mipi_phy_lane_clock(struct device *dev, void __iomem *base,
				    unsigned int lanes)
{
	unsigned int half;

	if (lanes < 3) {
		dev_info(dev, "lane clock: %u lanes, the HAL branch is not applied\n", lanes);
		return;
	}

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;
		u32 a0 = readl(h + 0x00);
		u32 a4 = readl(h + 0x04);

		a0 |= 0x100 | 0x200 | 0x400 | 0x800;
		a4 &= ~((0x7u << 16) | (0x7u << 8) | 0x7u | (0x3u << 12));
		a4 |= (0x1u << 8) | 0x2u | (0x3u << 12);
		a4 &= ~(0x7u << 20);
		a4 |= 0x4u << 20;

		writel(a0, h + 0x00);
		writel(a4, h + 0x04);

		dev_info(dev, "lane clock %s: 0x00 -> %08x, 0x04 -> %08x\n",
			 half ? "B" : "A", a0, a4);
	}
}

/*
 * The block of four registers the HAL writes next to the lane clock assignment, and the only part
 * of its D-PHY setup this driver still lacked: hal_csirx_dphy_init (libcam.halsensor.so 0x32078)
 * writes the same three fields into 0x20, 0x24, 0x28 and 0x2c for every port - bits 23:16 = 0x20
 * (:0x320b0-:0x3212c), bits 11:8 = 4 (:0x320f0-:0x3212c) and bit 29 (:0x32130-:0x3215c).
 * Four registers, three lanes worth of receive settings, written per half.
 */
static void mtk_mipi_phy_dphy_init_bank(struct device *dev, void __iomem *base)
{
	unsigned int half, off;

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;

		for (off = 0x20; off <= 0x2c; off += 4) {
			u32 v = readl(h + off);

			v = (v & ~0xff0000u) | (0x20u << 16);
			v = (v & ~0xf00u) | 0x400u;
			v |= BIT(29);
			writel(v, h + off);
		}

		dev_info(dev, "dphy init bank %s: 0x20 %08x 0x24 %08x 0x28 %08x 0x2c %08x\n",
			 half ? "B" : "A", readl(h + 0x20), readl(h + 0x24),
			 readl(h + 0x28), readl(h + 0x2c));
	}
}

/*
 * The enable bits decoded, so a trial reads like a list instead of a word: bit 0 is the C-PHY
 * select (RG_CSI0A_CPHY_EN, phy-mtk-mipi-csi-0-5-rx-reg.h:18), bit 2 the bias LPF, bit 3 the bias
 * core, bits 5/8/11 the per-lane clock mode enables and 6/9/12 their selects, and bits 0 and 16 of
 * 0x18, 0x1c and 0x20 the lane receivers themselves (the HAL's own lane-enable bits).
 */
static void mtk_mipi_phy_decode(struct device *dev, void __iomem *base, const char *when)
{
	unsigned int half;

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;
		u32 a0 = readl(h + 0x00);

		dev_info(dev,
			 "%s %s: cphy %u bg_lpf %u bg_core %u ckmode %u/%u/%u cksel %u/%u/%u\n",
			 when, half ? "B" : "A", a0 & 1, !!(a0 & BIT(2)), !!(a0 & BIT(3)),
			 !!(a0 & BIT(5)), !!(a0 & BIT(8)), !!(a0 & BIT(11)),
			 !!(a0 & BIT(6)), !!(a0 & BIT(9)), !!(a0 & BIT(12)));

		dev_info(dev, "%s %s: lane_rx 0x18 %u 0x1c %u 0x20 %u ready %08x\n",
			 when, half ? "B" : "A", !!(readl(h + 0x18) & 1),
			 !!(readl(h + 0x1c) & 1), !!(readl(h + 0x20) & 1),
			 readl(h + 0x48));
	}
}

static int mtk_mipi_phy_power_on(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	mtk_mipi_phy_dump(&phy->dev, base, "before");
	mtk_mipi_phy_lanes(&phy->dev, base, false);

	/*
	 * The driver currently supports DPHY and CD-PHY phys,
	 * but the only mode supported is DPHY,
	 * so CD-PHY capable phys must be configured in DPHY mode
	 */
	if (port->type == CDPHY) {
		mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN, 0);
		mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
				     RG_CSI0A_CPHY_EN, 0);
	}

	/*
	 * Lane configuration:
	 *
	 * Only 4 data + 1 clock is supported for now with the following mapping:
	 *
	 * CSIXA_LNR0 --> D2
	 * CSIXA_LNR1 --> D0
	 * CSIXA_LNR2 --> C
	 * CSIXB_LNR0 --> D1
	 * CSIXB_LNR1 --> D3
	 */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKMODE_EN, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L2_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	/* Byte clock invert */
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	/* Start ANA EQ tuning */
	if (port->type == CDPHY)
		mtk_phy_csi_cdphy_ana_eq_tune(base);
	else
		mtk_phy_csi_dphy_ana_eq_tune(base);

	/* End ANA EQ tuning */
	mtk_phy_set_bits(base + MIPI_RX_ANA40_CSIXA, 0x90);

	mtk_phy_update_field(base + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	/* ANA power on */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	usleep_range(20, 40);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);

	/*
	 * The fields the stock HAL sets, in its order, on both halves. This is what was missing for
	 * lanes 2, 3 and 4: mainline's own writes leave them out, and the phone's read-back showed
	 * exactly those bits clear while the receiver sat enabled with silent lanes.
	 */
	mtk_mipi_phy_vendor_ana(&phy->dev, base, "A");
	mtk_mipi_phy_vendor_ana(&phy->dev, base + CSIXB_OFFSET, "B");

	/* And the lane-clock assignment the HAL derives from the lane count. */
	mtk_mipi_phy_lane_clock(&phy->dev, base, port->num_lanes);
	mtk_mipi_phy_dphy_init_bank(&phy->dev, base);

	/*
	 * Two debug overrides for a trial, both off by default: dphy.cphy forces the C-PHY select bit
	 * either way (0 = D-PHY, which is what the stock register dump shows), and dphy.ckmode_always
	 * sets the per-lane clock-mode enables and clears their selects, i.e. "keep the lane clocks on"
	 * (the bits the in-tree header names CKMODE_EN and CKSEL).
	 */
	if (cphy >= 0) {
		unsigned int i;

		for (i = 0; i < 2; i++) {
			void __iomem *h = base + i * CSIXB_OFFSET;
			u32 a0 = readl(h + 0x00);

			a0 = cphy ? (a0 | BIT(0)) : (a0 & ~BIT(0));
			writel(a0, h + 0x00);
		}
		dev_info(&phy->dev, "cphy forced to %d\n", cphy);
	}
	if (ckmode_always) {
		unsigned int i;

		for (i = 0; i < 2; i++) {
			void __iomem *h = base + i * CSIXB_OFFSET;
			u32 a0 = readl(h + 0x00);

			a0 |= BIT(5) | BIT(8) | BIT(11);
			a0 &= ~(BIT(6) | BIT(9) | BIT(12));
			writel(a0, h + 0x00);
		}
		dev_info(&phy->dev, "clock mode enabled on every lane\n");
	}

	mtk_mipi_phy_decode(&phy->dev, base, "enables");

	/* and the receivers of the lanes themselves, then the HAL's ready flag */
	mtk_mipi_phy_lanes(&phy->dev, base, true);

		mtk_mipi_phy_dump(&phy->dev, base, "after");

	/* And once more when the sensor has had time to transmit. */
	schedule_delayed_work(&port->status_work, msecs_to_jiffies(500));

	return 0;
}

static int mtk_mipi_phy_power_off(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	cancel_delayed_work_sync(&port->status_work);

	/* Lane receivers first, then the bias, the order the HAL uses. */
	mtk_mipi_phy_lanes(&phy->dev, base, false);

	/* Disable MIPI BG. */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	return 0;
}

static struct phy *mtk_mipi_cdphy_xlate(struct device *dev,
					const struct of_phandle_args *args)
{
	struct mtk_mipi_cdphy_port *priv = dev_get_drvdata(dev);

	/*
	 * If PHY is CD-PHY then we need to get the operating mode
	 * For now only D-PHY mode is supported
	 */
	if (priv->type == CDPHY) {
		if (args->args_count != 1) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		switch (args->args[0]) {
		case PHY_TYPE_DPHY:
			priv->mode = DPHY;
			if (priv->num_lanes != 4) {
				dev_err(dev, "Only 4D1C mode is supported for now!\n");
				return ERR_PTR(-EINVAL);
			}
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", args->args[0]);
			return ERR_PTR(-EINVAL);
		}
	} else {
		if (args->args_count) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		priv->mode = DPHY;
	}

	return priv->phy;
}

static const struct phy_ops mtk_cdphy_ops = {
	.power_on	= mtk_mipi_phy_power_on,
	.power_off	= mtk_mipi_phy_power_off,
	.owner		= THIS_MODULE,
};

static int mtk_mipi_cdphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct mtk_mipi_cdphy_port *port;
	struct phy *phy;
	int ret;
	u32 phy_type;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	dev_set_drvdata(dev, port);

	port->dev = dev;

	port->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	ret = of_property_read_u32(dev->of_node, "num-lanes", &port->num_lanes);
	if (ret) {
		dev_err(dev, "Failed to read num-lanes property: %i\n", ret);
		return ret;
	}

	/*
	 * phy-type is optional, if not present, PHY is considered to be CD-PHY
	 */
	if (device_property_present(dev, "phy-type")) {
		ret = of_property_read_u32(dev->of_node, "phy-type", &phy_type);
		if (ret) {
			dev_err(dev, "Failed to read phy-type property: %i\n", ret);
			return ret;
		}
		switch (phy_type) {
		case PHY_TYPE_DPHY:
			port->type = DPHY;
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", phy_type);
			return -EINVAL;
		}
	} else {
		port->type = CDPHY;
	}

	phy = devm_phy_create(dev, NULL, &mtk_cdphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create PHY: %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}

	port->phy = phy;
	phy_set_drvdata(phy, port);
	INIT_DELAYED_WORK(&port->status_work, mtk_mipi_phy_status_work);

	phy_provider = devm_of_phy_provider_register(dev, mtk_mipi_cdphy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register PHY provider: %ld\n",
			PTR_ERR(phy_provider));
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static const struct of_device_id mtk_mipi_cdphy_of_match[] = {
	{ .compatible = "mediatek,mt8365-csi-rx" },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, mtk_mipi_cdphy_of_match);

static struct platform_driver mipi_cdphy_pdrv = {
	.probe = mtk_mipi_cdphy_probe,
	.driver	= {
		.name	= "mtk-mipi-csi-0-5",
		.of_match_table = mtk_mipi_cdphy_of_match,
	},
};
module_platform_driver(mipi_cdphy_pdrv);

MODULE_DESCRIPTION("MediaTek MIPI CSI CD-PHY v0.5 Driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL");
