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
	/* The window's physical base, for the stock-format lines. */
	phys_addr_t base_pa;
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

	/*
	 * The whole debug selector range, not only the four codes the HAL's D-PHY helper builds: that
	 * helper (hal_csirx_dphy_get_dbg_value, libcam.halsensor.so 0x337f0-0x3380c) packs one byte per
	 * lane and steps by 0x18, starting at 0x18, and all four of its codes read zero here. Whatever
	 * the lane monitor is, it answers to one of these selectors; only the non-zero ones are printed,
	 * so a live monitor shows up as a nameable line instead of noise.
	 */
	{
		unsigned int found = 0;

		for (sel = 0; sel <= 0x3f; sel++) {
			u32 a, b;

			writel((readl(base + 0x80) & ~0xff00u) | (sel << 8), base + 0x80);
			writel((readl(base + CSIXB_OFFSET + 0x80) & ~0xff00u) | (sel << 8),
			       base + CSIXB_OFFSET + 0x80);
			a = readl(base + 0x84);
			b = readl(base + CSIXB_OFFSET + 0x84);
			if (!a && !b)
				continue;
			found++;
			dev_info(port->dev, "status dbg sel 0x%02x: A 0x%08x B 0x%08x\n",
				 sel, a, b);
		}

		dev_info(port->dev, "status dbg: %u of 64 selectors non-zero\n", found);
	}
}

/*
 * The D-PHY wrapper page of this port: +0x2000 inside its 0x4000 analogue region.
 *
 * Where it is: SeninfDrvImp::init (libcam.halsensor.so 0x2e6d4-0x2e7a0) adds the .rodata pairs
 * 0x119a0 {0xb000, 0x2000} and 0x11a90 {0x6000, 0xa000} to the analogue mapping (0x11C80000) and
 * stores them at [this+0x130..0x148]; hal_csirx_dphy_init and hal_csirx_dphy_setting pick
 * [this+0x138] (ana+0x2000) for CSI0/0A/0B, [this+0x140] (ana+0x6000) for CSI1/1A/1B and
 * [this+0x148] (ana+0xa000) for CSI2/2A/2B. Checked on the phone 19.11: with no camera driver
 * the three ports read the same four-page pattern at 0x11c80000, 0x11c84000 and 0x11c88000
 * (A, B, this page, the C-PHY page), and this page of port 2 stayed at its reset values while
 * comb41 streamed, i.e. nothing had programmed it. Until comb42 these writes went to the A and B
 * analogue pages (base + 0x0000/0x1000), whose 0x00/0x04/0x20..0x2c are other registers.
 */
#define MIPI_RX_DPHY_PAGE	0x2000
#define MIPI_RX_CPHY_PAGE_OFF	0x3000

/* hal_csirx_dphy_init (0x32078): 0x20..0x2c, bits 23:16 = 0x20, bits 11:8 = 4, bit 29. */
static void mtk_mipi_phy_dphy_init_bank(struct device *dev, void __iomem *base)
{
	void __iomem *w = base + MIPI_RX_DPHY_PAGE;
	unsigned int off;

	for (off = 0x20; off <= 0x2c; off += 4) {
		u32 v = readl(w + off);

		v = (v & ~0xff0000u) | (0x20u << 16);
		v = (v & ~0xf00u) | 0x400u;
		v |= BIT(29);
		writel(v, w + off);
	}
	dev_info(dev, "dphy page init: 0x20 %08x 0x24 %08x 0x28 %08x 0x2c %08x\n",
		 readl(w + 0x20), readl(w + 0x24), readl(w + 0x28), readl(w + 0x2c));
}

/*
 * setSeninfCsiMipi (0x3261c-0x3265c), between dphy_init and the CSI-2 setting, for every sensor:
 * bit 4 of +0x18 of the port's C-PHY page ([this+0x120/0x128/0x130] = ana+0x3000/0x7000/0xb000).
 */
static void mtk_mipi_phy_cphy_page_bit4(struct device *dev, void __iomem *base)
{
	void __iomem *c = base + MIPI_RX_CPHY_PAGE_OFF;
	u32 want = readl(c + 0x18) | BIT(4);

	writel(want, c + 0x18);
	dev_info(dev, "cphy page +0x18: wrote %08x reads %08x%s\n", want, readl(c + 0x18),
		 readl(c + 0x18) == want ? "" : "  MISMATCH");
}

/*
 * hal_csirx_dphy_setting (0x32ed8), the branch a full port (CSI0/1/2) always takes
 * (:0x32f18-0x32fb8 and :0x330c4-0x330e8):
 *   +0x04: bits 18:16 = 0, bits 10:8 = 1, bits 2:0 = 2, bits 14:12 = 3, bits 22:20 = 4, bit 31
 *   +0x00: bits 8, 9, 10, 11 and 0 set, bit 1 clear
 * On this page bit 0 is not the analogue RG_CSI0A_CPHY_EN: it is the HAL's own D-PHY step.
 */
static void mtk_mipi_phy_dphy_setting(struct device *dev, void __iomem *base)
{
	void __iomem *w = base + MIPI_RX_DPHY_PAGE;
	u32 a4 = readl(w + 0x04), a0 = readl(w + 0x00), w4, w0;

	a4 &= ~((0x7u << 16) | (0x7u << 8) | 0x7u | (0x7u << 12) | (0x7u << 20));
	a4 |= (0x1u << 8) | 0x2u | (0x3u << 12) | (0x4u << 20) | BIT(31);
	a0 |= 0x100 | 0x200 | 0x400 | 0x800 | BIT(0);
	a0 &= ~BIT(1);
	writel(a4, w + 0x04);
	writel(a0, w + 0x00);
	w4 = readl(w + 0x04);
	w0 = readl(w + 0x00);
	dev_info(dev, "dphy page +0x00: wrote %08x reads %08x%s\n",
		 a0, w0, w0 == a0 ? "" : "  MISMATCH");
	dev_info(dev, "dphy page +0x04: wrote %08x reads %08x%s\n",
		 a4, w4, w4 == a4 ? "" : "  MISMATCH");
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

/*
 * The stock dump's analog words, replayed.
 *
 * Source: poco-m5s/evidence/stock-session-2026-11/camera-streaming.txt (stock MIUI, camera app in the
 * foreground, read by the stock kernel's REGDUMP module): 0x11c84000..0x11c840a8 and
 * 0x11c85000..0x11c850a8, the CSI2 pair. Both halves read the same word values on stock.
 *
 * phy.stock_analog=1 writes them over whatever this driver set, at the end of power-on, and prints
 * our own values for the same address list in the REGDUMP format so the two can be diffed. Note what
 * the values say: 0x11c84000 = 0x00800002 on stock is the same word our kernel reads with no camera
 * driver loaded at all (the 18.11 diff), i.e. the closed HAL did not configure this pair for its
 * streaming camera - so this replay is the test of that conclusion, not a known-good configuration.
 */
static bool stock_analog;
module_param(stock_analog, bool, 0644);
MODULE_PARM_DESC(stock_analog,
		 "write the stock dump's analog words over this driver's at power on");

static const struct {
	u32 off;
	u32 val;
} mtk_mipi_stock_analog[] = {
	{ 0x00, 0x00800002 }, { 0x04, 0x00084444 }, { 0x08, 0x10101010 },
	{ 0x0c, 0x00001010 }, { 0x18, 0x88508850 }, { 0x1c, 0x88508850 },
	{ 0x20, 0x88508850 }, { 0x24, 0x00000000 }, { 0x30, 0x10101010 },
	{ 0x34, 0x0020000f }, { 0x3c, 0x10101010 }, { 0x40, 0x00000000 },
	{ 0x80, 0x00000002 }, { 0xa8, 0x00000007 },
};

static void mtk_mipi_phy_stock_analog(struct device *dev, void __iomem *base,
				      phys_addr_t base_pa)
{
	unsigned int i, half;

	dev_info(dev, "stock analog: writing %zu stock words to 0x%08x and 0x%08x\n",
		 ARRAY_SIZE(mtk_mipi_stock_analog), (u32)base_pa,
		 (u32)(base_pa + CSIXB_OFFSET));

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;

		for (i = 0; i < ARRAY_SIZE(mtk_mipi_stock_analog); i++)
			writel(mtk_mipi_stock_analog[i].val, h + mtk_mipi_stock_analog[i].off);

		for (i = 0; i < ARRAY_SIZE(mtk_mipi_stock_analog); i++)
			dev_info(dev, "REGDUMP camera-streaming 0x%08x 0x%08x\n",
				 (u32)(base_pa + half * CSIXB_OFFSET +
				       mtk_mipi_stock_analog[i].off),
				 readl(h + mtk_mipi_stock_analog[i].off));
	}
}

/*
 * Lane-role permutations as runtime parameters, so one boot can walk them.
 *
 * The analog block names three lanes per half (phy-mtk-mipi-csi-0-5-rx-reg.h:18-27:
 * RG_CSIXA_DPHY_L0/L1/L2_CKMODE_EN = bits 5/8/11 and their CKSEL bits 6/9/12 of register 0x00), and
 * *which* of them carries the clock of a four-lane link is the one thing no source states for this
 * board. phy.clock_lane picks it: 0..2 = A half L0..L2, 3..5 = B half L0..L2, -1 = the driver's own
 * choice. phy.polarity=1 swaps the two equaliser sub-fields of each lane register (0x18/0x1c/0x20,
 * GENMASK(5,4)/(7,6) against GENMASK(21,20)/(23,22)) - the only per-lane order bit the header names.
 * Both are read at every power-on, i.e. at every stream start.
 */
static int clock_lane = -1;
module_param(clock_lane, int, 0644);
MODULE_PARM_DESC(clock_lane,
		 "lane carrying the clock: 0..2 = A L0..L2, 3..5 = B L0..L2, -1 = as written");

static bool polarity;
module_param(polarity, bool, 0644);
MODULE_PARM_DESC(polarity, "swap the per-lane equaliser sub-fields in 0x18/0x1c/0x20");

static void mtk_mipi_phy_lane_roles(struct device *dev, void __iomem *base)
{
	static const u32 ckmode[3] = { BIT(5), BIT(8), BIT(11) };
	static const u32 cksel[3] = { BIT(6), BIT(9), BIT(12) };
	unsigned int half, lane;

	for (half = 0; half < 2; half++) {
		void __iomem *h = base + half * CSIXB_OFFSET;
		u32 a0 = readl(h + 0x00);

		if (clock_lane >= 0 && clock_lane <= 5) {
			for (lane = 0; lane < 3; lane++)
				a0 &= ~(ckmode[lane] | cksel[lane]);
			if (half == (unsigned int)(clock_lane / 3))
				a0 |= ckmode[clock_lane % 3] | cksel[clock_lane % 3];
			writel(a0, h + 0x00);
		}

		if (polarity) {
			for (lane = 0; lane < 3; lane++) {
				u32 v = readl(h + 0x18 + 4 * lane);
				u32 lo = (v >> 4) & 0xf;
				u32 hi = (v >> 20) & 0xf;

				v &= ~((0xfu << 4) | (0xfu << 20));
				v |= (hi << 4) | (lo << 20);
				writel(v, h + 0x18 + 4 * lane);
			}
		}
	}

	dev_info(dev, "lane roles: clock_lane %d, polarity swap %d, 0x00 A %08x B %08x\n",
		 clock_lane, polarity, readl(base + 0x00),
		 readl(base + CSIXB_OFFSET + 0x00));
}

/*
 * The C-PHY page of this port, with the stock HAL's two writes and nothing else.
 *
 * Where the page is: the vendor kernel maps the analogue block as one window of 0xC000 at
 * 0x11C80000 (los-mt6785, drivers/misc/mediatek/imgsensor/src/mt6785/seninf/seninf_cfg.h:12-16,
 * SENINF_MAP_BASE_ANA / SENINF_MAP_LENGTH_ANA), and the HAL addresses a port's four pages inside it
 * as +0x0000 (A), +0x1000 (B), +0x2000 (D-PHY digital) and +0x3000 (C-PHY). The offsets are the
 * .rodata table SeninfDrvImp::init (libcam.halsensor.so 0x2e580) adds to that base: 0x2000, 0x6000,
 * 0xa000 for the D-PHY wrappers and 0x3000, 0x7000, 0xb000 for the C-PHY objects. This node covers
 * one 0x4000 region, so the page is 0x3000 above its own base.
 *
 * What is written, from the HAL: hal_csirx_cphy_setting (0x33108) sets bits 0, 1 and 2 of +0x00 and
 * clears bit 3, repeating bits 0-2 for the one- and two-trio cases; hal_csirx_cphy_init (0x32198)
 * sets bit 4 of +0x18. The read-back is checked and a mismatch is logged loudly - the previous
 * attempt at this wrote into the SENINF window by mistake and read zeros, and a silent zero is what
 * wasted a trial.
 */
#define MIPI_RX_CPHY_PAGE		0x3000
#define MIPI_RX_CPHY_SETTING_BITS	(BIT(0) | BIT(1) | BIT(2))
#define MIPI_RX_CPHY_INIT_BITS		BIT(4)

static void mtk_mipi_phy_cphy(struct device *dev, void __iomem *base, unsigned int trios)
{
	void __iomem *page = base + MIPI_RX_CPHY_PAGE;
	u32 v, want, got;

	v = readl(page + 0x00);
	v |= MIPI_RX_CPHY_SETTING_BITS;
	v &= ~BIT(3);
	if (trios < 3)
		v |= BIT(0) | BIT(1);
	want = v;
	writel(want, page + 0x00);
	got = readl(page + 0x00);
	dev_info(dev, "cphy +0x00: wrote 0x%08x reads 0x%08x%s\n", want, got,
		 got == want ? "" : "  MISMATCH");

	want = readl(page + 0x18) | MIPI_RX_CPHY_INIT_BITS;
	writel(want, page + 0x18);
	got = readl(page + 0x18);
	dev_info(dev, "cphy +0x18: wrote 0x%08x reads 0x%08x%s\n", want, got,
		 got == want ? "" : "  MISMATCH");
}

/*
 * hal_csirx_phyA_init (libcam.halsensor.so 0x32208-0x32514), all 45 read-modify-writes in the HAL's order,
 * on both halves of a full port. Each row is { offset, and-mask, or-mask }, copied from the disassembly.
 * Applied to the reset values this gives, for both halves: 0x04 00084444, 0x08 10101010, 0x0c 00001010,
 * 0x18/0x1c/0x20 83548354 (83558355 once the lane receivers are on), 0x30/0x3c 01080808 (computed 19.11
 * and compared with the phone: comb42 had bits 2 and 18 of 0x18/0x1c and bit 18 of 0x20 clear).
 * The HAL ends with Efuse(), which trims 0x08/0x0c from the efuse words; not ported yet.
 */
static const u32 mtk_hal_phya_init[][3] = {
	{ 0x04, 0xfffffff8, 0x00000004 },
	{ 0x04, 0xffffff8f, 0x00000040 },
	{ 0x04, 0xfffff8ff, 0x00000400 },
	{ 0x04, 0xffff8fff, 0x00004000 },
	{ 0x04, 0xfff0ffff, 0x00080000 },
	{ 0x18, 0xffffffcf, 0x00000010 },
	{ 0x18, 0xffcfffff, 0x00100000 },
	{ 0x1c, 0xffffffcf, 0x00000010 },
	{ 0x1c, 0xffcfffff, 0x00100000 },
	{ 0x20, 0xffffffcf, 0x00000010 },
	{ 0x20, 0xffcfffff, 0x00100000 },
	{ 0x18, 0xffffff3f, 0x00000040 },
	{ 0x18, 0xff3fffff, 0x00400000 },
	{ 0x1c, 0xffffff3f, 0x00000040 },
	{ 0x1c, 0xff3fffff, 0x00400000 },
	{ 0x20, 0xffffff3f, 0x00000040 },
	{ 0x20, 0xff3fffff, 0x00400000 },
	{ 0x18, 0xffffffff, 0x00000004 },
	{ 0x18, 0xffffffff, 0x00040000 },
	{ 0x1c, 0xffffffff, 0x00000004 },
	{ 0x1c, 0xffffffff, 0x00040000 },
	{ 0x20, 0xffffffff, 0x00000004 },
	{ 0x20, 0xffffffff, 0x00040000 },
	{ 0x18, 0xfffff0ff, 0x00000300 },
	{ 0x18, 0xf0ffffff, 0x03000000 },
	{ 0x1c, 0xfffff0ff, 0x00000300 },
	{ 0x1c, 0xf0ffffff, 0x03000000 },
	{ 0x20, 0xfffff0ff, 0x00000300 },
	{ 0x20, 0xf0ffffff, 0x03000000 },
	{ 0x08, 0xffffffe0, 0x00000010 },
	{ 0x08, 0xffffe0ff, 0x00001000 },
	{ 0x08, 0xffe0ffff, 0x00100000 },
	{ 0x08, 0xe0ffffff, 0x10000000 },
	{ 0x0c, 0xffffffe0, 0x00000010 },
	{ 0x0c, 0xffffe0ff, 0x00001000 },
	{ 0x30, 0xffffffe0, 0x00000008 },
	{ 0x30, 0xffffe0ff, 0x00000800 },
	{ 0x30, 0xffe0ffff, 0x00080000 },
	{ 0x3c, 0xffffffe0, 0x00000008 },
	{ 0x3c, 0xffffe0ff, 0x00000800 },
	{ 0x3c, 0xffe0ffff, 0x00080000 },
	{ 0x28, 0xffffffff, 0x00000001 },
	{ 0x34, 0xffffffff, 0x00000001 },
	{ 0x30, 0xe0ffffff, 0x01000000 },
	{ 0x3c, 0xe0ffffff, 0x01000000 },
};

static void mtk_mipi_phy_hal_phya_init(void __iomem *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_hal_phya_init); i++) {
		void __iomem *r = h + mtk_hal_phya_init[i][0];

		writel((readl(r) & mtk_hal_phya_init[i][1]) | mtk_hal_phya_init[i][2], r);
	}
}

/* The analogue half as the HAL leaves it (computed from its writes on the reset values), against ours. */
static void mtk_mipi_phy_hal_check(struct device *dev, void __iomem *base)
{
	static const u32 want[2][16] = {
		{ 0x00801a4e, 0x00084444, 0x10101010, 0x00001010, 0x80808080, 0x00000080, 0x83558355, 0x83558355, 0x83558355, 0x00000000, 0x0020000f, 0x00000000, 0x01080808, 0x0020000f, 0x00000000, 0x01080808 },
		{ 0x0080124e, 0x00084444, 0x10101010, 0x00001010, 0x80808080, 0x00000080, 0x83558355, 0x83558355, 0x83558355, 0x00000000, 0x0020000f, 0x00000000, 0x01080808, 0x0020000f, 0x00000000, 0x01080808 },
	};
	unsigned int half, i, bad = 0;

	for (half = 0; half < 2; half++)
		for (i = 0; i < 16; i++) {
			u32 got = readl(base + half * CSIXB_OFFSET + i * 4);

			if (got != want[half][i]) {
				bad++;
				dev_info(dev, "hal check %s +0x%02x: hal %08x ours %08x  DIFF\n",
					 half ? "B" : "A", i * 4, want[half][i], got);
			}
		}
	dev_info(dev, "hal check: %u of 32 analogue words differ from the HAL's state\n", bad);
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
		/*
		 * The mode bit the in-tree header names RG_CSI0A_CPHY_EN
		 * (phy-mtk-mipi-csi-0-5-rx-reg.h:18): 0 selects D-PHY, 1 selects C-PHY. Until now this
		 * driver always cleared it, i.e. only D-PHY was ever possible.
		 */
		bool cphy = port->mode == CPHY;

		mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN, cphy);
		mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
				     RG_CSI0A_CPHY_EN, cphy);
		dev_info(&phy->dev, "mode: %s (0x00 CPHY_EN %u on both halves)\n",
			 cphy ? "C-PHY" : "D-PHY", cphy);
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

	/*
	 * The HAL's analogue init (replaces MT8365's RG_CSIXA_RESERVE = 0x40 in 0x24, which the MT6785 HAL
	 * never writes: 0x24 stays at its reset value 0 on stock).
	 */
	mtk_mipi_phy_hal_phya_init(base);
	mtk_mipi_phy_hal_phya_init(base + CSIXB_OFFSET);
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

	/*
	 * The HAL's D-PHY page steps, in its order (setSeninfCsiMipi 0x325e8): dphy_init, the C-PHY
	 * page's bit 4, then (after the CSI-2 and top settings, which live in SENINF) dphy_setting.
	 * C-PHY mode takes the HAL's C-PHY setting instead of dphy_setting.
	 */
	mtk_mipi_phy_dphy_init_bank(&phy->dev, base);
	mtk_mipi_phy_cphy_page_bit4(&phy->dev, base);
	if (!(port->type == CDPHY && port->mode == CPHY))
		mtk_mipi_phy_dphy_setting(&phy->dev, base);

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

	/* The C-PHY page of this port, when the port is in C-PHY mode. */
	if (port->type == CDPHY && port->mode == CPHY)
		mtk_mipi_phy_cphy(&phy->dev, base, port->num_lanes);

	/* The lane-role permutations a trial asks for, then the state as it stands. */
	mtk_mipi_phy_lane_roles(&phy->dev, base);
	mtk_mipi_phy_decode(&phy->dev, base, "enables");

	/* The stock dump's analog words over ours, when a trial asks for it. */
	dev_info(&phy->dev,
		 "stock_analog=%d: set %s.stock_analog=1 on the command line, or write 1 to\n",
		 stock_analog, KBUILD_MODNAME);
	dev_info(&phy->dev, "  /sys/module/%s/parameters/stock_analog before streaming\n",
		 KBUILD_MODNAME);
	if (stock_analog)
		mtk_mipi_phy_stock_analog(&phy->dev, base, port->base_pa);

	/* and the receivers of the lanes themselves, then the HAL's ready flag */
	mtk_mipi_phy_lanes(&phy->dev, base, true);
	if (!(port->type == CDPHY && port->mode == CPHY))
		mtk_mipi_phy_hal_check(&phy->dev, base);

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
		case PHY_TYPE_CPHY:
			/*
			 * C-PHY: three trios, no separate clock lane. The block is a combo, so the same
			 * instance serves both modes; power_on selects the mode by the RG_CSI0A_CPHY_EN bit
			 * and its page is programmed by mtk_mipi_phy_cphy() below. This is the mode
			 * the OV64B40 on this board needs and the only one no trial has run.
			 */
			priv->mode = CPHY;
			if (priv->num_lanes < 1 || priv->num_lanes > 3) {
				dev_err(dev, "C-PHY takes 1 to 3 trios, this node says %u\n",
					priv->num_lanes);
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

	port->base_pa = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start;

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
