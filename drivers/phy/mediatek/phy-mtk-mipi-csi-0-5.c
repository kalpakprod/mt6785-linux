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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-csi-0-5-rx-reg.h"

#define CSIXB_OFFSET		0x1000

struct mtk_mipi_cdphy_port {
	struct device *dev;
	void __iomem *base;
	struct phy *phy;
	u32 type;
	u32 mode;
	u32 num_lanes;
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

static int mtk_mipi_phy_power_on(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	mtk_mipi_phy_dump(&phy->dev, base, "before");

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

		mtk_mipi_phy_dump(&phy->dev, base, "after");

	return 0;
}

static int mtk_mipi_phy_power_off(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

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
