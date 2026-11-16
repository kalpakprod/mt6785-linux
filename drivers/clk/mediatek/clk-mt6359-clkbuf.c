// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6359 PMIC clock buffer (XO) outputs.
 *
 * The MT6359 drives several 26 MHz clock buffers to the SoC and its
 * peripherals: SOC, WCN (connectivity), NFC, CEL and EXT (UFS). Each one has its
 * own control register, MT6359_XO_BUF_CTL0..4, and - see "Mechanism" below -
 * a vote field rather than a plain enable bit.
 *
 * Where the numbers come from (vendor 4.19, the only source that has them):
 *
 *   drivers/misc/mediatek/include/mt-plat/mt6785/include/mach/upmu_hw.h
 *     :295-299   MT6359_XO_BUF_CTL0 = PMIC_REG_BASE+0x54c ... CTL4 = +0x554
 *     :4907-4927 PMIC_XO_{SOC,WCN,NFC,CEL,EXT}_VOTE_{ADDR,MASK,SHIFT} - the mask
 *                is 0x7ff at shift 0 for every buffer, i.e. eleven vote bits and
 *                no separate enable bit in the same register
 *     :4928-4932 PMIC_XO_MODE_CONN_BT_MASK / PMIC_XO_BUF_CONN_BT_MASK in
 *                MT6359_XO_CONN_BT0 (+0x556), which are *conn/BT specific* and
 *                not touched here
 *   drivers/misc/mediatek/include/mt-plat/mtk-clkbuf-bridge.h:20-36
 *                enum clk_buf_id { BB_MD, CONN, NFC, RF, UFS } and
 *                enum xo_id { SOC, WCN, NFC, CEL, AUD, PD, EXT } - the buffer
 *                list exposed below, minus the two the vendor marks disabled
 *
 * Mechanism, and why it is implemented the way it is. The vendor's PMIC layer is
 * in this repository and settles the shape: mt6359_upmu_set_xo_nfc_vote() writes
 * the whole field through pmic_config_interface(MT6359_XO_BUF_CTL2, val,
 * PMIC_XO_NFC_VOTE_MASK 0x7ff, shift 0)
 * (4-modem/src/los-mt6785/drivers/misc/mediatek/pmic/mt6359/v1/pmic_api.c:10384-10393,
 * declared in pmic/include/mt6359/mtk_pmic_api.h:750). So a caller passes a whole
 * vote word, there is no separate enable bit in that register, and no getter
 * exists anywhere in the tree - the vendor never reads the field back. What is
 * still not knowable from these sources is which bit belongs to which master: the
 * clkbuf driver that calls this setter is not in the tree, so the individual
 * votes, and the value it passes to switch a buffer on, are unknown. Two
 * consequences, both deliberate:
 *
 *   - enable writes the whole vote field (0x7ff), which is the only value that
 *     cannot be mistaken for "somebody else's vote";
 *   - disable restores the value the register held when this driver probed,
 *     captured in ->boot_val, instead of writing zero. A zero would silently
 *     drop every other master's vote on a buffer this driver does not own.
 *
 * And every clock is registered with CLK_IGNORE_UNUSED: the clock core's
 * late-init cleanup disables clocks nobody enabled, and on this phone the
 * modem, Wi-Fi, BT and UFS buffers are enabled by the loader and their votes
 * must survive a kernel that has no consumer for them. A consumer that does ask
 * for a clock still gets a working enable/disable.
 *
 * Follows drivers/clk/clk-bd718x7.c - a PMIC clock output as an MFD child on the
 * parent's regmap, with a struct clk_hw per output and devm registration.
 */
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/mediatek,mt6359-clkbuf.h>

#define MT6359_XO_BUF_CTL0	0x54c
#define MT6359_XO_BUF_CTL1	0x54e
#define MT6359_XO_BUF_CTL2	0x550
#define MT6359_XO_BUF_CTL3	0x552
#define MT6359_XO_BUF_CTL4	0x554

/* Eleven vote bits at bit 0; see PMIC_XO_*_VOTE_MASK in the vendor header. */
#define MT6359_XO_BUF_VOTE_MASK	0x7ff

#define MT6359_XO_DEFAULT_HZ	26000000

struct mt6359_clkbuf {
	struct clk_hw hw;
	struct regmap *regmap;
	u16 reg;
	u32 rate;
	/* Value of the vote field when this driver probed: what disable() restores. */
	u32 boot_val;
};

static int mt6359_clkbuf_enable(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);

	return regmap_update_bits(c->regmap, c->reg, MT6359_XO_BUF_VOTE_MASK,
				  MT6359_XO_BUF_VOTE_MASK);
}

static void mt6359_clkbuf_disable(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);

	regmap_update_bits(c->regmap, c->reg, MT6359_XO_BUF_VOTE_MASK,
			   c->boot_val & MT6359_XO_BUF_VOTE_MASK);
}

static int mt6359_clkbuf_is_enabled(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);
	u32 val = 0;

	if (regmap_read(c->regmap, c->reg, &val))
		return -EIO;

	return !!(val & MT6359_XO_BUF_VOTE_MASK);
}

static unsigned long mt6359_clkbuf_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);

	(void)parent_rate;
	return c->rate;
}

static const struct clk_ops mt6359_clkbuf_ops = {
	.enable = mt6359_clkbuf_enable,
	.disable = mt6359_clkbuf_disable,
	.is_enabled = mt6359_clkbuf_is_enabled,
	.recalc_rate = mt6359_clkbuf_recalc_rate,
};

/* One entry per buffer the vendor bridge header lists as usable. */
static const struct {
	const char *name;
	u16 reg;
} mt6359_clkbufs[] = {
	[MT6359_CLKBUF_XO_SOC] = { "xo_soc", MT6359_XO_BUF_CTL0 },
	[MT6359_CLKBUF_XO_WCN] = { "xo_wcn", MT6359_XO_BUF_CTL1 },
	[MT6359_CLKBUF_XO_NFC] = { "xo_nfc", MT6359_XO_BUF_CTL2 },
	[MT6359_CLKBUF_XO_CEL] = { "xo_cel", MT6359_XO_BUF_CTL3 },
	[MT6359_CLKBUF_XO_EXT] = { "xo_ext", MT6359_XO_BUF_CTL4 },
};

static int mt6359_clkbuf_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *onecell;
	struct mt6359_clkbuf *bufs;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	unsigned int i;
	u32 rate = MT6359_XO_DEFAULT_HZ;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV,
				     "no PMIC regmap from the parent\n");

	of_property_read_u32(dev->of_node, "clock-frequency", &rate);

	bufs = devm_kcalloc(dev, ARRAY_SIZE(mt6359_clkbufs), sizeof(*bufs),
			    GFP_KERNEL);
	if (!bufs)
		return -ENOMEM;

	onecell = devm_kzalloc(dev, struct_size(onecell, hws,
						ARRAY_SIZE(mt6359_clkbufs)) +
				      sizeof(*onecell),
			       GFP_KERNEL);
	if (!onecell)
		return -ENOMEM;
	onecell->num = ARRAY_SIZE(mt6359_clkbufs);

	for (i = 0; i < ARRAY_SIZE(mt6359_clkbufs); i++) {
		struct clk_init_data init = { };
		struct mt6359_clkbuf *c = &bufs[i];
		u32 val = 0;

		c->regmap = regmap;
		c->reg = mt6359_clkbufs[i].reg;
		c->rate = rate;

		/*
		 * The value the loader left. Read, log and keep it: this is both the
		 * restore value for disable() and the evidence a trial reads to see
		 * what the bootloader had switched on.
		 */
		if (regmap_read(regmap, c->reg, &val)) {
			dev_warn(dev, "%s: cannot read register 0x%x\n",
				 mt6359_clkbufs[i].name, c->reg);
			continue;
		}
		c->boot_val = val & MT6359_XO_BUF_VOTE_MASK;
		dev_info(dev, "%s: reg 0x%x = 0x%04x, votes 0x%03x (%s)\n",
			 mt6359_clkbufs[i].name, c->reg, val, c->boot_val,
			 c->boot_val ? "on at probe" : "off at probe");

		init.name = mt6359_clkbufs[i].name;
		init.ops = &mt6359_clkbuf_ops;
		init.flags = CLK_IGNORE_UNUSED;
		init.num_parents = 0;
		c->hw.init = &init;

		ret = devm_clk_hw_register(dev, &c->hw);
		if (ret)
			return dev_err_probe(dev, ret, "registering %s failed\n",
					     init.name);
		onecell->hws[i] = &c->hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, onecell);
	if (ret)
		return dev_err_probe(dev, ret, "adding the clock provider failed\n");

	return 0;
}

static const struct of_device_id mt6359_clkbuf_of_match[] = {
	{ .compatible = "mediatek,mt6359-clkbuf" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6359_clkbuf_of_match);

static struct platform_driver mt6359_clkbuf_driver = {
	.probe = mt6359_clkbuf_probe,
	.driver = {
		.name = "mt6359-clkbuf",
		.of_match_table = mt6359_clkbuf_of_match,
	},
};
module_platform_driver(mt6359_clkbuf_driver);

MODULE_DESCRIPTION("MediaTek MT6359 PMIC XO clock buffers");
MODULE_LICENSE("GPL");
