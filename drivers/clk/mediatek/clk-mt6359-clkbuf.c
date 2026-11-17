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
 * Mechanism, and why it is implemented this way. The vendor's MT6785 clock buffer
 * driver is in this repository after all, and it settles the question the register
 * header alone could not:
 *
 *   drivers/misc/mediatek/base/power/clkbuf_v1/mt6785/mtk_clkbuf_hw.c
 *     :402-437   clk_buf_ctrl_internal(CLK_BUF_NFC, ...) - the only code that
 *                switches this buffer. FORCE_ON clears the CW00 mode field, then
 *                SETs EXTBUF3_EN_M; FORCE_OFF clears the mode field, then CLEARs
 *                EXTBUF3_EN_M; INIT_SETTING writes the mode from a board table.
 *                It touches neither the pwrap DCXO_ENABLE bit in a useful way (it
 *                clears it in both directions) nor any vote field.
 *     :747-769   clk_buf_ctrl(CLK_BUF_NFC, ...) only records the intent; the
 *                buffer's real control is the DCXO CW00 pair above.
 *     :778-805   clk_buf_disp_ctrl() does the same two writes - the display code
 *                borrows this buffer, which is how the sequence is confirmed.
 *
 *   upmu_hw.h
 *     :358-360   MT6359_DCXO_CW00 = base+0x788, CW00_SET = +0x78a, CW00_CLR = +0x78c
 *     :5535-5542 PMIC_XO_EXTBUF3_MODE (mask 0x3, shift 6) and _EN_M (mask 0x1,
 *                shift 8) - EXTBUF3 is the NFC buffer
 *
 * The vote field (XO_BUF_CTL2, 0x7ff at bit 0) exists and the PMIC API can write it
 * (mt6359_upmu_set_xo_nfc_vote(), pmic/mt6359/v1/pmic_api.c:10384-10393), but the
 * MT6785 clock buffer driver never calls that function: the field is what the
 * loader and the PMIC's own hardware use, and its bits are not attributable to
 * masters from any source in this repository. So this driver does not write it at
 * all - doing so could drop the modem's or Wi-Fi's votes.
 *
 * What it writes instead is the vendor's pair of set/clear registers, which is a
 * read-modify-write that cannot disturb anything else by construction: writing 1
 * to CW00_SET sets only that bit, writing 1 to CW00_CLR clears only that bit.
 * enable() clears the mode field and sets EN_M; disable() clears both; is_enabled()
 * reads CW00 itself, so the state reported is the state in the PMIC.
 *
 * Only the NFC buffer is writable. The other four (SOC, WCN, CEL, EXT) are exposed
 * read-only on purpose: the modem, Wi-Fi/BT and UFS own them through paths this
 * driver knows nothing about, and a kernel-side gate for them would be a way to
 * break them. They report their state from the vote register and are registered
 * with CLK_IGNORE_UNUSED so the clock core's late-init cleanup never touches them.
 *
 * Follows drivers/clk/clk-bd718x7.c - a PMIC clock output as an MFD child on the
 * parent's regmap, with a struct clk_hw per output and devm registration.
 */
#include <linux/clk-provider.h>
#include <linux/mfd/mt6397/core.h>
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

/* The NFC buffer, EXTBUF3, is controlled through the DCXO CW00 set/clear pair. */
#define MT6359_DCXO_CW00	0x788
#define MT6359_DCXO_CW00_SET	0x78a
#define MT6359_DCXO_CW00_CLR	0x78c
#define MT6359_XO_EXTBUF3_MODE_MASK	0x3
#define MT6359_XO_EXTBUF3_MODE_SHIFT	6
#define MT6359_XO_EXTBUF3_EN_M_MASK	0x1
#define MT6359_XO_EXTBUF3_EN_M_SHIFT	8

#define MT6359_XO_DEFAULT_HZ	26000000

struct mt6359_clkbuf {
	struct clk_hw hw;
	struct regmap *regmap;
	u16 reg;
	u32 rate;
	bool writable;
	/* Value of the vote field when this driver probed, logged as evidence. */
	u32 boot_val;
};

static int mt6359_clkbuf_enable(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);
	int ret;

	if (!c->writable)
		return -EOPNOTSUPP;

	/* The vendor's order (mtk_clkbuf_hw.c:402-412): mode first, then EN_M. */
	ret = regmap_write(c->regmap, MT6359_DCXO_CW00_CLR,
			   MT6359_XO_EXTBUF3_MODE_MASK
			   << MT6359_XO_EXTBUF3_MODE_SHIFT);
	if (ret)
		return ret;
	return regmap_write(c->regmap, MT6359_DCXO_CW00_SET,
			    MT6359_XO_EXTBUF3_EN_M_MASK
			    << MT6359_XO_EXTBUF3_EN_M_SHIFT);
}

static void mt6359_clkbuf_disable(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);

	if (!c->writable)
		return;

	/* Same two writes, both clearing; see mtk_clkbuf_hw.c:415-424. */
	regmap_write(c->regmap, MT6359_DCXO_CW00_CLR,
		     MT6359_XO_EXTBUF3_MODE_MASK << MT6359_XO_EXTBUF3_MODE_SHIFT);
	regmap_write(c->regmap, MT6359_DCXO_CW00_CLR,
		     MT6359_XO_EXTBUF3_EN_M_MASK << MT6359_XO_EXTBUF3_EN_M_SHIFT);
}

static int mt6359_clkbuf_is_enabled(struct clk_hw *hw)
{
	struct mt6359_clkbuf *c = container_of(hw, struct mt6359_clkbuf, hw);
	u32 val = 0;

	if (c->writable) {
		if (regmap_read(c->regmap, MT6359_DCXO_CW00, &val))
			return -EIO;
		return !!(val & (MT6359_XO_EXTBUF3_EN_M_MASK
				 << MT6359_XO_EXTBUF3_EN_M_SHIFT));
	}

	/* Read-only buffers: the vote field is the loader's, and reading it is the
	 * only thing this driver does with it. */
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

/*
 * prepare/unprepare, not enable/disable: the PMIC regmap goes through the pwrap and
 * sleeps, while clk_enable() runs under clk_core_enable_lock() in atomic context -
 * which produced "scheduling while atomic" in clk_core_enable_lock() ->
 * mt6359_clkbuf_enable() -> regmap_write() -> regmap_lock_mutex() on conn19.
 * The CCF guarantees that prepare-family callbacks may sleep, and every PMIC clock
 * driver with a sleeping regmap does it this way: drivers/clk/clk-bd718x7.c:74-78
 * (.prepare/.unprepare/.is_prepared over regmap_update_bits).
 */
static const struct clk_ops mt6359_clkbuf_ops = {
	.prepare = mt6359_clkbuf_enable,
	.unprepare = mt6359_clkbuf_disable,
	.is_prepared = mt6359_clkbuf_is_enabled,
	.recalc_rate = mt6359_clkbuf_recalc_rate,
};

/*
 * Only XO_NFC is writable: it is the one the vendor driver controls by hand
 * (mtk_clkbuf_hw.c:402-437). The others are reported, never touched.
 */
static const struct {
	const char *name;
	u16 reg;
	bool writable;
} mt6359_clkbufs[] = {
	[MT6359_CLKBUF_XO_SOC] = { "xo_soc", MT6359_XO_BUF_CTL0, false },
	[MT6359_CLKBUF_XO_WCN] = { "xo_wcn", MT6359_XO_BUF_CTL1, false },
	[MT6359_CLKBUF_XO_NFC] = { "xo_nfc", MT6359_XO_BUF_CTL2, true },
	[MT6359_CLKBUF_XO_CEL] = { "xo_cel", MT6359_XO_BUF_CTL3, false },
	[MT6359_CLKBUF_XO_EXT] = { "xo_ext", MT6359_XO_BUF_CTL4, false },
};

static int mt6359_clkbuf_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *onecell;
	struct mt6359_clkbuf *bufs;
	struct device *dev = &pdev->dev;
	struct mt6397_chip *chip = dev_get_drvdata(dev->parent);
	struct regmap *regmap;
	unsigned int i;
	u32 rate = MT6359_XO_DEFAULT_HZ;
	int ret;

	/*
	 * The regmap is the PMIC chip's, not the parent device's: mt6397-core takes
	 * it from *its* parent (the pwrap) and keeps it in struct mt6397_chip
	 * (drivers/mfd/mt6397-core.c:382), which every sibling uses - e.g.
	 * mt6359_regulator_probe() (drivers/regulator/mt6359-regulator.c:1040).
	 * dev_get_regmap(dev->parent) finds nothing here, because the MFD device owns
	 * no regmap of its own: that was the -ENODEV on conn16 that kept st-nci
	 * deferred on its clock supplier.
	 */
	if (!chip)
		return dev_err_probe(dev, -ENODEV, "no MT6397 chip from the parent\n");
	regmap = chip->regmap;
	if (!regmap)
		return dev_err_probe(dev, -ENODEV, "the MT6397 chip has no regmap\n");

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
		c->writable = mt6359_clkbufs[i].writable;

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
		dev_info(dev, "%s: %s, vote register 0x%x = 0x%04x (%s at probe)\n",
			 mt6359_clkbufs[i].name,
			 c->writable ? "writable through DCXO CW00" : "read-only",
			 c->reg, val,
			 c->boot_val ? "on" : "off");

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
