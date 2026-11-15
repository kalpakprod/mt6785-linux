// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): MediaTek MT6785 modem, CCCI over CCIF.
 *
 * M0: turn the modem's MTCMOS domain on from Linux and read back its power
 * status. Nothing else happens here, and in particular the CCIF registers are
 * not touched: they are only readable while the domain reports on, and
 * reading them with the domain off hangs the AXI bus. This file proves the
 * one prerequisite the whole port rests on -- that Linux can power the MD
 * domain and survive.
 *
 * The modem (MD1, generation 6295) is a second processor behind the SCPSYS
 * MTCMOS gate at 0x10006000, with its registers at 0x10209000 (AP side) and
 * 0x1020a000 (its own view). Mainline already contains the complete MT6785
 * MD power sequence (drivers/pmdomain/mediatek/mt6785-pm-domains.h:
 * MT6785_POWER_DOMAIN_MD, sta_mask BIT(0), ctl_offs 0x318, pwr_sta_offs 0x160,
 * ext_buck_iso 0x3b0, caps EXT_BUCK_ISO | MODEM_PWRSEQ), checked line by line
 * against the vendor's spm_mtcmos_ctrl_md1, so the domain is taken through the
 * generic power-domain API instead of poking 0x318 by hand.
 */
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define MD_PWR_STATUS		0x160	/* SPM PWR_STATUS, MT6785_PWR_STATUS */
#define MD_PWR_STA		BIT(0)	/* the md domain's sta_mask */

static bool m0_only = true;
module_param(m0_only, bool, 0444);
MODULE_PARM_DESC(m0_only,
		 "stop after the M0 power-domain check instead of booting the modem");

static int rosemary_ccci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *spm;
	unsigned int status;
	int ret;

	spm = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,spm");
	if (IS_ERR(spm))
		return dev_err_probe(dev, PTR_ERR(spm), "no SPM regmap\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/*
	 * Resume: attaching the device to its domain runs mainline's MT6785
	 * modem power sequence, and the usage reference keeps it on while the
	 * driver uses it.
	 */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the md domain\n");

	regmap_read(spm, MD_PWR_STATUS, &status);
	dev_info(dev, "rosemary_ccci: M0 md power status %u\n",
		 status & MD_PWR_STA);

	pm_runtime_put(dev);

	if (!m0_only)
		dev_warn(dev, "stages past M0 are not in this build yet\n");

	return 0;
}

static const struct of_device_id rosemary_ccci_of_match[] = {
	{ .compatible = "xiaomi,rosemary-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, rosemary_ccci_of_match);

static struct platform_driver rosemary_ccci_driver = {
	.probe = rosemary_ccci_probe,
	.driver = {
		.name = "rosemary-ccci",
		.of_match_table = rosemary_ccci_of_match,
	},
};
module_platform_driver(rosemary_ccci_driver);

MODULE_DESCRIPTION("MediaTek MT6785 modem (CCCI over CCIF), Poco M5s");
MODULE_LICENSE("GPL");
