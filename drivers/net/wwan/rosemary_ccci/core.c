// SPDX-License-Identifier: GPL-2.0-only
/*
 * Poco M5s (rosemary): MediaTek MT6785 modem -- probe and boot information.
 *
 * The modem is MD1, generation 6295: a second processor behind the SCPSYS
 * MTCMOS gate at 0x10006000, reaching the AP over CCIF for control and DPMAIF
 * for packet data. This file is the driver's entry point.
 *
 *   M0  probe takes the domain through runtime PM and reads back its power
 *       status. Nothing else happens: the modem's own registers are only legal
 *       to read while the domain reports on, and a read with it off hangs the
 *       AXI bus.
 *   M1  probe then maps the register windows and the two share-memory windows
 *       and reads the bootloader's boot information (tags.c), which prints the
 *       modem's DRAM window, the share-memory layout, the logging buffers and
 *       the image's own capability. Nothing is written to the modem here.
 *
 * The stages are selected by the module parameter stop_after; the build that
 * ships a later stage adds the boot state machine and the transports on top.
 *
 * Where the facts come from: the vendor's MT6785 platform code in LineageOS
 * android_kernel_xiaomi_mt6785 30485853fa89 (drivers/misc/mediatek/eccci/), the
 * port plan (poco-m5s/research/native/4-modem/5-port-plan.md), and for the part
 * mainline already owns, drivers/pmdomain/mediatek/mt6785-pm-domains.h.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "rosemary_ccci.h"

static bool m0_only;
module_param(m0_only, bool, 0444);
MODULE_PARM_DESC(m0_only, "stop at the M0 power-domain check");

static char *stop_after = "m1";
module_param(stop_after, charp, 0444);
MODULE_PARM_DESC(stop_after,
		 "the last stage this build runs: m0, m1, m2, m3 or m4");

static const char *const state_names[] = {
	[MD_STATE_OFF] = "off",
	[MD_STATE_BOOTING] = "booting",
	[MD_STATE_HS1] = "handshake",
	[MD_STATE_READY] = "ready",
	[MD_STATE_EXCEPTION] = "exception",
};

/* the stage the module was told to stop at; m0_only overrides it */
static int stage_limit(void)
{
	if (m0_only || !stop_after)
		return 0;
	if (!strcmp(stop_after, "m1"))
		return 1;
	if (!strcmp(stop_after, "m2"))
		return 2;
	if (!strcmp(stop_after, "m3"))
		return 3;
	if (!strcmp(stop_after, "m4"))
		return 4;
	return 0;
}

void ccci_set_state(struct ccci_md *md, enum ccci_state state)
{
	if (md->state == state)
		return;
	md->state = state;
	dev_info(md->dev, "state %s\n", state_names[state]);
	sysfs_notify(&md->dev->kobj, NULL, "state");
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct ccci_md *md = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", state_names[md->state]);
}
static DEVICE_ATTR_RO(state);

static struct attribute *ccci_attrs[] = {
	&dev_attr_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ccci);

static int md_map_regs(struct ccci_md *md)
{
	struct platform_device *pdev = to_platform_device(md->dev);

	md->ccif_ap = devm_platform_ioremap_resource_byname(pdev, "ap-ccif");
	if (IS_ERR(md->ccif_ap))
		return PTR_ERR(md->ccif_ap);
	md->ccif_md = devm_platform_ioremap_resource_byname(pdev, "md-ccif");
	if (IS_ERR(md->ccif_md))
		return PTR_ERR(md->ccif_md);

	md->spm_map = syscon_regmap_lookup_by_phandle(md->dev->of_node,
						     "mediatek,spm");
	if (IS_ERR(md->spm_map))
		return dev_err_probe(md->dev, PTR_ERR(md->spm_map),
				     "the SPM syscon\n");
	return 0;
}

static int rosemary_ccci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ccci_md *md;
	unsigned int status;
	int ret, i;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;
	md->dev = dev;
	md->first_boot = true;
	mutex_init(&md->lock);
	init_completion(&md->hs1);
	init_completion(&md->hs2);
	for (i = 0; i < CCIF_QUEUES; i++)
		spin_lock_init(&md->tx_lock[i]);
	platform_set_drvdata(pdev, md);

	/* M0: the domain must be powerable and PWR_STATUS must read 1 */
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the md domain\n");
	regmap_read(md->spm_map, SPM_PWR_STATUS, &status);
	pm_runtime_put(dev);
	dev_info(dev, "rosemary_ccci: M0 md power status %u\n",
		 status & BIT(0));

	if (stage_limit() < 1) {
		dev_info(dev, "M0 only: set stop_after=m1 for the boot information\n");
		return 0;
	}

	/* M1: read-only. Map the windows and walk the loader's tag list. */
	ret = md_map_regs(md);
	if (ret)
		return ret;
	ret = rosemary_ccci_tags_init(md);
	if (ret)
		return dev_err_probe(dev, ret,
				     "the loader's CCCI boot information\n");

	/* the two share-memory windows, mapped uncached like the modem sees them */
	md->nc = devm_memremap(dev, md->nc_phys, md->nc_size, MEMREMAP_WC);
	if (IS_ERR(md->nc))
		return dev_err_probe(dev, PTR_ERR(md->nc), "ap_md_nc_smem\n");
	md->c = devm_memremap(dev, md->c_phys, md->c_size, MEMREMAP_WC);
	if (IS_ERR(md->c))
		return dev_err_probe(dev, PTR_ERR(md->c), "ap_md_c_smem\n");

	dev_info(dev, "M1 done: nothing has been written to the modem\n");
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
		.dev_groups = ccci_groups,
		.of_match_table = rosemary_ccci_of_match,
	},
};
module_platform_driver(rosemary_ccci_driver);

MODULE_DESCRIPTION("MediaTek MT6785 modem (CCCI over CCIF), Poco M5s");
MODULE_LICENSE("GPL");
