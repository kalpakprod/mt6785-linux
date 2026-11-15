// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek eFuse copy handed over by the bootloader ("devinfo")
 *
 * On phones such as the MT6785 the eFuse block is readable only from the
 * secure world: non-secure reads of 0x11c10000 return zeroes. The loader (LK)
 * copies the eFuse words the kernel needs into the property "atag,devinfo"
 * of /chosen: a little-endian u32 count followed by that many u32 words, the
 * same array the vendor kernel reads with get_devinfo_with_index(). This
 * driver exposes the words as a read-only NVMEM device, word i at byte
 * offset 4 * i, so consumers (LVTS calibration) describe their cells by the
 * vendor index.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

struct mtk_devinfo {
	const u8 *data;
	size_t size;
};

static int mtk_devinfo_read(void *context, unsigned int offset, void *val,
			    size_t bytes)
{
	struct mtk_devinfo *di = context;

	if (offset >= di->size || bytes > di->size - offset)
		return -EINVAL;

	memcpy(val, di->data + offset, bytes);

	return 0;
}

static int mtk_devinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {};
	struct device_node *chosen;
	struct mtk_devinfo *di;
	const __le32 *prop;
	u32 count;
	int len;

	di = devm_kzalloc(dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return dev_err_probe(dev, -ENODEV, "no /chosen node\n");
	prop = of_get_property(chosen, "atag,devinfo", &len);
	of_node_put(chosen);
	if (!prop || len < sizeof(u32))
		return dev_err_probe(dev, -ENODEV,
				     "the bootloader passed no atag,devinfo\n");

	count = le32_to_cpu(prop[0]);
	if (!count || count > (len - sizeof(u32)) / sizeof(u32))
		return dev_err_probe(dev, -EINVAL,
				     "atag,devinfo: %u words in %d bytes\n",
				     count, len);

	di->data = devm_kmemdup(dev, prop + 1, count * sizeof(u32), GFP_KERNEL);
	if (!di->data)
		return -ENOMEM;
	di->size = count * sizeof(u32);

	config.dev = dev;
	config.name = "mtk-devinfo";
	config.id = NVMEM_DEVID_NONE;
	config.owner = THIS_MODULE;
	config.read_only = true;
	config.word_size = 1;
	config.stride = 1;
	config.size = di->size;
	config.reg_read = mtk_devinfo_read;
	config.priv = di;

	dev_info(dev, "%u eFuse words from the bootloader\n", count);

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id mtk_devinfo_of_match[] = {
	{ .compatible = "mediatek,mt6785-devinfo" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_devinfo_of_match);

static struct platform_driver mtk_devinfo_driver = {
	.probe = mtk_devinfo_probe,
	.driver = {
		.name = "mtk-devinfo",
		.of_match_table = mtk_devinfo_of_match,
	},
};
module_platform_driver(mtk_devinfo_driver);

MODULE_DESCRIPTION("MediaTek eFuse copy from the bootloader");
MODULE_LICENSE("GPL");
