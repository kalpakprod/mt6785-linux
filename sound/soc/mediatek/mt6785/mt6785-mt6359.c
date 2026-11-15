// SPDX-License-Identifier: GPL-2.0-only
/*
 * mt6785-mt6359.c -- ALSA SoC machine driver for the MT6785 boards with the
 * MT6359 PMIC codec and the Awinic AW87xxx speaker amplifiers.
 *
 * Copyright (c) 2020 MediaTek Inc. (the mt8192 board driver this follows)
 * Copyright (c) 2021 XiaoMi, Inc. (the vendor board driver, rosemary)
 *
 * The card is reduced to what the Poco M5s has: playback and capture through
 * the MT6359 codec, the codec's analog outputs taken by the two AW87xxx
 * amplifiers (the bottom speaker's and the earpiece's, which together make the
 * phone's stereo pair), and the headset jack, which the MT6359 ACCDET block
 * owns.
 *
 * Taken from the vendor machine driver sound/soc/mediatek/mt6785/
 * mt6785-mt6359.c of LineageOS' android_kernel_xiaomi_mt6785
 * (scratch/native/4-modem/src/los-mt6785), which routes "LINEOUT L" into the
 * bottom speaker's amplifier and "Receiver" into the earpiece's, and from the
 * mt8192 board driver in this tree for the card's shape.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../common/mtk-soundcard-driver.h"
#include "../../codecs/mt6359.h"

#define CARD_NAME	"mt6785_mt6359"
#define DRIVER_NAME	"mt6785_mt6359"

/* The front-end DAIs of the MT6785 front end this card plays through. */
/*
 * The codec side is named, not taken from a phandle: the MT6359 codec registers
 * itself as the component "mt6359-sound" (the MFD cell's device name) with the
 * two AIFs below. A component must be described by either its name or its node,
 * never by both, which is exactly how the other MediaTek cards do it.
 */
SND_SOC_DAILINK_DEFS(playback,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif1")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif2")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

static int mt6785_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;

	/* The AFE and the PMIC codec talk over MTKAIF protocol 2, clock phase 2 */
	mt6359_set_mtkaif_protocol(cmpnt_codec, MT6359_MTKAIF_PROTOCOL_2_CLK_P2);

	return 0;
}

static struct snd_soc_dai_link mt6785_mt6359_dai_links[] = {
	{
		.name = "MT6785_DL1",
		.stream_name = "Playback",
		.cpus = playback_cpus,
		.num_cpus = ARRAY_SIZE(playback_cpus),
		.codecs = playback_codecs,
		.num_codecs = ARRAY_SIZE(playback_codecs),
		.platforms = playback_platforms,
		.num_platforms = ARRAY_SIZE(playback_platforms),
		.init = mt6785_mt6359_init,
	},
	{
		.name = "MT6785_UL1",
		.stream_name = "Capture",
		.cpus = capture_cpus,
		.num_cpus = ARRAY_SIZE(capture_cpus),
		.codecs = capture_codecs,
		.num_codecs = ARRAY_SIZE(capture_codecs),
		.platforms = capture_platforms,
		.num_platforms = ARRAY_SIZE(capture_platforms),
		.init = mt6785_mt6359_init,
	},
};

static struct snd_soc_card mt6785_mt6359_card = {
	.name = CARD_NAME,
	.driver_name = DRIVER_NAME,
	.owner = THIS_MODULE,
	.dai_link = mt6785_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6785_mt6359_dai_links),
};

/*
 * The two amplifiers are not DAIs: they take their audio in analog from the
 * codec and are switched by the sound card's DAPM graph. Each one joins the
 * card as an auxiliary component, and the routes that carry audio into them
 * come from the board's "audio-routing" property ("Spk Amp" <- "LINEOUT L" and
 * "Rcv Amp" <- "Receiver"), which is why the two i2c nodes carry
 * sound-name-prefix "Spk" and "Rcv".
 */
static struct snd_soc_aux_dev mt6785_mt6359_aux_devs[2];
static struct snd_soc_dai_link_component mt6785_mt6359_aux_dlc[2];

static int mt6785_mt6359_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6785_mt6359_card;
	struct device *dev = &pdev->dev;
	struct device_node *platform_node;
	struct snd_soc_dai_link *link;
	int i, ret;

	card->dev = dev;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		return ret;

	platform_node = of_parse_phandle(dev->of_node, "mediatek,platform", 0);
	if (!platform_node)
		return dev_err_probe(dev, -EINVAL,
				     "mediatek,platform is missing\n");

	for_each_card_prelinks(card, i, link)
		link->platforms->of_node = platform_node;

	if (of_property_present(dev->of_node, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret)
			goto err_put;
	}

	ret = of_count_phandle_with_args(dev->of_node, "aux-devs", NULL);
	if (ret > 0 && ret <= ARRAY_SIZE(mt6785_mt6359_aux_devs)) {
		card->aux_dev = mt6785_mt6359_aux_devs;
		card->num_aux_devs = ret;
		for (i = 0; i < ret; i++) {
			mt6785_mt6359_aux_dlc[i].of_node =
				of_parse_phandle(dev->of_node, "aux-devs", i);
			mt6785_mt6359_aux_devs[i].dlc = mt6785_mt6359_aux_dlc[i];
		}
	}

	ret = devm_snd_soc_register_card(dev, card);

err_put:
	of_node_put(platform_node);

	return ret;
}

static const struct of_device_id mt6785_mt6359_dt_match[] = {
	{ .compatible = "mediatek,mt6785-mt6359-sound" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6785_mt6359_dt_match);

static struct platform_driver mt6785_mt6359_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mt6785_mt6359_dt_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = mt6785_mt6359_dev_probe,
};
module_platform_driver(mt6785_mt6359_driver);

MODULE_DESCRIPTION("MT6785-MT6359-AW87xxx ALSA SoC machine driver");
MODULE_AUTHOR("MediaTek Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
