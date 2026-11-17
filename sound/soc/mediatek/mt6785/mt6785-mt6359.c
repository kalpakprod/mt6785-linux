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
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../common/mtk-soundcard-driver.h"
#include "mt6785-afe-clk.h"
#include "mt6785-afe-common.h"
#include "mt6785-afe-gpio.h"
#include "../../codecs/mt6359-accdet.h"
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
/*
 * The card is dynamic, as every mainline MediaTek card is: the user-visible
 * PCMs are front ends whose data the AFE moves, and the codec sits on a back end
 * that is started with whichever front end is running. That is what gives the
 * AFE's ADDA DAI a stream (and with it its hw_params, the capture source's
 * configuration) and what makes the AFE's playback and capture ports live
 * widgets whenever audio flows.
 */

/* front ends: the memory interfaces userspace opens */
SND_SOC_DAILINK_DEFS(playback,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hostless_speech,
		     DAILINK_COMP_ARRAY(COMP_CPU("Hostless Speech DAI")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hostless_lpbk,
		     DAILINK_COMP_ARRAY(COMP_CPU("Hostless LPBK DAI")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Back ends. Both directions go through the codec's primary AIF: the MTKAIF wire
 * between the AFE and the PMIC carries playback and capture on AIF1, and AIF2 is
 * the second, four-channel link (the vendor's board driver splits it the same
 * way: its "Primary Codec" back end pairs the ADDA DAI with
 * "mt6359-snd-codec-aif1" for dpcm_playback and dpcm_capture alike, and only
 * "Primary Codec CH34" uses aif2).
 */
SND_SOC_DAILINK_DEFS(primary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("mt6359-sound",
						   "mt6359-snd-codec-aif1")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Voice calls: the modem's voice port is the AFE's second PCM interface (MD1 on
 * PCM 2 — "AFE_TO_MD1" is fed by "PCM 2 Playback" and "PCM 2 Capture" feeds
 * "MD1_TO_AFE"), and the AFE connects it straight to the codec with the routes
 * in mt6785-dai-hostless.c. No data goes through memory; the speech daemon holds
 * the hostless speech front end open for the length of a call and this back end
 * is started with it.
 */
SND_SOC_DAILINK_DEFS(modem_pcm2,
		     DAILINK_COMP_ARRAY(COMP_CPU("PCM 2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* The headset jack the codec's accessory detection reports into */
static struct snd_soc_jack mt6785_mt6359_jack;

static struct snd_soc_jack_pin mt6785_mt6359_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

/* The codec's accessory detection, found through mediatek,accdet at probe */
static struct snd_soc_component *mt6785_mt6359_accdet;

/*
 * The board's jack. The codec has the electrical ends of these paths
 * ("Headphone L"/"Headphone R", "AIN0"/"AIN1"/"AIN2", "LINEOUT L", "Receiver");
 * the jack is the one connector the board hangs an end on.
 *
 * The microphones are deliberately *not* here: they are the codec's own input pins
 * (SND_SOC_DAPM_INPUT("AIN0"/"AIN1"/"AIN2"), mt6359.c:2402-2404), which are DAPM source
 * endpoints, and a card-level microphone widget wired to them turned each of them into the
 * *sink* of a route instead - after which DAPM found no connected source behind the codec's
 * upstream chain and powered only the digital half of a capture (audio23: UL_SRC on,
 * 0x238e = 1, while the ADC clock, MIC_BIAS_0 and the MTKAIF transmitter stayed off and the
 * analog block sat at 0x4340 instead of the vendor's 0x5343). The vendor's own machine
 * driver has no microphone widgets or routes at all (vendor mt6785-mt6359.c:422-448): its
 * capture path is powered inside the codec.
 */
static const struct snd_soc_dapm_widget mt6785_mt6359_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_soc_dapm_route mt6785_mt6359_routes[] = {
	/*
	 * The DPCM link between each front end and the codec back end, stated explicitly.
	 * soc-pcm's dpcm_add_paths() connects a front end to a back end only when the back
	 * end's DAI widget shows up in the front end's connected-widget list
	 * (sound/soc/soc-pcm.c:1685-1730, dpcm_get_be() at :1610-1640), and on this card the
	 * two live in the same component (the MTK AFE), so the walk had no route to follow
	 * from the front end's UL1 widget to the back end's "ADDA Capture": a capture ran on
	 * UL1 alone, the codec's capture side never powered (audio21: 0x238e 0, 0x23e8
	 * 0x3000, ADC and mic bias 0), and the AFE's own generator was the only thing the
	 * memif could have carried. These two routes are that link.
	 */
	{ "UL1", NULL, "ADDA Capture" },
	{ "DL1", NULL, "ADDA Playback" },

	/* the codec's analog outputs feed the two side paths */
	{ "Headphone Jack", NULL, "Headphone L" },
	{ "Headphone Jack", NULL, "Headphone R" },
};

static int mt6785_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe = snd_soc_rtd_to_cpu(rtd, 0)->component;
	struct snd_soc_component *cmpnt_codec = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;
	int phase;
	unsigned int monitor;
	int test_done_1, test_done_2, test_done_3;
	int cycle_1, cycle_2, cycle_3;
	int prev_cycle_1, prev_cycle_2, prev_cycle_3;
	int counter;
	bool mtkaif_calib_ok = true;

	pm_runtime_get_sync(afe->dev);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 0);

	mt6359_mtkaif_calibration_enable(cmpnt_codec);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	afe_priv->mtkaif_calibration_num_phase = 42;	/* mt6359: 0 ~ 42 */
	afe_priv->mtkaif_chosen_phase[0] = -1;
	afe_priv->mtkaif_chosen_phase[1] = -1;
	afe_priv->mtkaif_chosen_phase[2] = -1;

	for (phase = 0;
	     phase <= afe_priv->mtkaif_calibration_num_phase && mtkaif_calib_ok;
	     phase++) {
		mt6359_set_mtkaif_calibration_phase(cmpnt_codec, phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done_1 = 0;
		test_done_2 = 0;
		test_done_3 = 0;
		cycle_1 = -1;
		cycle_2 = -1;
		cycle_3 = -1;
		counter = 0;
		while (test_done_1 == 0 || test_done_2 == 0 || test_done_3 == 0) {
			regmap_read(afe_priv->topckgen, CKSYS_AUD_TOP_MON, &monitor);

			test_done_1 = (monitor >> 28) & 0x1;
			test_done_2 = (monitor >> 29) & 0x1;
			test_done_3 = (monitor >> 30) & 0x1;
			if (test_done_1 == 1)
				cycle_1 = monitor & 0xf;
			if (test_done_2 == 1)
				cycle_2 = (monitor >> 4) & 0xf;
			if (test_done_3 == 1)
				cycle_3 = (monitor >> 8) & 0xf;

			/* handle if never test done */
			if (++counter > 10000) {
				dev_err(afe->dev,
					"%s(), test fail, cycle_1 %d, cycle_2 %d, cycle_3 %d, monitor 0x%x\n",
					__func__, cycle_1, cycle_2, cycle_3, monitor);
				mtkaif_calib_ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle_1 = cycle_1;
			prev_cycle_2 = cycle_2;
			prev_cycle_3 = cycle_3;
		}

		/*
		 * The numbers matter, not only the phase: a monitor that never changes means the
		 * codec's MISO is not driving, and the phase that gets chosen afterwards is then a
		 * fallback (a phase that never toggled keeps 0) rather than a measurement. Print
		 * every phase's monitor and its three cycle counts.
		 */
		dev_info(afe->dev, "mtkaif phase %d: monitor %#x, cycles %d/%d/%d (done %d%d%d)\n",
			 phase, monitor, cycle_1, cycle_2, cycle_3,
			 test_done_1, test_done_2, test_done_3);

		if (cycle_1 != prev_cycle_1 && afe_priv->mtkaif_chosen_phase[0] < 0) {
			afe_priv->mtkaif_chosen_phase[0] = phase - 1;
			afe_priv->mtkaif_phase_cycle[0] = prev_cycle_1;
		}
		if (cycle_2 != prev_cycle_2 && afe_priv->mtkaif_chosen_phase[1] < 0) {
			afe_priv->mtkaif_chosen_phase[1] = phase - 1;
			afe_priv->mtkaif_phase_cycle[1] = prev_cycle_2;
		}
		if (cycle_3 != prev_cycle_3 && afe_priv->mtkaif_chosen_phase[2] < 0) {
			afe_priv->mtkaif_chosen_phase[2] = phase - 1;
			afe_priv->mtkaif_phase_cycle[2] = prev_cycle_3;
		}

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x0);

		if (afe_priv->mtkaif_chosen_phase[0] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[1] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[2] >= 0)
			break;
	}

	dev_info(afe->dev, "mtkaif calibration result: chosen %d/%d/%d, cycles %d/%d/%d%s\n",
		 afe_priv->mtkaif_chosen_phase[0], afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2], afe_priv->mtkaif_phase_cycle[0],
		 afe_priv->mtkaif_phase_cycle[1], afe_priv->mtkaif_phase_cycle[2],
		 (afe_priv->mtkaif_chosen_phase[0] < 0 ||
		  afe_priv->mtkaif_chosen_phase[1] < 0 ||
		  afe_priv->mtkaif_chosen_phase[2] < 0) ?
		 " - a phase never toggled: the fallback is used" : " - measured");

	/* a phase that never toggled keeps the default 0, as upstream does */
	mt6359_set_mtkaif_calibration_phase(cmpnt_codec,
					    max(afe_priv->mtkaif_chosen_phase[0], 0),
					    max(afe_priv->mtkaif_chosen_phase[1], 0),
					    max(afe_priv->mtkaif_chosen_phase[2], 0));

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(cmpnt_codec);

	/*
	 * The calibration ends by switching the codec's MTKAIF transmitter off
	 * (mt6359_mtkaif_tx_disable() writes AFE_AUD_PAD_TOP = 0x3000). Nothing
	 * switches it back on for the first capture: its only other writer is
	 * the MTKAIF_TX supply's pre-power event, which runs when that widget
	 * powers up — and if DAPM already had it powered while the calibration
	 * disabled it behind DAPM's back, that event never runs again and the
	 * codec transmits nothing, which reads as exact zeros in a capture
	 * (audio10: 0x23e8 stayed 0x3000 idle and during the capture).
	 * Restore the transmitter here, the way mt8192's card leaves it after
	 * its calibration, and the register is 0x3900 from then on.
	 */
	mt6359_mtkaif_tx_enable(cmpnt_codec);

	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 0);
	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "%s(), mtkaif_chosen_phase[0/1/2]:%d/%d/%d\n",
		 __func__,
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2]);

	return 0;
}

static int mt6785_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe = snd_soc_rtd_to_cpu(rtd, 0)->component;
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;
	int ret;

	/*
	 * The AFE and the PMIC codec talk over MTKAIF protocol 2, clock phase 2.
	 * Both sides have to be told: the codec keeps the protocol in its own
	 * private data, the AFE keeps it in its own, and the AFE uses it to pick
	 * the pad-top value, the MTKAIF configuration and the receive delays. A
	 * card that only tells the codec leaves the AFE framing the stream as
	 * protocol 1 while the codec sends protocol 2, which a capture cannot
	 * parse: AFE_ADDA_MTKAIF_CFG0 stays 0 instead of 0x80010000 and
	 * AFE_AUD_PAD_TOP holds 0x31 instead of 0x38, and every recording is
	 * exact zeros with the whole path powered.
	 */
	mt6359_set_mtkaif_protocol(cmpnt_codec, MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	/*
	 * Then sweep the MTKAIF receive phase and keep the one that samples the
	 * codec's synchronisation pulse in the middle of its window: without it
	 * the AFE's receive side samples at phase 0 whatever the board's clock
	 * skew is, and a capture then records zeros while every widget on the
	 * path reads On. Both the vendor's board driver and mt8192's do this
	 * right after setting the protocol.
	 */
	mt6785_mt6359_mtkaif_calibration(rtd);

	/*
	 * The headset jack belongs to the codec's accessory detection block: the
	 * card publishes it and hands it to that block, which reports plug,
	 * unplug and the headset's buttons through it.
	 */
	if (!mt6785_mt6359_accdet)
		return 0;

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2 |
					 SND_JACK_BTN_3,
					 &mt6785_mt6359_jack,
					 mt6785_mt6359_jack_pins,
					 ARRAY_SIZE(mt6785_mt6359_jack_pins));
	if (ret)
		return ret;

	return mt6359_accdet_enable_jack_detect(mt6785_mt6359_accdet,
						&mt6785_mt6359_jack);
}

static struct snd_soc_dai_link mt6785_mt6359_dai_links[] = {
	/* front ends */
	{
		.name = "MT6785_DL1",
		.stream_name = "Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback),
	},
	{
		.name = "MT6785_UL1",
		.stream_name = "Capture",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture),
	},
	/*
	 * The voice call's front ends: the hostless speech path (powered for the
	 * length of a call, routes in mt6785-dai-hostless.c) and the ADDA
	 * loopback, which stands in for the modem when a call path is checked
	 * without one.
	 */
	{
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hostless_speech),
	},
	{
		.name = "Hostless_LPBK",
		.stream_name = "Hostless_LPBK",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE, SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		SND_SOC_DAILINK_REG(hostless_lpbk),
	},
	/* back ends */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6785_mt6359_init,
		SND_SOC_DAILINK_REG(primary_codec),
	},
	{
		.name = "Modem PCM 2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(modem_pcm2),
	},
};

static struct snd_soc_card mt6785_mt6359_card = {
	.name = CARD_NAME,
	.driver_name = DRIVER_NAME,
	.owner = THIS_MODULE,
	.dai_link = mt6785_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6785_mt6359_dai_links),
	.dapm_widgets = mt6785_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6785_mt6359_widgets),
	.dapm_routes = mt6785_mt6359_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6785_mt6359_routes),
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
	struct device_node *platform_node, *accdet_node;
	struct platform_device *accdet_pdev;
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

	/*
	 * The codec's accessory detection is an MFD child of the PMIC, so it is
	 * reached through the node the board names, not through a link.
	 */
	accdet_node = of_parse_phandle(dev->of_node, "mediatek,accdet", 0);
	if (accdet_node) {
		accdet_pdev = of_find_device_by_node(accdet_node);
		if (accdet_pdev) {
			mt6785_mt6359_accdet =
				snd_soc_lookup_component(&accdet_pdev->dev, NULL);
			put_device(&accdet_pdev->dev);
		}
		of_node_put(accdet_node);
	}

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
