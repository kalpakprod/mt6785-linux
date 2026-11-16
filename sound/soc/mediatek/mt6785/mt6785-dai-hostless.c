// SPDX-License-Identifier: GPL-2.0
//
// MediaTek ALSA SoC Audio DAI Hostless Control
//
// Copyright (c) 2018 MediaTek Inc.
// Author: KaiChieh Chuang <kaichieh.chuang@mediatek.com>
//
// Voice calls, the AP half: the modem's voice PCM goes straight in and out of
// the AFE, so no buffer in memory is involved and opening this front end only
// powers the path inside the AFE.
//
// On this SoC the modem's first PCM interface (MD1) is the AFE's PCM 2, which
// the PCM DAI wiring states itself:
//
//	{"AFE_TO_MD1", NULL, "PCM 2 Playback"},
//	{"PCM 2 Capture", NULL, "MD1_TO_AFE"},
//
// so the call's audio is
//
//	downlink: modem -> PCM 2 Capture -> ADDA_DL_CHx -> MTKAIF -> mt6359 ->
//		  receiver, headphones or the speaker amplifier
//	uplink:   microphones -> ADDA -> ADDA_UL_CHx -> PCM_2_PB_CHx -> modem
//
// Everything here is a connection inside the AFE. Which output the codec uses,
// the codec's gains and the modem's own gains and mutes are userspace's
// business (the codec muxes and the CCCI speech messages), exactly as the
// user space speech daemon of this platform does it.

#include "mt6785-afe-common.h"

static const struct snd_soc_dapm_route mtk_dai_hostless_routes[] = {
	/* Hostless ADDA loopback: the codec's output back into its input path */
	{"ADDA_DL_CH1", "ADDA_UL_CH1", "Hostless LPBK DL"},
	{"ADDA_DL_CH1", "ADDA_UL_CH2", "Hostless LPBK DL"},
	{"ADDA_DL_CH2", "ADDA_UL_CH1", "Hostless LPBK DL"},
	{"ADDA_DL_CH2", "ADDA_UL_CH2", "Hostless LPBK DL"},
	{"Hostless LPBK UL", NULL, "ADDA Capture"},

	/* Hostless speech: the modem's PCM 2 and the codec */
	{"ADDA_DL_CH1", "PCM_2_CAP_CH1", "Hostless Speech DL"},
	{"ADDA_DL_CH2", "PCM_2_CAP_CH1", "Hostless Speech DL"},
	{"ADDA_DL_CH2", "PCM_2_CAP_CH2", "Hostless Speech DL"},
	{"PCM_2_PB_CH1", "ADDA_UL_CH1", "Hostless Speech DL"},
	{"PCM_2_PB_CH2", "ADDA_UL_CH2", "Hostless Speech DL"},

	{"Hostless Speech UL", NULL, "PCM 2 Capture"},
	{"Hostless Speech UL", NULL, "ADDA Capture"},
};

static int mtk_dai_hostless_startup(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	return snd_soc_set_runtime_hwparams(substream, afe->mtk_afe_hardware);
}

static const struct snd_soc_dai_ops mtk_dai_hostless_ops = {
	.startup = mtk_dai_hostless_startup,
};

#define MTK_HOSTLESS_RATES (SNDRV_PCM_RATE_8000_48000 |\
			    SNDRV_PCM_RATE_88200 |\
			    SNDRV_PCM_RATE_96000 |\
			    SNDRV_PCM_RATE_176400 |\
			    SNDRV_PCM_RATE_192000)

#define MTK_HOSTLESS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			      SNDRV_PCM_FMTBIT_S24_LE |\
			      SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_hostless_driver[] = {
	{
		.name = "Hostless LPBK DAI",
		.id = MT6785_DAI_HOSTLESS_LPBK,
		.playback = {
			.stream_name = "Hostless LPBK DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.capture = {
			.stream_name = "Hostless LPBK UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.ops = &mtk_dai_hostless_ops,
	},
	{
		.name = "Hostless Speech DAI",
		.id = MT6785_DAI_HOSTLESS_SPEECH,
		.playback = {
			.stream_name = "Hostless Speech DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.capture = {
			.stream_name = "Hostless Speech UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.ops = &mtk_dai_hostless_ops,
	},
};

int mt6785_dai_hostless_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_hostless_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_hostless_driver);

	dai->dapm_routes = mtk_dai_hostless_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_hostless_routes);

	return 0;
}
