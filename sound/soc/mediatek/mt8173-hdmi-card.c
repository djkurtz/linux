/*
 * mt8173-hdmi-card.c  --  MT8173 HDMI ALSA SoC machine driver
 *
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Koro Chen <koro.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/mtk-hdmi.h"

static struct snd_soc_jack mt8173_hdmi_card_jack;

static int mt8173_hdmi_card_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_codec *codec = runtime->codec;
	int ret;

	/* enable jack detection */
	ret = snd_soc_card_jack_new(card, "HDMI Jack", SND_JACK_LINEOUT,
				    &mt8173_hdmi_card_jack, NULL, 0);
	if (ret) {
		dev_err(card->dev, "Can't new HDMI Jack %d\n", ret);
		return ret;
	}
	return mtk_hdmi_set_jack_detect(codec, &mt8173_hdmi_card_jack);
}

static struct snd_soc_dai_link mt8173_hdmi_card_dai[] = {
	{
		.name = "HDMI",
		.stream_name = "HDMI PCM",
		.cpu_dai_name = "HDMI",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dynamic = 1,
		.dpcm_playback = 1,
	},

	/* Back End DAI links */
	{
		.name = "HDMI BE",
		.cpu_dai_name = "HDMIO",
		.no_pcm = 1,
		.codec_name = "mtk-hdmi-codec",
		.codec_dai_name = "mtk-hdmi-hifi",
		.dpcm_playback = 1,
		.init = mt8173_hdmi_card_init,
	},
};

static struct snd_soc_card mt8173_hdmi_card_card = {
	.name = "mt8173-hdmi-card",
	.dai_link = mt8173_hdmi_card_dai,
	.num_links = ARRAY_SIZE(mt8173_hdmi_card_dai),
};

static int mt8173_hdmi_card_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8173_hdmi_card_card;
	struct device_node *platform_node;
	int i, ret;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}
	for (i = 0; i < card->num_links; i++) {
		if (mt8173_hdmi_card_dai[i].platform_name)
			continue;
		mt8173_hdmi_card_dai[i].platform_of_node = platform_node;
	}

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);
	return ret;
}

static int mt8173_hdmi_card_dev_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id mt8173_hdmi_card_dt_match[] = {
	{ .compatible = "mediatek,mt8173-hdmi-card", },
	{ }
};

static struct platform_driver mt8173_hdmi_card_driver = {
	.driver = {
		   .name = "mt8173-hdmi-card",
		   .owner = THIS_MODULE,
		   .of_match_table = mt8173_hdmi_card_dt_match,
#ifdef CONFIG_PM
		   .pm = &snd_soc_pm_ops,
#endif
	},
	.probe = mt8173_hdmi_card_dev_probe,
	.remove = mt8173_hdmi_card_dev_remove,
};

module_platform_driver(mt8173_hdmi_card_driver);

/* Module information */
MODULE_DESCRIPTION("MT8173 HDMI ALSA SoC machine driver");
MODULE_AUTHOR("Koro Chen <koro.chen@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt8173-hdmi-card");
MODULE_DEVICE_TABLE(of, mt8173_hdmi_card_dt_match);

