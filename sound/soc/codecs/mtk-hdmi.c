/*
 * mtk-hdmi.c  --  MTK HDMI ASoC codec driver
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
#include "mtk-hdmi.h"

static int mtk_hdmi_jack_detect(struct mtk_hdmi_priv *hdmi)
{
	u8 jack_status;

	if (!hdmi->jack)
		return 0;

	jack_status = hdmi->data.hpd_detect(hdmi->data.mediatek_hdmi) ?
			SND_JACK_LINEOUT : 0;
	if (jack_status != hdmi->jack_status) {
		snd_soc_jack_report(hdmi->jack, jack_status, SND_JACK_LINEOUT);
		dev_info(hdmi->dev, "jack report [%d->%d]\n",
			 hdmi->jack_status, jack_status);
		hdmi->jack_status = jack_status;
	}
	return 0;
}

static irqreturn_t mtk_hdmi_irq(int irq, void *dev_id)
{
	struct mtk_hdmi_priv *hdmi = dev_id;

	mtk_hdmi_jack_detect(hdmi);
	return IRQ_HANDLED;
}

int mtk_hdmi_set_jack_detect(struct snd_soc_codec *codec,
			     struct snd_soc_jack *jack)
{
	struct mtk_hdmi_priv *hdmi = snd_soc_codec_get_drvdata(codec);

	hdmi->jack = jack;
	mtk_hdmi_jack_detect(hdmi);
	return 0;
}

static int mtk_hdmi_dai_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *codec_dai)
{
	struct mtk_hdmi_priv *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	hdmi->data.enable(hdmi->data.mediatek_hdmi);
	return 0;
}

static int mtk_hdmi_dai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *codec_dai)
{
	struct mtk_hdmi_priv *hdmi = snd_soc_dai_get_drvdata(codec_dai);
	struct hdmi_audio_param hdmi_params;
	unsigned int rate, chan;

	chan = params_channels(params);
	switch (chan) {
	case 2:
		hdmi_params.aud_input_chan_type = HDMI_AUD_CHAN_TYPE_2_0;
		break;
	case 4:
		hdmi_params.aud_input_chan_type = HDMI_AUD_CHAN_TYPE_4_0;
		break;
	case 6:
		hdmi_params.aud_input_chan_type = HDMI_AUD_CHAN_TYPE_5_1;
		break;
	case 8:
		hdmi_params.aud_input_chan_type = HDMI_AUD_CHAN_TYPE_7_1;
		break;
	default:
		dev_err(codec_dai->dev, "channel[%d] not supported!\n", chan);
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: chan_num = %d.\n", chan);

	rate = params_rate(params);
	switch (rate) {
	case 32000:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_32000;
		hdmi_params.iec_frame_fs = HDMI_IEC_32K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0x3;
		break;
	case 44100:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_44100;
		hdmi_params.iec_frame_fs = HDMI_IEC_44K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0;
		break;
	case 48000:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_48000;
		hdmi_params.iec_frame_fs = HDMI_IEC_48K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0x2;
		break;
	case 88200:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_88200;
		hdmi_params.iec_frame_fs = HDMI_IEC_88K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0x8;
		break;
	case 96000:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_96000;
		hdmi_params.iec_frame_fs = HDMI_IEC_96K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0xa;
		break;
	case 176400:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_176400;
		hdmi_params.iec_frame_fs = HDMI_IEC_176K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0xc;
		break;
	case 192000:
		hdmi_params.aud_hdmi_fs = HDMI_AUDIO_SAMPLE_FREQUENCY_192000;
		hdmi_params.iec_frame_fs = HDMI_IEC_192K;
		/* channel status byte 3: fs and clock accuracy */
		hdmi_params.hdmi_l_channel_state[3] = 0xe;
		break;
	default:
		dev_err(codec_dai->dev, "rate[%d] not supported!\n", rate);
		return -EINVAL;
	}
	dev_dbg(codec_dai->dev, "[codec_dai]: sample_rate = %d.\n", rate);

	hdmi_params.aud_codec = HDMI_AUDIO_CODING_TYPE_PCM;
	hdmi_params.aud_sampe_size = HDMI_AUDIO_SAMPLE_SIZE_16;
	hdmi_params.aud_input_type = HDMI_AUD_INPUT_I2S;
	hdmi_params.aud_i2s_fmt = HDMI_I2S_MODE_I2S_24BIT;
	hdmi_params.aud_mclk = HDMI_AUD_MCLK_128FS;

	/* channel status */
	/* byte 0: no copyright is asserted, mode 0 */
	hdmi_params.hdmi_l_channel_state[0] = 1 << 2;
	/* byte 1: category code */
	hdmi_params.hdmi_l_channel_state[1] = 0;
	/* byte 2: source/channel number don't take into account */
	hdmi_params.hdmi_l_channel_state[2] = 0;
	/* byte 4: word length 16bits */
	hdmi_params.hdmi_l_channel_state[4] = 0x2;
	memcpy(hdmi_params.hdmi_r_channel_state,
	       hdmi_params.hdmi_l_channel_state,
	       sizeof(hdmi_params.hdmi_l_channel_state));

	return hdmi->data.set_audio_param(hdmi->data.mediatek_hdmi, &hdmi_params);
}

static int mtk_hdmi_dai_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *codec_dai)
{
	struct mtk_hdmi_priv *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		hdmi->data.enable(hdmi->data.mediatek_hdmi);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hdmi->data.disable(hdmi->data.mediatek_hdmi);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void mtk_hdmi_dai_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *codec_dai)
{
	struct mtk_hdmi_priv *hdmi = snd_soc_dai_get_drvdata(codec_dai);

	hdmi->data.disable(hdmi->data.mediatek_hdmi);
}

static const struct snd_soc_dapm_widget mtk_hdmi_widgets[] = {
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route mtk_hdmi_routes[] = {
	{ "TX", NULL, "TX Playback" },
};

static const struct snd_soc_dai_ops mtk_hdmi_dai_ops = {
	.startup = mtk_hdmi_dai_startup,
	.hw_params = mtk_hdmi_dai_hw_params,
	.trigger = mtk_hdmi_dai_trigger,
	.shutdown = mtk_hdmi_dai_shutdown,
};

static struct snd_soc_dai_driver mtk_hdmi_dai = {
	.name = "mtk-hdmi-hifi",
	.playback = {
		.stream_name = "TX Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &mtk_hdmi_dai_ops,
};

static const struct snd_soc_codec_driver mtk_hdmi_codec = {
	.dapm_widgets = mtk_hdmi_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mtk_hdmi_widgets),
	.dapm_routes = mtk_hdmi_routes,
	.num_dapm_routes = ARRAY_SIZE(mtk_hdmi_routes),
};

static int mtk_hdmi_probe(struct platform_device *pdev)
{
	struct mediatek_hdmi_audio_data *data = pdev->dev.platform_data;
	struct mtk_hdmi_priv *hdmi;
	int ret;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->data = *data;
	hdmi->dev = &pdev->dev;
	platform_set_drvdata(pdev, hdmi);

	ret = devm_request_threaded_irq(&pdev->dev, hdmi->data.irq,
					NULL, mtk_hdmi_irq,
					IRQF_SHARED | IRQF_TRIGGER_LOW |
					IRQF_ONESHOT,
					"mtk-hdmi-hotplug", hdmi);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed (%d)\n", ret);
		return ret;
	}
	ret = snd_soc_register_codec(&pdev->dev, &mtk_hdmi_codec,
				     &mtk_hdmi_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register codec failed (%d)\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "hdmi audio init success.\n");

	return 0;
}

static int mtk_hdmi_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver mtk_hdmi_driver = {
	.driver = {
		.name = "mtk-hdmi-codec",
	},
	.probe = mtk_hdmi_probe,
	.remove = mtk_hdmi_remove,
};
module_platform_driver(mtk_hdmi_driver);

/* Module information */
MODULE_DESCRIPTION("MTK HDMI codec driver");
MODULE_AUTHOR("Koro Chen <koro.chen@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" "mtk-hdmi-codec");
