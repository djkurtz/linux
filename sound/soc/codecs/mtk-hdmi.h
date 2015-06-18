/*
 * mtk-hdmi.h  --  MTK HDMI ASoC codec driver
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

#ifndef __MTK_HDMI_H__
#define __MTK_HDMI_H__

#include <drm/mediatek/mediatek_hdmi_audio.h>

struct mtk_hdmi_priv {
	struct device *dev;
	struct mediatek_hdmi_audio_data data;
	u8 jack_status;
	struct snd_soc_jack *jack;
};

int mtk_hdmi_set_jack_detect(struct snd_soc_codec *codec,
			     struct snd_soc_jack *jack);
#endif /* __MTK_HDMI_H__ */

