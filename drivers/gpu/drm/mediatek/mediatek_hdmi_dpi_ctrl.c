/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Jie Qiu <jie.qiu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <drm/drmP.h>
#include "mediatek_hdmi_dpi_ctrl.h"
#include "mediatek_hdmi_dpi_hw.h"

int mtk_hdmi_dpi_power_off(struct mediatek_hdmi_dpi *dpi)
{
	clk_disable(dpi->tvd_clk);
	hdmi_dpi_phy_disable(dpi);

	return 0;
}

int mtk_hdmi_dpi_power_on(struct mediatek_hdmi_dpi *dpi)
{
	int ret;

	ret = clk_prepare_enable(dpi->tvd_clk);
	if (ret) {
		mtk_hdmi_err("enable tvdpll failed!\n");
		return ret;
	}
	hdmi_dpi_phy_enable(dpi);

	return 0;
}

int mtk_hdmi_dpi_init(void *private_data)
{
	struct mediatek_hdmi_dpi *dpi = private_data;

	dpi->bit_num = HDMI_DPI_OUT_BIT_NUM_8BITS;
	dpi->channel_swap = HDMI_DPI_OUT_CHANNEL_SWAP_RGB;
	dpi->yc_map = HDMI_DPI_OUT_YC_MAP_RGB;
	dpi->color_format = HDMI_DPI_COLOR_FORMAT_RGB;

	return mtk_hdmi_dpi_power_on(dpi);
}

int mtk_hdmi_dpi_set_display_mode(struct drm_display_mode *display_mode,
				  void *private_data)
{
	struct mediatek_hdmi_dpi *dpi = private_data;
	struct mediatek_hdmi_dpi_yc_limit limit;
	struct mediatek_hdmi_dpi_polarity dpi_pol;
	struct mediatek_hdmi_dpi_sync_param hsync;
	struct mediatek_hdmi_dpi_sync_param vsync_lodd = { 0 };
	struct mediatek_hdmi_dpi_sync_param vsync_leven = { 0 };
	struct mediatek_hdmi_dpi_sync_param vsync_rodd = { 0 };
	struct mediatek_hdmi_dpi_sync_param vsync_reven = { 0 };

	if (!dpi) {
		mtk_hdmi_err("invalid argument\n");
		return -EINVAL;
	}

	limit.c_bottom = 0x0010;
	limit.c_top = 0x0FE0;
	limit.y_bottom = 0x0010;
	limit.y_top = 0x0FE0;

	dpi_pol.ck_pol = HDMI_DPI_POLARITY_FALLING;
	dpi_pol.de_pol = HDMI_DPI_POLARITY_RISING;
	dpi_pol.hsync_pol =
	    display_mode->
	    flags & DRM_MODE_FLAG_PHSYNC ? HDMI_DPI_POLARITY_FALLING :
	    HDMI_DPI_POLARITY_RISING;
	dpi_pol.vsync_pol =
	    display_mode->
	    flags & DRM_MODE_FLAG_PVSYNC ? HDMI_DPI_POLARITY_FALLING :
	    HDMI_DPI_POLARITY_RISING;

	hsync.sync_width = display_mode->hsync_end - display_mode->hsync_start;
	hsync.back_porch = display_mode->htotal - display_mode->hsync_end;
	hsync.front_porch = display_mode->hsync_start - display_mode->hdisplay;
	hsync.shift_half_line = false;

	vsync_lodd.sync_width =
	    display_mode->vsync_end - display_mode->vsync_start;
	vsync_lodd.back_porch = display_mode->vtotal - display_mode->vsync_end;
	vsync_lodd.front_porch =
	    display_mode->vsync_start - display_mode->vdisplay;
	vsync_lodd.shift_half_line = false;

	if (display_mode->flags & DRM_MODE_FLAG_INTERLACE &&
	    display_mode->flags & DRM_MODE_FLAG_3D_MASK) {
		vsync_leven = vsync_lodd;
		vsync_rodd = vsync_lodd;
		vsync_reven = vsync_lodd;
		vsync_leven.shift_half_line = true;
		vsync_reven.shift_half_line = true;
	} else if (display_mode->flags & DRM_MODE_FLAG_INTERLACE &&
		   !(display_mode->flags & DRM_MODE_FLAG_3D_MASK)) {
		vsync_leven = vsync_lodd;
		vsync_leven.shift_half_line = true;
	} else if (!(display_mode->flags & DRM_MODE_FLAG_INTERLACE) &&
		   display_mode->flags & DRM_MODE_FLAG_3D_MASK) {
		vsync_rodd = vsync_lodd;
	}
	hdmi_dpi_phy_sw_reset(dpi, true);
	hdmi_dpi_phy_config_pol(dpi, &dpi_pol);

	hdmi_dpi_phy_config_hsync(dpi, &hsync);
	hdmi_dpi_phy_config_vsync_lodd(dpi, &vsync_lodd);
	hdmi_dpi_phy_config_vsync_rodd(dpi, &vsync_rodd);
	hdmi_dpi_phy_config_vsync_leven(dpi, &vsync_leven);
	hdmi_dpi_phy_config_vsync_reven(dpi, &vsync_reven);

	hdmi_dpi_phy_config_3d(dpi,
			       !!(display_mode->flags & DRM_MODE_FLAG_3D_MASK));
	hdmi_dpi_phy_config_interface(dpi,
				      !!(display_mode->flags & DRM_MODE_FLAG_INTERLACE));
	if (display_mode->flags & DRM_MODE_FLAG_INTERLACE) {
		hdmi_dpi_phy_config_fb_size(dpi, display_mode->hdisplay,
					    display_mode->vdisplay / 2);
	} else {
		hdmi_dpi_phy_config_fb_size(dpi, display_mode->hdisplay,
					    display_mode->vdisplay);
	}

	hdmi_dpi_phy_config_channel_limit(dpi, &limit);
	hdmi_dpi_phy_config_bit_num(dpi, dpi->bit_num);
	hdmi_dpi_phy_config_channel_swap(dpi, dpi->channel_swap);
	hdmi_dpi_phy_config_yc_map(dpi, dpi->yc_map);
	hdmi_dpi_phy_config_color_format(dpi, dpi->color_format);
	hdmi_dpi_phy_config_2n_h_fre(dpi);
	hdmi_dpi_phy_sw_reset(dpi, false);

	memcpy(&dpi->display_mode, display_mode,
	       sizeof(dpi->display_mode));

	mtk_hdmi_info("set display mode success!\n");
	return 0;
}
