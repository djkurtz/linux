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
#include "mediatek_hdmi_dpi_regs.h"
#include "mediatek_hdmi_dpi_ctrl.h"
#include "mediatek_hdmi_dpi_hw.h"

static void mtk_hdmi_mask_dpi(struct mediatek_hdmi_dpi *dpi,
			      u32 offset, u32 val, u32 mask)
{
	u32 tmp = readl(dpi->regs + offset) & ~mask;

	tmp |= (val & mask);
	writel(tmp, dpi->regs + offset);
}

void hdmi_dpi_phy_sw_reset(struct mediatek_hdmi_dpi *dpi, bool reset)
{
	mtk_hdmi_mask_dpi(dpi, DPI_RET, reset ? RST : 0, RST);
}

void hdmi_dpi_phy_enable(struct mediatek_hdmi_dpi *dpi)
{
	mtk_hdmi_mask_dpi(dpi, DPI_EN, EN, EN);
}

void hdmi_dpi_phy_disable(struct mediatek_hdmi_dpi *dpi)
{
	mtk_hdmi_mask_dpi(dpi, DPI_EN, 0, EN);
}

void hdmi_dpi_phy_config_hsync(struct mediatek_hdmi_dpi *dpi,
			       struct mediatek_hdmi_dpi_sync_param *sync)
{
	mtk_hdmi_mask_dpi(dpi, DPI_TGEN_HWIDTH,
			  sync->sync_width << HPW, HPW_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_TGEN_HPORCH,
			  sync->back_porch << HBP, HBP_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_TGEN_HPORCH, sync->front_porch << HFP,
			  HFP_MASK);
}

static void hdmi_dpi_phy_config_vsync(struct mediatek_hdmi_dpi *dpi,
				      struct mediatek_hdmi_dpi_sync_param *sync,
				    u32 width_addr, u32 porch_addr)
{
	mtk_hdmi_mask_dpi(dpi, width_addr,
			  sync->sync_width << VSYNC_WIDTH_SHIFT,
			  VSYNC_WIDTH_MASK);
	mtk_hdmi_mask_dpi(dpi, width_addr,
			  sync->shift_half_line << VSYNC_HALF_LINE_SHIFT,
			  VSYNC_HALF_LINE_MASK);
	mtk_hdmi_mask_dpi(dpi, porch_addr,
			  sync->back_porch << VSYNC_BACK_PORCH_SHIFT,
			  VSYNC_BACK_PORCH_MASK);
	mtk_hdmi_mask_dpi(dpi, porch_addr,
			  sync->front_porch << VSYNC_FRONT_PORCH_SHIFT,
			  VSYNC_FRONT_PORCH_MASK);
}

void hdmi_dpi_phy_config_vsync_lodd(struct mediatek_hdmi_dpi *dpi,
				    struct mediatek_hdmi_dpi_sync_param *sync)
{
	hdmi_dpi_phy_config_vsync(dpi, sync, DPI_TGEN_VWIDTH, DPI_TGEN_VPORCH);
}

void hdmi_dpi_phy_config_vsync_leven(struct mediatek_hdmi_dpi *dpi,
				     struct mediatek_hdmi_dpi_sync_param *sync)
{
	hdmi_dpi_phy_config_vsync(dpi, sync, DPI_TGEN_VWIDTH_LEVEN,
				  DPI_TGEN_VPORCH_LEVEN);
}

void hdmi_dpi_phy_config_vsync_rodd(struct mediatek_hdmi_dpi *dpi,
				    struct mediatek_hdmi_dpi_sync_param *sync)
{
	hdmi_dpi_phy_config_vsync(dpi, sync, DPI_TGEN_VWIDTH_RODD,
				  DPI_TGEN_VPORCH_RODD);
}

void hdmi_dpi_phy_config_vsync_reven(struct mediatek_hdmi_dpi *dpi,
				     struct mediatek_hdmi_dpi_sync_param *sync)
{
	hdmi_dpi_phy_config_vsync(dpi, sync, DPI_TGEN_VWIDTH_REVEN,
				  DPI_TGEN_VPORCH_REVEN);
}

void hdmi_dpi_phy_config_pol(struct mediatek_hdmi_dpi *dpi,
			     struct mediatek_hdmi_dpi_polarity *dpi_pol)
{
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  dpi_pol->ck_pol ==
			  HDMI_DPI_POLARITY_RISING ? 0 : CK_POL,
			  CK_POL);
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  dpi_pol->de_pol ==
			  HDMI_DPI_POLARITY_RISING ? 0 : DE_POL,
			  DE_POL);
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  dpi_pol->hsync_pol ==
			  HDMI_DPI_POLARITY_RISING ? 0 : HSYNC_POL,
			  HSYNC_POL);
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  dpi_pol->vsync_pol ==
			  HDMI_DPI_POLARITY_RISING ? 0 : VSYNC_POL,
			  VSYNC_POL);
}

void hdmi_dpi_phy_config_3d(struct mediatek_hdmi_dpi *dpi, bool en_3d)
{
	mtk_hdmi_mask_dpi(dpi, DPI_CON, en_3d ? TDFP_EN : 0, TDFP_EN);
}

void hdmi_dpi_phy_config_interface(struct mediatek_hdmi_dpi *dpi,
				   bool inter)
{
	mtk_hdmi_mask_dpi(dpi, DPI_CON, inter ? INTL_EN : 0, INTL_EN);
}

void hdmi_dpi_phy_config_fb_size(struct mediatek_hdmi_dpi *dpi,
				 u32 width, u32 height)
{
	mtk_hdmi_mask_dpi(dpi, DPI_SIZE, width << HSIZE, HSIZE_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_SIZE, height << VSIZE, VSIZE_MASK);
}

void hdmi_dpi_phy_config_channel_limit(struct mediatek_hdmi_dpi *dpi,
				       struct mediatek_hdmi_dpi_yc_limit *limit)
{
	mtk_hdmi_mask_dpi(dpi, DPI_Y_LIMIT, limit->y_bottom << Y_LIMINT_BOT,
			  Y_LIMINT_BOT_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_Y_LIMIT, limit->y_top << Y_LIMINT_TOP,
			  Y_LIMINT_TOP_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_C_LIMIT, limit->c_bottom << C_LIMIT_BOT,
			  C_LIMIT_BOT_MASK);
	mtk_hdmi_mask_dpi(dpi, DPI_C_LIMIT, limit->c_top << C_LIMIT_TOP,
			  C_LIMIT_TOP_MASK);
}

void hdmi_dpi_phy_config_bit_swap(struct mediatek_hdmi_dpi *dpi,
				  bool swap)
{
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING, swap ? BIT_SWAP : 0,
			  BIT_SWAP);
}

void hdmi_dpi_phy_config_bit_num(struct mediatek_hdmi_dpi *dpi,
				 enum HDMI_DPI_OUT_BIT_NUM num)
{
	u32 val;

	switch (num) {
	case HDMI_DPI_OUT_BIT_NUM_8BITS:
		val = OUT_BIT_8;
		break;
	case HDMI_DPI_OUT_BIT_NUM_10BITS:
		val = OUT_BIT_10;
		break;
	case HDMI_DPI_OUT_BIT_NUM_12BITS:
		val = OUT_BIT_12;
		break;
	case HDMI_DPI_OUT_BIT_NUM_16BITS:
		val = OUT_BIT_16;
		break;
	default:
		val = OUT_BIT_8;
		break;
	}
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  val << OUT_BIT,
			  OUT_BIT_MASK);
}

void hdmi_dpi_phy_config_yc_map(struct mediatek_hdmi_dpi *dpi,
				enum HDMI_DPI_OUT_YC_MAP map)
{
	u32 val;

	switch (map) {
	case HDMI_DPI_OUT_YC_MAP_RGB:
		val = YC_MAP_RGB;
		break;
	case HDMI_DPI_OUT_YC_MAP_CYCY:
		val = YC_MAP_CYCY;
		break;
	case HDMI_DPI_OUT_YC_MAP_YCYC:
		val = YC_MAP_YCYC;
		break;
	case HDMI_DPI_OUT_YC_MAP_CY:
		val = YC_MAP_CY;
		break;
	case HDMI_DPI_OUT_YC_MAP_YC:
		val = YC_MAP_YC;
		break;
	default:
		val = YC_MAP_RGB;
		break;
	}

	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING,
			  val << YC_MAP, YC_MAP_MASK);
}

void hdmi_dpi_phy_config_channel_swap(struct mediatek_hdmi_dpi *dpi,
				      enum HDMI_DPI_OUT_CHANNEL_SWAP swap)
{
	u32 val;

	switch (swap) {
	case HDMI_DPI_OUT_CHANNEL_SWAP_RGB:
		val = SWAP_RGB;
		break;
	case HDMI_DPI_OUT_CHANNEL_SWAP_GBR:
		val = SWAP_GBR;
		break;
	case HDMI_DPI_OUT_CHANNEL_SWAP_BRG:
		val = SWAP_BRG;
		break;
	case HDMI_DPI_OUT_CHANNEL_SWAP_RBG:
		val = SWAP_RBG;
		break;
	case HDMI_DPI_OUT_CHANNEL_SWAP_GRB:
		val = SWAP_GRB;
		break;
	case HDMI_DPI_OUT_CHANNEL_SWAP_BGR:
		val = SWAP_BGR;
		break;
	default:
		val = SWAP_RGB;
		break;
	}
	mtk_hdmi_mask_dpi(dpi, DPI_OUTPUT_SETTING, val << CH_SWAP,
			  CH_SWAP_MASK);
}

void hdmi_dpi_phy_config_yuv422_enable(struct mediatek_hdmi_dpi *dpi,
				       bool enable)
{
	mtk_hdmi_mask_dpi(dpi, DPI_CON, enable ? YUV422_EN : 0, YUV422_EN);
}

void hdmi_dpi_phy_config_csc_enable(struct mediatek_hdmi_dpi *dpi,
				    bool enable)
{
	mtk_hdmi_mask_dpi(dpi, DPI_CON, enable ? CSC_ENABLE : 0, CSC_ENABLE);
}

void hdmi_dpi_phy_config_swap_input(struct mediatek_hdmi_dpi *dpi,
				    bool enable)
{
	mtk_hdmi_mask_dpi(dpi, DPI_CON, enable ? IN_RB_SWAP : 0, IN_RB_SWAP);
}

void hdmi_dpi_phy_config_2n_h_fre(struct mediatek_hdmi_dpi *dpi)
{
	mtk_hdmi_mask_dpi(dpi, DPI_H_FRE_CON, H_FRE_2N, H_FRE_2N);
}

void hdmi_dpi_phy_config_color_format(struct mediatek_hdmi_dpi *dpi,
				      enum HDMI_DPI_OUT_COLOR_FORMAT format)
{
	if ((HDMI_DPI_COLOR_FORMAT_YCBCR_444 == format) ||
	    (HDMI_DPI_COLOR_FORMAT_YCBCR_444_FULL == format)) {
		hdmi_dpi_phy_config_yuv422_enable(dpi, false);
		hdmi_dpi_phy_config_csc_enable(dpi, true);
		hdmi_dpi_phy_config_swap_input(dpi, false);
		hdmi_dpi_phy_config_channel_swap(dpi,
						 HDMI_DPI_OUT_CHANNEL_SWAP_BGR);
	} else if ((HDMI_DPI_COLOR_FORMAT_YCBCR_422 == format) ||
		   (HDMI_DPI_COLOR_FORMAT_YCBCR_422_FULL == format)) {
		hdmi_dpi_phy_config_yuv422_enable(dpi, true);
		hdmi_dpi_phy_config_csc_enable(dpi, true);
		hdmi_dpi_phy_config_swap_input(dpi, true);
		hdmi_dpi_phy_config_channel_swap(dpi,
						 HDMI_DPI_OUT_CHANNEL_SWAP_RGB);
	} else {
		hdmi_dpi_phy_config_yuv422_enable(dpi, false);
		hdmi_dpi_phy_config_csc_enable(dpi, false);
		hdmi_dpi_phy_config_swap_input(dpi, false);
		hdmi_dpi_phy_config_channel_swap(dpi,
						 HDMI_DPI_OUT_CHANNEL_SWAP_RGB);
	}
}
