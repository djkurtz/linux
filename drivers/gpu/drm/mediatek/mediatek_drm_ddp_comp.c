/*
 * Copyright (c) 2015 MediaTek Inc.
 * Authors:
 *	YT Shen <yt.shen@mediatek.com>
 *	CK Hu <ck.hu@mediatek.com>
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
#include "mediatek_drm_ddp_comp.h"


#define DISP_REG_OVL_INTEN			0x0004
#define DISP_REG_OVL_INTSTA			0x0008
#define DISP_REG_OVL_EN				0x000c
#define DISP_REG_OVL_RST			0x0014
#define DISP_REG_OVL_ROI_SIZE			0x0020
#define DISP_REG_OVL_ROI_BGCLR			0x0028
#define DISP_REG_OVL_SRC_CON			0x002c
#define DISP_REG_OVL_CON(n)			(0x0030 + 0x20 * n)
#define DISP_REG_OVL_SRC_SIZE(n)		(0x0038 + 0x20 * n)
#define DISP_REG_OVL_OFFSET(n)			(0x003c + 0x20 * n)
#define DISP_REG_OVL_PITCH(n)			(0x0044 + 0x20 * n)
#define DISP_REG_OVL_RDMA_CTRL(n)		(0x00c0 + 0x20 * n)
#define DISP_REG_OVL_RDMA_GMC(n)		(0x00c8 + 0x20 * n)
#define DISP_REG_OVL_ADDR(n)			(0x0f40 + 0x20 * n)

#define DISP_REG_RDMA_INT_ENABLE		0x0000
#define DISP_REG_RDMA_INT_STATUS		0x0004
#define DISP_REG_RDMA_GLOBAL_CON		0x0010
#define DISP_REG_RDMA_SIZE_CON_0		0x0014
#define DISP_REG_RDMA_SIZE_CON_1		0x0018
#define DISP_REG_RDMA_FIFO_CON			0x0040

#define DISP_OD_EN				0x0000
#define DISP_OD_INTEN				0x0008
#define DISP_OD_INTS				0x000c
#define DISP_OD_CFG				0x0020
#define DISP_OD_SIZE				0x0030

#define DISP_REG_UFO_START			0x0000

#define DISP_COLOR_CFG_MAIN			0x0400
#define DISP_COLOR_START			0x0c00

enum OVL_INPUT_FORMAT {
	OVL_INFMT_RGB565 = 0,
	OVL_INFMT_RGB888 = 1,
	OVL_INFMT_RGBA8888 = 2,
	OVL_INFMT_ARGB8888 = 3,
};

#define	OVL_RDMA_MEM_GMC	0x40402020
#define	OVL_AEN			BIT(8)
#define	OVL_ALPHA		0xff

#define	OD_RELAY_MODE		BIT(0)

#define	UFO_BYPASS		BIT(2)

#define	COLOR_BYPASS_ALL	BIT(7)
#define	COLOR_SEQ_SEL		BIT(13)

static void mediatek_color_start(void __iomem *color_base)
{
	writel(COLOR_BYPASS_ALL | COLOR_SEQ_SEL,
		color_base + DISP_COLOR_CFG_MAIN);
	writel(0x1, color_base + DISP_COLOR_START);
}

static void mediatek_od_config(void __iomem *od_base,
		unsigned int w, unsigned int h)
{
	writel(w << 16 | h, od_base + DISP_OD_SIZE);
}

static void mediatek_od_start(void __iomem *od_base)
{
	writel(OD_RELAY_MODE, od_base + DISP_OD_CFG);
	writel(1, od_base + DISP_OD_EN);
}

static void mediatek_ovl_enable_vblank(void __iomem *disp_base)
{
	writel(0x2, disp_base + DISP_REG_OVL_INTEN);
}

static void mediatek_ovl_disable_vblank(void __iomem *disp_base)
{
	writel(0x0, disp_base + DISP_REG_OVL_INTEN);
}

static void mediatek_ovl_clear_vblank(void __iomem *disp_base)
{
	writel(0x0, disp_base + DISP_REG_OVL_INTSTA);
}

static void mediatek_ovl_start(void __iomem *ovl_base)
{
	writel(0x1, ovl_base + DISP_REG_OVL_EN);
}

static void mediatek_ovl_config(void __iomem *ovl_base,
		unsigned int w, unsigned int h)
{
	if (of_find_compatible_node(NULL, NULL, "ite,it6151") != NULL)
		w = ((w + 3) >> 2) << 2;

	if (w != 0 && h != 0)
		writel(h << 16 | w, ovl_base + DISP_REG_OVL_ROI_SIZE);
	writel(0x0, ovl_base + DISP_REG_OVL_ROI_BGCLR);

	writel(0x1, ovl_base + DISP_REG_OVL_RST);
	writel(0x0, ovl_base + DISP_REG_OVL_RST);
}

static bool has_rb_swapped(unsigned int fmt)
{
	switch (fmt) {
	case DRM_FORMAT_BGR888:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
		return true;
	default:
		return false;
	}
}

static unsigned int ovl_fmt_convert(unsigned int fmt)
{
	switch (fmt) {
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
		return OVL_INFMT_RGB888;
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
		return OVL_INFMT_RGB565;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
		return OVL_INFMT_ARGB8888;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return OVL_INFMT_RGBA8888;
	default:
		DRM_ERROR("unsupport format[%08x]\n", fmt);
		return -EINVAL;
	}
}

static void mediatek_ovl_layer_on(void __iomem *ovl_base, unsigned int idx)
{
	unsigned int reg;

	writel(0x1, ovl_base + DISP_REG_OVL_RDMA_CTRL(idx));
	writel(OVL_RDMA_MEM_GMC, ovl_base + DISP_REG_OVL_RDMA_GMC(idx));

	reg = readl(ovl_base + DISP_REG_OVL_SRC_CON);
	reg = reg | (1 << idx);
	writel(reg, ovl_base + DISP_REG_OVL_SRC_CON);
}

static void mediatek_ovl_layer_off(void __iomem *ovl_base, unsigned int idx)
{
	unsigned int reg;

	reg = readl(ovl_base + DISP_REG_OVL_SRC_CON);
	reg = reg & ~(1 << idx);
	writel(reg, ovl_base + DISP_REG_OVL_SRC_CON);

	writel(0x0, ovl_base + DISP_REG_OVL_RDMA_CTRL(idx));
}

static void mediatek_ovl_layer_config(void __iomem *ovl_base, unsigned int idx,
		unsigned int addr, unsigned int pitch, unsigned int fmt,
		int x, int y, unsigned int size)
{
	unsigned int reg;

	reg = has_rb_swapped(fmt) << 24 | ovl_fmt_convert(fmt) << 12;
	if (idx != 0)
		reg |= OVL_AEN | OVL_ALPHA;

	writel(reg, ovl_base + DISP_REG_OVL_CON(idx));
	writel(pitch & 0xFFFF, ovl_base + DISP_REG_OVL_PITCH(idx));
	writel(size, ovl_base + DISP_REG_OVL_SRC_SIZE(idx));
	writel(y << 16 | x, ovl_base + DISP_REG_OVL_OFFSET(idx));
	writel(addr, ovl_base + DISP_REG_OVL_ADDR(idx));
}

static void mediatek_rdma_start(void __iomem *rdma_base)
{
	unsigned int reg;

	writel(0x4, rdma_base + DISP_REG_RDMA_INT_ENABLE);
	reg = readl(rdma_base + DISP_REG_RDMA_GLOBAL_CON);
	reg |= 1;
	writel(reg, rdma_base + DISP_REG_RDMA_GLOBAL_CON);
}

static void mediatek_rdma_config(void __iomem *rdma_base,
		unsigned width, unsigned height)
{
	unsigned int reg;

	reg = readl(rdma_base + DISP_REG_RDMA_SIZE_CON_0);
	reg = (reg & ~(0xFFF)) | (width & 0xFFF);
	writel(reg, rdma_base + DISP_REG_RDMA_SIZE_CON_0);

	reg = readl(rdma_base + DISP_REG_RDMA_SIZE_CON_1);
	reg = (reg & ~(0xFFFFF)) | (height & 0xFFFFF);
	writel(reg, rdma_base + DISP_REG_RDMA_SIZE_CON_1);

	writel(0x80F00008, rdma_base + DISP_REG_RDMA_FIFO_CON);
}

static void mediatek_ufoe_start(void __iomem *ufoe_base)
{
	writel(UFO_BYPASS, ufoe_base + DISP_REG_UFO_START);
}

static const struct mtk_ddp_comp_funcs ddp_aal = {
	.comp_type = DDP_COMPONENT_AAL,
};

static const struct mtk_ddp_comp_funcs ddp_color0 = {
	.comp_type = DDP_COMPONENT_COLOR0,
	.comp_power_on = mediatek_color_start,
};

static const struct mtk_ddp_comp_funcs ddp_color1 = {
	.comp_type = DDP_COMPONENT_COLOR1,
	.comp_power_on = mediatek_color_start,
};

static const struct mtk_ddp_comp_funcs ddp_dpi0 = {
	.comp_type = DDP_COMPONENT_DPI0,
};

static const struct mtk_ddp_comp_funcs ddp_dsi0 = {
	.comp_type = DDP_COMPONENT_DSI0,
};

static const struct mtk_ddp_comp_funcs ddp_gamma = {
	.comp_type = DDP_COMPONENT_GAMMA,
};

static const struct mtk_ddp_comp_funcs ddp_od = {
	.comp_type = DDP_COMPONENT_OD,
	.comp_config = mediatek_od_config,
	.comp_power_on = mediatek_od_start,
};

static const struct mtk_ddp_comp_funcs ddp_ovl0 = {
	.comp_type = DDP_COMPONENT_OVL0,
	.comp_config = mediatek_ovl_config,
	.comp_power_on = mediatek_ovl_start,
	.comp_enable_vblank = mediatek_ovl_enable_vblank,
	.comp_disable_vblank = mediatek_ovl_disable_vblank,
	.comp_clear_vblank = mediatek_ovl_clear_vblank,
	.comp_layer_on = mediatek_ovl_layer_on,
	.comp_layer_off = mediatek_ovl_layer_off,
	.comp_layer_config = mediatek_ovl_layer_config,
};

static const struct mtk_ddp_comp_funcs ddp_ovl1 = {
	.comp_type = DDP_COMPONENT_OVL1,
	.comp_config = mediatek_ovl_config,
	.comp_power_on = mediatek_ovl_start,
	.comp_enable_vblank = mediatek_ovl_enable_vblank,
	.comp_disable_vblank = mediatek_ovl_disable_vblank,
	.comp_clear_vblank = mediatek_ovl_clear_vblank,
	.comp_layer_on = mediatek_ovl_layer_on,
	.comp_layer_off = mediatek_ovl_layer_off,
	.comp_layer_config = mediatek_ovl_layer_config,
};

static const struct mtk_ddp_comp_funcs ddp_pwm0 = {
	.comp_type = DDP_COMPONENT_PWM0,
};

static const struct mtk_ddp_comp_funcs ddp_rdma0 = {
	.comp_type = DDP_COMPONENT_RDMA0,
	.comp_config = mediatek_rdma_config,
	.comp_power_on = mediatek_rdma_start,
};

static const struct mtk_ddp_comp_funcs ddp_rdma1 = {
	.comp_type = DDP_COMPONENT_RDMA1,
	.comp_config = mediatek_rdma_config,
	.comp_power_on = mediatek_rdma_start,
};

static const struct mtk_ddp_comp_funcs ddp_ufoe = {
	.comp_type = DDP_COMPONENT_UFOE,
	.comp_power_on = mediatek_ufoe_start,
};

const struct mtk_ddp_comp_funcs *mtk_ddp_get_comp(const char *comp_name)
{
	if (!strcmp(comp_name, "AAL"))
		return &ddp_aal;

	if (!strcmp(comp_name, "COLOR0"))
		return &ddp_color0;

	if (!strcmp(comp_name, "COLOR1"))
		return &ddp_color1;

	if (!strcmp(comp_name, "DPI0"))
		return &ddp_dpi0;

	if (!strcmp(comp_name, "DSI0"))
		return &ddp_dsi0;

	if (!strcmp(comp_name, "GAMMA"))
		return &ddp_gamma;

	if (!strcmp(comp_name, "OD"))
		return &ddp_od;

	if (!strcmp(comp_name, "OVL0"))
		return &ddp_ovl0;

	if (!strcmp(comp_name, "OVL1"))
		return &ddp_ovl1;

	if (!strcmp(comp_name, "PWM0"))
		return &ddp_pwm0;

	if (!strcmp(comp_name, "RDMA0"))
		return &ddp_rdma0;

	if (!strcmp(comp_name, "RDMA1"))
		return &ddp_rdma1;

	if (!strcmp(comp_name, "UFOE"))
		return &ddp_ufoe;

	return NULL;
}
