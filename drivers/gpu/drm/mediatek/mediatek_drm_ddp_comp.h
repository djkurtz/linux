/*
 * Copyright (c) 2015 MediaTek Inc.
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

#ifndef MEDIATEK_DRM_DDP_COMP_H
#define MEDIATEK_DRM_DDP_COMP_H

#define OVL_LAYER_NR	4


enum {
	DDP_COMPONENT_AAL,
	DDP_COMPONENT_COLOR0,
	DDP_COMPONENT_COLOR1,
	DDP_COMPONENT_DPI0,
	DDP_COMPONENT_DSI0,
	DDP_COMPONENT_GAMMA,
	DDP_COMPONENT_OD,
	DDP_COMPONENT_OVL0,
	DDP_COMPONENT_OVL1,
	DDP_COMPONENT_PWM0,
	DDP_COMPONENT_RDMA0,
	DDP_COMPONENT_RDMA1,
	DDP_COMPONENT_UFOE,
	DDP_COMPONENT_TYPE_MAX,
};

struct mtk_ddp_comp_funcs {
	unsigned int	comp_type;
	void (*comp_config)(void __iomem *ovl_base,
			unsigned int w, unsigned int h);
	void (*comp_power_on)(void __iomem *ovl_base);
	void (*comp_power_off)(void __iomem *ovl_base);
	void (*comp_enable_vblank)(void __iomem *ovl_base);
	void (*comp_disable_vblank)(void __iomem *ovl_base);
	void (*comp_clear_vblank)(void __iomem *ovl_base);
	void (*comp_layer_on)(void __iomem *ovl_base, unsigned int idx);
	void (*comp_layer_off)(void __iomem *ovl_base, unsigned int idx);
	void (*comp_layer_config)(void __iomem *ovl_base, unsigned int idx,
			unsigned int addr, unsigned int pitch, unsigned int fmt,
			int x, int y, unsigned int size);
};

const struct mtk_ddp_comp_funcs *mtk_ddp_get_comp(const char *comp_name);

#endif /* MEDIATEK_DRM_DDP_COMP_H */
