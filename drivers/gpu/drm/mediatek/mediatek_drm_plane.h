/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: CK Hu <ck.hu@mediatek.com>
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

#ifndef _MEDIATEK_DRM_PLANE_H_
#define _MEDIATEK_DRM_PLANE_H_

#include "mediatek_drm_crtc.h"

#define to_mtk_plane(x)	container_of(x, struct mtk_drm_plane, base)

struct mtk_drm_plane {
	struct drm_plane		base;
	struct mtk_drm_crtc		*mtk_crtc;
	unsigned int			idx;
	unsigned int			disp_size;

	struct mtk_drm_gem_obj		*flip_obj;
	unsigned			fence_context;
	atomic_t			fence_seqno;
	struct drm_reservation_cb	rcb;
	struct fence			*fence;
	struct fence			*pending_fence;
};

void mtk_plane_finish_page_flip(struct mtk_drm_plane *mtk_plane);
int mtk_plane_init(struct drm_device *dev, struct mtk_drm_plane *mtk_plane,
		   unsigned long possible_crtcs, enum drm_plane_type type,
		   unsigned int zpos, unsigned int max_plane);

#endif
