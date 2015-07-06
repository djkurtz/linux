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


#ifndef MEDIATEL_DRM_CRTC_H
#define MEDIATEL_DRM_CRTC_H

#include <drm/drm_sync_helper.h>

/*
 * MediaTek specific crtc structure.
 *
 * @base: crtc object.
 * @pipe: a crtc index created at load() with a new crtc object creation
 *	and the crtc object would be set to private->crtc array
 *	to get a crtc object corresponding to this pipe from private->crtc
 *	array when irq interrupt occurred. the reason of using this pipe is that
 *	drm framework doesn't support multiple irq yet.
 *	we can refer to the crtc to current hardware interrupt occurred through
 *	this pipe value.
 */
struct mtk_drm_crtc {
	struct drm_crtc				base;
	unsigned int				pipe;
	bool					enabled;
	void					*ctx;

	struct drm_pending_vblank_event		*event;
	unsigned int				pending_update;
	bool					do_flush;
	bool					pending_needs_vblank;
};

#define to_mtk_crtc(x) container_of(x, struct mtk_drm_crtc, base)

int mtk_drm_crtc_enable_vblank(struct drm_device *drm, int pipe);
void mtk_drm_crtc_disable_vblank(struct drm_device *drm, int pipe);
void mtk_drm_crtc_check_flush(struct mtk_drm_crtc *mtk_crtc);
void mtk_drm_crtc_plane_config(struct mtk_drm_crtc *crtc, unsigned int idx,
				  bool enable, dma_addr_t addr);
void mtk_drm_crtc_commit(struct mtk_drm_crtc *crtc);

#endif /* MEDIATEL_DRM_CRTC_H */
