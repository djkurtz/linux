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

#ifndef MEDIATEK_DRM_DRV_H
#define MEDIATEK_DRM_DRV_H

#define MAX_CRTC	2
#define MAX_CONNECTOR	2

struct mtk_drm_private {
	struct drm_fb_helper *fb_helper;

	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct drm_crtc *crtc[MAX_CRTC];
	struct drm_property *plane_zpos_property;
	unsigned int pipe;

	struct device *larb_dev[MAX_CRTC];
};

#endif /* MEDIATEK_DRM_DRV_H */
