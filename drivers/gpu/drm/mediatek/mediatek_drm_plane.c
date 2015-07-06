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

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <linux/dma-buf.h>
#include <linux/reservation.h>

#include "mediatek_drm_crtc.h"
#include "mediatek_drm_ddp_comp.h"
#include "mediatek_drm_drv.h"
#include "mediatek_drm_fb.h"
#include "mediatek_drm_gem.h"
#include "mediatek_drm_plane.h"

static const uint32_t formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
};

void mtk_plane_finish_page_flip(struct mtk_drm_plane *mtk_plane)
{
	if (mtk_plane->fence)
		drm_fence_signal_and_put(&mtk_plane->fence);
	mtk_plane->fence = mtk_plane->pending_fence;
	mtk_plane->pending_fence = NULL;
}

static void mtk_plane_update_cb(struct drm_reservation_cb *rcb, void *params)
{
	struct mtk_drm_plane *mtk_plane = params;
	struct mtk_drm_crtc *mtk_crtc = mtk_plane->mtk_crtc;
	struct drm_device *dev = mtk_crtc->base.dev;
	unsigned long flags;

	if (mtk_plane->flip_obj)
		mtk_drm_crtc_plane_config(mtk_crtc, mtk_plane->idx, true,
					    mtk_plane->flip_obj->dma_addr);

	mtk_crtc->pending_update &= ~(1 << mtk_plane->idx);
	mtk_drm_crtc_check_flush(mtk_crtc);

	spin_lock_irqsave(&dev->event_lock, flags);
	if (!mtk_crtc->event) {
		if (mtk_plane->fence)
			drm_fence_signal_and_put(&mtk_plane->fence);
		mtk_plane->fence = mtk_plane->pending_fence;
		mtk_plane->pending_fence = NULL;
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static int mtk_plane_update_sync(struct mtk_drm_crtc *mtk_crtc,
		struct mtk_drm_plane *mtk_plane,
		struct reservation_object *resv)
{
	struct fence *fence;
	int ret;

	ww_mutex_lock(&resv->lock, NULL);
	ret = reservation_object_reserve_shared(resv);
	if (ret < 0) {
		DRM_ERROR("Reserve space for shared fence: %d.\n", ret);
		goto err_mutex;
	}

	fence = drm_sw_fence_new(mtk_plane->fence_context,
			atomic_add_return(1, &mtk_plane->fence_seqno));
	if (IS_ERR(fence)) {
		ret = PTR_ERR(fence);
		DRM_ERROR("Failed to create fence: %d.\n", ret);
		goto err_mutex;
	}
	mtk_crtc->pending_update |= 1 << mtk_plane->idx;
	mtk_plane->pending_fence = fence;
	drm_reservation_cb_init(&mtk_plane->rcb, mtk_plane_update_cb, mtk_plane);
	ret = drm_reservation_cb_add(&mtk_plane->rcb, resv, false);
	if (ret < 0) {
		DRM_ERROR("Adding reservation to callback failed: %d.\n", ret);
		goto err_fence;
	}
	drm_reservation_cb_done(&mtk_plane->rcb);
	reservation_object_add_shared_fence(resv, mtk_plane->pending_fence);
	ww_mutex_unlock(&resv->lock);
	return 0;
err_fence:
	fence_put(mtk_plane->pending_fence);
	mtk_plane->pending_fence = NULL;
err_mutex:
	ww_mutex_unlock(&resv->lock);

	return ret;
}

static void mtk_plane_destroy(struct drm_plane *plane)
{
	struct mtk_drm_plane *mtk_plane = to_mtk_plane(plane);

	if (mtk_plane->fence)
		drm_fence_signal_and_put(&mtk_plane->fence);

	if (mtk_plane->pending_fence)
		drm_fence_signal_and_put(&mtk_plane->pending_fence);

	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs mtk_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = mtk_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int mtk_plane_atomic_check(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	struct mtk_drm_plane *mtk_plane = to_mtk_plane(plane);
	struct drm_crtc_state *crtc_state;
	bool visible;
	int ret;
	struct drm_rect dest = {
		.x1 = state->crtc_x,
		.y1 = state->crtc_y,
		.x2 = state->crtc_x + state->crtc_w,
		.y2 = state->crtc_y + state->crtc_h,
	};
	struct drm_rect src = {
		/* 16.16 fixed point */
		.x1 = state->src_x,
		.y1 = state->src_y,
		.x2 = state->src_x + state->src_w,
		.y2 = state->src_y + state->src_h,
	};
	struct drm_rect clip = { 0, };

	if (!fb)
		return 0;

	if (!mtk_fb_get_gem_obj(fb, 0)) {
		DRM_DEBUG_KMS("buffer is null\n");
		return -EFAULT;
	}

	if (state->crtc) {
		crtc_state = drm_atomic_get_crtc_state(state->state,
						       state->crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		clip.x2 = crtc_state->mode.hdisplay;
		clip.y2 = crtc_state->mode.vdisplay;

		ret = drm_plane_helper_check_update(plane, state->crtc, fb,
						    &src, &dest, &clip,
						    DRM_PLANE_HELPER_NO_SCALING,
						    DRM_PLANE_HELPER_NO_SCALING,
						    true, true, &visible);
		if (ret)
			return ret;
	}

	if (!visible)
		return 0;

	mtk_plane->disp_size = (dest.y2 - dest.y1) << 16 | (dest.x2 - dest.x1);

	return 0;
}

static void mtk_plane_atomic_update(struct drm_plane *plane,
		struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct drm_gem_object *gem_obj;
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_drm_plane *mtk_plane = to_mtk_plane(plane);

	if (!state->crtc)
		return;

	mtk_crtc = to_mtk_crtc(state->crtc);

	if (plane->fb)
		drm_framebuffer_unreference(plane->fb);
	if (state->fb)
		drm_framebuffer_reference(state->fb);
	plane->fb = state->fb;

	gem_obj = mtk_fb_get_gem_obj(state->fb, 0);
	mtk_plane->flip_obj = to_mtk_gem_obj(gem_obj);
	mtk_plane->mtk_crtc = mtk_crtc;

	if (gem_obj->dma_buf && gem_obj->dma_buf->resv)
		mtk_plane_update_sync(mtk_crtc, mtk_plane, gem_obj->dma_buf->resv);
	else
		mtk_plane_update_cb(NULL, mtk_plane);
}

static void mtk_plane_atomic_disable(struct drm_plane *plane,
		struct drm_plane_state *old_state)
{
	struct mtk_drm_plane *mtk_plane = to_mtk_plane(plane);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(old_state->crtc);
	unsigned int i;

	if (!old_state->crtc)
		return;

	if (mtk_crtc)
		for (i= 0; i < OVL_LAYER_NR; i++)
			mtk_drm_crtc_plane_config(mtk_crtc, i, false, 0);

	mtk_drm_crtc_commit(mtk_crtc);

	if (mtk_plane->pending_fence)
		drm_fence_signal_and_put(&mtk_plane->pending_fence);

	if (mtk_plane->fence)
		drm_fence_signal_and_put(&mtk_plane->fence);
}

static const struct drm_plane_helper_funcs mtk_plane_helper_funcs = {
	.atomic_check = mtk_plane_atomic_check,
	.atomic_update = mtk_plane_atomic_update,
	.atomic_disable = mtk_plane_atomic_disable,
};

static void mtk_plane_attach_zpos_property(struct drm_plane *plane,
		unsigned int zpos, unsigned int max_plane)
{
	struct drm_device *dev = plane->dev;
	struct mtk_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->plane_zpos_property;
	if (!prop) {
		prop = drm_property_create_range(dev, DRM_MODE_PROP_IMMUTABLE,
						 "zpos", 0, max_plane - 1);
		if (!prop)
			return;

		dev_priv->plane_zpos_property = prop;
	}

	drm_object_attach_property(&plane->base, prop, zpos);
}

int mtk_plane_init(struct drm_device *dev, struct mtk_drm_plane *mtk_plane,
		   unsigned long possible_crtcs, enum drm_plane_type type,
		   unsigned int zpos, unsigned int max_plane)
{
	int err;

	err = drm_universal_plane_init(dev, &mtk_plane->base, possible_crtcs,
			&mtk_plane_funcs, formats, ARRAY_SIZE(formats), type);

	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		return err;
	}

	drm_plane_helper_add(&mtk_plane->base, &mtk_plane_helper_funcs);
	mtk_plane->idx = zpos;
	mtk_plane->fence_context = fence_context_alloc(1);
	atomic_set(&mtk_plane->fence_seqno, 0);

	if (type == DRM_PLANE_TYPE_OVERLAY)
		mtk_plane_attach_zpos_property(&mtk_plane->base,
				zpos, max_plane);

	return 0;
}
