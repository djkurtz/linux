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

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-buf.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reservation.h>

#include "mediatek_drm_drv.h"
#include "mediatek_drm_crtc.h"
#include "mediatek_drm_ddp.h"
#include "mediatek_drm_ddp_comp.h"
#include "mediatek_drm_gem.h"
#include "mediatek_drm_plane.h"


struct crtc_ddp_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mtk_drm_crtc		*crtc;
	struct mtk_drm_plane		planes[OVL_LAYER_NR];
	int				pipe;

	struct device			*ddp_dev;
	u32				ddp_comp_nr;
	struct clk			**ddp_comp_clk;
	void __iomem			**ddp_comp_regs;
	const struct mtk_ddp_comp_funcs	**ddp_comp;

	bool				pending_config;
	unsigned int			pending_width;
	unsigned int			pending_height;

	bool				pending_ovl_config[OVL_LAYER_NR];
	bool				pending_ovl_enable[OVL_LAYER_NR];
	unsigned int			pending_ovl_addr[OVL_LAYER_NR];
	unsigned int			pending_ovl_pitch[OVL_LAYER_NR];
	unsigned int			pending_ovl_format[OVL_LAYER_NR];
	int				pending_ovl_x[OVL_LAYER_NR];
	int				pending_ovl_y[OVL_LAYER_NR];
	unsigned int			pending_ovl_size[OVL_LAYER_NR];
	bool				pending_ovl_dirty[OVL_LAYER_NR];
};

void mtk_drm_crtc_finish_page_flip(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_device *dev = mtk_crtc->base.dev;

	drm_send_vblank_event(dev, mtk_crtc->event->pipe, mtk_crtc->event);
	drm_crtc_vblank_put(&mtk_crtc->base);
	mtk_crtc->event = NULL;
}

static void mtk_drm_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static bool mtk_drm_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* drm framework doesn't check NULL */
	return true;
}

static void mtk_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct crtc_ddp_context *ctx = mtk_crtc->ctx;

	if (WARN_ON(!crtc->state))
		return;

	ctx->pending_width = crtc->mode.hdisplay;
	ctx->pending_height = crtc->mode.vdisplay;
	ctx->pending_config = true;
}

int mtk_drm_crtc_enable_vblank(struct drm_device *drm, int pipe)
{
	struct mtk_drm_private *priv = drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);
	struct crtc_ddp_context *ctx = mtk_crtc->ctx;

	if (ctx->ddp_comp[0]->comp_enable_vblank)
		ctx->ddp_comp[0]->comp_enable_vblank(ctx->ddp_comp_regs[0]);

	return 0;
}

void mtk_drm_crtc_disable_vblank(struct drm_device *drm, int pipe)
{
	struct mtk_drm_private *priv = drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);
	struct crtc_ddp_context *ctx = mtk_crtc->ctx;

	if (ctx->ddp_comp[0]->comp_disable_vblank)
		ctx->ddp_comp[0]->comp_disable_vblank(ctx->ddp_comp_regs[0]);
}

static void mtk_crtc_ddp_power_on(struct crtc_ddp_context *ctx)
{
	int ret;
	int i;

	DRM_INFO("mtk_crtc_ddp_power_on\n");
	for (i = 0; i < ctx->ddp_comp_nr; i++) {
		if (ctx->ddp_comp_clk[i])
			ret = clk_enable(ctx->ddp_comp_clk[i]);

		if (ret)
			DRM_ERROR("clk_enable(ctx->ddp_comp_clk[%d])\n", i);
	}
}

static void mtk_crtc_ddp_power_off(struct crtc_ddp_context *ctx)
{
	int i;

	DRM_INFO("mtk_crtc_ddp_power_off\n");
	for (i = 0; i < ctx->ddp_comp_nr; i++)
		if (ctx->ddp_comp_clk[i])
			clk_disable(ctx->ddp_comp_clk[i]);
}

static void mtk_crtc_ddp_hw_init(struct mtk_drm_crtc *crtc)
{
	struct crtc_ddp_context *ctx = crtc->ctx;
	unsigned int width, height;
	int i;

	if (ctx->crtc->base.state) {
		width = ctx->crtc->base.state->adjusted_mode.hdisplay;
		height = ctx->crtc->base.state->adjusted_mode.vdisplay;
	} else {
		width = 1920;
		height = 1080;
	}

	DRM_INFO("mtk_crtc_ddp_hw_init\n");
	mtk_ddp_clock_on(ctx->ddp_dev);
	mtk_crtc_ddp_power_on(ctx);

	DRM_INFO("mediatek_ddp_ddp_path_setup\n");
	for (i = 0; i < (ctx->ddp_comp_nr-1); i++) {
		mtk_ddp_add_comp_to_path(ctx->ddp_dev, ctx->pipe,
					 ctx->ddp_comp[i]->comp_type,
					 ctx->ddp_comp[i+1]->comp_type);
	}

	DRM_INFO("ddp_disp_path_power_on %dx%d\n", width, height);
	for (i = 0; i < ctx->ddp_comp_nr; i++) {
		if (ctx->ddp_comp[i]->comp_config)
			ctx->ddp_comp[i]->comp_config(ctx->ddp_comp_regs[i],
						      width, height);

		if (ctx->ddp_comp[i]->comp_power_on)
			ctx->ddp_comp[i]->comp_power_on(ctx->ddp_comp_regs[i]);
	}
}

static void mtk_crtc_ddp_hw_fini(struct mtk_drm_crtc *crtc)
{
	struct crtc_ddp_context *ctx = crtc->ctx;

	DRM_INFO("mtk_crtc_ddp_hw_fini\n");
	mtk_crtc_ddp_power_off(ctx);
	mtk_ddp_clock_off(ctx->ddp_dev);
}

static void mtk_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (mtk_crtc->enabled)
		return;

	mtk_crtc_ddp_hw_init(mtk_crtc);

	drm_crtc_vblank_on(crtc);

	mtk_crtc->enabled = true;
}

static void mtk_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	DRM_INFO("mtk_drm_crtc_disable %d\n", crtc->base.id);
	if (!mtk_crtc->enabled)
		return;

	drm_crtc_vblank_off(crtc);

	mtk_crtc_ddp_hw_fini(mtk_crtc);

	mtk_crtc->enabled = false;
}

static void mtk_drm_crtc_atomic_begin(struct drm_crtc *crtc)
{
	struct drm_crtc_state *state = crtc->state;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (state->event) {
		state->event->pipe = drm_crtc_index(crtc);
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);
		mtk_crtc->event = state->event;
		state->event = NULL;
	}
}

void mtk_drm_crtc_commit(struct mtk_drm_crtc *crtc)
{
	struct crtc_ddp_context *ctx = crtc->ctx;
	unsigned int i;

	for (i = 0; i < OVL_LAYER_NR; i++)
		if (ctx->pending_ovl_dirty[i]) {
			ctx->pending_ovl_config[i] = true;
			ctx->pending_ovl_dirty[i] = false;
		}
}

void mtk_drm_crtc_check_flush(struct mtk_drm_crtc *mtk_crtc)
{
	if (mtk_crtc->do_flush && !mtk_crtc->pending_update) {
		if (mtk_crtc->event)
			mtk_crtc->pending_needs_vblank = true;
		mtk_drm_crtc_commit(mtk_crtc);
		mtk_crtc->do_flush = false;
	}
}

static void mtk_drm_crtc_atomic_flush(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	mtk_crtc->do_flush = true;
	mtk_drm_crtc_check_flush(mtk_crtc);
}

static const struct drm_crtc_funcs mtk_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.destroy		= mtk_drm_crtc_destroy,
	.reset			= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_crtc_helper_funcs mtk_crtc_helper_funcs = {
	.mode_fixup	= mtk_drm_crtc_mode_fixup,
	.mode_set_nofb	= mtk_drm_crtc_mode_set_nofb,
	.enable		= mtk_drm_crtc_enable,
	.disable	= mtk_drm_crtc_disable,
	.atomic_begin	= mtk_drm_crtc_atomic_begin,
	.atomic_flush	= mtk_drm_crtc_atomic_flush,
};

struct mtk_drm_crtc *mtk_drm_crtc_create(struct drm_device *drm,
		struct drm_plane *primary, struct drm_plane *cursor, int pipe,
		void *ctx)
{
	struct mtk_drm_private *priv = drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc;
	int ret;

	mtk_crtc = devm_kzalloc(drm->dev, sizeof(*mtk_crtc), GFP_KERNEL);
	if (!mtk_crtc) {
		DRM_ERROR("failed to allocate mtk crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	mtk_crtc->pipe = pipe;
	mtk_crtc->ctx = ctx;
	mtk_crtc->enabled = false;

	priv->crtc[pipe] = &mtk_crtc->base;

	ret = drm_crtc_init_with_planes(drm, &mtk_crtc->base, primary, cursor,
			&mtk_crtc_funcs);
	if (ret)
		return ERR_PTR(ret);

	drm_crtc_helper_add(&mtk_crtc->base, &mtk_crtc_helper_funcs);

	return mtk_crtc;
}

static int mtk_crtc_ddp_ctx_initialize(struct crtc_ddp_context *ctx,
				   struct drm_device *drm_dev)
{
	struct mtk_drm_private *priv = drm_dev->dev_private;

	ctx->drm_dev = drm_dev;
	ctx->pipe = priv->pipe++;

	return 0;
}

static void mtk_crtc_ddp_ctx_remove(struct crtc_ddp_context *ctx)
{
}

static void mtk_crtc_ddp_dpms(struct mtk_drm_crtc *crtc, int mode)
{
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		mtk_crtc_ddp_hw_init(crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		mtk_crtc_ddp_hw_fini(crtc);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

void mtk_drm_crtc_plane_config(struct mtk_drm_crtc *crtc, unsigned int idx,
				  bool enable, dma_addr_t addr)
{
	struct crtc_ddp_context *ctx = crtc->ctx;
	struct drm_plane *plane = &ctx->planes[idx].base;
	unsigned int pitch, format;
	int x, y;

	if (!plane->fb || !plane->state)
		return;

	if (plane->fb) {
		pitch = plane->fb->pitches[0];
		format = plane->fb->pixel_format;
	}

	if (plane->state) {
		x = plane->state->crtc_x;
		y = plane->state->crtc_y;

		if (x < 0) {
			addr -= x * 4;
			x = 0;
		}

		if (y < 0) {
			addr -= y * pitch;
			y = 0;
		}
	}

	ctx->pending_ovl_enable[idx] = enable;
	ctx->pending_ovl_addr[idx] = addr;
	ctx->pending_ovl_pitch[idx] = pitch;
	ctx->pending_ovl_format[idx] = format;
	ctx->pending_ovl_x[idx] = x;
	ctx->pending_ovl_y[idx] = y;
	ctx->pending_ovl_size[idx] = ctx->planes[idx].disp_size;
	ctx->pending_ovl_dirty[idx] = true;
}

static void mtk_crtc_ddp_irq(struct crtc_ddp_context *ctx)
{
	struct drm_device *dev = ctx->drm_dev;
	struct mtk_drm_crtc *mtk_crtc = ctx->crtc;
	void __iomem *ovl_reg = ctx->ddp_comp_regs[0];
	unsigned int i;
	unsigned long flags;

	if (ctx->pending_config) {
		ctx->pending_config = false;

		for (i = 0; i < OVL_LAYER_NR; i++)
			ctx->ddp_comp[0]->comp_layer_off(ovl_reg, i);
		ctx->ddp_comp[0]->comp_config(ctx->ddp_comp_regs[0],
					      ctx->pending_width,
					      ctx->pending_height);
	}

	for (i = 0; i < OVL_LAYER_NR; i++) {
		if (ctx->pending_ovl_config[i]) {
			if (!ctx->pending_ovl_enable[i])
				ctx->ddp_comp[0]->comp_layer_off(ovl_reg, i);

			ctx->ddp_comp[0]->comp_layer_config(ovl_reg, i,
					ctx->pending_ovl_addr[i],
					ctx->pending_ovl_pitch[i],
					ctx->pending_ovl_format[i],
					ctx->pending_ovl_x[i],
					ctx->pending_ovl_y[i],
					ctx->pending_ovl_size[i]);

			if (ctx->pending_ovl_enable[i])
				ctx->ddp_comp[0]->comp_layer_on(ovl_reg, i);
		}

		ctx->pending_ovl_config[i] = false;
	}

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	spin_lock_irqsave(&dev->event_lock, flags);
	if (mtk_crtc->pending_needs_vblank) {
		mtk_drm_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;

		for (i = 0; i < OVL_LAYER_NR; i++) {
			mtk_plane_finish_page_flip(&ctx->planes[i]);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

irqreturn_t mtk_crtc_ddp_irq_handler(int irq, void *dev_id)
{
	struct crtc_ddp_context *ctx = dev_id;

	if (ctx->ddp_comp[0]->comp_clear_vblank)
		ctx->ddp_comp[0]->comp_clear_vblank(ctx->ddp_comp_regs[0]);

	if (ctx->pipe >= 0 && ctx->drm_dev)
		mtk_crtc_ddp_irq(ctx);

	return IRQ_HANDLED;
}

static int mtk_crtc_ddp_bind(struct device *dev, struct device *master,
		void *data)
{
	struct crtc_ddp_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	enum drm_plane_type type;
	unsigned int zpos;
	int ret;

	ret = mtk_crtc_ddp_ctx_initialize(ctx, drm_dev);
	if (ret) {
		DRM_ERROR("crtc_ddp_ctx_initialize failed.\n");
		return ret;
	}

	for (zpos = 0; zpos < OVL_LAYER_NR; zpos++) {
		type = (zpos == 0) ? DRM_PLANE_TYPE_PRIMARY :
				(zpos == 1) ? DRM_PLANE_TYPE_CURSOR :
						DRM_PLANE_TYPE_OVERLAY;
		ret = mtk_plane_init(drm_dev, &ctx->planes[zpos],
				1 << ctx->pipe, type, zpos, OVL_LAYER_NR);
		if (ret)
			return ret;
	}

	ctx->crtc = mtk_drm_crtc_create(drm_dev, &ctx->planes[0].base,
			&ctx->planes[1].base, ctx->pipe, ctx);

	if (IS_ERR(ctx->crtc)) {
		mtk_crtc_ddp_ctx_remove(ctx);
		return PTR_ERR(ctx->crtc);
	}

	mtk_crtc_ddp_hw_init(ctx->crtc);
	ctx->ddp_comp[0]->comp_layer_off(ctx->ddp_comp_regs[0], 0);

	return 0;
}

static void mtk_crtc_ddp_unbind(struct device *dev, struct device *master,
		void *data)
{
	struct crtc_ddp_context *ctx = dev_get_drvdata(dev);

	mtk_crtc_ddp_hw_fini(ctx->crtc);
}

static const struct component_ops mtk_crtc_ddp_component_ops = {
	.bind	= mtk_crtc_ddp_bind,
	.unbind = mtk_crtc_ddp_unbind,
};

static int mtk_crtc_ddp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct crtc_ddp_context *ctx;
	struct device_node *node;
	struct platform_device *ddp_pdev;
	struct resource *res;
	int irq;
	int ret;
	int i;
	const char *comp_name;

	if (!dev->of_node)
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	node = of_parse_phandle(dev->of_node, "ddp", 0);
	if (!node) {
		dev_err(dev, "Get ddp node fail.\n");
		ret = -EINVAL;
		goto err;
	}

	ddp_pdev = of_find_device_by_node(node);
	if (!ddp_pdev) {
		dev_err(dev, "Find ddp device fail.\n");
		ret = -EINVAL;
		goto err;
	}
	ctx->ddp_dev = &ddp_pdev->dev;

	ret = of_property_read_u32(dev->of_node,
			"ddp-comp-nr", &ctx->ddp_comp_nr);
	if (ret) {
		dev_err(dev, "Get ddp-comp-nr fail.\n");
		ret = -EINVAL;
		goto err;
	}

	ctx->ddp_comp_clk = devm_kmalloc_array(dev, ctx->ddp_comp_nr,
			sizeof(struct clk *), GFP_KERNEL);
	ctx->ddp_comp_regs = devm_kmalloc_array(dev, ctx->ddp_comp_nr,
			sizeof(void __iomem *), GFP_KERNEL);
	ctx->ddp_comp =	devm_kmalloc_array(dev, ctx->ddp_comp_nr,
			sizeof(struct mtk_ddp_comp_funcs *), GFP_KERNEL);

	for (i = 0; i < ctx->ddp_comp_nr; i++) {
		ctx->ddp_comp_clk[i] = of_clk_get(dev->of_node, i);
		if (IS_ERR(ctx->ddp_comp_clk[i])) {
			ctx->ddp_comp_clk[i] = NULL;
		} else {
			ret = clk_prepare(ctx->ddp_comp_clk[i]);
			if (ret) {
				dev_err(dev, "clk_prepare comp[%d]\n", i);
				ret = -EINVAL;
				goto unprepare;
			}
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (res)
			ctx->ddp_comp_regs[i] =	devm_ioremap_resource(dev, res);

		ret = of_property_read_string_index(dev->of_node,
				"ddp-comp-names", i, &comp_name);
		if (ret) {
			dev_err(dev, "Get ddp-comp-names fail.\n");
			ret = -EINVAL;
			goto unprepare;
		}

		ctx->ddp_comp[i] = mtk_ddp_get_comp(comp_name);
		if (!ctx->ddp_comp[i]) {
			dev_err(dev, "Get component %s fail\n", comp_name);
			ret = -EINVAL;
			goto unprepare;
		}
	}

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, irq, mtk_crtc_ddp_irq_handler,
			IRQF_TRIGGER_NONE, dev_name(dev), ctx);
	if (ret < 0) {
		dev_err(dev, "devm_request_irq %d fail %d\n", irq, ret);
		ret = -ENXIO;
		goto unprepare;
	}

	platform_set_drvdata(pdev, ctx);

	ret = component_add(&pdev->dev, &mtk_crtc_ddp_component_ops);
	if (ret) {
		dev_err(dev, "component_add fail\n");
		goto unprepare;
	}

	return 0;

unprepare:
	for (i = 0; i < ctx->ddp_comp_nr; i++)
		clk_prepare(ctx->ddp_comp_clk[i]);
err:
	if (node)
		of_node_put(node);

	return ret;
}

static int mtk_crtc_ddp_remove(struct platform_device *pdev)
{
	struct crtc_ddp_context *ctx = platform_get_drvdata(pdev);
	int idx;

	component_del(&pdev->dev, &mtk_crtc_ddp_component_ops);

	for (idx = 0; idx < ctx->ddp_comp_nr; idx++)
		if (ctx->ddp_comp_clk[idx])
			clk_unprepare(ctx->ddp_comp_clk[idx]);

	return 0;
}

static const struct of_device_id mtk_crtc_ddp_driver_dt_match[] = {
	{ .compatible = "mediatek,mt8173-crtc-ddp" },
	{},
};
MODULE_DEVICE_TABLE(of, mtk_crtc_ddp_driver_dt_match);

struct platform_driver mtk_crtc_ddp_driver = {
	.probe		= mtk_crtc_ddp_probe,
	.remove		= mtk_crtc_ddp_remove,
	.driver		= {
		.name	= "mediatek-crtc-ddp",
		.owner	= THIS_MODULE,
		.of_match_table = mtk_crtc_ddp_driver_dt_match,
	},
};

module_platform_driver(mtk_crtc_ddp_driver);

