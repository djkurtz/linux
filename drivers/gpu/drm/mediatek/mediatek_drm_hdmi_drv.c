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
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/component.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include "mediatek_hdmi_display_core.h"
#include "mediatek_hdmi.h"
#include "mediatek_hdmi_regs.h"

static const char * const mediatek_hdmi_clk_names[] = {
	"cec",
	"tvdpll",
	"dpi_sel",
	"dpi_div2",
	"dpi_div4",
	"dpi_div8",
	"hdmi_sel",
	"hdmi_div1",
	"hdmi_div2",
	"hdmi_div3",
	"hdmi_pixel",
	"hdmi_pll",
	"dpi_pixel",
	"dpi_enging",
	"aud_bclk",
	"aud_spdif",
};

static int mtk_hdmi_get_all_clk(struct mediatek_hdmi *hdmi,
				struct device_node *np)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mediatek_hdmi_clk_names); i++) {
		hdmi->clk[i] = of_clk_get_by_name(np,
						  mediatek_hdmi_clk_names[i]);
		if (IS_ERR(hdmi->clk[i]))
			return PTR_ERR(hdmi->clk[i]);
	}
	return 0;
}

static int mtk_hdmi_clk_all_enable(struct mediatek_hdmi *hdmi)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mediatek_hdmi_clk_names); i++) {
		ret = mtk_hdmi_clk_enable(hdmi, i);
		if (ret) {
			mtk_hdmi_err("enable clk %s failed!\n",
				     mediatek_hdmi_clk_names[i]);
			goto err;
		}
	}
	return 0;

err:
	for (i = i - 1; i >= 0; i--)
		mtk_hdmi_clk_disable(hdmi, i);

	return ret;
}

static void mtk_hdmi_clk_all_disable(struct mediatek_hdmi *hdmi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mediatek_hdmi_clk_names); i++)
		if (!IS_ERR_OR_NULL(hdmi->clk[i]))
			mtk_hdmi_clk_disable(hdmi, i);
}

static void hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static void hdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct mediatek_hdmi *hdmi = hdmi_ctx_from_encoder(encoder);

	mtk_hdmi_signal_output(hdmi, mode == DRM_MODE_DPMS_ON);
}

static bool hdmi_encoder_mode_fixup(struct drm_encoder *encoder,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void hdmi_encoder_mode_set(struct drm_encoder *encoder,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	struct mediatek_hdmi *hdmi = NULL;

	hdmi = hdmi_ctx_from_encoder(encoder);
	if (!hdmi) {
		mtk_hdmi_err("%s failed, invalid hdmi context!\n", __func__);
		return;
	}

	mtk_hdmi_info("cur info: name:%s, hdisplay:%d\n",
		      adjusted_mode->name,
		      adjusted_mode->hdisplay);
	mtk_hdmi_info("hsync_start:%d,hsync_end:%d, htotal:%d",
		      adjusted_mode->hsync_start,
		      adjusted_mode->hsync_end,
		      adjusted_mode->htotal);
	mtk_hdmi_info("hskew:%d, vdisplay:%d\n",
		      adjusted_mode->hskew, adjusted_mode->vdisplay);
	mtk_hdmi_info("vsync_start:%d, vsync_end:%d, vtotal:%d",
		      adjusted_mode->vsync_start,
		      adjusted_mode->vsync_end,
		      adjusted_mode->vtotal);
	mtk_hdmi_info("vscan:%d, flag:%d\n",
		      adjusted_mode->vscan, adjusted_mode->flags);
	mtk_hdmi_display_set_vid_format(hdmi->display_node,
					adjusted_mode);
}

static void hdmi_encoder_prepare(struct drm_encoder *encoder)
{
	/* DRM_MODE_DPMS_OFF? */

	/* drm framework doesn't check NULL. */
}

static void hdmi_encoder_commit(struct drm_encoder *encoder)
{
	/* DRM_MODE_DPMS_ON? */
}

static enum drm_connector_status hdmi_conn_detect(struct drm_connector *conn,
						  bool force)
{
	struct mediatek_hdmi *hdmi = hdmi_ctx_from_conn(conn);

	return hdmi->hpd ?
	       connector_status_connected : connector_status_disconnected;
}

static void hdmi_conn_destroy(struct drm_connector *conn)
{
	drm_connector_cleanup(conn);
}

static int hdmi_conn_set_property(struct drm_connector *conn,
				  struct drm_property *property, uint64_t val)
{
	return 0;
}

static int mtk_hdmi_conn_get_modes(struct drm_connector *conn)
{
	struct mediatek_hdmi *hdmi = hdmi_ctx_from_conn(conn);
	struct edid *edid;

	if (!hdmi->ddc_adpt)
		return -ENODEV;

	edid = drm_get_edid(conn, hdmi->ddc_adpt);
	if (!edid)
		return -ENODEV;

	hdmi->dvi_mode = !drm_detect_hdmi_monitor(edid);

	drm_mode_connector_update_edid_property(conn, edid);

	return drm_add_edid_modes(conn, edid);
}

static int mtk_hdmi_conn_mode_valid(struct drm_connector *conn,
				    struct drm_display_mode *mode)
{
	struct mediatek_hdmi *hdmi = hdmi_ctx_from_conn(conn);

	mtk_hdmi_info("xres=%d, yres=%d, refresh=%d, intl=%d clock=%d\n",
		      mode->hdisplay, mode->vdisplay, mode->vrefresh,
		      !!(mode->flags & DRM_MODE_FLAG_INTERLACE),
		      mode->clock * 1000);

	if (mode->clock >= hdmi->min_clock &&
	    mode->clock <= hdmi->max_clock &&
	    mode->hdisplay <= hdmi->max_hdisplay &&
	    mode->vdisplay <= hdmi->max_vdisplay)
		return MODE_OK;

	return MODE_BAD;
}

static struct drm_encoder *mtk_hdmi_conn_best_enc(struct drm_connector *conn)
{
	struct mediatek_hdmi *hdmi = hdmi_ctx_from_conn(conn);

	return &hdmi->encoder;
}

static const struct drm_encoder_funcs mtk_hdmi_encoder_funcs = {
	.destroy = hdmi_encoder_destroy,
};

static const struct drm_encoder_helper_funcs mtk_hdmi_encoder_helper_funcs = {
	.dpms = hdmi_encoder_dpms,
	.mode_fixup = hdmi_encoder_mode_fixup,
	.mode_set = hdmi_encoder_mode_set,
	.prepare = hdmi_encoder_prepare,
	.commit = hdmi_encoder_commit,
};

static const struct drm_connector_funcs mtk_hdmi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = hdmi_conn_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = hdmi_conn_destroy,
	.set_property = hdmi_conn_set_property,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs
		mtk_hdmi_connector_helper_funcs = {
	.get_modes = mtk_hdmi_conn_get_modes,
	.mode_valid = mtk_hdmi_conn_mode_valid,
	.best_encoder = mtk_hdmi_conn_best_enc,
};

static int mtk_hdmi_create_conn_enc(struct mediatek_hdmi *hdmi)
{
	int ret;

	ret = drm_encoder_init(hdmi->drm_dev, &hdmi->encoder,
			       &mtk_hdmi_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	if (ret) {
		mtk_hdmi_err("drm_encoder_init failed! ret = %d\n", ret);
		goto err;
	}
	drm_encoder_helper_add(&hdmi->encoder,
			       &mtk_hdmi_encoder_helper_funcs);

	ret = drm_connector_init(hdmi->drm_dev, &hdmi->conn,
				 &mtk_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		mtk_hdmi_err("drm_connector_init failed! ret = %d\n", ret);
		goto err;
	}
	drm_connector_helper_add(&hdmi->conn,
				 &mtk_hdmi_connector_helper_funcs);

	hdmi->conn.polled = DRM_CONNECTOR_POLL_HPD;
	hdmi->conn.interlace_allowed = true;
	hdmi->conn.doublescan_allowed = false;

	ret = drm_connector_register(&hdmi->conn);
	if (ret) {
		mtk_hdmi_err("drm_connector_register failed! (%d)\n", ret);
		goto err;
	}

	ret = drm_mode_connector_attach_encoder(&hdmi->conn,
						&hdmi->encoder);
	if (ret) {
		mtk_hdmi_err("drm_mode_connector_attach_encoder failed! (%d)\n",
			     ret);
		goto err;
	}

	hdmi->conn.encoder = &hdmi->encoder;
	hdmi->encoder.possible_crtcs = 0x2;

	mtk_hdmi_info("create encoder and connector success!\n");

	return 0;

err:
	return ret;
}

static void mtk_hdmi_destroy_conn_enc(struct mediatek_hdmi *hdmi)
{
	if (hdmi == NULL)
		return;

	drm_encoder_cleanup(&hdmi->encoder);
	drm_connector_unregister(&hdmi->conn);
	drm_connector_cleanup(&hdmi->conn);
}

static struct mediatek_hdmi_display_ops hdmi_display_ops = {
	.init = mtk_hdmi_output_init,
	.set_format = mtk_hdmi_output_set_display_mode,
};

static int mtk_hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct device_node *prenode;
	struct mediatek_hdmi *hdmi;
	struct mediatek_hdmi_display_node *display_node;

	hdmi = platform_get_drvdata(to_platform_device(dev));
	if (!hdmi) {
		mtk_hdmi_err("platform_get_drvdata failed!\n");
		return -EFAULT;
	}

	prenode = of_parse_phandle(dev->of_node, "prenode", 0);
	if (!prenode) {
		mtk_hdmi_err("find prenode node failed!\n");
		return -EFAULT;
	}

	display_node = mtk_hdmi_display_find_node(prenode);
	if (!display_node) {
		mtk_hdmi_err("find display node failed!\n");
		return -EFAULT;
	}

	if (mtk_hdmi_display_add_pre_node(hdmi->display_node,
					  display_node)) {
		mtk_hdmi_err("add display node failed!\n");
		return -EFAULT;
	}

	mtk_hdmi_power_on(hdmi);

	if (mtk_hdmi_display_init(hdmi->display_node)) {
		mtk_hdmi_err("init display failed!\n");
		return -EFAULT;
	}

	hdmi->audio_pdev_info.parent = dev;
	hdmi->audio_pdev_info.id = PLATFORM_DEVID_NONE;
	hdmi->audio_pdev_info.name = "mtk-hdmi-codec";
	hdmi->audio_pdev_info.dma_mask = DMA_BIT_MASK(32);

	hdmi->audio_data.irq = hdmi->irq;
	hdmi->audio_data.mediatek_hdmi = hdmi;
	hdmi->audio_data.enable = mtk_hdmi_audio_enable;
	hdmi->audio_data.disable = mtk_hdmi_audio_disable;
	hdmi->audio_data.set_audio_param = mtk_hdmi_audio_set_param;
	hdmi->audio_data.hpd_detect = mtk_hdmi_hpd_high;
	hdmi->audio_data.detect_dvi_monitor =
		mtk_hdmi_detect_dvi_monitor;

	hdmi->audio_pdev_info.data = &hdmi->audio_data;
	hdmi->audio_pdev_info.size_data =
		sizeof(hdmi->audio_data);
	hdmi->audio_pdev =
		platform_device_register_full(&hdmi->audio_pdev_info);
	hdmi->drm_dev = data;
	mtk_hdmi_info("hdmi = %p, data = %p , display_node = %p\n",
		      hdmi, data, display_node);

	return mtk_hdmi_create_conn_enc(hdmi);
}

static void mtk_hdmi_unbind(struct device *dev,
			    struct device *master, void *data)
{
	struct mediatek_hdmi *hdmi;

	hdmi = platform_get_drvdata(to_platform_device(dev));
	mtk_hdmi_destroy_conn_enc(hdmi);

	platform_device_unregister(hdmi->audio_pdev);
	mtk_hdmi_power_off(hdmi);

	hdmi->drm_dev = NULL;
}

static const struct component_ops mediatek_hdmi_component_ops = {
	.bind = mtk_hdmi_bind,
	.unbind = mtk_hdmi_unbind,
};

static void __iomem *mtk_hdmi_resource_ioremap(struct platform_device *pdev,
					       u32 index)
{
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, index);
	if (!mem) {
		mtk_hdmi_err("get memory source fail!\n");
		return ERR_PTR(-EFAULT);
	}

	mtk_hdmi_info("index :%d , physical adr: 0x%llx, end: 0x%llx\n", index,
		      mem->start, mem->end);

	return devm_ioremap_resource(&pdev->dev, mem);
}

static int mtk_hdmi_dt_parse_pdata(struct mediatek_hdmi *hdmi,
				   struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *i2c_np = NULL;
	int ret;

	hdmi->flt_n_5v_gpio = of_get_named_gpio(np,
							"flt_n_5v-gpios",
							0);
	if (hdmi->flt_n_5v_gpio < 0) {
		mtk_hdmi_err
		    ("hdmi->flt_n_5v_gpio = %d\n",
		     hdmi->flt_n_5v_gpio);
		goto err;
	}

	ret = mtk_hdmi_get_all_clk(hdmi, np);
	if (ret) {
		mtk_hdmi_err("get clk from dt failed!\n");
		goto err;
	}

	hdmi->sys_regs = mtk_hdmi_resource_ioremap(pdev, 0);
	if (IS_ERR(hdmi->sys_regs)) {
		mtk_hdmi_err("mem resource ioremap failed\n");
		goto err;
	}

	hdmi->pll_regs = mtk_hdmi_resource_ioremap(pdev, 1);
	if (IS_ERR(hdmi->pll_regs)) {
		mtk_hdmi_err("mem resource ioremap failed!\n");
		goto err;
	}

	hdmi->grl_regs = mtk_hdmi_resource_ioremap(pdev, 2);
	if (IS_ERR(hdmi->grl_regs)) {
		mtk_hdmi_err("mem resource ioremap failed!\n");
		goto err;
	}

	hdmi->cec_regs = mtk_hdmi_resource_ioremap(pdev, 3);
	if (IS_ERR(hdmi->cec_regs)) {
		mtk_hdmi_err("mem resource ioremap failed!\n");
		goto err;
	}

	hdmi->irq = irq_of_parse_and_map(np, 0);
	if (hdmi->irq < 0) {
		mtk_hdmi_err("get irq failed, irq = %d !\n",
			     hdmi->irq);
		goto err;
	}

	i2c_np = of_parse_phandle(np, "i2cnode", 0);
	if (!i2c_np) {
		mtk_hdmi_err("find i2c node failed!\n");
		goto err;
	}

	hdmi->ddc_adpt = of_find_i2c_adapter_by_node(i2c_np);
	if (!hdmi->ddc_adpt) {
		mtk_hdmi_err("Failed to get ddc i2c adapter by node\n");
		goto err;
	}

	ret = of_property_read_u32(np, "min_clock", &hdmi->min_clock);
	if (ret < 0) {
		mtk_hdmi_err("fail to get min clock\n");
		goto err;
	}

	ret = of_property_read_u32(np, "max_clock", &hdmi->max_clock);
	if (ret < 0) {
		mtk_hdmi_err("fail to get max clock\n");
		goto err;
	}

	ret = of_property_read_u32(np, "max_hdisplay", &hdmi->max_hdisplay);
	if (ret < 0) {
		mtk_hdmi_err("fail to get max hdisplay\n");
		goto err;
	}

	ret = of_property_read_u32(np, "max_vdisplay", &hdmi->max_vdisplay);
	if (ret < 0) {
		mtk_hdmi_err("fail to get max vdisplay\n");
		goto err;
	}

	ret = of_property_read_u32(np, "ibias", &hdmi->ibias);
	if (ret < 0) {
		mtk_hdmi_err("fail to get ibias\n");
		goto err;
	}

	ret = of_property_read_u32(np, "ibias_up", &hdmi->ibias_up);
	if (ret < 0) {
		mtk_hdmi_err("fail to get ibias up\n");
		goto err;
	}

	return 0;

err:
	return -ENXIO;
}

static irqreturn_t hdmi_flt_n_5v_irq_thread(int irq, void *arg)
{
	mtk_hdmi_err("detected 5v pin error status\n");
	return IRQ_HANDLED;
}

static irqreturn_t hdmi_htplg_isr_thread(int irq, void *arg)
{
	struct mediatek_hdmi *hdmi = arg;
	bool hpd;

	mtk_hdmi_htplg_irq_clr(hdmi);
	hpd = mtk_hdmi_hpd_high(hdmi);

	if (hdmi->hpd != hpd) {
		mtk_hdmi_info("hotplug event!,cur hpd = %d, hpd = %d\n",
			      hdmi->hpd, hpd);
		hdmi->hpd = hpd;
		if (hdmi->drm_dev)
			drm_helper_hpd_irq_event(hdmi->drm_dev);
	}
	return IRQ_HANDLED;
}

static int mtk_drm_hdmi_probe(struct platform_device *pdev)
{
	int ret;
	struct mediatek_hdmi *hdmi;
	struct device *dev = &pdev->dev;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	ret = mtk_hdmi_dt_parse_pdata(hdmi, pdev);
	if (ret) {
		mtk_hdmi_err("mtk_hdmi_dt_parse_pdata failed!!\n");
		return ret;
	}

	hdmi->flt_n_5v_irq = gpio_to_irq(hdmi->flt_n_5v_gpio);
	if (hdmi->flt_n_5v_irq < 0) {
		mtk_hdmi_err("hdmi->flt_n_5v_irq = %d\n",
			     hdmi->flt_n_5v_irq);
		return hdmi->flt_n_5v_irq;
	}

	ret = devm_request_threaded_irq(dev, hdmi->flt_n_5v_irq,
					NULL, hdmi_flt_n_5v_irq_thread,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"hdmi flt_n_5v", hdmi);
	if (ret) {
		mtk_hdmi_err("failed to register hdmi flt_n_5v interrupt\n");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, hdmi->irq,
					NULL, hdmi_htplg_isr_thread,
					IRQF_SHARED | IRQF_TRIGGER_LOW |
					IRQF_ONESHOT,
					"hdmi hpd", hdmi);
	if (ret) {
		mtk_hdmi_err("failed to register hdmi hpd interrupt\n");
		return ret;
	}

	hdmi->display_node = mtk_hdmi_display_create_node(&hdmi_display_ops,
							  pdev->dev.of_node,
							  hdmi);
	if (IS_ERR(hdmi->display_node)) {
		mtk_hdmi_err("mtk_hdmi_display_create_node failed!\n");
		return PTR_ERR(hdmi->display_node);
	}

	platform_set_drvdata(pdev, hdmi);
	mutex_init(&hdmi->hdmi_mutex);

	ret = mtk_drm_hdmi_debugfs_init(hdmi);
	if (ret) {
		mtk_hdmi_err("init debugfs failed!\n");
		mtk_hdmi_display_del_node(hdmi->display_node);
		return ret;
	}

	ret = component_add(&pdev->dev, &mediatek_hdmi_component_ops);
	if (ret) {
		mtk_hdmi_err("component_add failed !\n");
		mtk_drm_hdmi_debugfs_exit(hdmi);
		mtk_hdmi_display_del_node(hdmi->display_node);
		return ret;
	}

	ret = mtk_hdmi_clk_all_enable(hdmi);
	if (ret) {
		mtk_hdmi_err("enable all clk failed!\n");
		component_del(&pdev->dev, &mediatek_hdmi_component_ops);
		mtk_drm_hdmi_debugfs_exit(hdmi);
		mtk_hdmi_display_del_node(hdmi->display_node);
		return ret;
	}

	mtk_hdmi_info("mediatek hdmi probe success\n");
	return 0;
}

static int mtk_drm_hdmi_remove(struct platform_device *pdev)
{
	struct mediatek_hdmi *hdmi = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &mediatek_hdmi_component_ops);
	platform_set_drvdata(pdev, NULL);
	mtk_drm_hdmi_debugfs_exit(hdmi);
	mtk_hdmi_clk_all_disable(hdmi);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mtk_hdmi_suspend(struct device *dev)
{
	struct mediatek_hdmi *hdmi = dev_get_drvdata(dev);

	if (IS_ERR(hdmi)) {
		mtk_hdmi_info("hdmi suspend failed!\n");
		return PTR_ERR(hdmi);
	}
	mtk_hdmi_power_off(hdmi);
	mtk_hdmi_clk_all_disable(hdmi);
	mtk_hdmi_info("hdmi suspend success!\n");
	return 0;
}

static int mtk_hdmi_resume(struct device *dev)
{
	struct mediatek_hdmi *hdmi = dev_get_drvdata(dev);
	int ret = 0;

	if (IS_ERR(hdmi)) {
		mtk_hdmi_info("hdmi resume failed!\n");
		return PTR_ERR(hdmi);
	}

	ret = mtk_hdmi_clk_all_enable(hdmi);
	if (ret) {
		mtk_hdmi_info("hdmi resume failed!\n");
		return ret;
	}

	mtk_hdmi_power_on(hdmi);
	mtk_hdmi_display_set_vid_format(hdmi->display_node,
					&hdmi->display_node->mode);
	mtk_hdmi_info("hdmi resume success!\n");
	return 0;
}
#endif
static SIMPLE_DEV_PM_OPS(mediatek_hdmi_pm_ops,
			 mtk_hdmi_suspend, mtk_hdmi_resume);

static const struct of_device_id mediatek_drm_hdmi_of_ids[] = {
	{ .compatible = "mediatek,mt8173-hdmi", },
	{}
};

struct platform_driver mediatek_hdmi_driver = {
	.probe = mtk_drm_hdmi_probe,
	.remove = mtk_drm_hdmi_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mediatek-drm-hdmi",
		   .of_match_table = mediatek_drm_hdmi_of_ids,
		   .pm = &mediatek_hdmi_pm_ops,
		   },
};

static int __init mtk_hdmitx_init(void)
{
	int ret;

	ret = platform_driver_register(&mediatek_hdmi_ddc_driver);
	if (ret < 0) {
		mtk_hdmi_err("register hdmiddc platform driver failed!");
		goto err;
	}

	ret = platform_driver_register(&mediatek_hdmi_dpi_driver);
	if (ret < 0) {
		mtk_hdmi_err("register hdmidpi platform driver failed!");
		goto ddc_err;
	}

	ret = platform_driver_register(&mediatek_hdmi_driver);
	if (ret < 0) {
		mtk_hdmi_err("register hdmitx platform driver failed!");
		goto dpi_err;
	}

	return 0;

dpi_err:
	platform_driver_unregister(&mediatek_hdmi_dpi_driver);
ddc_err:
	platform_driver_unregister(&mediatek_hdmi_ddc_driver);
err:
	return ret;
}

static void __exit mtk_hdmitx_exit(void)
{
	platform_driver_unregister(&mediatek_hdmi_driver);
	platform_driver_unregister(&mediatek_hdmi_dpi_driver);
	platform_driver_unregister(&mediatek_hdmi_ddc_driver);
}

module_init(mtk_hdmitx_init);
module_exit(mtk_hdmitx_exit);

MODULE_AUTHOR("Jie Qiu <jie.qiu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek HDMI Driver");
MODULE_LICENSE("GPL");
