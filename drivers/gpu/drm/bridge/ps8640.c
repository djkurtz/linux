/*
 * Copyright (c) 2014 MediaTek Inc.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>

#define PAGE2_GPIO_L	0xa6
#define PAGE2_GPIO_H	0xa7
#define PS_GPIO9	BIT(1)

#define PAGE4_REV_L	0xf0
#define PAGE4_REV_H	0xf1
#define PAGE4_CHIP_L	0xf2
#define PAGE4_CHIP_H	0xf3

#define bridge_to_ps8640(e)	container_of(e, struct ps8640, bridge)
#define connector_to_ps8640(e)	container_of(e, struct ps8640, connector)

struct ps8640 {
	struct drm_connector connector;
	struct drm_bridge bridge;
	struct i2c_client *client;
	struct ps8640_driver_data *driver_data;
	struct regulator *pwr_1v2_supply;
	struct regulator *pwr_3v3_supply;
	struct drm_panel *panel;
	struct gpio_desc *gpio_rst_n;
	struct gpio_desc *gpio_pwr_n;
	struct gpio_desc *gpio_mode_sel_n;
	void *edid;
	u16 base_reg;
	bool enabled;
};

static int ps8640_regr(struct i2c_client *client, u16 i2c_addr,
		       u8 reg, u8 *value)
{
	int ret;

	client->addr = i2c_addr;

	ret = i2c_master_send(client, &reg, 1);
	if (ret <= 0) {
		DRM_ERROR("Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	ret = i2c_master_recv(client, value, 1);
	if (ret <= 0) {
		DRM_ERROR("Failed to recv i2c data, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ps8640_regw(struct i2c_client *client, u16 i2c_addr,
		       u8 reg, u8 value)
{
	int ret;
	char buf[2];

	client->addr = i2c_addr;

	buf[0] = reg;
	buf[1] = value;
	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret <= 0) {
		DRM_ERROR("Failed to send i2c command, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ps8640_check_valid_id(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->client;
	u8 rev_id_low, rev_id_high, chip_id_low, chip_id_high;
	int retry_cnt = 0;

	do {
		ps8640_regr(client, ps_bridge->base_reg + 4, PAGE4_CHIP_H,
			    &chip_id_high);
		if (chip_id_high != 0x30)
			DRM_INFO("chip_id_high = 0x%x\n", chip_id_high);
	} while ((retry_cnt++ < 2) && (chip_id_high != 0x30));

	ps8640_regr(client, ps_bridge->base_reg + 4, PAGE4_REV_L, &rev_id_low);
	ps8640_regr(client, ps_bridge->base_reg + 4, PAGE4_REV_H, &rev_id_high);
	ps8640_regr(client, ps_bridge->base_reg + 4, PAGE4_CHIP_L,
		    &chip_id_low);

	if ((rev_id_low == 0x00) && (rev_id_high == 0x0a) &&
	    (chip_id_low == 0x00) && (chip_id_high == 0x30))
		return 1;

	return 0;
}

static void ps8640_show_mcu_fw_version(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->client;
	u8 major_ver, minor_ver;

	ps8640_regr(client, ps_bridge->base_reg + 5, 0x4, &major_ver);
	ps8640_regr(client, ps_bridge->base_reg + 5, 0x5, &minor_ver);

	DRM_INFO_ONCE("ps8640 rom fw version %d.%d\n", major_ver, minor_ver);
}

static int ps8640_bdg_enable(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->client;

	if (ps8640_check_valid_id(ps_bridge)) {
		ps8640_regw(client, ps_bridge->base_reg + 3, 0xfe, 0x13);
		ps8640_regw(client, ps_bridge->base_reg + 3, 0xff, 0x18);
		ps8640_regw(client, ps_bridge->base_reg + 3, 0xfe, 0x13);
		ps8640_regw(client, ps_bridge->base_reg + 3, 0xff, 0x1c);

		return 0;
	}

	return -1;
}

static void ps8640_prepare(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->client;
	int err, retry_cnt = 0;
	u8 set_vdo_done;

	if (ps_bridge->enabled)
		return;

	if (drm_panel_prepare(ps_bridge->panel)) {
		DRM_ERROR("failed to prepare panel\n");
		return;
	}

	err = regulator_enable(ps_bridge->pwr_1v2_supply);
	if (err < 0) {
		DRM_ERROR("failed to enable pwr_1v2_supply: %d\n", err);
		return;
	}

	err = regulator_enable(ps_bridge->pwr_3v3_supply);
	if (err < 0) {
		DRM_ERROR("failed to enable pwr_3v3_supply: %d\n", err);
		return;
	}

	gpiod_set_value(ps_bridge->gpio_pwr_n, 1);
	gpiod_set_value(ps_bridge->gpio_rst_n, 0);
	usleep_range(500, 700);
	gpiod_set_value(ps_bridge->gpio_rst_n, 1);

	do {
		msleep(50);
		ps8640_regr(client, ps_bridge->base_reg + 2, PAGE2_GPIO_H,
			    &set_vdo_done);
	} while ((retry_cnt++ < 70) && ((set_vdo_done & PS_GPIO9) != PS_GPIO9));

	ps8640_show_mcu_fw_version(ps_bridge);
	ps_bridge->enabled = true;
}

static void ps8640_pre_enable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);

	ps8640_prepare(ps_bridge);
}

static void ps8640_enable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);

	ps8640_bdg_enable(ps_bridge);

	if (drm_panel_enable(ps_bridge->panel)) {
		DRM_ERROR("failed to enable panel\n");
		return;
	}
}

static void ps8640_disable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);

	if (!ps_bridge->enabled)
		return;

	ps_bridge->enabled = false;

	if (drm_panel_disable(ps_bridge->panel)) {
		DRM_ERROR("failed to disable panel\n");
		return;
	}

	regulator_disable(ps_bridge->pwr_1v2_supply);
	regulator_disable(ps_bridge->pwr_3v3_supply);
	gpiod_set_value(ps_bridge->gpio_rst_n, 0);
	gpiod_set_value(ps_bridge->gpio_pwr_n, 0);
}

static void ps8640_post_disable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);

	if (drm_panel_unprepare(ps_bridge->panel)) {
		DRM_ERROR("failed to unprepare panel\n");
		return;
	}
}

static int ps8640_get_modes(struct drm_connector *connector)
{
	struct ps8640 *ps_bridge = connector_to_ps8640(connector);
	struct i2c_client *client = ps_bridge->client;

	ps8640_prepare(ps_bridge);
	ps8640_regw(client, ps_bridge->base_reg + 2, 0xea, 0xd0);

	return drm_panel_get_modes(ps_bridge->panel);
}

static struct drm_encoder *ps8640_best_encoder(struct drm_connector *connector)
{
	struct ps8640 *ps_bridge;

	ps_bridge = connector_to_ps8640(connector);
	return ps_bridge->bridge.encoder;
}

static const struct drm_connector_helper_funcs
	ps8640_connector_helper_funcs = {
	.get_modes = ps8640_get_modes,
	.best_encoder = ps8640_best_encoder,
};

static enum drm_connector_status ps8640_detect(struct drm_connector *connector,
					       bool force)
{
	return connector_status_connected;
}

static void ps8640_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs ps8640_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ps8640_detect,
	.destroy = ps8640_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

int ps8640_bridge_attach(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	ret = drm_connector_init(bridge->dev, &ps_bridge->connector,
				 &ps8640_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);

	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&ps_bridge->connector,
				 &ps8640_connector_helper_funcs);
	drm_connector_register(&ps_bridge->connector);

	ps_bridge->connector.dpms = DRM_MODE_DPMS_ON;
	drm_mode_connector_attach_encoder(&ps_bridge->connector,
					  bridge->encoder);

	if (ps_bridge->panel)
		drm_panel_attach(ps_bridge->panel, &ps_bridge->connector);

	return ret;
}

static bool ps8640_bridge_mode_fixup(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	return true;
}

static const struct drm_bridge_funcs ps8640_bridge_funcs = {
	.attach = ps8640_bridge_attach,
	.mode_fixup = ps8640_bridge_mode_fixup,
	.disable = ps8640_disable,
	.post_disable = ps8640_post_disable,
	.pre_enable = ps8640_pre_enable,
	.enable = ps8640_enable,
};

static int ps8640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ps8640 *ps_bridge;
	struct device_node *np = dev->of_node;
	struct device_node *panel_node;
	int ret, size;
	u32 temp_reg;
	const u8 *edidp;

	ps_bridge = devm_kzalloc(dev, sizeof(*ps_bridge), GFP_KERNEL);
	if (!ps_bridge)
		return -ENOMEM;

	panel_node = of_parse_phandle(dev->of_node, "mediatek,panel", 0);
	if (panel_node) {
		ps_bridge->panel = of_drm_find_panel(panel_node);
		of_node_put(panel_node);
		if (!ps_bridge->panel)
			return -EPROBE_DEFER;
	}

	ps_bridge->client = client;

	edidp = of_get_property(np, "edid", &size);

	if (edidp)
		ps_bridge->edid = devm_kmemdup(&client->dev, edidp, size,
					       GFP_KERNEL);

	ps_bridge->pwr_3v3_supply = devm_regulator_get(dev, "ps8640-3v3");
	if (IS_ERR(ps_bridge->pwr_3v3_supply)) {
		dev_err(dev, "cannot get ps_bridge->pwr_3v3_supply\n");
		return PTR_ERR(ps_bridge->pwr_3v3_supply);
	}

	ps_bridge->pwr_1v2_supply = devm_regulator_get(dev, "ps8640-1v2");
	if (IS_ERR(ps_bridge->pwr_1v2_supply)) {
		dev_err(dev, "cannot get ps_bridge->pwr_1v2_supply\n");
		return PTR_ERR(ps_bridge->pwr_1v2_supply);
	}

	ps_bridge->gpio_mode_sel_n = devm_gpiod_get(&client->dev, "mode-sel");
	if (IS_ERR(ps_bridge->gpio_mode_sel_n)) {
		ret = PTR_ERR(ps_bridge->gpio_mode_sel_n);
		DRM_ERROR("cannot get gpio_mode_sel_n %d\n", ret);
		return ret;
	}

	ret = gpiod_direction_output(ps_bridge->gpio_mode_sel_n, 1);
	if (ret) {
		DRM_ERROR("cannot configure gpio_mode_sel_n\n");
		return ret;
	}

	ps_bridge->gpio_pwr_n = devm_gpiod_get(&client->dev, "power");
	if (IS_ERR(ps_bridge->gpio_pwr_n)) {
		ret = PTR_ERR(ps_bridge->gpio_pwr_n);
		DRM_ERROR("cannot get gpio_pwr_n %d\n", ret);
		return ret;
	}

	ret = gpiod_direction_output(ps_bridge->gpio_pwr_n, 1);
	if (ret) {
		DRM_ERROR("cannot configure gpio_pwr_n\n");
		return ret;
	}

	ps_bridge->gpio_rst_n = devm_gpiod_get(&client->dev, "reset");
	if (IS_ERR(ps_bridge->gpio_rst_n)) {
		ret = PTR_ERR(ps_bridge->gpio_rst_n);
		DRM_ERROR("cannot get gpio_rst_n %d\n", ret);
		return ret;
	}

	ret = gpiod_direction_output(ps_bridge->gpio_rst_n, 1);
	if (ret) {
		DRM_ERROR("cannot configure gpio_rst_n\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "reg", &temp_reg);
	if (ret) {
		DRM_ERROR("Can't read base_reg value\n");
		return ret;
	}
	ps_bridge->base_reg = temp_reg;

	ps_bridge->bridge.funcs = &ps8640_bridge_funcs;
	ps_bridge->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&ps_bridge->bridge);
	if (ret) {
		DRM_ERROR("Failed to add bridge\n");
		return ret;
	}

	i2c_set_clientdata(client, ps_bridge);

	return 0;
}

static int ps8640_remove(struct i2c_client *client)
{
	struct ps8640 *ps_bridge = i2c_get_clientdata(client);

	drm_bridge_remove(&ps_bridge->bridge);

	return 0;
}

static const struct i2c_device_id ps8640_i2c_table[] = {
	{"parade,ps8640", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ps8640_i2c_table);

static const struct of_device_id ps8640_match[] = {
	{ .compatible = "parade,ps8640" },
	{},
};
MODULE_DEVICE_TABLE(of, ps8640_match);

static struct i2c_driver ps8640_driver = {
	.id_table = ps8640_i2c_table,
	.probe = ps8640_probe,
	.remove = ps8640_remove,
	.driver = {
		.name = "parade,ps8640",
		.owner = THIS_MODULE,
		.of_match_table = ps8640_match,
	},
};
module_i2c_driver(ps8640_driver);

MODULE_AUTHOR("Jitao Shi <jitao.shi@mediatek.com>");
MODULE_AUTHOR("CK Hu <ck.hu@mediatek.com>");
MODULE_DESCRIPTION("PARADE ps8640 DSI-eDP converter driver");
MODULE_LICENSE("GPL v2");

