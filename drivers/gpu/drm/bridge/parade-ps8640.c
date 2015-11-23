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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_panel.h>

#include <drmP.h>
#include <drm_crtc_helper.h>
#include <drm_crtc.h>
#include <drm_mipi_dsi.h>
#include <drm_atomic_helper.h>

#define PAGE2_GPIO_L		0xa6
#define PAGE2_GPIO_H		0xa7
#define PS_GPIO9		BIT(1)
#define PAGE2_I2C_BYPASS	0xea
#define I2C_BYPASS_EN		0xd0

#define PAGE3_SET_ADD		0xfe
#define PAGE3_SET_VAL		0xff
#define VDO_CTL_ADD		0x13
#define VDO_DIS			0x18
#define VDO_EN			0x1c

#define PAGE4_REV_L		0xf0
#define PAGE4_REV_H		0xf1
#define PAGE4_CHIP_L		0xf2
#define PAGE4_CHIP_H		0xf3

#define bridge_to_ps8640(e)	container_of(e, struct ps8640, bridge)
#define connector_to_ps8640(e)	container_of(e, struct ps8640, connector)

struct ps8640 {
	struct drm_connector connector;
	struct drm_bridge bridge;
	struct i2c_client *page[6];
	struct ps8640_driver_data *driver_data;
	struct regulator *pwr_1v2_supply;
	struct regulator *pwr_3v3_supply;
	struct drm_panel *panel;
	struct gpio_desc *gpio_rst_n;
	struct gpio_desc *gpio_slp_n;
	struct gpio_desc *gpio_mode_sel_n;
	bool enabled;
};

static int ps8640_regr(struct i2c_client *client, u8 reg, u8 *value)
{
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if (ret <= 0) {
		dev_err(&client->dev, "Failed to send i2c command, ret=%d\n",
			ret);
		return ret;
	}

	ret = i2c_master_recv(client, value, 1);
	if (ret <= 0) {
		dev_err(&client->dev, "Failed to recv i2c data, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int ps8640_regw(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;
	char buf[2];

	buf[0] = reg;
	buf[1] = value;
	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret <= 0) {
		dev_err(&client->dev, "Failed to send i2c command, ret=%d\n",
			ret);
		return ret;
	}

	return 0;
}

static bool ps8640_check_valid_id(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->page[4];
	u8 rev_id_low, rev_id_high, chip_id_low, chip_id_high;
	int retry_cnt = 0;

	do {
		ps8640_regr(client, PAGE4_CHIP_H, &chip_id_high);
		if (chip_id_high != 0x30)
			dev_info(&client->dev, "chip_id_high = 0x%x\n",
				 chip_id_high);
	} while ((retry_cnt++ < 2) && (chip_id_high != 0x30));

	ps8640_regr(client, PAGE4_REV_L, &rev_id_low);
	ps8640_regr(client, PAGE4_REV_H, &rev_id_high);
	ps8640_regr(client, PAGE4_CHIP_L, &chip_id_low);

	if ((rev_id_low == 0x00) && (rev_id_high == 0x0a) &&
	    (chip_id_low == 0x00) && (chip_id_high == 0x30))
		return true;

	return false;
}

static void ps8640_show_mcu_fw_version(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->page[5];
	u8 major_ver, minor_ver;

	ps8640_regr(client, 0x4, &major_ver);
	ps8640_regr(client, 0x5, &minor_ver);

	DRM_INFO_ONCE("ps8640 rom fw version %d.%d\n", major_ver, minor_ver);
}

static int ps8640_bdg_enable(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->page[3];

	if (ps8640_check_valid_id(ps_bridge)) {
		ps8640_regw(client, PAGE3_SET_ADD, VDO_CTL_ADD);
		ps8640_regw(client, PAGE3_SET_VAL, VDO_DIS);
		ps8640_regw(client, PAGE3_SET_ADD, VDO_CTL_ADD);
		ps8640_regw(client, PAGE3_SET_VAL, VDO_DIS);
		return 0;
	}

	return -EINVAL;
}

static void ps8640_prepare(struct ps8640 *ps_bridge)
{
	struct i2c_client *client = ps_bridge->page[2];
	int err, retry_cnt = 0;
	u8 set_vdo_done;

	if (ps_bridge->enabled)
		return;

	err = drm_panel_prepare(ps_bridge->panel);
	if (err < 0) {
		DRM_ERROR("failed to prepare panel: %d\n", err);
		return;
	}

	/* delay for power stable */
	usleep_range(500, 700);

	err = regulator_enable(ps_bridge->pwr_1v2_supply);
	if (err < 0) {
		DRM_ERROR("failed to enable vdd12-supply: %d\n", err);
		goto err_panel_unprepare;
	}

	err = regulator_enable(ps_bridge->pwr_3v3_supply);
	if (err < 0) {
		DRM_ERROR("failed to enable vdd33-supply: %d\n", err);
		goto err_regulator_disable;
	}

	gpiod_set_value(ps_bridge->gpio_slp_n, 1);
	gpiod_set_value(ps_bridge->gpio_rst_n, 0);
	usleep_range(500, 700);
	gpiod_set_value(ps_bridge->gpio_rst_n, 1);

	do {
		msleep(50);
		ps8640_regr(client, PAGE2_GPIO_H, &set_vdo_done);
	} while ((retry_cnt++ < 70) && ((set_vdo_done & PS_GPIO9) != PS_GPIO9));

	ps8640_show_mcu_fw_version(ps_bridge);
	ps_bridge->enabled = true;

	return;

err_regulator_disable:
	regulator_disable(ps_bridge->pwr_1v2_supply);
err_panel_unprepare:
	drm_panel_unprepare(ps_bridge->panel);
}

static void ps8640_pre_enable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);

	ps8640_prepare(ps_bridge);
}

static void ps8640_enable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);
	int err;

	ps8640_bdg_enable(ps_bridge);

	err = drm_panel_enable(ps_bridge->panel);
	if (err < 0)
		DRM_ERROR("failed to enable panel: %d\n", err);
}

static void ps8640_disable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);
	int err;

	if (!ps_bridge->enabled)
		return;

	ps_bridge->enabled = false;

	err = drm_panel_disable(ps_bridge->panel);
	if (err < 0)
		DRM_ERROR("failed to disable panel: %d\n", err);

	regulator_disable(ps_bridge->pwr_1v2_supply);
	regulator_disable(ps_bridge->pwr_3v3_supply);
	gpiod_set_value(ps_bridge->gpio_rst_n, 0);
	gpiod_set_value(ps_bridge->gpio_slp_n, 0);
}

static void ps8640_post_disable(struct drm_bridge *bridge)
{
	struct ps8640 *ps_bridge = bridge_to_ps8640(bridge);
	int err;

	err = drm_panel_unprepare(ps_bridge->panel);
	if (err)
		DRM_ERROR("failed to unprepare panel: %d\n", err);
}

static int ps8640_get_modes(struct drm_connector *connector)
{
	struct ps8640 *ps_bridge = connector_to_ps8640(connector);
	struct i2c_client *client = ps_bridge->page[2];

	ps8640_prepare(ps_bridge);
	ps8640_regw(client, PAGE2_I2C_BYPASS, I2C_BYPASS_EN);

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
		DRM_ERROR("Failed to initialize connector with drm: %d\n", ret);
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
	struct device_node *port, *out_ep;
	struct device_node *panel_node = NULL;
	int i, ret;

	ps_bridge = devm_kzalloc(dev, sizeof(*ps_bridge), GFP_KERNEL);
	if (!ps_bridge)
		return -ENOMEM;

	/* port@1 is ps8640 output port */
	port = of_graph_get_port_by_id(np, 1);
	if (port) {
		out_ep = of_get_child_by_name(port, "endpoint");
		of_node_put(port);
		if (out_ep) {
			panel_node = of_graph_get_remote_port_parent(out_ep);
			of_node_put(out_ep);
		}
	}
	if (panel_node) {
		ps_bridge->panel = of_drm_find_panel(panel_node);
		of_node_put(panel_node);
		if (!ps_bridge->panel)
			return -EPROBE_DEFER;
	}

	ps_bridge->page[0] = client;
	for (i = 1; i < 6; i++)
		ps_bridge->page[i] = i2c_new_dummy(client->adapter,
						   client->addr + i);

	ps_bridge->pwr_3v3_supply = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(ps_bridge->pwr_3v3_supply)) {
		ret = PTR_ERR(ps_bridge->pwr_3v3_supply);
		dev_err(dev, "cannot get vdd33 supply: %d\n", ret);
		return ret;
	}

	ps_bridge->pwr_1v2_supply = devm_regulator_get(dev, "vdd12");
	if (IS_ERR(ps_bridge->pwr_1v2_supply)) {
		ret = PTR_ERR(ps_bridge->pwr_1v2_supply);
		dev_err(dev, "cannot get vdd12 supply: %d\n", ret);
		return ret;
	}

	ps_bridge->gpio_mode_sel_n = devm_gpiod_get(&client->dev, "mode-sel",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(ps_bridge->gpio_mode_sel_n)) {
		ret = PTR_ERR(ps_bridge->gpio_mode_sel_n);
		dev_err(dev, "cannot get gpio_mode_sel_n %d\n", ret);
		return ret;
	}

	ps_bridge->gpio_slp_n = devm_gpiod_get(&client->dev, "sleep",
					       GPIOD_OUT_HIGH);
	if (IS_ERR(ps_bridge->gpio_slp_n)) {
		ret = PTR_ERR(ps_bridge->gpio_slp_n);
		dev_err(dev, "cannot get gpio_slp_n: %d\n", ret);
		return ret;
	}

	ps_bridge->gpio_rst_n = devm_gpiod_get(&client->dev, "reset",
					       GPIOD_OUT_HIGH);
	if (IS_ERR(ps_bridge->gpio_rst_n)) {
		ret = PTR_ERR(ps_bridge->gpio_rst_n);
		dev_err(dev, "cannot get gpio_rst_n: %d\n", ret);
		return ret;
	}

	ps_bridge->bridge.funcs = &ps8640_bridge_funcs;
	ps_bridge->bridge.of_node = dev->of_node;
	ret = drm_bridge_add(&ps_bridge->bridge);
	if (ret) {
		dev_err(dev, "Failed to add bridge: %d\n", ret);
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
		.of_match_table = ps8640_match,
	},
};
module_i2c_driver(ps8640_driver);

MODULE_AUTHOR("Jitao Shi <jitao.shi@mediatek.com>");
MODULE_AUTHOR("CK Hu <ck.hu@mediatek.com>");
MODULE_DESCRIPTION("PARADE ps8640 DSI-eDP converter driver");
MODULE_LICENSE("GPL v2");
