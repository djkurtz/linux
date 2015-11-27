/*
 * Copyright(c) 2015, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/types.h>

#include "anx78xx.h"
#include "slimport_tx_drv.h"

void anx78xx_poweron(struct anx78xx *anx78xx)
{
	struct anx78xx_platform_data *pdata = anx78xx->pdata;

	if (pdata->gpiod_v10) {
		gpiod_set_value_cansleep(pdata->gpiod_v10, 1);
		usleep_range(1000, 2000);
	}

	gpiod_set_value_cansleep(pdata->gpiod_reset, 0);
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(pdata->gpiod_pd, 0);
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(pdata->gpiod_reset, 1);
}

void anx78xx_poweroff(struct anx78xx *anx78xx)
{
	struct anx78xx_platform_data *pdata = anx78xx->pdata;

	if (pdata->gpiod_v10) {
		gpiod_set_value_cansleep(pdata->gpiod_v10, 0);
		usleep_range(1000, 2000);
	}

	gpiod_set_value_cansleep(pdata->gpiod_reset, 0);
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(pdata->gpiod_pd, 1);
	usleep_range(1000, 2000);
}

static int anx78xx_init_gpio(struct anx78xx *anx78xx)
{
	struct device *dev = &anx78xx->client->dev;
	struct anx78xx_platform_data *pdata = anx78xx->pdata;

	/* gpio for cable detection */
	pdata->gpiod_cable_det = devm_gpiod_get(dev, "cable-det", GPIOD_IN);
	if (IS_ERR(pdata->gpiod_cable_det)) {
		dev_err(dev, "unable to claim cable-det gpio\n");
		return PTR_ERR(pdata->gpiod_cable_det);
	}

	/* gpio for chip power down */
	pdata->gpiod_pd = devm_gpiod_get(dev, "pd", GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->gpiod_pd)) {
		dev_err(dev, "unable to claim pd gpio\n");
		return PTR_ERR(pdata->gpiod_pd);
	}

	/* gpio for chip reset */
	pdata->gpiod_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpiod_reset)) {
		dev_err(dev, "unable to claim reset gpio\n");
		return PTR_ERR(pdata->gpiod_reset);
	}

	/* gpio for V10 power control */
	pdata->gpiod_v10 = devm_gpiod_get_optional(dev, "v10", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpiod_v10)) {
		dev_err(dev, "unable to claim v10 gpio\n");
		return PTR_ERR(pdata->gpiod_v10);
	}

	return 0;
}

bool anx78xx_cable_is_detected(struct anx78xx *anx78xx)
{
	int i, count = 0;

	for (i = 0; i < 10; i++) {
		if (gpiod_get_value(anx78xx->pdata->gpiod_cable_det))
			count++;
		mdelay(5);
	}

	return (count > 5);
}

/*
 * HPD IRQ Event: HPD pulse width greater than 0.25ms but narrower than 2ms
 * Hot Unplug Event: HPD pulse stays low longer than 2ms.
 *
 * AP just monitor HPD pulse high in this irq. If HPD is high, the driver
 * will power on the chip, and then the driver controls when to power down
 * the chip, if HPD event is HPD IRQ, the driver deals with IRQ event from
 * downstream, finally, if HPD event is Hot Plug, the driver power down the
 * chip.
 */
static irqreturn_t anx78xx_cable_isr(int irq, void *data)
{
	struct anx78xx *anx78xx = data;

	queue_delayed_work(anx78xx->workqueue, &anx78xx->work, 0);

	return IRQ_HANDLED;
}

static void anx78xx_work_func(struct work_struct *work)
{
	struct anx78xx *anx78xx = container_of(work, struct anx78xx,
					       work.work);

	if (sp_main_process(anx78xx))
		queue_delayed_work(anx78xx->workqueue, &anx78xx->work,
				   msecs_to_jiffies(500));
	else
		cancel_delayed_work(&anx78xx->work);
}

static inline struct anx78xx *bridge_to_anx78xx(struct drm_bridge *bridge)
{
	return container_of(bridge, struct anx78xx, bridge);
}

static int anx78xx_bridge_attach(struct drm_bridge *bridge)
{
	return 0;
}

static bool anx78xx_bridge_mode_fixup(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	struct anx78xx *anx78xx = bridge_to_anx78xx(bridge);

	dev_dbg(&anx78xx->client->dev, "mode_fixup %d<%d; %d; %d\n",
		sp_get_link_bandwidth(anx78xx), SP_LINK_5P4G,
		mode->clock, mode->flags & DRM_MODE_FLAG_INTERLACE);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return false;

	/* Max 720p at 2.7 Ghz, one lane */
	if (sp_get_link_bandwidth(anx78xx) < SP_LINK_5P4G &&
	    mode->clock > 74250)
		return false;

	/* Max 1200p at 5.4 Ghz, one lane */
	if (mode->clock > 154000)
		return false;

	return true;
}

static void anx78xx_bridge_disable(struct drm_bridge *bridge) {}
static void anx78xx_bridge_post_disable(struct drm_bridge *bridge) {}
static void anx78xx_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode) {}
static void anx78xx_bridge_pre_enable(struct drm_bridge *bridge) {}
static void anx78xx_bridge_enable(struct drm_bridge *bridge) {}

static const struct drm_bridge_funcs anx78xx_bridge_funcs = {
	.attach		= anx78xx_bridge_attach,
	.mode_fixup	= anx78xx_bridge_mode_fixup,
	.disable	= anx78xx_bridge_disable,
	.post_disable	= anx78xx_bridge_post_disable,
	.mode_set	= anx78xx_bridge_mode_set,
	.pre_enable	= anx78xx_bridge_pre_enable,
	.enable		= anx78xx_bridge_enable,
};

static int anx78xx_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct anx78xx *anx78xx;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "i2c bus does not support the device\n");
		return -ENODEV;
	}

	anx78xx = devm_kzalloc(&client->dev, sizeof(*anx78xx), GFP_KERNEL);
	if (!anx78xx)
		return -ENOMEM;

	anx78xx->pdata = devm_kzalloc(&client->dev,
				      sizeof(struct anx78xx_platform_data),
				      GFP_KERNEL);
	if (!anx78xx->pdata)
		return -ENOMEM;

#ifdef CONFIG_OF
	anx78xx->bridge.of_node = client->dev.of_node;
#endif
	anx78xx->bridge.funcs = &anx78xx_bridge_funcs;
	ret = drm_bridge_add(&anx78xx->bridge);
	if (ret < 0) {
		dev_err(&client->dev, "add drm bridge failed\n");
		return ret;
	}

	anx78xx->client = client;

	i2c_set_clientdata(client, anx78xx);

	ret = anx78xx_init_gpio(anx78xx);
	if (ret) {
		dev_err(&client->dev, "failed to initialize gpios\n");
		goto fail_remove_bridge;
	}

	INIT_DELAYED_WORK(&anx78xx->work, anx78xx_work_func);

	anx78xx->workqueue = create_singlethread_workqueue("anx78xx_work");
	if (!anx78xx->workqueue) {
		dev_err(&client->dev, "failed to create work queue\n");
		ret = -ENOMEM;
		goto fail_remove_bridge;
	}

	ret = sp_system_init(anx78xx);
	if (ret) {
		dev_err(&client->dev, "failed to initialize anx78xx\n");
		goto fail_free_wq;
	}

	client->irq = gpiod_to_irq(anx78xx->pdata->gpiod_cable_det);
	if (client->irq < 0) {
		dev_err(&client->dev, "failed to get irq: %d\n", client->irq);
		ret = client->irq;
		goto fail_free_wq;
	}

	ret = request_threaded_irq(client->irq, NULL, anx78xx_cable_isr,
				   IRQF_TRIGGER_RISING |
				   IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "anx78xx", anx78xx);
	if (ret) {
		dev_err(&client->dev, "failed to request threaded irq\n");
		goto fail_free_wq;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "failed to set irq wake\n");
		goto fail_free_wq;
	}

	ret = enable_irq_wake(client->irq);
	if (ret) {
		dev_err(&client->dev, "failed to enable irq wake\n");
		goto fail_free_wq;
	}

	/* enable driver */
	queue_delayed_work(anx78xx->workqueue, &anx78xx->work, 0);

	return 0;

fail_free_wq:
	destroy_workqueue(anx78xx->workqueue);
fail_remove_bridge:
	drm_bridge_remove(&anx78xx->bridge);
	return ret;
}

static int anx78xx_i2c_remove(struct i2c_client *client)
{
	struct anx78xx *anx78xx = i2c_get_clientdata(client);

	destroy_workqueue(anx78xx->workqueue);
	drm_bridge_remove(&anx78xx->bridge);

	return 0;
}

static int anx78xx_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct anx78xx *anx78xx = i2c_get_clientdata(client);

	if (anx78xx_cable_is_detected(anx78xx)) {
		cancel_delayed_work_sync(&anx78xx->work);
		flush_workqueue(anx78xx->workqueue);
		anx78xx_poweroff(anx78xx);
		sp_variable_init(anx78xx);
	}

	return 0;
}

static int anx78xx_i2c_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct anx78xx *anx78xx = i2c_get_clientdata(client);

	queue_delayed_work(anx78xx->workqueue, &anx78xx->work, 0);

	return 0;
}

static SIMPLE_DEV_PM_OPS(anx78xx_i2c_pm_ops,
			anx78xx_i2c_suspend, anx78xx_i2c_resume);

static const struct i2c_device_id anx78xx_id[] = {
	{"anx7814", 0},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(i2c, anx78xx_id);

#ifdef CONFIG_OF
static const struct of_device_id anx78xx_match_table[] = {
	{.compatible = "analogix,anx7814",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, anx78xx_match_table);
#endif

static struct i2c_driver anx78xx_driver = {
	.driver = {
		   .name = "anx7814",
		   .pm = &anx78xx_i2c_pm_ops,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(anx78xx_match_table),
#endif
		   },
	.probe = anx78xx_i2c_probe,
	.remove = anx78xx_i2c_remove,
	.id_table = anx78xx_id,
};

module_i2c_driver(anx78xx_driver);

MODULE_DESCRIPTION("Slimport transmitter ANX78XX driver");
MODULE_AUTHOR("Junhua Xia <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.1");
