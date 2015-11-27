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

#ifndef __ANX78xx_H
#define __ANX78xx_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>

#include <drm/drm_crtc.h>

struct anx78xx_platform_data {
	struct gpio_desc *gpiod_cable_det;
	struct gpio_desc *gpiod_pd;
	struct gpio_desc *gpiod_reset;
	struct gpio_desc *gpiod_v10;
};

struct anx78xx {
	struct drm_bridge bridge;
	struct i2c_client *client;
	struct anx78xx_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
};

void anx78xx_poweron(struct anx78xx *data);
void anx78xx_poweroff(struct anx78xx *data);
bool anx78xx_cable_is_detected(struct anx78xx *anx78xx);

#endif
