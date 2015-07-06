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
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "mediatek_drm_crtc.h"
#include "mediatek_drm_ddp.h"
#include "mediatek_drm_ddp_comp.h"


#define DISP_REG_CONFIG_DISP_OVL0_MOUT_EN	0x040
#define DISP_REG_CONFIG_DISP_OVL1_MOUT_EN	0x044
#define DISP_REG_CONFIG_DISP_OD_MOUT_EN		0x048
#define DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN	0x04c
#define DISP_REG_CONFIG_DISP_UFOE_MOUT_EN	0x050
#define DISP_REG_CONFIG_DISP_COLOR0_SEL_IN	0x084
#define DISP_REG_CONFIG_DISP_COLOR1_SEL_IN	0x088
#define DISP_REG_CONFIG_DPI_SEL_IN		0x0ac
#define DISP_REG_CONFIG_DISP_RDMA1_MOUT_EN	0x0c8
#define DISP_REG_CONFIG_MMSYS_CG_CON0		0x100

#define DISP_REG_CONFIG_MUTEX_EN(n)      (0x20 + 0x20 * n)
#define DISP_REG_CONFIG_MUTEX_MOD(n)     (0x2c + 0x20 * n)
#define DISP_REG_CONFIG_MUTEX_SOF(n)     (0x30 + 0x20 * n)

#define MUTEX_MOD_OVL0		11
#define MUTEX_MOD_OVL1		12
#define MUTEX_MOD_RDMA0		13
#define MUTEX_MOD_RDMA1		14
#define MUTEX_MOD_COLOR0	18
#define MUTEX_MOD_COLOR1	19
#define MUTEX_MOD_AAL		20
#define MUTEX_MOD_GAMMA		21
#define MUTEX_MOD_UFOE		22
#define MUTEX_MOD_PWM0		23
#define MUTEX_MOD_OD		25

#define MUTEX_SOF_DSI0		1
#define MUTEX_SOF_DPI0		3

#define OVL0_MOUT_EN_COLOR0	0x1
#define OD_MOUT_EN_RDMA0	0x1
#define UFOE_MOUT_EN_DSI0	0x1
#define COLOR0_SEL_IN_OVL0	0x1
#define OVL1_MOUT_EN_COLOR1	0x1
#define GAMMA_MOUT_EN_RDMA1	0x1
#define RDMA1_MOUT_DPI0		0x2
#define DPI0_SEL_IN_RDMA1	0x1
#define COLOR1_SEL_IN_OVL1	0x1

struct mtk_ddp {
	struct device			*dev;
	struct drm_device		*drm_dev;
	struct mediatek_drm_crtc	*crtc;
	int				pipe;

	struct clk			*mutex_disp_clk;

	void __iomem			*config_regs;
	void __iomem			*mutex_regs;
};


struct mtk_ddp_comp_in_sel {
	unsigned int reg_addr;
	unsigned int mask;
	unsigned int value;
};

static struct mtk_ddp_comp_in_sel
	comp_in_sel[DDP_COMPONENT_TYPE_MAX][DDP_COMPONENT_TYPE_MAX];

struct mtk_ddp_comp_out_enable {
	unsigned int reg_addr;
	unsigned int value;
};

static struct mtk_ddp_comp_out_enable
	comp_out_enable[DDP_COMPONENT_TYPE_MAX][DDP_COMPONENT_TYPE_MAX];

static unsigned int mutex_mod[DDP_COMPONENT_TYPE_MAX];

void mtk_ddp_add_comp_to_path(struct device *dev,
		unsigned int pipe, unsigned int cur, unsigned int next)
{
	struct mtk_ddp *ddp = dev_get_drvdata(dev);
	unsigned int addr, reg;

	addr = comp_out_enable[cur][next].reg_addr;
	reg = readl(ddp->config_regs + addr) | comp_out_enable[cur][next].value;
	writel(reg, ddp->config_regs + addr);

	addr = comp_in_sel[cur][next].reg_addr;
	reg = readl(ddp->config_regs + addr) & ~(comp_in_sel[cur][next].mask);
	reg |= comp_in_sel[cur][next].value;
	writel(reg, ddp->config_regs + addr);

	reg = readl(ddp->mutex_regs + DISP_REG_CONFIG_MUTEX_MOD(pipe));
	reg |= (1 << mutex_mod[cur]) | (1 << mutex_mod[next]);
	writel(reg, ddp->mutex_regs + DISP_REG_CONFIG_MUTEX_MOD(pipe));

	if (next == DDP_COMPONENT_DPI0)
		reg = MUTEX_SOF_DPI0;
	else
		reg = MUTEX_SOF_DSI0;

	writel(reg, ddp->mutex_regs + DISP_REG_CONFIG_MUTEX_SOF(pipe));
	writel(1, ddp->mutex_regs + DISP_REG_CONFIG_MUTEX_EN(pipe));

}

void mtk_ddp_clock_on(struct device *dev)
{
	struct mtk_ddp *ddp = dev_get_drvdata(dev);
	int ret;

	/* disp_mtcmos */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		DRM_ERROR("failed to get_sync(%d)\n", ret);

	ret = clk_prepare_enable(ddp->mutex_disp_clk);
	if (ret != 0)
		DRM_ERROR("clk_prepare_enable(mutex_disp_clk) error!\n");
}

void mtk_ddp_clock_off(struct device *dev)
{
	struct mtk_ddp *ddp = dev_get_drvdata(dev);

	clk_disable_unprepare(ddp->mutex_disp_clk);

	/* disp_mtcmos */
	pm_runtime_put_sync(dev);
}

static void mtk_ddp_comp_table_setup(void)
{
	comp_out_enable[DDP_COMPONENT_OVL0][DDP_COMPONENT_COLOR0].
		reg_addr = DISP_REG_CONFIG_DISP_OVL0_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_OVL0][DDP_COMPONENT_COLOR0].
		value = OVL0_MOUT_EN_COLOR0;

	comp_out_enable[DDP_COMPONENT_OD][DDP_COMPONENT_RDMA0].
		reg_addr = DISP_REG_CONFIG_DISP_OD_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_OD][DDP_COMPONENT_RDMA0].
		value = OD_MOUT_EN_RDMA0;

	comp_out_enable[DDP_COMPONENT_UFOE][DDP_COMPONENT_DSI0].
		reg_addr = DISP_REG_CONFIG_DISP_UFOE_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_UFOE][DDP_COMPONENT_DSI0].
		value = UFOE_MOUT_EN_DSI0;

	comp_in_sel[DDP_COMPONENT_OVL0][DDP_COMPONENT_COLOR0].
		reg_addr = DISP_REG_CONFIG_DISP_COLOR0_SEL_IN;
	comp_in_sel[DDP_COMPONENT_OVL0][DDP_COMPONENT_COLOR0].
		mask = COLOR0_SEL_IN_OVL0;
	comp_in_sel[DDP_COMPONENT_OVL0][DDP_COMPONENT_COLOR0].
		value = COLOR0_SEL_IN_OVL0;

	comp_out_enable[DDP_COMPONENT_OVL1][DDP_COMPONENT_COLOR1].
		reg_addr = DISP_REG_CONFIG_DISP_OVL1_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_OVL1][DDP_COMPONENT_COLOR1].
		value = OVL1_MOUT_EN_COLOR1;

	comp_out_enable[DDP_COMPONENT_GAMMA][DDP_COMPONENT_RDMA1].
		reg_addr = DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_GAMMA][DDP_COMPONENT_RDMA1].
		value = GAMMA_MOUT_EN_RDMA1;

	comp_out_enable[DDP_COMPONENT_RDMA1][DDP_COMPONENT_DPI0].
		reg_addr = DISP_REG_CONFIG_DISP_RDMA1_MOUT_EN;
	comp_out_enable[DDP_COMPONENT_RDMA1][DDP_COMPONENT_DPI0].
		value = RDMA1_MOUT_DPI0;

	comp_in_sel[DDP_COMPONENT_RDMA1][DDP_COMPONENT_DPI0].
		reg_addr = DISP_REG_CONFIG_DPI_SEL_IN;
	comp_in_sel[DDP_COMPONENT_RDMA1][DDP_COMPONENT_DPI0].
		mask = DPI0_SEL_IN_RDMA1;
	comp_in_sel[DDP_COMPONENT_RDMA1][DDP_COMPONENT_DPI0].
		value = DPI0_SEL_IN_RDMA1;

	comp_in_sel[DDP_COMPONENT_OVL1][DDP_COMPONENT_COLOR1].
		reg_addr = DISP_REG_CONFIG_DISP_COLOR1_SEL_IN;
	comp_in_sel[DDP_COMPONENT_OVL1][DDP_COMPONENT_COLOR1].
		mask = COLOR1_SEL_IN_OVL1;
	comp_in_sel[DDP_COMPONENT_OVL1][DDP_COMPONENT_COLOR1].
		value = COLOR1_SEL_IN_OVL1;

	mutex_mod[DDP_COMPONENT_AAL] = MUTEX_MOD_AAL;
	mutex_mod[DDP_COMPONENT_COLOR0] = MUTEX_MOD_COLOR0;
	mutex_mod[DDP_COMPONENT_COLOR1] = MUTEX_MOD_COLOR1;
	mutex_mod[DDP_COMPONENT_GAMMA] = MUTEX_MOD_GAMMA;
	mutex_mod[DDP_COMPONENT_OD] = MUTEX_MOD_OD;
	mutex_mod[DDP_COMPONENT_OVL0] = MUTEX_MOD_OVL0;
	mutex_mod[DDP_COMPONENT_OVL1] = MUTEX_MOD_OVL1;
	mutex_mod[DDP_COMPONENT_PWM0] = MUTEX_MOD_PWM0;
	mutex_mod[DDP_COMPONENT_RDMA0] = MUTEX_MOD_RDMA0;
	mutex_mod[DDP_COMPONENT_RDMA1] = MUTEX_MOD_RDMA1;
	mutex_mod[DDP_COMPONENT_UFOE] = MUTEX_MOD_UFOE;
}

static int mtk_ddp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ddp *ddp;
	struct resource *regs;

	if (!dev->of_node)
		return -ENODEV;

	ddp = devm_kzalloc(dev, sizeof(*ddp), GFP_KERNEL);
	if (!ddp)
		return -ENOMEM;

	ddp->mutex_disp_clk = devm_clk_get(dev, "mutex_disp");
	if (IS_ERR(ddp->mutex_disp_clk))
		return PTR_ERR(ddp->mutex_disp_clk);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ddp->config_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->config_regs))
		return PTR_ERR(ddp->config_regs);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ddp->mutex_regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->mutex_regs))
		return PTR_ERR(ddp->mutex_regs);

	platform_set_drvdata(pdev, ddp);

	pm_runtime_enable(dev);

	mtk_ddp_comp_table_setup();

	return 0;
}

static int mtk_ddp_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id ddp_driver_dt_match[] = {
	{ .compatible = "mediatek,mt8173-ddp" },
	{},
};
MODULE_DEVICE_TABLE(of, ddp_driver_dt_match);

struct platform_driver mtk_ddp_driver = {
	.probe		= mtk_ddp_probe,
	.remove		= mtk_ddp_remove,
	.driver		= {
		.name	= "mediatek-ddp",
		.owner	= THIS_MODULE,
		.of_match_table = ddp_driver_dt_match,
	},
};

module_platform_driver(mtk_ddp_driver);

