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
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>


#define HDMI_CON0		0x00
#define RG_HDMITX_PLL_EN		BIT(31)
#define RG_HDMITX_PLL_FBKDIV		(0x7f << 24)
#define PLL_FBKDIV_SHIFT		24
#define RG_HDMITX_PLL_FBKSEL		(0x3 << 22)
#define PLL_FBKSEL_SHIFT		22
#define RG_HDMITX_PLL_PREDIV		(0x3 << 20)
#define PREDIV_SHIFT			20
#define RG_HDMITX_PLL_POSDIV		(0x3 << 18)
#define POSDIV_SHIFT			18
#define RG_HDMITX_PLL_RST_DLY		(0x3 << 16)
#define RG_HDMITX_PLL_IR		(0xf << 12)
#define PLL_IR_SHIFT			12
#define RG_HDMITX_PLL_IC		(0xf << 8)
#define PLL_IC_SHIFT			8
#define RG_HDMITX_PLL_BP		(0xf << 4)
#define PLL_BP_SHIFT			4
#define RG_HDMITX_PLL_BR		(0x3 << 2)
#define PLL_BR_SHIFT			2
#define RG_HDMITX_PLL_BC		(0x3 << 0)
#define PLL_BC_SHIFT			0
#define HDMI_CON1		0x04
#define RG_HDMITX_PLL_DIVEN		(0x7 << 29)
#define PLL_DIVEN_SHIFT			29
#define RG_HDMITX_PLL_AUTOK_EN		BIT(28)
#define RG_HDMITX_PLL_AUTOK_KF		(0x3 << 26)
#define RG_HDMITX_PLL_AUTOK_KS		(0x3 << 24)
#define RG_HDMITX_PLL_AUTOK_LOAD	BIT(23)
#define RG_HDMITX_PLL_BAND		(0x3f << 16)
#define RG_HDMITX_PLL_REF_SEL		BIT(15)
#define RG_HDMITX_PLL_BIAS_EN		BIT(14)
#define RG_HDMITX_PLL_BIAS_LPF_EN	BIT(13)
#define RG_HDMITX_PLL_TXDIV_EN		BIT(12)
#define RG_HDMITX_PLL_TXDIV		(0x3 << 10)
#define PLL_TXDIV_SHIFT			10
#define RG_HDMITX_PLL_LVROD_EN		BIT(9)
#define RG_HDMITX_PLL_MONVC_EN		BIT(8)
#define RG_HDMITX_PLL_MONCK_EN		BIT(7)
#define RG_HDMITX_PLL_MONREF_EN		BIT(6)
#define RG_HDMITX_PLL_TST_EN		BIT(5)
#define RG_HDMITX_PLL_TST_CK_EN		BIT(4)
#define RG_HDMITX_PLL_TST_SEL		(0xf << 0)
#define HDMI_CON2		0x08
#define RGS_HDMITX_PLL_AUTOK_BAND	(0x7f << 8)
#define RGS_HDMITX_PLL_AUTOK_FAIL	BIT(1)
#define RG_HDMITX_EN_TX_CKLDO		BIT(0)
#define HDMI_CON3		0x0c
#define RG_HDMITX_SER_EN		(0xf << 28)
#define RG_HDMITX_PRD_EN		(0xf << 24)
#define RG_HDMITX_PRD_IMP_EN		(0xf << 20)
#define RG_HDMITX_DRV_EN		(0xf << 16)
#define RG_HDMITX_DRV_IMP_EN		(0xf << 12)
#define DRV_IMP_EN_SHIFT		12
#define RG_HDMITX_MHLCK_FORCE		BIT(10)
#define RG_HDMITX_MHLCK_PPIX_EN		BIT(9)
#define RG_HDMITX_MHLCK_EN		BIT(8)
#define RG_HDMITX_SER_DIN_SEL		(0xf << 4)
#define RG_HDMITX_SER_5T1_BIST_EN	BIT(3)
#define RG_HDMITX_SER_BIST_TOG		BIT(2)
#define RG_HDMITX_SER_DIN_TOG		BIT(1)
#define RG_HDMITX_SER_CLKDIG_INV	BIT(0)
#define HDMI_CON4		0x10
#define RG_HDMITX_PRD_IBIAS_CLK		(0xf << 24)
#define RG_HDMITX_PRD_IBIAS_D2		(0xf << 16)
#define RG_HDMITX_PRD_IBIAS_D1		(0xf << 8)
#define RG_HDMITX_PRD_IBIAS_D0		(0xf << 0)
#define PRD_IBIAS_CLK_SHIFT		24
#define PRD_IBIAS_D2_SHIFT		16
#define PRD_IBIAS_D1_SHIFT		8
#define PRD_IBIAS_D0_SHIFT		0
#define HDMI_CON5		0x14
#define RG_HDMITX_DRV_IBIAS_CLK		(0x3f << 24)
#define RG_HDMITX_DRV_IBIAS_D2		(0x3f << 16)
#define RG_HDMITX_DRV_IBIAS_D1		(0x3f << 8)
#define RG_HDMITX_DRV_IBIAS_D0		(0x3f << 0)
#define DRV_IBIAS_CLK_SHIFT		24
#define DRV_IBIAS_D2_SHIFT		16
#define DRV_IBIAS_D1_SHIFT		8
#define DRV_IBIAS_D0_SHIFT		0
#define HDMI_CON6		0x18
#define RG_HDMITX_DRV_IMP_CLK		(0x3f << 24)
#define RG_HDMITX_DRV_IMP_D2		(0x3f << 16)
#define RG_HDMITX_DRV_IMP_D1		(0x3f << 8)
#define RG_HDMITX_DRV_IMP_D0		(0x3f << 0)
#define DRV_IMP_CLK_SHIFT		24
#define DRV_IMP_D2_SHIFT		16
#define DRV_IMP_D1_SHIFT		8
#define DRV_IMP_D0_SHIFT		0
#define HDMI_CON7		0x1c
#define RG_HDMITX_MHLCK_DRV_IBIAS	(0x1f << 27)
#define RG_HDMITX_SER_DIN		(0x3ff << 16)
#define RG_HDMITX_CHLDC_TST		(0xf << 12)
#define RG_HDMITX_CHLCK_TST		(0xf << 8)
#define RG_HDMITX_RESERVE		(0xff << 0)
#define HDMI_CON8		0x20
#define RGS_HDMITX_2T1_LEV		(0xf << 16)
#define RGS_HDMITX_2T1_EDG		(0xf << 12)
#define RGS_HDMITX_5T1_LEV		(0xf << 8)
#define RGS_HDMITX_5T1_EDG		(0xf << 4)
#define RGS_HDMITX_PLUG_TST		BIT(0)

struct mtk_hdmi_pll {
	struct clk_hw hw;
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	unsigned long rate;
	u32 ibias;
	u8 drv_imp_clk;
	u8 drv_imp_d2;
	u8 drv_imp_d1;
	u8 drv_imp_d0;
 };

static inline struct mtk_hdmi_pll * to_mtk_hdmi_pll(struct clk_hw *hw)
{
	return container_of(hw, struct mtk_hdmi_pll, hw);
}

static inline void mtk_hdmi_pll_mask(struct mtk_hdmi_pll *hdmi_pll, u32 offset,
			      u32 val, u32 mask)
{
	u32 tmp = readl(hdmi_pll->regs  + offset) & ~mask;

	tmp |= (val & mask);
	writel(tmp, hdmi_pll->regs + offset);
}

static int mtk_hdmi_pll_enable(struct clk_hw *hw)
{
	struct mtk_hdmi_pll *hdmi_pll = to_mtk_hdmi_pll(hw);

	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, RG_HDMITX_PLL_AUTOK_EN,
			  RG_HDMITX_PLL_AUTOK_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0, RG_HDMITX_PLL_POSDIV,
			  RG_HDMITX_PLL_POSDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, 0, RG_HDMITX_MHLCK_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, RG_HDMITX_PLL_BIAS_EN,
			  RG_HDMITX_PLL_BIAS_EN);
	udelay(100);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0, RG_HDMITX_PLL_EN,
			  RG_HDMITX_PLL_EN);
	udelay(100);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, RG_HDMITX_PLL_BIAS_LPF_EN,
			  RG_HDMITX_PLL_BIAS_LPF_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, RG_HDMITX_PLL_TXDIV_EN,
			  RG_HDMITX_PLL_TXDIV_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, RG_HDMITX_SER_EN,
			  RG_HDMITX_SER_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, RG_HDMITX_PRD_EN,
			  RG_HDMITX_PRD_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, RG_HDMITX_DRV_EN,
			  RG_HDMITX_DRV_EN);
	udelay(100);

	return 0;
}

static void mtk_hdmi_pll_disable(struct clk_hw *hw)
{
	struct mtk_hdmi_pll *hdmi_pll = to_mtk_hdmi_pll(hw);

	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, 0, RG_HDMITX_DRV_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, 0, RG_HDMITX_PRD_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, 0, RG_HDMITX_SER_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, 0, RG_HDMITX_PLL_TXDIV_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, 0, RG_HDMITX_PLL_BIAS_LPF_EN);
	udelay(100);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0, 0, RG_HDMITX_PLL_EN);
	udelay(100);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, 0, RG_HDMITX_PLL_BIAS_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0, 0, RG_HDMITX_PLL_POSDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1, 0, RG_HDMITX_PLL_AUTOK_EN);
	udelay(100);
}


static int mtk_hdmi_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct mtk_hdmi_pll *hdmi_pll = to_mtk_hdmi_pll(hw);
	unsigned int pre_div;
	unsigned int div;

	dev_info(hdmi_pll->dev, "set rate : %lu, parent: %lu\n", rate, parent_rate);

	if (rate <= 27000000) {
		pre_div = 0;
		div = 3;
	}
	else if (rate <= 74250000) {
		pre_div = 1;
		div = 2;
	}
	else {
		pre_div = 1;
		div = 1;
	}
	mtk_hdmi_pll_disable(hw);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0,
			  (pre_div << PREDIV_SHIFT),
			  RG_HDMITX_PLL_PREDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0, RG_HDMITX_PLL_POSDIV,
			  RG_HDMITX_PLL_POSDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0,
			  (0x1 << PLL_IC_SHIFT) | (0x1 << PLL_IR_SHIFT),
			  RG_HDMITX_PLL_IC | RG_HDMITX_PLL_IR);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1,
			  (div << PLL_TXDIV_SHIFT),
			  RG_HDMITX_PLL_TXDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0,
			  (0x1 << PLL_FBKSEL_SHIFT) |
			  (19 << PLL_FBKDIV_SHIFT),
			  RG_HDMITX_PLL_FBKSEL | RG_HDMITX_PLL_FBKDIV);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON1,
			  (0x2 << PLL_DIVEN_SHIFT),
			  RG_HDMITX_PLL_DIVEN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON0,
			  (0xc << PLL_BP_SHIFT) |
			  (0x2 << PLL_BC_SHIFT) |
			  (0x1 << PLL_BR_SHIFT),
			  RG_HDMITX_PLL_BP | RG_HDMITX_PLL_BC |
			  RG_HDMITX_PLL_BR);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3, 0, RG_HDMITX_PRD_IMP_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON4,
			  (0x3 << PRD_IBIAS_CLK_SHIFT) |
			  (0x3 << PRD_IBIAS_D2_SHIFT) |
			  (0x3 << PRD_IBIAS_D1_SHIFT) |
			  (0x3 << PRD_IBIAS_D0_SHIFT),
			  RG_HDMITX_PRD_IBIAS_CLK |
			  RG_HDMITX_PRD_IBIAS_D2 |
			  RG_HDMITX_PRD_IBIAS_D1 |
			  RG_HDMITX_PRD_IBIAS_D0);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON3,
			  (0x0 << DRV_IMP_EN_SHIFT),
			  RG_HDMITX_DRV_IMP_EN);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON6,
			  (hdmi_pll->drv_imp_clk << DRV_IMP_CLK_SHIFT) |
			  (hdmi_pll->drv_imp_d2 << DRV_IMP_D2_SHIFT) |
			  (hdmi_pll->drv_imp_d1 << DRV_IMP_D1_SHIFT) |
			  (hdmi_pll->drv_imp_d0 << DRV_IMP_D0_SHIFT),
			  RG_HDMITX_DRV_IMP_CLK | RG_HDMITX_DRV_IMP_D2 |
			  RG_HDMITX_DRV_IMP_D1 | RG_HDMITX_DRV_IMP_D0);
	mtk_hdmi_pll_mask(hdmi_pll, HDMI_CON5,
			  (hdmi_pll->ibias << DRV_IBIAS_CLK_SHIFT) |
			  (hdmi_pll->ibias << DRV_IBIAS_D2_SHIFT) |
			  (hdmi_pll->ibias << DRV_IBIAS_D1_SHIFT) |
			  (hdmi_pll->ibias << DRV_IBIAS_D0_SHIFT),
			  RG_HDMITX_DRV_IBIAS_CLK | RG_HDMITX_DRV_IBIAS_D2 |
			  RG_HDMITX_DRV_IBIAS_D1 | RG_HDMITX_DRV_IBIAS_D0);
	mtk_hdmi_pll_enable(hw);
	return 0;
}

static long mtk_hdmi_pll_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct mtk_hdmi_pll *hdmi_pll = to_mtk_hdmi_pll(hw);

	hdmi_pll->rate = rate;
	if(rate <= 74250000)
		*parent_rate = rate;
	else
		*parent_rate = rate/2;

	return rate;
}

static unsigned long mtk_hdmi_pll_recalc_rate(struct clk_hw *hw,
			  unsigned long parent_rate)
{
	struct mtk_hdmi_pll *hdmi_pll = to_mtk_hdmi_pll(hw);
  	return hdmi_pll->rate;
}


 static const struct clk_ops hdmi_pll_ops = {
	.enable = mtk_hdmi_pll_enable,
	.disable = mtk_hdmi_pll_disable,
	.set_rate = mtk_hdmi_pll_set_rate,
	.round_rate = mtk_hdmi_pll_round_rate,
	.recalc_rate = mtk_hdmi_pll_recalc_rate,
 };

static struct clk *mtk_hdmi_pll_clk_src_get(struct of_phandle_args * args,void * data)
{
	struct mtk_hdmi_pll *hdmi_pll = data;
	return hdmi_pll->clk;
}

static const struct clk_init_data mtk_hdmi_pll_init = {
	.name = "hdmi_ana_pll",
	.parent_names = (const char *[]){
		[0] = "hdmi_ref",
	},
	.num_parents = 1,
	.ops = &hdmi_pll_ops,
	.flags = CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
};

static int mtk_hdmi_pll_probe(struct platform_device *pdev)
{
	struct mtk_hdmi_pll *hdmi_pll;
	struct device *dev = &pdev->dev;
	struct resource *mem;
	int ret;

	hdmi_pll = devm_kzalloc(dev, sizeof(*hdmi_pll), GFP_KERNEL);
	if(!hdmi_pll) {
		dev_err(dev, "Failed to alloc mem\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi_pll->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(hdmi_pll->regs)) {
		dev_err(dev, "Failed to get memory resource: %d\n", ret);
		return PTR_ERR(hdmi_pll->regs);
	}

	ret = of_property_read_u32(dev->of_node, "mediatek,ibias",
				   &hdmi_pll->ibias);
	if (ret) {
		dev_err(dev, "Failed to get ibias: %d\n", ret);
		return ret;
	}

	hdmi_pll->dev = dev;
	hdmi_pll->drv_imp_clk = 0x30;
	hdmi_pll->drv_imp_d2 = 0x30;
	hdmi_pll->drv_imp_d1 = 0x30;
	hdmi_pll->drv_imp_d0 = 0x30;
	hdmi_pll->hw.init = &mtk_hdmi_pll_init;

	hdmi_pll->clk = devm_clk_register(dev, &hdmi_pll->hw);
	if(IS_ERR(hdmi_pll->clk)) {
		dev_err(dev, "Failed to register hdmi pll clk\n");
		return PTR_ERR(hdmi_pll->clk);
	}

	ret = of_clk_add_provider(dev->of_node, mtk_hdmi_pll_clk_src_get, hdmi_pll);
	if(ret) {
		dev_err(dev, "Failed to add hdmi pll clk provider\n");
		return ret;
	}

	dev_info(dev, "hdmi pll probe success\n");
	return 0;
}

static int mtk_hdmi_pll_exit(struct platform_device * pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static const struct of_device_id mtk_hdmi_pll_of_ids[] = {
	{ .compatible = "mediatek,mt8173-hdmi-pll", },
	{}
};

static struct platform_driver mtk_hdmi_pll_driver = {
	.probe = mtk_hdmi_pll_probe,
	.remove = mtk_hdmi_pll_exit,
	.driver = {
		.name = "mediatek-hdmi-pll",
		.of_match_table = mtk_hdmi_pll_of_ids,
	},
};

module_platform_driver(mtk_hdmi_pll_driver);
MODULE_AUTHOR("Jie Qiu <jie.qiu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek HDMI i2c Driver");
MODULE_LICENSE("GPL v2");
