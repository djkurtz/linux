/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author:
 *  Chunfeng Yun <chunfeng.yun@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/usb/xhci_pdriver.h>

#include "xhci.h"
#include "xhci-mtk.h"

#define U3P_IP_PW_CTRL0	0x00
#define CTRL0_IP_SW_RST	BIT(0)

#define U3P_IP_PW_CTRL1	0x04
#define CTRL1_IP_HOST_PDN	BIT(0)

#define U3P_IP_PW_CTRL2	0x08
#define CTRL2_IP_DEV_PDN	BIT(0)

#define U3P_IP_PW_STS1	0x10
#define STS1_IP_SLEEP_STS	BIT(30)
#define STS1_U3_MAC_RST	BIT(16)
#define STS1_SYS125_RST	BIT(10)
#define STS1_REF_RST		BIT(8)
#define STS1_SYSPLL_STABLE	BIT(0)

#define U3P_IP_PW_STS2	0x14
#define STS2_U2_MAC_RST	BIT(0)

#define U3P_IP_XHCI_CAP	0x24
#define CAP_U3_PORT_NUM(p)	((p) & 0xff)
#define CAP_U2_PORT_NUM(p)	(((p) >> 8) & 0xff)

#define U3P_U3_CTRL_0P		0x30
#define CTRL_U3_PORT_HOST_SEL	BIT(2)
#define CTRL_U3_PORT_PDN	BIT(1)
#define CTRL_U3_PORT_DIS	BIT(0)

#define U3P_U2_CTRL_0P		0x50
#define CTRL_U2_PORT_HOST_SEL	BIT(2)
#define CTRL_U2_PORT_PDN	BIT(1)
#define CTRL_U2_PORT_DIS	BIT(0)

#define U3P_U2_PHY_PLL		0x7c
#define CTRL_U2_FORCE_PLL_STB	BIT(28)

#define U3P_U3_CTRL(p)	(U3P_U3_CTRL_0P + ((p) * 0x08))
#define U3P_U2_CTRL(p)	(U3P_U2_CTRL_0P + ((p) * 0x08))

#define PERI_WK_CTRL0		0x400
#define UWK_CTR0_0P_LS_PE	BIT(8)  /* posedge */
#define UWK_CTR0_0P_LS_NE	BIT(7)  /* negedge for 0p linestate*/
#define UWK_CTL1_1P_LS_C(x)	(((x) & 0xf) << 1)
#define UWK_CTL1_1P_LS_E	BIT(0)

#define PERI_WK_CTRL1		0x404
#define UWK_CTL1_IS_C(x)	(((x) & 0xf) << 26)
#define UWK_CTL1_IS_E		BIT(25)
#define UWK_CTL1_0P_LS_C(x)	(((x) & 0xf) << 21)
#define UWK_CTL1_0P_LS_E	BIT(20)
#define UWK_CTL1_IDDIG_C(x)	(((x) & 0xf) << 11)  /* cycle debounce */
#define UWK_CTL1_IDDIG_E	BIT(10) /* enable debounce */
#define UWK_CTL1_IDDIG_P	BIT(9)  /* polarity */
#define UWK_CTL1_0P_LS_P	BIT(7)
#define UWK_CTL1_IS_P		BIT(6)  /* polarity for ip sleep */

enum ssusb_wakeup_src {
	SSUSB_WK_IP_SLEEP = 1,
	SSUSB_WK_LINE_STATE = 2,
};

static int check_ip_clk_status(struct xhci_hcd_mtk *mtk)
{
	int ret;
	int u3_port_num;
	int u2_port_num;
	u32 xhci_cap;
	u32 val;
	void __iomem *ippc_base = mtk->ippc_base;

	xhci_cap = readl(ippc_base + U3P_IP_XHCI_CAP);
	u3_port_num = CAP_U3_PORT_NUM(xhci_cap);
	u2_port_num = CAP_U2_PORT_NUM(xhci_cap);

	ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS1, val,
			  (val & STS1_SYSPLL_STABLE), 100, 10000);
	if (ret) {
		dev_err(mtk->dev, "syspll is not stable!!!\n");
		return ret;
	}

	ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS1, val,
			  (val & STS1_REF_RST), 100, 10000);
	if (ret) {
		dev_err(mtk->dev, "ref_clk is still active!!!\n");
		return ret;
	}

	ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS1, val,
			   (val & STS1_SYS125_RST), 100, 10000);
	if (ret) {
		dev_err(mtk->dev, "sys125_ck is still active!!!\n");
		return ret;
	}

	if (u3_port_num) {
		ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS1, val,
				   (val & STS1_U3_MAC_RST), 100, 10000);
		if (ret) {
			dev_err(mtk->dev, "mac3_mac_ck is still active!!!\n");
			return ret;
		}
	}

	if (u2_port_num) {
		ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS2, val,
				   (val & STS2_U2_MAC_RST), 100, 10000);
		if (ret) {
			dev_err(mtk->dev, "mac2_sys_ck is still active!!!\n");
			return ret;
		}
	}

	return 0;
}

static int xhci_mtk_ports_enable(struct xhci_hcd_mtk *mtk)
{
	int i;
	u32 temp;
	int u3_port_num;
	int u2_port_num;
	void __iomem *ippc_base = mtk->ippc_base;

	temp = readl(ippc_base + U3P_IP_XHCI_CAP);
	u3_port_num = CAP_U3_PORT_NUM(temp);
	u2_port_num = CAP_U2_PORT_NUM(temp);
	dev_dbg(mtk->dev, "%s u2p:%d, u3p:%d\n", __func__,
			u2_port_num, u3_port_num);

	/* power on host ip */
	temp = readl(ippc_base + U3P_IP_PW_CTRL1);
	temp &= ~CTRL1_IP_HOST_PDN;
	writel(temp, ippc_base + U3P_IP_PW_CTRL1);

	/* power on and enable all u3 ports */
	for (i = 0; i < u3_port_num; i++) {
		temp = readl(ippc_base + U3P_U3_CTRL(i));
		temp &= ~(CTRL_U3_PORT_PDN | CTRL_U3_PORT_DIS);
		temp |= CTRL_U3_PORT_HOST_SEL;
		writel(temp, ippc_base + U3P_U3_CTRL(i));
	}

	/* power on and enable all u2 ports */
	for (i = 0; i < u2_port_num; i++) {
		temp = readl(ippc_base + U3P_U2_CTRL(i));
		temp &= ~(CTRL_U2_PORT_PDN | CTRL_U2_PORT_DIS);
		temp |= CTRL_U2_PORT_HOST_SEL;
		writel(temp, ippc_base + U3P_U2_CTRL(i));
	}

	/*
	 * wait for clocks to be stable, and clock domains reset to
	 * be inactive after power on and enable ports
	 */
	return check_ip_clk_status(mtk);
}

static int xhci_mtk_ports_disable(struct xhci_hcd_mtk *mtk)
{
	int i;
	u32 temp;
	int ret;
	int u3_port_num;
	int u2_port_num;
	void __iomem *ippc_base = mtk->ippc_base;

	temp = readl(ippc_base + U3P_IP_XHCI_CAP);
	u3_port_num = CAP_U3_PORT_NUM(temp);
	u2_port_num = CAP_U2_PORT_NUM(temp);
	dev_dbg(mtk->dev, "%s u2p:%d, u3p:%d\n", __func__,
			u2_port_num, u3_port_num);

	/* disable all u3 ports */
	for (i = 0; i < u3_port_num; i++) {
		temp = readl(ippc_base + U3P_U3_CTRL(i));
		temp |= CTRL_U3_PORT_PDN;
		writel(temp, ippc_base + U3P_U3_CTRL(i));
	}

	/* disable all u2 ports */
	for (i = 0; i < u2_port_num; i++) {
		temp = readl(ippc_base + U3P_U2_CTRL(i));
		temp |= CTRL_U2_PORT_PDN;
		writel(temp, ippc_base + U3P_U2_CTRL(i));
	}

	/* power off ip */
	temp = readl(ippc_base + U3P_IP_PW_CTRL1);
	temp |= CTRL1_IP_HOST_PDN;
	writel(temp, ippc_base + U3P_IP_PW_CTRL1);

	temp = readl(ippc_base + U3P_IP_PW_CTRL2);
	temp |= CTRL2_IP_DEV_PDN;
	writel(temp, ippc_base + U3P_IP_PW_CTRL2);

	ret = readl_poll_timeout(ippc_base + U3P_IP_PW_STS1, temp,
			  (temp & STS1_IP_SLEEP_STS), 100, 100000);
	if (ret) {
		dev_err(mtk->dev, "ip sleep failed!!!\n");
		return ret;
	}
	return 0;
}

static void xhci_mtk_ports_config(struct xhci_hcd_mtk *mtk)
{
	u32 temp;

	/* reset whole ip */
	temp = readl(mtk->ippc_base + U3P_IP_PW_CTRL0);
	temp |= CTRL0_IP_SW_RST;
	writel(temp, mtk->ippc_base + U3P_IP_PW_CTRL0);
	udelay(1);
	temp = readl(mtk->ippc_base + U3P_IP_PW_CTRL0);
	temp &= ~CTRL0_IP_SW_RST;
	writel(temp, mtk->ippc_base + U3P_IP_PW_CTRL0);

	xhci_mtk_ports_enable(mtk);
}

static int xhci_mtk_clks_enable(struct xhci_hcd_mtk *mtk)
{
	int ret;

	ret = clk_prepare_enable(mtk->sys_mac);
	if (ret) {
		dev_err(mtk->dev, "failed to enable u3phya_ref\n");
		goto u3phya_ref_err;
	}
	ret = clk_prepare_enable(mtk->wk_deb_p0);
	if (ret) {
		dev_err(mtk->dev, "failed to enable wk_deb_p0\n");
		goto usb_p0_err;
	}
	if (mtk->num_phys > 1) {
		ret = clk_prepare_enable(mtk->wk_deb_p1);
		if (ret) {
			dev_err(mtk->dev, "failed to enable wk_deb_p1\n");
			goto usb_p1_err;
		}
	}

	return 0;

usb_p1_err:
	clk_disable_unprepare(mtk->wk_deb_p0);
usb_p0_err:
	clk_disable_unprepare(mtk->sys_mac);
u3phya_ref_err:
	return -EINVAL;
}

static void xhci_mtk_clks_disable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->num_phys > 1)
		clk_disable_unprepare(mtk->wk_deb_p1);
	clk_disable_unprepare(mtk->wk_deb_p0);
	clk_disable_unprepare(mtk->sys_mac);
}

/* only clocks can be turn off for ip-sleep wakeup mode */
static void usb_wakeup_ip_sleep_en(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_IS_P;
	tmp &= ~(UWK_CTL1_IS_C(0xf));
	tmp |= UWK_CTL1_IS_C(0x8);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp | UWK_CTL1_IS_E);

	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	dev_dbg(mtk->dev, "%s(): WK_CTRL1[P6,E25,C26:29]=%#x\n",
		__func__, tmp);
}

static void usb_wakeup_ip_sleep_dis(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;

	regmap_read(mtk->pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_IS_E;
	regmap_write(mtk->pericfg, PERI_WK_CTRL1, tmp);
}

/*
* for line-state wakeup mode, phy's power should not power-down
* and only support cable plug in/out
*/
static void usb_wakeup_line_state_en(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	/* line-state of u2-port0 */
	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_0P_LS_P;
	tmp &= ~(UWK_CTL1_0P_LS_C(0xf));
	tmp |= UWK_CTL1_0P_LS_C(0x8);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);
	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp | UWK_CTL1_0P_LS_E);

	/* line-state of u2-port1 if support */
	if (mtk->num_phys > 1) {
		regmap_read(pericfg, PERI_WK_CTRL0, &tmp);
		tmp &= ~(UWK_CTL1_1P_LS_C(0xf));
		tmp |= UWK_CTL1_1P_LS_C(0x8);
		regmap_write(pericfg, PERI_WK_CTRL0, tmp);
		regmap_write(pericfg, PERI_WK_CTRL0, tmp | UWK_CTL1_1P_LS_E);
	}
}

static void usb_wakeup_line_state_dis(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_0P_LS_E;
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);

	if (mtk->num_phys > 1) {
		regmap_read(pericfg, PERI_WK_CTRL0, &tmp);
		tmp &= ~UWK_CTL1_1P_LS_E;
		regmap_write(pericfg, PERI_WK_CTRL0, tmp);
	}
}

static void usb_wakeup_enable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->wakeup_src == SSUSB_WK_IP_SLEEP)
		usb_wakeup_ip_sleep_en(mtk);
	else if (mtk->wakeup_src == SSUSB_WK_LINE_STATE)
		usb_wakeup_line_state_en(mtk);
}

static void usb_wakeup_disable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->wakeup_src == SSUSB_WK_IP_SLEEP)
		usb_wakeup_ip_sleep_dis(mtk);
	else if (mtk->wakeup_src == SSUSB_WK_LINE_STATE)
		usb_wakeup_line_state_dis(mtk);
}


static int xhci_mtk_setup(struct usb_hcd *hcd);
static const struct xhci_driver_overrides xhci_mtk_overrides __initconst = {
	.extra_priv_size = sizeof(struct xhci_hcd),
	.reset = xhci_mtk_setup,
};

static struct hc_driver __read_mostly xhci_mtk_hc_driver;

static int xhci_mtk_phy_enable(struct xhci_hcd_mtk *mtk)
{
	unsigned int i;
	int ret;

	for (i = 0; i < mtk->num_phys; i++) {
		ret = phy_init(mtk->phys[i]);
		if (ret)
			goto disable_phy;
		ret = phy_power_on(mtk->phys[i]);
		if (ret) {
			phy_exit(mtk->phys[i]);
			goto disable_phy;
		}
	}

	return 0;

disable_phy:
	for (; i > 0; i--) {
		phy_power_off(mtk->phys[i - 1]);
		phy_exit(mtk->phys[i - 1]);
	}
	return ret;
}

static void xhci_mtk_phy_disable(struct xhci_hcd_mtk *mtk)
{
	unsigned int i;

	for (i = 0; i < mtk->num_phys; i++) {
		phy_power_off(mtk->phys[i]);
		phy_exit(mtk->phys[i]);
	}
}

static int xhci_mtk_ldos_enable(struct xhci_hcd_mtk *mtk)
{
	int ret;

	ret = regulator_enable(mtk->vbus);
	if (ret) {
		dev_err(mtk->dev, "failed to enable vbus\n");
		return ret;
	}

	ret = regulator_enable(mtk->vusb33);
	if (ret) {
		dev_err(mtk->dev, "failed to enable vusb33\n");
		regulator_disable(mtk->vbus);
		return ret;
	}
	return 0;
}

static void xhci_mtk_ldos_disable(struct xhci_hcd_mtk *mtk)
{
	regulator_disable(mtk->vbus);
	regulator_disable(mtk->vusb33);
}

static void xhci_mtk_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_PLAT;
	xhci->quirks |= XHCI_MTK_HOST;
	/*
	 * MTK host controller gives a spurious successful event after a
	 * short transfer. Ignore it.
	 */
	xhci->quirks |= XHCI_SPURIOUS_SUCCESS;
}

/* called during probe() after chip reset completes */
static int xhci_mtk_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci;
	int ret;

	ret = xhci_gen_setup(hcd, xhci_mtk_quirks);
	if (ret)
		return ret;

	if (!usb_hcd_is_primary_hcd(hcd))
		return 0;

	xhci = hcd_to_xhci(hcd);
	return xhci_mtk_sch_init(xhci);
}


static int xhci_mtk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct usb_xhci_pdata *pdata = dev_get_platdata(dev);
	struct xhci_hcd_mtk *mtk;
	const struct hc_driver *driver;
	struct xhci_hcd *xhci;
	struct resource *res;
	struct usb_hcd *hcd;
	struct phy *phy;
	int phy_num;
	int ret = -ENODEV;
	int irq;

	if (usb_disabled())
		return -ENODEV;

	driver = &xhci_mtk_hc_driver;
	mtk = devm_kzalloc(dev, sizeof(*mtk), GFP_KERNEL);
	if (!mtk)
		return -ENOMEM;

	mtk->dev = dev;
	mtk->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(mtk->vbus)) {
		dev_err(dev, "fail to get vbus\n");
		return PTR_ERR(mtk->vbus);
	}

	mtk->vusb33 = devm_regulator_get(dev, "vusb33");
	if (IS_ERR(mtk->vusb33)) {
		dev_err(dev, "fail to get vusb33\n");
		return PTR_ERR(mtk->vusb33);
	}

	mtk->sys_mac = devm_clk_get(dev, "sys_ck");
	if (IS_ERR(mtk->sys_mac)) {
		dev_err(dev, "fail to get sys_ck\n");
		return PTR_ERR(mtk->sys_mac);
	}

	of_property_read_u32(node, "mediatek,wakeup-src", &mtk->wakeup_src);

	mtk->wk_deb_p0 = devm_clk_get(dev, "wakeup_deb_p0");
	if (IS_ERR(mtk->wk_deb_p0)) {
		dev_err(dev, "fail to get wakeup_deb_p0\n");
		return PTR_ERR(mtk->wk_deb_p0);
	}

	mtk->wk_deb_p1 = devm_clk_get(dev, "wakeup_deb_p1");
	if (IS_ERR(mtk->wk_deb_p1)) {
		dev_err(dev, "fail to get wakeup_deb_p1\n");
		return PTR_ERR(mtk->wk_deb_p1);
	}

	mtk->pericfg = syscon_regmap_lookup_by_phandle(node,
						"mediatek,syscon-wakeup");
	if (IS_ERR(mtk->pericfg)) {
		dev_err(dev, "fail to get pericfg regs\n");
		return PTR_ERR(mtk->pericfg);
	}

	mtk->num_phys = of_count_phandle_with_args(node,
			"phys", "#phy-cells");
	if (mtk->num_phys > 0) {
		mtk->phys = devm_kcalloc(dev, mtk->num_phys,
					sizeof(*mtk->phys), GFP_KERNEL);
		if (!mtk->phys)
			return -ENOMEM;
	} else {
		mtk->num_phys = 0;
	}
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	ret = xhci_mtk_ldos_enable(mtk);
	if (ret)
		goto disable_pm;

	ret = xhci_mtk_clks_enable(mtk);
	if (ret)
		goto disable_ldos;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		goto disable_clk;

	/* Initialize dma_mask and coherent_dma_mask to 32-bits */
	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret)
		goto disable_clk;

	if (!dev->dma_mask)
		dev->dma_mask = &dev->coherent_dma_mask;
	else
		dma_set_mask(dev, DMA_BIT_MASK(32));

	hcd = usb_create_hcd(driver, dev, dev_name(dev));
	if (!hcd) {
		ret = -ENOMEM;
		goto disable_clk;
	}

	/*
	 * USB 2.0 roothub is stored in the platform_device.
	 * Swap it with mtk HCD.
	 */
	mtk->hcd = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, mtk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto put_usb2_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mtk->ippc_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mtk->ippc_base)) {
		ret = PTR_ERR(mtk->ippc_base);
		goto put_usb2_hcd;
	}

	for (phy_num = 0; phy_num < mtk->num_phys; phy_num++) {
		phy = devm_of_phy_get_by_index(dev, node, phy_num);
		if (IS_ERR(phy)) {
			ret = PTR_ERR(phy);
			goto put_usb2_hcd;
		}
		mtk->phys[phy_num] = phy;
	}

	xhci_mtk_ports_config(mtk);
	xhci_mtk_phy_enable(mtk);
	device_init_wakeup(dev, 1);

	xhci = hcd_to_xhci(hcd);
	xhci->main_hcd = hcd;
	xhci->shared_hcd = usb_create_shared_hcd(driver, dev,
			dev_name(dev), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto disable_usb_phy;
	}

	if ((node && of_property_read_bool(node, "usb3-lpm-capable")) ||
			(pdata && pdata->usb3_lpm_capable))
		xhci->quirks |= XHCI_LPM_SUPPORT;

	if (HCC_MAX_PSA(xhci->hcc_params) >= 4)
		xhci->shared_hcd->can_do_streams = 1;

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;

	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret)
		goto dealloc_usb2_hcd;

	return 0;

dealloc_usb2_hcd:
	xhci_mtk_sch_exit(xhci);
	usb_remove_hcd(hcd);

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);

disable_usb_phy:
	xhci_mtk_phy_disable(mtk);
	device_init_wakeup(dev, 0);

put_usb2_hcd:
	usb_put_hcd(hcd);

disable_clk:
	xhci_mtk_clks_disable(mtk);

disable_ldos:
	xhci_mtk_ldos_disable(mtk);

disable_pm:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

static int xhci_mtk_remove(struct platform_device *dev)
{
	struct xhci_hcd_mtk *mtk = platform_get_drvdata(dev);
	struct usb_hcd	*hcd = mtk->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);

	usb_remove_hcd(xhci->shared_hcd);
	xhci_mtk_phy_disable(mtk);
	device_init_wakeup(&dev->dev, 0);

	usb_remove_hcd(hcd);
	usb_put_hcd(xhci->shared_hcd);
	usb_put_hcd(hcd);
	xhci_mtk_sch_exit(xhci);
	xhci_mtk_clks_disable(mtk);
	xhci_mtk_ldos_disable(mtk);
	pm_runtime_put_sync(&dev->dev);
	pm_runtime_disable(&dev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int xhci_mtk_suspend(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);

	xhci_mtk_ports_disable(mtk);
	xhci_mtk_phy_disable(mtk);
	xhci_mtk_clks_disable(mtk);
	usb_wakeup_enable(mtk);
	return 0;
}

static int xhci_mtk_resume(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);

	usb_wakeup_disable(mtk);
	xhci_mtk_clks_enable(mtk);
	xhci_mtk_phy_enable(mtk);
	xhci_mtk_ports_enable(mtk);
	return 0;
}

static const struct dev_pm_ops xhci_mtk_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xhci_mtk_suspend, xhci_mtk_resume)
};
#define DEV_PM_OPS	(&xhci_mtk_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id mtk_xhci_of_match[] = {
	{ .compatible = "mediatek,mt8173-xhci"},
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_xhci_of_match);
#endif

static struct platform_driver mtk_xhci_driver = {
	.probe	= xhci_mtk_probe,
	.remove	= xhci_mtk_remove,
	.driver	= {
		.name = "xhci-mtk",
		.pm = DEV_PM_OPS,
		.of_match_table = of_match_ptr(mtk_xhci_of_match),
	},
};

static int __init xhci_mtk_init(void)
{
	xhci_init_driver(&xhci_mtk_hc_driver, &xhci_mtk_overrides);
	return platform_driver_register(&mtk_xhci_driver);
}
module_init(xhci_mtk_init);

static void __exit xhci_mtk_exit(void)
{
	platform_driver_unregister(&mtk_xhci_driver);
}
module_exit(xhci_mtk_exit);

MODULE_DESCRIPTION("MediaTek xHCI Host Controller Driver");
MODULE_LICENSE("GPL v2");
