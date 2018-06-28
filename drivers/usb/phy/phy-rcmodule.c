// SPDX-License-Identifier: GPL-2.0+
/*
 * phy-rcmodule - USB PHY, talking to musb controller in RC Module PPC board.
 *
 * Copyright (C) 2018 by AstroSoft
 * Alexey Spirkov <alexeis@astrosoft.ru>
 * 
 * Some code has been taken from phy-keystone.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 * 
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/io.h>
#include <linux/of.h>

#include "phy-generic.h"

/* USB PHY control register offsets */
#define USB_PHY_RESET_OFFSET     0x0


#define USB_PHY_POR_RESET        BIT(0)
#define USB_PHY_UTMI_RESET_PHY   BIT(1)
#define USB_PHY_UTMI_RESET_MUSB  BIT(2)
#define USB_PHY_UTMI_SESPENDM_EN BIT(3)

struct rcmodule_usbphy {
	struct usb_phy_generic	usb_phy_gen;
	void __iomem			*phy_ctrl;
};

static inline u32 rcmodule_usbphy_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void rcmodule_usbphy_writel(void __iomem *base,
					  u32 offset, u32 value)
{
	writel(value, base + offset);
}

static int rcmodule_usbphy_init(struct usb_phy *phy)
{
	struct rcmodule_usbphy *k_phy = dev_get_drvdata(phy->dev);

	printk("rcmodule_usbphy_init");

    // USB reset sequence begin
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_POR_RESET | USB_PHY_UTMI_RESET_PHY);
    // T1 - POR LOW
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_UTMI_RESET_PHY);
    usleep_range(10, 1000);

    // T2 - SUSPENDM HIGH
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_UTMI_RESET_PHY | USB_PHY_UTMI_SESPENDM_EN);
	usleep_range(47, 1000);

    //T3 T4 UTMI_RESET LOW
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_UTMI_SESPENDM_EN);

    //T5 - Release reset USB controller
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_UTMI_SESPENDM_EN | USB_PHY_UTMI_RESET_MUSB);

	return 0;
}

static void rcmodule_usbphy_shutdown(struct usb_phy *phy)
{
	struct rcmodule_usbphy *k_phy = dev_get_drvdata(phy->dev);

	printk("rcmodule_usbphy_shutdown");
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_POR_RESET|USB_PHY_UTMI_SESPENDM_EN);
}

static int rcmodule_usbphy_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct rcmodule_usbphy	*k_phy;
	struct resource		*res;
	int ret;

//	printk("rcmodule_usbphy_probe");
	k_phy = devm_kzalloc(dev, sizeof(*k_phy), GFP_KERNEL);
	if (!k_phy)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	k_phy->phy_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(k_phy->phy_ctrl))
		return PTR_ERR(k_phy->phy_ctrl);

	ret = usb_phy_gen_create_phy(dev, &k_phy->usb_phy_gen, NULL);
	if (ret)
		return ret;

	k_phy->usb_phy_gen.phy.init = rcmodule_usbphy_init;
	k_phy->usb_phy_gen.phy.shutdown = rcmodule_usbphy_shutdown;

	platform_set_drvdata(pdev, k_phy);

	return usb_add_phy_dev(&k_phy->usb_phy_gen.phy);
}

static int rcmodule_usbphy_remove(struct platform_device *pdev)
{
	struct rcmodule_usbphy *k_phy = platform_get_drvdata(pdev);

	usb_remove_phy(&k_phy->usb_phy_gen.phy);

	return 0;
}

static const struct of_device_id rcmodule_usbphy_ids[] = {
	{ .compatible = "rc-module,usbphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcmodule_usbphy_ids);

static struct platform_driver rcmodule_usbphy_driver = {
	.probe          = rcmodule_usbphy_probe,
	.remove         = rcmodule_usbphy_remove,
	.driver         = {
		.name   = "rcmodule-usbphy",
		.of_match_table = rcmodule_usbphy_ids,
	},
};

module_platform_driver(rcmodule_usbphy_driver);

MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RC Module USB phy driver");
MODULE_LICENSE("GPL v2");
