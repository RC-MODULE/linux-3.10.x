// SPDX-License-Identifier: GPL-2.0+
/*
 * phy-rcmodule-usb - USB PHY, talking to musb controller in RC Module PPC board.
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
#include <linux/usb/musb.h>

/* USB PHY control register offsets */
#define USB_PHY_RESET_OFFSET     0x0


#define USB_PHY_POR_RESET        BIT(0)
#define USB_PHY_UTMI_RESET_PHY   BIT(1)
#define USB_PHY_UTMI_RESET_MUSB  BIT(2)
#define USB_PHY_UTMI_SESPENDM_EN BIT(3)

struct rcmodule_usbphy {
	void __iomem		*phy_ctrl;
	struct usb_phy		phy;
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

static int phy_rcmodule_usb2_init(struct phy *phy)
{
	struct rcmodule_usbphy *k_phy = phy_get_drvdata(phy);

	printk("phy_rcmodule_usb2_init");

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

static int phy_rcmodule_usb2_exit(struct phy *phy)
{
	struct rcmodule_usbphy *k_phy = phy_get_drvdata(phy);

	printk("phy_rcmodule_usb2_exit");
	rcmodule_usbphy_writel(k_phy->phy_ctrl, USB_PHY_RESET_OFFSET, USB_PHY_POR_RESET|USB_PHY_UTMI_SESPENDM_EN);

	return 0;
}


static const struct phy_ops phy_rcmodule_usb2_ops = {
	.init		= phy_rcmodule_usb2_init,
	.exit		= phy_rcmodule_usb2_exit,
//	.power_on	= phy_rcmodule_usb2_power_on,
//	.power_off	= phy_rcmodule_usb2_power_off,
	.owner		= THIS_MODULE,
};


static int rcmodule_usb_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	printk("rcmodule_usb_set_host");

	otg->host = host;
	if (!host)
		otg->state = OTG_STATE_UNDEFINED;

	//musb_mailbox(MUSB_ID_GROUND);

	return 0;
}

static int rcmodule_usb_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
	printk("rcmodule_usb_set_peripheral");

	otg->gadget = gadget;
	if (!gadget)
		otg->state = OTG_STATE_UNDEFINED;

	//musb_mailbox(MUSB_ID_FLOAT);

	return 0;
}

static int rcmodule_usbphy2_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct rcmodule_usbphy	*k_phy;
	struct resource		*res;
	struct phy *phy;
	struct phy_provider *phy_provider;
	struct usb_otg *otg;


	printk("rcmodule_usbphy2_probe");

	k_phy = devm_kzalloc(dev, sizeof(*k_phy), GFP_KERNEL);
	if (!k_phy)
		return -ENOMEM;

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	k_phy->phy_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(k_phy->phy_ctrl))
		return PTR_ERR(k_phy->phy_ctrl);

	k_phy->phy.dev = &pdev->dev;
	k_phy->phy.label = "rcmodule-usbphy";
	k_phy->phy.otg		= otg;
	k_phy->phy.type		= USB_PHY_TYPE_USB2;

	otg->set_host		= rcmodule_usb_set_host;
	otg->set_peripheral	= rcmodule_usb_set_peripheral;
	otg->usb_phy		= &k_phy->phy;
	otg->state			= OTG_STATE_UNDEFINED;


	phy = devm_phy_create(&pdev->dev, NULL, &phy_rcmodule_usb2_ops);
	if (IS_ERR(phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, k_phy);

	phy_provider =
		devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);

	usb_add_phy_dev(&k_phy->phy);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id rcmodule_usbphy2_ids[] = {
	{ .compatible = "rc-module,usbphy2" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcmodule_usbphy_ids);

static struct platform_driver phy_rcmodule_usb2_driver = {
	.probe          = rcmodule_usbphy2_probe,
	.driver         = {
		.name   = "rcmodule-usbphy2",
		.of_match_table = rcmodule_usbphy2_ids,
	},
};

module_platform_driver(phy_rcmodule_usb2_driver);

MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RC Module USB2 phy driver");
MODULE_LICENSE("GPL v2");
