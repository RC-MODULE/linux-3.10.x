// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 by AstroSoft
 * Alexey Spirkov <alexeis@astrosoft.ru>
 * 
 * Some code has been taken from omap2430.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2005-2007 by Texas Instruments 
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/usb/musb.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include "musb_core.h"

struct rcmodule_glue {
	struct device		*dev;
	struct platform_device	*musb;
};
#define glue_to_musb(g)		platform_get_drvdata(g->musb)


static irqreturn_t rcmodule_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);


	if((musb->int_usb & MUSB_INTR_DISCONNECT) && (musb->port1_status & USB_PORT_STAT_C_RESET << 16))
	{
		// skip disconnect irq after reset
		dev_dbg(musb->controller, "Skip disconnect irq after reset...");
		spin_unlock_irqrestore(&musb->lock, flags);
		return IRQ_HANDLED;
	}

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static int rcmodule_musb_init(struct musb *musb)
{
	int status = 0;
	struct device *dev = musb->controller;

	if (dev->parent->of_node) {
		musb->phy = devm_phy_get(dev->parent, "usb2-phy");
		musb->xceiv = devm_usb_get_phy_by_phandle(dev->parent,
		    "usb-phy", 0);
	} else {
		musb->xceiv = devm_usb_get_phy_dev(dev, 0);
		musb->phy = devm_phy_get(dev, "usb");
	}

	if (IS_ERR(musb->xceiv)) {
		status = PTR_ERR(musb->xceiv);

		if (status == -ENXIO)
			return status;

		dev_dbg(dev, "HS USB OTG: no transceiver configured\n");
		return -EPROBE_DEFER;
	}

	if (IS_ERR(musb->phy)) {
		dev_err(dev, "HS USB OTG: no PHY configured\n");
		return PTR_ERR(musb->phy);
	}
	musb->isr = rcmodule_musb_interrupt;

	phy_init(musb->phy);
	phy_power_on(musb->phy);

	return 0;
}

static int rcmodule_musb_exit(struct musb *musb)
{
	phy_power_off(musb->phy);
	phy_exit(musb->phy);
	musb->phy = NULL;
	return 0;
}

/*
 * Load an endpoint's FIFO
 */
static void __maybe_unused _write_fifo_8(struct musb_hw_ep *hw_ep, u16 len,
				    const u8 *src)
{
	//struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;

#ifdef DEBUG
	printk("%cX ep%d fifo %p count %d buf %p\n",
			'T', hw_ep->epnum, fifo, len, src);

	print_hex_dump(KERN_INFO, "TX: ", DUMP_PREFIX_OFFSET, 16, 1,
			src, len, true);
#endif
	if (unlikely(len == 0))
		return;

	prefetch((u8 *)src);

	iowrite8_rep(fifo, src, len);
}

/*
 * Unload an endpoint's FIFO
 */
static void __maybe_unused _read_fifo_8(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	//struct musb *musb = hw_ep->musb;
	void __iomem *fifo = hw_ep->fifo;

#ifdef DEBUG
	printk("%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->epnum, fifo, len, dst);
#endif
	if (unlikely(len == 0))
		return;

	/* byte aligned */
	ioread8_rep(fifo, dst, len);

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "RX: ", DUMP_PREFIX_OFFSET, 16, 1,
			dst, len, true);
#endif
}


static u16 _readw(const void __iomem *addr, unsigned offset)
{
	u16 data = le16_to_cpu(__raw_readw(addr + offset));
	return data;
}

static void _writew(void __iomem *addr, unsigned offset, u16 data)
{
	__raw_writew(cpu_to_le16(data), addr + offset);
}

static u32 _readl(const void __iomem *addr, unsigned offset)
{
	u32 data = le32_to_cpu(__raw_readl(addr + offset));

	return data;
}

static void _writel(void __iomem *addr, unsigned offset, u32 data)
{
	__raw_writel(cpu_to_le32(data), addr + offset);
}



static const struct musb_platform_ops rcmodule_ops = {
	.quirks		= MUSB_DMA_INVENTRA,
#ifdef CONFIG_USB_INVENTRA_DMA
	.dma_init	= musbhs_dma_controller_create,
	.dma_exit	= musbhs_dma_controller_destroy,
#endif
	.init		= rcmodule_musb_init,
	.exit		= rcmodule_musb_exit,

//	.read_fifo = _read_fifo_8,
//	.write_fifo = _write_fifo_8,

    .readw 		= _readw,
	.writew 	= _writew,
	.readl		= _readl, 
	.writel 	= _writel,
	.fifo_mode  = 4
};

static u64 rcmodule_dmamask = DMA_BIT_MASK(25); // use only first Gb

static int rcmodule_probe(struct platform_device *pdev)
{
	struct resource			musb_resources[3];
	struct musb_hdrc_platform_data	*pdata = dev_get_platdata(&pdev->dev);
	struct platform_device		*musb;
	struct rcmodule_glue		*glue;
	struct device_node		*np = pdev->dev.of_node;
	struct musb_hdrc_config		*config;
	int				ret = -ENOMEM, val;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		goto err0;

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err0;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &rcmodule_dmamask;
	musb->dev.coherent_dma_mask	= rcmodule_dmamask;
	musb->dev.dma_pfn_offset = pdev->dev.dma_pfn_offset;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;

	set_dma_offset(&musb->dev, - (musb->dev.dma_pfn_offset << PAGE_SHIFT));

	if (np) {
		struct device_node *sctl;
		struct platform_device *control_pdev;

		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err2;

		config = devm_kzalloc(&pdev->dev, sizeof(*config), GFP_KERNEL);
		if (!config)
			goto err2;

		of_property_read_u32(np, "mode", &val); pdata->mode = (u8) val;
		of_property_read_u32(np, "num-eps", &val); config->num_eps = (u8) val;
		of_property_read_u32(np, "ram-bits", &val); config->ram_bits = (u8) val;
		of_property_read_u32(np, "power", &val); pdata->power = (u8) val;

		ret = of_property_read_u32(np, "multipoint", &val);
		if (!ret && val)
			config->multipoint = true;

		pdata->board_data	= 0; // not needed for a while
		pdata->config		= config;

		// process control register		
		sctl = of_parse_phandle(np, "sctl", 0);
		if (sctl) {
			void* sctl_reg = 0;
			struct resource		*res;
			u32 sctl_val[3];
			control_pdev = of_find_device_by_node(sctl);
			if (!control_pdev) {
				dev_err(&pdev->dev, "Failed to get SCTL device\n");
				ret = -EINVAL;
				goto err2;
			}
			res = platform_get_resource(control_pdev, IORESOURCE_MEM, 0);
			sctl_reg = devm_ioremap_resource(&control_pdev->dev, res);
			if (IS_ERR(sctl_reg))
			{
				dev_err(&pdev->dev, "Failed to allocate SCTL resource\n");
				ret = -EINVAL;
				goto err2;
			}
			if(!of_property_read_u32_array(np, "sctl-reg", &sctl_val[0], 3))
			{
				u32 value = __raw_readl(sctl_reg + sctl_val[0]);
				dev_dbg(&pdev->dev, "Setup SCTL register %d, offset %d, bit %d", sctl_val[0], sctl_val[1], sctl_val[2]);
				value &= ~(1<<sctl_val[1]);
				value |=  sctl_val[2] << sctl_val[1];
				__raw_writel(value, sctl_reg + sctl_val[0]);
			}
			else
				dev_err(&pdev->dev, "Failed to read sctl-reg values\n");

			devm_iounmap(&control_pdev->dev, sctl_reg);
		}
	}
	pdata->platform_ops		= &rcmodule_ops;

	platform_set_drvdata(pdev, glue);

	memset(musb_resources, 0x00, sizeof(*musb_resources) *
			ARRAY_SIZE(musb_resources));

	musb_resources[0].name = pdev->resource[0].name;
	musb_resources[0].start = pdev->resource[0].start;
	musb_resources[0].end = pdev->resource[0].end;
	musb_resources[0].flags = pdev->resource[0].flags;

	musb_resources[1].name = pdev->resource[1].name;
	musb_resources[1].start = pdev->resource[1].start;
	musb_resources[1].end = pdev->resource[1].end;
	musb_resources[1].flags = pdev->resource[1].flags;

	musb_resources[2].name = pdev->resource[2].name;
	musb_resources[2].start = pdev->resource[2].start;
	musb_resources[2].end = pdev->resource[2].end;
	musb_resources[2].flags = pdev->resource[2].flags;

	ret = platform_device_add_resources(musb, musb_resources,
			ARRAY_SIZE(musb_resources));
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err2;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err2;
	}

	pm_runtime_enable(&pdev->dev);

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err3;
	}

	return 0;

err3:
	pm_runtime_disable(glue->dev);

err2:
	platform_device_put(musb);

err0:
	return ret;
}

static int rcmodule_remove(struct platform_device *pdev)
{
	struct rcmodule_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	pm_runtime_disable(glue->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rcmodule_id_table[] = {
	{
		.compatible = "rc-module,musb"
	},
	{},
};
MODULE_DEVICE_TABLE(of, rcmodule_id_table);
#endif

static struct platform_driver rcmodule_driver = {
	.probe		= rcmodule_probe,
	.remove		= rcmodule_remove,
	.driver		= {
		.name	= "musb-rcmodule",
		.of_match_table = of_match_ptr(rcmodule_id_table),
	},
};

module_platform_driver(rcmodule_driver);

MODULE_DESCRIPTION("RC-Module MUSB Glue Layer");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_LICENSE("GPL v2");
