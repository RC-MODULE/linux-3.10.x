/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * Copyright (C) 2011 RC Module.
 *
 * Author : Kirill Mikhailov <kirill.mikhailov@module.ru>
 *
 * This module is based on ohci-sh module
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>

static int ohci_uemd_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	ohci_hcd_init(ohci);
	ohci_init(ohci);
	ohci_run(ohci);
	hcd->state = HC_STATE_RUNNING;
	return 0;
}

static const struct hc_driver ohci_uemd_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"UEMD OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_uemd_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_uemd_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq = -1;
	int ret;

	if (usb_disabled())
		return -ENODEV;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource error.");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq error.");
		return -ENODEV;
	}

	/* initialize hcd */
	hcd = usb_create_hcd(&ohci_uemd_hc_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		dev_err(&pdev->dev, "Failed to create hcd");
		return -ENOMEM;
	}
	
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	
	
	hcd->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (hcd->regs == NULL) {
		dev_err(&pdev->dev, "Resource request/ioremap failed\n");
		usb_put_hcd(hcd);
		return -EFAULT;
	}

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to add hcd");
		usb_put_hcd(hcd);
		return ret;
	}

	return ret;
}

static int ohci_hcd_uemd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	return 0;
}

static const struct of_device_id uemd_ohci_dt_ids[] = {
	{ .compatible = "module,ohci" },
	{ }
};

MODULE_DEVICE_TABLE(of, uemd_ohci_dt_ids);

static struct platform_driver ohci_hcd_uemd_driver = {
	.driver		= {
		.name	= "uemd-ohci",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(uemd_ohci_dt_ids),
	},
	.probe		= ohci_hcd_uemd_probe,
	.remove		= ohci_hcd_uemd_remove,
	.shutdown	= usb_hcd_platform_shutdown,
};

MODULE_ALIAS("platform:uemd_ohci");
