// SPDX-License-Identifier: GPL-2.0-only
/**
 * Host side driver to map RCM-BASIS devices to host
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
//#define DEBUG

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/pci_ids.h>

#include <linux/pci_regs.h>

#include <linux/configfs.h>

#ifndef PCI_STD_NUM_BARS
#	define PCI_STD_NUM_BARS 6
#endif

#include "basis-controller.h"

#include "basis_addresses.h"
#include "basis_regs_gic.h"
#include "basis_regs_pcie.h"

#define IRQ_TYPE_UNDEFINED			-1
#define IRQ_TYPE_LEGACY				0
#define IRQ_TYPE_MSI				1
#define IRQ_TYPE_MSIX				2

#define RCM_PCI_MAP_SET_IRQTYPE 0
#define RCM_PCI_MAP_GET_IRQTYPE 1

static DEFINE_IDA(rcm_pci_map_ida);

static unsigned long basis_ep_mem_addr = 0x40000000;
module_param(basis_ep_mem_addr, ulong, 0);
MODULE_PARM_DESC(basis_ep_mem_addr,
                 "Address of PCIE-memory block (BASIS, enpoint side).");

static unsigned long basis_ep_mem_size = 0x40000000;
module_param(basis_ep_mem_size, ulong, 0);
MODULE_PARM_DESC(basis_ep_mem_size,
                 "Size of PCIE-memory block (BASIS, enpoint side).");

static unsigned long basis_ep_page_size = 4096;
module_param(basis_ep_page_size, ulong, 0);
MODULE_PARM_DESC(basis_ep_page_size,
                 "Page size (BASIS, enpoint side).");

static unsigned basis_ep_cnt_regions = 32;
module_param(basis_ep_cnt_regions, uint, 0);
MODULE_PARM_DESC(basis_ep_cnt_regions,
                 "Number of outbound regions (BASIS, enpoint side).");

#define to_data(priv) container_of((priv), struct rcm_pci_map_data, miscdev)

static int irq_type = IRQ_TYPE_MSIX;
module_param(irq_type, int, 0444);
MODULE_PARM_DESC(irq_type, "IRQ mode selection in rcm_pci_map (0 - Legacy, 1 - MSI, 2 - MSI-X)");

enum pci_barno {
	BAR_0,
	BAR_1,
	BAR_2,
	BAR_3,
	BAR_4,
	BAR_5,
};

struct rcm_pci_map_pci_to_cpu {
	u64 pci_addr;
	u32 cpu_addr;
};

#define CNT_MAP_ADDRESSES 16

struct rcm_pci_map_data {
	struct pci_dev                 *pdev;
	void __iomem                   *bar[PCI_STD_NUM_BARS];
	resource_size_t                 bar_size[PCI_STD_NUM_BARS];
	resource_size_t                 bar_phys[PCI_STD_NUM_BARS];
	struct irq_domain              *domain;
	int                             irq_type;
	int                             num_irqs;
	struct mutex                    mutex;
	struct miscdevice               miscdev;
	size_t                          alignment;

	struct config_group             group;

	struct basis_controller        *controller;

	struct rcm_pci_map_pci_to_cpu   map_addr[CNT_MAP_ADDRESSES];
};

static inline struct rcm_pci_map_data*
to_rcm_pci_map_data(struct config_item *item)
{
	return container_of(to_config_group(item), struct rcm_pci_map_data, group);
}

static void rcm_pci_map_irqhandler(struct irq_desc *desc)
{
	struct rcm_pci_map_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 reg;
	u64 status = 0;
	int i;
	unsigned int irq;
	unsigned long hwirq = desc->irq_data.hwirq; 

	chained_irq_enter(chip, desc);

	dev_dbg(&data->pdev->dev, "Interrupt (hwirq = %lu)\n", hwirq);

	if (data->irq_type != IRQ_TYPE_LEGACY) 
	{
		struct msi_desc *msi_desc = irq_data_get_msi_desc(&desc->irq_data);

		hwirq = 0;

		for (i = 0; i < data->num_irqs; ++i) {
			if (pci_irq_vector(data->pdev, i) == msi_desc->irq) {
				hwirq = i;
				break;
			}
		}
	}

	if ((data->irq_type == IRQ_TYPE_LEGACY) ||
	    ((hwirq == 0) && (data->num_irqs < 64))) {
		reg = readl(data->bar[0] + EXT_IRQ_GEN_BASE +
		            EXT_IRQ_GEN_Global_IRQ_Status_l);

		status = reg;

		reg = readl(data->bar[0] + EXT_IRQ_GEN_BASE +
		            EXT_IRQ_GEN_Global_IRQ_Status_h);

		status |= (u64)reg << 32;

		dev_dbg(&data->pdev->dev, "Interrupt (status = 0x%016llX)\n",
		        status);

		for (i = 0; i < 64; i++) {
			if (status & BIT_ULL(i)) {
				irq = irq_find_mapping(data->domain, i);
				if (irq != 0)
					generic_handle_irq(irq);
			}
		}
	} else {
		irq = irq_find_mapping(data->domain, hwirq);
		if (irq != 0)
			generic_handle_irq(irq);
	}

	chained_irq_exit(chip, desc);
}

static void rcm_pci_map_free_irq_vectors(struct rcm_pci_map_data *data)
{
	struct pci_dev *pdev = data->pdev;
	u32 page_size = data->controller->ep_mem->page_size;
	int i;

	pci_free_irq_vectors(pdev);

	for (i = 0; i < CNT_MAP_ADDRESSES; ++i) {
		if (data->map_addr[i].pci_addr != 0) {
			basis_controller_ep_unmap_addr(data->controller,
			                               data->map_addr[i].cpu_addr);
			basis_controller_ep_mem_free_addr(data->controller,
			                                  data->map_addr[i].cpu_addr,
			                                  page_size);
			data->map_addr[i].pci_addr = 0;
			data->map_addr[i].cpu_addr = 0;
		}
	}
}

static u32 rcm_pci_map_map_pci_addr(struct rcm_pci_map_data *data, u64 pci_addr)
{
	int i;
	u32 cpu_addr;
	int ret;
	u32 page_size = data->controller->ep_mem->page_size;
	u32 page_mask = page_size - 1;
	u32 offset = pci_addr & page_mask;

	for (i = 0; i < CNT_MAP_ADDRESSES; ++i) {
		if ((pci_addr - offset) == data->map_addr[i].pci_addr)
			break;
	}

	if (i < CNT_MAP_ADDRESSES)
		return data->map_addr[i].cpu_addr + offset;

	cpu_addr = basis_controller_ep_mem_alloc_addr(data->controller,
	                                              page_size);
	if (cpu_addr == 0)
		return 0;

	ret = basis_controller_ep_map_addr(data->controller, cpu_addr,
	                                   pci_addr - offset, page_size);

	if (ret)
		return 0;

	for (i = 0; i < CNT_MAP_ADDRESSES; ++i) {
		if (data->map_addr[i].pci_addr == 0) {
			data->map_addr[i].pci_addr = pci_addr - offset;
			data->map_addr[i].cpu_addr = cpu_addr;
			break;
		}
	}

	if (i >= CNT_MAP_ADDRESSES) {
		basis_controller_ep_unmap_addr(data->controller, cpu_addr);
		basis_controller_ep_mem_free_addr(data->controller,
		                                  cpu_addr, page_size);
		return 0;
	}

	return cpu_addr + offset;
}

static void rcm_pci_map_setup_irqs(struct rcm_pci_map_data *data,
                                   int type, int num_irqs)
{
	u32 ctrl = 0;
	u32 reg;
	int i;
	int irq;
	int addr;

	writel(0xFFFFFFFF, data->bar[0] + EXT_IRQ_GEN_BASE +
	       EXT_IRQ_GEN_Global_IRQ_Mask_l);
	writel(0xFFFFFFFF, data->bar[0] + EXT_IRQ_GEN_BASE +
	       EXT_IRQ_GEN_Global_IRQ_Mask_h);

	if (type != IRQ_TYPE_LEGACY) {
		irq = 0;
		addr = EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Map_0;
		while (irq < 64) {
			reg = 0;

			for (i = 0; i < 4; ++i) {
				if (irq < num_irqs)
					reg |= irq << (6 * i);
				++irq;
			}

			writel(reg, data->bar[0] + addr);
			addr += 4;
		}
	}

	if (type == IRQ_TYPE_MSIX) {
		addr = EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_v0_Message_Address;

		for (i = 0; i < num_irqs; ++i) {
			u32 a0 = readl(data->bar[1] + i * PCI_MSIX_ENTRY_SIZE +
			               PCI_MSIX_ENTRY_LOWER_ADDR);
			u32 a1 = readl(data->bar[1] + i * PCI_MSIX_ENTRY_SIZE +
			               PCI_MSIX_ENTRY_UPPER_ADDR);
			u32 d  = readl(data->bar[1] + i * PCI_MSIX_ENTRY_SIZE +
			               PCI_MSIX_ENTRY_DATA);

			u64 pci_addr = ((u64)a1 << 32) | a0;

			u32 cpu_addr = rcm_pci_map_map_pci_addr(data, pci_addr);

			writel(cpu_addr, data->bar[0] + addr + 
			                 PCI_MSIX_ENTRY_LOWER_ADDR);
			writel(d,        data->bar[0] + addr + 
			                 PCI_MSIX_ENTRY_DATA);
			writel(0,        data->bar[0] + addr + 
			                 PCI_MSIX_ENTRY_VECTOR_CTRL);

			addr += PCI_MSIX_ENTRY_SIZE;
		}
	}

	if (type == IRQ_TYPE_LEGACY) { 
		ctrl |= BIT(3);
	}

	writel(ctrl, data->bar[0] + EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Ctrl);
}


static bool rcm_pci_map_alloc_irq_vectors(struct rcm_pci_map_data *data,
                                          int type)
{
	int irq = -1;
	struct pci_dev *pdev = data->pdev;
	struct device *dev = &pdev->dev;
	bool res = true;

	switch (type) {
	case IRQ_TYPE_LEGACY:
		irq = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_LEGACY);
		if (irq < 0)
			dev_err(dev, "Failed to get Legacy interrupt\n");
		break;
	case IRQ_TYPE_MSI:
		irq = pci_alloc_irq_vectors(pdev, 1, 32, PCI_IRQ_MSI);
		if (irq < 0)
			dev_err(dev, "Failed to get MSI interrupts\n");
		break;
	case IRQ_TYPE_MSIX:
		irq = pci_alloc_irq_vectors(pdev, 1, 64, PCI_IRQ_MSIX);
		if (irq < 0)
			dev_err(dev, "Failed to get MSI-X interrupts\n");
		break;
	default:
		dev_err(dev, "Invalid IRQ type selected\n");
	}

	if (irq < 0) {
		irq = 0;
		res = false;
	}
	data->num_irqs = irq;

	if (data->num_irqs) {
		rcm_pci_map_setup_irqs(data, type, irq);
	}

	return res;
}

static void rcm_pci_map_release_irq(struct rcm_pci_map_data *data)
{
	int i;
	struct pci_dev *pdev = data->pdev;

	writel(0, data->bar[0] + EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Ctrl);

	writel(0xFFFFFFFF, data->bar[0] + EXT_IRQ_GEN_BASE +
	       EXT_IRQ_GEN_Global_IRQ_Mask_l);
	writel(0xFFFFFFFF, data->bar[0] + EXT_IRQ_GEN_BASE +
	       EXT_IRQ_GEN_Global_IRQ_Mask_h);

	for (i = 0; i < data->num_irqs; i++)
		irq_set_chained_handler_and_data(pci_irq_vector(pdev, i),
		                                 NULL, NULL);

	data->num_irqs = 0;
}

static bool rcm_pci_map_request_irq(struct rcm_pci_map_data *data)
{
	int i;
	struct pci_dev *pdev = data->pdev;

	for (i = 0; i < data->num_irqs; i++)
		irq_set_chained_handler_and_data(pci_irq_vector(pdev, i),
		                                 rcm_pci_map_irqhandler,
		                                 data);

	return true;
}

static bool rcm_pci_map_set_irqtype(struct rcm_pci_map_data *data,
                                    int req_irq_type)
{
	struct pci_dev *pdev = data->pdev;
	struct device *dev = &pdev->dev;

	if (req_irq_type < IRQ_TYPE_LEGACY || req_irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Invalid IRQ type option\n");
		return false;
	}

	if (data->irq_type == req_irq_type)
		return true;

	rcm_pci_map_release_irq(data);
	rcm_pci_map_free_irq_vectors(data);

	if (!rcm_pci_map_alloc_irq_vectors(data, req_irq_type))
		goto err;

	if (!rcm_pci_map_request_irq(data))
		goto err;

	data->irq_type = req_irq_type;
	return true;

err:
	rcm_pci_map_free_irq_vectors(data);
	data->irq_type = IRQ_TYPE_UNDEFINED;
	return false;
}

static void rcm_pci_map_irq_eoi(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pci_map_data *data = gc->private;
	u32 addr;

	if (d->hwirq >= 64)
		return;

	addr = (d->hwirq < 32) ? 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Status_l : 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Status_h;

	writel(BIT(d->hwirq % 32), data->bar[0] + addr);
}

static void rcm_pci_map_irq_mask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pci_map_data *data = gc->private;
	u32 addr;
	u32 reg;

	if (d->hwirq >= 64)
		return;

	addr = (d->hwirq < 32) ? 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Mask_l : 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Mask_h;

	reg = readl(data->bar[0] + addr);
	reg |= BIT(d->hwirq % 32);
	writel(reg, data->bar[0] + addr);
}

static void rcm_pci_map_irq_unmask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pci_map_data *data = gc->private;
	u32 addr;
	u32 reg;

	if (d->hwirq >= 64)
		return;

	addr = (d->hwirq < 32) ? 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Mask_l : 
	       EXT_IRQ_GEN_BASE + EXT_IRQ_GEN_Global_IRQ_Mask_h;

	reg = readl(data->bar[0] + addr);
	reg &= ~BIT(d->hwirq % 32);
	writel(reg, data->bar[0] + addr);
}

static int rcm_pci_map_irq_set_type(struct irq_data *d, unsigned int type)
{
//	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
//	struct rcm_pci_map_data *data = gc->private;

	if ( (d->hwirq >= 64) || 
	     ((type != IRQ_TYPE_LEVEL_HIGH) && (type != IRQ_TYPE_EDGE_RISING)) )
		return -EINVAL;

	return 0;
}

static long rcm_pci_map_ioctl(struct file *file, unsigned int cmd,
                              unsigned long arg)
{
	int ret = -EINVAL;
	struct rcm_pci_map_data *data = to_data(file->private_data);

	mutex_lock(&data->mutex);
	switch (cmd) {
	case RCM_PCI_MAP_SET_IRQTYPE:
		ret = rcm_pci_map_set_irqtype(data, arg);
		break;
	case RCM_PCI_MAP_GET_IRQTYPE:
		ret = data->irq_type;
		break;
	}

	mutex_unlock(&data->mutex);
	return ret;
}

static const struct basis_controller_ops rcm_pci_map_controller_ops = {
};

static const struct file_operations rcm_pci_map_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rcm_pci_map_ioctl,
};

static int rcm_pci_map_probe(struct pci_dev *pdev,
                             const struct pci_device_id *ent)
{
	int err;
	int id;
	char name[20];
	enum pci_barno bar;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct rcm_pci_map_data *data;
	struct miscdevice *misc_device;
	struct irq_chip_generic *gc;
	int num_chip;
	struct basis_controller *controller;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->alignment = 0;
	data->pdev = pdev;

	mutex_init(&data->mutex);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		return err;
	}

	err = pci_request_regions(pdev, "rcm-pci-map");
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
			base = pci_ioremap_bar(pdev, bar);
			if (!base) {
				dev_err(dev, "Failed to read BAR%d\n", bar);
			}
			data->bar[bar]      = base;
			data->bar_phys[bar] = pci_resource_start(pdev, bar);
			data->bar_size[bar] = pci_resource_len(pdev, bar);

			dev_dbg(dev, "BAR[%d] 0x%llX:0x%llX\n", 
			        bar, (u64)data->bar_phys[bar], (u64)data->bar_size[bar]);
		}
	}

	controller = basis_controller_create(dev,
	                                     &rcm_pci_map_controller_ops);
	if (IS_ERR(controller)) {
		dev_err(dev, "Failed to create BASIS controller.\n");
		goto err_iounmap;
	}

	err = basis_controller_ep_mem_init(controller, basis_ep_mem_addr,
	                                   basis_ep_mem_size,
	                                   basis_ep_page_size);
	if (err) {
		dev_err(dev, "Failed to initialize the EP-side memory space\n");
		goto err_controller_destroy;
	}

	err = basis_controller_ep_regions_init(controller,
	                                       data->bar[0], data->bar_phys[0],
	                                       basis_ep_cnt_regions);
	if (err) {
		dev_err(dev,
		        "Failed to initialize the EP-side outbound regions\n");
		goto err_mem_exit;
	}

	basis_controller_set_drvdata(controller, data);

	data->controller = controller;

	data->domain = irq_domain_create_linear(pdev->dev.fwnode, 64,
	                                        &irq_generic_chip_ops,
	                                        NULL);
	if (!data->domain) {
		dev_err(dev, "couldn't create irq domain.\n");
		goto err_mem_exit;
	}

	controller->domain = data->domain;

	err = irq_alloc_domain_generic_chips(
	    data->domain, 32, 1, "rcm-pci-map",
	    handle_fasteoi_irq, IRQ_NOREQUEST | IRQ_NOPROBE,
	    0, 0);
	if (err) {
		dev_err(dev, "couldn't allocate irq chips.\n");
		goto err_remove_domain;
	}

	for (num_chip = 0; num_chip < 2; ++num_chip) {
		gc = irq_get_domain_generic_chip(data->domain, num_chip * 32);
		gc->private = data;

		gc->chip_types[0].type              = IRQ_TYPE_SENSE_MASK;
		gc->chip_types[0].chip.irq_mask     = rcm_pci_map_irq_mask;
		gc->chip_types[0].chip.irq_unmask   = rcm_pci_map_irq_unmask;
		gc->chip_types[0].chip.irq_eoi      = rcm_pci_map_irq_eoi;
		gc->chip_types[0].chip.irq_set_type = rcm_pci_map_irq_set_type;
		gc->chip_types[0].chip.name         = "rcm-pci-map";

		gc->chip_types[0].chip.flags        = IRQCHIP_EOI_IF_HANDLED;
	}

	data->irq_type = irq_type;

	if (!rcm_pci_map_alloc_irq_vectors(data, data->irq_type))
		goto err_remove_domain;

	if (!rcm_pci_map_request_irq(data))
		goto err_disable_irq;

	pci_set_drvdata(pdev, data);

	id = ida_simple_get(&rcm_pci_map_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		dev_err(dev, "Unable to get id\n");
		goto err_release_irq;
	}

	snprintf(name, sizeof(name), "rcm-pci-map-%d", id);
	misc_device = &data->miscdev;
	misc_device->minor = MISC_DYNAMIC_MINOR;
	misc_device->name = kstrdup(name, GFP_KERNEL);
	if (!misc_device->name) {
		err = -ENOMEM;
		goto err_ida_remove;
	}
	misc_device->fops = &rcm_pci_map_fops,

	err = misc_register(misc_device);
	if (err) {
		dev_err(dev, "Failed to register device\n");
		goto err_kfree_name;
	}

	return 0;

err_kfree_name:
	kfree(misc_device->name);

err_ida_remove:
	ida_simple_remove(&rcm_pci_map_ida, id);

err_release_irq:
	rcm_pci_map_release_irq(data);

err_disable_irq:
	rcm_pci_map_free_irq_vectors(data);

err_remove_domain:
	irq_domain_remove(data->domain);

err_mem_exit:
	basis_controller_ep_mem_exit(controller);

err_controller_destroy:
	basis_controller_destroy(controller);

err_iounmap:
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (data->bar[bar])
			pci_iounmap(pdev, data->bar[bar]);
	}
	pci_release_regions(pdev);

err_disable_pdev:
	pci_disable_device(pdev);

	return err;
}

static void rcm_pci_map_remove(struct pci_dev *pdev)
{
	int id;
	enum pci_barno bar;
	struct rcm_pci_map_data *data = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &data->miscdev;

	if (sscanf(misc_device->name, "rcm-pci-map-%d", &id) != 1)
		return;
	if (id < 0)
		return;

	pci_disable_device(pdev);

	misc_deregister(&data->miscdev);
	kfree(misc_device->name);
	ida_simple_remove(&rcm_pci_map_ida, id);

	rcm_pci_map_release_irq(data);
	rcm_pci_map_free_irq_vectors(data);

	irq_domain_remove(data->domain);
	basis_controller_ep_mem_exit(data->controller);

	basis_controller_destroy(data->controller);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (data->bar[bar])
			pci_iounmap(pdev, data->bar[bar]);
	}

	pci_release_regions(pdev);
}

static const struct pci_device_id rcm_pci_map_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CDNS, 0x01FF) },
	{ }
};
MODULE_DEVICE_TABLE(pci, rcm_pci_map_tbl);

static struct pci_driver rcm_pci_map_driver = {
	.name		= "rcm-pci-map",
	.id_table	= rcm_pci_map_tbl,
	.probe		= rcm_pci_map_probe,
	.remove		= rcm_pci_map_remove,
};
module_pci_driver(rcm_pci_map_driver);

MODULE_DESCRIPTION("PCI MAP RCM-BASIS DEVICES DRIVER");
MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_LICENSE("GPL v2");
