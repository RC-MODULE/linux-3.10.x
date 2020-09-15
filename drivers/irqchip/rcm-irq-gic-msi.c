// SPDX-License-Identifier: GPL-2.0
/*
 * RCM ARM GIC MSI support
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#define MAX_MSI_VECTORS 32

#define POLLING_SLEEP 10000

struct rcm_gic_msi_data {
	struct list_head         list;

	DECLARE_BITMAP(used, MAX_MSI_VECTORS);
	struct mutex             lock;
	struct platform_device  *pdev;
	struct irq_domain       *msi_domain;
	struct irq_domain       *inner_domain;
	u32                      num_msix;
	phys_addr_t              addr_gic_base;

	void                    *buff;
	dma_addr_t               dma_addr;

	struct task_struct*      thread_polling;
	u32                      usleep_polling;
};

static struct msi_domain_info rcm_gic_msi_domain_info;

void rcm_handle_soft_irq(unsigned int hwirq)
{
	struct rcm_gic_msi_data *data = rcm_gic_msi_domain_info.data;
	u32 irq;

	if (!data)
		return;

	irq = irq_find_mapping(data->inner_domain, hwirq);
	if (irq)
		generic_handle_irq(irq);
	else
		dev_err(&data->pdev->dev, "unexpected MSI (hwirq = %u)\n",
		        hwirq);
}

static int rcm_gic_msi_polling(void* context)
{
	struct rcm_gic_msi_data *data = context;
	u32* tbl = data->buff;

	while (!kthread_should_stop()) {
		if (tbl[0] != 0) {
			tbl[0] = 0;
			local_irq_disable();
			rcm_handle_soft_irq(data->num_msix);
			local_irq_enable();
		}

		usleep_range(data->usleep_polling * 9 / 10,
		             data->usleep_polling * 11 / 10);
	}

	return 0;
}

static struct irq_chip rcm_gic_msi_irq_chip = {
	.name = "RCM-MSI",
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

int rcm_gic_msi_prepare(struct irq_domain *domain, struct device *dev, int nvec,
                        msi_alloc_info_t *arg)
{
	struct msi_domain_info *info = msi_get_domain_info(domain);
	struct rcm_gic_msi_data *data = info->data;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct msi_desc *desc = first_pci_msi_entry(pdev);

	if (desc->msi_attrib.is_msix)
		dev_info(&data->pdev->dev, "%s: %d MSI-X\n", __func__, nvec);
	else
		dev_info(&data->pdev->dev, "%s: %d MSI\n", __func__, nvec);

	if ((desc->msi_attrib.is_msix) && (nvec > data->num_msix)) {
		dev_warn(&data->pdev->dev,
		         "Can't prepare %d MSI-X vectors\n", nvec);
		return data->num_msix;
	} else if ((!desc->msi_attrib.is_msix) && (nvec > 1)) {
		dev_warn(&data->pdev->dev,
		         "Can't prepare %d MSI vectors\n", nvec);
		return 1;
	}

	return 0;
}

static struct msi_domain_ops rcm_gic_msi_domain_ops = {
	.msi_prepare	= rcm_gic_msi_prepare,
};

static struct msi_domain_info rcm_gic_msi_domain_info = {
	.flags  = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
	           MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.chip   = &rcm_gic_msi_irq_chip,
	.ops    = &rcm_gic_msi_domain_ops,
};

static void rcm_gic_compose_msi_msg(struct irq_data *irq_data,
                                    struct msi_msg *msg)
{
	struct rcm_gic_msi_data *data = irq_data_get_irq_chip_data(irq_data);
	struct msi_desc *desc = irq_data_get_msi_desc(irq_data);

	phys_addr_t addr;

	if (desc->msi_attrib.is_msix) {
		addr = data->addr_gic_base + GIC_DIST_SOFTINT;
		msg->data = BIT(16) | irq_data->hwirq;
	} else {
		addr = data->dma_addr;
		msg->data = BIT(15);
	}

	msg->address_lo = lower_32_bits(addr);
	msg->address_hi = upper_32_bits(addr);

	dev_dbg(&data->pdev->dev, "msi%s#%d address_hi %#x address_lo %#x\n",
	        (desc->msi_attrib.is_msix) ? "x" : "", 
	        (int)irq_data->hwirq, msg->address_hi, msg->address_lo);
}

static int rcm_gic_msi_set_affinity(struct irq_data *irq_data,
                                    const struct cpumask *mask, bool force)
{
	 return -EINVAL;
}

static struct irq_chip rcm_gic_msi_bottom_irq_chip = {
	.name			= "RCM-MSI",
	.irq_compose_msi_msg	= rcm_gic_compose_msi_msg,
	.irq_set_affinity	= rcm_gic_msi_set_affinity,
};

static int rcm_gic_msi_irq_domain_alloc_one(struct irq_domain *domain,
                                            unsigned int virq, void *args)
{
	msi_alloc_info_t *info = args;
	struct rcm_gic_msi_data *data = domain->host_data;
	unsigned long bit;

	mutex_lock(&data->lock);

	if (info->desc->msi_attrib.is_msix) {
		bit = find_first_zero_bit(data->used, data->num_msix);
		if (bit >= data->num_msix) {
			mutex_unlock(&data->lock);
			return -ENOSPC;
		}
	} else {
		if (test_bit(data->num_msix, data->used)) {
			mutex_unlock(&data->lock);
			return -ENOSPC;
		}
		bit = data->num_msix;
	}

	set_bit(bit, data->used);

	mutex_unlock(&data->lock);

	if ((!info->desc->msi_attrib.is_msix) && (!data->thread_polling)) {
		data->thread_polling =
			kthread_run(rcm_gic_msi_polling, data,
			            "rcm_msi_polling");

		if (IS_ERR(data->thread_polling)) {
			data->thread_polling = NULL;
			mutex_lock(&data->lock);
			__clear_bit(bit, data->used);
			mutex_unlock(&data->lock);
			dev_err(&data->pdev->dev,
			        "Failed to create polling thread\n");
			return -EINVAL;
		}
	}

	irq_domain_set_info(domain, virq, bit, &rcm_gic_msi_bottom_irq_chip,
	                    domain->host_data, handle_simple_irq,
	                    NULL, NULL);

	return 0;
}

static void rcm_gic_msi_irq_domain_free_one(struct irq_domain *domain,
                                            unsigned int virq)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct rcm_gic_msi_data *data = irq_data_get_irq_chip_data(d);

	mutex_lock(&data->lock);

	if (!test_bit(d->hwirq, data->used)) {
		dev_err(&data->pdev->dev, "trying to free unused MSI#%lu\n",
		        d->hwirq);
	} else {
		__clear_bit(d->hwirq, data->used);
	}

	mutex_unlock(&data->lock);

	if ((d->hwirq == data->num_msix) && (data->thread_polling)) {
		kthread_stop(data->thread_polling);
		data->thread_polling = NULL;
	}
}


static void rcm_gic_msi_irq_domain_free(struct irq_domain *domain,
                                        unsigned int virq, unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; ++i)
		rcm_gic_msi_irq_domain_free_one(domain, virq + i);
}

static int rcm_gic_msi_irq_domain_alloc(struct irq_domain *domain,
                                        unsigned int virq, unsigned int nr_irqs,
                                        void *args)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nr_irqs; ++i) {
		ret = rcm_gic_msi_irq_domain_alloc_one(domain, virq + i, args);
		if (ret)
			break;
	}

	if (ret) {
		rcm_gic_msi_irq_domain_free(domain, virq, i);
	}

	return ret;
}

static const struct irq_domain_ops msi_domain_ops = {
	.alloc	= rcm_gic_msi_irq_domain_alloc,
	.free	= rcm_gic_msi_irq_domain_free,
};

static int rcm_gic_msi_allocate_domains(struct rcm_gic_msi_data *data)
{
	struct fwnode_handle *fwnode = 
		of_node_to_fwnode(data->pdev->dev.of_node);

	if (rcm_gic_msi_domain_info.data) {
		dev_err(&data->pdev->dev, "Can't create multiple RCM-GIC-MSI\n");
		return -EINVAL;
	}

	data->inner_domain = irq_domain_add_linear(NULL, data->num_msix + 1,
	                                           &msi_domain_ops, data);
	if (!data->inner_domain) {
		dev_err(&data->pdev->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	rcm_gic_msi_domain_info.data = data;

	data->msi_domain = 
		pci_msi_create_irq_domain(fwnode, &rcm_gic_msi_domain_info,
		                          data->inner_domain);
	if (!data->msi_domain) {
		dev_err(&data->pdev->dev, "failed to create MSI domain\n");
		irq_domain_remove(data->inner_domain);
		return -ENOMEM;
	}

	return 0;
}

static void rcm_gic_msi_free_domains(struct rcm_gic_msi_data *data)
{
	irq_domain_remove(data->msi_domain);
	irq_domain_remove(data->inner_domain);
	if (rcm_gic_msi_domain_info.data == data)
		rcm_gic_msi_domain_info.data = NULL;
}

static int rcm_gic_msi_remove(struct platform_device *pdev)
{
	struct rcm_gic_msi_data *data = platform_get_drvdata(pdev);

	rcm_gic_msi_free_domains(data);

	if (data->buff)
		dma_free_coherent(&data->pdev->dev, PAGE_SIZE,
		                  data->buff, data->dma_addr);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int rcm_gic_msi_probe(struct platform_device *pdev)
{
	struct rcm_gic_msi_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *gic;
	struct resource res;
	int ret;

	dev_info(&pdev->dev, "%s >>>", __func__);

	data = devm_kzalloc(&pdev->dev, sizeof(struct rcm_gic_msi_data),
	                    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);
	data->pdev = pdev;

	gic = of_parse_phandle(np, "gic", 0);
	if (!gic) {
		dev_err(&pdev->dev, "failed to find gic node\n");
		return -EFAULT;
	}

	dev_info(&pdev->dev, "GIC: dev = 0x%08X, fwnode = 0x%08X\n", (u32)gic->fwnode.dev, (u32)(&gic->fwnode));

	if (of_address_to_resource(gic, 0, &res) != 0) {
		dev_err(&pdev->dev, "Failed to get GIC base address\n");
		return -EINVAL;
	}

	data->addr_gic_base = res.start;
	dev_info(&pdev->dev, "GICD physical base is %#x\n",
	         data->addr_gic_base);

	if (of_property_read_u32(np, "num-msix", &data->num_msix)) {
		data->num_msix = 0;
	}

	if (of_property_read_u32(np, "usleep-polling", &data->usleep_polling)) {
		data->usleep_polling = POLLING_SLEEP;
	}

	data->buff = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &data->dma_addr,
	                                GFP_KERNEL);
	if (!data->buff) {
		dev_err(&pdev->dev, "failed to allocate DMA memory\n");
		return -ENOMEM;
	}

	ret = rcm_gic_msi_allocate_domains(data);
	if (ret) {
		dma_free_coherent(&data->pdev->dev, PAGE_SIZE,
		                  data->buff, data->dma_addr);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "%s <<<", __func__);

	return 0;
}

static const struct of_device_id rcm_gic_msi_of_match[] = {
	{ .compatible = "rcm,arm-gic-msi", NULL },
	{ },
};

static struct platform_driver rcm_gic_msi_driver = {
	.driver = {
		.name = "rcm-gic-msi",
		.of_match_table = rcm_gic_msi_of_match,
	},
	.probe = rcm_gic_msi_probe,
	.remove = rcm_gic_msi_remove,
};

module_platform_driver(rcm_gic_msi_driver);

MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_DESCRIPTION("RCM IRQ GIC MSI");
MODULE_LICENSE("GPL");
