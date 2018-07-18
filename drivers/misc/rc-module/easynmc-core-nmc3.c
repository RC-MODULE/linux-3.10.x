/*
 * EasyNMC NMC3 core Driver
 * (c) RC Module 2014
 *
 * This driver provides a simple interface to userspace apps and 
 * is designed to be as simple as possible. 
 * Cores are registered with this framework by calling 
 *
 * easynmc_register_core() from respective platform drivers. 
 * 
 * A  
 *
 * 
 * License terms: GNU General Public License (GPL) version 2
 * Author: Andrew Andrianov <andrew@ncrmnt.org>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/easynmc.h>


#define DRVNAME "easynmc-nmc3"

struct nmc3_core { 
	struct nmc_core c;
	uint32_t *nmi_reg;
	uint32_t *reset_reg;
	uint32_t *hp_clr_reg;
	uint32_t *lp_clr_reg;
	uint32_t *hp_set_reg;
	uint32_t *lp_set_reg;
};


void nmc3_reset(struct nmc_core *self) 
{
	struct nmc3_core *core = (struct nmc3_core *) self;
	iowrite32(0xff, core->reset_reg);
}

void nmc3_send_interrupt(struct nmc_core *self, enum nmc_irq n) 
{
	struct nmc3_core *core = (struct nmc3_core *) self;
	switch (n) {
	case NMC_IRQ_NMI:
		iowrite32(0x1, core->nmi_reg);
		iowrite32(0x0, core->nmi_reg);
		break;
	case NMC_IRQ_HP:
		iowrite32(0x1, core->hp_set_reg);
		break;
	case NMC_IRQ_LP:
		iowrite32(0x1, core->lp_set_reg);
		break;		
	default:
		printk(DRVNAME ": Warning - invalid IRQ request - %d\n", n);
	}
}

void nmc3_clear_interrupt(struct nmc_core *self, enum nmc_irq n) 
{
	struct nmc3_core *core = (struct nmc3_core *) self;
	switch (n) {
	case NMC_IRQ_HP:
		iowrite32(0x1, core->hp_clr_reg);
		break;
	case NMC_IRQ_LP:
		iowrite32(0x1, core->lp_clr_reg);
		break;		
	default:
		printk(DRVNAME ": Warning - invalid IRQ clear request - %d\n", n);
	}
}


static int easynmc_probe (struct platform_device *pdev)
{
	int ret=-EIO;
	struct resource *res; 
	struct nmc3_core *core = kmalloc(sizeof(struct nmc3_core), GFP_KERNEL);
	if (!core) 
		return -ENOMEM;

#define GRAB_MEM_RESOURCE(name)						\
 	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, #name); \
	if (!res) {							\
		printk(DRVNAME ": failed to get mem resource " #name " \n"); \
		goto errfreemem;					\
									\
	}								\
	res->flags &= ~(IORESOURCE_CACHEABLE);				\
	core->name = devm_ioremap_resource(&pdev->dev, res);		\
	if (IS_ERR(core->name)) {					\
		printk(DRVNAME ": request/remap failed for " #name " \n"); \
		goto errfreemem;					\
	}								\

#define GRAB_IRQ_RESOURCE(name, to)					\
 	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, #name); \
	if (!res) {							\
		printk(DRVNAME ": failed to get irq resource " #name " \n"); \
		goto errfreemem;					\
									\
	}								\
	to=res->start;
	

	GRAB_MEM_RESOURCE(reset_reg);
	GRAB_MEM_RESOURCE(nmi_reg);
	GRAB_MEM_RESOURCE(hp_clr_reg);
	GRAB_MEM_RESOURCE(hp_set_reg);
	GRAB_MEM_RESOURCE(lp_clr_reg);
	GRAB_MEM_RESOURCE(lp_set_reg);

	GRAB_IRQ_RESOURCE(hp, core->c.irqs[NMC_IRQ_HP]);
	GRAB_IRQ_RESOURCE(lp, core->c.irqs[NMC_IRQ_LP]);
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "imem");	
	core->c.imem_size = res->end - res->start + 1;
	core->c.imem_phys = res->start;
	core->c.imem_virt = devm_ioremap_resource(&pdev->dev, res);

	core->c.reset             = nmc3_reset; 
	core->c.send_interrupt    = nmc3_send_interrupt; 
	core->c.clear_interrupt   = nmc3_clear_interrupt; 
	
	if (!core->c.imem_virt) {
		printk(DRVNAME ": ioremap of internal nmc memory failed");
		goto errfreemem;
	}
	
	ret =  of_property_read_string(pdev->dev.of_node, "core-name",
				       &core->c.name);
	if (ret!=0) 
		goto errfreemem;

	ret =  of_property_read_string(pdev->dev.of_node, "core-type",
				       &core->c.type);
	if (ret!=0) 
		goto errfreemem;
	
	printk(DRVNAME ": imem at phys 0x%lx virt 0x%lx size 0x%lx bytes\n", 
	       (unsigned long) core->c.imem_phys,
	       (unsigned long) core->c.imem_virt,
	       (unsigned long) core->c.imem_size);
	printk(DRVNAME ": HP IRQ %d LP IRQ %d\n", 
	       core->c.irqs[NMC_IRQ_HP],
	       core->c.irqs[NMC_IRQ_LP]
		);	

	core->c.dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, core);
	easynmc_register_core(&core->c);
	
	
	return 0;
errfreemem:
	kfree(core);
	return ret;
}


static int easynmc_remove (struct platform_device *pdev)
{
	// ToDo: devm_unmap ?
	struct nmc3_core *core = dev_get_drvdata(&pdev->dev);
	if (!core)
		BUG();
	return easynmc_deregister_core(&core->c);
}

static const struct of_device_id easynmc_nmc3_of_match_table[] = {
	{ .compatible = "rc-module,easynmc", },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, easynmc_nmc3_of_match_table);

static struct platform_driver easynmc_nmc3_driver = {
	.probe		= easynmc_probe,
	.remove		= easynmc_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRVNAME,
		.of_match_table = of_match_ptr(easynmc_nmc3_of_match_table),
	},
};
module_platform_driver(easynmc_nmc3_driver);
MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_DESCRIPTION("EasyNMC NMC3 Core Driver");
MODULE_LICENSE("GPL v2");
