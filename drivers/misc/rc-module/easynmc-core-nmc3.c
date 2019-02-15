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
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/easynmc.h>


#define DRVNAME "easynmc-nmc3"

struct nmc_control_reg {
	uint32_t offset;
	uint32_t bit;
};

struct nmc3_core { 
	struct nmc_core c;
	struct regmap *control;
	struct regmap *reset;
	uint32_t reset_bit;
	struct nmc_control_reg lp_status_reg;
	struct nmc_control_reg hp_status_reg;
	struct nmc_control_reg lp_set_reg;
	struct nmc_control_reg hp_set_reg;
	struct nmc_control_reg nmi_set_reg;
	struct nmc_control_reg lp_clr_reg;
	struct nmc_control_reg hp_clr_reg;
};

#define UNLOCK 0x1ACCE551

void nmc3_reset(struct nmc_core *self) 
{
	struct nmc3_core *core = (struct nmc3_core *) self;

	// active reset low
	regmap_write(core->reset, 0x00c, UNLOCK);   // allow changing other regs
	regmap_update_bits(core->reset, 0x018, BIT(core->reset_bit), 0); // write 0 to reset
	regmap_update_bits(core->reset, 0x018, BIT(core->reset_bit), BIT(core->reset_bit)); // write 1 to reset
	regmap_write(core->reset, 0x00c, 0);   // disable changing other regs

	// clear all interrupts
	regmap_write(core->control, core->hp_clr_reg.offset, BIT(core->hp_clr_reg.bit));
	regmap_write(core->control, core->lp_clr_reg.offset, BIT(core->lp_clr_reg.bit));

}

void nmc3_send_interrupt(struct nmc_core *self, enum nmc_irq n) 
{
	// all interrupt set bit cleared automaticaly after set 
	struct nmc3_core *core = (struct nmc3_core *) self;
	switch (n) {
	case NMC_IRQ_NMI:
		// clear all interrupts
		regmap_write(core->control, core->hp_clr_reg.offset, BIT(core->hp_clr_reg.bit));
		regmap_write(core->control, core->lp_clr_reg.offset, BIT(core->lp_clr_reg.bit));

		regmap_write(core->control, core->nmi_set_reg.offset, 0);
		regmap_write(core->control, core->nmi_set_reg.offset, BIT(core->nmi_set_reg.bit));
		break;
	case NMC_IRQ_HP:
		regmap_write(core->control, core->hp_set_reg.offset, BIT(core->hp_set_reg.bit));
		break;
	case NMC_IRQ_LP:
		regmap_write(core->control, core->lp_set_reg.offset, BIT(core->lp_set_reg.bit));
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
		regmap_write(core->control, core->hp_clr_reg.offset, BIT(core->hp_clr_reg.bit));
		break;
	case NMC_IRQ_LP:
		regmap_write(core->control, core->lp_clr_reg.offset, BIT(core->lp_clr_reg.bit));
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
	struct device_node		*np = pdev->dev.of_node;
	struct device_node *tmp;

	if (!core) 
		return -ENOMEM;

	memset(core, 0, sizeof(struct nmc3_core));

	// get control registers for reset and interrupt

	tmp = of_parse_phandle(np, "control", 0);
	if (!tmp) {
		printk(DRVNAME ": failed to find control register reference\n");
		goto errfreemem;					
	}
	core->control = syscon_node_to_regmap(tmp);

	tmp = of_parse_phandle(np, "reset", 0);
	if (!tmp) {
		printk(DRVNAME ": failed to find reset register reference\n");
		goto errfreemem;					
	}
	core->reset = syscon_node_to_regmap(tmp);




#define GRAB_IRQ_RESOURCE(name, to)					\
 	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, #name); \
	if (!res) {							\
		printk(DRVNAME ": failed to get irq resource " #name " \n"); \
		goto errfreemem;					\
									\
	}								\
	to=res->start;
	
#define GRAB_NMC_REG(name)	\
	if(of_property_read_u32_array(np, #name, (uint32_t*) &(core->name), 2)) \
	{ \
		printk(DRVNAME ": unable to get " #name " value"); \
		goto errfreemem; \
	} 

	GRAB_NMC_REG(lp_status_reg);
	GRAB_NMC_REG(hp_status_reg);
	GRAB_NMC_REG(lp_set_reg);
	GRAB_NMC_REG(hp_set_reg);
	GRAB_NMC_REG(nmi_set_reg);
	GRAB_NMC_REG(hp_clr_reg);
	GRAB_NMC_REG(lp_clr_reg);	




	if(of_property_read_u32(np, "reset_bit", (uint32_t*) &(core->reset_bit)))
	{
		printk(DRVNAME ": unable to get reset_bit value");		
		goto errfreemem;
	} 

	printk(DRVNAME ": nmi_set_reg: %x %x", core->nmi_set_reg.offset, core->nmi_set_reg.bit);		

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

	{
		uint32_t val;
		
		if( regmap_read(core->control, core->hp_status_reg.offset, &val) )  // read reset reg value
		{
			printk(DRVNAME ": Warning - unable to access reset regmap\n");
			goto errfreemem;
		}
		printk(DRVNAME ": IRQ status: %08X\n", val);
	}
	
	nmc3_clear_interrupt(&core->c, NMC_IRQ_HP);
	nmc3_clear_interrupt(&core->c, NMC_IRQ_LP);

	{
		uint32_t val;
		
		if( regmap_read(core->control, core->hp_status_reg.offset, &val) )  // read reset reg value
		{
			printk(DRVNAME ": Warning - unable to access reset regmap\n");
			goto errfreemem;
		}
		printk(DRVNAME ": IRQ status after clear: %08X\n", val);
	}


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
