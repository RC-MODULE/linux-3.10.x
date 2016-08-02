/*
 *  arch/arm/mach-rcm-k1879/board-dt.c
 *
 *  Copyright (C) 2011-2015 RC Module
 *
 *  Sergey Mironov <ierton@gmail.com>
 *  Andrew Andrianov <andrew@ncrmnt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include "hardware.h"

static void __iomem *k1879_mif;
static void __iomem *k1879_wdt;

static void __iomem *k1879_mif_base(void)
{
	BUG_ON(!k1879_mif);
	return k1879_mif;
}

static void __iomem *k1879_sctl_base(void)
{
	return (void __iomem *)RCM_K1879_SCTL_VIRT_BASE;
}

static void k1879_level_irq_i2c0_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1, k1879_mif_base() + RCM_K1879_MIF_I2C_INT_STAT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c1_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1 << 0, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c2_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1 << 1, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c3_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1 << 2, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}


static void (*k1879_i2c_fixups[])(unsigned int irq, struct irq_desc *desc) = {
	k1879_level_irq_i2c0_fixup,
	k1879_level_irq_i2c1_fixup,
	k1879_level_irq_i2c2_fixup,
	k1879_level_irq_i2c3_fixup
};

static void __init setup_i2c_fixups(void)
{
	struct device_node *np;
	int irq;
	u32 fixup_id = -1;

	for_each_compatible_node(np, NULL, "rcm,i2c-ocores") {
		int ret = of_property_read_u32(np, "rcm-irq-fixup-id",
					       &fixup_id);

		if (ret) {
			pr_warn("k1879xb1: rcm-irq-fixup-id invalid: %d\n",
				ret);
			continue;
		}
		if (fixup_id >= ARRAY_SIZE(k1879_i2c_fixups)) {
			pr_warn("k1879xb1: rcm-irq-fixup-id %d too big\n",
				fixup_id);
			continue;
		}
		irq = of_irq_get(np, 0);
		irq_set_handler(irq, k1879_i2c_fixups[fixup_id]);
	}
}

static void __init k1879_dt_mach_init(void)
{
	struct device_node *np;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	k1879_mif = ioremap(RCM_K1879_MIF_PHYS_BASE,
			    RCM_K1879_MIF_SIZE);
	BUG_ON(!k1879_mif);

	np = of_find_compatible_node(NULL, NULL, "arm,sp805");
	WARN_ON(!np);
	if (np)
		k1879_wdt = of_iomap(np, 0);

	/* Setup i2c interrupt fixups */
	setup_i2c_fixups();

	/* I2C0 (Internal, HDMI) needs some extra love */
	do {
		void __iomem *mif;

		mif = k1879_mif_base();
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE_ENA);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_ENA);
	} while (0);
}

static void k1879_restart(enum reboot_mode mode, const char *cmd)
{
	/* The recommended way to do a soft-reboot on this platform
	   is write directly to watchdog registers and cause an immediate
	   watchdog restart
	*/

	if (!k1879_wdt) {
		pr_err("k1879xb1: WDT regs were not properly mapped - reboot manually\n");
		return;
	}
	iowrite32(1, (k1879_wdt + 0xf00));
	iowrite32(1, (k1879_wdt + 0xf04));
	/* Goodbye! */
}

static const char * const k1879_dt_match[] = {
	"rcm,mb7707",
	"rcm,k1879xb1ya",
	NULL
};

static struct map_desc k1879_io_desc[] __initdata = {
	{
		.virtual	= RCM_K1879_AREA0_VIRT_BASE,
		.pfn	= __phys_to_pfn(RCM_K1879_AREA0_PHYS_BASE),
		.length	= RCM_K1879_AREA0_SIZE,
		.type	= MT_DEVICE,
	},
	// FIXME: This static mapping is actually redundant. We are using it for
	// direct access to multimedia registers.
	{
		.virtual	= 0xF9000000,
		.pfn	= __phys_to_pfn(RCM_K1879_AREA1_PHYS_BASE),
		.length	= 2*RCM_K1879_AREA1_SIZE,
		.type	= MT_DEVICE,
	}
};

static void __init k1879_map_io(void) {
    iotable_init(k1879_io_desc, ARRAY_SIZE(k1879_io_desc));
}

DT_MACHINE_START(K1879, "RC Module K1879XB1YA (Device Tree)")
	.map_io			= k1879_map_io,
	.init_machine           = k1879_dt_mach_init,
	.dt_compat              = k1879_dt_match,
	.restart                = k1879_restart
MACHINE_END
