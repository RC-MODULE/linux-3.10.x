/*
 *  arch/arm/mach-rcm-k1879/board-dt.c
 *
 *  Copyright (C) 2011-2015 RC Module
 *
 *  Sergey Mironov <ierton@gmail.com>
 *  Andrew Andrianov <andrew@ncrmnt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/io.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/setup.h>
#include <asm/memblock.h>

#include <linux/irqchip/arm-vic.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>

#include <linux/serial_8250.h>
#include <mach/hardware.h>

static struct map_desc k1879_io_desc[] __initdata = {
	{
		.virtual	= RCM_K1879_AREA0_VIRT_BASE,
		.pfn		= __phys_to_pfn(RCM_K1879_AREA0_PHYS_BASE),
		.length		= RCM_K1879_AREA0_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= RCM_K1879_AREA1_VIRT_BASE,
		.pfn		= __phys_to_pfn(RCM_K1879_AREA1_PHYS_BASE),
		.length		= RCM_K1879_AREA1_SIZE,
		.type		= MT_DEVICE,
	},
};

static void __iomem *g_k1879_mif;

static void __iomem *k1879_mif_base(void)
{
	BUG_ON(g_k1879_mif == 0);
	return g_k1879_mif;
}

static void __iomem *k1879_sctl_base(void)
{
	return (void __iomem *) RCM_K1879_SCTL_VIRT_BASE;
}

static void k1879_level_irq_i2c0_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1, k1879_mif_base() + RCM_K1879_MIF_I2C_INT_STAT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c1_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<0, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c2_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<1, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void k1879_level_irq_i2c3_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<2, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void __init k1879_map_io(void)
{
	iotable_init(k1879_io_desc, ARRAY_SIZE(k1879_io_desc));
}

static const struct of_device_id i2c_of_match[] __initconst = {
	{ .compatible = "aeroflexgaisler,i2cmst", },
	{}
};

static void __init setup_i2c_fixup(u64 base,
				   void (*fixup_handler)(unsigned int irq,
							 struct irq_desc *desc))
{
	struct device_node *np;
	int irq;

	np = of_find_matching_node_by_address(NULL, i2c_of_match, base);
	BUG_ON(NULL == np);
	irq = of_irq_get(np, 0);
	irq_set_handler(irq, fixup_handler);
}

static void __init k1879_dt_mach_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	g_k1879_mif = ioremap_nocache(RCM_K1879_MIF_PHYS_BASE,
				      RCM_K1879_MIF_SIZE);
	BUG_ON(g_k1879_mif == NULL);

	/* Setup i2c interrupt fixups */
	setup_i2c_fixup(RCM_K1879_GRI2C0_PHYS_BASE, k1879_level_irq_i2c0_fixup);
	setup_i2c_fixup(RCM_K1879_GRI2C1_PHYS_BASE, k1879_level_irq_i2c1_fixup);
	setup_i2c_fixup(RCM_K1879_GRI2C2_PHYS_BASE, k1879_level_irq_i2c2_fixup);
	setup_i2c_fixup(RCM_K1879_GRI2C3_PHYS_BASE, k1879_level_irq_i2c3_fixup);
	/* I2C0 (Internal, HDMI) needs some extra love */
	do {
		void __iomem *mif;

		mif = k1879_mif_base();
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE_ENA);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_ENA);
	} while (0);
}

void k1879_restart(enum reboot_mode mode, const char *cmd)
{
	/* The recommended way to do a soft-reboot on this platform
	   is write directly to watchdog registers and cause an immediate
	   system reboot
	*/
	void __iomem *regs;

	pr_info("k1879: Requested system restart\n");
	regs = ioremap_nocache(0x20025000, 0xfff);
	iowrite32(1, (regs + 0xf00));
	iowrite32(1, (regs + 0xf04));
	/* Goodbye! */
}

static const char * const module_dt_match[] = {
	"rcm,mb7707",
	"rcm,k1879x",
	NULL
};

DT_MACHINE_START(K1879, "RC Module K1879XB1YA")
	.map_io                 = k1879_map_io,
	.init_machine           = k1879_dt_mach_init,
	.dt_compat              = module_dt_match,
	.restart                = k1879_restart
MACHINE_END
