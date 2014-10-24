/*
 *  arch/arm/mach-uemd/uemd.c
 *
 *  Copyright (C) 2011
 *
 *  Sergey Mironov <ierton@gmail.com>
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
#include <mach/platform.h>

static struct map_desc k1879_io_desc[] __initdata = {
	{
		.virtual	= RCM_K1879_AREA0_VIRT_BASE,
		.pfn		= __phys_to_pfn(RCM_K1879_AREA0_PHYS_BASE),
		.length		= RCM_K1879_AREA0_SIZE,
		.type		= MT_DEVICE,
	},
	// FIXME: This static mapping is actually redundant. We are using it for
	// direct access to multimedia registers.
	{
		.virtual	= 0xF9000000,
		.pfn		= __phys_to_pfn(RCM_K1879_AREA1_PHYS_BASE),
		.length		= 2*RCM_K1879_AREA1_SIZE,
		.type		= MT_DEVICE,
	}
};

static void __init k1879_map_io(void)
{
	iotable_init(k1879_io_desc, ARRAY_SIZE(k1879_io_desc));
}

void __init k1879_timer_init(void);

static void __init k1879_dt_timer_init(void)
{
        of_clk_init(NULL);
	clocksource_of_init();
        k1879_timer_init();
}

static void __init k1879_dt_mach_init(void)
{
	printk("K1879: Initializing machine\n");
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

/* Lookup table for finding a DT node that represents the vic instance */
static const struct of_device_id vic_of_match[] __initconst = {
	{ .compatible = "arm,pl192-vic", },
	{}
};

static void __init k1879_init_irq(void)
{
	struct device_node *np;
	printk("K1879: Starting IRQ subsystem\n");
	np = of_find_matching_node_by_address(NULL, vic_of_match,
					      RCM_K1879_VIC0_PHYS_BASE);
	BUG_ON(!np);

	__vic_init(RCM_K1879_VIC0_VIRT_BASE, 0, 32,  ~0, 0, np);

	np = of_find_matching_node_by_address(NULL, vic_of_match,
					      RCM_K1879_VIC1_PHYS_BASE);

	BUG_ON(!np);
	__vic_init(RCM_K1879_VIC0_VIRT_BASE, 0, 64,  ~0, 0, np);

}


static const char *module_dt_match[] = {
        "module,mb7707",
        NULL
};

DT_MACHINE_START(UEMD, "Module MB77.07")
          .map_io                 = k1879_map_io,
          .init_irq               = k1879_init_irq,
          .init_time              = k1879_dt_timer_init,
          .init_machine           = k1879_dt_mach_init,
          .dt_compat              = module_dt_match,
//        .reserve                = uemd_reserve,
//        .restart                = uemd_pm_restart
MACHINE_END
