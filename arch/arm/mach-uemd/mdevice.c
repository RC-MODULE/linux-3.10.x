/*
 *  arch/arm/mach-uemd/mdevice.c
 *
 *  Copyright (C) 2010  RC Module
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

#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/serial_8250.h>
#include <linux/module_vdu.h>
#include <linux/module_vdu_grabber.h>
#include <linux/i2c-ocores-gaisler.h>
#include <linux/gshark.h>
#include <linux/msvdhd.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/amba/pl061.h>

#include <linux/mdemux.h>

#include "uemd.h"
#include "module_hdmi.h"

#define MDEVICE_BASE(dev)  UEMD_##dev##_PHYS_BASE
#define MDEVICE_VBASE(dev) UEMD_##dev##_VIRT_BASE
#define MDEVICE_SIZE(dev)  UEMD_##dev##_SIZE
#define MDEVICE_IRQ(dev)   UEMD_##dev##_IRQ

#define MDEVICE_CONST(name)   UEMD_##name

#if defined(CONFIG_MODULE_TSP) || defined(CONFIG_MDEMUX2)
struct mdemux_platform_data mdemux_pldata = {
	.i2c_bus_id = 2,
	.i2c_dvbs_demodulator_addr = 0xD4>>1,
	.i2c_dvbs_tuner_addr = 0xC0>>1,
	.i2c_dvbs_lnb_addr = 0x8,
};

static struct resource mdemux_resources[] = {
	[0] = {
		.name		= "registers",
		.start		= MDEVICE_BASE(TSP),
		.end		= MDEVICE_BASE(TSP) + PAGE_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.name		= "irq",
		.start		= MDEVICE_IRQ(TSP),
		.end		= MDEVICE_IRQ(TSP),
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device mdemux_dev = {
	.name = "module_demux",
	.id	= -1,
	.num_resources = ARRAY_SIZE(mdemux_resources),
	.resource = mdemux_resources,
	.dev = {
		.coherent_dma_mask	= ~0,
		.platform_data          = &mdemux_pldata
	},
};

static void __init mdemux_device_register(void)
{
	int ret;
	printk(KERN_INFO "mdemux: registering platform device\n");
	ret = platform_device_register(&mdemux_dev);
	if (ret)
		printk(KERN_WARNING "mdemux: unable to register platform device\n");
}


static struct resource mdvbci_resources[] = {
       [0] = {
               .name           = "registers_dvbci",
               .start          = 0x10050000,
               .end            = 0x1005ffff,
               .flags          = IORESOURCE_MEM,
       },
       [1] = {
               .name           = "irq",
               .start          = 37,
               .end            = 37,
               .flags          = IORESOURCE_IRQ,
       },
};

struct platform_device mdvbci_dev = {
       .name = "module_dvbci",
       .id     = -1,
       .num_resources = ARRAY_SIZE(mdemux_resources),
       .resource = mdvbci_resources,
       .dev = {
               .coherent_dma_mask      = ~0,
       },
};

static void __init mdvbci_device_register(void)
{
       int ret;
       printk(KERN_INFO "mdvbci: registering platform device\n");
       ret = platform_device_register(&mdvbci_dev);
       if (ret)
	       printk(KERN_WARNING "mdvbci: unable to register platform device\n");
}

#else
static inline void mdemux_device_register(void) {}
static inline void mdvbci_device_register(void) {}
#endif /* CONFIG_MODULE_TSP */

#if defined (CONFIG_MODULE_UARTIRDA)
static struct plat_serial8250_port uemd_uartirda_data[] = {
	{
		.mapbase	= MDEVICE_BASE(UARTIRDA),
		.membase	= (void*)MDEVICE_VBASE(UARTIRDA),
		.irq		= MDEVICE_IRQ(UARTIRDA),
		.flags		= UPF_SKIP_TEST | UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= UEMD_UARTIRDA_CLK,
	}, {
	},
};

static struct resource uemd_uartirda_resources[] = {
	{
		.start		= MDEVICE_BASE(UARTIRDA),
		.end		= MDEVICE_BASE(UARTIRDA) + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= MDEVICE_IRQ(UARTIRDA),
		.end		= MDEVICE_IRQ(UARTIRDA),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device uemd_uartirda = {
	.name = "serial8250",
	.id	= 2,
	.dev = {
		.platform_data	= uemd_uartirda_data,
	},
	.resource = uemd_uartirda_resources,
	.num_resources = ARRAY_SIZE(uemd_uartirda_resources),
};

static void __init register_uartirda_device(void)
{
	int ret;
	printk(KERN_INFO "uartirda: registering platform device\n");
	ret = platform_device_register(&uemd_uartirda);
	if (ret)
		printk(KERN_WARNING "uartirda: unable to register platform device\n");
}
#else
static void __init register_uartirda_device(void) {}
#endif /*CONFIG_MODULE_UARTIRDA*/

#if defined(CONFIG_MODULE_SCI)
static struct resource msci_resources[] = {
	[0] = {
		.name		= "registers",
		.start		= MDEVICE_BASE(SCI),
		.end		= MDEVICE_BASE(SCI) + 0x80 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.name		= "irq",
		.start		= MDEVICE_IRQ(SCI),
		.end		= MDEVICE_IRQ(SCI),
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device msci_dev = {
	.name = "msci",
	.id	= -1,
	.num_resources = ARRAY_SIZE(msci_resources),
	.resource = msci_resources,
};

static void __init msci_device_register(void)
{
	int ret;
	printk(KERN_INFO "msci: registering MSCI device\n");
	ret = platform_device_register(&msci_dev);
	if (ret)
		printk(KERN_WARNING "msci: unable to register MSCI device\n");
}
#else
static inline void msci_device_register(void){}
#endif /* CONFIG_MODULE_SCI */

void mdevice_platform_register(void)
{
	mdemux_device_register();
	mdvbci_device_register();
	register_uartirda_device();
	msci_device_register();
}
