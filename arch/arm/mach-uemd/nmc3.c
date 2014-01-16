/*
 * RC Module DCTS SoC. ARM-NMC3 interface platform stuff
 *
 * Copyright (C) RC Module 2011
 *	Gennadiy Kurtsman <gkurtsman@module.ru>
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/rcmod_soc_nmc3.h>

#include <mach/hardware.h>
#include <mach/nmc3.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/vic.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/irqs.h>

static struct resource ARM_NMC3_device_resources[] = {
	[0] = {
		.name		= "vic_LPintrs_ena",
		.start 		= UEMD_VIC0_PHYS_BASE + VIC_LPINTRS_ENA_OFF,
		.end		= (UEMD_VIC0_PHYS_BASE + VIC_LPINTRS_ENA_OFF +
				   sizeof (__u32)) - 1,
		.flags		= IORESOURCE_IO,
	},
	[1] = {
		.name		= "nmc3_reset_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_RESET_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_RESET_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[2] = {
		.name		= "nmc3_nmi_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_NMI_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_NMI_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[3] = {
		.name		= "nmc3_HPintrCLR_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_INTH_CLR_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_INTH_CLR_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[4] = {
		.name		= "nmc3_LPintrCLR_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_INTL_CLR_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_INTL_CLR_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[5] = {
		.name		= "nmc3_HPintrSET_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_INTH_SET_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_INTH_SET_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[6] = {
		.name		= "nmc3_LPintrSET_reg",
		.start		= SCTL_REGS_BASE + CTRL_NM_INTL_SET_REG,
		.end		= SCTL_REGS_BASE + CTRL_NM_INTL_SET_REG + sizeof(__u32) -1,
		.flags		= IORESOURCE_IO,
	},
	[7] = {
		.name		= "nmc3_bank_0_im1",
		.start		= NMC3_BANK_0_IM1,
		.end		= NMC3_BANK_0_IM1 + NMC3_BANK_LN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[8] = {
		.name		= "nmc3_bank_0_internal",
		.start		= NMC3_BANK_0_INTERNAL,
		.end		= NMC3_BANK_0_INTERNAL_END,
		.flags		= IORESOURCE_DMA,
	},
	[9] = {
		.name		= "nmc3_bank_1_im1",
		.start		= NMC3_BANK_1_IM1,
		.end		= NMC3_BANK_1_IM1 + NMC3_BANK_LN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[10] = {
		.name		= "nmc3_bank_1_internal",
		.start		= NMC3_BANK_1_INTERNAL,
		.end		= NMC3_BANK_1_INTERNAL_END,
		.flags		= IORESOURCE_DMA,
	},
	[11] = {
		.name		= "nmc3_bank_2_im3",
		.start		= NMC3_BANK_2_IM3,
		.end		= NMC3_BANK_2_IM3 + NMC3_BANK_LN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[12] = {
		.name		= "nmc3_bank_2_internal",
		.start		= NMC3_BANK_2_INTERNAL,
		.end		= NMC3_BANK_2_INTERNAL_END,
		.flags		= IORESOURCE_DMA,
	},
	[13] = {
		.name		= "nmc3_bank_3_im3",
		.start		= NMC3_BANK_3_IM3,
		.end		= NMC3_BANK_3_IM3 + NMC3_BANK_LN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[14] = {
		.name		= "nmc3_bank_3_internal",
		.start		= NMC3_BANK_3_INTERNAL,
		.end		= NMC3_BANK_3_INTERNAL_END,
		.flags		= IORESOURCE_DMA,
	},
	[15] = {
		.name		= "im0",
		.start		= NMC3_BANK_IM0,
		.end		= NMC3_BANK_IM0 + NMC3_BANK_IM0_LN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[16] = {
		.name		= "im0_internal",
		.start		= NMC3_BANK_IM0_INTERNAL,
		.end		= NMC3_BANK_IM0_INTERNAL_END,
		.flags		= IORESOURCE_DMA,
	},
	[17] = {
		.start		= NMC3_HP_IRQ,
		.end		= NMC3_HP_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
	[18] = {
		.start		= NMC3_LP_IRQ,
		.end		= NMC3_LP_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device ARM_NMC3_device = {
	.name		= "rcmod_soc_nmc3",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ARM_NMC3_device_resources),
	.resource	= ARM_NMC3_device_resources,
};

static struct platform_device* ARM_NMC3_devices [] = {
	&ARM_NMC3_device,
};

static int __init nmc3_init (void)
{
	int i;
	int ret = 0;

	dbg ("%s",__func__);
	for (i=0; i<ARRAY_SIZE(ARM_NMC3_devices); i++) {
		ret = platform_device_register(ARM_NMC3_devices[i]);
		if (ret<0) {
			printk ("platform_device_register() ERROR %d\n", ret);
			break;
		}
	}
	dbg ("ret = %d",ret);
	return ret;
}
device_initcall (nmc3_init);
