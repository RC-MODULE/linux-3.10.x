/*
 *  arch/arm/mach-uemd/include/mach/io.h
 *
 *  Copyright (C) 2011 Sergey Mironov <ierton@gmail.com>
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
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <asm/mach-types.h>
#include <mach/hardware.h>

#ifdef CONFIG_PCI
#error "UEMD PCI is not supported"
#endif

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a)	             __typesafe_io(a)
#define __mem_pci(a)         (a)


#define UEMD_MIF_I2C_INT_TYPE_ENA  0x94
#define UEMD_MIF_I2C_INT_TYPE      0x98
#define UEMD_MIF_I2C_INT_ENA       0x9C
#define UEMD_MIF_I2C_INT_STAT      0xA0
void __iomem * uemd_mif_base(void);

#define UEMD_SCTL_INT_P_OUT        0x08
void __iomem *uemd_sctl_base(void);

#endif

