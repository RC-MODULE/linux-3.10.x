/*
 *  arch/arm/mach-uemd/uemd.h
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

#ifndef __ASM_ARM_UEMD_H
#define __ASM_ARM_UEMD_H

#include "platform.h"

/* Use unprotected read/write with UEMD hardware */
#define uemd_raw_writel __raw_writel
#define uemd_raw_readl __raw_readl

void __init uemd_timer_init(void);

extern void mdevice_platform_register(void);
extern void mdevice_preallocate_buffers(void);

void __init pl192_init(void __iomem *base, unsigned int irq_start,
		     u32 pl192_sources, u32 resume_sources);
void __iomem *uemd_mif_base(void);

#endif

