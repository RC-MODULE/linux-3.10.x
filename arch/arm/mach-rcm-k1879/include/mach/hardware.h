/*
 *  arch/arm/mach-rcm-k1879/include/mach/hardware.h
 *
 *  Copyright (C) 2011 RC Module
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

#ifndef ASM_RCM_K1879_HARDWARE_H
#define ASM_RCM_K1879_HARDWARE_H

#define RCM_K1879_PHYS(bus, off)        (RCM_K1879_##bus##_PHYS_BASE + (off))
#define RCM_K1879_VIRT(bus, off)        (RCM_K1879_##bus##_VIRT_BASE + (off))

/* Areas of system memory space */
#define RCM_K1879_AREA0_PHYS_BASE      0x20000000
#define RCM_K1879_AREA0_VIRT_BASE      0xf8000000
#define RCM_K1879_AREA0_SIZE           SZ_1M

#define RCM_K1879_AREA1_PHYS_BASE      0x80000000
#define RCM_K1879_AREA1_SIZE           SZ_1M

#define RCM_K1879_AREA2_PHYS_BASE      0x10040000
#define RCM_K1879_AREA2_SIZE           (0x10050000-0x1004000)

/*
 * Bare minimum required to bring up the board.
 * The rest comes from DeviceTree
 */

#define RCM_K1879_UART0_OFF            0x0002b000
#define RCM_K1879_UART0_PHYS_BASE      RCM_K1879_PHYS(AREA0, RCM_K1879_UART0_OFF)
#define RCM_K1879_UART0_VIRT_BASE      RCM_K1879_VIRT(AREA0, RCM_K1879_UART0_OFF)

#define RCM_K1879_GRI2C1_OFF           0x00021000
#define RCM_K1879_GRI2C1_PHYS_BASE     RCM_K1879_PHYS(AREA0, RCM_K1879_GRI2C1_OFF)
#define RCM_K1879_GRI2C1_VIRT_BASE     RCM_K1879_VIRT(AREA0, RCM_K1879_GRI2C1_OFF)

#define RCM_K1879_GRI2C2_OFF           0x00026000
#define RCM_K1879_GRI2C2_PHYS_BASE     RCM_K1879_PHYS(AREA0, RCM_K1879_GRI2C2_OFF)
#define RCM_K1879_GRI2C2_VIRT_BASE     RCM_K1879_VIRT(AREA0, RCM_K1879_GRI2C2_OFF)

#define RCM_K1879_GRI2C3_OFF           0x0002d000
#define RCM_K1879_GRI2C3_PHYS_BASE     RCM_K1879_PHYS(AREA0, RCM_K1879_GRI2C3_OFF)
#define RCM_K1879_GRI2C3_VIRT_BASE     RCM_K1879_VIRT(AREA0, RCM_K1879_GRI2C3_OFF)

#define RCM_K1879_SCTL_OFF             0x0003c000
#define RCM_K1879_SCTL_PHYS_BASE       RCM_K1879_PHYS(AREA0, RCM_K1879_SCTL_OFF)
#define RCM_K1879_SCTL_VIRT_BASE       RCM_K1879_VIRT(AREA0, RCM_K1879_SCTL_OFF)
#define RCM_K1879_SCTL_SIZE            0x1000

#define RCM_K1879_MIF_OFF              0x00172000
#define RCM_K1879_MIF_PHYS_BASE        RCM_K1879_PHYS(AREA1, RCM_K1879_MIF_OFF)
#define RCM_K1879_MIF_SIZE             0x100

#define RCM_K1879_GRI2C0_OFF           0x00171000
#define RCM_K1879_GRI2C0_PHYS_BASE     RCM_K1879_PHYS(AREA1, RCM_K1879_GRI2C0_OFF)

#define RCM_K1879_MIF_I2C_INT_TYPE_ENA  0x94
#define RCM_K1879_MIF_I2C_INT_TYPE      0x98
#define RCM_K1879_MIF_I2C_INT_ENA       0x9C
#define RCM_K1879_MIF_I2C_INT_STAT      0xA0
#define RCM_K1879_SCTL_INT_P_OUT        0x08

#endif
