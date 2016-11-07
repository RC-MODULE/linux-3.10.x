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
#ifndef HARDWARE_H
#define HARDWARE_H

/*
 * This is the bare minimum required for arch code only.
 * The rest comes from DeviceTree
 */

#define RCM_K1879_PHYS(bus, off)        (RCM_K1879_##bus##_PHYS_BASE + (off))
#define RCM_K1879_VIRT(bus, off)        (RCM_K1879_##bus##_VIRT_BASE + (off))

/* Areas of system memory space */
#define RCM_K1879_AREA0_PHYS_BASE      0x20000000
#define RCM_K1879_AREA0_SIZE           SZ_512K
#define RCM_K1879_AREA0_VIRT_BASE      0xf8000000

#define RCM_K1879_AREA1_PHYS_BASE      0x80000000
#define RCM_K1879_AREA1_SIZE           SZ_2M
#define RCM_K1879_AREA1_VIRT_BASE      0xf8100000

#define RCM_K1879_SCTL_OFF             0x0003c000
#define RCM_K1879_SCTL_PHYS_BASE       RCM_K1879_PHYS(AREA0, RCM_K1879_SCTL_OFF)
#define RCM_K1879_SCTL_VIRT_BASE       RCM_K1879_VIRT(AREA0, RCM_K1879_SCTL_OFF)
#define RCM_K1879_SCTL_SIZE            0x1000

#define RCM_K1879_MIF_OFF              0x00172000
#define RCM_K1879_MIF_PHYS_BASE        RCM_K1879_PHYS(AREA1, RCM_K1879_MIF_OFF)
#define RCM_K1879_MIF_SIZE             0x100

#define RCM_K1879_MIF_I2C_INT_TYPE_ENA  0x94
#define RCM_K1879_MIF_I2C_INT_TYPE      0x98
#define RCM_K1879_MIF_I2C_INT_ENA       0x9C
#define RCM_K1879_MIF_I2C_INT_STAT      0xA0
#define RCM_K1879_SCTL_INT_P_OUT        0x08

#endif
