/*
 * arch/arm/mach-uemd/include/mach/debug-macro.S
 *
 * Copyright (C) 2011 RC Module
 *
 * Sergey Mironov <ierton@gmail.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include "hardware.h"

	.macro  addruart, rp, rv, tmp
        ldr     \rp, =UEMD_AREA0_PHYS_BASE
        ldr     \rv, =UEMD_AREA0_VIRT_BASE
        orr     \rp, \rp, #UEMD_UART0_OFF
        orr     \rv, \rv, #UEMD_UART0_OFF
        .endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>

