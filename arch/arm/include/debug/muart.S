/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#define RCM_DTRANS_OFFSET 0x20
#define RCM_FIFOSTATE_OFFSET 0x14

#define RCM_FIFO_TX_SHIFT 16

.macro	addruart, rp, rv, tmp
	ldr	\rp,      =(CONFIG_DEBUG_UART_PHYS)	@ physical base
	ldr	\rv,      =(CONFIG_DEBUG_UART_VIRT) @ virt base
.endm

.macro  senduart,rd,rx
	str    \rd, [\rx, #RCM_DTRANS_OFFSET]
.endm

.macro  waituart,rd,rx
1001:	ldr	\rd, [\rx, #(RCM_FIFOSTATE_OFFSET)]	@ Read Status Register
    lsr \rd, \rd, #RCM_FIFO_TX_SHIFT
	cmp	\rd, 255
	bge	1001b
.endm

.macro  busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(RCM_FIFOSTATE_OFFSET)]	@ Read Status Register
    lsr \rd, \rd, #RCM_FIFO_TX_SHIFT
	cmp	\rd, 0
	bne	1001b
.endm
