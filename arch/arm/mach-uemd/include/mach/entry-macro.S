/*
 * arch-arm/mach-uemd/include/mach/entry-macro.S
 *
 * Copyright (C) 2011 RC Module
 *
 * Sergey Mironov <ierton@gmail.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#include <mach/hardware.h>
#include <asm/hardware/vic.h>

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	/*
	 * NOTE: This code tests VIC0 before VIC1. Therefore, VIC0 interrupts
	 * may have a precendece.
	 */
	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr	\base, =UEMD_VIC0_VIRT_BASE
	ldr	\irqstat, [\base, #VIC_IRQ_STATUS] @ get masked status
	mov	\irqnr, #0
	teq	\irqstat, #0
	bne	1002f
1001:	ldr	\base, =UEMD_VIC1_VIRT_BASE
	ldr	\irqstat, [\base, #VIC_IRQ_STATUS] @ get masked status
	mov	\irqnr, #32
	teq	\irqstat, #0
	beq	1003f
1002:	tst	\irqstat, #1
	bne	1003f
	add	\irqnr, \irqnr, #1
	movs	\irqstat, \irqstat, lsr #1
	bne	1002b
1003:		/* EQ will be set if no irqs pending */
	.endm

