/*
 * Copyright (C) RC Module 2011
 *	Gennadiy Kurtsman <gkurtsman@module.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NMC3_H
#define __NMC3_H

#define		SCTL_REGS_BASE			UEMD_AREA0_PHYS_BASE + 0x3c000
#define		SCTL_REGS_ARRAY_LN		0x80

#define		CTRL_NM_RESET_REG		0x10
#define		CTRL_NM_NMI_REG			0x04
#define 	CTRL_NM_INTL_CLR_REG		0x30
#define 	CTRL_NM_INTH_CLR_REG		0x34
#define		CTRL_NM_INTH_SET_REG		0x38
#define		CTRL_NM_INTL_SET_REG		0x3c

#define		VIC_LPINTRS_ENA_OFF		0x0F00  /* writing into this register unlocks
							   interrupts of lower priority */
#define		NMC3_BANK_0_IM1			0x00140000
#define		NMC3_BANK_1_IM1			0x00160000
#define		NMC3_BANK_2_IM3			0x00180000
#define		NMC3_BANK_3_IM3			0x001A0000
#define		NMC3_BANK_LN			0x00020000

#define		NMC3_BANK_IM0			0x00100000
#define		NMC3_BANK_IM0_LN		0x00040000

#define		NMC3_BANK_0_INTERNAL		0x00000000
#define		NMC3_BANK_0_INTERNAL_END	0x00007fff
#define		NMC3_BANK_1_INTERNAL		0x00008000
#define		NMC3_BANK_1_INTERNAL_END	0x0000ffff
#define		NMC3_BANK_2_INTERNAL		0x00010000
#define		NMC3_BANK_2_INTERNAL_END	0x00017fff
#define		NMC3_BANK_3_INTERNAL		0x00018000
#define		NMC3_BANK_3_INTERNAL_END	0x0001ffff

#define		NMC3_BANK_IM0_INTERNAL		0x00040000
#define		NMC3_BANK_IM0_INTERNAL_END	0x0004ffff

#define		NMC3_HP_IRQ			UEMD_IRQ(14)
#define		NMC3_LP_IRQ			UEMD_IRQ(15)

#endif /* __NMC3_H */
