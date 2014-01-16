/*
 *  arch/arm/mach-uemd/platform.h
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
#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

#include <asm/sizes.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/hardware.h>

/* Decalre Area 0 memory region. This is statically-mapped area. 
 * UART0 and VICs are defined in <mach/hardware.h> */
#define UEMD_GRI2C1_OFF           0x00021000
#define UEMD_GRI2C1_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_GRI2C1_OFF)
#define UEMD_GRI2C1_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_GRI2C1_OFF)
#define UEMD_GRI2C1_IRQ           UEMD_IRQ(28)

#define UEMD_UARTIRDA_OFF         0x00022000
#define UEMD_UARTIRDA_PHYS_BASE   UEMD_PHYS(AREA0, UEMD_UARTIRDA_OFF)
#define UEMD_UARTIRDA_VIRT_BASE   UEMD_VIRT(AREA0, UEMD_UARTIRDA_OFF)
#define UEMD_UARTIRDA_IRQ         UEMD_IRQ(9)
#define UEMD_UARTIRDA_CLK         UEMD_FREQ_HZ

#define UEMD_TIMER0_OFF           0x00024000 
#define UEMD_TIMER0_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_TIMER0_OFF)
#define UEMD_TIMER0_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_TIMER0_OFF)
#define UEMD_TIMER0_IRQ           UEMD_IRQ(4)

#define UEMD_GRI2C2_OFF           0x00026000
#define UEMD_GRI2C2_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_GRI2C2_OFF)
#define UEMD_GRI2C2_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_GRI2C2_OFF)
#define UEMD_GRI2C2_IRQ           UEMD_IRQ(29)

#define UEMD_UART1_OFF            0x0002c000
#define UEMD_UART1_PHYS_BASE      UEMD_PHYS(AREA0, UEMD_UART1_OFF)
#define UEMD_UART1_VIRT_BASE      UEMD_VIRT(AREA0, UEMD_UART1_OFF)
#define UEMD_UART1_IRQ            UEMD_IRQ(8)
#define UEMD_UART1_CLK            UEMD_FREQ_HZ

#define UEMD_TSP_OFF              0x00030000
#define UEMD_TSP_PHYS_BASE        UEMD_PHYS(AREA0, UEMD_TSP_OFF)
#define UEMD_TSP_VIRT_BASE        UEMD_VIRT(AREA0, UEMD_TSP_OFF)
#define UEMD_TSP_IRQ              UEMD_IRQ(38)

#define UEMD_DMC1_OFF		  0x00031000
#define UEMD_DMC1_PHYS_BASE       UEMD_PHYS(AREA0, UEMD_DMC1_OFF)
#define UEMD_DMC1_VIRT_BASE       UEMD_VIRT(AREA0, UEMD_DMC1_OFF)

#define UEMD_GRI2C3_OFF           0x0002d000
#define UEMD_GRI2C3_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_GRI2C3_OFF)
#define UEMD_GRI2C3_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_GRI2C3_OFF)
#define UEMD_GRI2C3_IRQ           UEMD_IRQ(30)

#define UEMD_GRETH_OFF            0x00034000
#define UEMD_GRETH_PHYS_BASE      UEMD_PHYS(AREA0, UEMD_GRETH_OFF)
#define UEMD_GRETH_VIRT_BASE      UEMD_VIRT(AREA0, UEMD_GRETH_OFF)
#define UEMD_GRETH_IRQ            UEMD_IRQ(34)

#define UEMD_SCTL_OFF             0x0003c000
#define UEMD_SCTL_PHYS_BASE       UEMD_PHYS(AREA0, UEMD_SCTL_OFF)
#define UEMD_SCTL_VIRT_BASE       UEMD_VIRT(AREA0, UEMD_SCTL_OFF)
#define UEMD_SCTL_SIZE            0x1000

#define UEMD_CRYPTO_OFF           0x0003d000
#define UEMD_CRYPTO_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_CRYPTO_OFF)
#define UEMD_CRYPTO_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_CRYPTO_OFF)
#define UEMD_CRYPTO_IRQ           UEMD_IRQ(40)

#define UEMD_NAND_OFF             0x0003f000
#define UEMD_NAND_PHYS_BASE       UEMD_PHYS(AREA0, UEMD_NAND_OFF)
#define UEMD_NAND_VIRT_BASE       UEMD_VIRT(AREA0, UEMD_NAND_OFF)
#define UEMD_NAND_SIZE            0x1000
#define UEMD_NAND_IRQ             UEMD_IRQ(36) 

#define UEMD_CRYPTOAES_OFF           0x00040000
#define UEMD_CRYPTOAES_PHYS_BASE     UEMD_PHYS(AREA0, UEMD_CRYPTOAES_OFF)
#define UEMD_CRYPTOAES_VIRT_BASE     UEMD_VIRT(AREA0, UEMD_CRYPTOAES_OFF)
#define UEMD_CRYPTOAES_IRQ           UEMD_IRQ(39)

#define UEMD_GPIOA_OFF            0x0002a000
#define UEMD_GPIOA_PHYS_BASE      UEMD_PHYS(AREA0, UEMD_GPIOA_OFF)
#define UEMD_GPIOA_VIRT_BASE      UEMD_VIRT(AREA0, UEMD_GPIOA_OFF)
/* Inly combined irq is used by the driver */
#define UEMD_GPIOA_IRQ            UEMD_IRQ(63)
#define UEMD_GPIOA_SWIRQ_BASE     UEMD_IRQ(UEMD_SW_IRQS_BASE + 0)

#define UEMD_SCI_OFF              0x00027000
#define UEMD_SCI_PHYS_BASE        UEMD_PHYS(AREA0, UEMD_SCI_OFF)
#define UEMD_SCI_VIRT_BASE        UEMD_VIRT(AREA0, UEMD_SCI_OFF)
#define UEMD_SCI_IRQ              UEMD_IRQ(11)

/* Decalre Area 1 memory region. Those should be mapped dynamically. */
/* This I2C controller manages internal I2C bus used by HDMI */
#define UEMD_GRI2C0_OFF           0x00171000
#define UEMD_GRI2C0_PHYS_BASE     UEMD_PHYS(AREA1, UEMD_GRI2C0_OFF)
#define UEMD_GRI2C0_IRQ           UEMD_IRQ(33)

#define UEMD_MIF_OFF              0x00172000
#define UEMD_MIF_PHYS_BASE        UEMD_PHYS(AREA1, UEMD_MIF_OFF)
#define UEMD_MIF_SIZE             0x100

#define UEMD_MVDU_OFF             0x00173000
#define UEMD_MVDU_PHYS_BASE       UEMD_PHYS(AREA1, UEMD_MVDU_OFF)
#define UEMD_MVDU_IRQ             UEMD_IRQ(42)
#define UEMD_MVDU_SIZE            0x1000

#define UEMD_MAUDIO_I2S_OFF             0x00150000
#define UEMD_MAUDIO_I2S_PHYS_BASE       UEMD_PHYS(AREA1, UEMD_MAUDIO_I2S_OFF)

#define UEMD_MAUDIO_SPDIF_OFF           0x00160000
#define UEMD_MAUDIO_SPDIF_PHYS_BASE     UEMD_PHYS(AREA1, UEMD_MAUDIO_SPDIF_OFF)

#define UEMD_MAUDIO_DMA_OFF             0x00172800
#define UEMD_MAUDIO_DMA_PHYS_BASE       UEMD_PHYS(AREA1, UEMD_MAUDIO_DMA_OFF)
#define UEMD_MAUDIO_DMA_IRQ             UEMD_IRQ(53)

#define UEMD_MSVDHD_OFF           0x00180000
#define UEMD_MSVDHD_PHYS_BASE     UEMD_PHYS(AREA1, UEMD_MSVDHD_OFF)
#define UEMD_MSVDHD_IRQ           UEMD_IRQ(44)
#define UEMD_MSVDHD_SIZE          0x20000

/* Declare Area 2 memory region */
#define UEMD_USB_PHYS_BASE        UEMD_PHYS(AREA2, 0)
#define UEMD_USB_IRQ              UEMD_IRQ(35)

#endif

