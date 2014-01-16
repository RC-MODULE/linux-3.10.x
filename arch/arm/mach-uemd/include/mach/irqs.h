#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#define IRQ_LOCALTIMER      29
#define IRQ_LOCALWDOG       30

#define UEMD_HW_IRQS_BASE   0
#define UEMD_SW_IRQS_BASE   64
#define NR_IRQS             (64+64)

#define UEMD_IRQ(x)         ((x))

#endif

