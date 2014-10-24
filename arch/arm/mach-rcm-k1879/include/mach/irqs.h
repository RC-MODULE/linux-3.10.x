#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

//#define IRQ_LOCALTIMER      29
//#define IRQ_LOCALWDOG       30

//#define RCM_K1879_HW_IRQS_BASE   0
//#define RCM_K1879_SW_IRQS_BASE   64

#define NR_IRQS             (64+64)

#define RCM_K1879_IRQ(x)          (32 + (x))
#define RCM_K1879_VIC0_IRQ(x)     (32 + (x))
#define RCM_K1879_VIC1_IRQ(x)     (64 + (x))
#endif

