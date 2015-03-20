#ifndef __ASM_RCM_K1879_HARDWARE_H
#define __ASM_RCM_K1879_HARDWARE_H

/* Main system freq */
#if defined(CONFIG_RCM_K1879_LO_FREQ)
#define RCM_K1879_FREQ_HZ              40000000
#define RCM_K1879_FREQ_I2C_KHZ         40000
#define RCM_K1879_FREQ_I2C0_KHZ        60000
#else
#define RCM_K1879_FREQ_HZ              54000000
#define RCM_K1879_FREQ_I2C_KHZ         54000
#define RCM_K1879_FREQ_I2C0_KHZ        81000
#endif

#define RCM_K1879_PHYS(bus,off)        (RCM_K1879_##bus##_PHYS_BASE + (off))
#define RCM_K1879_VIRT(bus,off)        (RCM_K1879_##bus##_VIRT_BASE + (off))

/* Areas of system memory space */
#define RCM_K1879_AREA0_PHYS_BASE      0x20000000
#define RCM_K1879_AREA0_VIRT_BASE      0xf8000000
#define RCM_K1879_AREA0_SIZE           SZ_1M

#define RCM_K1879_AREA1_PHYS_BASE      0x80000000
#define RCM_K1879_AREA1_SIZE           SZ_1M

#define RCM_K1879_AREA2_PHYS_BASE      0x10040000
#define RCM_K1879_AREA2_SIZE           (0x10050000-0x1004000)

/* VICs */
#define RCM_K1879_VIC0_OFF             0x00000000
#define RCM_K1879_VIC0_PHYS_BASE       RCM_K1879_PHYS(AREA0, RCM_K1879_VIC0_OFF)
#define RCM_K1879_VIC0_VIRT_BASE       RCM_K1879_VIRT(AREA0, RCM_K1879_VIC0_OFF)
                                  
#define RCM_K1879_VIC1_OFF             0x00010000
#define RCM_K1879_VIC1_PHYS_BASE       RCM_K1879_PHYS(AREA0, RCM_K1879_VIC1_OFF)
#define RCM_K1879_VIC1_VIRT_BASE       RCM_K1879_VIRT(AREA0, RCM_K1879_VIC1_OFF)

/* Serial port used for system debug console */
#define RCM_K1879_UART0_OFF            0x0002b000
#define RCM_K1879_UART0_PHYS_BASE      RCM_K1879_PHYS(AREA0, RCM_K1879_UART0_OFF)
#define RCM_K1879_UART0_VIRT_BASE      RCM_K1879_VIRT(AREA0, RCM_K1879_UART0_OFF)
#define RCM_K1879_UART0_IRQ            RCM_K1879_IRQ(7)
#define RCM_K1879_UART0_CLK            RCM_K1879_FREQ_HZ

#endif                             

