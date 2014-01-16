#ifndef __ASM_UEMD_HARDWARE_H
#define __ASM_UEMD_HARDWARE_H

/* Main system freq */
#if defined(CONFIG_UEMD_LO_FREQ)
#define UEMD_FREQ_HZ              40000000
#define UEMD_FREQ_I2C_KHZ         40000
#define UEMD_FREQ_I2C0_KHZ        60000
#else
#define UEMD_FREQ_HZ              54000000
#define UEMD_FREQ_I2C_KHZ         54000
#define UEMD_FREQ_I2C0_KHZ        81000
#endif

#define UEMD_PHYS(bus,off)        (UEMD_##bus##_PHYS_BASE + (off))
#define UEMD_VIRT(bus,off)        (UEMD_##bus##_VIRT_BASE + (off))

/* Areas of system memory space */
#define UEMD_AREA0_PHYS_BASE      0x20000000
#define UEMD_AREA0_VIRT_BASE      0xf8000000
#define UEMD_AREA0_SIZE           SZ_1M

#define UEMD_AREA1_PHYS_BASE      0x80000000
#define UEMD_AREA1_SIZE           SZ_1M

#define UEMD_AREA2_PHYS_BASE      0x10040000
#define UEMD_AREA2_SIZE           (0x10050000-0x1004000)

/* VICs */
#define UEMD_VIC0_OFF             0x00000000
#define UEMD_VIC0_PHYS_BASE       UEMD_PHYS(AREA0, UEMD_VIC0_OFF)
#define UEMD_VIC0_VIRT_BASE       UEMD_VIRT(AREA0, UEMD_VIC0_OFF)
                                  
#define UEMD_VIC1_OFF             0x00010000
#define UEMD_VIC1_PHYS_BASE       UEMD_PHYS(AREA0, UEMD_VIC1_OFF)
#define UEMD_VIC1_VIRT_BASE       UEMD_VIRT(AREA0, UEMD_VIC1_OFF)

/* Serial port used for system debug console */
#define UEMD_UART0_OFF            0x0002b000
#define UEMD_UART0_PHYS_BASE      UEMD_PHYS(AREA0, UEMD_UART0_OFF)
#define UEMD_UART0_VIRT_BASE      UEMD_VIRT(AREA0, UEMD_UART0_OFF)
#define UEMD_UART0_IRQ            UEMD_IRQ(7)
#define UEMD_UART0_CLK            UEMD_FREQ_HZ

#endif                             

