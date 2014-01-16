#ifndef __ASM_UEMD_MEMORY_H
#define __ASM_UEMD_MEMORY_H

#ifdef  CONFIG_UEMD_LO_MEM
#define _PHYS_EM0_SIZE        SZ_16M
#else
#define _PHYS_EM0_SIZE        SZ_128M
#endif

#define PHYS_IM0              UL(0x00100000)
#define PHYS_IM0_SIZE         UL(0x00040000)
#define PHYS_IM1              UL(0x00140000)
#define PHYS_IM1_SIZE         UL(0x00040000)

#define PHYS_EM0              UL(0x40000000)
#define PHYS_EM0_SIZE         _PHYS_EM0_SIZE   
#define PHYS_EM1              UL(0xC0000000)
#define PHYS_EM1_SIZE         UL(0x08000000)

/* Physical DRAM offset - Bank EM0 */
#define PHYS_OFFSET           PHYS_EM0
#define PHYS_SIZE             PHYS_EM0_SIZE

#endif

