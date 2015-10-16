#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

extern phys_addr_t uemd_get_fb_base(void);
extern int uemd_setup_vmode(unsigned int hz, int hd);
extern int uemd_is_virgin(void);

#endif

