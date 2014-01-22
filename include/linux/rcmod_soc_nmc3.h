/*
 * Copyright (C) RC Module 2011
 *	Gennadiy Kurtsman <gkurtsman@module.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __RCMOD_SOC_NMC3_H
#define __RCMOD_SOC_NMC3_H

#ifdef DEBUG
#if 0
#define dbg(format, arg...) printk(KERN_DEBUG "%s:%d: " format "\n", __FILE__, __LINE__, ##arg)

#define DUMP_NMC3(bank,addr,ln) \
dbg ("dump_nmc3, \naddr=0x%x/0x%x,ln=0x%x/0x%x",(addr),(addr)>>2,(ln)<<2,(ln)); \
do { \
int i; \
for (i=0; i<(ln); i++) { \
if (!(i%4)) \
printk (KERN_DEBUG "\n0x%08x:",i); \
printk (KERN_DEBUG "0x%08x ", ioread32(((__u32*)(dscr->nmc3_bank_base_virt[bank] + addr) + i))); \
} \
printk (KERN_DEBUG "\n-------------\n"); \
} while (0)
#else
#define dbg(format, arg...) printk(KERN_ERR "%s:%d: " format "\n", __FILE__, __LINE__, ##arg)

#define DUMP_NMC3(bank,addr,ln) \
dbg ("dump_nmc3, \naddr=0x%x/0x%x,ln=0x%x/0x%x",(addr),(addr)>>2,(ln)<<2,(ln)); \
do { \
int i; \
for (i=0; i<(ln); i++) { \
if (!(i%4)) \
printk (KERN_ERR "\n0x%08x:",i); \
printk (KERN_ERR "0x%08x ", ioread32(((__u32*)(dscr->nmc3_bank_base_virt[bank] + addr) + i))); \
} \
printk (KERN_ERR "\n-------------\n"); \
} while (0)
#endif
#define PRINT_IF \
do { \
dbg ("TO_ARM = 0x%x",ioread32(&(dscr->NMC3_if_memory->TO_ARM))); \
dbg ("FROM_ARM = 0x%x",ioread32(&(dscr->NMC3_if_memory->FROM_ARM))); \
dbg ("ISR_FROM_ARM = 0x%x",ioread32(&(dscr->NMC3_if_memory->ISR_FROM_ARM))); \
dbg ("SYNC_TO_ARM:"); \
DUMP_NMC3(0,0x408,4);  /* SYNC_TO_ARM */ \
dbg ("SYNC_FROM_ARM"); \
DUMP_NMC3(0,0x418,4);   /* SYNC_FROM_ARM */ \
dbg ("stop's status"); \
DUMP_NMC3(0,0xa00,12);   /* stop() writes here */ \
dbg ("debug info"); \
DUMP_NMC3(0,0x440,18); /* debug array */ \
} while (0)
#else
#define dbg(format, arg...)
#define DUMP_NMC3(bank,addr,ln)
#define PRINT_IF
#endif

#define NMC3_PROCNAME "NMC3"

/* TODO: defines below are platform dependent */

#define		CTRL_NM_RESET_REG_VAL		0x000000ff
#define		CTRL_NM_NMI_REG_VAL		0x00000001
#define		CTRL_NM_INT_SET_VAL		0x1f
#define		CTRL_NM_INT_CLR_VAL		0xffffffff
#define		VIC_LPINTRS_ENA_VAL		0x01

#define		IRQ_CONTEXT_LIFE_TIME		1000	/* Target can be in irq context
							   no more than this sum of msecs */
#define		NMC3_IF_MEM_OFF			(0x100 * sizeof(__u32))

#define		TO_NMC3_HP_IRQ			16
#define		TO_NMC3_LP_IRQ			25

#define		ELF_PT_LOAD			1

/*********** NMC3 internal definitions **************/
#define ANY_COMMAND		7	/* Mask and empty command place */
#define NO_COMMAND		0	/* Do nothing */
#define RUN_PROGRAM		1	/* Run NMC3 application */
#define TEST_NMI		2	/* for testing aims */
#define IRQ_FROM_ARM		3	/* to synchronize interrupts */
#define STOP_UNLOCK_CMD		4	/* stop cycle for unlocking memory */
#define STOP_APP		5

#define ANY_MESSAGE		7	/* Mask and empty message place */
#define PROGRAM_FINISH		1	/* NMC3 application has been finished.
					   Interrupt to ARM has been requested */
#define IRQ_FROM_NMC3		2	/* NMC3 application has set interrupt request to ARM */

/* Struct of SYNC_{TO_FROM}_ARM memory block. Is taken from NMC3 startup sources */
struct ToArmBlock {
	__u32 irq_param; /* parameter of IRQ request. */
	__u32 ret_low; /* returned value, int or low part of long. */
	__u32 res [2]; /* reserved for future times. */
} __attribute__((packed));

struct FromArmBlock {
	__u32 entry; /* Entry point for program start. */
	__u32 param; /* param for NM irq request and application */
	__u32 res [2]; /* reserved for future times. */
};

struct __NMC3_if_memory {
	__u32 TO_ARM;
	__u32 FROM_ARM;
	__u32 SYNC_TO_ARM [4];
	__u32 SYNC_FROM_ARM [4];
	__u32 MAGIC_CODE [5];
	__u32 NMC3_NO;
	__u32 ISR_FROM_ARM;
	__u32 USER_ISR_PTR;
	__u32 DEBUG_ARRAY [16];
};

/*****************************************************/

enum __nmc3_states {
reset = 0,
running,
};

struct __NMC3_thread_descriptor {
	struct __NMC3_descriptor* dscr;
	int num;
	int busy;
	void (*app_cb) (struct __NMC3_thread_descriptor* thr, int ret_code);
	void *data;
};

struct __NMC3_descriptor {
	struct list_head list;
	int num;

	spinlock_t load_lock;

	char Name[ARRAY_SIZE(NMC3_PROCNAME) + 2]; /* NMC3_PROCNAME_x */
	void __iomem *vic_LPintrs_ena;
	void __iomem *nmnmi;
	void __iomem *nmreset;
	void __iomem *nmHPintr_clr;
	void __iomem *nmLPintr_clr;
	void __iomem *nmHPintr_set;
	void __iomem *nmLPintr_set;

	void __iomem *nmc3_bank_base_virt [16];	/* 16 - redundant maximum */
	__u32 nmc3_bank_base_phys [16];
	unsigned long nmc3_bank_ln [16];
	__u32 nmc3_bank_base_phys_internal [16];
	int nmc3_bank_count;

	struct __NMC3_thread_descriptor* NMC3_thread_dscr;
	struct __NMC3_if_memory __iomem *NMC3_if_memory; 

	unsigned int HPirq;
	unsigned int LPirq;
	void (*irq_cb) (int irq, __u32 msg);

	enum __nmc3_states status;

	int app_thr_active;
};

/* exported API */
#ifdef DEBUG
void print_if(struct __NMC3_thread_descriptor* thr_dscr);
#else
static inline void print_if(struct __NMC3_thread_descriptor* thr_dscr) {};
#endif
struct __NMC3_thread_descriptor* nmc3_find_device (char* device_name);
int nmc3_release_dscr (struct __NMC3_thread_descriptor* thr_dscr);
int nmc3_start (struct __NMC3_thread_descriptor* thr_dscr, void* init_code,
	size_t init_code_ln);
int nmc3_elf_upload (struct __NMC3_thread_descriptor* thr_dscr, void* app_ptr,
	size_t app_ln, __u32* app_entry);
int nmc3_app_run (struct __NMC3_thread_descriptor* thr_dscr, unsigned long app_entry,
	unsigned long param,
	void (*app_cb) (struct __NMC3_thread_descriptor* thr, int ret_code));
int nmc3_app_stop_nmi (struct __NMC3_thread_descriptor* thr_dscr);
int nmc3_app_stop_irq (struct __NMC3_thread_descriptor* thr_dscr);
int nmc3_request_interrupt (struct __NMC3_thread_descriptor* thr_dscr, __u32 param);
int nmc3_mem_write (struct __NMC3_thread_descriptor* thr_dscr, 
	unsigned long dst, void* src, size_t ln);
int nmc3_mem_read (struct __NMC3_thread_descriptor* thr_dscr,
	void* dst, unsigned long src, size_t ln);
int nmc3_set_handler (struct __NMC3_thread_descriptor* thr_dscr,
	void (*irq_cb) (int irq, __u32 msg));

#endif /* __RCMOD_SOC_NMC3_H */

