/*
 * Copyright (C) RC Module 2011
 *      Gennadiy Kurtsman <gkurtsman@module.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RCMOD_SOC_NMC3_TEST_H
#define __RCMOD_SOC_NMC3_TEST_H

#define NMC3_MAJOR 8
#define TEST_NMC3_MINORS 8

#define NMC3_IOCTL_MAGIC 'N'

#define IOCTL_NMC3_START _IOW(NMC3_IOCTL_MAGIC, 1,unsigned long)
#define IOCTL_NMC3_APP_RUN _IOW(NMC3_IOCTL_MAGIC, 2,unsigned long)
#define IOCTL_NMC3_IRQREQUEST _IOW(NMC3_IOCTL_MAGIC, 3,unsigned long)
#define IOCTL_NMC3_APP_WAIT _IOR(NMC3_IOCTL_MAGIC, 4, unsigned long)
#define IOCTL_NMC3_APP_STOP_NMI _IO(NMC3_IOCTL_MAGIC, 5)
#define IOCTL_NMC3_APP_STOP_IRQ _IO(NMC3_IOCTL_MAGIC, 6)
#define IOCTL_ARM_IRQWAIT _IOR(NMC3_IOCTL_MAGIC, 7, unsigned long)

#define NMC3_count 1
#define NM_APP_START_CODE	0x1BADF00D
#define NM_APP_STOP_CODE	0xDEADBEEF

#define IRQ_TEST_COUNT		10
#define NM_LP_IRQ		25
#define NM_HP_IRQ		16

/* types of interrupt waited from NM */

#define NM_STEST_TYPE		0	/* for stretch test */
#define NM_STOP_TYPE		1	/* for stop */
#define NM_2PTEST_TYPE		2	/* for child process */

struct __app_run_param {
	void* app_ptr;
	size_t app_ln;
	unsigned long param;
	int wait_nmapp_app;
};

struct __app_irq_param {
	int irq;
	__u32 param;
	__u32 irq_cb_type;
};

struct nm_irq_data {
	__u32 msg;
	__u32 array_ptr;
	__u32 array_ln;
};

#ifndef __KERNEL__

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

#ifdef DEBUG
#define dbg(format, arg...) printf("%s:%d: " format "\n", __FILE__, __LINE__, ##arg)
#else
#define dbg(format, arg...)
#endif

static inline void fdisplay_nmc3(int fl, __u32 addr, int ln)
{
	int i;
	__u32 rd_array [256];
	int actual_ln = ((unsigned int)ln < ARRAY_SIZE(rd_array)) ? ln : ARRAY_SIZE(rd_array);

	printf ("display_nmc3, \n"
		"addr=0x%x/0x%x,ln=0x%x/0x%x\n",
		(addr),(addr)>>2,(ln),(actual_ln >> 2));
	lseek (fl, 0x100, SEEK_SET);
	read (fl, &rd_array[0], sizeof (rd_array[0]));
	printf ("TO_ARM = 0x%x\n", rd_array[0]);
	lseek (fl, 0x101, SEEK_SET);
	read (fl, &rd_array[0], sizeof (rd_array[0]));
	printf ("FROM_ARM = 0x%x\n",rd_array[0]);
	lseek (fl, 0x110, SEEK_SET);
	read (fl, &rd_array[0], sizeof (rd_array[0]));
	printf ("ISR_FROM_ARM = 0x%x\n",rd_array[0]);
	lseek (fl, 0x102, SEEK_SET);
	read (fl, &rd_array[0], 16);
	printf ("SYNC_TO_ARM = 0x%x,0x%x,0x%x,0x%x\n", rd_array[0],
		rd_array[1], rd_array[2], rd_array[3]);
	lseek (fl, 0x106, SEEK_SET);
	read (fl, &rd_array[0], 16);
	printf ("SYNC_FROM_ARM = 0x%x,0x%x,0x%x,0x%x\n", rd_array[0],
		rd_array[1], rd_array[2], rd_array[3]);
	lseek (fl, addr, SEEK_SET);
	read (fl, rd_array, actual_ln);
	for (i=0; i<(actual_ln>>2); i++) {
		if (!(i%4))
			printf ("\n0x%08x:",i);
		printf ("0x%08x ", rd_array[i]);
	}
	printf ("\n-------------\n");
}

#else

#define NMC3_FILENAME "NMC3"

struct NMC3_extension
{
	char Name[ARRAY_SIZE(NMC3_FILENAME) + 2]; // <NMC3_FILENAME>_X

	struct cdev cdev;
	struct semaphore sem;
	struct semaphore app_complete_sem;
	int ret_code;

	void* nmc3_dscr [TEST_NMC3_MINORS];
};

static inline void kdisplay_nmc3(struct __NMC3_thread_descriptor* dscr,
		__u32 addr, int ln)
{
	int i;
	__u32 rd_array [256];
	int actual_ln = ln < ARRAY_SIZE (rd_array) ? ln : ARRAY_SIZE (rd_array);

	printk ("display_nmc3, \n"
		"addr=0x%x/0x%x,ln=0x%x/0x%x\n",
		(addr),(addr)>>2,(actual_ln),(actual_ln >> 2));
	nmc3_mem_read (dscr, &rd_array[0], 0x100, sizeof (rd_array[0]));
	printk ("TO_ARM = 0x%x\n", rd_array[0]);
	nmc3_mem_read (dscr, &rd_array[0], 0x101, sizeof (rd_array[0]));
	printk ("FROM_ARM = 0x%x\n",rd_array[0]);
	nmc3_mem_read (dscr, &rd_array[0], 0x110, sizeof (rd_array[0]));
	printk ("ISR_FROM_ARM = 0x%x\n",rd_array[0]);
 	nmc3_mem_read (dscr, rd_array, 0x102, 16);
	printk ("SYNC_TO_ARM = 0x%x,0x%x,0x%x,0x%x\n", rd_array[0],
		rd_array[1], rd_array[2], rd_array[3]);
 	nmc3_mem_read (dscr, rd_array, 0x106, 16);
	printk ("SYNC_FROM_ARM = 0x%x,0x%x,0x%x,0x%x\n", rd_array[0],
		rd_array[1], rd_array[2], rd_array[3]);
	nmc3_mem_read (dscr, rd_array, addr, actual_ln);
	for (i=0; i<(actual_ln >> 2); i++) {
		if (!(i%4))
			printk ("\n0x%08x:",i);
		printk ("0x%08x ", rd_array[i]);
	}
	printk ("\n-------------\n");
}

#endif /* __KERNEL__ */

#endif	/* __RCMOD_SOC_NMC3_TEST_H */
