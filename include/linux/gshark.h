/* GSHARK turbo register definitions */
#ifndef __LINUX_GSHARK_H__
#define __LINUX_GSHARK_H__

#include <linux/ioctl.h>

#ifdef __KERNEL__
#include <linux/semaphore.h>

struct gshark_device_data {
	unsigned long* offset;
	unsigned long phys;

	struct semaphore sem;
	unsigned current_lock_pid;
	unsigned last_set_pid;
};
#endif

#define GSHARK_REGISTERS_SIZE 0x00000400

#define GT_MEMREG_BIT	0x00000000
typedef enum {
     GT_CFIFO		= (GT_MEMREG_BIT | 0x000),	/* Cmd FIFO		*/
     GT_CFIFO_STAT	= (GT_MEMREG_BIT | 0x000),	/* Cmd FIFO Status	*/
     GT_CFIFO_CTRL	= (GT_MEMREG_BIT | 0x004),	/* Cmd FIFO Control	*/
     GT_RESET		= (GT_MEMREG_BIT | 0x008),	/* Reset		*/
     GT_IP_REV		= (GT_MEMREG_BIT | 0x008),	/* IP Revision		*/
     GT_INTR_MASK	= (GT_MEMREG_BIT | 0x00c),	/* Interrupt Mask	*/
     GT_INTR_CLEAR	= (GT_MEMREG_BIT | 0x010),	/* Interrupt Clear	*/
     GT_INTR_STAT	= (GT_MEMREG_BIT | 0x014),	/* Interrupt Status	*/
     GT_MISC_CTRL	= (GT_MEMREG_BIT | 0x018),	/* Misc Control		*/
     GT_READBACK	= (GT_MEMREG_BIT | 0x01c),	/* Readback		*/
     GT_BUSY_STAT	= (GT_MEMREG_BIT | 0x020),	/* Busy Status		*/
     GT_TE_STAT		= (GT_MEMREG_BIT | 0x024),	/* TE Status		*/
     GT_RZ_STAT		= (GT_MEMREG_BIT | 0x02c),	/* RZ Status		*/
     GT_PE_STAT		= (GT_MEMREG_BIT | 0x030),	/* PE Status		*/
     GT_MISC_STAT	= (GT_MEMREG_BIT | 0x034),	/* Misc Status		*/
} MemReg;

/*
	GSHARK_HRST
		hard reset.
	GSHARK_SRST
		soft reset.
	GSHARK_LOCK
		aquire access permission.
	GSHARK_UNLOCK
		abandon access permission.
	GSHARK_IS_CURRENT
		check PID.
	GSHARK_SET_CURRENT
		set PID.
*/

#define GSHARK_IOC_MAGIC	'g'
#define	GSHARK_HRST		_IO(GSHARK_IOC_MAGIC, 0)
#define	GSHARK_SRST		_IO(GSHARK_IOC_MAGIC, 1)

#define	GSHARK_LOCK		_IO(GSHARK_IOC_MAGIC, 70)
#define	GSHARK_UNLOCK		_IO(GSHARK_IOC_MAGIC, 71)
#define	GSHARK_IS_CURRENT	_IO(GSHARK_IOC_MAGIC, 72)
#define	GSHARK_SET_CURRENT	_IO(GSHARK_IOC_MAGIC, 73)

#endif
