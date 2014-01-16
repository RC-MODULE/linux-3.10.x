#ifndef LINUX_MODULE_VDU_GRABBER_H
#define LINUX_MODULE_VDU_GRABBER_H

#include <linux/ioctl.h>

struct mvdu_grabber_ioctl_data {

	/* Request parameters */
	int mode;
	int sync;
	int skip_count;
	int skip_count_2;		/* for second field */
	int grab_count;

	/* Request results */
	unsigned int *data1;		/* used for both modes */
	unsigned int *data2;		/* used for interleaved mode */
};

#define MVDU_GRABBER_MODE_PROGRESSIVE	0
#define MVDU_GRABBER_MODE_INTERLEAVED	1

#define MVDU_GRABBER_START_ON_RAISE	0
#define MVDU_GRABBER_START_ON_FALL	1

#define MVDU_GRABBER_MAX_GRAB_COUNT	65530

#define MVDU_GRAB		_IOWR('g', 0x0, struct mvdu_grabber_ioctl_data)

#ifdef __KERNEL__

struct mvdu_grabber_registers {
	u32 skip_count_2;	/* 0x820FFFD0 */
	u32 pad0;
	u32 reset;		/* 0x820FFFD8 */
	u32 pad1;
	u32 flags;		/* 0x820FFFE0 */
	u32 pad2;
	u32 ack_irq;		/* 0x820FFFE8 */
	u32 pad3;
	u32 skip_count;		/* 0x820FFFF0 */
	u32 pad4;
	u32 grab_count;		/* 0x820FFFF8 */
};
#define MVDU_GRABBER_REGISTERS_SIZE	sizeof(struct mvdu_grabber_registers)

#define MVDU_GRABBER_MEMORY_SIZE	(MVDU_GRABBER_MAX_GRAB_COUNT * 8)

#endif

#endif
