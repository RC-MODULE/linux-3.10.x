#ifndef EASYNMC_H
#define EASYNMC_H

#include <linux/wait.h>

#define EASYNMC_DEVNAME_LEN 16

enum nmc_irq { 
	NMC_IRQ_NMI=0,
	NMC_IRQ_HP,
	NMC_IRQ_LP,
	NMC_NUM_IRQS /* Keep this one last */
};

enum host_irq { 
	HOST_IRQ_HP=0,
	HOST_IRQ_LP,
	HOST_NUM_IRQS /* Keep this one last */
};


#define  NMC_CODEVERSION      0x20140715

#define  NMC_REG_CODEVERSION  (0x100)
#define  NMC_REG_CORE_STATUS  (0x101)
#define  NMC_REG_CORE_START   (0x102)
#define  NMC_REG_PROG_ENTRY   (0x103)
#define  NMC_REG_PROG_ARGC    (0x104)
#define  NMC_REG_PROG_ARGV    (0x105)
#define  NMC_REG_PROG_RETURN  (0x106)

/* 
 * FixMe: Is this the only way?  
 * We wrap up miscdevice struct so that we'll be able to obtain the pointer to core
 * by calling container_of() from file operations
 */
struct nmc_miscdev {
	struct nmc_core *core;
	struct miscdevice mdev;
};

#define EASYNMC_MISC2CORE(m)					\
	((container_of(m, struct nmc_miscdev, mdev))->core)


struct nmc_core_stats {
	int      started; 
	uint32_t irqs_sent[NMC_NUM_IRQS];
	uint32_t irqs_recv[HOST_NUM_IRQS];
};

/* This struct represents the generic NMC core */
struct nmc_core { 
	const char*        name; 
	const char*        type;
	struct device*     dev;
	int                irqs[HOST_NUM_IRQS];
	int                irq_state[HOST_NUM_IRQS];
	void             (*reset)(struct nmc_core *self);
	void             (*interrupt) (struct nmc_core *self, enum nmc_irq n);
	char __iomem      *imem_virt;  /* Pointer to nmc internal memory (remapped) */
	unsigned long      imem_phys; 
	size_t             imem_size; /* size in bytes */ 
	/* Private data */
	int                id; 
	int                started;
	struct nmc_miscdev mdev_io;
	struct nmc_miscdev mdev_mem;
	char               devname_io[EASYNMC_DEVNAME_LEN]; 
	char               devname_mem[EASYNMC_DEVNAME_LEN];
	wait_queue_head_t  qh;
	struct nmc_core_stats stats; 
	uint32_t          *stdout;
	uint32_t          *stdin;
	
};


int easynmc_register_core(struct nmc_core *core);
int easynmc_deregister_core(struct nmc_core *core); 

#define NMC3_IOCTL_MAGIC 'N'

#define IOCTL_NMC3_RESET           _IO(NMC3_IOCTL_MAGIC, 0)
#define IOCTL_NMC3_SEND_IRQ        _IO(NMC3_IOCTL_MAGIC, 1)
#define IOCTL_NMC3_GET_NAME        _IO(NMC3_IOCTL_MAGIC, 7)
#define IOCTL_NMC3_GET_TYPE        _IO(NMC3_IOCTL_MAGIC, 8)
#define IOCTL_NMC3_GET_STATS       _IO(NMC3_IOCTL_MAGIC, 5)
#define IOCTL_NMC3_RESET_STATS     _IO(NMC3_IOCTL_MAGIC, 6)
#define IOCTL_NMC3_WAIT_IRQ        _IO(NMC3_IOCTL_MAGIC, 2)


#define IOCTL_NMC3_SET_IRQ_MASK    _IO(NMC3_IOCTL_MAGIC, 3)
#define IOCTL_NMC3_ATTACH_STDIO    _IO(NMC3_IOCTL_MAGIC, 4)


#endif
