#ifndef EASYNMC_H
#define EASYNMC_H

#include <uapi/linux/easynmc.h>


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


/* This struct represents the generic NMC core */
struct nmc_core { 
	const char*        name; 
	const char*        type;
	struct device*     dev;
	int                irqs[NMC_NUM_IRQS];
	int                irq_state[NMC_NUM_IRQS];
	void             (*reset)(struct nmc_core *self);
	void             (*send_interrupt) (struct nmc_core *self, enum nmc_irq n);
	void             (*clear_interrupt) (struct nmc_core *self, enum nmc_irq n);
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
	int                token_id; /* Id for next token after reset */	
	int                cancel_id; /* Id of token being cancelled */
	struct semaphore   cancel_sem; /* semaphore */ 
	struct nmc_core_stats    stats; 
	struct nmc_core_stats    pollstats;
	struct nmc_stdio_channel  *stdout;
	struct nmc_stdio_channel  *stdin;
	int               reformat_stdout;
	int               reformat_stdin;
	int               nmi_on_close;
	/* Arbitary per-application data. Cleaned up after application is terminated. */
	void              *appdata;
	size_t            appdata_len;
	/* Internal linked list of cores registered */
	struct list_head  linkage; 
};


int easynmc_register_core(struct nmc_core *core);
int easynmc_deregister_core(struct nmc_core *core); 



#endif
