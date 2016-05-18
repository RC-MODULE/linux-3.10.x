#ifndef __UAPI_LINUX_EASYNMC_H__
#define __UAPI_LINUX_EASYNMC_H__

#ifndef __KERNEL__
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#endif

#define EASYNMC_DEVNAME_LEN 16

enum nmc_irq { 
	NMC_IRQ_NMI=0, /* Host doesn't receive this one, only nmc */
	NMC_IRQ_HP,
	NMC_IRQ_LP,
	NMC_NUM_IRQS /* Keep this one last in the list */
};

struct nmc_core_stats {
	int      started; 
	uint32_t irqs_sent[NMC_NUM_IRQS];
	uint32_t irqs_recv[NMC_NUM_IRQS];
};



#define POLLNMI  POLLHUP
#define POLLHP   POLLPRI
#define POLLLP   POLLIN

#define EPOLLNMI EPOLLHUP
#define EPOLLHP  EPOLLPRI
#define EPOLLLP  EPOLLIN



#define EASTNMC_EVT(_nirq)    (1<<_nirq)

#define EASYNMC_EVT_TIMEOUT   -1
#define EASYNMC_EVT_CANCELLED -2
#define EASYNMC_EVT_ERROR     -3
#define EASYNMC_EVT_NMI       EASTNMC_EVT(NMC_IRQ_NMI)
#define EASYNMC_EVT_HP        EASTNMC_EVT(NMC_IRQ_HP)
#define EASYNMC_EVT_LP        EASTNMC_EVT(NMC_IRQ_LP)
#define EASYNMC_EVT_ALL       (EASYNMC_EVT_LP | EASYNMC_EVT_HP | EASYNMC_EVT_NMI)

struct nmc_irq_token { 
	enum nmc_irq  event; 
	uint32_t      events_enabled;
	struct nmc_core_stats stats;
	uint32_t      timeout;
	int           id;
};

struct nmc_stdio_channel {
	uint32_t isr_on_io;
	uint32_t size;
	uint32_t head;
	uint32_t tail;
	uint32_t data[];
};

struct nmc_ioctl_buffer { 
	void  *data;
	size_t len; 
};

#define NMC3_IOCTL_MAGIC 'N'

#define IOCTL_NMC3_RESET           _IO(NMC3_IOCTL_MAGIC, 0)
#define IOCTL_NMC3_RESET_STATS     _IO(NMC3_IOCTL_MAGIC, 1)
#define IOCTL_NMC3_POLLMARK        _IO(NMC3_IOCTL_MAGIC, 2)

#define IOCTL_NMC3_SEND_IRQ        _IOW(NMC3_IOCTL_MAGIC, 3,   uint32_t             )

#define IOCTL_NMC3_GET_NAME        _IOR(NMC3_IOCTL_MAGIC, 4,   char *               )
#define IOCTL_NMC3_GET_TYPE        _IOR(NMC3_IOCTL_MAGIC, 5,   char *               )
#define IOCTL_NMC3_GET_IMEMSZ      _IOR(NMC3_IOCTL_MAGIC, 6,   size_t               )
#define IOCTL_NMC3_GET_STATS       _IOR(NMC3_IOCTL_MAGIC, 7,   struct nmc_core_stats)

#define IOCTL_NMC3_CANCEL_BY_TOKEN _IOWR(NMC3_IOCTL_MAGIC, 8,  struct nmc_irq_token)
#define IOCTL_NMC3_WAIT_ON_TOKEN   _IOWR(NMC3_IOCTL_MAGIC, 9,  struct nmc_irq_token)
#define IOCTL_NMC3_RESET_TOKEN     _IOWR(NMC3_IOCTL_MAGIC, 10, struct nmc_irq_token)

#define IOCTL_NMC3_ATTACH_STDOUT   _IOW(NMC3_IOCTL_MAGIC, 11,  uint32_t)
#define IOCTL_NMC3_ATTACH_STDIN    _IOW(NMC3_IOCTL_MAGIC, 12,  uint32_t)

#define IOCTL_NMC3_REFORMAT_STDOUT _IOW(NMC3_IOCTL_MAGIC, 13,  uint32_t)
#define IOCTL_NMC3_REFORMAT_STDIN  _IOW(NMC3_IOCTL_MAGIC, 14,  uint32_t)

#define IOCTL_NMC3_GET_APPDATA     _IOWR(NMC3_IOCTL_MAGIC, 15,  struct nmc_ioctl_buffer)
#define IOCTL_NMC3_SET_APPDATA     _IOWR(NMC3_IOCTL_MAGIC, 16,  struct nmc_ioctl_buffer)

#define IOCTL_NMC3_NUM_CORES       _IOR(NMC3_IOCTL_MAGIC,  17,  uint32_t)
#define IOCTL_NMC3_LIST_CORES      _IOWR(NMC3_IOCTL_MAGIC, 18,  struct nmc_ioctl_buffer)
#define IOCTL_NMC3_NMI_ON_CLOSE    _IOW(NMC3_IOCTL_MAGIC,  19,  int)
#define IOCTL_NMC3_ION2NMC         _IOWR(NMC3_IOCTL_MAGIC, 20,  uint32_t)

#endif
