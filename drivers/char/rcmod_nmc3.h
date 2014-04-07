#ifndef __BBP_NMC3__H__
#define __BBP_NMC3__H__

#define DEBUG

#define NMC3_MAJOR 8

// Enable undocumented facilities
//#define UNDOC

#ifdef DEBUG
//#define dbg(format, arg...) printk("%s:%s:%d: " format "\n", __FILE__, __func__, __LINE__, ##arg)
#define dbg(format, arg...) printk(KERN_DEBUG "%s:%s:%d: " format "\n", __FILE__, __func__, __LINE__, ##arg)
//#define dbg(format, arg...) printk(KERN_ERR "%s:%s:%d: " format "\n", __FILE__, __func__, __LINE__, ##arg)
#else
#define dbg(format, arg...)
#endif

#define NMC3_FILENAME "NMC3"
#define NMC3_COUNT 1

const u32 FROM_NM_offset = 0x100; // In 32-b words
const u32 HP_INT_SEND_offset = 0x10C; // In 32-b words

const u32 READY_FOR_COMMAND = 0x1;
const u32 PROGRAM_STOPPED   = 0x2;
const u32 SYNC_FLAG         = 0x4;

const u32 INT_NM_MEM_SIZE = 0x20000; // In 32-b words

const u32 NMUREGS_u = 0x2003C000;
const u32 NMUREGS_size = 0x40;

const u32 NMUINTREQ_offset = 0x4;
const u32 NMURESET_offset = 0x10;
const u32 NMUINTCLRH_offset = 0x30;
const u32 NMUINTCLRL_offset = 0x34;
const u32 NMUINTREQH_offset = 0x38;
const u32 NMUINTREQL_offset = 0x3C;

struct NMC3_extension
{
	unsigned int Num;
	char Name[ARRAY_SIZE(NMC3_FILENAME) + 2]; // <NMC3_FILENAME>_X

	struct cdev cdev;
	struct semaphore sem;
	int irq_HP, irq_LP;

	wait_queue_head_t queue;
	spinlock_t Wait_interrupt_lock;
	struct list_head Wait_interrupt_list;

	int Ready_for_command;
	int Program_stopped;
	int Sync_flag;
	int User_core_lock;
	spinlock_t NM_status_lock;

	u32 mem_im0_u;
	u32 mem_im1_3_u;
	u32 mem_em1_u;
	u32 mem_im0_size;
	u32 mem_im1_3_size;
	u32 mem_em1_size;
	void __iomem *mem_im0_m;
	void __iomem *mem_im1_3_m;
	void __iomem *mem_em1_m;

	struct tasklet_struct HP_tasklet;
	struct tasklet_struct LP_tasklet;
};

struct Wait_interrupt
{
	struct list_head list;
	int tgid;
	unsigned int int_type;
	int int_NM;
	u32 int_status;
};

#endif // __BBP_NMC3__H__
