/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>

/* Max number of descriptors per channel */
#define MDMA_NUM_DESCS          32
#define MDMA_NUM_SUB_DESCS      32
#define MDMA_DESC_SIZE(chan)    (chan->desc_size)
#define MDMA_POOL_SIZE          262144
#define MDMA_PM_TIMEOUT         100
#define MDMA_BUS_WIDTH_128      128

//#define MDMA_MAX_TRANS_LEN 0x7FFFFFC
#define MDMA_MAX_TRANS_LEN 128

// descriptor flags
#define MDMA_BD_OWN  0x80000000
#define MDMA_BD_LINK 0x40000000
#define MDMA_BD_INT  0x20000000
#define MDMA_BD_STOP 0x10000000
#define MDMA_BD_INC  0x8000000

#define MDMA_BD_LEN_MASK 0x03FFFFFF

// channel settings
#define MDMA_CHAN_DESC_NORMAL 0x00000000
#define MDMA_CHAN_DESC_LONG   0x00000002
#define MDMA_CHAN_DESC_PITCH  0x00000003
#define MDMA_CHAN_ADD_INFO    0x00000010
#define MDMA_CHAN_DESC_GAP_SHIFT 16

/* Reset values for data attributes */
#define MDMA_AXCACHE_VAL        0x3
#define MDMA_ARLEN_RST_VAL      0xF
#define MDMA_AWLEN_RST_VAL      0xF

#define MDMA_IRQ_STATUS_RX      BIT(0)
#define MDMA_IRQ_STATUS_TX      BIT(16)

#define MDMA_IRQ_SUSPEND_DONE   BIT(0)
#define MDMA_IRQ_CANCEL_DONE    BIT(1)
#define MDMA_IRQ_INT_DESC       BIT(2)
#define MDMA_IRQ_BAD_DESC       BIT(3)
#define MDMA_IRQ_STOP_DESC      BIT(4)
#define MDMA_IRQ_DISCARD_DESC   BIT(5)
#define MDMA_IRQ_WAXI_ERR       BIT(6)
#define MDMA_IRQ_AXI_ERR        BIT(7)
#define MDMA_IRQ_START_BY_EVENT BIT(8)
#define MDMA_IRQ_IGNORE_EVENT   BIT(9)

#define MDMA_IRQ_SENS_MASK (MDMA_IRQ_INT_DESC | MDMA_IRQ_BAD_DESC | \
                            MDMA_IRQ_DISCARD_DESC | MDMA_IRQ_WAXI_ERR | \
                            MDMA_IRQ_AXI_ERR)

#define MDMA_DESC_ALIGNMENT 16

#define to_chan(chan)      container_of(chan, struct mdma_chan, slave)
#define tx_to_desc(tx)     container_of(tx, struct mdma_desc_sw, async_tx)
#define pool_to_chan(pool) container_of(pool, struct mdma_chan, desc_pool)


typedef const volatile unsigned int roreg32;
typedef volatile unsigned int rwreg32;
typedef const volatile unsigned long long roreg64;
typedef volatile unsigned long long rwreg64;

struct channel_regs
{
    rwreg32 enable;                         /* 0x100 - enable channel       */
    rwreg32 suspend;                        /* 0x104 - suspend channel      */
    rwreg32 cancel;                         /* 0x108 - cancel channel       */
    roreg32 _skip01;                        /* 0x10C                        */
    rwreg32 settings;                       /* 0x110 - channel settings     */
    rwreg32 irq_mask;                       /* 0x114 - channel irq mask     */
    roreg32 status;                         /* 0x118 - channel status       */
    roreg32 _skip02;                        /* 0x11C                        */
    rwreg32 desc_addr;                      /* 0x120 - first decriptor addr */
    roreg32 _skip03;                        /* 0x124                        */
    roreg32 curr_desc_addr;                 /* 0x128 - current decript addr */
    roreg32 curr_addr;                      /* 0x12C - current trans addr   */
    roreg32 dma_state;                      /* 0x130 - state of DMA         */
    roreg32 _skip04[3];                     /* 0x134 - 0x13C                */
    rwreg32 desc_axlen;                     /* 0x140 - axlen for desc ops   */
    rwreg32 desc_axcache;                   /* 0x144 - axcache for desc ops */
    rwreg32 desc_axprot;                    /* 0x148 - axprot for desc ops  */
    rwreg32 desc_axlock;                    /* 0x14C - axlock for desc ops  */
    roreg32 desc_rresp;                     /* 0x150 - rresp of desc ops    */
    roreg32 desc_raxi_err_addr;             /* 0x154 - addr of axi read err */
    roreg32 desc_bresp;                     /* 0x158 - bresp of desc ops    */
    roreg32 desc_waxi_err_addr;             /* 0x15C - addr of axi write err*/
    rwreg32 desc_permut;                    /* 0x160 - byte swapping scheme */
    roreg32 _skip05[7];                     /* 0x164 - 0x17C                */
    rwreg32 max_trans;                      /* 0x180 - max unfinished trans */
    rwreg32 axlen;                          /* 0x184 - axi awlen            */
    rwreg32 axcache;                        /* 0x188 - axi awcache          */
    rwreg32 axprot;                         /* 0x18C - axi awprot           */
    rwreg32 axlock;                         /* 0x190 - axi awlock           */
    roreg32 bresp;                          /* 0x194 - axi operation bresp  */
    roreg32 xaxi_err_addr;                  /* 0x198 - addr of axi write err*/
    roreg32 _skip06;                        /* 0x19C                        */
    roreg32 state;                          /* 0x1A0 - axi state            */
    roreg32 avaliable_space;                /* 0x1A4 - num of free bytes    */
    rwreg32 permutation;                    /* 0x1A8 - byte swapping scheme */
    roreg32 _skip07[5];                     /* 0x1AC - 0x1BC                */
/************************ Device dependent registers ************************/
    rwreg32 sense_list;                     /* 0x1C0 - event mask           */
    rwreg32 signal_time;                    /* 0x1C4 - signal time in ticks */
    rwreg32 events_prior_l;                 /* 0x1C8 - events priority      */
    rwreg32 events_prior_h;                 /* 0x1CC - events priority      */
    rwreg32 active_events;                  /* 0x1D0 - events activity state*/
    rwreg32 ignore_events;                  /* 0x1D4 - events list - see ref*/
    rwreg32 synch_events;                   /* 0x1D8 - synchronization prio */
    roreg32 _skip08;                        /* 0x1DC                        */
    rwreg32 event_desc_addr[8];             /* 0x1E0-0x1FF- enevts descriptors*/
} __attribute__ ((packed));

struct mdma_regs
{
    roreg32 id;                             /* 0x000 - device id            */
    roreg32 version;                        /* 0x004 - device version       */
    rwreg32 soft_reset;                     /* 0x008 - soft reset           */
    roreg32 _skip01;                        /* 0x00C                        */
    rwreg32 event_sence_channel;            /* 0x010 - 0-read channel, 1-w  */
    roreg32 _skip02;                        /* 0x014                        */
    rwreg32 status;                         /* 0x018 - DMA status           */
    roreg32 _skip03[57];                    /* 0x01C - 0x0ff                */
    struct channel_regs rx;                 /* 0x100 - 0x1FF                */
    struct channel_regs tx;                 /* 0x200 - 0x2FF                */
} __attribute__ ((packed));

/**
 * struct mdma_desc_long_ll - Long HW descriptor
 * @usrdata_l: user data depends on descriptor kind
 * @usrdata_h: user data depends on descriptor kind
 * @memptr: Buffer address/Next descriptor address
 * @flags_length: Control word
 */
struct mdma_desc_long_ll {
	unsigned int usrdata_l;
	unsigned int usrdata_h;
	unsigned int memptr;
	unsigned int flags_length;
} __attribute__((packed, aligned(MDMA_DESC_ALIGNMENT)));

/**
 * struct mdma_desc_sw - Per Transaction structure
 * @src: Source address for simple mode dma
 * @dst: Destination address for simple mode dma
 * @len: Transfer length for simple mode dma
 * @node: Node in the channel descriptor list
 * @async_tx: Async transaction descriptor
 * @src_v: Virtual address of the src descriptor
 * @src_p: Physical address of the src descriptor
 * @dst_v: Virtual address of the dst descriptor
 * @dst_p: Physical address of the dst descriptor
 */
struct mdma_desc_sw {
	struct list_head node;
	struct dma_async_tx_descriptor async_tx;
	struct dma_slave_config config;
	unsigned pos;
	unsigned cnt;
	bool completed;
};

struct mdma_desc_pool {
	struct mdma_desc_long_ll *descs;
	unsigned                  size;
	unsigned                  cnt;
	unsigned                  head;
	unsigned                  next;

	struct sg_table           sgt;
	struct page             **pages;
	void*                     ptr;
};

struct mdma_chan {
	struct mdma_device *mdev;
	struct channel_regs __iomem *regs;
	spinlock_t lock;
	struct list_head pending_list;
	struct list_head free_list;
	struct mdma_desc_sw *active_desc;
	struct mdma_desc_sw *prepared_desc;
	struct mdma_desc_sw *sw_desc_pool;
	u32 desc_free_cnt;
	struct dma_chan slave;
	struct mdma_desc_pool desc_pool;
	struct device *dev;
	u32 desc_size;
	u32 type;
	enum dma_transfer_direction dir;
	bool err;
	u32 bus_width;
	struct dma_slave_config config;
};

struct mdma_of_data {
	bool max_transaction;
	enum dma_transfer_direction dirs[2];

	int (*device_alloc_chan_resources)(struct dma_chan *chan);
	void (*device_free_chan_resources)(struct dma_chan *chan);

	struct dma_async_tx_descriptor *(*device_prep_dma_memcpy)(
		struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags);
	struct dma_async_tx_descriptor *(*device_prep_slave_sg)(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context);
	int (*device_config)(struct dma_chan *chan,
	                     struct dma_slave_config *config);
	int (*device_terminate_all)(struct dma_chan *chan);
	void (*device_issue_pending)(struct dma_chan *chan);

	dma_cookie_t (*tx_submit)(struct dma_async_tx_descriptor *tx);

	irq_handler_t irq_handler;
	void (*tasklet_func)(unsigned long);
};

struct mdma_device {
	struct mdma_regs __iomem *regs;
	/* To protect channel manipulation */
	spinlock_t lock;
	struct dma_device slave;
	struct mdma_chan *ch[2];
	struct device* dev;
	struct clk *clk;
	struct tasklet_struct tasklet;
	const struct mdma_of_data* of_data;
};

int mdma_desc_pool_alloc(struct mdma_desc_pool* pool, unsigned cnt);
void mdma_desc_pool_free(struct mdma_desc_pool* pool);
dma_addr_t mdma_desc_pool_get_addr(struct mdma_desc_pool* pool, unsigned pos);
unsigned mdma_desc_pool_get(struct mdma_desc_pool* pool, 
                            unsigned cnt, unsigned *pos);
unsigned mdma_desc_pool_fill(struct mdma_desc_pool* pool, unsigned pos, 
                             dma_addr_t dma_addr, size_t len, bool stop_int);
unsigned mdma_desc_pool_fill_like(struct mdma_desc_pool* pool, unsigned pos, 
                                  dma_addr_t dma_addr, size_t len,
                                  bool stop_int,
                                  struct mdma_desc_pool* pool_base, 
                                  unsigned pos_base);
unsigned mdma_desc_pool_fill_sg(struct mdma_desc_pool* pool, unsigned pos, 
                                struct scatterlist *sg, unsigned int sg_len,
                                bool stop_int);
void mdma_desc_pool_put(struct mdma_desc_pool* pool,
                        unsigned pos, unsigned cnt);
void mdma_desc_pool_sync(struct mdma_desc_pool* pool,
                         unsigned pos, unsigned cnt);

unsigned mdma_cnt_desc_needed(struct mdma_chan *chan, struct scatterlist *sgl,
                              unsigned int sg_len, size_t *len);

bool mdma_prepare_transfer(struct mdma_chan *chan);
void mdma_start_transfer(struct mdma_chan *chan);
struct mdma_desc_sw *mdma_get_descriptor(struct mdma_chan *chan);
void mdma_free_descriptor(struct mdma_chan *chan, struct mdma_desc_sw *sdesc);
dma_cookie_t mdma_tx_submit(struct dma_async_tx_descriptor *tx);
int mdma_device_terminate_all(struct dma_chan *dchan);
int mdma_alloc_chan_resources(struct dma_chan *dchan);
void mdma_free_chan_resources(struct dma_chan *dchan);
int mdma_device_config(struct dma_chan *dchan, struct dma_slave_config *config);
void mdma_complete_descriptor(struct mdma_chan *chan, 
                              struct mdma_desc_sw *desc, bool status,
                              struct dmaengine_desc_callback* cb);
void mdma_reset(struct mdma_device *mdev);

bool mdma_check_align(struct mdma_chan *chan, dma_addr_t dma_addr);
bool mdma_check_align_sg(struct mdma_chan *chan, struct scatterlist *sg);
