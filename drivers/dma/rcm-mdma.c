/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/dmaengine.h>
#include <sound/dmaengine_pcm.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include "dmaengine.h"

/* Max number of descriptors per channel */
#define MDMA_NUM_DESCS          32
#define MDMA_NUM_SUB_DESCS      32
#define MDMA_DESC_SIZE(chan)    (chan->desc_size)
#define MDMA_POOL_SIZE          262144
#define MDMA_PM_TIMEOUT         100
#define MDMA_BUS_WIDTH_128      128

#define MDMA_MAX_TRANS_LEN 0x7FFFFFC

// descriptor flags
#define MDMA_BD_OWN  0x80000000
#define MDMA_BD_LINK 0x40000000
#define MDMA_BD_INT  0x20000000
#define MDMA_BD_STOP 0x10000000
#define MDMA_BD_INC  0x8000000

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

#define to_chan(chan)		container_of(chan, struct mdma_chan, \
					     slave)
#define tx_to_desc(tx)		container_of(tx, struct mdma_desc_sw, \
					     async_tx)
#define pool_to_chan(pool)	container_of(pool, struct mdma_chan, \
					     desc_pool)


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
	unsigned pos_src;
	unsigned cnt_src;
	unsigned pos_dst;
	unsigned cnt_dst;
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
	struct mdma_regs __iomem *regs;
	spinlock_t lock;
	struct list_head pending_list;
	struct list_head free_list;
	struct list_head active_list;
	struct list_head done_list;
	struct mdma_desc_sw *sw_desc_pool;
	u32 desc_free_cnt;
	struct dma_chan slave;
	struct mdma_desc_pool desc_pool;
	struct device *dev;
	int irq;
	struct tasklet_struct tasklet;
	bool idle;
	u32 desc_size;
	u32 type;
	bool err;
	u32 bus_width;
	struct dma_slave_config config;
};

struct mdma_device {
	/* To protect channel manipulation */
	spinlock_t lock;
	struct dma_device slave;
	struct mdma_chan *chan;
	struct device* dev;
	struct clk *clk;
};

static void mdma_desc_pool_free(struct mdma_desc_pool* pool)
{
	struct mdma_chan *chan = pool_to_chan(pool);

	if (pool->sgt.sgl) {
		dma_unmap_sg(chan->mdev->dev, pool->sgt.sgl, pool->sgt.nents,
		             DMA_TO_DEVICE);
		sg_free_table(&pool->sgt);
	}

	if (pool->pages) {
		kfree(pool->pages);
		pool->pages = NULL;
	}

	if (pool->ptr) {
		vfree(pool->ptr);
		pool->ptr = NULL;
	}
}

static int mdma_desc_pool_alloc(struct mdma_desc_pool* pool, unsigned cnt)
{
	struct mdma_chan *chan = pool_to_chan(pool);
	int ret = 0;
	int nr_pages; 
	struct scatterlist* sg;
	void *p; 
	int i;
	unsigned long size = cnt * sizeof(struct mdma_desc_long_ll);
	struct mdma_desc_long_ll link = {0};

	pool->ptr = vmalloc(size + MDMA_DESC_ALIGNMENT);

	if (!pool->ptr) {
		pr_err("%s: Failed to allocate memory (%lu bytes).\n", __func__,
		       size + MDMA_DESC_ALIGNMENT);
		return -ENOMEM;
	}

	memset(pool->ptr, 0, size + MDMA_DESC_ALIGNMENT);

	pool->descs = (struct mdma_desc_long_ll *)ALIGN((size_t)pool->ptr, 
	                                                MDMA_DESC_ALIGNMENT);

	pool->size = cnt;
	pool->cnt = cnt;
	pool->next = 0;
	pool->head = 0;

	nr_pages = DIV_ROUND_UP((unsigned long)pool->descs + size, PAGE_SIZE) -
	           (unsigned long)pool->descs / PAGE_SIZE;
	pool->pages = kmalloc_array(nr_pages, sizeof(struct page *), 
	                            GFP_KERNEL);
	if (!pool->pages) {
		pr_err("%s: Failed to allocate array for %d pages.\n",
		       __func__, nr_pages);
		ret = -ENOMEM;
		goto err_free_pool;
	}

	p = (void*)pool->descs - offset_in_page(pool->descs);
	for (i = 0; i < nr_pages; i++) {
		if (is_vmalloc_addr(p))
			pool->pages[i] = vmalloc_to_page(p);
		else
			pool->pages[i] = virt_to_page((void *)p);
		if (!pool->pages[i]) {
			pr_err("%s: Failed to get page for address 0x%px.\n",
			       __func__, p);
			ret = -EFAULT;
			goto err_free_pool;
		}

		p += PAGE_SIZE;
	}

/*
	ret = __sg_alloc_table_from_pages(&pool->sgt, pool->pages, nr_pages,
	                                  offset_in_page(pool->descs), size,
	                                  PAGE_SIZE, GFP_KERNEL);
*/

	ret = sg_alloc_table_from_pages(&pool->sgt, pool->pages, nr_pages,
	                                offset_in_page(pool->descs), size,
	                                GFP_KERNEL);

	if (ret) {
		pr_err("%s: Failed to allocate SG-table (%d pages).\n",
		       __func__, nr_pages);
		ret = -EFAULT;
		goto err_free_pool;
	}

	ret = dma_map_sg(chan->mdev->dev, pool->sgt.sgl, pool->sgt.nents, 
	                 DMA_TO_DEVICE);
	if (ret == 0) {
		pr_err("%s: Failed to map SG-table.\n", __func__);
		ret = -EFAULT;
		goto err_free_pool;
	}

	link.flags_length = MDMA_BD_LINK;

	p = (void*)pool->descs;

	for_each_sg(pool->sgt.sgl, sg, pool->sgt.nents, i) {
		u32 len  = sg_dma_len(sg);

		if (i == pool->sgt.nents - 1) {
			link.memptr = sg_dma_address(pool->sgt.sgl);
		} else {
			link.memptr = sg_dma_address(sg_next(sg));
		}

		memcpy(p + len - sizeof(link), &link, sizeof(link));

		p += len;
	}

	pr_debug("%s: descriptor's pool allocated "
	         "(%u descriptors in %u sg-entries)\n",
	         __func__, cnt, pool->sgt.nents);

	return 0;

err_free_pool:
	mdma_desc_pool_free(pool);
	return ret;
}

static dma_addr_t mdma_desc_pool_get_addr(struct mdma_desc_pool* pool,
                                          unsigned pos)
{
	struct scatterlist* sg;
	int i;

	for_each_sg(pool->sgt.sgl, sg, pool->sgt.nents, i) {
		u32 len = sg_dma_len(sg);
		u32 cnt = len / sizeof(struct mdma_desc_long_ll);

		if (cnt <= pos) {
			pos -= cnt;
		} else {
			dma_addr_t addr = sg_dma_address(sg);
			return addr + pos * sizeof(struct mdma_desc_long_ll);
		}
	}

	return 0;
}

static unsigned mdma_desc_pool_get(struct mdma_desc_pool* pool, 
                                   unsigned cnt, unsigned *pos)
{
	unsigned i;
	unsigned n;

	pr_debug("%s(%u) >>> next = %u, head = %u, cnt = %u, size = %u\n",
	          __func__, cnt, pool->next, pool->head, pool->cnt, pool->size);

	if (cnt > pool->cnt)
		return 0;

	i = pool->next;
	n = 0;

	while ((n != cnt) && (cnt <= pool->cnt)) {
		if (pool->descs[i].flags_length & MDMA_BD_LINK)
			++cnt;
		++n;
		i = (i + 1) % pool->size;
	}

	if (n != cnt) {
		pr_debug("%s: n = %u, cnt = %u\n", __func__, n, cnt);
		return 0;
	}

	i = pool->next;

	if (pos)
		*pos = i;

	pool->cnt -= cnt;
	pool->next = (pool->next + cnt) % pool->size;

	pr_debug("%s(%u) <<< next = %u, head = %u, cnt = %u, size = %u\n",
	          __func__, cnt, pool->next, pool->head, pool->cnt, pool->size);

	return cnt;
}

static void mdma_desc_pool_put(struct mdma_desc_pool* pool,
                               unsigned pos, unsigned cnt)
{
	pr_debug("%s(%u, %u) >>> next = %u, head = %u, cnt = %u, size = %u\n",
	          __func__, pos, cnt, pool->next, pool->head, pool->cnt, pool->size);

	if (pos != pool->head) {
		pr_warn("%s: incorrect pool operation: pool(head = %u, cnt = %u)"
		        ", trying to put %u items to pos %u.\n",
		        __func__, pool->head, pool->cnt, cnt, pos);
	}

	if (cnt > pool->size - pool->cnt) {
		pr_warn("%s: incorrect pool operation: "
		        "pool(size = %u, cnt = %u), trying to put %u items.\n",
		        __func__, pool->size, pool->cnt, cnt);
		cnt = pool->size - pool->cnt;
	}

	pool->head = (pool->head + cnt) % pool->size;
	pool->cnt += cnt;

	pr_debug("%s(%u, %u) <<< next = %u, head = %u, cnt = %u, size = %u\n",
	          __func__, pos, cnt, pool->next, pool->head, pool->cnt, pool->size);
}

static void mdma_desc_pool_sync(struct mdma_desc_pool* pool,
                                unsigned pos, unsigned cnt)
{
	struct mdma_chan *chan = pool_to_chan(pool);
	struct scatterlist* sg;
	int i;
	unsigned cnt_sync = 0;

	for_each_sg(pool->sgt.sgl, sg, pool->sgt.nents, i) {
		u32 len = sg_dma_len(sg);
		u32 n = len / sizeof(struct mdma_desc_long_ll);

		if (n <= pos) {
			pos -= n;
		} else {
			break;
		}
	}

	do {
		dma_addr_t addr = sg_dma_address(sg);
		u32 len = sg_dma_len(sg);

		dma_sync_single_for_device(chan->mdev->dev, addr, len, 
		                           DMA_TO_DEVICE);

		cnt_sync += len / sizeof(struct mdma_desc_long_ll);

		if (pos != 0) {
			cnt_sync -= pos;
			pos = 0;
		}

		sg = (sg_is_last(sg)) ? pool->sgt.sgl : sg_next(sg);
	} while (cnt_sync < cnt);
}

/**
 * mdma_start - Start DMA channel
 * @chan: MDMA channel pointer
 */
static void mdma_start(struct mdma_chan *chan)
{
	writel(MDMA_IRQ_SENS_MASK, &chan->regs->tx.irq_mask);
	writel(MDMA_IRQ_SENS_MASK, &chan->regs->rx.irq_mask);
	writel(1, &chan->regs->tx.enable);
	writel(1, &chan->regs->rx.enable);
	chan->idle = false;
}


/**
 * mdma_update_desc_to_ctrlr - Updates descriptor to the controller
 * @chan: MDMA DMA channel pointer
 * @desc: Transaction descriptor pointer
 */
static void mdma_update_desc_to_ctrlr(struct mdma_chan *chan,
				      struct mdma_desc_sw *desc)
{
	dma_addr_t src_addr = mdma_desc_pool_get_addr(&chan->desc_pool, 
	                                              desc->pos_src);
	dma_addr_t dst_addr = mdma_desc_pool_get_addr(&chan->desc_pool, 
	                                              desc->pos_dst);

	writel(src_addr, &chan->regs->rx.desc_addr);
	writel(dst_addr, &chan->regs->tx.desc_addr);
}


static void mdma_config(struct mdma_chan *chan, struct mdma_desc_sw *desc)
{
	// todo ADD_INFO for slave mode
	writel(chan->type | (MDMA_DESC_SIZE(chan) << MDMA_CHAN_DESC_GAP_SHIFT),
	       &chan->regs->rx.settings);

	writel(chan->type | (MDMA_DESC_SIZE(chan) << MDMA_CHAN_DESC_GAP_SHIFT),
	       &chan->regs->tx.settings);

	writel(desc->config.src_maxburst, &chan->regs->rx.axlen);
	writel(desc->config.dst_maxburst, &chan->regs->tx.axlen);
}


/**
 * mdma_start_transfer - Initiate the new transfer
 * @chan: MDMA channel pointer
 */
static void mdma_start_transfer(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc;

	if (!chan->idle)
		return;

	desc = list_first_entry_or_null(&chan->pending_list,
					struct mdma_desc_sw, node);
	if (!desc)
		return;

	mdma_config(chan, desc);

	list_splice_tail_init(&chan->pending_list, &chan->active_list);
	mdma_update_desc_to_ctrlr(chan, desc);
	mdma_start(chan);
}

/**
 * mdma_get_descriptor - Get the sw descriptor from the pool
 * @chan: MDMA channel pointer
 *
 * Return: The sw descriptor
 */
static struct mdma_desc_sw *
mdma_get_descriptor(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc;

	if (!chan->desc_free_cnt) {
		dev_dbg(chan->dev, "chan %p descs are not available\n", chan);
		return NULL;
	}

	desc = list_first_entry(&chan->free_list,
				struct mdma_desc_sw, node);
	list_del(&desc->node);
	chan->desc_free_cnt--;

	desc->cnt_src = 0;
	desc->cnt_dst = 0;

	desc->config = chan->config;

	return desc;
}


/**
 * mdma_free_descriptor - Free descriptor
 * @chan: MDMA channel pointer
 * @sdesc: Transaction descriptor pointer
 */
static void mdma_free_descriptor(struct mdma_chan *chan,
				 struct mdma_desc_sw *sdesc)
{
	if (sdesc->cnt_src)
		mdma_desc_pool_put(&chan->desc_pool, sdesc->pos_src,
		                   sdesc->cnt_src);
	if (sdesc->cnt_dst)
		mdma_desc_pool_put(&chan->desc_pool, sdesc->pos_dst,
		                   sdesc->cnt_dst);
	chan->desc_free_cnt++;
	list_add_tail(&sdesc->node, &chan->free_list);
}

/**
 * mdma_free_desc_list - Free descriptors list
 * @chan: MDMA channel pointer
 * @list: List to parse and delete the descriptor
 */
static void mdma_free_desc_list(struct mdma_chan *chan,
				      struct list_head *list)
{
	struct mdma_desc_sw *desc, *next;

	list_for_each_entry_safe(desc, next, list, node) {
		list_del(&desc->node);
		mdma_free_descriptor(chan, desc);
	}
}


/**
 * mdma_free_descriptors - Free channel descriptors
 * @chan: MDMA channel pointer
 */
static void mdma_free_descriptors(struct mdma_chan *chan)
{
	mdma_free_desc_list(chan, &chan->active_list);
	mdma_free_desc_list(chan, &chan->pending_list);
	mdma_free_desc_list(chan, &chan->done_list);
}

/**
 * mdma_tx_submit - Submit DMA transaction
 * @tx: Async transaction descriptor pointer
 *
 * Return: cookie value
 */
static dma_cookie_t mdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct mdma_chan *chan = to_chan(tx->chan);
	struct mdma_desc_sw *desc;
	dma_cookie_t cookie;
	unsigned long irqflags;

	desc = tx_to_desc(tx);
	spin_lock_irqsave(&chan->lock, irqflags);
	cookie = dma_cookie_assign(tx);

	list_add_tail(&desc->node, &chan->pending_list);
	spin_unlock_irqrestore(&chan->lock, irqflags);

	return cookie;
}


/**
 * mdma_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 *
 * Put the driver into low power mode.
 * Return: 0 on success and failure value on error
 */
static int __maybe_unused mdma_suspend(struct device *dev)
{
	if (!device_may_wakeup(dev))
		return pm_runtime_force_suspend(dev);

	return 0;
}

/**
 * mdma_resume - Resume from suspend
 * @dev:	Address of the device structure
 *
 * Resume operation after suspend.
 * Return: 0 on success and failure value on error
 */
static int __maybe_unused mdma_resume(struct device *dev)
{
	if (!device_may_wakeup(dev))
		return pm_runtime_force_resume(dev);

	return 0;
}

/**
 * mdma_runtime_suspend - Runtime suspend method for the driver
 * @dev:	Address of the device structure
 *
 * Put the driver into low power mode.
 * Return: 0 always
 */
static int __maybe_unused mdma_runtime_suspend(struct device *dev)
{
	struct mdma_device *mdev = dev_get_drvdata(dev);

	clk_disable_unprepare(mdev->clk);

	return 0;
}

/**
 * mdma_runtime_resume - Runtime suspend method for the driver
 * @dev:	Address of the device structure
 *
 * Put the driver into low power mode.
 * Return: 0 always
 */
static int __maybe_unused mdma_runtime_resume(struct device *dev)
{
	struct mdma_device *mdev = dev_get_drvdata(dev);
	int err;

	err = clk_prepare_enable(mdev->clk);
	if (err) {
		dev_err(dev, "Unable to enable main clock.\n");
		return err;
	}

	return 0;
}

static const struct dev_pm_ops mdma_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mdma_suspend, mdma_resume)
	SET_RUNTIME_PM_OPS(mdma_runtime_suspend,
			   mdma_runtime_resume, NULL)
};

/**
 * mdma_prep_memcpy - prepare descriptors for memcpy transaction
 * @dchan: DMA channel
 * @dma_dst: Destination buffer address
 * @dma_src: Source buffer address
 * @len: Transfer length
 * @flags: transfer ack flags
 *
 * Return: Async transaction descriptor on success and NULL on failure
 */
static struct dma_async_tx_descriptor *mdma_prep_memcpy(
				struct dma_chan *dchan, dma_addr_t dma_dst,
				dma_addr_t dma_src, size_t len, ulong flags)
{
	struct mdma_chan *chan;
	struct mdma_desc_sw *sw_desc = NULL;
	unsigned cnt_descs = 0;
	struct mdma_desc_long_ll *src_desc, *dst_desc;
	unsigned pos_src;
	unsigned pos_dst;
	size_t copy;
	unsigned long irqflags;
	int addr_mask;
	struct mdma_desc_pool pool_save;

	chan = to_chan(dchan);

	addr_mask = chan->bus_width/8 - 1;
	if((dma_src & addr_mask) || (dma_dst & addr_mask))
	{
		dev_dbg(chan->dev, "DMA unalligned access %x -> %x\n", dma_src, dma_dst);
		return NULL;
	}

	copy = len;

	do {
		++cnt_descs;
		copy -= min_t(size_t, copy, MDMA_MAX_TRANS_LEN);
	} while (copy);

	spin_lock_irqsave(&chan->lock, irqflags);

	sw_desc = mdma_get_descriptor(chan);

	pool_save = chan->desc_pool;

	sw_desc->cnt_src = mdma_desc_pool_get(&chan->desc_pool, cnt_descs,
	                                      &sw_desc->pos_src);
	sw_desc->cnt_dst = mdma_desc_pool_get(&chan->desc_pool, cnt_descs,
	                                      &sw_desc->pos_dst);

	if ((!sw_desc->cnt_src) || (!sw_desc->cnt_dst)) {
		chan->desc_pool = pool_save;
		mdma_free_descriptor(chan, sw_desc);
		spin_unlock_irqrestore(&chan->lock, irqflags);
		dev_dbg(chan->dev, 
		        "chan %p: can't get %u descriptors from pool\n",
		        chan, cnt_descs);
		return NULL;
	}

	spin_unlock_irqrestore(&chan->lock, irqflags);

	src_desc = &chan->desc_pool.descs[sw_desc->pos_src];
	dst_desc = &chan->desc_pool.descs[sw_desc->pos_dst];

	pos_src = sw_desc->pos_src;
	pos_dst = sw_desc->pos_dst;

	do {
		copy = min_t(size_t, len, MDMA_MAX_TRANS_LEN);
		len -= copy;

		if (src_desc->flags_length & MDMA_BD_LINK) {
			pr_debug("%s: Link at pos %u (RX)\n", __func__, pos_src);
			src_desc->flags_length = MDMA_BD_LINK;

			if (pos_src + 1 == chan->desc_pool.size)
				src_desc = chan->desc_pool.descs;
			else
				++src_desc;

			pos_src = (pos_src + 1) % chan->desc_pool.size;
		}

		if (dst_desc->flags_length & MDMA_BD_LINK) {
			pr_debug("%s: Link at pos %u (TX)\n", __func__, pos_dst);
			dst_desc->flags_length = MDMA_BD_LINK;

			if (pos_dst + 1 == chan->desc_pool.size)
				dst_desc = chan->desc_pool.descs;
			else
				++dst_desc;

			pos_dst = (pos_dst + 1) % chan->desc_pool.size;
		}

		if(!len)
		{
			// last sub descriptor
			dst_desc->flags_length = copy | MDMA_BD_STOP | MDMA_BD_INT;
			src_desc->flags_length = copy | MDMA_BD_STOP;
		}
		else
			dst_desc->flags_length = src_desc->flags_length = copy;

		src_desc->memptr = dma_src;
		dst_desc->memptr = dma_dst;
		dma_src += copy;
		dma_dst += copy;

		if (pos_src + 1 == chan->desc_pool.size)
			src_desc = chan->desc_pool.descs;
		else
			++src_desc;

		if (pos_dst + 1 == chan->desc_pool.size)
			dst_desc = chan->desc_pool.descs;
		else
			++dst_desc;

		pos_src = (pos_src + 1) % chan->desc_pool.size;
		pos_dst = (pos_dst + 1) % chan->desc_pool.size;
	} while (len);

	mdma_desc_pool_sync(&chan->desc_pool, sw_desc->pos_src,
	                    sw_desc->cnt_src);
	mdma_desc_pool_sync(&chan->desc_pool, sw_desc->pos_dst,
	                    sw_desc->cnt_dst);

	async_tx_ack(&sw_desc->async_tx);
	sw_desc->async_tx.flags = flags;

	return &sw_desc->async_tx;
}


/**
 * mdma_device_terminate_all - Aborts all transfers on a channel
 * @dchan: DMA channel pointer
 *
 * Return: Always '0'
 */
static int mdma_device_terminate_all(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);
	writel(1, &chan->regs->rx.cancel);
	writel(1, &chan->regs->tx.cancel);
	mdma_free_descriptors(chan);
	spin_unlock_irqrestore(&chan->lock, irqflags);
	return 0;
}


/**
 * mdma_issue_pending - Issue pending transactions
 * @dchan: DMA channel pointer
 */
static void mdma_issue_pending(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);
	mdma_start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, irqflags);
}


/**
 * of_mdma_xlate - Translation function
 * @dma_spec: Pointer to DMA specifier as found in the device tree
 * @ofdma: Pointer to DMA controller data
 *
 * Return: DMA channel pointer on success and NULL on error
 */
static struct dma_chan *of_mdma_xlate(struct of_phandle_args *dma_spec,
                                      struct of_dma *ofdma)
{
	struct mdma_device *mdev = ofdma->of_dma_data;

	return dma_get_slave_channel(&mdev->chan->slave);
}


/**
 * mdma_alloc_chan_resources - Allocate channel resources
 * @dchan: DMA channel
 *
 * Return: Number of descriptors on success and failure value on error
 */
static int mdma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	struct mdma_desc_sw *desc;
	int i, ret;

	ret = pm_runtime_get_sync(chan->dev);
	if (ret < 0)
		return ret;

	chan->sw_desc_pool = kcalloc(MDMA_NUM_DESCS, sizeof(*desc),
				     GFP_KERNEL);
	if (!chan->sw_desc_pool)
		return -ENOMEM;

	chan->idle = true;
	chan->desc_free_cnt = MDMA_NUM_DESCS;

	INIT_LIST_HEAD(&chan->free_list);

	for (i = 0; i < MDMA_NUM_DESCS; i++) {
		desc = chan->sw_desc_pool + i;
		dma_async_tx_descriptor_init(&desc->async_tx, &chan->slave);
		desc->async_tx.tx_submit = mdma_tx_submit;
		list_add_tail(&desc->node, &chan->free_list);
	}

	ret = mdma_desc_pool_alloc(&chan->desc_pool, MDMA_POOL_SIZE);

	if (ret)
		return ret;

	return MDMA_POOL_SIZE;
}

/**
 * mdma_free_chan_resources - Free channel resources
 * @dchan: DMA channel pointer
 */
static void mdma_free_chan_resources(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);
	mdma_free_descriptors(chan);
	spin_unlock_irqrestore(&chan->lock, irqflags);
	mdma_desc_pool_free(&chan->desc_pool);
	kfree(chan->sw_desc_pool);
	pm_runtime_mark_last_busy(chan->dev);
	pm_runtime_put_autosuspend(chan->dev);
}


/**
 * mdma_device_config - mdma device configuration
 * @dchan: DMA channel
 * @config: DMA device config
 *
 * Return: 0 always
 */
static int mdma_device_config(struct dma_chan *dchan,
                              struct dma_slave_config *config)
{
	struct mdma_chan *chan = to_chan(dchan);

	chan->config = *config;

	return 0;
}

/**
 * mdma_irq_handler - MDMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the MDMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t mdma_irq_handler(int irq, void *data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	u32 status;
	bool need_tasklet = false;

	status = readl(&chan->regs->status);

	pr_debug("%s: Channel %p interrupt, status %x\n",
	          __func__, chan, status);

	if ((status & (MDMA_IRQ_STATUS_TX | MDMA_IRQ_STATUS_RX)) == 0)
		return IRQ_NONE;

	if (status & MDMA_IRQ_STATUS_RX) {
		u32 rx_status = readl(&chan->regs->rx.status);
		pr_debug("%s: Channel %p RX-interrupt, status %x\n",
		         __func__, chan, rx_status);

		chan->err = true;
		need_tasklet = true;
	}

	if (status & MDMA_IRQ_STATUS_TX) {
		u32 tx_status = readl(&chan->regs->tx.status);
		pr_debug("%s: Channel %p TX-interrupt, status %x\n",
		         __func__, chan, tx_status);

		if (tx_status & MDMA_IRQ_INT_DESC) {
			chan->idle = true;
		} else {
			chan->err = true;
		}

		need_tasklet = true;
	}

	if (need_tasklet)
		tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

/**
 * mdma_chan_desc_cleanup - Cleanup the completed descriptors
 * @chan: MDMA channel
 */
static void mdma_chan_desc_cleanup(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc, *next;

	list_for_each_entry_safe(desc, next, &chan->done_list, node) {
		dma_async_tx_callback callback;
		void *callback_param;

		list_del(&desc->node);

		callback = desc->async_tx.callback;
		callback_param = desc->async_tx.callback_param;
		if (callback) {
			spin_unlock(&chan->lock);
			callback(callback_param);
			spin_lock(&chan->lock);
		}

		/* Run any dependencies, then free the descriptor */
		mdma_free_descriptor(chan, desc);
	}
}

/**
 * mdma_complete_descriptor - Mark the active descriptor as complete
 * @chan: MDMA channel pointer
 */
static void mdma_complete_descriptor(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc;

	desc = list_first_entry_or_null(&chan->active_list,
					struct mdma_desc_sw, node);
	if (!desc)
		return;
	list_del(&desc->node);
	dma_cookie_complete(&desc->async_tx);
	list_add_tail(&desc->node, &chan->done_list);
}


/**
 * mdma_init - Initialize the channel
 * @chan: MDMA channel pointer
 */
static void mdma_init(struct mdma_chan *chan)
{
	chan->idle = true;
}


/**
 * mdma_reset - Reset the channel
 * @chan: MDMA channel pointer
 */
static void mdma_reset(struct mdma_chan *chan)
{
	writel(1, &chan->regs->soft_reset);

	mdma_complete_descriptor(chan);
	mdma_chan_desc_cleanup(chan);
//	mdma_free_descriptors(chan);
	mdma_init(chan);
}


/**
 * mdma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the MDMA channel structure
 */
static void mdma_do_tasklet(unsigned long data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);

	if (chan->err) {
		mdma_reset(chan);
		chan->err = false;
		goto unlock;
	}

	mdma_complete_descriptor(chan);
	mdma_chan_desc_cleanup(chan);

	if (chan->idle)
		mdma_start_transfer(chan);

unlock:
	spin_unlock_irqrestore(&chan->lock, irqflags);
}


/**
 * mdma_chan_remove - Channel remove function
 * @chan: MDMA channel pointer
 */
static void mdma_chan_remove(struct mdma_chan *chan)
{
	if (!chan)
		return;

	if (chan->irq)
		devm_free_irq(chan->mdev->dev, chan->irq, chan);
	tasklet_kill(&chan->tasklet);
	list_del(&chan->slave.device_node);
}

/**
 * mdma_chan_probe - Per Channel Probing
 * @mdev: Driver specific device structure
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int mdma_chan_probe(struct mdma_device *mdev,
			   struct platform_device *pdev)
{
	struct mdma_chan *chan;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	int err;

	chan = devm_kzalloc(mdev->dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;
	chan->dev = mdev->dev;
	chan->mdev = mdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chan->regs = (struct mdma_regs*) devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chan->regs))
		return PTR_ERR(chan->regs);

	chan->bus_width = MDMA_BUS_WIDTH_128;
	chan->config.src_maxburst = MDMA_AWLEN_RST_VAL;
	chan->config.dst_maxburst = MDMA_ARLEN_RST_VAL;
	err = of_property_read_u32(node, "rcm,bus-width", &chan->bus_width);
	if (err < 0) {
		dev_err(&pdev->dev, "missing rcm,bus-width property\n");
		return err;
	}

	if (chan->bus_width != MDMA_BUS_WIDTH_128) {
		dev_err(mdev->dev, "invalid bus-width value");
		return -EINVAL;
	}

	mdev->chan = chan;
	tasklet_init(&chan->tasklet, mdma_do_tasklet, (ulong)chan);
	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->active_list);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->done_list);
	INIT_LIST_HEAD(&chan->free_list);

	dma_cookie_init(&chan->slave);
	chan->slave.device = &mdev->slave;
	list_add_tail(&chan->slave.device_node, &mdev->slave.channels);

	mdma_init(chan);
	chan->irq = platform_get_irq(pdev, 0);
	if (chan->irq < 0)
		return -ENXIO;
	err = devm_request_irq(&pdev->dev, chan->irq, mdma_irq_handler, 0,
	                       "mdma", chan);
	if (err)
		return err;

	chan->desc_size = sizeof(struct mdma_desc_long_ll);  // max size
	chan->type = MDMA_CHAN_DESC_LONG;
	chan->idle = true;
	return 0;
}


static int rcm_mdma_probe(struct platform_device *pdev)
{
	struct mdma_device *mdev;
	struct dma_device *p;
	int ret;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->dev = &pdev->dev;
	INIT_LIST_HEAD(&mdev->slave.channels);

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	dma_cap_set(DMA_MEMCPY, mdev->slave.cap_mask);

	p = &mdev->slave;
	p->device_prep_dma_memcpy = mdma_prep_memcpy;
	// p->device_prep_interleaved_dma = mdma_prep_interleaved_dma; // todo
	// p->device_prep_slave_sg = mdma_prep_slave_sg; // todo
	// p->device_prep_dma_memset = mdma_prep_dma_memset; // todo
	// p->device_prep_dma_memset_sg = mdma_prep_dma_memset_sg; // todo
	p->device_terminate_all = mdma_device_terminate_all;
	p->device_issue_pending = mdma_issue_pending;
	p->device_alloc_chan_resources = mdma_alloc_chan_resources;
	p->device_free_chan_resources = mdma_free_chan_resources;
	p->device_tx_status = dma_cookie_status;
	p->device_config = mdma_device_config;
	p->dev = &pdev->dev;

	mdev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mdev->clk)) {
		dev_err(&pdev->dev, "main clock not found.\n");
		return PTR_ERR(mdev->clk);
	}

	platform_set_drvdata(pdev, mdev);
	pm_runtime_set_autosuspend_delay(mdev->dev, MDMA_PM_TIMEOUT);
	pm_runtime_use_autosuspend(mdev->dev);
	pm_runtime_enable(mdev->dev);
	pm_runtime_get_sync(mdev->dev);
	if (!pm_runtime_enabled(mdev->dev)) {
		ret = mdma_runtime_resume(mdev->dev);
		if (ret)
			return ret;
	}

	ret = mdma_chan_probe(mdev, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Probing channel failed\n");
		goto err_disable_pm;
	}

	p->dst_addr_widths = BIT(mdev->chan->bus_width / 8);
	p->src_addr_widths = BIT(mdev->chan->bus_width / 8);

	dma_async_device_register(&mdev->slave);

	ret = of_dma_controller_register(pdev->dev.of_node,
					 of_mdma_xlate, mdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register DMA to DT\n");
		dma_async_device_unregister(&mdev->slave);
		goto free_chan_resources;
	}

	pm_runtime_mark_last_busy(mdev->dev);
	pm_runtime_put_sync_autosuspend(mdev->dev);

	dev_info(&pdev->dev, "MDMA driver Probe success\n");

	return 0;

free_chan_resources:
	mdma_chan_remove(mdev->chan);

err_disable_pm:
	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);
	pm_runtime_disable(mdev->dev);
	return ret;
}

static int rcm_mdma_remove(struct platform_device *pdev)
{
	struct mdma_device *mdev = platform_get_drvdata(pdev);

	of_dma_controller_free(mdev->dev->of_node);
	dma_async_device_unregister(&mdev->slave);
	mdma_chan_remove(mdev->chan);

	pm_runtime_disable(mdev->dev);
	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);

	return 0;
}

static const struct of_device_id rcm_mdma_dt_ids[] = {
	{
		.compatible = "rcm,mdma",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_mdma_dt_ids);

static struct platform_driver rcm_mdma_driver = {
	.driver =
		{
			.name = "rcm-mdma",
			.of_match_table = rcm_mdma_dt_ids,
			.pm = &mdma_dev_pm_ops,
		},
	.probe = rcm_mdma_probe,
	.remove = rcm_mdma_remove,
};

module_platform_driver(rcm_mdma_driver);


MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RC-Module MDMA driver");
MODULE_LICENSE("GPL");