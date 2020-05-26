/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 */

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
#define MDMA_NUM_DESCS	32
#define MDMA_DESC_SIZE(chan)	(chan->desc_size)
#define MDMA_PM_TIMEOUT			100
#define MDMA_BUS_WIDTH_128	128

/* Reset values for data attributes */
#define MDMA_AXCACHE_VAL    0x3
#define MDMA_ARLEN_RST_VAL	0xF
#define MDMA_AWLEN_RST_VAL	0xF


#define to_chan(chan)		container_of(chan, struct mdma_chan, \
					     slave)
#define tx_to_desc(tx)		container_of(tx, struct mdma_desc_sw, \
					     async_tx)

/**
 * struct mdma_desc_ll - Hw descriptor !!!todo
 * @addr: Buffer address
 * @size: Size of the buffer
 * @ctrl: Control word
 * @nxtdscraddr: Next descriptor base address
 * @rsvd: Reserved field and for Hw internal use.
 */
struct mdma_desc_ll {
	u64 addr;
	u32 size;
	u32 ctrl;
	u64 nxtdscraddr;
	u64 rsvd;
};

/**
 * struct mdma_desc_sw - Per Transaction structure
 * @src: Source address for simple mode dma
 * @dst: Destination address for simple mode dma
 * @len: Transfer length for simple mode dma
 * @node: Node in the channel descriptor list
 * @tx_list: List head for the current transfer
 * @async_tx: Async transaction descriptor
 * @src_v: Virtual address of the src descriptor
 * @src_p: Physical address of the src descriptor
 * @dst_v: Virtual address of the dst descriptor
 * @dst_p: Physical address of the dst descriptor
 */
struct mdma_desc_sw {
	u64 src;
	u64 dst;
	u32 len;
	struct list_head node;
	struct list_head tx_list;
	struct dma_async_tx_descriptor async_tx;
	struct mdma_desc_ll *src_v;
	dma_addr_t src_p;
	struct mdma_desc_ll *dst_v;
	dma_addr_t dst_p;
};

struct mdma_chan {
	struct mdma_device *mdev;
	void __iomem *regs;
	spinlock_t lock;
	struct list_head pending_list;
	struct list_head free_list;
	struct list_head active_list;
	struct mdma_desc_sw *sw_desc_pool;
	struct list_head done_list;
	struct dma_chan slave;
	void *desc_pool_v;
	dma_addr_t desc_pool_p;
	u32 desc_free_cnt;
	struct device *dev;
	int irq;
	struct tasklet_struct tasklet;
	bool idle;
	u32 desc_size;
	bool err;
	u32 bus_width;
	u32 src_burst_len;
	u32 dst_burst_len;
};

struct mdma_device {
	/* To protect channel manipulation */
	spinlock_t lock;
	struct dma_device slave;
	struct mdma_chan *chan;
    struct device* dev;
	struct clk *clk;
};

/**
 * mdma_start - Start DMA channel
 * @chan: MDMA channel pointer
 */
static void mdma_start(struct mdma_chan *chan)
{
    // todo start channel
	
    //writel(ZYNQMP_DMA_INT_EN_DEFAULT_MASK, chan->regs + ZYNQMP_DMA_IER);
	//writel(0, chan->regs + ZYNQMP_DMA_TOTAL_BYTE);
	//chan->idle = false;
	//writel(ZYNQMP_DMA_ENABLE, chan->regs + ZYNQMP_DMA_CTRL2);
}


/**
 * mdma_update_desc_to_ctrlr - Updates descriptor to the controller
 * @chan: MDMA DMA channel pointer
 * @desc: Transaction descriptor pointer
 */
static void mdma_update_desc_to_ctrlr(struct mdma_chan *chan,
				      struct mdma_desc_sw *desc)
{
	dma_addr_t addr;

    // todo write start descriptors addresses to controller
	addr = desc->src_p;
//	zynqmp_dma_writeq(chan, ZYNQMP_DMA_SRC_START_LSB, addr);
	addr = desc->dst_p;
//	zynqmp_dma_writeq(chan, ZYNQMP_DMA_DST_START_LSB, addr);
}


static void mdma_config(struct mdma_chan *chan)
{
	//u32 val;

    // todo config channel

	//val = readl(chan->regs + ZYNQMP_DMA_CTRL0);
	//val |= ZYNQMP_DMA_POINT_TYPE_SG;
	//writel(val, chan->regs + ZYNQMP_DMA_CTRL0);

	//val = readl(chan->regs + ZYNQMP_DMA_DATA_ATTR);
	//val = (val & ~ZYNQMP_DMA_ARLEN) |
	//	(chan->src_burst_len << ZYNQMP_DMA_ARLEN_OFST);
	//val = (val & ~ZYNQMP_DMA_AWLEN) |
	//	(chan->dst_burst_len << ZYNQMP_DMA_AWLEN_OFST);
	//writel(val, chan->regs + ZYNQMP_DMA_DATA_ATTR);
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

	mdma_config(chan);

	desc = list_first_entry_or_null(&chan->pending_list,
					struct mdma_desc_sw, node);
	if (!desc)
		return;

	list_splice_tail_init(&chan->pending_list, &chan->active_list);
	mdma_update_desc_to_ctrlr(chan, desc);
	mdma_start(chan);
}

/**
 * mdma_free_descriptor - Issue pending transactions
 * @chan: MDMA channel pointer
 * @sdesc: Transaction descriptor pointer
 */
static void mdma_free_descriptor(struct mdma_chan *chan,
				 struct mdma_desc_sw *sdesc)
{
	struct mdma_desc_sw *child, *next;

	chan->desc_free_cnt++;
	list_add_tail(&sdesc->node, &chan->free_list);
	list_for_each_entry_safe(child, next, &sdesc->tx_list, node) {
		chan->desc_free_cnt++;
		list_move_tail(&child->node, &chan->free_list);
	}
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

	list_for_each_entry_safe(desc, next, list, node)
		mdma_free_descriptor(chan, desc);
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
	struct mdma_desc_sw *desc, *new;
	dma_cookie_t cookie;
	unsigned long irqflags;

	new = tx_to_desc(tx);
	spin_lock_irqsave(&chan->lock, irqflags);
	cookie = dma_cookie_assign(tx);

	if (!list_empty(&chan->pending_list)) {
		desc = list_last_entry(&chan->pending_list,
				     struct mdma_desc_sw, node);
		if (!list_empty(&desc->tx_list))
			desc = list_last_entry(&desc->tx_list,
					       struct mdma_desc_sw, node);
		
        // todo setup 
        
        //desc->src_v->nxtdscraddr = new->src_p;
		//desc->src_v->ctrl &= ~ZYNQMP_DMA_DESC_CTRL_STOP;
		//desc->dst_v->nxtdscraddr = new->dst_p;
		//desc->dst_v->ctrl &= ~ZYNQMP_DMA_DESC_CTRL_STOP;
	}

	list_add_tail(&new->node, &chan->pending_list);
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
	struct mdma_desc_sw *new, *first = NULL;
	void *desc = NULL, *prev = NULL;
	size_t copy;
	u32 desc_cnt;
	unsigned long irqflags;

	chan = to_chan(dchan);

    // to do

    return 0;
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
    // todo terminate transfers
	// writel(ZYNQMP_DMA_IDS_DEFAULT_MASK, chan->regs + ZYNQMP_DMA_IDS);
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

	chan->desc_pool_v = dma_alloc_coherent(chan->dev,
					       (2 * chan->desc_size * MDMA_NUM_DESCS),
					       &chan->desc_pool_p, GFP_KERNEL);
	if (!chan->desc_pool_v)
		return -ENOMEM;

	for (i = 0; i < MDMA_NUM_DESCS; i++) {
		desc = chan->sw_desc_pool + i;
		desc->src_v = (struct mdma_desc_ll *) (chan->desc_pool_v +
					(i * MDMA_DESC_SIZE(chan) * 2));
		desc->dst_v = (struct mdma_desc_ll *) (desc->src_v + 1);
		desc->src_p = chan->desc_pool_p +
				(i * MDMA_DESC_SIZE(chan) * 2);
		desc->dst_p = desc->src_p + MDMA_DESC_SIZE(chan);
	}

	return MDMA_NUM_DESCS;
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
	dma_free_coherent(chan->dev,
		(2 * MDMA_DESC_SIZE(chan) * MDMA_NUM_DESCS),
		chan->desc_pool_v, chan->desc_pool_p);
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

	chan->src_burst_len = config->src_maxburst;
	chan->dst_burst_len = config->dst_maxburst;

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
	u32 isr, imr, status;
	irqreturn_t ret = IRQ_NONE;

    // todo handle int

//	isr = readl(chan->regs + ZYNQMP_DMA_ISR);
//	imr = readl(chan->regs + ZYNQMP_DMA_IMR);
//	status = isr & ~imr;

//	writel(isr, chan->regs + ZYNQMP_DMA_ISR);
//	if (status & ZYNQMP_DMA_INT_DONE) {
//		tasklet_schedule(&chan->tasklet);
//		ret = IRQ_HANDLED;
//	}

//	if (status & ZYNQMP_DMA_DONE)
//		chan->idle = true;

//	if (status & ZYNQMP_DMA_INT_ERR) {
//		chan->err = true;
//		tasklet_schedule(&chan->tasklet);
//		dev_err(chan->dev, "Channel %p has errors\n", chan);
//		ret = IRQ_HANDLED;
//	}

//	if (status & ZYNQMP_DMA_INT_OVRFL) {
//		zynqmp_dma_handle_ovfl_int(chan, status);
//		dev_dbg(chan->dev, "Channel %p overflow interrupt\n", chan);
//		ret = IRQ_HANDLED;
//	}

	return ret;
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
	u32 val;

// todo !!!

//	writel(ZYNQMP_DMA_IDS_DEFAULT_MASK, chan->regs + ZYNQMP_DMA_IDS);
//	val = readl(chan->regs + ZYNQMP_DMA_ISR);
//	writel(val, chan->regs + ZYNQMP_DMA_ISR);

//	val = readl(chan->regs + ZYNQMP_DMA_DATA_ATTR);
//	if (chan->is_dmacoherent) {
//		val = (val & ~ZYNQMP_DMA_ARCACHE) |
//			(ZYNQMP_DMA_AXCACHE_VAL << ZYNQMP_DMA_ARCACHE_OFST);
//		val = (val & ~ZYNQMP_DMA_AWCACHE) |
//			(ZYNQMP_DMA_AXCACHE_VAL << ZYNQMP_DMA_AWCACHE_OFST);
//	}
//	writel(val, chan->regs + ZYNQMP_DMA_DATA_ATTR);

	/* Clearing the interrupt account rgisters */
//	val = readl(chan->regs + ZYNQMP_DMA_IRQ_SRC_ACCT);
//	val = readl(chan->regs + ZYNQMP_DMA_IRQ_DST_ACCT);

	chan->idle = true;
}


/**
 * mdma_reset - Reset the channel
 * @chan: MDMA channel pointer
 */
static void mdma_reset(struct mdma_chan *chan)
{
	//writel(ZYNQMP_DMA_IDS_DEFAULT_MASK, chan->regs + ZYNQMP_DMA_IDS);
    // todo reset channels

	mdma_complete_descriptor(chan);
	mdma_chan_desc_cleanup(chan);
	mdma_free_descriptors(chan);
	mdma_init(chan);
}


/**
 * mdma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the MDMA channel structure
 */
static void mdma_do_tasklet(unsigned long data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	u32 count;
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);

	if (chan->err) {
		mdma_reset(chan);
		chan->err = false;
		goto unlock;
	}

	count = 1; // readl(chan->regs + ZYNQMP_DMA_IRQ_DST_ACCT);

	// todo iterate descriptors
    while (count) {
		mdma_complete_descriptor(chan);
		mdma_chan_desc_cleanup(chan);
		count--;
	}

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
	chan->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chan->regs))
		return PTR_ERR(chan->regs);

	chan->bus_width = MDMA_BUS_WIDTH_128;
	chan->dst_burst_len = MDMA_AWLEN_RST_VAL;
	chan->src_burst_len = MDMA_ARLEN_RST_VAL;
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

	chan->desc_size = sizeof(struct mdma_desc_ll);
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