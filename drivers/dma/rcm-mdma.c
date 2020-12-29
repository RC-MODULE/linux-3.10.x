/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 */
//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <sound/dmaengine_pcm.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include "rcm-mdma.h"

#ifdef CONFIG_BASIS_PLATFORM
#	define NO_HWIRQ	0xffffffff
#endif

bool mdma_check_align(struct mdma_chan *chan, dma_addr_t dma_addr)
{
	int addr_mask = chan->bus_width / 8 - 1;
	bool res = ((dma_addr & addr_mask) == 0);

	if (!res)
		dev_err(chan->dev,
		        "[%s] DMA unalligned access (addr: 0x%llx)\n",
		        chan->name, (u64)dma_addr);

	return res;
}

bool mdma_check_align_sg(struct mdma_chan *chan, struct scatterlist *sg)
{
	dma_addr_t dma_addr = sg_dma_address(sg);
	return mdma_check_align(chan, dma_addr);
}

unsigned mdma_cnt_desc_needed(struct mdma_chan *chan, struct scatterlist *sgl,
                              unsigned int sg_len, size_t *len)
{
	struct scatterlist *s;
	int i;
	unsigned cnt_descs = 0;

	if (len)
		*len = 0;

	for_each_sg(sgl, s, sg_len, i) {
		size_t seg_len  = sg_dma_len(s);

		if (len)
			*len += seg_len;

		while (seg_len > chan->mdev->of_data->max_transaction) {
			++cnt_descs;
			seg_len -= chan->mdev->of_data->max_transaction;
		}

		++cnt_descs;
	}

	return cnt_descs;
}

/**
 * mdma_start - Start DMA channel
 * @chan: MDMA channel pointer
 */
void mdma_start_transfer(struct mdma_chan *chan)
{
	writel(MDMA_IRQ_SENS_MASK, &chan->regs->irq_mask);
	writel(1, &chan->regs->enable);
}

/**
 * mdma_update_desc_to_ctrlr - Updates descriptor to the controller
 * @chan: MDMA DMA channel pointer
 * @desc: Transaction descriptor pointer
 */
static void mdma_update_desc_to_ctrlr(struct mdma_chan *chan,
                                      struct mdma_desc_sw *desc)
{
	dma_addr_t addr = mdma_desc_pool_get_addr(&chan->desc_pool, desc->pos);

	writel(addr, &chan->regs->desc_addr);
}


void mdma_config(struct mdma_chan *chan, struct mdma_desc_sw *desc)
{
	writel(chan->mdev->of_data->ch_settings, &chan->regs->settings);

	writel(desc->config.src_maxburst, &chan->regs->axlen);
}


/**
 * mdma_start_transfer - Initiate the new transfer
 * @chan: MDMA channel pointer
 */
bool mdma_prepare_transfer(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc;

	if (chan->active_desc)
		return false;

	desc = list_first_entry_or_null(&chan->pending_list,
	                                struct mdma_desc_sw, node);
	if (!desc)
		return false;

	list_del(&desc->node);

	desc->completed = false;
	desc->err       = false;
	chan->active_desc = desc;

	mdma_config(chan, desc);

	mdma_update_desc_to_ctrlr(chan, desc);

	return true;
}

/**
 * mdma_get_descriptor - Get the sw descriptor from the pool
 * @chan: MDMA channel pointer
 *
 * Return: The sw descriptor
 */
struct mdma_desc_sw *mdma_get_descriptor(struct mdma_chan *chan)
{
	struct mdma_desc_sw *desc;

	if (!chan->desc_free_cnt) {
		dev_dbg(chan->dev, "[%s] descs are not available\n",
		        chan->name);
		return NULL;
	}

	desc = list_first_entry(&chan->free_list,
				struct mdma_desc_sw, node);
	list_del(&desc->node);
	chan->desc_free_cnt--;

	desc->cnt = 0;

	desc->config = chan->config;

	return desc;
}


/**
 * mdma_free_descriptor - Free descriptor
 * @chan: MDMA channel pointer
 * @sdesc: Transaction descriptor pointer
 */
void mdma_free_descriptor(struct mdma_chan *chan, struct mdma_desc_sw *sdesc)
{
	if (sdesc->cnt)
		mdma_desc_pool_put(&chan->desc_pool, sdesc->pos, sdesc->cnt);
	chan->desc_free_cnt++;
	list_add_tail(&sdesc->node, &chan->free_list);
}

/**
 * mdma_free_desc_list - Free descriptors list
 * @chan: MDMA channel pointer
 * @list: List to parse and delete the descriptor
 */
static void mdma_free_desc_list(struct mdma_chan *chan, struct list_head *list)
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
	if (chan->prepared_desc) {
		mdma_free_descriptor(chan, chan->prepared_desc);
		chan->prepared_desc = NULL;
	}
	if (chan->active_desc) {
		mdma_free_descriptor(chan, chan->active_desc);
		chan->active_desc = NULL;
	}
	mdma_free_desc_list(chan, &chan->done_list);
	mdma_free_desc_list(chan, &chan->pending_list);
}

/**
 * mdma_tx_submit - Submit DMA transaction
 * @tx: Async transaction descriptor pointer
 *
 * Return: cookie value
 */
dma_cookie_t mdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct mdma_chan *chan = to_chan(tx->chan);
	struct mdma_desc_sw *desc;
	dma_cookie_t cookie;
	unsigned long irqflags;

	desc = tx_to_desc(tx);
	spin_lock_irqsave(&chan->lock, irqflags);

	if (desc != chan->prepared_desc) {
		spin_unlock_irqrestore(&chan->lock, irqflags);
		dev_err(chan->dev,
		        "[%s] Attempt to submit unexpected descriptor.\n",
		        chan->name);
		return -EFAULT;
	}

	chan->prepared_desc = NULL;

	cookie = dma_cookie_assign(tx);
	desc->cookie = cookie;

	list_add_tail(&desc->node, &chan->pending_list);
	spin_unlock_irqrestore(&chan->lock, irqflags);

	return cookie;
}

static size_t mdma_get_residue(struct mdma_chan *chan,
                               struct mdma_desc_sw *sw_desc)
{
	struct mdma_desc_pool* pool = &chan->desc_pool;
	size_t len = 0;
	unsigned i = 0;
	unsigned pos;
	struct mdma_desc_long_ll *desc;

	do {
		pos = (sw_desc->pos + i) % pool->size;
		desc = mdma_desc_pool_get_desc(pool, pos);
		len += desc->flags_length & chan->mdev->of_data->len_mask;
		++i;
	} while (((desc->flags_length & MDMA_BD_STOP) == 0) &&
	         (i < sw_desc->cnt));

	if (len > sw_desc->len) {
		dev_warn(chan->dev, "[%s] Transferred length more than "
		                    "requested (%llu > %llu).\n",
		                    chan->name, (u64)len, (u64)sw_desc->len);
		len = sw_desc->len;
	}

	return sw_desc->len - len;
}

static enum dma_status mdma_device_tx_status(struct dma_chan *dchan,
                                             dma_cookie_t cookie,
                                             struct dma_tx_state *txstate)
{
	struct mdma_chan *chan = to_chan(dchan);
	int ret;
	unsigned long irqflags;
	struct mdma_desc_sw *desc;
	bool found = false;

	ret = dma_cookie_status(dchan, cookie, txstate);
	if (ret != DMA_COMPLETE)
		return ret;

	if (!txstate)
		return ret;

	spin_lock_irqsave(&chan->lock, irqflags);

	list_for_each_entry(desc, &chan->done_list, node) {
		if (desc->cookie == cookie) {
			size_t residue = mdma_get_residue(chan, desc);

			dma_set_residue(txstate, residue);

			found = true;

			break;
		}
	}

	if (!found) {
		dev_warn(chan->dev,
		         "Descriptor was not found in DONE-list. Try to call "
		         "dmaengine_tx_status() from callback.\n");
	}

	spin_unlock_irqrestore(&chan->lock, irqflags);

	return DMA_COMPLETE;
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
 * mdma_device_terminate_all - Aborts all transfers on a channel
 * @dchan: DMA channel pointer
 *
 * Return: Always '0'
 */
int mdma_device_terminate_all(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;

	spin_lock_irqsave(&chan->lock, irqflags);
	writel(1, &chan->regs->cancel);
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
	if (mdma_prepare_transfer(chan))
		mdma_start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, irqflags);
}

/**
 * mdma_alloc_chan_resources - Allocate channel resources
 * @dchan: DMA channel
 *
 * Return: Number of descriptors on success and failure value on error
 */
int mdma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	struct mdma_device *mdev = chan->mdev;
	struct mdma_desc_sw *desc;
	int i, ret;

	ret = pm_runtime_get_sync(chan->dev);
	if (ret < 0)
		return ret;

	if (chan->sw_desc_pool) {
		dev_warn(chan->dev, "%s: [%s] Channel already allocated.\n",
		         __func__, chan->name);
		return -EFAULT;
	}

#ifndef CONFIG_BASIS_PLATFORM
	if (chan->irq == -EPROBE_DEFER) {
		chan->irq = platform_get_irq_byname(mdev->pdev, chan->name);
	}
#endif

	if (chan->irq >= 0) {
		tasklet_init(&chan->tasklet,
		             mdev->of_data->tasklet_func, (ulong)chan);

		snprintf(chan->irq_name, sizeof(chan->irq_name), "mdma-%s",
		         chan->name);

		ret = devm_request_irq(chan->dev, chan->irq,
		                       mdev->of_data->irq_handler, 0, 
		                       chan->irq_name, chan);
		if (ret) {
			dev_err(chan->dev,
			        "failed to allocate interrupt for "
			        "\"%s\" channel.\n", chan->name);
			return ret;
		}
	}

	chan->sw_desc_pool = kcalloc(mdev->of_data->sw_desc_pool_size,
	                             sizeof(*desc), GFP_KERNEL);
	if (!chan->sw_desc_pool)
		return -ENOMEM;

	chan->active_desc = NULL;
	chan->prepared_desc = NULL;
	chan->desc_free_cnt = mdev->of_data->sw_desc_pool_size;

	INIT_LIST_HEAD(&chan->free_list);

	for (i = 0; i < mdev->of_data->sw_desc_pool_size; i++) {
		desc = chan->sw_desc_pool + i;
		dma_async_tx_descriptor_init(&desc->async_tx, &chan->slave);
		desc->async_tx.tx_submit = chan->mdev->of_data->tx_submit;
		list_add_tail(&desc->node, &chan->free_list);
	}

	ret = mdma_desc_pool_alloc(&chan->desc_pool,
	                           mdev->of_data->hw_desc_pool_size,
	                           chan->dev, chan->name,
	                           mdev->of_data->max_transaction,
	                           mdev->of_data->len_mask, false);

	if (ret)
		return ret;

	return mdev->of_data->sw_desc_pool_size;
}

/**
 * mdma_free_chan_resources - Free channel resources
 * @dchan: DMA channel pointer
 */
void mdma_free_chan_resources(struct dma_chan *dchan)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;

	if (chan->irq >= 0) {
		devm_free_irq(chan->dev, chan->irq, chan);
		tasklet_kill(&chan->tasklet);
	}

	spin_lock_irqsave(&chan->lock, irqflags);
	mdma_free_descriptors(chan);
	spin_unlock_irqrestore(&chan->lock, irqflags);
	mdma_desc_pool_free(&chan->desc_pool);
	kfree(chan->sw_desc_pool);
	chan->sw_desc_pool = NULL;
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
int mdma_device_config(struct dma_chan *dchan, struct dma_slave_config *config)
{
	struct mdma_chan *chan = to_chan(dchan);

	chan->config = *config;

	return 0;
}

static struct dma_async_tx_descriptor *
mdma_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
                   unsigned int sg_len, enum dma_transfer_direction dir,
                   unsigned long flags, void *context)
{
	struct mdma_chan *chan = to_chan(dchan);
	unsigned long irqflags;
	struct mdma_desc_sw *sw_desc = NULL;
	struct mdma_desc_pool pool_save;
	unsigned cnt_descs = 0;
	unsigned cnt;
	size_t len;

	if (dir != chan->dir) {
		dev_err(chan->dev, "[%s] Unexpected transfer direction.\n",
		        chan->name);
		return NULL;
	}

	if (!mdma_check_align_sg(chan, sgl))
		return NULL;

	cnt_descs = mdma_cnt_desc_needed(chan, sgl, sg_len, &len);

	spin_lock_irqsave(&chan->lock, irqflags);

	if (chan->prepared_desc) {
		spin_unlock_irqrestore(&chan->lock, irqflags);
		dev_err(chan->dev,
		        "[%s] Previous prepared descriptor was not submitted.\n",
		        chan->name);
		return NULL;
	}

	pool_save = chan->desc_pool;

	sw_desc = mdma_get_descriptor(chan);

	if (!sw_desc)
		goto rollback;

	sw_desc->len = len;

	sw_desc->cnt = mdma_desc_pool_get(&chan->desc_pool, cnt_descs,
	                                  &sw_desc->pos);

	if (!sw_desc->cnt) {
		dev_dbg(chan->dev, 
		        "[%s] can't get %u descriptors from pool\n",
		        chan->name, cnt_descs);
		goto rollback;
	}

	chan->prepared_desc = sw_desc;

	cnt = mdma_desc_pool_fill_sg(&chan->desc_pool, sw_desc->pos,
	                             sgl, sg_len, true, true);
	if (cnt > sw_desc->cnt) {
		dev_err(chan->dev, 
		        "[%s] Descpitors number does not match (%u != %u)\n",
		        chan->name, cnt, sw_desc->cnt);
		goto rollback;
	}

	spin_unlock_irqrestore(&chan->lock, irqflags);

	async_tx_ack(&sw_desc->async_tx);
	sw_desc->async_tx.flags = flags;

	return &sw_desc->async_tx;

rollback:
	chan->prepared_desc = NULL;

	chan->desc_pool = pool_save;

	if (sw_desc)
		mdma_free_descriptor(chan, sw_desc);

	spin_unlock_irqrestore(&chan->lock, irqflags);

	return NULL;
}

/**
 * mdma_complete_descriptor - Mark the active descriptor as complete
 * @chan: MDMA channel pointer
 */
void mdma_complete_descriptor(struct mdma_chan *chan, 
                              struct mdma_desc_sw *desc, bool status,
                              struct dmaengine_desc_callback* cb)
{
	if (status)
		dma_cookie_complete(&desc->async_tx);

	dmaengine_desc_get_callback(&desc->async_tx, cb);
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

	if (status == 0)
		return IRQ_NONE;

	spin_lock(&chan->lock);

	if (!chan->active_desc) {
		dev_info(chan->dev, "[%s] Interrupt without active descriptor\n",
		         chan->name);
	} else {
//		dev_info(chan->dev, "[%s] Interrupt status = 0x%08X\n",
//		        chan->name, status);

		if ((status & MDMA_IRQ_INT_DESC) == 0)
			chan->active_desc->err = true;
		else 
			chan->active_desc->completed = true;
		need_tasklet = true;

		list_add_tail(&chan->active_desc->node, &chan->done_list);
		chan->active_desc = NULL;
	}

	if (mdma_prepare_transfer(chan))
		mdma_start_transfer(chan);
	else if (chan->dir == DMA_DEV_TO_MEM)
		dev_dbg(chan->dev, "[%s] IRQ: no pending descriptors.",
		        chan->name);

	spin_unlock(&chan->lock);

	if (need_tasklet)
		tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

/**
 * mdma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the MDMA channel structure
 */
static void mdma_do_tasklet(unsigned long data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	unsigned long irqflags;
	struct mdma_desc_sw *desc;

	struct dmaengine_desc_callback cb = {
		.callback = NULL,
		.callback_param = NULL,
		.callback_result = NULL
	};

	spin_lock_irqsave(&chan->lock, irqflags);

	while (!list_empty(&chan->done_list)) {
		desc = list_first_entry(&chan->done_list,
		                        struct mdma_desc_sw, node);

		if (desc->err) {
//			mdma_reset(chan->mdev);
		}

		mdma_complete_descriptor(chan, desc, !desc->err, &cb);

		if (dmaengine_desc_callback_valid(&cb)) {
			spin_unlock_irqrestore(&chan->lock, irqflags);

			dmaengine_desc_callback_invoke(&cb, NULL);

			spin_lock_irqsave(&chan->lock, irqflags);
		}

		list_del(&desc->node);
		mdma_free_descriptor(chan, desc);
	}

	if (mdma_prepare_transfer(chan))
		mdma_start_transfer(chan);

	spin_unlock_irqrestore(&chan->lock, irqflags);
}

/**
 * mdma_chan_remove - Channel remove function
 * @chan: MDMA channel pointer
 */
static void mdma_chan_remove(struct mdma_chan *chan)
{
	if ((!chan) || (!chan->regs))
		return;
	if (chan->slave.device) {
		list_del(&chan->slave.device_node);
		chan->slave.device = NULL;
	}
}

#ifdef CONFIG_BASIS_PLATFORM

static struct irq_domain *mdma_get_intc(struct device *dev, char* name)
{
	struct basis_device *device;

	device = basis_device_find(name);
	if (!device) {
		dev_err(dev, "Failed to found BASIS-device \"%s\"\n", name);
		return NULL;
	}

	if (!device->priv) {
		dev_err(dev,
		        "BASIS-device \"%s\" is not interrupt controller\n",
		        name);
		return NULL;
	}

	return device->priv;
}

static int mdma_chan_probe(struct mdma_chan *chan, int ch_num)
{
	struct mdma_device *mdev = chan->mdev;
	struct irq_domain *domain = mdma_get_intc(mdev->dev, mdev->intc);
	u32 hwirq = (chan->dir == DMA_MEM_TO_DEV) ? mdev->rx_hwirq[ch_num] : 
	                                            mdev->tx_hwirq[ch_num];

	if (!domain)
		return -ENODEV;

	chan->config.src_addr = 0;
	chan->config.dst_addr = 0;

	chan->bus_width = MDMA_BUS_WIDTH_128;

	if (hwirq == NO_HWIRQ) {
		dev_dbg(mdev->dev,
		        "HWIRQ for \"%s\" channel is not set.\n",
		        chan->name);
	} else {
		chan->irq = irq_create_mapping(domain, hwirq);
		if (chan->irq < 0) {
			dev_err(mdev->dev, "Failed to map irq #%u (%s).\n",
		        	mdev->hwirq, chan->name);
			return -ENXIO;
		}
	}

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->free_list);
	INIT_LIST_HEAD(&chan->done_list);

	chan->active_desc = NULL;

	dma_cookie_init(&chan->slave);
	chan->slave.device = &chan->mdev->slave;
	list_add_tail(&chan->slave.device_node, &chan->mdev->slave.channels);

	chan->desc_size = sizeof(struct mdma_desc_long_ll);  // max size
	return 0;
}

static int mdma_init_channels(struct mdma_device *mdev)
{
	struct basis_controller *controller = mdev->device->controller;
	int ret = 0;
	int i;
	bool have_tx = false;
	bool have_rx = false;

	for (i = 0; i < 2 * MDMA_MAX_CHANNELS; ++i) {
		char ch_name[8];
		struct mdma_chan* chan =  ((i % 2) == 0) ? &mdev->rx[i / 2] : 
		                                           &mdev->tx[i / 2];
		u32 reg;
		u32 reg_size;

		if ((i % 2) == 0) {
			snprintf(ch_name, sizeof(ch_name), "rx%d", i / 2);
			reg = mdev->reg_rx[i / 2];
			reg_size = mdev->reg_rx_size;
		} else {
			snprintf(ch_name, sizeof(ch_name), "tx%d", i / 2);
			reg = mdev->reg_tx[i / 2];
			reg_size = mdev->reg_tx_size;
		}

		if ((reg == 0) || (reg_size == 0))
			continue;

		chan->regs = devm_ioremap(mdev->dev,
		                          reg + controller->ep_base_phys,
		                          reg_size);
		if (IS_ERR(chan->regs)) {
			chan->regs = NULL;
			continue;
		}

		chan->dev = mdev->dev;
		chan->mdev = mdev;
		chan->dir = ((i % 2) == 0) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
		memcpy(chan->name, ch_name, sizeof(ch_name));

		ret = mdma_chan_probe(chan, i / 2);
		if (ret) {
			dev_err(mdev->dev, "Probing channel %s failed\n",
			        ch_name);
			break;
		}

		if ((i % 2) == 0)
			have_rx = true;
		else
			have_tx = true;
	}

	if (ret == 0) {
		if (!have_tx) {
			dev_err(mdev->dev, "No TX channel specified.\n");
			ret = -ENODEV;
		}

		if (!have_rx) {
			dev_err(mdev->dev, "No RX channel specified.\n");
			ret = -ENODEV;
		}
	}

	return ret;
}

static int mdma_init_interrupts(struct mdma_device *mdev)
{
	struct irq_domain *domain = mdma_get_intc(mdev->dev, mdev->intc);
	if (!domain)
		return -ENODEV;

	if (mdev->hwirq != NO_HWIRQ) {
		mdev->rx[0].irq = irq_create_mapping(domain, mdev->hwirq);
		if (mdev->rx[0].irq < 0) {
			dev_err(mdev->dev,
			        "Failed to map irq #%u.\n", mdev->hwirq);
			return -ENXIO;
		}
	}

	return 0;
}

static int mdma_bind(struct basis_device *device)
{
	struct mdma_device *mdev = basis_device_get_drvdata(device);
	const struct mdma_of_data *of_data = mdev->of_data;
	struct device *dev = &device->dev;
	struct dma_device *p;
	int ret;
	int i;

	if (WARN_ON_ONCE(!device))
		return -EINVAL;

	dev_info(dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	if (mdev->reg_cfg) {
		mdev->cfg = devm_ioremap(dev,
		                         mdev->reg_cfg +
		                         device->controller->ep_base_phys,
		                         mdev->reg_cfg_size);
		if (IS_ERR(mdev->cfg))
			mdev->cfg = NULL;
	}

	mdev->dev = dev;
	INIT_LIST_HEAD(&mdev->slave.channels);

	dma_set_mask(dev, DMA_BIT_MASK(32));
	if (of_data->device_prep_dma_memcpy)
		dma_cap_set(DMA_MEMCPY, mdev->slave.cap_mask);
	if (of_data->device_prep_slave_sg)
		dma_cap_set(DMA_SLAVE, mdev->slave.cap_mask);

	p = &mdev->slave;
	p->device_prep_dma_memcpy      = of_data->device_prep_dma_memcpy;
	// p->device_prep_interleaved_dma = mdma_prep_interleaved_dma; // todo
	p->device_prep_slave_sg        = of_data->device_prep_slave_sg;
	// p->device_prep_dma_memset = mdma_prep_dma_memset; // todo
	// p->device_prep_dma_memset_sg = mdma_prep_dma_memset_sg; // todo
	p->device_terminate_all        = of_data->device_terminate_all;
	p->device_issue_pending        = of_data->device_issue_pending;
	p->device_alloc_chan_resources = of_data->device_alloc_chan_resources;
	p->device_free_chan_resources  = of_data->device_free_chan_resources;
	p->device_tx_status            = of_data->device_tx_status;
	p->device_config               = of_data->device_config;
	p->dev = dev;

	pm_runtime_set_autosuspend_delay(mdev->dev, MDMA_PM_TIMEOUT);
	pm_runtime_use_autosuspend(mdev->dev);
	pm_runtime_enable(mdev->dev);
	pm_runtime_get_sync(mdev->dev);
	if (!pm_runtime_enabled(mdev->dev)) {
		ret = mdma_runtime_resume(mdev->dev);
		if (ret)
			return ret;
	}

	ret = mdma_init_channels(mdev);
	if (ret) 
		goto free_chan_resources;

	ret = mdma_init_interrupts(mdev);
	if (ret)
		goto free_chan_resources;

	p->dst_addr_widths = BIT(mdev->rx[0].bus_width / 8);
	p->src_addr_widths = BIT(mdev->rx[0].bus_width / 8);

	dma_async_device_register(&mdev->slave);

	device->priv = &mdev->slave;

	pm_runtime_mark_last_busy(mdev->dev);
	pm_runtime_put_sync_autosuspend(mdev->dev);

	dev_info(dev, "MDMA driver Probe success\n");

	return 0;

free_chan_resources:
	for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
		mdma_chan_remove(&mdev->rx[i]);
		mdma_chan_remove(&mdev->tx[i]);
	}

	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);
	pm_runtime_disable(mdev->dev);
	return ret;
}

static void mdma_unbind(struct basis_device *device)
{
	struct mdma_device *mdev = basis_device_get_drvdata(device);
	int i;

	dma_async_device_unregister(&mdev->slave);

	for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
		mdma_chan_remove(&mdev->rx[i]);
		mdma_chan_remove(&mdev->tx[i]);
	}

	pm_runtime_disable(mdev->dev);
	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);
}

static const struct basis_device_id mdma_ids[];

static int mdma_probe(struct basis_device *device)
{
	struct mdma_device *mdev;
	struct device *dev = &device->dev;
	const struct basis_device_id *match;
	int i;

	match = basis_device_match_device(mdma_ids, device);
	if (!match)
		return -EINVAL;

	mdev = devm_kzalloc(dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->device = device;
	mdev->of_data = (struct mdma_of_data *)match->driver_data;

	mdev->hwirq = NO_HWIRQ;

	for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
		mdev->rx_hwirq[i] = NO_HWIRQ;
		mdev->tx_hwirq[i] = NO_HWIRQ;
	}

	basis_device_set_drvdata(device, mdev);

	return 0;
}

extern const struct mdma_of_data mdma_gp_of_data;
const struct mdma_of_data mdma_muart_of_data;

static const struct basis_device_id mdma_ids[] = {
	{
		.name = "rcm-mdma-gp",
		.driver_data = (kernel_ulong_t)&mdma_gp_of_data
	},
	{
		.name = "rcm-mdma-muart",
		.driver_data = (kernel_ulong_t)&mdma_muart_of_data
	},
	{},
};

static struct basis_device_ops mdma_ops = {
	.unbind = mdma_unbind,
	.bind   = mdma_bind,
};

BASIS_DEV_ATTR_U32_SHOW(mdma_,  hwirq,         struct mdma_device);
BASIS_DEV_ATTR_U32_STORE(mdma_, hwirq,         struct mdma_device);

BASIS_DEV_ATTR_U32_SHOW(mdma_,  reg_cfg,       struct mdma_device);
BASIS_DEV_ATTR_U32_STORE(mdma_, reg_cfg,       struct mdma_device);

BASIS_DEV_ATTR_U32_SHOW(mdma_,  reg_cfg_size,  struct mdma_device);
BASIS_DEV_ATTR_U32_STORE(mdma_, reg_cfg_size,  struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_rx, 0, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_rx, 0, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_rx, 1, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_rx, 1, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_rx, 2, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_rx, 2, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_rx, 3, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_rx, 3, struct mdma_device);

BASIS_DEV_ATTR_U32_SHOW(mdma_,  reg_rx_size,   struct mdma_device);
BASIS_DEV_ATTR_U32_STORE(mdma_, reg_rx_size,   struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_tx, 0, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_tx, 0, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_tx, 1, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_tx, 1, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_tx, 2, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_tx, 2, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  reg_tx, 3, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, reg_tx, 3, struct mdma_device);

BASIS_DEV_ATTR_U32_SHOW(mdma_,  reg_tx_size,   struct mdma_device);
BASIS_DEV_ATTR_U32_STORE(mdma_, reg_tx_size,   struct mdma_device);

BASIS_DEV_ATTR_STR_SHOW(mdma_,  intc,          struct mdma_device);
BASIS_DEV_ATTR_STR_STORE(mdma_, intc,          struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  rx_hwirq, 0, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, rx_hwirq, 0, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  rx_hwirq, 1, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, rx_hwirq, 1, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  rx_hwirq, 2, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, rx_hwirq, 2, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  rx_hwirq, 3, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, rx_hwirq, 3, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  tx_hwirq, 0, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, tx_hwirq, 0, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  tx_hwirq, 1, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, tx_hwirq, 1, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  tx_hwirq, 2, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, tx_hwirq, 2, struct mdma_device);

BASIS_DEV_ATTR_ARR_U32_SHOW(mdma_,  tx_hwirq, 3, struct mdma_device);
BASIS_DEV_ATTR_ARR_U32_STORE(mdma_, tx_hwirq, 3, struct mdma_device);

CONFIGFS_ATTR(mdma_, hwirq);
CONFIGFS_ATTR(mdma_, reg_cfg);
CONFIGFS_ATTR(mdma_, reg_cfg_size);

CONFIGFS_ATTR(mdma_, reg_rx_0);
CONFIGFS_ATTR(mdma_, reg_rx_1);
CONFIGFS_ATTR(mdma_, reg_rx_2);
CONFIGFS_ATTR(mdma_, reg_rx_3);
CONFIGFS_ATTR(mdma_, reg_rx_size);
CONFIGFS_ATTR(mdma_, reg_tx_0);
CONFIGFS_ATTR(mdma_, reg_tx_1);
CONFIGFS_ATTR(mdma_, reg_tx_2);
CONFIGFS_ATTR(mdma_, reg_tx_3);
CONFIGFS_ATTR(mdma_, reg_tx_size);

CONFIGFS_ATTR(mdma_, intc);

CONFIGFS_ATTR(mdma_, rx_hwirq_0);
CONFIGFS_ATTR(mdma_, rx_hwirq_1);
CONFIGFS_ATTR(mdma_, rx_hwirq_2);
CONFIGFS_ATTR(mdma_, rx_hwirq_3);
CONFIGFS_ATTR(mdma_, tx_hwirq_0);
CONFIGFS_ATTR(mdma_, tx_hwirq_1);
CONFIGFS_ATTR(mdma_, tx_hwirq_2);
CONFIGFS_ATTR(mdma_, tx_hwirq_3);

static struct configfs_attribute *mdma_attrs[] = {
	&mdma_attr_hwirq,
	&mdma_attr_reg_cfg,
	&mdma_attr_reg_cfg_size,
	&mdma_attr_reg_rx_0,
	&mdma_attr_reg_rx_1,
	&mdma_attr_reg_rx_2,
	&mdma_attr_reg_rx_3,
	&mdma_attr_reg_rx_size,
	&mdma_attr_reg_tx_0,
	&mdma_attr_reg_tx_1,
	&mdma_attr_reg_tx_2,
	&mdma_attr_reg_tx_3,
	&mdma_attr_reg_tx_size,
	&mdma_attr_intc,
	&mdma_attr_rx_hwirq_0,
	&mdma_attr_rx_hwirq_1,
	&mdma_attr_rx_hwirq_2,
	&mdma_attr_rx_hwirq_3,
	&mdma_attr_tx_hwirq_0,
	&mdma_attr_tx_hwirq_1,
	&mdma_attr_tx_hwirq_2,
	&mdma_attr_tx_hwirq_3,
	NULL,
};

static struct basis_device_driver rcm_mdma_driver = {
	.driver = {
		.name = "rcm-mdma",
		.pm = &mdma_dev_pm_ops,
	},
	.probe          = mdma_probe,
	.id_table       = mdma_ids,
	.ops            = &mdma_ops,
	.owner          = THIS_MODULE,
	.attrs          = mdma_attrs,
};
module_basis_driver(rcm_mdma_driver);

#else // CONFIG_BASIS_PLATFORM

/**
 * mdma_chan_probe - Per Channel Probing
 * @mdev: Driver specific device structure
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int mdma_chan_probe(struct platform_device *pdev, struct mdma_chan *chan)
{
	struct device_node *node = chan->dev->of_node;
	int err;

	chan->config.src_addr = 0;
	chan->config.dst_addr = 0;

	chan->bus_width = MDMA_BUS_WIDTH_128;
	chan->config.src_maxburst = MDMA_AWLEN_RST_VAL;
	chan->config.dst_maxburst = MDMA_ARLEN_RST_VAL;
	err = of_property_read_u32(node, "rcm,bus-width", &chan->bus_width);
	if (err < 0) {
		dev_err(chan->dev, "missing rcm,bus-width property\n");
		return err;
	}

	if (chan->bus_width != MDMA_BUS_WIDTH_128) {
		dev_err(chan->dev, "invalid bus-width value");
		return -EINVAL;
	}

	chan->irq = platform_get_irq_byname(pdev, chan->name);
	if (chan->irq < 0) {
		dev_dbg(&pdev->dev,
		        "unable to get interrupt for \"%s\" channel.\n",
		        chan->name);
	}

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->free_list);
	INIT_LIST_HEAD(&chan->done_list);

	chan->active_desc = NULL;

	dma_cookie_init(&chan->slave);
	chan->slave.device = &chan->mdev->slave;
	list_add_tail(&chan->slave.device_node, &chan->mdev->slave.channels);

	chan->desc_size = sizeof(struct mdma_desc_long_ll);  // max size
	return 0;
}

static int mdma_init_channels(struct platform_device *pdev,
                              struct mdma_device *mdev)
{
	int ret = 0;
	int i;
	struct resource *res;
	bool have_tx = false;
	bool have_rx = false;

	for (i = 0; i < 2 * MDMA_MAX_CHANNELS; ++i) {
		char ch_name[8];
		struct mdma_chan* chan =  ((i % 2) == 0) ? &mdev->rx[i / 2] : 
		                                           &mdev->tx[i / 2];

		if ((i % 2) == 0)
			snprintf(ch_name, sizeof(ch_name), "rx%d", i / 2);
		else
			snprintf(ch_name, sizeof(ch_name), "tx%d", i / 2);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		                                   ch_name);
		if (!res)
			continue;
		chan->regs = (struct channel_regs __iomem *)
		             devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(chan->regs)) {
			chan->regs = NULL;
			continue;
		}

		chan->dev = mdev->dev;
		chan->mdev = mdev;
		chan->dir = ((i % 2) == 0) ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM;
		memcpy(chan->name, ch_name, sizeof(ch_name));

		ret = mdma_chan_probe(pdev, chan);
		if (ret) {
			dev_err(&pdev->dev, "Probing channel %s failed\n",
			        ch_name);
			break;
		}

		if ((i % 2) == 0)
			have_rx = true;
		else
			have_tx = true;
	}

	if (ret == 0) {
		if (!have_tx) {
			dev_err(&pdev->dev, "No TX channel specified.\n");
			ret = -ENODEV;
		}

		if (!have_rx) {
			dev_err(&pdev->dev, "No RX channel specified.\n");
			ret = -ENODEV;
		}
	}

	return ret;
}

static int mdma_init_interrupts(struct platform_device *pdev,
                                struct mdma_device *mdev)
{
	int cnt_ints =  platform_irq_count(pdev);
	int i;

	if (cnt_ints == 0) {
		dev_err(&pdev->dev, "No interrupts found.\n");
		return -ENXIO;
	}

	if (cnt_ints == 1) {
		for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
			if ((mdev->rx[i].regs) && (mdev->rx[i].irq >= 0))
				return 0;
			if ((mdev->tx[i].regs) && (mdev->tx[i].irq >= 0))
				return 0;
		}

		mdev->rx[0].irq = platform_get_irq(pdev, 0);
		if (mdev->rx[0].irq < 0) {
			dev_err(&pdev->dev,
			        "unable to get interrupt property.\n");
			return -ENXIO;
		}
	}

	return 0;
}

static const struct of_device_id rcm_mdma_dt_ids[];

static int rcm_mdma_probe(struct platform_device *pdev)
{
	struct mdma_device *mdev;
	const struct of_device_id *match;
	const struct mdma_of_data *of_data;
	struct dma_device *p;
	struct resource *res;
	int ret;
	int i;

	match = of_match_device(rcm_mdma_dt_ids, &pdev->dev);
	if (!match)
		return -EINVAL;

	of_data = (struct mdma_of_data *)match->data;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->of_data = of_data;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (res) {
		mdev->cfg = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(mdev->cfg))
			mdev->cfg = NULL;
	}

	mdev->dev = &pdev->dev;
	mdev->pdev = pdev;
	INIT_LIST_HEAD(&mdev->slave.channels);

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (of_data->device_prep_dma_memcpy)
		dma_cap_set(DMA_MEMCPY, mdev->slave.cap_mask);
	if (of_data->device_prep_slave_sg)
		dma_cap_set(DMA_SLAVE, mdev->slave.cap_mask);

	p = &mdev->slave;
	p->device_prep_dma_memcpy      = of_data->device_prep_dma_memcpy;
	// p->device_prep_interleaved_dma = mdma_prep_interleaved_dma; // todo
	p->device_prep_slave_sg        = of_data->device_prep_slave_sg;
	// p->device_prep_dma_memset = mdma_prep_dma_memset; // todo
	// p->device_prep_dma_memset_sg = mdma_prep_dma_memset_sg; // todo
	p->device_terminate_all        = of_data->device_terminate_all;
	p->device_issue_pending        = of_data->device_issue_pending;
	p->device_alloc_chan_resources = of_data->device_alloc_chan_resources;
	p->device_free_chan_resources  = of_data->device_free_chan_resources;
	p->device_tx_status            = of_data->device_tx_status;
	p->device_config               = of_data->device_config;
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

	ret = mdma_init_channels(pdev, mdev);
	if (ret) 
		goto free_chan_resources;

	ret = mdma_init_interrupts(pdev, mdev);
	if (ret)
		goto free_chan_resources;

	p->dst_addr_widths = BIT(mdev->rx[0].bus_width / 8);
	p->src_addr_widths = BIT(mdev->rx[0].bus_width / 8);

	dma_async_device_register(&mdev->slave);

	ret = of_dma_controller_register(pdev->dev.of_node,
	                                 of_dma_xlate_by_chan_id, &mdev->slave);
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
	for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
		mdma_chan_remove(&mdev->rx[i]);
		mdma_chan_remove(&mdev->tx[i]);
	}

	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);
	pm_runtime_disable(mdev->dev);
	return ret;
}

static int rcm_mdma_remove(struct platform_device *pdev)
{
	struct mdma_device *mdev = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(mdev->dev->of_node);
	dma_async_device_unregister(&mdev->slave);

	for (i = 0; i < MDMA_MAX_CHANNELS; ++i) {
		mdma_chan_remove(&mdev->rx[i]);
		mdma_chan_remove(&mdev->tx[i]);
	}

	pm_runtime_disable(mdev->dev);
	if (!pm_runtime_enabled(mdev->dev))
		mdma_runtime_suspend(mdev->dev);

	return 0;
}

#endif // CONFIG_BASIS_PLATFORM

extern const struct mdma_of_data mdma_gp_of_data;

const struct mdma_of_data mdma_mgeth_of_data = {
	.max_transaction             = 0x3FFC,
	.len_mask                    = 0x3FFF,
	.sw_desc_pool_size           = 64,
	.hw_desc_pool_size           = MDMA_POOL_CHUNK_SIZE / 
	                               sizeof(struct mdma_desc_long_ll),
	.ch_settings                 = 
		MDMA_CHAN_DESC_LONG | MDMA_CHAN_ADD_INFO | 
		(sizeof(struct mdma_desc_long_ll) << MDMA_CHAN_DESC_GAP_SHIFT),

	.device_alloc_chan_resources = mdma_alloc_chan_resources,
	.device_free_chan_resources  = mdma_free_chan_resources,
	.device_prep_slave_sg        = mdma_prep_slave_sg,
	.device_config               = mdma_device_config,
	.device_terminate_all        = mdma_device_terminate_all,
	.device_tx_status            = mdma_device_tx_status,
	.device_issue_pending        = mdma_issue_pending,

	.tx_submit                   = mdma_tx_submit,

	.irq_handler                 = mdma_irq_handler,
	.tasklet_func                = mdma_do_tasklet,
};

const struct mdma_of_data mdma_muart_of_data = {
	.max_transaction             = 0x3FFC,
	.len_mask                    = 0x3FFF,
	.sw_desc_pool_size           = 16,
	.hw_desc_pool_size           = 16,
	.ch_settings                 = 
		MDMA_CHAN_DESC_LONG | MDMA_CHAN_ADD_INFO | 
		(sizeof(struct mdma_desc_long_ll) << MDMA_CHAN_DESC_GAP_SHIFT),

	.device_alloc_chan_resources = mdma_alloc_chan_resources,
	.device_free_chan_resources  = mdma_free_chan_resources,
	.device_prep_slave_sg        = mdma_prep_slave_sg,
	.device_config               = mdma_device_config,
	.device_terminate_all        = mdma_device_terminate_all,
	.device_tx_status            = mdma_device_tx_status,
	.device_issue_pending        = mdma_issue_pending,

	.tx_submit                   = mdma_tx_submit,

	.irq_handler                 = mdma_irq_handler,
	.tasklet_func                = mdma_do_tasklet,
};

#ifndef CONFIG_BASIS_PLATFORM

static const struct of_device_id rcm_mdma_dt_ids[] = {
	{
		.compatible = "rcm,mdma-gp",
		.data = &mdma_gp_of_data,
	},
	{
		.compatible = "rcm,mdma-mgeth",
		.data = &mdma_mgeth_of_data,
	},
	{
		.compatible = "rcm,mdma-muart",
		.data = &mdma_muart_of_data,
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

#endif // CONFIG_BASIS_PLATFORM

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RC-Module MDMA driver");
MODULE_LICENSE("GPL");
