/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#define DEBUG

#include "dmaengine.h"
#include "rcm-mdma.h"

dma_cookie_t mdma_gp_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct mdma_chan *chan = to_chan(tx->chan);
	struct mdma_device *mdev = chan->mdev;
	dma_cookie_t cookie;

	int num_ch = (chan == mdev->ch[0]) ? 1 : 0;

	pr_debug("%s >>>\n", __func__);

	if (!mdev->ch[num_ch]->prepared_desc) {
		pr_err("%s: Attempt to submit incorrectly prepared "
		       "descriptor.\n", __func__);
		return -EFAULT;
	}

	cookie = mdma_tx_submit(tx);

	mdma_tx_submit(&mdev->ch[num_ch]->prepared_desc->async_tx);

	return cookie;
}

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
struct dma_async_tx_descriptor *
mdma_gp_prep_memcpy(struct dma_chan *dchan, dma_addr_t dma_dst,
                    dma_addr_t dma_src, size_t len, ulong flags)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	struct mdma_desc_sw *sw_desc0 = NULL;
	struct mdma_desc_sw *sw_desc1 = NULL;
	unsigned cnt_descs = 0;
	size_t copy;
	unsigned long irqflags;
	int addr_mask;
	struct mdma_desc_pool pool_save0;
	struct mdma_desc_pool pool_save1;
	unsigned cnt;

	pr_debug("%s >>>\n", __func__);

	addr_mask = mdev->ch[0]->bus_width/8 - 1;
	if((dma_src & addr_mask) || (dma_dst & addr_mask))
	{
		dev_dbg(mdev->ch[0]->dev, "DMA unalligned access %x -> %x\n",
		        dma_src, dma_dst);
		return NULL;
	}

	copy = len;

	do {
		++cnt_descs;
		copy -= min_t(size_t, copy, MDMA_MAX_TRANS_LEN);
	} while (copy);

	spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
	spin_lock(&mdev->ch[1]->lock);

	if ((mdev->ch[0]->prepared_desc) || (mdev->ch[1]->prepared_desc)) {
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
		pr_err("%s: Previous prepared descriptor was not submitted.\n",
		       __func__);
		return NULL;
	}

	sw_desc0 = mdma_get_descriptor(mdev->ch[0]);
	sw_desc1 = mdma_get_descriptor(mdev->ch[1]);

	if ((!sw_desc0) || (!sw_desc1)) {
		if (sw_desc0)
			mdma_free_descriptor(mdev->ch[0], sw_desc0);
		if (sw_desc1)
			mdma_free_descriptor(mdev->ch[1], sw_desc1);
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
		return NULL;
	}

	pool_save0 = mdev->ch[0]->desc_pool;
	pool_save1 = mdev->ch[1]->desc_pool;

	sw_desc0->cnt = mdma_desc_pool_get(&mdev->ch[0]->desc_pool, cnt_descs,
	                                   &sw_desc0->pos);
	sw_desc1->cnt = mdma_desc_pool_get(&mdev->ch[1]->desc_pool, cnt_descs,
	                                   &sw_desc1->pos);

	if ((!sw_desc0->cnt) || (!sw_desc1->cnt)) {
		mdev->ch[0]->desc_pool = pool_save0;
		mdev->ch[1]->desc_pool = pool_save1;
		mdma_free_descriptor(mdev->ch[0], sw_desc0);
		mdma_free_descriptor(mdev->ch[0], sw_desc1);
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
		dev_dbg(mdev->ch[0]->dev, 
		        "chan %p: can't get %u descriptors from pool\n",
		        mdev->ch[0], cnt_descs);
		return NULL;
	}

	mdev->ch[0]->prepared_desc = sw_desc0;
	mdev->ch[1]->prepared_desc = sw_desc1;

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);

	cnt = mdma_desc_pool_fill(&mdev->ch[0]->desc_pool, sw_desc0->pos,
	                          dma_src, len, false);
	if (cnt != sw_desc0->cnt)
		pr_warn("%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc0->cnt);

	cnt = mdma_desc_pool_fill_like(&mdev->ch[1]->desc_pool, sw_desc1->pos,
	                               dma_dst, len, true,
	                               &mdev->ch[0]->desc_pool, sw_desc0->pos);
	if (cnt != sw_desc1->cnt)
		pr_warn("%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc1->cnt);

	mdma_desc_pool_sync(&mdev->ch[0]->desc_pool, sw_desc0->pos,
	                    sw_desc0->cnt);
	mdma_desc_pool_sync(&mdev->ch[1]->desc_pool, sw_desc1->pos,
	                    sw_desc1->cnt);

	async_tx_ack(&sw_desc0->async_tx);
	sw_desc0->async_tx.flags = flags;

	async_tx_ack(&sw_desc1->async_tx);
	sw_desc1->async_tx.flags = flags;

	return &sw_desc0->async_tx;
}

struct dma_async_tx_descriptor *
mdma_gp_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
                      unsigned int sg_len, enum dma_transfer_direction dir,
                      unsigned long flags, void *context)
{
	struct mdma_chan *chan = to_chan(dchan);
	struct mdma_device *mdev = chan->mdev;
	struct mdma_desc_sw *sw_desc0 = NULL;
	struct mdma_desc_sw *sw_desc1 = NULL;
	struct mdma_desc_sw *sw_desc;
	struct mdma_desc_sw *sw_desc_linked;
	struct mdma_chan *chan_linked;
	dma_addr_t dma_addr;
	size_t len;
	unsigned cnt_descs = 0;
	unsigned long irqflags;
	struct mdma_desc_pool pool_save0;
	struct mdma_desc_pool pool_save1;
	unsigned cnt;
	bool stop_int;

	pr_debug("%s >>>\n", __func__);

	if (dir != DMA_MEM_TO_MEM) {
		pr_err("%s: MDMA-GP supports mem-to-mem transfers only.\n",
		       __func__);
		return NULL;
	}

	dma_addr = (chan == mdev->ch[0]) ? chan->config.dst_addr : 
	                                   chan->config.src_addr;

	if (dma_addr == 0) {
		pr_err("%s: DMA-address is not specified. "
		       "Use dmaengine_slave_config() to specify it.\n",
		       __func__);
		return NULL;
	}

	if (!mdma_check_align_sg(chan, sgl))
		return NULL;

	cnt_descs = mdma_cnt_desc_needed(chan, sgl, sg_len, &len);

	spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
	spin_lock(&mdev->ch[1]->lock);

	if ((mdev->ch[0]->prepared_desc) || (mdev->ch[1]->prepared_desc)) {
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
		pr_err("%s: Previous prepared descriptor was not submitted.\n",
		       __func__);
		return NULL;
	}

	pool_save0 = mdev->ch[0]->desc_pool;
	pool_save1 = mdev->ch[1]->desc_pool;

	sw_desc0 = mdma_get_descriptor(mdev->ch[0]);
	sw_desc1 = mdma_get_descriptor(mdev->ch[1]);

	if ((!sw_desc0) || (!sw_desc1))
		goto rollback;

	sw_desc0->cnt = mdma_desc_pool_get(&mdev->ch[0]->desc_pool, cnt_descs,
	                                   &sw_desc0->pos);
	sw_desc1->cnt = mdma_desc_pool_get(&mdev->ch[1]->desc_pool, cnt_descs,
	                                   &sw_desc1->pos);

	if ((!sw_desc0->cnt) || (!sw_desc1->cnt)) {
		dev_dbg(mdev->dev, "can't get %u descriptors from pool\n",
		        cnt_descs);
		goto rollback;
	}

	mdev->ch[0]->prepared_desc = sw_desc0;
	mdev->ch[1]->prepared_desc = sw_desc1;

	chan_linked    = (chan == mdev->ch[0]) ? mdev->ch[1] : mdev->ch[0];

	sw_desc        = (chan == mdev->ch[0]) ? sw_desc0 : sw_desc1;
	sw_desc_linked = (chan == mdev->ch[0]) ? sw_desc1 : sw_desc0;

	stop_int       = (chan == mdev->ch[0]) ? false : true;

	cnt = mdma_desc_pool_fill_sg(&chan->desc_pool, sw_desc->pos,
	                             sgl, sg_len, stop_int);
	if (cnt != sw_desc->cnt) {
		pr_err("%s: Descpitors number does not match (%u != %u)\n",
		       __func__, cnt, sw_desc->cnt);
		goto rollback;
	}

	cnt = mdma_desc_pool_fill_like(&chan_linked->desc_pool,
	                               sw_desc_linked->pos,
	                               dma_addr, len, !stop_int,
	                               &chan->desc_pool, sw_desc->pos);
	if (cnt != sw_desc_linked->cnt) {
		pr_warn("%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc_linked->cnt);
		goto rollback;
	}

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);

	mdma_desc_pool_sync(&mdev->ch[0]->desc_pool, sw_desc0->pos,
	                    sw_desc0->cnt);
	mdma_desc_pool_sync(&mdev->ch[1]->desc_pool, sw_desc1->pos,
	                    sw_desc1->cnt);

	async_tx_ack(&sw_desc0->async_tx);
	sw_desc0->async_tx.flags = flags;

	async_tx_ack(&sw_desc1->async_tx);
	sw_desc1->async_tx.flags = flags;

	return &sw_desc0->async_tx;

rollback:
	mdev->ch[0]->prepared_desc = NULL;
	mdev->ch[1]->prepared_desc = NULL;

	mdev->ch[0]->desc_pool = pool_save0;
	mdev->ch[1]->desc_pool = pool_save1;

	if (sw_desc0)
		mdma_free_descriptor(mdev->ch[0], sw_desc0);
	if (sw_desc1)
		mdma_free_descriptor(mdev->ch[1], sw_desc1);

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);

	return NULL;
}

int mdma_gp_device_terminate_all(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	mdma_device_terminate_all(&mdev->ch[0]->slave);
	mdma_device_terminate_all(&mdev->ch[1]->slave);

	return 0;
}

void mdma_gp_issue_pending(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	unsigned long irqflags;

	pr_debug("%s >>>\n", __func__);

	spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
	spin_lock(&mdev->ch[1]->lock);

	if ((mdma_prepare_transfer(mdev->ch[0])) && 
	    (mdma_prepare_transfer(mdev->ch[1]))) {
		mdma_start_transfer(mdev->ch[0]);
		mdma_start_transfer(mdev->ch[1]);
	}

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
}

int mdma_gp_alloc_chan_resources(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	int ret0;
	int ret1;

	ret0 = mdma_alloc_chan_resources(&mdev->ch[0]->slave);
	ret1 = mdma_alloc_chan_resources(&mdev->ch[1]->slave);

	if ((ret0 != ret1) || (ret0 <= 0)) {
		if (ret0 > 0)
			mdma_free_chan_resources(&mdev->ch[0]->slave);
		if (ret1 > 0)
			mdma_free_chan_resources(&mdev->ch[1]->slave);
		return -EFAULT;
	}

	return ret0;
}

void mdma_gp_free_chan_resources(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	mdma_free_chan_resources(&mdev->ch[0]->slave);
	mdma_free_chan_resources(&mdev->ch[1]->slave);
}

int mdma_gp_device_config(struct dma_chan *dchan,
                          struct dma_slave_config *config)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	mdma_device_config(&mdev->ch[0]->slave, config);
	mdma_device_config(&mdev->ch[1]->slave, config);

	return 0;
}

/**
 * mdma_irq_handler - MDMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the MDMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
irqreturn_t mdma_gp_irq_handler(int irq, void *data)
{
	struct mdma_device *mdev = (struct mdma_device *)data;
	u32 status;
	bool need_tasklet = false;

	status = readl(&mdev->regs->status);

	pr_debug("%s: Interrupt, status %x\n", __func__, status);

	if ((status & (MDMA_IRQ_STATUS_TX | MDMA_IRQ_STATUS_RX)) == 0)
		return IRQ_NONE;

	spin_lock(&mdev->ch[0]->lock);
	spin_lock(&mdev->ch[1]->lock);

	if (status & MDMA_IRQ_STATUS_RX) {
		u32 rx_status = readl(&mdev->ch[0]->regs->status);
		pr_debug("%s: RX-interrupt, status %x\n", __func__, rx_status);

		mdev->ch[0]->err = true;
		need_tasklet = true;
	}

	if (status & MDMA_IRQ_STATUS_TX) {
		u32 tx_status = readl(&mdev->ch[1]->regs->status);
		pr_debug("%s: TX-interrupt, status %x\n", __func__, tx_status);

		if ((tx_status & MDMA_IRQ_INT_DESC) == 0) {
			mdev->ch[1]->err = true;
		} else if (!mdev->ch[1]->active_desc) {
			pr_debug("%s: Interrupt without active descriptor\n",
			         __func__);
		} else {
			mdev->ch[0]->active_desc->completed = true;
			mdev->ch[1]->active_desc->completed = true;
			need_tasklet = true;
		}

	}

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock(&mdev->ch[0]->lock);

	if (need_tasklet)
		tasklet_schedule(&mdev->tasklet);

	return IRQ_HANDLED;
}

/**
 * mdma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the MDMA channel structure
 */
void mdma_gp_do_tasklet(unsigned long data)
{
	struct mdma_device *mdev = (struct mdma_device *)data;
	unsigned long irqflags;
	bool err = false;

	struct dmaengine_desc_callback cb0 = {
		.callback = NULL,
		.callback_param = NULL,
		.callback_result = NULL
	};
	struct dmaengine_desc_callback cb1 = {
		.callback = NULL,
		.callback_param = NULL,
		.callback_result = NULL
	};

	spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
	spin_lock(&mdev->ch[1]->lock);

	if ((mdev->ch[0]->err) || (mdev->ch[1]->err)) {
		mdma_reset(mdev);
		err = true;
	}

	if ( (mdev->ch[0]->active_desc) && 
	     ((err) || (mdev->ch[0]->active_desc->completed)) ) {
		mdma_complete_descriptor(mdev->ch[0], mdev->ch[0]->active_desc, 
		                         !err, &cb0);
		mdma_complete_descriptor(mdev->ch[1], mdev->ch[1]->active_desc, 
		                         !err, &cb1);
		mdev->ch[0]->active_desc = NULL;
		mdev->ch[1]->active_desc = NULL;
	}

	mdev->ch[0]->err = false;
	mdev->ch[1]->err = false;

	if (dmaengine_desc_callback_valid(&cb0)) {
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);

		dmaengine_desc_callback_invoke(&cb0, NULL);

		spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
		spin_lock(&mdev->ch[1]->lock);
	}

	if (dmaengine_desc_callback_valid(&cb1)) {
		spin_unlock(&mdev->ch[1]->lock);
		spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);

		dmaengine_desc_callback_invoke(&cb1, NULL);

		spin_lock_irqsave(&mdev->ch[0]->lock, irqflags);
		spin_lock(&mdev->ch[1]->lock);
	}

	if ((mdma_prepare_transfer(mdev->ch[0])) &&
	    (mdma_prepare_transfer(mdev->ch[1]))) {
		mdma_start_transfer(mdev->ch[0]);
		mdma_start_transfer(mdev->ch[1]);
	}

	spin_unlock(&mdev->ch[1]->lock);
	spin_unlock_irqrestore(&mdev->ch[0]->lock, irqflags);
}
