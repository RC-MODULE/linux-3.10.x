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

#define MDMA_GP_MAX_TRANS_LEN 0x7FFFFFC
#define MDMA_GP_LEN_MASK      0x7FFFFFF

struct mdma_gp_cfg
{
    roreg32 id;                             /* 0x000 - device id            */
    roreg32 version;                        /* 0x004 - device version       */
    rwreg32 soft_reset;                     /* 0x008 - soft reset           */
    roreg32 _skip01;                        /* 0x00C                        */
    rwreg32 event_sence_channel;            /* 0x010 - 0-read channel, 1-w  */
    roreg32 _skip02;                        /* 0x014                        */
    rwreg32 status;                         /* 0x018 - DMA status           */
    roreg32 _skip03[57];                    /* 0x01C - 0x0ff                */
} __attribute__ ((packed));

/**
 * mdma_reset - Reset the channel
 * @chan: MDMA channel pointer
 */
static void mdma_gp_reset(struct mdma_device *mdev)
{
	struct mdma_gp_cfg __iomem *cfg = mdev->cfg;

	writel(1, &cfg->soft_reset);
}

static dma_cookie_t mdma_gp_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct mdma_chan *chan = to_chan(tx->chan);
	struct mdma_device *mdev = chan->mdev;
	struct mdma_chan *chan1 = (chan == &mdev->rx[0]) ? 
	                          &mdev->tx[0] : &mdev->rx[0];
	dma_cookie_t cookie;

	if (!chan1->prepared_desc) {
		dev_err(chan->dev,
		        "[%s] Attempt to submit incorrectly prepared "
		       "descriptor.\n", chan1->name);
		return -EFAULT;
	}

	cookie = mdma_tx_submit(tx);

	mdma_tx_submit(&chan1->prepared_desc->async_tx);

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
static struct dma_async_tx_descriptor *
mdma_gp_prep_memcpy(struct dma_chan *dchan, dma_addr_t dma_dst,
                    dma_addr_t dma_src, size_t len, ulong flags)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	struct mdma_desc_sw *sw_desc_rx = NULL;
	struct mdma_desc_sw *sw_desc_tx = NULL;
	unsigned cnt_descs = 0;
	size_t copy;
	unsigned long irqflags;
	struct mdma_desc_pool pool_save_rx;
	struct mdma_desc_pool pool_save_tx;
	unsigned cnt;

	if ((!mdma_check_align(&mdev->rx[0], dma_src)) ||
	    (!mdma_check_align(&mdev->rx[0], dma_dst)))
		return NULL;

	copy = len;

	do {
		++cnt_descs;
		copy -= min_t(size_t, copy, mdev->rx[0].max_transaction);
	} while (copy);

	spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
	spin_lock(&mdev->tx[0].lock);

	if ((mdev->rx[0].prepared_desc) || (mdev->tx[0].prepared_desc)) {
		spin_unlock(&mdev->tx[0].lock);
		spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);
		dev_err(mdev->dev,
		        "%s: Previous prepared descriptor was not submitted.\n",
		        __func__);
		return NULL;
	}

	pool_save_rx = mdev->rx[0].desc_pool;
	pool_save_tx = mdev->tx[0].desc_pool;

	sw_desc_rx = mdma_get_descriptor(&mdev->rx[0]);
	sw_desc_tx = mdma_get_descriptor(&mdev->tx[0]);

	if ((!sw_desc_rx) || (!sw_desc_tx))
		goto rollback;

	sw_desc_rx->len = len;
	sw_desc_tx->len = len;

	sw_desc_rx->cnt = mdma_desc_pool_get(&mdev->rx[0].desc_pool, cnt_descs,
	                                     &sw_desc_rx->pos);
	sw_desc_tx->cnt = mdma_desc_pool_get(&mdev->tx[0].desc_pool, cnt_descs,
	                                     &sw_desc_tx->pos);

	if ((!sw_desc_rx->cnt) || (!sw_desc_tx->cnt)) {
		dev_dbg(mdev->dev, 
		        "%s: can't get %u descriptors from pool\n",
		        __func__, cnt_descs);
		goto rollback;
	}

	mdev->rx[0].prepared_desc = sw_desc_rx;
	mdev->tx[0].prepared_desc = sw_desc_tx;

	cnt = mdma_desc_pool_fill(&mdev->rx[0].desc_pool, sw_desc_rx->pos,
	                          dma_src, len, false);
	if (cnt > sw_desc_rx->cnt) {
		dev_err(mdev->dev,
		        "%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc_rx->cnt);
		goto rollback;
	}

	cnt = mdma_desc_pool_fill_like(&mdev->tx[0].desc_pool, sw_desc_tx->pos,
	                               dma_dst, len, true,
	                               &mdev->rx[0].desc_pool, sw_desc_rx->pos);
	if (cnt > sw_desc_tx->cnt) {
		dev_err(mdev->dev,
		        "%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc_tx->cnt);
		goto rollback;
	}

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

	mdma_desc_pool_sync(&mdev->rx[0].desc_pool, sw_desc_rx->pos,
	                    sw_desc_rx->cnt, true);
	mdma_desc_pool_sync(&mdev->tx[0].desc_pool, sw_desc_tx->pos,
	                    sw_desc_tx->cnt, true);

	async_tx_ack(&sw_desc_rx->async_tx);
	sw_desc_rx->async_tx.flags = flags;

	async_tx_ack(&sw_desc_tx->async_tx);
	sw_desc_tx->async_tx.flags = flags;

	return &sw_desc_rx->async_tx;

rollback:
	mdev->rx[0].prepared_desc = NULL;
	mdev->tx[0].prepared_desc = NULL;

	mdev->rx[0].desc_pool = pool_save_rx;
	mdev->tx[0].desc_pool = pool_save_tx;

	if (sw_desc_rx)
		mdma_free_descriptor(&mdev->rx[0], sw_desc_rx);
	if (sw_desc_tx)
		mdma_free_descriptor(&mdev->tx[0], sw_desc_tx);

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

	return NULL;
}

static struct dma_async_tx_descriptor *
mdma_gp_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
                      unsigned int sg_len, enum dma_transfer_direction dir,
                      unsigned long flags, void *context)
{
	struct mdma_chan *chan = to_chan(dchan);
	struct mdma_device *mdev = chan->mdev;
	struct mdma_desc_sw *sw_desc_rx = NULL;
	struct mdma_desc_sw *sw_desc_tx = NULL;
	struct mdma_desc_sw *sw_desc;
	struct mdma_desc_sw *sw_desc_linked;
	struct mdma_chan *chan_linked;
	dma_addr_t dma_addr;
	size_t len;
	unsigned cnt_descs = 0;
	unsigned long irqflags;
	struct mdma_desc_pool pool_save_rx;
	struct mdma_desc_pool pool_save_tx;
	unsigned cnt;
	bool stop_int;

	if (dir != DMA_MEM_TO_MEM) {
		dev_err(mdev->dev,
		        "%s: MDMA-GP supports mem-to-mem transfers only.\n",
		        __func__);
		return NULL;
	}

	dma_addr = (chan == &mdev->rx[0]) ? chan->config.dst_addr : 
	                                    chan->config.src_addr;

	if (dma_addr == 0) {
		dev_err(mdev->dev,
		        "%s: DMA-address is not specified. "
		        "Use dmaengine_slave_config() to specify it.\n",
		        __func__);
		return NULL;
	}

	if (!mdma_check_align_sg(chan, sgl))
		return NULL;

	cnt_descs = mdma_cnt_desc_needed(chan, sgl, sg_len, &len);

	spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
	spin_lock(&mdev->tx[0].lock);

	if ((mdev->rx[0].prepared_desc) || (mdev->tx[0].prepared_desc)) {
		spin_unlock(&mdev->tx[0].lock);
		spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);
		dev_err(mdev->dev,
		        "%s: Previous prepared descriptor was not submitted.\n",
		        __func__);
		return NULL;
	}

	pool_save_rx = mdev->rx[0].desc_pool;
	pool_save_tx = mdev->tx[0].desc_pool;

	sw_desc_rx = mdma_get_descriptor(&mdev->rx[0]);
	sw_desc_tx = mdma_get_descriptor(&mdev->tx[0]);

	if ((!sw_desc_rx) || (!sw_desc_tx))
		goto rollback;

	sw_desc_rx->len = len;
	sw_desc_tx->len = len;

	sw_desc_rx->cnt = mdma_desc_pool_get(&mdev->rx[0].desc_pool, cnt_descs,
	                                     &sw_desc_rx->pos);
	sw_desc_tx->cnt = mdma_desc_pool_get(&mdev->tx[0].desc_pool, cnt_descs,
	                                     &sw_desc_tx->pos);

	if ((!sw_desc_rx->cnt) || (!sw_desc_tx->cnt)) {
		dev_dbg(mdev->dev, "%s: can't get %u descriptors from pool\n",
		        __func__, cnt_descs);
		goto rollback;
	}

	mdev->rx[0].prepared_desc = sw_desc_rx;
	mdev->tx[0].prepared_desc = sw_desc_tx;

	chan_linked    = (chan == &mdev->rx[0]) ? &mdev->tx[0] : &mdev->rx[0];

	sw_desc        = (chan == &mdev->rx[0]) ? sw_desc_rx : sw_desc_tx;
	sw_desc_linked = (chan == &mdev->rx[0]) ? sw_desc_tx : sw_desc_rx;

	stop_int       = (chan == &mdev->rx[0]) ? false : true;

	cnt = mdma_desc_pool_fill_sg(&chan->desc_pool, sw_desc->pos,
	                             sgl, sg_len, stop_int);
	if (cnt > sw_desc->cnt) {
		dev_err(mdev->dev,
		        "%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc->cnt);
		goto rollback;
	}

	cnt = mdma_desc_pool_fill_like(&chan_linked->desc_pool,
	                               sw_desc_linked->pos,
	                               dma_addr, len, !stop_int,
	                               &chan->desc_pool, sw_desc->pos);
	if (cnt > sw_desc_linked->cnt) {
		dev_err(mdev->dev,
		        "%s: Descpitors number does not match (%u != %u)\n",
		        __func__, cnt, sw_desc->cnt);
		goto rollback;
	}

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

	mdma_desc_pool_sync(&mdev->rx[0].desc_pool, sw_desc_rx->pos,
	                    sw_desc_rx->cnt, true);
	mdma_desc_pool_sync(&mdev->tx[0].desc_pool, sw_desc_tx->pos,
	                    sw_desc_tx->cnt, true);

	async_tx_ack(&sw_desc_rx->async_tx);
	sw_desc_rx->async_tx.flags = flags;

	async_tx_ack(&sw_desc_tx->async_tx);
	sw_desc_tx->async_tx.flags = flags;

	return &sw_desc_rx->async_tx;

rollback:
	mdev->rx[0].prepared_desc = NULL;
	mdev->tx[0].prepared_desc = NULL;

	mdev->rx[0].desc_pool = pool_save_rx;
	mdev->tx[0].desc_pool = pool_save_tx;

	if (sw_desc_rx)
		mdma_free_descriptor(&mdev->rx[0], sw_desc_rx);
	if (sw_desc_tx)
		mdma_free_descriptor(&mdev->tx[0], sw_desc_tx);

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

	return NULL;
}

static int mdma_gp_device_terminate_all(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	mdma_device_terminate_all(&mdev->rx[0].slave);
	mdma_device_terminate_all(&mdev->tx[0].slave);

	return 0;
}

static void mdma_gp_issue_pending(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	unsigned long irqflags;

	spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
	spin_lock(&mdev->tx[0].lock);

	if ((mdma_prepare_transfer(&mdev->rx[0])) && 
	    (mdma_prepare_transfer(&mdev->tx[0]))) {
		mdma_start_transfer(&mdev->rx[0]);
		mdma_start_transfer(&mdev->tx[0]);
	}

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);
}

static int mdma_gp_alloc_chan_resources(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	int ret0;
	int ret1;

	ret0 = mdma_alloc_chan_resources(&mdev->rx[0].slave);
	ret1 = mdma_alloc_chan_resources(&mdev->tx[0].slave);

	if ((ret0 != ret1) || (ret0 <= 0)) {
		if (ret0 > 0)
			mdma_free_chan_resources(&mdev->rx[0].slave);
		if (ret1 > 0)
			mdma_free_chan_resources(&mdev->tx[0].slave);
		return -EFAULT;
	}

	return ret0;
}

static void mdma_gp_free_chan_resources(struct dma_chan *dchan)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;
	mdma_free_chan_resources(&mdev->rx[0].slave);
	mdma_free_chan_resources(&mdev->tx[0].slave);
}

static int mdma_gp_device_config(struct dma_chan *dchan,
                          struct dma_slave_config *config)
{
	struct mdma_device *mdev = to_chan(dchan)->mdev;

	mdma_device_config(&mdev->rx[0].slave, config);
	mdma_device_config(&mdev->tx[0].slave, config);

	return 0;
}

/**
 * mdma_irq_handler - MDMA Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the MDMA channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t mdma_gp_irq_handler(int irq, void *data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	struct mdma_device *mdev = chan->mdev;
	u32 status;
	bool need_tasklet = false;
	
	struct mdma_gp_cfg __iomem *cfg = mdev->cfg;

	status = readl(&cfg->status);

	dev_dbg(mdev->dev, "Interrupt, status %x\n", status);

	if ((status & (MDMA_IRQ_STATUS_TX | MDMA_IRQ_STATUS_RX)) == 0)
		return IRQ_NONE;

	spin_lock(&mdev->rx[0].lock);
	spin_lock(&mdev->tx[0].lock);

	if (status & MDMA_IRQ_STATUS_RX) {
		u32 rx_status = readl(&mdev->rx[0].regs->status);
		dev_dbg(mdev->dev, "RX-interrupt, status %x\n", rx_status);

		if (!mdev->rx[0].active_desc) {
			dev_dbg(mdev->dev,
			        "Interrupt without active descriptor\n");
		} else {
			mdev->rx[0].active_desc->err = true;
			need_tasklet = true;
		}
	}

	if (status & MDMA_IRQ_STATUS_TX) {
		u32 tx_status = readl(&mdev->tx[0].regs->status);
		dev_dbg(mdev->dev, "TX-interrupt, status %x\n", tx_status);

		if (!mdev->tx[0].active_desc) {
			dev_dbg(mdev->dev,
			        "Interrupt without active descriptor\n");
		} else {
			if ((tx_status & MDMA_IRQ_INT_DESC) == 0) {
				mdev->tx[0].active_desc->err = true;
			}
			else {
				mdev->rx[0].active_desc->completed = true;
				mdev->tx[0].active_desc->completed = true;
			}
			need_tasklet = true;
		}

	}

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock(&mdev->rx[0].lock);

	if (need_tasklet)
		tasklet_schedule(&chan->tasklet);

	return IRQ_HANDLED;
}

/**
 * mdma_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the MDMA channel structure
 */
static void mdma_gp_do_tasklet(unsigned long data)
{
	struct mdma_chan *chan = (struct mdma_chan *)data;
	struct mdma_device *mdev = chan->mdev;
	unsigned long irqflags;
	bool err = false;

	struct dmaengine_desc_callback cb_rx = {
		.callback = NULL,
		.callback_param = NULL,
		.callback_result = NULL
	};
	struct dmaengine_desc_callback cb_tx = {
		.callback = NULL,
		.callback_param = NULL,
		.callback_result = NULL
	};

	spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
	spin_lock(&mdev->tx[0].lock);

	if ((mdev->rx[0].active_desc->err) || (mdev->tx[0].active_desc->err)) {
		mdma_gp_reset(mdev);
		err = true;
	}

	if ((err) || (mdev->rx[0].active_desc->completed)) {
		mdma_complete_descriptor(&mdev->rx[0], mdev->rx[0].active_desc, 
		                         !err, &cb_rx);
		mdma_complete_descriptor(&mdev->tx[0], mdev->tx[0].active_desc, 
		                         !err, &cb_tx);
		mdev->rx[0].active_desc = NULL;
		mdev->tx[0].active_desc = NULL;
	}

	if (dmaengine_desc_callback_valid(&cb_rx)) {
		spin_unlock(&mdev->tx[0].lock);
		spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

		dmaengine_desc_callback_invoke(&cb_rx, NULL);

		spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
		spin_lock(&mdev->tx[0].lock);
	}

	if (dmaengine_desc_callback_valid(&cb_tx)) {
		spin_unlock(&mdev->tx[0].lock);
		spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);

		dmaengine_desc_callback_invoke(&cb_tx, NULL);

		spin_lock_irqsave(&mdev->rx[0].lock, irqflags);
		spin_lock(&mdev->tx[0].lock);
	}

	if ((mdma_prepare_transfer(&mdev->rx[0])) &&
	    (mdma_prepare_transfer(&mdev->tx[0]))) {
		mdma_start_transfer(&mdev->rx[0]);
		mdma_start_transfer(&mdev->tx[0]);
	}

	spin_unlock(&mdev->tx[0].lock);
	spin_unlock_irqrestore(&mdev->rx[0].lock, irqflags);
}

const struct mdma_of_data mdma_gp_of_data = {
	.max_transaction             = MDMA_GP_MAX_TRANS_LEN,
	.len_mask                    = MDMA_GP_LEN_MASK,
	.ch_settings                 = 
		MDMA_CHAN_DESC_LONG | 
		(sizeof(struct mdma_desc_long_ll) << MDMA_CHAN_DESC_GAP_SHIFT),

	.device_alloc_chan_resources = mdma_gp_alloc_chan_resources,
	.device_free_chan_resources  = mdma_gp_free_chan_resources,
	.device_prep_dma_memcpy      = mdma_gp_prep_memcpy,
	.device_prep_slave_sg        = mdma_gp_prep_slave_sg,
	.device_config               = mdma_gp_device_config,
	.device_terminate_all        = mdma_gp_device_terminate_all,
	.device_tx_status            = dma_cookie_status,
	.device_issue_pending        = mdma_gp_issue_pending,

	.tx_submit                   = mdma_gp_tx_submit,

	.irq_handler                 = mdma_gp_irq_handler,
	.tasklet_func                = mdma_gp_do_tasklet,
};
