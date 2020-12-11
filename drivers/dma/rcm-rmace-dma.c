/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

// ???
#define DEBUG

#define pr_fmt(fmt) "rcm-rmace-dma: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/rcm-rmace.h>
#include "dmaengine.h"

// may be already existed
#define MAX_DMA_CHANNEL_COUNT 32

static dma_cookie_t tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct rcm_rmace_dma_chan *rmace_chan = container_of(tx->chan, struct rcm_rmace_dma_chan, dma_chan); // ??? func
	struct rcm_rmace_dma_async_tx_desc *desc = container_of(tx, struct rcm_rmace_dma_async_tx_desc, async_tx); // ??? func
	unsigned long irq_flags;
	dma_cookie_t cookie;

	pr_debug("tx_submit\n"); // ???

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	cookie = dma_cookie_assign(tx);
	list_del(&desc->list); // in case the descriptor is in complete_list
	list_add_tail(&desc->list, &rmace_chan->pending_list);
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	return cookie;
}

/* ??? static void started_callback(struct rcm_rmace_ctx *ctx, void *arg)
{
	struct rcm_rmace_dma_async_tx_desc *desc = container_of(ctx, struct rcm_rmace_dma_async_tx_desc, ctx); // ??? func
	struct rcm_rmace_dma_chan *rmace_chan = container_of(desc->async_tx.chan, struct rcm_rmace_dma_chan, dma_chan); // ??? func
	unsigned long irq_flags;

	pr_debug("started_callback\n"); // ???

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	rmace_chan->used_cookie = desc->async_tx.cookie;
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
}*/

static void finished_callback(struct rcm_rmace_ctx *ctx, void *arg)
{
	struct rcm_rmace_dma_async_tx_desc *desc = container_of(ctx, struct rcm_rmace_dma_async_tx_desc, ctx); // ??? func
	struct rcm_rmace_dma_chan *rmace_chan = container_of(desc->async_tx.chan, struct rcm_rmace_dma_chan, dma_chan); // ??? func
	struct dmaengine_result result;
	unsigned long irq_flags;

	pr_debug("ctx_complete_callback\n"); // ???

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	// ??? rmace_chan->last_cookie = desc->async_tx.cookie;
	list_del(&desc->list); // from active_list  ???? move
	list_add_tail(&desc->list, &rmace_chan->complete_list);
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	dma_cookie_complete(&desc->async_tx); // ???

	result.result = DMA_TRANS_NOERROR;
	result.residue = 0;
	if ((ctx->status & RCM_RMACE_CTX_STATUS_ERROR) != 0)
		result.result = DMA_TRANS_READ_FAILED;
	else if ((ctx->status & RCM_RMACE_CTX_STATUS_ERROR) != 0)
		result.result = DMA_TRANS_ABORTED;
	dmaengine_desc_get_callback_invoke(&desc->async_tx, &result);

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	if (!async_tx_test_ack(&desc->async_tx)) {
		list_del(&desc->list); // from complete_list
		list_add_tail(&desc->list, &rmace_chan->free_list);
	}
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
}

static struct dma_async_tx_descriptor *prep_dma_memcpy(
	struct dma_chan *chan,
	dma_addr_t dst,
	dma_addr_t src,
	size_t len,
	unsigned long flags)
{
	struct rcm_rmace_dma_chan *rmace_chan = container_of(chan, struct rcm_rmace_dma_chan, dma_chan); // ??? func
	struct rcm_rmace_dma_async_tx_desc *desc;
	unsigned long irq_flags;

	pr_debug("prep_dma_memcpy\n"); // ???

	if (((dst & 7) != 0) || ((src & 7) != 0) || ((len & 7) != 0)) { // ??? const
		pr_warn("aligment error\n");
		return NULL;
	}

	if (len > RCM_RMACE_MAX_DATA_TRANSFER) {
		pr_warn("aligment error\n");
		return NULL;
	}

	if ((flags & ~(DMA_PREP_INTERRUPT | DMA_CTRL_ACK)) != 0) {
		pr_warn("flags 0x%08lX error\n", flags);
		return NULL;
	}

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	desc = list_first_entry_or_null(&rmace_chan->free_list, struct rcm_rmace_dma_async_tx_desc, list);
	if (desc != NULL)
		list_del_init(&desc->list); // *_init is for correct list_del in tx_submit ???
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
	if (desc == NULL) {
		pr_warn("no free descriptors\n");
		return NULL;
	}

	desc->async_tx.flags = flags;
	desc->src_desc_infos[1].address = src;
	desc->src_desc_infos[1].length = len;
	desc->dst_desc_info.address = dst;
	desc->dst_desc_info.length = len;

	return &desc->async_tx;
}

static enum dma_status tx_status(
	struct dma_chan *chan,
	dma_cookie_t cookie,
	struct dma_tx_state *txstate)
{
	struct rcm_rmace_dma_chan *rmace_chan = container_of(chan, struct rcm_rmace_dma_chan, dma_chan);
	struct rcm_rmace_dma_async_tx_desc *desc;
	// ??? dma_cookie_t last;
	// ??? dma_cookie_t used;
	enum dma_status status;
	unsigned long irq_flags;

	pr_debug("tx_status\n"); // ???

	/* ??? spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	last = rmace_chan->last_cookie;
	used = rmace_chan->last_cookie; // ???
	if (used < DMA_MIN_COOKIE) // ???
		used = DMA_MIN_COOKIE; // ???
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);*/

	/* ??? if (txstate) {
		txstate->last = last;
		txstate->used = used;
		txstate->residue = 0;
	}*/

	// ??? status = dma_async_is_complete(cookie, last, used);
	status = dma_cookie_status(chan, cookie, txstate);
	if (status == DMA_COMPLETE) {
		spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
		list_for_each_entry(desc, &rmace_chan->pending_list, list) {
			if (desc->async_tx.cookie == cookie) {
				if ((desc->ctx.status & RCM_RMACE_CTX_STATUS_ERROR) != 0)
					status = DMA_ERROR;
				break;
			}
		}
		spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
	}

	pr_debug("txstatus = %u\n", status); // ???

	return status;
}

static void issue_pending(struct dma_chan *chan)
{
	struct rcm_rmace_dma_chan *rmace_chan = container_of(chan, struct rcm_rmace_dma_chan, dma_chan);
	struct rcm_rmace_dma_async_tx_desc *desc;
	unsigned long irq_flags;

	pr_debug("issue_pending\n"); // ???

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	list_for_each_entry(desc, &rmace_chan->pending_list, list)
		rcm_rmace_ctx_schelude(&desc->ctx);
	list_splice_tail_init(&rmace_chan->pending_list, &rmace_chan->active_list);
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
}

int rcm_rmace_dma_register(struct rcm_rmace_dev *rmace)
{
	struct rcm_rmace_dma_chan *channel;
	unsigned i, j;
	int ret;

	// ??? optimize dma_channel_count
	ret = of_property_read_u32(rmace->pdev->dev.of_node, "dma-channel-count", &rmace->dma_channel_count);
	if (ret != 0) {
		pr_err("no dma-channel-count property");
		return ret;
	}
	if ((rmace->dma_channel_count == 0) || (rmace->dma_channel_count > MAX_DMA_CHANNEL_COUNT)) {
		pr_err("property dma-channel-count has wrong value\n");
		return -EINVAL;
	}

	rmace->dma_channels = devm_kzalloc(
		&rmace->pdev->dev,
		sizeof(struct rcm_rmace_dma_chan) * rmace->dma_channel_count,
		GFP_KERNEL);
	if (rmace->dma_channels == NULL) {
		pr_err("insufficient memory\n");
		return -ENOMEM;
	}

	// ??? optimize rmace->dma_dev
	rmace->dma_dev.dev = &rmace->pdev->dev;
	rmace->dma_dev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	rmace->dma_dev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	rmace->dma_dev.directions = BIT(DMA_MEM_TO_MEM);
	rmace->dma_dev.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_cap_set(DMA_MEMCPY, rmace->dma_dev.cap_mask);
	rmace->dma_dev.copy_align = DMAENGINE_ALIGN_8_BYTES; // ???
	INIT_LIST_HEAD(&rmace->dma_dev.channels);
	rmace->dma_dev.device_prep_dma_memcpy = prep_dma_memcpy;
	rmace->dma_dev.device_tx_status = tx_status;
	rmace->dma_dev.device_issue_pending = issue_pending;

	for (i = 0; i < rmace->dma_channel_count; ++i) {
		channel = &rmace->dma_channels[i];
		channel->dma_chan.device = &rmace->dma_dev;
		dma_cookie_init(&channel->dma_chan);
		list_add_tail(&channel->dma_chan.device_node, &rmace->dma_dev.channels);
		spin_lock_init(&channel->list_lock);
		INIT_LIST_HEAD(&channel->free_list);
		INIT_LIST_HEAD(&channel->pending_list);
		INIT_LIST_HEAD(&channel->active_list);
		INIT_LIST_HEAD(&channel->complete_list);
		for (j = 0; j < RCM_RMACE_ASYNC_TX_DESC_COUNT; ++j) { // ??? optimize channel->async_tx_descs[j...
			dma_async_tx_descriptor_init(&channel->async_tx_descs[j].async_tx, &channel->dma_chan);
			channel->async_tx_descs[j].async_tx.tx_submit = tx_submit;
			rcm_rmace_ctx_init(rmace, &channel->async_tx_descs[j].ctx);
			channel->async_tx_descs[j].ctx.src_desc_count = 2;
			channel->async_tx_descs[j].ctx.src_desc_infos = channel->async_tx_descs[j].src_desc_infos;
			channel->async_tx_descs[j].src_desc_infos[0].address = rmace->dma_data_phys_addr + offsetof(struct rcm_rmace_dma_data, memcpy_control_block);
			channel->async_tx_descs[j].src_desc_infos[0].length = 8; // ???
			channel->async_tx_descs[j].src_desc_infos[0].valid = true;
			channel->async_tx_descs[j].src_desc_infos[0].act2 = true;
			channel->async_tx_descs[j].src_desc_infos[1].act0 = true;
			channel->async_tx_descs[j].src_desc_infos[1].act2 = true;
			channel->async_tx_descs[j].ctx.dst_desc_count = 1;
			channel->async_tx_descs[j].ctx.dst_desc_infos = &channel->async_tx_descs[j].dst_desc_info;
			channel->async_tx_descs[j].dst_desc_info.act0 = true;
			channel->async_tx_descs[j].dst_desc_info.act2 = true;
			// ??? channel->async_tx_descs[j].ctx.started_callback = started_callback;
			channel->async_tx_descs[j].ctx.finished_callback = finished_callback;
			INIT_LIST_HEAD(&channel->async_tx_descs[j].list);
			list_add_tail(&channel->async_tx_descs[j].list, &channel->free_list);
		}
	}

	rmace->dma_data->memcpy_control_block = cpu_to_le64(1ULL << 56); // ??? const

	ret = dma_async_device_register(&rmace->dma_dev);
	if (ret != 0) {
		pr_err("dma device register error\n");
		return ret;
	}

	pr_debug("dma interface registered succesfully\n");

	return 0;
}
EXPORT_SYMBOL(rcm_rmace_dma_register);

void rcm_rmace_dma_unregister(struct rcm_rmace_dev *rmace)
{
	dma_async_device_unregister(&rmace->dma_dev);
}
EXPORT_SYMBOL(rcm_rmace_dma_unregister);


/* ??? static int __init rcm_rmace_dma_init(void)
{
	return 0;
}

static void __exit rcm_rmace_dma_exit(void)
{
}

module_init(rcm_rmace_dma_init);
module_exit(rcm_rmace_dma_exit);*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_DESCRIPTION("RCM RMACE DMA driver");
