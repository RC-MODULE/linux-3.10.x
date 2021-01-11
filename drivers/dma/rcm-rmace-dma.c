/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

#define pr_fmt(fmt) "rcm-rmace-dma: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_dma.h>
#include <linux/rcm-rmace.h>
#include "dmaengine.h"

#define MAX_CHANNEL_COUNT 32
#define ASYNC_TX_DESC_COUNT 16 // per channel
#define DMA_DESC_COUNT (RCM_RMACE_HW_DESC_COUNT - 1)
#define COPY_TRANSFER_SIZE (64 * 1024 * 1024 - 8)
#define XOR_TRANSFER_SIZE (4 * 1024)
#define XOR_MAX_SOURCES_COUNT 255

struct rmace_async_tx_desc
{
	struct dma_async_tx_descriptor async_tx;
	struct rcm_rmace_ctx rmace_ctx;
	struct rcm_rmace_hw_desc src_descs[DMA_DESC_COUNT];
	struct rcm_rmace_hw_desc dst_descs[DMA_DESC_COUNT];
	struct list_head list; // free list, pending list, active list
	u64 *control_block; // LE
};

struct rmace_dma_chan
{
	struct rcm_rmace_idma *idma;
	struct dma_chan dma_chan;
	struct rmace_async_tx_desc tx_descs[ASYNC_TX_DESC_COUNT];
	spinlock_t list_lock;
	struct list_head free_list; // protected by list_lock
	struct list_head created_list; // protected by list_lock
	struct list_head pending_list; // protected by list_lock
	struct list_head active_list; // protected by list_lock
	struct list_head completed_list; // protected by list_lock
};

struct dma_data
{
	u64 control_blocks[MAX_CHANNEL_COUNT][ASYNC_TX_DESC_COUNT]; // LE
};

struct rcm_rmace_idma
{
	struct rcm_rmace_dev *rmace;
	struct dma_device dma_dev;
	unsigned channel_count;
	struct rmace_dma_chan *channels;
	struct dma_data *dma_data;
	dma_addr_t dma_data_phys_addr;
};

static inline struct rmace_dma_chan *rmace_chan_from_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct rmace_dma_chan, dma_chan);
}

static inline struct rmace_async_tx_desc *rmace_tx_desc_from_async_tx_desc(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct rmace_async_tx_desc, async_tx);
}

static inline struct rmace_async_tx_desc *rmace_tx_desc_from_ctx(struct rcm_rmace_ctx *rmace_ctx)
{
	return container_of(rmace_ctx, struct rmace_async_tx_desc, rmace_ctx);
}

static dma_cookie_t tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(tx->chan);
	struct rmace_async_tx_desc *tx_desc = rmace_tx_desc_from_async_tx_desc(tx);
	unsigned long irq_flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	cookie = dma_cookie_assign(tx);
	list_move_tail(&tx_desc->list, &rmace_chan->pending_list); // from created_list or completed_list
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	return cookie;
}

static void rmace_callback(struct rcm_rmace_ctx *ctx, void *arg)
{
	struct rmace_async_tx_desc *tx_desc = rmace_tx_desc_from_ctx(ctx);
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(tx_desc->async_tx.chan);
	struct rmace_async_tx_desc *tx_temp;
	struct dmaengine_result result;
	unsigned long irq_flags;

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	dma_cookie_complete(&tx_desc->async_tx);
	list_move_tail(&tx_desc->list, &rmace_chan->completed_list); // from active_list
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	result.result = DMA_TRANS_NOERROR;
	result.residue = 0;
	if ((ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) == 0)
		result.result = DMA_TRANS_READ_FAILED;
	dmaengine_desc_get_callback_invoke(&tx_desc->async_tx, &result);

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	list_for_each_entry_safe(tx_desc, tx_temp, &rmace_chan->completed_list, list) {
		if (async_tx_test_ack(&tx_desc->async_tx))
			list_move_tail(&tx_desc->list, &rmace_chan->free_list); // from completed_list
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
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(chan);
	struct rmace_async_tx_desc *tx_desc;
	struct rcm_rmace_hw_desc *src_hw_desc;
	struct rcm_rmace_hw_desc *dst_hw_desc;
	unsigned long irq_flags;
	unsigned offset;
	unsigned size;

	if (((dst & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0)
		|| ((src & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0)
		|| ((len & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0))
	{
		pr_warn("aligment error\n");
		return NULL;
	}

	if (len > (size_t)COPY_TRANSFER_SIZE * (DMA_DESC_COUNT - 1)) {
		pr_warn("transfer size error\n");
		return NULL;
	}

	if ((flags & ~(DMA_PREP_INTERRUPT | DMA_CTRL_ACK)) != 0) {
		pr_warn("flags 0x%08lX error\n", flags);
		return NULL;
	}

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	tx_desc = list_first_entry_or_null(&rmace_chan->free_list, struct rmace_async_tx_desc, list);
	if (tx_desc != NULL)
		list_move_tail(&tx_desc->list, &rmace_chan->created_list); // from free_list
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	if (tx_desc == NULL) {
		pr_warn("no free descriptors\n");
		return NULL;
	}

	// some parameters have been already filled during the initialization
	tx_desc->async_tx.flags = flags;
	*(tx_desc->control_block) = cpu_to_le64(1ULL << RCM_RMACE_HEADER_TYPE_SHIFT);
	src_hw_desc = &tx_desc->src_descs[1]; // control block has already configured
	dst_hw_desc = tx_desc->dst_descs;
	offset = 0;
	while (len) {
		size = len;
		if (size > COPY_TRANSFER_SIZE)
			size = COPY_TRANSFER_SIZE;
		len -= size;
		src_hw_desc->address = cpu_to_le32(src + offset);
		src_hw_desc->data = cpu_to_le32(
			(len == 0 ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (size << RCM_RMACE_DESC_LEN_SHIFT));
		++src_hw_desc;
		dst_hw_desc->address = cpu_to_le32(dst + offset);
		dst_hw_desc->data = cpu_to_le32(
			(len == 0 ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (size << RCM_RMACE_DESC_LEN_SHIFT));
		++dst_hw_desc;
		offset += size;
	}
	tx_desc->rmace_ctx.src_desc_count = src_hw_desc - tx_desc->src_descs;
	tx_desc->rmace_ctx.dst_desc_count = dst_hw_desc - tx_desc->dst_descs;

	return &tx_desc->async_tx;
}

struct dma_async_tx_descriptor *prep_dma_xor(
	struct dma_chan *chan,
	dma_addr_t dst,
	dma_addr_t *src,
	unsigned int src_cnt,
	size_t len,
	unsigned long flags)
{
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(chan);
	struct rmace_async_tx_desc *tx_desc;
	struct rcm_rmace_hw_desc *src_hw_desc;
	struct rcm_rmace_hw_desc *dst_hw_desc;
	unsigned long irq_flags;
	unsigned offset;
	unsigned size;
	unsigned i;

	if ((src_cnt < 2) || (src_cnt > XOR_MAX_SOURCES_COUNT)) {
		pr_warn("xor source count error\n");
		return NULL;
	}

	if (((dst & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0)
		|| ((len & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0))
	{
		pr_warn("aligment error\n");
		return NULL;
	}

	for (i = 0; i < src_cnt; ++i)
		if ((src[i] & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0) {
			pr_warn("source aligment error\n");
			return NULL;
		}

	if (len > (size_t)XOR_TRANSFER_SIZE * ((DMA_DESC_COUNT - 1) / src_cnt)) {
		pr_warn("transfer size error\n");
		return NULL;
	}

	if ((flags & ~(DMA_PREP_INTERRUPT | DMA_CTRL_ACK)) != 0) {
		pr_warn("flags 0x%08lX error\n", flags);
		return NULL;
	}

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	tx_desc = list_first_entry_or_null(&rmace_chan->free_list, struct rmace_async_tx_desc, list);
	if (tx_desc != NULL)
		list_move_tail(&tx_desc->list, &rmace_chan->created_list); // from free_list
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);

	if (tx_desc == NULL) {
		pr_warn("no free descriptors\n");
		return NULL;
	}

	// some parameters have been already filled during the initialization
	tx_desc->async_tx.flags = flags;
	*(tx_desc->control_block) = cpu_to_le64(
		((u64)src_cnt << RCM_RMACE_HEADER_FEATURES_SHIFT)
		| (2ULL << RCM_RMACE_HEADER_TYPE_SHIFT));
	src_hw_desc = &tx_desc->src_descs[1]; // control block has already configured
	dst_hw_desc = tx_desc->dst_descs;
	offset = 0;
	while (len) {
		size = len;
		if (size > COPY_TRANSFER_SIZE)
			size = COPY_TRANSFER_SIZE;
		len -= size;
		for (i = 0; i < src_cnt; ++i) {
			src_hw_desc->address = cpu_to_le32(src[i] + offset);
			src_hw_desc->data = cpu_to_le32(
				RCM_RMACE_DESC_ACT0_MASK
				| RCM_RMACE_DESC_ACT2_MASK
				| (size << RCM_RMACE_DESC_LEN_SHIFT));
			++src_hw_desc;
		}
		dst_hw_desc->address = cpu_to_le32(dst + offset);
		dst_hw_desc->data = cpu_to_le32(
			(len == 0 ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (size << RCM_RMACE_DESC_LEN_SHIFT));
		++dst_hw_desc;
		offset += size;
	}
	tx_desc->rmace_ctx.src_desc_count = src_hw_desc - tx_desc->src_descs;
	tx_desc->rmace_ctx.dst_desc_count = dst_hw_desc - tx_desc->dst_descs;

	return &tx_desc->async_tx;
}

static enum dma_status tx_status(
	struct dma_chan *chan,
	dma_cookie_t cookie,
	struct dma_tx_state *txstate)
{
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(chan);
	struct rmace_async_tx_desc *tx_desc;
	enum dma_status status;
	unsigned long irq_flags;

	status = dma_cookie_status(chan, cookie, txstate);
	if (status == DMA_COMPLETE) {
		spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
		list_for_each_entry(tx_desc, &rmace_chan->completed_list, list) {
			if (tx_desc->async_tx.cookie == cookie) {
				if ((tx_desc->rmace_ctx.status & RCM_RMACE_CTX_STATUS_SUCCESS) == 0)
					status = DMA_ERROR;
				break;
			}
		}
		spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
	}

	return status;
}

static void issue_pending(struct dma_chan *chan)
{
	struct rmace_dma_chan *rmace_chan = rmace_chan_from_dma_chan(chan);
	struct rmace_async_tx_desc *tx_desc;
	unsigned long irq_flags;

	spin_lock_irqsave(&rmace_chan->list_lock, irq_flags);
	list_for_each_entry(tx_desc, &rmace_chan->pending_list, list)
		rcm_rmace_ctx_schedule(&tx_desc->rmace_ctx);
	list_splice_tail_init(&rmace_chan->pending_list, &rmace_chan->active_list);
	spin_unlock_irqrestore(&rmace_chan->list_lock, irq_flags);
}

static struct dma_chan *of_dma_rcm_xlate(struct of_phandle_args *dma_spec, struct of_dma *ofdma)
{
	struct rcm_rmace_idma *idma = ofdma->of_dma_data;
	unsigned int chan_index;

	if (!idma)
		return NULL;

	if (dma_spec->args_count != 1)
		return NULL;

	chan_index = dma_spec->args[0];
	if (chan_index >= idma->channel_count)
		return NULL;

	return dma_get_slave_channel(&idma->channels[chan_index].dma_chan);
}

int rcm_rmace_dma_register(struct rcm_rmace_dev *rmace)
{
	struct rcm_rmace_idma *idma;
	struct dma_device *dma_dev;
	struct rmace_async_tx_desc *tx_desc;
	struct rmace_dma_chan *channel;
	unsigned i, j;
	int ret;

	idma = devm_kzalloc(&rmace->pdev->dev, sizeof *idma, GFP_KERNEL);
	if (idma == NULL) {
		pr_err("insufficient memory\n");
		return -ENOMEM;
	}
	idma->rmace = rmace;
	rmace->idma = idma;

	ret = of_property_read_u32(rmace->pdev->dev.of_node, "dma-channel-count", &idma->channel_count);
	if (ret != 0) {
		pr_err("no dma-channel-count property");
		return ret;
	}
	if ((idma->channel_count == 0) || (idma->channel_count > MAX_CHANNEL_COUNT)) {
		pr_err("property dma-channel-count has wrong value\n");
		return -EINVAL;
	}

	idma->channels = devm_kzalloc(
		&rmace->pdev->dev,
		sizeof(struct rmace_dma_chan) * idma->channel_count,
		GFP_KERNEL);
	if (idma->channels == NULL) {
		pr_err("insufficient memory\n");
		return -ENOMEM;
	}

	idma->dma_data = dmam_alloc_coherent(
		&rmace->pdev->dev,
		sizeof *idma->dma_data,
		&idma->dma_data_phys_addr,
		GFP_KERNEL);
	if (idma->dma_data == NULL) {
		pr_err("cannot allocate DMA buffers");
		return -ENODEV;
	}

	dma_dev = &idma->dma_dev;
	dma_dev->dev = &rmace->pdev->dev;
	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->directions = BIT(DMA_MEM_TO_MEM);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_cap_set(DMA_XOR, dma_dev->cap_mask);
	dma_dev->copy_align = DMAENGINE_ALIGN_8_BYTES;
	dma_dev->xor_align = DMAENGINE_ALIGN_8_BYTES;
	dma_dev->max_xor = XOR_MAX_SOURCES_COUNT;
	INIT_LIST_HEAD(&dma_dev->channels);
	dma_dev->device_prep_dma_memcpy = prep_dma_memcpy;
	dma_dev->device_prep_dma_xor = prep_dma_xor;
	dma_dev->device_tx_status = tx_status;
	dma_dev->device_issue_pending = issue_pending;

	for (i = 0; i < idma->channel_count; ++i) {
		channel = &idma->channels[i];
		channel->idma = idma;
		channel->dma_chan.device = dma_dev;
		dma_cookie_init(&channel->dma_chan);
		list_add_tail(&channel->dma_chan.device_node, &dma_dev->channels);
		spin_lock_init(&channel->list_lock);
		INIT_LIST_HEAD(&channel->free_list);
		INIT_LIST_HEAD(&channel->created_list);
		INIT_LIST_HEAD(&channel->pending_list);
		INIT_LIST_HEAD(&channel->active_list);
		INIT_LIST_HEAD(&channel->completed_list);
		for (j = 0; j < ASYNC_TX_DESC_COUNT; ++j) {
			tx_desc = &channel->tx_descs[j];
			dma_async_tx_descriptor_init(&tx_desc->async_tx, &channel->dma_chan);
			tx_desc->async_tx.tx_submit = tx_submit;
			rcm_rmace_ctx_init(rmace, &tx_desc->rmace_ctx);
			tx_desc->rmace_ctx.src_descs = tx_desc->src_descs;
			tx_desc->src_descs[0].address = cpu_to_le32(
				idma->dma_data_phys_addr
				+ offsetof(struct dma_data, control_blocks)
				+ (sizeof idma->dma_data->control_blocks[0]) * i
				+ (sizeof idma->dma_data->control_blocks[0][0]) * j);
			tx_desc->src_descs[0].data = cpu_to_le32(
				RCM_RMACE_DESC_VALID_MASK
				| RCM_RMACE_DESC_ACT2_MASK
				| ((sizeof idma->dma_data->control_blocks[0][0]) << RCM_RMACE_DESC_LEN_SHIFT));
			tx_desc->rmace_ctx.dst_descs = tx_desc->dst_descs;
			tx_desc->rmace_ctx.callback = rmace_callback;
			INIT_LIST_HEAD(&tx_desc->list);
			tx_desc->control_block = &idma->dma_data->control_blocks[i][j];
			list_add_tail(&tx_desc->list, &channel->free_list);
		}
	}

	ret = dma_async_device_register(dma_dev);
	if (ret != 0) {
		pr_err("dma device register error\n");
		return ret;
	}

	pr_info("dma interface registered succesfully\n");

	ret = of_dma_controller_register(rmace->pdev->dev.of_node, of_dma_rcm_xlate, idma);
	if (ret != 0)
		pr_err("unable to register DMA to the generic DT DMA helpers\n");

	return 0;
}
EXPORT_SYMBOL(rcm_rmace_dma_register);

void rcm_rmace_dma_unregister(struct rcm_rmace_dev *rmace)
{
	dma_async_device_unregister(&rmace->idma->dma_dev);
}
EXPORT_SYMBOL(rcm_rmace_dma_unregister);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_DESCRIPTION("RCM SoCs RMACE DMA interface");
