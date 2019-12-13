/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/soc.h>

//#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/dmaengine.h>
#include <sound/dmaengine_pcm.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>

#include "dmaengine.h"

static int g_dma_debug = 0;

module_param_named(debug, g_dma_debug, int, 0);

#define TRACE(format, ...)                                                     \
	do {                                                                   \
		if (g_dma_debug) {                                             \
			printk("TRACE: rcm-audio-dma/%s:%d: " format "\n",     \
			       __func__, __LINE__, ##__VA_ARGS__);             \
		}                                                              \
	} while (0)

#define RCM_DMA_ENA_REG 0x00
#define RCM_DMA_CH0_BASE0 0x04
#define RCM_DMA_CH0_BASE1 0x08
#define RCM_DMA_CH1_BASE0 0x0C
#define RCM_DMA_CH1_BASE1 0x10
#define RCM_DMA_CH2_BASE0 0x14
#define RCM_DMA_CH2_BASE1 0x18
#define RCM_DMA_CH3_BASE0 0x1C
#define RCM_DMA_CH3_BASE1 0x20
#define RCM_DMA_CH0_END0 0x24
#define RCM_DMA_CH0_END1 0x28
#define RCM_DMA_CH1_END0 0x2C
#define RCM_DMA_CH1_END1 0x30
#define RCM_DMA_CH2_END0 0x34
#define RCM_DMA_CH2_END1 0x38
#define RCM_DMA_CH3_END0 0x3C
#define RCM_DMA_CH3_END1 0x40
#define RCM_DMA_SLV0_BASE 0x44
#define RCM_DMA_SLV1_BASE 0x48
#define RCM_DMA_SLV2_BASE 0x4C
#define RCM_DMA_SLV3_BASE 0x50
#define RCM_DMA_CH0_TRW 0x54
#define RCM_DMA_CH1_TRW 0x58
#define RCM_DMA_CH2_TRW 0x5C
#define RCM_DMA_CH3_TRW 0x60
#define RCM_DMA_SLV_OVRH 0x64
#define RCM_DMA_SLV0_BSIZE 0x68
#define RCM_DMA_SLV1_BSIZE 0x6C
#define RCM_DMA_SLV2_BSIZE 0x70
#define RCM_DMA_SLV3_BSIZE 0x74
#define RCM_DMA_AXI_PARAM 0x78
#define RCM_DMA_INT_MASK 0x7C
#define RCM_DMA_INT 0x80
#define RCM_DMA_BUF_STATUS 0x84

#define RCM_DMA_CHANNEL0_ACTIVE 0x01
#define RCM_DMA_ENA_SPDIF 0x10
#define RCM_DMA_CHANNEL0_SW 0x100

#define RCM_DMA_INT_CH0_WR_END 0x01
#define RCM_DMA_INT_CH1_WR_END 0x02
#define RCM_DMA_INT_CH2_WR_END 0x04
#define RCM_DMA_INT_CH3_WR_END 0x08

static const struct regmap_config rcm_dma_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_DMA_BUF_STATUS,
};

#define RCM_DMA_MODE_UNDEFINED 0
#define RCM_DMA_MODE_I2S 1
#define RCM_DMA_MODE_SPDIF 2

struct rcm_audio_dma_channel {
	/* DMA-Engine Channel */
	struct dma_chan chan;

	struct rcm_audio_dma *dmac;
	phys_addr_t fifo_addr;
	//	int base_len;
	struct dma_async_tx_descriptor desc;
	int active;
	int terminate;
};

struct rcm_audio_dma {
	/* To protect channel manipulation */
	spinlock_t lock;
	struct rcm_audio_dma_channel channel[5];
	struct regmap *regmap;
	struct dma_device slave;
	struct tasklet_struct task;
	// last dma_int value
	unsigned int dma_int;
	int mode;
};

static inline struct rcm_audio_dma_channel *to_chan(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return container_of(ch, struct rcm_audio_dma_channel, chan);
}

static inline struct rcm_audio_dma *to_dmac(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return to_chan(ch)->dmac;
}

static irqreturn_t rcm_dma_interrupt_handler(int irq, void *data)
{
	struct rcm_audio_dma *d = (struct rcm_audio_dma *)data;
	unsigned long flags;
	unsigned int dma_int = 0;

	regmap_read(d->regmap, RCM_DMA_INT, &dma_int);
	regmap_write(d->regmap, RCM_DMA_INT, dma_int); //	cleanup ints
	spin_lock_irqsave(&d->lock, flags);
	d->dma_int |= dma_int;
	spin_unlock_irqrestore(&d->lock, flags);
	tasklet_schedule(&d->task);
	TRACE("%08X", dma_int);

	return IRQ_HANDLED;
}

static dma_cookie_t rcm_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct rcm_audio_dma_channel *ch = to_chan(tx->chan);
	dma_cookie_t cookie;
	cookie = dma_cookie_assign(&ch->desc);
	TRACE("cookie %d", cookie);
	return cookie;
}

static struct dma_async_tx_descriptor *
	rcm_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t dma_addr,
			    size_t len, size_t period_len,
			    enum dma_transfer_direction direction,
			    unsigned long flags)
{
	struct rcm_audio_dma *d = to_dmac(chan);
	struct rcm_audio_dma_channel *ch = to_chan(chan);
	int ch_no = chan->chan_id & 0x3;
	unsigned long lock_flags;
	unsigned int ena;
	unsigned long dma_offset = 0x40000000; // to do fix on upper level
	// setup addresses for each descriptors based on numer of channels
	u32 page0_start = (u32)dma_addr + period_len + dma_offset;
	u32 page1_start = (u32)dma_addr + dma_offset;
	u32 page0_end = page0_start + period_len;
	u32 page1_end = page1_start + period_len;

	TRACE("len: %d, period_len: %d, flags: %d, dma_addr: %llx", (int)len,
	      (int)period_len, (int)flags, dma_addr);

	// we always have only 2 DMA descriptios for each hw channel.
	// call hw channels should works simultaniously, so we should prepare all of its at the same time
	regmap_read(d->regmap, RCM_DMA_ENA_REG, &ena);
	regmap_write(d->regmap, RCM_DMA_ENA_REG,
		     0x8000); // black magic - not works without it
	regmap_write(d->regmap, RCM_DMA_ENA_REG, ena); // restore

	ch->desc.chan = chan;
	ch->desc.cookie = 0;
	ch->desc.tx_submit = rcm_tx_submit;
	async_tx_ack(&ch->desc);
	dma_async_tx_descriptor_init(&ch->desc, &ch->chan);
	ch->desc.flags = flags;

	TRACE("channel %i, %llx, %08X, %08X, %08X, %08X", ch_no, ch->fifo_addr,
	      (u32)page1_start, (u32)page1_end, (u32)page0_start,
	      (u32)page0_end);

	spin_lock_irqsave(&d->lock, lock_flags);

	// write descriptors
	regmap_write(d->regmap, RCM_DMA_CH0_BASE0 + ch_no * 2 * 4, page0_start);
	regmap_write(d->regmap, RCM_DMA_CH0_BASE1 + ch_no * 2 * 4, page1_start);
	regmap_write(d->regmap, RCM_DMA_CH0_END0 + ch_no * 2 * 4, page0_end);
	regmap_write(d->regmap, RCM_DMA_CH0_END1 + ch_no * 2 * 4, page1_end);

	// size
	regmap_write(d->regmap, RCM_DMA_CH0_TRW + ch_no * 4, period_len / 4);

	// slave base
	regmap_write(d->regmap, RCM_DMA_SLV0_BASE + ch_no * 4,
		     (u32)ch->fifo_addr);

	// slave fifo size
	regmap_write(d->regmap, RCM_DMA_SLV0_BSIZE + ch_no * 4, 31);

	spin_unlock_irqrestore(&d->lock, lock_flags);

	return &ch->desc;
}

// Using DMA_RESIDUE_GRANULARITY_DESCRIPTOR mode - this function should not be called
// So its just for compatibility.
static enum dma_status rcm_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	enum dma_status ret;
	unsigned int residual = 0;

	ret = dma_cookie_status(chan, cookie, txstate);

	if (!txstate)
		return ret;

	dma_set_residue(txstate, residual);

	return ret;
}

static int rcm_config(struct dma_chan *chan,
		      struct dma_slave_config *slave_config)
{
	struct rcm_audio_dma_channel *ch = to_chan(chan);
	struct rcm_audio_dma *d = to_dmac(chan);
	int requested_mode;

	TRACE("slave_config->dst_addr = %llx, slave_config->dst_maxburst = %d",
	      slave_config->dst_addr, slave_config->dst_maxburst);

	ch->fifo_addr = slave_config->dst_addr;

	// number of channel used for switching dma controller between i2s and spdif mode
	// 0 means I2S, 1 - SPDIF
	if (chan->chan_id > 3) {
		requested_mode = RCM_DMA_MODE_SPDIF;
	} else {
		requested_mode = RCM_DMA_MODE_I2S;
	}

	if (requested_mode != d->mode) {
		if (d->mode != RCM_DMA_MODE_UNDEFINED) {
			dev_err(d->slave.dev, "conflict mode requested");
			return -EBUSY;
		}
		d->mode = requested_mode;
		TRACE("set mode to %s",
		      requested_mode == RCM_DMA_MODE_I2S ? "i2s" : "spdif");
	}

	return 0;
}

static int rcm_terminate_all(struct dma_chan *chan)
{
	struct rcm_audio_dma *d = to_dmac(chan);
	unsigned long flags;

	TRACE("");
	spin_lock_irqsave(&d->lock, flags);
	d->channel[chan->chan_id].terminate = 1;
	spin_unlock_irqrestore(&d->lock, flags);

	tasklet_schedule(&d->task);

	return 0;
}

static void rcm_issue_pending(struct dma_chan *chan)
{
	struct rcm_audio_dma *d = to_dmac(chan);
	unsigned int mask, origmask;
	unsigned long flags;
	unsigned int ena = 0;

	spin_lock_irqsave(&d->lock, flags);
	regmap_read(d->regmap, RCM_DMA_ENA_REG, &ena);
	regmap_read(d->regmap, RCM_DMA_INT_MASK, &origmask);
	if (d->mode == RCM_DMA_MODE_SPDIF)
		ena |= RCM_DMA_ENA_SPDIF;
	else
		ena &= ~RCM_DMA_ENA_SPDIF;

	ena |= ((RCM_DMA_CHANNEL0_ACTIVE | RCM_DMA_CHANNEL0_SW)
		<< (chan->chan_id & 0x3));
	mask = RCM_DMA_INT_CH0_WR_END << (chan->chan_id & 0x3);

	// enable DMA
	regmap_write(d->regmap, RCM_DMA_INT, mask); // cleanup int
	regmap_write(d->regmap, RCM_DMA_ENA_REG, ena); // allow channel
	regmap_write(d->regmap, RCM_DMA_INT_MASK,
		     origmask | mask); // channel interrupt

	// mark as active
	d->channel[chan->chan_id].active = 1;

	spin_unlock_irqrestore(&d->lock, flags);
	TRACE("%d, ena:%08X, dma:%08X", chan->chan_id, ena, origmask | mask);
}

static struct dma_chan *of_dma_rcm_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	int count = dma_spec->args_count;
	struct rcm_audio_dma *d = ofdma->of_dma_data;
	unsigned int chan_id;

	TRACE("");

	if (!d)
		return NULL;

	if (count != 1)
		return NULL;

	chan_id = dma_spec->args[0];
	if (chan_id > 4)
		return NULL;

	return dma_get_slave_channel(&d->channel[chan_id].chan);
}

static void rcm_dma_tasklet(unsigned long arg)
{
	struct rcm_audio_dma *d = (struct rcm_audio_dma *)arg;
	int i;
	unsigned int terminate_mask = 0;
	unsigned int ena_mask = 0;
	unsigned int wait_mask = 0;
	unsigned long flags;
	unsigned int dma_int = d->dma_int;

	spin_lock_irqsave(&d->lock, flags);
	d->dma_int = 0;

	TRACE("%08X", dma_int);

	if(d->mode == RCM_DMA_MODE_SPDIF)
	{
		if (dma_int & RCM_DMA_INT_CH0_WR_END) {
			struct dma_async_tx_descriptor *desc =
				&d->channel[4].desc;
			struct dmaengine_desc_callback cb;
			dmaengine_desc_get_callback(desc, &cb);
			if (dmaengine_desc_callback_valid(&cb)) {
				TRACE("invoke spdif");
				spin_unlock_irqrestore(&d->lock, flags);
				dmaengine_desc_callback_invoke(&cb, NULL);
				spin_lock_irqsave(&d->lock, flags);
			}
		}
	}
	else
	{
		// proceed channels callbacks
		for (i = 0; i < 4; i++) {
			if (dma_int & (RCM_DMA_INT_CH0_WR_END << (i&0x3))) {
				struct dma_async_tx_descriptor *desc =
					&d->channel[i].desc;
				struct dmaengine_desc_callback cb;
				dmaengine_desc_get_callback(desc, &cb);
				if (dmaengine_desc_callback_valid(&cb)) {
					TRACE("invoke %d", i);
					spin_unlock_irqrestore(&d->lock, flags);
					dmaengine_desc_callback_invoke(&cb, NULL);
					spin_lock_irqsave(&d->lock, flags);
				}
			}
		}
	}
	// proceed termitation
	for (i = 0; i < 5; i++) {
		if (d->channel[i].terminate == 1) {
			d->channel[i].terminate = 0;
			d->channel[i].active = 0;

			terminate_mask |= (RCM_DMA_CHANNEL0_SW << (i&0x3));
			ena_mask |= (RCM_DMA_CHANNEL0_ACTIVE << (i&0x3));
			wait_mask |= (0xff << (i&0x3));
		}
	}
	if (terminate_mask) {
		unsigned int v;
		unsigned int num_active = 0;
		// Wait for transfer finish
		TRACE("terminate %08X", terminate_mask);
		for (i = 0; i < 5; i++)
			if (d->channel[i].active)
				num_active++;

		if (num_active == 0) {
			TRACE("set mode to undefined");
			d->mode = RCM_DMA_MODE_UNDEFINED;
		}

		regmap_update_bits(d->regmap, RCM_DMA_ENA_REG, terminate_mask,
				   0);
		regmap_read_poll_timeout(d->regmap, RCM_DMA_BUF_STATUS, v,
					 (v & wait_mask) == 0, 0, 500000);
		regmap_update_bits(d->regmap, RCM_DMA_ENA_REG, ena_mask, 0);
	}
	spin_unlock_irqrestore(&d->lock, flags);
}

static int rcm_audio_dma_probe(struct platform_device *pdev)
{
	struct rcm_audio_dma *d;
	void __iomem *base;
	int ret, irq, i;
	struct resource *res;

	// hardware able to use only first Gb for transfers
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	platform_set_drvdata(pdev, d);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	d->regmap =
		devm_regmap_init_mmio(&pdev->dev, base, &rcm_dma_regmap_config);
	if (IS_ERR(d->regmap))
		return PTR_ERR(d->regmap);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get interrupt property");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, irq, rcm_dma_interrupt_handler, 0,
			       pdev->name, d);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return -ENODEV;
	};

	regmap_write(d->regmap, RCM_DMA_ENA_REG, 0x8000);
	regmap_write(d->regmap, RCM_DMA_ENA_REG, 0);

	INIT_LIST_HEAD(&d->slave.channels);
	spin_lock_init(&d->lock);

	for (i = 0; i < 5; i++) {
		d->channel[i].dmac = d;
		d->channel[i].terminate = 0;
		d->channel[i].chan.private = pdev->dev.of_node;
		d->channel[i].chan.device = &d->slave;
		list_add_tail(&d->channel[i].chan.device_node,
			      &d->slave.channels);
	}

	dma_cap_set(DMA_CYCLIC, d->slave.cap_mask);
	d->slave.dev = &pdev->dev;
	d->dma_int = 0;
	d->slave.device_prep_dma_cyclic = rcm_prep_dma_cyclic;
	d->slave.device_tx_status = rcm_tx_status;
	d->slave.device_config = rcm_config;
	d->slave.device_terminate_all = rcm_terminate_all;
	d->slave.device_issue_pending = rcm_issue_pending;
	d->slave.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	d->slave.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	d->slave.directions = BIT(DMA_MEM_TO_DEV);
	d->slave.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	d->slave.max_burst = 1;

	ret = dma_async_device_register(&d->slave);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		ret = of_dma_controller_register(pdev->dev.of_node,
						 of_dma_rcm_xlate, d);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to register DMA to the generic DT DMA helpers\n");
		}
	}

	tasklet_init(&d->task, rcm_dma_tasklet, (unsigned long)d);
	dev_info(&pdev->dev, "initialized\n");

	return 0;
}

static int rcm_audio_dma_remove(struct platform_device *pdev)
{
	struct rcm_audio_dma *d = platform_get_drvdata(pdev);

	tasklet_kill(&d->task);

	return 0;
}

static const struct of_device_id rcm_audio_dma_dt_ids[] = {
	{
		.compatible = "rcm,audio-dma",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_audio_dma_dt_ids);

static struct platform_driver rcm_audio_dma_driver = {
	.driver =
		{
			.name = "rcm-audio-dma",
			.of_match_table = rcm_audio_dma_dt_ids,
		},
	.probe = rcm_audio_dma_probe,
	.remove = rcm_audio_dma_remove,
};

module_platform_driver(rcm_audio_dma_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM Audio DMA driver");
MODULE_LICENSE("GPL");
