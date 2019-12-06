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

#define RCM_DMA_CHANNEL0_ACTIVE 0x02
#define RCM_DMA_ENA_SPDIF 0x10
#define RCM_DMA_CHANNEL0_SW 0x200

static const struct regmap_config rcm_dma_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_DMA_BUF_STATUS,
};

struct rcm_audio_dma {
	/* DMA-Engine Channel */
	struct dma_chan chan;

	/* To protect channel manipulation */
	spinlock_t lock;

	// number of real channels used - for multi channel audio
	int channels;

	struct regmap *regmap;
	struct dma_device slave;

	phys_addr_t fifo_addr;
	dma_addr_t fifo_dma;

	struct dma_async_tx_descriptor desc[2];
	int curr_desc;

	int desc_state[2];
	int desc_last_state[2];

	struct tasklet_struct task;
	int base_len;

	int terminate;
};

static inline struct rcm_audio_dma *to_dmac(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return container_of(ch, struct rcm_audio_dma, chan);
}

static irqreturn_t rcm_dma_interrupt_handler(int irq, void *data)
{
	struct rcm_audio_dma *d = (struct rcm_audio_dma *)data;
	unsigned long flags;
	unsigned int dma_int = 0;

	regmap_read(d->regmap, RCM_DMA_INT, &dma_int);
	regmap_write(d->regmap, RCM_DMA_INT, dma_int);  //	cleanup ints
//	printk("TRACE: rcm_dma_interrupt_handler start");

//	spin_lock_irqsave(&d->lock, flags);
//	d->desc_state[d->curr_desc] = DMA_COMPLETE;
//	d->curr_desc = 1 - d->curr_desc;

//	spin_unlock_irqrestore(&d->lock, flags);

	tasklet_schedule(&d->task);

	printk("TRACE: rcm_dma_interrupt_handler %08X", dma_int);

	return IRQ_HANDLED;
}

//static int rcm_alloc_chan_resources(struct dma_chan *chan)
//{
//	struct rcm_audio_dma *d = to_dmac(chan);

//	printk("TRACE: rcm_alloc_chan_resources");

//	return 1;
//}

static void rcm_free_chan_resources(struct dma_chan *chan)
{
//	struct rcm_audio_dma *d = to_dmac(chan);

	printk("TRACE: rcm_free_chan_resources");
}

static dma_cookie_t rcm_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct rcm_audio_dma *d = to_dmac(tx->chan);
	unsigned long flags;
	dma_cookie_t cookie;
	
	printk("TRACE: rcm_tx_submit");

	spin_lock_irqsave(&d->lock, flags);

	dma_cookie_assign(&d->desc[0]);
	cookie = dma_cookie_assign(&d->desc[1]);

	d->desc[0].callback = tx->callback;
	d->desc[0].callback_param = tx->callback_param;
	//tx->callback = 0;

	spin_unlock_irqrestore(&d->lock, flags);

	return cookie;
}

static struct dma_async_tx_descriptor *
	rcm_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t dma_addr,
			    size_t len, size_t period_len,
			    enum dma_transfer_direction direction,
			    unsigned long flags)
{
	struct rcm_audio_dma *d = to_dmac(chan);
	int i;
	unsigned long dma_offset = 0x40000000;		// to do fix on upper level
	d->base_len = period_len / d->channels;

	printk("TRACE: rcm_prep_dma_cyclic, len: %d, period_len: %d, flags: %d, dma_addr: %llx", (int) len,
	       (int) period_len, (int)  flags, dma_addr);

	regmap_write(d->regmap, RCM_DMA_ENA_REG, 0x8000); 
	regmap_write(d->regmap, RCM_DMA_ENA_REG, 0); 

	// we always have only 2 DMA descriptios for each hw channel.
	// call hw channels should works simultaniously, so we should prepare all of its at the same time
	
	// first descriptor is 1
	d->curr_desc = 1;

	d->desc[0].chan = chan;
	d->desc[0].cookie = 0;
	d->desc[0].tx_submit = rcm_tx_submit;
	async_tx_ack(&d->desc[0]);	
	dma_async_tx_descriptor_init(&d->desc[0], &d->chan);
	d->desc[0].flags = flags;

	d->desc[1].chan = chan;
	d->desc[1].cookie = 0;
	d->desc[1].tx_submit = rcm_tx_submit;
	async_tx_ack(&d->desc[1]);	
	dma_async_tx_descriptor_init(&d->desc[1], &d->chan);
	d->desc[1].flags = flags;

	// setup addresses for each descriptors based on numer of channels
	for (i = 0; i < d->channels; i++) {
		u32 page0_start = (u32) dma_addr + period_len + i * d->base_len + dma_offset;
		u32 page1_start = (u32) dma_addr + i * d->base_len + dma_offset;
		u32 page0_end =
			(u32) dma_addr + period_len + (i + 1) * d->base_len + dma_offset;
		u32 page1_end = (u32) dma_addr + (i + 1) * d->base_len + dma_offset;

		printk("TRACE rcm_prep_dma_cyclic: channel %i, %08X, %08X, %08X, %08X",
		       i, (u32) page1_start, (u32) page1_end, (u32) page0_start, (u32) page0_end);

		// write descriptors
		regmap_write(d->regmap, RCM_DMA_CH1_BASE0 + i * 2 * 4,
			     page0_start);
		regmap_write(d->regmap, RCM_DMA_CH1_BASE1 + i * 2 * 4,
			     page1_start);
		regmap_write(d->regmap, RCM_DMA_CH1_END0 + i * 2 * 4,
			     page0_end);
		regmap_write(d->regmap, RCM_DMA_CH1_END1 + i * 2 * 4,
			     page1_end);

		// size
		regmap_write(d->regmap, RCM_DMA_CH1_TRW + i * 4, d->base_len/4);

		// slave base
		regmap_write(d->regmap, RCM_DMA_SLV1_BASE + i * 4,
			     (u32)d->fifo_addr + i * 4);

		// slave fifo size
		regmap_write(d->regmap, RCM_DMA_SLV1_BSIZE + i * 4,
			     31); // todo read from channel info

	}
	// regmap_write(d->regmap, RCM_DMA_SLV_OVRH, 20); 

	return &d->desc[1];
}

static enum dma_status rcm_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	enum dma_status ret;
	struct rcm_audio_dma *d = to_dmac(chan);
	unsigned int residual = 0;
	unsigned long flags;

	ret = dma_cookie_status(chan, cookie, txstate);

	if (!txstate)
		return ret;
	
	//if(cookie == d->desc[d->curr_desc].cookie)
	//	ret = DMA_COMPLETE;

	if (ret == DMA_COMPLETE)
		goto out;

	spin_lock_irqsave(&d->lock, flags);

	//regmap_read(d->regmap, RCM_DMA_BUF_STATUS, &residual);
	//residual >>= 8;		// second channel temp
	if(d->curr_desc == 1)
		residual += d->base_len;

	spin_unlock_irqrestore(&d->lock, flags);

out:
//	printk("TRACE: rcm_tx_status %d %d", cookie, residual);
	dma_set_residue(txstate, residual);

	return ret;
}

static int rcm_config(struct dma_chan *chan,
		      struct dma_slave_config *slave_config)
{
	struct rcm_audio_dma *d = to_dmac(chan);

	printk("TRACE: rcm_config slave_config->dst_addr = %llx, slave_config->dst_maxburst = %d",
	       slave_config->dst_addr, slave_config->dst_maxburst);

	d->channels = slave_config->dst_maxburst;
	d->fifo_addr = slave_config->dst_addr;

	return 0;
}

static int rcm_terminate_all(struct dma_chan *chan)
{
	struct rcm_audio_dma *d = to_dmac(chan);

	printk("TRACE: rcm_terminate_all");
	d->terminate = 1;
	tasklet_schedule(&d->task);

	return 0;
}

static int rcm_pause(struct dma_chan *chan)
{
//	struct rcm_audio_dma *d = to_dmac(chan);

	printk("TRACE: rcm_pause");

	return 0;
}

static void rcm_issue_pending(struct dma_chan *chan)
{
	struct rcm_audio_dma *d = to_dmac(chan);
	int i;
	unsigned int mask;
	unsigned int ena = 0;	// to do apply SPDIF swithing RCM_DMA_ENA_SPDIF

	printk("TRACE: rcm_issue_pending");

	// enable DMA

	for(i = 0; i < d->channels; i++)
		ena |= ((RCM_DMA_CHANNEL0_ACTIVE | RCM_DMA_CHANNEL0_SW) << i);
	// set I2S mode temporary

	d->desc_last_state[0] = d->desc_last_state[1] = DMA_IN_PROGRESS;
	d->desc_state[0] = d->desc_state[1] = DMA_IN_PROGRESS;

	regmap_write(d->regmap, RCM_DMA_INT, -1);  		//	cleanup int

	mask = (2) << (d->channels-1);

	regmap_write(d->regmap, RCM_DMA_ENA_REG, ena); 

	regmap_write(d->regmap, RCM_DMA_INT_MASK, mask);  // allow all inerrupts for a while // zero channel0 interrupt
}

static struct dma_chan *of_dma_rcm_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	int count = dma_spec->args_count;
	struct rcm_audio_dma *d = ofdma->of_dma_data;
	unsigned int chan_id;

	printk("TRACE: of_dma_rcm_xlate");

	if (!d)
		return NULL;

	if (count != 1)
		return NULL;

	chan_id = dma_spec->args[0];
	if (chan_id >= 4)
		return NULL;

	return dma_get_slave_channel(&d->chan);
}

static void rcm_dma_tasklet(unsigned long arg)
{
	struct rcm_audio_dma *d = (struct rcm_audio_dma *)arg;
	if(d->terminate)
	{
		unsigned int v;
		d->terminate = 0;
		// Wait for transfer finish
		regmap_write_bits(d->regmap, RCM_DMA_ENA_REG, 0xF00, 0); 
		regmap_read_poll_timeout(d->regmap, RCM_DMA_BUF_STATUS, v, v==0, 0, 1000000);
		regmap_write_bits(d->regmap, RCM_DMA_ENA_REG, 0xF, 0); 
	}
	else
	{
		struct dma_async_tx_descriptor *desc = &d->desc[1];
		struct dmaengine_desc_callback cb;
		dmaengine_desc_get_callback(desc, &cb);
		if (dmaengine_desc_callback_valid(&cb)) {
			dmaengine_desc_callback_invoke(&cb, NULL);
		}
	}
}


static int rcm_audio_dma_probe(struct platform_device *pdev)
{
	struct rcm_audio_dma *d;
	void __iomem *base;
	int ret, irq;
	struct resource *res;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)); //???
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

	INIT_LIST_HEAD(&d->slave.channels);
	d->chan.private = pdev->dev.of_node;
	spin_lock_init(&d->lock);
	d->chan.device = &d->slave;
	list_add_tail(&d->chan.device_node, &d->slave.channels);

	dma_cap_set(DMA_CYCLIC, d->slave.cap_mask);
	d->slave.dev = &pdev->dev;
	//d->slave.device_alloc_chan_resources = rcm_alloc_chan_resources;
	d->slave.device_free_chan_resources = rcm_free_chan_resources;
	d->slave.device_prep_dma_cyclic = rcm_prep_dma_cyclic;
	d->slave.device_tx_status = rcm_tx_status;
	d->slave.device_config = rcm_config;
	d->slave.device_pause = rcm_pause;
	d->slave.device_terminate_all = rcm_terminate_all;
	d->slave.device_issue_pending = rcm_issue_pending;
	d->slave.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	d->slave.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	d->slave.directions = BIT(DMA_MEM_TO_DEV);
	d->slave.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	d->slave.max_burst = 4;

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

	d->terminate = 0;
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
