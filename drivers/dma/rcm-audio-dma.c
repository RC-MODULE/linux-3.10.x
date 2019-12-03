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

#define RCM_DMA_ENA_REG   0x00
#define RCM_DMA_CH0_BASE0 0x04
#define RCM_DMA_CH0_BASE1 0x08
#define RCM_DMA_CH1_BASE0 0x0C
#define RCM_DMA_CH1_BASE1 0x10
#define RCM_DMA_CH2_BASE0 0x14
#define RCM_DMA_CH2_BASE1 0x18
#define RCM_DMA_CH3_BASE0 0x1C
#define RCM_DMA_CH3_BASE1 0x20
#define RCM_DMA_CH0_END0  0x24
#define RCM_DMA_CH0_END1  0x28
#define RCM_DMA_CH1_END0  0x2C
#define RCM_DMA_CH1_END1  0x30
#define RCM_DMA_CH2_END0  0x34
#define RCM_DMA_CH2_END1  0x38
#define RCM_DMA_CH3_END0  0x3C
#define RCM_DMA_CH3_END1  0x40
#define RCM_DMA_SLV0_BASE 0x44
#define RCM_DMA_SLV1_BASE 0x48
#define RCM_DMA_SLV2_BASE 0x4C
#define RCM_DMA_SLV3_BASE 0x50
#define RCM_DMA_CH0_TRW   0x54
#define RCM_DMA_CH1_TRW   0x58
#define RCM_DMA_CH2_TRW   0x5C
#define RCM_DMA_CH3_TRW   0x60
#define RCM_DMA_SLV_OVRH  0x64
#define RCM_DMA_SLV0_BSIZE 0x68
#define RCM_DMA_SLV1_BSIZE 0x6C
#define RCM_DMA_SLV2_BSIZE 0x70
#define RCM_DMA_SLV3_BSIZE 0x74
#define RCM_DMA_AXI_PARAM  0x78
#define RCM_DMA_INT_MASK   0x7C
#define RCM_DMA_INT        0x80
#define RCM_DMA_BUF_STATUS 0x84

static const struct regmap_config rcm_dma_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_DMA_BUF_STATUS,
};


struct rcm_dma_chan {

	/* DMA-Engine Channel */
	struct dma_chan chan;
	struct rcm_audio_dma *dmac;

	/* To protect channel manipulation */
	spinlock_t lock;

	/* For D-to-M and M-to-D channels */
	int burst_sz; /* the peripheral fifo width */
	int burst_len; /* the number of burst */
	phys_addr_t fifo_addr;
	/* DMA-mapped view of the FIFO; may differ if an IOMMU is present */
	dma_addr_t fifo_dma;

};

struct rcm_audio_dma
{
	struct regmap *regmap;
	struct dma_device	slave;
    // we have fixed number of channels
    struct rcm_dma_chan channels[4];
};

static inline struct rcm_dma_chan *
to_pchan(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return container_of(ch, struct rcm_dma_chan, chan);
}

static irqreturn_t rcm_dma_interrupt_handler(int irq, void *data)
{
	struct rcm_audio_dma *d = (struct rcm_audio_dma *)data;

    printk("TRACE: rcm_dma_interrupt_handler");

	return IRQ_HANDLED;
}

static int rcm_alloc_chan_resources(struct dma_chan *chan)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_alloc_chan_resources");

	return 1;
}

static void rcm_free_chan_resources(struct dma_chan *chan)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_free_chan_resources");
}

static struct dma_async_tx_descriptor *rcm_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_prep_dma_cyclic");

    return NULL;
}

static enum dma_status
rcm_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
		 struct dma_tx_state *txstate)
{
    enum dma_status ret;
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;
	unsigned int residual = 0;
	unsigned long flags;

    printk("TRACE: rcm_tx_status");


	ret = dma_cookie_status(chan, cookie, txstate);

	if (!txstate)
		return ret;

	if (ret == DMA_COMPLETE)
		goto out;


	spin_lock_irqsave(&pch->lock, flags);




	spin_unlock_irqrestore(&pch->lock, flags);
out:
	dma_set_residue(txstate, residual);

    return ret;
}

static int rcm_config(struct dma_chan *chan,
			struct dma_slave_config *slave_config)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_config");

	return 0;
}

static int rcm_terminate_all(struct dma_chan *chan)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_terminate_all");


	return 0;
}

static int rcm_pause(struct dma_chan *chan)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_pause");

	return 0;
}

static void rcm_issue_pending(struct dma_chan *chan)
{
	struct rcm_dma_chan *pch = to_pchan(chan);
	struct rcm_audio_dma *d = pch->dmac;

    printk("TRACE: rcm_issue_pending");

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

	return dma_get_slave_channel(&d->channels[chan_id].chan);
}

static int rcm_audio_dma_probe(struct platform_device *pdev)
{
	struct rcm_audio_dma *d;
	void __iomem *base;
    int ret, irq, i;
	struct resource *res;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));  //???
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

	d->regmap = devm_regmap_init_mmio(&pdev->dev, base,
		&rcm_dma_regmap_config);
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
		return  -ENODEV;
	};

	INIT_LIST_HEAD(&d->slave.channels);

	for (i = 0; i < 4; i++) {
        struct rcm_dma_chan* ch;
		ch = &d->channels[i];
        ch->chan.private = pdev->dev.of_node;
        spin_lock_init(&ch->lock);
        ch->dmac = d;
        ch->chan.device = &d->slave;        
        list_add_tail(&ch->chan.device_node, &d->slave.channels);
    }

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
	d->slave.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	d->slave.max_burst = 32;

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


	dev_info(&pdev->dev, "initialized\n");

    return 0;
}

static int rcm_audio_dma_remove(struct platform_device *pdev)
{

    return 0;
}


static const struct of_device_id rcm_audio_dma_dt_ids[] = {
	{ .compatible = "rcm,audio-dma", },
	{}
};
MODULE_DEVICE_TABLE(of, rcm_audio_dma_dt_ids);


static struct platform_driver rcm_audio_dma_driver = {
	.driver		= {
		.name	= "rcm-audio-dma",
		.of_match_table = rcm_audio_dma_dt_ids,
	},
	.probe		= rcm_audio_dma_probe,
	.remove		= rcm_audio_dma_remove,
};

module_platform_driver(rcm_audio_dma_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM Audio DMA driver");
MODULE_LICENSE("GPL");
