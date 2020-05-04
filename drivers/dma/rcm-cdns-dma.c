// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/of_dma.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>

#include "dmaengine.h"

#define RCM_CDNS_DMA_CH0_CTRL    0x00
#define RCM_CDNS_DMA_CH0_SP_LO   0x04
#define RCM_CDNS_DMA_CH0_SP_UP   0x08
#define RCM_CDNS_DMA_CH0_ATTR_LO 0x0C
#define RCM_CDNS_DMA_CH0_ATTR_UP 0x10

#define RCM_CDNS_DMA_INT         0xA0
#define RCM_CDNS_DMA_INT_ENA     0xA4
#define RCM_CDNS_DMA_INT_DIS     0xA8

#define RCM_CDNS_DMA_VER         0xF8
#define RCM_CDNS_DMA_CONF        0xFC

#define RCM_CDNS_DMA_CH_AREA     0x14

#define RCM_CDNS_DMA_MAX_CHANNELS 8

#define RCM_CDNS_DMA_DESC_NUM     1024

#define RCM_CDNS_DMA_MAX_BULK_SIZE 0x1000000

struct rcm_cdns_dma_channel {
	struct dma_chan           chan;
	struct dma_slave_config   config;
	struct rcm_cdns_dma_desc *active_desc;
	int                       completed;
	int                       error;
};

struct rcm_cdns_dma_hw_desc;

struct rcm_cdns_dma_transfer {
	struct rcm_cdns_dma_hw_desc* descs;
	int                          cnt;
};

struct rcm_cdns_dma_desc {
	struct dma_async_tx_descriptor desc;
	struct list_head               node;

	struct rcm_cdns_dma_transfer*  transfers;
	int                            num_transfer;
	int                            cnt_transfers;

	struct scatterlist            *sgl;
	unsigned int                   sg_len;
	enum dma_transfer_direction    dir;
};

struct rcm_cdns_dma {
	spinlock_t                  lock;
	struct regmap              *reg;
	struct regmap              *desc_area;
	phys_addr_t                 addr_desc_area;
	size_t                      size_desc_area;
	struct dma_device           dmadev;
	struct tasklet_struct       task;

	struct rcm_cdns_dma_channel channels[RCM_CDNS_DMA_MAX_CHANNELS];

	struct rcm_cdns_dma_desc*   descs;

	u32                         cnt_desc;
	int                         cnt_channels;

	struct list_head            pending_list;
	struct list_head            free_list;

	int                         cnt_desc_per_chan;
};

#pragma pack(push, 1)

struct rcm_cdns_dma_version {
	u8 min_ver;
	u8 maj_ver;
	u16 reserve;
};

struct rcm_cdns_dma_config {
	u32 num_channels   : 4;
	u32 num_partitions : 4;
	u32 partition_size : 4;
	u32 sys_aw_gt_32   : 12;
	u32 sys_tw_gt_32   : 13;
	u32 ext_aw_gt_32   : 14;
	u32 ext_tw_gt_32   : 15;
	u32 reserve        : 16;
};

struct rcm_cdns_dma_hw_desc {
	u32 addr_axi;

	u32 ax_prot             : 3;
	u32 ax_cache            : 4;
	u32 ax_lock             : 1;
	u32 reserve1            : 24;

	u64 addr_pcie;

	u32 no_snoop            : 1;
	u32 relaxed_ordering    : 1;
	u32 id_based_ordering   : 1;
	u32 pcie_transfer_class : 3;
	u32 reserve2            : 3;
	u32 use_requester_id    : 1;
	u32 requester_id        : 16;
	u32 reserve3            : 6;

	u32 length              : 24;
	u32 interrupt           : 1;
	u32 continuity          : 2;
	u32 reserve4            : 2;
	u32 continue_ll         : 1;
	u32 reserve5            : 2;

	u8  bresp_presp         : 2;
	u8  err_integrity_axi   : 1;
	u8  reserve6            : 5;

	u8  bus_status          : 4;
	u8  reserve7            : 4;

	u8  completed           : 1;
	u8  pcie_incomplete     : 1;
	u8  axi_incomplete      : 1;
	u8  err_integrity_ram   : 1;
	u8  err_descriptor      : 1;
	u8  overflow            : 1;
	u8  underflow           : 1;
	u8  not_empty           : 1;

	u8  reserve8;

	u32 addr_next_desc;
};

#pragma pack(pop)

static const struct regmap_config rcm_cdns_dma_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_CDNS_DMA_CONF,
};

static inline struct rcm_cdns_dma_channel *to_chan(struct dma_chan *ch)
{
	if (!ch)
		return NULL;

	return container_of(ch, struct rcm_cdns_dma_channel, chan);
}

static inline struct rcm_cdns_dma *to_data(struct dma_device *dmadev)
{
	if (!dmadev)
		return NULL;

	return container_of(dmadev, struct rcm_cdns_dma, dmadev);
}

static inline struct rcm_cdns_dma_desc *
to_desc(struct dma_async_tx_descriptor *desc)
{
	if (!desc)
		return NULL;

	return container_of(desc, struct rcm_cdns_dma_desc, desc);
}

static struct rcm_cdns_dma_desc *
rcm_cdns_dma_get_descriptor(struct rcm_cdns_dma *data)
{
	struct rcm_cdns_dma_desc *desc;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (data->cnt_desc == 0)
		return NULL;

	desc = list_first_entry(&data->free_list, struct rcm_cdns_dma_desc, 
	                        node);
	list_del(&desc->node);
	--data->cnt_desc;

	spin_unlock_irqrestore(&data->lock, flags);

	return desc;
}

static void rcm_cdns_dma_free_descriptor(struct rcm_cdns_dma *data,
                                         struct rcm_cdns_dma_desc *desc)
{
	int i;

	if (desc->transfers) {
		for (i = 0; i < desc->cnt_transfers; ++i) {
			if (desc->transfers[i].descs) {
				kfree(desc->transfers[i].descs);
			}
		}

		kfree(desc->transfers);
		desc->transfers = NULL;
	}

	data->cnt_desc++;
	list_add_tail(&desc->node, &data->free_list);
}

static void rcm_cdns_dma_complete_descriptor(struct rcm_cdns_dma *data,
                                             struct rcm_cdns_dma_desc *desc,
                                             int status,
                                             struct dmaengine_desc_callback* cb)
{
	if (status) 
		dma_cookie_complete(&desc->desc);

	dmaengine_desc_get_callback(&desc->desc, cb);

	rcm_cdns_dma_free_descriptor(data, desc);
}

static void rcm_cdns_dma_process_desc(struct rcm_cdns_dma *data,
                                      struct rcm_cdns_dma_desc *desc)
{
	int chan_id = desc->desc.chan->chan_id;
	int i;
	int ch_offset = chan_id * RCM_CDNS_DMA_CH_AREA;
	u32 ctrl;
	struct rcm_cdns_dma_transfer *transfer = 
		&desc->transfers[desc->num_transfer];
	phys_addr_t addr_desc_area = data->addr_desc_area +
	                             chan_id * data->cnt_desc_per_chan * 
	                             sizeof(struct rcm_cdns_dma_hw_desc);

	for(i = 0; i < transfer->cnt; ++i)
	{
		struct rcm_cdns_dma_hw_desc *hw_desc = &transfer->descs[i];

		pr_debug("%s: hw_desc(0x%llX): addr_axi = 0x%08X, "
		         "addr_pcie = 0x%llX, len = %u, type = %u, "
		         "continue = %u\n",
		         __func__, (u64)(addr_desc_area + i * sizeof(*hw_desc)),
		         hw_desc->addr_axi, hw_desc->addr_pcie,
		         hw_desc->length, hw_desc->continuity,
		         hw_desc->continue_ll);

		hw_desc->addr_next_desc = addr_desc_area + 
		                          (i + 1) * sizeof(*hw_desc);

		regmap_bulk_write(data->desc_area,
		                  addr_desc_area - data->addr_desc_area +
		                  i * sizeof(*hw_desc),
		                  hw_desc, sizeof(*hw_desc) / 4);
	}

	ctrl = (desc->dir == DMA_MEM_TO_DEV) ? 0x3 : 0x1;

	regmap_write(data->reg, RCM_CDNS_DMA_INT_ENA,
	             BIT(chan_id) | BIT(chan_id + 8));
	regmap_write(data->reg, ch_offset + RCM_CDNS_DMA_CH0_SP_LO, 
	             (u32)addr_desc_area);
//	regmap_write(data->reg, ch_offset + RCM_CDNS_DMA_CH0_SP_UP, 
//	             (u32)(addr_desc_area >> 32));
	regmap_write(data->reg, ch_offset + RCM_CDNS_DMA_CH0_CTRL,  ctrl);
}

static void rcm_cdns_dma_start_transfer(struct rcm_cdns_dma *data)
{
	struct rcm_cdns_dma_desc *desc;
	struct rcm_cdns_dma_channel *ch;

	desc = list_first_entry_or_null(&data->pending_list,
	                                struct rcm_cdns_dma_desc, node);
	if (!desc)
		return;

	ch = to_chan(desc->desc.chan);

	if (ch->active_desc)
		return;

	list_del(&desc->node);

	ch->active_desc = desc;

	rcm_cdns_dma_process_desc(data, desc);
}

static irqreturn_t rcm_cdns_dma_irq_handler(int irq, void *dev_id)
{
	struct rcm_cdns_dma *data = dev_id;
	u32 dma_int;
	int chan_id;
	int need_tasklet = 0;

	regmap_read(data->reg, RCM_CDNS_DMA_INT, &dma_int);

	pr_debug("%s: dma_int = 0x%08X\n", __func__, dma_int);

	if (!dma_int) 
		return IRQ_NONE;

	regmap_write(data->reg, RCM_CDNS_DMA_INT, dma_int);

	spin_lock(&data->lock);

	for (chan_id = 0; chan_id < data->cnt_channels; ++chan_id) {
		struct rcm_cdns_dma_channel *ch = &data->channels[chan_id];
		u32 completed = dma_int & BIT(chan_id);
		u32 error     = dma_int & BIT(chan_id + 8);

		if ((completed) || (error)) {
			if (!ch->active_desc) {
				pr_warn("%s: Interrupt for channel without "
				        "active descriptor\n", __func__);
				continue;
			}

			++ch->active_desc->num_transfer;

			if (error) {
				ch->error = 1;
				need_tasklet = 1;
			} else if (ch->active_desc->num_transfer == 
			           ch->active_desc->cnt_transfers) {
				ch->completed = 1;
				need_tasklet = 1;
			} else {
				rcm_cdns_dma_process_desc(data,
				                          ch->active_desc);
			}
		}
	}

	rcm_cdns_dma_start_transfer(data);

	spin_unlock(&data->lock);

	if (need_tasklet)
		tasklet_schedule(&data->task);

	return IRQ_HANDLED;
}

static void rcm_cdns_dma_tasklet(unsigned long arg)
{
	unsigned long flags;
	struct rcm_cdns_dma *data = (struct rcm_cdns_dma *)arg;
	int chan_id;

	spin_lock_irqsave(&data->lock, flags);

	for (chan_id = 0; chan_id < data->cnt_channels; ++chan_id) {
		struct rcm_cdns_dma_channel *ch = &data->channels[chan_id];

		if ((ch->completed) || (ch->error)) {
			int status = (ch->error) ? 0 : 1;
			struct dmaengine_desc_callback cb = {
				.callback = NULL,
				.callback_param = NULL,
				.callback_result = NULL
			};

			rcm_cdns_dma_complete_descriptor(data, ch->active_desc,
			                                 status, &cb);

			ch->active_desc = NULL;
			ch->completed = 0;
			ch->error = 0;

			if (dmaengine_desc_callback_valid(&cb)) {
				spin_unlock_irqrestore(&data->lock, flags);

				dmaengine_desc_callback_invoke(&cb, NULL);

				spin_lock_irqsave(&data->lock, flags);
			}

			rcm_cdns_dma_start_transfer(data);
		}
	}

	spin_unlock_irqrestore(&data->lock, flags);
}

static int rcm_cdns_dma_config(struct dma_chan *chan,
                               struct dma_slave_config *config)
{
	struct rcm_cdns_dma_channel *ch = to_chan(chan);

	pr_debug("%s: chan id = %d, src_addr = 0x%llx, dst_addr = 0x%llx\n",
	         __func__, chan->chan_id, (u64)config->src_addr,
	         (u64)config->dst_addr);

	memcpy(&ch->config, config, sizeof(*config));

	return 0;
}

static void rcm_cdns_dma_issue_pending(struct dma_chan *chan)
{
	struct rcm_cdns_dma *data = to_data(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	rcm_cdns_dma_start_transfer(data);
	spin_unlock_irqrestore(&data->lock, flags);
}

static int
rcm_cdns_dma_prepare_descriptor(struct rcm_cdns_dma_desc *desc,
                                struct scatterlist *sgl, unsigned int sg_len,
                                enum dma_transfer_direction dir)
{
	struct rcm_cdns_dma_channel *ch = to_chan(desc->desc.chan);
	struct rcm_cdns_dma *data = to_data(ch->chan.device);
	struct scatterlist* sg;
	int i;
	struct rcm_cdns_dma_hw_desc *hw_desc;
	int cnt_descs = 0;
	int num_trans;
	int num_desc;
	int cnt_trans;
	u64 addr_pcie = (dir == DMA_MEM_TO_DEV) ? ch->config.dst_addr : 
	                                          ch->config.src_addr;

	for_each_sg(sgl, sg, sg_len, i) {
		u32 len  = sg_dma_len(sg);

		while (len > RCM_CDNS_DMA_MAX_BULK_SIZE) {
			++cnt_descs;
			len -= RCM_CDNS_DMA_MAX_BULK_SIZE;
		}

		++cnt_descs;
	}

	cnt_trans = (cnt_descs + data->cnt_desc_per_chan - 1) / 
	            data->cnt_desc_per_chan;

	desc->transfers = kmalloc_array(cnt_trans,
	                                sizeof(struct rcm_cdns_dma_transfer),
	                                GFP_KERNEL);
	if (!desc->transfers) {
		pr_err("%s: Failed to allocate transfer's array (%d elems).\n",
		       __func__, cnt_trans);
		return -ENOMEM;
	}

	memset(desc->transfers, 0,
	       sizeof(struct rcm_cdns_dma_transfer) * cnt_trans);

	desc->cnt_transfers = cnt_trans;
	desc->num_transfer = 0;

	num_trans = 0;
	num_desc = 0;

	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t addr = sg_dma_address(sg);
		u32        len  = sg_dma_len(sg);

		while (len > 0) {
			struct rcm_cdns_dma_transfer *trans = 
				&desc->transfers[num_trans];

			if (!trans->descs) {
				trans->cnt = (num_trans + 1 == cnt_trans) ? 
				             cnt_descs % data->cnt_desc_per_chan : 
				             data->cnt_desc_per_chan;
				trans->descs = kmalloc_array(trans->cnt,
				                             sizeof(*hw_desc),
				                             GFP_KERNEL);

				if (!trans->descs) {
					pr_err("%s: Failed to allocate "
					       "descriptors array (%d elems).\n",
					       __func__, trans->cnt);
					return -ENOMEM;
				}
			}


			hw_desc = &(trans->descs[num_desc]);

			memset(hw_desc, 0, sizeof(*hw_desc));

			hw_desc->addr_axi  = addr;
			hw_desc->addr_pcie = addr_pcie;

			if (len > RCM_CDNS_DMA_MAX_BULK_SIZE) {
				hw_desc->length = 0;
				len -= RCM_CDNS_DMA_MAX_BULK_SIZE;
				addr += RCM_CDNS_DMA_MAX_BULK_SIZE;
				addr_pcie += RCM_CDNS_DMA_MAX_BULK_SIZE;
			} else {
				addr_pcie += len;
				hw_desc->length = len;
				len = 0;
			}
			hw_desc->continue_ll = 1;

			++num_desc;

			if (num_desc == trans->cnt) {
				hw_desc->interrupt = 1;
				hw_desc->continue_ll = 0;

				num_desc = 0;
				++num_trans;
			}
		}
	}

	desc->dir = dir;

	return 0;
}

static struct dma_async_tx_descriptor *
rcm_cdns_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
                           unsigned int sg_len, enum dma_transfer_direction dir,
                           unsigned long flags, void *context)
{
	struct rcm_cdns_dma *data = to_data(chan->device);
	struct rcm_cdns_dma_desc *desc;

	pr_debug("%s: chan id = %d, dir = %u, sg_len = %u\n",
	         __func__, chan->chan_id, (u32)dir, sg_len);

	desc = rcm_cdns_dma_get_descriptor(data);

	if (!desc) {
		pr_warn("%s: no available descriptors\n", __func__);
		return NULL;
	}

	desc->desc.chan  = chan;
	desc->desc.flags = flags;

	if (rcm_cdns_dma_prepare_descriptor(desc, sgl, sg_len, dir)) {
		rcm_cdns_dma_free_descriptor(data, desc);
		return NULL;
	}

	return &desc->desc;
}

static dma_cookie_t rcm_cdns_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct rcm_cdns_dma *data = to_data(tx->chan->device);
	struct rcm_cdns_dma_desc *desc = to_desc(tx);
	dma_cookie_t cookie;
	unsigned long flags;

	pr_debug("%s: chan id = %d\n", __func__, tx->chan->chan_id);

	spin_lock_irqsave(&data->lock, flags);

	cookie = dma_cookie_assign(tx);
	list_add_tail(&desc->node, &data->pending_list);

	spin_unlock_irqrestore(&data->lock, flags);

	return cookie;
}

static void rcm_cdns_dma_free_desc_list(struct rcm_cdns_dma *data,
                                        struct list_head *list)
{
	struct rcm_cdns_dma_desc *desc;
	struct rcm_cdns_dma_desc *next;

	list_for_each_entry_safe(desc, next, list, node) {
		list_del(&desc->node);
		rcm_cdns_dma_free_descriptor(data, desc);
	}
}

static void rcm_cdns_dma_free_descriptors(struct rcm_cdns_dma *data)
{
	struct dma_chan *chan;
	struct dma_chan *next;

	list_for_each_entry_safe(chan, next, &data->dmadev.channels,
	                         device_node) {
		struct rcm_cdns_dma_channel *ch = to_chan(chan);
		if (ch->active_desc) {
			rcm_cdns_dma_free_descriptor(data, ch->active_desc);
			ch->active_desc = NULL;
		}
	}

	rcm_cdns_dma_free_desc_list(data, &data->pending_list);
}

static void rcm_cdns_dma_free_chan_resources(struct dma_chan *chan)
{
	struct rcm_cdns_dma *data = to_data(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	rcm_cdns_dma_free_descriptors(data);
	spin_unlock_irqrestore(&data->lock, flags);
	kfree(data->descs);
}

static int rcm_cdns_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct rcm_cdns_dma *data = to_data(chan->device);
	struct rcm_cdns_dma_desc *desc;
	int i;

	data->descs = kcalloc(RCM_CDNS_DMA_DESC_NUM, sizeof(*desc), GFP_NOWAIT);
	if (!data->descs) {
		pr_err("%s: failed to allocate descriptors array.\n", __func__);
		return -ENOMEM;
	}

	data->cnt_desc = RCM_CDNS_DMA_DESC_NUM;

	INIT_LIST_HEAD(&data->free_list);

	for (i = 0; i < RCM_CDNS_DMA_DESC_NUM; i++) {
		desc = data->descs + i;
		dma_async_tx_descriptor_init(&desc->desc, NULL);
		desc->desc.tx_submit = rcm_cdns_dma_tx_submit;
		list_add_tail(&desc->node, &data->free_list);
	}

	return RCM_CDNS_DMA_DESC_NUM;
}


static int rcm_cdns_dma_probe(struct platform_device *pdev)
{
	struct rcm_cdns_dma *data;
	struct dma_device *dma_dev;
	void __iomem *base;
	int ret, irq, i;
	struct resource *res;
	struct rcm_cdns_dma_version ver;
	struct rcm_cdns_dma_config conf;
	struct device_node* node_desc_area;
	struct resource res_desc_area;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, data);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init_mmio(&pdev->dev, base, 
	                                  &rcm_cdns_dma_regmap_config);
	if (IS_ERR(data->reg)) {
		dev_err(&pdev->dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	regmap_read(data->reg, RCM_CDNS_DMA_VER, (u32*)&ver);
	regmap_read(data->reg, RCM_CDNS_DMA_CONF, (u32*)&conf);

	pr_debug("%s: v%u.%u, %u channels, %u partitions, part_size = %u\n",
	         __func__, ver.maj_ver, ver.min_ver, conf.num_channels,
	         conf.num_partitions, conf.partition_size);

	node_desc_area = of_parse_phandle(pdev->dev.of_node, "desc_area", 0);
	if (!node_desc_area) {
		dev_err(&pdev->dev, "failed to find desc_area node\n");
		return -EFAULT;
	}
	if (of_address_to_resource(node_desc_area, 0, &res_desc_area)) {
		dev_err(&pdev->dev, "failed to get desc_area address\n");
		return -EFAULT;
	}

	data->addr_desc_area = res_desc_area.start;
	data->size_desc_area = resource_size(&res_desc_area);
	data->cnt_desc_per_chan = data->size_desc_area /
	                          sizeof(struct rcm_cdns_dma_hw_desc) /
	                          conf.num_channels;

	data->desc_area = syscon_node_to_regmap(node_desc_area);

	pr_debug("%s: desc_area(0x%llX:%llX), %d descriptors per channel\n",
	         __func__, (u64)data->addr_desc_area, (u64)data->size_desc_area,
	         data->cnt_desc_per_chan);

	data->cnt_channels = conf.num_channels;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get interrupt property\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, irq, rcm_cdns_dma_irq_handler,
	                       0, pdev->name, data);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return -ENODEV;
	};

	spin_lock_init(&data->lock);

	INIT_LIST_HEAD(&data->pending_list);
	INIT_LIST_HEAD(&data->free_list);

	dma_dev = &data->dmadev;

	/* Set DMA capabilities */
	dma_cap_zero(dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);

	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_dev->max_burst = 1;

	/* Init DMA link list */
	INIT_LIST_HEAD(&dma_dev->channels);

	for (i = 0; i < conf.num_channels; i++) {
		dma_cookie_init(&data->channels[i].chan);
		data->channels[i].chan.device = dma_dev;
		data->channels[i].active_desc = NULL;
		data->channels[i].completed = 0;
		data->channels[i].error = 0;
		list_add_tail(&data->channels[i].chan.device_node,
		              &dma_dev->channels);
	}

	/* Set base routines */
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_config = rcm_cdns_dma_config;
	dma_dev->device_issue_pending = rcm_cdns_dma_issue_pending;
	dma_dev->device_prep_slave_sg = rcm_cdns_dma_prep_slave_sg;

	dma_dev->device_alloc_chan_resources = rcm_cdns_dma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = rcm_cdns_dma_free_chan_resources;

	dma_dev->dev = &pdev->dev;

	/* Set DMA mask to 32 bits */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "unable to set coherent mask to 32\n");
		return ret;
	}

	ret = dma_async_device_register(dma_dev);
	if (ret)
	{
		dev_err(&pdev->dev, "failed to register dma device\n");
		return ret;
	}

	if (pdev->dev.of_node) {
		ret = of_dma_controller_register(pdev->dev.of_node,
		                                 of_dma_xlate_by_chan_id,
		                                 dma_dev);
		if (ret) {
			dev_err(&pdev->dev,
			        "unable to register DMA to "
			        "the generic DT DMA helpers\n");
		}
	}

	tasklet_init(&data->task, rcm_cdns_dma_tasklet, (unsigned long)data);

	dev_info(&pdev->dev, "initialized\n");

	return 0;
}

static int rcm_cdns_dma_remove(struct platform_device *pdev)
{
	struct rcm_cdns_dma *data = platform_get_drvdata(pdev);

	tasklet_kill(&data->task);

	return 0;
}

static const struct of_device_id rcm_cdns_dma_dt_ids[] = {
	{
		.compatible = "rcm,cdns-dma",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_cdns_dma_dt_ids);

static struct platform_driver rcm_cdns_dma_driver = {
	.driver =
		{
			.name = "rcm-cdns-dma",
			.of_match_table = rcm_cdns_dma_dt_ids,
		},
	.probe = rcm_cdns_dma_probe,
	.remove = rcm_cdns_dma_remove,
};

module_platform_driver(rcm_cdns_dma_driver);

MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_DESCRIPTION("RCM Cadence PCIE DMA driver");
MODULE_LICENSE("GPL");
