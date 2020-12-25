// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include "../../../dma/rcm-mdma.h"

#include "rcm_mgeth.h"

#define RCM_MGETH_MDMA_RX_POOL_SIZE 256
#define RCM_MGETH_MDMA_TX_POOL_SIZE 128

#define RCM_MGETH_MDMA_MAX_TRANSACTION 0x3FFC
#define RCM_MGETH_MDMA_LEN_MASK        0x3FFF

#define RCM_MGETH_DMA_BUFF_SIZE (((RCM_MGETH_MAX_FRAME_SIZE + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE)

struct rcm_mgeth_dma_buff
{
	dma_addr_t                dma_addr;
	void                     *buff;
#ifdef CONFIG_BASIS_PLATFORM
	u32                       ep_addr;
#endif
};

struct rcm_mgeth_mdma_req {
	struct list_head          node;
	struct sk_buff           *skb;
	void                     *buff;
	dma_addr_t                dma_addr;
#ifdef CONFIG_BASIS_PLATFORM
	u32                       ep_addr;
#endif
	u32                       size;
	rcm_mgeth_dma_tx_callback callback;
	bool                      completed;
	bool                      err;
};

struct rcm_mgeth_dma_chan {
	struct net_device           *netdev;
	struct channel_regs __iomem *regs;

	spinlock_t                   lock;

	enum dma_data_direction      dir;

	int                          num;
	char                         name[8];

	struct mdma_desc_pool        desc_pool;

	struct rcm_mgeth_dma_buff   *buffs;
	unsigned                     next_buff;
	unsigned                     first_buff;
	unsigned                     cnt_buffs;
	unsigned                     rest_buffs;

	bool                         single_alloc;

	struct
	{
		struct rcm_mgeth_mdma_req *requests;
		struct list_head           free_list;
		struct list_head           pending_list;
		struct list_head           active_list;
		struct list_head           done_list;
		struct tasklet_struct      tasklet;
	} tx;

	struct 
	{
		unsigned                  pos;
		unsigned                  num_buff;
		rcm_mgeth_dma_rx_callback callback;
	} rx;
};

struct net_device *rcm_mgeth_dma_get_net_device(struct rcm_mgeth_dma_chan *chan)
{
	return chan->netdev;
}

int rcm_mgeth_dma_get_number(struct rcm_mgeth_dma_chan *chan)
{
	return chan->num;
}

bool rcm_mgeth_dma_busy(struct rcm_mgeth_dma_chan *chan)
{
	unsigned long irqflags;
	bool busy;

	if (chan->dir == DMA_FROM_DEVICE)
		return false;

	spin_lock_irqsave(&chan->lock, irqflags);

	busy = list_empty(&chan->tx.free_list);

	spin_unlock_irqrestore(&chan->lock, irqflags);

	return busy;
}

static void rcm_mgeth_dma_start_tx(struct rcm_mgeth_dma_chan *chan)
{
	struct rcm_mgeth_mdma_req *req;
	dma_addr_t dma_addr;
	unsigned pos = 0;

	if (!list_empty(&chan->tx.active_list))
		return;

	if (list_empty(&chan->tx.pending_list))
		return;

	list_for_each_entry(req, &chan->tx.pending_list, node) {
		bool last = list_is_last(&req->node, &chan->tx.pending_list);

#ifdef CONFIG_BASIS_PLATFORM
		pos += mdma_desc_pool_fill(&chan->desc_pool, pos, req->ep_addr,
		                           req->size, last, last);
#else
		pos += mdma_desc_pool_fill(&chan->desc_pool, pos, req->dma_addr,
		                           req->size, last, last);
#endif
	}

	list_splice_tail_init(&chan->tx.pending_list, &chan->tx.active_list);

	dma_addr = mdma_desc_pool_get_addr(&chan->desc_pool, 0);

	writel(dma_addr, &chan->regs->desc_addr);
	rcm_mgeth_dma_enable_irq(chan);
	writel(1, &chan->regs->enable);
}

void rcm_mgeth_dma_irq(struct rcm_mgeth_dma_chan *chan)
{
	u32 status;

	status = readl(&chan->regs->status);

	if (status == 0) {
		netdev_warn(chan->netdev, "[%s] Wrong interrupt.\n",
		            chan->name);
		return;
	}

	if (chan->dir == DMA_TO_DEVICE) {
		struct rcm_mgeth_mdma_req *req;
		bool need_tasklet = false;

		spin_lock(&chan->lock);

		if (list_empty(&chan->tx.active_list)) {
			netdev_warn(chan->netdev,
			            "[%s] Interrupt without active descriptors\n",
			            chan->name);
		} else {
			list_for_each_entry(req, &chan->tx.active_list, node) {
				if ((status & MDMA_IRQ_INT_DESC) == 0)
					req->err = true;
				else 
					req->completed = true;
			}
			need_tasklet = true;

			list_splice_tail_init(&chan->tx.active_list, 
			                      &chan->tx.done_list);
		}

		rcm_mgeth_dma_start_tx(chan);

		spin_unlock(&chan->lock);

		if (need_tasklet)
			tasklet_schedule(&chan->tx.tasklet);
	}
}

static void rcm_mgeth_dma_tx_tasklet(unsigned long data)
{
	struct rcm_mgeth_dma_chan *chan = (struct rcm_mgeth_dma_chan *)data;
	unsigned long irqflags;
	struct rcm_mgeth_mdma_req *req;

	spin_lock_irqsave(&chan->lock, irqflags);

	list_for_each_entry(req, &chan->tx.done_list, node) {
		if (req->err) {
//			rcm_mgeth_reset(data);
		}

		spin_unlock_irqrestore(&chan->lock, irqflags);

		req->callback(chan, req->err, req->size);

		spin_lock_irqsave(&chan->lock, irqflags);

		if (req->skb) {
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_unmap_single(chan->netdev->dev.parent,
			                              req->dma_addr,
			                              req->ep_addr,
			                              req->skb->len,
			                              DMA_TO_DEVICE);
#else
			dma_unmap_single(chan->netdev->dev.parent,
			                 req->dma_addr, req->skb->len,
			                 DMA_TO_DEVICE);
#endif
			dev_kfree_skb(req->skb);
			req->skb = NULL;
		} else {
			chan->first_buff = (chan->first_buff + 1) % 
			                   chan->cnt_buffs;
			++chan->rest_buffs;
		}
	}

	list_splice_tail_init(&chan->tx.done_list, 
	                      &chan->tx.free_list);

	rcm_mgeth_dma_start_tx(chan);

	spin_unlock_irqrestore(&chan->lock, irqflags);
}

void rcm_mgeth_dma_enable_irq(struct rcm_mgeth_dma_chan *chan)
{
	writel(MDMA_IRQ_SENS_MASK, &chan->regs->irq_mask);
}

void rcm_mgeth_dma_disable_irq(struct rcm_mgeth_dma_chan *chan)
{
	writel(0, &chan->regs->irq_mask);
}

bool rcm_mgeth_dma_submit_tx(struct rcm_mgeth_dma_chan *chan,
                             struct sk_buff *skb,
                             rcm_mgeth_dma_tx_callback callback)
{
	unsigned long irqflags;
	struct rcm_mgeth_mdma_req *req;
	struct rcm_mgeth_dma_buff *buff;
	bool copy_skb = (skb_shinfo(skb)->nr_frags != 0) ||
	                ((offset_in_page(skb->data) & 0xF) != 0) ||
	                (skb->len + offset_in_page(skb->data) > PAGE_SIZE) ||
	                (skb->len < RCM_MGETH_MIN_FRAME_SIZE);

#ifdef CONFIG_BASIS_PLATFORM
	copy_skb = true;
#endif

	if (chan->dir == DMA_FROM_DEVICE) {
		netdev_err(chan->netdev,
		           "[%s] Attemp to submit packet to RX-channel.\n",
		           chan->name);
		return false;
	}

	spin_lock_irqsave(&chan->lock, irqflags);

	req = list_first_entry_or_null(&chan->tx.free_list,
	                               struct rcm_mgeth_mdma_req, node);
	if (!req) {
		netdev_err(chan->netdev,
		           "[%s] Attemp to submit packet to busy channel.\n",
		           chan->name);
		spin_unlock_irqrestore(&chan->lock, irqflags);
		return false;
	}

	if (copy_skb) {
		if (chan->rest_buffs == 0) {
			netdev_err(chan->netdev,
			           "[%s] No buffer for operation.\n",
			           chan->name);
			spin_unlock_irqrestore(&chan->lock, irqflags);
			return false;
		}

		buff = &chan->buffs[chan->next_buff];
		chan->next_buff = (chan->next_buff + 1) % chan->cnt_buffs;
		--chan->rest_buffs;

		req->skb = NULL;
		req->buff = buff->buff;
		req->dma_addr = buff->dma_addr;
#ifdef CONFIG_BASIS_PLATFORM
		req->ep_addr = buff->ep_addr;
#endif
		req->size = skb->len;
		skb_copy_bits(skb, 0, req->buff, skb->len);
		if (skb->len < RCM_MGETH_MIN_FRAME_SIZE) {
			memset(req->buff + skb->len, 0,
			       RCM_MGETH_MIN_FRAME_SIZE - skb->len);
			req->size = RCM_MGETH_MIN_FRAME_SIZE;
		}
		dev_kfree_skb(skb);
	} else {
		req->skb = skb;
		req->buff = skb->data;
		req->dma_addr = 
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_map_single(chan->netdev->dev.parent,
			                            skb->data, skb->len,
			                            DMA_TO_DEVICE,
			                            &req->ep_addr);
#else
			dma_map_single(chan->netdev->dev.parent, skb->data,
			               skb->len, DMA_TO_DEVICE);
#endif

		if (req->dma_addr == DMA_MAPPING_ERROR) {
			netdev_err(chan->netdev,
			           "[%s] Failed to map packet for DMA.\n",
			           chan->name);
			spin_unlock_irqrestore(&chan->lock, irqflags);
			return false;
		}

		req->size = skb->len;
	}

	req->callback = callback;

	list_move_tail(&req->node, &chan->tx.pending_list);

	rcm_mgeth_dma_start_tx(chan);

	spin_unlock_irqrestore(&chan->lock, irqflags);

	return true;
}

int rcm_mgeth_dma_start_rx(struct rcm_mgeth_dma_chan *chan,
                           rcm_mgeth_dma_rx_callback callback)
{
	unsigned long irqflags;
	dma_addr_t dma_addr;

	if (chan->dir == DMA_TO_DEVICE) {
		netdev_err(chan->netdev,
		           "[%s] Attemp to start receiving on TX-channel.\n",
		           chan->name);
		return -EINVAL;
	}

	spin_lock_irqsave(&chan->lock, irqflags);

	chan->rx.pos = 0;
	chan->rx.num_buff = 0;
	chan->rx.callback = callback;

	dma_addr = mdma_desc_pool_get_addr(&chan->desc_pool, 0);

	spin_unlock_irqrestore(&chan->lock, irqflags);

	writel(dma_addr, &chan->regs->desc_addr);
	rcm_mgeth_dma_enable_irq(chan);
	writel(1, &chan->regs->enable);

	netdev_dbg(chan->netdev, "[%s] RX started.\n", chan->name);

	return 0;
}

void rcm_mgeth_dma_stop_rx(struct rcm_mgeth_dma_chan *chan)
{
	if (chan->dir == DMA_TO_DEVICE) {
		netdev_warn(chan->netdev,
		           "[%s] Attemp to stop receiving TX-channel.\n",
		           chan->name);
		return;
	}

	writel(1, &chan->regs->cancel);
}

int rcm_mgeth_dma_rx_poll(struct rcm_mgeth_dma_chan *chan, int budget)
{
	unsigned long irqflags;
	int work_done = 0;
	struct mdma_desc_long_ll *desc;
	u32 size;
	u32 status;
	void *buff;
	u32 enabled;

	if (chan->dir == DMA_TO_DEVICE) {
		netdev_err(chan->netdev,
		           "[%s] Attemp to poll TX-channel.\n", chan->name);
		return 0;
	}

	spin_lock_irqsave(&chan->lock, irqflags);

	enabled = readl(&chan->regs->enable);

	if (budget > work_done)
		readl(&chan->regs->status);

	while (budget > work_done) {
		desc = mdma_desc_pool_get_desc(&chan->desc_pool, chan->rx.pos);
		if ((desc->flags_length & MDMA_BD_OWN) == 0) {
			status = readl(&chan->regs->status);

			if (status == 0)
				break;
			else
				continue;
		}

		if (desc->flags_length & MDMA_BD_LINK) {
			desc->flags_length = MDMA_BD_LINK;
			chan->rx.pos = (chan->rx.pos + 1) % chan->desc_pool.size;
			continue;
		}

		buff = chan->buffs[chan->rx.num_buff].buff;
		size = (desc->flags_length & RCM_MGETH_MDMA_LEN_MASK);

		spin_unlock_irqrestore(&chan->lock, irqflags);

		chan->rx.callback(chan, buff, size);

		spin_lock_irqsave(&chan->lock, irqflags);

		mdma_desc_pool_fill(&chan->desc_pool, chan->rx.pos,
#ifdef CONFIG_BASIS_PLATFORM
		                    chan->buffs[chan->rx.num_buff].ep_addr,
#else
		                    chan->buffs[chan->rx.num_buff].dma_addr,
#endif
		                    RCM_MGETH_MAX_FRAME_SIZE,
		                    false, true);
		chan->rx.pos = (chan->rx.pos + 1) % chan->desc_pool.size;
		chan->rx.num_buff = (chan->rx.num_buff + 1) % chan->cnt_buffs;
		++work_done;
	}

	if (budget > work_done) {
		if (enabled == 0) {
			dma_addr_t dma_addr = 
				mdma_desc_pool_get_addr(&chan->desc_pool,
				                        chan->rx.pos);

			netdev_info(chan->netdev, "[%s] Restart RX\n", chan->name);

			writel(dma_addr, &chan->regs->desc_addr);
			writel(1, &chan->regs->enable);
		}
	}

	spin_unlock_irqrestore(&chan->lock, irqflags);

	netdev_dbg(chan->netdev, "[%s] RX-pool (work_done = %d)\n",
	           chan->name, work_done);

	return work_done;
}

struct rcm_mgeth_dma_chan *rcm_mgeth_dma_chan_create(struct net_device *netdev,
                                                     char *name,
                                                     void __iomem *regs,
                                                     int num,
                                                     enum dma_data_direction dir)
{
	struct rcm_mgeth_dma_chan *chan;
	int ret;
	int i;
	unsigned size_pool = (dir == DMA_TO_DEVICE) ?
	                     RCM_MGETH_MDMA_TX_POOL_SIZE :
	                     RCM_MGETH_MDMA_RX_POOL_SIZE;

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		netdev_err(netdev,
		           "[%s] Failed to allocate DMA-channel data.\n",
		           name);
		return NULL;
	}

	strncpy(chan->name, name, sizeof(chan->name));
	chan->regs = regs;
	chan->dir = dir;
	chan->num = num;
	chan->netdev = netdev;

	spin_lock_init(&chan->lock);

	ret = mdma_desc_pool_alloc(&chan->desc_pool,
	                           size_pool,
	                           netdev->dev.parent, chan->name,
	                           RCM_MGETH_MDMA_MAX_TRANSACTION,
	                           RCM_MGETH_MDMA_LEN_MASK, true);

	if (ret) {
		netdev_err(netdev,
		           "[%s] Failed to allocate descriptors pool.\n", name);
		rcm_mgeth_dma_chan_remove(chan);
		return NULL;
	}

	chan->cnt_buffs = chan->desc_pool.size - chan->desc_pool.cnt_chunks;
	chan->rest_buffs = chan->cnt_buffs;
	chan->next_buff = 0;
	chan->first_buff = 0;

	chan->buffs = kcalloc(chan->cnt_buffs,
	                      sizeof(struct rcm_mgeth_dma_buff),
	                      GFP_KERNEL);
	if (!chan->buffs) {
		netdev_err(netdev,
		           "[%s] Failed to allocate DMA-buffers.\n", name);
		rcm_mgeth_dma_chan_remove(chan);
		return NULL;
	}

	chan->buffs[0].buff = 
#ifdef CONFIG_BASIS_PLATFORM
		basis_device_dma_alloc_coherent(
			netdev->dev.parent,
			RCM_MGETH_DMA_BUFF_SIZE * chan->cnt_buffs,
			&chan->buffs[0].dma_addr,
			&chan->buffs[0].ep_addr,
			GFP_KERNEL);
#else
		dma_alloc_coherent(netdev->dev.parent,
		                   RCM_MGETH_DMA_BUFF_SIZE * chan->cnt_buffs,
		                   &chan->buffs[0].dma_addr,
		                   GFP_KERNEL);
#endif
	chan->single_alloc = (chan->buffs[0].buff != NULL);

	for (i = 0; i < chan->cnt_buffs; ++i) {
		if (chan->single_alloc) {
			chan->buffs[i].buff     = chan->buffs[0].buff + 
			                          RCM_MGETH_DMA_BUFF_SIZE * i;
			chan->buffs[i].dma_addr = chan->buffs[0].dma_addr + 
			                          RCM_MGETH_DMA_BUFF_SIZE * i;
#ifdef CONFIG_BASIS_PLATFORM
			chan->buffs[i].ep_addr  = chan->buffs[0].ep_addr + 
			                          RCM_MGETH_DMA_BUFF_SIZE * i;
#endif
			continue;
		}

		chan->buffs[i].buff = 
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_alloc_coherent(netdev->dev.parent,
			                                RCM_MGETH_DMA_BUFF_SIZE,
			                                &chan->buffs[i].dma_addr,
			                                &chan->buffs[i].ep_addr,
			                                GFP_KERNEL);
#else
			dma_alloc_coherent(netdev->dev.parent,
			                   RCM_MGETH_DMA_BUFF_SIZE,
			                   &chan->buffs[i].dma_addr,
			                   GFP_KERNEL);
#endif

		if (!chan->buffs[i].buff) {
			netdev_err(netdev,
			           "[%s] Failed to allocate DMA-buffer.\n",
			           name);
			rcm_mgeth_dma_chan_remove(chan);
			return NULL;
		}
	}

	if (dir == DMA_TO_DEVICE) {
		tasklet_init(&chan->tx.tasklet,
		             rcm_mgeth_dma_tx_tasklet, (ulong)chan);

		chan->tx.requests = kcalloc(chan->cnt_buffs,
		                            sizeof(struct rcm_mgeth_mdma_req),
		                            GFP_KERNEL);
		if (!chan->tx.requests) {
			netdev_err(netdev,
			           "[%s] Failed to allocate TX-requests array.\n",
			           name);
			rcm_mgeth_dma_chan_remove(chan);
			return NULL;
		}

		INIT_LIST_HEAD(&chan->tx.free_list);
		INIT_LIST_HEAD(&chan->tx.pending_list);
		INIT_LIST_HEAD(&chan->tx.active_list);
		INIT_LIST_HEAD(&chan->tx.done_list);

		for (i = 0; i < chan->cnt_buffs; i++) {
			list_add_tail(&chan->tx.requests[i].node,
			              &chan->tx.free_list);
		}
	} else {
		unsigned pos = 0;

		for (i = 0; i < chan->cnt_buffs; ++i) {
			pos += mdma_desc_pool_fill(&chan->desc_pool, pos,
#ifdef CONFIG_BASIS_PLATFORM
			                           chan->buffs[i].ep_addr,
#else
			                           chan->buffs[i].dma_addr,
#endif
			                           RCM_MGETH_MAX_FRAME_SIZE,
			                           false, true);
		}

		chan->rx.pos = 0;
	}

	return chan;
}

void rcm_mgeth_dma_chan_remove(struct rcm_mgeth_dma_chan *chan)
{
	int i;

	tasklet_kill(&chan->tx.tasklet);
	if (chan->tx.requests)
		kfree(chan->tx.requests);
	if (chan->buffs) {
		if (chan->single_alloc) {
			if (chan->buffs[0].buff) {
#ifdef CONFIG_BASIS_PLATFORM
				basis_device_dma_free_coherent(
					chan->netdev->dev.parent,
					RCM_MGETH_DMA_BUFF_SIZE * chan->cnt_buffs,
					chan->buffs[0].buff,
					chan->buffs[0].dma_addr,
					chan->buffs[0].ep_addr);
#else
				dma_free_coherent(
					chan->netdev->dev.parent, 
					RCM_MGETH_DMA_BUFF_SIZE * chan->cnt_buffs, 
					chan->buffs[0].buff,
					chan->buffs[0].dma_addr);
#endif
			}
		} else {
			for (i = 0; i < chan->cnt_buffs; ++i) {
				if (chan->buffs[i].buff) {
#ifdef CONFIG_BASIS_PLATFORM
					basis_device_dma_free_coherent(
						chan->netdev->dev.parent,
						RCM_MGETH_DMA_BUFF_SIZE,
						chan->buffs[i].buff,
						chan->buffs[i].dma_addr,
						chan->buffs[i].ep_addr);
#else
					dma_free_coherent(
						chan->netdev->dev.parent, 
						RCM_MGETH_DMA_BUFF_SIZE, 
						chan->buffs[i].buff,
						chan->buffs[i].dma_addr);
#endif
				}
			}
		}

		kfree(chan->buffs);
	}
	mdma_desc_pool_free(&chan->desc_pool);
	kfree(chan);
}
