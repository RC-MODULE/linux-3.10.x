// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
//#define DEBUG

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/phy/phy.h>
#include <linux/of_mdio.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>

#define RCM_MGETH_ID            0x0000
#define RCM_MGETH_VERSION       0x0004
#define RCM_MGETH_SW_RST        0x0008
#define RCM_MGETH_GLOBAL_STATUS 0x000C

#define RCM_MGETH_STATUS        0x0010
#define RCM_MGETH_IRQ_MASK      0x0018
#define RCM_MGETH_CONTROL       0x001C

#define RCM_MGETH_LEN_MASK_CH(ch)    (0x0020 + (ch) * 4)

#define RCM_MGETH_TX_DELAY_TIMER(ch) (0x0030 + (ch) * 4)

#define RCM_MGETH_RX_ETH_MASK_VALUE(val) ((val) * 4)

#define RCM_MGETH_RX_ETH_MASK_ACTIVE(val) (0x80 + (val) * 4)

#define RCM_MGETH_CNT_BUFFS_PER_CHAN 32
#define RCM_MGETH_MAX_DMA_CHANNELS 4

#define RCM_MGETH_MIN_FRAME_SIZE 60
#define RCM_MGETH_MAX_FRAME_SIZE 10240
#define RCM_MGETH_DMA_BUFF_SIZE (((RCM_MGETH_MAX_FRAME_SIZE + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE)

#define RCM_MGETH_RX_TIMEOUT 200

struct rcm_mgeth_dma_buff {
	struct rcm_mgeth_dma*           dma;
	dma_cookie_t                    cookie;
	struct sk_buff                 *skb;
	struct scatterlist              sg;
	struct completion               dma_completion;
	size_t                          residue;
	bool                            error;
};

struct rcm_mgeth_dma {
	struct dma_chan          *chan;
	struct rcm_mgeth_dma_buff buffs[RCM_MGETH_CNT_BUFFS_PER_CHAN];
	int                       next_buff;
	int                       first_buff;
	int                       cnt_buffs;
	spinlock_t                lock;
	dma_addr_t                dma_addr;
	void*                     buff;

	struct task_struct*       thread;

	int                       ch_num;
};

struct rcm_mgeth_data {
	struct device      *dev;
	struct net_device  *netdev;
	struct regmap      *reg;
	struct regmap      *mask[RCM_MGETH_MAX_DMA_CHANNELS];

	int                 irq;
	struct irq_domain  *domain;

	struct phy_device  *phydev;
	struct device_node *phynode;
	phy_interface_t     phy_if;

	int speed;
	int duplex;
	int link;

	struct rcm_mgeth_dma tx[RCM_MGETH_MAX_DMA_CHANNELS];
	struct rcm_mgeth_dma rx[RCM_MGETH_MAX_DMA_CHANNELS];
};

static ssize_t rcm_mgeth_show_stats(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct rcm_mgeth_data *data = netdev_priv(netdev);

	u32 aFramesReceivedOK;
	u64 aOctetsReceivedOK;
	u32 ifInUcastPkts;
	u32 ifInMulticastPkts;
	u32 ifInBroadcastPkts;
	u32 aFrameCheckSequenceErrors;
	u32 ifInErrors;
	u32 etherStatsDropEvents;
	u64 etherStatsOctets;
	u32 etherStatsPkts;
	u32 etherStatsUndersizePkts;
	u32 etherStatsOversizePkts;
	u32 etherStatsPkts64Octets;
	u32 etherStatsPkts65to127Octets;
	u32 etherStatsPkts128to255Octets;
	u32 etherStatsPkts256to511Octets;
	u32 etherStatsPkts512to1023Octets;
	u32 etherStatsPkts1024to1518Octets;
	u32 etherStatsPkts1519to10240Octets;
	u32 etherStatsJabbers;
	u32 etherStatsFragments;
	u32 aFramesTransmittedOK;
	u64 aOctetsTransmittedOK;
	u32 ifOutUcastPkts;
	u32 ifOutMulticastPkts;
	u32 ifOutBroadcastPkts;

	regmap_read(data->reg, 0x100, &aFramesReceivedOK);
	regmap_bulk_read(data->reg, 0x104, &aOctetsReceivedOK, 2);
	regmap_read(data->reg, 0x10C, &ifInUcastPkts);
	regmap_read(data->reg, 0x110, &ifInMulticastPkts);
	regmap_read(data->reg, 0x114, &ifInBroadcastPkts);
	regmap_read(data->reg, 0x118, &aFrameCheckSequenceErrors);
	regmap_read(data->reg, 0x11C, &ifInErrors);
	regmap_read(data->reg, 0x120, &etherStatsDropEvents);
	regmap_bulk_read(data->reg, 0x124, &etherStatsOctets, 2);
	regmap_read(data->reg, 0x12C, &etherStatsPkts);
	regmap_read(data->reg, 0x130, &etherStatsUndersizePkts);
	regmap_read(data->reg, 0x134, &etherStatsOversizePkts);
	regmap_read(data->reg, 0x138, &etherStatsPkts64Octets);
	regmap_read(data->reg, 0x13C, &etherStatsPkts65to127Octets);
	regmap_read(data->reg, 0x140, &etherStatsPkts128to255Octets);
	regmap_read(data->reg, 0x144, &etherStatsPkts256to511Octets);
	regmap_read(data->reg, 0x148, &etherStatsPkts512to1023Octets);
	regmap_read(data->reg, 0x14C, &etherStatsPkts1024to1518Octets);
	regmap_read(data->reg, 0x150, &etherStatsPkts1519to10240Octets);
	regmap_read(data->reg, 0x154, &etherStatsJabbers);
	regmap_read(data->reg, 0x158, &etherStatsFragments);
	regmap_read(data->reg, 0x180, &aFramesTransmittedOK);
	regmap_bulk_read(data->reg, 0x184, &aOctetsTransmittedOK, 2);
	regmap_read(data->reg, 0x18C, &ifOutUcastPkts);
	regmap_read(data->reg, 0x190, &ifOutMulticastPkts);
	regmap_read(data->reg, 0x194, &ifOutBroadcastPkts);

	return sprintf(buf, "aFramesReceivedOK               = %u\n"
	                    "aOctetsReceivedOK               = %llu\n"
	                    "ifInUcastPkts                   = %u\n"
	                    "ifInMulticastPkts               = %u\n"
	                    "ifInBroadcastPkts               = %u\n"
	                    "aFrameCheckSequenceErrors       = %u\n"
	                    "ifInErrors                      = %u\n"
	                    "etherStatsDropEvents            = %u\n"
	                    "etherStatsOctets                = %llu\n"
	                    "etherStatsPkts                  = %u\n"
	                    "etherStatsUndersizePkts         = %u\n"
	                    "etherStatsOversizePkts          = %u\n"
	                    "etherStatsPkts64Octets          = %u\n"
	                    "etherStatsPkts65to127Octets     = %u\n"
	                    "etherStatsPkts128to255Octets    = %u\n"
	                    "etherStatsPkts256to511Octets    = %u\n"
	                    "etherStatsPkts512to1023Octets   = %u\n"
	                    "etherStatsPkts1024to1518Octets  = %u\n"
	                    "etherStatsPkts1519to10240Octets = %u\n"
	                    "etherStatsJabbers               = %u\n"
	                    "etherStatsFragments             = %u\n"
	                    "aFramesTransmittedOK            = %u\n"
	                    "aOctetsTransmittedOK            = %llu\n"
	                    "ifOutUcastPkts                  = %u\n"
	                    "ifOutMulticastPkts              = %u\n"
	                    "ifOutBroadcastPkts              = %u\n",
	                    aFramesReceivedOK,
	                    aOctetsReceivedOK,
	                    ifInUcastPkts,
	                    ifInMulticastPkts,
	                    ifInBroadcastPkts,
	                    aFrameCheckSequenceErrors,
	                    ifInErrors,
	                    etherStatsDropEvents,
	                    etherStatsOctets,
	                    etherStatsPkts,
	                    etherStatsUndersizePkts,
	                    etherStatsOversizePkts,
	                    etherStatsPkts64Octets,
	                    etherStatsPkts65to127Octets,
	                    etherStatsPkts128to255Octets,
	                    etherStatsPkts256to511Octets,
	                    etherStatsPkts512to1023Octets,
	                    etherStatsPkts1024to1518Octets,
	                    etherStatsPkts1519to10240Octets,
	                    etherStatsJabbers,
	                    etherStatsFragments,
	                    aFramesTransmittedOK,
	                    aOctetsTransmittedOK,
	                    ifOutUcastPkts,
	                    ifOutMulticastPkts,
	                    ifOutBroadcastPkts);
}

static DEVICE_ATTR(stats, 0400, rcm_mgeth_show_stats, NULL);

static struct rcm_mgeth_dma* rcm_mgeth_get_dma(struct rcm_mgeth_data *data,
                                               bool is_tx)
{
	int i;
	struct rcm_mgeth_dma* dmas = (is_tx) ? data->tx : data->rx;
	unsigned long flags;
	struct rcm_mgeth_dma* dma = NULL;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		if (!dmas[i].chan)
			continue;

		spin_lock_irqsave(&dmas[i].lock, flags);

		if (dmas[i].cnt_buffs < RCM_MGETH_CNT_BUFFS_PER_CHAN)
			dma = &dmas[i];

		spin_unlock_irqrestore(&dmas[i].lock, flags);

		if (dma)
			break;
	}

	return dma;
}

static bool rcm_mgeth_read_mac_addr(struct rcm_mgeth_data *data, u8 *dest)
{
	u8 addr[6];
	u32 len;
	u32 val0;
	u32 val1;

	regmap_read(data->reg, RCM_MGETH_LEN_MASK_CH(0), &len);
	regmap_read(data->mask[0], RCM_MGETH_RX_ETH_MASK_ACTIVE(0), &val0);
	regmap_read(data->mask[0], RCM_MGETH_RX_ETH_MASK_ACTIVE(1), &val1);

	if (((len != 6) && (len != 0)) ||
	    (val0 != 0xFFFFFFFF) || (val1 != 0x0000FFFF))
		return false;

	regmap_read(data->mask[0], RCM_MGETH_RX_ETH_MASK_VALUE(0), &val0);
	regmap_read(data->mask[0], RCM_MGETH_RX_ETH_MASK_VALUE(1), &val1);

	memcpy(addr,     &val0, 4);
	memcpy(&addr[4], &val1, 2);

	if (!is_valid_ether_addr(addr))
		return false;

	memcpy(dest, addr, 6);

	netdev_dbg(data->netdev, "%s: %02X %02X %02X %02X %02X %02X\n",
	           __func__, addr[0], addr[1], addr[2], addr[3], addr[4],
	           addr[5]);

	return true;
}

static int rcm_mgeth_irq_set_type(struct irq_data *d, unsigned int type)
{
	if ((type & IRQ_TYPE_EDGE_BOTH) == 0)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	return 0;
}

static void rcm_mgeth_irq(struct irq_desc *desc)
{
	struct rcm_mgeth_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 status;
	int i;
	unsigned irq = 0;

	chained_irq_enter(chip, desc);

	regmap_read(data->reg, RCM_MGETH_GLOBAL_STATUS, &status);

	netdev_dbg(data->netdev, "Interrupt (status = 0x%08X)\n", status);

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; i++) {
		if (status & BIT(i)) {
			irq = irq_find_mapping(data->domain, i);
			if (irq != 0)
				generic_handle_irq(irq);
		}
		if (status & BIT(i + 16)) {
			irq = irq_find_mapping(data->domain, i + 4);
			if (irq != 0)
				generic_handle_irq(irq);
		}
	}

	chained_irq_exit(chip, desc);
}

static int rcm_mgeth_reset(struct rcm_mgeth_data *data)
{
	int ret = 0;
	u32 val;

	regmap_write(data->reg, RCM_MGETH_SW_RST, 1);

	ret = regmap_read_poll_timeout(data->reg, RCM_MGETH_SW_RST, val,
	                               (val == 0), 1000, 1000000);

	if (ret < 0) {
		netdev_err(data->netdev,
		           "Unable to reset (timeout expired).\n");
		return ret;
	}
	
	return ret;
}

static void rcm_mgeth_adjust_link(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	struct phy_device *phydev = netdev->phydev;
	int new_state = 0;

	netdev_dbg(netdev,
	           "Link state changed: link = %u, autoneg_complete = %u, "
	           "speed = %d, duplex = %d\n",
	           phydev->link, phydev->autoneg_complete, phydev->speed,
	           phydev->duplex);

	if (phydev->link) {
		u32 ctrl;

		regmap_read(data->reg, RCM_MGETH_CONTROL, &ctrl);

		if (data->duplex != phydev->duplex) {
			new_state = 1;
			if (phydev->duplex)
				ctrl |= BIT(0);
			else
				ctrl &= ~BIT(0);
			data->duplex = phydev->duplex;
		}

		if (data->speed != phydev->speed) {
			new_state = 1;
			if (phydev->speed == 10)
				ctrl = (ctrl & ~GENMASK(1,2));
			else if (phydev->speed == 100)
				ctrl = (ctrl & ~GENMASK(1,2)) | (1 << 1);
			else if (phydev->speed == 1000)
				ctrl = (ctrl & ~GENMASK(1,2)) | (1 << 2);
			else
				netdev_warn(netdev,
				            "Speed (%d) is not 10/100/1000!\n",
				            phydev->speed);
			data->speed = phydev->speed;
		}

		regmap_write(data->reg, RCM_MGETH_CONTROL, ctrl);

		if (!data->link)
			new_state = 1;
	} else if (data->link) {
		new_state = 1;
		data->speed = 0;
		data->duplex = -1;
	}

	data->link = phydev->link;

	if (new_state)
		phy_print_status(phydev);
}

static void rcm_mgeth_tx_dma_callback(void *param)
{
	struct rcm_mgeth_dma_buff *buff = param;
	struct rcm_mgeth_dma* dma = buff->dma;
	struct rcm_mgeth_data *data = dma->chan->private;
	unsigned long flags;

	spin_lock_irqsave(&dma->lock, flags);
	
	do {
		if (dma->cnt_buffs == 0) {
			netdev_warn(data->netdev, 
			            "Unexpected DMA callback (TX%u): "
			            "no submitted transfers.\n",
			            dma->ch_num);
			break;
		}

		if (buff != &dma->buffs[dma->first_buff])
		{
			netdev_warn(data->netdev,
			            "Unexpected DMA callback (TX%u): "
			            "unexpected buffer completed.\n",
			            dma->ch_num);
			break;
		}

		if (dma_async_is_tx_complete(dma->chan, buff->cookie,
		                             NULL, NULL) != DMA_COMPLETE) {
			netdev_warn(data->netdev, 
			            "DMA error (TX%u, cookie = 0x%08X).\n",
			            dma->ch_num, buff->cookie);
		} else {
			netdev_dbg(data->netdev, 
			           "DMA operation finished successfully "
			           "(TX%u, %u bytes, cookie = 0x%08X).\n", 
			           dma->ch_num, sg_dma_len(&buff->sg),
			           buff->cookie);
		}

		if (buff->skb) {
			dma_unmap_single(data->dev, sg_dma_address(&buff->sg),
			                 buff->skb->len, DMA_TO_DEVICE);
			dev_kfree_skb(buff->skb);
			buff->skb = NULL;
		}

		dma->first_buff = (dma->first_buff + 1) % 
		                  RCM_MGETH_CNT_BUFFS_PER_CHAN;
		--dma->cnt_buffs;

		if (netif_queue_stopped(data->netdev))
			netif_wake_queue(data->netdev);
	} while (0);

	spin_unlock_irqrestore(&dma->lock, flags);
}

static netdev_tx_t rcm_mgeth_start_xmit(struct sk_buff *skb,
                                        struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	struct rcm_mgeth_dma* dma;
	struct rcm_mgeth_dma_buff *buff;
	struct dma_async_tx_descriptor *desc;
	unsigned long flags;
	bool copy_skb = (skb_shinfo(skb)->nr_frags != 0) ||
	                ((offset_in_page(skb->data) & 0xF) != 0) ||
	                (skb->len + offset_in_page(skb->data) > PAGE_SIZE) ||
	                (skb->len < RCM_MGETH_MIN_FRAME_SIZE);

	if (skb->len > RCM_MGETH_MAX_FRAME_SIZE) {
		netdev_err(netdev, "Frame length is too much: %u bytes.\n",
		           skb->len);
		return NET_XMIT_DROP;
	}

	dma = rcm_mgeth_get_dma(data, true);

	if (!dma) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	spin_lock_irqsave(&dma->lock, flags);

	buff = &dma->buffs[dma->next_buff];

	if (copy_skb) {
		buff->skb = NULL;
		sg_dma_address(&buff->sg) = 
			dma->dma_addr + dma->next_buff * RCM_MGETH_DMA_BUFF_SIZE;
		skb_copy_bits(skb, 0,
		              dma->buff + dma->next_buff * RCM_MGETH_DMA_BUFF_SIZE,
		              skb->len);
		if (skb->len < RCM_MGETH_MIN_FRAME_SIZE)
			memset(dma->buff + skb->len, 0,
			       RCM_MGETH_MIN_FRAME_SIZE - skb->len);
	} else {
		sg_dma_address(&buff->sg) = 
			dma_map_single(data->dev, skb->data, skb_headlen(skb),
			               DMA_TO_DEVICE);
		buff->skb = skb;
	}

	sg_dma_len(&buff->sg) = (skb->len >= RCM_MGETH_MIN_FRAME_SIZE) ? 
	                        skb->len : RCM_MGETH_MIN_FRAME_SIZE;

	netdev_dbg(netdev, "TX%u: DMA operation ready (%u bytes).\n", 
	           dma->ch_num, sg_dma_len(&buff->sg));

	desc = dmaengine_prep_slave_sg(dma->chan, &buff->sg, 1, DMA_MEM_TO_DEV, 
	                               DMA_PREP_INTERRUPT);

	if (!desc) {
		netdev_err(netdev, "Failed prepare DMA TX-operation.\n");
		if (!copy_skb)
			dma_unmap_single(data->dev, sg_dma_address(&buff->sg),
			                 skb->len, DMA_TO_DEVICE);
		spin_unlock_irqrestore(&dma->lock, flags);
		return NETDEV_TX_BUSY;
	}

	desc->callback = rcm_mgeth_tx_dma_callback;
	desc->callback_param = buff;

	netdev_dbg(netdev, "TX%u: DMA operation prepared (%u bytes).\n", 
	           dma->ch_num, sg_dma_len(&buff->sg));

	buff->cookie = dmaengine_submit(desc);
	dma_async_issue_pending(dma->chan); 

	netdev_dbg(netdev, "TX%u: DMA operation submitted "
	                   "(%u bytes, cookie = 0x%08X).\n", 
	           dma->ch_num, sg_dma_len(&buff->sg), buff->cookie);

	if (copy_skb)
		dev_kfree_skb(skb);

	dma->next_buff = (dma->next_buff + 1) % RCM_MGETH_CNT_BUFFS_PER_CHAN;
	++dma->cnt_buffs;

	spin_unlock_irqrestore(&dma->lock, flags);

	return NETDEV_TX_OK;
}

static void rcm_mgeth_rx_dma_callback(void *param)
{
	struct rcm_mgeth_dma_buff *buff = param;
	struct rcm_mgeth_dma* dma = buff->dma;
	struct rcm_mgeth_data *data = dma->chan->private;
	unsigned long flags;
	enum dma_status status;
	struct dma_tx_state state;

	spin_lock_irqsave(&dma->lock, flags);

	status = dmaengine_tx_status(dma->chan, buff->cookie, &state);

	if (status != DMA_COMPLETE) {
		netdev_warn(data->netdev,
		            "DMA error (RX%u, cookie = 0x%08X).\n",
		            dma->ch_num, buff->cookie);
	} else {
		netdev_dbg(data->netdev, "DMA operation finished successfully "
		                         "(RX%u, cookie = 0x%08X).\n", 
		                         dma->ch_num, buff->cookie);
	}

	buff->error = (status != DMA_COMPLETE);
	buff->residue = state.residue;
	complete_all(&buff->dma_completion);

	spin_unlock_irqrestore(&dma->lock, flags);
}

static int rcm_mgeth_rx_start(struct rcm_mgeth_dma* dma)
{
	struct rcm_mgeth_data *data = dma->chan->private;
	unsigned long flags;
	struct rcm_mgeth_dma_buff *buff;
	struct dma_async_tx_descriptor *desc;
	int cnt = 0;

	spin_lock_irqsave(&dma->lock, flags);

	while (true) {
		if (dma->cnt_buffs == RCM_MGETH_CNT_BUFFS_PER_CHAN)
			break;

		buff = &dma->buffs[dma->next_buff];

		buff->skb = NULL;
		sg_dma_address(&buff->sg) = 
			dma->dma_addr + dma->next_buff * RCM_MGETH_DMA_BUFF_SIZE;
		sg_dma_len(&buff->sg) = RCM_MGETH_MAX_FRAME_SIZE;

//		memset(dma->buff + dma->next_buff * RCM_MGETH_DMA_BUFF_SIZE,
//		       0xCC, RCM_MGETH_DMA_BUFF_SIZE);

		netdev_dbg(data->netdev,
		           "RX%u: DMA operation ready (%u bytes).\n", 
		           dma->ch_num, sg_dma_len(&buff->sg));

		desc = dmaengine_prep_slave_sg(dma->chan, &buff->sg, 1,
		                               DMA_DEV_TO_MEM,
		                               DMA_PREP_INTERRUPT);

		if (!desc) {
			netdev_warn(data->netdev,
			            "Failed prepare DMA RX-operation.\n");
			break;
		}

		reinit_completion(&buff->dma_completion);

		desc->callback = rcm_mgeth_rx_dma_callback;
		desc->callback_param = buff;

		netdev_dbg(data->netdev,
		           "RX%u: DMA operation prepared (%u bytes).\n", 
		           dma->ch_num, sg_dma_len(&buff->sg));

		buff->cookie = dmaengine_submit(desc);
		dma_async_issue_pending(dma->chan); 

		netdev_dbg(data->netdev,
		           "RX%u: DMA operation submitted "
		           "(%u bytes, cookie = 0x%08X).\n", 
		           dma->ch_num, sg_dma_len(&buff->sg), buff->cookie);

		dma->next_buff = (dma->next_buff + 1) % RCM_MGETH_CNT_BUFFS_PER_CHAN;
		++dma->cnt_buffs;

		++cnt;
	}

	spin_unlock_irqrestore(&dma->lock, flags);

	return cnt;
}

static bool rcm_mgeth_rx_wait(struct rcm_mgeth_dma* dma)
{
	unsigned long flags;
	struct rcm_mgeth_dma_buff *buff = NULL;

	spin_lock_irqsave(&dma->lock, flags);

	if (dma->cnt_buffs > 0) 
		buff = &dma->buffs[dma->first_buff];

	spin_unlock_irqrestore(&dma->lock, flags);

	if (!buff)
		return false;

	if (!wait_for_completion_timeout(&buff->dma_completion,
	                                 msecs_to_jiffies(RCM_MGETH_RX_TIMEOUT)))
		return false;

	return true;
}

static int rcm_mgeth_rx_process(struct rcm_mgeth_dma* dma)
{
	struct rcm_mgeth_data *data = dma->chan->private;
	unsigned long flags;
	struct rcm_mgeth_dma_buff *buff;
	struct sk_buff *skb;
	int processed = 0;

	spin_lock_irqsave(&dma->lock, flags);

	while (true) {
		if (dma->cnt_buffs == 0)
			break;

		buff = &dma->buffs[dma->first_buff];

		if (!try_wait_for_completion(&buff->dma_completion))
			break;

		if (buff->error) {
			netdev_warn(data->netdev,
			            "DMA error (RX%u, cookie = 0x%08X).\n",
			            dma->ch_num, buff->cookie);
		} else {
			char* pkt = dma->buff + dma->first_buff * RCM_MGETH_DMA_BUFF_SIZE;

			skb = netdev_alloc_skb(data->netdev,
			                       sg_dma_len(&buff->sg) -
			                       buff->residue);

			if (!skb) {
				netdev_warn(data->netdev,
				            "Failed to allocate packet (RX%u).\n",
				            dma->ch_num);
			} else {
				skb_put_data(skb, 
				             dma->buff + dma->first_buff * RCM_MGETH_DMA_BUFF_SIZE, 
				             sg_dma_len(&buff->sg) - buff->residue);
				skb->protocol = eth_type_trans(skb, data->netdev);

				netif_rx(skb);
			}
		}

		dma->first_buff = (dma->first_buff + 1) % 
		                  RCM_MGETH_CNT_BUFFS_PER_CHAN;
		--dma->cnt_buffs;
		++processed;
	}

	spin_unlock_irqrestore(&dma->lock, flags);

	return processed;
}

static int rcm_mgeth_rx(void* context)
{
	struct rcm_mgeth_dma* dma = context;

	while (!kthread_should_stop()) {
		rcm_mgeth_rx_start(dma);

		if (rcm_mgeth_rx_wait(dma))
			rcm_mgeth_rx_process(dma);
	}

	return 0;
}

/* Netdevice operations */

static int rcm_mgeth_open(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	int err;
	int i;

	err = rcm_mgeth_reset(data);

	if (err)
		return err;

	regmap_write(data->reg, RCM_MGETH_IRQ_MASK, 0);

	data->phydev = of_phy_connect(netdev, data->phynode,
	                              &rcm_mgeth_adjust_link, 0, data->phy_if);

	if (!data->phydev) {
		netdev_err(netdev, "Could not find the PHY\n");
		return -ENODEV;
	}

	phy_attached_info(data->phydev);

	phy_start(data->phydev);

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		struct rcm_mgeth_dma* dma =  &data->rx[i];

		if (!dma->chan)
			continue;

		dma->thread = kthread_run(rcm_mgeth_rx, dma, "rcm_mgeth_rx");

		if (IS_ERR(dma->thread))
		{
			netdev_err(netdev, "Failed to start RX-thread.\n");
			err = PTR_ERR(dma->thread);
			return err;
		};
	}

	netdev_dbg(netdev, "Attached to PHY %d UID 0x%08x Link = %d\n", 
	           data->phydev->mdio.addr, data->phydev->phy_id,
	           data->phydev->link);

	return 0;
}

static int rcm_mgeth_close(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	int i;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		struct rcm_mgeth_dma* dma =  &data->rx[i];

		if (!dma->chan)
			continue;

		if (dma->thread) {
			kthread_stop(dma->thread);
			dma->thread = NULL;
		}
	}

	if (data->phydev) {
		phy_stop(data->phydev);
		phy_disconnect(data->phydev);
		data->phydev = NULL;
	}

	free_irq(data->irq, netdev);

	return 0;
}

static int rcm_mgeth_set_mac(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;
	char *mac = (u8 *)addr->sa_data;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, mac, netdev->addr_len);

	return 0;
}

static void rcm_mgeth_set_rx_mode(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);

	int i = 0;

	if (netdev->flags & IFF_PROMISC) {
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(0), 0);
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(1), 0);
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(2), 0);
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(3), 0);
		return;
	}

	for (i = 0; i < 3; ++i) {
		regmap_write(data->mask[i], RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
		             0xFFFFFFFF);
		regmap_write(data->mask[i], RCM_MGETH_RX_ETH_MASK_ACTIVE(1),
		             0x0000FFFF);
		regmap_write(data->mask[i], RCM_MGETH_RX_ETH_MASK_VALUE(0),
		             netdev->dev_addr[0] | (netdev->dev_addr[1] << 8) | 
		             (netdev->dev_addr[2] << 16) | 
		             (netdev->dev_addr[3] << 24));
		regmap_write(data->mask[i], RCM_MGETH_RX_ETH_MASK_VALUE(1), 
		             netdev->dev_addr[4] | (netdev->dev_addr[5] << 8));
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(i), 6);
	}

	// Broadcast and multicast
	regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(3), 1);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_ACTIVE(0), 0x01);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_VALUE(0),  0x01);
/*
	regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(1), 6);
	regmap_write(data->mask[1], RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
	             0xFFFFFFFF);
	regmap_write(data->mask[1], RCM_MGETH_RX_ETH_MASK_ACTIVE(1),
	             0x0000FFFF);
	regmap_write(data->mask[1], RCM_MGETH_RX_ETH_MASK_VALUE(0), 0xFFFFFFFF);
	regmap_write(data->mask[1], RCM_MGETH_RX_ETH_MASK_VALUE(1), 0x0000FFFF);

	if ((netdev_mc_empty(netdev)) && 
	    ((netdev->flags & IFF_ALLMULTI) == 0)) {
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(2), 6);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
		             0xFFFFFFFF);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_ACTIVE(1),
		             0x0000FFFF);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_VALUE(0),
		             0xFFFFFFFF);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_VALUE(1),
		             0x0000FFFF);
	} else if ((netdev->flags & IFF_ALLMULTI) || 
	           (netdev_mc_count(netdev) > 2)) {
		regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(2), 1);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
		             0x01);
		regmap_write(data->mask[2], RCM_MGETH_RX_ETH_MASK_VALUE(0),
		             0x01);
	} else if ((netdev_mc_count(netdev) == 1) ||
	           (netdev_mc_count(netdev) == 2)) {
		struct netdev_hw_addr *ha;
		int ch = 2;

		netdev_for_each_mc_addr(ha, netdev) {
			regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(ch), 6);
			regmap_write(data->mask[ch],
			             RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
			             0xFFFFFFFF);
			regmap_write(data->mask[ch],
			             RCM_MGETH_RX_ETH_MASK_ACTIVE(1),
			             0x0000FFFF);
			regmap_write(data->mask[ch],
			             RCM_MGETH_RX_ETH_MASK_VALUE(0),
			             ha->addr[0] | (ha->addr[1] << 8) | 
			             (ha->addr[2] << 16) | (ha->addr[3] << 24));
			regmap_write(data->mask[ch],
			             RCM_MGETH_RX_ETH_MASK_VALUE(1), 
			             ha->addr[4] | (ha->addr[5] << 8));
			++ch;
		}

		if (netdev_mc_count(netdev) == 2)
			return;
	}

	regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(3), 6);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_ACTIVE(0),
	             0xFFFFFFFF);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_ACTIVE(1),
	             0x0000FFFF);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_VALUE(0), 0);
	regmap_write(data->mask[3], RCM_MGETH_RX_ETH_MASK_VALUE(1), 0);
*/
}

static int rcm_mgeth_init_phy(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	struct device_node *np = data->dev->of_node;
	struct phy *ifphy;
	int ret;

	data->link = 0;
	data->speed = 0;
	data->duplex = -1;

	ifphy = devm_of_phy_get(data->dev, np, NULL);
	if (IS_ERR(ifphy)) {
		netdev_err(netdev, "%pOF: Error retrieving port phy: %ld\n",
		           np, PTR_ERR(ifphy));
		return PTR_ERR(ifphy);
	}

	ret = phy_init(ifphy);
	if (ret) {
		netdev_err(netdev, "Failed to init PHY: %d\n", ret);
		return ret;
	}

	ret = of_get_phy_mode(np, &data->phy_if);
	if (ret) {
		netdev_err(netdev, "%pOF read phy-mode err %d\n", np, ret);
		return ret;
	}

	data->phynode = of_parse_phandle(np, "phy-handle", 0);

	if (!data->phynode) {
		netdev_err(netdev, "%pOF no phy found\n", np);
		return -ENODEV;
	}

	return 0;
}

static void rcm_mgeth_free_dma(struct rcm_mgeth_data *data)
{
	int i;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS * 2; ++i) {
		struct rcm_mgeth_dma* dma =  (i < RCM_MGETH_MAX_DMA_CHANNELS) ? 
		                             &data->tx[i] : 
		                             &data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS];

		if (!dma->chan)
			continue;

		if (dma->buff) {
			dma_free_coherent(data->dev, 
			                  RCM_MGETH_CNT_BUFFS_PER_CHAN * RCM_MGETH_DMA_BUFF_SIZE, 
			                  dma->buff, dma->dma_addr);
			dma->buff = NULL;
		}

		dma_release_channel(dma->chan);
		dma->chan = NULL;
	}
}

static int rcm_mgeth_prep_dma(struct rcm_mgeth_data *data)
{
	int ret = 0;
	int i;
	int j;
	bool have_tx = false;
	bool have_rx = false;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS * 2; ++i) {
		char ch_name[8];
		struct rcm_mgeth_dma* dma =  (i < RCM_MGETH_MAX_DMA_CHANNELS) ? 
		                             &data->tx[i] : 
		                             &data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS];

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			snprintf(ch_name, sizeof(ch_name), "tx%d", i);
		else
			snprintf(ch_name, sizeof(ch_name), "rx%d",
			         i - RCM_MGETH_MAX_DMA_CHANNELS);

		dma->chan = dma_request_chan(data->dev, ch_name);

		if (IS_ERR(dma->chan)) {
			dma->chan = NULL;
			continue;
		}

		dma->chan->private = data;

		dma->ch_num = i % RCM_MGETH_MAX_DMA_CHANNELS;
		spin_lock_init(&dma->lock);

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			have_tx = true;
		else
			have_rx = true;

		dma->buff = 
			dma_alloc_coherent(data->dev,
			                   RCM_MGETH_CNT_BUFFS_PER_CHAN * RCM_MGETH_DMA_BUFF_SIZE,
			                   &dma->dma_addr, GFP_KERNEL);

		if (!dma->buff) {
			netdev_err(data->netdev,
			           "Failed to allocate DMA-buffer.\n");
			ret = -ENOMEM;
			break;
		}

		for (j = 0; j < RCM_MGETH_CNT_BUFFS_PER_CHAN; ++j) {
			dma->buffs[j].dma = dma;
			sg_init_table(&dma->buffs[j].sg, 1);
			init_completion(&dma->buffs[j].dma_completion);
		}
	}

	if (ret == 0) {
		if (!have_tx) {
			netdev_err(data->netdev,
			           "No DMA TX channel specified.\n");
			ret = -ENODEV;
		}

		if (!have_rx) {
			netdev_err(data->netdev,
			           "No DMA RX channel specified.\n");
			ret = -ENODEV;
		}
	}

	if (ret != 0)
		rcm_mgeth_free_dma(data);

	return ret;
}

static const struct net_device_ops rcm_mgeth_netdev_ops = {
	.ndo_open               = rcm_mgeth_open,
	.ndo_stop               = rcm_mgeth_close,
	.ndo_start_xmit         = rcm_mgeth_start_xmit,
	.ndo_set_mac_address    = rcm_mgeth_set_mac,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_set_rx_mode        = rcm_mgeth_set_rx_mode
};

static const struct ethtool_ops rcm_mgeth_ethtool_ops = {
	.get_link               = ethtool_op_get_link,
};

static const struct regmap_config rcm_mgeth_regmap_cfg_0 = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x1FC,
	.fast_io = true,
};

static const struct regmap_config rcm_mgeth_regmap_cfg_1 = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xFC,
	.fast_io = true,
};

static const struct of_device_id rcm_mgeth_of_match[];

static int rcm_mgeth_probe(struct platform_device *pdev)
{
	struct rcm_mgeth_data *data;
	struct net_device *netdev;
	struct resource *res;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	int err = -ENOMEM;
	const u8 *addr;
	int i;
	u32 id;
	u32 ver;
	struct irq_chip_generic *gc;

	match = of_match_device(rcm_mgeth_of_match, dev);
	if (!match)
		return -EINVAL;

	netdev = devm_alloc_etherdev(dev, sizeof(struct rcm_mgeth_data));

	if (netdev == NULL)
		return -ENOMEM;

	data = netdev_priv(netdev);
	data->netdev = netdev;
	data->dev = dev;

	SET_NETDEV_DEV(netdev, dev);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init_mmio(dev, base, &rcm_mgeth_regmap_cfg_0);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		char name[16];

		snprintf(name, sizeof(name), "rx-mask%d", i);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		base = devm_ioremap_resource(dev, res);
		if (IS_ERR(base)) {
			dev_err(dev, "missing \"%s\"\n", name);
			return PTR_ERR(base);
		}

		data->mask[i] = devm_regmap_init_mmio(dev, base,
		                                      &rcm_mgeth_regmap_cfg_1);
		if (IS_ERR(data->mask[i])) {
			dev_err(dev, "failed to init regmap\n");
			return PTR_ERR(data->mask[i]);
		}
	}

	regmap_read(data->reg, RCM_MGETH_ID,      &id);
	regmap_read(data->reg, RCM_MGETH_VERSION, &ver);

	dev_info(dev, "ID: 0x%08X, VER: 0x%08X.\n", id, ver);

	err = rcm_mgeth_init_phy(netdev);

	if (err != 0) 
		return err;

	data->irq = platform_get_irq(pdev, 0);

	data->domain = irq_domain_add_linear(np, RCM_MGETH_MAX_DMA_CHANNELS * 2,
	                                     &irq_generic_chip_ops, data);
	if (!data->domain) {
		dev_err(dev, "couldn't allocate irq domain.\n");
		return -ENODEV;
	}

	err = irq_alloc_domain_generic_chips(data->domain,
	                                     RCM_MGETH_MAX_DMA_CHANNELS * 2,
	                                     1, np->name, handle_level_irq,
	                                     IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_LEVEL,
	                                     0, 0);
	if (err) {
		dev_err(dev, "couldn't allocate irq chip.\n");
		return err;
	}

	gc = irq_get_domain_generic_chip(data->domain, 0);
	gc->private = data;
	gc->chip_types[0].type = IRQ_TYPE_LEVEL_MASK;
/*
	gc->chip_types[0].chip.irq_ack      = rcm_mgeth_irq_ack;
	gc->chip_types[0].chip.irq_mask     = rcm_mgeth_irq_mask;
	gc->chip_types[0].chip.irq_unmask   = rcm_mgeth_irq_unmask;
*/
	gc->chip_types[0].chip.irq_set_type = rcm_mgeth_irq_set_type;

	gc->chip_types[0].chip.name         = np->name;

	irq_set_chained_handler_and_data(data->irq, rcm_mgeth_irq, data);

	err = rcm_mgeth_prep_dma(data);

	if (err != 0) 
		return err;

	err = rcm_mgeth_reset(data);

	if (err) 
		goto err_dma;

	netdev->netdev_ops = &rcm_mgeth_netdev_ops;
	netdev->ethtool_ops = &rcm_mgeth_ethtool_ops;

	netdev->max_mtu = RCM_MGETH_MAX_FRAME_SIZE;

	/* Check if a mac address was given */
	i = netdev->addr_len;
	addr = of_get_mac_address(np);
	if (!IS_ERR(addr)) {
		for (i = 0; i < netdev->addr_len; i++)
			if (addr[i] != 0)
				break;

		if (i < netdev->addr_len)
			memcpy(netdev->dev_addr, addr, netdev->addr_len);
	}

	if (i == netdev->addr_len) {
		if (!rcm_mgeth_read_mac_addr(data, netdev->dev_addr))
			eth_hw_addr_random(netdev);
	}

	err = register_netdev(netdev);
	if (err)
		goto err_dma;

	if (device_create_file(data->dev, &dev_attr_stats)) {
		dev_warn(dev, "Failed to create \"stats\" file.\n");
	}

	platform_set_drvdata(pdev, netdev);

	dev_info(dev, "initialized\n");

	return 0;

err_dma:
	rcm_mgeth_free_dma(data);

	return err;
}

static int rcm_mgeth_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct rcm_mgeth_data *data = netdev_priv(netdev);

	device_remove_file(data->dev, &dev_attr_stats);

	unregister_netdev(netdev);

	rcm_mgeth_free_dma(data);

	return 0;
}

static const struct of_device_id rcm_mgeth_of_match[] = {
	{ .compatible = "rcm,mgeth", },
	{},
};

MODULE_DEVICE_TABLE(of, greth_of_match);

static struct platform_driver rcm_mgeth_driver = {
	.driver = {
		.name	= "rcm-mgeth",
		.of_match_table = rcm_mgeth_of_match,
	},
	.probe		= rcm_mgeth_probe,
	.remove		= rcm_mgeth_remove,
};

module_platform_driver(rcm_mgeth_driver);

MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_DESCRIPTION("RC-Module MGETH Driver");
MODULE_LICENSE("GPL");
