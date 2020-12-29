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
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/phy/phy.h>
#include <linux/of_mdio.h>
#include <linux/irq.h>

#include "rcm_mgeth.h"

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

static struct rcm_mgeth_dma_chan* rcm_mgeth_get_chan(struct rcm_mgeth_data *data,
                                                     bool is_tx)
{
	int i;
	struct rcm_mgeth_dma_chan **channels = (is_tx) ? data->tx : data->rx;
	struct rcm_mgeth_dma_chan *chan = NULL;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		if (!rcm_mgeth_dma_busy(channels[i]))
			chan = channels[i];

		if (chan)
			break;
	}

	return chan;
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

static irqreturn_t rcm_mgeth_irq(int irq, void *dev_id)
{
	struct rcm_mgeth_data *data = dev_id;
	u32 status;
	int i;
	bool start_pooling = false;

	regmap_read(data->reg, RCM_MGETH_GLOBAL_STATUS, &status);

	if (status == 0)
		return IRQ_NONE;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; i++) {
		if (status & BIT(i)) {
			rcm_mgeth_dma_irq(data->tx[i]);
		}
		if (status & BIT(i + 16)) {
			rcm_mgeth_dma_irq(data->rx[i]);
			start_pooling = true;
		}
	}

	if (start_pooling) {
		for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; i++)
			rcm_mgeth_dma_disable_irq(data->rx[i]);

		napi_schedule(&data->napi);
	}

	return IRQ_HANDLED;
}

static int rcm_mgeth_poll(struct napi_struct *napi, int budget)
{
	struct rcm_mgeth_data *data = 
		container_of(napi, struct rcm_mgeth_data, napi);
	int work_done = 0;
	int i;

	for (i = 0; (i < RCM_MGETH_MAX_DMA_CHANNELS) && (work_done < budget);
	     i++)
		work_done += rcm_mgeth_dma_rx_poll(data->rx[i], budget - work_done);

	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; i++)
			rcm_mgeth_dma_enable_irq(data->rx[i]);
	}

	return work_done;
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
	           "Link state changed: link = %u "
	           "speed = %d, duplex = %d\n",
	           phydev->link, phydev->speed,
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

static void rcm_mgeth_tx_completed(struct rcm_mgeth_dma_chan* chan,
                                   int status, u32 size)
{
	struct net_device *netdev = rcm_mgeth_dma_get_net_device(chan);
	int ch_num = rcm_mgeth_dma_get_number(chan);

	if (status != 0) {
		netdev->stats.tx_dropped++;
		netdev_warn(netdev, "DMA error (TX%u).\n", ch_num);
	} else {
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += size;
		netdev_dbg(netdev, 
		           "DMA operation finished successfully "
		           "(TX%u, %u bytes).\n", 
		           ch_num, size);
	}

	if (netif_queue_stopped(netdev))
		netif_wake_queue(netdev);
}

static netdev_tx_t rcm_mgeth_start_xmit(struct sk_buff *skb,
                                        struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	struct rcm_mgeth_dma_chan *chan;
	int ch_num;

	if (skb->len > RCM_MGETH_MAX_FRAME_SIZE) {
		netdev->stats.tx_dropped++;
		netdev_err(netdev, "Frame length is too much: %u bytes.\n",
		           skb->len);
		return NET_XMIT_DROP;
	}

	chan = rcm_mgeth_get_chan(data, true);

	if (!chan) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	ch_num = rcm_mgeth_dma_get_number(chan);

	if (!rcm_mgeth_dma_submit_tx(chan, skb, rcm_mgeth_tx_completed)) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}

	netdev_dbg(netdev, "TX%u: DMA operation submitted (%u bytes).\n", 
	           ch_num, skb->len);

	return NETDEV_TX_OK;
}

static void rcm_mgeth_rx_completed(struct rcm_mgeth_dma_chan* chan,
                                   void *packet, u32 size)
{
	struct net_device *netdev = rcm_mgeth_dma_get_net_device(chan);
	int ch_num = rcm_mgeth_dma_get_number(chan);
	struct sk_buff *skb;

	netdev_dbg(netdev, "DMA operation finished successfully "
	                   "(RX%u, size = %u).\n", 
	                   ch_num, size);

	skb = netdev_alloc_skb(netdev, size);

	if (!skb) {
		netdev->stats.rx_dropped++;
		netdev_warn(netdev, "Failed to allocate packet (RX%u).\n",
		            ch_num);
	} else {
		skb_put_data(skb, packet, size);
		skb->protocol = eth_type_trans(skb, netdev);

		netdev->stats.rx_bytes += size;
		netdev->stats.rx_packets++;
		netif_receive_skb(skb);
	}
}

/* Netdevice operations */

static int rcm_mgeth_open(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	int err;
	int i;
#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device *device;
#endif

	err = request_irq(data->irq, rcm_mgeth_irq, IRQF_SHARED, "rcm_mgeth",
	                  (void *)data);
	if (err) {
		netdev_err(netdev, "Could not allocate interrupt %d\n",
		           data->irq);
		return err;
	}

	err = rcm_mgeth_reset(data);

	if (err)
		return err;

	regmap_write(data->reg, RCM_MGETH_IRQ_MASK, 0);

#ifdef CONFIG_BASIS_PLATFORM
	device = basis_device_find(data->phy_dev);
	if (!device) {
		netdev_err(netdev, "Failed to found BASIS-device \"%s\"\n",
		           data->phy_dev);
		return -ENODEV;
	}

	if (!device->priv) {
		netdev_err(netdev, "BASIS-device \"%s\" is not PHY-Device\n",
		           data->phy_dev);
		return -ENODEV;
	}

	data->phydev = device->priv;

	err = phy_connect_direct(netdev, data->phydev, &rcm_mgeth_adjust_link,
	                         data->phy_if);
	put_device(&data->phydev->mdio.dev);
	if (err)
		data->phydev = NULL;
#else
	data->phydev = of_phy_connect(netdev, data->phynode,
	                              &rcm_mgeth_adjust_link, 0, data->phy_if);
#endif
	if (!data->phydev) {
		netdev_err(netdev, "Could not find the PHY\n");
		return -ENODEV;
	}

	phy_attached_info(data->phydev);

	phy_start(data->phydev);

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		struct rcm_mgeth_dma_chan* chan =  data->rx[i];

		err = rcm_mgeth_dma_start_rx(chan, rcm_mgeth_rx_completed);

		if (err)
			return err;
	}

	napi_enable(&data->napi);

	netdev_dbg(netdev, "Attached to PHY %d UID 0x%08x Link = %d\n", 
	           data->phydev->mdio.addr, data->phydev->phy_id,
	           data->phydev->link);

	return 0;
}

static int rcm_mgeth_close(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	int i;

	napi_disable(&data->napi);

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		struct rcm_mgeth_dma_chan* chan =  data->rx[i];

		rcm_mgeth_dma_stop_rx(chan);
	}

	if (data->phydev) {
		phy_stop(data->phydev);
		phy_disconnect(data->phydev);
		data->phydev = NULL;
	}

	free_irq(data->irq, (void *)data);

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

//	regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(1), 10244);
//	regmap_write(data->reg, RCM_MGETH_LEN_MASK_CH(2), 10244);

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

static void rcm_mgeth_free_dma(struct rcm_mgeth_data *data)
{
	int i;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS * 2; ++i) {
		struct rcm_mgeth_dma_chan* chan = 
			(i < RCM_MGETH_MAX_DMA_CHANNELS) ? 
			data->tx[i] : 
			data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS];

		if (!chan)
			continue;

		rcm_mgeth_dma_chan_remove(chan);

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			data->tx[i] = NULL;
		else
			data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS] = NULL;
	}
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

#ifdef CONFIG_BASIS_PLATFORM

static int rcm_mgeth_init_phy(struct net_device *netdev)
{
	struct rcm_mgeth_data *data = netdev_priv(netdev);
	struct basis_device *device;
	struct phy *ifphy;

	int ret;
	int i;

	data->link = 0;
	data->speed = 0;
	data->duplex = -1;

	device = basis_device_find(data->phy);
	if (!device) {
		netdev_err(netdev, "Failed to found BASIS-device \"%s\"\n",
		           data->phy);
		return -ENODEV;
	}

	if (!device->priv) {
		netdev_err(netdev, "BASIS-device \"%s\" is not PHY\n",
		           data->phy);
		return -ENODEV;
	}

	ifphy = device->priv;

	ret = phy_init(ifphy);
	if (ret) {
		netdev_err(netdev, "Failed to init PHY: %d\n", ret);
		return ret;
	}

	data->phy_if = PHY_INTERFACE_MODE_NA;

	for (i = 0; i < PHY_INTERFACE_MODE_MAX; i++)
		if (!strcasecmp(data->phy_mode, phy_modes(i))) {
			data->phy_if = i;
			break;
		}

	if (data->phy_if == PHY_INTERFACE_MODE_NA) {
		netdev_err(netdev, "Unknown phy-mode: \"%s\"\n",
		           data->phy_mode);
		return -EINVAL;
	}

	return 0;
}

static int rcm_mgeth_prep_dma(struct rcm_mgeth_data *data)
{
	int ret = 0;
	int i;
	void __iomem *base;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS * 2; ++i) {
		char ch_name[8];

		struct rcm_mgeth_dma_chan* chan = NULL;

		enum dma_data_direction dir = (i < RCM_MGETH_MAX_DMA_CHANNELS) ?
		                              DMA_TO_DEVICE :
		                              DMA_FROM_DEVICE;
		u32 reg;
		u32 reg_size;

		if (i < RCM_MGETH_MAX_DMA_CHANNELS) {
			snprintf(ch_name, sizeof(ch_name), "tx%d", i);
			reg = data->reg_tx_chan[i];
			reg_size = data->reg_tx_chan_size;
		} else {
			snprintf(ch_name, sizeof(ch_name), "rx%d",
			         i - RCM_MGETH_MAX_DMA_CHANNELS);
			reg = data->reg_rx_chan[i - RCM_MGETH_MAX_DMA_CHANNELS];
			reg_size = data->reg_rx_chan_size;
		}

		base = devm_ioremap(data->dev,
		                    reg + data->device->controller->ep_base_phys,
		                    reg_size);
		if (IS_ERR(base)) {
			netdev_err(data->netdev, "missing \"%s\"\n", ch_name);
			ret = PTR_ERR(base);
			break;
		}

		chan = rcm_mgeth_dma_chan_create(data->netdev, ch_name, base,
		                                 (i % RCM_MGETH_MAX_DMA_CHANNELS),
		                                 dir);

		if (!chan) {
			ret = -EFAULT;
			break;
		}

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			data->tx[i] = chan;
		else
			data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS] = chan;
	}

	if (ret != 0)
		rcm_mgeth_free_dma(data);

	return ret;
}

static int rcm_mgeth_bind(struct basis_device *device)
{
	struct device *dev = &device->dev;
	struct rcm_mgeth_data *data = basis_device_get_drvdata(device);
	struct net_device *netdev = data->netdev;
	void __iomem *base;
	int err = -ENOMEM;
	int i;
	u32 id;
	u32 ver;

	dev_info(dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	base = devm_ioremap(dev, data->regs + device->controller->ep_base_phys,
	                    data->regs_size);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init(dev, &basis_regmap_bus, base,
	                             &rcm_mgeth_regmap_cfg_0);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS; ++i) {
		base = devm_ioremap(dev,
		                    data->reg_rx_mask[i] + 
		                    device->controller->ep_base_phys,
	                            data->reg_rx_mask_size);
		if (IS_ERR(base)) {
			dev_err(dev, "missing \"rx-mask%d\"\n", i);
			return PTR_ERR(base);
		}

		data->mask[i] = devm_regmap_init(dev, &basis_regmap_bus, base,
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

	data->irq = irq_create_mapping(device->controller->domain,
	                               data->hwirq);

	err = rcm_mgeth_prep_dma(data);

	if (err != 0) 
		return err;

	err = rcm_mgeth_reset(data);

	if (err) 
		goto err_dma;

	netdev->netdev_ops = &rcm_mgeth_netdev_ops;
	netdev->ethtool_ops = &rcm_mgeth_ethtool_ops;

	netdev->max_mtu = RCM_MGETH_MAX_FRAME_SIZE;

	if (!rcm_mgeth_read_mac_addr(data, netdev->dev_addr))
		eth_hw_addr_random(netdev);

	err = register_netdev(netdev);
	if (err)
		goto err_dma;

	if (device_create_file(data->dev, &dev_attr_stats)) {
		dev_warn(dev, "Failed to create \"stats\" file.\n");
	}

	netif_napi_add(netdev, &data->napi, rcm_mgeth_poll, 64);

	dev_info(dev, "initialized\n");

	return 0;

err_dma:
	rcm_mgeth_free_dma(data);

	return err;
}

static void rcm_mgeth_unbind(struct basis_device *device)
{
	struct rcm_mgeth_data *data = basis_device_get_drvdata(device);

	device_remove_file(data->dev, &dev_attr_stats);

	unregister_netdev(data->netdev);

	rcm_mgeth_free_dma(data);
}

static int rcm_mgeth_probe(struct basis_device *device)
{
	struct device *dev = &device->dev;
	struct net_device *netdev;
	struct rcm_mgeth_data *data;

	netdev = devm_alloc_etherdev(dev, sizeof(struct rcm_mgeth_data));

	if (netdev == NULL)
		return -ENOMEM;

	data = netdev_priv(netdev);
	data->netdev = netdev;
	data->dev = dev;
	data->device = device;

	SET_NETDEV_DEV(netdev, dev);

	basis_device_set_drvdata(device, data);

	return 0;
}

static const struct basis_device_id rcm_mgeth_ids[] = {
	{
		.name = "rcm-mgeth",
	},
	{},
};

static struct basis_device_ops rcm_mgeth_ops = {
	.unbind = rcm_mgeth_unbind,
	.bind   = rcm_mgeth_bind,
};

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  hwirq,     struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, hwirq,     struct rcm_mgeth_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  regs,      struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, regs,      struct rcm_mgeth_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  regs_size, struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, regs_size, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_mask, 0, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_mask, 0, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_mask, 1, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_mask, 1, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_mask, 2, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_mask, 2, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_mask, 3, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_mask, 3, struct rcm_mgeth_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  reg_rx_mask_size, struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, reg_rx_mask_size, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_chan, 0, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_chan, 0, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_chan, 1, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_chan, 1, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_chan, 2, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_chan, 2, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_rx_chan, 3, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_rx_chan, 3, struct rcm_mgeth_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  reg_rx_chan_size, struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, reg_rx_chan_size, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_tx_chan, 0, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_tx_chan, 0, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_tx_chan, 1, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_tx_chan, 1, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_tx_chan, 2, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_tx_chan, 2, struct rcm_mgeth_data);

BASIS_DEV_ATTR_ARR_U32_SHOW(rcm_mgeth_,  reg_tx_chan, 3, struct rcm_mgeth_data);
BASIS_DEV_ATTR_ARR_U32_STORE(rcm_mgeth_, reg_tx_chan, 3, struct rcm_mgeth_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mgeth_,  reg_tx_chan_size, struct rcm_mgeth_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mgeth_, reg_tx_chan_size, struct rcm_mgeth_data);

BASIS_DEV_ATTR_STR_SHOW(rcm_mgeth_,  phy,       struct rcm_mgeth_data);
BASIS_DEV_ATTR_STR_STORE(rcm_mgeth_, phy,       struct rcm_mgeth_data);

BASIS_DEV_ATTR_STR_SHOW(rcm_mgeth_,  phy_dev,   struct rcm_mgeth_data);
BASIS_DEV_ATTR_STR_STORE(rcm_mgeth_, phy_dev,   struct rcm_mgeth_data);

BASIS_DEV_ATTR_STR_SHOW(rcm_mgeth_,  phy_mode,  struct rcm_mgeth_data);
BASIS_DEV_ATTR_STR_STORE(rcm_mgeth_, phy_mode,  struct rcm_mgeth_data);

CONFIGFS_ATTR(rcm_mgeth_, hwirq);
CONFIGFS_ATTR(rcm_mgeth_, regs);
CONFIGFS_ATTR(rcm_mgeth_, regs_size);

CONFIGFS_ATTR(rcm_mgeth_, reg_rx_mask_0);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_mask_1);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_mask_2);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_mask_3);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_mask_size);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_chan_0);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_chan_1);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_chan_2);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_chan_3);
CONFIGFS_ATTR(rcm_mgeth_, reg_rx_chan_size);
CONFIGFS_ATTR(rcm_mgeth_, reg_tx_chan_0);
CONFIGFS_ATTR(rcm_mgeth_, reg_tx_chan_1);
CONFIGFS_ATTR(rcm_mgeth_, reg_tx_chan_2);
CONFIGFS_ATTR(rcm_mgeth_, reg_tx_chan_3);
CONFIGFS_ATTR(rcm_mgeth_, reg_tx_chan_size);

CONFIGFS_ATTR(rcm_mgeth_, phy);
CONFIGFS_ATTR(rcm_mgeth_, phy_dev);
CONFIGFS_ATTR(rcm_mgeth_, phy_mode);

static struct configfs_attribute *rcm_mgeth_attrs[] = {
	&rcm_mgeth_attr_hwirq,
	&rcm_mgeth_attr_regs,
	&rcm_mgeth_attr_regs_size,
	&rcm_mgeth_attr_reg_rx_mask_0,
	&rcm_mgeth_attr_reg_rx_mask_1,
	&rcm_mgeth_attr_reg_rx_mask_2,
	&rcm_mgeth_attr_reg_rx_mask_3,
	&rcm_mgeth_attr_reg_rx_mask_size,
	&rcm_mgeth_attr_reg_rx_chan_0,
	&rcm_mgeth_attr_reg_rx_chan_1,
	&rcm_mgeth_attr_reg_rx_chan_2,
	&rcm_mgeth_attr_reg_rx_chan_3,
	&rcm_mgeth_attr_reg_rx_chan_size,
	&rcm_mgeth_attr_reg_tx_chan_0,
	&rcm_mgeth_attr_reg_tx_chan_1,
	&rcm_mgeth_attr_reg_tx_chan_2,
	&rcm_mgeth_attr_reg_tx_chan_3,
	&rcm_mgeth_attr_reg_tx_chan_size,
	&rcm_mgeth_attr_phy,
	&rcm_mgeth_attr_phy_dev,
	&rcm_mgeth_attr_phy_mode,
	NULL,
};

static struct basis_device_driver rcm_mgeth_driver = {
	.driver.name    = "rcm-mgeth",
	.probe          = rcm_mgeth_probe,
	.id_table       = rcm_mgeth_ids,
	.ops            = &rcm_mgeth_ops,
	.owner          = THIS_MODULE,
	.attrs          = rcm_mgeth_attrs,
};
module_basis_driver(rcm_mgeth_driver);

#else /* CONFIG_BASIS_PLATFORM */

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

static int rcm_mgeth_prep_dma(struct rcm_mgeth_data *data,
                              struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct resource *res;
	void __iomem *base;

	for (i = 0; i < RCM_MGETH_MAX_DMA_CHANNELS * 2; ++i) {
		char ch_name[8];

		struct rcm_mgeth_dma_chan* chan = NULL;

		enum dma_data_direction dir = (i < RCM_MGETH_MAX_DMA_CHANNELS) ?
		                              DMA_TO_DEVICE :
		                              DMA_FROM_DEVICE;

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			snprintf(ch_name, sizeof(ch_name), "tx%d", i);
		else
			snprintf(ch_name, sizeof(ch_name), "rx%d",
			         i - RCM_MGETH_MAX_DMA_CHANNELS);

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		                                   ch_name);
		base = devm_ioremap_resource(data->dev, res);
		if (IS_ERR(base)) {
			netdev_err(data->netdev, "missing \"%s\"\n", ch_name);
			ret = PTR_ERR(base);
			break;
		}

		chan = rcm_mgeth_dma_chan_create(data->netdev, ch_name, base,
		                                 (i % RCM_MGETH_MAX_DMA_CHANNELS),
		                                 dir);

		if (!chan) {
			ret = -EFAULT;
			break;
		}

		if (i < RCM_MGETH_MAX_DMA_CHANNELS)
			data->tx[i] = chan;
		else
			data->rx[i - RCM_MGETH_MAX_DMA_CHANNELS] = chan;
	}

	if (ret != 0)
		rcm_mgeth_free_dma(data);

	return ret;
}

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

	err = rcm_mgeth_prep_dma(data, pdev);

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

	netif_napi_add(netdev, &data->napi, rcm_mgeth_poll, 64);

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

MODULE_DEVICE_TABLE(of, rcm_mgeth_of_match);

static struct platform_driver rcm_mgeth_driver = {
	.driver = {
		.name	= "rcm-mgeth",
		.of_match_table = rcm_mgeth_of_match,
	},
	.probe		= rcm_mgeth_probe,
	.remove		= rcm_mgeth_remove,
};

module_platform_driver(rcm_mgeth_driver);

#endif /* CONFIG_BASIS_PLATFORM */

MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_DESCRIPTION("RC-Module MGETH Driver");
MODULE_LICENSE("GPL");
