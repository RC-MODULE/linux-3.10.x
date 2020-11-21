// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#include <linux/netdevice.h>
#include <linux/regmap.h>
#include <linux/phy.h>
#include <linux/of_device.h>

#ifdef CONFIG_BASIS_PLATFORM
#	include "../../../misc/rcm/basis/basis-device.h"
#	include "../../../misc/rcm/basis/basis-controller.h"
#	include "../../../misc/rcm/basis/basis-cfs.h"
#endif

#define RCM_MGETH_MAX_DMA_CHANNELS 4

#define RCM_MGETH_MIN_FRAME_SIZE 60
#define RCM_MGETH_MAX_FRAME_SIZE 10240

struct rcm_mgeth_dma_chan;

struct rcm_mgeth_data {
	struct device      *dev;
	struct net_device  *netdev;
	struct napi_struct  napi;
	struct regmap      *reg;
	struct regmap      *mask[RCM_MGETH_MAX_DMA_CHANNELS];

	int                 irq;

	struct phy_device  *phydev;
	struct device_node *phynode;
	phy_interface_t     phy_if;

	int speed;
	int duplex;
	int link;

	struct rcm_mgeth_dma_chan *tx[RCM_MGETH_MAX_DMA_CHANNELS];
	struct rcm_mgeth_dma_chan *rx[RCM_MGETH_MAX_DMA_CHANNELS];

#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device    *device;
	u32                     regs;
	u32                     regs_size;
	u32                     reg_rx_mask[RCM_MGETH_MAX_DMA_CHANNELS];
	u32                     reg_rx_mask_size;
	u32                     reg_rx_chan[RCM_MGETH_MAX_DMA_CHANNELS];
	u32                     reg_rx_chan_size;
	u32                     reg_tx_chan[RCM_MGETH_MAX_DMA_CHANNELS];
	u32                     reg_tx_chan_size;
	u32                     hwirq;
	char                    phy[64];
	char                    phy_dev[64];
	char                    phy_mode[64];
#endif
};

typedef void (*rcm_mgeth_dma_tx_callback)(struct rcm_mgeth_dma_chan*, int,   u32);
typedef void (*rcm_mgeth_dma_rx_callback)(struct rcm_mgeth_dma_chan*, void*, u32);

bool rcm_mgeth_dma_busy(struct rcm_mgeth_dma_chan *chan);
void rcm_mgeth_dma_irq(struct rcm_mgeth_dma_chan *chan);
struct net_device *rcm_mgeth_dma_get_net_device(struct rcm_mgeth_dma_chan *chan);
int rcm_mgeth_dma_get_number(struct rcm_mgeth_dma_chan *chan);
bool rcm_mgeth_dma_submit_tx(struct rcm_mgeth_dma_chan *chan,
                             struct sk_buff *skb,
                             rcm_mgeth_dma_tx_callback callback);
int rcm_mgeth_dma_start_rx(struct rcm_mgeth_dma_chan *chan,
                           rcm_mgeth_dma_rx_callback callback);
void rcm_mgeth_dma_stop_rx(struct rcm_mgeth_dma_chan *chan);
struct rcm_mgeth_dma_chan *rcm_mgeth_dma_chan_create(struct net_device *netdev,
                                                     char *name,
                                                     void __iomem *regs,
                                                     int num,
                                                     enum dma_data_direction dir);
void rcm_mgeth_dma_chan_remove(struct rcm_mgeth_dma_chan *chan);
void rcm_mgeth_dma_enable_irq(struct rcm_mgeth_dma_chan *chan);
void rcm_mgeth_dma_disable_irq(struct rcm_mgeth_dma_chan *chan);
int rcm_mgeth_dma_rx_poll(struct rcm_mgeth_dma_chan *chan, int budget);
