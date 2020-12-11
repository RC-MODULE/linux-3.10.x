/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

#ifndef __RCM_RMACE_INCLUDED_
#define __RCM_RMACE_INCLUDED_

#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/dmaengine.h>

#define RCM_RMACE_MAX_KEY_SIZE_8 4 // in 8-byte words
#define RCM_RMACE_MAX_IV_SIZE_8 2 // in 8-byte words

#define RCM_RMACE_HW_DESC_COUNT 1024
#define RCM_RMACE_CRYPTO_DESC_COUNT (RCM_RMACE_HW_DESC_COUNT - 1)
#define RCM_RMACE_ASYNC_TX_DESC_COUNT (RCM_RMACE_HW_DESC_COUNT - 1)
#define RCM_RMACE_MAX_DATA_TRANSFER (64 * 1024 * 1024 - 1)

#define RCM_RMACE_CTX_FLAGS_RESET BIT(0) // reset is needed before the operation

#define RCM_RMACE_CTX_STATUS_SCHEDULED  BIT(0) // !EXECUTING && !FINISHED
#define RCM_RMACE_CTX_STATUS_EXECUTING  BIT(1) // !SCHEDULED && !FINISHED
#define RCM_RMACE_CTX_STATUS_FINISHED   BIT(2) // !SCHEDULED && !EXECUTING
#define RCM_RMACE_CTX_STATUS_SUCCESS    BIT(3) // FINISHED && !TERMINATED && !ERROR
#define RCM_RMACE_CTX_STATUS_TERMINATED BIT(4) // FINISHED && !SUCCESS && !ERROR
#define RCM_RMACE_CTX_STATUS_ERROR      BIT(5) // FINISHED && !SUCCESS && !TERMINATED

#ifdef CONFIG_CRYPTO_RCM_RMACE
// ??? struct rcm_rmace_crypto;
#endif

#ifdef CONFIG_RCM_RMACE_DMA
// ??? struct rcm_rmace_dma_dev;
#endif

// ??? make flags instead bool
struct rcm_rmace_desc_info {
	dma_addr_t address;
	int length;
	bool valid;
	bool int_on;
	bool act0;
	bool act1;
	bool act2;
};

struct rcm_rmace_dev;

struct rcm_rmace_ctx
{
	struct rcm_rmace_dev *rmace;
	unsigned flags; // RCM_RMACE_CTX_FLAGSS*
	unsigned status; // RCM_RMACE_CTX_STATUS*

	unsigned src_desc_count;
	struct rcm_rmace_desc_info *src_desc_infos;

	unsigned dst_desc_count;
	struct rcm_rmace_desc_info *dst_desc_infos;

	// executed in a threaded interrupt context
	// ??? void (*started_callback)(struct rcm_rmace_ctx *ctx, void *arg);
	void (*finished_callback)(struct rcm_rmace_ctx *ctx, void *arg); // ??? name
	void *callbacks_arg; // ??? need ??? name without s

	struct list_head scheduled_list;
};

struct rcm_rmace_hw_desc {
	u32 data; // LE
	u32 address; // LE
} __attribute__((packed, aligned(8)));

struct rcm_rmace_dma_data {
	struct rcm_rmace_hw_desc src_hw_descs[RCM_RMACE_HW_DESC_COUNT];
	struct rcm_rmace_hw_desc dst_hw_descs[RCM_RMACE_HW_DESC_COUNT];
#ifdef CONFIG_CRYPTO_RCM_RMACE
	u64 control_block[1 /* header */ + RCM_RMACE_MAX_KEY_SIZE_8 + RCM_RMACE_MAX_IV_SIZE_8]; // LE  ??? name
#endif
#ifdef CONFIG_RCM_RMACE_DMA
	u64 memcpy_control_block;
#endif
};

struct rcm_rmace_dma_async_tx_desc
{
	struct dma_async_tx_descriptor async_tx;
	struct rcm_rmace_ctx ctx;
	struct rcm_rmace_desc_info src_desc_infos[2]; // config, source buffer
	struct rcm_rmace_desc_info dst_desc_info; // destination buffer
	struct list_head list; // free list, pending list, active list
};

// ???
struct rcm_rmace_dma_chan
{
	struct dma_chan dma_chan;
	struct rcm_rmace_dma_async_tx_desc async_tx_descs[RCM_RMACE_ASYNC_TX_DESC_COUNT];
	spinlock_t list_lock;
	struct list_head free_list; // protected by list_lock ??? need
	struct list_head pending_list; // protected by list_lock ??? need
	struct list_head active_list; // protected by list_lock ??? need
	struct list_head complete_list; // protected by list_lock ??? need
	// ??? dma_cookie_t last_cookie; // protected by list_lock ???
	// ??? dma_cookie_t used_cookie; // protected by list_lock ???
};

struct rcm_rmace_dev
{
	struct platform_device *pdev;
	struct regmap *control;
	spinlock_t reg_lock;
	void __iomem *base;
	u32 device_version;
	struct rcm_rmace_dma_data *dma_data;
	dma_addr_t dma_data_phys_addr;

	spinlock_t scheduled_lock;
	struct list_head scheduled_ctx_list; // protected by scheduled_lock
	struct rcm_rmace_ctx *current_ctx; // protected by scheduled_lock // ??? cur or curr

#ifdef CONFIG_CRYPTO_RCM_RMACE
	struct crypto_engine *engine;
	struct skcipher_request *crypto_cur_req; // ??? or curr
	int crypto_cur_req_src_nents;
	int crypto_cur_req_dst_nents;
	struct rcm_rmace_ctx crypto_ctx;
	struct rcm_rmace_desc_info crypto_src_desc_infos[RCM_RMACE_CRYPTO_DESC_COUNT];
	struct rcm_rmace_desc_info crypto_dst_desc_infos[RCM_RMACE_CRYPTO_DESC_COUNT];
#endif

#ifdef CONFIG_RCM_RMACE_DMA
	struct dma_device dma_dev;
	unsigned dma_channel_count;
	struct rcm_rmace_dma_chan *dma_channels;
#endif
};

// Pointer to a single RCM RMACE device (the driver allows only one device).
// The poiner can be NULL if the device is not probed.
// It is used by other devices for direct transactions (for exmaple RCM VDU).
extern struct rcm_rmace_dev *rcm_rmace_dev_single_ptr;

void rcm_rmace_ctx_init(struct rcm_rmace_dev *rmace, struct rcm_rmace_ctx *ctx);

// The function can be called from an interrupt context.
void rcm_rmace_ctx_schelude(struct rcm_rmace_ctx *ctx);

// The function can be called from an interrupt context.
// The function does not wait for the end of the termination.
// The function does nothing if the operation already done (the callback
// function is called with SUCCESS status).
// Then the termnitation is ready the callback function is called
// with TERMINATED status.
void rcm_rmace_ctx_terminate(struct rcm_rmace_ctx *ctx);

#ifdef CONFIG_CRYPTO_RCM_RMACE
// implemented in driver/crypto/rcm-rmace-crypto.c
int rcm_rmace_crypto_register(struct rcm_rmace_dev *rmace);
void rcm_rmace_crypto_unregister(struct rcm_rmace_dev *rmace);
#endif

#ifdef CONFIG_RCM_RMACE_DMA
// implemented in driver/dma/rcm-rmace-dma.c
int rcm_rmace_dma_register(struct rcm_rmace_dev *rmace);
void rcm_rmace_dma_unregister(struct rcm_rmace_dev *rmace);
#endif

#endif // __RCM_RMACE_INCLUDED_
