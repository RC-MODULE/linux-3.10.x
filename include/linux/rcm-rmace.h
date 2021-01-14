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

#define RCM_RMACE_HW_DESC_COUNT 2048
#define RCM_RMACE_HARDWARE_ALIGN_MASK 0x7

#define RCM_RMACE_CTX_STATUS_SCHEDULED BIT(0) // !EXECUTING && !FINISHED
#define RCM_RMACE_CTX_STATUS_EXECUTING BIT(1) // !SCHEDULED && !FINISHED
#define RCM_RMACE_CTX_STATUS_FINISHED BIT(2) // !SCHEDULED && !EXECUTING
#define RCM_RMACE_CTX_STATUS_SUCCESS BIT(3) // FINISHED

#define RCM_RMACE_DESC_VALID_MASK BIT(0)
#define RCM_RMACE_DESC_ERR_MASK BIT(1)
#define RCM_RMACE_DESC_INT_MASK BIT(2)
#define RCM_RMACE_DESC_ACT0_MASK BIT(3)
#define RCM_RMACE_DESC_ACT1_MASK BIT(4)
#define RCM_RMACE_DESC_ACT2_MASK BIT(5)
#define RCM_RMACE_DESC_LEN_SHIFT 6

#define RCM_RMACE_HEADER_ENC_DEC_SHIFT 32
#define RCM_RMACE_HEADER_ENDIAN_SHIFT 36
#define RCM_RMACE_HEADER_MODE_SHIFT 40
#define RCM_RMACE_HEADER_FEATURES_SHIFT 48
#define RCM_RMACE_HEADER_TYPE_SHIFT 56

#ifdef CONFIG_CRYPTO_RCM_RMACE
// implemented in driver/crypto/rcm-rmace-crypto.c
struct rcm_rmace_icrypto;
#endif

#ifdef CONFIG_RCM_RMACE_DMA
// implemented in driver/dma/rcm-rmace-dma.c
struct rcm_rmace_idma;
#endif

struct rcm_rmace_hw_desc {
	u32 data; // LE, RCM_RMACE_DESC*
	u32 address; // LE
} __attribute__((packed, aligned(8)));

struct rcm_rmace_dev;

struct rcm_rmace_ctx
{
	struct rcm_rmace_dev *rmace;
	unsigned status; // RCM_RMACE_CTX_STATUS*

	unsigned src_desc_count;
	struct rcm_rmace_hw_desc *src_descs;

	unsigned dst_desc_count;
	struct rcm_rmace_hw_desc *dst_descs;

	// executed in a threaded interrupt context
	void (*callback)(struct rcm_rmace_ctx *ctx, void *arg);
	void *callback_arg;

	struct list_head scheduled_list;
};

struct rcm_rmace_dma_data {
	struct rcm_rmace_hw_desc src_hw_descs[RCM_RMACE_HW_DESC_COUNT];
	struct rcm_rmace_hw_desc dst_hw_descs[RCM_RMACE_HW_DESC_COUNT];
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
	struct rcm_rmace_ctx *cur_ctx; // protected by scheduled_lock

#ifdef CONFIG_CRYPTO_RCM_RMACE
	struct rcm_rmace_icrypto *icrypto;
#endif

#ifdef CONFIG_RCM_RMACE_DMA
	struct rcm_rmace_idma *idma;
#endif
};

// Pointer to a single RCM RMACE device (the driver allows only one device).
// The poiner can be NULL if the device is not probed.
// It is used by other devices for direct transactions (for exmaple RCM VDU).
extern struct rcm_rmace_dev *rcm_rmace_dev_single_ptr;

void rcm_rmace_ctx_init(struct rcm_rmace_dev *rmace, struct rcm_rmace_ctx *ctx);

// The function can be called from an interrupt context.
void rcm_rmace_ctx_schedule(struct rcm_rmace_ctx *ctx);

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
