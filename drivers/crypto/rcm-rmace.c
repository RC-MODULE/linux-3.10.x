/*
 * RCM SoCs RMACE crypto block core driver.
 *
 * Copyright (c) 2019, AstroSoft.  All rights reserved.
 *
 * Author: Mikhail Petrov <Mikhail.Petrov@dev.mir>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/internal/skcipher.h>

#define MAX_KEY_SIZE_8 4 /* in 8-byte words */
#define MAX_IV_SIZE_8 2 /* in 8-byte words */

#define HSIF_REG_HSIF_ID_REG 0x000
#define HSIF_REG_RMACE_RADDR_EXTEND 0x018
#define HSIF_REG_RMACE_WADDR_EXTEND 0x01C

#define RMACE_REG_ID 0x000
#define RMACE_REG_VERSION 0x004
#define RMACE_REG_SW_RST 0x008

#define RMACE_REG_ADMA_SW_RST 0x808
#define RMACE_REG_ADMA_CH_STATUS 0x80C

#define RMACE_REG_RDMA_SETTINGS 0x900
#define RMACE_REG_RDMA_STATUS 0x904
#define RMACE_REG_RDMA_SYS_ADDR 0x908
#define RMACE_REG_RDMA_TBL_SIZE 0x90C
// [***] #define RMACE_REG_RDMA_ADDR 0x910
// [***] #define RMACE_REG_RDMA_LEN 0x914
#define RMACE_REG_RDMA_DESC_ADDR 0x918

#define RMACE_REG_WDMA_SETTINGS 0xA00
#define RMACE_REG_WDMA_STATUS 0xA04
#define RMACE_REG_WDMA_SYS_ADDR 0xA08
#define RMACE_REG_WDMA_TBL_SIZE 0xA0C
// [***] #define RMACE_REG_WDMA_ADDR 0xA10
// [***] #define RMACE_REG_WDMA_LEN 0xA14
#define RMACE_REG_WDMA_DESC_ADDR 0xA18

#define DMA_CH_STATUS_RDMA_IRQ_BIT_POS 0
#define DMA_CH_STATUS_WDMA_IRQ_BIT_POS 16

#define DMA_SETTINGS_BAD_DESC 2
#define DMA_SETTINGS_AXI_ERROR 5
#define DMA_SETTINGS_EN_DMA_BIT_POS 28
#define DMA_SETTINGS_EN_DMA_DESC_TBL_BIT_POS 29
#define DMA_SETTINGS_DMA_LONG_LEN_BIT_POS 30

#define DMA_STATUS_BAD_DESC 2
#define DMA_STATUS_AXI_ERROR 5

#define DESC_VALID_BIT_POS 0
#define DESC_ERR_BIT_POS 1
#define DESC_INT_BIT_POS 2
#define DESC_ACT0_BIT_POS 3
#define DESC_ACT1_BIT_POS 4
#define DESC_ACT2_BIT_POS 5
#define DESC_LEN_BIT_POS 6

#define HEADER_ENC_DEC_BIT_POS 32
#define HEADER_ENDIAN_BIT_POS 36
#define HEADER_MODE_BIT_POS 40
#define HEADER_FEATURES_BIT_POS 48
#define HEADER_TYPE_BIT_POS 56

#define ALG_MODE_DECRYPT BIT(0)
#define ALG_MODE_DES BIT(1)
#define ALG_MODE_DES3 BIT(2)
#define ALG_MODE_AES BIT(3)
#define ALG_MODE_ECB BIT(4)
#define ALG_MODE_CBC BIT(5)
#define ALG_MODE_OFB BIT(6)

struct desc_info {
	dma_addr_t address;
	int length;
	bool valid;
	bool int_on;
	bool act0;
	bool act1;
	bool act2;
};

struct dma_desc {
	u32 data; // LE
	u32 address; // LE
} __attribute__((packed, aligned(8)));

struct dma_buffers {
	u8 src_buffer[PAGE_SIZE * 4];
	u8 dst_buffer[PAGE_SIZE * 4];
	struct dma_desc src_descriptors[3]; // config, data, null
	struct dma_desc dst_descriptors[3]; // data, config, null
	u64 control_block[1 /* header */ + MAX_KEY_SIZE_8 + MAX_IV_SIZE_8];
};

struct rcm_rmace_dev {
	struct platform_device *pdev;

	struct regmap *control;
	void __iomem *base;

	u32 device_version;

	struct dma_buffers *buffers;
	dma_addr_t buffers_mapping;

	struct crypto_engine *engine;
	bool alg_is_registered;

	bool error_status; // if set it is never cleared

	struct skcipher_request *cur_req;

	unsigned src_length;
	struct scatterlist *src_page;
	unsigned src_offset;

	unsigned dst_length;
	struct scatterlist *dst_page;
	unsigned dst_offset;
};

struct rcm_rmace_ctx {
	struct crypto_engine_ctx enginectx;
	struct rcm_rmace_dev *rmace_dev;
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[MAX_KEY_SIZE_8];
};

struct rcm_rmace_reqctx {
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[MAX_KEY_SIZE_8];
};

// /* [***] static void dump_regs(struct rcm_rmace_dev *rmace_dev)
// {
// 	dev_info(&rmace_dev->pdev->dev, "WDMA SETTINGS = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_SETTINGS));
// 	// dev_info(&rmace_dev->pdev->dev, "WDMA STATUS = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_STATUS));
// 	dev_info(&rmace_dev->pdev->dev, "WDMA SYS ADDR = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_SYS_ADDR));
// 	dev_info(&rmace_dev->pdev->dev, "WDMA TBL SIZE = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_TBL_SIZE));
// 	dev_info(&rmace_dev->pdev->dev, "WDMA ADDR = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_ADDR));
// 	dev_info(&rmace_dev->pdev->dev, "WDMA LEN = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_LEN));
// 	dev_info(&rmace_dev->pdev->dev, "WDMA DESC ADDR = %08X\n", ioread32(rmace_dev->base + RMACE_REG_WDMA_DESC_ADDR));
// }*/

static struct rcm_rmace_dev *last_rmace_dev = NULL;
static DEFINE_SPINLOCK(last_rmace_dev_lock);

static void reset_rmace(struct rcm_rmace_dev *rmace_dev)
{
	iowrite32(0x1, rmace_dev->base + RMACE_REG_SW_RST);
	while ((ioread32(rmace_dev->base + RMACE_REG_SW_RST) & 0x1) != 0) {
		// wait loop
	}
}

static void reset_dma(struct rcm_rmace_dev *rmace_dev)
{
	iowrite32(0x1, rmace_dev->base + RMACE_REG_ADMA_SW_RST);
	while ((ioread32(rmace_dev->base + RMACE_REG_ADMA_SW_RST) & 0x1) != 0) {
		// wait loop
	}
}

static void set_dev_error(
	struct rcm_rmace_dev *rmace_dev,
	const char *message)
{
	rmace_dev->error_status = true;

	reset_rmace(rmace_dev);
	reset_dma(rmace_dev);

	dev_err(&rmace_dev->pdev->dev, message);

	if (rmace_dev->cur_req != NULL) {
		struct skcipher_request *req = rmace_dev->cur_req;
		rmace_dev->cur_req = NULL;
		crypto_finalize_skcipher_request(rmace_dev->engine, req, EIO);
	}
}

static void clear_desc_info(struct desc_info *info) {
	memset(info, 0, sizeof *info);
}

static void set_desc_info(const struct desc_info *info, struct dma_desc *desc)
{
	u32 data = 0;
	data |= (info->length << DESC_LEN_BIT_POS);
	if (info->valid)
		data |= (1 << DESC_VALID_BIT_POS);
	if (info->int_on)
		data |= (1 << DESC_INT_BIT_POS);
	if (info->act0)
		data |= (1 << DESC_ACT0_BIT_POS);
	if (info->act1)
		data |= (1 << DESC_ACT1_BIT_POS);
	if (info->act2)
		data |= (1 << DESC_ACT2_BIT_POS);
	desc->data = __cpu_to_le32(data);

	desc->address = __cpu_to_le32((u32)info->address);
}

static void src_fill(struct rcm_rmace_dev *rmace_dev, bool config)
{
	struct rcm_rmace_reqctx *req_ctx =
		skcipher_request_ctx(rmace_dev->cur_req);
	struct dma_desc *descriptor = rmace_dev->buffers->src_descriptors;
	void *buffer = rmace_dev->buffers->src_buffer;
	unsigned free_bytes = sizeof rmace_dev->buffers->src_buffer;
	struct desc_info info;

	if (config) {
		// add the configuration descriptor
		u64 *p = rmace_dev->buffers->control_block;
		u64 *s;
		u64 data = 0; 
		unsigned i;

		if (req_ctx->alg_mode & ALG_MODE_DES)
			data |= (4ULL << HEADER_TYPE_BIT_POS); // des
		if (req_ctx->alg_mode & ALG_MODE_DES3) {
			data |= (5ULL << HEADER_TYPE_BIT_POS); // des3
			data |= (5ULL << HEADER_FEATURES_BIT_POS); // ede
		}
		if (req_ctx->alg_mode & ALG_MODE_AES) {
			switch (req_ctx->key_size_8)
			{
			case AES_KEYSIZE_128 / 8:
				data |= (6ULL << HEADER_TYPE_BIT_POS);
				break;
			case AES_KEYSIZE_192 / 8:
				data |= (7ULL << HEADER_TYPE_BIT_POS);
				break;
			case AES_KEYSIZE_256 / 8:
				data |= (8ULL << HEADER_TYPE_BIT_POS);
				break;
			default:
				panic("Unexpecting key size");
			}
		}
		if (req_ctx->alg_mode & ALG_MODE_CBC)
			data |= (1ULL << HEADER_MODE_BIT_POS); // cbc
		if (req_ctx->alg_mode & ALG_MODE_OFB)
			data |= (3ULL << HEADER_MODE_BIT_POS); // ofb
		if (req_ctx->alg_mode & ALG_MODE_DECRYPT)
			data |= (1ULL << HEADER_ENC_DEC_BIT_POS); // decrypt
		*(p++) = __cpu_to_le64(data);

		s = req_ctx->key;
		for (i = 0; i < req_ctx->key_size_8; ++i) {
			*(p++) = __cpu_to_le64(*(s++));
		}

		s = (u64*)rmace_dev->cur_req->iv;
		for (i = 0; i < req_ctx->iv_size_8; ++i) {
			*(p++) = __cpu_to_le64(*(s++));
		}

		clear_desc_info(&info);
		info.address =	rmace_dev->buffers_mapping +
			offsetof(struct dma_buffers, control_block);
		info.length = (p - rmace_dev->buffers->control_block) * 8;
		info.valid = true;
		info.act2 = true;
		set_desc_info(&info, descriptor);
		++descriptor;
	}

	// fill the src buffer
	while ((free_bytes != 0) && (rmace_dev->src_length != 0)) {
		unsigned page_size = min(free_bytes,
			rmace_dev->src_page->length - rmace_dev->src_offset);
		sg_pcopy_to_buffer(rmace_dev->src_page, 1, buffer, page_size,
			rmace_dev->src_offset);
		free_bytes -= page_size;
		buffer += page_size;
		rmace_dev->src_length -= page_size;
		rmace_dev->src_offset += page_size;
		if (rmace_dev->src_offset == rmace_dev->src_page->length) {
			rmace_dev->src_page = sg_next(rmace_dev->src_page);
			rmace_dev->src_offset = 0;
		}
	}

	// add the data descriptor
	clear_desc_info(&info);
	info.address =	rmace_dev->buffers_mapping +
		offsetof(struct dma_buffers, src_buffer);
	info.length = buffer - (void*)rmace_dev->buffers->src_buffer;
	info.act0 = (rmace_dev->src_length == 0); // [***] !!!
	info.act2 = true;
	set_desc_info(&info, descriptor);
	++descriptor;

	// add an empty descriptor
	clear_desc_info(&info);
	set_desc_info(&info, descriptor);
	++descriptor;

	// program and start the DMA channel
	iowrite32(rmace_dev->buffers_mapping
			+ offsetof(struct dma_buffers, src_descriptors),
		rmace_dev->base + RMACE_REG_RDMA_SYS_ADDR);
	iowrite32(
		(void*)descriptor - (void*)rmace_dev->buffers->src_descriptors,
		rmace_dev->base + RMACE_REG_RDMA_TBL_SIZE);
	iowrite32(rmace_dev->buffers_mapping
			+ offsetof(struct dma_buffers, src_descriptors),
		rmace_dev->base + RMACE_REG_RDMA_DESC_ADDR);
	iowrite32(BIT(DMA_SETTINGS_BAD_DESC)
		| BIT(DMA_SETTINGS_AXI_ERROR)
		| BIT(DMA_SETTINGS_EN_DMA_BIT_POS)
		| BIT(DMA_SETTINGS_EN_DMA_DESC_TBL_BIT_POS)
		| BIT(DMA_SETTINGS_DMA_LONG_LEN_BIT_POS),
		rmace_dev->base + RMACE_REG_RDMA_SETTINGS);
}

static void dst_init(struct rcm_rmace_dev *rmace_dev)
{
	struct rcm_rmace_reqctx *req_ctx =
		skcipher_request_ctx(rmace_dev->cur_req);
	struct dma_desc *descriptor = rmace_dev->buffers->dst_descriptors;
	unsigned size = min(rmace_dev->dst_length,
		sizeof rmace_dev->buffers->dst_buffer);
	struct desc_info info;

	// add a data descriptor
	clear_desc_info(&info);
	info.address = rmace_dev->buffers_mapping +
		offsetof(struct dma_buffers, dst_buffer);
	info.length = size;
	info.act2 = true;
	set_desc_info(&info, descriptor);
	++descriptor;

	if ((rmace_dev->dst_length - size == 0)	&& (req_ctx->iv_size_8 != 0)) {
		// add save configuration descriptor
		clear_desc_info(&info);
		info.address =	rmace_dev->buffers_mapping +
			offsetof(struct dma_buffers, control_block);
		info.length = (1 + req_ctx->key_size_8 + req_ctx->iv_size_8)
			* 8;
		info.valid = true;
		info.act2 = true;
		set_desc_info(&info, descriptor);
		++descriptor;
	}

	// add an empty descriptor
	clear_desc_info(&info);
	set_desc_info(&info, descriptor);
	++descriptor;

	// program and start the DMA channel
	iowrite32(rmace_dev->buffers_mapping
			+ offsetof(struct dma_buffers, dst_descriptors),
		rmace_dev->base + RMACE_REG_WDMA_SYS_ADDR);
	iowrite32(
		(void*)descriptor - (void*)rmace_dev->buffers->dst_descriptors,
		rmace_dev->base + RMACE_REG_WDMA_TBL_SIZE);
	iowrite32(rmace_dev->buffers_mapping
			+ offsetof(struct dma_buffers, dst_descriptors),
		rmace_dev->base + RMACE_REG_WDMA_DESC_ADDR);
	iowrite32(BIT(DMA_SETTINGS_BAD_DESC)
		| BIT(DMA_SETTINGS_AXI_ERROR)
		| BIT(DMA_SETTINGS_EN_DMA_BIT_POS)
		| BIT(DMA_SETTINGS_EN_DMA_DESC_TBL_BIT_POS)
		| BIT(DMA_SETTINGS_DMA_LONG_LEN_BIT_POS),
		rmace_dev->base + RMACE_REG_WDMA_SETTINGS);
}

static void dst_read(struct rcm_rmace_dev *rmace_dev) {
	struct rcm_rmace_reqctx *req_ctx =
		skcipher_request_ctx(rmace_dev->cur_req);
	unsigned size = min(rmace_dev->dst_length,
		sizeof rmace_dev->buffers->dst_buffer);
	void *buffer = rmace_dev->buffers->dst_buffer;

	while (size != 0) {
		unsigned page_size = min(size,
			rmace_dev->dst_page->length - rmace_dev->dst_offset);
		sg_pcopy_from_buffer(rmace_dev->dst_page, 1, buffer, page_size,
			rmace_dev->dst_offset);
		size -= page_size;
		buffer += page_size;
		rmace_dev->dst_length -= page_size;
		rmace_dev->dst_offset += page_size;
		if (rmace_dev->dst_offset == rmace_dev->dst_page->length) {
			rmace_dev->dst_page = sg_next(rmace_dev->dst_page);
			rmace_dev->dst_offset = 0;
		}
	}

	if (rmace_dev->dst_length == 0) {
		// save configuration
		u64 *p = rmace_dev->buffers->control_block + 1 /* header */
			+ req_ctx->key_size_8;
		u64 *d = (u64*)rmace_dev->cur_req->iv;
		unsigned i;
		for (i = 0; i < req_ctx->iv_size_8; ++i)
			*(d++) = __le64_to_cpu(*(p++));
	}
}

static irqreturn_t rmace_irq(int irq, void *arg)
{
	struct rcm_rmace_dev *rmace_dev = arg;
	u32 irq_ch_status =
		ioread32(rmace_dev->base + RMACE_REG_ADMA_CH_STATUS);

	if ((irq_ch_status &
		(BIT(DMA_CH_STATUS_RDMA_IRQ_BIT_POS)
			| BIT(DMA_CH_STATUS_WDMA_IRQ_BIT_POS))) != 0)
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static irqreturn_t rmace_irq_thread(int irq, void *arg)
{
	struct rcm_rmace_dev *rmace_dev = arg;
	u32 rdma_status, wdma_status;

	rdma_status = ioread32(rmace_dev->base + RMACE_REG_RDMA_STATUS);

	if ((rdma_status & BIT(DMA_STATUS_AXI_ERROR)) != 0)
		set_dev_error(rmace_dev, "AXI read error");

	if (((rdma_status & BIT(DMA_STATUS_BAD_DESC)) != 0)
		&& (rmace_dev->src_length != 0)
		&& !rmace_dev->error_status)
		src_fill(rmace_dev, false);

	wdma_status = ioread32(rmace_dev->base + RMACE_REG_WDMA_STATUS);

	if ((wdma_status & BIT(DMA_STATUS_AXI_ERROR)) != 0)
		set_dev_error(rmace_dev, "AXI write error");

	if (((wdma_status & BIT(DMA_STATUS_BAD_DESC)) != 0)
		&& !rmace_dev->error_status) {
		dst_read(rmace_dev);
		if (rmace_dev->dst_length != 0)
			dst_init(rmace_dev);
		else {
			struct skcipher_request *req = rmace_dev->cur_req;
			rmace_dev->cur_req = NULL;
			crypto_finalize_skcipher_request(rmace_dev->engine,
				req, 0);
		}		
	}

	return IRQ_HANDLED;
}

static int rcm_rmace_cipher_one_req(
	struct crypto_engine *engine,
	void *areq)
{	
	struct skcipher_request *req = container_of(areq,
		struct skcipher_request,
		base);
	struct rcm_rmace_ctx *ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	struct rcm_rmace_dev *rmace_dev = ctx->rmace_dev;

	if (rmace_dev->error_status)
		return -EIO;

	reset_rmace(rmace_dev); // [***] <-- work only with this line (terrible)

	rmace_dev->cur_req = req;
	rmace_dev->src_length = req->cryptlen;
	rmace_dev->src_page = req->src;
	rmace_dev->src_offset = 0;
	rmace_dev->dst_length = req->cryptlen;
	rmace_dev->dst_page = req->dst;
	rmace_dev->dst_offset = 0;

	dst_init(rmace_dev);
	src_fill(rmace_dev, true);

	return 0;
}

static int rcm_rmace_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	struct rcm_rmace_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->key_size_8 = key_size / 8;
	memcpy(ctx->key, key, key_size);

	return 0;
}

static int rcm_rmace_crypt(struct skcipher_request *req, bool decrypt)
{
	struct rcm_rmace_ctx *ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	struct rcm_rmace_reqctx *req_ctx = skcipher_request_ctx(req);

	req_ctx->alg_mode = ctx->alg_mode;
	if (decrypt)
		req_ctx->alg_mode |= ALG_MODE_DECRYPT;
	req_ctx->key_size_8 = ctx->key_size_8;
	req_ctx->iv_size_8 = ctx->iv_size_8;
	memcpy(req_ctx->key, ctx->key, ctx->key_size_8 * 8);

	return crypto_transfer_skcipher_request_to_engine(
		ctx->rmace_dev->engine,	req);
}

static int rcm_rmace_ctx_init(
	struct crypto_skcipher *tfm,
	unsigned alg_mode,
	unsigned default_key_size,
	unsigned default_iv_size)
{
	struct rcm_rmace_ctx *ctx = crypto_skcipher_ctx(tfm);

	spin_lock(&last_rmace_dev_lock);
	ctx->rmace_dev =  last_rmace_dev;
	spin_unlock(&last_rmace_dev_lock);
	if (ctx->rmace_dev == NULL)
		return -ENODEV;

	ctx->alg_mode = alg_mode;
	ctx->key_size_8 = default_key_size / 8;
	ctx->iv_size_8 = default_iv_size / 8;

	crypto_skcipher_set_reqsize(tfm, sizeof(struct rcm_rmace_reqctx));

	ctx->enginectx.op.prepare_request = NULL;
	ctx->enginectx.op.unprepare_request = NULL;
	ctx->enginectx.op.do_one_request = rcm_rmace_cipher_one_req;

	return 0;
}

static int rcm_rmace_encrypt(struct skcipher_request *req)
{
	return rcm_rmace_crypt(req, false);
}

static int rcm_rmace_decrypt(struct skcipher_request *req)
{
	return rcm_rmace_crypt(req, true);
}

static int rcm_rmace_des_ecb_ctx_init(struct crypto_skcipher *tfm)
{
 	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_ECB,
		DES_KEY_SIZE, 0);
}

static int rcm_rmace_des_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_CBC,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_OFB,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des3_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_ECB,
		DES_KEY_SIZE, 0);
}

static int rcm_rmace_des3_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_CBC,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des3_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_OFB,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_aes_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_ECB,
		AES_KEYSIZE_128, 0);
}

static int rcm_rmace_aes_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_CBC,
		AES_KEYSIZE_128, AES_BLOCK_SIZE);
}

static int rcm_rmace_aes_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_OFB,
		AES_KEYSIZE_128, AES_BLOCK_SIZE);
}

static int rcm_rmace_des_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if (key_size != DES_KEY_SIZE)
		return -EINVAL;

	return rcm_rmace_setkey(tfm, key, key_size);
}

static int rcm_rmace_des3_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if (key_size != DES_KEY_SIZE * 3)
		return -EINVAL;

	return rcm_rmace_setkey(tfm, key, key_size);
}

static int rcm_rmace_aes_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if ((key_size != AES_KEYSIZE_128)
		&& (key_size != AES_KEYSIZE_192)
		&& (key_size != AES_KEYSIZE_256))
		return -EINVAL;

	return rcm_rmace_setkey(tfm, key, key_size);
}

static struct skcipher_alg crypto_algs[] = {
	{
		.base.cra_name = "ecb(des)",
		.base.cra_driver_name = "rmace-ecb-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des_ecb_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = 0,
		.setkey	= rcm_rmace_des_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "cbc(des)",
		.base.cra_driver_name = "rmace-cbc-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des_cbc_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = DES_BLOCK_SIZE,
		.setkey	= rcm_rmace_des_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "ofb(des)",
		.base.cra_driver_name = "rmace-ofb-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des_ofb_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = DES_BLOCK_SIZE,
		.setkey	= rcm_rmace_des_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "ecb(des3_ede)",
		.base.cra_driver_name = "rmace-ecb-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des3_ecb_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = 0,
		.setkey	= rcm_rmace_des3_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "cbc(des3_ede)",
		.base.cra_driver_name = "rmace-cbc-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des3_cbc_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = DES_BLOCK_SIZE,
		.setkey	= rcm_rmace_des3_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "ofb(des3_ede)",
		.base.cra_driver_name = "rmace-ofb-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_des3_ofb_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = DES_BLOCK_SIZE,
		.setkey	= rcm_rmace_des3_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "ecb(aes)",
		.base.cra_driver_name = "rmace-ecb-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_aes_ecb_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = 0,
		.setkey	= rcm_rmace_aes_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "cbc(aes)",
		.base.cra_driver_name = "rmace-cbc-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_aes_cbc_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = AES_BLOCK_SIZE,
		.setkey	= rcm_rmace_aes_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
	{
		.base.cra_name = "ofb(aes)",
		.base.cra_driver_name = "rmace-ofb-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rcm_rmace_aes_ofb_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = AES_BLOCK_SIZE,
		.setkey	= rcm_rmace_aes_setkey,
		.encrypt = rcm_rmace_encrypt,
		.decrypt = rcm_rmace_decrypt
	},
};

static void rcm_rmace_cleanup(struct rcm_rmace_dev *rmace_dev)
{
	if (rmace_dev->alg_is_registered)
		crypto_unregister_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));

	if (rmace_dev->engine != NULL)
		crypto_engine_exit(rmace_dev->engine);

	if (rmace_dev->buffers != NULL) {
		dma_free_coherent(&rmace_dev->pdev->dev,
			sizeof(struct dma_buffers),
			rmace_dev->buffers,
			rmace_dev->buffers_mapping);
	}

	spin_lock(&last_rmace_dev_lock);
	if (last_rmace_dev == rmace_dev)
		last_rmace_dev = NULL;
	spin_unlock(&last_rmace_dev_lock);
} 

static int rcm_rmace_probe_internal(struct rcm_rmace_dev *rmace_dev)
{
	int ret, irq;
	struct device_node *syscon_np;
	u32 control_id;
	u32 device_id;

	struct rcm_rmace_dev *last_rmace_dev_value;
	spin_lock(&last_rmace_dev_lock);
	last_rmace_dev_value = last_rmace_dev;
	if (last_rmace_dev_value == NULL)
		last_rmace_dev = rmace_dev;
	spin_unlock(&last_rmace_dev_lock);
	if (last_rmace_dev_value != NULL)
		return -ENODEV;

	syscon_np = of_parse_phandle(rmace_dev->pdev->dev.of_node,
		"control", 0);
	if (!syscon_np) {
		dev_err(&rmace_dev->pdev->dev, "no control property\n");
		return -ENODEV;
	}

	rmace_dev->control = syscon_node_to_regmap(syscon_np);
	if (IS_ERR(rmace_dev->control)) {
		dev_err(&rmace_dev->pdev->dev, "cannot get control regmap\n");
		return -ENODEV;
	}

	ret = regmap_read(rmace_dev->control, HSIF_REG_HSIF_ID_REG, &control_id);
	if (ret) {
		dev_err(&rmace_dev->pdev->dev,
			"control ID register read error\n");
		return -ENODEV;
	}
	if (control_id != 0x46495348) {
		dev_err(&rmace_dev->pdev->dev,
			"unknow control device ID: %08X\n", control_id);
		return -ENODEV;
	}

	rmace_dev->base = devm_ioremap_resource(&rmace_dev->pdev->dev,
		rmace_dev->pdev->resource);
	if (IS_ERR(rmace_dev->base)) {
		dev_err(&rmace_dev->pdev->dev, "register allocation failed\n");
		return -ENODEV;
	}

	reset_rmace(rmace_dev);
	reset_dma(rmace_dev);

	device_id = ioread32(rmace_dev->base + RMACE_REG_ID);
	if (device_id != 0x4543414D) {
		dev_err(&rmace_dev->pdev->dev, "unknow device ID: %08X\n",
			device_id);
		return -ENODEV;
	}

	irq = platform_get_irq(rmace_dev->pdev, 0);
	if (irq < 0) {
		dev_err(&rmace_dev->pdev->dev, "get IRQ resource error\n");
		return -ENODEV;
	}
	ret = devm_request_threaded_irq(&rmace_dev->pdev->dev, irq,
		rmace_irq, rmace_irq_thread, IRQF_ONESHOT, NULL, rmace_dev);
	if (ret) {
		dev_err(&rmace_dev->pdev->dev, "grab IRQ error\n");
		return -ENODEV;
	}

	rmace_dev->device_version = ioread32(
		rmace_dev->base + RMACE_REG_VERSION);

	// [***]
	ret = regmap_write(rmace_dev->control,
		HSIF_REG_RMACE_RADDR_EXTEND,
		0x1);
	if (ret) {
		dev_err(&rmace_dev->pdev->dev, "HSIF RDMA setup error\n");
		return -ENODEV;
	}
	ret = regmap_write(rmace_dev->control,
		HSIF_REG_RMACE_WADDR_EXTEND,
		0x1);
	if (ret) {
		dev_err(&rmace_dev->pdev->dev, "HSIF WDMA setup error\n");
		return -ENODEV;
	}

	rmace_dev->buffers = dma_alloc_coherent(&rmace_dev->pdev->dev,
		sizeof(struct dma_buffers), &rmace_dev->buffers_mapping,
		GFP_KERNEL);
	if (rmace_dev->buffers == NULL) {
		dev_err(&rmace_dev->pdev->dev, "cannot allocate DMA buffers");
		return -ENODEV;
	}

	rmace_dev->engine = crypto_engine_alloc_init(
		&rmace_dev->pdev->dev,
		true);
	if (!rmace_dev->engine) {
		dev_err(&rmace_dev->pdev->dev,
			"could not init crypto engine\n");
		return -ENOMEM;
	}

	ret = crypto_engine_start(rmace_dev->engine);
	if (ret) {
		dev_err(&rmace_dev->pdev->dev,
			"could not start crypto engine\n");
		return -ENODEV;
	}

	ret = crypto_register_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
	if (ret) {
		dev_err(&rmace_dev->pdev->dev, "could not register algs\n");
		return -ENODEV;
	}
	rmace_dev->alg_is_registered = true;

	return 0;
}

static int rcm_rmace_probe(struct platform_device *pdev)
{
	int ret;
	struct rcm_rmace_dev *rmace_dev;

	rmace_dev = devm_kzalloc(&pdev->dev, sizeof(*rmace_dev), GFP_KERNEL);
	if (!rmace_dev) {
		dev_err(&pdev->dev, "insufficient memory\n");
		return -ENOMEM;
	}

	rmace_dev->pdev = pdev;
	platform_set_drvdata(pdev, rmace_dev);

	ret = rcm_rmace_probe_internal(rmace_dev);
	if (ret) {
		rcm_rmace_cleanup(rmace_dev);
		return ret;
	}

	dev_info(&pdev->dev, "initialized (hardware version=%u.%u)",
		(rmace_dev->device_version >> 8) & 0xFF,
		(rmace_dev->device_version >> 0) & 0xFF);

	return 0;
}

static int rcm_rmace_remove(struct platform_device *pdev)
{
	struct rcm_rmace_dev *rmace_dev = platform_get_drvdata(pdev);

	rcm_rmace_cleanup(rmace_dev);

	return 0;	
}

static struct of_device_id rcm_rmace_id_table[] = {
	{ .compatible = "rcm,rmace" },
	{ },
};

MODULE_DEVICE_TABLE(of, rcm_rmace_id_table);

static struct platform_driver rcm_rmace_driver = {
	.driver = {
		.name = "rcm-rmace",
		.owner = THIS_MODULE,
		.of_match_table = rcm_rmace_id_table,
	},
	.probe = rcm_rmace_probe,
	.remove = rcm_rmace_remove,
};

module_platform_driver(rcm_rmace_driver);

MODULE_DESCRIPTION("RCM SoCs RMACE crypto block core driver");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_LICENSE("GPL v2");
