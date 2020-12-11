/*
 * RCM SoCs RMACE crypto interface.
 *
 * Copyright (c) 2019, AstroSoft.  All rights reserved.
 *
 * Author: Mikhail Petrov <Mikhail.Petrov@dev.mir>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#define DEBUG

#define pr_fmt(fmt) "rcm-rmace-crypto: " fmt

#include <linux/module.h>
#include <linux/rcm-rmace.h>

#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/if_alg.h>
#include <crypto/internal/skcipher.h>

// ??? mask - no shift
#define HEADER_ENC_DEC_BIT_POS 32
#define HEADER_ENDIAN_BIT_POS 36
#define HEADER_MODE_BIT_POS 40
#define HEADER_FEATURES_BIT_POS 48
#define HEADER_TYPE_BIT_POS 56

// ??? mask
#define ALG_MODE_DECRYPT BIT(0)
#define ALG_MODE_DES BIT(1)
#define ALG_MODE_DES3 BIT(2)
#define ALG_MODE_AES BIT(3)
#define ALG_MODE_ECB BIT(4)
#define ALG_MODE_CBC BIT(5)
#define ALG_MODE_OFB BIT(6)

struct rcm_rmace_crypto_ctx {
	struct crypto_engine_ctx enginectx;
	struct rcm_rmace_dev *rmace;
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[RCM_RMACE_MAX_KEY_SIZE_8];
};

struct rcm_rmace_crypto_reqctx {
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[RCM_RMACE_MAX_KEY_SIZE_8];
};

static void config_src_key_iv(
	struct rcm_rmace_dev *rmace,
	struct skcipher_request *req,
	struct rcm_rmace_ctx *hw_ctx)
{
	struct rcm_rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);
	struct rcm_rmace_desc_info *info = &hw_ctx->src_desc_infos[hw_ctx->src_desc_count++];
	u64 *p = rmace->dma_data->control_block;
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
		hw_ctx->flags |= RCM_RMACE_CTX_FLAGS_RESET; // [***] <-- work only with this line (terrible)
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

	s = (u64*)rmace->crypto_cur_req->iv;
	for (i = 0; i < req_ctx->iv_size_8; ++i) {
		*(p++) = __cpu_to_le64(*(s++));
	}

	// ??? clear_desc_info(&info);
	memset(info, 0, sizeof *info); // ??? no if masks
	info->address =	rmace->dma_data_phys_addr + offsetof(struct rcm_rmace_dma_data, control_block);
	info->length = (p - rmace->dma_data->control_block) * 8;
	info->valid = true;
	info->act2 = true;
}

static void config_dst_key_iv(
	struct rcm_rmace_dev *rmace,
	struct skcipher_request *req,
	struct rcm_rmace_ctx *hw_ctx)
{
	struct rcm_rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);
	struct rcm_rmace_desc_info *info = &hw_ctx->dst_desc_infos[hw_ctx->dst_desc_count++];

	// ??? clear_desc_info(&info);
	memset(info, 0, sizeof *info); // ??? no if masks
	info->address =	rmace->dma_data_phys_addr + offsetof(struct rcm_rmace_dma_data, control_block);
	info->length = (1 + req_ctx->key_size_8 + req_ctx->iv_size_8) * 8;
	info->valid = true;
	info->act2 = true;
}

static int config_src_data(
	struct rcm_rmace_dev *rmace,
	struct scatterlist *sglist,
	int nents,
	struct rcm_rmace_ctx *hw_ctx)
{
	struct scatterlist *sg;
	struct rcm_rmace_desc_info *info;
	int i;

	// ??? pr_debug("config_src_data: address=0x%08X\n", (unsigned)sg_dma_address(sglist)); // ???

	for_each_sg(sglist, sg, nents, i) {
		info = &hw_ctx->src_desc_infos[hw_ctx->src_desc_count++];
		memset(info, 0, sizeof *info); // ??? no if masks
		info->address =	sg_dma_address(sg);
		if ((info->address & 7) != 0) // check for alignment
			return -EINVAL;
		info->length = sg_dma_len(sg);
		info->act0 = (i + 1 == nents); // [***] !!!
		info->act2 = true;
	}

	return 0;
}

static int config_dst_data( // ???? like src
	struct rcm_rmace_dev *rmace,
	struct scatterlist *sglist,
	int nents,
	struct rcm_rmace_ctx *hw_ctx)
{
	struct scatterlist *sg;
	struct rcm_rmace_desc_info *info;
	int i;

	// ??? pr_debug("config_dst_data: address=0x%08X\n", (unsigned)sg_dma_address(sglist)); // ???

	for_each_sg(sglist, sg, nents, i) {
		info = &hw_ctx->dst_desc_infos[hw_ctx->dst_desc_count++];
		memset(info, 0, sizeof *info); // ??? no if masks
		info->address =	sg_dma_address(sg);
		if ((info->address & 7) != 0) // check for alignment
			return -EINVAL;
		info->length = sg_dma_len(sg);
		info->act0 = (i + 1 == nents); // [***] !!!
		info->act2 = true;
	}

	return 0;
}

static void rcm_rmace_finished_callback(struct rcm_rmace_ctx *ctx, void *arg)
{
	struct rcm_rmace_dev *rmace = ctx->rmace;
	struct rcm_rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(rmace->crypto_cur_req);
	u64 *p, *d;
	unsigned i;

	// ??? pr_debug("rcm_rmace_req_callback: ctx=0x%08X, arg=0x%08X\n", (unsigned)ctx, (unsigned)arg);

	if ((ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) != 0) {
		// save configuration
		p = rmace->dma_data->control_block + 1 /* header */ + req_ctx->key_size_8;
		d = (u64*)rmace->crypto_cur_req->iv;
		for (i = 0; i < req_ctx->iv_size_8; ++i)
			*(d++) = __le64_to_cpu(*(p++));
	}

	dma_unmap_sg(&rmace->pdev->dev, rmace->crypto_cur_req->src, rmace->crypto_cur_req_src_nents, DMA_TO_DEVICE);
	dma_unmap_sg(&rmace->pdev->dev, rmace->crypto_cur_req->dst, rmace->crypto_cur_req_dst_nents, DMA_FROM_DEVICE);

	crypto_finalize_skcipher_request(rmace->engine, rmace->crypto_cur_req,
		(ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) != 0 ? 0 : -EFAULT); // cur_req -> arg ????
	// ??? ctx->rmace->crypto_cur_req = NULL; // ??? need ???
}

// ??? delete pr_debug in all

// ??? test names of functions (no rcm_rmace)
static int rcm_rmace_cipher_one_req(
	struct crypto_engine *engine,
	void *areq)
{	
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct rcm_rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct rcm_rmace_dev *rmace = crypto_ctx->rmace;
	struct rcm_rmace_ctx *hw_ctx = &rmace->crypto_ctx;
	int src_mapped_nents;
	int dst_mapped_nents;
	int ret;

	rmace->crypto_cur_req = req; // ??? optimize req

	// check max length
	if (req->cryptlen > ALG_MAX_PAGES * PAGE_SIZE)
		return -EINVAL;

	// calculate source/destincation length in nents
	rmace->crypto_cur_req_src_nents = sg_nents_for_len(req->src, req->cryptlen);
	if (rmace->crypto_cur_req_src_nents < 0)
		return rmace->crypto_cur_req_src_nents;
	rmace->crypto_cur_req_dst_nents = sg_nents_for_len(req->dst, req->cryptlen);
	if (rmace->crypto_cur_req_dst_nents < 0)
		return rmace->crypto_cur_req_dst_nents;

	// map source buffers
	ret = dma_map_sg(&rmace->pdev->dev, req->src, rmace->crypto_cur_req_src_nents, DMA_TO_DEVICE);
	if (ret < 0)
		return ret;
	src_mapped_nents = ret;

	// map destination buffers
	ret = dma_map_sg(&rmace->pdev->dev, req->dst, rmace->crypto_cur_req_dst_nents, DMA_TO_DEVICE);
	if (ret < 0)
		goto unmap_src;
	dst_mapped_nents = ret;

	hw_ctx->flags = 0;

	// no descriptors yet
	hw_ctx->src_desc_count = 0;
	hw_ctx->dst_desc_count = 0;

	config_src_key_iv(rmace, req, hw_ctx);
	ret = config_src_data(rmace, req->src, src_mapped_nents, hw_ctx);
	if (ret != 0)
		goto unmap_dst;

	ret = config_dst_data(rmace, req->dst, dst_mapped_nents, hw_ctx);
	if (ret != 0)
		goto unmap_dst;
	config_dst_key_iv(rmace, req, hw_ctx);

	rcm_rmace_ctx_schelude(hw_ctx);

	return 0;

unmap_dst:
	dma_unmap_sg(&rmace->pdev->dev, req->dst, rmace->crypto_cur_req_dst_nents, DMA_FROM_DEVICE);

unmap_src:
	dma_unmap_sg(&rmace->pdev->dev, req->src, rmace->crypto_cur_req_src_nents, DMA_TO_DEVICE);

	return ret;
}

static int rcm_rmace_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	struct rcm_rmace_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->key_size_8 = key_size / 8;
	memcpy(ctx->key, key, key_size);

	return 0;
}

static int rcm_rmace_crypt(struct skcipher_request *req, bool decrypt)
{
	struct rcm_rmace_crypto_ctx *ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	struct rcm_rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);

	req_ctx->alg_mode = ctx->alg_mode;
	if (decrypt)
		req_ctx->alg_mode |= ALG_MODE_DECRYPT;
	req_ctx->key_size_8 = ctx->key_size_8;
	req_ctx->iv_size_8 = ctx->iv_size_8;
	memcpy(req_ctx->key, ctx->key, ctx->key_size_8 * 8);

	return crypto_transfer_skcipher_request_to_engine(
		ctx->rmace->engine, req);
}

static int rcm_rmace_crypto_ctx_init(
	struct crypto_skcipher *tfm,
	unsigned alg_mode,
	unsigned default_key_size,
	unsigned default_iv_size)
{
	struct rcm_rmace_crypto_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->rmace =  rcm_rmace_dev_single_ptr;
	if (ctx->rmace == NULL)
		return -ENODEV;

	ctx->alg_mode = alg_mode;
	ctx->key_size_8 = default_key_size / 8;
	ctx->iv_size_8 = default_iv_size / 8;

	crypto_skcipher_set_reqsize(tfm, sizeof(struct rcm_rmace_crypto_reqctx));

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
 	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_ECB,
		DES_KEY_SIZE, 0);
}

static int rcm_rmace_des_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_CBC,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES | ALG_MODE_OFB,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des3_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_ECB,
		DES_KEY_SIZE, 0);
}

static int rcm_rmace_des3_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_CBC,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_des3_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_DES3 | ALG_MODE_OFB,
		DES_KEY_SIZE, DES_BLOCK_SIZE);
}

static int rcm_rmace_aes_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_ECB,
		AES_KEYSIZE_128, 0);
}

static int rcm_rmace_aes_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_CBC,
		AES_KEYSIZE_128, AES_BLOCK_SIZE);
}

static int rcm_rmace_aes_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rcm_rmace_crypto_ctx_init(tfm, ALG_MODE_AES | ALG_MODE_OFB,
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
		.base.cra_flags = CRYPTO_ALG_ASYNC, // ??? CRYPTO_ALG_TYPE_SKCIPHER
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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
		.base.cra_alignmask = 7,
		.base.cra_ctxsize = sizeof(struct rcm_rmace_crypto_ctx),
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

int rcm_rmace_crypto_register(struct rcm_rmace_dev *rmace)
{
	int ret;

	rcm_rmace_ctx_init(rmace, &rmace->crypto_ctx);
	rmace->crypto_ctx.src_desc_infos = rmace->crypto_src_desc_infos;
	rmace->crypto_ctx.dst_desc_infos = rmace->crypto_dst_desc_infos;
	rmace->crypto_ctx.finished_callback = rcm_rmace_finished_callback; // ??? name

	rmace->engine = crypto_engine_alloc_init(&rmace->pdev->dev, true);
	if (!rmace->engine) {
		pr_err("could not init crypto engine\n");
		return -ENOMEM;
	}

	ret = crypto_engine_start(rmace->engine);
	if (ret) {
		pr_err("could not start crypto engine\n");
		goto free_engine;
	}

	ret = crypto_register_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
	if (ret) {
		pr_err("could not register algs\n");
		goto free_engine;
	}

	pr_info("crypto interface registered successfully\n");

	return 0;

free_engine:
	crypto_engine_exit(rmace->engine);

	return ret;
}
EXPORT_SYMBOL(rcm_rmace_crypto_register);

void rcm_rmace_crypto_unregister(struct rcm_rmace_dev *rmace)
{
	crypto_unregister_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
	crypto_engine_exit(rmace->engine);
}
EXPORT_SYMBOL(rcm_rmace_crypto_unregister);

static int __init rcm_rmace_crypto_init(void)
{
	return 0;
}

static void __exit rcm_rmace_crypto_exit(void)
{
}

module_init(rcm_rmace_crypto_init);
module_exit(rcm_rmace_crypto_exit);

MODULE_DESCRIPTION("RCM SoCs RMACE crypto interface");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_LICENSE("GPL v2");
