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

#define pr_fmt(fmt) "rcm-rmace-crypto: " fmt

#include <linux/module.h>
#include <linux/rcm-rmace.h>

#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/if_alg.h>
#include <crypto/internal/skcipher.h>

#define ALG_MODE_DECRYPT_MASK BIT(0)
#define ALG_MODE_DES_MASK BIT(1)
#define ALG_MODE_DES3_MASK BIT(2)
#define ALG_MODE_AES_MASK BIT(3)
#define ALG_MODE_ECB_MASK BIT(4)
#define ALG_MODE_CBC_MASK BIT(5)
#define ALG_MODE_OFB_MASK BIT(6)

struct dma_data
{
	u64 control_block[1 /* header */ + RCM_RMACE_MAX_KEY_SIZE_8 + RCM_RMACE_MAX_IV_SIZE_8]; // LE
};

struct rcm_rmace_icrypto
{
	struct rcm_rmace_dev *rmace;
	struct dma_data *dma_data;
	dma_addr_t dma_data_phys_addr;
	struct crypto_engine *engine;
	struct skcipher_request *cur_req;
	int cur_req_src_nents;
	int cur_req_dst_nents;
	struct rcm_rmace_ctx rmace_ctx;
	struct rcm_rmace_desc_info src_desc_infos[RCM_RMACE_CRYPTO_DESC_COUNT];
	struct rcm_rmace_desc_info dst_desc_infos[RCM_RMACE_CRYPTO_DESC_COUNT];
};

struct rmace_crypto_ctx
{
	struct crypto_engine_ctx enginectx;
	struct rcm_rmace_icrypto *icrypto;
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[RCM_RMACE_MAX_KEY_SIZE_8];
};

struct rmace_crypto_reqctx
{
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[RCM_RMACE_MAX_KEY_SIZE_8];
};

static void config_src_key_iv(
	struct rcm_rmace_icrypto *icrypto,
	struct skcipher_request *req,
	struct rcm_rmace_ctx *rmace_ctx)
{
	struct rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);
	struct rcm_rmace_desc_info *info = &rmace_ctx->src_desc_infos[rmace_ctx->src_desc_count++];
	u64 *p = icrypto->dma_data->control_block;
	u64 *s;
	u64 data = 0; 
	unsigned i;

	if (req_ctx->alg_mode & ALG_MODE_DES_MASK)
		data |= (4ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // des
	if (req_ctx->alg_mode & ALG_MODE_DES3_MASK) {
		data |= (5ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // des3
		data |= (5ULL << RCM_RMACE_HEADER_FEATURES_SHIFT); // ede
	}
	if (req_ctx->alg_mode & ALG_MODE_AES_MASK) {
		switch (req_ctx->key_size_8)
		{
		case AES_KEYSIZE_128 / 8:
			data |= (6ULL << RCM_RMACE_HEADER_TYPE_SHIFT);
			break;
		case AES_KEYSIZE_192 / 8:
			data |= (7ULL << RCM_RMACE_HEADER_TYPE_SHIFT);
			break;
		case AES_KEYSIZE_256 / 8:
			data |= (8ULL << RCM_RMACE_HEADER_TYPE_SHIFT);
			break;
		default:
			panic("Unexpecting key size");
		}
		rmace_ctx->flags |= RCM_RMACE_CTX_FLAGS_RESET; // [***] <-- work only with this line (terrible)
	}
	if (req_ctx->alg_mode & ALG_MODE_CBC_MASK)
		data |= (1ULL << RCM_RMACE_HEADER_MODE_SHIFT); // cbc
	if (req_ctx->alg_mode & ALG_MODE_OFB_MASK)
		data |= (3ULL << RCM_RMACE_HEADER_MODE_SHIFT); // ofb
	if (req_ctx->alg_mode & ALG_MODE_DECRYPT_MASK)
		data |= (1ULL << RCM_RMACE_HEADER_ENC_DEC_SHIFT); // decrypt
	*(p++) = __cpu_to_le64(data);

	s = req_ctx->key;
	for (i = 0; i < req_ctx->key_size_8; ++i) {
		*(p++) = __cpu_to_le64(*(s++));
	}

	s = (u64*)icrypto->cur_req->iv;
	for (i = 0; i < req_ctx->iv_size_8; ++i) {
		*(p++) = __cpu_to_le64(*(s++));
	}

	// ??? clear_desc_info(&info);
	memset(info, 0, sizeof *info); // ??? no if masks
	info->address =	icrypto->dma_data_phys_addr + offsetof(struct dma_data, control_block);
	info->length = (p - icrypto->dma_data->control_block) * 8;
	info->valid = true;
	info->act2 = true;
}

static void config_dst_key_iv(
	struct rcm_rmace_icrypto *icrypto,
	struct skcipher_request *req,
	struct rcm_rmace_ctx *rmace_ctx)
{
	struct rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);
	struct rcm_rmace_desc_info *info = &rmace_ctx->dst_desc_infos[rmace_ctx->dst_desc_count++];

	// ??? clear_desc_info(&info);
	memset(info, 0, sizeof *info); // ??? no if masks
	info->address = icrypto->dma_data_phys_addr + offsetof(struct dma_data, control_block);
	info->length = (1 + req_ctx->key_size_8 + req_ctx->iv_size_8) * 8;
	info->valid = true;
	info->act2 = true;
}

static int config_src_data(
	struct scatterlist *sglist,
	int nents,
	struct rcm_rmace_ctx *rmace_ctx)
{
	struct scatterlist *sg;
	struct rcm_rmace_desc_info *info;
	int i;

	for_each_sg(sglist, sg, nents, i) {
		info = &rmace_ctx->src_desc_infos[rmace_ctx->src_desc_count++];
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

static int config_dst_data(
	struct scatterlist *sglist,
	int nents,
	struct rcm_rmace_ctx *rmace_ctx)
{
	struct scatterlist *sg;
	struct rcm_rmace_desc_info *info;
	int i;

	for_each_sg(sglist, sg, nents, i) {
		info = &rmace_ctx->dst_desc_infos[rmace_ctx->dst_desc_count++];
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

static void rmace_callback(struct rcm_rmace_ctx *rmace_ctx, void *arg)
{
	struct rcm_rmace_icrypto *icrypto = container_of(rmace_ctx, struct rcm_rmace_icrypto, rmace_ctx);
	struct rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(icrypto->cur_req);
	u64 *p, *d;
	unsigned i;

	if ((rmace_ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) != 0) {
		// save configuration
		p = icrypto->dma_data->control_block + 1 /* header */ + req_ctx->key_size_8;
		d = (u64*)icrypto->cur_req->iv;
		for (i = 0; i < req_ctx->iv_size_8; ++i)
			*(d++) = __le64_to_cpu(*(p++));
	}

	dma_unmap_sg(&icrypto->rmace->pdev->dev, icrypto->cur_req->src, icrypto->cur_req_src_nents, DMA_TO_DEVICE);
	dma_unmap_sg(&icrypto->rmace->pdev->dev, icrypto->cur_req->dst, icrypto->cur_req_dst_nents, DMA_FROM_DEVICE);

	crypto_finalize_skcipher_request(icrypto->engine, icrypto->cur_req,
		(rmace_ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) != 0 ? 0 : -EFAULT);
}


static int rmace_cipher_one_req(
	struct crypto_engine *engine,
	void *areq)
{	
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct rcm_rmace_icrypto *icrypto = crypto_ctx->icrypto;
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	int src_mapped_nents;
	int dst_mapped_nents;
	int ret;

	icrypto->cur_req = req;

	// check max length
	if (req->cryptlen > ALG_MAX_PAGES * PAGE_SIZE)
		return -EINVAL;

	// calculate source/destincation length in nents
	icrypto->cur_req_src_nents = sg_nents_for_len(req->src, req->cryptlen);
	if (icrypto->cur_req_src_nents < 0)
		return icrypto->cur_req_src_nents;
	icrypto->cur_req_dst_nents = sg_nents_for_len(req->dst, req->cryptlen);
	if (icrypto->cur_req_dst_nents < 0)
		return icrypto->cur_req_dst_nents;

	// map source buffers
	ret = dma_map_sg(&icrypto->rmace->pdev->dev, req->src, icrypto->cur_req_src_nents, DMA_TO_DEVICE);
	if (ret < 0)
		return ret;
	src_mapped_nents = ret;

	// map destination buffers
	ret = dma_map_sg(&icrypto->rmace->pdev->dev, req->dst, icrypto->cur_req_dst_nents, DMA_TO_DEVICE);
	if (ret < 0)
		goto unmap_src;
	dst_mapped_nents = ret;

	rmace_ctx->flags = 0;

	// no descriptors yet
	rmace_ctx->src_desc_count = 0;
	rmace_ctx->dst_desc_count = 0;

	config_src_key_iv(icrypto, req, rmace_ctx);
	ret = config_src_data(req->src, src_mapped_nents, rmace_ctx);
	if (ret != 0)
		goto unmap_dst;
	ret = config_dst_data(req->dst, dst_mapped_nents, rmace_ctx);
	if (ret != 0)
		goto unmap_dst;
	config_dst_key_iv(icrypto, req, rmace_ctx);

	rcm_rmace_ctx_schelude(rmace_ctx);

	return 0;

unmap_dst:
	dma_unmap_sg(&icrypto->rmace->pdev->dev, req->dst, icrypto->cur_req_dst_nents, DMA_FROM_DEVICE);

unmap_src:
	dma_unmap_sg(&icrypto->rmace->pdev->dev, req->src, icrypto->cur_req_src_nents, DMA_TO_DEVICE);

	return ret;
}

static int rmace_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	struct rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(tfm);

	crypto_ctx->key_size_8 = key_size / 8;
	memcpy(crypto_ctx->key, key, key_size);

	return 0;
}

static int rmace_crypt(struct skcipher_request *req, bool decrypt)
{
	struct rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(
		crypto_skcipher_reqtfm(req));
	struct rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);

	req_ctx->alg_mode = crypto_ctx->alg_mode;
	if (decrypt)
		req_ctx->alg_mode |= ALG_MODE_DECRYPT_MASK;
	req_ctx->key_size_8 = crypto_ctx->key_size_8;
	req_ctx->iv_size_8 = crypto_ctx->iv_size_8;
	memcpy(req_ctx->key, crypto_ctx->key, crypto_ctx->key_size_8 * 8);

	return crypto_transfer_skcipher_request_to_engine(
		crypto_ctx->icrypto->engine, req);
}

static int rmace_crypto_ctx_init(
	struct crypto_skcipher *tfm,
	unsigned alg_mode,
	unsigned default_key_size,
	unsigned default_iv_size)
{
	struct rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(tfm);

	if (rcm_rmace_dev_single_ptr == NULL)
		return -ENODEV;

	crypto_ctx->icrypto = rcm_rmace_dev_single_ptr->icrypto;
	crypto_ctx->alg_mode = alg_mode;
	crypto_ctx->key_size_8 = default_key_size / 8;
	crypto_ctx->iv_size_8 = default_iv_size / 8;

	crypto_skcipher_set_reqsize(tfm, sizeof(struct rmace_crypto_reqctx));

	crypto_ctx->enginectx.op.prepare_request = NULL;
	crypto_ctx->enginectx.op.unprepare_request = NULL;
	crypto_ctx->enginectx.op.do_one_request = rmace_cipher_one_req;

	return 0;
}

static int rmace_encrypt(struct skcipher_request *req)
{
	return rmace_crypt(req, false);
}

static int rmace_decrypt(struct skcipher_request *req)
{
	return rmace_crypt(req, true);
}

static int rmace_des_ecb_ctx_init(struct crypto_skcipher *tfm)
{
 	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES_MASK | ALG_MODE_ECB_MASK,
		DES_KEY_SIZE,
		0);
}

static int rmace_des_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES_MASK | ALG_MODE_CBC_MASK,
		DES_KEY_SIZE,
		DES_BLOCK_SIZE);
}

static int rmace_des_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES_MASK | ALG_MODE_OFB_MASK,
		DES_KEY_SIZE,
		DES_BLOCK_SIZE);
}

static int rmace_des3_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES3_MASK | ALG_MODE_ECB_MASK,
		DES_KEY_SIZE,
		0);
}

static int rmace_des3_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES3_MASK | ALG_MODE_CBC_MASK,
		DES_KEY_SIZE,
		DES_BLOCK_SIZE);
}

static int rmace_des3_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_DES3_MASK | ALG_MODE_OFB_MASK,
		DES_KEY_SIZE,
		DES_BLOCK_SIZE);
}

static int rmace_aes_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_AES_MASK | ALG_MODE_ECB_MASK,
		AES_KEYSIZE_128,
		0);
}

static int rmace_aes_cbc_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_AES_MASK | ALG_MODE_CBC_MASK,
		AES_KEYSIZE_128,
		AES_BLOCK_SIZE);
}

static int rmace_aes_ofb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_AES_MASK | ALG_MODE_OFB_MASK,
		AES_KEYSIZE_128,
		AES_BLOCK_SIZE);
}

static int rmace_des_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if (key_size != DES_KEY_SIZE)
		return -EINVAL;

	return rmace_setkey(tfm, key, key_size);
}

static int rmace_des3_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if (key_size != DES_KEY_SIZE * 3)
		return -EINVAL;

	return rmace_setkey(tfm, key, key_size);
}

static int rmace_aes_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if ((key_size != AES_KEYSIZE_128)
		&& (key_size != AES_KEYSIZE_192)
		&& (key_size != AES_KEYSIZE_256))
		return -EINVAL;

	return rmace_setkey(tfm, key, key_size);
}

static struct skcipher_alg crypto_algs[] = {
	{
		.base.cra_name = "ecb(des)",
		.base.cra_driver_name = "rmace-ecb-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des_ecb_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = 0,
		.setkey = rmace_des_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "cbc(des)",
		.base.cra_driver_name = "rmace-cbc-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des_cbc_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = DES_BLOCK_SIZE,
		.setkey = rmace_des_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "ofb(des)",
		.base.cra_driver_name = "rmace-ofb-des",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des_ofb_ctx_init,
		.min_keysize = DES_KEY_SIZE,
		.max_keysize = DES_KEY_SIZE,
		.ivsize = DES_BLOCK_SIZE,
		.setkey = rmace_des_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "ecb(des3_ede)",
		.base.cra_driver_name = "rmace-ecb-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des3_ecb_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = 0,
		.setkey = rmace_des3_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "cbc(des3_ede)",
		.base.cra_driver_name = "rmace-cbc-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des3_cbc_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = DES_BLOCK_SIZE,
		.setkey = rmace_des3_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "ofb(des3_ede)",
		.base.cra_driver_name = "rmace-ofb-des3",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = DES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_des3_ofb_ctx_init,
		.min_keysize = DES_KEY_SIZE * 3,
		.max_keysize = DES_KEY_SIZE * 3,
		.ivsize = DES_BLOCK_SIZE,
		.setkey = rmace_des3_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "ecb(aes)",
		.base.cra_driver_name = "rmace-ecb-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_aes_ecb_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = 0,
		.setkey = rmace_aes_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "cbc(aes)",
		.base.cra_driver_name = "rmace-cbc-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_aes_cbc_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = rmace_aes_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "ofb(aes)",
		.base.cra_driver_name = "rmace-ofb-aes",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_aes_ofb_ctx_init,
		.min_keysize = AES_KEYSIZE_128,
		.max_keysize = AES_KEYSIZE_256,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = rmace_aes_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
};

int rcm_rmace_crypto_register(struct rcm_rmace_dev *rmace)
{
	struct rcm_rmace_icrypto *icrypto;
	int ret;

	icrypto = devm_kzalloc(&rmace->pdev->dev, sizeof *icrypto, GFP_KERNEL);
	if (icrypto == NULL) {
		pr_err("insufficient memory\n");
		return -ENOMEM;
	}
	icrypto->rmace = rmace;
	rmace->icrypto = icrypto;

	icrypto->dma_data = dmam_alloc_coherent(
		&rmace->pdev->dev,
		sizeof *icrypto->dma_data,
		&icrypto->dma_data_phys_addr,
		GFP_KERNEL);
	if (icrypto->dma_data == NULL) {
		pr_err("cannot allocate DMA buffers");
		return -ENODEV;
	}

	rcm_rmace_ctx_init(rmace, &icrypto->rmace_ctx);
	icrypto->rmace_ctx.src_desc_infos = icrypto->src_desc_infos;
	icrypto->rmace_ctx.dst_desc_infos = icrypto->dst_desc_infos;
	icrypto->rmace_ctx.callback = rmace_callback;

	icrypto->engine = crypto_engine_alloc_init(&rmace->pdev->dev, true);
	if (!icrypto->engine) {
		pr_err("could not init crypto engine\n");
		return -ENOMEM;
	}

	ret = crypto_engine_start(icrypto->engine);
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
	crypto_engine_exit(icrypto->engine);

	return ret;
}
EXPORT_SYMBOL(rcm_rmace_crypto_register);

void rcm_rmace_crypto_unregister(struct rcm_rmace_dev *rmace)
{
	crypto_unregister_skciphers(crypto_algs, ARRAY_SIZE(crypto_algs));
	crypto_engine_exit(rmace->icrypto->engine); 
}
EXPORT_SYMBOL(rcm_rmace_crypto_unregister);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_DESCRIPTION("RCM SoCs RMACE crypto interface");
