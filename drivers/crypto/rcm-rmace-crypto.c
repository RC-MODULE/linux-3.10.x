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

#define CRYPTO_DESC_COUNT (RCM_RMACE_HW_DESC_COUNT - 1)

#define GOST89_BLOCK_SIZE 8
#define GOST89_KEY_SIZE 32

#define MAX_KEY_SIZE_8 4 // in 8-byte words
#define MAX_IV_SIZE_8 2 // in 8-byte words
#define MAX_CUSTOM_SIZE_8 8 // in 8-byte words
#define AES_KEY_PREFIX_SIZE_8 2 // in 8-byte words
#define DUMMY_DATA_BLOCK_SIZE_8 2 // in 8-byte words

#define ALG_MODE_DECRYPT_MASK BIT(0)
#define ALG_MODE_DES_MASK BIT(1)
#define ALG_MODE_DES3_MASK BIT(2)
#define ALG_MODE_AES_MASK BIT(3)
#define ALG_MODE_GOST_MASK BIT(4)
#define ALG_MODE_ECB_MASK BIT(5)
#define ALG_MODE_CBC_MASK BIT(6)
#define ALG_MODE_OFB_MASK BIT(7)
#define ALG_MODE_GAMMA_MASK BIT(8)
#define ALG_MODE_GAMMA_FB_MASK BIT(9)

struct dma_data
{
	u64 control_block[1 /* header */ + MAX_KEY_SIZE_8 + MAX_IV_SIZE_8 + MAX_CUSTOM_SIZE_8]; // LE
	u8 dst_bounce_buf[ALG_MAX_PAGES * PAGE_SIZE];

	// for AES RCM RMACE key optimization issue workaround
	u64 dummy_control_block[1 /* header */ + MAX_KEY_SIZE_8 + MAX_IV_SIZE_8]; // LE
	u64 dummy_data_block_src[DUMMY_DATA_BLOCK_SIZE_8];
	u64 dummy_data_block_dst[DUMMY_DATA_BLOCK_SIZE_8];
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
	bool use_dst_bounce_buf;
	struct rcm_rmace_ctx rmace_ctx;
	struct rcm_rmace_hw_desc src_descs[CRYPTO_DESC_COUNT];
	struct rcm_rmace_hw_desc dst_descs[CRYPTO_DESC_COUNT];

	// for AES RCM RMACE key optimization issue workaround
	u64 last_aes_key_perfix[AES_KEY_PREFIX_SIZE_8];
};

struct rmace_crypto_ctx
{
	struct crypto_engine_ctx enginectx;
	struct rcm_rmace_icrypto *icrypto;
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[MAX_KEY_SIZE_8];
};

struct rmace_crypto_reqctx
{
	unsigned alg_mode; // ALG_MODE*
	unsigned key_size_8; // (in 8-byte words) always defined
	unsigned iv_size_8; // (in 8-byte words) can be 0
	u64 key[MAX_KEY_SIZE_8];
};

static const u64 GOST98_PERMUTATION[8] = {
	// Gost28147_TestParamSet
	// 0xc8b6e3294a750df1ULL,
	// 0xc2867ea095f314bdULL,
	// 0xefc95863d1270ab4ULL,
	// 0x2b30e9a48df517c6ULL,
	// 0x352bc64ef9801ad7ULL,
	// 0xb9067cfe243ad185ULL,
	// 0x95701832afd6c4beULL,
	// 0x35f7c1b6e08d29a4ULL

	// Gost28147_TC26ParamSetZ
	0x2BC96AF43850DE71ULL,
	0x73AD0B4FC19652E8ULL,
	0x0E34187BAC296FD5ULL,
	0xC24BE390D618A5F7ULL,
	0xB9E35A076F4D128CULL,
	0x069C471EDAF2853BULL,
	0xF0DB74E1C5A93286ULL,
	0x1F307D8E9B5A264CULL
};

static void aes_rmace_issue_workaround(struct rcm_rmace_icrypto *icrypto, struct rmace_crypto_reqctx *req_ctx)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct rcm_rmace_hw_desc *desc;
	u64 *p;

	p = icrypto->dma_data->dummy_control_block;
	icrypto->last_aes_key_perfix[0] = req_ctx->key[0] + 1; // change the key
	icrypto->last_aes_key_perfix[1] = req_ctx->key[1];
	*(p++) = cpu_to_le64(6ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // EAS-128 ECB
	*(p++) = cpu_to_le64(icrypto->last_aes_key_perfix[0]);
	*(p++) = cpu_to_le64(icrypto->last_aes_key_perfix[1]);
	p += 2; // IV does't matter

	desc = &rmace_ctx->src_descs[rmace_ctx->src_desc_count++];
	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, dummy_control_block));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_VALID_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| (((p - icrypto->dma_data->dummy_control_block) * 8) << RCM_RMACE_DESC_LEN_SHIFT));

	desc = &rmace_ctx->src_descs[rmace_ctx->src_desc_count++];
	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, dummy_data_block_src));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_ACT0_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| ((DUMMY_DATA_BLOCK_SIZE_8 * 8) << RCM_RMACE_DESC_LEN_SHIFT));

	desc = &rmace_ctx->dst_descs[rmace_ctx->dst_desc_count++];
	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, dummy_data_block_dst));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_ACT0_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| ((DUMMY_DATA_BLOCK_SIZE_8 * 8) << RCM_RMACE_DESC_LEN_SHIFT));
}

static int config_src_key_iv(struct rcm_rmace_icrypto *icrypto, struct rmace_crypto_reqctx *req_ctx)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct rcm_rmace_hw_desc *desc;
	u64 *p = icrypto->dma_data->control_block;
	u64 *sk, *sv;
	u64 data = 0; 
	unsigned i;

	if (req_ctx->alg_mode & ALG_MODE_DES_MASK)
		data |= (4ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // des
	else if (req_ctx->alg_mode & ALG_MODE_DES3_MASK) {
		data |= (5ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // des3
		data |= (5ULL << RCM_RMACE_HEADER_FEATURES_SHIFT); // ede
	} else if (req_ctx->alg_mode & ALG_MODE_AES_MASK) {
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
			panic("unexpecting key size");
		}
	} else if (req_ctx->alg_mode & ALG_MODE_GOST_MASK) {
		data |= (9ULL << RCM_RMACE_HEADER_TYPE_SHIFT); // gost
		data |= (0x80ULL << RCM_RMACE_HEADER_FEATURES_SHIFT); // ch_tbl
		data |= (3ULL << RCM_RMACE_HEADER_ENDIAN_SHIFT); // change endianness for input/output
	}

	if (req_ctx->alg_mode & ALG_MODE_CBC_MASK)
		data |= (1ULL << RCM_RMACE_HEADER_MODE_SHIFT); // cbc
	else if (req_ctx->alg_mode & ALG_MODE_OFB_MASK)
		data |= (3ULL << RCM_RMACE_HEADER_MODE_SHIFT); // ofb
	else if (req_ctx->alg_mode & ALG_MODE_GAMMA_MASK)
		data |= (1ULL << RCM_RMACE_HEADER_MODE_SHIFT); // gamma
	else if (req_ctx->alg_mode & ALG_MODE_GAMMA_FB_MASK)
		data |= (2ULL << RCM_RMACE_HEADER_MODE_SHIFT); // gamma-fb

	if (req_ctx->alg_mode & ALG_MODE_DECRYPT_MASK)
		data |= (1ULL << RCM_RMACE_HEADER_ENC_DEC_SHIFT); // decrypt

	*(p++) = cpu_to_le64(data);

	sk = req_ctx->key;
	sv = (u64*)icrypto->cur_req->iv;
	if (req_ctx->alg_mode & ALG_MODE_GOST_MASK) {
		*(p++) = sk[3];
		*(p++) = sk[2];
		*(p++) = sk[1];
		*(p++) = sk[0];
		if (req_ctx->iv_size_8 == 0)
			*(p++) = 0; // fake IV
		else
			*(p++) = *sv; // real IV
		for (i = 0; i < ARRAY_SIZE(GOST98_PERMUTATION); ++i)
			*(p++) = cpu_to_le64(GOST98_PERMUTATION[i]);
	} else {
		for (i = 0; i < req_ctx->key_size_8; ++i)
			*(p++) = swab64(*(sk++));
		for (i = 0; i < req_ctx->iv_size_8; ++i)
			*(p++) = swab64(*(sv++));
	}

	if (rmace_ctx->src_desc_count == CRYPTO_DESC_COUNT)
		return -EINVAL;
	desc = &rmace_ctx->src_descs[rmace_ctx->src_desc_count++];
	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, control_block));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_VALID_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| (((p - icrypto->dma_data->control_block) * 8) << RCM_RMACE_DESC_LEN_SHIFT));

	return 0;
}

static int config_dst_key_iv(struct rcm_rmace_icrypto *icrypto, struct rmace_crypto_reqctx *req_ctx)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct rcm_rmace_hw_desc *desc;

	if (rmace_ctx->dst_desc_count == CRYPTO_DESC_COUNT)
		return -EINVAL;
	desc = &rmace_ctx->dst_descs[rmace_ctx->dst_desc_count++];

	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, control_block));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_VALID_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| (((1 + req_ctx->key_size_8 + req_ctx->iv_size_8) * 8) << RCM_RMACE_DESC_LEN_SHIFT));

	return 0;
}

static int config_src_data(struct rcm_rmace_icrypto *icrypto, struct scatterlist *sglist, int nents)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct scatterlist *sg;
	struct rcm_rmace_hw_desc *desc;
	dma_addr_t addr;
	unsigned len;
	int i;

	for_each_sg(sglist, sg, nents, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		// check for alignment
		if (((addr & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0)
			|| ((len & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0))
			return -EINVAL;
		if (rmace_ctx->src_desc_count == CRYPTO_DESC_COUNT)
			return -EINVAL;
		desc = &rmace_ctx->src_descs[rmace_ctx->src_desc_count++];
		desc->address = cpu_to_le32(addr);
		desc->data = cpu_to_le32(
			(i + 1 == nents ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (len << RCM_RMACE_DESC_LEN_SHIFT));
	}

	return 0;
}

static int config_dst_data(struct rcm_rmace_icrypto *icrypto, struct scatterlist *sglist, int nents)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct scatterlist *sg;
	struct rcm_rmace_hw_desc *desc;
	dma_addr_t addr;
	unsigned len;
	int i;

	for_each_sg(sglist, sg, nents, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		// check for alignment
		if (((addr & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0)
			|| ((len & RCM_RMACE_HARDWARE_ALIGN_MASK) != 0))
			return -EINVAL;
		if (rmace_ctx->dst_desc_count == CRYPTO_DESC_COUNT)
			return -EINVAL;
		desc = &rmace_ctx->dst_descs[rmace_ctx->dst_desc_count++];
		desc->address = cpu_to_le32(addr);
		desc->data = cpu_to_le32(
			(i + 1 == nents ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (len << RCM_RMACE_DESC_LEN_SHIFT));
	}

	return 0;
}

static int config_dst_bounce_buffer(struct rcm_rmace_icrypto *icrypto)
{
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	struct rcm_rmace_hw_desc *desc;

	sg_pcopy_to_buffer(
		icrypto->cur_req->dst,
		icrypto->cur_req_dst_nents,
		icrypto->dma_data->dst_bounce_buf,
		icrypto->cur_req->cryptlen,
		0);

	if (rmace_ctx->dst_desc_count == CRYPTO_DESC_COUNT)
		return -EINVAL;
	desc = &rmace_ctx->dst_descs[rmace_ctx->dst_desc_count++];
	desc->address = cpu_to_le32(icrypto->dma_data_phys_addr + offsetof(struct dma_data, dst_bounce_buf));
	desc->data = cpu_to_le32(
		RCM_RMACE_DESC_ACT0_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| (icrypto->cur_req->cryptlen << RCM_RMACE_DESC_LEN_SHIFT));

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
		if ((req_ctx->alg_mode & ALG_MODE_GOST_MASK) && (req_ctx->iv_size_8 != 0))
			*d = *p;
		else
			for (i = 0; i < req_ctx->iv_size_8; ++i)
				*(d++) = swab64(*(p++));
	}

	dma_unmap_sg(&icrypto->rmace->pdev->dev, icrypto->cur_req->src, icrypto->cur_req_src_nents, DMA_TO_DEVICE);

	if (icrypto->use_dst_bounce_buf)
		sg_pcopy_from_buffer(
			icrypto->cur_req->dst,
			icrypto->cur_req_dst_nents,
			icrypto->dma_data->dst_bounce_buf,
			icrypto->cur_req->cryptlen,
			0);
	else
		dma_unmap_sg(&icrypto->rmace->pdev->dev, icrypto->cur_req->dst, icrypto->cur_req_dst_nents, DMA_FROM_DEVICE);

	crypto_finalize_skcipher_request(icrypto->engine, icrypto->cur_req,
		(rmace_ctx->status & RCM_RMACE_CTX_STATUS_SUCCESS) != 0 ? 0 : -EFAULT);
}

static int rmace_cipher_one_req(
	struct crypto_engine *engine,
	void *areq)
{	
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct rmace_crypto_reqctx *req_ctx = skcipher_request_ctx(req);
	struct rmace_crypto_ctx *crypto_ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct rcm_rmace_icrypto *icrypto = crypto_ctx->icrypto;
	struct rcm_rmace_ctx *rmace_ctx = &icrypto->rmace_ctx;
	unsigned save_dst_desc_count;
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

	icrypto->use_dst_bounce_buf = false;

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

	// no descriptors yet
	rmace_ctx->src_desc_count = 0;
	rmace_ctx->dst_desc_count = 0;

	// AES RCM RMACE key otimization issue workaround
	if (req_ctx->alg_mode & ALG_MODE_AES_MASK) {
		if ((req_ctx->key[0] == icrypto->last_aes_key_perfix[0])
			&& (req_ctx->key[1] == icrypto->last_aes_key_perfix[1]))
		{
			aes_rmace_issue_workaround(icrypto, req_ctx);
		}
		icrypto->last_aes_key_perfix[0] = req_ctx->key[0];
		icrypto->last_aes_key_perfix[1] = req_ctx->key[1];
	}

	ret = config_src_key_iv(icrypto, req_ctx);
	if (ret != 0)
		goto unmap_dst;
	ret = config_src_data(icrypto, req->src, src_mapped_nents);
	if (ret != 0)
		goto unmap_dst;
	save_dst_desc_count = rmace_ctx->dst_desc_count;
	ret = config_dst_data(icrypto, req->dst, dst_mapped_nents);
	if (ret != 0) {
		// destination bounce buffer workaround
		dma_unmap_sg(&icrypto->rmace->pdev->dev, req->dst, icrypto->cur_req_dst_nents, DMA_FROM_DEVICE);
		icrypto->use_dst_bounce_buf = true;
		rmace_ctx->dst_desc_count = save_dst_desc_count;
		ret = config_dst_bounce_buffer(icrypto);
		if (ret != 0)
			goto unmap_dst;
	}
	ret = config_dst_key_iv(icrypto, req_ctx);
	if (ret != 0)
		goto unmap_dst;

	rcm_rmace_ctx_schedule(rmace_ctx);

	return 0;

unmap_dst:
	if (!icrypto->use_dst_bounce_buf)
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

static int rmace_gost89_ecb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_GOST_MASK | ALG_MODE_ECB_MASK,
		GOST89_KEY_SIZE,
		0);
}

static int rmace_gost89_gamma_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_GOST_MASK | ALG_MODE_GAMMA_MASK,
		GOST89_KEY_SIZE,
		GOST89_BLOCK_SIZE);
}

static int rmace_gost89_gamma_fb_ctx_init(struct crypto_skcipher *tfm)
{
	return rmace_crypto_ctx_init(
		tfm,
		ALG_MODE_GOST_MASK | ALG_MODE_GAMMA_FB_MASK,
		GOST89_KEY_SIZE,
		GOST89_BLOCK_SIZE);
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

static int rmace_gost89_setkey(
	struct crypto_skcipher *tfm,
	const u8 *key,
	unsigned key_size)
{
	if (key_size != GOST89_KEY_SIZE)
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
	{
		.base.cra_name = "ecb(gost89)",
		.base.cra_driver_name = "rmace-ecb-gost89",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = GOST89_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_gost89_ecb_ctx_init,
		.min_keysize = GOST89_KEY_SIZE,
		.max_keysize = GOST89_KEY_SIZE,
		.ivsize = 0,
		.setkey = rmace_gost89_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "gamma(gost89)",
		.base.cra_driver_name = "rmace-gamma-gost89",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = GOST89_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_gost89_gamma_ctx_init,
		.min_keysize = GOST89_KEY_SIZE,
		.max_keysize = GOST89_KEY_SIZE,
		.ivsize = GOST89_BLOCK_SIZE,
		.setkey = rmace_gost89_setkey,
		.encrypt = rmace_encrypt,
		.decrypt = rmace_decrypt
	},
	{
		.base.cra_name = "gamma-fb(gost89)",
		.base.cra_driver_name = "rmace-gamma-fb-gost89",
		.base.cra_priority = 200,
		.base.cra_flags = CRYPTO_ALG_ASYNC,
		.base.cra_blocksize = GOST89_BLOCK_SIZE,
		.base.cra_alignmask = RCM_RMACE_HARDWARE_ALIGN_MASK,
		.base.cra_ctxsize = sizeof(struct rmace_crypto_ctx),
		.base.cra_module = THIS_MODULE,
		.init = rmace_gost89_gamma_fb_ctx_init,
		.min_keysize = GOST89_KEY_SIZE,
		.max_keysize = GOST89_KEY_SIZE,
		.ivsize = GOST89_BLOCK_SIZE,
		.setkey = rmace_gost89_setkey,
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
	icrypto->rmace_ctx.src_descs = icrypto->src_descs;
	icrypto->rmace_ctx.dst_descs = icrypto->dst_descs;
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
