// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "rcm-mdma.h"

#ifdef CONFIG_BASIS_PLATFORM
#	include "../misc/rcm/basis/basis-device.h"
#endif

void mdma_desc_pool_free(struct mdma_desc_pool* pool)
{
	int i;

	for (i = 0; i < pool->cnt_chunks; ++i) {
		if (pool->chunks[i].descs) {
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_free_coherent(pool->dev,
			                               MDMA_POOL_CHUNK_SIZE,
			                               pool->chunks[i].descs,
			                               pool->chunks[i].dma_addr,
			                               pool->chunks[i].ep_addr);
#else
			dma_free_coherent(pool->dev, MDMA_POOL_CHUNK_SIZE,
			                  pool->chunks[i].descs,
			                  pool->chunks[i].dma_addr);
#endif
			pool->chunks[i].descs = NULL;
		}
	}

	if (pool->chunks) {
		kfree(pool->chunks);
		pool->chunks = NULL;
	}
}

int mdma_desc_pool_alloc(struct mdma_desc_pool* pool, unsigned cnt,
                         struct device *dev, char* name,
                         size_t max_transaction, size_t len_mask,
                         bool do_size_clip)
{
	int ret = 0;
	unsigned i = 0;
	struct mdma_desc_long_ll link = {0};
	const unsigned cnt_in_chunk = MDMA_POOL_CHUNK_SIZE / 
	                              sizeof(struct mdma_desc_long_ll);

	pool->cnt_chunks = (cnt + cnt_in_chunk - 1) / cnt_in_chunk;

	pool->chunks = kmalloc_array(pool->cnt_chunks,
	                             sizeof(struct mdma_pool_chunk),
	                             GFP_KERNEL);

	if (!pool->chunks) {
		pool->cnt_chunks = 0;
		dev_err(dev, 
		        "%s: Failed to allocate chunks array (%u items).\n",
		        __func__, pool->cnt_chunks);
		ret = -ENOMEM;
		goto err_free_pool;
	}

	memset(pool->chunks, 0,
	       sizeof(struct mdma_pool_chunk) * pool->cnt_chunks);

	pool->size = 0;

	while (pool->size < cnt) {
		pool->chunks[i].descs = 
#ifdef CONFIG_BASIS_PLATFORM
			basis_device_dma_alloc_coherent(
				dev, MDMA_POOL_CHUNK_SIZE,
				&pool->chunks[i].dma_addr,
				&pool->chunks[i].ep_addr,
				GFP_KERNEL);
#else
			dma_alloc_coherent(dev, MDMA_POOL_CHUNK_SIZE,
			                   &pool->chunks[i].dma_addr,
			                   GFP_KERNEL);
#endif

		if (!pool->chunks[i].descs) {
			dev_err(dev, 
			        "%s: Failed to allocate DMA-coherent memory "
			        "(%d bytes).\n",
			        __func__, MDMA_POOL_CHUNK_SIZE);
			ret = -ENOMEM;
			goto err_free_pool;
		}

		memset(pool->chunks[i].descs, 0, MDMA_POOL_CHUNK_SIZE);

		pool->size += cnt_in_chunk;

		++i;
	}

	if (do_size_clip)
		pool->size = cnt;

	pool->cnt = pool->size;
	pool->next = 0;
	pool->head = 0;

	pool->dev = dev;
	pool->max_transaction = max_transaction;
	pool->len_mask = len_mask;
	pool->name = name;

	link.flags_length = MDMA_BD_LINK;

	for (i = 0; i < pool->cnt_chunks; ++i) {
		unsigned num_in_chunk = (i + 1 < pool->cnt_chunks) ?
		                        cnt_in_chunk - 1 :
		                        (pool->size - 1) % cnt_in_chunk;

		if (i + 1 < pool->cnt_chunks)
#ifdef CONFIG_BASIS_PLATFORM
			link.memptr = pool->chunks[i + 1].ep_addr;
#else
			link.memptr = pool->chunks[i + 1].dma_addr;
#endif
		else
#ifdef CONFIG_BASIS_PLATFORM
			link.memptr = pool->chunks[0].ep_addr;
#else
			link.memptr = pool->chunks[0].dma_addr;
#endif

		pool->chunks[i].descs[num_in_chunk] = link;
	}

	dev_dbg(dev, "[%s] descriptor's pool allocated "
	             "(%u descriptors in %u chunks)\n",
	             name, cnt, pool->cnt_chunks);

	return 0;

err_free_pool:
	mdma_desc_pool_free(pool);
	return ret;
}

dma_addr_t mdma_desc_pool_get_addr(struct mdma_desc_pool* pool, unsigned pos)
{
	const unsigned cnt_in_chunk = MDMA_POOL_CHUNK_SIZE / 
	                              sizeof(struct mdma_desc_long_ll);
	unsigned num_chunk = pos / cnt_in_chunk;

#ifdef CONFIG_BASIS_PLATFORM
	return pool->chunks[num_chunk].ep_addr + 
	       (pos % cnt_in_chunk) * sizeof(struct mdma_desc_long_ll);
#else
	return pool->chunks[num_chunk].dma_addr + 
	       (pos % cnt_in_chunk) * sizeof(struct mdma_desc_long_ll);
#endif
}

struct mdma_desc_long_ll* mdma_desc_pool_get_desc(struct mdma_desc_pool* pool,
                                                  unsigned pos)
{
	const unsigned cnt_in_chunk = MDMA_POOL_CHUNK_SIZE / 
	                              sizeof(struct mdma_desc_long_ll);
	unsigned num_chunk = pos / cnt_in_chunk;

	return pool->chunks[num_chunk].descs + (pos % cnt_in_chunk);
}

unsigned mdma_desc_pool_get(struct mdma_desc_pool* pool, 
                            unsigned cnt, unsigned *pos)
{
	const unsigned cnt_in_chunk = MDMA_POOL_CHUNK_SIZE / 
	                              sizeof(struct mdma_desc_long_ll);
	unsigned i;
	unsigned n;

	if (cnt > pool->cnt)
		return 0;

	i = pool->next;
	n = 0;

	while ((n != cnt) && (cnt <= pool->cnt)) {
		if ((i % cnt_in_chunk) == cnt_in_chunk - 1)
			++cnt;
		++n;
		i = (i + 1) % pool->size;
	}

	if (n != cnt)
		return 0;

	i = pool->next;

	if (pos)
		*pos = i;

	pool->cnt -= cnt;
	pool->next = (pool->next + cnt) % pool->size;

	return cnt;
}

void mdma_desc_pool_put(struct mdma_desc_pool* pool,
                        unsigned pos, unsigned cnt)
{
	if (pos != pool->head) {
		dev_warn(pool->dev,
		         "%s: incorrect pool operation: pool(head = %u, "
		         "cnt = %u), trying to put %u items to pos %u.\n",
		         __func__, pool->head, pool->cnt, cnt, pos);
	}

	if (cnt > pool->size - pool->cnt) {
		dev_warn(pool->dev,
		         "%s: incorrect pool operation: "
		         "pool(size = %u, cnt = %u), trying to put %u items.\n",
		         __func__, pool->size, pool->cnt, cnt);
		cnt = pool->size - pool->cnt;
	}

	pool->head = (pool->head + cnt) % pool->size;
	pool->cnt += cnt;
}

unsigned mdma_desc_pool_fill(struct mdma_desc_pool* pool, unsigned pos, 
                             dma_addr_t dma_addr, size_t len,
                             bool stop, bool interrupt)
{
	struct mdma_desc_long_ll *desc;
	size_t seg_len;
	unsigned cnt = 0;

	do {
		desc = mdma_desc_pool_get_desc(pool, pos);

		if (desc->flags_length & MDMA_BD_LINK) {
			desc->flags_length = MDMA_BD_LINK;
		} else {
			seg_len = min_t(size_t, len, pool->max_transaction);
			len -= seg_len;

			desc->flags_length = seg_len;

			if(!len) {
				if (stop) 
					desc->flags_length |= MDMA_BD_STOP;
				if (interrupt) 
					desc->flags_length |= MDMA_BD_INT;
			}

			desc->memptr = dma_addr;
			dma_addr += seg_len;
		}

		pos = (pos + 1) % pool->size;
		++cnt;
	} while (len);

	return cnt;
}

unsigned mdma_desc_pool_fill_like(struct mdma_desc_pool* pool, unsigned pos, 
                                  dma_addr_t dma_addr, size_t len,
                                  bool stop, bool interrupt,
                                  struct mdma_desc_pool* pool_base, 
                                  unsigned pos_base)
{
	struct mdma_desc_long_ll *desc;
	struct mdma_desc_long_ll *desc_base;
	size_t seg_len;
	unsigned cnt = 0;

	do {
		desc = mdma_desc_pool_get_desc(pool, pos);
		desc_base = mdma_desc_pool_get_desc(pool_base, pos_base);

		if (desc_base->flags_length & MDMA_BD_LINK) {
			pos_base = (pos_base + 1) % pool_base->size;
			continue;
		}

		seg_len = (desc_base->flags_length & 
		           pool->len_mask);
		pos_base = (pos_base + 1) % pool_base->size;

		if (desc->flags_length & MDMA_BD_LINK) {
			desc->flags_length = MDMA_BD_LINK;

			pos = (pos + 1) % pool->size;
			++cnt;

			desc = mdma_desc_pool_get_desc(pool, pos);
		}

		if (len < seg_len) {
			dev_err(pool->dev, "%s: Incorrect data passed.\n",
			        __func__);
			break;
		}

		len -= seg_len;

		desc->flags_length = seg_len;

		if(!len) {
			if (stop) 
				desc->flags_length |= MDMA_BD_STOP;
			if (interrupt) 
				desc->flags_length |= MDMA_BD_INT;
		}

		desc->memptr = dma_addr;
		dma_addr += seg_len;

		pos = (pos + 1) % pool->size;
		++cnt;
	} while (len);

	return cnt;
}

unsigned mdma_desc_pool_fill_sg(struct mdma_desc_pool* pool, unsigned pos, 
                                struct scatterlist *sg, unsigned int sg_len,
                                bool stop, bool interrupt)
{
	struct mdma_desc_long_ll *desc;
	size_t seg_len;
	unsigned cnt = 0;
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, sg_len, i) {
		dma_addr_t dma_addr = sg_dma_address(s);
		size_t dma_len  = sg_dma_len(s);

		while (dma_len > 0) {
			desc = mdma_desc_pool_get_desc(pool, pos);

			seg_len = min_t(size_t, dma_len,
			                pool->max_transaction);
			dma_len -= seg_len;

			if (desc->flags_length & MDMA_BD_LINK) {
				desc->flags_length = MDMA_BD_LINK;

				pos = (pos + 1) % pool->size;
				++cnt;

				desc = mdma_desc_pool_get_desc(pool, pos);
			}

			desc->flags_length = seg_len;

			if ((i + 1 == sg_len) && (dma_len == 0)) {
				if (stop)
					desc->flags_length |= MDMA_BD_STOP;
				if (interrupt) 
					desc->flags_length |= MDMA_BD_INT;
			}

			desc->memptr = dma_addr;
			dma_addr += seg_len;

			pos = (pos + 1) % pool->size;
			++cnt;
		}
	}

	return cnt;
}
