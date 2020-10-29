// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#define MDMA_POOL_CHUNK_SIZE    16384
#define MDMA_DESC_ALIGNMENT 16

/**
 * struct mdma_desc_long_ll - Long HW descriptor
 * @usrdata_l: user data depends on descriptor kind
 * @usrdata_h: user data depends on descriptor kind
 * @memptr: Buffer address/Next descriptor address
 * @flags_length: Control word
 */
struct mdma_desc_long_ll {
	unsigned int usrdata_l;
	unsigned int usrdata_h;
	unsigned int memptr;
	unsigned int flags_length;
} __attribute__((packed, aligned(MDMA_DESC_ALIGNMENT)));


struct mdma_pool_chunk {
	struct mdma_desc_long_ll *descs;
	dma_addr_t                dma_addr;
#ifdef CONFIG_BASIS_PLATFORM
	u32                       ep_addr;
#endif
};

struct mdma_desc_pool {
	struct mdma_pool_chunk*   chunks;
	unsigned                  cnt_chunks;

	unsigned                  size;
	unsigned                  cnt;
	unsigned                  head;
	unsigned                  next;

	struct device            *dev;
	size_t                    max_transaction;
	size_t                    len_mask;
	char*                     name;
};

int mdma_desc_pool_alloc(struct mdma_desc_pool* pool, unsigned cnt,
                         struct device *dev, char* name,
                         size_t max_transaction, size_t len_mask,
                         bool do_size_clip);
void mdma_desc_pool_free(struct mdma_desc_pool* pool);
dma_addr_t mdma_desc_pool_get_addr(struct mdma_desc_pool* pool, unsigned pos);
struct mdma_desc_long_ll* mdma_desc_pool_get_desc(struct mdma_desc_pool* pool,
                                                  unsigned pos);
unsigned mdma_desc_pool_get(struct mdma_desc_pool* pool, 
                            unsigned cnt, unsigned *pos);
unsigned mdma_desc_pool_fill(struct mdma_desc_pool* pool, unsigned pos, 
                             dma_addr_t dma_addr, size_t len,
                             bool stop, bool interrupt);
unsigned mdma_desc_pool_fill_like(struct mdma_desc_pool* pool, unsigned pos, 
                                  dma_addr_t dma_addr, size_t len,
                                  bool stop, bool interrupt,
                                  struct mdma_desc_pool* pool_base, 
                                  unsigned pos_base);
unsigned mdma_desc_pool_fill_sg(struct mdma_desc_pool* pool, unsigned pos, 
                                struct scatterlist *sg, unsigned int sg_len,
                                bool stop, bool interrupt);
void mdma_desc_pool_put(struct mdma_desc_pool* pool,
                        unsigned pos, unsigned cnt);
