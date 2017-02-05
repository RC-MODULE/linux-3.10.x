/*
 * drivers/crypto/mcrypto.c - Module 3DES crypto accelerator driver
 *
 *	Copyright (C) 2010 Module
 *	Written by Sergey Mironov <ierton@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/crypto.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/module_crypto.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/of_platform.h>

#include <crypto/algapi.h>
#include <crypto/des.h>

#define DRIVER_NAME "mcrypto_3des"

/*{{{ Helper macros definition*/
#define ERR(x, args...) \
        do{ printk(KERN_WARNING DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define INFO(x, args...) \
        do{ printk(KERN_INFO DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define DBG(x, args...) \
        do{ if(g_mcrypto_debug) \
                printk(KERN_INFO DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define DBGVAL(val) DBG(#val ": 0x%08X", (unsigned int)val)

#define IOWRITE32(what,where) do{ \
	u32 __what = ( what ); \
        DBG("0x%08X ( " #where " ) < 0x%08X ( " #what " )", \
            (unsigned int)where, (unsigned int)__what); \
        __raw_writel(__what, where); \
        smp_wmb();\
    }while(0)
static inline unsigned long IOREAD32(void __iomem *addr) 
{
    unsigned long c = (__raw_readl(addr));
    smp_rmb();
    return c;
}

#define IOWRITE_RAW(what,where) do{ \
        DBG("0x%08X ( " #where " ) < 0x%08X { " #what " }", \
            (unsigned int)where, (unsigned int)what); \
        __raw_writel(be32_to_cpu(what), where); \
    }while(0)
static inline unsigned long IOREAD_RAW(void __iomem *addr) 
{
        return cpu_to_be32(__raw_readl(addr));
}

/* Following macros make register log to be less ugly */
/* base is a local variable containig base address of a register */
#define REG(x) (base + (x))
#define DMAREG(x) (base + (x))
/*}}}*/

#define DES3_EDE_MIN_BLOCK_SIZE     DES3_EDE_BLOCK_SIZE
#define MCRYPTO_3DES_WAIT_CYCLES    1000000

static int g_mcrypto_debug = 0;
module_param_named(debug, g_mcrypto_debug, int, 0);

typedef int mcrypto_3des_keysign_t;
#define MCRYPTO_NOKEYS_SIGN 0
static atomic_t g_keysign = ATOMIC_INIT(MCRYPTO_NOKEYS_SIGN);

/* Register map{{{*/
struct regmap {
    int conf;
    int key0_l;
    int key0_h;
    int key1_l;
    int key1_h;
    int key2_l;
    int key2_h;
    int iv_l;
    int iv_h;
    int status;
};

static struct regmap regmap[] = { {
    /* Word-based mode (sync) registers */
    .conf =     MCRYPTO_CONF_W,
    .key0_l =  MCRYPTO_KEY_W0_LOW,
    .key0_h =  MCRYPTO_KEY_W0_HIGH,
    .key1_l =  MCRYPTO_KEY_W1_LOW,
    .key1_h =  MCRYPTO_KEY_W1_HIGH,
    .key2_l =  MCRYPTO_KEY_W2_LOW,
    .key2_h =  MCRYPTO_KEY_W2_HIGH,
    .iv_l =     MCRYPTO_IV_W_LOW,
    .iv_h =     MCRYPTO_IV_W_HIGH,
    .status =  MCRYPTO_STATUS_WBW,
} , {
    /* Stream mode (async, DMA) registers */
    .conf =     MCRYPTO_CONF_S,
    .key0_l =  MCRYPTO_KEY_S0_LOW,
    .key0_h =  MCRYPTO_KEY_S0_HIGH,
    .key1_l =  MCRYPTO_KEY_S1_LOW,
    .key1_h =  MCRYPTO_KEY_S1_HIGH,
    .key2_l =  MCRYPTO_KEY_S2_LOW,
    .key2_h =  MCRYPTO_KEY_S2_HIGH,
    .iv_l =     MCRYPTO_IV_S_LOW,
    .iv_h =     MCRYPTO_IV_S_HIGH,
    .status =  MCRYPTO_STATUS,
} };
/*}}}*/

/*{{{ Cipher context */
typedef enum {
    mcrypto_encrypt,
    mcrypto_decrypt,
    mcrypto_nomode
} mcrypto_3des_mode_t;

typedef enum {
    mcrypto_cbc,
    mcrypto_ecb,
    mcrypto_noscheme
} mcrypto_3des_scheme_t;

typedef enum {
    mcrypto_word = 0,
    mcrypto_stream = 1,
    mcrypto_unknown
} mcrypto_3des_block_t;

struct mcrypto_3des_ctx
{
    u32 keys[6];
    mcrypto_3des_keysign_t keysign;
    mcrypto_3des_mode_t mode;
    mcrypto_3des_scheme_t scheme;
    u8 *iv;
};

static void mcrypto_3des_ctx_loadkeys(struct mcrypto_3des_ctx *ctx, const u8* keys)
{
	int i;
	const u32* K = (const u32*) keys;
	for(i=0; i<6; ++i)
		ctx->keys[i] = (K[i]);
	ctx->keysign = atomic_inc_return(&g_keysign);
	/* we don't whant invalid value.*/
	if(ctx->keysign == MCRYPTO_NOKEYS_SIGN)
		ctx->keysign = atomic_inc_return(&g_keysign);
}
/*}}}*/

/*{{{ Device structure*/
typedef enum {
	MCRYPTO_STATE_IDLE = 0<<0,
	MCRYPTO_STATE_BUSY = 1<<0,
	MCRYPTO_STATE_IN_DMAREADY = 1<<1,
	MCRYPTO_STATE_OUT_DMAREADY = 1<<2
} mcrypto_state_t;

#define MCRYPTO_STATE_ALL_DMAREADY \
	(MCRYPTO_STATE_BUSY | MCRYPTO_STATE_OUT_DMAREADY | MCRYPTO_STATE_IN_DMAREADY)

struct mcrypto_device {
	/* public: */
	struct device* dev;
	struct resource *res_mem;
	unsigned int irq;
	void __iomem *io;

	struct __block {
		mcrypto_3des_mode_t mode;
		mcrypto_3des_scheme_t scheme;
		mcrypto_3des_keysign_t keysign;

		/* Device lock. Needed for all the _io_ functions in given mode */
		spinlock_t lock;
	} block[2];

	wait_queue_head_t dma_queue;
	mcrypto_state_t dma_state;

	/* debugfs */
	struct dentry *debugfs_dir;
	int cnt_irq[MCRYPTO_NIRQ];
	int last_irq_status;
	int cnt_idle_err;
	int cnt_ready_err;
};

static struct mcrypto_device g_mcrypto_device;

static inline spinlock_t * stream_lock(struct mcrypto_device* d)
{
	return &d->block[mcrypto_stream].lock;
}

static inline spinlock_t * word_lock(struct mcrypto_device* d)
{
	return &d->block[mcrypto_word].lock;
}
/*}}}*/

/* init/uninit{{{*/
static void mcrypto_3des_init(struct mcrypto_device* device)
{
	int i;

	for(i=0;i<2;i++) {
		device->block[i].mode = mcrypto_nomode;
		device->block[i].scheme = mcrypto_noscheme;
		device->block[i].keysign = MCRYPTO_NOKEYS_SIGN;
		spin_lock_init(&device->block[i].lock);
	}
	init_waitqueue_head(&device->dma_queue);
	device->dma_state = MCRYPTO_STATE_IDLE;

	device->debugfs_dir = debugfs_create_dir("mcrypto", NULL);
	debugfs_create_u32("dma_state", 0400,
		device->debugfs_dir, &device->dma_state);
	debugfs_create_x32("status", 0400,
		device->debugfs_dir, &device->last_irq_status);
	debugfs_create_u32("cnt_idle_err", 0600,
		device->debugfs_dir, &device->cnt_idle_err);
	debugfs_create_u32("cnt_ready_err", 0600,
		device->debugfs_dir, &device->cnt_ready_err);
	debugfs_create_u32("cnt_irq_oaxi", 0600,
		device->debugfs_dir, &device->cnt_irq[MCRYPTO_STATUS_OUTAXI]);
	debugfs_create_u32("cnt_irq_iaxi", 0600,
		device->debugfs_dir, &device->cnt_irq[MCRYPTO_STATUS_INAXI]);
	debugfs_create_u32("cnt_irq_obufend", 0600,
		device->debugfs_dir, &device->cnt_irq[MCRYPTO_STATUS_OUTDMA_BUFEND]);
	debugfs_create_u32("cnt_irq_ibufend", 0600,
		device->debugfs_dir, &device->cnt_irq[MCRYPTO_STATUS_INDMA_BUFEND]);
}

static void mcrypto_3des_uninit(struct mcrypto_device* device)
{
	debugfs_remove_recursive(device->debugfs_dir);
}

static int mcrypto_3des_io_testreg(void *base)
{
	int ret = 0;
	char data[4] = {0,0,0,1};
	IOWRITE32(1, REG(MCRYPTO_MASK));
	if (*((u32*)data) != IOREAD_RAW(REG(MCRYPTO_MASK))) 
		ret = -ENODEV;
	IOWRITE_RAW(0, base+MCRYPTO_MASK);
	return ret;
}
/*}}}*/

/*mcrypto_sw_reset{{{*/
static void mcrypto_io_reset(void __iomem *base)
{
	register int busy=1, protect = 1000;

	IOWRITE32(
		MCRYPTO_DataChEnableCR_EnableCh_1(0) |
		MCRYPTO_DataChEnableCR_ReqBufEnable(0),
		REG(MCRYPTO_DataChEnableCR));

	do {
		busy = IOREAD32(REG(MCRYPTO_AxiInStatus)) &
			MCRYPTO_AxiInStatus_ActiveTrans;
		cpu_relax();
	} while( 0!=(--protect) && 0!=busy );

	BUG_ON(protect==0);

	IOWRITE32(
		MCRYPTO_DataChEnableCR_SWReset,
		REG(MCRYPTO_DataChEnableCR));
}
/*}}}*/

/* mcrypto_3des_io_setctx{{{*/
/* Apply crytpo context to the device */
static void mcrypto_3des_io_setctx(
	struct mcrypto_device *device,
	mcrypto_3des_block_t blk,
	struct mcrypto_3des_ctx *ctx)
{
	int oldconf, conf;
	void __iomem *base = device->io;

	oldconf = IOREAD32(REG(regmap[blk].conf));
	conf = oldconf;

	BUG_ON(ctx->scheme == mcrypto_noscheme);
	if(device->block[blk].scheme != ctx->scheme) {
		conf &= MCRYPTO_CONF_3DES_SCHEME_MASK;
		switch(ctx->scheme) {
			case mcrypto_ecb:
				conf |= MCRYPTO_CONF_3DES_SCHEME_ECB;
				break;
			case mcrypto_cbc:
				conf |= MCRYPTO_CONF_3DES_SCHEME_CBC;
				/* Force rewriting encrypt bits */
				device->block[blk].mode = mcrypto_nomode;
				break;
			default:
				BUG();
		}
		device->block[blk].scheme = ctx->scheme;
	}

	BUG_ON(ctx->mode == mcrypto_nomode);
	if(device->block[blk].mode != ctx->mode) {
		conf &= MCRYPTO_CONF_3DES_MODE_MASK;
		switch(ctx->mode) {
			case mcrypto_encrypt:
				conf |= MCRYPTO_CONF_3DES_MODE_EDE;
				if(ctx->scheme == mcrypto_cbc)
					conf |= MCRYPTO_CONF_3DES_CBC_ENC;
				break;
			case mcrypto_decrypt:
				conf |= MCRYPTO_CONF_3DES_MODE_DED;
				if(ctx->scheme == mcrypto_cbc)
					conf |= MCRYPTO_CONF_3DES_CBC_DEC;
				break;
			default:
				BUG();
		}
		device->block[blk].mode = ctx->mode;
		device->block[blk].keysign = MCRYPTO_NOKEYS_SIGN;
	}

	if(oldconf != conf) IOWRITE32(conf, REG(regmap[blk].conf));

	BUG_ON(ctx->keysign == MCRYPTO_NOKEYS_SIGN);
	if(device->block[blk].keysign != ctx->keysign) {
		DBG("key_0: %08X%08X", ctx->keys[0], ctx->keys[1]);
		DBG("key_1: %08X%08X", ctx->keys[2], ctx->keys[3]);
		DBG("key_2: %08X%08X", ctx->keys[4], ctx->keys[5]);

		switch(device->block[blk].mode) {
			case mcrypto_encrypt:
				IOWRITE_RAW(ctx->keys[1], REG(regmap[blk].key0_l));
				IOWRITE_RAW(ctx->keys[0], REG(regmap[blk].key0_h));
				IOWRITE_RAW(ctx->keys[3], REG(regmap[blk].key1_l));
				IOWRITE_RAW(ctx->keys[2], REG(regmap[blk].key1_h));
				IOWRITE_RAW(ctx->keys[5], REG(regmap[blk].key2_l));
				IOWRITE_RAW(ctx->keys[4], REG(regmap[blk].key2_h));
				break;
			case mcrypto_decrypt:
				IOWRITE_RAW(ctx->keys[1], REG(regmap[blk].key2_l));
				IOWRITE_RAW(ctx->keys[0], REG(regmap[blk].key2_h));
				IOWRITE_RAW(ctx->keys[3], REG(regmap[blk].key1_l));
				IOWRITE_RAW(ctx->keys[2], REG(regmap[blk].key1_h));
				IOWRITE_RAW(ctx->keys[5], REG(regmap[blk].key0_l));
				IOWRITE_RAW(ctx->keys[4], REG(regmap[blk].key0_h));
				break;
			default:
				BUG();
		}
		device->block[blk].keysign = ctx->keysign;
	}

	if(ctx->iv != NULL) {
		IOWRITE_RAW((*(u32*)(&ctx->iv[4])), REG(regmap[blk].iv_l));
		IOWRITE_RAW((*(u32*)(&ctx->iv[0])), REG(regmap[blk].iv_h));
	}
}
/*}}}*/

/* Word-by-word cipher {{{*/
/* Performs encryption word by word. Device should be locked by the caller */
static void mcrypto_3des_crypt(void __iomem *base, u8 *out8, const u8 *in8, int size)
{
    int i, w;

    const u32* in = (const u32*) in8;
    u32* out = (u32*) out8;

    DBG("entering. size=%d bytes", size);
    size /= sizeof(u32);

    for(w=0; w<size; w+=2) {
        int status=0, oldstatus=0;

        IOWRITE_RAW((in[w+1]), base + MCRYPTO_APB_IN_LOW);
        IOWRITE_RAW((in[w+0]), base + MCRYPTO_APB_IN_HIGH);

        for(i=0; i<MCRYPTO_3DES_WAIT_CYCLES; i++) {
            cpu_relax();
            oldstatus = status;
            status = IOREAD32(REG(regmap[mcrypto_word].status));
            if(status != oldstatus) {
                DBG("Status changed from 0x%08X to: 0x%08X", oldstatus, status);
            }
            if( status )
                break;
        }
        if(i==MCRYPTO_3DES_WAIT_CYCLES) {
            ERR("**ERROR** i==max(%d); Status: 0x%08X", i, status);
        }

        DBG("ready, i=%d", i);
        out[w+1] = (IOREAD_RAW(base + MCRYPTO_APB_OUT_LOW));
        out[w+0] = (IOREAD_RAW(base + MCRYPTO_APB_OUT_HIGH));
        DBG("out: %08X %08X", out[w+0], out[w+1]);
    }
}

static int mcrypto_3des_setkey(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	struct mcrypto_3des_ctx *ctx = crypto_tfm_ctx(tfm);
	const u32 *K = (const u32 *)key;
	u32 *flags = &tfm->crt_flags;

	if(len != DES3_EDE_KEY_SIZE) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	if (unlikely(!((K[0] ^ K[2]) | (K[1] ^ K[3])) ||
			!((K[2] ^ K[4]) | (K[3] ^ K[5])))) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_SCHED;
		return -EINVAL;
	}

	mcrypto_3des_ctx_loadkeys(ctx, key);
	return 0;
}

static void mcrypto_3des_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	unsigned long iflags;
	struct mcrypto_device *device = &g_mcrypto_device;
	struct mcrypto_3des_ctx *ctx = crypto_tfm_ctx(tfm);
	ctx->mode = mcrypto_encrypt;
	ctx->scheme = mcrypto_ecb;
	ctx->iv = NULL;

	spin_lock_irqsave(word_lock(device), iflags);
	mcrypto_3des_io_setctx(device, mcrypto_word, ctx);
	mcrypto_3des_crypt(device->io, out, in, DES3_EDE_BLOCK_SIZE);
	spin_unlock_irqrestore(word_lock(device), iflags);
}

static void mcrypto_3des_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	unsigned long iflags;
	struct mcrypto_device *device = &g_mcrypto_device;
	struct mcrypto_3des_ctx *ctx = crypto_tfm_ctx(tfm);
	ctx->mode = mcrypto_decrypt;
	ctx->scheme = mcrypto_ecb;
	ctx->iv = NULL;

	spin_lock_irqsave(word_lock(device), iflags);
	mcrypto_3des_io_setctx(device, mcrypto_word, ctx);
	mcrypto_3des_crypt(device->io, out, in, DES3_EDE_BLOCK_SIZE);
	spin_unlock_irqrestore(word_lock(device), iflags);
}

static struct crypto_alg mcrypto_3des_alg = {
	.cra_name           =   "des3_ede",
	.cra_driver_name    =   "mcrypto_3des",
	.cra_priority       =   300,
	.cra_alignmask      =   3,
	.cra_flags          =   CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize      =   DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize        =   sizeof(struct mcrypto_3des_ctx),
	.cra_module         =   THIS_MODULE,
	.cra_list           =   LIST_HEAD_INIT(mcrypto_3des_alg.cra_list),
	.cra_u              =   {
	.cipher = {
			.cia_min_keysize    =   DES3_EDE_KEY_SIZE,
			.cia_max_keysize    =   DES3_EDE_KEY_SIZE,
			.cia_setkey         =   mcrypto_3des_setkey,
			.cia_encrypt        =   mcrypto_3des_encrypt,
			.cia_decrypt        =   mcrypto_3des_decrypt
		}
	}
};
/*}}}*/

/* Stream cipher {{{*/
static void mcrypto_3des_io_setdma_addr(
	void __iomem *base, dma_addr_t addr, size_t size, int addr_step)
{
	IOWRITE32(addr,
		DMAREG(MCRYPTO_DMA_DESC_1));

	IOWRITE32(addr + size - 1,
		DMAREG(MCRYPTO_DMA_DESC_2));

	IOWRITE32(
		MCRYPTO_DMA_DESC_3_WR_DMA_DESCR |
		MCRYPTO_DMA_DESC_3_DMA_BUFTYPE_S |
		MCRYPTO_DMA_DESC_3_DMA_SEGSIZE(size) |
		MCRYPTO_DMA_DESC_3_DMA_CHANNEL(0),
		DMAREG(MCRYPTO_DMA_DESC_3));

	IOWRITE32(
		MCRYPTO_DMA_CONF_1_IRQ_LAST_SEGADDR(1) |
		MCRYPTO_DMA_CONF_1_IRQ_LAST_BADDR(1) |
		MCRYPTO_DMA_CONF_1_DATA_SEND_IRQ(1) |
		MCRYPTO_DMA_CONF_1_DMA_ADDR_STEP(addr_step),
		DMAREG(MCRYPTO_DMA_CONF_1));
}

static void mcrypto_3des_io_setdma(struct mcrypto_device *device,
	dma_addr_t addr_in, dma_addr_t addr_out, size_t size)
{
	void __iomem * base = device->io;
	void *dma_base_in = base + MCRYPTO_DMAIN_BASE;
	void *dma_base_out = base + MCRYPTO_DMAOUT_BASE;

	BUG_ON(0 != (size % DES3_EDE_MIN_BLOCK_SIZE));

	mcrypto_3des_io_setdma_addr(dma_base_in, addr_in, size, 128);
	mcrypto_3des_io_setdma_addr(dma_base_out, addr_out, size, 8);

	IOWRITE32(0xF,
		REG(MCRYPTO_AxiAwlen));

	IOWRITE32(0,
		REG(MCRYPTO_AxiParamCR_o));

	IOWRITE32(
		IOREAD32(base + MCRYPTO_DataChConfigCR_o) |
		MCRYPTO_DataChConfigCR_o_BufReqEdge(96) |
		MCRYPTO_DataChConfigCR_o_StopForLastDmaBufferWord_1(1),
		REG(MCRYPTO_DataChConfigCR_o));

#ifdef __LITTLE_ENDIAN
	IOWRITE32(
		MCRYPTO_ENDIAN_LE,
		REG(MCRYPTO_ENDIAN));
#endif

	/* 0xC8 register AXI_ERR_MASK is skipped */
	IOWRITE32(
		MCRYPTO_DataChEnableCR_EnableCh_1(1) |
		MCRYPTO_DataChEnableCR_ReqBufEnable(1),
		REG(MCRYPTO_DataChEnableCR));
}

static inline int dma_ready(struct mcrypto_device* device)
{
	return device->dma_state == MCRYPTO_STATE_ALL_DMAREADY;
}

static inline int dma_idle(struct mcrypto_device* device)
{
	return device->dma_state == MCRYPTO_STATE_IDLE;
}

#if 0
/* Sets up dma and waits for completion using waitqueue. */
static void mcrypto_3des_dmacrypt_irq(
	struct mcrypto_device *device, 
	struct mcrypto_3des_ctx *ctx,
	dma_addr_t src_dma, dma_addr_t dst_dma, 
	size_t size)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(stream_lock(device), flags);
	while(!dma_idle(device)) {
		spin_unlock_irqrestore(stream_lock(device), flags);

		ret = wait_event_interruptible_timeout(
			device->dma_queue, dma_idle(device), HZ);
		if( ret <= 0 ) {
			ERR("wait for idle failed: ret %d", ret);
			device->cnt_idle_err++;
			return;
		}
		spin_lock_irqsave(stream_lock(device), flags);
	}

	device->dma_state = MCRYPTO_STATE_BUSY;
	mcrypto_3des_io_setctx(device, mcrypto_stream, ctx);
	mcrypto_3des_io_setdma(device, src_dma, dst_dma, size);

	while(!dma_ready(device)) {
		spin_unlock_irqrestore(stream_lock(device), flags);
		ret = wait_event_interruptible_timeout(
			device->dma_queue, dma_ready(device), HZ);
		spin_lock_irqsave(stream_lock(device), flags);
		if( ret <= 0 ) {
			ERR("wait for completion failed: ret %d", ret);
			device->cnt_ready_err++;
			break;
		}
	}
	device->dma_state = MCRYPTO_STATE_IDLE;
	spin_unlock_irqrestore(stream_lock(device), flags);
}
#endif

/* Sets up dma and waits for completion using busyloop. */
void mcrypto_3des_dmacrypt_irq_atomic(
	struct mcrypto_device *device, 
	struct mcrypto_3des_ctx *ctx,
	dma_addr_t src_dma, dma_addr_t dst_dma, 
	size_t size)
{
	unsigned long flags;

	spin_lock_irqsave(stream_lock(device), flags);
	while(!dma_idle(device)) {
		spin_unlock_irqrestore(stream_lock(device), flags);

		while(!dma_idle(device)) {
			cpu_relax();
		}

		spin_lock_irqsave(stream_lock(device), flags);
	}

	device->dma_state = MCRYPTO_STATE_BUSY;
	mcrypto_3des_io_setctx(device, mcrypto_stream, ctx);
	mcrypto_3des_io_setdma(device, src_dma, dst_dma, size);

	while(!dma_ready(device)) {
		spin_unlock_irqrestore(stream_lock(device), flags);

		while(!dma_ready(device)) {
			cpu_relax();
		}

		spin_lock_irqsave(stream_lock(device), flags);
	}
	device->dma_state = MCRYPTO_STATE_IDLE;
	spin_unlock_irqrestore(stream_lock(device), flags);
}

static int mcrypto_3des_blkcrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct blkcipher_walk walk;
	struct mcrypto_device *device = &g_mcrypto_device;
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	int ret;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	ret = blkcipher_walk_virt(desc, &walk);
	if(ret != 0) {
		ERR("blkcipher_walk_virt error: ret %d", ret);
		return ret;
	}
	ctx->iv = walk.iv;

	while ((nbytes = walk.nbytes)) {
		u8 *wsrc = walk.src.virt.addr;
		u8 *wdst = walk.dst.virt.addr;
		size_t size = nbytes - (nbytes % DES3_EDE_MIN_BLOCK_SIZE);

		dma_addr_t dst_buf_dma, src_buf_dma;
		u8 *src_buf = dma_alloc_coherent(device->dev, size, &src_buf_dma, GFP_ATOMIC);
		u8 *dst_buf = dma_alloc_coherent(device->dev, size, &dst_buf_dma, GFP_ATOMIC);

		BUG_ON(src_buf==NULL);
		BUG_ON(dst_buf==NULL);

		/* Inefficient but safe. Suitable since it is a testing driver. */
		memcpy(src_buf, wsrc, size);
		memset(dst_buf, 0x33, size);
		mcrypto_3des_dmacrypt_irq_atomic(device, ctx, src_buf_dma, dst_buf_dma, size);
		memcpy(wdst, dst_buf, size);

		dma_free_coherent(device->dev, size, src_buf, src_buf_dma);
		dma_free_coherent(device->dev, size, dst_buf, dst_buf_dma);

		ret = blkcipher_walk_done(desc, &walk, nbytes-size);
	}

	return ret;
}

#if 0
/* This version uses dma_map_page to obtain dma-capable memory*/
static int mcrypto_3des_blkcrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct mcrypto_device *device = &g_mcrypto_device;
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	DBG("entering");
	if(desc->flags & CRYPTO_TFM_REQ_MAY_SLEEP) 
		DBG("Algorithm may sleep");
	else
		DBG("Algorithm may not sleep");

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_phys(desc, &walk);
	ctx->iv = walk.iv;

	while((nbytes = walk.nbytes)) {
		dma_addr_t src_dma, dst_dma;
		size_t size = nbytes - (nbytes % DES3_EDE_MIN_BLOCK_SIZE);

		src_dma = dma_map_page(device->dev, walk.src.phys.page,
			walk.src.phys.offset, size, DMA_TO_DEVICE);
		dst_dma = dma_map_page(device->dev, walk.dst.phys.page,
			walk.dst.phys.offset, size, DMA_FROM_DEVICE);

#ifdef CONFIG_SPARC
		/* SPARC LEON3 bug workaround */
		if(src_dma < phys_base) src_dma += phys_base;
		if(dst_dma < phys_base) dst_dma += phys_base;
#endif

		DBG("in_addr: 0x%08X, out_addr: 0x%08X, size: %d", src_dma, dst_dma, size);

		mcrypto_3des_dmacrypt_irq_atomic(device, ctx, src_dma, dst_dma, size);

		dma_unmap_page(device->dev, dst_dma, size, DMA_FROM_DEVICE);
		dma_unmap_page(device->dev, src_dma, size, DMA_TO_DEVICE);

		err = blkcipher_walk_done(desc, &walk, nbytes-size);
		BUG_ON(err != 0);
	}
	return err;
}
#endif

static int mcrypto_3des_ecb_encrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	ctx->mode = mcrypto_encrypt;
	ctx->scheme = mcrypto_ecb;
	return mcrypto_3des_blkcrypt(desc, dst, src, nbytes);
}

static int mcrypto_3des_ecb_decrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	ctx->mode = mcrypto_decrypt;
	ctx->scheme = mcrypto_ecb;
	return mcrypto_3des_blkcrypt(desc, dst, src, nbytes);
}

static struct crypto_alg mcrypto_3des_ecb_alg = {
	.cra_name       =   "ecb(des3_ede)",
	.cra_driver_name    =   "mcrypto_3des_ecb",
	.cra_priority       =   400,
	.cra_flags          =   CRYPTO_ALG_TYPE_BLKCIPHER | CRYPTO_ALG_ASYNC,
	.cra_blocksize      =   DES3_EDE_MIN_BLOCK_SIZE,
	.cra_ctxsize        =   sizeof(struct mcrypto_3des_ctx),
	.cra_alignmask      =   DES3_EDE_MIN_BLOCK_SIZE-1,
	.cra_type           =   &crypto_blkcipher_type,
	.cra_module         =   THIS_MODULE,
	.cra_list           =   LIST_HEAD_INIT(mcrypto_3des_ecb_alg.cra_list),
	.cra_u              =   {
		.blkcipher  =   {
			.min_keysize    =   DES3_EDE_KEY_SIZE,
			.max_keysize    =   DES3_EDE_KEY_SIZE,
			.setkey         =   mcrypto_3des_setkey,
			.encrypt        =   mcrypto_3des_ecb_encrypt,
			.decrypt        =   mcrypto_3des_ecb_decrypt,
		}
	}
};

static int mcrypto_3des_cbc_encrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	ctx->mode = mcrypto_encrypt;
	ctx->scheme = mcrypto_cbc;
	return mcrypto_3des_blkcrypt(desc, dst, src, nbytes);
}

static int mcrypto_3des_cbc_decrypt(struct blkcipher_desc *desc,
	struct scatterlist *dst, struct scatterlist *src, unsigned int nbytes)
{
	struct mcrypto_3des_ctx *ctx = crypto_blkcipher_ctx(desc->tfm);
	ctx->mode = mcrypto_decrypt;
	ctx->scheme = mcrypto_cbc;
	return mcrypto_3des_blkcrypt(desc, dst, src, nbytes);
}

static struct crypto_alg mcrypto_3des_cbc_alg = {
	.cra_name       =   "cbc(des3_ede)",
	.cra_driver_name    =   "mcrypto_3des_cbc",
	.cra_priority       =   400,
	.cra_flags          =   CRYPTO_ALG_TYPE_BLKCIPHER | CRYPTO_ALG_ASYNC,
	.cra_blocksize      =   DES3_EDE_MIN_BLOCK_SIZE,
	.cra_ctxsize        =   sizeof(struct mcrypto_3des_ctx),
	.cra_alignmask      =   DES3_EDE_MIN_BLOCK_SIZE-1,
	.cra_type           =   &crypto_blkcipher_type,
	.cra_module         =   THIS_MODULE,
	.cra_list           =   LIST_HEAD_INIT(mcrypto_3des_cbc_alg.cra_list),
	.cra_u              =   {
		.blkcipher  = {
			.min_keysize    =   DES3_EDE_KEY_SIZE,
			.max_keysize    =   DES3_EDE_KEY_SIZE,
			.ivsize         =   DES_BLOCK_SIZE,
			.setkey         =   mcrypto_3des_setkey,
			.encrypt        =   mcrypto_3des_cbc_encrypt,
			.decrypt        =   mcrypto_3des_cbc_decrypt
		}
	}
};
/*}}}*/

/*IRQ {{{*/
static irqreturn_t mcrypto_3des_irqfn(int irq, void *data)
{
	int i;
	unsigned long flags, status;
	struct mcrypto_device *device = (struct mcrypto_device *)data;
	void* base = device->io;

	spin_lock_irqsave(stream_lock(device), flags);
	status = IOREAD32(REG(regmap[mcrypto_stream].status));
	device->last_irq_status = status;

	if(device->dma_state & MCRYPTO_STATE_BUSY) {
		if(status & (1<<MCRYPTO_STATUS_INDMA_BUFEND))
			device->dma_state |= MCRYPTO_STATE_IN_DMAREADY;
		if(status & (1<<MCRYPTO_STATUS_OUTDMA_BUFEND))
			device->dma_state |= MCRYPTO_STATE_OUT_DMAREADY;
	}

	for(i=0; i<MCRYPTO_NIRQ; i++) {
		if(status & (1<<i))
			device->cnt_irq[i]++;
	}

	spin_unlock_irqrestore(stream_lock(device), flags);

	wake_up(&device->dma_queue);

	return IRQ_HANDLED;
}
/*}}}*/

/* Probe/Remove {{{*/
static int of_mcrypto_probe(struct platform_device *pdev)
{
	int ret;
	struct mcrypto_device *device;

	device = &g_mcrypto_device;
	memset(device, 0, sizeof(struct mcrypto_device));

	device->dev = &pdev->dev;
	platform_set_drvdata(pdev, device);

    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
    pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	printk("ENTER_TO_\n");	
	device->res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!device->res_mem) {
		dev_err(device->dev, "failed to find device registers\n");
		return -EINVAL;
	}

	device->irq = platform_get_irq(pdev, 0);
	if (!device->irq) {
		dev_err(device->dev, "failed to find device irq\n");
		return -EINVAL;
	}

	device->io = devm_ioremap_resource(&pdev->dev, device->res_mem);
	if (!device->io) {
		dev_err(device->dev, "cant remap io memory\n");
		return -ENOMEM;
	}

	mcrypto_io_reset(device->io);

	mcrypto_3des_init(device);

	ret = devm_request_irq(&pdev->dev, device->irq,
		mcrypto_3des_irqfn, 0, pdev->name, device);
	if (ret) {
		dev_err(device->dev, "failed to install interrupt handler: ret %d\n", ret);
		return -ENOMEM;
	}

	ret = mcrypto_3des_io_testreg(device->io);
	if(ret) {
		dev_err(device->dev, "mcrypto_3des_testreg() failed. Indian problem?\n");
		goto error_free_irq;
	}

	IOWRITE32(-1, device->io + MCRYPTO_MASK);

	ret = crypto_register_alg(&mcrypto_3des_alg);
	if(ret) {
		dev_err(device->dev,
			"crypto_register_alg(&mcrypto_3des_alg) failed: ret %d\n", ret);
		goto error_free_irq;
	}

	ret = crypto_register_alg(&mcrypto_3des_ecb_alg);
	if(ret) {
		dev_err(device->dev,
			"crypto_register_alg(&mcrypto_3des_ecb_alg) failed: ret %d\n", ret);
		goto error_unregister_3des;
	}

	ret = crypto_register_alg(&mcrypto_3des_cbc_alg);
	if(ret) {
		dev_err(device->dev,
			"crypto_register_alg(&mcrypto_3des_cbc_alg) failed: ret %d\n", ret);
		goto error_unregister_3des_ecb;
	}
	printk("OUT_OF_\n");
out:
	DBG("leaving");
	return ret;

error_unregister_3des_ecb:
	crypto_unregister_alg(&mcrypto_3des_ecb_alg);
error_unregister_3des:
	crypto_unregister_alg(&mcrypto_3des_alg);
error_free_irq:
	IOWRITE32(0, device->io + MCRYPTO_MASK);
	mcrypto_3des_uninit(device);

	goto out;
}

static int of_mcrypto_remove(struct platform_device *pdev)
{
	struct mcrypto_device *device = platform_get_drvdata(pdev);
	if(!device)
		goto out;

	crypto_unregister_alg(&mcrypto_3des_cbc_alg);
	crypto_unregister_alg(&mcrypto_3des_ecb_alg);
	crypto_unregister_alg(&mcrypto_3des_alg);

	IOWRITE32(0, device->io+ MCRYPTO_MASK);

	mcrypto_3des_uninit(device);
out:
	DBG("leaving");
	return 0;
}
/*}}}*/

static struct of_device_id of_platform_mcrypto_table[] = {
	{ .compatible = "rcm,mcrypto" },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, of_platform_mcrypto_table);

static struct platform_driver of_platform_mcrypto_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_platform_mcrypto_table,
	},
	.probe = of_mcrypto_probe,
	.remove = of_mcrypto_remove
};
module_platform_driver(of_platform_mcrypto_driver);

MODULE_PARM_DESC(debug, "Enables lots of debug messages");

MODULE_AUTHOR("Sergey Mironov <ierton@gmail.com>");
MODULE_DESCRIPTION("ModuleCrypto device driver");
MODULE_LICENSE("GPL");
/*}}}*/

