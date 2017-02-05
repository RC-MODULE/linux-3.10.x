/*
 * include/linux/mcrypto_aes.c
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
#include <linux/platform_device.h>
#include <crypto/algapi.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/err.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <linux/module_crypto_aes.h>

#define DRIVER_NAME	"mcrypto_aes"
#define MCRYPTO_AES_WAIT_CYCLES   1000000

#define MCRYPTO_AES_KEYSIZE AES_KEYSIZE_128

/*{{{ Helper macros definition*/
/* Debug output */
#define ERR(x, args...) \
        do{ printk(KERN_WARNING DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define INFO(x, args...) \
        do{ printk(KERN_INFO DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define DBG(x, args...) \
        do{ if(g_debug) \
                printk(KERN_INFO DRIVER_NAME ":%s(): " x "\n", __FUNCTION__, ## args); } while(0)

#define DBGVAL(val) DBG(#val ": 0x%08X", (unsigned int)val)

/* Port I/O */
#define IOWRITE32(what,where) do{ \
        DBG("0x%08X ( " #where " ) < 0x%08X ( " #what " )", \
            (unsigned int)where, (unsigned int)what); \
        __raw_writel((what), where); \
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

/* Debug print helper */
#define REG(x) (base + (x))

/* Lockers */
#define MCRYPTO_DEVICE_LOCK(dev, flags) \
    spin_lock_irqsave(&dev->lock, flags)

#define MCRYPTO_DEVICE_UNLOCK(dev,flags) \
    spin_unlock_irqrestore(&dev->lock, flags)
/*}}}*/

int g_debug = 0;
module_param_named(debug, g_debug, int, 0);
MODULE_PARM_DESC(debug, "Enables lots of debug messages");

int g_ignore_timeout = 1;
module_param_named(ignore_timeout, g_ignore_timeout, int, 0);
MODULE_PARM_DESC(ignore_timeout, "Ignores crypt timeout (will not trigger BUG())");

/*{{{ Crypto operation context*/
typedef int mcrypto_aes_keysign_t;
#define MCRYPTO_NOKEYS_SIGN 0
static atomic_t g_keysign = ATOMIC_INIT(MCRYPTO_NOKEYS_SIGN);

struct mcrypto_aes_ctx
{
	u32 keys[6];
	mcrypto_aes_keysign_t keysign;
	struct crypto_cipher *fallback;
};

inline int mcrtypto_aes_ctx_keys_loaded(struct mcrypto_aes_ctx *ctx)
{
	return ctx->keysign != MCRYPTO_NOKEYS_SIGN;
}

void mcrypto_aes_ctx_loadkeys(struct mcrypto_aes_ctx *ctx, const u8* keys)
{
	int i;
	const u32* K = (const u32*) keys;
	DBG("entering");
	for(i=0; i<4; ++i)
		ctx->keys[i] = (K[i]);
	ctx->keysign = atomic_inc_return(&g_keysign);
	/* we don't whant invalid value.*/
	if(ctx->keysign == MCRYPTO_NOKEYS_SIGN)
		ctx->keysign = atomic_inc_return(&g_keysign);
}

void mcrypto_aes_ctx_unloadkeys(struct mcrypto_aes_ctx *ctx)
{
	ctx->keysign = MCRYPTO_NOKEYS_SIGN;
}
/*}}}*/

/*{{{ Device structure*/
struct mcrypto_device
{
	struct device* dev;
	struct resource *res_mem;
	unsigned int irq;
	void __iomem *io;

	mcrypto_aes_keysign_t keysign;
	spinlock_t lock;

};

static struct mcrypto_device g_mcrypto_device;

int mcrypto_aes_init(struct mcrypto_device* device)
{
	spin_lock_init(&device->lock);
	device->keysign = MCRYPTO_NOKEYS_SIGN;
	return 0;
}

void mcrypto_aes_uninit(struct mcrypto_device* device)
{
}
/*}}}*/

/*mcrypto_aes_io_setctx{{{*/
void mcrypto_aes_io_setctx(struct mcrypto_device *device, struct mcrypto_aes_ctx *ctx) 
{
	void __iomem *base = device->io;

	BUG_ON(ctx->keysign == MCRYPTO_NOKEYS_SIGN);
	if(device->keysign != ctx->keysign) {
		DBG("key_0: 0x%08X", ctx->keys[0]);
		DBG("key_1: 0x%08X", ctx->keys[1]);
		DBG("key_2: 0x%08X", ctx->keys[2]);
		DBG("key_3: 0x%08X", ctx->keys[3]);

		IOWRITE_RAW(ctx->keys[0], REG(MCRYPTO_AES_KEY_0));
		IOWRITE_RAW(ctx->keys[1], REG(MCRYPTO_AES_KEY_1));
		IOWRITE_RAW(ctx->keys[2], REG(MCRYPTO_AES_KEY_2));
		IOWRITE_RAW(ctx->keys[3], REG(MCRYPTO_AES_KEY_3));
		device->keysign = ctx->keysign;
	}
}
/*}}}*/

/*mcrypto_aes_crypt{{{*/
static void mcrypto_aes_io_crypt(struct mcrypto_device *device, 
	u8 *out8, const u8 *in8, int size)
{
	int i, w, status;

	const u32* in = (const u32*) in8;
	u32* out = (u32*) out8;
	void __iomem *base = device->io;

	DBG("entering. size=%d bytes", size);
	size /= sizeof(u32);

	for(w=0; w<size; w+=4) {
		DBG("in: %08X %08X %08X %08X", in[w+0], in[w+1], in[w+2], in[w+3]);
		IOWRITE_RAW((in[w+0]), REG(MCRYPTO_AES_DATA_IN_0));
		IOWRITE_RAW((in[w+1]), REG(MCRYPTO_AES_DATA_IN_1));
		IOWRITE_RAW((in[w+2]), REG(MCRYPTO_AES_DATA_IN_2));
		IOWRITE_RAW((in[w+3]), REG(MCRYPTO_AES_DATA_IN_3));

		for(i=0; i<MCRYPTO_AES_WAIT_CYCLES; i++) {
			cpu_relax();
			status = IOREAD32(REG(MCRYPTO_AES_STATUS));
			if(status & MCRYPTO_AES_STATUS_IDLE)
				break;
		}

		BUG_ON(!g_ignore_timeout && (i==MCRYPTO_AES_WAIT_CYCLES));

		out[w+0] = IOREAD_RAW(REG(MCRYPTO_AES_DATA_OUT_0));
		out[w+1] = IOREAD_RAW(REG(MCRYPTO_AES_DATA_OUT_1));
		out[w+2] = IOREAD_RAW(REG(MCRYPTO_AES_DATA_OUT_2));
		out[w+3] = IOREAD_RAW(REG(MCRYPTO_AES_DATA_OUT_3));
		DBG("out: %08X %08X %08X %08X", out[w+0], out[w+1], out[w+2], out[w+3]);
	}
	DBG("leaving");
}
/*}}}*/

/*{{{ CRYPTO_ALG_TYPE_CIPHER aes_ede chipher*/
static void mcrypto_aes_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct mcrypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	DBG("entering");
	/* Our HW can't encrypt anything */
	crypto_cipher_encrypt_one(ctx->fallback, out, in);
}

static void mcrypto_aes_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	unsigned long iflags;
	struct mcrypto_device *device = &g_mcrypto_device;
	struct mcrypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	DBG("entering");

	if(mcrtypto_aes_ctx_keys_loaded(ctx)) {
		MCRYPTO_DEVICE_LOCK(device, iflags);
		mcrypto_aes_io_setctx(device, ctx);
		mcrypto_aes_io_crypt(device, out, in, AES_BLOCK_SIZE);
		MCRYPTO_DEVICE_UNLOCK(device, iflags);
	}
	else {
		DBG("fallback to default cipher");
		crypto_cipher_decrypt_one(ctx->fallback, out, in);
	}
}

static int mcrypto_aes_setkey(struct crypto_tfm *tfm, const u8 *key, unsigned int len)
{
	struct mcrypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	int ret;

	DBG("entering");

	if(len == MCRYPTO_AES_KEYSIZE) {
		mcrypto_aes_ctx_loadkeys(ctx, key);
	}
	else {
		mcrypto_aes_ctx_unloadkeys(ctx);
	}

	ctx->fallback->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	ctx->fallback->base.crt_flags |= (tfm->crt_flags & CRYPTO_TFM_REQ_MASK);

	ret = crypto_cipher_setkey(ctx->fallback, key, len);
	if (ret) {
		ERR("Fallback chipher returned error: %d", ret);
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (ctx->fallback->base.crt_flags & CRYPTO_TFM_RES_MASK);
	}
	return ret;
}

static int fallback_init(struct crypto_tfm *tfm)
{
	const char *name = tfm->__crt_alg->cra_name;
	struct mcrypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	DBG("entering");

	ctx->fallback = crypto_alloc_cipher(name, 0,
			CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(ctx->fallback)) {
		ERR( "Error allocating fallback algo %s\n", name);
		return PTR_ERR(ctx->fallback);
	}
	return 0;
}

static void fallback_exit(struct crypto_tfm *tfm)
{
	struct mcrypto_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	DBG("entering");
	crypto_free_cipher(ctx->fallback);
	ctx->fallback = NULL;
}
	

static struct crypto_alg mcrypto_aes_alg = {
	.cra_name		=	"aes",
	.cra_driver_name	=	"mcrypto_aes",
	.cra_priority		=	300,
	.cra_alignmask		=	3,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER | CRYPTO_ALG_NEED_FALLBACK,
	.cra_init		=	fallback_init,
	.cra_exit		=	fallback_exit,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct mcrypto_aes_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(mcrypto_aes_alg.cra_list),
	.cra_u			=	{
		.cipher	=	{
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey		=	mcrypto_aes_setkey,
			.cia_encrypt		=	mcrypto_aes_encrypt,
			.cia_decrypt		=	mcrypto_aes_decrypt
		}
	}
};
/*}}}*/

/*probe/remove{{{*/
/*
 * Linux Device Model integration
 */
static int of_mcrypto_aes_probe(struct platform_device *pdev)
{
	int ret;
	struct mcrypto_device *device;
	printk("ENTER AES\n");
	DBG("entering");
	device = &g_mcrypto_device;
	device->dev = &pdev->dev;
	platform_set_drvdata(pdev, device);

	device->res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!device->res_mem) {
		dev_err(device->dev, "failed to find device registers\n");
		ret = -EINVAL;
		goto error_kfree;
	}

	device->irq = platform_get_irq(pdev, 0);
	if (!device->irq) {
		dev_err(device->dev, "failed to find device irq\n");
		ret = -EINVAL;
		goto error_kfree;
	}

	device->io = devm_ioremap_resource(&pdev->dev, device->res_mem);
	if (!device->io) {
		dev_err(device->dev, "cant remap io memory\n");
		ret = -ENOMEM;
		goto error_release_mem_reg;
	}

	ret = mcrypto_aes_init(device);
	if(ret) {
		dev_err(device->dev, "mcrypto_aes_ede_init() failed. Error code: %d\n", ret);
		goto error_iounmap;
	}

	ret = crypto_register_alg(&mcrypto_aes_alg);
	if(ret) {
		dev_err(device->dev, "crypto_register_alg() failed. Error code: %d\n", ret);
		goto error_uninit_mcrypto;
	}
	printk("EXIT PROBE\n");
out:
	DBG("leaving");
	return ret;

error_uninit_mcrypto:
	mcrypto_aes_uninit(device);
error_iounmap:
	iounmap(device->io);
error_release_mem_reg:
	release_mem_region(device->res_mem->start, device->res_mem->end - device->res_mem->start + 1);
error_kfree:
	goto out;
}

static int of_mcrypto_aes_remove(struct platform_device *pdev)
{
	struct mcrypto_device *device = platform_get_drvdata(pdev);
	DBG("entering");
	if(!device)
		goto out;

	crypto_unregister_alg(&mcrypto_aes_alg);
	mcrypto_aes_uninit(device);
out:
	DBG("leaving");
	return 0;
}
/*}}}*/

static struct of_device_id of_platform_mcrypto_aes_table[] = {
	{ .compatible = "rcm,mcrypto_aes" },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, of_platform_mcrypto_aes_table);

static struct platform_driver of_platform_mcrypto_aes_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_platform_mcrypto_aes_table,
	},
	.probe = of_mcrypto_aes_probe,
	.remove = of_mcrypto_aes_remove
};

module_platform_driver(of_platform_mcrypto_aes_driver);

MODULE_AUTHOR("Sergey Mironov <ierton@gmail.com>");
MODULE_DESCRIPTION("ModuleCrypto AES device driver");
MODULE_LICENSE("GPL");

