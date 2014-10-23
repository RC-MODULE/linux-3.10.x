/*
 * drivers/mtd/devices/mnand.c
 *
 *	Copyright (C) 2010 Module
 *	Written by Kirill Mikhailov
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>


#include <linux/mtd/mnand.h>

#define DRIVER_NAME "mnand"

#define MNAND_DECLARE_PARAM(name, def) \
	static int g_mnand_##name = def; \
	module_param_named(name, g_mnand_##name, int, 0);

MNAND_DECLARE_PARAM(debug, 0);

#define TRACE(level, format, ... ) \
	do { \
		if(g_mnand_debug) { \
			printk(level DRIVER_NAME "/%s:%d: " format, __func__, __LINE__, ##__VA_ARGS__); \
		} \
	} while(0);

struct mem_resource {
        struct resource* res;
        void __iomem* io;
};

//FixMe: Dirty Hack
static volatile int init_flag=0;

static void release_mem_resource(struct mem_resource* ret)
{
        iounmap(ret->io);
        TRACE(KERN_DEBUG,"releasing mem region(0x%08X,0x%08X)\n",
              ret->res->start, ret->res->end - ret->res->start+1);
        release_mem_region(ret->res->start, ret->res->end - ret->res->start + 1);
}

enum {
        MNAND_IDLE = 0,
        MNAND_WRITE,
        MNAND_READ,
        MNAND_ERASE,
        MNAND_READID,
        MNAND_RESET
};

struct mnand_chip {
        struct platform_device* dev;
        struct mem_resource regs;
        int irq;

        uint32_t state;

        dma_addr_t dma_handle;
        size_t dma_size;
        void* dma_area;

        uint64_t active_page;
        struct mtd_info mtd;

        uint32_t status1;
        uint32_t status2;

        uint32_t ecc_corrected;
        uint32_t ecc_failed;
	uint64_t chip_size[2];

} static g_chip;

struct write_info {
        uint32_t address;
        uint32_t value;
};

struct write_info g_write_buffer[1024];
int g_write_pos = 0;

struct page_log {
        uint32_t address;
        uint8_t data[2048+64];
};

struct page_log g_write_page_log[100];
int g_write_page_pos = 0;

struct page_log g_read_page_log[100];
int g_read_page_pos = 0;

#define MNAND_STATUS_ECC_ERROR 0x100

#define MNAND_ECC_BLOCKSIZE 256
#define MNAND_ECC_BYTESPERBLOCK 3

static struct nand_ecclayout g_ecclayout = {
        .eccbytes = 24,
        .eccpos = {
                1, 2, 3, 4, 5, 6, 7, 8,
                9, 10, 11, 12, 13, 14, 15, 16,
                17, 18, 19, 20, 21, 22, 23, 24
        },
        .oobfree = {
                {
                        .offset = 25,
                        .length = 39
                }
        },
        .oobavail = 39
            };

static DEFINE_SEMAPHORE(g_mutex);
static DECLARE_COMPLETION(g_completion);

void mnand_set(uint32_t addr, uint32_t val)
{
        struct write_info wi = { addr, val };

        if(g_write_pos == 1024)
                g_write_pos = 0;

        g_write_buffer[g_write_pos++] = wi;

        TRACE(KERN_DEBUG, "set(0x%08X, 0x%08X)\n", addr, val);
        iowrite32(val, g_chip.regs.io + addr);
}

uint32_t mnand_get(uint32_t addr)
{
        uint32_t r = ioread32(g_chip.regs.io + addr);
        TRACE(KERN_DEBUG, "0x%08X = get(0x%08X)\n", r, addr);
        return r;
}

static inline char* mnand_get_read_ecc(size_t i)
{
        return g_chip.dma_area + g_chip.mtd.writesize + g_ecclayout.eccpos[MNAND_ECC_BYTESPERBLOCK * i];
}

static inline char* mnand_get_calculated_ecc(size_t i)
{
        static int data;
        char* cdata = (char*)&data;
        char c;
        data = mnand_get(0xD8+i*sizeof(uint32_t));
        c = cdata[0];
        cdata[0] = cdata[2];
        cdata[2] = c;
        return cdata;
}

void mnand_cs(int cs) 
{
        uint32_t tmp;
        BUG_ON(cs>1);
        tmp = mnand_get(0x4);
        tmp &= ~(1 << 26);
        tmp |= cs << 26;
        mnand_set(0x4, tmp);
}

off_t mnand_chip_offset(off_t size) 
{
	int cs = 0;
	if (size >= g_chip.chip_size[0]) {
		cs++;
		size-=g_chip.chip_size[0];
	}
	mnand_cs(cs);
	return size;
}

void mnand_reset_grabber(void)
{
        /*    mnand_set(0x1008,0x8);
        	  mnand_set(0x100C, 1);
        	  mnand_set(0x100C, 0);

        	  mnand_set(0x2008, 0xffffffff);
        	  mnand_set(0x200C, 1);
        	  mnand_set(0x200C, 0);

        	  mnand_set(0x3008, 0xffffffff);
        	  mnand_set(0x300C, 1);
        	  mnand_set(0x300C, 0);

        	  mnand_set(0x400C, 1);
        	  mnand_set(0x400C, 0);


        	  mnand_set(0x500C, 1);
        	  mnand_set(0x500C, 0);

        	  mnand_set(0x800C, 1);
        	  mnand_set(0x800C, 0);
        	  mnand_set(0x8008, 0xffffffff);*/
}

static irqreturn_t interrupt_handler(int irq, void* data)
{
        if (init_flag==0) {
                printk("HACK: Preventing a race condition");
                return IRQ_HANDLED;
        }
        g_chip.status1 = mnand_get(0x10);
        g_chip.status2 = mnand_get(0xb4);

        TRACE(KERN_DEBUG,"0x%08X 0x%08X\n", g_chip.status1, g_chip.status2);

        BUG_ON(g_chip.state == MNAND_IDLE);

        switch(g_chip.state) {
        case MNAND_READ:
        case MNAND_READID:
                if((g_chip.status2 & 0x100)) {
                        g_chip.state = MNAND_IDLE;
                }
                break;
        case MNAND_WRITE:
                if(g_chip.status1 & 0x2) {
                        g_chip.state = MNAND_IDLE;
                }
                break;
        case MNAND_ERASE:
                if(g_chip.status1 & 0x1) {
                        g_chip.state = MNAND_IDLE;
                }
                break;
        case MNAND_RESET:
                if(g_chip.status1 & 0x20) {
                        g_chip.state = MNAND_IDLE;
                }
                break;
        default:
                BUG_ON(1);
                break;
        }

        BUG_ON(g_chip.state != MNAND_IDLE);

        if(g_chip.state == MNAND_IDLE)
                complete_all(&g_completion);

        return IRQ_HANDLED;
}

void mnand_hw_init(void)
{
        mnand_set(0x6C, 0xF);
        mnand_set(0x74, 1);
        mnand_set(0x94, 0x31);


#ifndef CONFIG_MTD_MNAND_SLOW
        mnand_set(0xB8, 0x01010106); // tADL=17, tALH=2, tALS=5, tCH=2
        mnand_set(0xBC, 0x01010201); // tCLH=2, tCLS=5, tCS=6, tDH=2
        mnand_set(0xC0, 0x02010301); // tWP=5, tWH=3, tWC=9, tDS=4
        mnand_set(0xC4, 0x03010102); // tOH=1, tCLR=2, tAR=3, tWW=9
        mnand_set(0xC8, 0x01030102); // tREH=3, tRC=9, tIR=1, tCOH=1
        mnand_set(0xCC, 0x02010902); // tRP=5, tRLOH=1, tRHW=17, tRHOH=1
        mnand_set(0xD0, 0x04040602); // tCHZ=9, tCEA=9, tWHR=10, tRR=4
        mnand_set(0xD4, 0x000A0A03); // tWB=17, tRHZ=17, tREA=4
#else
        mnand_set(0xB8, 0x02050211); // tADL=17, tALH=2, tALS=5, tCH=2
        mnand_set(0xBC, 0x02060502); // tCLH=2, tCLS=5, tCS=6, tDH=2
        mnand_set(0xC0, 0x05030904); // tWP=5, tWH=3, tWC=9, tDS=4
        mnand_set(0xC4, 0x01020309); // tOH=1, tCLR=2, tAR=3, tWW=9
        mnand_set(0xC8, 0x03090101); // tREH=3, tRC=9, tIR=1, tCOH=1
        mnand_set(0xCC, 0x05011101); // tRP=5, tRLOH=1, tRHW=17, tRHOH=1
        mnand_set(0xD0, 0x09090A04); // tCHZ=9, tCEA=9, tWHR=10, tRR=4
        mnand_set(0xD4, 0x00111104); // tWB=17, tRHZ=17, tREA=4
        printk("mnand: using fail-safe timings\n");
#endif

#ifdef CONFIG_MTD_MNAND_TWOBYTES
        mnand_set(0x04, 0x4204E3);
#else
        mnand_set(0x04, 0x24204E3);
#endif

        mnand_set(0x1C, 0xC0);
        mnand_set(0x24, 0xF);
        mnand_set(0x2C, 0);
        mnand_set(0x38, 1);
}

static void mnand_prepare_dma_read(size_t bytes)
{
        TRACE(KERN_DEBUG, "bytes %d\n", bytes);

        mnand_set(0x88, g_chip.dma_handle);
        mnand_set(0x8C, g_chip.dma_handle + bytes - 1);
        mnand_set(0x90, 0x80004000);

        while(mnand_get(0x90) & 0x80000000);
}

static void mnand_prepare_dma_write(size_t bytes)
{
        TRACE(KERN_DEBUG, "bytes %d\n", bytes);

        mnand_set(0x40, g_chip.dma_handle);
        mnand_set(0x44, g_chip.dma_handle + bytes - 1);
        mnand_set(0x48, 0x80001000);
        mnand_set(0x4C, 0x2000);

        while(mnand_get(0x48) & 0x80000000);

        mnand_set(0x18, 0x40000001);
}

int mnand_ready(void)
{
        return (mnand_get(0) & 0x200) != 0;
}

static int mnand_core_reset(void)
{

        init_completion(&g_completion);
        init_flag=1;
        g_chip.state = MNAND_RESET;

        mnand_set(0x08, 0x2C);
        wait_for_completion_io(&g_completion);

        BUG_ON(!mnand_ready());

        g_chip.state = MNAND_IDLE;

        return 0;
}

static int mnand_core_erase(loff_t off)
{
        BUG_ON(g_chip.state != MNAND_IDLE);
        BUG_ON(!mnand_ready());

	off = mnand_chip_offset(off);

        mnand_reset_grabber();

        init_completion(&g_completion);

        g_chip.state = MNAND_ERASE;

        mnand_set(0x8, 0x2b);
        mnand_set(0xc, (off >> g_chip.mtd.erasesize_shift) << (ffs(g_chip.mtd.erasesize)));

        wait_for_completion_io(&g_completion);

        BUG_ON(!mnand_ready());

        g_chip.state = MNAND_IDLE;

        return (mnand_get(0) & NAND_STATUS_FAIL) ? -EIO : 0;
}

static void mnand_core_read_id(size_t bytes)
{
        mnand_reset_grabber();

        BUG_ON(g_chip.state != MNAND_IDLE);
        BUG_ON(!mnand_ready());

        mnand_prepare_dma_read(bytes);

        init_completion(&g_completion);

        g_chip.state = MNAND_READID;

        mnand_set(0x08, 0x25);
        mnand_set(0x0C, bytes << 8);

        wait_for_completion_io(&g_completion);

        BUG_ON(!mnand_ready());
        g_chip.state = MNAND_IDLE;
}

/*
 * invparity is a 256 byte table that contains the odd parity
 * for each byte. So if the number of bits in a byte is even,
 * the array element is 1, and when the number of bits is odd
 * the array eleemnt is 0.
 */
static const char invparity[256] = {
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
        1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};



static const char addressbits[256] = {
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,
        0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,
        0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
        0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05,
        0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
        0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05,
        0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,
        0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,
        0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
        0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05,
        0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
        0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05,
        0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
        0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09,
        0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b,
        0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09,
        0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b,
        0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d,
        0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f,
        0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d,
        0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f,
        0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09,
        0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b,
        0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09,
        0x0a, 0x0a, 0x0b, 0x0b, 0x0a, 0x0a, 0x0b, 0x0b,
        0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d,
        0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f,
        0x0c, 0x0c, 0x0d, 0x0d, 0x0c, 0x0c, 0x0d, 0x0d,
        0x0e, 0x0e, 0x0f, 0x0f, 0x0e, 0x0e, 0x0f, 0x0f
};

static const char bitsperbyte[256] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};


static int mnand_correct_data(struct mtd_info *mtd, unsigned char *buf,
                              unsigned char *read_ecc, unsigned char *calc_ecc)
{
        unsigned char b0, b1, b2;
        unsigned char byte_addr, bit_addr;
        /* 256 or 512 bytes/ecc  */
        const uint32_t eccsize_mult = 1;

        /*
         * b0 to b2 indicate which bit is faulty (if any)
         * we might need the xor result  more than once,
         * so keep them in a local var
         */
#ifdef CONFIG_MTD_NAND_ECC_SMC
        b0 = read_ecc[0] ^ calc_ecc[0];
        b1 = read_ecc[1] ^ calc_ecc[1];
#else
        b0 = read_ecc[1] ^ calc_ecc[1];
        b1 = read_ecc[0] ^ calc_ecc[0];
#endif
        b2 = read_ecc[2] ^ calc_ecc[2];

        /* check if there are any bitfaults */

        /* repeated if statements are slightly more efficient than switch ... */
        /* ordered in order of likelihood */

        if ((b0 | b1 | b2) == 0)
                return 0;	/* no error */

        if ((((b0 ^ (b0 >> 1)) & 0x55) == 0x55) &&
            (((b1 ^ (b1 >> 1)) & 0x55) == 0x55) &&
            ((eccsize_mult == 1 && ((b2 ^ (b2 >> 1)) & 0x54) == 0x54) ||
             (eccsize_mult == 2 && ((b2 ^ (b2 >> 1)) & 0x55) == 0x55))) {
                /* single bit error */
                /*
                 * rp17/rp15/13/11/9/7/5/3/1 indicate which byte is the faulty
                 * byte, cp 5/3/1 indicate the faulty bit.
                 * A lookup table (called addressbits) is used to filter
                 * the bits from the byte they are in.
                 * A marginal optimisation is possible by having three
                 * different lookup tables.
                 * One as we have now (for b0), one for b2
                 * (that would avoid the >> 1), and one for b1 (with all values
                 * << 4). However it was felt that introducing two more tables
                 * hardly justify the gain.
                 *
                 * The b2 shift is there to get rid of the lowest two bits.
                 * We could also do addressbits[b2] >> 1 but for the
                 * performace it does not make any difference
                 */
                if (eccsize_mult == 1)
                        byte_addr = (addressbits[b1] << 4) + addressbits[b0];
                else
                        byte_addr = (addressbits[b2 & 0x3] << 8) +
                                    (addressbits[b1] << 4) + addressbits[b0];
                bit_addr = addressbits[b2 >> 2];
                /* flip the bit */
                buf[byte_addr] ^= (1 << bit_addr);
                return 1;

        }
        /* count nr of bits; use table lookup, faster than calculating it */
        if ((bitsperbyte[b0] + bitsperbyte[b1] + bitsperbyte[b2]) == 1)
                return 1;	/* error in ecc data; no action needed */

        printk(KERN_ERR "uncorrectable error : \n");
        return -1;
}

int mnand_calculate_ecc(struct mtd_info *mtd, const unsigned char *buf,
                        unsigned char *code)
{
        int i;
        const uint32_t *bp = (uint32_t *)buf;
        /* 256 or 512 bytes/ecc  */
        const uint32_t eccsize_mult = 1;

        uint32_t cur;		/* current value in buffer */
        /* rp0..rp15..rp17 are the various accumulated parities (per byte) */
        uint32_t rp0, rp1, rp2, rp3, rp4, rp5, rp6, rp7;
        uint32_t rp8, rp9, rp10, rp11, rp12, rp13, rp14, rp15, rp16;
        uint32_t uninitialized_var(rp17);	/* to make compiler happy */
        uint32_t par;		/* the cumulative parity for all data */
        uint32_t tmppar;	/* the cumulative parity for this iteration;
						   for rp12, rp14 and rp16 at the end of the
						   loop */

        par = 0;
        rp4 = 0;
        rp6 = 0;
        rp8 = 0;
        rp10 = 0;
        rp12 = 0;
        rp14 = 0;
        rp16 = 0;

        /*
         * The loop is unrolled a number of times;
         * This avoids if statements to decide on which rp value to update
         * Also we process the data by longwords.
         * Note: passing unaligned data might give a performance penalty.
         * It is assumed that the buffers are aligned.
         * tmppar is the cumulative sum of this iteration.
         * needed for calculating rp12, rp14, rp16 and par
         * also used as a performance improvement for rp6, rp8 and rp10
         */
        for (i = 0; i < eccsize_mult << 2; i++) {
                cur = *bp++;
                tmppar = cur;
                rp4 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp6 ^= tmppar;
                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp8 ^= tmppar;

                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                rp6 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp6 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp10 ^= tmppar;

                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                rp6 ^= cur;
                rp8 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp6 ^= cur;
                rp8 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                rp8 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp8 ^= cur;

                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                rp6 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp6 ^= cur;
                cur = *bp++;
                tmppar ^= cur;
                rp4 ^= cur;
                cur = *bp++;
                tmppar ^= cur;

                par ^= tmppar;
                if ((i & 0x1) == 0)
                        rp12 ^= tmppar;
                if ((i & 0x2) == 0)
                        rp14 ^= tmppar;
                if (eccsize_mult == 2 && (i & 0x4) == 0)
                        rp16 ^= tmppar;
        }

        /*
         * handle the fact that we use longword operations
         * we'll bring rp4..rp14..rp16 back to single byte entities by
         * shifting and xoring first fold the upper and lower 16 bits,
         * then the upper and lower 8 bits.
         */
        rp4 ^= (rp4 >> 16);
        rp4 ^= (rp4 >> 8);
        rp4 &= 0xff;
        rp6 ^= (rp6 >> 16);
        rp6 ^= (rp6 >> 8);
        rp6 &= 0xff;
        rp8 ^= (rp8 >> 16);
        rp8 ^= (rp8 >> 8);
        rp8 &= 0xff;
        rp10 ^= (rp10 >> 16);
        rp10 ^= (rp10 >> 8);
        rp10 &= 0xff;
        rp12 ^= (rp12 >> 16);
        rp12 ^= (rp12 >> 8);
        rp12 &= 0xff;
        rp14 ^= (rp14 >> 16);
        rp14 ^= (rp14 >> 8);
        rp14 &= 0xff;
        if (eccsize_mult == 2) {
                rp16 ^= (rp16 >> 16);
                rp16 ^= (rp16 >> 8);
                rp16 &= 0xff;
        }

        /*
         * we also need to calculate the row parity for rp0..rp3
         * This is present in par, because par is now
         * rp3 rp3 rp2 rp2 in little endian and
         * rp2 rp2 rp3 rp3 in big endian
         * as well as
         * rp1 rp0 rp1 rp0 in little endian and
         * rp0 rp1 rp0 rp1 in big endian
         * First calculate rp2 and rp3
         */
#ifdef __BIG_ENDIAN
        rp2 = (par >> 16);
        rp2 ^= (rp2 >> 8);
        rp2 &= 0xff;
        rp3 = par & 0xffff;
        rp3 ^= (rp3 >> 8);
        rp3 &= 0xff;
#else
        rp3 = (par >> 16);
        rp3 ^= (rp3 >> 8);
        rp3 &= 0xff;
        rp2 = par & 0xffff;
        rp2 ^= (rp2 >> 8);
        rp2 &= 0xff;
#endif

        /* reduce par to 16 bits then calculate rp1 and rp0 */
        par ^= (par >> 16);
#ifdef __BIG_ENDIAN
        rp0 = (par >> 8) & 0xff;
        rp1 = (par & 0xff);
#else
        rp1 = (par >> 8) & 0xff;
        rp0 = (par & 0xff);
#endif

        /* finally reduce par to 8 bits */
        par ^= (par >> 8);
        par &= 0xff;

        /*
         * and calculate rp5..rp15..rp17
         * note that par = rp4 ^ rp5 and due to the commutative property
         * of the ^ operator we can say:
         * rp5 = (par ^ rp4);
         * The & 0xff seems superfluous, but benchmarking learned that
         * leaving it out gives slightly worse results. No idea why, probably
         * it has to do with the way the pipeline in pentium is organized.
         */
        rp5 = (par ^ rp4) & 0xff;
        rp7 = (par ^ rp6) & 0xff;
        rp9 = (par ^ rp8) & 0xff;
        rp11 = (par ^ rp10) & 0xff;
        rp13 = (par ^ rp12) & 0xff;
        rp15 = (par ^ rp14) & 0xff;
        if (eccsize_mult == 2)
                rp17 = (par ^ rp16) & 0xff;

        /*
         * Finally calculate the ecc bits.
         * Again here it might seem that there are performance optimisations
         * possible, but benchmarks showed that on the system this is developed
         * the code below is the fastest
         */
#ifdef CONFIG_MTD_NAND_ECC_SMC
        code[0] =
                (invparity[rp7] << 7) |
                (invparity[rp6] << 6) |
                (invparity[rp5] << 5) |
                (invparity[rp4] << 4) |
                (invparity[rp3] << 3) |
                (invparity[rp2] << 2) |
                (invparity[rp1] << 1) |
                (invparity[rp0]);
        code[1] =
                (invparity[rp15] << 7) |
                (invparity[rp14] << 6) |
                (invparity[rp13] << 5) |
                (invparity[rp12] << 4) |
                (invparity[rp11] << 3) |
                (invparity[rp10] << 2) |
                (invparity[rp9] << 1)  |
                (invparity[rp8]);
#else
        code[1] =
                (invparity[rp7] << 7) |
                (invparity[rp6] << 6) |
                (invparity[rp5] << 5) |
                (invparity[rp4] << 4) |
                (invparity[rp3] << 3) |
                (invparity[rp2] << 2) |
                (invparity[rp1] << 1) |
                (invparity[rp0]);
        code[0] =
                (invparity[rp15] << 7) |
                (invparity[rp14] << 6) |
                (invparity[rp13] << 5) |
                (invparity[rp12] << 4) |
                (invparity[rp11] << 3) |
                (invparity[rp10] << 2) |
                (invparity[rp9] << 1)  |
                (invparity[rp8]);
#endif
        if (eccsize_mult == 1)
                code[2] =
                        (invparity[par & 0xf0] << 7) |
                        (invparity[par & 0x0f] << 6) |
                        (invparity[par & 0xcc] << 5) |
                        (invparity[par & 0x33] << 4) |
                        (invparity[par & 0xaa] << 3) |
                        (invparity[par & 0x55] << 2) |
                        3;
        else
                code[2] =
                        (invparity[par & 0xf0] << 7) |
                        (invparity[par & 0x0f] << 6) |
                        (invparity[par & 0xcc] << 5) |
                        (invparity[par & 0x33] << 4) |
                        (invparity[par & 0xaa] << 3) |
                        (invparity[par & 0x55] << 2) |
                        (invparity[rp17] << 1) |
                        (invparity[rp16] << 0);
        return 0;
}

static int mnand_core_read(loff_t off)
{
        loff_t page;  
        size_t corrected = 0;
        size_t failed = 0;

        off = mnand_chip_offset(off);
        page = off & (~(g_chip.mtd.writesize-1));

        BUG_ON(g_chip.state != MNAND_IDLE);
        BUG_ON(!mnand_ready());

        mnand_reset_grabber();

        //    printk(KERN_DEBUG "core_read %08llX %08llX\n", off, page);

        if(g_chip.active_page != page) {
                g_chip.active_page = -1;

                mnand_prepare_dma_read(g_chip.mtd.writesize + g_chip.mtd.oobsize);

                init_completion(&g_completion);
                g_chip.state = MNAND_READ;

                mnand_set(0x08, 0x20);
                mnand_set(0x0C, (off >> g_chip.mtd.writesize_shift) << 12);

                wait_for_completion_io(&g_completion);

                BUG_ON(!mnand_ready());
                g_chip.state = MNAND_IDLE;

                g_read_page_log[g_read_page_pos].address = page;
                memcpy(g_read_page_log[g_read_page_pos].data, g_chip.dma_area, 2048+64);

                ++g_read_page_pos;

                if(g_read_page_pos == sizeof(g_read_page_log) / sizeof(g_read_page_log[0]))
                        g_read_page_pos = 0;

                if(/*mnand_get(0) & MNAND_STATUS_ECC_ERROR*/1) {
                        size_t i;
                        /*Check if we had a bad block*/
                        if(*((char*)(g_chip.dma_area + g_chip.mtd.writesize)) == 0xFF) {
                                for(i = 0; i < g_chip.mtd.writesize/MNAND_ECC_BLOCKSIZE; ++i) {
                                        char ecc[3];
                                        mnand_calculate_ecc(&g_chip.mtd, g_chip.dma_area + i * MNAND_ECC_BLOCKSIZE, ecc);
                                        if(memcmp(mnand_get_calculated_ecc(i),ecc,3)!=0) {
                                                printk("\n\necc mismatch %02X %02X %02X - %02x %02X %02X \n\n",
                                                       ecc[0], ecc[1], ecc[2],
                                                       mnand_get_calculated_ecc(i)[0],
                                                       mnand_get_calculated_ecc(i)[1],
                                                       mnand_get_calculated_ecc(i)[2]);

                                                BUG_ON(1);
                                        }
                                        if(memcmp(mnand_get_read_ecc(i), mnand_get_calculated_ecc(i), MNAND_ECC_BYTESPERBLOCK)) {
                                                int stat = mnand_correct_data(&g_chip.mtd,
                                                                              g_chip.dma_area + i * MNAND_ECC_BLOCKSIZE,
                                                                              mnand_get_read_ecc(i),
                                                                              mnand_get_calculated_ecc(i));
                                                if( stat < 0 ) {
                                                        ++failed;
                                                } else {
                                                        ++corrected;
                                                }
                                        }
                                }
                        }
                }

                g_chip.ecc_corrected += corrected;
                g_chip.ecc_failed += failed;

                /*if(!failed)
                  g_chip.active_page = page;*/
        }

        return 0;
}

static int mnand_core_write(loff_t off)
{
        BUG_ON(!mnand_ready());
        BUG_ON(g_chip.state != MNAND_IDLE);
	
        off = mnand_chip_offset(off);

        mnand_reset_grabber();

        g_chip.active_page = -1;

        mnand_prepare_dma_write(g_chip.mtd.writesize + g_chip.mtd.oobsize);

        init_completion(&g_completion);

        g_chip.state = MNAND_WRITE;

        g_write_page_log[g_write_page_pos].address = off;
        memcpy(g_write_page_log[g_write_page_pos].data, g_chip.dma_area, 2048+64);

        ++g_write_page_pos;

        if((g_write_page_pos) == sizeof(g_write_page_log)/sizeof(g_write_page_log[0]))
                g_write_page_pos = 0;

        mnand_set(0x08, 0x27);
        mnand_set(0x0C, (u32)((off >> g_chip.mtd.writesize_shift) << 12));
        BUG_ON(mnand_get(0x0C) != (u32)((off >> g_chip.mtd.writesize_shift) << 12));

        wait_for_completion_io(&g_completion);

        BUG_ON(!mnand_ready());
        g_chip.state = MNAND_IDLE;

        return (mnand_get(0) & NAND_STATUS_FAIL) ? -EIO : 0;
}

static int mnand_read_id(int cs)
{
        struct nand_flash_dev* type = 0;
        int i;
        int retries=5;
        char* vendor = "unknown";
        mnand_cs(cs);

        /* hw weirdness: sometimes after an unlucky reset or incomplete
         * powerdown a bad id may be read.
         */

        for (i=0; i<retries; i++) {
                mnand_core_read_id(2);
                if (*(((uint8_t*)g_chip.dma_area)+1)!=0x00)
                        break;
        }

        if (i==retries) {
                printk("mnand: Bad chip id or no chip at CS%d\n", cs);
                return -ENODEV;
        }

        /* Lookup the flash vendor */
        for (i = 0; nand_manuf_ids[i].name != NULL; i++) {
                if (((uint8_t*)g_chip.dma_area)[0] == nand_manuf_ids[i].id) {
                        vendor =  nand_manuf_ids[i].name;
                        break;
                }
        }

        /* Lookup the flash id */
        for (i = 0; nand_flash_ids[i].name != NULL; i++) {
                if (((uint8_t*)g_chip.dma_area)[1] == nand_flash_ids[i].dev_id &&
                        nand_flash_ids[i].mfr_id == 0) {
                        type = &nand_flash_ids[i];
                        break;
                }
        }

        if (!type)
                return -ENODEV;



/* We're not likely work if erase/write/oob sizes on different CS
 *  differ.
 */

#define NBUG(a,b) \
       { if (cs && (a!=b)) goto inconsistent; }

        if(!type->pagesize) {
                uint8_t extid;
                u_long tmp;

                mnand_core_read_id(5);

                extid = ((uint8_t*)g_chip.dma_area)[3];
                printk(KERN_DEBUG "flash ext_id 0x%02X\n", extid);

                tmp = 1024 << (extid & 0x3);
                NBUG(g_chip.mtd.writesize, tmp);
                g_chip.mtd.writesize = tmp;

                extid >>= 2;

                tmp = (8 << (extid & 0x01)) * (g_chip.mtd.writesize >> 9);
                NBUG(g_chip.mtd.oobsize, tmp);
                g_chip.mtd.oobsize = tmp;

                extid >>= 2;
                tmp = (64 * 1024) << (extid & 0x03);
                NBUG(g_chip.mtd.erasesize, tmp);
                g_chip.mtd.erasesize = tmp;

        } else {
                g_chip.mtd.erasesize = type->erasesize;
                g_chip.mtd.writesize = type->pagesize;
                g_chip.mtd.oobsize = g_chip.mtd.writesize / 32;
        }

        g_chip.mtd.writebufsize = g_chip.mtd.writesize;
        g_chip.mtd.size += (uint64_t) type->chipsize << 20;
        g_chip.chip_size[cs] = (uint64_t) type->chipsize << 20;
 
        printk("%s CS%d %s size(%u) writesize(%u) oobsize(%u) erasesize(%u)\n",
               g_chip.mtd.name, cs, vendor, type->chipsize, g_chip.mtd.writesize,
               g_chip.mtd.oobsize, g_chip.mtd.erasesize);

        return 0;

inconsistent:
        printk("mnand: Chip configurations differ, ignoring CS1\n");
        return -EIO;
}

static int mnand_erase(struct mtd_info* mtd, struct erase_info* instr)
{
        int err;

        TRACE(KERN_DEBUG, "addr=0x%08llX, len=%lld\n", instr->addr, instr->len);
	
        if(instr->addr >= g_chip.mtd.size ||
            instr->addr + instr->len > g_chip.mtd.size  ||
            instr->len != g_chip.mtd.erasesize)
                return -EINVAL;

        down(&g_mutex);

        err = mnand_core_erase(instr->addr);

        up(&g_mutex);

        if(err) {
                printk(KERN_ERR "erase failed with %d at 0x%08llx\n", err, instr->addr);
                instr->state = MTD_ERASE_FAILED;
                instr->fail_addr = instr->addr;
        } else {
                instr->state = MTD_ERASE_DONE;
        }

        mtd_erase_callback(instr);

        return 0;
}

#define PAGE_ALIGNED(x) (((x) & (g_chip.mtd.writesize-1)) ==0)

static uint8_t* mnand_fill_oob(uint8_t *oob, struct mtd_oob_ops *ops)
{
        size_t len = ops->ooblen;

        switch(ops->mode) {
        case MTD_OPS_PLACE_OOB:
        case MTD_OPS_RAW:
                memcpy(g_chip.dma_area + g_chip.mtd.writesize + ops->ooboffs, oob, len);
                return oob + len;
        case MTD_OPS_AUTO_OOB: {
                struct nand_oobfree *free = g_chip.mtd.ecclayout->oobfree;
                uint32_t boffs = 0, woffs = ops->ooboffs;
                size_t bytes = 0;

                for(; free->length && len; free++, len -= bytes) {
                        /* Write request not from offset 0 ? */
                        if (unlikely(woffs)) {
                                if (woffs >= free->length) {
                                        woffs -= free->length;
                                        continue;
                                }
                                boffs = free->offset + woffs;
                                bytes = min_t(size_t, len,
                                              (free->length - woffs));
                                woffs = 0;
                        } else {
                                bytes = min_t(size_t, len, free->length);
                                boffs = free->offset;
                        }
                        memcpy(g_chip.dma_area + g_chip.mtd.writesize + boffs, oob, bytes);
                        oob += bytes;
                }
                return oob;
        }
        default:
                BUG();
        }
        return NULL;
}

static uint8_t* mnand_transfer_oob(uint8_t *oob, struct mtd_oob_ops *ops, size_t len)
{
        switch(ops->mode) {

        case MTD_OPS_PLACE_OOB:
        case MTD_OPS_RAW:
                TRACE(KERN_DEBUG, "raw transfer\n");
                memcpy(oob, g_chip.dma_area + g_chip.mtd.writesize + ops->ooboffs, len);
                return oob + len;

        case MTD_OPS_AUTO_OOB: {
                struct nand_oobfree *free = g_chip.mtd.ecclayout->oobfree;
                uint32_t boffs = 0, roffs = ops->ooboffs;
                size_t bytes = 0;

                for(; free->length && len; free++, len -= bytes) {
                        /* Read request not from offset 0 ? */
                        if (unlikely(roffs)) {
                                if (roffs >= free->length) {
                                        roffs -= free->length;
                                        continue;
                                }
                                boffs = free->offset + roffs;
                                bytes = min_t(size_t, len,
                                              (free->length - roffs));
                                roffs = 0;
                        } else {
                                bytes = min_t(size_t, len, free->length);
                                boffs = free->offset;
                        }
                        memcpy(oob, g_chip.dma_area + g_chip.mtd.writesize + boffs, bytes);
                        oob += bytes;
                }
                return oob;
        }
        default:
                BUG();
        }
        return NULL;
}

static int mnand_write_oob(struct mtd_info* mtd, loff_t to, struct mtd_oob_ops* ops)
{
        uint8_t* data = ops->datbuf;
        uint8_t* dataend = data ? data + ops->len : 0;
        uint8_t* oob = ops->oobbuf;
        uint8_t* oobend = oob ? oob + ops->ooblen : 0;

        int err;

        TRACE(KERN_DEBUG, "to=0x%08llX, ops.mode=%d, ops.len=%d, ops.ooblen=%d ops.ooboffs=0x%08X\n",
              to, ops->mode, ops->len, ops->ooblen, ops->ooboffs);

        ops->retlen = 0;
        ops->oobretlen = 0;

        if(to >= g_chip.mtd.size || !PAGE_ALIGNED(to) || !PAGE_ALIGNED(dataend - data)) {
                printk(KERN_DEBUG "writing non page aligned data to 0x%08llx 0x%08x\n", to, ops->len);
                return -EINVAL;
        }

        if(ops->ooboffs > mtd->oobsize) {
                printk(KERN_ERR "oob > mtd->oobsize" );
                return -EINVAL;
        }

        err = down_killable(&g_mutex);

        if(0 == err) {
                for(; (data !=dataend) || (oob !=oobend);) {
                        memset(g_chip.dma_area, 0xFF, g_chip.mtd.writesize + g_chip.mtd.oobsize);

                        if(data != dataend) {
                                memcpy(g_chip.dma_area, data, g_chip.mtd.writesize);
                                data += g_chip.mtd.writesize;
                        }

                        if(oob != oobend) {
                                oob = mnand_fill_oob(oob, ops);
                                if(!oob) {
                                        err = -EINVAL;
                                        printk(KERN_ERR "oob einval\n");
                                        break;
                                }
                        }

                        err = mnand_core_write(to);

                        if(err)
                                break;

                        to += g_chip.mtd.writesize;
                }

                up(&g_mutex);
        }

        if(err) {
                printk(KERN_ERR "core write returned error at 0x%08llx\n", to);
                return err;
        }

        ops->retlen = ops->len;
        ops->oobretlen = ops->ooblen;

        return err;
}

static int mnand_read_oob(struct mtd_info* mtd, loff_t from, struct mtd_oob_ops* ops)
{
        uint8_t* data = ops->datbuf;
        uint8_t* oob = ops->oobbuf;
        int err;

        uint8_t* dataend = data + ops->len;
        uint8_t* oobend = oob ? oob + ops->ooblen : 0;

        TRACE(KERN_DEBUG,
              "from=0x%08llX, ops.mode=%d, ops.len=%d, ops.ooblen=%d ops.ooboffs=0x%08X data=%p\n",
              from, ops->mode, ops->len, ops->ooblen, ops->ooboffs, data);

        TRACE(KERN_DEBUG, "oob %p, oobend %p\n", oob, oobend);

        if(ops->len != 0 && data == 0) {
                ops->len = 0;
                dataend = 0;
        }

        ops->retlen = 0;
        ops->oobretlen = 0;

        err = down_killable(&g_mutex);

        if(0 == err) {
                for(;;) {
                        err = mnand_core_read(from);

                        if(err != 0) {
                                break;
                        }

                        if(data != dataend) {
                                size_t shift = (from & (mtd->writesize -1));
                                size_t bytes = min_t(size_t,dataend-data, mtd->writesize-shift);

                                TRACE(KERN_DEBUG, "data %p dataend %p shift 0x%X bytes 0x%X\n",
                                      data, dataend, shift, bytes);

                                memcpy(data, g_chip.dma_area + shift, bytes);
                                data += bytes;
                        }

                        if(oob != oobend) {
                                TRACE(KERN_DEBUG, "oob %p, oobend %p\n", oob, oobend);

                                oob = mnand_transfer_oob(oob, ops, oobend-oob);
                                if(!oob) {
                                        err = -EINVAL;
                                        break;
                                }
                        }

                        if(data == dataend && oob == oobend)
                                break;

                        from &= ~((1 << mtd->writesize_shift)-1);
                        from += mtd->writesize;
                }

                if(data) {
                        if(g_chip.mtd.ecc_stats.failed != g_chip.ecc_failed) {
                                g_chip.mtd.ecc_stats.failed = g_chip.ecc_failed;
                                g_chip.mtd.ecc_stats.corrected = g_chip.ecc_corrected;
                                err = -EBADMSG;
                        } else if(g_chip.mtd.ecc_stats.corrected != g_chip.ecc_corrected) {
                                g_chip.mtd.ecc_stats.failed = g_chip.ecc_failed;
                                g_chip.mtd.ecc_stats.corrected = g_chip.ecc_corrected;
                                err = -EUCLEAN;
                        }
                }

                up(&g_mutex);
        }

	if(err==0 || err == -EUCLEAN) {
		ops->retlen = ops->len;
		ops->oobretlen = ops->ooblen;
	}

	return err;
}

static int mnand_isbad(struct mtd_info* mtd, loff_t off)
{
        uint8_t f=0;
        int err;
        struct mtd_oob_ops ops = {
                .mode = MTD_OPS_RAW,
                .ooblen = 1,
                .oobbuf = &f,
                .ooboffs = 0
        };

        TRACE(KERN_DEBUG, "off 0x%08llX\n", off);

        err = mnand_read_oob(mtd, off, &ops);

        TRACE(KERN_DEBUG,"err %d, f=%0x02X\n", err, f);

        return (err == 0 && f == 0xFF) ? 0 : 1;
}

static int mnand_markbad(struct mtd_info* mtd, loff_t off)
{
        uint8_t f=0;
        struct mtd_oob_ops ops = {
                .mode = MTD_OPS_RAW,
                .ooblen = 1,
                .oobbuf = &f,
                .ooboffs = 0
        };

        TRACE(KERN_DEBUG, "off 0x%08llX\n", off);

        return mnand_write_oob(mtd, off, &ops);
}

static int mnand_write(struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, const u_char* buf)
{
        struct mtd_oob_ops ops = {
                .datbuf = (uint8_t*)buf,
                .len = len
        };
        int err;

        TRACE(KERN_DEBUG, "off=0x%08llX, len=%d\n", off, len);

        err = mnand_write_oob(mtd, off, &ops);
        *retlen = ops.retlen;
        return err;
}

static int mnand_read(struct mtd_info* mtd, loff_t off, size_t len, size_t* retlen, u_char* buf)
{
        struct mtd_oob_ops ops = {
                .datbuf = buf,
                .len = len,
        };
        int err;
        int i;

        TRACE(KERN_DEBUG, "off=0x%08llX, len=%d\n", off, len);

        if(buf == 0)
                return -EINVAL;

        err = mnand_read_oob(mtd, off, &ops);
        *retlen = ops.retlen;
        return err;
}

static struct of_device_id of_platform_nand_table[];
static int of_mnand_probe(struct platform_device* ofdev)
{
        const struct of_device_id *match;
        struct device_node *of_node = ofdev->dev.of_node;
        const char *part_probes[] = { "cmdlinepart", NULL, };
        int err;

        match = of_match_device(of_platform_nand_table, &ofdev->dev);
        if (!match)
                return -EINVAL;

        memset(&g_chip, 0, sizeof(g_chip));

        g_chip.dev = ofdev;
        g_chip.active_page = -1;

        g_chip.regs.io = of_iomap(of_node, 0);
        if (WARN_ON(!g_chip.regs.io))
                return -EIO;

        g_chip.irq = irq_of_parse_and_map(of_node, 0);

        if((err=request_irq(g_chip.irq, interrupt_handler, 0, DRIVER_NAME, &g_chip))) {
                printk(KERN_ERR "Failed to set handler on irq #%d (code: %d)\n", g_chip.irq, err);
                goto error_irq;
        }

        g_chip.dev->dev.coherent_dma_mask=DMA_BIT_MASK(32);
        g_chip.dma_area = dma_alloc_coherent(
                                  &g_chip.dev->dev, 4096, &g_chip.dma_handle, GFP_KERNEL|GFP_DMA);

        if(!g_chip.dma_area) {
                printk(KERN_ERR "Failed to request DMA area\n");
                err = -ENOMEM;
                goto error_dma;
        }

        memset(g_chip.dma_area, 0xFF, 4096);

        sema_init(&g_mutex, 1);

        down(&g_mutex);

        mnand_hw_init();
        mnand_reset_grabber();
        mnand_core_reset();

        g_chip.mtd.name = "mnand";
        g_chip.mtd.size = 0;
        g_chip.chip_size[0] = 0;
        g_chip.chip_size[1] = 0;

        err = mnand_read_id(0);
        err = mnand_read_id(1);

        up(&g_mutex);

        g_chip.mtd.owner = THIS_MODULE;
        g_chip.mtd.type = MTD_NANDFLASH;
        g_chip.mtd.flags = MTD_WRITEABLE;
        g_chip.mtd._erase = mnand_erase;
        g_chip.mtd._read = mnand_read;
        g_chip.mtd._write = mnand_write;
        g_chip.mtd._block_isbad = mnand_isbad;
        g_chip.mtd._block_markbad = mnand_markbad;
        g_chip.mtd._read_oob = mnand_read_oob;
        g_chip.mtd._write_oob = mnand_write_oob;
        g_chip.mtd.ecclayout = &g_ecclayout;

        g_chip.mtd.dev.parent = &ofdev->dev;

        printk("mnand: Detected %llu bytes of NAND\n", g_chip.mtd.size);

        err = mtd_device_parse_register(&g_chip.mtd, part_probes, 0, NULL, 0);
        if (err) {
                printk(KERN_ERR "Failed add mtd device (code: %d)\n", err);
                goto error_dma;
        }
        return 0;

error_dma:
        dma_free_coherent(&g_chip.dev->dev, 4096, g_chip.dma_area, g_chip.dma_handle);
error_irq:
        free_irq(g_chip.irq, &g_chip);
        g_chip.irq = 0;
        return err;
}

static int of_mnand_remove(struct platform_device* ofdev)
{
        mtd_device_unregister(&g_chip.mtd);
        dma_free_coherent(&g_chip.dev->dev, 4096, g_chip.dma_area, g_chip.dma_handle);
        free_irq(g_chip.irq, &g_chip);
        release_mem_resource(&g_chip.regs);
        return 0;
}

static struct of_device_id of_platform_nand_table[] = {
        { .compatible = "module,mnand" },
        { /* end of list */ },
};

static struct platform_driver of_platform_mnand_driver = {
        .driver = {
                .name = DRIVER_NAME,
                .owner = THIS_MODULE,
                .of_match_table = of_platform_nand_table,
        },
        .probe = of_mnand_probe,
        .remove = of_mnand_remove
};

module_platform_driver(of_platform_mnand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kirill Mikhailov kmikhailov@module.ru");
MODULE_DESCRIPTION("RC Module NAND Controller Driver");

