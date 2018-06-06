/*
 * KeyASIC MMC/SD/SDIO driver
 * Designed for RC Module's Flavor of KeyASIC's MMC/SDIO and 
 * relies on a few extra regs added by RC Module to simplify things
 *
 * Authors: Andrew Andrianov
 * Copyright (C) 2012 RC Module.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.ь
 */


/*
 * Misc hardware notes
 * --------------------  
 * 
 * It looks like SDIO IP uses _CARD_ clock to do DMA from pp-buffers
 * to AXI. This is pretty lame, so we use minimum burst sizes to 
 * minimize the impact of this 'design solution' on the AXI.
 * 
 * Still, a shitty card working at 400kHz CAN theoretically screw things up. 
 *
 * The hardware itself is very interrupt noisy. And we have to keep track of the 
 * different interrupt bits, since any one of them missing is most likely to
 * indicate possible data loss (sic!)
 * 
 * Nice people at RC Module created a few extra registers to help poor 
 * programers out. They are actually additional interrupt mask registers
 * and a set of interrupt status registers. 
 * Card clock is also set up via the 'extra' registers.
 * 
 * If hacking this driver, do read KeyAsic's PDFs with extra care. There are 
 * a few lies inside. If doubt - check comments in this file
 *
 */

//#define DEBUG

//#define DBG_IO
//#define DBG_VLOG
//#define DBG_DUMP_BUFFERS	
//#define DEBUG_ERRORBITS
//#define DBG_RESP
//#define DBG_CALLS
//#define DBG_CMD
//#define DBG_LINES

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mbus.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>

#include "rcmodule-sdio.h"

#define DRIVER_NAME	"rmsdio"

#define RMSDIO_CLKDIV_MAX  0xff
#define RMSDIO_TIMEOUT (2*HZ)

#define MATCH_BITS(v,bits) ((v & bits)==bits)

#define FPGA_DTRACE() { \
		printk(KERN_INFO "rmsdio: %s %s %s %s")			\
}
	
struct rmsdio_platform_data 
{
	int gpio_card_detect; //card detect
	int gpio_write_protect; // r/w gpio
	int clock; //Incoming clock rate
};
	
struct rmsdio_host {
	void __iomem *base;
	struct mmc_request *mrq;
	spinlock_t lock;
	unsigned long iflags; /* for spinlock save/restore */
	unsigned int xfer_mode;
	unsigned int intr_en;
	uint32_t ctrl_reg; /* ctrl value stored by set_ios. OR bits to it */
	uint32_t shedcmd; /* sheduled cmd, to be issued after dma */
	unsigned int sg_frags;
	unsigned int ns_per_clk;
	unsigned int clock;
	unsigned int busy; /* host busy flag */
	unsigned int base_clock;
	unsigned int iostate;
	struct mmc_host *mmc;
	struct device *dev;
	int next_buffer;
	dma_addr_t dma_target;
	char* cpu_target;
	int irqstat;
	wait_queue_head_t queue;
	struct resource *res;
	int dma_mode;
	int irq;
	int sdio_irq_enabled;
	int gpio_card_detect;
	int gpio_write_protect;
};


#define is_sdio_irq_enabled() (host->sdio_irq_enabled)

#define print_vlog(...) printk(KERN_INFO "vlog: " __VA_ARGS__);

#ifndef DBG_IO
/*
//#define rmsdio_write(offs, val)	{	\
//		writel(val, iobase + (offs));}				
*/

#define rmsdio_write(offs, val)	{				\
	writel(val, iobase + (offs));				\
	if ((offs == RMSDIO_ERR_ENABLE) && is_sdio_irq_enabled()) {	\
		if (!(val & RMSDIO_ERR_ENABLE_CARD_INT)) BUG();		\
	} }


#define rmsdio_read(offs)		readl(iobase + (offs))
#else
#define rmsdio_write(offs, val)						\
	print_vlog("rmsdio_write(32'h%02x, 32'h%08x);\n", offs, val);	\
	writel(val, iobase + (offs));

#define rmsdio_read(offs)		readl(iobase + (offs))
#endif

#ifdef DBG_CALLS
	#define MARK() printk(KERN_INFO "rmsdio: ==========> %s \n", __FUNCTION__)
	#define STEP(n) printk(KERN_INFO "   rmsdio: ======> %s(%d)\n", __FUNCTION__, n)
#else	
	#define MARK()
	#define STEP(n)
#endif

#define MAXREG 0x60 

#ifdef DBG_LINES
#define DUMP_LINES {							\
		printk(KERN_INFO "RMSDIO %d: lines %x\n", __LINE__, rmsdio_read(0x314)) ; \
	}
#else	
#define DUMP_LINES
#endif	

/*
static void dump_regs(void *dev)
{
        struct rmsdio_host *host = dev;
	void __iomem *iobase = host->base;
	int i;
	for (i=0; i<=0x60; i=i+4)
		dev_dbg(host->dev, "RMSDIO 0x%02x = 0x%08x\n",i,rmsdio_read(i));	
}
*/

/* Waits and cleans up irqstat */
#define rmsdio_wait_irq(w,q,c,t)	{			\
		w = wait_event_interruptible_timeout(		\
			q,c,t					\
			);					\
	}



#define ckint(bit,t) if (i & bit) {	\
		printk(KERN_INFO t);	\
	}

#ifdef DEBUG
static void decode_intr(struct rmsdio_host *h, char* text, uint32_t i)
{
	printk(KERN_INFO "rmsdio: %s ", text); 
	ckint(RMSDIO_IRQ_CH0_DONE, " ch0d ");
	ckint(RMSDIO_IRQ_CH1_DONE, " ch1d ");
	ckint(RMSDIO_IRQ_TRANFINISH, " tfinish ");
	ckint(RMSDIO_IRQ_DATABOUND, " databound ");
	ckint(RMSDIO_IRQ_TRANDONE, " trandone ");
	ckint(RMSDIO_IRQ_CMDDONE, " cmddone ");
	ckint(RMSDIO_IRQ_CARDERROR, " ce ");
	ckint(RMSDIO_IRQ_SDIO, " sdio ");
	printk(KERN_INFO "\n");
}
#else
#define decode_intr(h,t,i) 
#endif




//+

extern int _read_tlb_entry(uint32_t ea, uint32_t * tlb, uint32_t dummy);

typedef enum 
{
	TLBSID_4K   = 0x00,
	TLBSID_16K  = 0x01,
	TLBSID_64K  = 0x03,
	TLBSID_1M   = 0x07,
	TLBSID_16M  = 0x0F,
	TLBSID_256M = 0x1F,
	TLBSID_1G   = 0x3F,
	TLBSID_ERR  = 0xFF
} TLB_SIZE_ID;

////////////////////////////////////////////////////////////////

typedef struct 
{
	TLB_SIZE_ID sid;
	const char * name;
} tlb_sid_name;

static const tlb_sid_name tlb_sid_names[] = 
{
	{ TLBSID_4K,   "4k"   },
	{ TLBSID_16K,  "16k"  },
	{ TLBSID_64K,  "64k"  },
	{ TLBSID_1M,   "1m"   },
	{ TLBSID_16M,  "16m"  },
	{ TLBSID_256M, "256m" },
	{ TLBSID_1G,   "1g"   },
	{ TLBSID_ERR,  ""     }
};	

static const char * get_tlb_sid_name(TLB_SIZE_ID tlb_sid)
{
	uint i;
	for ( i = 0; tlb_sid_names[i].sid != TLBSID_ERR; ++ i ) 
		if ( tlb_sid_names[i].sid == tlb_sid )
			break;
	return tlb_sid_names[i].name;
}

////////////////////////////////////////////////////////////////

/* Current:
  epn  v ts  sz   _  rpn  _ erpn  __  ili ild u wimg  e  _ uxwr sxwr
 00000 1  0   1m  f 00000 2  010 2000  1   1  0   4  BE  0   0    7
 10000 1  0 256m  b 10000 0  010 0000  1   1  0   5  LE  0   0    3
 20000 1  0 256m  b 20000 0  010 0000  1   1  0   5  LE  0   0    3
 30000 1  0 256m  f 30000 2  010 0000  1   1  0   5  LE  0   0    3
 40000 1  0 256m  3 00000 3  000 0000  1   1  0   4  BE  0   7    7
 fff00 1  0   1m  f fff00 3  3ff 2000  1   1  0   5  BE  0   0    5
*/

static void print_tlb_hdr(void)
{
	printk(KERN_INFO "  epn  v ts  sz   _" "  rpn  _ erpn" "  __  ili ild u wimg  e  _ uxwr sxwr");
}

typedef struct 
{
  uint32_t epn   : 20;  // [0:19]
  uint32_t v     : 1;   // [20]
  uint32_t ts    : 1;   // [21]
  uint32_t dsiz  : 6;   // [22:27]  set 0x0, 0x1, 0x3, 0x7, 0xF, 0x1F, 0x3F (4K, 16K, 64K, 1M, 16M, 256M, 1GB) 
  uint32_t blank : 4;   // [28:31]
} tlb_entr_0;

typedef union
{
	tlb_entr_0 entr;
	uint32_t   data;
} tlb_entr_data_0;

typedef struct 
{
  uint32_t rpn   : 20;  // [0:19]
  uint32_t blank : 2;   // [20:21]
  uint32_t erpn  : 10;  // [22:31]
} tlb_entr_1;

typedef union
{
	tlb_entr_1 entr;
	uint32_t   data;
} tlb_entr_data_1;

typedef struct 
{
  uint32_t blank1 : 14;  // [0:13]
  uint32_t il1i   : 1;   // [14]
  uint32_t il1d   : 1;   // [15]
  uint32_t u      : 4;   // [16:19]
  uint32_t wimg   : 4;   // [20:23]
  uint32_t e      : 1;   // [24]  0-BE, 1-LE
  uint32_t blank2 : 1;   // [25]
  uint32_t uxwr   : 3;   // [26:28]
  uint32_t sxwr   : 3;   // [29:31]
} tlb_entr_2;

typedef union
{
	tlb_entr_2 entr;
	uint32_t   data;
} tlb_entr_data_2;

static void print_tlb_entr(tlb_entr_0 * tlb0, tlb_entr_1 * tlb1, tlb_entr_2 * tlb2)
{
	printk(KERN_INFO " %05x %d  %d %4s  %01x" " %05x %01x  %03x" " %04x  %d   %d  %01x   %01x  %s  %d   %01x    %01x", 
	                  tlb0->epn, tlb0->v, tlb0->ts, get_tlb_sid_name((TLB_SIZE_ID) tlb0->dsiz), tlb0->blank, 
										tlb1->rpn, tlb1->blank, tlb1->erpn, 
										tlb2->blank1, tlb2->il1i, tlb2->il1d, tlb2->u, tlb2->wimg, (tlb2->e ? "LE" : "BE"), tlb2->blank2, tlb2->uxwr, tlb2->sxwr);
}

typedef struct 
{
	uint32_t tlb_0;
	uint32_t tlb_1;
	uint32_t tlb_2;
} tlb_entr;

static tlb_entr s_tlb_arr[1024];
static int s_tlb_cnt = -1;

#define countof(arr)  (sizeof(arr) / sizeof((arr)[0]))

int mem_map_tlbs(bool print_all) 
{
	uint32_t ea;
	const uint32_t dlt = 0x1 << 12;
	
	print_tlb_hdr();
	printk(KERN_INFO "\n");
	
	s_tlb_cnt = -1;
	for ( ea = 0x00000000; ea + dlt > ea; ea += dlt ) {
		uint32_t tlb[3] = { 0, 0, 0 };	
		uint32_t res = _read_tlb_entry(ea, tlb, 0);
		if ( res != 0 && (s_tlb_cnt == -1 || s_tlb_arr[s_tlb_cnt].tlb_0 != tlb[0]) ) {
			if ( ++ s_tlb_cnt == countof(s_tlb_arr) )
				return -1;
			s_tlb_arr[s_tlb_cnt].tlb_0 = tlb[0];
			s_tlb_arr[s_tlb_cnt].tlb_1 = tlb[1];
			s_tlb_arr[s_tlb_cnt].tlb_2 = tlb[2];
			if ( print_all || ((tlb_entr_1 *) & tlb[1])->erpn != 0 ) 
				print_tlb_entr((tlb_entr_0 *) & tlb[0], (tlb_entr_1 *) & tlb[1], (tlb_entr_2 *) & tlb[2]);
		}			
	}
	return 0;
}

//-


/*
  static u32 rmsdio_finish_data(struct rmsdio_host *host, struct mmc_data *data,
  u32 err_status);
*/

static inline void rmsdio_dma_xfer(struct rmsdio_host *host, 
				   int dir, int buf,  
				   dma_addr_t addr, size_t sz)
{	
	void __iomem *iobase = host->base;
	uint32_t channel, areg, tctl, tmpl;
	
	MARK();

	dev_dbg(host->dev, "dma %d bytes: 0x%08x %s buf%d\n", 
		sz, (uint) addr, (dir == DMA_FROM_DEVICE) ? "<=" : "=>", buf );  // !!!
	/* channel 0 can only do 'to-device', whilst ch1 can only 'from-device' */
	BUG_ON(buf>1);
	channel = (dir == DMA_TO_DEVICE) ? RMSDIO_DMA_CH0 : RMSDIO_DMA_CH1;
	areg = (dir == DMA_TO_DEVICE) ? RMSDIO_DCSSAR : RMSDIO_DCDSAR;
	tctl = 0x4 | buf;
	tmpl = (dir == DMA_FROM_DEVICE) ? RMSDIO_DCCR_TEMPLATE_R : RMSDIO_DCCR_TEMPLATE_W; 
	tctl |= (dir == DMA_TO_DEVICE) ? 2 : 0;
	rmsdio_write(RMSDIO_TRAN_CTL, tctl);
	rmsdio_write(channel + RMSDIO_DCDTR,  sz);
	rmsdio_write(channel + areg, (uint) addr);  // !!!
	rmsdio_write(channel + RMSDIO_DCCR, (tmpl | 0x1));
}

  
/** 
 * Get a suitable bit value that corrsponds to the 
 *
 * @param data mmc_data struct 
 *
 * @return bitvalue
 */

static inline uint32_t get_block_sz_bits(struct mmc_data *data)
{
	uint32_t bits=0;

	MARK();

	switch(data->blksz)
	{
	case SZ_2K:
		bits=RMSDIO_BLOCK_SET_2048;
		break;
	case SZ_1K:
		bits=RMSDIO_BLOCK_SET_1024;
		break;
	case 512:
		bits=RMSDIO_BLOCK_SET_512;
		break;
	default:
		BUG_ON(bits > 512);
		bits=RMSDIO_BLOCK_SET_CBLSR;
		bits|=(data->blksz << 16);
		break;
	}
	return bits;
}

static void rmsdio_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct rmsdio_host *host = mmc_priv(mmc);
	void __iomem *iobase = host->base;
	uint32_t eereg;
	uint32_t wv;
	
	MARK();

	spin_lock_irqsave(&host->lock, host->iflags);
	dev_dbg(host->dev, " %s sdio card interrupt\n", enable ? "enabling" : "disabling");
	eereg = rmsdio_read(RMSDIO_ERR_ENABLE);
	wv = rmsdio_read(RMSDIO_IRQ_MASKS);

	if (enable) {
		eereg |= RMSDIO_ERR_ENABLE_CARD_INT;
		eereg |= RMSDIO_ERR_ENABLE_CTRL_INT;
		wv |= RMSDIO_IRQ_SDIO;
	} else {
		if (!is_sdio_irq_enabled()) {
			printk(KERN_INFO " is_sdio_irq_enabled BUG!!!\n"); 
			BUG();
		}
		eereg &= ~RMSDIO_ERR_ENABLE_CARD_INT;
		wv &= ~RMSDIO_IRQ_SDIO;
		eereg |= RMSDIO_ERR_ENABLE_CTRL_INT;
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x20);
	}

	host->sdio_irq_enabled = enable;
	rmsdio_write(RMSDIO_IRQ_MASKS, wv);
	rmsdio_write(RMSDIO_ERR_ENABLE, eereg);
	spin_unlock_irqrestore(&host->lock, host->iflags);

}


/** 
 * Cleans up pending isr bits in sdio registers
 * Ignores bits specified in ignore_bits
 * Silicon weirdness: cleaning must be done one bit
 * at a time
 * @param host rmsdio host to use
 * @param ignore_bits bits to ignore
 */

void rmsdio_clean_isr(struct rmsdio_host *host, int ignore_bits) 
{
	void __iomem *iobase = host->base;
	//uint32_t breg=0;
	uint32_t rreg = host->irqstat & (~ignore_bits);

	/* CH1 must be cleaned before TFINISH || we're screwed */
	if (rreg & RMSDIO_IRQ_CH1_DONE)
		rmsdio_write(RMSDIO_DMA_CH1+RMSDIO_DCCR, 0x80000);

	if (rreg & RMSDIO_IRQ_CMDDONE)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x4);
	
	if (rreg & RMSDIO_IRQ_TRANDONE)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x2);
	
	if (rreg & RMSDIO_IRQ_TRANFINISH)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x1);
	
	if (rreg & RMSDIO_IRQ_CARDERROR)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x10);
	
	if (rreg & RMSDIO_IRQ_DATABOUND)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x8);

	if (rreg & RMSDIO_IRQ_SDIO)
		rmsdio_write(RMSDIO_BUF_TRAN_RESP_REG, 0x20);
	
	if (rreg & RMSDIO_IRQ_CH0_DONE)
		rmsdio_write(RMSDIO_DMA_CH0+RMSDIO_DCCR, 0x80000);

	/* Remove pending bits from irqstat */
	host->irqstat &= ~rreg;	
}

static inline void rmsdio_setup_data(struct rmsdio_host *host, struct mmc_data *data)
{
	void __iomem *iobase = host->base;
	uint32_t blkbits;
	
	MARK();

	dev_dbg(host->dev, "data: %d blocks %d bytes each\n", data->blocks, data->blksz);
	
		
	BUG_ON(data->blocks > 255);
	BUG_ON(data->blksz > 512); 

	blkbits = get_block_sz_bits(data);
	blkbits |= data->blocks;

	rmsdio_write(RMSDIO_BLOCK_SET, blkbits);
	rmsdio_write(RMSDIO_ADDRESS, host->next_buffer);
	dev_dbg(host->dev, "using buffer %d\n", host->next_buffer);
	dev_dbg(host->dev, "block set reg is now 0x%04x\n", blkbits);
}


static u32 rmsdio_finish_cmd(struct rmsdio_host *host, struct mmc_command *cmd,
			     u32 err_status)
{
	void __iomem *iobase = host->base;
	//uint32_t eereg;
	
	MARK();
	
	if (cmd->data) dev_dbg(host->dev, "==============DATA=================\n");	
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* That one caused a lot of pain. Damn it! */
			/* KeyASIC delivers 'em reversed */
			cmd->resp[3] = rmsdio_read(RMSDIO_RESPONSE1);
			cmd->resp[2] = rmsdio_read(RMSDIO_RESPONSE2);
			cmd->resp[1] = rmsdio_read(RMSDIO_RESPONSE3);
			cmd->resp[0] = rmsdio_read(RMSDIO_RESPONSE4);
		} else {
			cmd->resp[0] = rmsdio_read(RMSDIO_RESPONSE1);
			cmd->resp[1] = rmsdio_read(RMSDIO_RESPONSE2);
			cmd->resp[2] = rmsdio_read(RMSDIO_RESPONSE3);
			cmd->resp[3] = rmsdio_read(RMSDIO_RESPONSE4);
		}
	}
#ifdef DBG_RESP
	dev_dbg(host->dev, "r1 0x%x\n",cmd->resp[0]);	
	dev_dbg(host->dev, "r2 0x%x\n",cmd->resp[1]);
	dev_dbg(host->dev, "r3 0x%x\n",cmd->resp[2]);
	dev_dbg(host->dev, "r4 0x%x\n",cmd->resp[3]);
#endif
	/* reset any err bits */
//	rmsdio_write(RMSDIO_STATUS, (err_status & (~RMSDIO_STATUS_ERRMASK)));
	
#ifdef DEBUG_ERRORBITS
	dev_dbg(host->dev, "--- error bits present ---\n");
	if (err_status & RMSDIO_STATUS_CMDTO)
		dev_dbg(host->dev, "error: command timeout\n");
	if (err_status & RMSDIO_STATUS_CMDCRC)
		dev_dbg(host->dev, "error: command crc7 failed\n");
	if (err_status & RMSDIO_STATUS_CMDEB)
		dev_dbg(host->dev, "error: command end bit error\n");
	if (err_status & RMSDIO_STATUS_CMDIDX)
		dev_dbg(host->dev, "error: command idx error\n");
	if (err_status & RMSDIO_STATUS_DATCRC)
		dev_dbg(host->dev, "error: data crc error\n");
	if (err_status & RMSDIO_STATUS_DATEB)
		dev_dbg(host->dev, "error: data endbit error\n");
	dev_dbg(host->dev, "--- error bits present ---\n");
#endif
	
	
	/* Detect if someone removed the card */
	/* We should't do that if data present */
	/*
	  if (!(cmd->data) && !(err_status & RMSDIO_STATUS_CARDIN))
	  {
	  dev_dbg(host->dev, "who stole the card? \n");
	  mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	  cmd->error=-EIO;
	  } else */

	if (err_status & RMSDIO_STATUS_CMDTO) {
		dev_dbg(host->dev, "error: command timeout\n");
		cmd->error = -ETIMEDOUT;
	} else if (err_status & RMSDIO_STATUS_ERRMASK)
	{
		dev_dbg(host->dev, "error: errors present: 0x%04x\n",err_status);
		cmd->error = -EILSEQ;
	} else
		cmd->error = 0; 


		return 0;
}

/* 
 * FixMe: This one works, but ended up looking really ugly 
 */
static inline int rmsdio_write_flow(struct mmc_host *mmc, struct mmc_request *mrq, u32 ctrlreg)
{
	struct rmsdio_host *host = mmc_priv(mmc);
	void __iomem *iobase = host->base;
	//struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	//u32 intr = 0;
	u32 wintr = 0;
	int wret;
	int blocks = 0;

	MARK();

	/* Send first chunk of data into the pp-buffer */
	wintr = RMSDIO_IRQ_CH0_DONE | RMSDIO_IRQ_TRANFINISH;
	rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
	rmsdio_dma_xfer(host, DMA_TO_DEVICE, 0, host->dma_target, data->blksz);

	rmsdio_wait_irq(
		wret, host->queue,
		(MATCH_BITS(host->irqstat, 
			    (wintr))),
		RMSDIO_TIMEOUT
		);

	rmsdio_clean_isr(host, 0);

	if ( wret<= 0 )
		goto bailout;
	
	if (data->blocks!=1)
		wintr = RMSDIO_IRQ_DATABOUND | RMSDIO_IRQ_CMDDONE;
	else
		wintr = RMSDIO_IRQ_TRANDONE | RMSDIO_IRQ_CMDDONE;
	
	/* 
	 * First block is transfered via writing RMSDIO_CTRL,
	 * subsequent via setting ne
	 */
	rmsdio_write(RMSDIO_CTRL, ctrlreg);

	data->bytes_xfered += data->blksz;
	host->cpu_target += data->blksz;
	host->dma_target += data->blksz;
	host->next_buffer = !host->next_buffer;
	
	do {
		blocks++;	
		if ((blocks > 1) && (blocks <= data->blocks)) {
			dev_dbg(host->dev, "block transfer: buf%d -> card\n", 
				host->next_buffer);
			dev_dbg(host->dev, "block %d/%d\n", 
				blocks, data->blocks);

			rmsdio_write(RMSDIO_ADDRESS, host->next_buffer<<2 );
			host->next_buffer = !host->next_buffer;

			if ( blocks == data->blocks )  
				wintr |= RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_TRANDONE;
			else
				wintr |= RMSDIO_IRQ_DATABOUND;
		}
		
		/* Start filling up next buffer, if needed */
		if (blocks < data->blocks ) {
			wintr |= RMSDIO_IRQ_CH0_DONE | RMSDIO_IRQ_TRANFINISH;
			rmsdio_dma_xfer(host, DMA_TO_DEVICE, host->next_buffer, host->dma_target, data->blksz);
			data->bytes_xfered += data->blksz;
			host->cpu_target += data->blksz;
			host->dma_target += data->blksz;
		}

		rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
		rmsdio_wait_irq(
			wret, host->queue,
			(MATCH_BITS(host->irqstat, 
				    (wintr))),
			RMSDIO_TIMEOUT
			);

		if (wret<=0)
			goto bailout;

		rmsdio_clean_isr(host, 0);
		wintr = 0;
	} while (blocks <= data->blocks);
	
	return wret;
bailout:
        printk(KERN_INFO "rmsdio: TIMEOUT!\n");
	dev_err(host->dev, "Timed out waiting for interrupt bits\n");
	decode_intr(host," flags want: ", wintr);
	decode_intr(host," flags have: ", host->irqstat);
	return wret;
}

static inline int rmsdio_read_flow(struct mmc_host *mmc, struct mmc_request *mrq, u32 ctrlreg)
{
	struct rmsdio_host *host = mmc_priv(mmc);
	void __iomem *iobase = host->base;
	//struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	u32 wintr = 0;
	int wret;
	int blocks = 0;

	MARK();
	
	ctrlreg |= RMSDIO_CTRL_DATAREAD;
	
	if (data->blocks != 1) 
		wintr = RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_DATABOUND;
	else 
		wintr = RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_TRANDONE;

	BUG_ON(host->irqstat);
	rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
	rmsdio_write(RMSDIO_CTRL, ctrlreg);
	/* 
	 * Okay, first block is now on the way pp-buffer, we're good to loop
	 * Starting with an isr wait
	 */
	do {
		blocks++;
		//rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
		/* 
		 * Only allow context switch here, while waiting. 
		 */
		rmsdio_wait_irq(wret, host->queue,
				(MATCH_BITS(host->irqstat, 
					    (wintr))),
				RMSDIO_TIMEOUT
			);
		if (wret <= 0)
			goto bailout;

		/* DO clean everything, see note below */
		rmsdio_clean_isr(host, 0);
		wintr = 0; /* Reset isr wait flags */
		
		if (blocks <= data->blocks) {
			wintr |= RMSDIO_IRQ_CH1_DONE | RMSDIO_IRQ_TRANFINISH;
			rmsdio_dma_xfer(host, DMA_FROM_DEVICE, host->next_buffer, host->dma_target, data->blksz);
			host->next_buffer = ! host->next_buffer;
			data->bytes_xfered += data->blksz;
			host->cpu_target += data->blksz;
			host->dma_target += data->blksz;
		} else
			break;
		
		if (blocks < data->blocks) {
			/* 
			 * Okay, this hardware is really retarted
			 * I admit that. The last block from card
			 * should give us some different flags
			 */
			
			if ( blocks == data->blocks - 1 ) 
				wintr |= RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_TRANDONE;
			else
				wintr |= RMSDIO_IRQ_DATABOUND;
		
			/* 
			 * NOTE TO SELF: DATASHEET LIES! 
			 * Writing RMSDIO_ADDR actually starts
			 * next transfer, not the DATABOUND flag cleanup
			 * If we follow the flow in the spec, sheduler
			 * MIGHT kick in, and delay us, so that we 
			 * clean up the newly arrived databound flag
			 * This way we can safely avoid spinlock_irqsave
			 * Actually, turning on reg tracing is enough to
			 * cause a nice race.
			 */

			/* rmsdio_clean_isr(host, ~RMSDIO_IRQ_DATABOUND); */
			rmsdio_write(RMSDIO_ADDRESS, host->next_buffer << 2 );
			rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
		}
		rmsdio_write(RMSDIO_IRQ_MASKS, wintr);
	} while (1);
	return wret;
bailout:
/* Clean up anything remaining */
        printk(KERN_INFO "TIMEOUT!!!\n");
	dev_err(host->dev, "Timed out waiting for interrupt bits\n");
	decode_intr(host," flags want: ", wintr);
	decode_intr(host," flags have: ", host->irqstat);
	return wret;
	
}

static void rmsdio_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct rmsdio_host *host = mmc_priv(mmc);
	void __iomem *iobase = host->base;
	struct mmc_command *cmd = mrq->cmd;
	u32 ctrlreg = 0, intr = 0, err_status = 0;
	int wret;//, dir;
	struct mmc_data *data = mrq->data;
	uint32_t eereg;
	
	MARK();
#ifdef DBG_CMD	
	printk(KERN_INFO " >CMD: cmd=%d, resp= 0x%x, arg=0x%x %s", cmd->opcode, mmc_resp_type(cmd), cmd->arg, (data ? "[DATA]" : ""));
#endif	
	
	/* Since we can't queue up cmds, we can only have one request ongoing */
	spin_lock_irqsave(&host->lock, host->iflags);
	if (host->busy) {
		spin_unlock_irqrestore(&host->lock, host->iflags);
		mrq->cmd->error = -EAGAIN;
		mmc_request_done(mmc, mrq);
		return;
	}
	host->busy++;
	spin_unlock_irqrestore(&host->lock, host->iflags);

	eereg = rmsdio_read(RMSDIO_ERR_ENABLE) & 
		(RMSDIO_ERR_ENABLE_CTRL_INT);
	BUG_ON(host->mrq != NULL);
	host->irqstat = 0; /* reset flags */
        host->shedcmd = 0;
	host->mrq = mrq;
	ctrlreg = host->ctrl_reg |  RMSDIO_CMD_IDX(cmd->opcode);
	if (mmc_resp_type(cmd) == MMC_RSP_R2)
	{
		ctrlreg|=RMSDIO_CTRL_RESPONSE_R2;
		dev_dbg(host->dev, "Expecting R2 response\n");
	} else if (mmc_resp_type(cmd) == MMC_RSP_R1B)
	{
		ctrlreg|=RMSDIO_CTRL_RESPONSE_R1B;
		dev_dbg(host->dev, "Expecting R1B response\n");
	} else if (mmc_resp_type(cmd) & MMC_RSP_PRESENT)
	{
		dev_dbg(host->dev, "Expecting R1367 response\n");
		ctrlreg|=RMSDIO_CTRL_RESPONSE_R1367;
	}
	
	if (cmd->flags & MMC_RSP_CRC)
	{
		ctrlreg |= RMSDIO_CTRL_CMDCRC;
		eereg |= RMSDIO_ERR_CMDCRC;
	}
	        
	if (cmd->flags & MMC_RSP_OPCODE)
	{
		ctrlreg |= RMSDIO_CTRL_IDXCHK;
		eereg|=RMSDIO_ERR_CMDIDX;
	}

	host->next_buffer=0;

	eereg |= RMSDIO_ERR_ENABLE_CTRL_INT;
	eereg |= RMSDIO_ERR_CMDEB | RMSDIO_ERR_CMDTO; 
	rmsdio_write(RMSDIO_CMD_ARGUMENT,cmd->arg);
	
	if (is_sdio_irq_enabled())
		eereg |= RMSDIO_ERR_ENABLE_CARD_INT; 

	if (mrq->data && cmd->opcode != 51)  // !!!
	{
		int dir = (data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
		/* Prepare dma stuff */
		host->sg_frags = dma_map_sg(mmc_dev(host->mmc), data->sg,
					    data->sg_len, dir);
		host->dma_target = sg_dma_address(data->sg);
		host->cpu_target = sg_virt(data->sg);
		/* Do all the register voodoo */
		ctrlreg |= RMSDIO_CTRL_HAVEDATA;
		rmsdio_setup_data(host, data);

		if (data->stop)
			ctrlreg |= RMSDIO_CTRL_AUTOCMD12 ;

		dev_dbg(host->dev, "i/o - we're planning to: %s %s\n", 
			(data->flags & MMC_DATA_WRITE) ? "write" : "read",
			(data->stop) ? "autocmd12" : ""
			);
		
		eereg |= RMSDIO_ERR_DATEB | RMSDIO_ERR_DATCRC;
		rmsdio_write(RMSDIO_ERR_ENABLE, eereg);
		
		/* Get stuff ready for DMA */		
		host->sg_frags = dma_map_sg(mmc_dev(host->mmc), data->sg,
					    data->sg_len, dir);
		host->dma_target = sg_dma_address(data->sg);
		host->cpu_target = sg_virt(data->sg);

		
		if (data->flags & MMC_DATA_WRITE) 
			wret = rmsdio_write_flow(mmc, mrq, ctrlreg);
		else
			wret = rmsdio_read_flow(mmc, mrq, ctrlreg);

		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->sg_frags,
			     dir);

		if (wret<=0)
			goto error;
		
		goto finish_rq;
	}

	if (mrq->data && cmd->opcode == 51) {
		static uint8_t buf[8] = { 0x02, 0x35, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
		memcpy(sg_virt(data->sg), buf, sizeof(buf));
	}

	/* We only reach this place when no xfer data issued */
	rmsdio_write(RMSDIO_ERR_ENABLE, eereg);	
	intr |= RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_CARDERROR;
	rmsdio_write(RMSDIO_IRQ_MASKS, intr);
	rmsdio_write(RMSDIO_CTRL, ctrlreg);

	/* cmd done or card error. either of those indicate we're done here */ 
	rmsdio_wait_irq(wret, host->queue, 
			(host->irqstat & (RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_CARDERROR)),
			RMSDIO_TIMEOUT
		);
	
	rmsdio_clean_isr(host, 0);
	
	if ( wret<= 0 )
		goto error;
	
	goto finish_rq;

error:
	cmd->error = -EIO;
finish_rq:

	err_status = rmsdio_read(RMSDIO_STATUS) & RMSDIO_STATUS_ERRMASK;
	/* This should handle error statuses, if any */
	spin_lock_irqsave(&host->lock, host->iflags);
	rmsdio_finish_cmd(host, cmd, err_status);
	host->mrq = NULL;
	host->busy = 0;
	mmc_request_done(host->mmc, mrq);
	if (is_sdio_irq_enabled()) 
		rmsdio_enable_sdio_irq(mmc,1);
	spin_unlock_irqrestore(&host->lock, host->iflags);
	DUMP_LINES
}

static irqreturn_t rmsdio_irq(int irq, void *dev)
{
	struct rmsdio_host *host = dev;
	void __iomem *iobase = host->base;
	u32 wv, tr_resp; 
	int irqstat = rmsdio_read(RMSDIO_IRQ_STATUS);
	
	MARK();

	host->irqstat |= (irqstat & (~RMSDIO_IRQ_SDIO));
	decode_intr(host, "IRQ! new irqstat: ", host->irqstat);
	wv = rmsdio_read(RMSDIO_IRQ_MASKS);
	/* We need to read this one at least once, even 
	 * if we do not use it.  
	 */
	tr_resp = rmsdio_read(RMSDIO_BUF_TRAN_RESP_REG);

	/* 
	 * Disable interrupts that we've collected so far.
	 * except for sdio isr. We handle this on in interrupt context
	 * completely.
	 */

        /* SDIO ISR Handling */ 
	if (irqstat & RMSDIO_IRQ_SDIO) {
		DUMP_LINES
		printk(KERN_INFO "<====SDIO IRQ ARRIVED====>\n");
		mmc_signal_sdio_irq(host->mmc);
		
	}	
	wv &= ~irqstat; 
	rmsdio_write(RMSDIO_IRQ_MASKS, wv);
	/* 
	 * wake up the state machine 
	 */
	wake_up_interruptible(&host->queue);
	
	printk(KERN_INFO "<====SDIO IRQ HANDLED====>\n");
	
	return IRQ_HANDLED;
}

static irqreturn_t rmsdio_card_detect_irq(int irq, void *dev)
{
	struct rmsdio_host *host = dev;

	MARK();

	pr_debug("rmsdio: === Card Detect Interrupt arrived ===\n");
	mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}


static int rmsdio_get_ro(struct mmc_host *mmc)
{
	MARK();

	return -ENOSYS;
}

static void rmsdio_power_up(struct rmsdio_host *host)
{
	void __iomem *iobase = host->base;	
	dev_dbg(host->dev, "powering up\n");
	rmsdio_write(RMSDIO_ENABLE, RMSDIO_ENABLE_MODE_SDIO);
	/* Do a hard reset here */
	rmsdio_write(RMSDIO_CTRL, RMSDIO_CTRL_CRSR_HR);
}

static void rmsdio_power_down(struct rmsdio_host *host)
{
	void __iomem *iobase = host->base;
	dev_dbg(host->dev, "power down\n");
	/* In SPI mode nearly no logic gets clocked 
	 * So it's the best way to turn things off
	 */
	rmsdio_write(RMSDIO_ENABLE, RMSDIO_ENABLE_MODE_SPI);
}

static void rmsdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct rmsdio_host *host = mmc_priv(mmc);
	void __iomem *iobase = host->base;
	u32 ctrl_reg = 0;

	MARK();

	dev_dbg(host->dev, "rmsdio: setting i/o params\n");
	dev_dbg(host->dev, "pmode: %d\n", ios->power_mode);
	dev_dbg(host->dev, "clock: %d\n", ios->clock);
	dev_dbg(host->dev, "bw: %d\n", ios->bus_width);

	rmsdio_write(RMSDIO_CTRL, RMSDIO_CTRL_CRSR_HR);
	if (ios->power_mode == MMC_POWER_UP)
		rmsdio_power_up(host);
	if (ios->clock == 0) {
		rmsdio_write(RMSDIO_CLKDIV,RMSDIO_CLKDIV_MAX);
		host->clock=0;
		dev_dbg(host->dev, "clock somewhat disabled\n");
	} else if (ios->clock != host->clock)
	{
		uint32_t m = DIV_ROUND_UP(host->base_clock, ios->clock) - 1;
		if (m > RMSDIO_CLKDIV_MAX)
			m = RMSDIO_CLKDIV_MAX;
		//m = 0x18;
		rmsdio_write(RMSDIO_CLKDIV, m);
		host->clock = ios->clock;
		host->ns_per_clk = 1000000000 / (host->base_clock / (m+1));
		dev_dbg(host->dev, "clock=%d (%d), div=0x%04x\n",
			ios->clock, host->base_clock / (m+1), m);
	}
	ctrl_reg = rmsdio_read(RMSDIO_CTRL);
	ctrl_reg &= ~(RMSDIO_CTRL_CRSR_BUSMASK) ;
	//ctrl_reg |=  RMSDIO_CTRL_CRSR_NO_FDRIVE ;
	switch (ios->bus_width)
	{	
	case MMC_BUS_WIDTH_1:
		ctrl_reg |= RMSDIO_CTRL_CRSR_BUS1;
		break;
	case MMC_BUS_WIDTH_8:
		ctrl_reg |= RMSDIO_CTRL_CRSR_BUS8;
		break;
	case MMC_BUS_WIDTH_4:
		/* Intentional fall-through */
	default:
		ctrl_reg |= RMSDIO_CTRL_CRSR_BUS4;
		break;
	};
	
	/* TODO: Force drive mode? Move to module params? */
	host->ctrl_reg = ctrl_reg;
	dev_dbg(host->dev, "ctrl is now 0x%04x \n", ctrl_reg);
	if (ios->power_mode == MMC_POWER_OFF)
		rmsdio_power_down(host);
}

static const struct mmc_host_ops rmsdio_ops = {
	.request		= rmsdio_request,
	.get_ro			= rmsdio_get_ro,
	.set_ios		= rmsdio_set_ios,
	.enable_sdio_irq	= rmsdio_enable_sdio_irq,
};

static struct rmsdio_platform_data s_rmsd_data =
{
	.gpio_card_detect = 0x3C03C000,
	.gpio_write_protect = 0,
	.clock = 50000000
};

static int __init rmsdio_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc = NULL;
	struct rmsdio_host *host = NULL;
	const struct rmsdio_platform_data *rmsd_data;
	struct resource *r;
	int ret, irq;
	
	printk(KERN_INFO "rmsdio: RC Module SD/SDIO/MMC driver (c) 2012\n");
	
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	rmsd_data = pdev->dev.platform_data;
	
	if ( ! rmsd_data )  // !!!
		rmsd_data = pdev->dev.platform_data = & s_rmsd_data;
	
	if (!r || irq < 0 || !rmsd_data)
		return -ENXIO;

	STEP(1);

	r = request_mem_region(r->start, SZ_1K, DRIVER_NAME);
	if (!r)
		return -EBUSY;

	STEP(2);

	mmc = mmc_alloc_host(sizeof(struct rmsdio_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	STEP(3);

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = &pdev->dev;
	host->res = r;
	host->base_clock = rmsd_data->clock / 2;
	mmc->ops = &rmsdio_ops;
	/* TODO: More caps & ocr_avail */

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;

	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0xff; 

	mmc->max_segs = 1;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	mmc->f_min = DIV_ROUND_UP(host->base_clock, RMSDIO_CLKDIV_MAX);
	mmc->f_max = host->base_clock;
	pr_debug("rmsdio: maximum clock is %d Hz minimum is %d Hz\n", mmc->f_max, mmc->f_min);
	host->sdio_irq_enabled = 0;
	spin_lock_init(&host->lock);	
	host->base = ioremap(r->start, SZ_4K);
	printk(KERN_INFO "rmsdio start is 0x%Lx, base is 0x%x\n", r->start, (uint) host->base);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

	STEP(4);

	rmsdio_power_down(host);
	ret = request_irq(irq, rmsdio_irq, 0, DRIVER_NAME, host);
	if (ret) {
		pr_err("%s: cannot assign irq %d\n", DRIVER_NAME, irq);
		goto out;
	}

	STEP(5);

	host->irq = irq;
	pr_debug("rmsdio: Using irq %d\n", irq);

	if (rmsd_data->gpio_card_detect) {
		ret = gpio_request(rmsd_data->gpio_card_detect, "carddetect-gpio"/*DRIVER_NAME " cd"*/);
		// !!! ret = gpio_request_by_name(dev, "carddetect-gpio", 0, & rmsd_data->gpio_card_detect, GPIOD_IS_IN);
		pr_debug("rmsdio: gpio_card_detect: gpio_request: ret=%d\n", ret);
		if (ret == 0) {
			gpio_direction_input(rmsd_data->gpio_card_detect);
			irq = gpio_to_irq(rmsd_data->gpio_card_detect);
			ret = request_irq(irq, rmsdio_card_detect_irq,
					  IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING,
					  DRIVER_NAME " cd", host);
			pr_debug("rmsdio: gpio_card_detect: request_irq: ret=%d\n", ret);
			if (ret == 0)
				host->gpio_card_detect =
					rmsd_data->gpio_card_detect;
			else
				gpio_free(rmsd_data->gpio_card_detect);
		}
	}
	/*
	   if (!host->gpio_card_detect) 
	   mmc->caps |= MMC_CAP_NEEDS_POLL;
	*/

	STEP(6);

	if (rmsd_data->gpio_write_protect) {
		ret = gpio_request(rmsd_data->gpio_write_protect,
				   DRIVER_NAME " wp");
		if (ret == 0) {
			gpio_direction_input(rmsd_data->gpio_write_protect);
			host->gpio_write_protect =
				rmsd_data->gpio_write_protect;
		}
	}
	rmsdio_clean_isr(host, 0); /* Clean up anything */
	platform_set_drvdata(pdev, mmc);
	ret = mmc_add_host(mmc);
	if (ret)
		goto out;

	/* Now we need to enable all in-host isrs. We'll use 
	   added regs for fine-tuning */
				    
	STEP(7);

	pr_notice("%s: %s driver initialized, ",
		  mmc_hostname(mmc), DRIVER_NAME);
	if (host->gpio_card_detect)
		printk(KERN_INFO "using GPIO %d for card detection\n",
		       host->gpio_card_detect);
	else
		printk(KERN_INFO "lacking card detect (fall back to polling)\n");
	
	host->dev->coherent_dma_mask=DMA_BIT_MASK(32);
	init_waitqueue_head(&host->queue);
	printk(KERN_INFO "rmsdio becomes ready\n");
	return 0;
	
out:
	if (host) {
		if (host->irq)
			free_irq(host->irq, host);
		if (host->gpio_card_detect) {
			free_irq(gpio_to_irq(host->gpio_card_detect), host);
			gpio_free(host->gpio_card_detect);
		}
		if (host->gpio_write_protect)
			gpio_free(host->gpio_write_protect);
		if (host->base)
			iounmap(host->base);
	}
	if (r)
		release_resource(r);
	if (mmc)
		mmc_free_host(mmc);

	return ret;
}

static int __exit rmsdio_remove(struct platform_device *pdev)
{
	MARK();

	return 0;
}

/*
#ifdef CONFIG_PM
static int rmsdio_suspend(struct platform_device *dev, pm_message_t state)
{
	MARK();

	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc);

	return ret;
}

static int rmsdio_resume(struct platform_device *dev)
{
	MARK();

	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define rmsdio_suspend	NULL
#define rmsdio_resume	NULL
#endif

static struct platform_driver rmsdio_driver = {
	.remove		= __exit_p(rmsdio_remove),
	.suspend	= rmsdio_suspend,
	.resume		= rmsdio_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init rmsdio_init(void)
{
	printk(KERN_INFO "\n\n  !!! RMSDIO init !!!\n\n"); 
	
	return platform_driver_probe(&rmsdio_driver, rmsdio_probe);
}

static void __exit rmsdio_exit(void)
{
	platform_driver_unregister(&rmsdio_driver);
}

module_init(rmsdio_init);
module_exit(rmsdio_exit);
*/

static const struct of_device_id mpw7705_mmc_of_match[] = {
	{ .compatible = "rc-module,mmc-0.2", },
	{ .compatible = "rc-module,mpw7705", },	
	{}
};
MODULE_DEVICE_TABLE(of, mpw7705_mmc_of_match);

static struct platform_driver mpw7705_mmc_driver = {
	.probe		= rmsdio_probe,
	.remove		= rmsdio_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(mpw7705_mmc_of_match),
	},
};

module_platform_driver(mpw7705_mmc_driver);

MODULE_DESCRIPTION("MPW7705 SD/MMC driver");
MODULE_AUTHOR("Astrosoft <astrosoft@astrosoft.ru>");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:mmc-rcmodule");
