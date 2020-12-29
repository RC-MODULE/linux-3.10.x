/*
 * KeyASIC MMC/SD/SDIO driver
 * Designed for RCM's Flavor of KeyASIC's MMC/SDIO and 
 * relies on a few extra regs added by RCM to simplify things
 *
 * Authors: Andrew Andrianov
 * Copyright (C) 2012 RCM.
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
 * Nice people at RCM created a few extra registers to help poor 
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
//#define DEBUG_ERRORBITS
//#define DBG_RESPONSE
//#define DBG_CALLS
//#define DBG_CMD
//#define DBG_LINES
//#define DBG_BLOCKS
//#define DBG_INTR
//#define DBG_MARKS

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
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include "rcm-sdio.h"

#ifdef CONFIG_BASIS_PLATFORM
#	include "../../misc/rcm/basis/basis-device.h"
#	include "../../misc/rcm/basis/basis-controller.h"
#	include "../../misc/rcm/basis/basis-cfs.h"
#endif

#define DRIVER_NAME	"rmsdio"

#define RMSDIO_CLKDIV_MAX  0xFF

#define MATCH_BITS(v,bits) ((v & bits)==bits)

#define FPGA_DTRACE() { \
		printk(KERN_INFO "rmsdio: %s %s %s %s")			\
}
	
struct rmsdio_platform_data 
{
//	int gpio_card_detect; //card detect
//	int gpio_write_protect; // r/w gpio
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
//	int gpio_card_detect;
//	int gpio_write_protect;
	unsigned int dccr0_flags;
	unsigned int dccr1_flags;
	unsigned int sdio_timeout;

	dma_addr_t dma_addr;
	void      *buff;
#ifdef CONFIG_BASIS_PLATFORM
	u32        ep_addr;
#endif

#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device    *device;
	u32                     regs;
	u32                     regs_size;
	u32                     hwirq;
	u32                     min_frequency;
	u32                     max_frequency;
	u32                     axi_awlen;
	u32                     axi_arlen;
	u32                     axi_awsize;
	u32                     axi_arsize;
	u32                     sdio_timeout_ms;
#endif
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

#ifdef DBG_MARKS
#define LIGHT()  printk(KERN_INFO "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
#define DO_MARK()  printk(KERN_INFO "rmsdio: ==========> %s \n", __FUNCTION__)
#else
#define LIGHT()
#define DO_MARK()
#endif
#ifdef DBG_CALLS
	#define MARK() DO_MARK()
	#define STEP(n)  printk(KERN_INFO "   rmsdio: ======> %s(%d)\n", __FUNCTION__, n)
#else	
	#define MARK()
	#define STEP(n)
#endif

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

#if defined(DEBUG) || defined(DBG_INTR)
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

static bool isprintable(const char c)
{
	return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '!' && c <= '?') || c == ' ') ? true : false;
}

static void __maybe_unused print_buf(const u8 * buf, uint size)
{
	char tmp[20], out[20 + 2 * 512], * ptr = out;
	uint len = (size < 512 ? size : 512), sub_len, i;
	bool is_zero = true;
	
	sub_len = sprintf(tmp, "buf[%d]=", size);
	strcpy(ptr, tmp);
	ptr += sub_len;
	
	for ( i = 0; i < len; ++ i ) 
		if ( buf[i] != '\0' ) {
			is_zero = false;
			break;
		}
	if ( ! is_zero ) {	 
		while ( len -- ) {
			char c = * buf ++;
			if ( isprintable(c) ) sprintf(tmp, "_%c", c);
			else                  sprintf(tmp, "%02x", c);
			strcpy(ptr, tmp);
			ptr += 2;
		}
	} else
		strcpy(ptr, "[ZERO]");
	
	printk(KERN_INFO "%s", out);
}

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

	BUG_ON(((u64) addr >> 32) != 0);  // !!! We use only 32 bits here	
	
	dev_dbg(host->dev, "dma %llu bytes: 0x%Lx %s buf%d\n", 
	                   (u64)sz, (long long) addr, (dir == DMA_FROM_DEVICE) ? "<=" : "=>", buf );  
	/* channel 0 can only do 'to-device', whilst ch1 can only 'from-device' */
	BUG_ON(buf>1);
	channel = (dir == DMA_TO_DEVICE) ? RMSDIO_DMA_CH0 : RMSDIO_DMA_CH1;
	areg = (dir == DMA_TO_DEVICE) ? RMSDIO_DCSSAR : RMSDIO_DCDSAR;
	tctl = 0x4 | buf;
	tmpl = (dir == DMA_FROM_DEVICE) ? RMSDIO_DCCR_TEMPLATE_R : RMSDIO_DCCR_TEMPLATE_W; 
	tctl |= (dir == DMA_TO_DEVICE) ? 2 : 0;
	if(channel == RMSDIO_DMA_CH0) {
		rmsdio_write(RMSDIO_DCCR0, host->dccr0_flags);
	}
	else {
		rmsdio_write(RMSDIO_DCCR1, host->dccr1_flags);
	}
	rmsdio_write(RMSDIO_TRAN_CTL, tctl);
	rmsdio_write(channel + RMSDIO_DCDTR,  sz);
	rmsdio_write(channel + areg, addr);  
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
	
	DO_MARK();

	spin_lock_irqsave(&host->lock, host->iflags);
	LIGHT();
	dev_dbg(host->dev, " %s sdio card interrupt\n", enable ? "enabling" : "disabling");
	LIGHT();
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
#ifdef DBG_RESPONSE
	dev_dbg(host->dev, "r1 0x%x\n",cmd->resp[0]);	
	dev_dbg(host->dev, "r2 0x%x\n",cmd->resp[1]);
	dev_dbg(host->dev, "r3 0x%x\n",cmd->resp[2]);
	dev_dbg(host->dev, "r4 0x%x\n",cmd->resp[3]);
#endif
	/* reset any err bits */
//	rmsdio_write(RMSDIO_STATUS, (err_status & (~RMSDIO_STATUS_ERRMASK)));
	
#ifdef DEBUG_ERRORBITS
	if ( err_status & (RMSDIO_STATUS_CMDCRC | RMSDIO_STATUS_CMDEB |  // except RMSDIO_STATUS_CMDTO 
	                   RMSDIO_STATUS_CMDIDX | RMSDIO_STATUS_DATCRC | RMSDIO_STATUS_DATEB) ) {
		dev_dbg(host->dev, "--- error bits present ---\n");
		if ( err_status & RMSDIO_STATUS_CMDCRC )
			dev_dbg(host->dev, "error: command crc7 failed\n");
		if ( err_status & RMSDIO_STATUS_CMDEB )
			dev_dbg(host->dev, "error: command end bit error\n");
		if ( err_status & RMSDIO_STATUS_CMDIDX )
			dev_dbg(host->dev, "error: command idx error\n");
		if ( err_status & RMSDIO_STATUS_DATCRC )
			dev_dbg(host->dev, "error: data crc error\n");
		if ( err_status & RMSDIO_STATUS_DATEB )
			dev_dbg(host->dev, "error: data endbit error\n");
		dev_dbg(host->dev, "--- error bits present ---\n");
	}
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

	if ( err_status & RMSDIO_STATUS_CMDTO ) {
		dev_dbg(host->dev, "error: command timeout\n");
		cmd->error = -ETIMEDOUT;
	} else if ( err_status & RMSDIO_STATUS_ERRMASK ) {
		dev_dbg(host->dev, "error: errors present: 0x%04x\n", err_status);
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
		host->sdio_timeout
		);

	if ( wret<= 0 && !MATCH_BITS(rmsdio_read(RMSDIO_IRQ_STATUS), (wintr)))
		goto bailout;

	rmsdio_clean_isr(host, 0);

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
		if (blocks < data->blocks )
			wintr |= RMSDIO_IRQ_CH0_DONE | RMSDIO_IRQ_TRANFINISH;
	
		rmsdio_write(RMSDIO_IRQ_MASKS, wintr);

		if (blocks < data->blocks ) {
			rmsdio_dma_xfer(host, DMA_TO_DEVICE, host->next_buffer, host->dma_target, data->blksz);
			data->bytes_xfered += data->blksz;
			host->cpu_target += data->blksz;
			host->dma_target += data->blksz;
		}

		rmsdio_wait_irq(
			wret, host->queue,
			(MATCH_BITS(host->irqstat, 
					(wintr))),
			host->sdio_timeout
			);

		if (wret <= 0 && !MATCH_BITS(rmsdio_read(RMSDIO_IRQ_STATUS), (wintr)))
			goto bailout;

		rmsdio_clean_isr(host, 0);
		wintr = 0;
	} while (blocks <= data->blocks);
	
	return wret;
bailout:
	dev_err(host->dev, "Writing: Timed out waiting for interrupt bits\n");
	decode_intr(host," flags want: ", wintr);
	decode_intr(host," flags have: ", rmsdio_read(RMSDIO_IRQ_STATUS));
	host->irqstat |= rmsdio_read(RMSDIO_IRQ_STATUS);
	rmsdio_clean_isr(host, 0);

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
				host->sdio_timeout
		);

		if (wret <= 0 && !MATCH_BITS(rmsdio_read(RMSDIO_IRQ_STATUS), (wintr)))
			goto bailout;
		
#ifdef DBG_BLOCKS		
		printk(KERN_INFO "$$%d$$", blocks);
#endif

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
		
#ifdef DBG_BLOCKS			
		if ( blocks != 1 )  
			print_buf(host->cpu_target - 2 * data->blksz, data->blksz);
#endif			
		
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
	dev_err(host->dev, "Reading: Timed out waiting for interrupt bits\n");
	decode_intr(host," flags want: ", wintr);
	decode_intr(host," flags have: ", host->irqstat);
	decode_intr(host," flags in reg: ", rmsdio_read(RMSDIO_IRQ_STATUS));

	host->irqstat |= rmsdio_read(RMSDIO_IRQ_STATUS);
	rmsdio_clean_isr(host, 0);

	return wret;
	
}

static void rmsdio_request(struct mmc_host * mmc, struct mmc_request * mrq)
{
	struct rmsdio_host * host = mmc_priv(mmc);
	void __iomem * iobase = host->base;
	struct mmc_command * cmd = mrq->cmd;
	struct mmc_data * data = mrq->data;
	uint32_t ctrlreg = 0, intr = 0, err_status = 0;
	uint32_t eereg;
	int wret = 0;
	size_t copied;
	
	MARK();
	
#ifdef DBG_CMD	
	if ( data ) printk(KERN_INFO " >CMD: cmd=%d, resp= 0x%x, arg=0x%x [%d x %d]", 
	                             cmd->opcode, mmc_resp_type(cmd), cmd->arg, data->blocks, data->blksz);
	else        printk(KERN_INFO " >CMD: cmd=%d, resp= 0x%x, arg=0x%x", cmd->opcode, mmc_resp_type(cmd), cmd->arg);
#endif	
	
	/* Since we can't queue up cmds, we can only have one request ongoing */
	spin_lock_irqsave(& host->lock, host->iflags);
	if ( host->busy ) {
		spin_unlock_irqrestore(& host->lock, host->iflags);
		mrq->cmd->error = -EAGAIN;
		mmc_request_done(mmc, mrq);
		return;
	}
	++ host->busy;
	spin_unlock_irqrestore(& host->lock, host->iflags);

	eereg = rmsdio_read(RMSDIO_ERR_ENABLE) & (RMSDIO_ERR_ENABLE_CTRL_INT);
	BUG_ON(host->mrq != NULL);
	host->irqstat = 0; /* reset flags */
	host->shedcmd = 0;
	host->mrq = mrq;
	
	ctrlreg = host->ctrl_reg |  RMSDIO_CMD_IDX(cmd->opcode);
	
	{ // Set responce kind
		const char * resp_str = NULL;
		if ( mmc_resp_type(cmd) == MMC_RSP_R2 ) {
			ctrlreg |= RMSDIO_CTRL_RESPONSE_R2;
			resp_str = "R2";
		} else if ( mmc_resp_type(cmd) == MMC_RSP_R1B ) {
			ctrlreg |= RMSDIO_CTRL_RESPONSE_R1B;
			resp_str = "R1B";
		} else if ( mmc_resp_type(cmd) & MMC_RSP_PRESENT ) { 
			ctrlreg |= RMSDIO_CTRL_RESPONSE_R1367;
			resp_str = "R1367";
		}
#ifdef DBG_RESPONSE
		if ( resp_str ) 
			dev_dbg(host->dev, "Expecting %s response\n", resp_str);
#endif
	}
	
	if ( cmd->flags & MMC_RSP_CRC ) {
		ctrlreg |= RMSDIO_CTRL_CMDCRC;
		eereg |= RMSDIO_ERR_CMDCRC;
	}
	        
	if ( cmd->flags & MMC_RSP_OPCODE ) {
		ctrlreg |= RMSDIO_CTRL_IDXCHK;
		eereg |= RMSDIO_ERR_CMDIDX;
	}

	host->next_buffer = 0;

	eereg |= RMSDIO_ERR_ENABLE_CTRL_INT;
	eereg |= RMSDIO_ERR_CMDEB | RMSDIO_ERR_CMDTO; 
	rmsdio_write(RMSDIO_CMD_ARGUMENT, cmd->arg);
	
	if ( is_sdio_irq_enabled() )
		eereg |= RMSDIO_ERR_ENABLE_CARD_INT; 

	if ( data ) {  
		if (data->flags & MMC_DATA_WRITE) {
			copied = sg_copy_to_buffer(data->sg, data->sg_len, 
			                           host->buff,
			                           data->blocks * data->blksz);
			if (copied != data->blocks * data->blksz) {
				dev_err(host->dev,
				        "cannot copy data from sg list\n");
				wret = -EFAULT;
			}
		}

		if (wret == 0) {
#ifdef CONFIG_BASIS_PLATFORM
			host->dma_target = host->ep_addr;
#else
			host->dma_target = host->dma_addr;
#endif
			host->cpu_target = host->buff;
			/* Do all the register voodoo */
			ctrlreg |= RMSDIO_CTRL_HAVEDATA;
			rmsdio_setup_data(host, data);

			if ( data->stop )
				ctrlreg |= RMSDIO_CTRL_AUTOCMD12;

			dev_dbg(host->dev, "i/o - we're planning to: %s %s\n",
				(data->flags & MMC_DATA_WRITE) ? "write" : "read",
				(data->stop) ? "autocmd12" : "");

			eereg |= RMSDIO_ERR_DATEB | RMSDIO_ERR_DATCRC;
			rmsdio_write(RMSDIO_ERR_ENABLE, eereg);

			wret = (data->flags & MMC_DATA_WRITE ? rmsdio_write_flow : rmsdio_read_flow)(mmc, mrq, ctrlreg);

			if ((wret > 0) &&
			    ((data->flags & MMC_DATA_WRITE) == 0)) {
				copied = sg_copy_from_buffer(data->sg,
				                             data->sg_len, 
				                             host->buff,
				                             data->blocks * data->blksz);
				if (copied != data->blocks * data->blksz) {
					dev_err(host->dev,
					        "cannot copy data to sg list\n");
					wret = -EFAULT;
				}
			}
		}
	} else {
		/* We only reach this place when no xfer data issued */
		rmsdio_write(RMSDIO_ERR_ENABLE, eereg);
		intr |= RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_CARDERROR;
		rmsdio_write(RMSDIO_IRQ_MASKS, intr);
		rmsdio_write(RMSDIO_CTRL, ctrlreg);

		/* cmd done or card error. either of those indicate we're done here */ 
		rmsdio_wait_irq(wret, host->queue,
			(host->irqstat & (RMSDIO_IRQ_CMDDONE | RMSDIO_IRQ_CARDERROR)),
			host->sdio_timeout);

		rmsdio_clean_isr(host, 0);
	}
	
	if ( wret <= 0 )
		cmd->error = -EIO;
	
	err_status = rmsdio_read(RMSDIO_STATUS) & RMSDIO_STATUS_ERRMASK;
	/* This should handle error statuses, if any */
	spin_lock_irqsave(& host->lock, host->iflags);
	rmsdio_finish_cmd(host, cmd, err_status);
	host->mrq = NULL;
	host->busy = 0;
	mmc_request_done(host->mmc, mrq);
	if ( is_sdio_irq_enabled() ) 
		rmsdio_enable_sdio_irq(mmc, 1);
	spin_unlock_irqrestore(& host->lock, host->iflags);
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
#ifdef DEBUG	
	decode_intr(host, "IRQ! new irqstat: ", host->irqstat);
#endif	
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
	
	return IRQ_HANDLED;
}

static irqreturn_t __maybe_unused rmsdio_card_detect_irq(int irq, void *dev)
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

	dev_dbg(host->dev, "rmsdio: setting i/o params: pmode=%d, clock=%d, bw=%d\n", ios->power_mode, ios->clock, ios->bus_width);

	rmsdio_write(RMSDIO_CTRL, RMSDIO_CTRL_CRSR_HR);
	if (ios->power_mode == MMC_POWER_UP)
		rmsdio_power_up(host);
	if (ios->clock == 0) {
		rmsdio_write(RMSDIO_CLKDIV,RMSDIO_CLKDIV_MAX);
		host->clock=0;
		dev_dbg(host->dev, "clock somewhat disabled\n");
	} else if (ios->clock != host->clock)
	{
		uint32_t m = DIV_ROUND_UP(host->base_clock, ios->clock * 2) - 1;
		if (m > RMSDIO_CLKDIV_MAX)
			m = RMSDIO_CLKDIV_MAX;
		//m = 0x6;
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

// HW unable to reenable SDIO after SPI - so - disable it for a while
//	if (ios->power_mode == MMC_POWER_OFF)
//		rmsdio_power_down(host);
}

static const struct mmc_host_ops rmsdio_ops = {
	.request		= rmsdio_request,
	.get_ro			= rmsdio_get_ro,
	.set_ios		= rmsdio_set_ios,
	.enable_sdio_irq	= rmsdio_enable_sdio_irq,
};

#ifdef CONFIG_BASIS_PLATFORM

static int rmsdio_bind(struct basis_device *device)
{
	struct rmsdio_host *host = basis_device_get_drvdata(device);
	struct mmc_host *mmc = host->mmc;
	int ret = 0, irq;

	printk(KERN_INFO "rmsdio: RC Module SD/SDIO/MMC driver (c) 2018\n");

	dev_info(&device->dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	irq = irq_create_mapping(device->controller->domain, host->hwirq);

	if (irq < 0)
		return -ENXIO;

	STEP(1);

	mmc->ops = &rmsdio_ops;
	/* TODO: More caps & ocr_avail */

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;

	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0xff; 

	mmc->max_segs = 1;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	mmc->f_max = host->max_frequency;
	mmc->f_min = host->min_frequency;

	host->sdio_timeout = msecs_to_jiffies(host->sdio_timeout_ms);

	host->dccr0_flags = (host->axi_awlen & RMSDIO_AXI_BURST_MASK) << RMSDIO_AXI_BURST_SHIFT;
	host->dccr0_flags |= (host->axi_awsize & RMSDIO_AXI_SIZE_MASK) << RMSDIO_AXI_SIZE_SHIFT;
	host->dccr1_flags = (host->axi_arlen & RMSDIO_AXI_BURST_MASK) << RMSDIO_AXI_BURST_SHIFT;
	host->dccr1_flags |= (host->axi_arsize & RMSDIO_AXI_SIZE_MASK) << RMSDIO_AXI_SIZE_SHIFT;

	pr_debug("rmsdio: maximum clock is %d Hz minimum is %d Hz\n", mmc->f_max, mmc->f_min);
	host->sdio_irq_enabled = 0;
	spin_lock_init(&host->lock);
	host->base = devm_ioremap(&device->dev,
	                          host->regs + device->controller->ep_base_phys,
	                          host->regs_size);
	printk(KERN_INFO "rmsdio start is 0x%Lx, base is 0x%llx\n", 
	       (long long)(host->regs + device->controller->ep_base_phys),
	       (u64) host->base);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

#ifdef CONFIG_1888TX018
	device->dev.archdata.dma_offset = - (device->dev.dma_pfn_offset << PAGE_SHIFT);
#endif

	host->buff = basis_device_dma_alloc_coherent(&device->dev,
	                                             mmc->max_req_size,
	                                             &host->dma_addr,
	                                             &host->ep_addr,
	                                             GFP_KERNEL);
	if (!host->buff) {
		dev_err(&device->dev,
		        "Failed to allocate DMA-coherent memory (%u bytes).\n",
		        mmc->max_req_size);
		goto out;
	}

	STEP(4);

	init_waitqueue_head(&host->queue);

	rmsdio_power_down(host);
	ret = request_irq(irq, rmsdio_irq, 0, DRIVER_NAME, host);
	if (ret) {
		LIGHT();
		pr_err("%s: cannot assign irq %d\n", DRIVER_NAME, irq);
		LIGHT();
		goto out;
	}

	STEP(5);

	host->irq = irq;
	pr_debug("rmsdio: Using irq %d\n", irq);

	rmsdio_clean_isr(host, 0); /* Clean up anything */
	ret = mmc_add_host(mmc);
	if (ret)
		goto out;

	/* Now we need to enable all in-host isrs. We'll use 
	   added regs for fine-tuning */
				    
	STEP(7);

	pr_notice("%s: %s driver initialized, ",
		  mmc_hostname(mmc), DRIVER_NAME);

	host->dev->coherent_dma_mask=DMA_BIT_MASK(25);
	printk(KERN_INFO "rmsdio becomes ready\n");
	return 0;
	
out:
	if (host) {
		if (host->irq)
			free_irq(host->irq, host);
		if (!host->buff)
			basis_device_dma_free_coherent(&device->dev,
			                               mmc->max_req_size,
			                               host->buff,
			                               host->dma_addr,
			                               host->ep_addr);
	}
	if (mmc)
		mmc_free_host(mmc);

	return ret;
}

static void rmsdio_unbind(struct basis_device *device)
{
}

static int rmsdio_probe(struct basis_device *device)
{
	struct device *dev = &device->dev;
	struct mmc_host *mmc;
	struct rmsdio_host *host;

	mmc = mmc_alloc_host(sizeof(struct rmsdio_host), dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = dev;

	host->axi_arlen = 15;	// 16 data frames
	host->axi_awlen = 15;	// 16 data frames
	host->axi_arsize = 2;	// 4 bytes
	host->axi_awsize = 2;	// 4 bytes

	host->sdio_timeout_ms = 200;

	host->device = device;

	basis_device_set_drvdata(device, host);

	return 0;
}

static const struct basis_device_id rmsdio_ids[] = {
	{
		.name = "rcm-sdio",
	},
	{},
};

static struct basis_device_ops rcm_sdio_ops = {
	.unbind = rmsdio_unbind,
	.bind   = rmsdio_bind,
};

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  hwirq,           struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, hwirq,           struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  regs,            struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, regs,            struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  regs_size,       struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, regs_size,       struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  base_clock,      struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, base_clock,      struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  min_frequency,   struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, min_frequency,   struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  max_frequency,   struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, max_frequency,   struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  axi_awlen,       struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, axi_awlen,       struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  axi_arlen,       struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, axi_arlen,       struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  axi_awsize,      struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, axi_awsize,      struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  axi_arsize,      struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, axi_arsize,      struct rmsdio_host);

BASIS_DEV_ATTR_U32_SHOW(rmsdio_,  sdio_timeout_ms, struct rmsdio_host);
BASIS_DEV_ATTR_U32_STORE(rmsdio_, sdio_timeout_ms, struct rmsdio_host);

CONFIGFS_ATTR(rmsdio_, hwirq);
CONFIGFS_ATTR(rmsdio_, regs);
CONFIGFS_ATTR(rmsdio_, regs_size);
CONFIGFS_ATTR(rmsdio_, base_clock);
CONFIGFS_ATTR(rmsdio_, min_frequency);
CONFIGFS_ATTR(rmsdio_, max_frequency);
CONFIGFS_ATTR(rmsdio_, axi_awlen);
CONFIGFS_ATTR(rmsdio_, axi_arlen);
CONFIGFS_ATTR(rmsdio_, axi_awsize);
CONFIGFS_ATTR(rmsdio_, axi_arsize);
CONFIGFS_ATTR(rmsdio_, sdio_timeout_ms);

static struct configfs_attribute *rmsdio_attrs[] = {
	&rmsdio_attr_hwirq,
	&rmsdio_attr_regs,
	&rmsdio_attr_regs_size,
	&rmsdio_attr_base_clock,
	&rmsdio_attr_min_frequency,
	&rmsdio_attr_max_frequency,
	&rmsdio_attr_axi_awlen,
	&rmsdio_attr_axi_arlen,
	&rmsdio_attr_axi_awsize,
	&rmsdio_attr_axi_arsize,
	&rmsdio_attr_sdio_timeout_ms,
	NULL,
};

static struct basis_device_driver rcm_mmc_driver = {
	.driver.name    = DRIVER_NAME,
	.probe          = rmsdio_probe,
	.id_table       = rmsdio_ids,
	.ops            = &rcm_sdio_ops,
	.owner          = THIS_MODULE,
	.attrs          = rmsdio_attrs,
};
module_basis_driver(rcm_mmc_driver);

#else /* CONFIG_BASIS_PLATFORM */

static int rmsdio_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc = NULL;
	struct rmsdio_host *host = NULL;
	struct resource *r;
	int ret = 0, irq;
	struct clk *clk;
	struct device_node		*np = pdev->dev.of_node;
	unsigned int axi_arlen = 15;	// 16 data frames
	unsigned int axi_awlen = 15;	// 16 data frames
	unsigned int axi_arsize = 2;	// 4 bytes
	unsigned int axi_awsize = 2;	// 4 bytes
	
	printk(KERN_INFO "rmsdio: RC Module SD/SDIO/MMC driver (c) 2018\n");
		
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	
	if (!r || irq < 0 )
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
	mmc->ops = &rmsdio_ops;
	/* TODO: More caps & ocr_avail */

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;

	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0xff; 

	mmc->max_segs = 1;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	host->sdio_timeout = msecs_to_jiffies(200); // 200 ms by default

	clk = devm_clk_get(&pdev->dev, "mmc");
	if (IS_ERR(clk)) 
		return -ENODEV;

	clk_enable(clk);

	host->base_clock = clk_get_rate(clk);

	if (!host->base_clock)
		return -EINVAL;

	if(mmc_of_parse(mmc))
	{
		pr_err("%s: unable to parse mmc definition\n", DRIVER_NAME);
		goto out;
	}

	if (np) {
		unsigned int val;
		if(of_property_read_u32(np, "axi-awlen", &val) == 0)
			axi_awlen = val;
		if(of_property_read_u32(np, "axi-arlen", &val) == 0)
			axi_arlen = val;
		if(of_property_read_u32(np, "axi-awsize", &val) == 0)
			axi_awsize = val;
		if(of_property_read_u32(np, "axi-arsize", &val) == 0)
			axi_arsize = val;
		if(of_property_read_u32(np, "sdio-timeout", &val) == 0)
				host->sdio_timeout = msecs_to_jiffies(val);
	}

	host->dccr0_flags = (axi_awlen & RMSDIO_AXI_BURST_MASK) << RMSDIO_AXI_BURST_SHIFT;
	host->dccr0_flags |= (axi_awsize & RMSDIO_AXI_SIZE_MASK) << RMSDIO_AXI_SIZE_SHIFT;
	host->dccr1_flags = (axi_arlen & RMSDIO_AXI_BURST_MASK) << RMSDIO_AXI_BURST_SHIFT;
	host->dccr1_flags |= (axi_arsize & RMSDIO_AXI_SIZE_MASK) << RMSDIO_AXI_SIZE_SHIFT;	

//	mmc->f_min = DIV_ROUND_UP(host->base_clock, 2*(RMSDIO_CLKDIV_MAX+1));
//	if (np) {
//		int val;
//		if(of_property_read_u32(np, "min-frequency", &val) == 0)
//		{
//			printk(KERN_INFO "rmsdio: limit min speed to %d Hz", val);		
//			mmc->f_min = val;
//		}
//	}

//	if (np) {
//		int val;
//		if(of_property_read_u32(np, "max-frequency", &val) == 0)
//		{
//			printk(KERN_INFO "rmsdio: limit max speed to %d Hz", val);		
//			mmc->f_max = val;
//		}
//	}
//	else
//	{
//		mmc->f_max = host->base_clock;
//	}

	pr_debug("rmsdio: maximum clock is %d Hz minimum is %d Hz\n", mmc->f_max, mmc->f_min);
	host->sdio_irq_enabled = 0;
	spin_lock_init(&host->lock);	
	host->base = ioremap(r->start, SZ_4K);
	printk(KERN_INFO "rmsdio start is 0x%Lx, base is 0x%x\n", (long long) r->start, (uint) host->base);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

#ifdef CONFIG_1888TX018
	pdev->dev.archdata.dma_offset = - (pdev->dev.dma_pfn_offset << PAGE_SHIFT); /* before v5.5 it was: set_dma_offset(&pdev->dev, - (pdev->dev.dma_pfn_offset << PAGE_SHIFT)); */
#endif

	host->buff = dma_alloc_coherent(&pdev->dev, mmc->max_req_size,
	                                &host->dma_addr, GFP_KERNEL);
	if (!host->buff) {
		dev_err(&pdev->dev,
		        "Failed to allocate DMA-coherent memory (%u bytes).\n",
		        mmc->max_req_size);
		goto out;
	}

	STEP(4);

	init_waitqueue_head(&host->queue);

	rmsdio_power_down(host);
	ret = request_irq(irq, rmsdio_irq, 0, DRIVER_NAME, host);
	if (ret) {
		LIGHT();
		pr_err("%s: cannot assign irq %d\n", DRIVER_NAME, irq);
		LIGHT();
		goto out;
	}

	STEP(5);

	host->irq = irq;
	pr_debug("rmsdio: Using irq %d\n", irq);

/* card detect and write protect proceeded now by mmc_of_parse so following functionlaity not needed anymore */

//	if (rmsd_data->gpio_card_detect) {
//		ret = gpio_request(rmsd_data->gpio_card_detect, "carddetect-gpio"/*DRIVER_NAME " cd"*/);
//		// !!! ret = gpio_request_by_name(dev, "carddetect-gpio", 0, & rmsd_data->gpio_card_detect, GPIOD_IS_IN);
//		pr_debug("rmsdio: gpio_card_detect: gpio_request: ret=%d\n", ret);
//		if (ret == 0) {
//			gpio_direction_input(rmsd_data->gpio_card_detect);
//			irq = gpio_to_irq(rmsd_data->gpio_card_detect);
//			ret = request_irq(irq, rmsdio_card_detect_irq,
//					  IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING,
//					  DRIVER_NAME " cd", host);
//			pr_debug("rmsdio: gpio_card_detect: request_irq: ret=%d\n", ret);
//			if (ret == 0)
//				host->gpio_card_detect =
//					rmsd_data->gpio_card_detect;
//			else
//				gpio_free(rmsd_data->gpio_card_detect);
//		}
//	}
	/*
	   if (!host->gpio_card_detect) 
	   mmc->caps |= MMC_CAP_NEEDS_POLL;
	*/

//	STEP(6);

//	if (rmsd_data->gpio_write_protect) {
//		ret = gpio_request(rmsd_data->gpio_write_protect,
//				   DRIVER_NAME " wp");
//		if (ret == 0) {
//			gpio_direction_input(rmsd_data->gpio_write_protect);
//			host->gpio_write_protect =
//				rmsd_data->gpio_write_protect;
//		}
//	}

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

/* card detect and write protect proceeded now by mmc_of_parse so following functionlaity not needed anymore */

//	if (host->gpio_card_detect)
//		printk(KERN_INFO "using GPIO %d for card detection\n",
//		       host->gpio_card_detect);
//	else
//		printk(KERN_INFO "lacking card detect (fall back to polling)\n");
	
	host->dev->coherent_dma_mask=DMA_BIT_MASK(25);
	printk(KERN_INFO "rmsdio becomes ready\n");
	return 0;
	
out:
	if (host) {
		if (host->irq)
			free_irq(host->irq, host);
/* card detect and write protect proceeded now by mmc_of_parse so following functionlaity not needed anymore */
//		if (host->gpio_card_detect) {
//			free_irq(gpio_to_irq(host->gpio_card_detect), host);
//			gpio_free(host->gpio_card_detect);
//		}
//		if (host->gpio_write_protect)
//			gpio_free(host->gpio_write_protect);
		if (host->buff)
			dma_free_coherent(&pdev->dev, mmc->max_req_size,
			                  host->buff, host->dma_addr);
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

static const struct of_device_id rcm_mmc_of_match[] = {
	{ .compatible = "rcm,mmc-0.2", },
	{}
};
MODULE_DEVICE_TABLE(of, rcm_mmc_of_match);

static struct platform_driver rcm_mmc_driver = {
	.probe		= rmsdio_probe,
	.remove		= rmsdio_remove,
	.driver		= {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(rcm_mmc_of_match),
	},
};

module_platform_driver(rcm_mmc_driver);

#endif /* CONFIG_BASIS_PLATFORM */

MODULE_DESCRIPTION("RCM SD/MMC driver");
MODULE_AUTHOR("Astrosoft <astrosoft@astrosoft.ru>");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:mmc-rcm");
