/*
 *  Copyright (C) 2012 RCM, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RMSDIO_H
#define __RMSDIO_H

/*
 * Clock rates are from the platdata
 */


/*
 * Register offsets
 */
#define RMSDIO_BLOCK_SET            0x00
/* 7:0 card block count register 0-255*/

#define RMSDIO_CBCNT_MASK          0x000000ff
#define RMSDIO_CBSZR_MASK          0x0000ff00
#define RMSDIO_CBLSR_MASK          0xffff0000
#define RMSDIO_BLOCK_SET_CBLSR     0x00000000
#define RMSDIO_BLOCK_SET_512       0x00000100
#define RMSDIO_BLOCK_SET_1024      0x00000200
#define RMSDIO_BLOCK_SET_2048      0x00000300

#define RMSDIO_CTRL                 0x04

#define RMSDIO_CTRL_CRSR_BUSMASK    0x03
#define RMSDIO_CTRL_CRSR_BUS1       0x00
#define RMSDIO_CTRL_CRSR_BUS4       0x01
#define RMSDIO_CTRL_CRSR_BUS8       0x02
#define RMSDIO_CTRL_CRSR_NO_FDRIVE     (1<<4)

#define RMSDIO_CTRL_CRSR_SR        (1<<6)             
#define RMSDIO_CTRL_CRSR_HR        (1<<7)
#define RMSDIO_CTRL_CRSR_FD        (1<<4)

#define RMSDIO_CTRL_RESPONSE_R2    (1<<10)
#define RMSDIO_CTRL_RESPONSE_R1367 (2<<10)
#define RMSDIO_CTRL_RESPONSE_R1B   (3<<10)

#define RMSDIO_CTRL_AUTOCMD12      (1<<9)
#define RMSDIO_CTRL_DATAREAD       (1<<8)
#define RMSDIO_CTRL_CMDCRC         (1<<13)
#define RMSDIO_CTRL_HAVEDATA       (1<<14)
#define RMSDIO_CTRL_IDXCHK         (1<<12)

#define RMSDIO_CMD_ARGUMENT         0x08
#define RMSDIO_ADDRESS              0x0c

#define RMSDIO_STATUS               0x10
#define RMSDIO_STATUS_CARDIN        (1<<11)
#define RMSDIO_STATUS_ERRMASK       0x6f0000
#define RMSDIO_STATUS_CMDTO         (1<<16)
#define RMSDIO_STATUS_CMDCRC        (1<<17)
#define RMSDIO_STATUS_CMDEB         (1<<18)
#define RMSDIO_STATUS_CMDIDX        (1<<19)
#define RMSDIO_STATUS_DATCRC        (1<<21)
#define RMSDIO_STATUS_DATEB         (1<<22)



#define RMSDIO_ERR_ENABLE           0x14

#define RMSDIO_ERR_ENABLE_CARD_INT   (1<<9)
#define RMSDIO_ERR_ENABLE_CTRL_INT   (1<<8)
#define RMSDIO_ERR_CMDTO             (1<<0)
#define RMSDIO_ERR_CMDCRC            (1<<1)
#define RMSDIO_ERR_CMDEB             (1<<2)
#define RMSDIO_ERR_CMDIDX            (1<<3)
#define RMSDIO_ERR_DATCRC            (1<<5)
#define RMSDIO_ERR_DATEB             (1<<6)
#define RMSDIO_ERR_ALL               0x6f




#define RMSDIO_RESPONSE1            0x18
#define RMSDIO_RESPONSE2            0x1C
#define RMSDIO_RESPONSE3            0x20
#define RMSDIO_RESPONSE4            0x24

#define RMSDIO_BUF_TRAN_RESP_REG    0x48

#define RMSDIO_DCCR0                  0x28 // DMA Channel0 Control Register
#define RMSDIO_DCDSAR0                0x30 // DMA Channel0 start address
#define RMSDIO_DCDTR0                 0x34 // DMA Channel0 total data count

#define RMSDIO_DCCR1                  0x38 // DMA Channel1 Control Register
#define RMSDIO_DCDSAR1                0x40 // DMA Channel1 start address
#define RMSDIO_DCDTR1                 0x44 // DMA Channel1 total data count

#define RMSDIO_TRAN_CTL       0x50 // DMA Channel1 total data count
#define RMSDIO_TRAN_FINISH    0x60 // DMA Channel1 total data count

#define RMSDIO_ENABLE         0x300
#define RMSDIO_ENABLE_MODE_SDIO      0x1
#define RMSDIO_ENABLE_MODE_SPI      0x0

#define RMSDIO_CLKDIV         0x304
#define RMSDIO_IRQ_MASKS      0x30c
#define RMSDIO_IRQ_STATUS     0x308

#define RMSDIO_IRQ_SDIO       (1<<7)
#define RMSDIO_IRQ_CARDERROR  (1<<6)
#define RMSDIO_IRQ_CMDDONE    (1<<5)
#define RMSDIO_IRQ_TRANDONE   (1<<4)
#define RMSDIO_IRQ_DATABOUND  (1<<3)
#define RMSDIO_IRQ_TRANFINISH (1<<2)
#define RMSDIO_IRQ_CH1_DONE   (1<<1)
#define RMSDIO_IRQ_CH0_DONE   (1<<0)



#define RMSDIO_DMA_CH0      0x28
#define RMSDIO_DMA_CH1      0x38

#define RMSDIO_DCCR     0x00

/* hardcoded tmplate. 4 bytes AXI width, 16 bytes bursts */
#define RMSDIO_DCCR_TEMPLATE_R     0x20F00
#define RMSDIO_DCCR_TEMPLATE_W     0x20F0

#define RMSDIO_DCCR     0x00
#define RMSDIO_DCSSAR   0x04
#define RMSDIO_DCDSAR   0x08
#define RMSDIO_DCDTR    0x0C


/* Silicon bug: dma channel finish registers are NEVER set */
#define RMSDIO_DMA_CH_FINISH 0x60


#define RMSDIO_CMD_IDX(x) ((x) << 16)



/*
#define FIFO_XFER_DONE		0x1
#define SD_XFER_DONE		0x2
#define SD_CMD_DONE		0x4
#define SD_DATA_BOUND		0x8

#define BUF_XFER_START		(0x1  << 2)
#define BUF_SELECT_R		(0x0  << 1)
#define BUF_SELECT_W		(0x1  << 1)
#define BUF_SELECT_1		0x1
#define BUF_SELECT_0		0x0

#define DMA_CH0_INTR		0x1
#define DMA_CH1_INTR		0x2
*/

#define COMMANDS_TIMEOUT    2000000 //1000000


#endif
