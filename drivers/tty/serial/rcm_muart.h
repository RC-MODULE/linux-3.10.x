// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 *  2020 Vladimir Shalyt <Vladimir.Shalyt@mir.dev>
 *  - DMA support
 */

/*
 *Information about a serial port
 *
 * @base: Register base address
 */
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/irqdomain.h>

#ifdef CONFIG_BASIS_PLATFORM
#	include "../../misc/rcm/basis/basis-device.h"
#	include "../../misc/rcm/basis/basis-controller.h"
#	include "../../misc/rcm/basis/basis-cfs.h"
#endif

#define MUART_DMA_CNT_BUFFS_PER_CHAN 8

struct muart_regs {
	u32 id; // 0x000
	u32 version; // 0x004
	u32 sw_rst; // 0x008
	u32 reserve_1; // 0x00C
	u32 gen_status; // 0x010
	u32 fifo_state; // 0x014
	u32 status; // 0x018
	u32 reserve_2; // 0x01C
	u32 dtrans; // 0x020
	u32 reserve_3; // 0x024
	u32 drec; // 0x028
	u32 reserve_4; // 0x02C
	u32 bdiv; // 0x030
	u32 reserve_5; // 0x034
	u32 reserve_6; // 0x038
	u32 reserve_7; // 0x03C
	u32 fifowm; // 0x040
	u32 ctrl; // 0x044
	u32 mask; // 0x048
	u32 rxtimeout; // 0x04C
	u32 reserve_8; // 0x050
	u32 txtimeout; // 0x054
	u32 reserve_58[42];
};

/* Deals with DMA transactions */

struct muart_dma_buff {
	struct muart_dma*     dma;
	dma_cookie_t          cookie;
	struct scatterlist    sg;
	struct completion     dma_completion;
	size_t                residue;
	bool                  error;
};

struct muart_dma {
	struct dma_chan      *chan;
	struct muart_dma_buff buffs[MUART_DMA_CNT_BUFFS_PER_CHAN];
	int                   next_buff;
	int                   first_buff;
	int                   cnt_buffs;
	spinlock_t            lock;
	dma_addr_t            dma_addr;
#ifdef CONFIG_BASIS_PLATFORM
	u32                   ep_addr;
#endif
	void*                 buff;

	struct task_struct*   thread;
};

struct muart_port {
	struct uart_port        port;
	struct clk             *clk;
	unsigned long           baud;
	struct irq_domain      *domain;
	/* DMA stuff */
	bool                    using_dma;
	struct muart_dma        tx;
	struct muart_dma        rx;
#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device    *device;
	u32                     reg;
	u32                     reg_size;
	u32                     hwirq;
	u32                     uartclk;
	char                    dma_dev[64];
	u32                     tx_ch_num;
	u32                     rx_ch_num;
#endif
};

#define to_muart_port(uport) container_of(uport, struct muart_port, port)

bool muart_dma_tx_start(struct muart_port *uart);
int muart_dma_startup(struct muart_port *uart);
void muart_dma_shutdown(struct muart_port *uart);
void muart_dma_stop_rx(struct muart_port *uart);
void muart_free_dma(struct muart_port *uart);
int muart_alloc_dma(struct muart_port *uart);
