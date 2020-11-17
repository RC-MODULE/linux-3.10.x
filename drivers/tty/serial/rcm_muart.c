/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/module.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include "../../dma/rcm-mdma.h"

#define MUART_ID 0x55415254
#define MUART_VERSION 0x10190

#define MUART_TX_FIFO_SIZE 0x3ff
#define MUART_RX_FIFO_SIZE 0x3ff
#define MUART_FIFO_MASK 0x7ff

//MUART_CTRL
#define MUART_CTRL_MEN_i 0
#define MUART_CTRL_LBE_i 1
#define MUART_CTRL_APB_MD_i 2
#define MUART_CTRL_MDS_i 3
#define MUART_CTRL_RTSen_i 4
#define MUART_CTRL_CTSen_i 5
#define MUART_CTRL_RTS_POL 6
#define MUART_CTRL_PEN_i 7
#define MUART_CTRL_EPS_i 8
#define MUART_CTRL_SPS_i 9
#define MUART_CTRL_STP2_i 10
#define MUART_CTRL_WLEN_i 12
#define MUART_CTRL_DUM_i 15

//MUART_BDIV
#define MUART_BAUD_DIV_i 0
#define MUART_N_DIV 24

//MUART_FIFO_STATE
#define MUART_RXFS_i 0
#define MUART_TXFS_i 16

//MUART_DREG
#define MUART_DREG_FE (1 << 12) // frame error
#define MUART_DREG_PE (1 << 13) // parity error
#define MUART_DREG_BE (1 << 14) // break error
#define MUART_DREG_OE (1 << 15) // overflow error
#define MUART_DREG_DUMMY_RX (1 << 16) // dummy flag
#define MUART_DREG_ERR                                                         \
	(MUART_DREG_FE | MUART_DREG_PE | MUART_DREG_BE | MUART_DREG_OE)

// MUART_FIFOWM shifts
#define MUART_FIFOWM_RXFS_i 0
#define MUART_FIFOWM_TXFS_i 16

//MUART_STATUS /MASK
#define MUART_IRQ_RX 0x01 // RX IRQ
#define MUART_IRQ_TX 0x02 // TX IRQ
#define MUART_IRQ_RTR 0x04 // RX timeout
#define MUART_IRQ_FER 0x08 // Stop bit error
#define MUART_IRQ_PER 0x08 // Parity bit error
#define MUART_IRQ_BER 0x10 // Line hangup error
#define MUART_IRQ_RER 0x20 // RX FIFO overflow
#define MUART_IRQ_TER 0x40 // TX FIFO overflow
#define MUART_IRQ_TTR 0x80 // TX CTS error

/*
 *Information about a serial port
 *
 * @base: Register base address
 */

struct muart_regs {
	unsigned long id; // 0x000
	unsigned long version; // 0x004
	unsigned long sw_rst; // 0x008
	unsigned long reserve_1; // 0x00C
	unsigned long gen_status; // 0x010
	unsigned long fifo_state; // 0x014
	unsigned long status; // 0x018
	unsigned long reserve_2; // 0x01C
	unsigned long dtrans; // 0x020
	unsigned long reserve_3; // 0x024
	unsigned long drec; // 0x028
	unsigned long reserve_4; // 0x02C
	unsigned long bdiv; // 0x030
	unsigned long reserve_5; // 0x034
	unsigned long reserve_6; // 0x038
	unsigned long reserve_7; // 0x03C
	unsigned long fifowm; // 0x040
	unsigned long ctrl; // 0x044
	unsigned long mask; // 0x048
	unsigned long rxtimeout; // 0x04C, Определяет тайм-аут,в битовых интервалах,при превышении которого при приёме данных возникает соответствующее прерывание
	unsigned long reserve_8; // 0x050  В наиболее частом случае посылка байта занимает 10 интервалов. DMA максимальный размер 16383 байта. Пока 200. На 9800 это ~ 20 мс.
	unsigned long txtimeout; // 0x054
	unsigned long reserve_58[42];
	unsigned long tx_enable_r;				// 0х0100, Включение/выключение канала
	unsigned long tx_suspend_r;				// 0х0104, Приостановка работы канала
	unsigned long tx_cancel_r;				// 0х0108, Остановка работы канала с точностью до дескриптора
	unsigned long reserve_10C;				// 0x010C 
	unsigned long tx_settings_r;			// 0x0110, Настройка канала
	unsigned long tx_irq_mask_r;			// 0x0114, Маска прерываний канала
	unsigned long tx_status_r;				// 0x0118, Состояние линии прерываний канала
	unsigned long reserve_11C;				// 0x011C
	unsigned long tx_desc_addr_r;			// 0x0120, Адрес первого дескриптора
	unsigned long reserve_124;				// 0x0124
	unsigned long tx_cur_desc_addr_r;		// 0x0128, Адрес текущего дескриптора
	unsigned long tx_cur_addr_r;			// 0x012C, Адрес текущей транзакции
	unsigned long tx_dma_state_r;			// 0x0130, Регистр состояния канала
	unsigned long reserve_134;				// 0x0134
	unsigned long reserve_138;				// 0x0138
	unsigned long reserve_13C;				// 0x013C
	unsigned long tx_desc_axlen_r;			// 0x0140
	unsigned long tx_desc_acache_r;			// 0x0144
	unsigned long tx_desc_aprot_r;			// 0x0148
	unsigned long tx_desc_alock_r;			// 0x014C
	unsigned long tx_desc_rresp_r;			// 0x0150
	unsigned long tx_desc_raxi_err_addr_r; 	// 0x0154
	unsigned long tx_desc_bresp_r;			// 0x0158
	unsigned long tx_desc_waxi_err_addr_r;	// 0x015C
	unsigned long tx_desc_permut_r;			// 0x0160
	unsigned long reserve_164;				// 0x0164
	unsigned long reserve_168;				// 0x0168
	unsigned long reserve_16C;				// 0x016C
	unsigned long reserve_170;				// 0x0170
	unsigned long reserve_174;				// 0x0174
	unsigned long reserve_178;				// 0x0178
	unsigned long reserve_17C;				// 0x017C 
	unsigned long tx_r_max_trans;			// 0x0180
	unsigned long tx_arlen;					// 0x0184
	unsigned long tx_arcache;				// 0x0188
	unsigned long tx_arprot;				// 0x018C
	unsigned long tx_arlock;				// 0x0190
	unsigned long tx_rresp;					// 0x0194
	unsigned long tx_raxi_err_addr;			// 0x0198
	unsigned long reserve_19C;				// 0x019C
	unsigned long tx_raxi_state;			// 0x01A0
	unsigned long tx_r_available_space;		// 0x01A4  
	unsigned long tx_r_permutation;			// 0x01A8  
	unsigned long reserve_1AC;				// 0x01AC
	unsigned long reserve_1B0;				// 0x01B0
	unsigned long reserve_1B4;				// 0x01B4
	unsigned long reserve_1B8;				// 0x01B8
	unsigned long reserve_1BC;				// 0x01BC
	unsigned long reserve_1C0[16];			// 0x01C0..0x01FF
	unsigned long rx_enable_w;				// 0x0200, Включение/выключение канала
	unsigned long rx_suspend_w;				// 0x0204, Приостановка работы канала
	unsigned long rx_cancel_w;				// 0x0208, Остановка работы канала с точностью до дескриптора
	unsigned long reserve_20C;				// 0x020C
	unsigned long rx_settings_w;			// 0x0210, Настройка канала
	unsigned long rx_irq_mask_w;			// 0x0214, Маска прерываний канала
	unsigned long rx_status_w;				// 0x0218, Состояние линии прерываний канала
	unsigned long reserve_21C;				// 0x021C
	unsigned long rx_desc_addr_w;			// 0x0220, Адрес первого дескриптора
	unsigned long reserve_224;				// 0x0224
};

// Custom flags
#define MDMA_MUART_FLAG_RTR 				(1<<19)	// 115,Ошибка таймаута
#define MDMA_MUART_FLAG_TFL 				(1<<18) // 114,Ошибка потери временной метки
#define MDMA_MUART_FLAG_OER 				(1<<17) // 113,Было переполнение FIFO
#define MDMA_MUART_FLAG_BER					(1<<16) // 112,Ошибка обрыва линии
#define MDMA_MUART_FLAG_PER 				(1<<15) // 111,Ошибка четности
#define MDMA_MUART_FLAG_FER 				(1<<14) // 110,Ошибка стопового бита

#define MUART_IRQ_ST_MSK_SUSPEND_DONE		(1<<0)	// Завершено выполнение приостановки MDMA
#define MUART_IRQ_ST_MSK_CANCEL_DONE		(1<<1)	// Завершена остановка MDMA (cancel) 
#define MUART_IRQ_ST_MSK_INT_DESC			(1<<2)	// Завершено выполнение дескриптора с флагом Int 
#define MUART_IRQ_ST_MSK_BAD_DESC			(1<<3)	// Считан дескриптор недоступный для MDMA (флаг ownership равен 1) 
#define MUART_IRQ_ST_MSK_STOP_DESC			(1<<4)	// Завершено выполнение дескриптора с флагом Stop 
#define MUART_IRQ_ST_MSK_DISCARD_DESC		(1<<5)	// Ошибка при обращении по шине AXI при чтении дескриптора 
#define MUART_IRQ_ST_MSK_WAXI_ERR			(1<<6)	// Ошибка при обращении по шине AXI при записи дескриптора 
#define MUART_IRQ_ST_MSK_AXI_ERR			(1<<7)	// Ошибка при обращении по шине AXI при чтении/записи данных 
#define MUART_IRQ_ST_MSK_START_BY_EVENT		(1<<8)	// Произошел запуск по внешнему событию 
#define MUART_IRQ_ST_MSK_IGNORE_EVENT		(1<<9)	// Событие, запускающее MDMA, произошло во время активной работы MDMA 
#define MUART_IRQ_ST_MSK_FULL				((1<<10)-1)

#define MUART_TX_DESC_CNT 8
#define MUART_RX_DESC_CNT 2
#define MAURT_DESC_GAP (sizeof(struct mdma_desc_long_ll))

#define MUART_TX_DESC_POOL_SIZE				(MUART_TX_DESC_CNT*sizeof(struct mdma_desc_long_ll))
#define MUART_RX_DESC_POOL_SIZE				(MUART_RX_DESC_CNT*sizeof(struct mdma_desc_long_ll))

#define MUART_MDMA_DESC_BUF_SIZE (1<<14)

#define MUART_MDMA_RX_TIMEOUT 5000 // в битовых интервалах
#define MUART_MDMA_TXRX_SETTINGS ((MAURT_DESC_GAP << 16) | (1<<4) | 2)

// MUART_GEN_STATUS
#define MUART_IRQ	0x01	// MUART interrupt
#define MDMA_RD_IRQ	0x02	// MDMA read channel interrupt  
#define MDMA_WR_IRQ	0x04	// MDMA write channel interrupt 

#define dev_dbg_(dev,...) printk(__VA_ARGS__);
#define DBGP printk("%s(%u)",__FILE__,__LINE__);

#define PRINT_ASYNC_DESCR(DESCR) printk("%s,%u(%s),Desc:hw_desc=%08x,memptr=%08x,fllen=%08x,buf=%08x,phys=%08x\n", \
										__FILE__, __LINE__,  __func__, \
										(unsigned int)DESCR->hw_desc, DESCR->hw_desc->memptr, DESCR->hw_desc->flags_length, \
										(unsigned int)DESCR->buf, DESCR->phys);
#define PRINT_BIT(OBJ,BIT) printk("%s:%u\n", #BIT, OBJ&BIT?1:0);
#define PRINT_DMA_STAT(ST) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_SUSPEND_DONE) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_CANCEL_DONE) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_INT_DESC) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_BAD_DESC) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_STOP_DESC) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_DISCARD_DESC) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_WAXI_ERR) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_AXI_ERR) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_START_BY_EVENT) \
	PRINT_BIT(ST, MUART_IRQ_ST_MSK_IGNORE_EVENT)
#define PRINT_CUSTOM_FLAGS(FL) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_RTR) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_TFL) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_OER) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_BER) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_PER) \
	PRINT_BIT(FL, MDMA_MUART_FLAG_FER) \
	printk("Length=%u\n", FL&(MUART_MDMA_DESC_BUF_SIZE-1))

#define PRINT_BUF(BUF) printk("Buffer=%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n", \
	((unsigned char*)BUF)[0], ((unsigned char*)BUF)[1], ((unsigned char*)BUF)[2], ((unsigned char*)BUF)[3], \
	((unsigned char*)BUF)[4], ((unsigned char*)BUF)[5], ((unsigned char*)BUF)[6], ((unsigned char*)BUF)[7]);

struct muart_port;

/* Deals with DMA transactions */

struct muart_sgbuf {
	struct scatterlist sg;
	char *buf;
};

struct muart_async_descriptor {
	void* buf;
	unsigned int off;
	struct mdma_desc_long_ll* hw_desc;
	dma_addr_t phys;
};

struct muart_dmarx_data {
	bool running;
	struct completion rx_suspend;
	struct completion rx_cancel;
	struct tasklet_struct rx_tasklet;
	int ready;
	int curr;
	struct mdma_desc_long_ll* hw_descr_pool;
	dma_addr_t hw_descr_pool_phys;
	struct muart_async_descriptor rx_descr[MUART_RX_DESC_CNT];
};

struct muart_dmatx_data {
	bool running;
	struct completion tx_suspend;
	struct completion tx_cancel;
	struct tasklet_struct tx_tasklet;
	int free;
	int sent;
	int ready;
	struct mdma_desc_long_ll* hw_descr_pool;
	dma_addr_t hw_descr_pool_phys;
	struct muart_async_descriptor tx_descr[MUART_TX_DESC_CNT];
};

struct muart_port {
	struct uart_port port;
	struct clk *clk;
	unsigned long baud;

#if defined(CONFIG_DMA_ENGINE) 
	/* DMA stuff */
	bool using_dma;
	bool loopback;
	struct muart_dmarx_data dmarx;
	struct muart_dmatx_data	dmatx;
	bool dma_probed;
#endif
};

static inline int muart_dma_probe(struct muart_port *uap);
static inline void muart_dma_remove(struct muart_port *uap);
static inline int muart_dma_startup(struct muart_port *uap);
static inline void muart_dma_shutdown(struct muart_port *uap);
static inline void muart_dma_tx_start_cmd(struct muart_regs* regs, struct muart_async_descriptor* tx_descr);
static inline void muart_dma_rx_start_cmd(struct muart_regs* regs, struct muart_async_descriptor* rx_descr);
static inline void muart_dma_tx_irq(struct muart_port *uap);
static inline void muart_dma_rx_irq(struct muart_port *uap);
static inline int muart_dma_tx_stop(struct muart_port *uap);
static inline int muart_dma_rx_stop(struct muart_port *uap);
static void muart_dma_rx_tasklet(unsigned long data);
static void muart_dma_tx_tasklet(unsigned long data);
static inline bool muart_dma_tx_start(struct muart_port *uap);

#define to_muart_port(uport) container_of(uport, struct muart_port, port)
static struct muart_port *muart_ports[CONFIG_SERIAL_MUART_NR_PORTS];

#ifdef CONFIG_SERIAL_MUART_CONSOLE
static struct console muart_console;
#endif

#define MUART_SERIAL_DEV_NAME "ttyRCM"

#define DRIVER_NAME "rcm-muart"

static struct uart_driver muart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = MUART_SERIAL_DEV_NAME,
	.major = 0,
	.minor = 0,
	.nr = CONFIG_SERIAL_MUART_NR_PORTS,
#ifdef CONFIG_SERIAL_MUART_CONSOLE
	.cons = &muart_console,
#endif
};

static inline void muart_enable_interrupts(struct uart_port *port,
					   unsigned int mask)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	writel(readl(&regs->mask) | mask, &regs->mask);
}

static inline void muart_disable_interrupts(struct uart_port *port,
					    unsigned int mask)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	writel(readl(&regs->mask) & ~mask, &regs->mask);
}

static void muart_stop_rx(struct uart_port *port)
{
	// disable rx irq
	// todo
}

static void muart_stop_tx(struct uart_port *port)
{
	// wait for finish transfer
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & MUART_FIFO_MASK) !=
	       0)
		cpu_relax();

	muart_disable_interrupts(port, MUART_IRQ_TX);

	//struct muart_port *muap =
	//    container_of(port, struct muart_port, port);		
	//muart_dma_tx_stop(muap);
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int muart_tx_empty(struct uart_port *port)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	if (((readl(&regs->fifo_state) >> MUART_TXFS_i) & MUART_FIFO_MASK) == 0)
		return TIOCSER_TEMT;

	return 0;
}

static void muart_rx_chars(struct uart_port *port) __releases(&port->lock)
	__acquires(&port->lock)
{
	unsigned int ch, flag;
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	// put everything that we have in fifo to tty
	while ((readl(&regs->fifo_state) >> MUART_RXFS_i) & MUART_FIFO_MASK) {
		ch = readl(&regs->drec) | MUART_DREG_DUMMY_RX;
		port->icount.rx++;
		flag = TTY_NORMAL;

		if (unlikely(ch & MUART_DREG_ERR)) {
			if (ch & MUART_DREG_BE) {
				ch &= ~(MUART_DREG_FE | MUART_DREG_PE);
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (ch & MUART_DREG_PE)
				port->icount.parity++;
			else if (ch & MUART_DREG_FE)
				port->icount.frame++;
			if (ch & MUART_DREG_OE)
				port->icount.overrun++;

			ch &= port->read_status_mask;

			if (ch & MUART_DREG_BE)
				flag = TTY_BREAK;
			else if (ch & MUART_DREG_PE)
				flag = TTY_PARITY;
			else if (ch & MUART_DREG_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch & 255))
			continue;

		uart_insert_char(port, ch, MUART_DREG_OE, ch, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}

/* internal function - shuold be used to send char to fifo */
static bool muart_tx_char(struct uart_port *port, unsigned char ch,
			  bool from_irq)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	if (unlikely(!from_irq)) {
		// check fifo state
		int symbols_in_fifo =
			(readl(&regs->fifo_state) >> MUART_TXFS_i) &
			MUART_FIFO_MASK;
		if (symbols_in_fifo > MUART_TX_FIFO_SIZE)
			return false;
	}

	writel(ch, &regs->dtrans);
	port->icount.tx++;

	return true;
}

/* internal driver procedure */
/* Returns true if tx interrupts have to be (kept) enabled  */
static bool muart_tx_chars(struct uart_port *port, bool from_irq)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct muart_port* uap = to_muart_port(port);

	if (unlikely(port->x_char)) {
		if (!muart_tx_char(port, port->x_char, from_irq))
			return true;
		port->x_char = 0;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		muart_stop_tx(port);
		return false;
	}

	if (uap->using_dma)
		return muart_dma_tx_start(uap);

	do {
		if (!muart_tx_char(port, xmit->buf[xmit->tail], from_irq))
			break;

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	} while (!uart_circ_empty(xmit));

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit)) {
		muart_stop_tx(port);
		return false;
	}
	return true;
}

/*
 * port is locked and interrupts are disabled
 * uart_start( ) calls us under the port spinlock irqsave
 */
static void muart_start_tx(struct uart_port *port)
{
//	struct muart_port *muap =
//	    container_of(port, struct muart_port, port);
	if (muart_tx_chars(port, false))
		muart_enable_interrupts(port, MUART_IRQ_TX);
}

static irqreturn_t muart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	unsigned int status = readl(&regs->status);

    unsigned int gen_status = readl(&regs->gen_status);
	struct muart_port* uap = to_muart_port(port);

	if (gen_status & MDMA_RD_IRQ) {
		spin_lock(&port->lock);
		dev_dbg_(port->dev, "MDMA_RD_IRQ interrupt\n");
		muart_dma_tx_irq(uap);
		spin_unlock(&port->lock);
		return IRQ_HANDLED;
	}
	if (gen_status & MDMA_WR_IRQ) {
		spin_lock(&port->lock);
		dev_dbg_(port->dev, "MDMA_WR_IRQ interrupt\n");
		muart_dma_rx_irq(uap);
		spin_unlock(&port->lock);
		return IRQ_HANDLED;
	}
	if (status & MUART_IRQ_RX) {
		spin_lock(&port->lock);
		muart_rx_chars(port);
		spin_unlock(&port->lock);
	}
	if (status & MUART_IRQ_TX) {
		muart_disable_interrupts(port, MUART_IRQ_TX);
		spin_lock(&port->lock);
		if (!uart_tx_stopped(port))
			if (muart_tx_chars(port, true))
				muart_enable_interrupts(port, MUART_IRQ_TX);

		spin_unlock(&port->lock);
	}
	return IRQ_HANDLED;
}

static unsigned int muart_get_mctrl(struct uart_port *port)
{
	/*
	 * Pretend we have a Modem status reg and following bits are
	 *  always set, to satify the serial core state machine
	 *  (DSR) Data Set Ready
	 *  (CTS) Clear To Send
	 *  (CAR) Carrier Detect
	 */
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void muart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct muart_port *uart = to_muart_port(port);
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	dev_dbg_(&port->dev, "%s\n",  __func__);

	if (uart->loopback)
		return;
	/* able to setup only LOOPBACK, but if only loopback not enabled in dts */
	if (mctrl & TIOCM_LOOP) {
		writel(readl(&regs->ctrl) | (1 << MUART_CTRL_LBE_i),
			&regs->ctrl);
	} else {
		writel(readl(&regs->ctrl) & ~(1 << MUART_CTRL_LBE_i),
			&regs->ctrl);
	}
}

static void muart_break_ctl(struct uart_port *port, int break_state)
{
	/* MUART doesn't support sending Break signal */
}

static int muart_startup(struct uart_port *port)
{
	struct muart_port *uart = to_muart_port(port);
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	int retval;

	dev_dbg_(port.dev, "%s\n",  __func__);

	retval = clk_prepare_enable(uart->clk);
	if (retval)
		return retval;

	uart->port.uartclk = clk_get_rate(uart->clk);

	// disable all interrupts
	writel(0, &regs->mask);
	// clear pending interrupts
	readl(&regs->status);
	readl(&regs->gen_status);

	retval = request_irq(port->irq, muart_isr, 0, "muart uart rx-tx", port);
	if (retval) {
		dev_warn(port->dev, "Unable to attach MUART UART intr\n");
		goto clk_dis;
	}

	// interrupt levels
	writel(((MUART_TX_FIFO_SIZE / 8) << MUART_FIFOWM_TXFS_i) |
		       (0 << MUART_FIFOWM_RXFS_i),
	       &regs->fifowm); // half of tx fifo, 1 byte rx

	// enable RX IRQ
	if (!uart->using_dma)
		muart_enable_interrupts(port, MUART_IRQ_RX);
	else
		muart_dma_startup(uart);

	return 0;

clk_dis:
	clk_disable_unprepare(uart->clk);
	return retval;
}

static void muart_shutdown(struct uart_port *port)
{
	struct muart_port *uart = to_muart_port(port);
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	dev_dbg_(port.dev, "%s\n",  __func__);

	// disable all interrupts
	writel(0, &regs->mask);
	// clear pending interrupts
	readl(&regs->status);
	readl(&regs->gen_status);

	if (uart->using_dma)
		muart_dma_shutdown(uart);

	free_irq(port->irq, port);
	clk_disable_unprepare(uart->clk);
}

static void muart_setup_status_masks(struct uart_port *port,
				     struct ktermios *termios)
{
	port->read_status_mask = MUART_DREG_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= MUART_DREG_FE | MUART_DREG_PE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= MUART_DREG_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= MUART_DREG_FE | MUART_DREG_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= MUART_DREG_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= MUART_DREG_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= MUART_DREG_DUMMY_RX;
}

static void muart_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	unsigned int baud;
	unsigned int N = 0;
	unsigned int divisor;
	unsigned int ctrl_value;
	unsigned long flags;
	struct muart_port* muap = to_muart_port(port);


	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk / 8);

	// there are two possible predividers 8 and 10 - choose best one to reach exact BR
	if (((port->uartclk / 8) % baud) != 0)
		if (((port->uartclk / 10) % baud) == 0)
			N = 1;

	divisor = port->uartclk / ((N ? 10 : 8) * baud);

	// todo DMA
	ctrl_value = (1 << MUART_CTRL_MEN_i) |	(1 << MUART_CTRL_RTS_POL); // Note: UART is assumed to be RTS active high.
	if (!muap->using_dma)
		ctrl_value |= (1 << MUART_CTRL_APB_MD_i); // APB mode,
	else
		ctrl_value |= (1 << MUART_CTRL_DUM_i); // data only
	if (muap->loopback) {
		ctrl_value |= (1 << MUART_CTRL_LBE_i);
		ctrl_value |= (1 << MUART_CTRL_CTSen_i) | (1 << MUART_CTRL_RTSen_i);
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		ctrl_value |= 0 << MUART_CTRL_WLEN_i;
		break;
	case CS6:
		ctrl_value |= 1 << MUART_CTRL_WLEN_i;
		break;
	case CS7:
		ctrl_value |= 2 << MUART_CTRL_WLEN_i;
		break;
	default: // CS8
		ctrl_value |= 3 << MUART_CTRL_WLEN_i;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		ctrl_value |= 2 << MUART_CTRL_STP2_i;
	else
		ctrl_value |= 0 << MUART_CTRL_STP2_i;

	if (termios->c_cflag & PARENB) {
		ctrl_value |= 1 << MUART_CTRL_PEN_i;
		if ((termios->c_cflag & PARODD))
			ctrl_value |= 1 << MUART_CTRL_EPS_i;

		if (termios->c_cflag & CMSPAR)
			ctrl_value |= 1 << MUART_CTRL_SPS_i;
	}

	if (termios->c_cflag & CRTSCTS)
		ctrl_value |=
			(1 << MUART_CTRL_CTSen_i) | (1 << MUART_CTRL_RTSen_i);

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	muart_setup_status_masks(port, termios);

	// disable MUART
	writel(0, &regs->ctrl);

	// set new baud rate
	writel((N << MUART_N_DIV) | (divisor << MUART_BAUD_DIV_i), &regs->bdiv);

	// enable MUART
	writel(ctrl_value, &regs->ctrl);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *muart_type(struct uart_port *port)
{
	return port->type == PORT_RCM ? DRIVER_NAME : NULL;
}

static void muart_release_port(struct uart_port *port)
{
}

static int muart_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int muart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != PORT_RCM)
		return -EINVAL;
	if (ser->baud_base < 50 || ser->baud_base > 12500000)
		return -EINVAL;
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void muart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_RCM;
		muart_request_port(port);
	}
}

static int muart_rs485_config(struct uart_port *port,
			      struct serial_rs485 *rs485)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	unsigned int ctrl = readl(&regs->ctrl);

	if (rs485->flags & SER_RS485_ENABLED) {
		/*
		 * RTS needs to be logic HIGH either during transer _or_ after
		 * transfer, other variants are not supported by the hardware.
		 */

		if (!(rs485->flags &
		      (SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND)))
			rs485->flags |= SER_RS485_RTS_ON_SEND;

		if (rs485->flags & SER_RS485_RTS_ON_SEND &&
		    rs485->flags & SER_RS485_RTS_AFTER_SEND)
			rs485->flags &= ~SER_RS485_RTS_AFTER_SEND;

		/*
		 * The hardware defaults to RTS logic HIGH while transfer.
		 * Switch polarity in case RTS shall be logic HIGH
		 * after transfer.
		 * Note: UART is assumed to be active high.
		 */
		if (rs485->flags & SER_RS485_RTS_ON_SEND)
			ctrl |= 1 << MUART_CTRL_RTS_POL;
		else if (rs485->flags & SER_RS485_RTS_AFTER_SEND)
			ctrl &= ~(1 << MUART_CTRL_RTS_POL);

		ctrl |= 1 << MUART_CTRL_MDS_i;
	} else {
		ctrl |= 1 << MUART_CTRL_MDS_i;
		ctrl |= 1
			<< MUART_CTRL_RTS_POL; // Note: UART is assumed to be active high.
	}

	writel(ctrl, &regs->ctrl);
	port->rs485 = *rs485;

	return 0;
}

#ifdef CONFIG_CONSOLE_POLL

static void muart_poll_putchar(struct uart_port *port, unsigned char chr)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & MUART_FIFO_MASK) >
	       1023)
		cpu_relax();

	writel(chr, &regs->dtrans);
}

static int muart_poll_getchar(struct uart_port *port)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	while (readl(&regs->fifo_state) >> MUART_RXFS_i) & MUART_FIFO_MASK == 0)
        cpu_relax();

	return readl(&regs->drec) & 0xff;
}

#endif /* CONFIG_CONSOLE_POLL */

static const struct uart_ops muart_serial_pops = {
	.tx_empty = muart_tx_empty,
	.set_mctrl = muart_set_mctrl,
	.get_mctrl = muart_get_mctrl,
	.stop_tx = muart_stop_tx,
	.start_tx = muart_start_tx,
	.stop_rx = muart_stop_rx,
	.break_ctl = muart_break_ctl,
	.startup = muart_startup,
	.shutdown = muart_shutdown,
	.set_termios = muart_set_termios,
	.type = muart_type,
	.release_port = muart_release_port,
	.request_port = muart_request_port,
	.config_port = muart_config_port,
	.verify_port = muart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char = muart_poll_putchar,
	.poll_get_char = muart_poll_getchar,
#endif
};

#ifdef CONFIG_SERIAL_MUART_CONSOLE
static int muart_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int ret;
	int baud = 6250000;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= CONFIG_SERIAL_MUART_NR_PORTS)
		co->index = 0;

	/*
	 * The uart port backing the console (e.g. ttyRCM1) might not have been
	 * init yet. If so, defer the console setup to after the port.
	 */
	port = &muart_ports[co->index]->port;
	if (!port->membase)
		return -ENODEV;

	ret = clk_prepare(muart_ports[co->index]->clk);
	if (ret)
		return ret;

	port->uartclk = clk_get_rate(muart_ports[co->index]->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	/*
	 * Serial core will call port->ops->set_termios( )
	 * which will set the baud reg
	 */
	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void muart_serial_console_putchar(struct uart_port *port, int ch)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & MUART_FIFO_MASK) >
	       1023)
		cpu_relax();

	writel(ch, &regs->dtrans);
}

/*
 * Interrupts are disabled on entering
 */
static void muart_serial_console_write(struct console *co, const char *s,
				       unsigned int count)
{
	struct uart_port *port = &muart_ports[co->index]->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	uart_console_write(port, s, count, muart_serial_console_putchar);
	spin_unlock_irqrestore(&port->lock, flags);
}

/**
 *	muart_console_match - non-standard console matching
 *	@co:	  registering console
 *	@name:	  name from console command line
 *	@idx:	  index from console command line
 *	@options: ptr to option string from console command line
 *
 *	Only attempts to match console command lines of the form:
 *	    console=muart,0x<addr>[,<options>]
 *	This form is used to register an initial earlycon boot console and
 *
 *	Returns 0 if console matches; otherwise non-zero to use default matching
 */
static int __init muart_console_match(struct console *co, char *name, int idx,
				      char *options)
{
	unsigned char iotype;
	resource_size_t addr;
	int i;

	printk("muart_console_match\n");

	if (strcmp(name, "muart") != 0)
		return -ENODEV;

	if (uart_parse_earlycon(options, &iotype, &addr, &options))
		return -ENODEV;

	if (iotype != UPIO_MEM32)
		return -ENODEV;

	/* try to match the port specified on the command line */
	for (i = 0; i < ARRAY_SIZE(muart_ports); i++) {
		struct uart_port *port;

		if (!muart_ports[i])
			continue;

		port = &muart_ports[i]->port;

		if (port->mapbase != addr)
			continue;

		co->index = i;
		port->cons = co;
		return muart_serial_console_setup(co, options);
	}
	return -ENODEV;
}

static struct console muart_console = { .name = MUART_SERIAL_DEV_NAME,
					.write = muart_serial_console_write,
					.device = uart_console_device,
					.setup = muart_serial_console_setup,
					.match = muart_console_match,
					.flags = CON_PRINTBUFFER,
					.index = -1,
					.data = &muart_driver };

static void muart_putc(struct uart_port *port, int c)
{
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	while ((readl(&regs->fifo_state) & 0x7ff0000) >= 0x3ff0000)
		;
	cpu_relax();

	writel(c, &regs->dtrans);
}

static void muart_early_write(struct console *con, const char *s,
			      unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, muart_putc);
}

static int __init muart_early_console_setup(struct earlycon_device *dev,
					    const char *opt)
{
	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = muart_early_write;

	return 0;
}
OF_EARLYCON_DECLARE(muart_uart, "rcm,muart", muart_early_console_setup);

#endif /* CONFIG_SERIAL_MUART_CONSOLE */

int muart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct muart_port *uart;
	struct uart_port *port;
	struct muart_regs *regs;
	int dev_id = -1;
	unsigned int i;
	unsigned int ctrl_value;

	printk("muart_probe %s\n", pdev->name);

	/* no device tree device */
	if (!np)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(muart_ports); i++) {
		if (muart_ports[i] == NULL) {
			dev_id = i;
			break;
		}
	}
	if (dev_id >= ARRAY_SIZE(muart_ports)) {
		dev_err(&pdev->dev, "serial%d out of range\n", dev_id);
		return -EINVAL;
	}

	uart = devm_kzalloc(&pdev->dev, sizeof(struct muart_port), GFP_KERNEL);

	port = &uart->port;

	uart->clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(uart->clk)) {
		dev_err(&pdev->dev, "%s: Unable to get parent clocks\n",
			pdev->name);
		return PTR_ERR(uart->clk);
	}

	port->membase = of_iomap(np, 0);
	if (!port->membase)
		/* No point of dev_err since UART itself is hosed here */
		return -ENXIO;

	regs = (struct muart_regs *)port->membase;
	if (readl(&regs->id) != MUART_ID ||
	    readl(&regs->version) != MUART_VERSION) {
		dev_err(&pdev->dev,
			"%s: error: Detected illegal version of uart core: 0x%08x 0x%08x\n",
			pdev->name, readl(&regs->id), readl(&regs->version));
		return -EINVAL;
	}

	dev_set_drvdata(&pdev->dev, uart);

	uart->using_dma = of_property_read_bool(np, "using-dma");
	uart->loopback = of_property_read_bool(np, "loopback");

	ctrl_value = ((0 << MUART_CTRL_EPS_i) | (0 << MUART_CTRL_STP2_i) | (3 << MUART_CTRL_WLEN_i) | (1 << MUART_CTRL_MEN_i)); // n prity,1 stop bit,8 data bit,enable
	if (!uart->using_dma)  
		ctrl_value |= (1 << MUART_CTRL_APB_MD_i); // APB mode,
	else
		ctrl_value |= (1 << MUART_CTRL_DUM_i); // data only
	if (uart->loopback) {
		ctrl_value |= (1 << MUART_CTRL_LBE_i);
		ctrl_value |= (1 << MUART_CTRL_CTSen_i) | (1 << MUART_CTRL_RTSen_i);
	}
	writel(ctrl_value, &regs->ctrl);

	port->irq = irq_of_parse_and_map(np, 0);

	port->dev = &pdev->dev;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->line = dev_id;
	port->ops = &muart_serial_pops;
	port->rs485_config = muart_rs485_config;

	port->fifosize = MUART_TX_FIFO_SIZE;

	/*
	 * uart_insert_char( ) uses it in decideding whether to ignore a
	 * char or not. Explicitly setting it here, removes the subtelty
	 */
	port->ignore_status_mask = 0;

	muart_ports[dev_id] = uart;

	return uart_add_one_port(&muart_driver, &muart_ports[dev_id]->port);
}

static int muart_remove(struct platform_device *pdev)
{
	// not needed for a while
	return 0;
}

static const struct of_device_id muart_dt_ids[] = { { .compatible =
							      "rcm,muart" },
						    { /* Sentinel */ } };
MODULE_DEVICE_TABLE(of, muart_dt_ids);

static struct platform_driver muart_platform_driver = {
	.probe = muart_probe,
	.remove = muart_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table  = muart_dt_ids,
	 },
};

static int __init muart_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: RC-Module MUART driver\n");

	ret = uart_register_driver(&muart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&muart_platform_driver);
	if (ret)
		uart_unregister_driver(&muart_driver);

	return ret;
}

static void __exit muart_exit(void)
{
	platform_driver_unregister(&muart_platform_driver);
	uart_unregister_driver(&muart_driver);
}

/*
 * While this can be a module, if builtin it's most likely the console
 * So let's leave module_exit but move module_init to an earlier place
 */
arch_initcall(muart_init);
module_exit(muart_exit);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RC Module MUART serial port driver");
MODULE_LICENSE("GPL");

static inline int muart_dma_alloc_async_descriptors(struct muart_port *uap)
{
	struct device* dev = uap->port.dev;

	dev_dbg_(dev,  __func__);

	uap->dmatx.hw_descr_pool = dma_alloc_coherent(dev, MUART_TX_DESC_POOL_SIZE,
													&uap->dmatx.hw_descr_pool_phys, GFP_KERNEL);
	if (!uap->dmatx.hw_descr_pool)
		goto err_alloc;
	
	uap->dmarx.hw_descr_pool = dma_alloc_coherent(dev, MUART_RX_DESC_POOL_SIZE,
													&uap->dmarx.hw_descr_pool_phys, GFP_KERNEL);

	if (!uap->dmarx.hw_descr_pool) {
		dma_free_coherent(dev, MUART_TX_DESC_POOL_SIZE,
							uap->dmatx.hw_descr_pool, uap->dmatx.hw_descr_pool_phys);
		goto err_alloc;
	}

	return 0;

err_alloc:
	dev_err(dev, "%s: dma_alloc_coherent failed!\n",  __func__);
	return -ENOMEM;
}

static inline void muart_dma_free_async_descriptors(struct muart_port *uap)
{
	struct device* dev = uap->port.dev;

	dev_dbg_(uap->port.dev,  __func__);

	dma_free_coherent(dev, MUART_RX_DESC_POOL_SIZE,
						uap->dmarx.hw_descr_pool, uap->dmarx.hw_descr_pool_phys);
	dma_free_coherent(dev, MUART_TX_DESC_POOL_SIZE,
						uap->dmatx.hw_descr_pool, uap->dmatx.hw_descr_pool_phys);

}

static inline int muart_dma_probe(struct muart_port *uap)
{
	int ret;

	// dev_dbg_(uap->port.dev,  __func__);

	ret = dma_set_mask_and_coherent(uap->port.dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(uap->port.dev, "%s: Set DMA mask failed (%d)\n", __func__, ret);
		return ret;
	}

	ret = muart_dma_alloc_async_descriptors(uap);
	if (ret) {
		dev_err(uap->port.dev, "%s: Alloc DMA memory failed (%d)\n", __func__, ret);
		return ret;
	}

	init_completion(&uap->dmarx.rx_suspend);
	init_completion(&uap->dmarx.rx_cancel);
	init_completion(&uap->dmatx.tx_suspend);
	init_completion(&uap->dmatx.tx_cancel);

	tasklet_init(&uap->dmarx.rx_tasklet, muart_dma_rx_tasklet, (unsigned long)uap);
	tasklet_init(&uap->dmatx.tx_tasklet, muart_dma_tx_tasklet, (unsigned long)uap);
	return 0;
}

static inline void muart_dma_remove(struct muart_port *uap)
{
	// dev_dbg_(muap->port.dev,  __func__);
	muart_dma_free_async_descriptors(uap);
	uap->dma_probed = false;
}

static inline void muart_free_rx_descritors(struct muart_port *uap)
{
	unsigned int i;
	struct device* dev = uap->port.dev;
	struct muart_async_descriptor* descr;

	for (i=0; i<MUART_RX_DESC_CNT; i++) {
		descr = &uap->dmarx.rx_descr[i];
		if (descr->buf) {
			dma_unmap_single(dev, descr->hw_desc->memptr, MUART_MDMA_DESC_BUF_SIZE, DMA_FROM_DEVICE);
			kfree(descr->buf);
			descr->buf = NULL;
		}
	}
}

static inline int muart_dma_startup(struct muart_port *uap)
{
	int ret;
	struct device* dev = uap->port.dev;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;
	struct muart_async_descriptor* descr;
	unsigned int i;

	dev_dbg_(uap->port.dev,  __func__);

	if (!uap->dma_probed) {
		ret = muart_dma_probe(uap);
		if (ret) {
			dev_err(dev, "MUART DMA startup failed\n");
			return ret;
		}
		uap->dma_probed = true;
	}

	writel(MUART_MDMA_TXRX_SETTINGS, &regs->rx_settings_w);
	writel(MUART_MDMA_TXRX_SETTINGS, &regs->tx_settings_r);
	writel(MUART_MDMA_RX_TIMEOUT, &regs->rxtimeout);

	for (i=0; i<MUART_TX_DESC_CNT; i++) {
		descr = &uap->dmatx.tx_descr[i];
		descr->hw_desc = &uap->dmatx.hw_descr_pool[i];
		descr->hw_desc->flags_length |= (MDMA_BD_INT | MDMA_BD_STOP | MDMA_BD_OWN);
		descr->phys = uap->dmatx.hw_descr_pool_phys + i*sizeof(struct mdma_desc_long_ll);
	}
	uap->dmatx.sent = -1;
	uap->dmatx.free = 0;
	uap->dmatx.ready = -1;

	for (i=0; i<MUART_RX_DESC_CNT; i++) {
		descr = &uap->dmarx.rx_descr[i];
		descr->hw_desc = &uap->dmarx.hw_descr_pool[i];
		descr->hw_desc->flags_length = MDMA_BD_OWN;
		descr->phys = uap->dmarx.hw_descr_pool_phys + i*sizeof(struct mdma_desc_long_ll);
		descr->buf = kmalloc(MUART_MDMA_DESC_BUF_SIZE, GFP_KERNEL);
		if (!descr->buf) {
			dev_err(dev, "%s: kmalloc error\n", __func__);
			goto error_startup;
		}
		descr->hw_desc->memptr = dma_map_single(dev, descr->buf, MUART_MDMA_DESC_BUF_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, descr->hw_desc->memptr)) {
			dev_err(dev, "%s: dma_mapping_error\n", __func__);
			goto error_startup;
		}
	}
	uap->dmarx.curr = 0;
	uap->dmarx.ready = -1;
	uap->dmarx.running = true;
	descr = &uap->dmarx.rx_descr[0];
	muart_dma_rx_start_cmd(regs, descr);
	return 0;

error_startup:
	muart_free_rx_descritors(uap);
	return -ENOMEM;
}

static inline int muart_dma_tx_stop(struct muart_port *uap)
{
	struct device* dev = uap->port.dev;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;

	dev_dbg_(muap->port.dev,  __func__);

	writel(1, &regs->tx_cancel_r);							// инициируем остановку
	if (wait_for_completion_timeout(&uap->dmatx.tx_cancel,	// подождем прерывания с флагом подтверждения остановки
									usecs_to_jiffies(50000)) == 0) {
		dev_err(dev, "Tx DMA cancel timeout\n");
		return -ETIMEDOUT;
	}
	uap->dmatx.running = false;
	return 0;
}

static inline int muart_dma_rx_stop(struct muart_port *uap)
{
	struct device* dev = uap->port.dev;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;

	dev_dbg_(muap->port.dev,  __func__);

	writel(1, &regs->rx_cancel_w);							// инициируем остановку
	if (wait_for_completion_timeout(&uap->dmarx.rx_cancel,	// подождем прерывания с флагом подтверждения остановки
									usecs_to_jiffies(5000000)) == 0) {
		dev_err(dev, "Rx DMA cancel timeout\n");
		return -ETIMEDOUT;
	}
	uap->dmarx.running = false;
	return 0;
}

static inline void muart_dma_reset(struct muart_port *uap)
{
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;
	writel(1, &regs->sw_rst);
	while(readl(&regs->sw_rst));
}

static inline void muart_dma_stop(struct muart_port *uap)
{
	muart_dma_tx_stop(uap);
	muart_dma_rx_stop(uap);			// как прервать ожидание приема, когда на входе еще нет первого стоп-бита? cancel не работает, но suspend работает
}

static inline void muart_dma_shutdown(struct muart_port *uap)
{
	//struct device* dev = uap->port.dev;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;

	dev_dbg_(muap->port.dev,  __func__);

	// muart_dma_stop(uap);
	muart_dma_reset(uap);

	readl(&regs->status);
	readl(&regs->gen_status);
	muart_free_rx_descritors(uap);
}

static inline void muart_dma_rx_start_cmd(struct muart_regs* regs, struct muart_async_descriptor* rx_descr)
{
	unsigned int mask = MUART_IRQ_ST_MSK_FULL;
	rx_descr->hw_desc->flags_length = MUART_MDMA_DESC_BUF_SIZE-1;
	rx_descr->hw_desc->flags_length |= (MDMA_BD_INT | MDMA_BD_STOP);
	rx_descr->hw_desc->flags_length &= ~MDMA_BD_OWN;
	PRINT_ASYNC_DESCR(rx_descr)
	writel(mask, &regs->rx_irq_mask_w);
	writel(rx_descr->phys, &regs->rx_desc_addr_w);
	readl(&regs->rx_status_w);
	writel(1, &regs->rx_enable_w);
}

static inline void muart_dma_tx_start_cmd(struct muart_regs* regs, struct muart_async_descriptor* tx_descr)
{
	unsigned int mask = MUART_IRQ_ST_MSK_FULL;
	PRINT_ASYNC_DESCR(tx_descr)
	writel(mask, &regs->tx_irq_mask_r);
	writel(tx_descr->phys, &regs->tx_desc_addr_r);
	readl(&regs->tx_status_r);
	writel(1, &regs->tx_enable_r);
}

static inline void muart_dma_tx_irq(struct muart_port *uap)
{
	unsigned int next;
	unsigned int status;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;
	struct muart_async_descriptor* tx_descr;

	dev_dbg_(uap->port.dev,  __func__);

	status = readl(&regs->tx_status_r);
	PRINT_DMA_STAT(status);

	if (status & MUART_IRQ_ST_MSK_STOP_DESC) {

		dev_dbg_(uap->port.dev, "%s entry: free %d, sent %d, descr %08x\n",  __func__, uap->dmatx.free, uap->dmatx.sent, readl(&regs->tx_cur_desc_addr_r));

		BUG_ON(uap->dmatx.sent == -1);

		BUG_ON(uap->dmatx.ready != -1);
	
		BUG_ON(!(uap->dmatx.tx_descr[uap->dmatx.sent].hw_desc->flags_length & MDMA_BD_OWN));	// дескриптор должен быть освобожден

		PRINT_ASYNC_DESCR((&uap->dmatx.tx_descr[uap->dmatx.sent]))

		uap->dmatx.ready = uap->dmatx.sent;
		next = ( uap->dmatx.sent+1) % MUART_TX_DESC_CNT;

		tx_descr = &uap->dmatx.tx_descr[next];

		if ((tx_descr->hw_desc->flags_length & MDMA_BD_OWN) == 0) { // todo: сделать аппаратно
			muart_dma_tx_start_cmd(regs, tx_descr);
			uap->dmatx.sent = next;
		}
		else
			uap->dmatx.sent = -1;

		dev_dbg_(uap->port.dev, "%s return: free %d, sent %d\n",  __func__, uap->dmatx.free, uap->dmatx.sent);

		tasklet_schedule(&uap->dmatx.tx_tasklet);
	}

	if (status &  MUART_IRQ_ST_MSK_SUSPEND_DONE) {
		complete(&uap->dmatx.tx_suspend);
	}

	if (status &  MUART_IRQ_ST_MSK_CANCEL_DONE) {
		complete(&uap->dmatx.tx_cancel);
	}
}

static void muart_dma_tx_tasklet(unsigned long data)
{
	unsigned long flags;
	struct muart_port* uap = (struct muart_port*)data;
	struct uart_port* port = &uap->port;
	struct device* dev = port->dev;
	//struct circ_buf* xmit = &port->state->xmit;
	struct muart_async_descriptor* descr;
	unsigned int length;

	dev_dbg_(muap->port.dev,  __func__);

	BUG_ON(uap->dmatx.ready == -1);

	spin_lock_irqsave(&port->lock, flags);
	descr = &uap->dmatx.tx_descr[uap->dmatx.ready];
	uap->dmatx.ready = -1;
	length = descr->hw_desc->flags_length & (MUART_MDMA_DESC_BUF_SIZE-1);
//	xmit->tail += length;
//	xmit->tail &= UART_XMIT_SIZE-1;
//	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
//		uart_write_wakeup(port);
	dma_unmap_single(dev, descr->hw_desc->memptr, length, DMA_TO_DEVICE);
	spin_unlock_irqrestore(&port->lock, flags);
}
////////////////////////////////// rework!!!
static inline bool muart_dma_tx_start(struct muart_port *uap)
{
	unsigned long flags;
	struct circ_buf *xmit;
	unsigned int len;
	struct uart_port* port = &uap->port;
	struct muart_async_descriptor* tx_descr;

	dev_dbg_(uap->port.dev,  __func__);

	dev_dbg_(uap->port.dev, "%s entry: free %d, sent %d\n",  __func__, uap->dmatx.free, uap->dmatx.sent);

	spin_lock_irqsave(&port->lock, flags);

	xmit = &uap->port.state->xmit;

	len = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (len > MUART_MDMA_DESC_BUF_SIZE-1) {
		dev_err(uap->port.dev, "%s: Too big write buffer " "(%d bytes).\n", __func__, len);
		spin_unlock_irqrestore(&port->lock, flags);
		return false; // todo-multiple descriptors
	}

	if (uap->dmatx.free == uap->dmatx.sent) {
		dev_err(uap->port.dev, "%s: Not free descriptors.\n", __func__);
		spin_unlock_irqrestore(&port->lock, flags);
		return false;
	}

	tx_descr = &uap->dmatx.tx_descr[uap->dmatx.free];
	tx_descr->buf = xmit->buf + xmit->tail;

	dev_dbg_(uap->port.dev,
			"xmit len %u (%02x,%02x,%02x,%02x..%02x,%02x,%02x,%02x)\n",
			len,
			((char*)(tx_descr->buf))[0], ((char*)(tx_descr->buf))[1], ((char*)(tx_descr->buf))[2], ((char*)(tx_descr->buf))[3],
			((char*)(tx_descr->buf))[len-4], ((char*)(tx_descr->buf))[len-3], ((char*)(tx_descr->buf))[len-2], ((char*)(tx_descr->buf))[len-1]);

	tx_descr->hw_desc->memptr = dma_map_single(uap->port.dev, tx_descr->buf, len, DMA_TO_DEVICE);
	tx_descr->hw_desc->flags_length = len;											// длина и флаги
	tx_descr->hw_desc->flags_length |= (MDMA_BD_INT | MDMA_BD_STOP);				// | MDMA_BD_INC);
	tx_descr->hw_desc->flags_length &= ~MDMA_BD_OWN;

	if (uap->dmatx.sent == -1) {
		uap->dmatx.sent = uap->dmatx.free;
		muart_dma_tx_start_cmd((struct muart_regs *)uap->port.membase, tx_descr);
	}

	xmit->tail += len;
	xmit->tail &= UART_XMIT_SIZE-1;
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	uap->dmatx.free = (uap->dmatx.free+1) % MUART_TX_DESC_CNT;

	spin_unlock_irqrestore(&port->lock, flags);

	dev_dbg_(uap->port.dev, "%s return: free %d, sent %d\n",  __func__, uap->dmatx.free, uap->dmatx.sent);

	return true;
}

static inline void muart_dma_rx_irq(struct muart_port *uap)
{
	unsigned int status;
	struct muart_regs* regs = (struct muart_regs*)uap->port.membase;
	struct muart_async_descriptor* ready_descr;
	struct muart_async_descriptor* next_descr;
	unsigned int flags_length;

	dev_dbg_(muap->port.dev,  __func__);

	status = readl(&regs->rx_status_w);
	PRINT_DMA_STAT(status);

	if (status & MUART_IRQ_ST_MSK_STOP_DESC) {
		ready_descr = &uap->dmarx.rx_descr[uap->dmarx.curr];
		flags_length = ready_descr->hw_desc->flags_length;
// remove after!!!
		PRINT_CUSTOM_FLAGS(flags_length);
		PRINT_ASYNC_DESCR(ready_descr)
		PRINT_BUF(ready_descr->buf)
		PRINT_BUF((char*)ready_descr->buf+(flags_length & (MUART_MDMA_DESC_BUF_SIZE-1))-8);

		if (flags_length & (MUART_MDMA_DESC_BUF_SIZE-1)) {
			uap->dmarx.ready = uap->dmarx.curr;
			tasklet_schedule(&uap->dmarx.rx_tasklet);
		}

		uap->dmarx.curr = (uap->dmarx.curr+1) % MUART_RX_DESC_CNT;
		next_descr = &uap->dmarx.rx_descr[uap->dmarx.curr];
		muart_dma_rx_start_cmd(regs, next_descr);
	}

	if (status &  MUART_IRQ_ST_MSK_SUSPEND_DONE) {
		complete(&uap->dmarx.rx_suspend);
	}

	if (status &  MUART_IRQ_ST_MSK_CANCEL_DONE) {
		complete(&uap->dmarx.rx_cancel);
	}
}

static void muart_dma_rx_tasklet(unsigned long data)
{
	unsigned long flags;
	struct muart_port* uap = (struct muart_port*)data;
	struct uart_port* port = &uap->port;
	struct device* dev = port->dev;
	//struct circ_buf* xmit = &port->state->xmit;
	struct tty_port* tty_port = &port->state->port;
	struct muart_async_descriptor* descr;
	unsigned int length;
	int room;

	dev_dbg_(dev,  __func__);

	BUG_ON(uap->dmarx.ready == -1);

	descr = &uap->dmarx.rx_descr[uap->dmarx.ready];
	length = descr->hw_desc->flags_length & (MUART_MDMA_DESC_BUF_SIZE-1);
	uap->dmarx.ready = -1;

	PRINT_BUF(descr->buf)
	PRINT_BUF((char*)descr->buf+length-8);

	room = tty_buffer_request_room(tty_port, length);
	dev_dbg_(dev, "Room %u, length %u\n", room, length);
	if (room < length)
		dev_dbg_(dev, "Rx overrun: dropping %u bytes\n",
			 length - room);
	if (!room)
		return;

	tty_insert_flip_string(tty_port, descr->buf, length);

	PRINT_BUF(descr->buf)
	PRINT_BUF((char*)descr->buf+length-8);

	spin_lock_irqsave(&port->lock, flags);
	tty_flip_buffer_push(tty_port);
	spin_unlock_irqrestore(&port->lock, flags);
}

#undef DEBUG