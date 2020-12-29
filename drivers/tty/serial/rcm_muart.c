// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 *  2020 Vladimir Shalyt <Vladimir.Shalyt@mir.dev>
 *  - DMA support
 */
//#define DEBUG
#include <linux/module.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqchip/chained_irq.h>

#ifdef CONFIG_BASIS_PLATFORM
#	undef CONFIG_SERIAL_MUART_CONSOLE
#endif

#include "rcm_muart.h"

#ifndef PORT_RCM
#	define PORT_RCM 123
#endif

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

// MUART_GEN_STATUS
#define MUART_IRQ 0x01 // MUART interrupt
#define MDMA_RD_IRQ 0x02 // MDMA read channel interrupt
#define MDMA_WR_IRQ 0x04 // MDMA write channel interrupt

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

static void muart_irq_eoi(struct irq_data *d)
{
}

static int muart_irq_set_type(struct irq_data *d, unsigned int type)
{
	if ( (d->hwirq >= 2) || 
	     ((type != IRQ_TYPE_LEVEL_HIGH) && (type != IRQ_TYPE_EDGE_RISING)) )
		return -EINVAL;

	return 0;
}

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
	struct muart_port *muap =
		container_of(port, struct muart_port, port);
	if (muap->using_dma) {
		muart_dma_stop_rx(muap);
		return;
	}
	// disable rx irq
	// todo
}

static void muart_stop_tx(struct uart_port *port)
{
	struct muart_port *muap =
		container_of(port, struct muart_port, port);
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	if (muap->using_dma) {
//		if (muap->rx.chan)
//			dmaengine_terminate_async(muap->rx.chan);
		return;
	}
	// wait for finish transfer
	while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & MUART_FIFO_MASK) !=
		0)
		cpu_relax();

	muart_disable_interrupts(port, MUART_IRQ_TX);
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

	if (unlikely(port->x_char)) {
		if (!muart_tx_char(port, port->x_char, from_irq))
			return true;
		port->x_char = 0;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		muart_stop_tx(port);
		return false;
	}

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
	struct muart_port *uap = container_of(port, struct muart_port, port);
	
	if (uap->using_dma) {
		muart_dma_tx_start(uap);
		return;
	}

	if (muart_tx_chars(port, false))
		muart_enable_interrupts(port, MUART_IRQ_TX);
}

static irqreturn_t muart_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	unsigned int status = readl(&regs->status);

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

static void muart_chained_handler(struct irq_desc *desc)
{
	struct uart_port *port = irq_desc_get_handler_data(desc);
	struct muart_regs *regs = (struct muart_regs *)port->membase;
	struct muart_port *uart = to_muart_port(port);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned irq = 0;

	unsigned int __maybe_unused status = readl(&regs->status);
	unsigned int gen_status = readl(&regs->gen_status);

	chained_irq_enter(chip, desc);

	if (gen_status & MDMA_RD_IRQ) {
		irq = irq_find_mapping(uart->domain, 0);
		if (irq != 0)
			generic_handle_irq(irq);
	}

	if (gen_status & MDMA_WR_IRQ) {
		irq = irq_find_mapping(uart->domain, 1);
		if (irq != 0)
			generic_handle_irq(irq);
	}

	chained_irq_exit(chip, desc);
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
	struct muart_regs *regs = (struct muart_regs *)port->membase;

	/* able to setup only LOOPBACK */
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

	retval = clk_prepare_enable(uart->clk);
	if (retval)
		return retval;

#ifndef CONFIG_BASIS_PLATFORM
	uart->port.uartclk = clk_get_rate(uart->clk);
#else
	uart->port.uartclk = uart->uartclk;
#endif

	// disable all interrupts
	writel(0, &regs->mask);
	// clear pending interrupts
	readl(&regs->status);
	readl(&regs->gen_status);

	if (!uart->using_dma) {
		retval = request_irq(port->irq, muart_isr, 0,
		                     "muart uart rx-tx", port);
		if (retval) {
			dev_warn(port->dev, "Unable to attach MUART UART intr\n");
			goto clk_dis;
		}
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

	// disable all interrupts
	writel(0, &regs->mask);
	// clear pending interrupts
	readl(&regs->status);
	readl(&regs->gen_status);

	if (uart->using_dma)
		muart_dma_shutdown(uart);
	else
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

	ctrl_value = (1 << MUART_CTRL_MEN_i) |	(1 << MUART_CTRL_RTS_POL); // Note: UART is assumed to be RTS active high.

	if ((!muap->using_dma) || (!muap->tx.chan))
		ctrl_value |= (1 << MUART_CTRL_APB_MD_i); // APB mode,
	else
		ctrl_value |= (1 << MUART_CTRL_DUM_i); // data only

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

static int muart_create_irq_domain(struct muart_port *uart, struct device *dev)
{
	int res;
	struct irq_chip_generic *gc;

	uart->domain = irq_domain_create_linear(dev->fwnode, 2,
	                                        &irq_generic_chip_ops,
	                                        NULL);
	if (!uart->domain) {
		dev_err(dev, "couldn't create irq domain.\n");
		return -EINVAL;
	}

	res = irq_alloc_domain_generic_chips(
		uart->domain, 2, 1, "rcm-muart",
		handle_fasteoi_irq, IRQ_NOREQUEST | IRQ_NOPROBE,
		0, 0);
	if (res) {
		dev_err(dev, "couldn't allocate irq chips.\n");
		return res;
	}

	gc = irq_get_domain_generic_chip(uart->domain, 0);
	gc->private = uart;

	gc->chip_types[0].type              = IRQ_TYPE_SENSE_MASK;
	gc->chip_types[0].chip.irq_eoi      = muart_irq_eoi;
	gc->chip_types[0].chip.irq_set_type = muart_irq_set_type;
	gc->chip_types[0].chip.name         = "rcm-muart";

	gc->chip_types[0].chip.flags        = IRQCHIP_EOI_IF_HANDLED;

	irq_set_chained_handler_and_data(uart->port.irq,
	                                 muart_chained_handler, &uart->port);

#ifdef CONFIG_BASIS_PLATFORM
	uart->device->priv = uart->domain;
#endif

	return res;
}

#ifdef CONFIG_BASIS_PLATFORM

static void muart_unbind(struct basis_device *device)
{
	struct muart_port *uart = basis_device_get_drvdata(device);

	muart_free_dma(uart);

	if (uart->domain)
		irq_domain_remove(uart->domain);

	if (uart->port.irq) {
		irq_dispose_mapping(uart->port.irq);
		uart->port.irq = 0;
	}
}

static int muart_bind(struct basis_device *device)
{
	struct muart_port *uart = basis_device_get_drvdata(device);
	struct device *dev = &device->dev;
	struct uart_port *port;
	struct muart_regs *regs;
	int dev_id = -1;
	unsigned int i;
	unsigned int ctrl_value;
	int err;

	dev_info(dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	if (WARN_ON_ONCE(!device))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(muart_ports); i++) {
		if (muart_ports[i] == NULL) {
			dev_id = i;
			break;
		}
	}
	if (dev_id >= ARRAY_SIZE(muart_ports)) {
		dev_err(dev, "serial%d out of range\n", dev_id);
		return -EINVAL;
	}

	port = &uart->port;

	port->membase = ioremap(uart->reg + device->controller->ep_base_phys,
	                        uart->reg_size);
	if (!port->membase)
		/* No point of dev_err since UART itself is hosed here */
		return -ENXIO;

	regs = (struct muart_regs *)port->membase;
	if (readl(&regs->id) != MUART_ID ||
	    readl(&regs->version) != MUART_VERSION) {
		dev_err(dev,
			"error: Detected illegal version of uart core: 0x%08x 0x%08x\n",
			readl(&regs->id), readl(&regs->version));
		return -EINVAL;
	}

	if (uart->using_dma)
		dev_info(dev, "DMA support enabled\n");

	// n prity,1 stop bit,8 data bit,enable
	ctrl_value = ((0 << MUART_CTRL_EPS_i)  |
	              (0 << MUART_CTRL_STP2_i) |
	              (3 << MUART_CTRL_WLEN_i) |
	              (1 << MUART_CTRL_MEN_i)  |
	              (1 << MUART_CTRL_APB_MD_i));

	writel(ctrl_value, &regs->ctrl);

	port->irq = irq_create_mapping(device->controller->domain,
	                               uart->hwirq);

	if (port->irq == 0) {
		dev_warn(dev, "Failed to map irq #%u.\n", uart->hwirq);
	}

	if (uart->using_dma) {
		err = muart_create_irq_domain(uart, dev);
		if (err)
			return err;
	}

	port->dev = dev;
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

	if (uart->using_dma) {
		err = muart_alloc_dma(uart);
		if (err != 0) 
			return err;
	}

	return uart_add_one_port(&muart_driver, &muart_ports[dev_id]->port);
}

static int muart_probe(struct basis_device *device)
{
	struct muart_port *uart;
	struct device *dev = &device->dev;

	uart = devm_kzalloc(dev, sizeof(*uart), GFP_KERNEL);
	if (!uart)
		return -ENOMEM;

	uart->device = device;

	basis_device_set_drvdata(device, uart);

	return 0;
}

static int muart_remove(struct basis_device *device)
{
	return 0;
}
#else /* CONFIG_BASIS_PLATFORM */
int muart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct muart_port *uart;
	struct uart_port *port;
	struct muart_regs *regs;
	int dev_id = -1;
	unsigned int i;
	unsigned int ctrl_value;
	int err;

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
	if (uart->using_dma)
		dev_info(&pdev->dev, "DMA support enabled\n");

	 // n prity,1 stop bit,8 data bit,enable
	ctrl_value = ((0 << MUART_CTRL_EPS_i)  |
	              (0 << MUART_CTRL_STP2_i) |
	              (3 << MUART_CTRL_WLEN_i) |
	              (1 << MUART_CTRL_MEN_i)  |
	              (1 << MUART_CTRL_APB_MD_i));

	writel(ctrl_value, &regs->ctrl);

	port->irq = irq_of_parse_and_map(np, 0);

	if (uart->using_dma) {
		err = muart_create_irq_domain(uart, &pdev->dev);
		if (err)
			return err;
	}

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

	if (uart->using_dma) {
		err = muart_alloc_dma(uart);
		if (err != 0) 
			return err;
	}

	return uart_add_one_port(&muart_driver, &muart_ports[dev_id]->port);
}

static int muart_remove(struct platform_device *pdev)
{
	struct muart_port *uart = dev_get_drvdata(&pdev->dev);

	muart_free_dma(uart);

	if (uart->domain)
		irq_domain_remove(uart->domain);

	return 0;
}
#endif /* CONFIG_BASIS_PLATFORM */

#ifdef CONFIG_BASIS_PLATFORM
static const struct basis_device_id muart_ids[] = {
	{
		.name = "muart",
	},
	{},
};

static struct basis_device_ops muart_ops = {
	.unbind = muart_unbind,
	.bind   = muart_bind,
};

BASIS_DEV_ATTR_U32_SHOW(muart_,  reg,       struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, reg,       struct muart_port);

BASIS_DEV_ATTR_U32_SHOW(muart_,  reg_size,  struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, reg_size,  struct muart_port);

BASIS_DEV_ATTR_U32_SHOW(muart_,  hwirq,     struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, hwirq,     struct muart_port);

BASIS_DEV_ATTR_U32_SHOW(muart_,  uartclk,   struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, uartclk,   struct muart_port);

BASIS_DEV_ATTR_STR_SHOW(muart_,  dma_dev,   struct muart_port);
BASIS_DEV_ATTR_STR_STORE(muart_, dma_dev,   struct muart_port);

BASIS_DEV_ATTR_U32_SHOW(muart_,  tx_ch_num, struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, tx_ch_num, struct muart_port);

BASIS_DEV_ATTR_U32_SHOW(muart_,  rx_ch_num, struct muart_port);
BASIS_DEV_ATTR_U32_STORE(muart_, rx_ch_num, struct muart_port);

static ssize_t muart_using_dma_store(struct config_item *item,
                                     const char *page, size_t len)
{
	struct basis_device *device = config_item_to_basis_device(item);
	struct muart_port *uart = basis_device_get_drvdata(device);
	int ret;
	bool using_dma;

	ret = kstrtobool(page, &using_dma);
	if (ret)
		return ret;

	uart->using_dma = using_dma;

	return len;
}

static ssize_t muart_using_dma_show(struct config_item *item, char *page)
{
	struct basis_device *device = config_item_to_basis_device(item);
	struct muart_port *uart = basis_device_get_drvdata(device);
	return sprintf(page, "%d\n", (int)uart->using_dma);
}

CONFIGFS_ATTR(muart_, reg);
CONFIGFS_ATTR(muart_, reg_size);
CONFIGFS_ATTR(muart_, hwirq);
CONFIGFS_ATTR(muart_, uartclk);
CONFIGFS_ATTR(muart_, using_dma);
CONFIGFS_ATTR(muart_, dma_dev);
CONFIGFS_ATTR(muart_, tx_ch_num);
CONFIGFS_ATTR(muart_, rx_ch_num);

static struct configfs_attribute *muart_attrs[] = {
	&muart_attr_reg,
	&muart_attr_reg_size,
	&muart_attr_hwirq,
	&muart_attr_uartclk,
	&muart_attr_using_dma,
	&muart_attr_dma_dev,
	&muart_attr_tx_ch_num,
	&muart_attr_rx_ch_num,
	NULL,
};

static struct basis_device_driver muart_basis_driver = {
	.driver.name    = DRIVER_NAME,
	.probe          = muart_probe,
	.remove         = muart_remove,
	.id_table       = muart_ids,
	.ops            = &muart_ops,
	.owner          = THIS_MODULE,
	.attrs          = muart_attrs,
};
#else
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
#endif

static int __init muart_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: RC-Module MUART driver\n");

	ret = uart_register_driver(&muart_driver);
	if (ret)
		return ret;

#ifdef CONFIG_BASIS_PLATFORM
	ret = basis_device_register_driver(&muart_basis_driver);
#else
	ret = platform_driver_register(&muart_platform_driver);
#endif
	if (ret)
		uart_unregister_driver(&muart_driver);

	return ret;
}

static void __exit muart_exit(void)
{
#ifdef CONFIG_BASIS_PLATFORM
	basis_device_unregister_driver(&muart_basis_driver);
#else
	platform_driver_unregister(&muart_platform_driver);
#endif
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
