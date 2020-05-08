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
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>


#define MUART_ID 0x55415254
#define MUART_VERSION 0x10190

#define MUART_TX_FIFO_SIZE 0x3ff

//MUART_CTRL
#define MUART_MEN_i 0
#define MUART_LBE_i 1
#define MUART_APB_MD 2
#define MUART_MDS_i 3
#define MUART_RTSen_i 4
#define MUART_CTSen_i 5
#define MUART_RTS_POL 6
#define MUART_PEN_i 7
#define MUART_EPS_i 8
#define MUART_SPS_i 9
#define MUART_STP2_i 10
#define MUART_WLEN_i 12
#define MUART_DUM_i 15

//MUART_BDIV
#define MUART_BAUD_DIV_i 0
#define MUART_N_DIV 24


//MUART_FIFO_STATE
#define MUART_RXFS_i 0
#define MUART_TXFS_i 16

/*
 *Information about a serial port
 *
 * @base: Register base address
 */

struct muart_regs {
    unsigned long id;            // 0x000
    unsigned long version;       // 0x004
    unsigned long sw_rst;        // 0x008
    unsigned long reserve_1;     // 0x00C
    unsigned long gen_status;    // 0x010
    unsigned long fifo_state;    // 0x014
    unsigned long status;        // 0x018
    unsigned long reserve_2;     // 0x01C
    unsigned long dtrans;        // 0x020
    unsigned long reserve_3;     // 0x024
    unsigned long drec;          // 0x028
    unsigned long reserve_4;     // 0x02C
    unsigned long bdiv;          // 0x030
    unsigned long reserve_5;     // 0x034
    unsigned long reserve_6;     // 0x038
    unsigned long reserve_7;     // 0x03C
    unsigned long fifowm;        // 0x040
    unsigned long ctrl;          // 0x044
    unsigned long mask;          // 0x048
    unsigned long rxtimeout;     // 0x04C
    unsigned long reserve_8;     // 0x050
    unsigned long txtimeout;     // 0x054
    // dma control registers - not needed for a while
};

struct muart_port {
	struct uart_port port;
	struct clk	*clk;
	unsigned long baud;
};



#define to_muart_port(uport)  container_of(uport, struct muart_port, port)
static struct muart_port* muart_ports[CONFIG_SERIAL_MUART_NR_PORTS];

#ifdef CONFIG_SERIAL_MUART_CONSOLE
static struct console muart_console;
#endif

#define MUART_SERIAL_DEV_NAME	"ttyRCM"

#define DRIVER_NAME	"rcm-muart"

static struct uart_driver muart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= MUART_SERIAL_DEV_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		    = CONFIG_SERIAL_MUART_NR_PORTS,
#ifdef CONFIG_SERIAL_MUART_CONSOLE
	.cons		= &muart_console,
#endif
};

// int muart_getc(struct udevice *dev)
// {
// 	struct muart_priv *priv = dev_get_priv(dev);
//     struct muart_regs* regs = priv->regs;

//     int fifo_len = (ioread32(&regs->fifo_state) >> MUART_RXFS_i) & 0x7ff;
//     if(fifo_len == 0)
//         return -EAGAIN;
    
//     return ioread32(&regs->drec) & 0xff;    
// }

// int muart_putc(struct udevice *dev, const char ch)
// {
// 	struct muart_priv *priv = dev_get_priv(dev);
//     struct muart_regs* regs = priv->regs;
//     int fifo_len = (ioread32(&regs->fifo_state) >> MUART_TXFS_i) & 0x7ff;
    
//     if(fifo_len > 1023)
//     {
//         return -EAGAIN;
//     }

// 	iowrite32(ch, &regs->dtrans);

//     return 0;
// }

// int muart_pending(struct udevice *dev, bool input)
// {
// 	struct muart_priv *priv = dev_get_priv(dev);
//     struct muart_regs* regs = priv->regs;

//     int fifo_len = (ioread32(&regs->fifo_state) >> MUART_RXFS_i) & 0x7ff;

//     return fifo_len;
// }

// int muart_setbrg(struct udevice *dev, int baudrate)
// {
// 	struct muart_priv *priv = dev_get_priv(dev);
//     struct muart_regs* regs = priv->regs;
//     unsigned int N = 0;
//     unsigned int divisor;

//     if(priv->freq == 0)
//         return -EINVAL;

//     if(baudrate > (priv->freq/8))
//     {
//         printf("%s: error: unable to set requested baudrate %d - too fast\n", dev->name, baudrate);
//         return -EINVAL;
//     }

//     // there are two possible predividers 8 and 10 - choose best one to reach exact BR
//     if(((priv->freq / 8) % baudrate) != 0)
//         if(((priv->freq / 10) % baudrate) == 0)
//             N = 1;

//     divisor = priv->freq / ((N?10:8) * baudrate);

//     if(divisor > 0xFFFFFF)
//     {
//         printf("%s: error: unable to set requested baudrate %d - too slow\n", dev->name, baudrate);
//         return -EINVAL;
//     }

//     // disable MUART
//     iowrite32(ioread32(&regs->ctrl) & ~(1 << MUART_MEN_i), &regs->ctrl);

//     // set new baud rate
//     iowrite32((N<<MUART_N_DIV) | (divisor << MUART_BAUD_DIV_i), &regs->bdiv);

//     // enable MUART
//     iowrite32(ioread32(&regs->ctrl) | (1 << MUART_MEN_i), &regs->ctrl);

//     return 0;
// };


static void muart_stop_rx(struct uart_port *port)
{
    // disable rx irq
	// todo
}

static void muart_stop_tx(struct uart_port *port)
{
    // wait for finish transfer
    struct muart_regs* regs = (struct muart_regs*) port->membase;

    while (((readl(&regs->fifo_state) >> MUART_RXFS_i) & 0x7ff) != 0)
        cpu_relax();

    // tx irq dusable to do
}


/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int muart_tx_empty(struct uart_port *port)
{

    return 0;
}

/*
 * port is locked and interrupts are disabled
 * uart_start( ) calls us under the port spinlock irqsave
 */
static void muart_start_tx(struct uart_port *port)
{

}


/*
 * A note on the Interrupt handling state machine of this driver
 *
 * kernel printk writes funnel thru the console driver framework and in order
 * to keep things simple as well as efficient, it writes to UART in polled
 * mode, in one shot, and exits.
 *
 * OTOH, Userland output (via tty layer), uses interrupt based writes as there
 * can be undeterministic delay between char writes.
 *
 * Thus Rx-interrupts are always enabled, while tx-interrupts are by default
 * disabled.
 *
 * When tty has some data to send out, serial core calls driver's start_tx
 * which
 *   -checks-if-tty-buffer-has-char-to-send
 *   -writes-data-to-uart
 *   -enable-tx-intr
 *
 * Once data bits are pushed out, controller raises the Tx-room-avail-Interrupt.
 * The first thing Tx ISR does is disable further Tx interrupts (as this could
 * be the last char to send, before settling down into the quiet polled mode).
 * It then calls the exact routine used by tty layer write to send out any
 * more char in tty buffer. In case of sending, it re-enables Tx-intr. In case
 * of no data, it remains disabled.
 * This is how the transmit state machine is dynamically switched on/off
 */

static irqreturn_t muart_isr(int irq, void *dev_id)
{

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
	/* MCR not present */
}

static void muart_break_ctl(struct uart_port *port, int break_state)
{
	/* MUART doesn't support sending Break signal */
}

static int muart_startup(struct uart_port *port)
{
	/* Before we hook up the ISR, Disable all UART Interrupts */
	// ToDo

	if (request_irq(port->irq, muart_isr, 0, "muart uart rx-tx", port)) {
		dev_warn(port->dev, "Unable to attach MUART UART intr\n");
		return -EBUSY;
	}

	// enable RX IRQ
                     /* Only Rx IRQ enabled to begin with */

	return 0;
}

static void muart_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
}

static void
muart_set_termios(struct uart_port *port, struct ktermios *new,
		       struct ktermios *old)
{
	struct muart_port *uart = to_muart_port(port);

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
static int
muart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != PORT_RCM)
		return -EINVAL;

	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void muart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_RCM;
}

#ifdef CONFIG_CONSOLE_POLL

static void muart_poll_putchar(struct uart_port *port, unsigned char chr)
{
    struct muart_regs* regs = (struct muart_regs*) port->membase;
     
    while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & 0x7ff) > 1023)
        cpu_relax();
    
 	writel(chr, &regs->dtrans);
}

static int muart_poll_getchar(struct uart_port *port)
{
     struct muart_regs* regs = (struct muart_regs*) port->membase;

     while ( readl(&regs->fifo_state) >> MUART_RXFS_i) & 0x7ff == 0)
        cpu_relax();
    
     return readl(&regs->drec) & 0xff;    
}

#endif /* CONFIG_CONSOLE_POLL */


static const struct uart_ops muart_serial_pops = {
	.tx_empty	= muart_tx_empty,
	.set_mctrl	= muart_set_mctrl,
	.get_mctrl	= muart_get_mctrl,
	.stop_tx	= muart_stop_tx,
	.start_tx	= muart_start_tx,
	.stop_rx	= muart_stop_rx,
	.break_ctl	= muart_break_ctl,
	.startup	= muart_startup,
	.shutdown	= muart_shutdown,
	.set_termios	= muart_set_termios,
	.type		= muart_type,
	.release_port	= muart_release_port,
	.request_port	= muart_request_port,
	.config_port	= muart_config_port,
	.verify_port	= muart_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char = muart_poll_putchar,
	.poll_get_char = muart_poll_getchar,
#endif
};


#ifdef CONFIG_SERIAL_MUART_CONSOLE
static int muart_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
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
    struct muart_regs* regs = (struct muart_regs*) port->membase;
     
    while (((readl(&regs->fifo_state) >> MUART_TXFS_i) & 0x7ff) > 1023)
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


static struct console muart_console = {
	.name	= MUART_SERIAL_DEV_NAME,
	.write	= muart_serial_console_write,
	.device	= uart_console_device,
	.setup	= muart_serial_console_setup,
	.match	= muart_console_match,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &muart_driver
};

static void muart_putc(struct uart_port *port, int c)
{
    struct muart_regs* regs = (struct muart_regs*) port->membase;

	while ((readl(&regs->fifo_state) & 0x7ff0000) >= 0x3ff0000);
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
    printk("muart_early_console_setup\n");

	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = muart_early_write;

	return 0;
}
OF_EARLYCON_DECLARE(muart_uart, "rcm,muart", muart_early_console_setup);

#endif	/* CONFIG_SERIAL_MUART_CONSOLE */


// default values
// id:             0x55415254
// version:        0x00010190
// sw_rst:         0x00000000
// gen_status:     0x00000000
// fifo_state:     0x004e0000
// status:         0x00000000
// dtrans:         0x00000078
// drec:           0x00000000
// bdiv:           0x00000008
// fifowm:         0x02000200
// ctrl:           0x00003005
// mask:           0x00000000
// rxtimeout:      0x00000000
// txtimeout:      0x00000000

int muart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct muart_port *uart;
	struct uart_port *port;
	struct muart_regs *regs;
	int dev_id;
	unsigned int val, i;

    printk("muart_probe %s\n", pdev->name);

	/* no device tree device */
	if (!np)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(muart_ports); i++)
    {
		if (muart_ports[i] == NULL)
        {
			dev_id = i;
            break;
        }
    }
	if (dev_id >= ARRAY_SIZE(muart_ports)) {
		dev_err(&pdev->dev, "serial%d out of range\n", dev_id);
		return -EINVAL;
	}

    uart = devm_kzalloc(&pdev->dev, sizeof(struct muart_port),
            GFP_KERNEL);

	port = &uart->port;

	uart->clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(uart->clk))
    {
        dev_err(&pdev->dev, "%s: Unable to get parent clocks\n", pdev->name); 
		return PTR_ERR(uart->clk);
    }

	port->membase = of_iomap(np, 0);
	if (!port->membase)
		/* No point of dev_err since UART itself is hosed here */
		return -ENXIO;

    regs = (struct muart_regs*) port->membase;
    if(readl(&regs->id) != MUART_ID || readl(&regs->version) != MUART_VERSION)
    {
        dev_err(&pdev->dev, "%s: error: Detected illegal version of uart core: 0x%08x 0x%08x\n", pdev->name,
            readl(&regs->id), 
            readl(&regs->version));
        return -EINVAL;
    }

    // switch on muart with APB mode (no dma), 8n1 
    writel(
        (1 << MUART_MEN_i)  |   // enable
        (1 << MUART_APB_MD) |   // APB mode
        (0 << MUART_EPS_i)  |   // n prity
        (0 << MUART_STP2_i) |   // 1 stop bit
        (3 << MUART_WLEN_i),    // 8 data bit 
        &regs->ctrl);

	port->irq = irq_of_parse_and_map(np, 0);

	port->dev = &pdev->dev;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->line = dev_id;
	port->ops = &muart_serial_pops;

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
	struct muart_port *uart;// = (dev);
    int i;
	int busy = 0;

	uart_remove_one_port(&muart_driver, &uart->port);

	for (i = 0; i < ARRAY_SIZE(muart_ports); i++) {
		if (muart_ports[i] == uart)
			muart_ports[i] = NULL;
		else if (muart_ports[i])
			busy = 1;
	}

	if (busy == 0)
		uart_unregister_driver(&muart_driver);

	return 0;
}

static const struct of_device_id muart_dt_ids[] = {
	{ .compatible = "rcm,muart" },
	{ /* Sentinel */ }
};
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
