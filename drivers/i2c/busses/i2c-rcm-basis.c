/* i2c-rcm.c: I2C bus driver for RC "MODULE" I2C controller.
 *
 * Alexey Ivanov <ivanov.a1exey@yandex.ru>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>

#ifdef CONFIG_BASIS_PLATFORM
#	include "../../misc/rcm/basis/basis-device.h"
#	include "../../misc/rcm/basis/basis-controller.h"
#	include "../../misc/rcm/basis/basis-cfs.h"
#endif

/*
 * Registers description.
 */
#define RCM_I2C_ID	0x00			/* Core Identifier register */

#define RCM_I2C_ISR	0x04			/* Interrupt Status Register */
#define		RCM_I2C_ISR_DNE		BIT(0)	/* One byte transaction done */
#define		RCM_I2C_ISR_ARB		BIT(1)	/* Arbitration lost */
#define		RCM_I2C_ISR_TXE		BIT(2)	/* TX FIFO EMPTY */
#define		RCM_I2C_ISR_RXF		BIT(3)	/* RX FIFO FULL */
#define		RCM_I2C_ISR_TXNE	BIT(4)	/* TX FIFO nearly empty */
#define		RCM_I2C_ISR_RXNE	BIT(5)	/* RX FIFO nearly full */
#define		RCM_I2C_ISR_NACK	BIT(6)	/* No ACK */
#define RCM_I2C_IER	0x08			/* Interrupt Enable Register */
#define		RCM_I2C_IER_DNE		BIT(0)	/* Enable DNE IRQ */
#define		RCM_I2C_IER_ARB		BIT(1)	/* Enable ARB LOSR IRQ */
#define		RCM_I2C_IER_TXE		BIT(2)	/* Enable TX FIFO EPMTY IRQ */
#define		RCM_I2C_IER_RXF		BIT(3)	/* Enable RX FIFO FULL IRQ */
#define		RCM_I2C_IER_TXNE	BIT(4)	/* Enable TX FIFO NEMPRY IRQ */
#define		RCM_I2C_IER_RXNE	BIT(5)	/* Enable RX FIFO NEMPTY IRQ */
#define		RCM_I2C_IER_NACK	BIT(6)	/* Enable NACK IRQ */
#define RCM_I2C_SFTR	0x0C			/* Soft Reset Register */
#define RCM_I2C_CTRL	0x10			/* Control Register */
#define		RCM_I2C_CTRL_EN		BIT(0)	/* Enable I2C controller */
#define		RCM_I2C_CTRL_START	BIT(1)	/* Send START condition */
#define		RCM_I2C_CTRL_FIFO_RST	BIT(2)	/* Reset TX FIFO */
#define		RCM_I2C_CTRL_R		BIT(3)	/* Read command */
#define		RCM_I2C_CTRL_W		BIT(4)	/* Write command */
#define		RCM_I2C_CTRL_REPEAT	BIT(5)	/* Send REPEAT START */
#define		RCM_I2C_CTRL_STOP	BIT(6)	/* Send STOP cindition */
#define RCM_I2C_SR	0x18			/* Status Register */
#define		RCM_I2C_SR_BSY		BIT(0)	/* I2C bus busy */
#define		RCM_I2C_SR_ARB		BIT(1)	/* Arbitration lost */
#define		RCM_I2C_SR_DNE		BIT(2)	/* One byte complete trans */
#define		RCM_I2C_SR_RXNF		BIT(3)	/* RX FIFO NEARLY FULL */
#define		RCM_I2C_SR_TXE		BIT(4)	/* TX FIFO EMPTY */
#define		RCM_I2C_SR_RXF		BIT(5)	/* RX FIFO FULL */
#define		RCM_I2C_SR_TXF		BIT(6)	/* TX FIFO FULL */
#define		RCM_I2C_SR_RXE		BIT(7)	/* RX FIFO EMPTY */
#define		RCM_I2C_SR_NACK		BIT(9)	/* Not Acknowledge */
#define RCM_I2C_TX	0x30			/* TX FIFO */
#define RCM_I2C_NMBR	0x1C			/* Number of bytes to receive */
#define RCM_I2C_RX	0x430			/* RX FIFO */
#define RCM_I2C_CLK	0x24			/* Clock Prescale Register*/
#define RCM_I2C_FILL	0x28			/* FIFO FILL SET */
#define RCM_I2C_RES_STAT	0x20		/* Reset Status Register */

#define RCM_I2C_IRQ_MASK	(RCM_I2C_IER_ARB | RCM_I2C_IER_TXE	\
				| RCM_I2C_IER_NACK)

#define RCM_I2C_TIMEOUT		100000
#define RCM_I2C_XFER_TIMEOUT	(msecs_to_jiffies(500))

#define FIRST_MESS	1

#define FIFO_SIZE_TX	250
#define FIFO_SIZE_RX	250

#define FILL_RX(X)	((X) << 16)
#define FILL_TX(X)	((X) << 0)

int presc = -1;

module_param(presc, int, S_IRUGO | S_IWUSR);

/*
 * rcm_i2c - I2C device context
 * @base: pointer to register struct
 * @msg: pointer to current message
 * @mlen: number of bytes transferred in msg
 * @dev: device reference
 * @adap: i2c core abstraction
 * @msg_complete: xfer completion object
 * @clk: reference for i2c input clock
 * @err: error occured
 * @num: number of message to transfer
 * @curr: current number of transfer massage
 * @buf: ptr to msg buffer
 * @bus_clock: current i2c bus clock rate
 * @lock: spinlock for IRQ synchronization
 */
struct rcm_i2c {
	void __iomem *base;
	struct i2c_msg *msg;
	size_t mlen;
	struct device *dev;
	struct i2c_adapter adap;
	struct completion msg_complete;
	struct clk *clk;
	int err;
	int num;
	int curr;
	u32 addr;
	u8 *buf;
	u32 bus_clock;
	spinlock_t lock;

#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device    *device;
	u32                     regs;
	u32                     regs_size;
	u32                     hwirq;
	u32                     i2c_clk;
#endif
};

static const struct of_device_id rcm_i2c_match[] = {
	{
		.compatible = "rcm,i2c-bc048",
	},
	{
		.compatible = "atmel,24c64",
	},
	{
		.compatible = "microchip,24c64",
	},
	{}
};

static inline void i2c_write(uint32_t value, void *base, uint32_t addr)
{
	writel(value, base + addr);

#if defined DEBUG
	//dev_dbg(rdev->dev, "iowrite32(0x%x, base + 0x%x);\n", value, addr);
#endif
}

static inline uint32_t i2c_read(void *base, uint32_t addr)
{
	uint32_t reg =  readl(base + addr);

#if defined DEBUG
	//dev_dbg(rdev->dev, "/* ioread32(base + 0x%x) == 0x%x */\n", addr, reg);
#endif
	return reg;
}

static void
rcm_i2c_int_enable(struct rcm_i2c *rdev, u32 mask, bool enable)
{
	unsigned long flag;
	u32 int_reg;

	spin_lock_irqsave(&rdev->lock, flag);

	int_reg = i2c_read(rdev->base, RCM_I2C_IER);

	if (enable)
		int_reg = mask;
	else
		int_reg &= ~mask;

	i2c_write(int_reg, rdev->base, RCM_I2C_IER);

	spin_unlock_irqrestore(&rdev->lock, flag);
}

static void rcm_i2c_int_clear(struct rcm_i2c *rdev, u32 mask)
{
	u32 int_en = i2c_read(rdev->base, RCM_I2C_ISR);
	int_en = i2c_read(rdev->base, RCM_I2C_ISR);
}

void rcm_i2c_res_stat(struct rcm_i2c *rdev)
{
	i2c_write(0x01, rdev->base, RCM_I2C_RES_STAT);
}

void rcm_i2c_reset(struct rcm_i2c *rdev)
{
	i2c_write(0xFF, rdev->base, RCM_I2C_SFTR);
}

void rcm_i2c_set_stat(struct rcm_i2c *rdev)
{
	i2c_write(0x00, rdev->base, RCM_I2C_RES_STAT);
}

static void rcm_i2c_disable(struct rcm_i2c *rdev)
{
	u32 tmp = i2c_read(rdev->base, RCM_I2C_CTRL);

	tmp &= ~(RCM_I2C_CTRL_START | RCM_I2C_CTRL_REPEAT |
		 RCM_I2C_CTRL_STOP | RCM_I2C_CTRL_W | RCM_I2C_CTRL_R);

	i2c_write(tmp, rdev->base, RCM_I2C_CTRL);
}

static void rcm_i2c_transfer(struct rcm_i2c *rdev, u32 data)
{
	i2c_write(data, rdev->base, RCM_I2C_TX);
}

static void fill_tx_fifo(struct rcm_i2c *rdev)
{
	size_t tx_fifo_avail = FIFO_SIZE_TX;
	int bytes_to_transfer = min(tx_fifo_avail, rdev->mlen);

	while (bytes_to_transfer-- > 0) {
		rcm_i2c_transfer(rdev, *rdev->buf++);
		rdev->mlen--;
	}
}

static void fill_rx_fifo(struct rcm_i2c *rdev)
{
	size_t rx_fifo_avail = FIFO_SIZE_RX/2;
	int receive = min(rx_fifo_avail, rdev->mlen);

	while (receive-- > 0) {
		*rdev->buf++ = i2c_read(rdev->base, RCM_I2C_RX);
		rdev->mlen--;
	}
}

int rcm_i2c_wait_bus_idle(struct rcm_i2c *rdev)
{
	unsigned long stop_time;
	u32 status;

	stop_time = jiffies + msecs_to_jiffies(100) + 1;
	do {
		status = i2c_read(rdev->base, RCM_I2C_SR);

		if (!(status & RCM_I2C_SR_BSY))
			return 0;

		usleep_range(50, 200);
	} while (time_before(jiffies, stop_time));

	return -EBUSY;
}

void rcm_i2c_snd_stop(struct rcm_i2c *rdev)
{
	u32 control = i2c_read(rdev->base, RCM_I2C_CTRL);

	i2c_write(control | RCM_I2C_CTRL_STOP, rdev->base, RCM_I2C_CTRL);
}

void rcm_i2c_clear_start(struct rcm_i2c *rdev)
{
	u32 control = i2c_read(rdev->base, RCM_I2C_CTRL);

	control &= ~(RCM_I2C_CTRL_START);

	i2c_write(control, rdev->base, RCM_I2C_CTRL);
}

static irqreturn_t rcm_i2c_isr(int irq, void *dev)
{
	struct rcm_i2c *rdev = dev;
	u32 status, int_stat;
	int read;

	status = i2c_read(rdev->base, RCM_I2C_SR);
	int_stat =  i2c_read(rdev->base, RCM_I2C_ISR);

	read = rdev->msg->flags & I2C_M_RD;

	if (!(int_stat & RCM_I2C_ISR_DNE)) {
		dev_dbg(rdev->dev, "Could not get DNE!\n");
		rcm_i2c_int_clear(rdev, RCM_I2C_ISR_ARB);
		rdev->err = -EAGAIN;
		goto stop;
	}

	if (unlikely(int_stat & RCM_I2C_ISR_ARB)) {
		dev_dbg(rdev->dev, "Get ARB LOST!\n");
		rcm_i2c_int_clear(rdev, RCM_I2C_ISR_ARB);
		rdev->err = -EAGAIN;
		goto stop;
	} else if (unlikely(int_stat & RCM_I2C_ISR_NACK)) {
		dev_dbg(rdev->dev, "Get NACK!\n");
		rcm_i2c_int_clear(rdev, RCM_I2C_ISR_NACK);

		if (!(read && (int_stat & RCM_I2C_ISR_RXNE))) {
			rdev->err = -ENXIO;
			goto stop;
		}
	}

	if (read) {
		rcm_i2c_int_clear(rdev, RCM_I2C_ISR_RXNE);
		fill_rx_fifo(rdev);

		if (!rdev->mlen)
			goto stop;
	} else {
		rcm_i2c_int_clear(rdev, RCM_I2C_ISR_TXE);

		if (rdev->mlen > 0)
			fill_tx_fifo(rdev);
	}

	rcm_i2c_res_stat(rdev);
stop:
	if ((rdev->err < 0) || (rdev->mlen == 0)) {
		rcm_i2c_int_enable(rdev, RCM_I2C_IRQ_MASK, false);
		rcm_i2c_int_clear(rdev, RCM_I2C_IRQ_MASK);
		rcm_i2c_res_stat(rdev);
		complete(&rdev->msg_complete);
	}

	return IRQ_HANDLED;
}

void rcm_i2c_repeat_start(struct rcm_i2c *rdev)
{
	u32 control = i2c_read(rdev->base, RCM_I2C_CTRL);

	i2c_write(control | RCM_I2C_CTRL_REPEAT, rdev->base, RCM_I2C_CTRL);
}

static int rcm_i2c_set_start(struct rcm_i2c *rdev)
{
	u32 start = 0, r_start = 0;

	start |= RCM_I2C_CTRL_START | RCM_I2C_CTRL_EN;
	r_start |= RCM_I2C_CTRL_REPEAT | RCM_I2C_CTRL_START;

	return rdev->curr == FIRST_MESS ? start : r_start;
}

static u32 rcm_i2c_set_stop(struct rcm_i2c *rdev, struct i2c_msg *msg)
{
	u32 stop = 0;

	/*
	 * Set STOP bit in the begining
	 * in case of READ operation.
	 */
	if ((msg->flags & I2C_M_RD) &&
		(rdev->curr == rdev->num))
		stop |= RCM_I2C_CTRL_STOP;

	return stop;
}

void i2c_send_start(struct rcm_i2c *rdev, struct i2c_msg *msg)
{
	u32 start = i2c_read(rdev->base, RCM_I2C_CTRL);

	start |= rcm_i2c_set_start(rdev);

	start |= rcm_i2c_set_stop(rdev, msg);

	if (msg->flags & I2C_M_RD)
		start |= RCM_I2C_CTRL_R;
	else
		start |= RCM_I2C_CTRL_W;

	i2c_write(start, rdev->base, RCM_I2C_CTRL);
}

u32 i2c_addr_mode(struct i2c_msg *msg)
{
	return (msg->flags & I2C_M_TEN) != 0;
}

void i2c_send_addr(struct rcm_i2c *rdev, struct i2c_msg *msg)
{
	u32 addr_m, addr_l;

	if (i2c_addr_mode(msg)) {
		addr_m = 0xF0 | ((msg->addr >> 7) & 0x3);
		addr_l = msg->addr & 0xFF;
		i2c_write(addr_m, rdev->base, RCM_I2C_TX);
		i2c_write(addr_l, rdev->base, RCM_I2C_TX);
	} else {
		i2c_write(msg->addr, rdev->base, RCM_I2C_TX);
	}
}

static void rcm_i2c_start_trans(struct rcm_i2c *rdev, struct i2c_msg *msg)
{
	u32 length;
	u32 addr = msg->addr << 1;
	u32 imask  = RCM_I2C_IER_ARB | RCM_I2C_IER_NACK;
	u32 read = msg->flags & I2C_M_RD;

	if (read) {
		imask |= RCM_I2C_IER_RXNE;
		length = (rdev->mlen < FIFO_SIZE_RX) ? FILL_RX(rdev->mlen) :
						FILL_RX(FIFO_SIZE_RX/2);
		addr |= 1;
	} else {
		length = FILL_TX(0x00);
		imask |= RCM_I2C_IER_TXE;
	}

	msg->addr = addr;

	i2c_read(rdev->base, RCM_I2C_CTRL);

	i2c_write(length, rdev->base, RCM_I2C_FILL);

	if (msg->flags & I2C_M_RD)
		i2c_write(rdev->mlen, rdev->base, RCM_I2C_NMBR);

	i2c_send_addr(rdev, msg);

	if (!read)
		fill_tx_fifo(rdev);

	i2c_read(rdev->base, RCM_I2C_ISR);

	rcm_i2c_int_enable(rdev, imask, true);

	rcm_i2c_set_stat(rdev);

	i2c_send_start(rdev, msg);
}

static int rcm_i2c_xfer_msg(struct rcm_i2c *rdev, struct i2c_msg *msg)
{
	unsigned long time;

	rdev->msg = msg;
	rdev->mlen = msg->len;
	rdev->addr = msg->addr;
	rdev->buf = msg->buf;
	rdev->err = 0;

	reinit_completion(&rdev->msg_complete);

	rcm_i2c_start_trans(rdev, msg);

	time = wait_for_completion_timeout(&rdev->msg_complete,
					RCM_I2C_XFER_TIMEOUT);

	if (time == 0)
		rdev->err = -ETIMEDOUT;

	rdev->curr++;

	return	rdev->err;
}

static int rcm_i2c_init(struct rcm_i2c *rdev)
{
	u32 bus_clk_khz = rdev->bus_clock / 1000;
#ifdef CONFIG_BASIS_PLATFORM
	u32 clk_khz  = rdev->i2c_clk / 1000;
#else
	u32 clk_khz  = clk_get_rate(rdev->clk) / 1000;
#endif
	int prescale;
	int diff;

	prescale = clk_khz / (5 * bus_clk_khz) - 1;
	prescale = clamp(prescale, 0, 0xFFFF);

	diff = clk_khz / (5 * (prescale + 1)) - bus_clk_khz;

	if (abs(diff) > bus_clk_khz / 10) {
		dev_err(rdev->dev,
			"Unsupported clock settings: clk: %d KHz, bus: %d KHz\n",
			clk_khz, bus_clk_khz);
		return -EINVAL;
	}

	if (presc != -1)
		i2c_write(presc, rdev->base, RCM_I2C_CLK);
	else
		i2c_write(prescale, rdev->base, RCM_I2C_CLK);

	rcm_i2c_int_enable(rdev, RCM_I2C_IRQ_MASK, false);

	return 0;
}

static void rcm_i2c_reinit(struct rcm_i2c *rdev)
{
	/*
	 * Reset controller state and make reinitialization.
	 */
	rcm_i2c_reset(rdev);
	rcm_i2c_init(rdev);
}

static int
rcm_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct rcm_i2c *rdev = i2c_get_adapdata(adap);
	int i, ret = 0;

	rcm_i2c_reinit(rdev);

	rdev->curr = FIRST_MESS;
	rdev->num = num;

	for (i = 0; (ret == 0) && (i < num); i++)
		ret = rcm_i2c_xfer_msg(rdev, msgs++);

	rcm_i2c_snd_stop(rdev);

	if (rdev->err == 0)
		rdev->err = rcm_i2c_wait_bus_idle(rdev);

	rcm_i2c_disable(rdev);

	return ret ? : num;
}

static u32 rcm_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm rcm_i2c_algo = {
	.master_xfer = rcm_i2c_xfer,
	.functionality = rcm_i2c_func,
};

#ifdef CONFIG_BASIS_PLATFORM

static int rcm_i2c_bind(struct basis_device *device)
{
	struct rcm_i2c *rdev = basis_device_get_drvdata(device);
	struct device *dev = &device->dev;
	int irq, ret;

	rdev->base = devm_ioremap(dev,
	                          rdev->regs + device->controller->ep_base_phys,
	                          rdev->regs_size);
	if (IS_ERR(rdev->base))
		return PTR_ERR(rdev->base);

	irq = irq_create_mapping(device->controller->domain, rdev->hwirq);

	if (irq < 0) {
		dev_err(dev, "Missing interrupt resource\n");
		return irq;
	}

	rdev->dev = dev;
	init_completion(&rdev->msg_complete);
	spin_lock_init(&rdev->lock);

	if (rdev->bus_clock == 0) {
		dev_warn(dev, "Default to 100kHz\n");
		rdev->bus_clock = 100000;	/* default clock rate */
	}

	if (rdev->bus_clock > 400000) {
		dev_err(dev, "Invalid clock-frequency %d\n", rdev->bus_clock);
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, rcm_i2c_isr, 0, "rcm-i2c", rdev);

	if (ret) {
		dev_err(dev, "Failed to claim IRQ %d\n", irq);
		return ret;
	}

	rcm_i2c_init(rdev);

	i2c_set_adapdata(&rdev->adap, rdev);
	strlcpy(rdev->adap.name, "rcm-i2c", sizeof(rdev->adap.name));

	rdev->adap.owner = THIS_MODULE;
	rdev->adap.algo = &rcm_i2c_algo;
	rdev->adap.dev.parent = dev;
//	rdev->adap.dev.of_node  = pdev->dev.of_node;

	ret = i2c_add_adapter(&rdev->adap);

	if (ret)
		return ret;

	dev_info(dev, "RC Module I2C probe complete\n");

	return 0;
}

static void rcm_i2c_unbind(struct basis_device *device)
{
	struct rcm_i2c *rdev = basis_device_get_drvdata(device);

	i2c_del_adapter(&rdev->adap);
}

static int rcm_i2c_probe(struct basis_device *device)
{
	struct rcm_i2c *data;
	struct device *dev = &device->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->device = device;

	basis_device_set_drvdata(device, data);

	return 0;
}

static const struct basis_device_id rcm_i2c_ids[] = {
	{
		.name = "rcm-i2c",
	},
	{},
};

static struct basis_device_ops rcm_i2c_ops = {
	.unbind = rcm_i2c_unbind,
	.bind   = rcm_i2c_bind,
};

BASIS_DEV_ATTR_U32_SHOW(rcm_i2c_,  hwirq,       struct rcm_i2c);
BASIS_DEV_ATTR_U32_STORE(rcm_i2c_, hwirq,       struct rcm_i2c);

BASIS_DEV_ATTR_U32_SHOW(rcm_i2c_,  regs,        struct rcm_i2c);
BASIS_DEV_ATTR_U32_STORE(rcm_i2c_, regs,        struct rcm_i2c);

BASIS_DEV_ATTR_U32_SHOW(rcm_i2c_,  regs_size,   struct rcm_i2c);
BASIS_DEV_ATTR_U32_STORE(rcm_i2c_, regs_size,   struct rcm_i2c);

BASIS_DEV_ATTR_U32_SHOW(rcm_i2c_,  i2c_clk,     struct rcm_i2c);
BASIS_DEV_ATTR_U32_STORE(rcm_i2c_, i2c_clk,     struct rcm_i2c);

BASIS_DEV_ATTR_U32_SHOW(rcm_i2c_,  bus_clock,   struct rcm_i2c);
BASIS_DEV_ATTR_U32_STORE(rcm_i2c_, bus_clock,   struct rcm_i2c);

CONFIGFS_ATTR(rcm_i2c_, hwirq);
CONFIGFS_ATTR(rcm_i2c_, regs);
CONFIGFS_ATTR(rcm_i2c_, regs_size);
CONFIGFS_ATTR(rcm_i2c_, i2c_clk);
CONFIGFS_ATTR(rcm_i2c_, bus_clock);

static struct configfs_attribute *rcm_i2c_attrs[] = {
	&rcm_i2c_attr_hwirq,
	&rcm_i2c_attr_regs,
	&rcm_i2c_attr_regs_size,
	&rcm_i2c_attr_i2c_clk,
	&rcm_i2c_attr_bus_clock,
	NULL,
};

static struct basis_device_driver rcm_i2c_driver = {
	.driver.name    = "rcm-i2c",
	.probe          = rcm_i2c_probe,
	.id_table       = rcm_i2c_ids,
	.ops            = &rcm_i2c_ops,
	.owner          = THIS_MODULE,
	.attrs          = rcm_i2c_attrs,
};
module_basis_driver(rcm_i2c_driver);

#else /* CONFIG_BASIS_PLATFORM */

static int rcm_i2c_probe(struct platform_device *pdev)
{
	struct rcm_i2c *rdev = NULL;
	struct resource *res;
	int irq, ret;
	u32 val;

	rdev = devm_kzalloc(&pdev->dev, sizeof(*rdev), GFP_KERNEL);

	if (!rdev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rdev->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(rdev->base))
		return PTR_ERR(rdev->base);

	irq = platform_get_irq(pdev, 0);

	if (irq < 0) {
		dev_err(&pdev->dev, "Missing interrupt resource\n");
		return irq;
	}

	rdev->clk =  devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(rdev->clk)) {
		dev_err(&pdev->dev, "Missing clock\n");
		return PTR_ERR(rdev->clk);
	}

	rdev->dev = &pdev->dev;
	init_completion(&rdev->msg_complete);
	spin_lock_init(&rdev->lock);

	val =  of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				       &rdev->bus_clock);

	if (val) {
		dev_warn(&pdev->dev, "Default to 100kHz\n");
		rdev->bus_clock = 100000;	/* default clock rate */
	}

	if (rdev->bus_clock > 400000) {
		dev_err(&pdev->dev, "Invalid clock-frequency %d\n",
			rdev->bus_clock);
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, irq, rcm_i2c_isr, 0,
				pdev->name, rdev);

	if (ret) {
		dev_err(&pdev->dev, "Failed to claim IRQ %d\n", irq);
		return ret;
	}

	ret = clk_prepare_enable(rdev->clk);

	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	rcm_i2c_init(rdev);

	i2c_set_adapdata(&rdev->adap, rdev);
	strlcpy(rdev->adap.name, pdev->name, sizeof(rdev->adap.name));

	rdev->adap.owner = THIS_MODULE;
	rdev->adap.algo = &rcm_i2c_algo;
	rdev->adap.dev.parent = &pdev->dev;
	rdev->adap.dev.of_node  = pdev->dev.of_node;

	platform_set_drvdata(pdev, rdev);

	ret = i2c_add_adapter(&rdev->adap);

	if (ret) {
		clk_disable_unprepare(rdev->clk);
		return ret;
	}

	dev_info(&pdev->dev, "RC Module I2C probe complete\n");

	return 0;
}

static int rcm_i2c_remove(struct platform_device *pdev)
{
	struct rcm_i2c *rdev = platform_get_drvdata(pdev);

	clk_disable_unprepare(rdev->clk);
	i2c_del_adapter(&rdev->adap);

	return 0;
}

static struct platform_driver rcm_i2c_driver = {
	.probe = rcm_i2c_probe,
	.remove = rcm_i2c_remove,
	.driver = {
		.name = "rcm-i2c",
		.of_match_table = rcm_i2c_match,
	},
};

module_platform_driver(rcm_i2c_driver);

#endif /* CONFIG_BASIS_PLATFORM */

MODULE_AUTHOR("Alexey Ivanov");
MODULE_DESCRIPTION("RC MODULE I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rcm-i2c");
