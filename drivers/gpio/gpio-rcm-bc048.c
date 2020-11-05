/* gpio-rcm.c: GPIO driver for RC "MODULE" SoC.
 *
 * Alexey Ivanov <ivanov.a1exey@yandex.ru>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/of.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/bitops.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>

#define RCM_GPIO_ID		0x00
#define RCM_GPIO_VER		0x04
#define RCM_GPIO_PAD_DIR	0x08
#define RCM_GPIO_WR_DATA	0x0C
#define RCM_GPIO_RD_DATA	0x10
#define RCM_GPIO_STATUS		0x14
#define RCM_GPIO_IRQ_MASK	0x18
#define RCM_GPIO_WR_DATA1	0x1C
#define RCM_GPIO_WR_DATA0	0x20
#define RCM_GPIO_SRC		0x24

#define RCM_GPIO_MAX_NGPIO	8

#define GPIO_OFFSET	4

struct rcm_gpio_chip {
	struct irq_domain *domain;
	struct gpio_chip gchip;
	spinlock_t lock;
	void __iomem *regs;
	int irq;
	u32 type;
};

static inline void gpio_write(uint32_t value, void *base, uint32_t addr)
{
	writel(value, base + addr);
#if defined DEBUG
	//dev_dbg(rdev->dev, "iowrite32(0x%x, base + 0x%x);\n", value, addr);
#endif
}

static inline uint32_t gpio_read(void *base, uint32_t addr)
{
	uint32_t reg =  readl(base + addr);
#if defined DEBUG
	//dev_dbg(rdev->dev, "/* ioread32(base + 0x%x) == 0x%x */\n", addr, reg);
#endif
	return reg;
}

/*
 * Call after set type of interrupt trigger event.
 * echo > "rising" > /sys/class/gpio/gpioN/edge
 */
static void rcm_gpio_irq_mask(struct irq_data *data)
{
	struct rcm_gpio_chip *gc = irq_data_get_irq_chip_data(data);
	void __iomem *base = gc->regs;
	unsigned long flags;
	u32 gpio = irqd_to_hwirq(data);

	spin_lock_irqsave(&gc->lock, flags);

	gc->type &= ~(BIT(gpio*GPIO_OFFSET) | BIT(gpio*GPIO_OFFSET + 1));

	gpio_write(gc->type, base, RCM_GPIO_IRQ_MASK);

	spin_unlock_irqrestore(&gc->lock, flags);
}

/*
 * Call after set type of interrupt trigger event.
 * echo > "none" > /sys/class/gpio/gpioN/edge
 */
static void rcm_gpio_irq_unmask(struct irq_data *data)
{
	struct rcm_gpio_chip *gc = irq_data_get_irq_chip_data(data);
	void __iomem *base = gc->regs;
	unsigned long flags;

	spin_lock_irqsave(&gc->lock, flags);

	gpio_write(gc->type, base, RCM_GPIO_IRQ_MASK);

	spin_unlock_irqrestore(&gc->lock, flags);
}

/*
 * Call when set interrupt event type.
 * echo > "rising" > /sys/class/gpio/gpioN/edge
 */
static int rcm_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct rcm_gpio_chip *gc = irq_data_get_irq_chip_data(data);
	u32 gpio = irqd_to_hwirq(data);
	unsigned long flags;

	spin_lock_irqsave(&gc->lock, flags);

	if (type & IRQ_TYPE_EDGE_RISING)
		gc->type |= BIT(gpio*GPIO_OFFSET);
	else
		gc->type &= (~BIT(gpio*GPIO_OFFSET));

	if (type & IRQ_TYPE_EDGE_FALLING)
		gc->type |= BIT(gpio*GPIO_OFFSET + 1);
	else
		gc->type &= (~BIT(gpio*GPIO_OFFSET + 1));

	spin_unlock_irqrestore(&gc->lock, flags);

	return 0;
}

static struct irq_chip rcm_irq_chip = {
	.name = "rcm-gpio",
	.irq_mask = rcm_gpio_irq_mask,
	.irq_unmask = rcm_gpio_irq_unmask,
	.irq_set_type = rcm_gpio_irq_set_type,
};

static inline struct rcm_gpio_chip *to_rcm_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct rcm_gpio_chip, gchip);
}

/*
 * echo > "in" > /sys/class/gpio/gpioN/direction
 */
static int rcm_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct rcm_gpio_chip *gc = to_rcm_gpio(chip);
	unsigned long flag;
	u32 data_dir;

	spin_lock_irqsave(&gc->lock, flag);

	data_dir = gpio_read(gc->regs, RCM_GPIO_PAD_DIR);
	data_dir &= ~BIT(offset);

	gpio_write(data_dir, gc->regs, RCM_GPIO_PAD_DIR);

	spin_unlock_irqrestore(&gc->lock, flag);

	return 0;
}

/*
 * echo > "out" > /sys/class/gpio/gpioN/direction
 */
static int rcm_gpio_direction_output(struct gpio_chip *chip,
			unsigned offset, int value)
{
	struct rcm_gpio_chip *gc = to_rcm_gpio(chip);
	unsigned long flag;
	u32 data_reg, data_dir;

	spin_lock_irqsave(&gc->lock, flag);

	data_reg = gpio_read(gc->regs, RCM_GPIO_WR_DATA);

	if (value)
		data_reg |= BIT(offset);
	else
		data_reg &= ~BIT(offset);

	gpio_write(data_reg, gc->regs, RCM_GPIO_WR_DATA);

	data_dir = gpio_read(gc->regs, RCM_GPIO_PAD_DIR);
	data_dir |= BIT(offset);
	gpio_write(data_dir, gc->regs, RCM_GPIO_PAD_DIR);

	spin_unlock_irqrestore(&gc->lock, flag);

	return 0;
}

static int rcm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct rcm_gpio_chip *gc = to_rcm_gpio(chip);

	return !!(gpio_read(gc->regs, RCM_GPIO_RD_DATA) & BIT(offset));
}

static void rcm_gpio_set(struct gpio_chip *chip,
			unsigned offset, int value)
{
	struct rcm_gpio_chip *gc = to_rcm_gpio(chip);
	unsigned long flag;
	unsigned int data_reg;

	spin_lock_irqsave(&gc->lock, flag);

	data_reg = gpio_read(gc->regs, RCM_GPIO_WR_DATA);

	if (value)
		data_reg |= BIT(offset);
	else
		data_reg &= ~BIT(offset);

	gpio_write(data_reg, gc->regs, RCM_GPIO_WR_DATA);

	spin_unlock_irqrestore(&gc->lock, flag);
}

static int rcm_gpio_irq_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, d->host_data);
	irq_set_chip_and_handler(irq, &rcm_irq_chip,
					handle_simple_irq);
	irq_set_noprobe(irq);
	return 0;
}

static struct irq_domain_ops rcm_gpio_irq_domain_ops = {
	.map = rcm_gpio_irq_domain_map,
};

static void rcm_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct rcm_gpio_chip *rcm_gc = irq_get_handler_data(irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	void __iomem *base;
	u32 status, mask, gpio, pending;

	chained_irq_enter(chip, desc);

	base = rcm_gc->regs;

	status = gpio_read(base, RCM_GPIO_STATUS);
	mask = gpio_read(base, RCM_GPIO_IRQ_MASK);
	pending = status & mask;

	while (pending) {
		gpio = __ffs(pending);
		pending &= ~BIT(gpio);

		generic_handle_irq(
				irq_find_mapping(rcm_gc->domain, gpio));
	}

	chained_irq_exit(chip, desc);
}

static int rcm_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	struct rcm_gpio_chip *rcm = to_rcm_gpio(chip);

	return irq_create_mapping(rcm->domain, gpio);
}

static int rcm_gpio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct rcm_gpio_chip *rcm_gc;
	struct resource *res;
	int ngpio, ret, i;

	rcm_gc = devm_kzalloc(&pdev->dev, sizeof(*rcm_gc), GFP_KERNEL);

	if (!rcm_gc)
		return -ENOMEM;

	spin_lock_init(&rcm_gc->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	rcm_gc->regs = devm_ioremap_resource(&pdev->dev, res);

	if (!rcm_gc->regs)
		goto free;

	if (!of_property_read_u32(node, "rcm,ngpio", &ngpio))
		rcm_gc->gchip.ngpio = ngpio;
	else
		rcm_gc->gchip.ngpio = RCM_GPIO_MAX_NGPIO;

	if (rcm_gc->gchip.ngpio > RCM_GPIO_MAX_NGPIO) {
		dev_warn(&pdev->dev, "ngpio is greater than MAX!\n");
		rcm_gc->gchip.ngpio = RCM_GPIO_MAX_NGPIO;
	}

	rcm_gc->gchip.direction_input = rcm_gpio_direction_input;
	rcm_gc->gchip.direction_output = rcm_gpio_direction_output;
	rcm_gc->gchip.get = rcm_gpio_get;
	rcm_gc->gchip.set = rcm_gpio_set;
	rcm_gc->gchip.owner = THIS_MODULE;
	rcm_gc->gchip.to_irq = rcm_gpio_to_irq;
	rcm_gc->gchip.base = -1;
	rcm_gc->gchip.parent = &pdev->dev;
	rcm_gc->gchip.label = pdev->name;

	platform_set_drvdata(pdev, rcm_gc);

	rcm_gc->domain = irq_domain_add_linear(pdev->dev.of_node,
						rcm_gc->gchip.ngpio,
						&rcm_gpio_irq_domain_ops,
						rcm_gc);
	if (!rcm_gc->domain)
		return -ENODEV;

	rcm_gc->irq = platform_get_irq(pdev, 0);

	if (rcm_gc->irq < 0)
		goto free;

	ret = gpiochip_add(&rcm_gc->gchip);

	if (ret) {
		dev_err(&pdev->dev,
			"Failed to register gpio chip (%d)\n", ret);
		goto free;
	}

	for (i = 0; i < rcm_gc->gchip.ngpio; i++) {
		int irq = rcm_gpio_to_irq(&rcm_gc->gchip, i);

		irq_set_chip_and_handler(irq, &rcm_irq_chip, handle_simple_irq);
		irq_set_noprobe(irq);
	}

	irq_set_chained_handler(rcm_gc->irq, rcm_irq_handler);
	irq_set_handler_data(rcm_gc->irq, rcm_gc);

	dev_info(&pdev->dev, "RC Module GPIO probe complete (%d..%d)\n",
				rcm_gc->gchip.base,
				rcm_gc->gchip.base + rcm_gc->gchip.ngpio - 1);

	return 0;
free:
	kfree(rcm_gc);
	return -ENOMEM;
}

static int rcm_gpio_remove(struct platform_device *pdev)
{
	struct rcm_gpio_chip *rcm_gc = platform_get_drvdata(pdev);
	gpiochip_remove(&rcm_gc->gchip);
	return 0;
}

static const struct of_device_id rcm_gpio_of_match[] = {
	{ .compatible = "rcm,gpio-bc048" },
	{ },
};

MODULE_DEVICE_TABLE(of, rcm_gpio_of_match);

static struct platform_driver rcm_gpio_driver = {
	.probe = rcm_gpio_probe,
	.remove = rcm_gpio_remove,
	.driver = {
		.name = "rcm-gpio",
		.of_match_table = of_match_ptr(rcm_gpio_of_match),
	},
};

module_platform_driver(rcm_gpio_driver);

MODULE_DESCRIPTION("RC MODULE GPIO driver");
MODULE_AUTHOR("Alexey Ivanov");
MODULE_LICENSE("GPL");
