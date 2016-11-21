/*
 * Copyright (C) 2014 RC Module.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for Fujitsu GPIO IP
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>


#define DRVNAME "fj_gpio"

#define NR_GPIO 32
#define NR_GPIO_PER_REG 8


/* Offsets are in uint32_t-s */
#define FJ_OFF_PORT 0x00
#define FJ_OFF_DDR  0x4
#define FJ_OFF_PFR  0x8


struct fj_gpio {
	struct gpio_chip gpio_chip;
	__iomem uint32_t  *regs;
};

static inline void fj_write_bit(uint32_t *off, int bit, int v)
{
	int n = 0;

	while (bit >= 8)
		bit -= 8, n += 1;
	off[n] &= ~(1<<bit);
	off[n] |= (v<<bit);
}

static inline uint32_t fj_get_bit(uint32_t *off, int bit)
{
	int n = 0;

	while (bit >= 8)
		bit -= 8, n += 1;
	return !!(off[n] & (1<<bit));
}

static int fj_gpio_output(struct gpio_chip *gc, unsigned int offset,
				int value)
{
	struct fj_gpio *chip =  (struct fj_gpio *) gc;

	fj_write_bit(chip->regs + FJ_OFF_DDR,  offset, 1);
	fj_write_bit(chip->regs + FJ_OFF_PORT, offset, value);
	return 0;
}

static int fj_gpio_input(struct gpio_chip *gc, unsigned int offset)
{
	struct fj_gpio *chip =  (struct fj_gpio *) gc;

	fj_write_bit(chip->regs + FJ_OFF_DDR,  offset, 0);
	return 0;
}

static int fj_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct fj_gpio *chip =  (struct fj_gpio *) gc;

	return fj_get_bit(chip->regs + FJ_OFF_PORT, offset);
}

static void fj_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct fj_gpio *chip =  (struct fj_gpio *) gc;

	fj_write_bit(chip->regs + FJ_OFF_PORT,  offset, value);
}

static int fj_gpio_remove(struct platform_device *pdev)
{
	struct fj_gpio *fj_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&fj_gpio->gpio_chip);
	return 0;
}

static int fj_gpio_probe(struct platform_device *pdev)
{
	struct fj_platform_data *pdata;
	struct fj_gpio *fj_gpio;
	struct resource *res;
	int ret;
	uint32_t base;

	pdata = dev_get_platdata(pdev->dev.parent);
	fj_gpio = devm_kzalloc(&pdev->dev,
				sizeof(*fj_gpio), GFP_KERNEL);
	if (!fj_gpio) {
		dev_err(&pdev->dev, "Could not allocate fj_gpio\n");
		return -ENOMEM;
	}
	fj_gpio->gpio_chip.owner = THIS_MODULE;
	fj_gpio->gpio_chip.label = pdev->name;
	/*fj_gpio->gpio_chip.dev = &pdev->dev; */
	fj_gpio->gpio_chip.ngpio = 32;

	fj_gpio->gpio_chip.direction_output = fj_gpio_output;
	fj_gpio->gpio_chip.direction_input = fj_gpio_input;
	fj_gpio->gpio_chip.set	= fj_gpio_set;
	fj_gpio->gpio_chip.get	= fj_gpio_get;

#ifdef CONFIG_OF_GPIO
	fj_gpio->gpio_chip.of_node = pdev->dev.of_node;
#endif

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");

	fj_gpio->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!fj_gpio->regs)
		goto errfree;
	ret =  of_property_read_u32(pdev->dev.of_node, "gpio-base",
				       &base);
	if (ret != 0)
		fj_gpio->gpio_chip.base = -1; /* dynamic */
	else
		fj_gpio->gpio_chip.base = base;
	ret = gpiochip_add(&fj_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, fj_gpio);
	printk(DRVNAME ": Added %d gpio lines at base %d\n", NR_GPIO, base);
	return ret;
errfree:
	kfree(fj_gpio);
	return -EIO;
}

static const struct of_device_id of_match_table[] = {
	{ .compatible = "rcm,fgpio", },
	{ /* end of list */ }
};

MODULE_DEVICE_TABLE(of, of_match_table);


static struct platform_driver fjgpio_platform_driver = {
	.probe		= fj_gpio_probe,
	.remove		= fj_gpio_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRVNAME,
		.of_match_table = of_match_ptr(of_match_table),
	},
};

module_platform_driver(fjgpio_platform_driver);

MODULE_DESCRIPTION("Fujitsu GPIO IP driver for RC Module SoCs");
MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_LICENSE("GPL");
