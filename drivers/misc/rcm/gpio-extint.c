// SPDX-License-Identifier: GPL-2.0
/**
 * Test driver to test gpio external interrupts.
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>

struct gpio_extint_data {
	int irq;
};

static const struct of_device_id gpio_extint_of_match[];

static irqreturn_t gpio_extint_irq_handler(int irq, void *dev_id)
{
	pr_debug("%s(%d) >>>\n", __func__, irq);
	pr_debug("%s <<<\n", __func__);

	return IRQ_HANDLED;
}

static int gpio_extint_probe(struct platform_device *pdev)
{
	struct gpio_extint_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	int res;

	match = of_match_device(gpio_extint_of_match, dev);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(struct gpio_extint_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		dev_err(dev, "Interrupt is not specified.\n");
		return data->irq;
	}

	res = devm_request_irq(dev, data->irq, gpio_extint_irq_handler, 0, 
	                       "gpio_extint", data);
	if (res < 0) {
		dev_err(dev, "cannot get IRQ (%d)\n", res);
		return res;
	}

	dev_set_drvdata(dev, data);

	dev_info(dev, "initialized\n");

	return 0;
}

static int gpio_extint_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_extint_data *data = dev_get_drvdata(dev);

//	device_remove_file(dev, &dev_attr_);

//	device_remove_file(dev, &dev_attr_);

	return 0;
}

static const struct of_device_id gpio_extint_of_match[] = {
	{
		.compatible = "rcm,gpio-extint",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gpio_extint_of_match);

static struct platform_driver gpio_extint_driver = {
	.driver = {
		.name = "gpio-extint",
		.of_match_table = gpio_extint_of_match,
	},
	.probe = gpio_extint_probe,
	.remove = gpio_extint_remove,
};
module_platform_driver(gpio_extint_driver);
