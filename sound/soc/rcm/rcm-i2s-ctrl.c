/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#include "rcm-i2s.h"

static const struct regmap_config rcm_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 8,
	.val_bits = 32,
	.max_register = RCM_I2S_REG_FIFO3,
};

static int rcm_i2s_reset(struct rcm_i2s_control *i2s)
{
	unsigned int status;
	printk("TRACE: rcm_i2s_reset");
    i2s->hw_params[0] = i2s->hw_params[1] = 0;
	regmap_update_bits(i2s->regmap, RCM_I2S_REG_CTRL0, 1, 1);
	return regmap_read_poll_timeout(i2s->regmap, RCM_I2S_REG_CTRL0, status, (status&1) == 0, 100, 10000);
}

static irqreturn_t i2s_interrupt_handler(int irq, void *data)
{
	struct rcm_i2s_control *i2s = (struct rcm_i2s_control *)data;
	unsigned int stat0;
	unsigned int stat1;
	regmap_read(i2s->regmap, RCM_I2S_REG_STAT0, &stat0);
	regmap_read(i2s->regmap, RCM_I2S_REG_STAT1, &stat1);

	printk("TRACE: i2s_interrupt_handler: %08X, %08X", stat0, stat1);

	return IRQ_HANDLED;
}

static int rcm_i2s_ctrl_probe(struct platform_device *pdev)
{
	struct rcm_i2s_control *i2s;
	void __iomem *base;
    int ret, irq;

	printk("TRACE: rcm_i2s_ctrl_probe");

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2s);

	i2s->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, i2s->res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, base,
		&rcm_i2s_regmap_config);
	if (IS_ERR(i2s->regmap))
		return PTR_ERR(i2s->regmap);

    spin_lock_init(&i2s->lock);

	if(rcm_i2s_reset(i2s))
	{
        dev_err(&pdev->dev, "unable to find i2s hardware");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
        dev_err(&pdev->dev, "unable to get interrupt property");
		return -ENODEV;
    }
	ret = devm_request_irq(&pdev->dev, irq, i2s_interrupt_handler, 0,
			       pdev->name, i2s);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return  -ENODEV;
	};

    i2s->started_count = 0;
    i2s->opened_count = 0;
    i2s->rcm_i2s_reset = rcm_i2s_reset;

	printk("TRACE: rcm_i2s_ctrl_probe return %d", ret);

	return ret;
}

static int rcm_i2s_ctrl_remove(struct platform_device *pdev)
{
	//struct rcm_i2s_control *i2s = platform_get_drvdata(pdev);
	printk("TRACE: rcm_i2s_ctrl_remove");
	return 0;
}

static const struct of_device_id rcm_i2s_ctrl_of_match[] = {
	{ .compatible = "rcm,i2s-ctrl", },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_i2s_ctrl_of_match);

static struct platform_driver rcm_i2s_ctrl_driver = {
	.driver = {
		.name = "rcm-i2s-ctrl",
		.of_match_table = rcm_i2s_ctrl_of_match,
	},
	.probe = rcm_i2s_ctrl_probe,
	.remove = rcm_i2s_ctrl_remove,
};
module_platform_driver(rcm_i2s_ctrl_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM I2S control driver");
MODULE_LICENSE("GPL");
