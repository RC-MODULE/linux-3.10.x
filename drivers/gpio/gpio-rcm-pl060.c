// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#define DEBUG

#include <linux/gpio/driver.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#define RCM_PL060_PDR0     0x00
#define RCM_PL060_DDR0     0x10
#define RCM_PL060_EIENB    0x20
#define RCM_PL060_EIREQ    0x24
#define RCM_PL060_EILVL    0x28
#define RCM_PL060_RA_CONTR 0x30
#define RCM_PL060_RA_VAL   0x34
#define RCM_PL060_REGIME   0x38

#define RCM_PL060_GPIO_NR  8

#define RCM_PL060_REGIME_RA    0x0
#define RCM_PL060_REGIME_EXIRC 0x2
#define RCM_PL060_REGIME_GPIO  0x3

struct rcm_pl060_chip {
	struct gpio_chip   chip;
	struct regmap     *reg;
	struct irq_domain *domain;
	int                cnt_irqs;
};

static const struct regmap_config rcm_pl060_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_PL060_REGIME,
};

static const struct of_device_id rcm_pl060_of_match[];

static int rcm_pl060_check_irq(struct rcm_pl060_chip *data, unsigned long hwirq)
{
	if (hwirq >= data->cnt_irqs) {
		pr_err("%s: Incorrect child hardware IRQ offset (%lu). "
		       "Possible offsets are [0, %u]\n", __func__, 
		       hwirq, data->cnt_irqs - 1);
		return 0;
	}

	return 1;
}

static int rcm_pl060_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);
	unsigned val;

	if (offset >= RCM_PL060_GPIO_NR)
		return -EINVAL;

	regmap_read(data->reg, RCM_PL060_PDR0, &val);

	return (val & BIT(offset)) ? 1 : 0;
}

static void rcm_pl060_set_value(struct gpio_chip *chip, unsigned offset,
                                int value)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);

	if (offset >= RCM_PL060_GPIO_NR)
		return;

	regmap_update_bits(data->reg, RCM_PL060_PDR0,
	                   BIT(offset), value ? BIT(offset) : 0);
}

static int rcm_pl060_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);
	unsigned val;

	if (offset >= RCM_PL060_GPIO_NR)
		return -EINVAL;

	regmap_read(data->reg, RCM_PL060_DDR0, &val);

	return (val & BIT(offset)) ? GPIO_LINE_DIRECTION_OUT : 
	                             GPIO_LINE_DIRECTION_IN;
}

static int rcm_pl060_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);

	if (offset >= RCM_PL060_GPIO_NR)
		return -EINVAL;

	regmap_update_bits(data->reg, RCM_PL060_DDR0, BIT(offset), 0);

	return 0;
}

static int rcm_pl060_direction_output(struct gpio_chip *chip, unsigned offset,
                                      int value)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);

	if (offset >= RCM_PL060_GPIO_NR)
		return -EINVAL;

	rcm_pl060_set_value(chip, offset, value);
	regmap_update_bits(data->reg, RCM_PL060_DDR0, BIT(offset), BIT(offset));

	return 0;
}

static void rcm_pl060_set_regime(struct rcm_pl060_chip *data, 
                                 unsigned r0, unsigned r1,
                                 unsigned r2, unsigned r3)
{
	unsigned val = (r0 & 0x3)        | ((r1 & 0x3) << 2) | 
	               ((r2 & 0x3) << 4) | ((r3 & 0x3) << 6);

	regmap_update_bits(data->reg, RCM_PL060_REGIME,
	                   0xFF, val);
}

static void rcm_pl060_irq_ack(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pl060_chip *data = gc->private;

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIREQ,
	                   BIT(d->hwirq), 0);
}

static void rcm_pl060_irq_mask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pl060_chip *data = gc->private;

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIENB,
	                   BIT(d->hwirq), 0);
}

static void rcm_pl060_irq_unmask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pl060_chip *data = gc->private;

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIENB,
	                   BIT(d->hwirq), BIT(d->hwirq));
}

static int rcm_pl060_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct rcm_pl060_chip *data = gc->private;
	unsigned val = 0;
	unsigned type_parent;

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return -EINVAL;

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		val = 0x2;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val = 0x3;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		val = 0x1;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		val = 0x0;
		break;
	default:
		pr_err("%s: Unsupported IRQ type = %u.\n", __func__, flow_type);
		return -EINVAL;
	}

	regmap_update_bits(data->reg, RCM_PL060_EILVL,
	                   0x03 << (2 * d->hwirq), val << (2 * d->hwirq));

	if ((flow_type & IRQ_TYPE_EDGE_BOTH) == 0)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	type_parent = (flow_type & IRQ_TYPE_EDGE_BOTH) ? IRQ_TYPE_EDGE_RISING : 
	                                                 IRQ_TYPE_LEVEL_HIGH;

	return 0;
}

static void rcm_pl060_irq_handler(struct irq_desc *desc)
{
	struct rcm_pl060_chip *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned req;
	unsigned i;

	chained_irq_enter(chip, desc);

	regmap_read(data->reg, RCM_PL060_EIREQ, &req);

	for (i = 0; i < data->cnt_irqs; i++)
		if (req & BIT(i)) {
			unsigned irq = 
				irq_find_mapping(data->domain, i);

			if (WARN_ON(irq == 0))
				continue;

			generic_handle_irq(irq);
		}

	chained_irq_exit(chip, desc);
}

static int rcm_pl060_probe(struct platform_device *pdev)
{
	struct rcm_pl060_chip *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	void __iomem *base;
	struct resource *res;
	struct irq_chip_generic *gc;
	int i;
	int err;

	match = of_match_device(rcm_pl060_of_match, dev);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(struct rcm_pl060_chip), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->cnt_irqs = of_irq_count(np);

	if (data->cnt_irqs > RCM_PL060_GPIO_NR)
		data->cnt_irqs = RCM_PL060_GPIO_NR;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init_mmio(dev, base, 
	                                  &rcm_pl060_regmap_config);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	rcm_pl060_set_regime(data, RCM_PL060_REGIME_GPIO, 
	                           RCM_PL060_REGIME_GPIO,
	                           RCM_PL060_REGIME_GPIO,
	                           RCM_PL060_REGIME_GPIO);

	data->chip.base      = -1;
	data->chip.label     = dev_name(dev);
	data->chip.parent    = dev;
	data->chip.owner     = THIS_MODULE;
	data->chip.ngpio     = RCM_PL060_GPIO_NR;
	data->chip.can_sleep = false;
	data->chip.of_node   = np;

	data->chip.get_direction    = rcm_pl060_get_direction;
	data->chip.direction_input  = rcm_pl060_direction_input;
	data->chip.direction_output = rcm_pl060_direction_output;
	data->chip.get              = rcm_pl060_get_value;
	data->chip.set              = rcm_pl060_set_value;

	if (data->cnt_irqs) {
		data->domain = irq_domain_add_linear(np, data->cnt_irqs,
		                                     &irq_generic_chip_ops,
		                                     NULL);
		if (!data->domain) {
			dev_err(dev, "couldn't allocate irq domain %s (DT).\n",
			        data->chip.label);
			return -ENODEV;
		}

		err = irq_alloc_domain_generic_chips(
		    data->domain, data->cnt_irqs, 2, np->name,
		    handle_level_irq, IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_LEVEL,
		    0, 0);
		if (err) {
			dev_err(dev, "couldn't allocate irq chips %s (DT).\n",
			        data->chip.label);
			goto err_domain;
		}

		gc = irq_get_domain_generic_chip(data->domain, 0);
		gc->private = data;

		gc->chip_types[0].type = IRQ_TYPE_LEVEL_MASK;
		gc->chip_types[1].type = IRQ_TYPE_EDGE_BOTH;
		gc->chip_types[1].handler = handle_edge_irq;

		for (i = 0; i < 2; ++i) {
			gc->chip_types[i].chip.irq_ack = 
				rcm_pl060_irq_ack;
			gc->chip_types[i].chip.irq_mask = 
				rcm_pl060_irq_mask;
			gc->chip_types[i].chip.irq_unmask = 
				rcm_pl060_irq_unmask;
			gc->chip_types[i].chip.irq_set_type = 
				rcm_pl060_irq_set_type;
			gc->chip_types[i].chip.name = data->chip.label;
		}

		for (i = 0; i < data->cnt_irqs; i++) {
			int irq = platform_get_irq(pdev, i);

			if (irq < 0)
				continue;
			irq_set_chained_handler_and_data(irq,
			                                 rcm_pl060_irq_handler,
			                                 data);
		}

		for (i = 0; i < data->cnt_irqs; ++i)
			irq_create_mapping(data->domain, i);
	}

	devm_gpiochip_add_data(dev, &data->chip, data);

	dev_info(dev, "initialized\n");

	return 0;

err_domain:
	irq_domain_remove(data->domain);

	return err;
}

static int rcm_pl060_remove(struct platform_device *pdev)
{
	struct rcm_pl060_chip *data = platform_get_drvdata(pdev);
	int i;

	if (data->domain) {
		for (i = 0; i < data->cnt_irqs; ++i)
			irq_dispose_mapping(irq_find_mapping(data->domain, i));
		irq_domain_remove(data->domain);
	}

	return 0;
}

static const struct of_device_id rcm_pl060_of_match[] = {
	{
		.compatible = "rcm,pl060",
	},
	{},
};

static struct platform_driver rcm_pl060_driver = {
	.driver = {
		.name = "gpio-rcm-pl060",
		.of_match_table = rcm_pl060_of_match,
	},
	.probe = rcm_pl060_probe,
	.remove = rcm_pl060_remove
};
builtin_platform_driver(rcm_pl060_driver);
