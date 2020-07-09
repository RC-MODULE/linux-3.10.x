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
//	struct irq_domain *domain;
	struct irq_chip    intc;
	int               *irqs;
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
	if (hwirq >= data->chip.irq.num_parents) {
		pr_err("%s: Incorrect child hardware IRQ offset (%lu). "
		       "Possible offsets are [0, %u]\n", __func__, 
		       hwirq, data->chip.irq.num_parents - 1);
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
/*
static int rcm_pl060_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);

	return irq_create_mapping(data->domain, offset);
}
*/
static void rcm_pl060_set_regime(struct rcm_pl060_chip *data, 
                                 unsigned r0, unsigned r1,
                                 unsigned r2, unsigned r3)
{
	unsigned val = (r0 & 0x3)        | ((r1 & 0x3) << 2) | 
	               ((r2 & 0x3) << 4) | ((r3 & 0x3) << 6);

	regmap_update_bits(data->reg, RCM_PL060_REGIME,
	                   0x0F, val);
}

static void rcm_pl060_irq_ack(struct irq_data *d)
{
	struct rcm_pl060_chip *data = irq_data_get_irq_chip_data(d);

	pr_debug("%s(%ld) >>>\n", __func__, d->hwirq);

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIREQ,
	                   BIT(d->hwirq), 0);

	pr_debug("%s <<<\n", __func__);
}

static void rcm_pl060_irq_mask(struct irq_data *d)
{
	struct rcm_pl060_chip *data = irq_data_get_irq_chip_data(d);

	pr_debug("%s(%ld) >>>\n", __func__, d->hwirq);

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIENB,
	                   BIT(d->hwirq), BIT(d->hwirq));

	pr_debug("%s <<<\n", __func__);
}

static void rcm_pl060_irq_unmask(struct irq_data *d)
{
	struct rcm_pl060_chip *data = irq_data_get_irq_chip_data(d);

	pr_debug("%s(%ld) >>>\n", __func__, d->hwirq);

	if (!rcm_pl060_check_irq(data, d->hwirq))
		return;

	regmap_update_bits(data->reg, RCM_PL060_EIENB,
	                   BIT(d->hwirq), 0);

	pr_debug("%s <<<\n", __func__);
}

static int rcm_pl060_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct rcm_pl060_chip *data = irq_data_get_irq_chip_data(d);
	unsigned val = 0;
	unsigned type_parent;

	pr_debug("%s(%ld, %u) >>>\n", __func__, d->hwirq, flow_type);

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

	return irq_chip_set_type_parent(d, type_parent);
}

static int rcm_pl060_child_to_parent_hwirq(struct gpio_chip *chip,
                                           unsigned int child_hwirq,
                                           unsigned int child_type,
                                           unsigned int *parent_hwirq,
                                           unsigned int *parent_type)
{
	struct rcm_pl060_chip *data = gpiochip_get_data(chip);
	struct irq_data *irq_data;

	pr_debug("%s >>>\n", __func__);

	if (!rcm_pl060_check_irq(data, child_hwirq))
		return -EINVAL;

	irq_data = irq_get_irq_data(chip->irq.parents[child_hwirq]);

	*parent_hwirq = irq_data->hwirq - 32;
	*parent_type = child_type;

	pr_debug("%s: child_hwirq = %u, child_type = %u, parent_hwirq = %u\n", 
	         __func__, child_hwirq, child_type, *parent_hwirq);

	return 0;
}

static void rcm_pl060_populate_parent_fwspec(struct gpio_chip *chip,
                                             struct irq_fwspec *fwspec,
                                             unsigned int parent_hwirq,
                                             unsigned int parent_type)
{
	fwspec->param_count = 3;
	fwspec->param[0] = 0;
	fwspec->param[1] = parent_hwirq;
	fwspec->param[2] = parent_type;
}

static void rcm_pl060_irq_handler(struct irq_desc *desc)
{
	struct rcm_pl060_chip *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned req;
	unsigned i;

	pr_debug("%s(%ld) >>>\n", __func__, desc->irq_data.hwirq);

	chained_irq_enter(chip, desc);

	regmap_read(data->reg, RCM_PL060_EIREQ, &req);

	pr_debug("%s: REQ = 0x%02X\n", __func__, req);

	for (i = 0; i < RCM_PL060_GPIO_NR; i++)
		if (req & BIT(i)) {
			unsigned irq = 
				irq_find_mapping(data->chip.irq.domain, i);

			if (WARN_ON(irq == 0))
				continue;

			generic_handle_irq(irq);
		}

	chained_irq_exit(chip, desc);

	pr_debug("%s <<<\n", __func__);
}

static int rcm_pl060_probe(struct platform_device *pdev)
{
	struct rcm_pl060_chip *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	void __iomem *base;
	struct resource *res;
	int cnt_irqs;
//	struct irq_chip_generic *gc;
	int i;
//	int err;

	match = of_match_device(rcm_pl060_of_match, dev);
	if (!match)
		return -EINVAL;

	cnt_irqs = of_irq_count(np);

	data = devm_kzalloc(dev, sizeof(struct rcm_pl060_chip), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

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
/*
	if (cnt_irqs)
		data->chip.to_irq   = rcm_pl060_to_irq;
*/
	if (cnt_irqs) {
/*
		data->domain = irq_domain_add_linear(np, RCM_PL060_GPIO_NR,
		                                     &irq_generic_chip_ops,
		                                     NULL);
		if (!data->domain) {
			dev_err(dev, "couldn't allocate irq domain %s (DT).\n",
			        data->chip.label);
			return -ENODEV;
		}

		err = irq_alloc_domain_generic_chips(
		    data->domain, RCM_PL060_GPIO_NR, 2, np->name,
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

		for (i = 0; i < RCM_PL060_GPIO_NR; i++) {
			int irq = platform_get_irq(pdev, i);

			if (irq < 0)
				continue;
			irq_set_chained_handler_and_data(irq,
			                                 rcm_pl060_irq_handler,
			                                 data);
		}
*/

		struct device_node *irq_parent;
		struct gpio_irq_chip *irq;

		if (cnt_irqs < RCM_PL060_GPIO_NR) {
			dev_warn(dev, "Count interrupts (%d IRQs) does not "
			              "match to GPIO count (%d GPIOs).\n",
			              cnt_irqs, data->chip.ngpio);
		}

		if (cnt_irqs > RCM_PL060_GPIO_NR)
			cnt_irqs = RCM_PL060_GPIO_NR;

		data->irqs = devm_kcalloc(dev, cnt_irqs, sizeof(*data->irqs),
		                          GFP_KERNEL);
		if (!data->irqs)
			return -ENOMEM;

		for (i = 0; i < cnt_irqs; i++) {
			int irq = platform_get_irq(pdev, i);
			if (irq < 0)
				return irq;
pr_debug("%s: irq[%d] = %d\n", __func__, i, irq);
			data->irqs[i] = irq;
		}

		data->intc.name = data->chip.label;
		data->intc.irq_ack      = rcm_pl060_irq_ack;
		data->intc.irq_mask     = rcm_pl060_irq_mask;
		data->intc.irq_unmask   = rcm_pl060_irq_unmask;
		data->intc.irq_set_type = rcm_pl060_irq_set_type;

		irq = &data->chip.irq;
		irq->chip = &data->intc;
		irq->fwnode = of_node_to_fwnode(np);
pr_debug("%s: fwnode = 0x%X\n", __func__, (u32)irq->fwnode);
		irq->child_to_parent_hwirq  = rcm_pl060_child_to_parent_hwirq;
		irq->populate_parent_fwspec = rcm_pl060_populate_parent_fwspec;
		irq->handler = handle_simple_irq;
		irq->default_type = IRQ_TYPE_NONE;
		irq->parent_handler = rcm_pl060_irq_handler;
		irq->parent_handler_data = data;
		irq->num_parents = cnt_irqs;
		irq->parents = data->irqs;

		irq_parent = of_irq_find_parent(np);
		if (!irq_parent) {
			dev_err(dev, "no IRQ parent node\n");
			return -ENODEV;
		}

		irq->parent_domain = irq_find_host(irq_parent);
		if (!irq->parent_domain) {
			dev_err(dev, "no IRQ parent domain\n");
			return -ENODEV;
		}
	}

	devm_gpiochip_add_data(dev, &data->chip, data);

	dev_info(dev, "initialized\n");

	return 0;
/*
err_domain:
	irq_domain_remove(data->domain);

	return err;
*/
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
};
builtin_platform_driver(rcm_pl060_driver);
