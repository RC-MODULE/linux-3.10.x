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
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include "rcm-clk-pll.h"

static int g_pll_div_debug = 0;

module_param_named(debug, g_pll_div_debug, int, 0);

#define TRACE(format, ...)                                                     \
	do {                                                                   \
		if (g_pll_div_debug) {                                         \
			printk("TRACE: rcm-clk-div/%s:%d: " format "\n",       \
			       __func__, __LINE__, ##__VA_ARGS__);             \
		}                                                              \
	} while (0)

struct rcm_pll_div {
	struct clk_divider div;
	struct rcm_pll *pll;
	unsigned int reg;
	unsigned int value;
	struct device *dev;
};

static inline struct rcm_pll_div *to_div(struct clk_hw *hw)
{
	return container_of(container_of(hw, struct clk_divider, hw),
			    struct rcm_pll_div, div);
}

static int rcm_pll_div_enable(struct clk_hw *hw)
{
	struct rcm_pll_div *div = to_div(hw);
	unsigned long flags;
	int val = 0;
	int ret = -EINVAL;
	TRACE("");

	spin_lock_irqsave(&div->pll->lock, flags);

	regmap_write(div->pll->regmap, RCM_PLL_WR_LOCK, RCM_PLL_WRUNLOCK);
	regmap_read(div->pll->regmap, RCM_PLL_WR_LOCK, &val);
	if (val == 0) {
		regmap_write(div->pll->regmap, div->reg + 4, 1); // enable PLL
		regmap_write(div->pll->regmap, RCM_PLL_UPD_CK,
			     0x10); // allow new values
		regmap_write(div->pll->regmap, RCM_PLL_WR_LOCK, 0); // lock back
		ret = 0;
	}
	spin_unlock_irqrestore(&div->pll->lock, flags);
	return ret;
}

static void rcm_pll_div_disable(struct clk_hw *hw)
{
	struct rcm_pll_div *div = to_div(hw);
	unsigned long flags;
	int val = 0;
	TRACE("");

	spin_lock_irqsave(&div->pll->lock, flags);

	regmap_write(div->pll->regmap, RCM_PLL_WR_LOCK, RCM_PLL_WRUNLOCK);
	regmap_read(div->pll->regmap, RCM_PLL_WR_LOCK, &val);
	if (val == 0) {
		regmap_write(div->pll->regmap, div->reg + 4, 0); // disable PLL
		regmap_write(div->pll->regmap, RCM_PLL_UPD_CK,
			     0x10); // allow new values
		regmap_write(div->pll->regmap, RCM_PLL_WR_LOCK, 0); // lock back
	}
	spin_unlock_irqrestore(&div->pll->lock, flags);
}

static unsigned long rcm_pll_clk_div_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct rcm_pll_div *div = to_div(hw);
	int val;
	regmap_read(div->pll->regmap, div->reg, &val);
	val++;
	if (val == 0)
		return 0;

	TRACE("%ld", parent_rate / val);
	return parent_rate / val;
}

static long rcm_pll_clk_div_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	struct rcm_pll_div *div = to_div(hw);
	//    TRACE("%ld, %ld", rate, *prate);

	return divider_round_rate(hw, rate, prate, 0, div->div.width,
				  div->div.flags);
}

static int rcm_pll_clk_div_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct rcm_pll_div *div = to_div(hw);

	TRACE("");

	if (!(parent_rate % rate) && (parent_rate / rate) <= 257) {
		unsigned int val = parent_rate / rate - 1;
		unsigned int current;

		regmap_read(div->pll->regmap, div->reg, &current);
		if (current != val) {
			unsigned long flags;
			int ret = -EBUSY;

			TRACE("set divisor to %d", val);
			spin_lock_irqsave(&div->pll->lock, flags);

			regmap_write(div->pll->regmap, RCM_PLL_WR_LOCK,
				     RCM_PLL_WRUNLOCK);
			regmap_read(div->pll->regmap, RCM_PLL_WR_LOCK, &val);
			if (val == 0) {
				// write new data to PLL control
				regmap_write(div->pll->regmap, div->reg, val);

				regmap_write(div->pll->regmap, RCM_PLL_UPD_CK,
					     0x1); // allow new values

				ret = 0;
			}
			spin_unlock_irqrestore(&div->pll->lock, flags);

			return ret;
		}
	} else {
		dev_err(div->dev, "unable to set %ld frequency", rate);
	}

	return -EINVAL;
}

static const struct clk_ops rcm_pll_div_ops = {
	.enable = rcm_pll_div_enable,
	.disable = rcm_pll_div_disable,
	.recalc_rate = rcm_pll_clk_div_recalc_rate,
	.round_rate = rcm_pll_clk_div_round_rate,
	.set_rate = rcm_pll_clk_div_set_rate
};

static int rcm_pll_div_probe(struct platform_device *pdev)
{
	struct rcm_pll_div *d;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const char *clk_name;
	const char *parent_name;
	struct clk_init_data init;
	struct clk *clk;
	struct device_node *parent_clk;
	struct platform_device *parent_dev;

	TRACE("");

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	platform_set_drvdata(pdev, d);

	if (of_property_read_u32(node, "reg", &d->reg)) {
		dev_err(dev, "failed to divider offset");
		return -EINVAL;
	}
	TRACE("reg: %x", d->reg);

	parent_clk = of_get_parent(node);
	if (!parent_clk) {
		dev_err(dev, "unable to find parent PLL");
		return -EINVAL;
	}
	parent_dev = of_find_device_by_node(parent_clk);
	if (parent_dev) {
		TRACE("parent_dev");
		d->pll = platform_get_drvdata(parent_dev);
	}

	if (!d->pll) {
		TRACE("deffer");
		return -EPROBE_DEFER;
	}
	TRACE("allok");

	memset(&init, 0, sizeof(init));
	clk_name = node->name;
	init.name = clk_name;
	init.ops = &rcm_pll_div_ops;
	parent_name = of_clk_get_parent_name(node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;
	d->div.hw.init = &init;
	d->div.reg = &d->value;
	d->div.shift = 0;
	d->div.width = 8;
	d->div.hw.init = &init;
	d->div.table = 0;

	d->dev = dev;

	clk = devm_clk_register(dev, &d->div.hw);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to register %s clock (%ld)\n", clk_name,
			PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static const struct of_device_id rcm_pll_div_dt_ids[] = {
	{
		.compatible = "rcm,pll-div",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_pll_div_dt_ids);

static struct platform_driver rcm_pll_div_driver = {
	.driver =
		{
			.name = "rcm-pll-div",
			.of_match_table = rcm_pll_div_dt_ids,
		},
	.probe = rcm_pll_div_probe,
};

module_platform_driver(rcm_pll_div_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM Clock divider driver");
MODULE_LICENSE("GPL");
