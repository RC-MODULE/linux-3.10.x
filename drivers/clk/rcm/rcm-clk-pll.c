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
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include "rcm-clk-pll.h"

static int g_pll_debug = 0;

module_param_named(debug, g_pll_debug, int, 0);

#define TRACE(format, ...)                                                     \
	do {                                                                   \
		if (g_pll_debug) {                                             \
			printk("TRACE: rcm-clk-pll/%s:%d: " format "\n",       \
			       __func__, __LINE__, ##__VA_ARGS__);             \
		}                                                              \
	} while (0)

static const struct regmap_config rcm_pll_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_PLL_CKENX,
};

static inline struct rcm_pll *to_pll(struct clk_hw *hw)
{
	return container_of(hw, struct rcm_pll, hw);
}

static int rcm_pll_wait_for_stable(struct rcm_pll *pll)
{
	unsigned int val = 0;
	int ret;
	ret = regmap_read_poll_timeout(pll->regmap, RCM_PLL_CTRL, val,
				       (val & 1) == 0, 0,
				       RCM_PLL_STABILITY_TIMEOUT);
	if (!ret)
		ret = regmap_read_poll_timeout(pll->regmap, RCM_PLL_STATE, val,
					       val == 1, 0,
					       RCM_PLL_STABILITY_TIMEOUT);
	TRACE("%d", ret);
	return ret;
}

static int rcm_pll_enable(struct clk_hw *hw)
{
	struct rcm_pll *pll = to_pll(hw);
	unsigned long flags;
	int val = 0;
	int ret = -EINVAL;
	TRACE("");

	spin_lock_irqsave(&pll->lock, flags);

	regmap_write(pll->regmap, RCM_PLL_WR_LOCK, RCM_PLL_WRUNLOCK);
	regmap_read(pll->regmap, RCM_PLL_WR_LOCK, &val);
	if (val == 0) {
		regmap_update_bits(pll->regmap, RCM_PLL_CTRL, BIT(1),
				   BIT(1)); // enable PLL
		regmap_write(pll->regmap, RCM_PLL_WR_LOCK, 0); // lock back
		if (!rcm_pll_wait_for_stable(pll))
			ret = 0;
	}
	spin_unlock_irqrestore(&pll->lock, flags);
	return ret;
}

static void rcm_pll_disable(struct clk_hw *hw)
{
	struct rcm_pll *pll = to_pll(hw);
	unsigned long flags;
	unsigned int val = 1;
	TRACE("");

	spin_lock_irqsave(&pll->lock, flags);

	regmap_write(pll->regmap, RCM_PLL_WR_LOCK, RCM_PLL_WRUNLOCK);
	regmap_read(pll->regmap, RCM_PLL_WR_LOCK, &val);
	if (val == 0) {
		regmap_update_bits(pll->regmap, RCM_PLL_CTRL, BIT(1),
				   0); // disable PLL
		regmap_write(pll->regmap, RCM_PLL_WR_LOCK, 0); // lock back
	}
	spin_unlock_irqrestore(&pll->lock, flags);
}

static unsigned long rcm_get_rate_by_divisors(struct device *dev,
					      unsigned long parent,
					      unsigned int prdiv,
					      unsigned int fbdiv,
					      unsigned int psdiv)
{
	unsigned long rate = 0;
	unsigned int postdiv = 1;

	if ((prdiv < 1) || (prdiv > 31)) {
		dev_err(dev, "illegal PRDIV value %d", prdiv);
		return 0;
	}

	if ((fbdiv < 8) || (fbdiv > 511)) {
		dev_err(dev, "illegal FBDIV value %d", fbdiv);
		return 0;
	}

	if ((psdiv > 3)) {
		dev_err(dev, "illegal PSDIV value %d", psdiv);
		return 0;
	}

	switch (psdiv) {
	default:
	case 0:
		postdiv = 1;
		break;
	case 1:
		postdiv = 2;
		break;
	case 2:
		postdiv = 4;
		break;
	case 3:
		postdiv = 8;
		break;
	}

	rate = parent / prdiv * fbdiv / postdiv;
	//TRACE("from %ld with pr:%d, fb:%d, ps:%d to %ld", parent, prdiv, fbdiv, psdiv, rate);

	return rate;
}

static unsigned long rcm_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct rcm_pll *pll = to_pll(hw);
	unsigned long rate;
	unsigned int psdiv, fbdiv, prdiv;

	if (regmap_read(pll->regmap, RCM_PLL_PRDIV, &prdiv))
		return 0;
	if (regmap_read(pll->regmap, RCM_PLL_FBDIV, &fbdiv))
		return 0;
	if (regmap_read(pll->regmap, RCM_PLL_PSDIV, &psdiv))
		return 0;

	rate = rcm_get_rate_by_divisors(pll->dev, parent_rate, prdiv, fbdiv,
					psdiv);

	pll->current_freq = rate;

	TRACE("%ld", rate);

	return rate;
}

static long rcm_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	struct rcm_pll *pll = to_pll(hw);
	int i;
	unsigned long nearest_freq = pll->current_freq;
	for (i = 0; i < pll->freq_num; i++) {
		unsigned int psdiv, fbdiv, prdiv, new_rate;
		psdiv = pll->freqs[i].psdiv;
		prdiv = pll->freqs[i].prdiv;
		fbdiv = pll->freqs[i].fbdiv;
		new_rate = rcm_get_rate_by_divisors(pll->dev, *prate, prdiv,
						    fbdiv, psdiv);
		if (!new_rate)
			return 0;

		if (abs((long)new_rate - rate) <
		    abs((long)new_rate - nearest_freq))
			nearest_freq = new_rate;
	}
	TRACE("requested %ld, calculated %ld", rate, nearest_freq);

	return nearest_freq;
}

static int rcm_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct rcm_pll *pll = to_pll(hw);
	unsigned long flags;
	int ret = -EINVAL;
	int i;

	TRACE("%ld, %ld", rate, parent_rate);

	for (i = 0; i < pll->freq_num; i++) {
		unsigned int psdiv, fbdiv, prdiv, new_rate;
		new_rate = rcm_get_rate_by_divisors(pll->dev, parent_rate,
						    pll->freqs[i].prdiv,
						    pll->freqs[i].fbdiv,
						    pll->freqs[i].psdiv);
		if (rate == new_rate) {
			// first check that we really need to change frequency
			regmap_read(pll->regmap, RCM_PLL_FBDIV, &fbdiv);
			regmap_read(pll->regmap, RCM_PLL_PSDIV, &psdiv);
			regmap_read(pll->regmap, RCM_PLL_PRDIV, &prdiv);
			if (pll->freqs[i].fbdiv != fbdiv ||
			    pll->freqs[i].prdiv != prdiv ||
			    pll->freqs[i].psdiv != psdiv) {
				unsigned int val = 1;

				TRACE("set new values %d, %d, %d",
				      pll->freqs[i].prdiv, pll->freqs[i].fbdiv,
				      pll->freqs[i].psdiv);
				spin_lock_irqsave(&pll->lock, flags);

				regmap_write(pll->regmap, RCM_PLL_WR_LOCK,
					     RCM_PLL_WRUNLOCK);
				regmap_read(pll->regmap, RCM_PLL_WR_LOCK, &val);
				if (val == 0) {
					regmap_read(pll->regmap, RCM_PLL_CTRL,
						    &val);
					TRACE("current CTRL: %08x", val);
					regmap_read(pll->regmap,
						    RCM_PLL_RST_CTRL, &val);
					TRACE("current RST_CTRL: %x", val);
					// write new data to PLL control
					regmap_write(pll->regmap, RCM_PLL_FBDIV,
						     pll->freqs[i].fbdiv);
					regmap_write(pll->regmap, RCM_PLL_PRDIV,
						     pll->freqs[i].prdiv);
					regmap_write(pll->regmap, RCM_PLL_PSDIV,
						     pll->freqs[i].psdiv);

					regmap_update_bits(pll->regmap,
							   RCM_PLL_CTRL, 1,
							   1); // restart PLL
					regmap_write(pll->regmap,
						     RCM_PLL_WR_LOCK,
						     0); // lock back

					if (!rcm_pll_wait_for_stable(pll)) {
						ret = 0;
						pll->current_freq = rate;
					}

					if (ret) {
						TRACE("failed to update PLL frequency");
					}
				} else
					dev_err(pll->dev,
						"unable to unlock PLL registers");

				spin_unlock_irqrestore(&pll->lock, flags);
			}
		}
	}
	return ret;
}

static const struct clk_ops rcm_pll_ops = {
	.enable = rcm_pll_enable,
	.disable = rcm_pll_disable,
	.recalc_rate = rcm_pll_recalc_rate,
	.round_rate = rcm_pll_round_rate,
	.set_rate = rcm_pll_set_rate,
};

static int rcm_pll_probe(struct platform_device *pdev)
{
	struct rcm_pll *d;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const char *clk_name;
	const char *parent_name;
	struct clk_init_data init;
	struct clk *clk;
	struct resource *res;
	struct property *prop;
	int sz, count;

	TRACE("");

	prop = of_find_property(node, "freqs", &sz);
	if (!prop) {
		dev_err(dev, "failed to find frequency table");
		return -ENOMEM;
	}

	count = sz / sizeof(struct rcm_pll_freq_mode);

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->freqs = devm_kzalloc(dev,
				(count + 1) * sizeof(struct rcm_pll_freq_mode),
				GFP_KERNEL);
	if (!d->freqs)
		return -ENOMEM;
	d->freq_num = count;

	platform_set_drvdata(pdev, d);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	d->regmap = devm_regmap_init_mmio(dev, base, &rcm_pll_regmap_config);
	if (IS_ERR(d->regmap))
		return PTR_ERR(d->regmap);

	if (of_property_read_u32_array(pdev->dev.of_node, "freqs",
				       (u32 *)d->freqs,
				       sz / sizeof(unsigned int))) {
		dev_err(dev, "failed to read frequency table");
		return -EINVAL;
	}

	memset(&init, 0, sizeof(init));
	clk_name = node->name;
	init.name = clk_name;
	init.ops = &rcm_pll_ops;
	parent_name = of_clk_get_parent_name(node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;
	d->hw.init = &init;
	d->dev = dev;

	spin_lock_init(&d->lock);

	clk = devm_clk_register(dev, &d->hw);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to register %s clock (%ld)\n", clk_name,
			PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static const struct of_device_id rcm_pll_dt_ids[] = { {
							      .compatible =
								      "rcm,pll",
						      },
						      {} };
MODULE_DEVICE_TABLE(of, rcm_pll_dt_ids);

static struct platform_driver rcm_pll_driver = {
	.driver =
		{
			.name = "rcm-pll",
			.of_match_table = rcm_pll_dt_ids,
		},
	.probe = rcm_pll_probe,
};

module_platform_driver(rcm_pll_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM PLL driver");
MODULE_LICENSE("GPL");
