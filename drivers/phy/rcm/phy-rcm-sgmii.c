// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#define DEBUG

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>

#include <linux/phy/phy.h>

struct rcm_sgmii_phy_data {
	struct regmap     *reg;
	struct regmap     *ctrl;
	u32                ctrl_offset;
	struct phy        *phy;
	bool               auto_negotiation;
};

static const struct of_device_id rcm_sgmii_phy_of_table[];

static int rcm_sgmii_phy_power_on(struct phy *phy)
{
	struct rcm_sgmii_phy_data *data = phy_get_drvdata(phy);
	u32 val;
	int ret;

	ret = regmap_write(data->ctrl, data->ctrl_offset, 1);
	if(ret) {
		pr_debug("%s: failed to write to CTRL register.", __func__);
		return ret;
	}

	ret = regmap_read_poll_timeout(data->ctrl, data->ctrl_offset, val, 
	                               val == 0x000001F1, 1000, 1000000);

	if (ret < 0) {
		pr_debug("%s: failed to power on (val = 0x%X).", __func__, val);
		return -EIO;
	}

	return 0;
}

static int rcm_sgmii_phy_power_off(struct phy *phy)
{
	struct rcm_sgmii_phy_data *data = phy_get_drvdata(phy);

	return regmap_write(data->ctrl, data->ctrl_offset, 0);
}

static int rcm_sgmii_phy_init(struct phy *phy)
{
	struct rcm_sgmii_phy_data *data = phy_get_drvdata(phy);
	u32 val;

	pr_debug("%s >>> aneg = %s\n", __func__,
	         (data->auto_negotiation) ? "on" : "off");

	regmap_write(data->reg, 0x0000, 0x40803004);
	regmap_write(data->reg, 0x0400, 0x40803004);
	regmap_write(data->reg, 0x0800, 0x40803004);
	regmap_write(data->reg, 0x0C00, 0x40803004);

	regmap_write(data->reg, 0x1004, 0x00130000);
	regmap_write(data->reg, 0x1008, 0x710001F0);
	regmap_write(data->reg, 0x100C, 0x00000002);
	regmap_write(data->reg, 0x1020, 0x07000000);

	regmap_write(data->reg, 0x0108, 0x0000CEA6);
	regmap_write(data->reg, 0x0508, 0x0000CEA6);
	regmap_write(data->reg, 0x0908, 0x0000CEA6);
	regmap_write(data->reg, 0x0D08, 0x0000CEA6);

	val = (data->auto_negotiation) ? 0x00001140 : 0x00000140;

	regmap_write(data->reg, 0x0200, val);
	regmap_write(data->reg, 0x0600, val);
	regmap_write(data->reg, 0x0A00, val);
	regmap_write(data->reg, 0x0E00, val);

	return rcm_sgmii_phy_power_on(phy);
}

static int rcm_sgmii_phy_exit(struct phy *phy)
{
	return 0;
}

static struct phy_ops rcm_sgmii_phy_ops = {
	.power_on	= rcm_sgmii_phy_power_on,
	.power_off	= rcm_sgmii_phy_power_off,
	.init		= rcm_sgmii_phy_init,
	.exit		= rcm_sgmii_phy_exit,
};

static const struct regmap_config rcm_sgmii_phy_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x1FFFC,
	.fast_io = true,
};

static int rcm_sgmii_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct rcm_sgmii_phy_data *data;
	struct resource *res;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct of_phandle_args args;
	int ret;

	match = of_match_device(rcm_sgmii_phy_of_table, dev);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init_mmio(dev, base, 
	                                  &rcm_sgmii_phy_regmap_config);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	ret = of_parse_phandle_with_fixed_args(np, "sctl", 1, 0, &args);
	if (ret < 0) {
		dev_err(dev, "failed to parse sctl node\n");
		return ret;
	}

	data->ctrl = syscon_node_to_regmap(args.np);
	data->ctrl_offset = args.args[0];

	if (IS_ERR(data->ctrl)) {
		dev_err(dev, "failed to map SCTL\n");
		return PTR_ERR(data->ctrl);
	}

	data->phy = devm_phy_create(dev, NULL, &rcm_sgmii_phy_ops);
	if (IS_ERR(data->phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(data->phy);
	}

	data->auto_negotiation = 
		(of_find_property(np, "auto-negotiation", NULL) != NULL);

	phy_set_drvdata(data->phy, data);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id rcm_sgmii_phy_of_table[] = {
	{ .compatible = "rcm,sgmii-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcm_sgmii_phy_of_table);

static struct platform_driver rcm_sgmii_phy_driver = {
	.probe		= rcm_sgmii_phy_probe,
	.driver		= {
		.name		= "rcm-sgmii-phy",
		.of_match_table	= rcm_sgmii_phy_of_table,
	},
};
module_platform_driver(rcm_sgmii_phy_driver);

MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_DESCRIPTION("RC-Module SGMII PHY Driver");
MODULE_LICENSE("GPL");
