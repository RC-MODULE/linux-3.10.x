/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <alexeis@astrosoft.ru>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#ifdef CONFIG_PPC_DCR
#include <asm/dcr.h>
#endif

#define CONTROL_REG_EXT_MEM_MUX 0x30
#define SRAM_NOR_CTRL 0x1C
#define CONTROL_REG_CE_MANAGE 0x04
#define EM2_PLB6MCIF2_DCR_BASE 0x80160000
#define EM3_PLB6MCIF2_DCR_BASE 0x80180000

#ifdef CONFIG_PPC_DCR
// briges magic
static void plb6mcif_initbridge(void)
{
	mtdcrx(EM2_PLB6MCIF2_DCR_BASE + 0x0f, 0x00000000);
	mtdcrx(EM2_PLB6MCIF2_DCR_BASE + 0x10, 0x00000004);
	mtdcrx(EM2_PLB6MCIF2_DCR_BASE + 0x11, 0x40009001);
	mtdcrx(EM2_PLB6MCIF2_DCR_BASE + 0x09, 0x400000f1);

	mtdcrx(EM3_PLB6MCIF2_DCR_BASE + 0x0f, 0x00000000);
	mtdcrx(EM3_PLB6MCIF2_DCR_BASE + 0x10, 0x00000006);
	mtdcrx(EM3_PLB6MCIF2_DCR_BASE + 0x11, 0x60009001);
	mtdcrx(EM3_PLB6MCIF2_DCR_BASE + 0x09, 0x400000f1);
}
#endif

static int rcm_mtd_arbiter_probe(struct platform_device *pdev)
{
	struct device_node *tmp;
	struct regmap *control;
	struct regmap *sctl;
	u32 sram_nor_mux = 0;
	u32 ext_mem_mux_mode;
	u32 ce_manage;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "sram-nor-mux",
				   &sram_nor_mux);
	if (ret != 0 || sram_nor_mux > 1) {
		dev_err(&pdev->dev,
			"sram-nor-mux should be explicitly defined to 0 or 1\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ce-manage", &ce_manage);
	if (ret != 0 || ce_manage > 7) {
		dev_err(&pdev->dev,
			"ce-manage should be explicitly defined between 0 to 7\n");
		return ret;
	}

	tmp = of_parse_phandle(pdev->dev.of_node, "sctl", 0);
	if (!tmp) {
		dev_err(&pdev->dev, "failed to find sctl register reference\n");
		return -ENOENT;
	}
	sctl = syscon_node_to_regmap(tmp);

	if (sram_nor_mux == 1) {
#ifdef CONFIG_PPC_DCR
		plb6mcif_initbridge();
#endif
		ret = regmap_write(sctl, SRAM_NOR_CTRL, ce_manage << 1 | 1);
		if (ret != 0) {
			dev_err(&pdev->dev, "Write SCTL register error %i\n",
				ret);
			return ret;
		}
	} else {
		if (ce_manage != 7)
			ce_manage = 6 - ce_manage;

		tmp = of_parse_phandle(pdev->dev.of_node, "control", 0);
		if (!tmp) {
			dev_err(&pdev->dev,
				"failed to find control register reference\n");
			return -ENOENT;
		}
		control = syscon_node_to_regmap(tmp);

		ret = regmap_write(sctl, SRAM_NOR_CTRL, 0);
		if (ret != 0) {
			dev_err(&pdev->dev, "Write SCTL register error %i\n",
				ret);
			return ret;
		}

		ret = of_property_read_u32(pdev->dev.of_node,
					   "ext-mem-mux-mode",
					   &ext_mem_mux_mode);
		if (ret == 0) {
			if (ext_mem_mux_mode < 0 || ext_mem_mux_mode > 2) {
				dev_err(&pdev->dev,
					"Illegal ext-mem-mux-mode shuld be between 0 and 2\n");
				return -EINVAL;
			}
			ret = regmap_write(control, CONTROL_REG_EXT_MEM_MUX,
					   ext_mem_mux_mode);
			if (ret != 0) {
				dev_err(&pdev->dev,
					"Write MII_MUX register error %i\n",
					ret);
				return ret;
			}
		}

		ret = regmap_write(control, CONTROL_REG_CE_MANAGE, ce_manage);
		if (ret != 0) {
			dev_err(&pdev->dev, "Write MII_MUX register error %i\n",
				ret);
			return ret;
		}
	}
	dev_info(&pdev->dev, "External memory signals routed to %s\n",
		 sram_nor_mux ? "MCIF" : "LSIF0");

	if (ce_manage == 7)
		dev_info(&pdev->dev, "SRAM/NOR chip selects disabled\n");
	else
		dev_info(
			&pdev->dev,
			"%d SRAM/NOR chip selects allocated to first controller\n",
			sram_nor_mux ? 6 - ce_manage : ce_manage);

	return 0;
}

static int rcm_mtd_arbiter_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rcm_mtd_arbiter_match[] = {
	{ .compatible = "rcm,sram-nor-arbiter" },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_mtd_match);

static struct platform_driver rcm_mtd_driver = {
	.probe = rcm_mtd_arbiter_probe,
	.remove = rcm_mtd_arbiter_remove,
	.driver =
		{
			.name = "rcm-sram-nor-arbiter",
			.of_match_table = rcm_mtd_arbiter_match,
		},
};

module_platform_driver(rcm_mtd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC SRAM/NOR controller arbiter driver");
