/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <alexeis@astrosoft.ru>
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/cfi.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#ifdef CONFIG_PPC_DCR
#include <asm/dcr.h>
#endif

// #define NOR_DEBUG

#define NORMC_ID_VAL_lsif 0x20524F4E
#define NORMC_ID_VAL_mcif 0x524f4e53
#define NORMC_VERSION_VAL 0x00000201
#define SRAMNOR__chip_num_i 0
#define SRAMNOR__ce_mode_i 8
#define SRAMNOR__addr_size_i 16
#define SRAMNOR__ecc_mode_i 24
#define SRAMNOR_TIMINGS 0x1f1f0808

#define SRAMNOR_REG_id 0x00
#define SRAMNOR_REG_version 0x04
#define SRAMNOR_REG_config 0x08
#define SRAMNOR_REG_timings 0x0c
#define SRAMNOR_REG_mask_irq 0x10
#define SRAMNOR_REG_status_irq 0x14
#define SRAMNOR_REG_ecc_err_addr 0x18
#define SRAMNOR_REG_reserve 0x1c
#define SRAMNOR_REG_management_ecc 0x20
#define SRAMNOR_REG_data_ecc_write_mem 0x24
#define SRAMNOR_REG_data_ecc_read_mem 0x28

typedef u32 (*reg_readl_fn)(void *rcm_mtd, u32 offset);
typedef void (*reg_writel_read_fn)(void *rcm_mtd, u32 offset, u32 val);

struct rcm_mtd {
	void *regs;
#ifdef CONFIG_PPC_DCR
	dcr_host_t dcr_host;
#endif /* CONFIG_PPC_DCR */
	u32 high_addr;
	reg_readl_fn readl;
	reg_writel_read_fn writel;
};

u32 lsif_reg_readl(void *base, u32 offset)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	return readl(rcm_mtd->regs + offset);
}

void lsif_reg_writel(void *base, u32 offset, u32 val)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	writel(val, rcm_mtd->regs + offset);
}

#ifdef CONFIG_PPC_DCR
u32 dcr_reg_readl(void *base, u32 offset)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	return dcr_read(rcm_mtd->dcr_host, offset);
}

void dcr_reg_writel(void *base, u32 offset, u32 val)
{
	struct rcm_mtd *rcm_mtd = (struct rcm_mtd *)base;
	dcr_write(rcm_mtd->dcr_host, offset, val);
}
#endif

static int rcm_controller_setup(struct platform_device *pdev)
{
	struct rcm_mtd *rcm_mtd = platform_get_drvdata(pdev);
	u32 chip_num, timings, cs_mode, addr_size;
	int ret;

	// get mux mode, chip-num, cs_mode and timings
	ret = of_property_read_u32(pdev->dev.of_node, "chip-num", &chip_num);
	if (ret != 0) {
		dev_err(&pdev->dev, "chip-num must be defined\n");
		return ret;
	}
	if (chip_num > 0x2F) {
		dev_err(&pdev->dev, "chip_num must be between 0 and 0x2F\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "cs-mode", &cs_mode);
	if (ret != 0) {
		dev_err(&pdev->dev, "cs_mode must be defined\n");
		return ret;
	}
	if (cs_mode > 1) {
		dev_err(&pdev->dev, "cs-mode must be 0 or 1\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "addr-size", &addr_size);
	if (ret != 0) {
		dev_err(&pdev->dev, "addr-size must be defined\n");
		return ret;
	}
	if (addr_size < 10 || addr_size > 26) {
		dev_err(&pdev->dev, "addr-size must be between 10 and 26\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "timings",
				 &timings)) {
		dev_warn(&pdev->dev, "Setup default NOR timings\n");
		timings = SRAMNOR_TIMINGS;
	}

	{
		// check id and version first
		u32 id = rcm_mtd->readl(rcm_mtd, SRAMNOR_REG_id);
		u32 version = rcm_mtd->readl(rcm_mtd, SRAMNOR_REG_version);
		u32 config = rcm_mtd->readl(rcm_mtd, SRAMNOR_REG_config);
#ifdef NOR_DEBUG
		printk("TRACE: rcm_controller_setup: id %x, ver %x, conf %x",
		       id, version, config);
#endif
		if ((id != NORMC_ID_VAL_lsif && id != NORMC_ID_VAL_mcif) ||
		    version != NORMC_VERSION_VAL)
			dev_warn(&pdev->dev,
				 "Check chip ID (%x) and version (%x)\n", id,
				 version);

		// configure controller
		config = (chip_num << SRAMNOR__chip_num_i) |
			 (cs_mode << SRAMNOR__ce_mode_i) |
			 (addr_size << SRAMNOR__addr_size_i) |
			 (0 << SRAMNOR__ecc_mode_i); // switch off ECC

#ifdef NOR_DEBUG
		printk("TRACE: rcm_controller_setup: write config %x", config);
#endif
		rcm_mtd->writel(rcm_mtd, SRAMNOR_REG_config, config);

		rcm_mtd->writel(rcm_mtd, SRAMNOR_REG_reserve,
				rcm_mtd->high_addr << 28);

		// setup timings
		rcm_mtd->writel(rcm_mtd, SRAMNOR_REG_timings, timings);
	}

	return 0;
}

static int rcm_mtd_probe(struct platform_device *pdev)
{
	struct rcm_mtd *rcm_mtd;
	struct resource *ctrl;

	rcm_mtd = devm_kzalloc(&pdev->dev, sizeof(struct rcm_mtd), GFP_KERNEL);
	if (!rcm_mtd)
		return -ENOMEM;

	platform_set_drvdata(pdev, rcm_mtd);

	if (of_property_read_bool(pdev->dev.of_node, "dcr-reg")) {
#ifdef CONFIG_PPC_DCR
		const u32 *ranges;
		u32 rlen;
		phys_addr_t phys_addr =
			dcr_resource_start(pdev->dev.of_node, 0);
		rcm_mtd->dcr_host =
			dcr_map(pdev->dev.of_node, phys_addr,
				dcr_resource_len(pdev->dev.of_node, 0));
		rcm_mtd->readl = &dcr_reg_readl;
		rcm_mtd->writel = &dcr_reg_writel;

		ranges = of_get_property(pdev->dev.of_node, "ranges", &rlen);
		if (!ranges || rlen < 2) {
			dev_err(&pdev->dev, "memory ranges must be defined\n");
			return -ENOENT;
		}
		rcm_mtd->high_addr =
			ranges[1]; // save high addr of ranges for controller setup

#else
		dev_err(&pdev->dev, "DCR functionality not avaliable\n");
		return -ENOENT;
#endif
	} else {
		ctrl = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!ctrl) {
			dev_err(&pdev->dev, "failed to get control resource\n");
			return -ENOENT;
		}
		rcm_mtd->regs = devm_ioremap_resource(&pdev->dev, ctrl);
		if (IS_ERR(rcm_mtd->regs))
			return PTR_ERR(rcm_mtd->regs);

		rcm_mtd->readl = &lsif_reg_readl;
		rcm_mtd->writel = &lsif_reg_writel;
	}

	if (rcm_controller_setup(pdev)) {
		dev_err(&pdev->dev, "hw setup failed\n");
		return -ENXIO;
	}

	dev_info(&pdev->dev, "registered\n");
	return 0;
}

static int rcm_mtd_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rcm_mtd_match[] = {
	{ .compatible = "rcm,sram-nor" },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_mtd_match);

static struct platform_driver rcm_mtd_driver = {
	.probe = rcm_mtd_probe,
	.remove = rcm_mtd_remove,
	.driver =
		{
			.name = "rcm-sram-nor",
			.of_match_table = rcm_mtd_match,
		},
};

module_platform_driver(rcm_mtd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC SRAM/NOR controller driver");
