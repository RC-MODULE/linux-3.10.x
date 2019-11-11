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

#define CONTROL_REG_CE_MANAGE   0x04
#define CONTROL_REG_EXT_MEM_MUX 0x30
#define NORMC_ID_VAL            0x20524F4E
#define NORMC_VERSION_VAL       0x00000201
#define SRAMNOR__chip_num_i     0
#define SRAMNOR__ce_mode_i      8
#define SRAMNOR__addr_size_i    16
#define SRAMNOR__ecc_mode_i     24
#define SRAMNOR_TIMINGS         0x1f1f0808

struct nor_regs {
	u32 id;
	u32 version;
	u32 config;
	u32 timings;
	u32 mask_irq;
	u32 ecc_err_addr;
	u32 reserve;
	u32 management_ecc;
	u32 data_ecc_write_mem;
	u32 data_ecc_read_mem;
};


struct rcm_mtd {
    struct nor_regs *regs;
    struct regmap   *control;    
	struct resource *res;
	struct mtd_info *mtd;
	struct map_info *map;
};

static const char rcm_map_name[] = "rcm_nor";

static int rcm_controller_setup(struct platform_device *pdev)
{
    struct rcm_mtd *rcm_mtd = platform_get_drvdata(pdev);
    struct nor_regs *regs = rcm_mtd->regs;
    u32 ext_mem_mux_mode;
    u32 cs;
	int ret;
    u32 nor_timings;


    // get mux mode, cs and timings
	ret = of_property_read_u32(pdev->dev.of_node, "nor-cs", &cs);
    if (ret != 0) {
        dev_err(&pdev->dev, "nor-cs must be defined\n");
        return ret;
    }

    if(of_property_read_u32(pdev->dev.of_node, "nor-timings", &nor_timings))
    {
        dev_warn(&pdev->dev, "Setup default NOR timings\n");
        nor_timings = SRAMNOR_TIMINGS;
    }

	ret = of_property_read_u32(pdev->dev.of_node, "ext-mem-mux-mode", &ext_mem_mux_mode);
    if (ret == 0) {
        if(ext_mem_mux_mode < 0 || ext_mem_mux_mode > 2)
        {
            dev_err(&pdev->dev, "Illegal ext-mem-mux-mode shuld be between 0 and 2\n");
            return -EINVAL;
        }
        // setup LSIF0 mux to SRAM_NOR
        // todo - move to LSIF driver
        ret = regmap_write(rcm_mtd->control, CONTROL_REG_EXT_MEM_MUX, ext_mem_mux_mode);
        if (ret != 0) {
            dev_err(&pdev->dev, "Write MII_MUX register error %i\n", ret);
            return ret;
        }
    }

    // hardcode ce_manage to share ce4/ce5 between 2 controllers
    // todo - move to LSIF driver
    ret = regmap_write(rcm_mtd->control, CONTROL_REG_CE_MANAGE, 0);
    if (ret != 0) {
        dev_err(&pdev->dev, "Write MII_MUX register error %i\n", ret);
        return ret;
    }

    {
        // check id and version first
        u32 id = readl(&regs->id);
        u32 version = readl(&regs->version);
        u32 config = readl(&regs->config);

        if(id != NORMC_ID_VAL || version != NORMC_VERSION_VAL)
            dev_warn(&pdev->dev, "Check chip ID (%x) and version (%x)\n", id, version);

        // configure controller
        config = ((1 << cs) << SRAMNOR__chip_num_i)    // setup cs 
                | (0b1 << SRAMNOR__ce_mode_i)   // setup ce according cs
                | (26 << SRAMNOR__addr_size_i)  // width of address
                | (0  << SRAMNOR__ecc_mode_i);  // switch off ECC
        writel(config, &regs->config);

        // setup timings
        writel(nor_timings, &regs->timings);
    }

    return 0;
}

static map_word
rcm_read32(struct map_info *map, unsigned long adr)
{
	unsigned long flags;
	map_word temp;

	temp.x[0] = readl(map->virt + adr);
    printk("TRACE: rcm_read32 %08x -> %08x", adr, temp.x[0]);
	return temp;
}

static void
rcm_write32(struct map_info *map, map_word d, unsigned long adr)
{
	unsigned long flags;
    printk("TRACE: rcm_write32 %08x <- %08x", adr, d.x[0]);
	writel(d.x[0], map->virt + adr);
}


static int
rcm_mtd_probe(struct platform_device *pdev)
{
	struct rcm_mtd *rcm_mtd;
	struct cfi_private *cfi;
	struct resource *ctrl;
    struct device_node *tmp;
	int err;

	rcm_mtd = devm_kzalloc(&pdev->dev, sizeof(struct rcm_mtd), GFP_KERNEL);
	if (!rcm_mtd)
		return -ENOMEM;

	platform_set_drvdata(pdev, rcm_mtd);

    ctrl = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	if (!ctrl) {
		dev_err(&pdev->dev, "failed to get control resource\n");
		return -ENOENT;
	}
    rcm_mtd->regs = devm_ioremap_resource(&pdev->dev, ctrl);
	if (IS_ERR(rcm_mtd->regs))
		return PTR_ERR(rcm_mtd->regs);

	rcm_mtd->res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "map");
	if (!rcm_mtd->res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		return -ENOENT;
	}

    tmp = of_parse_phandle(pdev->dev.of_node, "control", 0);
    if (!tmp) {
        dev_err(&pdev->dev, "failed to find control register reference\n");
		return -ENOENT;
    }
    rcm_mtd->control = syscon_node_to_regmap(tmp);

	rcm_mtd->map = devm_kzalloc(&pdev->dev, sizeof(struct map_info),
				    GFP_KERNEL);
	if (!rcm_mtd->map)
		return -ENOMEM;

	rcm_mtd->map->phys = rcm_mtd->res->start;
	rcm_mtd->map->size = resource_size(rcm_mtd->res);
	rcm_mtd->map->virt = devm_ioremap_resource(&pdev->dev, rcm_mtd->res);
	if (IS_ERR(rcm_mtd->map->virt))
		return PTR_ERR(rcm_mtd->map->virt);

	rcm_mtd->map->name = rcm_map_name;
	rcm_mtd->map->bankwidth = 4;

    if(rcm_controller_setup(pdev))
    {
		dev_err(&pdev->dev, "hw setup failed\n");
		return -ENXIO;
    }

	rcm_mtd->map->read = rcm_read32;
	rcm_mtd->map->write = rcm_write32;
	//rcm_mtd->map->copy_from = ltq_copy_from;
	//rcm_mtd->map->copy_to = ltq_copy_to;

	//rcm_mtd->map->map_priv_1 = LTQ_NOR_PROBING;
	rcm_mtd->mtd = do_map_probe("cfi_probe", rcm_mtd->map);
	//rcm_mtd->map->map_priv_1 = LTQ_NOR_NORMAL;

	if (!rcm_mtd->mtd) {
		dev_err(&pdev->dev, "probing failed\n");
		return -ENXIO;
	}

	rcm_mtd->mtd->dev.parent = &pdev->dev;
	mtd_set_of_node(rcm_mtd->mtd, pdev->dev.of_node);

	cfi = rcm_mtd->map->fldrv_priv;
	cfi->addr_unlock1 ^= 1;
	cfi->addr_unlock2 ^= 1;

	err = mtd_device_register(rcm_mtd->mtd, NULL, 0);
	if (err) {
		dev_err(&pdev->dev, "failed to add partitions\n");
		goto err_destroy;
	}

	return 0;

err_destroy:
	map_destroy(rcm_mtd->mtd);
	return err;
}





static int
rcm_mtd_remove(struct platform_device *pdev)
{
	struct rcm_mtd *rcm_mtd = platform_get_drvdata(pdev);

	if (rcm_mtd && rcm_mtd->mtd) {
		mtd_device_unregister(rcm_mtd->mtd);
		map_destroy(rcm_mtd->mtd);
	}
	return 0;
}


static const struct of_device_id rcm_mtd_match[] = {
	{ .compatible = "rcm,nor" },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_mtd_match);

static struct platform_driver rcm_mtd_driver = {
	.probe = rcm_mtd_probe,
	.remove = rcm_mtd_remove,
	.driver = {
		.name = "rcm-nor",
		.of_match_table = rcm_mtd_match,
	},
};

module_platform_driver(rcm_mtd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC NOR");
