// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
//#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/regmap.h>
#include <linux/io.h>

#ifdef CONFIG_BASIS_PLATFORM
#	include "../../../misc/rcm/basis/basis-device.h"
#	include "../../../misc/rcm/basis/basis-controller.h"
#	include "../../../misc/rcm/basis/basis-cfs.h"
#endif

#define PHY_ID_MASK  0x1F
#define PHY_REG_MASK 0x1F

#define RCM_MDIO_ID            0x00
#define RCM_MDIO_VERSION       0x04
#define RCM_MDIO_STATUS        0x08
#define RCM_MDIO_IRQ_MASK      0x0C
#define RCM_MDIO_PHY_IRQ_STATE 0x10
#define RCM_MDIO_CONTROL       0x14
#define RCM_MDIO_ETH_RST_N     0x18
#define RCM_MDIO_FREQ_DIVIDER  0x1C
#define RCM_MDIO_EN            0x20

#define RCM_START_WR 0x00
#define RCM_START_RD 0x01
#define RCM_BUSY     0x02

struct rcm_mdio_data {
	struct regmap  *reg;
	struct device  *dev;
	struct mii_bus *bus;
#ifdef CONFIG_BASIS_PLATFORM
	struct basis_device    *device;
	u32                     regs;
	u32                     regs_size;
	u32                     sctl;
	u32                     sctl_size;
#endif
};

static int rcm_mdio_reset(struct mii_bus *bus)
{
	struct rcm_mdio_data *data = bus->priv;
	int ret;

	ret = regmap_write(data->reg, RCM_MDIO_ETH_RST_N, 1);

	if (ret < 0)
		return ret;

	ret = regmap_write(data->reg, RCM_MDIO_EN, 1);

	if (ret < 0)
		return ret;

	return 0;
}

static int rcm_mdio_read(struct mii_bus *bus, int phy_id, int phy_reg)
{
	struct rcm_mdio_data *data = bus->priv;
	u32 val;
	int ret;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	ret = regmap_read_poll_timeout(data->reg, RCM_MDIO_CONTROL, val,
	                               ((val & BIT(RCM_BUSY)) == 0),
	                               1000, 1000000);

	if (ret < 0) {
		if (ret == -ETIMEDOUT)
			pr_err("%s: Failed to MDIO-read (timeout expired).\n",
			       __func__);
		return ret;
	}

	ret = regmap_write(data->reg, RCM_MDIO_CONTROL,
	                   BIT(RCM_START_RD) | phy_id << 3 | phy_reg << 8);

	if (ret < 0)
		return ret;

	ret = regmap_read_poll_timeout(data->reg, RCM_MDIO_CONTROL, val,
	                               ((val & BIT(RCM_BUSY)) == 0),
	                               1000, 1000000);

	if (ret < 0) {
		if (ret == -ETIMEDOUT)
			pr_err("%s: Failed to MDIO-read (timeout expired).\n",
			       __func__);
		return ret;
	}

	ret = regmap_read(data->reg, RCM_MDIO_CONTROL, &val);

	if (ret < 0)
		return ret;

	return val >> 16;
}

static int rcm_mdio_write(struct mii_bus *bus, int phy_id,
                          int phy_reg, u16 phy_data)
{
	struct rcm_mdio_data *data = bus->priv;
	u32 val;
	int ret;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	ret = regmap_read_poll_timeout(data->reg, RCM_MDIO_CONTROL, val,
	                               ((val & BIT(RCM_BUSY)) == 0),
	                               1000, 1000000);

	if (ret < 0) {
		if (ret == -ETIMEDOUT)
			pr_err("%s: Failed to MDIO-read (timeout expired).\n",
			       __func__);
		return ret;
	}

	ret = regmap_write(data->reg, RCM_MDIO_CONTROL,
	                   BIT(RCM_START_WR) | phy_id << 3 | phy_reg << 8 | 
	                   phy_data << 16);

	if (ret < 0)
		return ret;

	return 0;
}

static const struct regmap_config rcm_mdio_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RCM_MDIO_EN,
	.fast_io = true,
};

#ifdef CONFIG_BASIS_PLATFORM

static int rcm_mdio_bind(struct basis_device *device)
{
	struct device *dev = &device->dev;
	struct rcm_mdio_data *data = basis_device_get_drvdata(device);
	void __iomem *base;
	struct phy_device *phy;
	int ret;
	int addr;
	u32 id;
	u32 ver;

	dev_info(dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	data->bus = devm_mdiobus_alloc(dev);
	if (!data->bus) {
		dev_err(dev, "failed to alloc mii bus\n");
		return -ENOMEM;
	}

	snprintf(data->bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	data->bus->name		= device->name;
	data->bus->read		= rcm_mdio_read,
	data->bus->write	= rcm_mdio_write,
	data->bus->reset	= rcm_mdio_reset,
	data->bus->parent	= dev;
	data->bus->priv		= data;

	dev_set_drvdata(dev, data);
	data->dev = dev;

	base = devm_ioremap(dev, data->regs + device->controller->ep_base_phys,
	                    data->regs_size);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init(dev, &basis_regmap_bus, base,
	                             &rcm_mdio_regmap_config);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	regmap_read(data->reg, RCM_MDIO_ID,      &id);
	regmap_read(data->reg, RCM_MDIO_VERSION, &ver);

	dev_info(dev, "ID: 0x%08X, conf: %u, vers: %u, freq: %u.\n", 
	         id, ver & 0xFF, (ver & 0xFF00) >> 8, (ver & 0x0FFF0000) >> 16);

	regmap_write(data->reg, RCM_MDIO_IRQ_MASK, 0);

	ret = mdiobus_register(data->bus);
	if (ret) {
		dev_err(dev, "failed to register mdio-bus.\n");
		return ret;
	}

	/* scan and dump the bus */
	for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
		phy = mdiobus_get_phy(data->bus, addr);
		if (phy) {
			device->priv = phy;
			dev_info(dev, "phy[%d]: device %s, driver %s\n",
			         phy->mdio.addr, phydev_name(phy),
			         phy->drv ? phy->drv->name : "unknown");
		}
	}

	dev_info(dev, "initialized\n");

	return 0;
}

static void rcm_mdio_unbind(struct basis_device *device)
{
	struct rcm_mdio_data *data = basis_device_get_drvdata(device);

	device->priv = NULL;

	if (data->bus)
		mdiobus_unregister(data->bus);
}

static int rcm_mdio_probe(struct basis_device *device)
{
	struct rcm_mdio_data *data;
	struct device *dev = &device->dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->device = device;

	basis_device_set_drvdata(device, data);

	return 0;
}

static const struct basis_device_id rcm_mdio_ids[] = {
	{
		.name = "rcm-mdio",
	},
	{},
};

static struct basis_device_ops rcm_mdio_ops = {
	.unbind = rcm_mdio_unbind,
	.bind   = rcm_mdio_bind,
};

BASIS_DEV_ATTR_U32_SHOW(rcm_mdio_,  regs,        struct rcm_mdio_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mdio_, regs,        struct rcm_mdio_data);

BASIS_DEV_ATTR_U32_SHOW(rcm_mdio_,  regs_size,   struct rcm_mdio_data);
BASIS_DEV_ATTR_U32_STORE(rcm_mdio_, regs_size,   struct rcm_mdio_data);

CONFIGFS_ATTR(rcm_mdio_, regs);
CONFIGFS_ATTR(rcm_mdio_, regs_size);

static struct configfs_attribute *rcm_mdio_attrs[] = {
	&rcm_mdio_attr_regs,
	&rcm_mdio_attr_regs_size,
	NULL,
};

static struct basis_device_driver rcm_mdio_driver = {
	.driver.name    = "rcm_mdio",
	.probe          = rcm_mdio_probe,
	.id_table       = rcm_mdio_ids,
	.ops            = &rcm_mdio_ops,
	.owner          = THIS_MODULE,
	.attrs          = rcm_mdio_attrs,
};
module_basis_driver(rcm_mdio_driver);

#else /* CONFIG_BASIS_PLATFORM */

static const struct of_device_id rcm_mdio_of_mtable[];

static int rcm_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rcm_mdio_data *data;
	struct resource *res;
	void __iomem *base;
	struct phy_device *phy;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	int ret;
	int addr;
	u32 id;
	u32 ver;

	match = of_match_device(rcm_mdio_of_mtable, dev);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bus = devm_mdiobus_alloc(dev);
	if (!data->bus) {
		dev_err(dev, "failed to alloc mii bus\n");
		return -ENOMEM;
	}

	snprintf(data->bus->id, MII_BUS_ID_SIZE, "%s", pdev->name);

	data->bus->name		= dev_name(dev);
	data->bus->read		= rcm_mdio_read,
	data->bus->write	= rcm_mdio_write,
	data->bus->reset	= rcm_mdio_reset,
	data->bus->parent	= dev;
	data->bus->priv		= data;

	dev_set_drvdata(dev, data);
	data->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(base);
	}

	data->reg = devm_regmap_init_mmio(dev, base, &rcm_mdio_regmap_config);
	if (IS_ERR(data->reg)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(data->reg);
	}

	regmap_read(data->reg, RCM_MDIO_ID,      &id);
	regmap_read(data->reg, RCM_MDIO_VERSION, &ver);

	dev_info(dev, "ID: 0x%08X, conf: %u, vers: %u, freq: %u.\n", 
	         id, ver & 0xFF, (ver & 0xFF00) >> 8, (ver & 0x0FFF0000) >> 16);

	regmap_write(data->reg, RCM_MDIO_IRQ_MASK, 0);

	ret = of_mdiobus_register(data->bus, np);
	if (ret) {
		dev_err(dev, "failed to register mdio-bus.\n");
		return ret;
	}

	/* scan and dump the bus */
	for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
		phy = mdiobus_get_phy(data->bus, addr);
		if (phy) {
			dev_info(dev, "phy[%d]: device %s, driver %s\n",
			         phy->mdio.addr, phydev_name(phy),
			         phy->drv ? phy->drv->name : "unknown");
		}
	}

	dev_info(dev, "initialized\n");

	return 0;
}

static int rcm_mdio_remove(struct platform_device *pdev)
{
	struct rcm_mdio_data *data = platform_get_drvdata(pdev);

	if (data->bus)
		mdiobus_unregister(data->bus);

	return 0;
}

static const struct of_device_id rcm_mdio_of_mtable[] = {
	{ .compatible = "rcm,mdio", },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_mdio_of_mtable);

static struct platform_driver rcm_mdio_driver = {
	.driver = {
		.name           = "rcm_mdio",
		.of_match_table = rcm_mdio_of_mtable,
	},
	.probe = rcm_mdio_probe,
	.remove = rcm_mdio_remove,
};

module_platform_driver(rcm_mdio_driver);

#endif /* CONFIG_BASIS_PLATFORM */

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RCM MDIO driver");
