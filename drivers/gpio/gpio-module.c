/*
 * drivers/gpio/gpio-module.c - Module pinmux control driver
 *
 *	Copyright (C) 2014 Promwad
 *	Written by Vladimir Trubiloff <vtrubiloff@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/io.h>

#include <mach/platform.h>

#define DRIVER_NAME "module,pinmux"

struct pmux_device {
	struct resource *reg;
	void __iomem *io;
};

static char *modules[] = {
	"DVB_CI",
	"TS1",
	"TS2",
	"NAND",
};

static int of_pinmux_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	unsigned int nmodes, *mode_list;
	int mask = 0;
	struct property *prop;
	struct pmux_device *device;

	if (!uemd_is_virgin()) {
		dev_err(&pdev->dev, "can't set muxers for pins: OTP ROM was flashed\n");
		goto out;
	}

	device = devm_kzalloc(&pdev->dev, sizeof(struct pmux_device),
				GFP_KERNEL);
	if (!device) {
		dev_err(&pdev->dev, "failed to allocate memory for device structure\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Parsing fields in DTS */
	device->reg = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"controlH");
	if (!device->reg) {
		dev_err(&pdev->dev, "failed to find controlH register\n");
		ret = -ENOENT;
		goto out;
	}

	prop = of_find_property(pdev->dev.of_node, "settings", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "failed to find property 'settings' in DTS\n");
		ret = -ENOENT;
		goto out;
	}

	nmodes = prop->length / sizeof(unsigned long);
	if (nmodes != ARRAY_SIZE(modules)) {
		dev_err(&pdev->dev,
				"invalid modules count. Initialization will be skipped\n");
		ret = -EINVAL;
		goto out;
	}

	mode_list = devm_kzalloc(&pdev->dev, nmodes * sizeof(*mode_list),
					GFP_KERNEL);
	if (!mode_list) {
		dev_err(&pdev->dev, "failed to allocate memory for mode list\n");
		ret = -ENOMEM;
		goto out;
	}

	of_property_read_u32_array(pdev->dev.of_node, "settings", mode_list,
		nmodes);

	/* Setting modes according to module configuration */
	for (i = 0; i < ARRAY_SIZE(modules); i++) {
		dev_info(&pdev->dev, "%s mode is %s (%d)\n", modules[i],
			mode_list[i] ? "peripheral" : "GPIO", mode_list[i]);
		mask |= ((mode_list[i] ? 1 : 0) << (i + 3));
	}

	/* Obtaining a pointer to io memory */
	device->io = devm_request_and_ioremap(&pdev->dev, device->reg);
	if (!device->io) {
		dev_err(&pdev->dev, "cant remap io memory\n");
		ret = -ENOMEM;
		goto out;
	}
	iowrite32((ioread32(device->io) & ~0x7c) | mask, device->io);

out:
	return ret;
}

static int of_pinmux_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id of_platform_pinmux_table[] = {
	{ .compatible = DRIVER_NAME },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, of_platform_pinmux_table);

static struct platform_driver of_platform_pinmux_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_platform_pinmux_table,
	},
	.probe = of_pinmux_probe,
	.remove = of_pinmux_remove
};
module_platform_driver(of_platform_pinmux_driver);

MODULE_AUTHOR("Vladimir Trubiloff <vtrubiloff@gmail.com>");
MODULE_DESCRIPTION("Module pinmux control driver");
MODULE_LICENSE("GPL");

