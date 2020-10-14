// SPDX-License-Identifier: GPL-2.0-only
/**
 * Test RCM-BASIS device driver
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#define DEBUG
#include <linux/module.h>
#include <linux/device.h>
#include <linux/configfs.h>
#include <linux/interrupt.h>

#include "basis-device.h"
#include "basis-controller.h"
#include "basis-cfs.h"

struct dev_test_data {
	struct basis_device *device;
	unsigned             hwirq;
	unsigned             irq;
};

static irqreturn_t dev_test_irq_handler(int irq, void *dev_id)
{
	struct dev_test_data *data = dev_id;
	struct device *dev = &data->device->dev;

	dev_dbg(dev, "%s(%d) >>>\n", __func__, irq);
	dev_dbg(dev, "%s <<<\n", __func__);

	return IRQ_HANDLED;
}

static void dev_test_unbind(struct basis_device *device)
{
	struct dev_test_data *data = basis_device_get_drvdata(device);
	struct device *dev = &device->dev;

	dev_info(dev, "%s >>>\n", __func__);

	if (data->irq) {
		free_irq(data->irq, data);
		irq_dispose_mapping(data->irq);
		data->irq = 0;
	}

	dev_info(dev, "%s <<<\n", __func__);
}

static int dev_test_bind(struct basis_device *device)
{
	struct dev_test_data *data = basis_device_get_drvdata(device);
	struct device *dev = &device->dev;
	int ret;

	dev_info(dev, "%s >>>\n", __func__);

	dev_info(dev, "%s: controller: \"%s\"\n",
	         __func__, dev_name(&device->controller->dev));

	if (WARN_ON_ONCE(!device))
		return -EINVAL;

	if (device->controller->domain) {
		data->irq = irq_create_mapping(device->controller->domain,
		                               data->hwirq);

		if (data->irq == 0) {
			dev_warn(dev, "Failed to map irq #%u.\n", data->hwirq);
		} else {
			ret = request_irq(data->irq, dev_test_irq_handler, 0,
			                  "dev_test", data);
			if (ret) {
				dev_warn(dev, "Failed to request irq #%u.\n",
				         data->hwirq);
				irq_dispose_mapping(data->irq);
				data->irq = 0;
			}
		}
	}

	dev_info(dev, "%s <<<\n", __func__);

	return 0;
}

static int dev_test_probe(struct basis_device *device)
{
	struct dev_test_data *data;
	struct device *dev = &device->dev;

	dev_info(dev, "%s >>>\n", __func__);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->device = device;

	basis_device_set_drvdata(device, data);

	dev_info(dev, "%s <<<\n", __func__);

	return 0;
}

static int dev_test_remove(struct basis_device *device)
{
	struct device *dev = &device->dev;

	dev_info(dev, "%s >>>\n", __func__);
	dev_info(dev, "%s <<<\n", __func__);

	return 0;
}

static struct basis_device_ops dev_test_ops = {
	.unbind = dev_test_unbind,
	.bind   = dev_test_bind,
};

static const struct basis_device_id dev_test_ids[] = {
	{
		.name = "dev_test",
	},
	{},
};

static ssize_t dev_test_irq_store(struct config_item *item, const char *page,
                                  size_t len)
{
	struct basis_device *device = config_item_to_basis_device(item);
	struct dev_test_data *data = basis_device_get_drvdata(device);
	int ret;
	u32 val;

	ret = kstrtou32(page, 0, &val);
	if (ret)
		return ret;

	data->hwirq = val;

	return len;
}

static ssize_t dev_test_irq_show(struct config_item *item, char *page)
{
	struct basis_device *device = config_item_to_basis_device(item);
	struct dev_test_data *data = basis_device_get_drvdata(device);

	return sprintf(page, "%d\n", data->hwirq);
}

CONFIGFS_ATTR(dev_test_, irq);

static struct configfs_attribute *dev_test_attrs[] = {
	&dev_test_attr_irq,
	NULL,
};

static struct basis_device_driver dev_test_driver = {
	.driver.name    = "basis_device_test",
	.probe          = dev_test_probe,
	.remove         = dev_test_remove,
	.id_table       = dev_test_ids,
	.ops            = &dev_test_ops,
	.owner          = THIS_MODULE,
	.attrs          = dev_test_attrs,
};

static int __init dev_test_init(void)
{
	int ret;

	ret = basis_device_register_driver(&dev_test_driver);
	if (ret) {
		pr_err("Failed to register BASIS test driver --> %d\n", ret);
		return ret;
	}

	pr_info("Device test driver registered sucessfully.\n");

	return ret;
}
module_init(dev_test_init);

static void __exit dev_test_exit(void)
{
	basis_device_unregister_driver(&dev_test_driver);
}
module_exit(dev_test_exit);

MODULE_DESCRIPTION("RCM-BASIS DEVICE TEST DRIVER");
MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_LICENSE("GPL v2");
