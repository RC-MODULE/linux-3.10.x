// SPDX-License-Identifier: GPL-2.0
/**
 * Basis Device library
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "basis-controller.h"
#include "basis-device.h"
#include "basis-cfs.h"

static DEFINE_MUTEX(basis_device_mutex);

static struct bus_type basis_device_bus_type;
static const struct device_type basis_device_type;

static int basis_regmap_read(void *context, unsigned int reg, unsigned int *val)
{
	void __iomem *base = context;

	*val = readl(base + reg);

	return 0;
}

static int basis_regmap_write(void *context, unsigned int reg, unsigned int val)
{
	void __iomem *base = context;

	writel(val, base + reg);

	return 0;
}

struct regmap_bus basis_regmap_bus = {
	.reg_write = basis_regmap_write,
	.reg_read = basis_regmap_read,
};
EXPORT_SYMBOL_GPL(basis_regmap_bus);

struct basis_driver_find_device_data
{
	const char*          name;
	struct basis_device *device;
};

static int basis_driver_find_device(struct device_driver *drv, void* d)
{
	struct basis_device_driver *driver = to_basis_device_driver(drv);
	struct basis_driver_find_device_data* data = d;
	struct config_group *group;
	struct config_item *item;

	list_for_each_entry(group, &driver->device_group, group_entry) {
		item = config_group_find_item(group, data->name);
		if (item) {
			data->device = config_item_to_basis_device(item);
			config_item_put(item);
			return 1;
		}
	}

	return 0;
};

struct basis_device *basis_device_find(const char *name)
{
	struct basis_driver_find_device_data data;
	data.name = name;
	data.device = NULL;

	bus_for_each_drv(&basis_device_bus_type, NULL, &data,
	                 basis_driver_find_device);

	return data.device;
}
EXPORT_SYMBOL_GPL(basis_device_find);

void basis_device_unbind(struct basis_device *device)
{
	if (!device->driver) {
		dev_WARN(&device->dev, "BASIS device not bound to driver\n");
		return;
	}

	device->driver->ops->unbind(device);
	module_put(device->driver->owner);
}
EXPORT_SYMBOL_GPL(basis_device_unbind);

int basis_device_bind(struct basis_device *device)
{
	if (!device->driver) {
		dev_WARN(&device->dev, "BASIS device not bound to driver\n");
		return -EINVAL;
	}

	if (!try_module_get(device->driver->owner))
		return -EAGAIN;

	return device->driver->ops->bind(device);
}
EXPORT_SYMBOL_GPL(basis_device_bind);

static void basis_device_remove_cfs(struct basis_device_driver *driver)
{
	struct config_group *group, *tmp;

	mutex_lock(&basis_device_mutex);
	list_for_each_entry_safe(group, tmp, &driver->device_group, group_entry)
		basis_cfs_remove_device_group(group);
	list_del(&driver->device_group);
	mutex_unlock(&basis_device_mutex);
}

void basis_device_unregister_driver(struct basis_device_driver *driver)
{
	basis_device_remove_cfs(driver);
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(basis_device_unregister_driver);

static int basis_device_add_cfs(struct basis_device_driver *driver)
{
	struct config_group *group;
	const struct basis_device_id *id;

	INIT_LIST_HEAD(&driver->device_group);

	id = driver->id_table;
	while (id->name[0]) {
		group = basis_cfs_add_device_group(id->name);
		if (IS_ERR(group)) {
			basis_device_remove_cfs(driver);
			return PTR_ERR(group);
		}

		mutex_lock(&basis_device_mutex);
		list_add_tail(&group->group_entry, &driver->device_group);
		mutex_unlock(&basis_device_mutex);
		id++;
	}

	return 0;
}

int __basis_device_register_driver(struct basis_device_driver *driver,
                                   struct module *owner)
{
	int ret;

	if (!driver->ops)
		return -EINVAL;

	if (!driver->ops->bind || !driver->ops->unbind)
		return -EINVAL;

	driver->driver.bus = &basis_device_bus_type;
	driver->driver.owner = owner;

	ret = driver_register(&driver->driver);
	if (ret)
		return ret;

	basis_device_add_cfs(driver);

	return 0;
}
EXPORT_SYMBOL_GPL(__basis_device_register_driver);

void basis_device_destroy(struct basis_device *device)
{
	pr_info("%s >>>\n", __func__);
	device_unregister(&device->dev);
}
EXPORT_SYMBOL_GPL(basis_device_destroy);

struct basis_device *basis_device_create(const char *name)
{
	int ret;
	struct basis_device *device;
	struct device *dev;
	int len;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device)
		return ERR_PTR(-ENOMEM);

	len = strchrnul(name, '.') - name;
	device->name = kstrndup(name, len, GFP_KERNEL);
	if (!device->name) {
		kfree(device);
		return ERR_PTR(-ENOMEM);
	}

	dev = &device->dev;
	device_initialize(dev);
	dev->bus = &basis_device_bus_type;
	dev->type = &basis_device_type;

	ret = dev_set_name(dev, "%s", name);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = device_add(dev);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}

	return device;
}
EXPORT_SYMBOL_GPL(basis_device_create);

const struct basis_device_id *
basis_device_match_device(const struct basis_device_id *id,
                          struct basis_device *device)
{
	if (!id || !device)
		return NULL;

	while (*id->name) {
		if (strcmp(device->name, id->name) == 0)
			return id;
		id++;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(basis_device_match_device);

static void basis_device_dev_release(struct device *dev)
{
	struct basis_device *device = to_basis_device(dev);

	kfree(device->name);
	kfree(device);
}

struct basis_driver_find_data
{
	const char*                 name;
	struct basis_device_driver *driver;
};

static int basis_driver_match(struct device_driver *drv, void* d)
{
	struct basis_device_driver *driver = to_basis_device_driver(drv);
	struct basis_driver_find_data* data = d;
	const struct basis_device_id *id = driver->id_table;

	while (id->name[0]) {
		if (strcmp(data->name, id->name) == 0) {
			data->driver = driver;
			return 1;
		}
		id++;
	}

	return 0;
};

struct basis_device_driver *basis_driver_find(const char* group_name)
{
	struct basis_driver_find_data data;
	data.name = group_name;
	data.driver = NULL;

	bus_for_each_drv(&basis_device_bus_type, NULL, &data,
	                 basis_driver_match);

	return data.driver;
};

static const struct device_type basis_device_type = {
	.release = basis_device_dev_release,
};

static int
basis_device_match_id(const struct basis_device_id *id,
                      const struct basis_device *device)
{
	while (id->name[0]) {
		if (strcmp(device->name, id->name) == 0)
			return true;
		id++;
	}

	return false;
}

static int basis_device_match(struct device *dev, struct device_driver *drv)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_device_driver *driver = to_basis_device_driver(drv);

	if (driver->id_table)
		return basis_device_match_id(driver->id_table, device);

	return !strcmp(device->name, drv->name);
}

static int basis_device_probe(struct device *dev)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_device_driver *driver = to_basis_device_driver(dev->driver);

	if (!driver->probe)
		return -ENODEV;

	device->driver = driver;

	return driver->probe(device);
}

static int basis_device_remove(struct device *dev)
{
	int ret = 0;
	struct basis_device *device = to_basis_device(dev);
	struct basis_device_driver *driver = to_basis_device_driver(dev->driver);

	if (driver->remove)
		ret = driver->remove(device);
	device->driver = NULL;

	return ret;
}

static struct bus_type basis_device_bus_type = {
	.name   = "basis-device",
	.match  = basis_device_match,
	.probe  = basis_device_probe,
	.remove = basis_device_remove,
};

int basis_device_init(void)
{
	int ret;

	ret = bus_register(&basis_device_bus_type);
	if (ret) {
		pr_err("failed to register BASIS device  bus --> %d\n", ret);
		return ret;
	}

	return 0;
}

void basis_device_exit(void)
{
	bus_unregister(&basis_device_bus_type);
}
