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

#ifndef BASIS_DMA_MASK_SIZE
#	define BASIS_DMA_MASK_SIZE 32
#endif

static DEFINE_MUTEX(basis_device_mutex);

static struct bus_type basis_device_bus_type;
static const struct device_type basis_device_type;

void* basis_device_dma_alloc_coherent(struct device *dev,
                                      size_t size, dma_addr_t *dma_addr,
                                      u32 *ep_addr, gfp_t gfp)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_controller* controller = device->controller;
	void* cpu_addr;
	int ret;
	size_t sz = 1ULL << fls64(size - 1);

	if (!controller) {
		dev_err(dev,
		        "No BASIS-controller bind. Can't allocate DMA memory.\n");
		return NULL;
	}

	cpu_addr = dma_alloc_coherent(dev, sz, dma_addr, gfp);

	if (!cpu_addr) {
		dev_err(dev,
		        "Failed to allocate DMA-coherent memory (%u bytes).\n",
		        (unsigned)sz);
		return NULL;
	}

	if (!IS_ALIGNED(*dma_addr, sz)) {
		dev_err(dev,
		        "Allocated DMA-buffer is not aligned as required "
		        "(dma_addr: 0x%016llx, required alignment: 0x%08x).\n",
		        (u64)(*dma_addr), (unsigned)sz);
		goto err_free_coherent;
	}

	*ep_addr = basis_controller_ep_mem_alloc_addr(controller, sz);

	if (*ep_addr == 0) {
		dev_err(dev,
		        "Failed to allocate EP-memory address (%u bytes).\n",
		        (unsigned)sz);
		goto err_free_coherent;
	}

	ret = basis_controller_ep_map_addr(controller, *ep_addr, *dma_addr, 
	                                   sz);
	if (ret) {
		dev_err(dev, "Failed to map EP-memory.\n");
		goto err_ep_mem_free;
	}

	return cpu_addr;

err_ep_mem_free:
	basis_controller_ep_mem_free_addr(controller, *ep_addr, sz);
	*ep_addr = 0;

err_free_coherent:
	dma_free_coherent(dev, sz, cpu_addr, *dma_addr);
	*dma_addr = 0;

	return NULL;
}
EXPORT_SYMBOL_GPL(basis_device_dma_alloc_coherent);

void basis_device_dma_free_coherent(struct device *dev,
                                    size_t size, void* cpu_addr,
                                    dma_addr_t dma_addr, u32 ep_addr)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_controller* controller = device->controller;
	size_t sz = 1ULL << fls64(size - 1);

	if (!controller)
		return;

	basis_controller_ep_unmap_addr(controller, ep_addr);
	basis_controller_ep_mem_free_addr(controller, ep_addr, sz);
	dma_free_coherent(dev, sz, cpu_addr, dma_addr);
}
EXPORT_SYMBOL_GPL(basis_device_dma_free_coherent);

dma_addr_t basis_device_dma_map_single(struct device *dev, void *ptr,
                                       size_t size, enum dma_data_direction dir,
                                       u32 *ep_addr)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_controller* controller = device->controller;
	int ret;
	dma_addr_t dma_addr;
	size_t sz = 1ULL << fls64(size - 1);

	if (!controller) {
		dev_err(dev,
		        "No BASIS-controller bind. Can't map to DMA memory.\n");
		return DMA_MAPPING_ERROR;
	}

	dma_addr = dma_map_single(dev, ptr, sz, dir);
	if (dma_addr == DMA_MAPPING_ERROR)
		return DMA_MAPPING_ERROR;

	if (!IS_ALIGNED(dma_addr, sz)) {
		dev_err(dev,
		        "Mapped DMA-buffer is not aligned as required "
		        "(dma_addr: 0x%016llx, required alignment: 0x%08x).\n",
		        (u64)dma_addr, (unsigned)sz);
		goto err_dma_unmap;
	}

	*ep_addr = basis_controller_ep_mem_alloc_addr(controller, sz);

	if (*ep_addr == 0) {
		dev_err(dev,
		        "Failed to allocate EP-memory address (%u bytes).\n",
		        (unsigned)sz);
		goto err_dma_unmap;
	}

	ret = basis_controller_ep_map_addr(controller, *ep_addr, dma_addr, sz);
	if (ret) {
		dev_err(dev, "Failed to map EP-memory.\n");
		goto err_ep_mem_free;
	}

	return dma_addr;

err_ep_mem_free:
	basis_controller_ep_mem_free_addr(controller, *ep_addr, sz);
	*ep_addr = 0;

err_dma_unmap:
	dma_unmap_single(dev, dma_addr, sz, dir);

	return DMA_MAPPING_ERROR;
}
EXPORT_SYMBOL_GPL(basis_device_dma_map_single);

void basis_device_dma_unmap_single(struct device *dev, dma_addr_t dma_addr,
                                   u32 ep_addr, size_t size,
                                   enum dma_data_direction dir)
{
	struct basis_device *device = to_basis_device(dev);
	struct basis_controller* controller = device->controller;
	size_t sz = 1ULL << fls64(size - 1);

	if (!controller)
		return;

	basis_controller_ep_unmap_addr(controller, ep_addr);
	basis_controller_ep_mem_free_addr(controller, ep_addr, sz);
	dma_unmap_single(dev, dma_addr, sz, dir);
}
EXPORT_SYMBOL_GPL(basis_device_dma_unmap_single);

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

	ret = dma_coerce_mask_and_coherent(dev,
	                                   DMA_BIT_MASK(BASIS_DMA_MASK_SIZE));
	if (ret < 0) {
		dev_err(dev, "Failed to set DMA mask: %d\n", ret);
		put_device(dev);
		return ERR_PTR(ret);
	}

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
