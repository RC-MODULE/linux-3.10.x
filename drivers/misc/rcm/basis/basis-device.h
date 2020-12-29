/* SPDX-License-Identifier: GPL-2.0 */
/**
 * BASIS Device header file
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#ifndef __BASIS_DEVICE_H
#define __BASIS_DEVICE_H

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>

#ifndef DMA_MAPPING_ERROR
#	define DMA_MAPPING_ERROR (~(dma_addr_t)0)
#endif

struct basis_device;

struct basis_device_ops {
	int     (*bind)(struct basis_device *device);
	void    (*unbind)(struct basis_device *device);
};

#define BASIS_DEVICE_NAME_SIZE 20

struct basis_device_id {
	char name[BASIS_DEVICE_NAME_SIZE];
	kernel_ulong_t driver_data;
};

struct basis_device_driver {
	int     (*probe)(struct basis_device *device);
	int     (*remove)(struct basis_device *device);

	struct device_driver            driver;
	struct basis_device_ops        *ops;
	struct module                  *owner;
	struct list_head                device_group;
	const struct basis_device_id   *id_table;

	struct configfs_attribute     **attrs;
};

#define to_basis_device_driver(drv) (container_of((drv), \
                                     struct basis_device_driver, driver))

struct basis_device {
	struct device                   dev;
	const char                     *name;
	u8                              dev_no;

	struct basis_controller        *controller;
	struct basis_device_driver     *driver;
	struct list_head                list;

	void                           *priv;
};

#define to_basis_device(basis_dev) container_of((basis_dev), \
                                                struct basis_device, dev)

#define basis_device_register_driver(driver)    \
	__basis_device_register_driver((driver), THIS_MODULE)

struct basis_device_driver *basis_driver_find(const char* group_name);

static inline void basis_device_set_drvdata(struct basis_device *device,
                                            void *data)
{
	dev_set_drvdata(&device->dev, data);
}

static inline void *basis_device_get_drvdata(struct basis_device *device)
{
	return dev_get_drvdata(&device->dev);
}

const struct basis_device_id *
basis_device_match_device(const struct basis_device_id *id,
                          struct basis_device *device);
struct basis_device *basis_device_create(const char *name);
void basis_device_destroy(struct basis_device *device);
int __basis_device_register_driver(struct basis_device_driver *driver,
                                   struct module *owner);
void basis_device_unregister_driver(struct basis_device_driver *driver);
int basis_device_bind(struct basis_device *device);
void basis_device_unbind(struct basis_device *device);

struct basis_device *basis_device_find(const char *name);

void* basis_device_dma_alloc_coherent(struct device *dev,
                                      size_t size, dma_addr_t *dma_addr,
                                      u32 *ep_addr, gfp_t gfp);
void basis_device_dma_free_coherent(struct device *dev,
                                    size_t size, void* cpu_addr,
                                    dma_addr_t dma_addr, u32 ep_addr);
dma_addr_t basis_device_dma_map_single(struct device *dev, void *ptr,
                                       size_t size, enum dma_data_direction dir,
                                       u32 *ep_addr);
void basis_device_dma_unmap_single(struct device *dev, dma_addr_t dma_addr,
                                   u32 ep_addr, size_t size,
                                   enum dma_data_direction dir);


#define BASIS_DEV_ATTR_U32_SHOW(_pfx, _name, _data_type)		       \
static ssize_t _pfx##_name##_show(struct config_item *item, char *page)	       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	return sprintf(page, "0x%04x\n", data->_name);			       \
}

#define BASIS_DEV_ATTR_U32_STORE(_pfx, _name, _data_type)		       \
static ssize_t _pfx##_name##_store(struct config_item *item, const char *page, \
                                   size_t len)				       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	int ret;							       \
	u32 val;							       \
	ret = kstrtou32(page, 0, &val);					       \
	if (ret)							       \
		return ret;						       \
	data->_name = val;						       \
	return len;							       \
}

#define BASIS_DEV_ATTR_ARR_U32_SHOW(_pfx, _name, _idx, _data_type)	       \
static ssize_t _pfx##_name##_##_idx##_show(struct config_item *item,	       \
                                           char *page)			       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	return sprintf(page, "0x%04x\n", data->_name[_idx]);		       \
}

#define BASIS_DEV_ATTR_ARR_U32_STORE(_pfx, _name, _idx, _data_type)	       \
static ssize_t _pfx##_name##_##_idx##_store(struct config_item *item,	       \
                                         const char *page, size_t len)	       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	int ret;							       \
	u32 val;							       \
	ret = kstrtou32(page, 0, &val);					       \
	if (ret)							       \
		return ret;						       \
	data->_name[_idx] = val;					       \
	return len;							       \
}

#define BASIS_DEV_ATTR_STR_SHOW(_pfx, _name, _data_type)		       \
static ssize_t _pfx##_name##_show(struct config_item *item, char *page)	       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	return sprintf(page, "%s\n", data->_name);			       \
}

#define BASIS_DEV_ATTR_STR_STORE(_pfx, _name, _data_type)		       \
static ssize_t _pfx##_name##_store(struct config_item *item, const char *page, \
                                   size_t len)				       \
{									       \
	struct basis_device *device = config_item_to_basis_device(item);       \
	_data_type *data = basis_device_get_drvdata(device);		       \
	size_t size = min(len, sizeof(data->_name) - 1);		       \
	strncpy(data->_name, page, size);				       \
	while ((size > 0) && (data->_name[size - 1] == '\n')) {		       \
		data->_name[size - 1] = 0;				       \
		--size;							       \
	}								       \
	return len;							       \
}

#define module_basis_driver(__driver) \
	module_driver(__driver, basis_device_register_driver, \
	              basis_device_unregister_driver)

extern struct regmap_bus basis_regmap_bus;

#endif /* __BASIS_DEVICE_H */
