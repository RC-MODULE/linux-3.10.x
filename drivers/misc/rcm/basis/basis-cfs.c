// SPDX-License-Identifier: GPL-2.0
/**
 * configfs to configure the PCI endpoint
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#include <linux/module.h>
#include <linux/idr.h>
#include <linux/slab.h>

#include "basis-controller.h"
#include "basis-device.h"
#include "basis-cfs.h"

static DEFINE_IDR(devices_idr);
static DEFINE_MUTEX(devices_mutex);
static struct config_group *devices_group;
static struct config_group *controllers_group;

struct basis_device_group {
	struct config_group group;
	struct basis_device *device;
	int index;
};

struct basis_controller_group {
	struct config_group group;
	struct basis_controller *controller;
	bool start;
	unsigned long device_num_map;
};

static inline struct basis_device_group *
to_basis_device_group(struct config_item *item)
{
	return container_of(to_config_group(item), struct basis_device_group,
	                    group);
}

static inline struct basis_controller_group *
to_basis_controller_group(struct config_item *item)
{
	return container_of(to_config_group(item),
	                    struct basis_controller_group, group);
}

struct basis_device *config_item_to_basis_device(struct config_item *item)
{
	struct basis_device_group *device_group = to_basis_device_group(item);

	return device_group->device;
}
EXPORT_SYMBOL(config_item_to_basis_device);

static ssize_t basis_controller_start_store(struct config_item *item,
                                            const char *page, size_t len)
{
	int ret;
	bool start;
	struct basis_controller *controller;
	struct basis_controller_group *controller_group = 
		to_basis_controller_group(item);

	controller = controller_group->controller;

	ret = kstrtobool(page, &start);
	if (ret)
		return ret;

	if (!start) {
		basis_controller_stop(controller);
		return len;
	}

	ret = basis_controller_start(controller);
	if (ret) {
		dev_err(&controller->dev, "failed to start BASIS controller\n");
		return -EINVAL;
	}

	controller_group->start = start;

	return len;
}

static ssize_t basis_controller_start_show(struct config_item *item, char *page)
{
	return sprintf(page, "%d\n",
	               to_basis_controller_group(item)->start);
}

CONFIGFS_ATTR(basis_controller_, start);

static struct configfs_attribute *basis_controller_attrs[] = {
	&basis_controller_attr_start,
	NULL,
};

static int basis_controller_device_link(struct config_item *controller_item,
                                        struct config_item *device_item)
{
	int ret;
	u32 dev_no = 0;
	struct basis_device_group *device_group = 
		to_basis_device_group(device_item);
	struct basis_controller_group *controller_group = 
		to_basis_controller_group(controller_item);
	struct basis_controller *controller = controller_group->controller;
	struct basis_device *device = device_group->device;

	dev_no = find_first_zero_bit(&controller_group->device_num_map,
	                             BITS_PER_LONG);
	if (dev_no >= BITS_PER_LONG)
		return -EINVAL;

	set_bit(dev_no, &controller_group->device_num_map);
	device->dev_no = dev_no;

	ret = basis_controller_add_device(controller, device);
	if (ret)
		goto err_add_device;

	ret = basis_device_bind(device);
	if (ret)
		goto err_device_bind;

	return 0;

err_device_bind:
	basis_controller_remove_device(controller, device);

err_add_device:
	clear_bit(dev_no, &controller_group->device_num_map);

	return ret;
}

static void basis_controller_device_unlink(struct config_item *controller_item,
                                           struct config_item *device_item)
{
	struct basis_controller *controller;
	struct basis_device *device;
	struct basis_device_group *device_group =
		to_basis_device_group(device_item);
	struct basis_controller_group *controller_group =
		to_basis_controller_group(controller_item);

	WARN_ON_ONCE(controller_group->start);

	controller = controller_group->controller;
	device = device_group->device;
	clear_bit(device->dev_no, &controller_group->device_num_map);
	basis_device_unbind(device);
	basis_controller_remove_device(controller, device);
}

static struct configfs_item_operations basis_controller_item_ops = {
	.allow_link     = basis_controller_device_link,
	.drop_link      = basis_controller_device_unlink,
};

static const struct config_item_type basis_controller_type = {
	.ct_item_ops    = &basis_controller_item_ops,
	.ct_attrs       = basis_controller_attrs,
	.ct_owner       = THIS_MODULE,
};

struct config_group *basis_cfs_add_controller_group(const char *name)
{
	int ret;
	struct basis_controller *controller;
	struct config_group *group;
	struct basis_controller_group *controller_group;

	controller_group = kzalloc(sizeof(*controller_group), GFP_KERNEL);
	if (!controller_group) {
		ret = -ENOMEM;
		goto err;
	}

	group = &controller_group->group;

	config_group_init_type_name(group, name, &basis_controller_type);
	ret = configfs_register_group(controllers_group, group);
	if (ret) {
		pr_err("failed to register configfs group for %s\n", name);
		goto err_register_group;
	}

	controller = basis_controller_get(name);
	if (IS_ERR(controller)) {
		pr_err("failed to get controller for %s\n", name);
		ret = PTR_ERR(controller);
		goto err_controller_get;
	}

	controller_group->controller = controller;

	return group;

err_controller_get:
	configfs_unregister_group(group);

err_register_group:
	kfree(controller_group);

err:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(basis_cfs_add_controller_group);

void basis_cfs_remove_controller_group(struct config_group *group)
{
	struct basis_controller_group *controller_group;

	if (!group)
		return;

	controller_group = container_of(group, struct basis_controller_group,
	                                group);
	basis_controller_put(controller_group->controller);
	configfs_unregister_group(&controller_group->group);
	kfree(controller_group);
}
EXPORT_SYMBOL(basis_cfs_remove_controller_group);

static void basis_device_release(struct config_item *item)
{
	struct basis_device_group *device_group = to_basis_device_group(item);

	pr_info("%s >>>\n", __func__);

	mutex_lock(&devices_mutex);
	idr_remove(&devices_idr, device_group->index);
	mutex_unlock(&devices_mutex);
	basis_device_destroy(device_group->device);
	kfree(item->ci_type);
	kfree(device_group);
}

static struct configfs_item_operations basis_device_ops = {
	.release = basis_device_release,
};
/*
static const struct config_item_type basis_device_type = {
	.ct_item_ops    = &basis_device_ops,
	.ct_owner       = THIS_MODULE,
};
*/
static struct config_group *basis_device_make(struct config_group *group,
                                              const char *name)
{
	struct basis_device_group *device_group;
	struct config_item_type *item_type;
	struct basis_device *device;
	char *dev_name;
	int index, err;
	struct basis_device_driver *driver;

	driver = basis_driver_find(group->cg_item.ci_name);
	if (!driver) {
		pr_warn("Failed to find BASIS driver for group \"%s\"\n",
		         group->cg_item.ci_name);
	}

	item_type = kzalloc(sizeof(*item_type), GFP_KERNEL);
	if (!item_type)
		return ERR_PTR(-ENOMEM);

	item_type->ct_item_ops = &basis_device_ops;
	item_type->ct_owner    = THIS_MODULE;

	if (driver) {
		item_type->ct_attrs = driver->attrs;
	}

	device_group = kzalloc(sizeof(*device_group), GFP_KERNEL);
	if (!device_group)
	{
		err = -ENOMEM;
		goto free_type;
	}

	mutex_lock(&devices_mutex);
	index = idr_alloc(&devices_idr, device_group, 0, 0, GFP_KERNEL);
	mutex_unlock(&devices_mutex);
	if (index < 0) {
		err = index;
		goto free_group;
	}

	device_group->index = index;

	config_group_init_type_name(&device_group->group, name, item_type);

	dev_name = kasprintf(GFP_KERNEL, "%s.%d",
	                     group->cg_item.ci_name, device_group->index);
	if (!dev_name) {
		err = -ENOMEM;
		goto remove_idr;
	}

	device = basis_device_create(dev_name);
	if (IS_ERR(device)) {
		pr_err("failed to create BASIS device\n");
		err = -EINVAL;
		goto free_name;
	}

	device_group->device = device;

	kfree(dev_name);

	return &device_group->group;

free_name:
	kfree(dev_name);

remove_idr:
	mutex_lock(&devices_mutex);
	idr_remove(&devices_idr, device_group->index);
	mutex_unlock(&devices_mutex);

free_group:
	kfree(device_group);

free_type:
	kfree(item_type);

	return ERR_PTR(err);
}

static void basis_device_drop(struct config_group *group,
                              struct config_item *item)
{
	config_item_put(item);
}

static struct configfs_group_operations basis_device_group_ops = {
	.make_group     = &basis_device_make,
	.drop_item      = &basis_device_drop,
};

static const struct config_item_type basis_device_group_type = {
	.ct_group_ops   = &basis_device_group_ops,
	.ct_owner       = THIS_MODULE,
};

struct config_group *basis_cfs_add_device_group(const char *name)
{
	struct config_group *group;

	group = configfs_register_default_group(devices_group, name,
	                                        &basis_device_group_type);
	if (IS_ERR(group))
		pr_err("failed to register configfs group for %s device\n",
		       name);

	return group;
}
EXPORT_SYMBOL(basis_cfs_add_device_group);

void basis_cfs_remove_device_group(struct config_group *group)
{
	if (IS_ERR_OR_NULL(group))
		return;

	configfs_unregister_default_group(group);
}
EXPORT_SYMBOL(basis_cfs_remove_device_group);

static const struct config_item_type basis_devices_type = {
	.ct_owner = THIS_MODULE,
};

static const struct config_item_type basis_controllers_type = {
	.ct_owner = THIS_MODULE,
};

static const struct config_item_type basis_type = {
	.ct_owner = THIS_MODULE,
};

static struct configfs_subsystem basis_cfs_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "basis",
			.ci_type = &basis_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(basis_cfs_subsys.su_mutex),
};

int basis_cfs_init(void)
{
	int ret;
	struct config_group *root = &basis_cfs_subsys.su_group;

	config_group_init(root);

	ret = configfs_register_subsystem(&basis_cfs_subsys);
	if (ret) {
		pr_err("Error %d while registering subsystem %s\n",
		       ret, root->cg_item.ci_namebuf);
		goto err;
	}

	devices_group = configfs_register_default_group(root, "devices",
	                                                &basis_devices_type);
	if (IS_ERR(devices_group)) {
		ret = PTR_ERR(devices_group);
		pr_err("Error %d while registering devices group\n", ret);
		goto err_devices_group;
	}

	controllers_group =
		configfs_register_default_group(root, "controllers",
		                                &basis_controllers_type);
	if (IS_ERR(controllers_group)) {
		ret = PTR_ERR(controllers_group);
		pr_err("Error %d while registering controllers group\n", ret);
		goto err_controllers_group;
	}

	return 0;

err_controllers_group:
	configfs_unregister_default_group(devices_group);

err_devices_group:
	configfs_unregister_subsystem(&basis_cfs_subsys);

err:
	return ret;
}

void basis_cfs_exit(void)
{
	configfs_unregister_default_group(controllers_group);
	configfs_unregister_default_group(devices_group);
	configfs_unregister_subsystem(&basis_cfs_subsys);
}
