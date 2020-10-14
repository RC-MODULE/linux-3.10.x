/* SPDX-License-Identifier: GPL-2.0 */
/**
 * BASIS Controller header file
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#ifndef __BASIS_CONTROLLER_H
#define __BASIS_CONTROLLER_H

#include <linux/irqdomain.h>

#include "basis-device.h"

struct basis_controller;

struct basis_controller_ops {
	int     (*start)(struct basis_controller *controller);
	void    (*stop)(struct basis_controller *controller);
	struct module *owner;
};

struct basis_ep_mem {
	u32             phys_base;
	u32             size;
	unsigned long  *bitmap;
	u32             page_size;
	int             pages;
};

struct basis_controller {
	struct device                           dev;
	struct list_head                        devices;
	const struct basis_controller_ops      *ops;
	struct config_group                    *group;
	spinlock_t                              lock;
	struct irq_domain                      *domain;
	struct basis_ep_mem                    *ep_mem;

	void __iomem                           *ep_base;
	phys_addr_t                             ep_base_phys;
	u32                                    *ep_ob_addr;
	unsigned                                ep_cnt_regions;
};

#define to_basis_controller(device) container_of((device), struct basis_controller, dev)

#define basis_controller_create(dev, ops)    \
	__basis_controller_create((dev), (ops), THIS_MODULE)
#define devm_basis_controller_create(dev, ops)    \
		__devm_basis_controller_create((dev), (ops), THIS_MODULE)

static inline void 
basis_controller_set_drvdata(struct basis_controller *controller, void *data)
{
	dev_set_drvdata(&controller->dev, data);
}

static inline void *
basis_controller_get_drvdata(struct basis_controller *controller)
{
	return dev_get_drvdata(&controller->dev);
}

struct basis_controller *
__devm_basis_controller_create(struct device *dev,
                               const struct basis_controller_ops *ops,
                               struct module *owner);
struct basis_controller *
__basis_controller_create(struct device *dev,
                          const struct basis_controller_ops *ops,
                          struct module *owner);
void devm_basis_controller_destroy(struct device *dev,
                                   struct basis_controller *controller);
void basis_controller_destroy(struct basis_controller *controller);
int basis_controller_add_device(struct basis_controller *controller, 
                                struct basis_device *device);
void basis_controller_remove_device(struct basis_controller *controller,
                                    struct basis_device *device);
int basis_controller_start(struct basis_controller *controller);
void basis_controller_stop(struct basis_controller *controller);
struct basis_controller *basis_controller_get(const char *name);
void basis_controller_put(struct basis_controller *controller);

u32 basis_controller_ep_mem_alloc_addr(struct basis_controller *controller,
                                       u32 size);
void basis_controller_ep_mem_free_addr(struct basis_controller *controller,
                                       u32 phys_addr, u32 size);
int basis_controller_ep_mem_init(struct basis_controller *controller,
                                 u32 phys_base, u32 size, u32 page_size);
void basis_controller_ep_mem_exit(struct basis_controller *controller);

int basis_controller_ep_regions_init(struct basis_controller *controller,
                                     void __iomem *base, phys_addr_t base_phys, 
                                     unsigned cnt_regions);
int basis_controller_ep_map_addr(struct basis_controller *controller, 
                                 u32 addr, u64 pci_addr, u32 size);
void basis_controller_ep_unmap_addr(struct basis_controller *controller,
                                    u32 addr);

#endif /* __BASIS_CONTROLLER_H */
