/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Basis ConfigFS header file
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#ifndef __BASIS_CFS_H
#define __BASIS_CFS_H

#include <linux/configfs.h>

struct config_group *basis_cfs_add_controller_group(const char *name);
void basis_cfs_remove_controller_group(struct config_group *group);
struct config_group *basis_cfs_add_device_group(const char *name);
void basis_cfs_remove_device_group(struct config_group *group);

struct basis_device *config_item_to_basis_device(struct config_item *item);

#endif /* __BASIS_CFS_H */
