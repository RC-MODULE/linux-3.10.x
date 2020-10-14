// SPDX-License-Identifier: GPL-2.0
/**
 * Basis library
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */
#include <linux/module.h>

#include "basis-controller.h"
#include "basis-device.h"
#include "basis-cfs.h"

int basis_controller_init(void);
void basis_controller_exit(void);
int basis_device_init(void);
void basis_device_exit(void);
int basis_cfs_init(void);
void basis_cfs_exit(void);

static int __init basis_core_init(void)
{
	int ret = basis_cfs_init();
	if (ret)
		return ret;

	ret = basis_controller_init();
	if (ret)
		goto err_controller;

	ret = basis_device_init();
	if (ret)
		goto err_device;

	return 0;
err_device:
	basis_controller_exit();
err_controller:
	basis_cfs_exit();

	return ret;
}
module_init(basis_core_init);

static void __exit basis_core_exit(void)
{
	basis_device_exit();
	basis_controller_exit();
	basis_cfs_exit();
}
module_exit(basis_core_exit);

MODULE_DESCRIPTION("BASIS Library");
MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_LICENSE("GPL v2");
