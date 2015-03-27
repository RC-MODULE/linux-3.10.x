/*
 * Copyright (C) 2015 RC Module 
 * Andrew Andrianov <andrew@ncrmnt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Generic devicetree physmem driver for ION Memory Manager 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include "ion.h"
#include "ion_priv.h"

#define DRVNAME "ion-physmem"

static struct ion_device *idev;

static const struct of_device_id of_match_table[] = {
	{ .compatible = "ion,physmem", },
	{ /* end of list */ }
};

struct physmem_ion_dev {
	struct ion_platform_heap *data;
	struct ion_heap          *heap;
};

MODULE_DEVICE_TABLE(of, of_match_table);
static int ion_physmem_probe(struct platform_device *pdev)
{
	int ret;
	u32 ion_heap_id, ion_heap_align, ion_heap_type;
	ion_phys_addr_t addr;
	u64 size;
	const char *ion_heap_name;
	struct resource *res;
	struct physmem_ion_dev *ipdev;

	/* 
	   Looks like we can only have one ION device in our system. 
	   Therefore we call ion_device_create on first probe and only 
	   add heaps to it on subsequent probe calls.
	   FixMe: Do we need to hold a spinlock here once device probing
	   becomes async?
	*/
	
	if (!idev) { 
		idev = ion_device_create(NULL);
		printk("ion-physmem: ION PhysMem Driver. (c) RC Module 2015\n");
		if (!idev) 
			return -ENOMEM;
	}

	ipdev = kzalloc(sizeof(struct physmem_ion_dev), GFP_KERNEL);
	if (!ipdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ipdev);

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-id",
				    &ion_heap_id);
	if (ret != 0)
		return ret;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-type",
				    &ion_heap_id);
	if (ret != 0)
		return ret;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-align",
				    &ion_heap_align);
	if (ret != 0)
		return ret;

	ret =  of_property_read_string(pdev->dev.of_node, "ion-heap-name",
				       &ion_heap_name);
	if (ret != 0)
		return ret;

 	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "memory");
	if (!res) 
		return -ENODEV;
	
	size = (res->end - res->start + 1);
	addr = res->start;
	
	ipdev->data = kzalloc(sizeof(struct ion_platform_heap), GFP_KERNEL);
	if (!ipdev->data)
		goto errfreeipdev;
	
	ipdev->data->id    = ion_heap_id; 
	ipdev->data->type  = ion_heap_type;
	ipdev->data->name  = ion_heap_name;
	ipdev->data->align = ion_heap_align;
	ipdev->data->base  = addr;
	ipdev->data->size  = size;
	
	ipdev->heap = ion_heap_create(ipdev->data);
	if (!ipdev->heap) 
		goto errfreeheapdata;

	if (!try_module_get(THIS_MODULE))
		goto errfreeheap;
	
	ion_device_add_heap(idev, ipdev->heap);	

	printk("ion-physmem: heap %s id %d type %d align 0x%x at 0x%lx len %lu KiB\n", 
	       ion_heap_name, ion_heap_id, ion_heap_type, ion_heap_align, 
	       (long unsigned int) addr, ((long unsigned int) size / 1024));

	return 0;

errfreeheap:
	kfree(ipdev->heap);
errfreeheapdata:
	kfree(ipdev->data);
errfreeipdev:
	kfree(ipdev);
	return -ENOMEM;
}

static int ion_physmem_remove(struct platform_device *pdev)
{
	struct physmem_ion_dev *ipdev = platform_get_drvdata(pdev);
	ion_heap_destroy(ipdev->heap);
	kfree(ipdev->heap);
	kfree(ipdev->data);
	kfree(ipdev);
	module_put(THIS_MODULE);
	return 0;
}

static void __exit ion_physmem_exit(void)
{
	if (idev)
		ion_device_destroy(idev);	
}
__exitcall(ion_physmem_exit);

static struct platform_driver ion_physmem_driver = {
	.probe		= ion_physmem_probe,
	.remove		= ion_physmem_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ion-physmem",
		.of_match_table = of_match_ptr(of_match_table),
	},
};
module_platform_driver(ion_physmem_driver);

MODULE_DESCRIPTION("Generic physmem driver for ION");
MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_LICENSE("GPL");
