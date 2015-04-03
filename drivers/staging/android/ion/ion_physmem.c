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
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include "ion.h"
#include "ion_priv.h"

#define DRVNAME "ion-physmem"

static struct ion_device *idev;

static const struct of_device_id of_match_table[] = {
	{ .compatible = "ion,physmem", },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, of_match_table);

/*
#define PHYSMAP_ION_DEBUG y
*/

struct physmem_ion_dev {
	struct ion_platform_heap  data;
	struct ion_heap          *heap;
	int                       need_free_coherent;
	void                     *freepage_ptr;
};

static long ion_physmem_custom_ioctl(struct ion_client *client,
				     unsigned int cmd,
				     unsigned long arg)
{
/* Just an ugly debugging hack. */
#ifdef PHYSMAP_ION_DEBUG
	if ((cmd == 0xff)) {
		ion_phys_addr_t addr;
		size_t len;
		struct ion_handle *hndl;

		printk(KERN_INFO "=== ION PHYSMEM DEBUG PRINTOUT ===\n");
		printk(KERN_INFO "N.B. DO disable CONFIG_ION_PHYSMEM_DEBUG in production\n");
		printk(KERN_INFO "Shared fd is %d\n", (int) arg);
		hndl = ion_import_dma_buf(client, arg);
		if (!hndl) {
			printk(KERN_INFO "Failed to import shared descriptor.\n");
			return 0;
		}
		ion_phys(client, hndl, &addr, &len);
		printk(KERN_INFO "shared buffer: phys 0x%lx len %ul\n",
		       addr, len);
		printk(KERN_INFO "=== ION PHYSMEM DEBUG PRINTOUT ===\n");
	}
#endif
	return 0;
}

static int ion_physmem_probe(struct platform_device *pdev)
{
	int ret;
	u32 ion_heap_id, ion_heap_align, ion_heap_type;
	ion_phys_addr_t addr;
	size_t size = 0;
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
		idev = ion_device_create(ion_physmem_custom_ioctl);
		dev_info(&pdev->dev, "ion-physmem: ION PhysMem Driver. (c) RC Module 2015\n");
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
		goto errfreeipdev;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-type",
				    &ion_heap_type);
	if (ret != 0)
		goto errfreeipdev;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-align",
				    &ion_heap_align);
	if (ret != 0)
		goto errfreeipdev;

	ret =  of_property_read_string(pdev->dev.of_node, "ion-heap-name",
				       &ion_heap_name);
	if (ret != 0)
		goto errfreeipdev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "memory");
	/* Not always needed, throw no error if missing */
	if (res) {
		/* Fill in some defaults */
		addr = res->start;
		size = resource_size(res);
	}

	switch (ion_heap_type) {
	case ION_HEAP_TYPE_DMA:
		if (res) {
			ret = dma_declare_coherent_memory(&pdev->dev,
							  res->start,
							  res->start,
							  resource_size(res),
							  DMA_MEMORY_MAP |
							  DMA_MEMORY_EXCLUSIVE);
			if (ret == 0) {
				ret = -ENODEV;
				goto errfreeipdev;
			}
		}
		/*
		 *  If no memory region declared in dt we assume that
		 *  the user is okay with plain dma_alloc_coherent.
		 */
		break;
	case ION_HEAP_TYPE_CARVEOUT:
	case ION_HEAP_TYPE_CHUNK:
	{
		if (size == 0) {
			ret = -EIO;
			goto errfreeipdev;
		}
		ipdev->freepage_ptr = alloc_pages_exact(size, GFP_KERNEL);
		if (ipdev->freepage_ptr) {
			addr = virt_to_phys(ipdev->freepage_ptr);
		} else {
			ret = -ENOMEM;
			goto errfreeipdev;
		}
		break;
	}
	}

	ipdev->data.id    = ion_heap_id;
	ipdev->data.type  = ion_heap_type;
	ipdev->data.name  = ion_heap_name;
	ipdev->data.align = ion_heap_align;
	ipdev->data.base  = addr;
	ipdev->data.size  = size;
	/* This one make dma_declare_coherent_memory actually work */
	ipdev->data.priv  = &pdev->dev;


	ipdev->heap = ion_heap_create(&ipdev->data);
	if (!ipdev->heap)
		goto errfreeipdev;

	if (!try_module_get(THIS_MODULE))
		goto errfreeheap;

	ion_device_add_heap(idev, ipdev->heap);

	dev_info(&pdev->dev, "ion-physmem: heap %s id %d type %d align 0x%x at phys 0x%lx len %lu KiB\n",
	       ion_heap_name, ion_heap_id, ion_heap_type, ion_heap_align,
	       (unsigned long int) addr, ((unsigned long int) size / 1024));

	return 0;

errfreeheap:
	kfree(ipdev->heap);
errfreeipdev:
	kfree(ipdev);
	return -ENOMEM;
}

static int ion_physmem_remove(struct platform_device *pdev)
{
	struct physmem_ion_dev *ipdev = platform_get_drvdata(pdev);

	ion_heap_destroy(ipdev->heap);
	if (ipdev->need_free_coherent)
		dma_release_declared_memory(&pdev->dev);
	if (ipdev->freepage_ptr)
		free_pages_exact(ipdev->freepage_ptr, ipdev->data.size);
	kfree(ipdev->heap);
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
