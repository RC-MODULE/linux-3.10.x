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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include "ion.h"
#include "ion_priv.h"

#define DRVNAME "ion-physmem"

static struct ion_device *idev;
static uint32_t claimed_heap_ids;

static const struct of_device_id of_match_table[] = {
	{ .compatible = "ion,physmem", },
	{ /* end of list */ }
};

struct physmem_ion_dev {
	struct ion_platform_heap  data;
	struct ion_heap          *heap;
	int                       need_free_coherent;
	void                     *freepage_ptr;
	struct clk               *clk;
	uint32_t                  heap_id;
};

static int ion_physmem_probe(struct platform_device *pdev)
{
	int ret;
	u32 ion_heap_id, ion_heap_align, ion_heap_type;
	ion_phys_addr_t addr;
	size_t size = 0;
	const char *ion_heap_name = NULL;
	struct resource *res;
	struct physmem_ion_dev *ipdev;

	/*
	   Looks like we can only have one ION device in our system.
	   Therefore we call ion_device_create on first probe and only
	   add heaps to it on subsequent probe calls.
	   FixMe:
	   1. Do we need to hold a spinlock here?
	   2. Can several probes race here on SMP?
	*/

	if (!idev) {
		idev = ion_device_create(NULL);
		dev_info(&pdev->dev, "ION PhysMem Driver. (c) RC Module 2015\n");
		if (!idev)
			return -ENOMEM;
	}

	ipdev = kzalloc(sizeof(*ipdev), GFP_KERNEL);
	if (!ipdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ipdev);

	/* Read out name first for the sake of sane error-reporting */
	ret =  of_property_read_string(pdev->dev.of_node, "ion-heap-name",
				       &ion_heap_name);
	if (ret != 0)
		goto errfreeipdev;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-id",
				    &ion_heap_id);
	if (ret != 0)
		goto errfreeipdev;

	/* Check id to be sane first */
	if ((ion_heap_id < 0) || (ion_heap_id >= ION_NUM_HEAP_IDS)) {
		dev_err(&pdev->dev, "bad heap id specified: %d\n",
			ion_heap_id);
		goto errfreeipdev;
	}

	if ((1 << ion_heap_id) & claimed_heap_ids) {
		dev_err(&pdev->dev, "heap id %d is already claimed\n",
			ion_heap_id);
		goto errfreeipdev;
	}

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-type",
				    &ion_heap_type);
	if (ret != 0)
		goto errfreeipdev;

	ret =  of_property_read_u32(pdev->dev.of_node, "ion-heap-align",
				    &ion_heap_align);
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

	/* If it's needed - take care enable clocks */
	ipdev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ipdev->clk))
		ipdev->clk = NULL;
	else
		clk_prepare_enable(ipdev->clk);

	ion_device_add_heap(idev, ipdev->heap);
	claimed_heap_ids |= (1 << ion_heap_id);
	ipdev->heap_id = ion_heap_id;

	dev_dbg(&pdev->dev, "heap %s id %d type %d align 0x%x at phys 0x%lx len %lu KiB\n",
		ion_heap_name, ion_heap_id, ion_heap_type, ion_heap_align,
		(unsigned long int)addr, ((unsigned long int)(size / 1024)));
	return 0;

errfreeipdev:
	kfree(ipdev);
	dev_err(&pdev->dev, "Failed to register heap: %s\n",
		ion_heap_name);
	return -ENOMEM;
}

static int ion_physmem_remove(struct platform_device *pdev)
{
	struct physmem_ion_dev *ipdev = platform_get_drvdata(pdev);

	ion_heap_destroy(ipdev->heap);
	claimed_heap_ids &= ~(1 << ipdev->heap_id);
	if (ipdev->need_free_coherent)
		dma_release_declared_memory(&pdev->dev);
	if (ipdev->freepage_ptr)
		free_pages_exact(ipdev->freepage_ptr, ipdev->data.size);
	kfree(ipdev->heap);
	if (ipdev->clk)
		clk_disable_unprepare(ipdev->clk);
	kfree(ipdev);

	/* We only remove heaps and disable clocks here.
	 * There's no point in nuking the device itself, since:
	 * a). ION driver modules can't be unloaded (yet?)
	 * b). ion_device_destroy() looks like a stub and doesn't
	 * take care to free heaps/clients.
	 * c). I can't think of a scenario where it will be required
	 */

	return 0;
}

static struct platform_driver ion_physmem_driver = {
	.probe		= ion_physmem_probe,
	.remove		= ion_physmem_remove,
	.driver		= {
		.name	= "ion-physmem",
		.of_match_table = of_match_ptr(of_match_table),
	},
};

static int __init ion_physmem_init(void)
{
	return platform_driver_register(&ion_physmem_driver);
}
device_initcall(ion_physmem_init);
