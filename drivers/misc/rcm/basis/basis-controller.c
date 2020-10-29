// SPDX-License-Identifier: GPL-2.0
/**
 * BASIS Controller library
 *
*  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
  */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "basis-controller.h"
#include "basis-device.h"
#include "basis-cfs.h"

#include "basis_addresses.h"
#include "basis_regs_pcie.h"

static struct class *basis_controller_class;

void basis_controller_ep_set_outbound_region(void __iomem *base, u32 r,
                                             u32 cpu_addr, u64 pci_addr,
                                             size_t size)
{
	u64 sz = 1ULL << fls64(size - 1);
	int nbits = ilog2(sz);
	u32 addr0, addr1, desc0, desc1;
	void __iomem *base_addr = base + PCIE_CORE_BASE + PCIe_Core_AXIConfig +
	                          PCIe_AXI_outregion_0_addr_translation_0;

	if (nbits < 8)
		nbits = 8;

	/* Set the PCI address */
	addr0 = (nbits - 1) | (lower_32_bits(pci_addr) & GENMASK(31, 8));
	addr1 = upper_32_bits(pci_addr);

	writel(addr0, base_addr + r * 0x20);
	writel(addr1, base_addr + r * 0x20 + 0x4);

	desc0 = 2;
	desc1 = 0;

	writel(desc0, base_addr + r * 0x20 + 0x8);
	writel(desc1, base_addr + r * 0x20 + 0xC);

	addr0 = (nbits - 1) | (cpu_addr & GENMASK(31, 8));
	addr1 = 0;

	writel(addr0, base_addr + r * 0x20 + 0x18);
	writel(addr1, base_addr + r * 0x20 + 0x1C);
}

void basis_controller_ep_reset_outbound_region(void __iomem *base, u32 r)
{
	void __iomem *base_addr = base + PCIE_CORE_BASE + PCIe_Core_AXIConfig +
	                          PCIe_AXI_outregion_0_addr_translation_0;

	writel(0, base_addr + r * 0x20 + 0x8);
	writel(0, base_addr + r * 0x20 + 0xC);

	udelay(1000);

	writel(0, base_addr + r * 0x20 + 0x18);
	writel(0, base_addr + r * 0x20 + 0x1C);

	writel(0, base_addr + r * 0x20);
	writel(0, base_addr + r * 0x20 + 0x4);
}

int basis_controller_ep_map_addr(struct basis_controller *controller, 
                                 u32 addr, u64 pci_addr, u32 size)
{
	u32 r = 0;

	while ((r < controller->ep_cnt_regions) && (controller->ep_ob_addr[r]))
		++r;

	if (r >= controller->ep_cnt_regions) {
		dev_err(&controller->dev, "No free outbound region\n");
		return -EINVAL;
	}

	basis_controller_ep_set_outbound_region(controller->ep_base, r,
	                                        addr, pci_addr, size);

	controller->ep_ob_addr[r] = addr;

	dev_dbg(&controller->dev,
	        "EP address mapped 0x%08X -> 0x%016llx (0x%X bytes) "
	        "(region #%u)\n",
	        addr, pci_addr, size, r);

	return 0;
}
EXPORT_SYMBOL_GPL(basis_controller_ep_map_addr);

void basis_controller_ep_unmap_addr(struct basis_controller *controller,
                                    u32 addr)
{
	u32 r = 0;

	while ((r < controller->ep_cnt_regions) && 
	       (controller->ep_ob_addr[r] != addr))
		++r;

	if (r >= controller->ep_cnt_regions)
		return;

	basis_controller_ep_reset_outbound_region(controller->ep_base, r);

	controller->ep_ob_addr[r] = 0;

	dev_dbg(&controller->dev, "EP address unmapped 0x%08X (region #%u)\n",
	        addr, r);
}
EXPORT_SYMBOL_GPL(basis_controller_ep_unmap_addr);

int basis_controller_ep_regions_init(struct basis_controller *controller,
                                     void __iomem *base, phys_addr_t base_phys, 
                                     unsigned cnt_regions)
{
	u32 r;

	controller->ep_ob_addr = devm_kcalloc(&controller->dev,
	                                      cnt_regions,
	                                      sizeof(*controller->ep_ob_addr),
	                                      GFP_KERNEL);
	if (!controller->ep_ob_addr)
		return -ENOMEM;

	for (r = 0; r < cnt_regions; ++r)
		basis_controller_ep_reset_outbound_region(base, r);

	controller->ep_base = base;
	controller->ep_base_phys = base_phys;
	controller->ep_cnt_regions = cnt_regions;

	return 0;
}
EXPORT_SYMBOL_GPL(basis_controller_ep_regions_init);

static int basis_controller_ep_mem_get_order(struct basis_ep_mem *mem,
                                             size_t size)
{
	int order;
	unsigned int page_shift = ilog2(mem->page_size);

	size--;
	size >>= page_shift;
#if BITS_PER_LONG == 32
	order = fls(size);
#else
	order = fls64(size);
#endif
	return order;
}

u32 basis_controller_ep_mem_alloc_addr(struct basis_controller *controller,
                                       u32 size)
{
	int pageno;
	u32 phys_addr;
	struct basis_ep_mem *mem = controller->ep_mem;
	unsigned int page_shift = ilog2(mem->page_size);
	int order;

	size = ALIGN(size, mem->page_size);
	order = basis_controller_ep_mem_get_order(mem, size);

	pageno = bitmap_find_free_region(mem->bitmap, mem->pages, order);
	if (pageno < 0)
		return 0;

	phys_addr = mem->phys_base + ((phys_addr_t)pageno << page_shift);

	return phys_addr;
}
EXPORT_SYMBOL_GPL(basis_controller_ep_mem_alloc_addr);

void basis_controller_ep_mem_free_addr(struct basis_controller *controller,
                                       u32 phys_addr, u32 size)
{
	int pageno;
	struct basis_ep_mem *mem = controller->ep_mem;
	unsigned int page_shift = ilog2(mem->page_size);
	int order;

	pageno = (phys_addr - mem->phys_base) >> page_shift;
	size = ALIGN(size, mem->page_size);
	order = basis_controller_ep_mem_get_order(mem, size);
	bitmap_release_region(mem->bitmap, pageno, order);
}
EXPORT_SYMBOL_GPL(basis_controller_ep_mem_free_addr);

int basis_controller_ep_mem_init(struct basis_controller *controller,
                                 u32 phys_base, u32 size, u32 page_size)
{
	int ret;
	struct basis_ep_mem *mem;
	unsigned long *bitmap;
	unsigned int page_shift;
	int pages;
	int bitmap_size;

	if (page_size < PAGE_SIZE)
		page_size = PAGE_SIZE;

	page_shift = ilog2(page_size);
	pages = size >> page_shift;
	bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		ret = -ENOMEM;
		goto err;
	}

	bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!bitmap) {
		ret = -ENOMEM;
		goto err_mem;
	}

	mem->bitmap = bitmap;
	mem->phys_base = phys_base;
	mem->page_size = page_size;
	mem->pages = pages;
	mem->size = size;

	controller->ep_mem = mem;

	return 0;

err_mem:
	kfree(mem);

err:
	return ret;
}
EXPORT_SYMBOL_GPL(basis_controller_ep_mem_init);

void basis_controller_ep_mem_exit(struct basis_controller *controller)
{
	struct basis_ep_mem *mem = controller->ep_mem;

	controller->ep_mem = NULL;
	kfree(mem->bitmap);
	kfree(mem);
}
EXPORT_SYMBOL_GPL(basis_controller_ep_mem_exit);

static void devm_basis_controller_release(struct device *dev, void *res)
{
	struct basis_controller *controller = *(struct basis_controller **)res;

	basis_controller_destroy(controller);
}

static int devm_basis_controller_match(struct device *dev, void *res,
                                       void *match_data)
{
	struct basis_controller **controller = res;

	return *controller == match_data;
}

void basis_controller_put(struct basis_controller *controller)
{
	if (!controller || IS_ERR(controller))
		return;

	module_put(controller->ops->owner);
	put_device(&controller->dev);
}
EXPORT_SYMBOL_GPL(basis_controller_put);

struct basis_controller *basis_controller_get(const char *name)
{
	int ret = -EINVAL;
	struct basis_controller *controller;
	struct device *dev;
	struct class_dev_iter iter;

	class_dev_iter_init(&iter, basis_controller_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		if (strcmp(name, dev_name(dev)))
			continue;

		controller = to_basis_controller(dev);
		if (!try_module_get(controller->ops->owner)) {
			ret = -EINVAL;
			goto err;
		}

		class_dev_iter_exit(&iter);
		get_device(&controller->dev);
		return controller;
	}

err:
	class_dev_iter_exit(&iter);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(basis_controller_get);

void basis_controller_stop(struct basis_controller *controller)
{
	unsigned long flags;

	if (IS_ERR(controller) || !controller->ops->stop)
		return;

	spin_lock_irqsave(&controller->lock, flags);
	controller->ops->stop(controller);
	spin_unlock_irqrestore(&controller->lock, flags);
}
EXPORT_SYMBOL_GPL(basis_controller_stop);

int basis_controller_start(struct basis_controller *controller)
{
	int ret;
	unsigned long flags;

	if (IS_ERR(controller))
		return -EINVAL;

	if (!controller->ops->start)
		return 0;

	spin_lock_irqsave(&controller->lock, flags);
	ret = controller->ops->start(controller);
	spin_unlock_irqrestore(&controller->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(basis_controller_start);

int basis_controller_add_device(struct basis_controller *controller,
                                struct basis_device *device)
{
	unsigned long flags;

	if (device->controller)
		return -EBUSY;

	if (IS_ERR(controller))
		return -EINVAL;

	device->controller = controller;

	spin_lock_irqsave(&controller->lock, flags);
	list_add_tail(&device->list, &controller->devices);
	spin_unlock_irqrestore(&controller->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(basis_controller_add_device);

void basis_controller_remove_device(struct basis_controller *controller,
                                    struct basis_device *device)
{
	unsigned long flags;

	if (!controller || IS_ERR(controller) || !device)
		return;

	spin_lock_irqsave(&controller->lock, flags);
	list_del(&device->list);
	device->controller = NULL;
	spin_unlock_irqrestore(&controller->lock, flags);
}
EXPORT_SYMBOL_GPL(basis_controller_remove_device);

void basis_controller_destroy(struct basis_controller *controller)
{
	basis_cfs_remove_controller_group(controller->group);
	device_unregister(&controller->dev);
	kfree(controller);
}
EXPORT_SYMBOL_GPL(basis_controller_destroy);

void devm_basis_controller_destroy(struct device *dev,
                                   struct basis_controller *controller)
{
	int r;

	r = devres_destroy(dev, devm_basis_controller_release,
	                   devm_basis_controller_match, controller);
	dev_WARN_ONCE(dev, r, "couldn't find BASIS Controller resource\n");
}
EXPORT_SYMBOL_GPL(devm_basis_controller_destroy);

struct basis_controller *
__basis_controller_create(struct device *dev,
                          const struct basis_controller_ops *ops,
                          struct module *owner)
{
	int ret;
	struct basis_controller *controller;

	if (WARN_ON(!dev)) {
		ret = -EINVAL;
		goto err_ret;
	}

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller) {
		ret = -ENOMEM;
		goto err_ret;
	}

	spin_lock_init(&controller->lock);
	INIT_LIST_HEAD(&controller->devices);

	device_initialize(&controller->dev);
	controller->dev.class = basis_controller_class;
	controller->dev.parent = dev;
	controller->ops = ops;

	ret = dev_set_name(&controller->dev, "%s", dev_name(dev));
	if (ret)
		goto put_dev;

	ret = device_add(&controller->dev);
	if (ret)
		goto put_dev;

	controller->group = basis_cfs_add_controller_group(dev_name(dev));

	return controller;

put_dev:
	put_device(&controller->dev);
	kfree(controller);

err_ret:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(__basis_controller_create);

struct basis_controller *
__devm_basis_controller_create(struct device *dev,
                               const struct basis_controller_ops *ops,
                               struct module *owner)
{
	struct basis_controller **ptr, *controller;

	ptr = devres_alloc(devm_basis_controller_release, sizeof(*ptr),
	                   GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	controller = __basis_controller_create(dev, ops, owner);
	if (!IS_ERR(controller)) {
		*ptr = controller;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return controller;
}
EXPORT_SYMBOL_GPL(__devm_basis_controller_create);

static void basis_controller_release(struct device *dev)
{
}

int basis_controller_init(void)
{
	basis_controller_class = class_create(THIS_MODULE, "basis_controller");
	if (IS_ERR(basis_controller_class)) {
		pr_err("failed to create BASIS controller class --> %ld\n",
		       PTR_ERR(basis_controller_class));
		return PTR_ERR(basis_controller_class);
	}

	basis_controller_class->dev_release = basis_controller_release;

	return 0;
}

void basis_controller_exit(void)
{
	class_destroy(basis_controller_class);
}
