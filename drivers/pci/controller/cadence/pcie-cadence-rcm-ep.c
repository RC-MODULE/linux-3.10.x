/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci-epc.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/mfd/syscon.h>

#include "pcie-cadence-rcm.h"

#define CDNS_PCIE_EP_MIN_APERTURE		128	/* 128 bytes */
#define CDNS_PCIE_EP_IRQ_PCI_ADDR_NONE		0x1

void rcm_cdns_pcie_set_outbound_region(struct cdns_pcie *pcie, u8 fn,
                                       u32 r, bool is_io,
                                       u64 cpu_addr, u64 pci_addr, size_t size)
{
	/*
	 * roundup_pow_of_two() returns an unsigned long, which is not suited
	 * for 64bit values.
	 */
	u64 sz = 1ULL << fls64(size - 1);
	int nbits = ilog2(sz);
	u32 addr0, addr1, desc0, desc1;

	if (nbits < 8)
		nbits = 8;

	/* Set the PCI address */
	addr0 = CDNS_PCIE_AT_OB_REGION_PCI_ADDR0_NBITS(nbits) |
	        (lower_32_bits(pci_addr) & GENMASK(31, 8));
	addr1 = upper_32_bits(pci_addr);

	pr_debug("%s: addr0 = 0x%08X\n", __func__, addr0);
	pr_debug("%s: addr1 = 0x%08X\n", __func__, addr1);

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_PCI_ADDR0(r), addr0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_PCI_ADDR1(r), addr1);

	/* Set the PCIe header descriptor */
	if (is_io)
		desc0 = CDNS_PCIE_AT_OB_REGION_DESC0_TYPE_IO;
	else
		desc0 = CDNS_PCIE_AT_OB_REGION_DESC0_TYPE_MEM;
	desc1 = 0;

	/*
	 * Whatever Bit [23] is set or not inside DESC0 register of the outbound
	 * PCIe descriptor, the PCI function number must be set into
	 * Bits [26:24] of DESC0 anyway.
	 *
	 * In Root Complex mode, the function number is always 0 but in Endpoint
	 * mode, the PCIe controller may support more than one function. This
	 * function number needs to be set properly into the outbound PCIe
	 * descriptor.
	 *
	 * Besides, setting Bit [23] is mandatory when in Root Complex mode:
	 * then the driver must provide the bus, resp. device, number in
	 * Bits [7:0] of DESC1, resp. Bits[31:27] of DESC0. Like the function
	 * number, the device number is always 0 in Root Complex mode.
	 *
	 * However when in Endpoint mode, we can clear Bit [23] of DESC0, hence
	 * the PCIe controller will use the captured values for the bus and
	 * device numbers.
	 */
	if (pcie->is_rc) {
		/* The device and function numbers are always 0. */
		desc0 |= CDNS_PCIE_AT_OB_REGION_DESC0_HARDCODED_RID |
		         CDNS_PCIE_AT_OB_REGION_DESC0_DEVFN(0);
		desc1 |= CDNS_PCIE_AT_OB_REGION_DESC1_BUS(pcie->bus);
	} else {
		/*
		 * Use captured values for bus and device numbers but still
		 * need to set the function number.
		 */
		desc0 |= CDNS_PCIE_AT_OB_REGION_DESC0_DEVFN(fn);
	}

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_DESC0(r), desc0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_DESC1(r), desc1);

	/* Set the CPU address */
	addr0 = CDNS_PCIE_AT_OB_REGION_CPU_ADDR0_NBITS(nbits) |
	        (lower_32_bits(cpu_addr) & GENMASK(31, 8));
	addr1 = upper_32_bits(cpu_addr);

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_CPU_ADDR0(r), addr0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_OB_REGION_CPU_ADDR1(r), addr1);

	pr_debug("%s: cpu_addr = 0x%llX, pci_addr = 0x%llX, size = %u,"
	         " fn = %u, %s\n",
	         __func__, cpu_addr, pci_addr, size, (u32)fn,
	         is_io ? "IO" : "MEM");
}

static int rcm_cdns_pcie_ep_write_header(struct pci_epc *epc, u8 fn,
                                         struct pci_epf_header *hdr)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;

	cdns_pcie_ep_fn_writew(pcie, fn, PCI_DEVICE_ID, hdr->deviceid);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_REVISION_ID, hdr->revid);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CLASS_PROG, hdr->progif_code);
	cdns_pcie_ep_fn_writew(pcie, fn, PCI_CLASS_DEVICE,
	                       hdr->subclass_code | hdr->baseclass_code << 8);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CACHE_LINE_SIZE,
	                       hdr->cache_line_size);
	cdns_pcie_ep_fn_writew(pcie, fn, PCI_SUBSYSTEM_ID, hdr->subsys_id);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_INTERRUPT_PIN, hdr->interrupt_pin);

	/*
	 * Vendor ID can only be modified from function 0, all other functions
	 * use the same vendor ID as function 0.
	 */
	if (fn == 0) {
		/* Update the vendor IDs. */
		u32 id = CDNS_PCIE_LM_ID_VENDOR(hdr->vendorid) |
			 CDNS_PCIE_LM_ID_SUBSYS(hdr->subsys_vendor_id);

		cdns_pcie_writel(pcie, CDNS_PCIE_LM_ID, id);
	}

	return 0;
}

static int rcm_cdns_pcie_ep_set_bar(struct pci_epc *epc, u8 fn,
                                    struct pci_epf_bar *epf_bar)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	dma_addr_t bar_phys = epf_bar->phys_addr;
	enum pci_barno bar = epf_bar->barno;
	int flags = epf_bar->flags;
	u32 addr0, addr1, reg, cfg, b, aperture, ctrl;
	u64 sz;

	/* BAR size is 2^(aperture + 7) */
	sz = max_t(size_t, epf_bar->size, CDNS_PCIE_EP_MIN_APERTURE);
	/*
	 * roundup_pow_of_two() returns an unsigned long, which is not suited
	 * for 64bit values.
	 */
	sz = 1ULL << fls64(sz - 1);
	aperture = ilog2(sz) - 7; /* 128B -> 0, 256B -> 1, 512B -> 2, ... */

	if ((flags & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
		ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_IO_32BITS;
	} else {
		bool is_prefetch = !!(flags & PCI_BASE_ADDRESS_MEM_PREFETCH);
		bool is_64bits = sz > SZ_2G;

		if (is_64bits && (bar & 1))
			return -EINVAL;

		if (is_64bits && !(flags & PCI_BASE_ADDRESS_MEM_TYPE_64))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;

		if (is_64bits && is_prefetch)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_PREFETCH_MEM_64BITS;
		else if (is_prefetch)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_PREFETCH_MEM_32BITS;
		else if (is_64bits)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_MEM_64BITS;
		else
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_MEM_32BITS;
	}

	addr0 = lower_32_bits(bar_phys);
	addr1 = upper_32_bits(bar_phys);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR0(fn, bar),
	                 addr0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR1(fn, bar),
	                 addr1);

	if (bar < BAR_4) {
		reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG0(fn);
		b = bar;
	} else {
		reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG1(fn);
		b = bar - BAR_4;
	}

	cfg = cdns_pcie_readl(pcie, reg);
	cfg &= ~(CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE_MASK(b) |
	         CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL_MASK(b));
	cfg |= (CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE(b, aperture) |
	        CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL(b, ctrl));
	cdns_pcie_writel(pcie, reg, cfg);

	return 0;
}

static void rcm_cdns_pcie_ep_clear_bar(struct pci_epc *epc, u8 fn,
                                       struct pci_epf_bar *epf_bar)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	enum pci_barno bar = epf_bar->barno;
	u32 reg, cfg, b, ctrl;

	if (bar < BAR_4) {
		reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG0(fn);
		b = bar;
	} else {
		reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG1(fn);
		b = bar - BAR_4;
	}

	ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_DISABLED;
	cfg = cdns_pcie_readl(pcie, reg);
	cfg &= ~(CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE_MASK(b) |
	         CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL_MASK(b));
	cfg |= CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL(b, ctrl);
	cdns_pcie_writel(pcie, reg, cfg);

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR0(fn, bar), 0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR1(fn, bar), 0);
}

static int rcm_cdns_pcie_ep_map_addr(struct pci_epc *epc, u8 fn,
                                     phys_addr_t addr, u64 pci_addr,
                                     size_t size)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 r;

	r = find_first_zero_bit(&ep->ob_region_map,
	                        sizeof(ep->ob_region_map) * BITS_PER_LONG);
	if (r >= ep->max_regions - 1) {
		dev_err(&epc->dev, "no free outbound region\n");
		return -EINVAL;
	}

	pr_debug("%s: addr = 0x%X, pci_addr = 0x%llX, size = %u, r = %u,"
	         " fn = %u\n",
	         __func__, addr, pci_addr, size, r, (u32)fn);

	rcm_cdns_pcie_set_outbound_region(pcie, fn, r, false, addr, pci_addr,
	                                  size);

	set_bit(r, &ep->ob_region_map);
	ep->ob_addr[r] = addr;

	return 0;
}

static void rcm_cdns_pcie_ep_unmap_addr(struct pci_epc *epc, u8 fn,
                                        phys_addr_t addr)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 r;

	for (r = 0; r < ep->max_regions - 1; r++)
		if (ep->ob_addr[r] == addr)
			break;

	if (r == ep->max_regions - 1)
		return;

	cdns_pcie_reset_outbound_region(pcie, r);

	ep->ob_addr[r] = 0;
	clear_bit(r, &ep->ob_region_map);
}

static int rcm_cdns_pcie_ep_set_msi(struct pci_epc *epc, u8 fn, u8 mmc)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u16 flags;

	/*
	 * Set the Multiple Message Capable bitfield into the Message Control
	 * register.
	 */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	flags = (flags & ~PCI_MSI_FLAGS_QMASK) | (mmc << 1);
	flags |= PCI_MSI_FLAGS_64BIT;
	flags &= ~PCI_MSI_FLAGS_MASKBIT;
	cdns_pcie_ep_fn_writew(pcie, fn, cap + PCI_MSI_FLAGS, flags);

	return 0;
}

static int rcm_cdns_pcie_ep_get_msi(struct pci_epc *epc, u8 fn)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u16 flags, mme;

	/* Validate that the MSI feature is actually enabled. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	if (!(flags & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	/*
	 * Get the Multiple Message Enable bitfield from the Message Control
	 * register.
	 */
	mme = (flags & PCI_MSI_FLAGS_QSIZE) >> 4;

	return mme;
}

static int rcm_cdns_pcie_ep_send_legacy_irq(struct rcm_cdns_pcie_ep *ep, u8 fn, 
                                            u8 intx)
{
	u16 cmd;
	u32 reg;

	cmd = cdns_pcie_ep_fn_readw(&ep->pcie, fn, PCI_COMMAND);
	if (cmd & PCI_COMMAND_INTX_DISABLE)
	{
		pr_info("%s: INTx Message Disabled\n", __func__);
		return -EINVAL;
	}

	regmap_write_bits(ep->csc, 0x18, 0x100, 0x100);

	regmap_write_bits(ep->csc, 0x20, 0x01, 0x01);

	do
	{
		regmap_read(ep->csc, 0x20, &reg);
	} while((reg & 0x04) == 0);

	regmap_write(ep->csc, 0x20, reg | 0x02);

	do
	{
		regmap_read(ep->csc, 0x20, &reg);
	} while((reg & 0x04) == 0);

	regmap_write_bits(ep->csc, 0x18, 0x100, 0x000);

	return 0;
}

static int rcm_cdns_pcie_ep_send_msi_irq(struct rcm_cdns_pcie_ep *ep, u8 fn,
                                         u8 interrupt_num)
{
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u16 flags, mme, data, data_mask;
	u8 msi_count;
	u64 pci_addr, pci_addr_mask = 0xff;

	pr_debug("%s: fn = %u, interrupt_num = %u\n", __func__,
	         (u32)fn, (u32)interrupt_num);

	/* Check whether the MSI feature has been enabled by the PCI host. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	if (!(flags & PCI_MSI_FLAGS_ENABLE))
	{
		pr_debug("%s: MSI is not enabled\n", __func__);
		return -EINVAL;
	}

	/* Get the number of enabled MSIs */
	mme = (flags & PCI_MSI_FLAGS_QSIZE) >> 4;
	msi_count = 1 << mme;
	if (!interrupt_num || interrupt_num > msi_count)
	{
		pr_debug("%s: interrupt_num = %u, msi_count = %d\n",
		         __func__, (u32)interrupt_num, (u32)msi_count);
		return -EINVAL;
	}

	/* Compute the data value to be written. */
	data_mask = msi_count - 1;
	data = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_DATA_64);
	data = (data & ~data_mask) | ((interrupt_num - 1) & data_mask);

	/* Get the PCI address where to write the data into. */
	pci_addr = cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_HI);
	pci_addr <<= 32;
	pci_addr |= cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_LO);
	pci_addr &= GENMASK_ULL(63, 2);

	pr_debug("%s: data = 0x%X, pci_addr = 0x%llX, irq_phys_addr = 0x%X\n",
	         __func__, (u32)data, pci_addr, ep->irq_phys_addr);

	/* Set the outbound region if needed. */
	if (unlikely(ep->irq_pci_addr != (pci_addr & ~pci_addr_mask) ||
	             ep->irq_pci_fn != fn)) {
		/* First region was reserved for IRQ writes. */
		rcm_cdns_pcie_set_outbound_region(pcie, fn, 0,
		                                  false,
		                                  ep->irq_phys_addr,
		                                  pci_addr & ~pci_addr_mask,
		                                  pci_addr_mask + 1);
		ep->irq_pci_addr = (pci_addr & ~pci_addr_mask);
		ep->irq_pci_fn = fn;
	}
	writel(data, ep->irq_cpu_addr + (pci_addr & pci_addr_mask));

	return 0;
}

static int rcm_cdns_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn,
                                      enum pci_epc_irq_type type,
                                      u16 interrupt_num)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return rcm_cdns_pcie_ep_send_legacy_irq(ep, fn, 0);

	case PCI_EPC_IRQ_MSI:
		return rcm_cdns_pcie_ep_send_msi_irq(ep, fn, interrupt_num);

	default:
		break;
	}

	return -EINVAL;
}

static int rcm_cdns_pcie_ep_start(struct pci_epc *epc)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	struct pci_epf *epf;
	u32 cfg;

	/*
	 * BIT(0) is hardwired to 1, hence function 0 is always enabled
	 * and can't be disabled anyway.
	 */
	cfg = BIT(0);
	list_for_each_entry(epf, &epc->pci_epf, list)
		cfg |= BIT(epf->func_no);
	cdns_pcie_writel(pcie, CDNS_PCIE_LM_EP_FUNC_CFG, cfg);

	return 0;
}

static const struct pci_epc_features rcm_cdns_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
};

static const struct pci_epc_features*
rcm_cdns_pcie_ep_get_features(struct pci_epc *epc, u8 func_no)
{
	return &rcm_cdns_pcie_epc_features;
}

static const struct pci_epc_ops rcm_cdns_pcie_epc_ops = {
	.write_header   = rcm_cdns_pcie_ep_write_header,
	.set_bar        = rcm_cdns_pcie_ep_set_bar,
	.clear_bar      = rcm_cdns_pcie_ep_clear_bar,
	.map_addr       = rcm_cdns_pcie_ep_map_addr,
	.unmap_addr     = rcm_cdns_pcie_ep_unmap_addr,
	.set_msi        = rcm_cdns_pcie_ep_set_msi,
	.get_msi        = rcm_cdns_pcie_ep_get_msi,
	.raise_irq      = rcm_cdns_pcie_ep_raise_irq,
	.start          = rcm_cdns_pcie_ep_start,
	.get_features   = rcm_cdns_pcie_ep_get_features,
};


int rcm_cdns_pcie_ep_setup(struct rcm_cdns_pcie_ep *ep)
{
	struct device *dev = ep->pcie.dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	struct cdns_pcie *pcie = &ep->pcie;
	struct resource *res;
	struct pci_epc *epc;
	struct device_node* node_csc;
	int ret;

	pcie->is_rc = false;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg");
	pcie->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->reg_base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(pcie->reg_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mem");
	if (!res) {
		dev_err(dev, "missing \"mem\"\n");
		return -EINVAL;
	}
	pcie->mem_res = res;

	ret = of_property_read_u32(np, "cdns,max-outbound-regions",
	                           &ep->max_regions);
	if (ret < 0) {
		dev_err(dev, "missing \"cdns,max-outbound-regions\"\n");
		return ret;
	}
	ep->ob_addr = devm_kcalloc(dev,
	                           ep->max_regions, sizeof(*ep->ob_addr),
	                           GFP_KERNEL);
	if (!ep->ob_addr)
		return -ENOMEM;

	/* Disable all but function 0 (anyway BIT(0) is hardwired to 1). */
	cdns_pcie_writel(pcie, CDNS_PCIE_LM_EP_FUNC_CFG, BIT(0));

	epc = devm_pci_epc_create(dev, &rcm_cdns_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		ret = PTR_ERR(epc);
		goto err_init;
	}

	epc_set_drvdata(epc, ep);

	if (of_property_read_u8(np, "max-functions", &epc->max_functions) < 0)
		epc->max_functions = 1;

	ret = pci_epc_mem_init(epc, pcie->mem_res->start,
	                       resource_size(pcie->mem_res));
	if (ret < 0) {
		dev_err(dev, "failed to initialize the memory space\n");
		goto err_init;
	}

	ep->irq_cpu_addr = pci_epc_mem_alloc_addr(epc, &ep->irq_phys_addr,
	                                          SZ_128K);
	if (!ep->irq_cpu_addr) {
		dev_err(dev, "failed to reserve memory space for MSI\n");
		ret = -ENOMEM;
		goto free_epc_mem;
	}
	ep->irq_pci_addr = CDNS_PCIE_EP_IRQ_PCI_ADDR_NONE;
	/* Reserve region 0 for IRQs */
	set_bit(0, &ep->ob_region_map);

	cdns_pcie_reset_outbound_region(pcie, 0);

	node_csc = of_parse_phandle(np, "csc", 0);
	if (!node_csc) {
		dev_err(dev, "failed to find csc node\n");
		ret = -EFAULT;
		goto free_epc_mem;
	}
	ep->csc = syscon_node_to_regmap(node_csc);

	return 0;

 free_epc_mem:
	pci_epc_mem_exit(epc);

 err_init:
	pm_runtime_put_sync(dev);

	return ret;
}
