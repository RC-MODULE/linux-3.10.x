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

#include "pcie-cadence-rcm.h"

#define CDNS_PCIE_EP_MIN_APERTURE		128	/* 128 bytes */
#define CDNS_PCIE_EP_IRQ_PCI_ADDR_NONE		0x1

#if defined(CONFIG_TARGET_1879VM8YA) || defined(CONFIG_TARGET_1888BC048)

int rcm_cdns_pcie_ep_setup_priv(struct rcm_cdns_pcie_ep *ep);
int rcm_cdns_pcie_ep_send_legacy_irq(struct rcm_cdns_pcie_ep *ep, u8 fn, 
                                     u8 intx);

#endif


static int rcm_cdns_pcie_ep_write_header(struct pci_epc *epc, u8 fn,
                                         struct pci_epf_header *hdr)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;

	rcm_cdns_pcie_ep_fn_writew(pcie, fn, PCI_DEVICE_ID, hdr->deviceid);
	rcm_cdns_pcie_ep_fn_writeb(pcie, fn, PCI_REVISION_ID, hdr->revid);
	rcm_cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CLASS_PROG, hdr->progif_code);
	rcm_cdns_pcie_ep_fn_writew(pcie, fn, PCI_CLASS_DEVICE,
	                           hdr->subclass_code | 
	                           hdr->baseclass_code << 8);
	rcm_cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CACHE_LINE_SIZE,
	                           hdr->cache_line_size);
	rcm_cdns_pcie_ep_fn_writew(pcie, fn, PCI_SUBSYSTEM_ID, hdr->subsys_id);
	rcm_cdns_pcie_ep_fn_writeb(pcie, fn, PCI_INTERRUPT_PIN,
	                           hdr->interrupt_pin);

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
	rcm_cdns_pcie_ep_fn_writew(pcie, fn, cap + PCI_MSI_FLAGS, flags);

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

static int rcm_cdns_pcie_ep_set_msix(struct pci_epc *epc, u8 fn, u16 interrupts)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u16 flags;
	u32 reg;

	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSIX_FLAGS);
	flags &= ~PCI_MSIX_FLAGS_QSIZE;
	flags |= interrupts;
	rcm_cdns_pcie_ep_fn_writew(pcie, fn, cap + PCI_MSIX_FLAGS, flags);

	reg = ep->bar_msix_table;

	cdns_pcie_ep_fn_writel(pcie, fn, cap + PCI_MSIX_TABLE, reg);

	reg = ep->bar_msix_table | ((interrupts + 1) * PCI_MSIX_ENTRY_SIZE);

	cdns_pcie_ep_fn_writel(pcie, fn, cap + PCI_MSIX_PBA, reg);

	return 0;
}

static int rcm_cdns_pcie_ep_get_msix(struct pci_epc *epc, u8 fn)
{
	struct rcm_cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u16 flags;

	/* Validate that the MSI feature is actually enabled. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSIX_FLAGS);
	if (!(flags & PCI_MSIX_FLAGS_ENABLE))
		return -EINVAL;

	return flags & PCI_MSIX_FLAGS_QSIZE;
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

static int rcm_cdns_pcie_ep_send_msix_irq(struct rcm_cdns_pcie_ep *ep, u8 fn,
                                          u8 interrupt_num)
{
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u16 flags;
	u16 msix_count;
	u32 tbl_offset, bir;
	u32 bar_addr_upper, bar_addr_lower;
	u32 msg_addr_upper, msg_addr_lower;
	u64 tbl_addr;
	u64 msg_addr;
	u64 pci_addr_mask = 0xff;
	u32 msg_data;
	u32 vec_ctrl;
	void __iomem *msix_tbl;
	u32 reg;

	pr_debug("%s: fn = %u, interrupt_num = %u\n", __func__,
	         (u32)fn, (u32)interrupt_num);

	/* Check whether the MSI-X feature has been enabled by the PCI host. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSIX_FLAGS);
	if (!(flags & PCI_MSIX_FLAGS_ENABLE))
	{
		pr_err("%s: MSI-X is not enabled\n", __func__);
		return -EINVAL;
	}

	msix_count = (flags & PCI_MSIX_FLAGS_QSIZE) + 1;

	if (!interrupt_num || interrupt_num > msix_count)
	{
		pr_err("%s: interrupt_num = %u, msi_count = %d\n",
		       __func__, (u32)interrupt_num, (u32)msix_count);
		return -EINVAL;
	}

	tbl_offset = cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSIX_TABLE);
	bir = (tbl_offset & PCI_MSIX_TABLE_BIR);
	tbl_offset &= PCI_MSIX_TABLE_OFFSET;

	reg = PCI_BASE_ADDRESS_0 + (4 * bir);

	bar_addr_lower = 
		cdns_pcie_readl(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR0(fn, bir));
	bar_addr_upper = 
		cdns_pcie_readl(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR1(fn, bir));

	tbl_addr = ((u64) bar_addr_upper) << 32 | bar_addr_lower;
	tbl_addr += (tbl_offset + ((interrupt_num - 1) * PCI_MSIX_ENTRY_SIZE));
	tbl_addr &= PCI_BASE_ADDRESS_MEM_MASK;

	msix_tbl = phys_to_virt(tbl_addr);
	if (!msix_tbl) {
		pr_err("%s: Failed to map MSI-X table\n", __func__);
		return -EINVAL;
	}

	msg_addr_lower = readl(msix_tbl + PCI_MSIX_ENTRY_LOWER_ADDR);
	msg_addr_upper = readl(msix_tbl + PCI_MSIX_ENTRY_UPPER_ADDR);
	msg_addr = ((u64) msg_addr_upper) << 32 | msg_addr_lower;
	msg_data = readl(msix_tbl + PCI_MSIX_ENTRY_DATA);
	vec_ctrl = readl(msix_tbl + PCI_MSIX_ENTRY_VECTOR_CTRL);

	pr_debug("%s: msg_addr_lower = 0x%X, msg_addr_upper = 0x%X,"
	         "msg_data = 0x%X, vec_ctrl = 0x%X\n", __func__,
	         msg_addr_lower, msg_addr_upper, msg_data, vec_ctrl);

	if (vec_ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT) {
		pr_info("MSI-X entry ctrl set\n");
		return -EPERM;
	}

	/* Set the outbound region if needed. */
	if (unlikely(ep->irq_pci_addr != (msg_addr & ~pci_addr_mask) ||
	             ep->irq_pci_fn != fn)) {
		/* First region was reserved for IRQ writes. */
		rcm_cdns_pcie_set_outbound_region(pcie, fn, 0,
		                                  false,
		                                  ep->irq_phys_addr,
		                                  msg_addr & ~pci_addr_mask,
		                                  pci_addr_mask + 1);
		ep->irq_pci_addr = msg_addr & ~pci_addr_mask;
		ep->irq_pci_fn = fn;
	}

	writel(msg_data, ep->irq_cpu_addr + (msg_addr & pci_addr_mask));

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

	case PCI_EPC_IRQ_MSIX:
		return rcm_cdns_pcie_ep_send_msix_irq(ep, fn, interrupt_num);

	default:
		pr_err("Can't raise IRQ (unsupported type = %d)\n", (int)type);

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
	.msix_capable = true,
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
	.set_msix       = rcm_cdns_pcie_ep_set_msix,
	.get_msix       = rcm_cdns_pcie_ep_get_msix,
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

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	if (of_property_read_u8(np, "max-functions", &epc->max_functions) < 0)
		epc->max_functions = 1;

	if (of_property_read_u32(np, "bar-msix-table", &ep->bar_msix_table) < 0)
		ep->bar_msix_table = 0;

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

#if defined(CONFIG_TARGET_1879VM8YA) || defined(CONFIG_TARGET_1888BC048)
	ret = rcm_cdns_pcie_ep_setup_priv(ep);

	if (ret)
		goto free_epc_mem;
#endif

	return 0;

 free_epc_mem:
	pci_epc_mem_exit(epc);

 err_init:
	pm_runtime_put_sync(dev);

	return ret;
}
