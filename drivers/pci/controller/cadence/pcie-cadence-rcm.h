/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#ifndef _PCIE_CADENCE_RCM_H
#define _PCIE_CADENCE_RCM_H

#include <linux/regmap.h>

#include "pcie-cadence.h"

/**
 * struct cdns_pcie_ep - private data for this PCIe endpoint controller driver
 * @pcie: Cadence PCIe controller
 * @max_regions: maximum number of regions supported by hardware
 * @ob_region_map: bitmask of mapped outbound regions
 * @ob_addr: base addresses in the AXI bus where the outbound regions start
 * @irq_phys_addr: base address on the AXI bus where the MSI/legacy IRQ
 *		   dedicated outbound regions is mapped.
 * @irq_cpu_addr: base address in the CPU space where a write access triggers
 *		  the sending of a memory write (MSI) / normal message (legacy
 *		  IRQ) TLP through the PCIe bus.
 * @irq_pci_addr: used to save the current mapping of the MSI/legacy IRQ
 *		  dedicated outbound region.
 * @irq_pci_fn: the latest PCI function that has updated the mapping of
 *		the MSI/legacy IRQ dedicated outbound region.
 * @irq_pending: bitmask of asserted legacy IRQs.
 * @csc: CSC region.
 */
struct rcm_cdns_pcie_ep {
	struct cdns_pcie      pcie;
	struct pci_epc       *epc;
	u32                   max_regions;
	u32                   bar_msix_table;
	unsigned long         ob_region_map;
	phys_addr_t          *ob_addr;
	phys_addr_t           irq_phys_addr;
	void __iomem         *irq_cpu_addr;
	u64                   irq_pci_addr;
	u8                    irq_pci_fn;
#ifdef CONFIG_TARGET_1879VM8YA
	struct regmap        *csc;
#endif
#ifdef CONFIG_TARGET_1888BC048
	void __iomem         *ext_irq_gen_base;
	void __iomem         *timer_base;
	int                   irq_timer;
	struct tasklet_struct tasklet;
#endif
};

/* Register access */
static inline void rcm_cdns_pcie_writeb(struct cdns_pcie *pcie, u32 reg,
                                        u8 value)
{
	u32 off = (reg & 0x3);
	u32 val = readl(pcie->reg_base + reg - off);
	u32 mask = ~(0xFF << (off * 8));

	val = (val & mask) | ((u32)value << (off * 8));

	writel(val, pcie->reg_base + reg - off);
}

static inline void rcm_cdns_pcie_writew(struct cdns_pcie *pcie, u32 reg,
                                        u16 value)
{
	u32 off = (reg & 0x3);
	u32 val = readl(pcie->reg_base + reg - off);
	u32 mask = ~(0xFFFF << (off * 8));

	val = (val & mask) | ((u32)value << (off * 8));

	writel(val, pcie->reg_base + reg - off);
}

/* Root Port register access */
static inline void rcm_cdns_pcie_rp_writeb(struct cdns_pcie *pcie,
                                           u32 reg, u8 value)
{
	rcm_cdns_pcie_writeb(pcie, CDNS_PCIE_RP_BASE + reg, value);
}

static inline void rcm_cdns_pcie_rp_writew(struct cdns_pcie *pcie,
                                           u32 reg, u16 value)
{
	rcm_cdns_pcie_writew(pcie, CDNS_PCIE_RP_BASE + reg, value);
}

/* Endpoint Function register access */
static inline void rcm_cdns_pcie_ep_fn_writeb(struct cdns_pcie *pcie, u8 fn,
                                              u32 reg, u8 value)
{
	rcm_cdns_pcie_writeb(pcie, CDNS_PCIE_EP_FUNC_BASE(fn) + reg, value);
}

static inline void rcm_cdns_pcie_ep_fn_writew(struct cdns_pcie *pcie, u8 fn,
                                              u32 reg, u16 value)
{
	rcm_cdns_pcie_writew(pcie, CDNS_PCIE_EP_FUNC_BASE(fn) + reg, value);
}

#ifdef CONFIG_RCM_PCIE_CADENCE_HOST
int rcm_cdns_pcie_host_setup(struct cdns_pcie_rc *rc);
#else
static inline int rcm_cdns_pcie_host_setup(struct cdns_pcie_rc *rc)
{
	return 0;
}
#endif

#ifdef CONFIG_RCM_PCIE_CADENCE_EP
int rcm_cdns_pcie_ep_setup(struct rcm_cdns_pcie_ep *ep);
#else
static inline int rcm_cdns_pcie_ep_setup(struct rcm_cdns_pcie_ep *ep)
{
	return 0;
}
#endif

void rcm_cdns_pcie_set_outbound_region(struct cdns_pcie *pcie, u8 fn,
                                       u32 r, bool is_io,
                                       u64 cpu_addr, u64 pci_addr, size_t size);

#endif /* _PCIE_CADENCE_RCM_H */
