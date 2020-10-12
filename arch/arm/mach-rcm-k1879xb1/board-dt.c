/*
 *  arch/arm/mach-rcm-k1879/board-dt.c
 *
 *  Copyright (C) 2011-2015 RC Module
 *
 *  Sergey Mironov <ierton@gmail.com>
 *  Andrew Andrianov <andrew@ncrmnt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>

#if defined(CONFIG_FB_RCM_VDU) || defined(CONFIG_FB_RCM_VDU_MODULE)
#include <linux/rcm_vdu.h>
#endif

#if defined(CONFIG_RCM_HDMI_BI)
#include "module_hdmi.h"
#endif

#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/memblock.h>
#include "hardware.h"

static void __iomem *k1879_mif;
static void __iomem *k1879_wdt;

static void __iomem *k1879_mif_base(void)
{
	BUG_ON(!k1879_mif);
	return k1879_mif;
}

static void __iomem *k1879_sctl_base(void)
{
	return (void __iomem *)RCM_K1879_SCTL_VIRT_BASE;
}

static void k1879_level_irq_i2c0_fixup(struct irq_desc *desc)
{
	writel(1, k1879_mif_base() + RCM_K1879_MIF_I2C_INT_STAT);
	handle_level_irq(desc);
}

static void k1879_level_irq_i2c1_fixup(struct irq_desc *desc)
{
	writel(1 << 0, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(desc);
}

static void k1879_level_irq_i2c2_fixup(struct irq_desc *desc)
{
	writel(1 << 1, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(desc);
}

static void k1879_level_irq_i2c3_fixup(struct irq_desc *desc)
{
	writel(1 << 2, k1879_sctl_base() + RCM_K1879_SCTL_INT_P_OUT);
	handle_level_irq(desc);
}

static void (*k1879_i2c_fixups[])(struct irq_desc *desc) = {
	k1879_level_irq_i2c0_fixup,
	k1879_level_irq_i2c1_fixup,
	k1879_level_irq_i2c2_fixup,
	k1879_level_irq_i2c3_fixup
};

static void __init setup_i2c_fixups(void)
{
	struct device_node *np;
	int irq;
	u32 fixup_id = -1;

	for_each_compatible_node(np, NULL, "rcm,i2c-ocores") {
		int ret = of_property_read_u32(np, "rcm-irq-fixup-id",
					       &fixup_id);

		if (ret) {
			pr_warn("k1879xb1: rcm-irq-fixup-id invalid: %d\n",
				ret);
			continue;
		}
		if (fixup_id >= ARRAY_SIZE(k1879_i2c_fixups)) {
			pr_warn("k1879xb1: rcm-irq-fixup-id %d too big\n",
				fixup_id);
			continue;
		}
		irq = of_irq_get(np, 0);
		irq_set_handler(irq, k1879_i2c_fixups[fixup_id]);
	}
}
/*
static void __init k1879_dt_timer_init(void) // need?
{
	of_clk_init(NULL);
	timer_probe();
}
*/
static void __init k1879_dt_mach_init(void)
{
	struct device_node *np;
	void __iomem *address;
	void __iomem *mif;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	k1879_mif = ioremap(RCM_K1879_MIF_PHYS_BASE,
			    RCM_K1879_MIF_SIZE);
	BUG_ON(!k1879_mif);

	np = of_find_compatible_node(NULL, NULL, "arm,sp805");
	WARN_ON(!np);
	if (np)
		k1879_wdt = of_iomap(np, 0);

	/* Setup i2c interrupt fixups */
	setup_i2c_fixups();

	//setting DDR QOS (all priority to vdu)
	iowrite32(4, (void __iomem*)0xf8032004);
	iowrite32(3, (void __iomem*)0xf8032124);
	iowrite32(0, (void __iomem*)0xf8032004);
	iowrite32(0, (void __iomem*)0xf8032004);

	//setting DDR QOS (all priority to msvdhddec)
	iowrite32(3, (void __iomem*)0xf8031004);
	while((ioread32((void __iomem*)0xf8031000) & 0x03) != 0x2);
	iowrite32(4, (void __iomem*)0xf8031004);
	while((ioread32((void __iomem*)0xf8031000) & 0x03) != 0x0);

	iowrite32(0x00001450, (void __iomem*)0xf8031010);
	iowrite32(0x00000005, (void __iomem*)0xf8031028);
	iowrite32(0x00000023, (void __iomem*)0xf803102c);
	iowrite32(0x00000027, (void __iomem*)0xf8031044);
	iowrite32(0x00000014, (void __iomem*)0xf8031048);
	iowrite32(0x00000014, (void __iomem*)0xf8031054);
	iowrite32(0x00000003, (void __iomem*)0xf803112c);

	iowrite32(0, (void __iomem*)0xf8031004);

	//some more media qos
	address = ioremap_nocache(0x80174400, 4*1024);
	iowrite32(0xC, address);
	iowrite32(0x400, address + 4);

	// Some audio dma-related stuff
	iowrite32(0x0, (void __iomem*)0xf803c000);
	iowrite32(0x1, (void __iomem*)0xf803c000);
	iowrite32(0x1, (void __iomem*)0xf803c000);
	iowrite32(0x1, (void __iomem*)0xf803c000);
	iowrite32(0x1, (void __iomem*)0xf803c000);
	iowrite32(0x1, (void __iomem*)0xf803c000);

	iowrite32(0x2ac1, (void __iomem*)0xf803c000);

	/* set axi2spi to APB mode; 0xf802e100 - AXI_SPI_MODE reg */
	iowrite32(0x1, (void __iomem*)0xf802e100);

#if defined(CONFIG_RCM_GRI2C_HDMI)
	/* I2C0 (Internal, HDMI) needs some extra love */
	do {
		pr_notice("i2c hdmi init\n");
		mif = k1879_mif_base();
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE_ENA);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_TYPE);
		writel(1, mif + RCM_K1879_MIF_I2C_INT_ENA);
	} while (0);
#endif
}

#if defined(CONFIG_RCM_MVDU_CORE)
int rcm_setup_vmode(/*unsigned int hz, int hd*/struct mvdu_device *dev)
{
	const struct mvdu_mode *mode = mvdu_get_modeinfo(dev, dev->current_mode);
	unsigned int hz = mode->pixclock;
	bool hd = (mode->mode > 2);
	u32 __iomem *mif_regs = k1879_mif_base();

	iowrite32(1, &mif_regs[0]);


	if (hz >= 74250000) {
		iowrite32(1, &mif_regs[1]);
	} else {
		iowrite32(0, &mif_regs[1]);
	}

#if defined(CONFIG_RCM_GRI2C_HDMI)
	if (hd)
		module_hdmi_video_setup_hd();
	else
		module_hdmi_video_setup_sd();
#endif

	pr_notice("vmode change: pixclock %u mode %s", hz, hd ? "HD" : "SD");

	return 0;
}
#else
int rcm_setup_vmode(/*unsigned int hz, int hd*/ struct mvdu_device *dev) {return 0;};
#endif /* end CONFIG_MODULE_MVDU_CORE */
EXPORT_SYMBOL(rcm_setup_vmode);

static void k1879_restart(enum reboot_mode mode, const char *cmd)
{
	/* The recommended way to do a soft-reboot on this platform
	 *  is write directly to watchdog registers and cause an immediate
	 *  watchdog restart
	 */

	if (!k1879_wdt) {
		pr_err("k1879xb1: WDT regs were not properly mapped - reboot manually\n");
		return;
	}
	iowrite32(1, (k1879_wdt + 0xf00));
	iowrite32(1, (k1879_wdt + 0xf04));
	/* Goodbye! */
}

static const char * const k1879_dt_match[] = {
	"rcm,mb7707",
	"rcm,k1879xb1ya",
	NULL
};

static struct map_desc k1879_io_desc[] __initdata = {
	{
		.virtual	= RCM_K1879_AREA0_VIRT_BASE,
		.pfn	= __phys_to_pfn(RCM_K1879_AREA0_PHYS_BASE),
		.length	= RCM_K1879_AREA0_SIZE,
		.type	= MT_DEVICE,
	},
	// FIXME: This static mapping is actually redundant. We are using it for
	// direct access to multimedia registers.
	{
		.virtual	= 0xF9000000,
		.pfn	= __phys_to_pfn(RCM_K1879_AREA1_PHYS_BASE),
		.length	= 2*RCM_K1879_AREA1_SIZE,
		.type	= MT_DEVICE,
	}
};

static void __init k1879_map_io(void) {
    iotable_init(k1879_io_desc, ARRAY_SIZE(k1879_io_desc));
}

DT_MACHINE_START(K1879, "Module MB77.07  (Device Tree)")
	.map_io			= k1879_map_io,
	.init_machine   = k1879_dt_mach_init,
	.dt_compat      = k1879_dt_match,
	.restart        = k1879_restart
MACHINE_END
