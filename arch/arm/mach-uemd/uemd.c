/*
 *  arch/arm/mach-uemd/uemd.c
 *
 *  Copyright (C) 2011
 *
 *  Sergey Mironov <ierton@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>

#include <mach/memory.h>
#include <mach/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/setup.h>
#include <asm/hardware/vic.h>
#include <asm/memblock.h>

#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/module_vdu.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>


#include <linux/serial_8250.h>

#include "platform.h"
#include "module_hdmi.h"
#include "uemd.h"

void __iomem *g_uemd_mif = 0;

void __iomem *uemd_mif_base(void)
{
	BUG_ON(g_uemd_mif == 0);
	return g_uemd_mif;
}

void __iomem *uemd_sctl_base(void)
{
	return (void __iomem *)UEMD_SCTL_VIRT_BASE;
}


static void uemd_level_irq_i2c0_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1, uemd_mif_base() + UEMD_MIF_I2C_INT_STAT);
	handle_level_irq(irq, desc);
}

static void uemd_level_irq_i2c1_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<0, uemd_sctl_base() + UEMD_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void uemd_level_irq_i2c2_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<1, uemd_sctl_base() + UEMD_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

static void uemd_level_irq_i2c3_fixup(unsigned int irq, struct irq_desc *desc)
{
	writel(1<<2, uemd_sctl_base() + UEMD_SCTL_INT_P_OUT);
	handle_level_irq(irq, desc);
}

int uemd_is_virgin()
{
	void __iomem *address = (void*)(UEMD_AREA0_VIRT_BASE + SMCONFIG_REG_H- UEMD_AREA0_PHYS_BASE);
	return ~(ioread32(address) & SMCONFIG_CONF_LOCK);
}
EXPORT_SYMBOL(uemd_is_virgin);

static struct map_desc uemd_io_desc[] __initdata = {
	{
		.virtual	= UEMD_AREA0_VIRT_BASE,
		.pfn		= __phys_to_pfn(UEMD_AREA0_PHYS_BASE),
		.length		= UEMD_AREA0_SIZE,
		.type		= MT_DEVICE,
	},
	// FIXME: This static mapping is actually redundant. We are using it for
	// direct access to multimedia registers.
	{
		.virtual	= 0xF9000000,
		.pfn		= __phys_to_pfn(UEMD_AREA1_PHYS_BASE),
		.length		= 2*UEMD_AREA1_SIZE,
		.type		= MT_DEVICE,
	}
};

static void __init uemd_map_io(void)
{
	iotable_init(uemd_io_desc, ARRAY_SIZE(uemd_io_desc));
}

/* Setup the memory banks */
void uemd_fixup(struct machine_desc *mdesc, struct tag *tags, char **from,
	struct meminfo *meminfo)
{
	meminfo->bank[0].start = PHYS_OFFSET;
	meminfo->bank[0].size = PHYS_SIZE;
	meminfo->nr_banks = 1;
}

#if defined(CONFIG_MODULE_MSVDHD)
static inline void msvdhd_preinit(void)
{
	/* Before registering msvdhd device, setup memory controller
	 * to workaround MSVDHD issues.
	 *
	 * Need to configure OS channel priority to maximum
	 */

	struct dmc_regs {
		u32 status;	/* offset 0x000 */
		u32 command;	/* offset 0x004 */
		unsigned char gap[0x104];
		u32 preg;	/* offset 0x10c */
	};

	struct dmc_regs __iomem *dmc;

	pr_info("msvdhd: configuring memory\n");

	/* Map registers */
	dmc = ioremap(UEMD_DMC1_PHYS_BASE, sizeof(*dmc));
	if (unlikely(!dmc)) {
		pr_err("msvdhd: could not map memory controller regs\n");
		return;
	}

	/* Send pause command */
	iowrite32(0x3, &dmc->command);
	while ((ioread32(&dmc->status) & 0x3) != 0x2);

	/* Send configure command */
	iowrite32(0x4, &dmc->command);
	while ((ioread32(&dmc->status) & 0x3) != 0x0);

	/* configure it */
	iowrite32(0x3, &dmc->preg);

	/* Send go command */
	iowrite32(0x0, &dmc->command);

	iounmap(dmc);
}
#else
static inline void msvdhd_preinit(void) {}
#endif /* MSVDHD */

void uemd_pm_restart(char mode, const char *cmd)
{
	void __iomem *regs;
	pr_info("platform: Requested system restart\n");
	regs = ioremap_nocache(0x20025000, 0xfff);
	iowrite32(1, (regs + 0xf00));
	iowrite32(1, (regs + 0xf04));
	/* Goodbye! */
}

static void __init uemd_init(void)
{
	void __iomem *address;

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	g_uemd_mif = ioremap_nocache(UEMD_MIF_PHYS_BASE, UEMD_MIF_SIZE);
	if(g_uemd_mif == NULL) {
		panic("UEMD: Unable to map MIF registers");
	}

	/* Check OTP ROM state */
	pr_info("OTP ROM is %s flashed\n", uemd_is_virgin() ? "not" : "");

	/* i2c interrupts bug fix */
	irq_set_handler(33, uemd_level_irq_i2c0_fixup);
	irq_set_handler(28, uemd_level_irq_i2c1_fixup);
	irq_set_handler(29, uemd_level_irq_i2c2_fixup);
	irq_set_handler(30, uemd_level_irq_i2c3_fixup);

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

#if defined (CONFIG_MODULE_TSP)
	// Enable TSP external interface
	r = ioread32((void __iomem*)0xf8033084);
	r|=0x30;
	iowrite32(r, (void __iomem*)0xf8033084);

	// DVBCI-related magic
	address = (void*)(UEMD_AREA0_VIRT_BASE + (0x20033084 - UEMD_AREA0_PHYS_BASE));
	iowrite32(ioread32(address) | 0x8, address);
#endif

#if defined(CONFIG_MODULE_GRI2C_HDMI)
	do {
		void __iomem *mif;
		mif = uemd_mif_base();
		writel(1, mif + UEMD_MIF_I2C_INT_TYPE_ENA);
		writel(1, mif + UEMD_MIF_I2C_INT_TYPE);
		writel(1, mif + UEMD_MIF_I2C_INT_ENA);
	} while(0);
#endif

	msvdhd_preinit();
}

#if defined(CONFIG_MODULE_VDU_FB) || defined(CONFIG_MODULE_VDU_FB_MODULE)
static phys_addr_t uemd_phys_fb_base = (phys_addr_t) NULL;

void __init uemd_reserve_fb_memblock(void)
{
	const phys_addr_t size = MVDU_OSD_BYTESPERPIXEL *
			  MVDU_OSD_BUFFER_WIDTH * MVDU_OSD_BUFFER_HEIGHT;
	phys_addr_t paddr;

	if (!size)
		return;

	paddr = arm_memblock_steal(size, SZ_1M);
	if (!paddr) {
		pr_err("%s: failed to reserve %llx bytes\n",
			  __func__, (unsigned long long)size);
		return;
	};

	uemd_phys_fb_base = paddr;
}

phys_addr_t uemd_get_fb_base(void)
{
	return uemd_phys_fb_base;
}
#else
phys_addr_t uemd_get_fb_base(void) {return NULL;};
#endif
EXPORT_SYMBOL(uemd_get_fb_base);

static void __init uemd_reserve(void)
{
#if defined(CONFIG_MODULE_VDU_FB) || defined(CONFIG_MODULE_VDU_FB_MODULE)
	uemd_reserve_fb_memblock();
#endif
}

#if defined(CONFIG_MODULE_MVDU_CORE)
int uemd_setup_vmode(struct mvdu_device *dev)
{
	u32 __iomem *mif_regs = uemd_mif_base();

	iowrite32(1, &mif_regs[0]);

	if (dev->current_mode == MVDU_MODE_SD_486_I_30 ||
	    dev->current_mode == MVDU_MODE_SD_576_I_25) {
		iowrite32(0, &mif_regs[1]);

#if defined(CONFIG_MODULE_GRI2C_HDMI)
		module_hdmi_video_setup_sd();
#endif
	} else {
		iowrite32(1, &mif_regs[1]);

#if defined(CONFIG_MODULE_GRI2C_HDMI)
		module_hdmi_video_setup_hd();
#endif
	};

	return 0;
}
#else
int uemd_setup_vmode(struct mvdu_device *dev) {return 0;};
#endif /* end CONFIG_MODULE_MVDU_CORE */
EXPORT_SYMBOL(uemd_setup_vmode);


static void __init uemd_dt_timer_init(void)
{
	of_clk_init(NULL);
	clocksource_of_init();

	uemd_timer_init();
}

static const char *module_dt_match[] = {
	"module,mb7707",
	NULL
};

DT_MACHINE_START(UEMD, "Module MB77.07")
	.map_io			= uemd_map_io,
	.nr_irqs		= NR_IRQS_LEGACY,
	.init_irq		= irqchip_init,
	.init_time		= uemd_dt_timer_init,
	.init_machine	        = uemd_init,
	.dt_compat		= module_dt_match,
	.reserve		= uemd_reserve,
	.restart                = uemd_pm_restart
MACHINE_END
