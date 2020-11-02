/*
 * drivers/video/rcm/rcm-vdu-core.c - Module vdu hardware abstraction
 *
 *	Copyright (C) 2008 Module
 *	Written by MSU, CMC dept., LVK lab http://lvk.cs.msu.su
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#undef DEBUG

#ifdef DEBUG

/* Dump register accesses */
#undef MVDU_DEBUG_REGISTER_READS
#undef MVDU_DEBUG_REGISTER_WRITES

/* Show verbose IRQ information at bottom of IRQ processing */
#undef MVDU_DEBUG_IRQ

#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/sysfs.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <video/rcm-vdu.h>

#define CRG_VDU_PLL_STATE 0x000
#define CRG_VDU_PLL_CTRL 0x004
#define CRG_VDU_WR_LOCK 0x00C
#define CRG_VDU_RST_MON 0x010
#define CRG_VDU_RST_CTRL 0x014
#define CRG_VDU_RST_EN 0x018
#define CRG_VDU_UPD_CK 0x01C
#define CRG_VDU_CKALIGN 0x020
#define CRG_VDU_CKDIVSTAT 0x024
#define CRG_VDU_PLL_PRDIV 0x030
#define CRG_VDU_PLL_FBDIV 0x034
#define CRG_VDU_PLL_PSDIV 0x038

#define CRG_VDU_CKDIVMODE_VOUT0 0x100
#define CRG_VDU_CKEN_VOUT0 0x104

#define CRG_VDU_CKDIVMODE_VOUT1 0x110
#define CRG_VDU_CKEN_VOUT1 0x114

#define CRG_VDU_CKDIVMODE_CODEC 0x120
#define CRG_VDU_CKEN_CODEC 0x124

#define CRG_VDU_PLL_STATE_PLL_RDY (1U << 0)

#define CRG_VDU_PLL_CTRL_PLL_RESTART (1U << 0)
#define CRG_VDU_PLL_CTRL_PLL_ENABLE (1U << 1)
#define CRG_VDU_PLL_CTRL_PLL_BYPASS (1U << 4)

#define CRG_VDU_WR_LOCK_LOCKED 0x00000001
#define CRG_VDU_WR_LOCK_UNLOCKED 0x00000000
#define CRG_VDU_WR_LOCK_LOCK 0x00000001
#define CRG_VDU_WR_LOCK_UNLOCK 0x1ACCE551

#define CRG_VDU_UPD_CK_UPD_CKDIV (1U << 0)

#ifndef STR
#define __STR(x) #x
#define STR(x) __STR(x)
#endif

#define DEFAULT_MAX_OSD_AREAS	4
static int max_osd_areas = DEFAULT_MAX_OSD_AREAS;
module_param(max_osd_areas, int, 0444);
MODULE_PARM_DESC(max_osd_areas, "Number of OSD areas supported (default: " STR(DEFAULT_MAX_OSD_AREAS) ")");

static int initial_mode = MVDU_MODE_DEFAULT;
module_param(initial_mode, int, 0444);
MODULE_PARM_DESC(initial_mode, "Initial video mode number (default: arch-specific)");

#define shift_value(value, field)	(((value) << MVDU_REG_##field##_SHIFT) &\
					MVDU_REG_##field##_MASK)

static inline void write_reg(struct mvdu_device *dev, u32 val, unsigned int offset)
{
#ifdef MVDU_DEBUG_REGISTER_WRITES
	dev_dbg(dev->dev, "[%08lx] <= %08x\n", dev->regs_phys + offset, val);
#endif
	iowrite32(val, (u32 __iomem *)((unsigned long)dev->regs + offset));
}

static inline u32 read_reg(struct mvdu_device *dev, unsigned int offset)
{
	u32 val = ioread32((u32 __iomem *)((unsigned long)dev->regs + offset));
#ifdef MVDU_DEBUG_REGISTER_READS
	dev_dbg(dev->dev, "[%08lx] => %08x\n", dev->regs_phys + offset, val);
#endif
	return val;
}

#ifdef DEBUG
void mvdu_dump_registers(struct mvdu_device *dev)
{
	printk("CTRL_VDU_ID = 0x%08x\n", read_reg(dev, MVDU_REG_CTRL_VDU_ID));
	printk("CTRL_VDU_ENA = 0x%08x\n", read_reg(dev, MVDU_REG_CTRL_VDU_ENA));
	printk("CTRL_SOFT_RESET = 0x%08x\n", read_reg(dev, MVDU_REG_CTRL_SOFT_RESET));
	printk("STAT_VDU_DISP = 0x%08x\n", read_reg(dev, MVDU_REG_STAT_VDU_DISP));
	printk("STAT_VDU_AVMP = 0x%08x\n", read_reg(dev, MVDU_REG_STAT_VDU_AVMP));
	printk("STAT_VDU_FIFO = 0x%08x\n", read_reg(dev, MVDU_REG_STAT_VDU_FIFO));
	printk("INT_VDU_ENA = 0x%08x\n", read_reg(dev, MVDU_REG_INT_VDU_ENA));
	printk("INT_VDU_STAT = 0x%08x\n", read_reg(dev, MVDU_REG_INT_VDU_STAT));
	printk("SCALER_CTRL = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_CTRL));
	printk("SCALER_SCH_Y = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SCH_Y));
	printk("SCALER_SCV_Y = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SCV_Y));
	printk("SCALER_SCH_C = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SCH_C));
	printk("SCALER_SCV_C = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SCV_C));
	printk("SCALER_SIZE_Y = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SIZE_Y));
	printk("SCALER_SIZE_C = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SIZE_C));
	printk("SCALER_FLT_Y_C0 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_Y_C0));
	printk("SCALER_FLT_Y_C1 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_Y_C1));
	printk("SCALER_FLT_Y_C2 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_Y_C2));
	printk("SCALER_FLT_Y_C3 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_Y_C3));
	printk("SCALER_FLT_Y_C4 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_Y_C4));
	printk("SCALER_FLT_NORM = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_NORM));
	printk("SCALER_FLT_C_C0 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_C_C0));
	printk("SCALER_FLT_C_C1 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_C_C1));
	printk("SCALER_FLT_C_C2 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_C_C2));
	printk("SCALER_FLT_C_C3 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_C_C3));
	printk("SCALER_FLT_C_C4 = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_FLT_C_C4));
	printk("SCALER_PHS_CUT_V = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_PHS_CUT_V));
	printk("SCALER_SIZE_CUT = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_SIZE_CUT));
	printk("SCALER_PHS_CUT_H = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_PHS_CUT_H));
	printk("SCALER_MVL_CLR_Y = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_MVL_CLR_Y));
	printk("SCALER_MVL_CLR_Cb = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_MVL_CLR_Cb));
	printk("SCALER_MVL_CLR_Cr = 0x%08x\n", read_reg(dev, MVDU_REG_SCALER_MVL_CLR_Cr));
	printk("DIF_CTRL = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_CTRL));
	printk("DIF_BGR = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_BGR));
	printk("DIF_MVL_START = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_MVL_START));
	printk("DIF_BLANK = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_BLANK));
	printk("DIF_FSIZE = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_FSIZE));
	printk("DIF_ASIZE = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_ASIZE));
	printk("DIF_HSYNC = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_HSYNC));
	printk("DIF_VSYNC = 0x%08x\n", read_reg(dev, MVDU_REG_DIF_VSYNC));
	printk("OSD_BASEreg0 = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_BASEreg0));
	printk("OSD_BASEreg1 = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_BASEreg1));
	printk("OSD_COLOR_Y = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_COLOR_Y));
	printk("OSD_COLOR_Cb = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_COLOR_Cb));
	printk("OSD_COLOR_Cr = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_COLOR_Cr));
	printk("OSD_FIFO_END = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_FIFO_END));
	printk("OSD_CTRL = 0x%08x\n", read_reg(dev, MVDU_REG_OSD_CTRL));
	printk("MI_CTRL = 0x%08x\n", read_reg(dev, MVDU_REG_MI_CTRL));
	printk("MI_MVL_Y_BAreg0 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Y_BAreg0));
	printk("MI_MVL_Cb_BAreg0 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Cb_BAreg0));
	printk("MI_MVL_Cr_BAreg0 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Cr_BAreg0));
	printk("MI_MVL_Y_BAreg1 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Y_BAreg1));
	printk("MI_MVL_Cb_BAreg1 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Cb_BAreg1));
	printk("MI_MVL_Cr_BAreg1 = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Cr_BAreg1));
	printk("MI_MVL_Y_SIZE = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_Y_SIZE));
	printk("MI_MVL_C_SIZE = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_C_SIZE));
	printk("MI_MVL_FULL_SIZE = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_FULL_SIZE));
	printk("MI_AXI_MVL_PARAM = 0x%08x\n", read_reg(dev, MVDU_REG_MI_AXI_MVL_PARAM));
	printk("MI_MVL_FIFO_START = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_FIFO_START));
	printk("MI_MVL_FIFO_END = 0x%08x\n", read_reg(dev, MVDU_REG_MI_MVL_FIFO_END));
	printk("MI_AXI_OSD_PARAM = 0x%08x\n", read_reg(dev, MVDU_REG_MI_AXI_OSD_PARAM));
}
#endif

static struct mvdu_mode mvdu_modes[] = {
        [MVDU_MODE_HD_768_P] = {
                .width = 1024,
                .height = 768,
                .vmode = MVDU_MODE_PROGRESSIVE,
                .pixclock = 74250000,
                .mode = 0x1B,
                .h_blank = 284,
                .h_active = 1024,
                .h_total = 1316,
                .hs_start = 24,
                .hs_len = 136,
                .hs_delay = 132,
                .v_blank_beg = 36,
                .v_active = 768,
                .v_total = 806,
                .v_blank_end = 2,
                .vs_start = 1,
                .vs_len = 6,
                .vs_p = 1,
        },
		[MVDU_MODE_HD_480_P] = {
			.width = 800,
			.height = 480,
			.vmode = MVDU_MODE_PROGRESSIVE,
			.pixclock = 27000000,
			.mode = 0x12,
			.h_blank = 168,
			.h_active = 800,
			.h_total = 976,
			.hs_start = 40,
			.hs_len = 48,
			.hs_delay = 88,
			.v_blank_beg = 41,
			.v_active = 480,
			.v_total = 525,
			.v_blank_end = 7,
			.vs_start = 6,
			.vs_len = 3,
			.vs_p = 0,
		},


	[MVDU_MODE_SD_486_I_30] = {
		.width = 720,
		.height = 486,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 27000000,
		.mode = 0,
		.h_blank = 134,
		.h_active = 720,
		.h_total = 858,
		.hs_start = 19,
		.hs_len = 62,
		.hs_delay = 57,
		.v_blank_beg = 20,
		.v_active = 486,
		.v_total = 525,
		.v_blank_end = 0,
		.vs_start = 3,
		.vs_len = 3,
		.vs_p = 0,
	},

	[MVDU_MODE_SD_576_I_25] = {
		.width = 720,
		.height = 576,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 27000000,
		.mode = 1,
		.h_blank = 140,
		.h_active = 720,
		.h_total = 864,
		.hs_start = 12,
		.hs_len = 63,
		.hs_delay = 69,
		.v_blank_beg = 22,
		.v_active = 576,
		.v_total = 625,
		.v_blank_end = 2,
		.vs_start = 0,
		.vs_len = 3,
		.vs_p = 0,
	},

	[MVDU_MODE_HD_480_I_30] = {
		.width = 720,
		.height = 480,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 27027000,
		.mode = 0x10,
		.h_blank = 268,
		.h_active = 1440,
		.h_total = 1716,
		.hs_start = 38,
		.hs_len = 124,
		.hs_delay = 114,
		.v_blank_beg = 21,
		.v_active = 480,
		.v_total = 525,
		.v_blank_end = 1,
		.vs_start = 3,
		.vs_len = 3,
		.vs_p = 0,
	},

	[MVDU_MODE_HD_576_I_25] = {
		.width = 720,
		.height = 576,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 27000000,
		.mode = 0x11,
		.h_blank = 280,
		.h_active = 1440,
		.h_total = 1728,
		.hs_start = 24,
		.hs_len = 126,
		.hs_delay = 138,
		.v_blank_beg = 22,
		.v_active = 576,
		.v_total = 625,
		.v_blank_end = 2,
		.vs_start = 0,
		.vs_len = 3,
		.vs_p = 0,
	},

	[MVDU_MODE_HD_480_P_60] = {
		.width = 720,
		.height = 480,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 27027000,
		.mode = 0x12,
		.h_blank = 130,
		.h_active = 720,
		.h_total = 858,
		.hs_start = 16,
		.hs_len = 62,
		.hs_delay = 60,
		.v_blank_beg = 42,
		.v_active = 480,
		.v_total = 525,
		.v_blank_end = 3,
		.vs_start = 6,
		.vs_len = 6,
		.vs_p = 0,
	},

	[MVDU_MODE_HD_576_P_50] = {
		.width = 720,
		.height = 576,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 27000000,
		.mode = 0x13,
		.h_blank = 136,
		.h_active = 720,
		.h_total = 864,
		.hs_start = 12,
		.hs_len = 64,
		.hs_delay = 68,
		.v_blank_beg = 44,
		.v_active = 576,
		.v_total = 625,
		.v_blank_end = 5,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 0,
	},

	[MVDU_MODE_HD_720_P_60] = {
		.width = 1280,
		.height = 720,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 74250000,
		.mode = 0x16,
		.h_blank = 362,
		.h_active = 1280,
		.h_total = 1650,
		.hs_start = 110,
		.hs_len = 40,
		.hs_delay = 220,
		.v_blank_beg = 25,
		.v_active = 720,
		.v_total = 750,
		.v_blank_end = 5,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},

	[MVDU_MODE_HD_720_P_50] = {
		.width = 1280,
		.height = 720,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 74250000,
		.mode = 0x17,
		.h_blank = 692,
		.h_active = 1280,
		.h_total = 1980,
		.hs_start = 440,
		.hs_len = 40,
		.hs_delay = 220,
		.v_blank_beg = 25,
		.v_active = 720,
		.v_total = 750,
		.v_blank_end = 5,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},

	[MVDU_MODE_HD_1080_I_30] = {
		.width = 1920,
		.height = 1080,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 74250000,
		.mode = 0x18,
		.h_blank = 272,
		.h_active = 1920,
		.h_total = 2200,
		.hs_start = 88,
		.hs_len = 44,
		.hs_delay = 148,
		.v_blank_beg = 20,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank_end = 2,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},

	[MVDU_MODE_HD_1080_I_25] = {
		.width = 1920,
		.height = 1080,
		.vmode = MVDU_MODE_INTERLACED,
		.pixclock = 74250000,
		.mode = 0x19,
		.h_blank = 712,
		.h_active = 1920,
		.h_total = 2640,
		.hs_start = 528,
		.hs_len = 44,
		.hs_delay = 148,
		.v_blank_beg = 20,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank_end = 2,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},

	[MVDU_MODE_HD_1080_P_30] = {
		.width = 1920,
		.height = 1080,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 74250000,
		.mode = 0x1A,
		.h_blank = 272,
		.h_active = 1920,
		.h_total = 2200,
		.hs_start = 88,
		.hs_len = 44,
		.hs_delay = 148,
		.v_blank_beg = 41,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank_end = 4,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},

	[MVDU_MODE_HD_1080_P_25] = {
		.width = 1920,
		.height = 1080,
		.vmode = MVDU_MODE_PROGRESSIVE,
		.pixclock = 74250000,
		.mode = 0x1B,
		.h_blank = 712,
		.h_active = 1920,
		.h_total = 2640,
		.hs_start = 528,
		.hs_len = 44,
		.hs_delay = 148,
		.v_blank_beg = 41,
		.v_active = 1080,
		.v_total = 1125,
		.v_blank_end = 4,
		.vs_start = 0,
		.vs_len = 5,
		.vs_p = 1,
	},
};

/*
 * Notes on interrupt handling
 * ---------------------------
 *
 * Since VDU is very interrupt-noisy, interrupt handling is a bit tricky.
 *
 * To avoid getting unwanted interrupts, we have to change interrupt mask
 * dynamically. This opens a set of corner cases when we could get
 * just-disabled interrupt, or loose just-enabled one.
 *
 * We handle that by introducing several rules.
 *
 * Any changes to irq mask should be done with dev->lock held, and dev->lock
 * should be released only when everything is ready for updated irq mask.
 *
 * It is guaranteed that:
 *
 * - while dev->lock is held by any code on any thread or cpu, irq handler
 *   will not be called (technically, it may be called on SMP or PREEMPT_RT
 *   kernel, but in that case it will wait until dev->lock is released),
 *
 * - irq handler won't process or clear any masked status bits,
 *
 * - when dev->lock is not held, irq handler will process and clear all
 *   unmasked status bits.
 *
 * When a bit is being unmasked, it could happen that it is already pending.
 * Code may explicily clear the bit. However, there is a race between this
 * clearing and re-setting by hardware, so code should be prepared for that.
 */

#define IRQ_BIT(name) MVDU_REG_INT_VDU_STAT_##name##_MASK

static inline void __mvdu_enable_irq(struct mvdu_device *dev, u32 bit)
{
	/* dev->lock MUST be held */

	dev->irq_mask |= bit;
	write_reg(dev, dev->irq_mask, MVDU_REG_INT_VDU_ENA);
}
#define mvdu_enable_irq(dev, name) __mvdu_enable_irq(dev, IRQ_BIT(name))

static inline void __mvdu_disable_irq(struct mvdu_device *dev, u32 bit)
{
	/* dev->lock MUST be held */

	/* Have to protect against any unmaskable irq */
	write_reg(dev, dev->irq_mask & ~bit, MVDU_REG_INT_VDU_ENA);
	dev->irq_mask = read_reg(dev, MVDU_REG_INT_VDU_ENA);
}
#define mvdu_disable_irq(dev, name) __mvdu_disable_irq(dev, IRQ_BIT(name))

static inline void __mvdu_clear_irq(struct mvdu_device *dev, u32 bit)
{
	/* better to have dev->lock held */

	write_reg(dev, bit, MVDU_REG_INT_VDU_STAT);
}
#define mvdu_clear_irq(dev, name) __mvdu_clear_irq(dev, IRQ_BIT(name))

static inline void __mvdu_clear_and_enable_irq(struct mvdu_device *dev, u32 bit)
{
	/* dev->lock MUST be held */

	if (!(dev->irq_mask & bit)) {
		__mvdu_clear_irq(dev, bit);
		__mvdu_enable_irq(dev, bit);
	}
}
#define mvdu_clear_and_enable_irq(dev, name) \
		__mvdu_clear_and_enable_irq(dev, IRQ_BIT(name))


/*
 * Lots of actions could be done either when VDU is disabled, or at VBLANK
 * time.
 *
 * mvdu_disabled_or_vblanking() checks for that condition.
 *
 * mvdu_add_sa_waiter() / mvdu_remove_sa_waiter() manage SA interrupt
 * to wait for that condition.
 */

static inline int mvdu_disabled_or_vblanking(struct mvdu_device *dev)
{
	u32 val;

	if (dev->vdu_state == VDU_STATE_OFF)
		return 1;

	val = read_reg(dev, MVDU_REG_STAT_VDU_DISP);
	if (val & MVDU_REG_STAT_VDU_DISP_VBB_MASK)
		return 1;

	return 0;
}

static inline void mvdu_add_sa_waiter(struct mvdu_device *dev)
{
	if (dev->sa_waiters == 0)
		mvdu_clear_and_enable_irq(dev, SA);
	dev->sa_waiters++;
}

static inline void mvdu_remove_sa_waiter(struct mvdu_device *dev)
{
	dev->sa_waiters--;
	if (dev->sa_waiters == 0)
		mvdu_disable_irq(dev, SA);
}

static inline void mvdu_wake_sa_waiters(struct mvdu_device *dev)
{
	if (dev->sa_waiters) {
		dev->sa_id++;
		wake_up_all(&dev->sa_wq);
	}
}

/* Similar code for OSD_FR_RD_END */

static inline void mvdu_add_osd_fr_rd_end_waiter(struct mvdu_device *dev)
{
	if (dev->osd_fr_rd_end_waiters == 0)
		mvdu_clear_and_enable_irq(dev, OSD_FR_RD_END);
	dev->osd_fr_rd_end_waiters++;
}

static inline void mvdu_remove_osd_fr_rd_end_waiter(struct mvdu_device *dev)
{
	dev->osd_fr_rd_end_waiters--;
	if (dev->osd_fr_rd_end_waiters == 0)
		mvdu_disable_irq(dev, OSD_FR_RD_END);
}

static inline void mvdu_wake_osd_fr_rd_end_waiters(struct mvdu_device *dev)
{
	if (dev->osd_fr_rd_end_waiters) {
		dev->osd_fr_rd_end_id++;
		wake_up_all(&dev->osd_fr_rd_end_wq);
	}
}

/*
 * Since VDU and individual layer enable/disable is much more complex
 * than simple register writing (it contains waiting, etc), that is contolled
 * in automata style. There is current state defined, and flags that request
 * state changes.
 */

static inline void mvdu_set_vdu_state(struct mvdu_device *dev, int state)
{
	dev->vdu_state = state;
#ifdef DEBUG
	if (dev->vdu_state == VDU_STATE_ON)
		dev_dbg(dev->dev, "VDU state: ON\n");
	else if (dev->vdu_state == VDU_STATE_OFF)
		dev_dbg(dev->dev, "VDU state: OFF\n");
	else if (dev->vdu_state == VDU_STATE_STOPPING)
		dev_dbg(dev->dev, "VDU state: STOPPING\n");
#endif
}

static inline void mvdu_set_osd_state(struct mvdu_device *dev, int state)
{
	dev->osd_state = state;
#ifdef DEBUG
	if (dev->osd_state == OSD_STATE_ON)
		dev_dbg(dev->dev, "OSD state: ON\n");
	else if (dev->osd_state == OSD_STATE_STARTING)
		dev_dbg(dev->dev, "OSD state: STARTING\n");
	else if (dev->osd_state == OSD_STATE_OFF)
		dev_dbg(dev->dev, "OSD state: OFF\n");
	else if (dev->osd_state == OSD_STATE_STOPPING)
		dev_dbg(dev->dev, "OSD state: STOPPING\n");
#endif
}

static inline void mvdu_set_mvl_state(struct mvdu_device *dev, int state)
{
	dev->mvl_state = state;
	/* No prints since those break timing badly */
}


/*
 * Write current mode information to hardware
 */
static void mvdu_write_dif(struct mvdu_device *dev)
{
	struct mvdu_mode *mode;
	u32 val;

	mode = &mvdu_modes[dev->current_mode];

	val = shift_value(mode->mode, DIF_CTRL_VDU_MODE) |
		shift_value(mode->vs_p, DIF_CTRL_VSYNC_P) |
		shift_value(mode->vs_p, DIF_CTRL_HSYNC_P) |
		dev->output_flags;
	write_reg(dev, val, MVDU_REG_DIF_CTRL);

	val = shift_value((mode->h_blank - 1), DIF_BLANK_HBLANK) |
		shift_value(mode->v_blank_beg, DIF_BLANK_VBLANK_BEG) |
		shift_value(mode->v_blank_end, DIF_BLANK_VBLANK_END);
	write_reg(dev, val, MVDU_REG_DIF_BLANK);

	val = shift_value(mode->h_total, DIF_FSIZE_HTOTAL) |
		shift_value(mode->v_total, DIF_FSIZE_VTOTAL);
	write_reg(dev, val, MVDU_REG_DIF_FSIZE);

	val = shift_value((mode->h_active - 1), DIF_ASIZE_HACTIVE) |
		shift_value(mode->v_active, DIF_ASIZE_VACTIVE);
	write_reg(dev, val, MVDU_REG_DIF_ASIZE);

	val = shift_value(mode->hs_start, DIF_HSYNC_HSYNC_START) |
		shift_value(mode->hs_len, DIF_HSYNC_HSYNC_LEN) |
		shift_value(mode->hs_delay, DIF_HSYNC_HSYNC_DELAY);
	write_reg(dev, val, MVDU_REG_DIF_HSYNC);

	val = shift_value(mode->vs_start, DIF_VSYNC_VSYNC_START) |
		shift_value(mode->vs_len, DIF_VSYNC_VSYNC_LEN);
	write_reg(dev, val, MVDU_REG_DIF_VSYNC);

	dev_dbg(dev->dev, "configured hardware for %s %dx%d%s\n",
		mvdu_mode_def_str(dev->current_mode),
		mode->width, mode->height,
		mode->vmode == MVDU_MODE_PROGRESSIVE ? "p" : "i");
}

/*
 * Write background color to hardware
 */
static void mvdu_write_bgcolor(struct mvdu_device *dev)
{
	int r, g, b;
	int y, cb, cr;
	int y_r, y_g, y_b,  cb_r, cb_g, cb_b,  cr_r, cr_g, cr_b;
	u32 val;

	/*
	 * Conversion formulas for SD mode:
	 *   Y = 16 + 0.257*R + 0.504*G + 0.098*B
	 *   Cb = 128 - 0.148*R - 0.291*G + 0.439*B
	 *   Cr = 128 + 0.439*R - 0.368*G - 0.071*B
	 *
	 * Conversion formulas for HD mode:
	 *   Y = 16 + 0.183*R + 0.614*G + 0.062*B
	 *   Cb = 128 - 0.101*R - 0.338*G + 0.439*B
	 *   Cr = 128 + 0.439*R - 0.399*G - 0.040*B
	 *
	 * This assumes 8-bit RGB and YCbCr components.
	 *
	 * We will use integer ooperations, by multiplying the coefficients
	 * by 4096 - that should be enough to keep presition.
	 */

	r = dev->bgcolor.red;
	g = dev->bgcolor.green;
	b = dev->bgcolor.blue;

	if (mvdu_mode_def(dev->current_mode) == MVDU_MODE_SD) {

		y_r = 1053;	/* round(0.257 * 4096) */
		y_g = 2064;	/* round(0.504 * 4096) */
		y_b = 401;	/* round(0.098 * 4096) */

		cb_r = -606;	/* round(-0.148 * 4096) */
		cb_g = -1192;	/* round(-0.291 * 4096) */
		cb_b = 1798;	/* round(0.439 * 4096) */

		cr_r = 1798;	/* round(0.439 * 4096) */
		cr_g = -1507;	/* round(-0.368 * 4096) */
		cr_b = -291;	/* round(-0.071 * 4096) */

	} else {

		y_r = 750;	/* round(0.183 * 4096) */
		y_g = 2515;	/* round(0.614 * 4096) */
		y_b = 254;	/* round(0.062 * 4096) */

		cb_r = -414;	/* round(-0.101 * 4096) */
		cb_g = -1384;	/* round(-0.338 * 4096) */
		cb_b = 1798;	/* round(0.439 * 4096) */

		cr_r = 1798;	/* round(0.439 * 4096) */
		cr_g = -1636;	/* round(-0.399 * 4096) */
		cr_b = -164;	/* round(-0.040 * 4096) */

	}

	y = 16 + ((r * y_r) + (g * y_g) + (b * y_b) + 2048) / 4096;
	cb = 128 + ((r * cb_r) + (g * cb_g) + (b * cb_b) + 2048) / 4096;
	cr = 128 + ((r * cr_r) + (g * cr_g) + (b * cr_b) + 2048) / 4096;

	y = max(0x10, min(0xf0, y));
	cb = max(0, min(255, cb));
	cr = max(0, min(255, cr));

	val = shift_value(y, DIF_BGR_Y) | shift_value(cb, DIF_BGR_Cb) |
		shift_value(cr, DIF_BGR_Cr);
	write_reg(dev, val, MVDU_REG_DIF_BGR);

	dev_dbg(dev->dev, "background color set to "
			  "RGB #%02x%02x%02x, %s YCbCr #%02x%02x%02x\n",
		r, g, b,
		mvdu_mode_def_str(dev->current_mode),
		y, cb, cr);
}

/*
 * Write OSD color conversion weights to hardware
 */
static void mvdu_write_osd_weights(struct mvdu_device *dev)
{
	int y_r, y_g, y_b,  cb_r, cb_g, cb_b,  cr_r, cr_g, cr_b;
	u32 val;

	/* Values from VDU documentation */

	if (mvdu_mode_def(dev->current_mode) == MVDU_MODE_SD) {
#ifdef CONFIG_1888TX018
		y_r  = 0x42;  y_g  = 0x81;  y_b = 0x19;
		cb_r = 0x126; cb_g = 0x14A; cb_b = 0x70;
		cr_r = 0x70;  cr_g = 0x15E; cr_b = 0x112;
#else
		y_r  = 0x21;  y_g  = 0x41;  y_b = 0xd;
		cb_r = 0x113; cb_g = 0x125; cb_b = 0x38;
		cr_r = 0x38;  cr_g = 0x12f; cr_b = 0x109;
#endif
	} else {
#ifdef CONFIG_1888TX018
		y_r  = 0x2F;  y_g  = 0x9D;  y_b = 0x10;
		cb_r = 0x11a; cb_g = 0x157; cb_b = 0x70;
		cr_r = 0x70;  cr_g = 0x166; cr_b = 0x10A;
#else
		y_r  = 0x17;  y_g  = 0x4f;  y_b = 0x8;
		cb_r = 0x10d; cb_g = 0x12b; cb_b = 0x38;
		cr_r = 0x38;  cr_g = 0x133; cr_b = 0x105;
#endif
	}

	val = shift_value(y_r, OSD_COLOR_Y_R) |
		shift_value(y_g, OSD_COLOR_Y_G) |
		shift_value(y_b, OSD_COLOR_Y_B);
	write_reg(dev, val, MVDU_REG_OSD_COLOR_Y);

	val = shift_value(cb_r, OSD_COLOR_Cb_R) |
		shift_value(cb_g, OSD_COLOR_Cb_G) |
		shift_value(cb_b, OSD_COLOR_Cb_B);
	write_reg(dev, val, MVDU_REG_OSD_COLOR_Cb);

	val = shift_value(cr_r, OSD_COLOR_Cr_R) |
		shift_value(cr_g, OSD_COLOR_Cr_G) |
		shift_value(cr_b, OSD_COLOR_Cr_B);
	write_reg(dev, val, MVDU_REG_OSD_COLOR_Cr);

	dev_dbg(dev->dev, "set OSD color conversion for %s mode\n",
		mvdu_mode_def_str(dev->current_mode));
}

/*
 * Write OSD headers locations to hardware
 */
static inline void mvdu_write_osd_headers_dma(struct mvdu_device *dev)
{
	if (dev->osd_reg0_is_odd) {
		write_reg(dev, dev->osd_odd_headers_dma, MVDU_REG_OSD_BASEreg0);
		write_reg(dev, dev->osd_even_headers_dma, MVDU_REG_OSD_BASEreg1);
	} else {
		write_reg(dev, dev->osd_even_headers_dma, MVDU_REG_OSD_BASEreg0);
		write_reg(dev, dev->osd_odd_headers_dma, MVDU_REG_OSD_BASEreg1);
	}
}

/*
 * Do proper OSD header processing before OSD enable
 */
static void mvdu_process_osd_headers_dma(struct mvdu_device *dev)
{
	/* Ensure that VDU will generate next OSD (sub)frame using odd headers */
	dev->osd_reg0_is_odd = read_reg(dev, MVDU_REG_STAT_VDU_AVMP) &
					MVDU_REG_STAT_VDU_AVMP_OSD_AVMP_MASK;

	/* Do write the headers */
	mvdu_write_osd_headers_dma(dev);

	/* Hardware will use dev->osd_cur_group, regardless of any future
	   AVMP reads */
	dev->osd_last_change_avmp = -1;
}

/*
 * VDU enable/disable subsystem
 * ----------------------------
 *
 * We try to keep VDU in ON state when it is enabled at external API level,
 * but also support temporary disable and re-enable for runtime mode switching
 * and for error recovery
 */

static inline int vdu_ready(struct mvdu_device *dev)
{
	return dev->vdu_ena_req && !dev->vdu_restart_req &&
	       !dev->vdu_delayed_restart_req;
}

#ifdef CONFIG_1888TX018
int rcm_setup_vmode(struct mvdu_device *dev)
{
	int ret;
	unsigned data;
	bool locked;
	unsigned long timeout;

	const struct mvdu_mode *mode = mvdu_get_modeinfo(dev, dev->current_mode);
	unsigned hz = mode->pixclock;
	unsigned divmode = (594000000 + hz / 2) / hz - 1;

	// unlock the PLL controller if it is locked
	ret = regmap_read(dev->pll_control, CRG_VDU_WR_LOCK, &data);
	if (ret != 0)
		return ret;
	locked = (data == CRG_VDU_WR_LOCK_LOCKED);
	if (locked) {
		ret = regmap_write(dev->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_UNLOCK);
		if (ret != 0)
			return ret;
	}

	// set the divider
	ret = regmap_write(dev->pll_control,
		dev->vdu_index == 0 ? CRG_VDU_CKDIVMODE_VOUT0 : CRG_VDU_CKDIVMODE_VOUT1,
		divmode);
	if (ret != 0)
		return ret;
	ret = regmap_write(dev->pll_control, CRG_VDU_UPD_CK, CRG_VDU_UPD_CK_UPD_CKDIV);
	if (ret != 0)
		return ret;

	// wait until the PLL controller is ready
	timeout = jiffies + HZ / 10;
	while (true) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		ret = regmap_read(dev->pll_control, CRG_VDU_UPD_CK, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_UPD_CK_UPD_CKDIV) == 0)
			break;
	}

	// lock the PLL controller if needed
	if (locked) {
		ret = regmap_write(dev->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_LOCK);
		if (ret != 0)
			return ret;
	}

	return 0;
}
#else

int rcm_setup_vmode(struct mvdu_device *dev);

#endif // CONFIG_1888TX018

/*
 * Perform possible pending VDU_STATE_OFF -> VDU_STATE_ON and
 * VDU_STATE_ON -> VDU_STATE_STOPPING transitions.
 *
 * dev->lock MUST be held.
 */

static void mvdu_update_vdu_state(struct mvdu_device *dev)
{
	u32 control;

	if (dev->vdu_state == VDU_STATE_OFF) {

		if (vdu_ready(dev)) {

			/* Always write mode before start - it is either not
			 * yet written, or broken by reset at previous stop */
			mvdu_write_dif(dev);

			/* Write background color if pending */
			if (dev->write_bgcolor_on_ena) {
				mvdu_write_bgcolor(dev);
				dev->write_bgcolor_on_ena = 0;
			}

			/* Write OSD color weights if pending */
			if (dev->write_osd_weights_on_ena) {
				mvdu_write_osd_weights(dev);
				dev->write_osd_weights_on_ena = 0;
			}

			/* Start checking for underflow and syserr */
			mvdu_clear_and_enable_irq(dev, O_FIFO_EMPTY);
			mvdu_clear_and_enable_irq(dev, INT_SYS_ERR);

			/* Enable hardware */
			control = MVDU_REG_CTRL_VDU_ENA_VDU_ENA_MASK;
			if (dev->osd_state == OSD_STATE_ON) {
				mvdu_process_osd_headers_dma(dev);
				control |= (MVDU_REG_CTRL_VDU_ENA_OSD_ENA_MASK |
					    MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK);
			}
			if (dev->mvl_state == MVL_STATE_READY) {
				control |= (MVDU_REG_CTRL_VDU_ENA_MVL_ENA_MASK |
					    MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK);
			}
			write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

			/* In in MVL_STATE_READY, enter MVL_STATE_RUNNING */
			if (dev->mvl_state == MVL_STATE_READY) {
				mvdu_clear_and_enable_irq(dev, MVL_FR_RD_END);
				mvdu_set_mvl_state(dev, MVL_STATE_RUNNING);
			}

			/* State transition complete */
			mvdu_set_vdu_state(dev, VDU_STATE_ON);

			/* Notify low-level */
			dev_info(dev->dev, "about to call setup_vmode");
			(void) rcm_setup_vmode(dev);
		}

	} else if (dev->vdu_state == VDU_STATE_ON) {

		if (!vdu_ready(dev)) {

			mod_timer(&dev->restart_watchdog_timer, jiffies + HZ);

			/* No longer check for underflow */
			mvdu_disable_irq(dev, O_FIFO_EMPTY);

			/* Expect stop complete interrupt */
			mvdu_clear_and_enable_irq(dev, VDU_OFF);

			/* Clear the VDU_ENA bit.
			 *
			 * Also clear SW_ENA here to disallow start of next
			 * frame prefetching. Prefetching is not helpful since
			 * we will reset anyway, and may be harmful because
			 * frame parameters may change while stopping.
			 */
			control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
			control &= ~(MVDU_REG_CTRL_VDU_ENA_VDU_ENA_MASK |
				     MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK |
				     MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK);
			write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

			/* Update OSD/MVL states */

			switch (dev->osd_state) {
			case OSD_STATE_STARTING:
				mvdu_set_osd_state(dev, OSD_STATE_ON);
				mvdu_remove_sa_waiter(dev);
				break;
			case OSD_STATE_STOPPING:
				mvdu_disable_irq(dev, FR_PRC_END);
				mvdu_set_osd_state(dev, OSD_STATE_OFF);
				break;
			}

			switch (dev->mvl_state) {
			case MVL_STATE_READY:
				mvdu_remove_sa_waiter(dev);
				break;
			case MVL_STATE_RUNNING:
				mvdu_disable_irq(dev, MVL_FR_RD_END);
				mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_O);
				break;
			case MVL_STATE_STOPPING_S:
				mvdu_remove_sa_waiter(dev);
				break;
			case MVL_STATE_STOPPING_M:
				mvdu_remove_sa_waiter(dev);
				mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_O);
				break;
			}

			/* State transition complete */
			mvdu_set_vdu_state(dev, VDU_STATE_STOPPING);
		}
	}
}

/*
 * Handle VDU_OFF interrupt.
 *
 * Perform VDU_STATE_STOPPING -> VDU_STATE_OFF transition (that includes VDU
 * soft-reset and partial restore of hardware settings), then call
 * mvdu_update_vdu_state() to perform possibly pending VDU_STATE_OFF ->
 * VDU_STATE_ON transition.
 *
 * Called from interrupt handler, dev->lock MUST be held.
 */
static void mvdu_mvl_vdu_stop_complete(struct mvdu_device *dev);
static void mvdu_handle_off(struct mvdu_device *dev)
{
	int tries;

	BUG_ON(dev->vdu_state != VDU_STATE_STOPPING);

	del_timer_sync(&dev->restart_watchdog_timer);

	/* Soft-reset VDU */
	write_reg(dev, 1, MVDU_REG_CTRL_SOFT_RESET);
	for (tries = 100; tries > 0; tries--) {
		if ((read_reg(dev, MVDU_REG_CTRL_SOFT_RESET) & 1) == 0)
			break;
	}
	if (tries == 0) {
		/* This should not happen. Hardware likely misbehaves. */
		dev_err(dev->dev,
			"timeout waiting for VDU reset to complete\n");
		/* Let's try to continue, although nobody knows ... */
	}

#ifdef CONFIG_1888TX018
	// without this code a repeated fb device open does not work
	dev->write_bgcolor_on_ena = 1;
	dev->write_osd_weights_on_ena = 1;
#endif

	/* Clear all interrupts and restore irq mask after reset */
	write_reg(dev, ~0, MVDU_REG_INT_VDU_STAT);
	write_reg(dev, dev->irq_mask, MVDU_REG_INT_VDU_ENA);

	/* Mask VDU_OFF and INT_SYS_ERR (VDU is now disabled) */
	mvdu_disable_irq(dev, VDU_OFF);
	mvdu_disable_irq(dev, INT_SYS_ERR);

	/* Restore AXI registers after reset */
	if (dev->axi_mvl_param)
		write_reg(dev, dev->axi_mvl_param, MVDU_REG_MI_AXI_MVL_PARAM);
	if (dev->axi_osd_param)
		write_reg(dev, dev->axi_osd_param, MVDU_REG_MI_AXI_OSD_PARAM);

	/* Restore MI_CTRL and SCALER_CTRL */
	write_reg(dev, dev->mi_ctrl_saved, MVDU_REG_MI_CTRL);
	write_reg(dev, dev->scaler_ctrl_saved, MVDU_REG_SCALER_CTRL);

	/* Initialize MVDU_REG_CTRL_VDU_ENA with zero - more on entering _ON */
	write_reg(dev, 0, MVDU_REG_CTRL_VDU_ENA);

	/* State transition complete */
	mvdu_set_vdu_state(dev, VDU_STATE_OFF);

	/* Notify MVL about stop complete */
	mvdu_mvl_vdu_stop_complete(dev);

	/* VDU is now off, so wake up those waiting for "off or vblanking" */
	mvdu_wake_sa_waiters(dev);

	/* "Stop part of restart" complete */
	if (dev->vdu_restart_req)
		dev->vdu_restart_req = 0;

	/* Notify low-level */
	if (dev->notify_stopped)
		dev->notify_stopped(dev);

	/* Perform possibly pending transition to VDU_STATE_ON */
	mvdu_update_vdu_state(dev);
}

/*
 * Handle underflow interrupt by triggering VDU restart.
 *
 * Called from interrupt handler, dev->lock MUST be held
 */
static void mvdu_handle_underflow(struct mvdu_device *dev)
{
// Don't screw us up
	/*
	dev_err(dev->dev,
		"underflow interrupt (FIFO status 0x%08x), restarting VDU ...\n",
		read_reg(dev, MVDU_REG_STAT_VDU_FIFO));
	*/
	/* Trigger restart */
	dev->vdu_restart_req = 1;
	mvdu_update_vdu_state(dev);
}

/*
 * Handle system error interrupt by triggering delayed VDU restart.
 *
 * Called from interrupt handler, dev->lock MUST be held
 */
static void mvdu_handle_system_error(struct mvdu_device *dev)
{
	dev_err(dev->dev, "system error interrupt, disabling VDU\n");

	/* Mask system error interrupt immediately */
	mvdu_disable_irq(dev, INT_SYS_ERR);

	/* Trigger delayed restart */
	dev->vdu_delayed_restart_req = 1;
	mvdu_update_vdu_state(dev);

	mod_timer(&dev->delayed_restart_timer, jiffies + 3 * HZ);
}

static void mvdu_delayed_restart_timeout(struct timer_list *t)
{
	struct mvdu_device *dev = from_timer(dev, t, delayed_restart_timer);

	dev_err(dev->dev, "trying to recover after system error\n");

	/* Complete delayed restart */
	dev->vdu_delayed_restart_req = 0;
	mvdu_update_vdu_state(dev);
}

static void mvdu_restart_watchdog_timeout(struct timer_list *t)
{
	struct mvdu_device *dev = from_timer(dev, t, restart_watchdog_timer);
	u32 control;

	dev_err(dev->dev, "restart watchdog timeout, disabling OSD and MVL\n");

	control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	control &= ~(MVDU_REG_CTRL_VDU_ENA_OSD_ENA_MASK |
		     MVDU_REG_CTRL_VDU_ENA_MVL_ENA_MASK);
	write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);
}

int mvdu_enable_vdu(struct mvdu_device *dev)
{
	unsigned long flags;
dev_info(dev->dev, "enable vdu\n");
	spin_lock_irqsave(&dev->lock, flags);
	if (!dev->enable_counter) {
		dev->vdu_ena_req = 1;
		mvdu_update_vdu_state(dev);
	}
	dev->enable_counter++;
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
EXPORT_SYMBOL(mvdu_enable_vdu);

int mvdu_disable_vdu(struct mvdu_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->enable_counter) {
		dev->enable_counter--;
		if (dev->enable_counter == 0) {
			dev->vdu_ena_req = 0;
			mvdu_update_vdu_state(dev);
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
EXPORT_SYMBOL(mvdu_disable_vdu);


/*
 * Video mode changing
 * -------------------
 *
 * There are two interfaces affecting video mode:
 * - mvdu_set_mode()
 * - output mode for video streaming
 * The second of these two has a priority. If streaming is on, any requests
 * to change video mode return -EBUSY and don't change anything.
 *
 * Rationale: if software controlling OSD and MVL don't agree on the current
 * video mode, it is issue in that software and should be fixed there.
 * Supporting their conflicting desicions at driver level increases complexity
 * and may cause ugly flicker on MVL/OSD enable/disable, espicially in
 * temporary-disable scenario.
 */

static void mvdu_osd_adjust_areas(struct mvdu_device *dev,
		int old_mode, int new_mode);

/*
 * Actual mode changer.
 *
 * Called with lock held, with new mode already checked to be valid, supported,
 * and different from the current mode.
 */
static void mvdu_apply_mode(struct mvdu_device *dev, int new_mode)
{
	int old_mode;

	old_mode = dev->current_mode;
	dev->current_mode = new_mode;

	if (mvdu_mode_def(old_mode) != mvdu_mode_def(new_mode)) {
		dev->write_bgcolor_on_ena = 1;
		dev->write_osd_weights_on_ena = 1;
	}

	if (dev->vdu_state == VDU_STATE_ON) {
		/* Trigger restart into new mode */
		dev->vdu_restart_req = 1;
		mvdu_update_vdu_state(dev);
	}

	/* Now VDU definitly won't load new OSD page before restart,
	 * so it is safe to adjust areas.
	 *
	 * However, adjust MUST be completed before VDU is restarted
	 * in new mode, so we adjust before unlocking, so VDU_OFF won't
	 * be caught and restart won't be triggered. This is ensured by
	 * still holding the lock.
	 */
	mvdu_osd_adjust_areas(dev, old_mode, new_mode);

	/* Notify _fb about mode change so it could adjust it's data */
	if (dev->mode_notifier)
		dev->mode_notifier(dev);
}

/*
 * External mode-changing API.
 *
 * Has lower priority than video mode setting by video streaming code.
 */
int mvdu_set_mode(struct mvdu_device *dev, int new_mode,
					int call_notifier_on_busy)
{
	int old_mode = dev->current_mode;
	int ret = 0;
	unsigned long flags;

	if (new_mode == MVDU_MODE_DEFAULT)
		new_mode = dev->default_mode;

	if (!MVDU_MODE_VALID(new_mode))
		return -EINVAL;
	if ((dev->supported_modes & (1 << new_mode)) == 0)
		return -ENOTSUPP;

	spin_lock_irqsave(&dev->lock, flags);

	if (new_mode == old_mode)
		goto out;

	if (dev->mvl_state != MVL_STATE_OFF &&
	    dev->mvl_state != MVL_STATE_STOPPING_S) {
		ret = -EBUSY;
		if (call_notifier_on_busy && dev->mode_notifier)
			dev->mode_notifier(dev);
		goto out;
	}

	mvdu_apply_mode(dev, new_mode);

out:
	spin_unlock_irqrestore(&dev->lock, flags);
	return ret;

}
EXPORT_SYMBOL(mvdu_set_mode);

const struct mvdu_mode *mvdu_get_modeinfo(struct mvdu_device *dev, int mode_num)
{
	if (mode_num == MVDU_MODE_DEFAULT)
		mode_num = dev->default_mode;

	if (!MVDU_MODE_VALID(mode_num))
		return 0;
	if ((dev->supported_modes & (1 << mode_num)) == 0)
		return 0;

	return &mvdu_modes[mode_num];
}
EXPORT_SYMBOL(mvdu_get_modeinfo);


/*
 * Changing background color
 * -------------------------
 *
 * If VDU is enabled, changing has to be delayed until vblanking
 */

static void mvdu_do_set_bgcolor(struct mvdu_device *dev,
					const struct rgb888 *color)
{
	if (dev->bgcolor.red == color->red &&
	    dev->bgcolor.green == color->green &&
	    dev->bgcolor.blue == color->blue)
		return;

	dev->bgcolor = *color;

	if (dev->vdu_state == VDU_STATE_OFF) {
		mvdu_write_bgcolor(dev);
	} else if (dev->vdu_state == VDU_STATE_ON) {
		dev->bgcolor_dirty = 1;
		mvdu_add_sa_waiter(dev);
	} else if (dev->vdu_state == VDU_STATE_STOPPING) {
		dev->write_bgcolor_on_ena = 1;
	}
}

int mvdu_set_bgcolor(struct mvdu_device *dev, const struct rgb888 *color)
{
	unsigned long flags;

	/* Change color only with dev->lock held to avoid racing against
	   writing background to hardware (from within interrupt, etc) */
	spin_lock_irqsave(&dev->lock, flags);
	mvdu_do_set_bgcolor(dev, color);
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}
EXPORT_SYMBOL(mvdu_set_bgcolor);

/*
 * Read current background color.
 *
 * There is a race: bgcolor may be changed by another thread at the very
 * same time. By locking, we avoid return of partially-overwritten
 * struct rgb888. However, returned data may become obsolete before caller
 * will get it.
 */

int mvdu_get_bgcolor(struct mvdu_device *dev, struct rgb888 *color)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	*color = dev->bgcolor;
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}
EXPORT_SYMBOL(mvdu_get_bgcolor);


/*
 * Wait until nearest vblanking or off.
 *
 * Note that calling thread may be preempted at any time, so there is
 * no guarantee that VDU will still be off or vblanking when this call
 * returns.
 *
 * However it is safe to use as "wait until current frame completes".
 */

int mvdu_wait_notactive(struct mvdu_device *dev)
{
	int sa_id, res;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	sa_id = dev->sa_id;
	mvdu_add_sa_waiter(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	res = wait_event_interruptible_timeout(dev->sa_wq,
					(sa_id != dev->sa_id), HZ / 5);

	spin_lock_irqsave(&dev->lock, flags);
	mvdu_remove_sa_waiter(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (res > 0)
		return 0;
	else if (res == 0)
		return -ETIMEDOUT;
	else
		return res;
}
EXPORT_SYMBOL(mvdu_wait_notactive);

/*
 * Wait until nearest OSD (sub)frame read end
 */

int mvdu_wait_osd_fr_rd_end(struct mvdu_device *dev)
{
	int osd_fr_rd_end_id, res;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	osd_fr_rd_end_id = dev->osd_fr_rd_end_id;
	mvdu_add_osd_fr_rd_end_waiter(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	res = wait_event_interruptible_timeout(dev->osd_fr_rd_end_wq,
			(osd_fr_rd_end_id != dev->osd_fr_rd_end_id), HZ / 5);

	spin_lock_irqsave(&dev->lock, flags);
	mvdu_remove_osd_fr_rd_end_waiter(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (res > 0)
		return 0;
	else if (res == 0)
		return -ETIMEDOUT;
	else
		return res;
}
EXPORT_SYMBOL(mvdu_wait_osd_fr_rd_end);

/*
 * Read current sync values.
 *
 * Note that this is racy and that is unavoidable because of the nature
 * of linux: thread that calls this may be preempted just after return,
 * and at resume time sync state will likely be different. Even if not
 * prrempted, sync state may just change in a moment anter it was read.
 *
 * So this routine has "best effort" semantics.
 */

void mvdu_get_blanking(struct mvdu_device *dev, int *hblanking, int *vblanking)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->vdu_state == VDU_STATE_OFF) {
		reg = read_reg(dev, MVDU_REG_STAT_VDU_DISP);
		*hblanking = (reg & MVDU_REG_STAT_VDU_DISP_HBB_MASK) ? 1 : 0;
		*vblanking = (reg & MVDU_REG_STAT_VDU_DISP_VBB_MASK) ? 1 : 0;
	} else {
		*hblanking = 1;
		*vblanking = 1;
	}

	spin_unlock_irqrestore(&dev->lock, flags);
}
EXPORT_SYMBOL(mvdu_get_blanking);


/*
 * OSD enable/disable subsystem
 * ----------------------------
 *
 * OSD is ready when (1) we have osd enable request, (2) one or more
 * OSD areas are defined.
 *
 * Transitions are: OFF -> STARTING -> ON -> STOPPING. Transition from
 * STARTING to OFF is also possible if OSD becomes no longer ready. Other
 * transitions are not possible.
 */

static inline int osd_ready(struct mvdu_device *dev)
{
	return dev->osd_ena_req && dev->osd_areas_ready;
}

static void mvdu_update_osd_state(struct mvdu_device *dev)
{
	u32 control;

	if (dev->osd_state == OSD_STATE_OFF && osd_ready(dev)) {

		if (dev->vdu_state == VDU_STATE_ON) {

			/* Will start without restarting VDU.
			 *
			 * Enter OSD_STATE_STARTING and wait for SA.
			 */
			mvdu_set_osd_state(dev, OSD_STATE_STARTING);
			mvdu_add_sa_waiter(dev);

		} else {

			/* VDU is either not started at all, or is restarting.
			 *
			 * Enter OSD_STATE_ON immediately.
			 * Actual enable of OSD hardware will happen at
			 * VDU start time.
			 */
			mvdu_set_osd_state(dev, OSD_STATE_ON);
		}

	} else if (dev->osd_state == OSD_STATE_STARTING && !osd_ready(dev)) {

		mvdu_set_osd_state(dev, OSD_STATE_OFF);
		mvdu_remove_sa_waiter(dev);

	} else if (dev->osd_state == OSD_STATE_ON && !osd_ready(dev)) {

		if (dev->vdu_state == VDU_STATE_ON) {

			/* Need "soft stop" of OSD to avoid garbage left
			   in hardware OSD pipelines */
			mvdu_set_osd_state(dev, OSD_STATE_STOPPING);

			/* Disable SW_ENA immediately */
			control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
			control &= ~MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK;
			write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

			/* Now we should wait for OSD to finish reading and
			 * processing current frame. Do do so, we need to enable
			 * FR_PRC_END interrupt, and before that we have to
			 * clear it (since it is set by previous frames).
			 *
			 * However there is a race here: FR_PRC_END could happen
			 * between above clear of SW_ENA and ongoing FR_PRC_END
			 * clear. If this happens, we won't disable OSD in time,
			 * and will get underflow.
			 *
			 * Underflow is unpleasant, but detectable and
			 * recoverable. That is better than clearing FR_PRC_END
			 * before clearing SW_ENA - in that case race is still
			 * possible, and will cause early OSD disable, that will
			 * lead to garbage left in VDU OSD pipelenes, and futher
			 * bad OSD misbehaviour.
			 */
			mvdu_clear_and_enable_irq(dev, FR_PRC_END);

		} else {

			/* VDU is not running.
			 *
			 * If VDU is being stopped, it will disable and reset
			 * OSD hardware on stop complete.
			 * If VDU is already stopped, OSD hardware is disabled.
			 *
			 * So just enter OSD_STATE_OFF.
			 */
			mvdu_set_osd_state(dev, OSD_STATE_OFF);
		}
	}
}

static void mvdu_update_osd_state_to_running(struct mvdu_device *dev)
{
	u32 control;

	/* We should get here only if VDU is still running, and soft OSD
	 * startup is in progress.
	 */
	BUG_ON(dev->vdu_state != VDU_STATE_ON);
	BUG_ON(dev->osd_state != OSD_STATE_STARTING);

	/* Enable OSD */

	mvdu_process_osd_headers_dma(dev);

	control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	control |= MVDU_REG_CTRL_VDU_ENA_OSD_ENA_MASK |
		   MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK;
	write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

	/* Now in OSD_STATE_ON */
	mvdu_set_osd_state(dev, OSD_STATE_ON);
}

static void mvdu_update_osd_state_to_off(struct mvdu_device *dev)
{
	u32 control;

	/* We should get here only if VDU is still running, and soft OSD
	 * stop is in progress.
	 */
	BUG_ON(dev->vdu_state != VDU_STATE_ON);
	BUG_ON(dev->osd_state != OSD_STATE_STOPPING);

	/* Disable OSD completely */
	control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	control &= ~(MVDU_REG_CTRL_VDU_ENA_OSD_ENA_MASK |
		    MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK);
	write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

	/* Now in OSD_STATE_OFF */
	mvdu_disable_irq(dev, FR_PRC_END);
	mvdu_set_osd_state(dev, OSD_STATE_OFF);

	/* Maybe conditions changed since disable process started */
	mvdu_update_osd_state(dev);
}

int mvdu_enable_osd(struct mvdu_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dev->osd_ena_req = 1;
	mvdu_update_osd_state(dev);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
EXPORT_SYMBOL(mvdu_enable_osd);

int mvdu_disable_osd(struct mvdu_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dev->osd_ena_req = 0;
	mvdu_update_osd_state(dev);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
EXPORT_SYMBOL(mvdu_disable_osd);


/*
 * OSD header groups processing.
 *
 * Current OSD area description is stored in dev->osd_areas. Areas are stored
 * already sorted by screen order.
 *
 * Whenever hardware-visible OSD headers could change,
 * mvdu_update_osd_headers() is called. That routine picks currently-unused
 * header group, and calls mvdu_fill_osd_header_group() to fill this group
 * with proper data based on current dev->osd_areas. Then hardware is pointed
 * to the new group, and newly-unused group is detected, in race-free manner.
 */

/* We need 3 independent data sets for safe updates, and 2 groups in each set
 * to get separate headers for odd and even frames */
#define MVDU_OSD_HEADER_GROUPS		6

#define PROGRESSIVE_GROUP(n)	n
#define EVEN_GROUP(n)		n
#define ODD_GROUP(n)		(n + 3)

#define OSD_HEADER_PROGRESSIVE	0
#define OSD_HEADER_EVEN		1
#define OSD_HEADER_ODD		2

#define MVDU_OSD_HEADERS_ALLOC_SIZE(dev) \
		(sizeof(struct osd_area_header) * \
		 dev->osd_areas_max * MVDU_OSD_HEADER_GROUPS)

static inline struct osd_area_header *mvdu_osd_header(struct mvdu_device *dev,
		int group, int in_group)
{
	return &dev->osd_headers[group * dev->osd_areas_max + in_group];
}

static inline dma_addr_t mvdu_osd_header_dma(struct mvdu_device *dev,
		int group, int in_group)
{
	return dev->osd_headers_dma + sizeof(struct osd_area_header) *
		(group * dev->osd_areas_max + in_group);
}

static void mvdu_fill_osd_header(struct mvdu_device *dev, int header_type,
		struct osd_area_header *header, struct mvdu_osd_area *data,
		dma_addr_t next_header_dma)
{
	unsigned long address;
	unsigned short line, linestep, height;

	/* First compute address of the area corner in the framebuffer */
	address = data->buffer_pa;
#define START_AT_NEXT_LINE \
	address += (MVDU_OSD_BYTESPERPIXEL * data->linestep)

	if (header_type == OSD_HEADER_PROGRESSIVE) {
		/* Progressive frame */
		linestep = data->linestep;
		line = data->y;
		height = data->h;
	} else {
		/* Interleaved frame */
		linestep = 2 * data->linestep;
		if ((data->y & 1) == 0 && (data->h & 1) == 0) {
			/* Even (2k) data->y, even (2k) data->h */
			if (header_type == OSD_HEADER_EVEN) {
				line = data->y / 2;
				height = data->h / 2;
			} else {
				line = data->y / 2;
				height = data->h / 2;
				START_AT_NEXT_LINE;
			}
		} else if ((data->y & 1) == 1 && (data->h & 1) == 0) {
			/* Odd (2k+1) data->y, even (2k) data->h */
			if (header_type == OSD_HEADER_EVEN) {
				line = (data->y + 1) / 2;
				height = data->h / 2;
				START_AT_NEXT_LINE;
			} else {
				line = data->y / 2;
				height = data->h / 2;
			}
		} else if ((data->y & 1) == 0 && (data->h & 1) == 1) {
			/* Even (2k) data->y, odd (2k+1) data->h */
			if (header_type == OSD_HEADER_EVEN) {
				line = data->y / 2;
				height = (data->h + 1) / 2;
			} else {
				line = data->y / 2;
				height = data->h / 2;
				START_AT_NEXT_LINE;
			}
		} else {
			/* Odd (2k+1) data->y, odd (2k+1) data->h */
			if (header_type == OSD_HEADER_EVEN) {
				line = (data->y + 1) / 2;
				height = data->h / 2;
				START_AT_NEXT_LINE;
			} else {
				line = data->y / 2;
				height = (data->h + 1) / 2;
			}
		}
	}

	header->osd_next = cpu_to_le32(next_header_dma);
	header->osd_base = cpu_to_le32(address);
	header->osd_hor_size = cpu_to_le16(data->w - 1);
	header->osd_ver_size = cpu_to_le16(height - 1);
	header->osd_hor_start = cpu_to_le16(data->x);
	header->osd_ver_start = cpu_to_le16(line);

	header->full_line_size = cpu_to_le16(linestep);
	header->df = data->color_mode;

#ifdef CONFIG_1888TX018
	header->alpha = data->alpha | (data->alpha << 4);
#else
	header->alpha = data->alpha;
#endif
}

static void mvdu_fill_osd_header_group(struct mvdu_device *dev,
						int header_type, int group)
{
	struct osd_area_header *header;
	struct mvdu_osd_area *data;
	dma_addr_t next_header_dma;
	int i;

	next_header_dma = (dma_addr_t) 0;

	for (i = dev->osd_areas_nr - 1; i >= 0; i--) {
		header = mvdu_osd_header(dev, group, i);
		data = &dev->osd_areas[i];
		mvdu_fill_osd_header(dev, header_type, header,
							data, next_header_dma);
		next_header_dma = mvdu_osd_header_dma(dev, group, i);
	}

	wmb();
}

static void mvdu_update_osd_headers(struct mvdu_device *dev)
{
	u32 val;
	int new_group;

	if (!dev->osd_areas_nr) {
		dev->osd_areas_ready = 0;
		goto out;
	}
	dev->osd_areas_ready = 1;

	new_group = dev->osd_free_group;
	if (mvdu_modes[dev->current_mode].vmode == MVDU_MODE_INTERLACED) {
		mvdu_fill_osd_header_group(dev, OSD_HEADER_EVEN,
				EVEN_GROUP(new_group));
		dev->osd_even_headers_dma = mvdu_osd_header_dma(dev,
				EVEN_GROUP(new_group), 0);
		mvdu_fill_osd_header_group(dev, OSD_HEADER_ODD,
				ODD_GROUP(new_group));
		dev->osd_odd_headers_dma = mvdu_osd_header_dma(dev,
				ODD_GROUP(new_group), 0);
	} else {
		mvdu_fill_osd_header_group(dev, OSD_HEADER_PROGRESSIVE,
				PROGRESSIVE_GROUP(new_group));
		dev->osd_even_headers_dma = dev->osd_odd_headers_dma =
			mvdu_osd_header_dma(dev,
				PROGRESSIVE_GROUP(new_group), 0);
	}

	/* If OSD is not active, we don't need to write DMA addresses now,
	 * those will be written at OSD enable time from within
	 * mvdu_process_osd_headers_dma().
	 */
	if (dev->vdu_state != VDU_STATE_ON || dev->osd_state != OSD_STATE_ON) {
		dev->osd_cur_group = new_group;
		goto out;
	}

	/* OSD is working NOW, so race-free update is needed */

	/* Disallow page switching */

	val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	val &= ~MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK;
	write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);

	/*
	 * If at least one OSD_FR_RD_END arrived since last update,
	 * then hardware is using dev->osd_cur_group.
	 *
	 * If not, check OSD_AVMP. If it is the same as at last update time,
	 * then hardware is still using dev->osd_prev_group. Otherwise,
	 * hardware is using dev->osd_cur_group.
	 */

	val = read_reg(dev, MVDU_REG_STAT_VDU_AVMP)
			& MVDU_REG_STAT_VDU_AVMP_OSD_AVMP_MASK;

	if (dev->osd_last_change_avmp == val)
		dev->osd_free_group = dev->osd_cur_group;
	else {	/* either -1, or ~val */
		dev->osd_free_group = dev->osd_prev_group;
		dev->osd_prev_group = dev->osd_cur_group;
		dev->osd_last_change_avmp = val;
		mvdu_add_osd_fr_rd_end_waiter(dev);
	}

	dev->osd_cur_group = new_group;
	mvdu_write_osd_headers_dma(dev);

	/* Enable SW_ENA - after both clear and new group write */
	val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	val |= MVDU_REG_CTRL_VDU_ENA_OSD_BASE_SW_ENA_MASK;
	write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);

out:
	mvdu_update_osd_state(dev);
}


#ifdef DEBUG
static void mvdu_dump_areas(struct mvdu_device *dev)
{
	int n = dev->osd_areas_nr;
	struct mvdu_osd_area *a = dev->osd_areas;
	int i;

	dev_dbg(dev->dev, "%d OSD area(s)", n);

	for (i = 0; i < n; i++) {
		printk("%s %dx%d+%d+%d alpha=%d color_mode=%s",
			i == 0 ? ":" : ",",
			a[i].w, a[i].h, a[i].x, a[i].y, a[i].alpha,
			mvdu_color_mode_str(a[i].color_mode));
	}

	printk("\n");
}
#endif

static void mvdu_osd_adjust_areas(struct mvdu_device *dev,
		int old_mode, int new_mode)
{
	int i, j;
	struct mvdu_osd_area *p, *q;
	int old_w, old_h, new_w, new_h;

	old_w = mvdu_modes[old_mode].width;
	old_h = mvdu_modes[old_mode].height;
	new_w = mvdu_modes[new_mode].width;
	new_h = mvdu_modes[new_mode].height;

	p = q = dev->osd_areas;
	i = j = 0;
	while (i < dev->osd_areas_nr) {

		/* Discard areas that are now out of screen */
		if (p->x >= new_w || p->y >= new_h) {
			i++;
			p++;
			continue;
		}

		/* Truncate areas that are now partually offscreen */
		if (p->x + p->w > new_w)
			p->w = new_w - p->x;
		if (p->y + p->h > new_h)
			p->h = new_h - p->y;

		if (p != q)
			*q = *p;

		i++; j++;
		p++; q++;
	}

	dev->osd_areas_nr = j;

	mvdu_update_osd_headers(dev);

#ifdef DEBUG
	mvdu_dump_areas(dev);
#endif
}

static inline int mvdu_osd_area_ok(struct mvdu_device *dev,
	       const struct mvdu_osd_area *area)
{
	struct mvdu_mode *mode = &mvdu_modes[dev->current_mode];

	if (area->x < 0 || area->x >= mode->width ||
	    area->y < 0 || area->y >= mode->height ||
	    area->w <= 0 || area->x + area->w > mode->width ||
	    area->h <= 1 || area->y + area->h > mode->height)
		return 0;

	if (area->alpha < 0 || area->alpha > 15)
		return 0;

	if (area->color_mode != MVDU_COLOR_MODE_565 &&
	    area->color_mode != MVDU_COLOR_MODE_5551 &&
	    area->color_mode != MVDU_COLOR_MODE_4444)
		return 0;

	return 1;
}

static inline int mvdu_osd_area_pair_ok(struct mvdu_device *dev,
		const struct mvdu_osd_area *area1,
		const struct mvdu_osd_area *area2)
{
	if (area2->y >= area1->y + area1->h ||
	    area1->y >= area2->y + area2->h)
		return 1;
	else
		return 0;
}

int mvdu_osd_set_areas(struct mvdu_device *dev, int areas_nr,
		const struct mvdu_osd_area *areas)
{
	int i, j, best;
	struct mvdu_osd_area *save;
	unsigned long flags;
#ifdef CONFIG_1888TX018
	// dirty hack, because the function is called from the mode notifier
	// when dev->lock is already locked
	bool locked = spin_is_locked(&dev->lock);
#endif

	if (areas_nr < 0 || areas_nr > dev->osd_areas_max)
		return -EINVAL;

	for (i = 0; i < areas_nr; i++) {
		if (!mvdu_osd_area_ok(dev, &areas[i]))
			return -EINVAL;
		for (j = 0; j < i; j++) {
			if (!mvdu_osd_area_pair_ok(dev, &areas[i], &areas[j]))
				return -EINVAL;
		}
	}
	save = dev->osd_areas;

#ifdef CONFIG_1888TX018
	if (!locked)
#endif
	spin_lock_irqsave(&dev->lock, flags);

	dev->osd_areas_nr = areas_nr;

	/* We have to sort areas for hardware */
	for (i = 0; i < areas_nr; i++) {
		best = -1;
		for (j = 0; j < areas_nr; j++) {
			if (i > 0 && areas[j].y <= save[i-1].y)
				continue;
			if (best >= 0 && areas[j].y > areas[best].y)
				continue;
			best = j;
		}
		BUG_ON(best == -1);
		save[i] = areas[best];
	}

	mvdu_update_osd_headers(dev);

#ifdef CONFIG_1888TX018
	if (!locked)
#endif
	spin_unlock_irqrestore(&dev->lock, flags);

#ifdef DEBUG
	mvdu_dump_areas(dev);
#endif
	return 0;
}
EXPORT_SYMBOL(mvdu_osd_set_areas);

int mvdu_osd_get_areas(struct mvdu_device *dev, int *areas_nr,
		struct mvdu_osd_area *areas)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dev->lock, flags);

	*areas_nr = dev->osd_areas_nr;
	for (i = 0; i < dev->osd_areas_nr; i++)
		areas[i] = dev->osd_areas[i];

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}
EXPORT_SYMBOL(mvdu_osd_get_areas);

static void mvdu_handle_osd_fr_rd_end(struct mvdu_device *dev)
{
	if (dev->osd_last_change_avmp != -1) {
		dev->osd_last_change_avmp = -1;
		mvdu_remove_osd_fr_rd_end_waiter(dev);
	}

	mvdu_wake_osd_fr_rd_end_waiters(dev);
}

/*
 * MVL parameter processing
 * ------------------------
 *
 * External-visible MVL parameters come within struct mvdu_mvl_params.
 *
 * This object may contain only part of available parameters, so
 * it has a 'valid' field defining what object actually has.
 *
 * mvdu_mvl_merge_params() merges data from a new struct mvdu_mvl_params
 * into another struct mvdu_mvl_params.
 *
 * mvdu_mvl_check_params() checks a struct mvdu_mvl_params for completeness
 * and correctness.
 *
 * mvdu_mvl_compile_params() computes VDU register values for current
 * parameter set, and detects changes in register values. This happens
 * when new valid parameters are received.
 *
 * mvdu_mvl_write_params() writes pre-compited parameters to hardware.
 * This happens on MVL state change.
 */

static void mvdu_mvl_normalize_filter_params(struct mvdu_filter_params *fp)
{
	unsigned int sum;

	if (fp->prev > 0x10000 || fp->this > 0x10000 || fp->next > 0x10000) {
		fp->prev >>= 8;
		fp->this >>= 8;
		fp->next >>= 8;
	}

	sum = fp->prev + fp->this + fp->next;
	fp->prev = fp->prev * 253 / sum;
	fp->this = fp->this * 253 / sum;
	fp->next = fp->next * 253 / sum;
}

static struct mvdu_filter_params default_filter_params = {
	.prev = 0,
	.this = 253,
	.next = 0,
};

void mvdu_mvl_merge_params(struct mvdu_video_params *params,
				const struct mvdu_video_params *update)
{
	if (update->valid & MVDU_VALID_IN_MODE) {
		params->valid |= MVDU_VALID_IN_MODE;
		params->in_mode = update->in_mode;
	}

	if (update->valid & MVDU_VALID_OUT_MODE) {
		params->valid |= MVDU_VALID_OUT_MODE;
		params->out_mode = update->out_mode;
	}

	if (update->valid & MVDU_VALID_FORMAT) {
		params->valid |= MVDU_VALID_FORMAT;
		params->pixel_format = update->pixel_format;
		params->planes_nr = update->planes_nr;
		params->plane_format = update->plane_format;
		params->color_space = update->color_space;
		params->byte_order = update->byte_order;
		params->use_dropper = update->use_dropper;
		params->y_linestep = update->y_linestep;
		params->c_linestep = update->c_linestep;
		MVDU_OFFSET_Y(params) = MVDU_OFFSET_Y(update);
		MVDU_OFFSET_CB(params) = MVDU_OFFSET_CB(update);
		MVDU_OFFSET_CR(params) = MVDU_OFFSET_CR(update);
	}

	if (update->valid & MVDU_VALID_SRC) {
		params->valid |= MVDU_VALID_SRC;
		params->src = update->src;
	}

	if (update->valid & MVDU_VALID_DST) {
		params->valid |= MVDU_VALID_DST;
		params->dst = update->dst;
	}

	if (update->valid & MVDU_VALID_FILTER) {
		params->valid |= MVDU_VALID_FILTER;
		params->filter_enabled = update->filter_enabled;
		if (params->filter_enabled) {
			params->y_hor = update->y_hor;
			mvdu_mvl_normalize_filter_params(&params->y_hor);
			params->c_hor = update->c_hor;
			mvdu_mvl_normalize_filter_params(&params->c_hor);
			params->y_ver = update->y_ver;
			mvdu_mvl_normalize_filter_params(&params->y_ver);
			params->c_ver = update->c_ver;
			mvdu_mvl_normalize_filter_params(&params->c_ver);
		} else {
			params->y_hor = default_filter_params;
			params->c_hor = default_filter_params;
			params->y_ver = default_filter_params;
			params->c_ver = default_filter_params;
		}
	}

	if (update->valid & MVDU_VALID_BG) {
		params->valid |= MVDU_VALID_BG;
		params->bg_enabled = update->bg_enabled;
		if (params->bg_enabled)
			params->bg_color = update->bg_color;
	}
}
EXPORT_SYMBOL(mvdu_mvl_merge_params);

int mvdu_mvl_check_params(struct mvdu_device *dev,
		const struct mvdu_video_params *p)
{
	struct mvdu_mode *mode;
	int h_to_scale;

	if ((p->valid & MVDU_VALID_ENOUGH) != MVDU_VALID_ENOUGH)
		return -EINVAL;		/* not everything configured */

	if (!MVDU_VIDEO_IN_VALID(p->in_mode))
		return -EINVAL;

	if (!MVDU_MODE_VALID(p->out_mode))
		return -EINVAL;
	if ((dev->supported_modes & (1 << p->out_mode)) == 0)
		return -EINVAL;

	if (mvdu_mode_vmode(p->out_mode) != mvdu_vmode_for_in(p->in_mode))
		return -EINVAL;

	if (p->planes_nr == MVDU_THREE_PLANES &&
	    p->plane_format == MVDU_PLANE_FORMAT_MACROBLOCK)
		return -EINVAL;

	if (p->src.x < 0 || p->src.y < 0 ||
	    p->src.x >= MVDU_VIDEO_IN_MAX_WIDTH ||
	    p->src.y >= MVDU_VIDEO_IN_MAX_HEIGHT ||
	    p->src.w < MVDU_VIDEO_IN_MIN_WIDTH ||
	    p->src.w > MVDU_VIDEO_IN_MAX_WIDTH ||
	    p->src.h < MVDU_VIDEO_IN_MIN_HEIGHT ||
	    p->src.h > MVDU_VIDEO_IN_MAX_HEIGHT ||
	    p->src.x + p->src.w > MVDU_VIDEO_IN_MAX_WIDTH ||
	    p->src.y + p->src.h > MVDU_VIDEO_IN_MAX_HEIGHT)
		return -EINVAL;

	mode = &mvdu_modes[p->out_mode];
	if (p->dst.x < 0 || p->dst.y < 0 ||
	    p->dst.x >= mode->width || p->dst.y >= mode->height ||
	    p->dst.w <= 0 || p->dst.h <= 0 ||
	    p->dst.w > mode->width || p->dst.y > mode->height ||
	    p->dst.x + p->dst.w > mode->width ||
	    p->dst.y + p->dst.h > mode->height)
		return -EINVAL;

	if (p->in_mode == MVDU_VIDEO_IN_2FRAMES_TO_I_FULL)
		h_to_scale = 2 * p->src.h;
	else if (p->in_mode == MVDU_VIDEO_IN_SUBFRAMES_TO_P)
		h_to_scale = p->src.h / 2;
	else
		h_to_scale = p->src.h;

	if (p->src.w < p->dst.w / 4 || p->src.w > p->dst.w * 3 ||
	    h_to_scale < p->dst.h / 3 || h_to_scale > p->dst.h * 3)
		return -EINVAL;

	if (p->filter_enabled) {
		if (!p->y_hor.prev && !p->y_hor.this && !p->y_hor.next)
			return -EINVAL;
		if (!p->c_hor.prev && !p->c_hor.this && !p->c_hor.next)
			return -EINVAL;
		if (!p->y_ver.prev && !p->y_ver.this && !p->y_ver.next)
			return -EINVAL;
		if (!p->c_ver.prev && !p->c_ver.this && !p->c_ver.next)
			return -EINVAL;
	}

	if (p->use_dropper && p->in_mode != MVDU_VIDEO_IN_FRAME_TO_I &&
			      p->in_mode != MVDU_VIDEO_IN_2FRAMES_TO_I_NORM)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(mvdu_mvl_check_params);

static inline int linear_offset(int x, int y, int linesize)
{
	return y * linesize + x;
}

static inline int macroblock_offset(int x, int y, int mb_in_line, int small)
{
	/* small=0 - Y or 4:2:2 C macroblock - 256 bytes, 16x16
	 * small=1 - 4:2:0 C macroblock - 128 bytes, 16x8 */
	int mb_size = small ? 128 : 256;
	int mb_x = x >> 4;
	int in_mb_x = x & 15;
	int mb_y = small ? y >> 3 : y >> 4;
	int in_mb_y = small ?
		(((y & 6) >> 1) | ((y & 1) << 2)) :
		(((y & 14) >> 1) | ((y & 1) << 3));

	return mb_size * (mb_y * mb_in_line + mb_x) + 16 * in_mb_y + in_mb_x;
}

static void mvdu_mvl_compile_params(struct mvdu_device *dev)
{
	struct mvdu_video_params *p = &dev->mvl_params;
	u32 val;

	int three_planes = (p->planes_nr == MVDU_THREE_PLANES) ? 1 : 0;
	int format_420 = (p->pixel_format == MVDU_PIXEL_FORMAT_420) ? 1 : 0;
	int macroblocks = (p->plane_format == MVDU_PLANE_FORMAT_MACROBLOCK) ? 1 : 0;
	int big_endian = (p->byte_order == MVDU_VIDEO_BIG_ENDIAN) ? 1 : 0;

	int ycbcr_format, plane_num,
	    plane_access, fild_access;	/* mi_ctrl */

	int src_y_x, src_c_x,		/* byte position in source line */
	    src_y_y, src_c_y,		/* number of source line */

	    src_y_w, src_y_h,
	    src_c_w, src_c_h,		/* source height and width to write */

	    dst_x, dst_y,		/* where to start MVL on screen */

	    dst_y_w, dst_y_h,		/* dest height and width to write */
	    dst_c_w, dst_c_h,

	    y_full_size, c_full_size;	/* full line sizes to write */

	int h_scale, v_scale, h_upscale, v_upscale,
	    h_cut, v_cut, h_flt, v_flt;	/* scaler_ctrl */

	int y_y, y_cb, y_cr,
	    cb_y, cb_cb, cb_cr,
	    cr_y, cr_cb, cr_cr;		/* color conversions */

	int src_w_extra, dst_w_extra;	/* "take extra and cut" support */

#define UPDATE_PARAM(reg) do {			\
	if (dev->reg != val) {			\
		dev->reg = val;			\
		dev->mvl_change_pending = 1;	\
	}					\
} while (0)

	ycbcr_format = format_420 ? 0 : 1;
	plane_num = three_planes ? 1 : 0;
	plane_access = macroblocks ? 1 : 0;
	/* fild_access should be 0 when data is read in progressive mode,
	   and 1 if data is read in interlaced mode */
	switch (p->in_mode) {
	case MVDU_VIDEO_IN_FRAME_TO_I:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_NORM:
		fild_access = p->use_dropper ? 0 : 1;
		break;
	default:
		fild_access = 0;
		break;
	}
	val = shift_value(big_endian, MI_CTRL_BUF_BIT_ENDIAN) |
	      shift_value(ycbcr_format, MI_CTRL_YCbCr_FORMAT) |
	      shift_value(plane_num, MI_CTRL_PLANE_NUM) |
	      shift_value(plane_access, MI_CTRL_PLANE_ACCESS) |
	      shift_value(fild_access, MI_CTRL_FILD_ACCESS);
	UPDATE_PARAM(mi_ctrl);

	/* Sizes for DMA module, and address offsets
	 *
	 * For Y plane, these directly match image size. When reading
	 * subframe, or half of a full frame, vertical size is 1/2
	 * of image size.
	 *
	 * For C plane:
	 * - horizontal size is always 1/2 of image horizontal size,
	 * - vertical size is:
	 *   - for YCbCr 4:2:0, entire C data is read even for
	 *     interleaved modes. Because of that, C vertical size is 1/2 of
	 *     image size if memory contains frames, or 1/4 of
	 *     image size if memory contains subframes.
	 *   - for YCbCr 4:2:2, only lines used for output are read,
	 *     that is always same as Y vertical size.
	 */

	if (three_planes)
		src_w_extra = p->src.x & 15;
	else
		src_w_extra = p->src.x & 7;

	dst_w_extra = ((src_w_extra * p->dst.w) + (p->src.w / 2)) / p->src.w;
	dst_w_extra &= ~1;	/* hardware may cut only even number of pixels */

	src_y_x = p->src.x - src_w_extra;
	src_y_w = p->src.w + src_w_extra;

	src_c_x = three_planes ? src_y_x / 2 : src_y_x;
	src_c_w = src_y_w / 2;

	switch (p->in_mode) {

	case MVDU_VIDEO_IN_FRAME_TO_P:
	case MVDU_VIDEO_IN_FRAME_TO_PP:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_FULL:
	read_full_area:

		/* Read entire frame area p->src */

		src_y_y = p->src.y;
		src_y_h = p->src.h;
		y_full_size = p->y_linestep;
		src_c_y = format_420 ? p->src.y / 2 : p->src.y;
		src_c_h = format_420 ? p->src.h / 2 : p->src.h;
		c_full_size = p->c_linestep;

		if (macroblocks) {
			dev->mvl_odd_y_offset = macroblock_offset(
				src_y_x, src_y_y, p->y_linestep, 0);
			dev->mvl_odd_c_offset = macroblock_offset(
				src_c_x, src_c_y, p->c_linestep, format_420);
		} else {
			dev->mvl_odd_y_offset = linear_offset(
				src_y_x, src_y_y, p->y_linestep);
			dev->mvl_odd_c_offset = linear_offset(
				src_c_x, src_c_y, p->c_linestep);
		}
		dev->mvl_even_y_offset = dev->mvl_odd_y_offset;
		dev->mvl_even_c_offset = dev->mvl_odd_c_offset;

		break;

	case MVDU_VIDEO_IN_FRAME_TO_I:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_NORM:

		if (p->use_dropper)
			goto read_full_area;

		/* Y and 4:2:2 C - read every other line starting at p->src.y
		 * 4:2:0 C - read entire frame area p->src */

		src_y_y = p->src.y;
		src_y_h = p->src.h / 2;
		src_c_y = format_420 ? p->src.y / 2 : p->src.y;
		src_c_h = p->src.h / 2;

		if (macroblocks) {
			y_full_size = p->y_linestep;
			c_full_size = p->c_linestep;

			dev->mvl_odd_y_offset = macroblock_offset(
				src_y_x, src_y_y, p->y_linestep, 0);
			dev->mvl_odd_c_offset = macroblock_offset(
				src_c_x, src_c_y, p->c_linestep, format_420);

			dev->mvl_even_y_offset = dev->mvl_odd_y_offset + 128;
			dev->mvl_even_c_offset = format_420 ?
				dev->mvl_odd_c_offset :
				dev->mvl_odd_c_offset + 128;
		} else {
			y_full_size = 2 * p->y_linestep;
			c_full_size = format_420 ?
				p->c_linestep : 2 * p->c_linestep;

			dev->mvl_odd_y_offset = linear_offset(
				src_y_x, src_y_y, p->y_linestep);
			dev->mvl_odd_c_offset = linear_offset(
				src_c_x, src_c_y, p->c_linestep);

			dev->mvl_even_y_offset =
				dev->mvl_odd_y_offset + p->y_linestep;
			dev->mvl_even_c_offset = format_420 ?
				dev->mvl_odd_c_offset :
				dev->mvl_odd_c_offset + p->c_linestep;
		}

		break;

	case MVDU_VIDEO_IN_SUBFRAMES_TO_P:
	case MVDU_VIDEO_IN_SUBFRAMES_TO_I:
	default:	/* stop warnings */

		/* Read entire subframe (half vertical sizes) */

		src_y_y = p->src.y / 2;
		src_y_h = p->src.h / 2;
		y_full_size = p->y_linestep;
		src_c_y = format_420 ? p->src.y / 4 : p->src.y / 2;
		src_c_h = format_420 ? p->src.h / 4 : p->src.h / 2;
		c_full_size = p->c_linestep;

		if (macroblocks) {
			dev->mvl_odd_y_offset = macroblock_offset(
				src_y_x, src_y_y, p->y_linestep, 0);
			dev->mvl_odd_c_offset = macroblock_offset(
				src_c_x, src_c_y, p->c_linestep, format_420);
		} else {
			dev->mvl_odd_y_offset = linear_offset(
				src_y_x, src_y_y, p->y_linestep);
			dev->mvl_odd_c_offset = linear_offset(
				src_c_x, src_c_y, p->c_linestep);
		}
		dev->mvl_even_y_offset = dev->mvl_odd_y_offset;
		dev->mvl_even_c_offset = dev->mvl_odd_c_offset;

		break;
	}

	val = shift_value(src_y_w - 1, MI_MVL_Y_H_SIZE) |
	      shift_value(src_y_h - 1, MI_MVL_Y_V_SIZE);
	UPDATE_PARAM(mi_mvl_y_size);

	val = shift_value(src_c_w - 1, MI_MVL_C_H_SIZE) |
	      shift_value(src_c_h - 1, MI_MVL_C_V_SIZE);
	UPDATE_PARAM(mi_mvl_c_size);

	val = shift_value(y_full_size, MI_MVL_Y_FULL_SIZE) |
	      shift_value(c_full_size, MI_MVL_C_FULL_SIZE);
	UPDATE_PARAM(mi_mvl_full_size);


	dst_x = p->dst.x;
	dst_y_w = p->dst.w + dst_w_extra;
	dst_c_w = dst_y_w / 2;

	/* Destination Y height is same as destination height - that is,
	 * - p->dst.y for progressive output modes,
	 * - p->dst.y/2 for interlaced output modes.
	 */
	switch (p->in_mode) {
	case MVDU_VIDEO_IN_FRAME_TO_P:
	case MVDU_VIDEO_IN_FRAME_TO_PP:
	case MVDU_VIDEO_IN_SUBFRAMES_TO_P:
		dst_y = p->dst.y;
		dst_y_h = p->dst.h;
		break;
	case MVDU_VIDEO_IN_FRAME_TO_I:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_NORM:
		dst_y = p->dst.y / 2;
		if (p->use_dropper)
			dst_y_h = p->dst.h;
		else
			dst_y_h = p->dst.h / 2;
		break;
	default:
		dst_y = p->dst.y / 2;
		dst_y_h = p->dst.h / 2;
		break;
	}

	/* For scaler, destination C height is "in the same units" as
	 * source C height, so:
	 * - if full-frame input:
	 *   - 4:2:2 input, FRAME_TO_P, FRAME_TO_PP => dst.h
	 *   - 4:2:0 input, FRAME_TO_P, FRAME_TO_PP => dst.h / 2
	 *   - 4:2:2 input, FRAME_TO_I, 2FRAMES_TO_I_NORM => dst.h / 2
	 *   - 4:2:0 input, 2FRAMES_TO_I_FULL => dst.h / 4 [same as above
	 *     but we need downscale twice vertically]
	 *   - 4:2:0 input, FRAME_TO_I, 2FRAMES_TO_I_NORM => dst.h / 2 [since
	 *     entire frame is read both for displaying odd and even subframe]
	 *   - 4:2:0 input, 2FRAMES_TO_I_FULL => dst.h / 4 [same as above
	 *     but we need downscale twise vertically]
	 * - if subframe input - all C data is read, so
	 *     SUBFRAMES_TO_I, 4:2:2 input => dst.h / 2
	 *     SUBFRAMES_TO_I, 4:2:0 input => dst.h / 4
	 *     SUBFRAMES_TO_P - need upscale twice
	 */
	switch (p->in_mode) {
	case MVDU_VIDEO_IN_FRAME_TO_P:
	case MVDU_VIDEO_IN_FRAME_TO_PP:
	case MVDU_VIDEO_IN_SUBFRAMES_TO_P:
		dst_c_h = format_420 ? p->dst.h / 2 : p->dst.h;
		break;
	case MVDU_VIDEO_IN_FRAME_TO_I:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_NORM:
		if (p->use_dropper)
			dst_c_h = format_420 ? p->dst.h / 2 : p->dst.h;
		else
			dst_c_h = p->dst.h / 2;
		break;
	case MVDU_VIDEO_IN_SUBFRAMES_TO_I:
	case MVDU_VIDEO_IN_2FRAMES_TO_I_FULL:
	default:	/* stop warning */
		dst_c_h = format_420 ? p->dst.h / 4 : p->dst.h / 2;
		break;
	}

	val = shift_value(dst_x, DIF_MVL_START_H) |
	      shift_value(dst_y, DIF_MVL_START_V);
	UPDATE_PARAM(dif_mvl_start);

	h_scale = (src_y_w == dst_y_w) ? 0 : 1;
	v_scale = (src_y_h == dst_y_h) ? 0 : 1;
	h_upscale = (dst_y_w > src_y_w) ? 1 : 0;
	v_upscale = (dst_y_h > src_y_h) ? 1 : 0;
	h_cut = (dst_w_extra != 0) ? 1 : 0;
	v_cut = 0;
	h_flt = v_flt = (p->filter_enabled ? 1 : 0);
	val = shift_value(h_scale, SCALER_H_SCALER_ENA) |
	      shift_value(v_scale, SCALER_V_SCALER_ENA) |
	      shift_value(h_upscale, SCALER_H_UPSIZE) |
	      shift_value(v_upscale, SCALER_V_UPSIZE) |
	      shift_value(h_cut, SCALER_H_CUT_ENA) |
	      shift_value(v_cut, SCALER_V_CUT_ENA) |
	      shift_value(h_flt, SCALER_H_FLT_ENA) |
	      shift_value(v_flt, SCALER_V_FLT_ENA) |
	      shift_value(p->use_dropper, SCAL_DROP_ENA_STR);
	UPDATE_PARAM(scaler_ctrl);

	val = ((src_y_w - 1) << 16) / (dst_y_w - 1);
	UPDATE_PARAM(scaler_sch_y);

	val = ((src_y_h - 1) << 16) / (dst_y_h - 1);
	UPDATE_PARAM(scaler_scv_y);

	val = ((src_c_w - 1) << 16) / (dst_c_w - 1);
	UPDATE_PARAM(scaler_sch_c);

	val = ((src_c_h - 1) << 16) / (dst_c_h - 1);
	UPDATE_PARAM(scaler_scv_c);

	val = shift_value(dst_y_w - 1, SCALER_SIZE_HY) |
	      shift_value(dst_y_h - 1, SCALER_SIZE_VY);
	UPDATE_PARAM(scaler_size_y);

	val = shift_value(dst_c_w - 1, SCALER_SIZE_HC) |
	      shift_value(dst_c_h - 1, SCALER_SIZE_VC);
	UPDATE_PARAM(scaler_size_c);

	val = shift_value(dst_y_w - dst_w_extra - 1, SCALER_SIZE_CUT_H) |
	      shift_value(dst_y_h - 1, SCALER_SIZE_CUT_V);
	UPDATE_PARAM(scaler_size_cut);

	val = shift_value(dst_w_extra, SCALER_CUT_H_L);
	UPDATE_PARAM(scaler_size_cut_h);

	/* Colormap conversion */

	if (mvdu_mode_def(p->out_mode) == MVDU_MODE_SD &&
	    p->color_space == MVDU_COLOR_SPACE_HD) {

		/* conversion from HD to SD */
		y_y = 0x24a;  y_cb = 0x19;   y_cr = 0x31;
		cb_y = 0x1f;  cb_cb = 0xfd;  cb_cr = 0x21c;
		cr_y = 0x17;  cr_cb = 0x213; cr_cr = 0xfc;

	} else if (mvdu_mode_def(p->out_mode) == MVDU_MODE_HD &&
		   p->color_space == MVDU_COLOR_SPACE_SD) {

		/* conversion from SD to HD */
		y_y = 0x53;   y_cb = 0x21e;  y_cr = 0x235;
		cb_y = 0x222; cb_cb = 0x105; cb_cr = 0x1d;
		cr_y = 0x21a; cr_cb = 0x13;  cr_cr = 0x107;

	} else {

		/* no colormap conversion */

		y_y = 0x0;   y_cb = 0x0;    y_cr = 0x0;
		cb_y = 0x0;  cb_cb = 0x100; cb_cr = 0x0;
		cr_y = 0x0;  cr_cb = 0x0;   cr_cr = 0x100;
	}

	val = shift_value(y_y, SCALER_MVL_CLR_Y_Y) |
	      shift_value(y_cb, SCALER_MVL_CLR_Y_Cb) |
	      shift_value(y_cr, SCALER_MVL_CLR_Y_Cr);
	UPDATE_PARAM(scaler_mvl_clr_y);

	val = shift_value(cb_y, SCALER_MVL_CLR_Cb_Y) |
	      shift_value(cb_cb, SCALER_MVL_CLR_Cb_Cb) |
	      shift_value(cb_cr, SCALER_MVL_CLR_Cb_Cr);
	UPDATE_PARAM(scaler_mvl_clr_cb);

	val = shift_value(cr_y, SCALER_MVL_CLR_Cr_Y) |
	      shift_value(cr_cb, SCALER_MVL_CLR_Cr_Cb) |
	      shift_value(cr_cr, SCALER_MVL_CLR_Cr_Cr);
	UPDATE_PARAM(scaler_mvl_clr_cr);

	/* Filter registers */

	val = shift_value(p->y_ver.prev + 1, SCALER_FLT_C0_VY) |
	      shift_value(p->y_hor.next + 1, SCALER_FLT_C0_HY);
	UPDATE_PARAM(scaler_flt_y0);

	val = shift_value(p->y_ver.this + 1, SCALER_FLT_C1_VY) |
	      shift_value(p->y_hor.this + 1, SCALER_FLT_C1_HY);
	UPDATE_PARAM(scaler_flt_y1);

	val = shift_value(p->y_ver.next + 1, SCALER_FLT_C2_VY) |
	      shift_value(p->y_hor.prev + 1, SCALER_FLT_C2_HY);
	UPDATE_PARAM(scaler_flt_y2);

	val = shift_value(p->c_ver.prev + 1, SCALER_FLT_C0_VC) |
	      shift_value(p->c_hor.next + 1, SCALER_FLT_C0_HC);
	UPDATE_PARAM(scaler_flt_c0);

	val = shift_value(p->c_ver.this + 1, SCALER_FLT_C1_VC) |
	      shift_value(p->c_hor.this + 1, SCALER_FLT_C1_HC);
	UPDATE_PARAM(scaler_flt_c1);

	val = shift_value(p->c_ver.next + 1, SCALER_FLT_C2_VC) |
	      shift_value(p->c_hor.prev + 1, SCALER_FLT_C2_HC);
	UPDATE_PARAM(scaler_flt_c2);

#undef UPDATE_PARAM
}

static void mvdu_mvl_write_params(struct mvdu_device *dev)
{
	write_reg(dev, dev->dif_mvl_start, MVDU_REG_DIF_MVL_START);
	write_reg(dev, dev->mi_ctrl, MVDU_REG_MI_CTRL);
	write_reg(dev, dev->mi_mvl_y_size, MVDU_REG_MI_MVL_Y_SIZE);
	write_reg(dev, dev->mi_mvl_c_size, MVDU_REG_MI_MVL_C_SIZE);
	write_reg(dev, dev->mi_mvl_full_size, MVDU_REG_MI_MVL_FULL_SIZE);
	write_reg(dev, dev->scaler_ctrl, MVDU_REG_SCALER_CTRL);
	write_reg(dev, dev->scaler_sch_y, MVDU_REG_SCALER_SCH_Y);
	write_reg(dev, dev->scaler_scv_y, MVDU_REG_SCALER_SCV_Y);
	write_reg(dev, dev->scaler_sch_c, MVDU_REG_SCALER_SCH_C);
	write_reg(dev, dev->scaler_scv_c, MVDU_REG_SCALER_SCV_C);
	write_reg(dev, dev->scaler_size_y, MVDU_REG_SCALER_SIZE_Y);
	write_reg(dev, dev->scaler_size_c, MVDU_REG_SCALER_SIZE_C);
	write_reg(dev, dev->scaler_size_cut, MVDU_REG_SCALER_SIZE_CUT);
	write_reg(dev, dev->scaler_size_cut_h, MVDU_REG_SCALER_PHS_CUT_H);
	write_reg(dev, dev->scaler_mvl_clr_y, MVDU_REG_SCALER_MVL_CLR_Y);
	write_reg(dev, dev->scaler_mvl_clr_cb, MVDU_REG_SCALER_MVL_CLR_Cb);
	write_reg(dev, dev->scaler_mvl_clr_cr, MVDU_REG_SCALER_MVL_CLR_Cr);
	write_reg(dev, dev->scaler_flt_y0, MVDU_REG_SCALER_FLT_Y_C0);
	write_reg(dev, dev->scaler_flt_y1, MVDU_REG_SCALER_FLT_Y_C1);
	write_reg(dev, dev->scaler_flt_y2, MVDU_REG_SCALER_FLT_Y_C2);
	write_reg(dev, dev->scaler_flt_c0, MVDU_REG_SCALER_FLT_C_C0);
	write_reg(dev, dev->scaler_flt_c1, MVDU_REG_SCALER_FLT_C_C1);
	write_reg(dev, dev->scaler_flt_c2, MVDU_REG_SCALER_FLT_C_C2);
	write_reg(dev, 0xFFFFFFFF, MVDU_REG_SCALER_FLT_NORM);

	dev->mvl_change_pending = 0;

	if (dev->mvl_params.bg_enabled)
		mvdu_do_set_bgcolor(dev, &dev->mvl_params.bg_color);

	/* Save registers to restore at reset time, when originals may be
	 * already overwritten */
	dev->mi_ctrl_saved = dev->mi_ctrl;
	dev->scaler_ctrl_saved = dev->scaler_ctrl;
}


/*
 * Receive buffers and parameters for MVL
 *
 * Parameters, if given, are merged into current parameters (dev->mvl_params).
 * Then parameters object is released.
 *
 * If current parameters are not consistent, buffer is released immediately,
 * and the whole procedure is repeated in hope that next buffer will come
 * with better parameters.
 *
 * If current parameters are consistent, buffer is prepared for usage, and
 * stored in dev->new_buffer.
 */
static void mvdu_mvl_fetch_new_buffer(struct mvdu_device *dev)
{
	struct mvdu_video_buf *buffer;
	struct mvdu_video_params *params;
	struct mvdu_video_params *cparams = &dev->mvl_params;

retry:
	if (dev->mvl_new_buffer || !dev->mvl_new_buffer_offered)
		return;

	/* First receive buffer and params */
	dev->mvl_new_buffer_offered =
		dev->mvl_fetch_buffer(dev, &buffer, &params);

	/* Save and release params, if any */
	if (params) {

		mvdu_mvl_merge_params(&dev->mvl_params, params);
		dev->mvl_release_params(dev, params);

		if (mvdu_mvl_check_params(dev, &dev->mvl_params))
			dev->mvl_params_consistent = 0;
		else {
			dev->mvl_params_consistent = 1;

			/* Support only even coordinates and sizes */
			cparams->src.x &= ~1;
			cparams->src.w &= ~1;
			if (cparams->pixel_format == MVDU_PIXEL_FORMAT_420 &&
			    (cparams->in_mode == MVDU_VIDEO_IN_SUBFRAMES_TO_P ||
			     cparams->in_mode == MVDU_VIDEO_IN_SUBFRAMES_TO_I)) {
				cparams->src.y &= ~3;
				cparams->src.h &= ~3;
			} else {
				cparams->src.y &= ~1;
				cparams->src.h &= ~1;
			}
			cparams->dst.x &= ~1;
			cparams->dst.w &= ~1;
			cparams->dst.y &= ~1;
			cparams->dst.h &= ~1;

			/* "Compile" params, detect changes */
			mvdu_mvl_compile_params(dev);
		}
	}

	/* Silently drop buffer if current params are inconsistent */
	if (!dev->mvl_params_consistent) {
		dev->mvl_release_buffer(dev, buffer);
		goto retry;
	}

	/* Prepare addresses to write to hardware */

	switch (cparams->in_mode) {
	case MVDU_VIDEO_IN_FRAME_TO_P:
	case MVDU_VIDEO_IN_FRAME_TO_PP:
	case MVDU_VIDEO_IN_FRAME_TO_I:
		buffer->odd_y_addr = MVDU_ADDR_Y(buffer);
		buffer->even_y_addr = buffer->odd_y_addr;
		if (cparams->planes_nr == MVDU_THREE_PLANES) {
			buffer->odd_cb_addr = MVDU_ADDR_CB(buffer);
			buffer->even_cb_addr = buffer->odd_cb_addr;
			buffer->odd_cr_addr = MVDU_ADDR_CR(buffer);
			buffer->even_cr_addr = buffer->odd_cr_addr;
		} else {
			buffer->odd_cb_addr = MVDU_ADDR_C(buffer);
			buffer->even_cb_addr = buffer->odd_cb_addr;
		}
		break;
	default:
		buffer->odd_y_addr = MVDU_ADDR_FR1_Y(buffer);
		buffer->even_y_addr = MVDU_ADDR_FR2_Y(buffer);
		if (cparams->planes_nr == MVDU_THREE_PLANES) {
			buffer->odd_cb_addr = MVDU_ADDR_FR1_CB(buffer);
			buffer->even_cb_addr = MVDU_ADDR_FR2_CB(buffer);
			buffer->odd_cr_addr = MVDU_ADDR_FR1_CR(buffer);
			buffer->even_cr_addr = MVDU_ADDR_FR2_CR(buffer);
		} else {
			buffer->odd_cb_addr = MVDU_ADDR_FR1_C(buffer);
			buffer->even_cb_addr = MVDU_ADDR_FR2_C(buffer);
		}
		break;
	}
	buffer->odd_y_addr += dev->mvl_odd_y_offset;
	buffer->even_y_addr += dev->mvl_even_y_offset;
	buffer->odd_cb_addr += dev->mvl_odd_c_offset;
	buffer->even_cb_addr += dev->mvl_even_c_offset;
	if (cparams->planes_nr == MVDU_THREE_PLANES) {
		buffer->odd_cr_addr += dev->mvl_odd_c_offset;
		buffer->even_cr_addr += dev->mvl_even_c_offset;
		buffer->cr_valid = 1;
	} else
		buffer->cr_valid = 0;

	/* In MVDU_VIDEO_IN_FRAME_TO_P input mode, buffer is normally used
	 * for one pipeline entry. In other input modes, buffer is normally
	 * used for 2 pipeline entries.
	 */
	buffer->single_use = (cparams->in_mode == MVDU_VIDEO_IN_FRAME_TO_P);

	/* Buffer received */
	buffer->users = 0;
	dev->mvl_new_buffer = buffer;
}

/*
 * Buffer use count management
 */
static inline void mvdu_video_buf_get(struct mvdu_device *dev,
					struct mvdu_video_buf *buffer)
{
	buffer->users++;
}
static inline void mvdu_video_buf_put(struct mvdu_device *dev,
					struct mvdu_video_buf *buffer)
{
	buffer->users--;
	if (!buffer->users)
		dev->mvl_release_buffer(dev, buffer);
}


/*
 * MVL pipeline macros
 */
#define ODD_ENTRY(entry)	(((entry) & 1) == 0)
#define EVEN_ENTRY(entry)	(((entry) & 1) != 0)
#define NEXT_ENTRY(entry)	(((entry) + 1) & (MVDU_MVL_PIPELINE_SIZE - 1))
#define PREV_ENTRY(entry)	(((entry) - 1) & (MVDU_MVL_PIPELINE_SIZE - 1))

/*
 * Initialize a pipeline entry.
 *
 * If needed, a new buffer is fetched.
 *
 * If new buffer is needed but not available, previous buffer is reused,
 * and 'hungry' flag is set in the pipeline entry.
 *
 * If new buffer brings new parameters, it is also not used. Previous
 * buffer is used instead, and 'old_params' flag is set on the pipeline
 * entry. Parameter change is handled elsewhere.
 */
static void mvdu_mvl_init_pipeline_entry(struct mvdu_device *dev, int entry)
{
	struct mvdu_mvl_pipeline_entry *pe, *prev_pe;

	pe = &dev->mvl_pipeline[entry];
	prev_pe = &dev->mvl_pipeline[PREV_ENTRY(entry)];

	/* Normally, we want to use a new buffer for each even subframe, and
	 * for odd subframe if buffer is single_use. For odd subframes not in
	 * single_use mode, we want to use the same buffer as used for the
	 * previous even subframe.
	 *
	 * "Non-normal" situations:
	 * - new buffer is wanted, but is either not available, or brings
	 *   parameter changes
	 *   - then we use the previous buffer
	 * - we are not in single_use mode, and are selecting buffer for even
	 *   subframe; however when selecting previous odd subframe, we had
	 *   to use "previous buffer", and now a new one is available
	 *   - then use a new buffer
	 */

	/* Maybe we don't need even to try a new buffer */
	if (EVEN_ENTRY(entry) &&
	    !prev_pe->buffer->single_use && !prev_pe->hungry) {
		pe->buffer = prev_pe->buffer;
		pe->hungry = 0;
		pe->old_params = 0;
		goto out;
	}

	/* We do want a new buffer */
	mvdu_mvl_fetch_new_buffer(dev);

	/* If new buffer is not available, we are "hungry" */
	if (!dev->mvl_new_buffer) {
		pe->buffer = prev_pe->buffer;
		pe->hungry = 1;
		pe->old_params = 0;
		goto out;
	}

	/* If changing mode from single_use buffers to non-signle_use buffers,
	 * refuse to do that before even subframe, silently repeat last
	 * buffer instead
	 */
	if (dev->mvl_change_pending && EVEN_ENTRY(entry) &&
	    prev_pe->buffer->single_use && !dev->mvl_new_buffer->single_use) {
		pe->buffer = prev_pe->buffer;
		pe->hungry = 0;
		pe->old_params = 0;
		goto out;
	}

	/* If new parameters, repeat last buffer and set flag
	 */
	if (dev->mvl_change_pending) {
		pe->buffer = prev_pe->buffer;
		pe->hungry = 0;
		pe->old_params = 1;
		goto out;
	}

	/* Accept new buffer */
	pe->buffer = dev->mvl_new_buffer;
	dev->mvl_new_buffer = 0;
	pe->hungry = 0;
	pe->old_params = 0;

out:
	mvdu_video_buf_get(dev, pe->buffer);
}


/*
 * Choose registers to write odd/even enties.
 *
 * Next register hardware will use is in MVL_AVMP bit.
 * Next pipeline entry to show is in dev->mvl_hw_curr_entry.
 */
static void mvdu_mvl_choose_odd_even_regs(struct mvdu_device *dev)
{
	/* AVMP bit is zero => will start at reg0, one => at reg1 */
	int next_entry_uses_reg0 =
		(read_reg(dev, MVDU_REG_STAT_VDU_AVMP) &
				MVDU_REG_STAT_VDU_AVMP_MVL_AVMP_MASK) ? 0 : 1;

	dev->mvl_reg0_is_odd =
		((next_entry_uses_reg0 && ODD_ENTRY(dev->mvl_hw_curr_entry)) ||
		 (!next_entry_uses_reg0 && !ODD_ENTRY(dev->mvl_hw_curr_entry)));
}

/*
 * Write addresses for the given pipeline entry to hardware
 */
static void mvdu_mvl_write_pipeline_entry(struct mvdu_device *dev, int entry)
{
	int y_reg, cb_reg, cr_reg;
	u32 y_addr, cb_addr, cr_addr;
	struct mvdu_video_buf *b = dev->mvl_pipeline[entry].buffer;

	if (ODD_ENTRY(entry)) {
		y_addr = b->odd_y_addr;
		cb_addr = b->odd_cb_addr;
		cr_addr = b->odd_cr_addr;
	} else {
		y_addr = b->even_y_addr;
		cb_addr = b->even_cb_addr;
		cr_addr = b->even_cr_addr;
	}

	if ((ODD_ENTRY(entry) && dev->mvl_reg0_is_odd) ||
	    (EVEN_ENTRY(entry) && !dev->mvl_reg0_is_odd)) {
		y_reg = MVDU_REG_MI_MVL_Y_BAreg0;
		cb_reg = MVDU_REG_MI_MVL_Cb_BAreg0;
		cr_reg = MVDU_REG_MI_MVL_Cr_BAreg0;
	} else {
		y_reg = MVDU_REG_MI_MVL_Y_BAreg1;
		cb_reg = MVDU_REG_MI_MVL_Cb_BAreg1;
		cr_reg = MVDU_REG_MI_MVL_Cr_BAreg1;
	}

	write_reg(dev, y_addr, y_reg);
	write_reg(dev, cb_addr, cb_reg);
	if (b->cr_valid)
		write_reg(dev, cr_addr, cr_reg);
}

/*
 * Initialize MVL pipeline
 */
static void mvdu_mvl_init_pipeline(struct mvdu_device *dev)
{
	dev->mvl_unfilled_entry = 0;
	dev->mvl_hw_curr_entry = 0;
	dev->mvl_unfreed_entry = 0;
}

/*
 * Feed MVL pipeline: init next an antry and write it's addresses
 */
static void mvdu_mvl_feed_pipeline(struct mvdu_device *dev)
{
	int entry;

	entry = dev->mvl_unfilled_entry;
	mvdu_mvl_init_pipeline_entry(dev, entry);
	mvdu_mvl_write_pipeline_entry(dev, entry);
	dev->mvl_unfilled_entry = NEXT_ENTRY(entry);
}

/*
 * Cleanup MVL pipeline: remove already-used entries
 */
static void mvdu_mvl_cleanup_pipeline(struct mvdu_device *dev)
{
	struct mvdu_mvl_pipeline_entry *pe;

	while (dev->mvl_unfreed_entry != dev->mvl_hw_curr_entry) {
		pe = &dev->mvl_pipeline[dev->mvl_unfreed_entry];
		mvdu_video_buf_put(dev, pe->buffer);
		pe->buffer = 0;
		dev->mvl_unfreed_entry = NEXT_ENTRY(dev->mvl_unfreed_entry);
	}
}

/*
 * Flush MVL pipeline: remove all entries
 */
static void mvdu_mvl_flush_pipeline(struct mvdu_device *dev, int flush_new)
{
	struct mvdu_mvl_pipeline_entry *pe;

	while (dev->mvl_unfreed_entry != dev->mvl_unfilled_entry) {
		pe = &dev->mvl_pipeline[dev->mvl_unfreed_entry];
		mvdu_video_buf_put(dev, pe->buffer);
		pe->buffer = 0;
		dev->mvl_unfreed_entry = NEXT_ENTRY(dev->mvl_unfreed_entry);
	}

	if (flush_new && dev->mvl_new_buffer) {
		dev->mvl_release_buffer(dev, dev->mvl_new_buffer);
		dev->mvl_new_buffer = 0;
	}
}

/*
 * Init MVL hardware and pipeline: used before starting MVL with new
 * parameters
 */
static void mvdu_mvl_init_pipeline_and_hardware(struct mvdu_device *dev)
{
	mvdu_mvl_write_params(dev);

	if (dev->current_mode != dev->mvl_params.out_mode)
		mvdu_apply_mode(dev, dev->mvl_params.out_mode);

	mvdu_mvl_init_pipeline(dev);
	mvdu_mvl_choose_odd_even_regs(dev);
	mvdu_mvl_feed_pipeline(dev);	/* 1st entry */
	mvdu_mvl_feed_pipeline(dev);	/* 2nd entry */
}

/*
 * MVL states
 * ----------
 *
 * *) MVL_STATE_OFF: no streaming
 *
 *    MVL is completely disabled, no start request is pending.
 *    MVL pipeline is empty, driver does not own any buffers.
 *
 *    Transition MVL_STATE_OFF -> MVL_STATE_READY happens:
 *    - immediately after entering MVL_STATE_OFF, if mvdu_mvl_announce_buffer()
 *      processing is pending, or
 *    - later, if mvdu_mvl_announce_buffer() is called.
 *
 *    On transition MVL_STATE_OFF -> MVL_STATE_READY:
 *    - parameters are written to hardware, including video mode change,
 *    - 2 pipeline entries are filled, addresses are written to hardware,
 *    - if in VDU_STATE_ON, SA waiter is added.
 *
 * *) MVL_STATE_READY: about to start straming
 *
 *    MVL and MVL SW_ENA are disabled.
 *    MVL pipeline is filled with 2 entries.
 *    Parameters and addresses have been written to hardware.
 *    Current video mode and current background color have been switched
 *    to what was set in video parameters.
 *
 *    Transition MVL_STATE_READY -> MVL_STATE_OFF happens if
 *    mvdu_mvl_stop_streaming() is called.
 *
 *    On transition MVL_STATE_READY -> MVL_STATE_OFF:
 *    - pipeline is flushed,
 *    - if in VDU_STATE_ON, SA waiter is removed.
 *
 *    Transition MVL_STATE_READY -> MVL_STATE_RUNNING happens:
 *    - if VDU_STATE_ON is entered,
 *    - if while in VDU_STATE_ON, SA interrupt happens, and either in
 *      prorgessive mode, or next subframe will be even.
 *
 *    On transition MVL_STATE_READY -> MVL_STATE_RUNNING:
 *    - MVL and MVL SW_ENA are enabled,
 *    - mvl_fr_rd_end interrupt is cleared and enabled,
 *    - if in SA handler, SA waiter is removed.
 *
 *    If, while in MVL_STATE_READY, VDU_STATE_ON is left:
 *    - SA waiter is removed
 *
 * *) MVL_STATE_RUNNING: streaming
 *
 *    VDU is in VDU_STATE_ON, MVL and MVL SW_ENA both are enabled.
 *    Pipeline is kept filled "two entries ahead" by hooking at mvl_fr_rd_end
 *    and ensuring that (N+1)th and (N+2)th entries are filled [assuming
 *    hardware is processing Nth entry], and addresses for (N+1)th and
 *    (N+2)th entries written to hardware. If buffer to fill pipeline is not
 *    available yet [hungry state], or buffer brings parameter change, then
 *    pipeline is filled with copies of the previous buffer.
 *
 *    Transition MVL_STATE_RUNNING -> MVL_STATE_STOPPING_S happens if
 *    mvdu_mvl_stop_streaming() is called.
 *
 *    Transition MVL_STATE_RUNNING -> MVL_STATE_STOPPING_O happens if
 *    VDU_STATE_ON is left (e.g. underflow).
 *
 *    Transition MVL_STATE_RUNNING -> MVL_STATE_STOPPING_M happens if
 *    (sub)frame next after one being currently displayed by hardware
 *    should have different parameters.
 *
 *    On all these transitions:
 *    - MVL SW_ENA is disabled,
 *    - mvl_fr_rd_end interrupt is disabled,
 *    - if entering MVL_STATE_STOPPING_S or MVL_STATE_STOPPING_M, SA waiter
 *      is added
 *
 * *) MVL_STATE_STOPPING_S: streaming stop in progress
 *
 *    MVL is enabled, MVL SW_ENA is disabled, waiting for hardware to complete
 *    displaying (sub)frame for which fetch already started.
 *
 *    If in VDU_STATE_ON, SA waiter is active. If leaving VDU_STATE_ON,
 *    SA waiter is removed.
 *
 *    Transition MVL_STATE_STOPPING_S -> MVL_STATE_OFF happens:
 *    - in SA interrupt, with "sticky mvl_fr_rd_end",
 *    - if VDU_STATE_OFF is entered.
 *
 *    On MVL_STATE_STOPPING_S -> MVL_STATE_OFF transition:
 *    - MVL is disabled,
 *    - pipeline is dropped,
 *    - if in VDU_STATE_ON, SA waiter is removed.
 *
 * *) MVL_STATE_STOPPING_O: handling VDU_STATE_ON leave.
 *
 *    MVL is enabled, MVL SW_ENA is disabled, waiting for hardware to complete
 *    VDU stop.
 *
 *    Transition MVL_STATE_STOPPING_O -> MVL_STATE_STOPPING_S happens
 *    if mvdu_mvl_stop_streaming() is called.
 *
 *    Transition MVL_STATE_STOPPING_O -> MVL_STATE_READY happens
 *    if VDU_STATE_OFF is entered.
 *
 *    On MVL_STATE_STOPPING_O -> MVL_STATE_READY transition:
 *    - MVL is disabled (if still enabled),
 *    - already-displayed entries and possible following even subframe [if not
 *      in single_use buffer mode] are dequeued from pipeline,
 *    - if to-be-displayed frame requires different parameters, those
 *      are written to hardware, possibly new video mode is applied,
 *    - new entries are fetched such as two entries become available,
 *      and addresses are written to hardware,
 *    - SA waiter is removed.
 *
 * *) MVL_STATE_STOPPING_M: handling streaming parameter change.
 *
 *    MVL is enabled, MVL SW_ENA is disabled, waiting for hardware to complete
 *    displaying (sub)frame for which fetch already started.
 *
 *    SA waiter is active.
 *
 *    Transition MVL_STATE_STOPPING_M -> MVL_STATE_STOPPING_S happens
 *    if mvdu_mvl_stop_streaming() is called.
 *
 *    Transition MVL_STATE_STOPPING_M -> MVL_STATE_STOPPING_O happens
 *    VDU_STATE_ON is left (e.g. underflow).
 *
 *    Transition MVL_STATE_STOPPING_M -> MVL_STATE_READY happens if
 *    "sticky mvl_fr_rd_end" is detected in SA interrupt,
 *    IF new video mode differs from the current one.
 *
 *    On MVL_STATE_STOPPING_M -> MVL_STATE_READY transition:
 *    - MVL is disabled,
 *    - pipeline is dropped,
 *    - new parameters are applied, including video mode change,
 *    - two new entries are fetched, and addresses written to hardware,
 *    - SA waiter is removed.
 *
 *    Transition MVL_STATE_STOPPING_M -> MVL_STATE_RUNNING happens if
 *    "sticky mvl_fr_rd_end" is detected in SA interrupt,
 *    IF new video mode does not differ from the current one.
 *
 *    On MVL_STATE_STOPPING_M -> MVL_STATE_READY transition:
 *    - pipeline is dropped,
 *    - new parameters are applied,
 *    - two new entries are fetched, and addresses written to hardware,
 *    - SA waiter is removed.
 */

static void mvdu_mvl_off_to_ready(struct mvdu_device *dev)
{
	mvdu_mvl_fetch_new_buffer(dev);
	if (!dev->mvl_new_buffer)
		return;		/* no valid buffer or params */

	mvdu_mvl_init_pipeline_and_hardware(dev);

	if (dev->vdu_state == VDU_STATE_ON)
		mvdu_add_sa_waiter(dev);

	mvdu_set_mvl_state(dev, MVL_STATE_READY);
}

static void mvdu_mvl_ready_to_off(struct mvdu_device *dev)
{
	if (dev->vdu_state == VDU_STATE_ON)
		mvdu_remove_sa_waiter(dev);

	mvdu_mvl_flush_pipeline(dev, 1);

	mvdu_set_mvl_state(dev, MVL_STATE_OFF);
}

static void mvdu_mvl_ready_to_running(struct mvdu_device *dev)
{
	/* Called when in VDU_STATE_ON, from SA handler */

	u32 control;

	control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	control |= (MVDU_REG_CTRL_VDU_ENA_MVL_ENA_MASK |
		    MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK);
	write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

	mvdu_clear_and_enable_irq(dev, MVL_FR_RD_END);

	mvdu_set_mvl_state(dev, MVL_STATE_RUNNING);
}

static void mvdu_mvl_complete_mode_change(struct mvdu_device *dev)
{
	/* Called in MVDU_STATE_STOPPING_M, when vblanking */

	u32 control;

	mvdu_mvl_flush_pipeline(dev, 0);

	/* Issue: from within mvdu_mvl_init_pipeline_and_hardware()
	 * mode switch may be called, which will see MVL_STATE_STOPPING_M
	 * and call mvdu_remove_sa_waiter(), breaking things.
	 * Fool it by setting MVL_STATE_STOPPING_O - it will be overwritten
	 * a few lines below */
	mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_O);
	mvdu_mvl_init_pipeline_and_hardware(dev);

	if (dev->vdu_state == VDU_STATE_ON) {

		control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
		control |= MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
		write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

		mvdu_clear_and_enable_irq(dev, MVL_FR_RD_END);

		mvdu_set_mvl_state(dev, MVL_STATE_RUNNING);
	} else
		mvdu_set_mvl_state(dev, MVL_STATE_READY);
}

/*
 * Check for "sticky MVL_FR_RD_END" condition
 */
static int mvdu_sticky_mvl_fr_rd_end(struct mvdu_device *dev)
{
	u32 val;

	write_reg(dev, IRQ_BIT(MVL_FR_RD_END), MVDU_REG_INT_VDU_STAT);
	val = read_reg(dev, MVDU_REG_INT_VDU_STAT);
	return (val & IRQ_BIT(MVL_FR_RD_END)) ? 1 : 0;
}


static void mvdu_mvl_running_to_stopping_m(struct mvdu_device *dev)
{
	/* Called from mvl_fr_rd_end handler, with MVL SW_ENA off */

	mvdu_disable_irq(dev, MVL_FR_RD_END);

	mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_M);

	mvdu_add_sa_waiter(dev);
	if (mvdu_disabled_or_vblanking(dev) && mvdu_sticky_mvl_fr_rd_end(dev)) {
		mvdu_mvl_complete_mode_change(dev);
		mvdu_remove_sa_waiter(dev);
	}
}

static void mvdu_mvl_complete_stop(struct mvdu_device *dev)
{
	/* Called in MVDU_STATE_STOPPING_S, when off or vblanking */

	u32 control;

	if (dev->vdu_state == VDU_STATE_ON) {
		control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
		control &= ~MVDU_REG_CTRL_VDU_ENA_MVL_ENA_MASK;
		write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);
	}

	mvdu_mvl_flush_pipeline(dev, 1);
	mvdu_set_mvl_state(dev, MVL_STATE_OFF);

	if (dev->mvl_new_buffer_offered)
		mvdu_mvl_off_to_ready(dev);
}

static void mvdu_mvl_running_to_stopping_s(struct mvdu_device *dev)
{
	/* Called from mvdu_mvl_stop_streaming() */

	u32 control;

	control = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	control &= ~MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
	write_reg(dev, control, MVDU_REG_CTRL_VDU_ENA);

	mvdu_disable_irq(dev, MVL_FR_RD_END);

	mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_S);

	mvdu_add_sa_waiter(dev);
	if (mvdu_disabled_or_vblanking(dev) && mvdu_sticky_mvl_fr_rd_end(dev)) {
		mvdu_mvl_complete_stop(dev);
		mvdu_remove_sa_waiter(dev);
	}
}

static void mvdu_mvl_stopping_o_to_ready(struct mvdu_device *dev)
{
	/* Called when in VDU_STATE_OFF */

	int entry;

	/* Entry considered "current" won't be displayed any more */
	entry = NEXT_ENTRY(dev->mvl_hw_curr_entry);

	/* Skip next entry as well if it is even and not single_use buffer */
	if (EVEN_ENTRY(entry) && !dev->mvl_pipeline[entry].buffer->single_use)
		entry = NEXT_ENTRY(dev->mvl_hw_curr_entry);

	/* At least 1 entry after dev->mvl_hw_curr_entry is definitly fetched,
	 * but we could go two entries */
	if (entry == dev->mvl_unfilled_entry)
		mvdu_mvl_feed_pipeline(dev);

	/* Check if entry requires parameter change */
	if (dev->mvl_pipeline[entry].old_params) {
		mvdu_mvl_flush_pipeline(dev, 0);
		mvdu_mvl_init_pipeline_and_hardware(dev);
	}

	mvdu_mvl_choose_odd_even_regs(dev);
	mvdu_set_mvl_state(dev, MVL_STATE_READY);
}

static void mvdu_mvl_vdu_stop_complete(struct mvdu_device *dev)
{
	/* Called in transition to VDU_STATE_OFF, after VDU reset */

	switch (dev->mvl_state) {
	case MVL_STATE_STOPPING_S:
		mvdu_mvl_complete_stop(dev);
		break;
	case MVL_STATE_STOPPING_O:
		mvdu_mvl_stopping_o_to_ready(dev);
		break;
	}
}

/*
 * Check MVL AVMP and catch situation when hardware DMA engine andvanced
 * forward more than expected. That may happen because of late call of
 * MVL_FR_RD_END handler, or if hardware switched to next frame after
 * interrupt disable, but before SW_ENA disable in hungry processing.
 *
 * Returns either dev->mvl_hw_curr_entry or NEXT_ENTRY(dev->mvl_hw_curr_entry).
 */
static int mvdu_mvl_guess_hw_curr_entry(struct mvdu_device *dev)
{
	/* AVMP bit is zero => processing reg1, one => precessing reg0 */
	int processing_reg0 =
		(read_reg(dev, MVDU_REG_STAT_VDU_AVMP) &
				MVDU_REG_STAT_VDU_AVMP_MVL_AVMP_MASK) ? 0 : 1;

	int processing_odd = (dev->mvl_reg0_is_odd && processing_reg0) ||
			     (!dev->mvl_reg0_is_odd && !processing_reg0);

	if ((ODD_ENTRY(dev->mvl_hw_curr_entry) && processing_odd) ||
	    (EVEN_ENTRY(dev->mvl_hw_curr_entry) && !processing_odd))
		return dev->mvl_hw_curr_entry;
	else
		return NEXT_ENTRY(dev->mvl_hw_curr_entry);
}

/*
 * Handle MVL_FR_RD_END interrupt (used in MVL_STATE_RUNNING only)
 */
static void mvdu_handle_mvl_fr_rd_end(struct mvdu_device *dev)
{
	u32 val;
	int next, after_next;
	struct mvdu_mvl_pipeline_entry *next_pe;

	/* Handler may be called when already not in MVL_STATE_RUNNING
	 * because of a race - just ignore it then.
	 */
	if (dev->mvl_state != MVL_STATE_RUNNING)
		return;

	/* Disable MVL SW_ENA */
	val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	val &= ~MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
	write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);

	/* Since we got MVL_FR_RD_END with MVL SW_ENA enabled,
	 * hardware DMA engine moved forward for at least one
	 * entry.
	 */
	dev->mvl_hw_curr_entry = NEXT_ENTRY(dev->mvl_hw_curr_entry);

	/* In case of late call, dev->mvl_hw_curr_entry may be still
	 * out of sync.
	 *
	 * Currently we can't detect out-of-sync for more than one
	 * entry. Although it is possible to do that by monitoring time.
	 */
	dev->mvl_hw_curr_entry = mvdu_mvl_guess_hw_curr_entry(dev);

	/* In case of late first call after startup:
	 * - hardware already re-fetched first frame,
	 * - hw_curr_entry now points to not-yet-fetched entry.
	 * Currently we just move hw_curr_entry backwards. FIXME.
	 */
	if (dev->mvl_hw_curr_entry == dev->mvl_unfilled_entry) {
		dev->mvl_hw_curr_entry = PREV_ENTRY(dev->mvl_hw_curr_entry);
		dev->mvl_hw_curr_entry = PREV_ENTRY(dev->mvl_hw_curr_entry);
	}

	/* Ensure that two extra entries are available */
	next = NEXT_ENTRY(dev->mvl_hw_curr_entry);
	if (next == dev->mvl_unfilled_entry)
		mvdu_mvl_feed_pipeline(dev);
	after_next = NEXT_ENTRY(next);
	if (after_next == dev->mvl_unfilled_entry)
		mvdu_mvl_feed_pipeline(dev);

	/* Check for ongoing mode change. In not, enable SW_ENA again */
	next_pe = &dev->mvl_pipeline[next];
	if (next_pe->old_params)
		mvdu_mvl_running_to_stopping_m(dev);
	else {
		val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
		val |= MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
		write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);
	}

	/* Free old pipeline entries */
	mvdu_mvl_cleanup_pipeline(dev);

	/* After SW_ENA enabled, good time to compile newer params */
	if (!dev->mvl_new_buffer && dev->mvl_new_buffer_offered)
		mvdu_mvl_fetch_new_buffer(dev);
}

/*
 * Handle availability of a new incoming buffer while in MVL_STATE_RUNNING.
 *
 * If in "hungry" state, it's time to try to recover.
 */
static void mvdu_mvl_maybe_feed_hungry(struct mvdu_device *dev)
{
	int curr_entry, orig_unfilled_entry;
	struct mvdu_mvl_pipeline_entry *pe;
	u32 val;

	if (!dev->mvl_pipeline[PREV_ENTRY(dev->mvl_unfilled_entry)].hungry)
		return;

	mvdu_mvl_fetch_new_buffer(dev);
	if (!dev->mvl_new_buffer || dev->mvl_change_pending)
		return;

	/* Disable MVL SW_ENA */
	val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	val &= ~MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
	write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);

	/* Find out current hardware position */
	curr_entry = mvdu_mvl_guess_hw_curr_entry(dev);

	/* Save current fetch position */
	orig_unfilled_entry = dev->mvl_unfilled_entry;

	/* Drop still-unused hungry entries */
	while (PREV_ENTRY(dev->mvl_unfilled_entry) != curr_entry &&
	       dev->mvl_pipeline[PREV_ENTRY(dev->mvl_unfilled_entry)].hungry) {
		dev->mvl_unfilled_entry = PREV_ENTRY(dev->mvl_unfilled_entry);
		pe = &dev->mvl_pipeline[dev->mvl_unfilled_entry];
		mvdu_video_buf_put(dev, pe->buffer);
		pe->buffer = 0;
	}

	/* Re-fetch new entries instead of dropped entries */
	while (dev->mvl_unfilled_entry != orig_unfilled_entry)
		mvdu_mvl_feed_pipeline(dev);

	/* Enable MVL SW_ENA */
	val = read_reg(dev, MVDU_REG_CTRL_VDU_ENA);
	val |= MVDU_REG_CTRL_VDU_ENA_MVL_BASE_SW_ENA_MASK;
	write_reg(dev, val, MVDU_REG_CTRL_VDU_ENA);
}


/* MVL interface routine */
void mvdu_mvl_announce_buffer(struct mvdu_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	dev->mvl_new_buffer_offered = 1;

	switch (dev->mvl_state) {
	case MVL_STATE_OFF:
		mvdu_mvl_off_to_ready(dev);
		break;
	case MVL_STATE_RUNNING:
		mvdu_mvl_maybe_feed_hungry(dev);
		break;
	}

	spin_unlock_irqrestore(&dev->lock, flags);
}
EXPORT_SYMBOL(mvdu_mvl_announce_buffer);

/* MVL interface routine */
void mvdu_mvl_stop_streaming(struct mvdu_device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	dev->mvl_new_buffer_offered = 0;

	switch (dev->mvl_state) {
	case MVL_STATE_READY:
		mvdu_mvl_ready_to_off(dev);
		break;
	case MVL_STATE_RUNNING:
		mvdu_mvl_running_to_stopping_s(dev);
		break;
	case MVL_STATE_STOPPING_O:
	case MVL_STATE_STOPPING_M:
		mvdu_set_mvl_state(dev, MVL_STATE_STOPPING_S);
		break;
	}

	spin_unlock_irqrestore(&dev->lock, flags);
}
EXPORT_SYMBOL(mvdu_mvl_stop_streaming);


static void mvdu_handle_sa(struct mvdu_device *dev)
{
	if (!mvdu_disabled_or_vblanking(dev))
		goto out;

	if ((dev->mvl_state == MVL_STATE_STOPPING_S ||
	     dev->mvl_state == MVL_STATE_STOPPING_M) &&
	    mvdu_sticky_mvl_fr_rd_end(dev)) {

		if (dev->mvl_state == MVL_STATE_STOPPING_S)
			mvdu_mvl_complete_stop(dev);
		else
			mvdu_mvl_complete_mode_change(dev);

		mvdu_remove_sa_waiter(dev);

		if (!mvdu_disabled_or_vblanking(dev))
			goto out;
	}

	if (dev->mvl_state == MVL_STATE_READY &&
	    dev->vdu_state == VDU_STATE_ON &&
	    (mvdu_modes[dev->current_mode].vmode == MVDU_MODE_PROGRESSIVE ||
			(read_reg(dev, MVDU_REG_STAT_VDU_DISP) &
			 MVDU_REG_STAT_VDU_DISP_FIELD_MASK) == 0)) {

		mvdu_mvl_ready_to_running(dev);

		mvdu_remove_sa_waiter(dev);

		if (!mvdu_disabled_or_vblanking(dev))
			goto out;
	}

	if (dev->osd_state == OSD_STATE_STARTING &&
	    (mvdu_modes[dev->current_mode].vmode == MVDU_MODE_PROGRESSIVE ||
			(read_reg(dev, MVDU_REG_STAT_VDU_DISP) &
			 MVDU_REG_STAT_VDU_DISP_FIELD_MASK) == 0)) {
		mvdu_update_osd_state_to_running(dev);

		mvdu_remove_sa_waiter(dev);

		if (!mvdu_disabled_or_vblanking(dev))
			goto out;
	}

	if (dev->bgcolor_dirty) {
		mvdu_write_bgcolor(dev);
		dev->bgcolor_dirty = 0;
		mvdu_remove_sa_waiter(dev);
	}

out:
	mvdu_wake_sa_waiters(dev);
}

static irqreturn_t mvdu_interrupt(int irq, void *data)
{
	struct mvdu_device *dev = data;
	u32 status, bits;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	status = read_reg(dev, MVDU_REG_INT_VDU_STAT);

	/*
	 * We are going to process and clear only enabled interrupts.
	 *
	 * However, because of a race between hardware setting bits and software
	 * acquiring/releasing lock, it may happen that no currently-
	 * unmasked bits are set.
	 *
	 * We can't return IRQ_NONE in that case because it is still "our"
	 * irq and we should not state otherwise.
	 * So currently we unconditionally return IRQ_HANDLED, which is not
	 * really correct. FIXME.
	 */

	bits = status & dev->irq_mask;
	write_reg(dev, bits, MVDU_REG_INT_VDU_STAT);

	/* Order matters - first MVL_FR_RD_END, then SA, then VDU_OFF */

	if (bits & IRQ_BIT(MVL_FR_RD_END))
		mvdu_handle_mvl_fr_rd_end(dev);

	if (bits & IRQ_BIT(OSD_FR_RD_END))
		mvdu_handle_osd_fr_rd_end(dev);

	if (bits & IRQ_BIT(SA))
		mvdu_handle_sa(dev);

	if (bits & IRQ_BIT(VDU_OFF))
		mvdu_handle_off(dev);

	if (bits & IRQ_BIT(O_FIFO_EMPTY))
		mvdu_handle_underflow(dev);

	if (bits & IRQ_BIT(INT_SYS_ERR))
		mvdu_handle_system_error(dev);

	if (bits & IRQ_BIT(FR_PRC_END)) {
		if (dev->osd_state == OSD_STATE_STOPPING)
			mvdu_update_osd_state_to_off(dev);
		else
			mvdu_disable_irq(dev, FR_PRC_END);
	}

	spin_unlock_irqrestore(&dev->lock, flags);

#if defined(MVDU_DEBUG_IRQ) && defined(DEBUG)

	/*
	 * Debugging output AFTER processing - to avoid late-processing
	 * because of slow serial output
	 */

#define show_irq_bit(name) do {						\
	if (status & IRQ_BIT(name))					\
		printk(" %s%s",						\
			(bits & IRQ_BIT(name)) ? "*" : "", #name);	\
} while (0)

	dev_dbg(dev->dev, "IRQ status %08x", status);
	show_irq_bit(INT_SYS_ERR);
	show_irq_bit(VDU_OFF);
	show_irq_bit(OSD_FR_RD_END);
	show_irq_bit(MVL_FR_RD_END);
	show_irq_bit(FR_PRC_END);
	show_irq_bit(V_SYNC);
	show_irq_bit(H_SYNC);
	show_irq_bit(SA);
	show_irq_bit(O_FIFO_EMPTY);
	show_irq_bit(I_FIFO_EMPTY);
	printk("\n");

#undef show_irq_bit

#endif
	return IRQ_HANDLED;
}


/*
 * Device enumeration
 */

static LIST_HEAD(mvdu_devices);

struct mvdu_device *mvdu_get_device(int num)
{
	struct list_head *p;
	struct mvdu_device *dev;
	int i = 0;

	list_for_each(p, &mvdu_devices) {
		if (i == num) {
			dev = list_entry(p, struct mvdu_device, devices_list);
			return dev;
		}
		i++;
	}

	return 0;
}
EXPORT_SYMBOL(mvdu_get_device);


/*
 * Linux Device Model integration
 */

#ifdef CONFIG_SYSFS

static ssize_t sync_show(struct device *d, struct device_attribute *attr,
		char *buf);
static ssize_t sync_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t n);

static DEVICE_ATTR(ext_sync, 0600, sync_show, sync_store);
static DEVICE_ATTR(int_sync, 0600, sync_show, sync_store);
static DEVICE_ATTR(sd_par_output, 0600, sync_show, sync_store);

static inline u32 attr_to_mask(struct device_attribute *attr)
{
	if (attr == &dev_attr_ext_sync)
		return MVDU_REG_DIF_CTRL_EXT_SYNC_EN_MASK;
	if (attr == &dev_attr_int_sync)
		return MVDU_REG_DIF_CTRL_INT_SYNC_EN_MASK;
	if (attr == &dev_attr_sd_par_output)
		return MVDU_REG_DIF_CTRL_SDTV_FORM_MASK;

	BUG();
	return 0;	/* Kill warning */
}

static ssize_t sync_show(struct device *d, struct device_attribute *attr,
		char *buf)
{
	struct mvdu_device *dev = d->platform_data;
	unsigned long mask = attr_to_mask(attr);

	return sprintf(buf, "%s\n", (dev->output_flags & mask) ? "1" : "0");
}

static ssize_t sync_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct mvdu_device *dev = d->platform_data;
	unsigned long mask = attr_to_mask(attr);
	unsigned long flags;

	if (n != 1 && (n != 2 || buf[1] != '\n'))
		return -EINVAL;
	if (buf[0] != '0' && buf[0] != '1')
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);

	if (buf[0] == '1')
		dev->output_flags |= mask;
	else
		dev->output_flags &= ~mask;

	if (dev->vdu_state == VDU_STATE_ON) {
		dev->vdu_restart_req = 1;
		mvdu_update_vdu_state(dev);
	}

	spin_unlock_irqrestore(&dev->lock, flags);

	return n;
}

#endif

#if defined(CONFIG_RCM_VDU_VIDEO) || defined(CONFIG_RCM_VDU_VIDEO_MODULE)
static int allocate_video_buffer(struct mvdu_device *vpu)
{
	void* p;
	static const size_t VIDEO_BUFFER_SIZE =
		MVDU_VIDEO_MAX_BUFFER_SIZE * MVDU_VIDEO_MAX_BUFFERS;

	pr_info("mvdu: will allocate buffers\n");

	p = devm_ioremap_resource(vpu->dev, &vpu->vpubuffer_res);

	if (IS_ERR(p)) {
		dev_err(vpu->dev, "cant remap video io memory\n");
		return PTR_ERR(p);
	};

	vpu->video_buffer_cpu = p;
	vpu->video_buffer_pa = vpu->vpubuffer_res.start;
	vpu->video_buffer_dma = vpu->vpubuffer_res.start;
	vpu->video_buffer_size = VIDEO_BUFFER_SIZE;

#ifdef CONFIG_1888TX018
	vpu->video_buffer_dma = (dma_addr_t)(vpu->video_buffer_pa - (vpu->dev->dma_pfn_offset << PAGE_SHIFT));
#endif

	pr_info("mvdu: did allocate buffers %p\n", p);

	return 0;

}
#endif

static int module_vdu_core_of_probe(struct platform_device *pdev,
				struct mvdu_device *vpu)
{
	struct device_node *np = pdev->dev.of_node;

	if(0 > of_property_read_u32(np, "supported_modes", &vpu->supported_modes)) {
		dev_err(&pdev->dev, "Missing required parameter 'supported_modes'\n");
		return -ENODEV;
	};
	if(0 > of_property_read_u32(np, "default_mode", &vpu->default_mode)) {
		dev_err(&pdev->dev, "Missing required parameter 'default_mode'\n");
		return -ENODEV;
	};
	if(0 > of_property_read_u32(np, "output_flags", &vpu->output_flags)) {
		dev_err(&pdev->dev, "Missing required parameter 'output_flags'\n");
		return -ENODEV;
	};
	if(0 > of_property_read_u32(np, "axi_osd_param", &vpu->axi_osd_param)) {
		dev_err(&pdev->dev, "Missing required parameter 'axi_osd_param'\n");
		return -ENODEV;
	};
	if(0 > of_property_read_u32(np, "axi_mvl_param", &vpu->axi_mvl_param)) {
		dev_err(&pdev->dev, "Missing required parameter 'axi_mvl_param'\n");
		return -ENODEV;
	};

#ifdef CONFIG_1888TX018
	{
		int ret = of_property_read_u32(np, "vdu-index", &vpu->vdu_index);
		if (ret != 0) {
			dev_err(&pdev->dev, "Missing required parameter 'vdu-index'\n");
			return -ENODEV;
		}
		if (vpu->vdu_index > 1) {
			dev_err(&pdev->dev, "parameter 'vdu-index' must be from 0 to 1\n");
			return -ENODEV;
		}
	}
#endif

#if defined(CONFIG_RCM_VDU_VIDEO) || defined(CONFIG_RCM_VDU_VIDEO_MODULE)
	/* vdu video support */
	{
		const struct property *prop;
		prop = of_find_property(np, "vdubuffer", NULL);
		if (prop && prop->value) {
			const __be32 *val;

			int nr = prop->length / sizeof(u32);
			if (nr != 2) {
				dev_err(&pdev->dev, "%s: Invalid vdubuffer\n", __func__);
				return -EINVAL;
			};
			val = prop->value;
			vpu->vpubuffer_res.start = be32_to_cpup(val++);
			vpu->vpubuffer_res.end = vpu->vpubuffer_res.start + be32_to_cpup(val) - 1;
			vpu->vpubuffer_res.flags = IORESOURCE_MEM;
			vpu->allocate_video_buffer = allocate_video_buffer;
		};
	}
#endif

	return 0;
}

#if defined(CONFIG_FB_RCM_VDU) || defined(CONFIG_FB_RCM_VDU_MODULE)
phys_addr_t k1879_dt_get_fb_base(void);
static int mvdu_get_fb_params(struct platform_device *pdev, struct mvdu_device *vdu)
{
	/*const*/ phys_addr_t fb_base = 0; //k1879_dt_get_fb_base();
	/*const*/ size_t fb_size = MVDU_OSD_BYTESPERPIXEL *
			  MVDU_OSD_BUFFER_WIDTH * MVDU_OSD_BUFFER_HEIGHT;

	struct device_node *of_node = pdev->dev.of_node;

#if 1
	struct device_node *mem_region;
	struct resource res;
	int ret;

	mem_region = of_parse_phandle(of_node, "memory-region", 0);
	if (!mem_region) {
		dev_err(&pdev->dev, "no memory-region phandle\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(mem_region, 0, &res);


	if (ret != 0) {
		dev_err(&pdev->dev,"failed to locate DT /reserved-memory resource\n");
		return -EINVAL;
	}
#endif
#if 0
	struct resource* res;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fbmem");
	if(!res) {
	    pr_info("mvdu_get_fb_params NULL res\n");
	    return -1;
	}
#endif
//	fb_size = resource_size(&res);
	fb_base = res.start;

	if(fb_base == 0) {
		dev_err(&pdev->dev,"fb_base is null\n");
		return -1;
	};

//	vdu->fb_buffer_cpu = ioremap(fb_base, fb_size);
//	vdu->fb_buffer_dma = (dma_addr_t) fb_base;
//	vdu->fb_buffer_size = fb_size;

	dev_info(&pdev->dev,"got fb address! 0x%08X\n", (unsigned)res.start);

	vdu->fb_buffer_cpu = phys_to_virt(fb_base);
	vdu->fb_buffer_pa = fb_base;
	vdu->fb_buffer_dma = (dma_addr_t) fb_base;
	vdu->fb_buffer_size = fb_size;

#ifdef CONFIG_1888TX018
	vdu->fb_buffer_dma = (dma_addr_t)(fb_base - (pdev->dev.dma_pfn_offset << PAGE_SHIFT));
#endif

	dev_info(&pdev->dev,"fb_buffer_cpu %px, fb_buffer_pa 0x%08X, fb_buffer_dma 0x%08X, fb_buffer_size 0x%X\n",
		vdu->fb_buffer_cpu, (unsigned)vdu->fb_buffer_pa, (unsigned)vdu->fb_buffer_dma, vdu->fb_buffer_size);

	return 0;
}
#else
static inline int mvdu_get_fb_params(struct platform_device *pdev, struct mvdu_device *vdu) {return 0;}
#endif

#ifdef CONFIG_1888TX018
static int pll_setup(struct mvdu_device *dev)
{
	int ret;
	unsigned data;
	bool locked;
	unsigned long timeout;

	// unlock the PLL controller if it is locked
	ret = regmap_read(dev->pll_control, CRG_VDU_WR_LOCK, &data);
	if (ret != 0)
		return ret;
	locked = (data == CRG_VDU_WR_LOCK_LOCKED);
	if (locked) {
		ret = regmap_write(dev->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_UNLOCK);
		if (ret != 0)
			return ret;
	}

	// restart the PLL controller
	ret = regmap_write(dev->pll_control, CRG_VDU_PLL_PRDIV, 1); // div 1
	if (ret != 0)
		return ret;
	ret = regmap_write(dev->pll_control, CRG_VDU_PLL_FBDIV, 88); // mul 88
	if (ret != 0)
		return ret;
	ret = regmap_write(dev->pll_control, CRG_VDU_PLL_PSDIV, 2); // dev 4
	if (ret != 0)
		return ret;
	ret = regmap_write(dev->pll_control, CRG_VDU_PLL_CTRL,
		CRG_VDU_PLL_CTRL_PLL_ENABLE | CRG_VDU_PLL_CTRL_PLL_RESTART);
	if (ret != 0)
		return ret;

	// wait until the PLL controller is ready
	timeout = jiffies + HZ / 10;
	while (true) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		ret = regmap_read(dev->pll_control, CRG_VDU_PLL_CTRL, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_PLL_CTRL_PLL_RESTART) != 0)
			continue;
		ret = regmap_read(dev->pll_control, CRG_VDU_PLL_STATE, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_PLL_STATE_PLL_RDY) != 0)
			break;
	}

	// lock the PLL controller if needed
	if (locked) {
		ret = regmap_write(dev->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_LOCK);
		if (ret != 0)
			return ret;
	}

	return 0;
}
#endif // CONFIG_1888TX018

static int module_vdu_core_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct resource *res;
	struct mvdu_device *vdu;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to find device registers\n");
		return -ENODEV;
	};

	irq = platform_get_irq(pdev, 0);
	if (0 > irq) {
		dev_err(&pdev->dev, "failed to find device irq\n");
		return -EINVAL;
	};

	vdu = devm_kzalloc(&pdev->dev, sizeof(*vdu), GFP_KERNEL);
	if (!vdu) {
		return -ENOMEM;
	};
	vdu->irq = irq;

	vdu->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vdu->regs)) {
		dev_err(&pdev->dev, "cant remap io memory\n");
		return PTR_ERR(vdu->regs);
	};

#ifdef CONFIG_1888TX018
	{
		struct device_node *syscon_node = of_parse_phandle(pdev->dev.of_node, "crg-control", 0);
		if (!syscon_node) {
			dev_err(&pdev->dev, "failed to find PLL control register reference\n");
			return -EINVAL;
		}
		vdu->pll_control = syscon_node_to_regmap(syscon_node);
		if (IS_ERR(vdu->pll_control)) {
			dev_err(&pdev->dev, "cant remap PLL control register reference\n");
			return PTR_ERR(vdu->pll_control);
		}
		ret = pll_setup(vdu);
		if (ret != 0) {
			dev_err(&pdev->dev, "pll setup error (%i)\n", ret);
			return ret;
		}
	}
	// setup dma -offset for bus
	pdev->dev.archdata.dma_offset = -(pdev->dev.dma_pfn_offset << PAGE_SHIFT); 	/* before v5.5 it was: set_dma_offset(&pdev->dev, -(pdev->dev.dma_pfn_offset << PAGE_SHIFT)); */
#endif

	vdu->regs_phys = res->start;

	ret = module_vdu_core_of_probe(pdev, vdu);
	if (ret) {
		dev_err(&pdev->dev, "cant of probe\n");
		return ret;
	};

	/*
	 * Before installing interrupt handler, make sure that VDU is off
	 * and interrupt handling subsystem is ready.
	 */
	write_reg(vdu, 0, MVDU_REG_CTRL_VDU_ENA);

	spin_lock_init(&vdu->lock);

	/* no need to lock before interrupt handler installed */
	__mvdu_disable_irq(vdu, ~0);
	__mvdu_clear_irq(vdu, ~0);

	mvdu_get_fb_params(pdev, vdu);
	dev_info(&pdev->dev, "after mvdu_get_fb_params\n");

	ret = devm_request_irq(&pdev->dev, vdu->irq, mvdu_interrupt, 0,
			       pdev->name, vdu);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return ret;
	};
	dev_info(&pdev->dev, "irq handler installed\n");

	vdu->dev = &pdev->dev;
	platform_set_drvdata(pdev, vdu);

	vdu->osd_areas_max = max_osd_areas;
	vdu->osd_areas = devm_kzalloc(&pdev->dev, vdu->osd_areas_max * sizeof(struct mvdu_osd_area), GFP_KERNEL);
	if (!vdu->osd_areas) {
		dev_err(&pdev->dev, "failed to allocale OSD area descriptors\n");
		return -ENOMEM;
	}

	vdu->osd_headers = dma_alloc_coherent(&pdev->dev,
			MVDU_OSD_HEADERS_ALLOC_SIZE(vdu), &vdu->osd_headers_dma,
			GFP_KERNEL);
	if (!vdu->osd_headers) {
		dev_err(&pdev->dev, "failed to allocale OSD heders\n");
		return -ENOMEM;
	}

	dev_info(&pdev->dev, "found VDU device at %08x, id <%08x>\n",
			(u32)vdu->regs_phys, read_reg(vdu, MVDU_REG_CTRL_VDU_ID));

	list_add_tail(&vdu->devices_list, &mvdu_devices);

	/* Initialize the rest */
	if (vdu->axi_mvl_param)
		write_reg(vdu, vdu->axi_mvl_param, MVDU_REG_MI_AXI_MVL_PARAM);
	if (vdu->axi_osd_param)
		write_reg(vdu, vdu->axi_osd_param, MVDU_REG_MI_AXI_OSD_PARAM);

	init_waitqueue_head(&vdu->sa_wq);
	init_waitqueue_head(&vdu->osd_fr_rd_end_wq);
	timer_setup(&vdu->delayed_restart_timer,
			mvdu_delayed_restart_timeout, TIMER_IRQSAFE);
	timer_setup(&vdu->restart_watchdog_timer,
			mvdu_restart_watchdog_timeout, TIMER_IRQSAFE);

//	write_reg(vdu, 0, MVDU_REG_OSD_CTRL);	/* Use RGBA, not ARGB */
	write_reg(vdu, 1, MVDU_REG_OSD_CTRL);
	write_reg(vdu, 0, MVDU_REG_SCALER_CTRL);

	if (initial_mode == MVDU_MODE_DEFAULT)
		vdu->current_mode = vdu->default_mode;
	else if (!MVDU_MODE_VALID(initial_mode)) {
		dev_warn(&pdev->dev,
			 "initial_mode %d is not valid, using default\n",
			 initial_mode);
		vdu->current_mode = vdu->default_mode;
	} else if ((vdu->supported_modes & (1 << initial_mode)) == 0) {
		vdu->current_mode = vdu->default_mode;
		dev_warn(&pdev->dev,
			 "initial_mode %d is not supported, using default\n",
			 initial_mode);
	} else
		vdu->current_mode = initial_mode;

	vdu->write_bgcolor_on_ena = vdu->write_osd_weights_on_ena = 1;

	vdu->osd_last_change_avmp = -1;
	vdu->osd_free_group = 0;	/* These 3 must be different */
	vdu->osd_cur_group = 1;
	vdu->osd_prev_group = 2;

	vdu->mvl_params.y_hor = default_filter_params;
	vdu->mvl_params.c_hor = default_filter_params;
	vdu->mvl_params.y_ver = default_filter_params;
	vdu->mvl_params.c_ver = default_filter_params;

#ifdef CONFIG_SYSFS
	if (device_create_file(&pdev->dev, &dev_attr_ext_sync) != 0 ||
	    device_create_file(&pdev->dev, &dev_attr_int_sync) != 0 ||
	    device_create_file(&pdev->dev, &dev_attr_sd_par_output) != 0)
		dev_warn(&pdev->dev, "failed to creater device attributes");
#endif

	return 0;
}

static int module_vdu_core_remove(struct platform_device *pdev)
{
	struct mvdu_device *const vdu = platform_get_drvdata(pdev);

#ifdef CONFIG_SYSFS
	device_remove_file(&pdev->dev, &dev_attr_ext_sync);
	device_remove_file(&pdev->dev, &dev_attr_int_sync);
	device_remove_file(&pdev->dev, &dev_attr_sd_par_output);
#endif

	del_timer_sync(&vdu->restart_watchdog_timer);
	del_timer_sync(&vdu->delayed_restart_timer);
	list_del(&vdu->devices_list);
	dma_free_coherent(&pdev->dev, MVDU_OSD_HEADERS_ALLOC_SIZE(vdu),
			vdu->osd_headers, vdu->osd_headers_dma);
	return 0;
}

static const struct of_device_id module_vdu_dt_ids[] = {
	{ .compatible = "rcm,vdu", },
	{}
};

static struct platform_driver module_vdu_driver = {
	.probe		= module_vdu_core_probe,
	.remove		= module_vdu_core_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rcm_vdu",
		.of_match_table = of_match_ptr(module_vdu_dt_ids),
	},
};

module_platform_driver(module_vdu_driver);

MODULE_AUTHOR("Nikita Youshchenko <yoush@cs.msu.su>, "
	      "Alexander Gerasiov <gq@cs.msu.su>");
MODULE_DESCRIPTION("Module VDU low-level device driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, module_vdu_dt_ids);
