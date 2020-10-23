/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/mfd/syscon.h>
#include <linux/dma-mapping.h>
#include <linux/regmap.h>

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_encoder.h>
#include <drm/drm_bridge.h>
#include <drm/drm_vblank.h>
#include <drm/drm_irq.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "drm-rcm-vdu-reg.h"
#include "drm-rcm-vdu-pll-reg.h"

#define DRIVER_NAME "rcm-vdu-drm"
#define DRIVER_DESC "rcm-vdu KMS driver"
#define DRIVER_DATE "2020"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

#define BACKGROUND_RGB 0xFFFF00 // ??? just for test

#define HARDWARE_PITCH_BITS 13

#define MAX_BIT_VALUE(bits) ((1U << (bits)) - 1U)

struct osd_area_desc {
	__le32 next;
	__le32 image_base;
	__le16 width;
	__le16 height;
	__le16 x;
	__le16 y;
	__le16 pitch;
	u8 flags;
	u8 alpha;
	u8 pad[12];
};

struct dma_data {
	struct osd_area_desc osd_area_descs[2];
};

struct vdu_device {
	struct drm_device drm_dev;
	struct platform_device *pdev;

	void __iomem *regs;
	unsigned regs_size;
	spinlock_t reg_lock;

	u32 vdu_index; // for PLL setup
	struct regmap *pll_control;
	struct dma_data *dma_data;
	dma_addr_t dma_data_phys_addr;

	struct drm_crtc crtc;
	struct drm_plane primary_plane;
	bool primary_plane_initialized;
	struct drm_encoder encoder;

	unsigned osd_area_desc_free_index;
	struct drm_pending_vblank_event *pending_vblank_event;
	struct completion vdu_off_done;
};

struct y_cb_cr_color
{
	u8 y;
	u8 cb;
	u8 cr;
};

DEFINE_DRM_GEM_CMA_FOPS(rcm_vdu_fops);
static irqreturn_t irq_handler(int irq, void *arg);

static struct drm_driver rcm_vdu_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.fops = &rcm_vdu_fops,
	.irq_handler = irq_handler,
	DRM_GEM_CMA_VMAP_DRIVER_OPS
};

static struct drm_framebuffer *fb_create(
	struct drm_device *dev, struct drm_file *file, const struct drm_mode_fb_cmd2 *mode_cmd);

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const uint32_t plane_formats[] = {
	// ??? DRM_FORMAT_RGB565,
	// ??? DRM_FORMAT_RGB888,
	DRM_FORMAT_ARGB8888 // ???
};

static const uint64_t plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static const struct drm_plane_funcs plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state
};

static int plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state);
static void plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state);

static const struct drm_plane_helper_funcs plane_helper_funcs = {
	.atomic_check = plane_atomic_check,
	.atomic_update = plane_atomic_update
};

int (*enable_vblank)(struct drm_crtc *crtc);

static int crtc_enable_vblank(struct drm_crtc *crtc);
static void crtc_disable_vblank(struct drm_crtc *crtc);

static const struct drm_crtc_funcs crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = crtc_enable_vblank,
	.disable_vblank = crtc_disable_vblank
};

static enum drm_mode_status crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode);
static void crtc_atomic_flush(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state);
static void crtc_atomic_enable(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state);
static void crtc_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state);

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_valid = crtc_mode_valid,
	.atomic_flush = crtc_atomic_flush,
	.atomic_enable = crtc_atomic_enable,
	.atomic_disable = crtc_atomic_disable
};

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static inline struct vdu_device *drm_dev_to_vdu(struct drm_device *drm_dev)
{
	return container_of(drm_dev, struct vdu_device, drm_dev);
}

static u32 read_vdu_reg(struct vdu_device *vdu, u32 offset)
{
	return ioread32(vdu->regs + offset);
}

static void write_vdu_reg(struct vdu_device *vdu, u32 offset, u32 data)
{
	iowrite32(data, vdu->regs + offset);
}

// RGB bits:
// 0-7 - B
// 8-15 - G
// 16-23 - R
struct y_cb_cr_color rgb_to_y_cb_cr(u32 rgb)
{
	struct y_cb_cr_color res;

	int y_r = 750;    // round(0.183 * 4096)
	int y_g = 2515;   // round(0.614 * 4096)
	int y_b = 254;    // round(0.062 * 4096)
	int cb_r = -414;  //round(-0.101 * 4096)
	int cb_g = -1384; //round(-0.338 * 4096)
	int cb_b = 1798;  // round(0.439 * 4096)
	int cr_r = 1798;  // round(0.439 * 4096)
	int cr_g = -1636; //round(-0.399 * 4096)
	int cr_b = -164;  //round(-0.040 * 4096)

	int r = (rgb >> 16) & 0xFF;
	int g = (rgb >> 8) & 0xFF;
	int b = (rgb >> 0) & 0xFF;

	int y = 16 + ((r * y_r) + (g * y_g) + (b * y_b) + 2048) / 4096;
	int cb = 128 + ((r * cb_r) + (g * cb_g) + (b * cb_b) + 2048) / 4096;
	int cr = 128 + ((r * cr_r) + (g * cr_g) + (b * cr_b) + 2048) / 4096;
	
	res.y = (u8)max(16, min(235, y));
	res.cb = (u8)max(16, min(240, cb));
	res.cr = (u8)max(16, min(240, cr));

	return res;
}

static int init_pll(struct vdu_device *vdu)
{
	int ret;
	unsigned data;
	bool locked;
	unsigned long timeout;

	// unlock the PLL controller if it is locked
	ret = regmap_read(vdu->pll_control, CRG_VDU_WR_LOCK, &data);
	if (ret != 0)
		return ret;
	locked = (data == CRG_VDU_WR_LOCK_LOCKED);
	if (locked) {
		ret = regmap_write(vdu->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_UNLOCK);
		if (ret != 0)
			return ret;
	}

	// restart the PLL controller
	ret = regmap_write(vdu->pll_control, CRG_VDU_PLL_PRDIV, 1); // div 1
	if (ret != 0)
		return ret;
	ret = regmap_write(vdu->pll_control, CRG_VDU_PLL_FBDIV, 88); // mul 88
	if (ret != 0)
		return ret;
	ret = regmap_write(vdu->pll_control, CRG_VDU_PLL_PSDIV, 2); // dev 4
	if (ret != 0)
		return ret;
	ret = regmap_write(vdu->pll_control, CRG_VDU_PLL_CTRL,
		CRG_VDU_PLL_CTRL_PLL_ENABLE | CRG_VDU_PLL_CTRL_PLL_RESTART);
	if (ret != 0)
		return ret;

	// wait until the PLL controller is ready
	timeout = jiffies + HZ / 10;
	while (true) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		ret = regmap_read(vdu->pll_control, CRG_VDU_PLL_CTRL, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_PLL_CTRL_PLL_RESTART) != 0)
			continue;
		ret = regmap_read(vdu->pll_control, CRG_VDU_PLL_STATE, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_PLL_STATE_PLL_RDY) != 0)
			break;
	}

	// lock the PLL controller if needed
	if (locked) {
		ret = regmap_write(vdu->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_LOCK);
		if (ret != 0)
			return ret;
	}

	return 0;
}

int setup_pll_pixel_clock(struct vdu_device *vdu, unsigned hz)
{
	int ret;
	unsigned data;
	bool locked;
	unsigned long timeout;
	unsigned divmode;
	
	divmode = (594000000 + hz / 2) / hz - 1;

	// unlock the PLL controller if it is locked
	ret = regmap_read(vdu->pll_control, CRG_VDU_WR_LOCK, &data);
	if (ret != 0)
		return ret;
	locked = (data == CRG_VDU_WR_LOCK_LOCKED);
	if (locked) {
		ret = regmap_write(vdu->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_UNLOCK);
		if (ret != 0)
			return ret;
	}

	// set the divider
	ret = regmap_write(vdu->pll_control,
		vdu->vdu_index == 0 ? CRG_VDU_CKDIVMODE_VOUT0 : CRG_VDU_CKDIVMODE_VOUT1,
		divmode);
	if (ret != 0)
		return ret;
	ret = regmap_write(vdu->pll_control, CRG_VDU_UPD_CK, CRG_VDU_UPD_CK_UPD_CKDIV);
	if (ret != 0)
		return ret;

	// wait until the PLL controller is ready
	timeout = jiffies + HZ / 10;
	while (true) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		ret = regmap_read(vdu->pll_control, CRG_VDU_UPD_CK, &data);
		if (ret != 0)
			return ret;
		if ((data & CRG_VDU_UPD_CK_UPD_CKDIV) == 0)
			break;
	}

	// lock the PLL controller if needed
	if (locked) {
		ret = regmap_write(vdu->pll_control, CRG_VDU_WR_LOCK, CRG_VDU_WR_LOCK_LOCK);
		if (ret != 0)
			return ret;
	}

	return 0;
}

static int reset_vdu(struct vdu_device *vdu)
{
	unsigned i;

	// disable and clear all interrupts
	write_vdu_reg(vdu, VDU_REG_INT_ENA, 0);
	write_vdu_reg(vdu, VDU_REG_INT_STAT, 0xFFFFFFFF);

	// turn off the hardware if it is needed
	if ((read_vdu_reg(vdu, VDU_REG_CTRL_ENA) & VDU_REG_CTRL_ENA_VDU_ENA_MASK) != 0)
	{
		write_vdu_reg(vdu, VDU_REG_CTRL_ENA, 0);
		udelay(100000);
		if ((read_vdu_reg(vdu, VDU_REG_INT_STAT) & VDU_REG_INT_VDU_ENA_MASK) == 0) {
			dev_err(&vdu->pdev->dev, "hardware off timeout");
			return -ETIMEDOUT;
		}
		write_vdu_reg(vdu, VDU_REG_INT_STAT, 0xFFFFFFFF);
	}

	// reset the hardware
	write_vdu_reg(vdu, VDU_REG_CTRL_SOFT_RESET, VDU_REG_CTRL_SOFT_RESET_MASK);
	udelay(1000);
	if ((read_vdu_reg(vdu, VDU_REG_CTRL_SOFT_RESET) & VDU_REG_CTRL_SOFT_RESET_MASK) != 0) {
		dev_err(&vdu->pdev->dev, "hardware reset timeout");
		return -ETIMEDOUT;
	}

	// clear all the registers
	for (i = 0; i < vdu->regs_size; i += 4)
		iowrite32(0, vdu->regs + i);

	// setup AXI bus parameters
	write_vdu_reg(vdu, VDU_REG_MI_AXI_MVL_PARAM, 0x0F310000);
	write_vdu_reg(vdu, VDU_REG_MI_AXI_OSD_PARAM, 0x0F310000);

	return 0;
}

// VDU must be disabled before the call
static void set_crtc_mode(struct vdu_device *vdu, struct drm_display_mode *mode)
{
	struct y_cb_cr_color background;
	u32 val;
	int ret;

	pr_info("*** set_crtc_mode VDU_REG_CTRL_ENA=0x%08X\n", read_vdu_reg(vdu, VDU_REG_CTRL_ENA)); // ???

	WARN_ON((read_vdu_reg(vdu, VDU_REG_CTRL_ENA) & VDU_REG_CTRL_ENA_VDU_ENA_MASK) != 0);

	ret = setup_pll_pixel_clock(vdu, mode->clock * 1000);
	if (ret != 0)
		dev_err(&vdu->pdev->dev, "PLL setup error (%i)\n", ret);

	val = VDU_REG_DIF_CTRL_HDTV_MASK
		| VDU_REG_DIF_CTRL_EXT_SYNC_EN_MASK
		| VDU_REG_DIF_CTRL_SDTV_FORM_MASK
		| VDU_REG_DIF_CTRL_HSYNC_P_MASK // ??? get from mode
		| VDU_REG_DIF_CTRL_VSYNC_P_MASK // ??? get from mode
		| VDU_REG_DIF_CTRL_DIF_444_MODE_MASK;
	if (drm_mode_vrefresh(mode) == 50)
		val |= VDU_REG_DIF_CTRL_VREFRESH_50_HZ_MASK;
	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) == 0)
		val |= VDU_REG_DIF_CTRL_NO_INTERLACE_MASK;
	write_vdu_reg(vdu, VDU_REG_DIF_CTRL, val);

	background = rgb_to_y_cb_cr(BACKGROUND_RGB);
	val = (background.y << VDU_REG_DIF_BGR_Y_SHIFT)
		| (background.cb << VDU_REG_DIF_BGR_CB_SHIFT)
		| (background.cr << VDU_REG_DIF_BGR_CR_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_BGR, val);

	val = ((mode->htotal - mode->hdisplay - 4 - 4 - 1) << VDU_REG_DIF_BLANK_HBLANK_SHIFT)
		| ((mode->vtotal - mode->vdisplay) << VDU_REG_DIF_BLANK_VBLANK_BEG_SHIFT)
		| (0 << VDU_REG_DIF_BLANK_VBLANK_END_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_BLANK, val);

	val = (mode->htotal << VDU_REG_DIF_FSIZE_HTOTAL_SHIFT)
		| (mode->crtc_vtotal << VDU_REG_DIF_FSIZE_VTOTAL_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_FSIZE, val);

	val = ((mode->hdisplay - 1) << VDU_REG_DIF_ASIZE_HACTIVE_SHIFT)
		| (mode->vdisplay << VDU_REG_DIF_ASIZE_VACTIVE_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_ASIZE, val);

	val = ((mode->hsync_start - mode->hdisplay) << VDU_REG_DIF_HSYNC_START_SHIFT)
		| ((mode->hsync_end - mode->hsync_start) << VDU_REG_DIF_HSYNC_LEN_SHIFT)
		| ((mode->htotal - mode->hsync_end) << VDU_REG_DIF_HSYNC_DELAY_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_HSYNC, val);

	val = ((mode->vsync_start - mode->vdisplay) << VDU_REG_DIF_VSYNC_START_SHIFT)
		| ((mode->vsync_end - mode->vsync_start) << VDU_REG_DIF_VSYNC_LEN_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_VSYNC, val);

	// ??? pr_info("*** VDU_REG_DIF_CTRL = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_CTRL));
	// ??? pr_info("*** VDU_REG_DIF_BGR = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_BGR));
	// ??? pr_info("*** VDU_REG_DIF_BLANK = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_BLANK));
	// ??? pr_info("*** VDU_REG_DIF_FSIZE = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_FSIZE));
	// ??? pr_info("*** VDU_REG_DIF_ASIZE = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_ASIZE));
	// ??? pr_info("*** VDU_REG_DIF_HSYNC = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_HSYNC));
	// ??? pr_info("*** VDU_REG_DIF_VSYNC = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_DIF_VSYNC));

	val = (0x2F << VDU_REG_OSD_COLOR_Y_R_SHIFT)
		| (0x9D << VDU_REG_OSD_COLOR_Y_G_SHIFT)
		| (0x10 << VDU_REG_OSD_COLOR_Y_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_Y, val);

	val = (0x11A << VDU_REG_OSD_COLOR_CB_R_SHIFT)
		| (0x157 << VDU_REG_OSD_COLOR_CB_G_SHIFT)
		| (0x70 << VDU_REG_OSD_COLOR_CB_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_CB, val);

	val = (0x70 << VDU_REG_OSD_COLOR_CR_R_SHIFT)
		| (0x166 << VDU_REG_OSD_COLOR_CR_G_SHIFT)
		| (0x10a << VDU_REG_OSD_COLOR_CR_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_CR, val);
}

// fb field in the primary plane state must be not-NULL
static void fill_osd_descriptor(struct vdu_device *vdu, struct osd_area_desc *desc)
{
	struct drm_plane_state *state = vdu->primary_plane.state;

	desc->next = 0;
	desc->image_base = cpu_to_le32(drm_fb_cma_get_gem_addr(state->fb, state, 0));
	desc->width = cpu_to_le16(state->dst.x2 - state->dst.x1 - 1);
	desc->height = cpu_to_le16(state->dst.y2 - state->dst.y1 - 1);
	desc->x = cpu_to_le16(0);
	desc->y = cpu_to_le16(0);
	desc->pitch = cpu_to_le16(state->fb->pitches[0] / state->fb->format->char_per_block[0]);
	desc->flags = 0; // ???5:6:6 ARGB - ???
	desc->alpha = 0xFF;

	// ??? pr_info("*** osd next = 0x%08X\n", (unsigned)le32_to_cpu(desc->next));
	// ??? pr_info("*** osd image_base = 0x%08X\n", (unsigned)le32_to_cpu(desc->image_base));
	// ??? pr_info("*** osd width = %u\n", (unsigned)le16_to_cpu(desc->width));
	// ??? pr_info("*** osd height = %u\n", (unsigned)le16_to_cpu(desc->height));
	// ??? pr_info("*** osd x = %u\n", (unsigned)le16_to_cpu(desc->x));
	// ??? pr_info("*** osd y = %u\n", (unsigned)le16_to_cpu(desc->y));
	// ??? pr_info("*** osd pitch = %u\n", (unsigned)le16_to_cpu(desc->pitch));
	// ??? pr_info("*** osd flags = 0x%08X\n", (unsigned)desc->flags);
	// ??? pr_info("*** osd alpha = %u\n", (unsigned)desc->alpha);
}

static struct drm_framebuffer *fb_create(
	struct drm_device *dev, struct drm_file *file, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	bool pixel_format_found;
	const struct drm_format_info *format;
	int i;

	// check format
	pixel_format_found = false;
	for (i = 0; i < ARRAY_SIZE(plane_formats); ++i) {
		if (plane_formats[i] == mode_cmd->pixel_format) {
			pixel_format_found = true;
			break;
		}
	}
	if (!pixel_format_found)
		return ERR_PTR(-EINVAL);

	// check pitch
	format = drm_format_info(mode_cmd->pixel_format);
	if (mode_cmd->pitches[0] / format->char_per_block[0] > MAX_BIT_VALUE(HARDWARE_PITCH_BITS))
		return ERR_PTR(-EINVAL);

	return drm_gem_fb_create(dev, file, mode_cmd);
}

static int plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_crtc_state *crtc_state;

	crtc_state = drm_atomic_get_new_crtc_state(plane_state->state, &vdu->crtc);

	return drm_atomic_helper_check_plane_state(plane_state, crtc_state,
		DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING, false, true);
}

// It is an empty function.
// All work is done in the crtc_atomic_flush function, but the function must be present
// (the DRM framework expectes a non-null pointer to the function).
static void plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
}

static int crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct vdu_device *vdu = drm_dev_to_vdu(crtc->dev);
	unsigned long irq_flags;
	u32 val;

	spin_lock_irqsave(&vdu->reg_lock, irq_flags);
	write_vdu_reg(vdu, VDU_REG_INT_STAT, VDU_REG_INT_VDU_STAT_SA_MASK); // clear pending interrupt
	val = read_vdu_reg(vdu, VDU_REG_INT_ENA);
	val |= VDU_REG_INT_VDU_STAT_SA_MASK;
	write_vdu_reg(vdu, VDU_REG_INT_ENA, val);
	spin_unlock_irqrestore(&vdu->reg_lock, irq_flags);

	pr_info("*** crtc_enable_vblank\n"); // ???

	return 0;
}

static void crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct vdu_device *vdu = drm_dev_to_vdu(crtc->dev);
	unsigned long irq_flags;
	u32 val;

	spin_lock_irqsave(&vdu->reg_lock, irq_flags);
	val = read_vdu_reg(vdu, VDU_REG_INT_ENA);
	val &= ~VDU_REG_INT_VDU_STAT_SA_MASK;
	write_vdu_reg(vdu, VDU_REG_INT_ENA, val);
	spin_unlock_irqrestore(&vdu->reg_lock, irq_flags);

	pr_info("*** crtc_disable_vblank\n"); // ???
}

static enum drm_mode_status crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	if ((mode->flags & DRM_MODE_FLAG_INTERLACE))
		return MODE_NO_INTERLACE; // [***]

	if (drm_mode_vrefresh(mode) < 25)
		return MODE_BAD;

	if (drm_mode_vrefresh(mode) > 60)
		return MODE_BAD;

	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (mode->clock >  148500)
		return MODE_CLOCK_HIGH;

	if (mode->htotal - mode->hdisplay - 1 < 4 + 4)
		return MODE_HBLANK_NARROW;
	if (mode->htotal - mode->hdisplay - 1 > MAX_BIT_VALUE(VDU_REG_DIF_BLANK_HBLANK_BITS))
		return MODE_HBLANK_WIDE;

	if (mode->vtotal - mode->vdisplay > MAX_BIT_VALUE(VDU_REG_DIF_BLANK_VBLANK_BEG_BITS))
		return MODE_VBLANK_WIDE;

	if (mode->htotal > MAX_BIT_VALUE(VDU_REG_DIF_FSIZE_HTOTAL_BITS))
		return MODE_VIRTUAL_X;

	if (mode->vtotal > MAX_BIT_VALUE(VDU_REG_DIF_FSIZE_VTOTAL_BITS))
		return MODE_VIRTUAL_Y;

	if (mode->hsync_start - mode->hdisplay > MAX_BIT_VALUE(VDU_REG_DIF_HSYNC_START_BITS))
		return MODE_HSYNC;

	if (mode->hsync_end - mode->hsync_start > MAX_BIT_VALUE(VDU_REG_DIF_HSYNC_LEN_BITS))
		return MODE_HSYNC;

	if (mode->htotal - mode->hsync_end > MAX_BIT_VALUE(VDU_REG_DIF_HSYNC_DELAY_BITS))
		return MODE_HSYNC;

	if (mode->vsync_start - mode->vdisplay > MAX_BIT_VALUE(VDU_REG_DIF_VSYNC_START_BITS))
		return MODE_VSYNC;

	if (mode->vsync_end - mode->vsync_start > MAX_BIT_VALUE(VDU_REG_DIF_VSYNC_LEN_BITS))
		return MODE_VSYNC;

	return MODE_OK;
}

static void crtc_atomic_flush(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(crtc->dev);
	dma_addr_t phys_addr;
	u32 val;

	pr_info("*** crt_atomic_flush\n"); // ???

	if (crtc->state->mode_changed)
		set_crtc_mode(vdu, &crtc->state->mode);

	if (vdu->primary_plane.state->fb != NULL) {
		fill_osd_descriptor(vdu, &vdu->dma_data->osd_area_descs[vdu->osd_area_desc_free_index]); // ??? need fill_osd_descriptor
		phys_addr = vdu->dma_data_phys_addr + offsetof(struct dma_data, osd_area_descs)
			+ vdu->osd_area_desc_free_index * sizeof(struct osd_area_desc);
		vdu->osd_area_desc_free_index ^= 1; // flip next index

		// [***] actually we mustnot do it, but we do
		val = read_vdu_reg(vdu, VDU_REG_OSD_CTRL);
		val |= VDU_REG_OSD_CTRL_ARGB_RGBA_MASK;
		write_vdu_reg(vdu, VDU_REG_OSD_CTRL, val);
		write_vdu_reg(vdu, VDU_REG_OSD_BASE0, phys_addr);
		write_vdu_reg(vdu, VDU_REG_OSD_BASE1, phys_addr);

		spin_lock_irq(&vdu->reg_lock);
		val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
		val |= VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK;
		write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
		spin_unlock_irq(&vdu->reg_lock);

	} else {
		spin_lock_irq(&vdu->reg_lock);
		val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
		val &= ~(VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK);
		write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
		spin_unlock_irq(&vdu->reg_lock);
	}

	// if the event was not handled before it must be handled here
	if (crtc->state->event) {
		pr_info("*** crtc_atomic_flush event 0x%08X\n", (unsigned)crtc->state->event); // ???
		spin_lock_irq(&crtc->dev->event_lock);
		vdu->pending_vblank_event = crtc->state->event;
		crtc->state->event = NULL;
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static void crtc_atomic_enable(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(crtc->dev);
	u32 val;

	spin_lock_irq(&vdu->reg_lock);
	val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
	val |= VDU_REG_CTRL_ENA_VDU_ENA_MASK;
	write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
	spin_unlock_irq(&vdu->reg_lock);

	drm_crtc_vblank_on(&vdu->crtc);

	pr_info("*** crtc_atomic_enable\n"); // ???
}

static void crtc_atomic_disable(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(crtc->dev);
	u32 val;
	int ret;

	WARN_ON((read_vdu_reg(vdu, VDU_REG_CTRL_ENA) & VDU_REG_CTRL_ENA_VDU_ENA_MASK) == 0);

	drm_crtc_vblank_off(&vdu->crtc);

	if ((read_vdu_reg(vdu, VDU_REG_CTRL_ENA) & VDU_REG_CTRL_ENA_VDU_ENA_MASK) == 0)
		return;

	reinit_completion(&vdu->vdu_off_done);

	spin_lock_irq(&vdu->reg_lock);

	// clear and enable VDU off interrupt
	write_vdu_reg(vdu, VDU_REG_INT_STAT, VDU_REG_INT_VDU_ENA_MASK);
	val = read_vdu_reg(vdu, VDU_REG_INT_ENA);
	val |= VDU_REG_INT_VDU_ENA_MASK;
	write_vdu_reg(vdu, VDU_REG_INT_ENA, val);

	// switch VDU off
	val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
	val &= ~(VDU_REG_CTRL_ENA_VDU_ENA_MASK | VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK);
	write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);

	spin_unlock_irq(&vdu->reg_lock);

	ret = wait_for_completion_timeout(&vdu->vdu_off_done, HZ / 5);
	if (ret == 0)
		dev_err(&vdu->pdev->dev, "vdu off error\n");

	// disable VDU off intettupt
	spin_lock_irq(&vdu->reg_lock);
	val = read_vdu_reg(vdu, VDU_REG_INT_ENA);
	val &= ~VDU_REG_INT_VDU_ENA_MASK;
	write_vdu_reg(vdu, VDU_REG_INT_ENA, val);
	spin_unlock_irq(&vdu->reg_lock);

	// in case the apadter has been disabled all the resources can be released
	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	pr_info("*** crtc_atomic_disable\n"); // ???
}

static irqreturn_t irq_handler(int irq, void *arg)
{
	struct drm_device *drm_dev = arg;
	struct vdu_device *vdu = drm_dev_to_vdu(drm_dev);
	u32 irq_status;

	pr_info("*** %lu %u irq 0x%08X 0x%08X 0x%08X\n", jiffies, HZ, read_vdu_reg(vdu, VDU_REG_INT_STAT), read_vdu_reg(vdu, VDU_REG_INT_ENA),
		read_vdu_reg(vdu, VDU_REG_CTRL_ENA)); // ???

	spin_lock(&vdu->reg_lock);
	irq_status = read_vdu_reg(vdu, VDU_REG_INT_STAT);
	irq_status &= read_vdu_reg(vdu, VDU_REG_INT_ENA);
	write_vdu_reg(vdu, VDU_REG_INT_STAT, irq_status);
	spin_unlock(&vdu->reg_lock);

	pr_info("*** stat = 0x%08X\n", read_vdu_reg(vdu, VDU_REG_INT_STAT)); // ???

	if ((irq_status & VDU_REG_INT_VDU_STAT_SA_MASK) != 0) {
		drm_crtc_handle_vblank(&vdu->crtc);

		if (vdu->pending_vblank_event) {
			spin_lock(&vdu->crtc.dev->event_lock);
			drm_crtc_send_vblank_event(&vdu->crtc, vdu->pending_vblank_event);
			vdu->pending_vblank_event = NULL;
			spin_unlock(&vdu->crtc.dev->event_lock);
		}
	}

	if ((irq_status & VDU_REG_INT_VDU_ENA_MASK) != 0)
		complete_all(&vdu->vdu_off_done);

	return IRQ_HANDLED;
}

static void mode_config_init(struct vdu_device *vdu)
{
	struct drm_device *dev = &vdu->drm_dev;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 320;
	dev->mode_config.min_height = 200;
	dev->mode_config.max_width = 1920;
	dev->mode_config.max_height = 1080;
	// ??? dev->mode_config.preferred_depth = 16; // ???
	// ??? dev->mode_config.prefer_shadow = 0; // ???
	dev->mode_config.funcs = &mode_config_funcs;

	// it is needed for correct working DRM_IOCTL_MODE_ADDFB2
	dev->mode_config.quirk_addfb_prefer_host_byte_order = true;
}

static int crtc_init(struct vdu_device *vdu)
{
	int ret;

	ret = drm_universal_plane_init(
		&vdu->drm_dev,
		&vdu->primary_plane,
		0,
		&plane_funcs,
		plane_formats,
		ARRAY_SIZE(plane_formats),
		plane_format_modifiers,
		DRM_PLANE_TYPE_PRIMARY,
		NULL);
	if (ret != 0)
		return ret;
	drm_plane_helper_add(&vdu->primary_plane, &plane_helper_funcs);

	ret = drm_crtc_init_with_planes(
		&vdu->drm_dev,
		&vdu->crtc,
		&vdu->primary_plane,
		NULL, // cursor plane
		&crtc_funcs,
		NULL); // name
	if (ret != 0)
		return ret;
	drm_crtc_helper_add(&vdu->crtc, &crtc_helper_funcs);

	// allow connection the primary plane with CRTC
	vdu->primary_plane.possible_crtcs = drm_crtc_mask(&vdu->crtc);

	return 0;
}

static int encoder_init(struct vdu_device *vdu)
{
	struct device_node *encoder_node;
	struct drm_bridge *bridge;
	int ret;

	encoder_node = of_graph_get_remote_node(vdu->pdev->dev.of_node, 0, 0);
	if (encoder_node == NULL) {
		dev_err(&vdu->pdev->dev, "encoder not found\n");
		return -ENODEV;
	}

	bridge = of_drm_find_bridge(encoder_node);
	of_node_put(encoder_node);
	if (!bridge)
		return -EPROBE_DEFER;

	ret = drm_encoder_init(&vdu->drm_dev, &vdu->encoder, &encoder_funcs, DRM_MODE_ENCODER_TMDS, NULL);
	if (ret != 0)
		return ret;

	ret = drm_bridge_attach(&vdu->encoder, bridge, NULL);
	if (ret)
		return ret;

	// allow connection with CRTC
	vdu->encoder.possible_crtcs = drm_crtc_mask(&vdu->crtc);
	vdu->encoder.possible_clones = drm_encoder_mask(&vdu->encoder);

	return 0;
}

static void cleanup(struct platform_device *pdev)
{
	struct vdu_device *vdu = platform_get_drvdata(pdev);
	if (vdu == NULL)
		return;

	if (vdu->drm_dev.registered)
		drm_dev_unregister(&vdu->drm_dev);

	if (vdu->drm_dev.irq_enabled)
		drm_irq_uninstall(&vdu->drm_dev);

	drm_kms_helper_poll_fini(&vdu->drm_dev);
	drm_mode_config_cleanup(&vdu->drm_dev);
	drm_dev_fini(&vdu->drm_dev);
}

static int probe_internal(struct platform_device *pdev)
{
	struct vdu_device *vdu;
	struct device_node *syscon_node;
	u32 hardware_id;
	int irq;
	int ret;

	// setup dma -offset for bus
	pdev->dev.archdata.dma_offset = -(pdev->dev.dma_pfn_offset << PAGE_SHIFT);

	vdu = devm_kzalloc(&pdev->dev, sizeof(*vdu), GFP_KERNEL);
	if (vdu == NULL)
		return -ENOMEM;

	vdu->pdev = pdev;
	platform_set_drvdata(pdev, vdu);

	spin_lock_init(&vdu->reg_lock);
	init_completion(&vdu->vdu_off_done);

	vdu->regs = devm_ioremap_resource(&pdev->dev, pdev->resource);
	if (IS_ERR(vdu->regs)) {
		dev_err(&vdu->pdev->dev, "cannot remap io register memory");
		return PTR_ERR(vdu->regs);
	}
	vdu->regs_size = pdev->resource->end - pdev->resource->start;

	hardware_id = read_vdu_reg(vdu, VDU_REG_CTRL_VDU_ID);
	if (hardware_id != 0xEBEBAB01) {
		dev_err(&vdu->pdev->dev, "unknown hardware id 0x%08X", hardware_id);
		return -ENODEV;
	}

	vdu->dma_data = dmam_alloc_coherent(&pdev->dev, sizeof *(vdu->dma_data), &vdu->dma_data_phys_addr, GFP_KERNEL);
	if (vdu->dma_data == NULL)
		return -ENOMEM;

	ret = of_property_read_u32(pdev->dev.of_node, "vdu-index", &vdu->vdu_index);
	if (ret != 0) {
		dev_err(&pdev->dev, "missing required parameter 'vdu-index'\n");
		return -ENODEV;
	}
	if (vdu->vdu_index > 1) {
		dev_err(&pdev->dev, "parameter 'vdu-index' must be from 0 to 1\n");
		return -ENODEV;
	}

	// setup PLL control
	syscon_node = of_parse_phandle(pdev->dev.of_node, "crg-control", 0);
	if (!syscon_node) {
		dev_err(&pdev->dev, "failed to find PLL control register reference\n");
		return -EINVAL;
	}
	vdu->pll_control = syscon_node_to_regmap(syscon_node);
	of_node_put(syscon_node);
	if (IS_ERR(vdu->pll_control)) {
		dev_err(&pdev->dev, "cant remap PLL control register reference\n");
		return PTR_ERR(vdu->pll_control);
	}
	ret = init_pll(vdu);
	if (ret != 0) {
		dev_err(&pdev->dev, "PLL initialization error (%i)\n", ret);
		return ret;
	}

	ret = reset_vdu(vdu);
	if (ret != 0)
		return ret;

	ret = drm_dev_init(&vdu->drm_dev, &rcm_vdu_drm_driver, &pdev->dev);
	if (ret != 0)
		return ret;

	mode_config_init(vdu);

	ret = crtc_init(vdu);
	if (ret != 0)
		return ret;

	ret = drm_vblank_init(&vdu->drm_dev, 1);
	if (ret != 0)
		return ret;

	ret = encoder_init(vdu);
	if (ret != 0)
		return ret;

	drm_mode_config_reset(&vdu->drm_dev);
	drm_kms_helper_poll_init(&vdu->drm_dev);
	drm_crtc_vblank_off(&vdu->crtc);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to find device irq\n");
		return -EINVAL;
	};
	vdu->drm_dev.dev_private = vdu; // needs for drm_irq_install
	ret = drm_irq_install(&vdu->drm_dev, irq);
	if (ret != 0)
		return ret;

	ret = drm_dev_register(&vdu->drm_dev, 0);
	if (ret != 0)
		return ret;

	// ??? ret = drm_fbdev_generic_setup(&vdu->drm_dev, vdu->drm_dev.mode_config.preferred_depth);
	// ??? if (ret != 0)
	// ??? 	return ret;

	return 0;
}

static int rcm_vdu_probe(struct platform_device *pdev)
{
	int ret;

	ret = probe_internal(pdev);
	if (ret == 0)
		dev_info(&pdev->dev, "probed successfully");
	else
		cleanup(pdev);

	return ret;
}

static int rcm_vdu_remove(struct platform_device *pdev)
{
	cleanup(pdev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rcm_vdu_dt_ids[] = {
	{ .compatible = "rcm,vdu-drm" },
	{ } // sentinel
};
MODULE_DEVICE_TABLE(of, coda_dt_ids);
#endif

static struct platform_driver rcm_vdu_driver = {
	.probe	= rcm_vdu_probe,
	.remove	= rcm_vdu_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(rcm_vdu_dt_ids)
	}
};

module_platform_driver(rcm_vdu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_DESCRIPTION("RCM VDU DRM driver");
