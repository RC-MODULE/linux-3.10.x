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
#include <drm/drm_damage_helper.h>

#include "drm-rcm-vdu-reg.h"
#include "drm-rcm-vdu-pll-reg.h"

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
#include <linux/rcm-rmace.h>
#endif

#define DRIVER_NAME "rcm-vdu-drm"
#define DRIVER_DESC "rcm-vdu KMS driver"
#define DRIVER_DATE "2020"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

#define BACKGROUND_RGB 0x000000 // 0xFFFF00 (yellow) for testing

#define MAX_WIDTH 1920
#define MAX_HEIGHT 1080

#define MIN_CURSOR_WIDTH 8
#define MIN_CURSOR_HEIGHT 8
#define MAX_CURSOR_WIDTH 256
#define MAX_CURSOR_HEIGHT 256

#define HARDWARE_PITCH_BITS 13
#define SHADOW_PITCH MAX_WIDTH // in pixels

#define MAX_BIT_VALUE(bits) ((1U << (bits)) - 1U)

enum color_space {
	COLOR_SPACE_YUV444,
	COLOR_SPACE_YUV422
};

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
	struct osd_area_desc primary_osd_descs[2];
	struct osd_area_desc secondary_osd_descs[2];
	u8 screen_buffer[MAX_WIDTH * MAX_HEIGHT * 4]; // 4 for ARGB
#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	u64 rmace_control_block[6];
#endif
};

struct vdu_device {
	struct drm_device drm_dev;
	struct platform_device *pdev;

	void __iomem *regs;
	unsigned regs_size;
	spinlock_t reg_lock;

	u32 vdu_index; // for PLL setup
	struct regmap *pll_control;
	enum color_space output_color_space;
	u32 max_width;
	u32 max_height;

	struct dma_data *dma_data;
	dma_addr_t dma_data_phys_addr;

	struct drm_crtc crtc;
	struct drm_plane primary_plane;
	struct drm_plane cursor_plane;
	struct drm_plane overlay_plane;
	struct drm_encoder encoder;

	unsigned primary_osd_free_index;
	u32 primary_osd_format; // 0 if primary plane osd rendering is off
	u32 primary_osd_fb_phys_addr; // 0 if primary plane osd rendering is off

	unsigned secondary_osd_free_index;

	struct drm_pending_vblank_event *pending_vblank_event;
	struct completion vdu_off_done;

	spinlock_t mvl_setting_lock;
	bool mvl_setting_present;
	u32 mvl_setting_mvl_ctrl;
	u32 mvl_setting_mvl_y_base;
	u32 mvl_setting_mvl_cb_base;
	u32 mvl_setting_mvl_cr_base;
	u32 mvl_setting_mvl_y_size;
	u32 mvl_setting_mvl_c_size;
	u32 mvl_setting_mvl_full_size;
	u32 mvl_setting_dif_mvl_start;
	u32 mvl_setting_scaler_ctrl;
	u32 mvl_setting_scaler_sch_y;
	u32 mvl_setting_scaler_scv_y;
	u32 mvl_setting_scaler_sch_c;
	u32 mvl_setting_scaler_scv_c;
	u32 mvl_setting_scaler_size_y;
	u32 mvl_setting_scaler_size_c;
	u32 mvl_setting_scaler_size_cut;

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	struct rcm_rmace_ctx rmace_ctx;
	struct rcm_rmace_hw_desc rmace_src_descs[MAX_HEIGHT + 1];
	struct rcm_rmace_hw_desc rmace_dst_descs[MAX_HEIGHT];
	struct completion rmace_completion;
#endif
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

static const uint32_t primary_plane_formats[] = {
	DRM_FORMAT_RGB565, // native
	DRM_FORMAT_RGB565 | DRM_FORMAT_BIG_ENDIAN, // shadow
	DRM_FORMAT_XRGB8888, // shadow
	DRM_FORMAT_BGRX8888 // shadow
};

static const uint64_t primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static const uint32_t cursor_plane_formats[] = {
	DRM_FORMAT_ARGB8888 // native
};

static const uint64_t cursor_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static const uint32_t overlay_plane_formats[] = {
	DRM_FORMAT_YUV420 // native
};

static const uint64_t overlay_plane_format_modifiers[] = {
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

static int primary_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state);
static void primary_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state);

static const struct drm_plane_helper_funcs primary_plane_helper_funcs = {
	.atomic_check = primary_plane_atomic_check,
	.atomic_update = primary_plane_atomic_update
};

static int cursor_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state);
static void cursor_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state);

static const struct drm_plane_helper_funcs cursor_plane_helper_funcs = {
	.atomic_check = cursor_plane_atomic_check,
	.atomic_update = cursor_plane_atomic_update
};

static int overlay_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state);
static void overlay_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state);

static const struct drm_plane_helper_funcs overlay_plane_helper_funcs = {
	.atomic_check = overlay_plane_atomic_check,
	.atomic_update = overlay_plane_atomic_update
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
static struct y_cb_cr_color rgb_to_y_cb_cr(u32 rgb)
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
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_MI_AXI_OSD_PARAM, 0x0F310000);

	return 0;
}

// VDU must be disabled before the call
static void set_crtc_mode(struct vdu_device *vdu, struct drm_display_mode *mode)
{
	struct y_cb_cr_color background;
	u32 val;
	int ret;

	WARN_ON((read_vdu_reg(vdu, VDU_REG_CTRL_ENA) & VDU_REG_CTRL_ENA_VDU_ENA_MASK) != 0);

	ret = setup_pll_pixel_clock(vdu, mode->clock * 1000);
	if (ret != 0)
		dev_err(&vdu->pdev->dev, "PLL setup error (%i)\n", ret);

	val = (0x1 << VDU_REG_SCALER_FLT_Y_C_H_SHIFT)
		| (0x1 << VDU_REG_SCALER_FLT_Y_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_Y_C0, val);
	val = (0xFE << VDU_REG_SCALER_FLT_Y_C_H_SHIFT)
		| (0xFE << VDU_REG_SCALER_FLT_Y_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_Y_C1, val);
	val = (0x1 << VDU_REG_SCALER_FLT_Y_C_H_SHIFT)
		| (0x1 << VDU_REG_SCALER_FLT_Y_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_Y_C2, val);
	val = (0x0 << VDU_REG_SCALER_FLT_Y_C_H_SHIFT)
		| (0x0 << VDU_REG_SCALER_FLT_Y_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_Y_C3, val);
	val = (0 << VDU_REG_SCALER_FLT_Y_C_H_SHIFT)
		| (0 << VDU_REG_SCALER_FLT_Y_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_Y_C4, val);
	val = (0xFF << VDU_REG_SCALER_FLT_NORM_HFLT_Y_N_SHIFT)
		| (0xFF << VDU_REG_SCALER_FLT_NORM_VFLT_Y_N_SHIFT)
		| (0xFF << VDU_REG_SCALER_FLT_NORM_HFLT_C_N_SHIFT)
		| (0xFF << VDU_REG_SCALER_FLT_NORM_VFLT_C_N_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_NORM, val);
	val = (0x1 << VDU_REG_SCALER_FLT_C_C_H_SHIFT)
		| (0x1 << VDU_REG_SCALER_FLT_C_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_C_C0, val);
	val = (0xFE << VDU_REG_SCALER_FLT_C_C_H_SHIFT)
		| (0xFE << VDU_REG_SCALER_FLT_C_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_C_C1, val);
	val = (0x1 << VDU_REG_SCALER_FLT_C_C_H_SHIFT)
		| (0x1 << VDU_REG_SCALER_FLT_C_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_C_C2, val);
	val = (0x0 << VDU_REG_SCALER_FLT_C_C_H_SHIFT)
		| (0x0 << VDU_REG_SCALER_FLT_C_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_C_C3, val);
	val = (0 << VDU_REG_SCALER_FLT_C_C_H_SHIFT)
		| (0 << VDU_REG_SCALER_FLT_C_C_V_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_FLT_C_C4, val);
	write_vdu_reg(vdu, VDU_REG_SCALER_PHS_CUT_V, 0);
	write_vdu_reg(vdu, VDU_REG_SCALER_PHS_CUT_H, 0);
	val = (0x0 << VDU_REG_SCALER_MVL_CLR_Y_Y_SHIFT)
		| (0x0 << VDU_REG_SCALER_MVL_CLR_Y_CB_SHIFT)
		| (0x0 << VDU_REG_SCALER_MVL_CLR_Y_CR_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_MVL_CLR_Y, val);
	val = (0x0 << VDU_REG_SCALER_MVL_CLR_CB_Y_SHIFT)
		| (0x100 << VDU_REG_SCALER_MVL_CLR_CB_CB_SHIFT)
		| (0x0 << VDU_REG_SCALER_MVL_CLR_CB_CR_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_MVL_CLR_CB, val);
	val = (0x0 << VDU_REG_SCALER_MVL_CLR_CR_Y_SHIFT)
		| (0x0 << VDU_REG_SCALER_MVL_CLR_CR_CB_SHIFT)
		| (0x100 << VDU_REG_SCALER_MVL_CLR_CR_CR_SHIFT);
	write_vdu_reg(vdu, VDU_REG_SCALER_MVL_CLR_CR, val);

	val = VDU_REG_DIF_CTRL_HDTV_MASK
		| VDU_REG_DIF_CTRL_EXT_SYNC_EN_MASK
		| VDU_REG_DIF_CTRL_HSYNC_P_MASK
		| VDU_REG_DIF_CTRL_VSYNC_P_MASK
		| VDU_REG_DIF_CTRL_CH_SEL;
	if (vdu->output_color_space == COLOR_SPACE_YUV444)
		val |= VDU_REG_DIF_CTRL_SDTV_FORM_MASK | VDU_REG_DIF_CTRL_444_MODE_MASK;
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

	val = (0x2F << VDU_REG_OSD_COLOR_Y_R_SHIFT)
		| (0x9D << VDU_REG_OSD_COLOR_Y_G_SHIFT)
		| (0x10 << VDU_REG_OSD_COLOR_Y_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_Y, val);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_COLOR_Y, val);

	val = (0x11A << VDU_REG_OSD_COLOR_CB_R_SHIFT)
		| (0x157 << VDU_REG_OSD_COLOR_CB_G_SHIFT)
		| (0x70 << VDU_REG_OSD_COLOR_CB_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_CB, val);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_COLOR_CB, val);

	val = (0x70 << VDU_REG_OSD_COLOR_CR_R_SHIFT)
		| (0x166 << VDU_REG_OSD_COLOR_CR_G_SHIFT)
		| (0x10a << VDU_REG_OSD_COLOR_CR_B_SHIFT);
	write_vdu_reg(vdu, VDU_REG_OSD_COLOR_CR, val);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_COLOR_CR, val);
}

static void fill_primary_osd_descriptor(struct vdu_device *vdu, struct osd_area_desc *desc, u32 format, u32 fb_phys_addr)
{
	struct drm_plane_state *state = vdu->primary_plane.state;

	u32 screen_phys_addr = vdu->dma_data_phys_addr + offsetof(struct dma_data,screen_buffer);
	u16 fb_pitch = state->fb->pitches[0] / state->fb->format->char_per_block[0];

	desc->next = 0;
	desc->width = cpu_to_le16(state->dst.x2 - state->dst.x1 - 1);
	desc->height = cpu_to_le16(state->dst.y2 - state->dst.y1 - 1);
	desc->x = cpu_to_le16(0);
	desc->y = cpu_to_le16(0);
	switch (format)
	{
	case DRM_FORMAT_RGB565:
		desc->image_base = cpu_to_le32(fb_phys_addr);
		desc->pitch = cpu_to_le16(fb_pitch);
		desc->flags = 1;
		break;
	case DRM_FORMAT_RGB565 | DRM_FORMAT_BIG_ENDIAN:
		desc->image_base = cpu_to_le32(screen_phys_addr);
		desc->pitch = cpu_to_le16(SHADOW_PITCH);
		desc->flags = 1;
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_BGRX8888:
		desc->image_base = cpu_to_le32(screen_phys_addr);
		desc->pitch = cpu_to_le16(SHADOW_PITCH);
		desc->flags = 0;
		break;
	default:
		dev_err(&vdu->pdev->dev, "error format\n");
		desc->image_base = cpu_to_le32(screen_phys_addr);
		desc->pitch = cpu_to_le16(SHADOW_PITCH);
		desc->flags = 1;
		break;
	}
	desc->alpha = 0xFF;
}

static void primary_osd_rendering_on(struct vdu_device *vdu, u32 format, u32 fb_phys_addr)
{
	dma_addr_t osd_desc_phys_addr;
	u32 val;

	if ((format == vdu->primary_osd_format)
		&& ((format != DRM_FORMAT_RGB565) || (fb_phys_addr == vdu->primary_osd_fb_phys_addr)))
	{
		return;
	}

	fill_primary_osd_descriptor(vdu, &vdu->dma_data->primary_osd_descs[vdu->primary_osd_free_index], format, fb_phys_addr);
	osd_desc_phys_addr = vdu->dma_data_phys_addr + offsetof(struct dma_data, primary_osd_descs)
		+ vdu->primary_osd_free_index * sizeof(struct osd_area_desc);
	vdu->primary_osd_free_index ^= 1; // flip next index

	// [***] actually we mustnot do it, but we do
	val = read_vdu_reg(vdu, VDU_REG_OSD_CTRL);
	val |= VDU_REG_OSD_CTRL_ARGB_RGBA_MASK;
	write_vdu_reg(vdu, VDU_REG_OSD_CTRL, val);
	write_vdu_reg(vdu, VDU_REG_OSD_BASE0, osd_desc_phys_addr);
	write_vdu_reg(vdu, VDU_REG_OSD_BASE1, osd_desc_phys_addr);

	spin_lock_irq(&vdu->reg_lock);
	val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
	val |= VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK;
	write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
	spin_unlock_irq(&vdu->reg_lock);

	vdu->primary_osd_format = format;
	vdu->primary_osd_fb_phys_addr = fb_phys_addr;
}

static void primary_osd_rendering_off(struct vdu_device *vdu)
{
	u32 val;

	spin_lock_irq(&vdu->reg_lock);
	val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
	val &= ~(VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK);
	write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
	spin_unlock_irq(&vdu->reg_lock);

	vdu->primary_osd_format = 0;
	vdu->primary_osd_fb_phys_addr = 0;
}

static void fill_secondary_osd_descriptor(struct vdu_device *vdu, struct osd_area_desc *desc)
{
	struct drm_plane_state *state = vdu->cursor_plane.state;
	struct drm_gem_cma_object *gem = drm_fb_cma_get_gem_obj(state->fb, 0);

	u32 fb_phys_addr = gem->paddr
		+ state->fb->offsets[0]
		+ (state->src.y1 >> 16) * state->fb->pitches[0]
		+ (state->src.x1 >> 16) * state->fb->format->char_per_block[0];

	desc->next = 0;
	desc->width = cpu_to_le16(state->dst.x2 - state->dst.x1 - 1);
	desc->height = cpu_to_le16(state->dst.y2 - state->dst.y1 - 1);
	desc->x = cpu_to_le16(state->dst.x1);
	desc->y = cpu_to_le16(state->dst.y1);
	desc->image_base = cpu_to_le32(fb_phys_addr);
	desc->pitch = cpu_to_le16(state->fb->pitches[0] / state->fb->format->char_per_block[0]);
	desc->flags = 0;
	desc->alpha = 0xFF;
}

static void secondary_osd_rendering_on(struct vdu_device *vdu)
{
	dma_addr_t osd_desc_phys_addr;
	u32 val;

	fill_secondary_osd_descriptor(vdu, &vdu->dma_data->secondary_osd_descs[vdu->secondary_osd_free_index]);
	osd_desc_phys_addr = vdu->dma_data_phys_addr + offsetof(struct dma_data, secondary_osd_descs)
		+ vdu->secondary_osd_free_index * sizeof(struct osd_area_desc);
	vdu->secondary_osd_free_index ^= 1; // flip next index

	// [***] actually we mustnot do it, but we do
	val = read_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_CTRL);
	val |= VDU_REG_OSD_CTRL_ARGB_RGBA_MASK;
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_CTRL, val);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_BASE0, osd_desc_phys_addr);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_OSD_BASE1, osd_desc_phys_addr);

	// no lock is needed
	val = read_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA);
	val |= VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK;
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA, val);
}

static void secondary_osd_rendering_off(struct vdu_device *vdu)
{
	u32 val;

	// no lock is needed
	val = read_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA);
	val &= ~(VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA, val);
}

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
// src_buffer and dest_buffer are pointers to the physical memory
static void blit_rgb565_be(struct vdu_device *vdu, void *src_buffer, unsigned src_pitch, void *dest_buffer, unsigned dest_pitch, int dx, int dy)
{
	int y;
	u32 src_aligned = ((u32)src_buffer) & ~RCM_RMACE_HARDWARE_ALIGN_MASK;
	u32 dest_aligned = ((u32)dest_buffer) & ~RCM_RMACE_HARDWARE_ALIGN_MASK;
	u32 len_aligned = ((u32)src_buffer - src_aligned + dx * 2 + RCM_RMACE_HARDWARE_ALIGN_MASK) & ~RCM_RMACE_HARDWARE_ALIGN_MASK;

	for (y = 0; y < dy; ++y) {
		vdu->rmace_src_descs[y + 1].address = cpu_to_le32(src_aligned);
		vdu->rmace_src_descs[y + 1].data = cpu_to_le32(
			(y + 1 == dy ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (len_aligned << RCM_RMACE_DESC_LEN_SHIFT));
		src_aligned += src_pitch;

		vdu->rmace_dst_descs[y].address = cpu_to_le32(dest_aligned);
		vdu->rmace_dst_descs[y].data = cpu_to_le32(
			(y + 1 == dy ? RCM_RMACE_DESC_ACT0_MASK : 0)
			| RCM_RMACE_DESC_ACT2_MASK
			| (len_aligned << RCM_RMACE_DESC_LEN_SHIFT));
		dest_aligned += dest_pitch;
	}
	vdu->rmace_ctx.src_desc_count = dy + 1;
	vdu->rmace_ctx.dst_desc_count = dy;

	reinit_completion(&vdu->rmace_completion);
	rcm_rmace_ctx_schedule(&vdu->rmace_ctx);
}
#else // CONFIG_DRM_RCM_VDU_USE_RMACE
static void blit_rgb565_be(void *src_buffer, unsigned src_pitch, void *dest_buffer, unsigned dest_pitch, int dx, int dy)
{
	int x;
	int y;
	u16 *src;
	u16 *dest;

	for (y = 0; y < dy; ++y) {
		src = src_buffer;
		dest = dest_buffer;
		for (x = 0; x < dx; ++x)
			*(dest++) = swab16(*(src++));
		src_buffer += src_pitch;
		dest_buffer += dest_pitch;
	}
}
#endif // CONFIG_DRM_RCM_VDU_USE_RMACE

static void blit_xrgb8888(void *src_buffer, unsigned src_pitch, void *dest_buffer, unsigned dest_pitch, int dx, int dy)
{
	int x;
	int y;
	u32 *src;
	u32 *dest;

	for (y = 0; y < dy; ++y) {
		src = src_buffer;
		dest = dest_buffer;
		for (x = 0; x < dx; ++x)
			*(dest++) = *(src++) | cpu_to_le32(0xFF000000);
		src_buffer += src_pitch;
		dest_buffer += dest_pitch;
	}
}

static void blit_bgrx8888(void *src_buffer, unsigned src_pitch, void *dest_buffer, unsigned dest_pitch, int dx, int dy)
{
	int x;
	int y;
	u32 *src;
	u32 *dest;

	for (y = 0; y < dy; ++y) {
		src = src_buffer;
		dest = dest_buffer;
		for (x = 0; x < dx; ++x)
			*(dest++) = swab32(*(src++)) | cpu_to_le32(0xFF000000);
		src_buffer += src_pitch;
		dest_buffer += dest_pitch;
	}
}

static void blit_if_necessary(struct vdu_device *vdu, struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct drm_gem_cma_object *gem;
	struct drm_rect rect;
	void *src_buffer;
	void *dest_buffer;
	unsigned src_pitch;
	unsigned dest_pitch;
	int dx;
	int dy;
	unsigned bpp;

	if (vdu->primary_osd_format == DRM_FORMAT_RGB565)
		return;
	
	if (!drm_atomic_helper_damage_merged(old_state, plane->state, &rect))
		return;

	bpp = plane->state->fb->format->char_per_block[0];
	gem = drm_fb_cma_get_gem_obj(plane->state->fb, 0);

	src_pitch = plane->state->fb->pitches[0];
	src_buffer = gem->vaddr + plane->state->fb->offsets[0]
		+ rect.y1 * src_pitch
		+ rect.x1 * bpp;

	dest_pitch = SHADOW_PITCH * bpp;
	dest_buffer = vdu->dma_data->screen_buffer
		+ rect.y1 * dest_pitch
		+ rect.x1 * bpp;

	dx = rect.x2 - rect.x1;
	dy = rect.y2 - rect.y1;

	switch (vdu->primary_osd_format)
	{
	case DRM_FORMAT_RGB565 | DRM_FORMAT_BIG_ENDIAN:
#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
		src_buffer = (void*)((u32)gem->paddr + plane->state->fb->offsets[0]
			+ rect.y1 * src_pitch
			+ rect.x1 * bpp);
		dest_buffer = (void*)((u32)vdu->dma_data_phys_addr
			+ offsetof(struct dma_data, screen_buffer)
			+ rect.y1 * dest_pitch
			+ rect.x1 * bpp);
		blit_rgb565_be(vdu, src_buffer, src_pitch, dest_buffer, dest_pitch, dx, dy);
#else
		blit_rgb565_be(src_buffer, src_pitch, dest_buffer, dest_pitch, dx, dy);
#endif
		break;
	case DRM_FORMAT_XRGB8888:
		blit_xrgb8888(src_buffer, src_pitch, dest_buffer, dest_pitch, dx, dy);
		break;
	case DRM_FORMAT_BGRX8888:
		blit_bgrx8888(src_buffer, src_pitch, dest_buffer, dest_pitch, dx, dy);
		break;
	}
}

static struct drm_framebuffer *fb_create(
	struct drm_device *dev, struct drm_file *file, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	bool pixel_format_found;
	const struct drm_format_info *format;
	unsigned plane_index;
	int i;

	// check format
	pixel_format_found = false;
	for (i = 0; i < ARRAY_SIZE(primary_plane_formats); ++i) {
		if (primary_plane_formats[i] == mode_cmd->pixel_format) {
			pixel_format_found = true;
			break;
		}
	}
	if (!pixel_format_found) {
		for (i = 0; i < ARRAY_SIZE(cursor_plane_formats); ++i) {
			if (cursor_plane_formats[i] == mode_cmd->pixel_format) {
				pixel_format_found = true;
				break;
			}
		}
	}
	if (!pixel_format_found) {
		for (i = 0; i < ARRAY_SIZE(overlay_plane_formats); ++i) {
			if (overlay_plane_formats[i] == mode_cmd->pixel_format) {
				pixel_format_found = true;
				break;
			}
		}
	}
	if (!pixel_format_found)
		return ERR_PTR(-EINVAL);

	// check pitch for each plane (in case for native mode)
	format = drm_format_info(mode_cmd->pixel_format);
	for (plane_index = 0; plane_index < format->num_planes; ++plane_index)
		if (mode_cmd->pitches[plane_index] / format->char_per_block[plane_index] > MAX_BIT_VALUE(HARDWARE_PITCH_BITS))
			return ERR_PTR(-EINVAL);
	// Cb/Cr plane pitches must be the same
	if ((format->num_planes == 3) && (mode_cmd->pitches[1] != mode_cmd->pitches[2]))
		return ERR_PTR(-EINVAL);

	return drm_gem_fb_create_with_dirty(dev, file, mode_cmd);
}

static int primary_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_crtc_state *crtc_state;

	crtc_state = drm_atomic_get_new_crtc_state(plane_state->state, &vdu->crtc);

	return drm_atomic_helper_check_plane_state(plane_state, crtc_state,
		DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING, false, true);
}

static void primary_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_plane_state *state = plane->state;
	u32 format;
	u32 fb_phys_addr;

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	// wait the transfer is finished
	wait_for_completion(&vdu->rmace_completion);
#endif

	if (state->visible) {
		format = state->fb->format->format;
		fb_phys_addr = drm_fb_cma_get_gem_addr(state->fb, state, 0);
		primary_osd_rendering_on(vdu, format, fb_phys_addr);
		blit_if_necessary(vdu, plane, old_state);
	} else
		primary_osd_rendering_off(vdu);
}

static int cursor_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_crtc_state *crtc_state;
	int ret;

	crtc_state = drm_atomic_get_new_crtc_state(plane_state->state, &vdu->crtc);

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
		DRM_PLANE_HELPER_NO_SCALING, DRM_PLANE_HELPER_NO_SCALING, true, true);
	if (ret != 0)
		return ret;

	if ((plane_state->visible)
		&& ((plane_state->crtc_w < MIN_CURSOR_WIDTH)
			|| (plane_state->crtc_w > MAX_CURSOR_WIDTH)
			|| (plane_state->crtc_h < MIN_CURSOR_HEIGHT)
			|| (plane_state->crtc_h > MAX_CURSOR_HEIGHT)))
	{
		return -EINVAL;
	}

	return 0;
}

static void cursor_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);

	if (plane->state->visible)
		secondary_osd_rendering_on(vdu);
	else
		secondary_osd_rendering_off(vdu);
}

static int overlay_plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *plane_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_crtc_state *crtc_state;
	int ret;
	unsigned src_width;
	unsigned src_height;
	unsigned dst_width;
	unsigned dst_height;
	unsigned hfactor;
	unsigned vfactor;

	crtc_state = drm_atomic_get_new_crtc_state(plane_state->state, &vdu->crtc);

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
		1, 256 << 16, true, true);
	if (ret != 0)
		return ret;

	if (plane_state->visible) {
		// test non-clipping coordinates
		if (((plane_state->src_x & 0xFFFF) != 0)
			|| ((plane_state->src_y & 0xFFFF) != 0)
			|| ((plane_state->src_w & 0xFFFF) != 0)
			|| ((plane_state->src_h & 0xFFFF) != 0))
		{
			return -ERANGE;
		}
		// cannot cross the screen edge
		if ((plane_state->crtc_x < 0)
			|| (plane_state->crtc_y < 0)
			|| (plane_state->crtc_x  + plane_state->crtc_w > crtc_state->mode.hdisplay)
			|| (plane_state->crtc_y  + plane_state->crtc_h > crtc_state->mode.vdisplay))
		{
			return -ERANGE;
		}
		src_width = plane_state->src_w >> 16;
		src_height = plane_state->src_h >> 16;
		dst_width = plane_state->crtc_w;
		dst_height = plane_state->crtc_h;
		if ((src_width < 2) || (src_height < 2) || (dst_width < 2) || (dst_height < 2))
			return -EINVAL;
		hfactor = ((src_width - 1) << 16) / (dst_width - 1);
		vfactor = ((src_height - 1) << 16) / (dst_height - 1);
		if ((hfactor > VDU_REG_SCALER_SCH_Y_MASK) || (vfactor > VDU_REG_SCALER_SCV_Y_MASK))
			return -ERANGE;
	}

	return 0;
}

static void overlay_plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct vdu_device *vdu = drm_dev_to_vdu(plane->dev);
	struct drm_plane_state *state = plane->state;
	u32 fb_y_pitch_in_px;
	u32 fb_cbcr_pitch_in_px;
	u32 src_width;
	u32 src_height;
	u32 dst_width;
	u32 dst_height;
	u32 val;

	if (state->visible) {
		src_width = state->src_w >> 16;
		src_height = state->src_h >> 16;
		dst_width = state->crtc_w;
		dst_height = state->crtc_h;
		fb_y_pitch_in_px = state->fb->pitches[0] / state->fb->format->char_per_block[0];
		fb_cbcr_pitch_in_px = state->fb->pitches[1] / state->fb->format->char_per_block[1];

		spin_lock_irq(&vdu->mvl_setting_lock);

		spin_lock(&vdu->reg_lock); // not spin_lock_irq
		val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
		val &= ~VDU_REG_CTRL_ENA_MVL_BASE_SW_ENA_MASK;
		write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
		spin_unlock(&vdu->reg_lock); // not spin_unlock_irq

		vdu->mvl_setting_mvl_ctrl = VDU_REG_MI_CTRL_PLANE_NUM_MASK;
		vdu->mvl_setting_mvl_y_base= drm_fb_cma_get_gem_addr(state->fb, state, 0);
		vdu->mvl_setting_mvl_cb_base = drm_fb_cma_get_gem_addr(state->fb, state, 1);
		vdu->mvl_setting_mvl_cr_base = drm_fb_cma_get_gem_addr(state->fb, state, 2);
		vdu->mvl_setting_mvl_y_size = ((src_width - 1) << VDU_REG_MI_MVL_Y_SIZE_H_SHIFT)
			| ((src_height - 1) << VDU_REG_MI_MVL_Y_SIZE_V_SHIFT);
		vdu->mvl_setting_mvl_c_size = ((src_width / 2 - 1) << VDU_REG_MI_MVL_C_SIZE_H_SHIFT)
			| ((src_height / 2 - 1) << VDU_REG_MI_MVL_C_SIZE_V_SHIFT);
		vdu->mvl_setting_mvl_full_size = (fb_y_pitch_in_px << VDU_REG_MI_MVL_FULL_SIZE_Y_SHIFT)
			| (fb_cbcr_pitch_in_px << VDU_REG_MI_MVL_FULL_SIZE_C_SHIFT);

		vdu->mvl_setting_dif_mvl_start = (state->dst.x1 << VDU_REG_DIF_MVL_START_H_SHIFT)
			| (state->dst.y1 << VDU_REG_DIF_MVL_START_V_SHIFT);

		vdu->mvl_setting_scaler_ctrl = 0;
		if (dst_width != src_width)
			vdu->mvl_setting_scaler_ctrl |= VDU_REG_SCALER_CTRL_H_SCALER_ENA_MASK;
		if (dst_width < src_width)
			vdu->mvl_setting_scaler_ctrl |= VDU_REG_SCALER_CTRL_H_FLT_ENA_MASK;
		if (dst_height != src_height)
			vdu->mvl_setting_scaler_ctrl |= VDU_REG_SCALER_CTRL_V_SCALER_ENA_MASK;
		if (dst_height < src_height)
			vdu->mvl_setting_scaler_ctrl |= VDU_REG_SCALER_CTRL_V_FLT_ENA_MASK;
		vdu->mvl_setting_scaler_sch_y = ((src_width - 1) << 16) / (dst_width - 1);
		vdu->mvl_setting_scaler_scv_y = ((src_height - 1) << 16) / (dst_height - 1);
		vdu->mvl_setting_scaler_sch_c = ((src_width / 2 - 1) << 16) / (dst_width / 2 - 1);
		vdu->mvl_setting_scaler_scv_c = ((src_height / 2 - 1) << 16) / (dst_height / 2 - 1);
		vdu->mvl_setting_scaler_size_y = ((dst_width - 1) << VDU_REG_SCALER_SIZE_Y_H_SHIFT)
			| ((dst_height - 1) << VDU_REG_SCALER_SIZE_Y_V_SHIFT);
		vdu->mvl_setting_scaler_size_c = ((dst_width / 2 - 1) << VDU_REG_SCALER_SIZE_C_H_SHIFT)
			| ((dst_height / 2 - 1) << VDU_REG_SCALER_SIZE_C_V_SHIFT);
		vdu->mvl_setting_scaler_size_cut = ((dst_width - 1) << VDU_REG_SCALER_SIZE_CUT_H_SHIFT)
			| ((dst_height - 1) << VDU_REG_SCALER_SIZE_CUT_V_SHIFT);

		vdu->mvl_setting_present = true;

		spin_unlock_irq(&vdu->mvl_setting_lock);
	} else {
		spin_lock_irq(&vdu->mvl_setting_lock);

		spin_lock(&vdu->reg_lock); // not spin_lock_irq
		val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
		val &= ~(VDU_REG_CTRL_ENA_MVL_ENA_MASK | VDU_REG_CTRL_ENA_MVL_BASE_SW_ENA_MASK);
		write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
		spin_unlock(&vdu->reg_lock); // not spin_unlock_irq

		vdu->mvl_setting_present = false;

		spin_unlock_irq(&vdu->mvl_setting_lock);
	}
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

	if (crtc->state->mode_changed)
		set_crtc_mode(vdu, &crtc->state->mode);

	// if the event was not handled before it must be handled here
	if (crtc->state->event) {
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
	val = read_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA);
	val &= ~(VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK);
	write_vdu_reg(vdu, VDU_REG_SECOND_CORE_BASE + VDU_REG_CTRL_ENA, val);
	val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
	val &= ~(VDU_REG_CTRL_ENA_VDU_ENA_MASK
		| VDU_REG_CTRL_ENA_OSD_ENA_MASK | VDU_REG_CTRL_ENA_OSD_BASE_SW_ENA_MASK
		| VDU_REG_CTRL_ENA_MVL_ENA_MASK | VDU_REG_CTRL_ENA_MVL_BASE_SW_ENA_MASK);
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
}

static irqreturn_t irq_handler(int irq, void *arg)
{
	struct drm_device *drm_dev = arg;
	struct vdu_device *vdu = drm_dev_to_vdu(drm_dev);
	u32 irq_status;
	u32 val;

	spin_lock(&vdu->reg_lock);
	irq_status = read_vdu_reg(vdu, VDU_REG_INT_STAT);
	irq_status &= read_vdu_reg(vdu, VDU_REG_INT_ENA);
	write_vdu_reg(vdu, VDU_REG_INT_STAT, irq_status);
	spin_unlock(&vdu->reg_lock);

	if ((irq_status & VDU_REG_INT_VDU_STAT_SA_MASK) != 0) {
		spin_lock(&vdu->mvl_setting_lock);
		if (vdu->mvl_setting_present) {
			write_vdu_reg(vdu, VDU_REG_MI_CTRL, vdu->mvl_setting_mvl_ctrl);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_Y_BASE0, vdu->mvl_setting_mvl_y_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_CB_BASE0, vdu->mvl_setting_mvl_cb_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_CR_BASE0, vdu->mvl_setting_mvl_cr_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_Y_BASE1, vdu->mvl_setting_mvl_y_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_CB_BASE1, vdu->mvl_setting_mvl_cb_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_CR_BASE1, vdu->mvl_setting_mvl_cr_base);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_Y_SIZE, vdu->mvl_setting_mvl_y_size);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_C_SIZE, vdu->mvl_setting_mvl_c_size);
			write_vdu_reg(vdu, VDU_REG_MI_MVL_FULL_SIZE, vdu->mvl_setting_mvl_full_size);
			write_vdu_reg(vdu, VDU_REG_DIF_MVL_START, vdu->mvl_setting_dif_mvl_start);
			write_vdu_reg(vdu, VDU_REG_SCALER_CTRL, vdu->mvl_setting_scaler_ctrl);
			write_vdu_reg(vdu, VDU_REG_SCALER_SCH_Y, vdu->mvl_setting_scaler_sch_y);
			write_vdu_reg(vdu, VDU_REG_SCALER_SCV_Y, vdu->mvl_setting_scaler_scv_y);
			write_vdu_reg(vdu, VDU_REG_SCALER_SCH_C, vdu->mvl_setting_scaler_sch_c);
			write_vdu_reg(vdu, VDU_REG_SCALER_SCV_C, vdu->mvl_setting_scaler_scv_c);
			write_vdu_reg(vdu, VDU_REG_SCALER_SIZE_Y, vdu->mvl_setting_scaler_size_y);
			write_vdu_reg(vdu, VDU_REG_SCALER_SIZE_C, vdu->mvl_setting_scaler_size_c);
			write_vdu_reg(vdu, VDU_REG_SCALER_SIZE_CUT, vdu->mvl_setting_scaler_size_cut);
			spin_lock(&vdu->reg_lock);
			val = read_vdu_reg(vdu, VDU_REG_CTRL_ENA);
			val |= VDU_REG_CTRL_ENA_MVL_ENA_MASK | VDU_REG_CTRL_ENA_MVL_BASE_SW_ENA_MASK;
			write_vdu_reg(vdu, VDU_REG_CTRL_ENA, val);
			spin_unlock(&vdu->reg_lock);
			vdu->mvl_setting_present = false;
		}
		spin_unlock(&vdu->mvl_setting_lock);

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

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
void rmace_callback(struct rcm_rmace_ctx *ctx, void *arg)
{
	struct vdu_device *vdu = container_of(ctx, struct vdu_device, rmace_ctx);
	complete_all(&vdu->rmace_completion);
}
#endif


static void mode_config_init(struct vdu_device *vdu)
{
	struct drm_device *dev = &vdu->drm_dev;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = vdu->max_width;
	dev->mode_config.max_height = vdu->max_height;
	dev->mode_config.preferred_depth = 16;
	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.funcs = &mode_config_funcs;
	dev->mode_config.cursor_width = MAX_CURSOR_WIDTH;
	dev->mode_config.cursor_height = MAX_CURSOR_HEIGHT;

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
		primary_plane_formats,
		ARRAY_SIZE(primary_plane_formats),
		primary_plane_format_modifiers,
		DRM_PLANE_TYPE_PRIMARY,
		NULL);
	if (ret != 0)
		return ret;
	drm_plane_helper_add(&vdu->primary_plane, &primary_plane_helper_funcs);

	ret = drm_universal_plane_init(
		&vdu->drm_dev,
		&vdu->cursor_plane,
		0,
		&plane_funcs,
		cursor_plane_formats,
		ARRAY_SIZE(cursor_plane_formats),
		cursor_plane_format_modifiers,
		DRM_PLANE_TYPE_CURSOR,
		NULL);
	if (ret != 0)
		return ret;
	drm_plane_helper_add(&vdu->cursor_plane, &cursor_plane_helper_funcs);

	ret = drm_universal_plane_init(
		&vdu->drm_dev,
		&vdu->overlay_plane,
		0,
		&plane_funcs,
		overlay_plane_formats,
		ARRAY_SIZE(overlay_plane_formats),
		overlay_plane_format_modifiers,
		DRM_PLANE_TYPE_OVERLAY,
		NULL);
	if (ret != 0)
		return ret;
	drm_plane_helper_add(&vdu->overlay_plane, &overlay_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(
		&vdu->drm_dev,
		&vdu->crtc,
		&vdu->primary_plane,
		&vdu->cursor_plane,
		&crtc_funcs,
		NULL); // name
	if (ret != 0)
		return ret;
	drm_crtc_helper_add(&vdu->crtc, &crtc_helper_funcs);

	// allow connection the planes with CRTC
	vdu->primary_plane.possible_crtcs = drm_crtc_mask(&vdu->crtc);
	vdu->cursor_plane.possible_crtcs = drm_crtc_mask(&vdu->crtc);
	vdu->overlay_plane.possible_crtcs = drm_crtc_mask(&vdu->crtc);

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

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	// wait the transfer is finished
	wait_for_completion(&vdu->rmace_completion);
#endif

	if (vdu->drm_dev.registered)
		drm_dev_unregister(&vdu->drm_dev);

	if (vdu->drm_dev.irq_enabled)
		drm_irq_uninstall(&vdu->drm_dev);

	if (vdu->drm_dev.dev_private != NULL) {
		drm_kms_helper_poll_fini(&vdu->drm_dev);
		drm_mode_config_cleanup(&vdu->drm_dev);
		drm_dev_fini(&vdu->drm_dev);
	}
}

static int probe_internal(struct platform_device *pdev)
{
	struct vdu_device *vdu;
	struct device_node *syscon_node;
	const char *color_space_id;
	u32 hardware_id;
	int irq;
	int ret;

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	if (rcm_rmace_dev_single_ptr == NULL)
		return -EPROBE_DEFER;
#endif

	// setup dma -offset for bus
	pdev->dev.archdata.dma_offset = -(pdev->dev.dma_pfn_offset << PAGE_SHIFT);

	vdu = devm_kzalloc(&pdev->dev, sizeof(*vdu), GFP_KERNEL);
	if (vdu == NULL)
		return -ENOMEM;

	vdu->pdev = pdev;
	platform_set_drvdata(pdev, vdu);

	spin_lock_init(&vdu->reg_lock);
	spin_lock_init(&vdu->mvl_setting_lock);
	init_completion(&vdu->vdu_off_done);

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	init_completion(&vdu->rmace_completion);
	complete_all(&vdu->rmace_completion); // no transactions
	rcm_rmace_ctx_init(rcm_rmace_dev_single_ptr, &vdu->rmace_ctx);
	vdu->rmace_ctx.src_descs = vdu->rmace_src_descs;
	vdu->rmace_ctx.dst_descs = vdu->rmace_dst_descs;
	vdu->rmace_ctx.callback = rmace_callback;
#endif

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

#ifdef CONFIG_DRM_RCM_VDU_USE_RMACE
	vdu->rmace_src_descs[0].address = cpu_to_le32(vdu->dma_data_phys_addr + offsetof(struct dma_data, rmace_control_block));
	vdu->rmace_src_descs[0].data = cpu_to_le32(
		RCM_RMACE_DESC_VALID_MASK
		| RCM_RMACE_DESC_ACT2_MASK
		| ((sizeof vdu->dma_data->rmace_control_block) << RCM_RMACE_DESC_LEN_SHIFT));
	vdu->dma_data->rmace_control_block[0] = cpu_to_le64(
		(3ULL << RCM_RMACE_HEADER_TYPE_SHIFT)
		| (1ULL << RCM_RMACE_HEADER_MODE_SHIFT));
	vdu->dma_data->rmace_control_block[1] = cpu_to_le64(0x6745230100000000ULL);
#endif

	ret = of_property_read_u32(pdev->dev.of_node, "vdu-index", &vdu->vdu_index);
	if (ret != 0) {
		dev_err(&pdev->dev, "missing required parameter 'vdu-index'\n");
		return -ENODEV;
	}
	if (vdu->vdu_index > 1) {
		dev_err(&pdev->dev, "parameter 'vdu-index' must be from 0 to 1\n");
		return -ENODEV;
	}

	ret = of_property_read_string(pdev->dev.of_node, "output-color-space", &color_space_id);
	if (ret != 0) {
		dev_err(&pdev->dev, "missing required parameter 'output-color-space'\n");
		return -ENODEV;
	}
	if (strcmp(color_space_id, "yuv444") == 0)
		vdu->output_color_space = COLOR_SPACE_YUV444;
	else if (strcmp(color_space_id, "yuv422") == 0)
		vdu->output_color_space = COLOR_SPACE_YUV422;
	else {
		dev_err(&pdev->dev, "parameter 'output-color-space' must be yuv444 or yuv422\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "max-width", &vdu->max_width);
	if (ret != 0) {
		dev_err(&pdev->dev, "missing required parameter 'max-width'\n");
		return -ENODEV;
	}
	if (vdu->max_width > MAX_WIDTH) {
		dev_err(&pdev->dev, "parameter 'max-width' out of range\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "max-height", &vdu->max_height);
	if (ret != 0) {
		dev_err(&pdev->dev, "missing required parameter 'max-height'\n");
		return -ENODEV;
	}
	if (vdu->max_height > MAX_HEIGHT) {
		dev_err(&pdev->dev, "parameter 'max-height' out of range\n");
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
	vdu->drm_dev.dev_private = vdu; // needs for drm_irq_install and as 'initialized' flag

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
	ret = drm_irq_install(&vdu->drm_dev, irq);
	if (ret != 0)
		return ret;

	ret = drm_dev_register(&vdu->drm_dev, 0);
	if (ret != 0)
		return ret;

	ret = drm_fbdev_generic_setup(&vdu->drm_dev, 0);
	if (ret != 0)
		return ret;

	return 0;
}

static int rcm_vdu_probe(struct platform_device *pdev)
{
	int ret;

	ret = probe_internal(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "probe error (%i)\n", ret);
		cleanup(pdev);
		return ret;
	}

	dev_info(&pdev->dev, "probed successfully\n");

	return 0;
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
MODULE_DEVICE_TABLE(of, rcm_vdu_dt_ids);
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
