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
#include <linux/regmap.h>

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_encoder.h>
#include <drm/drm_bridge.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fb_helper.h>

#include "drm-rcm-vdu-reg.h"

#define DRIVER_NAME "rcm-vdu-drm"
#define DRIVER_DESC "rcm-vdu KMS driver"
#define DRIVER_DATE "2020"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

// ??????
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
// ????

// ???
#define RCM_VDU_MAX_PITCH 4096 /* ???? not used */

struct vdu_device {
	struct drm_device drm_dev;
	struct platform_device *pdev;

	void __iomem *regs;
	unsigned regs_size;

	u32 vdu_index; // for PLL setup
	struct regmap *pll_control;

	struct drm_crtc crtc;
	struct drm_plane primary_plane;
	bool primary_plane_initialized;
	struct drm_encoder encoder;
};

DEFINE_DRM_GEM_FOPS(rcm_vdu_fops);

static struct drm_driver rcm_vdu_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.fops = &rcm_vdu_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
};

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create, // ??? check format
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
	// ??? .reset = rcar_du_plane_reset,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	// ??? .atomic_duplicate_state = rcar_du_plane_atomic_duplicate_state,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	// ??? .atomic_destroy_state = rcar_du_plane_atomic_destroy_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	// ??? .atomic_set_property = rcar_du_plane_atomic_set_property,
	// ??? .atomic_get_property = rcar_du_plane_atomic_get_property,
};

static void plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state);

static const struct drm_plane_helper_funcs plane_helper_funcs = {
	// ??? .atomic_check = rcar_du_plane_atomic_check,
	.atomic_update = plane_atomic_update,
};

static const struct drm_crtc_funcs crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	// ??? .reset = rcar_du_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	// ??? .atomic_duplicate_state = rcar_du_crtc_atomic_duplicate_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	// ??? .atomic_destroy_state = rcar_du_crtc_atomic_destroy_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	// ??? .enable_vblank = rcar_du_crtc_enable_vblank,
	// ??? .disable_vblank = rcar_du_crtc_disable_vblank,
};

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	// ??? .atomic_check = rcar_du_crtc_atomic_check,
	// ??? .atomic_begin = rcar_du_crtc_atomic_begin,
	// ??? .atomic_flush = rcar_du_crtc_atomic_flush,
	// ??? .atomic_enable = rcar_du_crtc_atomic_enable,
	// ??? .atomic_disable = rcar_du_crtc_atomic_disable,
	// ??? .mode_valid = rcar_du_crtc_mode_valid,
};

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

// ???
static u32 read_vdu_reg(struct vdu_device *vdu, u32 offset)
{
	return ioread32(vdu->regs + offset);
}

static void write_vdu_reg(struct vdu_device *vdu, u32 offset, u32 data)
{
	iowrite32(data, vdu->regs + offset);
}

static int init_pll(struct vdu_device *dev)
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

// ???
// ??? comment
static int turn_off_vdu(struct vdu_device *vdu)
{
	// disable and clear all interrupts
	write_vdu_reg(vdu, VDU_REG_INT_ENA, 0);
	write_vdu_reg(vdu, VDU_REG_INT_STAT, 0xFFFFFFFF);

	// turn off the hardware
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

	return 0;
}
// ???

// ???
static int reset_vdu(struct vdu_device *vdu)
{
	unsigned i;
	int ret;

	ret = turn_off_vdu(vdu);
	if (ret != 0)
		return 0;

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

	return 0;
}
// ???

// ??? drmM functions

// ???
// ??? comment
static int set_crtc_mode(struct vdu_device *vdu, struct drm_display_mode *mode)
{
	u32 val;
	int ret;

	ret = turn_off_vdu(vdu);
	if (ret != 0)
		return 0;

	ret = setup_pll_pixel_clock(vdu,74250000); // ???
	if (ret != 0) {
		dev_err(&vdu->pdev->dev, "PLL setup error (%i)\n", ret);
		return ret;
	}

	val = VDU_REG_DIF_CTRL_EXT_SYNC_EN_MASK
		| VDU_REG_DIF_CTRL_SDTV_FORM_MASK
		| VDU_REG_DIF_CTRL_HSYNC_P_MASK
		| VDU_REG_DIF_CTRL_VSYNC_P_MASK
		| VDU_REG_DIF_CTRL_DIF_444_MODE_MASK
		| 0x1A; // ??? 1920x1080p 30 Hz
	write_vdu_reg(vdu, VDU_REG_DIF_CTRL, val);

	val = ((272 - 1) << VDU_REG_DIF_BLANK_HBLANK_SHIFT) // ???
		| (41 << VDU_REG_DIF_BLANK_VBLANK_BEG_SHIFT) // ???
		| (4 << VDU_REG_DIF_BLANK_VBLANK_END_SHIFT); // ???
	write_vdu_reg(vdu, VDU_REG_DIF_BLANK, val);

	val = (mode->crtc_htotal << VDU_REG_DIF_FSIZE_HTOTAL_SHIFT) // ???
		| (mode->crtc_vtotal << VDU_REG_DIF_FSIZE_VTOTAL_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_FSIZE, val);

	val = (mode->crtc_hdisplay << VDU_REG_DIF_ASIZE_HACTIVE_SHIFT) // ???
		| (mode->crtc_vdisplay << VDU_REG_DIF_ASIZE_VACTIVE_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_FSIZE, val);

	val = (88 << VDU_REG_DIF_HSYNC_START_SHIFT) // ???
		| (44 << VDU_REG_DIF_HSYNC_LEN_SHIFT) // ???
		| (148 << VDU_REG_DIF_HSYNC_DELAY_SHIFT);
	write_vdu_reg(vdu, VDU_REG_DIF_HSYNC, val);

	val = (0< VDU_REG_DIF_VSYNC_START_SHIFT) // ???
		| (5 << VDU_REG_DIF_VSYNC_LEN_SHIFT); // ???
	write_vdu_reg(vdu, VDU_REG_DIF_VSYNC, val);


	/* ??? [MVDU_MODE_HD_1080_P_30] = {
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
	}*/

	return 0;
}
// ???

static void plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	//
	// ???
	//
}

static void mode_config_init(struct vdu_device *vdu)
{
	struct drm_device *dev = &vdu->drm_dev;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 0; // ???
	dev->mode_config.min_height = 0; // ???
	dev->mode_config.max_width = 1920; // ??? RCM_VDU_MAX_PITCH / 2; // ???
	dev->mode_config.max_height = 1080; // ???
	dev->mode_config.preferred_depth = 16; // ???
	dev->mode_config.prefer_shadow = 0; // ???
	dev->mode_config.funcs = &mode_config_funcs;
}

static int crtc_init(struct vdu_device *vdu)
{
	int ret;

	ret = drm_universal_plane_init(
		&vdu->drm_dev,
		&vdu->primary_plane,
		0, // ??? crtcs,
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
// ???
// ???	drm_crtc_helper_add(crtc, &crtc_helper_funcs);
// ???
// ???	/* Start with vertical blanking interrupt reporting disabled. */
// ???	drm_crtc_vblank_off(crtc);
// ???
// ???	/* Register the interrupt handler. */
// ???	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_CRTC_IRQ_CLOCK)) {
// ???		/* The IRQ's are associated with the CRTC (sw)index. */
// ???		irq = platform_get_irq(pdev, swindex);
// ???		irqflags = 0;
// ???	} else {
// ???		irq = platform_get_irq(pdev, 0);
// ???		irqflags = IRQF_SHARED;
// ???	}
// ???
// ???	if (irq < 0) {
// ???		dev_err(rcdu->dev, "no IRQ for CRTC %u\n", swindex);
// ???		return irq;
// ???	}
// ???
// ???	ret = devm_request_irq(rcdu->dev, irq, rcar_du_crtc_irq, irqflags,
// ???			       dev_name(rcdu->dev), rcrtc);
// ???	if (ret < 0) {
// ???		dev_err(rcdu->dev,
// ???			"failed to register IRQ for CRTC %u\n", swindex);
// ???		return ret;
// ???	}
// ???
// ???	rcar_du_crtc_crc_init(rcrtc);

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

	drm_kms_helper_poll_fini(&vdu->drm_dev); // ???
	drm_mode_config_cleanup(&vdu->drm_dev);
	drm_dev_fini(&vdu->drm_dev);
}

static int probe_internal(struct platform_device *pdev)
{
	struct vdu_device *vdu;
	struct device_node *syscon_node;
	u32 hardware_id;
	int ret;

	vdu = devm_kzalloc(&pdev->dev, sizeof(*vdu), GFP_KERNEL);
	if (vdu == NULL)
		return -ENOMEM;

	vdu->pdev = pdev;
	platform_set_drvdata(pdev, vdu);

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

	ret = reset_vdu(vdu); // ???
	if (ret != 0)
		return ret;

	ret = drm_dev_init(&vdu->drm_dev, &rcm_vdu_drm_driver, &pdev->dev);
	if (ret != 0)
		return ret;

	// ???
	mode_config_init(vdu);

	ret = crtc_init(vdu);
	if (ret != 0)
		return ret;

	ret = encoder_init(vdu);
	if (ret != 0)
		return ret;

	// ???
	drm_mode_config_reset(&vdu->drm_dev); // ???

	drm_kms_helper_poll_init(&vdu->drm_dev); // ???

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
