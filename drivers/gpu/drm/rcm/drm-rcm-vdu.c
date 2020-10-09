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

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_encoder.h>
#include <drm/drm_bridge.h>
#include <drm/drm_gem_shmem_helper.h>
// ??? #include <drm/drm_simple_kms_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fb_helper.h>

#define DRIVER_NAME "rcm-vdu-drm"
#define DRIVER_DESC "rcm-vdu KMS driver"
#define DRIVER_DATE "2020"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

// ???
#define RCM_VDU_MAX_PITCH 4096 /* ???? */

struct vdu_device {
	struct drm_device drm_dev;
	struct platform_device *pdev;
	struct drm_crtc crtc;
	struct drm_plane primary_plane;
	bool primary_plane_initialized;
	struct drm_encoder encoder;
	// ??? bool drm_dev_registered;
	// ??? struct drm_connector	       conn;
	// ??? unsigned int		       cpp;
	// ??? unsigned int		       pitch;
	// ??? void __iomem		       *vram;
	// ??? void __iomem		       *mmio;
};

DEFINE_DRM_GEM_FOPS(rcm_vdu_fops);

static struct drm_driver rcm_vdu_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET /*??? | DRIVER_ATOMIC*/,
	.name		 = DRIVER_NAME,
	.desc		 = DRIVER_DESC,
	.date		 = DRIVER_DATE,
	.major		 = DRIVER_MAJOR,
	.minor		 = DRIVER_MINOR,
	.fops		 = &rcm_vdu_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
};

static const struct drm_mode_config_funcs mode_config_funcs = {
	// ??? .fb_create = cirrus_fb_create,
	// ??? .atomic_check = drm_atomic_helper_check,
	// ??? .atomic_commit = drm_atomic_helper_commit,
};

static const uint32_t plane_formats[] = {
	// ??? DRM_FORMAT_RGB565,
	// ??? DRM_FORMAT_RGB888,
	DRM_FORMAT_XRGB8888 // ???
};

static const uint64_t plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static const struct drm_plane_funcs plane_funcs = {
	// ??? .update_plane = drm_atomic_helper_update_plane,
	// ??? .disable_plane = drm_atomic_helper_disable_plane,
	// ??? .reset = rcar_du_plane_reset,
	.destroy = drm_plane_cleanup,
	// ??? .atomic_duplicate_state = rcar_du_plane_atomic_duplicate_state,
	// ??? .atomic_destroy_state = rcar_du_plane_atomic_destroy_state,
	// ??? .atomic_set_property = rcar_du_plane_atomic_set_property,
	// ??? .atomic_get_property = rcar_du_plane_atomic_get_property,
};

static const struct drm_crtc_funcs crtc_funcs = {
	// ??? .reset = rcar_du_crtc_reset,
	.destroy = drm_crtc_cleanup,
	// ??? .set_config = drm_atomic_helper_set_config,
	// ??? .page_flip = drm_atomic_helper_page_flip,
	// ??? .atomic_duplicate_state = rcar_du_crtc_atomic_duplicate_state,
	// ??? .atomic_destroy_state = rcar_du_crtc_atomic_destroy_state,
	// ??? .enable_vblank = rcar_du_crtc_enable_vblank,
	// ??? .disable_vblank = rcar_du_crtc_disable_vblank,
};

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void mode_config_init(struct vdu_device *vdu)
{
	struct drm_device *dev = &vdu->drm_dev;

	drm_mode_config_init(dev);
	dev->mode_config.min_width = 0; // ???
	dev->mode_config.min_height = 0; // ???
	dev->mode_config.max_width = RCM_VDU_MAX_PITCH / 2; // ???
	dev->mode_config.max_height = 1024; // ???
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
// ???	static const unsigned int mmio_offsets[] = {
// ???		DU0_REG_OFFSET, DU1_REG_OFFSET, DU2_REG_OFFSET, DU3_REG_OFFSET
// ???	};
// ???
// ???	struct rcar_du_device *rcdu = rgrp->dev;
// ???	struct platform_device *pdev = to_platform_device(rcdu->dev);
// ???	struct rcar_du_crtc *rcrtc = &rcdu->crtcs[swindex];
// ???	struct drm_crtc *crtc = &rcrtc->crtc;
// ???	struct drm_plane *primary;
// ???	unsigned int irqflags;
// ???	struct clk *clk;
// ???	char clk_name[9];
// ???	char *name;
// ???	int irq;
// ???	int ret;
// ???
// ???	/* Get the CRTC clock and the optional external clock. */
// ???	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_CRTC_IRQ_CLOCK)) {
// ???		sprintf(clk_name, "du.%u", hwindex);
// ???		name = clk_name;
// ???	} else {
// ???		name = NULL;
// ???	}
// ???
// ???	rcrtc->clock = devm_clk_get(rcdu->dev, name);
// ???	if (IS_ERR(rcrtc->clock)) {
// ???		dev_err(rcdu->dev, "no clock for DU channel %u\n", hwindex);
// ???		return PTR_ERR(rcrtc->clock);
// ???	}
// ???
// ???	sprintf(clk_name, "dclkin.%u", hwindex);
// ???	clk = devm_clk_get(rcdu->dev, clk_name);
// ???	if (!IS_ERR(clk)) {
// ???		rcrtc->extclock = clk;
// ???	} else if (PTR_ERR(clk) == -EPROBE_DEFER) {
// ???		return -EPROBE_DEFER;
// ???	} else if (rcdu->info->dpll_mask & BIT(hwindex)) {
// ???		/*
// ???		 * DU channels that have a display PLL can't use the internal
// ???		 * system clock and thus require an external clock.
// ???		 */
// ???		ret = PTR_ERR(clk);
// ???		dev_err(rcdu->dev, "can't get dclkin.%u: %d\n", hwindex, ret);
// ???		return ret;
// ???	}
// ???
// ???	init_waitqueue_head(&rcrtc->flip_wait);
// ???	init_waitqueue_head(&rcrtc->vblank_wait);
// ???	spin_lock_init(&rcrtc->vblank_lock);
// ???
// ???	rcrtc->dev = rcdu;
// ???	rcrtc->group = rgrp;
// ???	rcrtc->mmio_offset = mmio_offsets[hwindex];
// ???	rcrtc->index = hwindex;
// ???	rcrtc->dsysr = (rcrtc->index % 2 ? 0 : DSYSR_DRES) | DSYSR_TVM_TVSYNC;
// ???
// ???	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_VSP1_SOURCE))
// ???		primary = &rcrtc->vsp->planes[rcrtc->vsp_pipe].plane;
// ???	else
// ???		primary = &rgrp->planes[swindex % 2].plane;
// ???
	ret = drm_crtc_init_with_planes(
		&vdu->drm_dev,
		&vdu->crtc,
		&vdu->primary_plane,
		NULL, // cursor plane
		&crtc_funcs,
		NULL); // name
	if (ret != 0)
		return ret;
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
	int ret;

	vdu = devm_kzalloc(&pdev->dev, sizeof(*vdu), GFP_KERNEL);
	if (vdu == NULL)
		return -ENOMEM;

	ret = drm_dev_init(&vdu->drm_dev, &rcm_vdu_drm_driver, &pdev->dev);
	if (ret != 0)
		return ret;
	vdu->pdev = pdev;
	platform_set_drvdata(pdev, vdu);

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
