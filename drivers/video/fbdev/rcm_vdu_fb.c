/*
 * drivers/video/module_vdu_fb.c - Module's VDU, frame buffer support.
 *
 *	Copyright (C) 2008 Module
 *	Written by MSU, CMC dept., LVK lab http://lvk.cs.msu.su
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/rcm_vdu.h>

static bool no_osd_areas = 0;
module_param(no_osd_areas, bool, 0444);
MODULE_PARM_DESC(no_osd_areas, "Start with no OSD areas (start in fullscreen mode)");

static uint xres = 1280;
module_param_named(width, xres, uint, 0644);
MODULE_PARM_DESC(width, "Framebuffer width");

static uint yres = 720;
module_param_named(height, yres, uint, 0644);
MODULE_PARM_DESC(height, "Framebuffer height");


/* Don't miss this with MVDU_COLOR_MODE_DEFAULT! This is the value that
 * MVDU_COLOR_MODE_DEFAULT expands into */
#define MVDU_FB_DEFAULT_COLOR_MODE MVDU_COLOR_MODE_565

/*
 * Some notes about buffer and areas
 * ---------------------------------
 *
 * Framebuffer interface assumes that:
 * - there is a (probably large) frame buffer,
 * - there is a single viewport into it, with dimensions defined by
 *   video mode,
 * - pixel format is defined for entire framebuffer.
 *
 * We want to support both linux framebuffer interface, and allow multiple
 * areas, with buffers located anywhere in the frame buffer, and with
 * different per-area settings.
 *
 * Because of that, we support two modes - "fulscreen" and "enhanced".
 *
 * In "fullscreen" mode, there is always one full-screen OSD area, it's
 * geometry is changed when video mode is changed to keep it full-screen,
 * it follows viewport position and color mode changes via FBIOPAN_DISPLAY or
 * FBIOPUT_VSCREENINFO.
 *
 * In "enhanced" mode, areas are controlled by user space directly via
 * FBIOPUT_OSDAREAS. When in this mode, the only area change that
 * can happen outside FBIOPUT_OSDAREAS is area cutting part that no longer
 * fits on the screen after video mode change.
 *
 * Modes are changed with FBIOPUT_OSDAREAS. If given params exactly match
 * current vscreeninfo (one area, sized xres/yres, positioned at (0,0)
 * buffer position (xoffset,yoffset)), then "standard" mode is set.
 * Otherwise, "enhanced" mode is set.
 *
 * By default driver starts in "fullscreen" mode. If 'no_osd_areas' param
 * is given, drivers starts in "enhanced" mode with no OSD areas.
 *
 * xres/yres fields of fb_var_screeninfo correspond to dimensions of
 * the current video mode, and are used to change it.
 * xres_virtual / yres_virtual are always constants set by the driver,
 * any attempts to change are ignored.
 */


#define BUFFER_OFFSET(x, y) \
	MVDU_OSD_BYTESPERPIXEL * ((y) * MVDU_OSD_BUFFER_WIDTH + (x))

static int score_for_pair(int have, int need)
{
	int a, b;

	if (have == need)
		return 500;

	a = min(have, need);
	b = max(have, need);
	return a * 100 / b;
}

static int mode_score_for_var(const struct fb_var_screeninfo *var,
					const struct mvdu_mode *mode)
{
	int score;

	BUG_ON(!mode);
	BUG_ON(!var);

	/* Dimensions match is a must */
	if (var->xres != mode->width || var->yres != mode->height)
		return 0;

	/* Dimensions match gives 20000 points */
	score = 20000;

	/* vmode match gives 5000 points */
	if ((var->vmode == FB_VMODE_NONINTERLACED &&
				mode->vmode == MVDU_MODE_PROGRESSIVE) ||
	    (var->vmode == FB_VMODE_INTERLACED &&
				mode->vmode == MVDU_MODE_INTERLACED))
		score += 5000;

	/* Each of hs_len, vs_len, and margins may add 500 points
	   for exact match, and up to 100 points for inexact match */
	if (var->hsync_len)
		score += score_for_pair(var->hsync_len, mode->hs_len);
	if (var->vsync_len)
		score += score_for_pair(var->vsync_len, mode->vs_len);
	if (var->left_margin)
		score += score_for_pair(var->left_margin,
			mode->h_blank - mode->hs_start - mode->hs_len);
	if (var->right_margin)
		score += score_for_pair(var->right_margin, mode->hs_start);
	if (var->upper_margin)
		score += score_for_pair(var->upper_margin,
			mode->v_blank_beg - mode->vs_start - mode->vs_len);
	if (var->lower_margin)
		score += score_for_pair(var->lower_margin,
			mode->vs_start + mode->v_blank_end);

	return score;
}

static int best_mode_for_var(struct mvdu_device *dev,
				const struct fb_var_screeninfo *var)
{
	int best = -EINVAL, best_score = -1, score, i;
	const struct mvdu_mode *mode;

	for (i = 1; i <= MVDU_MODE_LAST; i++) {
		mode = mvdu_get_modeinfo(dev, i);
		score = mode_score_for_var(var, mode);
		if (score && score > best_score) {
			best = i;
			best_score = score;
		}
	}

	return best;
}

static int best_color_mode_for_var(const struct fb_var_screeninfo *var)

{
	switch (var->transp.length) {
	case 0:
		return MVDU_COLOR_MODE_565;
	case 1:
		return MVDU_COLOR_MODE_5551;
	case 4:
		return MVDU_COLOR_MODE_4444;
	default:
		return MVDU_FB_DEFAULT_COLOR_MODE;
	}
}

static void mode_to_var(struct mvdu_device *dev, int mode_num, struct fb_var_screeninfo *var)
{
	const struct mvdu_mode *mode = mvdu_get_modeinfo(dev, mode_num);

	BUG_ON(!mode);

	var->xres = mode->width;
	var->yres = mode->height;

	var->xres_virtual = MVDU_OSD_BUFFER_WIDTH;
	var->yres_virtual = MVDU_OSD_BUFFER_HEIGHT;

	if (var->xoffset < 0)
		var->xoffset = 0;
	if (var->xoffset + var->xres > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;

	if (var->yoffset < 0)
		var->yoffset = 0;
	if (var->yoffset + var->yres > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	var->width = var->xres;		/* placeholder */
	var->height = var->yres;	/* placeholder */

	/* Convert Hz to picoseconds */
	var->pixclock = KHZ2PICOS(mode->pixclock / 1000);

	var->left_margin = mode->h_blank - mode->hs_start - mode->hs_len;
	var->right_margin = mode->hs_start;
	var->upper_margin = mode->v_blank_beg - mode->vs_start - mode->vs_len;
	var->lower_margin = mode->vs_start + mode->v_blank_end;
	var->hsync_len = mode->hs_len;
	var->vsync_len = mode->vs_len;

	var->sync = (mode->vs_p) ?
		(FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT) : 0;
	var->vmode =
		(mode->vmode == MVDU_MODE_PROGRESSIVE) ?
			FB_VMODE_NONINTERLACED : FB_VMODE_INTERLACED;
	var->rotate = 0;
}

static void color_mode_to_var(int color_mode, struct fb_var_screeninfo *var)
{
	var->bits_per_pixel = MVDU_OSD_BITSPERPIXEL;
	var->grayscale = 0;
	var->nonstd = 0;

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	switch (color_mode) {

	case MVDU_COLOR_MODE_4444:
		var->transp.offset = 12;
		var->transp.length = 4;
		var->red.offset = 8;
		var->red.length = 4;
		var->green.offset = 4;
		var->green.length = 4;
		var->blue.offset = 0;
		var->blue.length = 4;
		break;

	case MVDU_COLOR_MODE_5551:
		/*var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 6;
		var->green.length = 5;
		var->blue.offset = 1;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 1;*/
		var->transp.offset = 15;
		var->transp.length = 1;
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;

	case MVDU_COLOR_MODE_565:
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;

	default:
		BUG();
	}
}

static unsigned long xy_to_pa(struct mvdu_device *dev, int x, int y)
{
	return dev->fb_buffer_dma +
		MVDU_OSD_BYTESPERPIXEL * (y * MVDU_OSD_BUFFER_WIDTH + x);
}

static void pa_to_xy(struct mvdu_device *dev, unsigned long pa, int *x, int *y)
{
	unsigned long pixpos =
		(pa - dev->fb_buffer_dma) / MVDU_OSD_BYTESPERPIXEL;
	*x = pixpos % MVDU_OSD_BUFFER_WIDTH;
	*y = pixpos / MVDU_OSD_BUFFER_WIDTH;
}

static int check_fb_area_buf_pos(struct mvdu_fb_area *area)
{
	if (area->buf_x < 0 ||
	    area->buf_x >= MVDU_OSD_BUFFER_WIDTH ||
	    area->buf_x + area->w > MVDU_OSD_BUFFER_WIDTH ||
	    area->buf_y < 0 ||
	    area->buf_y >= MVDU_OSD_BUFFER_HEIGHT ||
	    area->buf_y + area->h > MVDU_OSD_BUFFER_HEIGHT)
		return -EINVAL;
	else
		return 0;
	    
}

static void area_ext_to_int(struct mvdu_device *dev,
		struct mvdu_fb_area *exta, struct mvdu_osd_area *inta)
{
	inta->x = exta->x;
	inta->y = exta->y;
	inta->w = exta->w;
	inta->h = exta->h;
	inta->alpha = exta->alpha;

	switch (exta->color_mode) {
	case MVDU_COLOR_MODE_565:
	case MVDU_COLOR_MODE_5551:
	case MVDU_COLOR_MODE_4444:
		inta->color_mode = exta->color_mode;
		break;
	default:
		inta->color_mode = best_color_mode_for_var(&dev->fb_info->var);
		break;
	}

	inta->buffer_pa = xy_to_pa(dev, exta->buf_x, exta->buf_y);
	inta->linestep = MVDU_OSD_BUFFER_WIDTH;
}

static void area_int_to_ext(struct mvdu_device *dev,
		struct mvdu_osd_area *inta, struct mvdu_fb_area *exta)
{
	exta->x = inta->x;
	exta->y = inta->y;
	exta->w = inta->w;
	exta->h = inta->h;
	exta->alpha = inta->alpha;
	exta->color_mode = inta->color_mode;
	pa_to_xy(dev, inta->buffer_pa, &exta->buf_x, &exta->buf_y);
}

static void set_areas_for_var(struct mvdu_device *dev,
					struct fb_var_screeninfo *var)
{
	struct mvdu_osd_area area;

	area.x = 0;
	area.y = 0;
	area.w = var->xres;
	area.h = var->yres;
	area.alpha = 15;	/* hardware maximum */
	area.color_mode = best_color_mode_for_var(var);
	area.buffer_pa = xy_to_pa(dev, var->xoffset, var->yoffset);
	area.linestep = MVDU_OSD_BUFFER_WIDTH;

	mvdu_osd_set_areas(dev, 1, &area);
}

static int is_area_for_var(struct mvdu_device *dev,
		struct mvdu_fb_area *area, struct fb_var_screeninfo *var)
{
	return (area->x == 0 && area->y == 0 &&
		area->w == var->xres && area->h == var->yres &&
		area->buf_x == area->x && area->buf_y == area->y);
}

static void mode_notifier(struct mvdu_device *dev)
{
	struct fb_var_screeninfo *var = &dev->fb_info->var;

	/* Problem here.
	 *
	 * This may be called at whatever moment, and alter var, racing
	 * against whatever var reader elsewhere. As a result,
	 * FBIOGET_VARSCREENINFO may return partially-overwritten structure.
	 *
	 * This looks unavoidable within current framebuffer subsystem:
	 * external-initiated var altering looks completely unsupported.
	 *
	 * Let's live with this for now.
	 */
	mode_to_var(dev, dev->current_mode, var);

	/* Handle possible xoffset / yoffset change */
	if (dev->fb_fullscreen)
		set_areas_for_var(dev, var);
}


static int module_vdu_fb_check_var(struct fb_var_screeninfo *var,
						struct fb_info *info)
{
	struct mvdu_device *dev = info->par;
	int mode_num, color_mode;
	int ret = 0;

	if (var->bits_per_pixel != MVDU_OSD_BITSPERPIXEL)
		return -EINVAL;

	mutex_lock(&dev->fb_mutex);

	mode_num = best_mode_for_var(dev, var);
	if (mode_num < 0) {
		ret = -EINVAL;
		goto out;
	}
	color_mode = best_color_mode_for_var(var);

	mode_to_var(dev, mode_num, var);
	color_mode_to_var(color_mode, var);

out:
	mutex_unlock(&dev->fb_mutex);
	return ret;
}

static int module_vdu_fb_set_par(struct fb_info *info)
{
	struct mvdu_device *dev = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int mode_num, ret;

	mutex_lock(&dev->fb_mutex);

	mode_num = best_mode_for_var(dev, var);
	if (mode_num < 0) {
		ret = -EINVAL;
		goto out;
	}

	mode_num = best_mode_for_var(dev, &info->var);
	BUG_ON(mode_num < 0);
	pr_info("Selected video mode is %d\n", mode_num);

	ret = mvdu_set_mode(dev, mode_num, 1);
	/* From within, mode setting callback will be called unconditionally,
	 * and setting framebuffer params will be called from there */

out:
	mutex_unlock(&dev->fb_mutex);
	return ret;
}

static int module_vdu_fb_open(struct fb_info *info, int user)
{
	struct mvdu_device *dev = info->par;

	mutex_lock(&dev->fb_mutex);
	dev_info(dev->dev, "vdu_fb_open\n");

	dev->fb_open_count++;
	if (dev->fb_open_count == 1) {

		/* Turn on OSD if not blanked.
		   If there are no areas, _core will handle that */
		if (!dev->fb_blanked) {
			mvdu_enable_vdu(dev);
			mvdu_enable_osd(dev);
		}
	}

	mutex_unlock(&dev->fb_mutex);
	return 0;
}

static int module_vdu_fb_release(struct fb_info *info, int user)
{
	struct mvdu_device *dev = info->par;

	mutex_lock(&dev->fb_mutex);

	if (dev->fb_open_count == 1) {
		if (!dev->fb_blanked) {
			mvdu_disable_osd(dev);
			mvdu_disable_vdu(dev);
		}
	}
	dev->fb_open_count--;

	mutex_unlock(&dev->fb_mutex);
	return 0;
}

static int module_vdu_fb_blank(int blank, struct fb_info *info)
{
	struct mvdu_device *dev = info->par;

	mutex_lock(&dev->fb_mutex);

	if (dev->fb_blanked && blank == FB_BLANK_UNBLANK) {
		mvdu_enable_vdu(dev);
		mvdu_enable_osd(dev);
		dev->fb_blanked = 0;
	} else if (!dev->fb_blanked && blank != FB_BLANK_UNBLANK) {
		dev->fb_blanked = 1;
		mvdu_disable_osd(dev);
		mvdu_disable_vdu(dev);
	}

	mutex_unlock(&dev->fb_mutex);
	return 0;
}

static int module_vdu_fb_pan_display(struct fb_var_screeninfo *var,
							struct fb_info *info)
{
	/* Here we need only to program hardware, all other is done elsewhere */
	struct mvdu_device *dev = info->par;
	if (dev->fb_fullscreen)
		set_areas_for_var(dev, var);

	return 0;
}

static int module_vdu_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct mvdu_device *dev = info->par;
	struct rgb888 color;
	struct mvdu_fb_areas areas;
	struct mvdu_fb_area *fba;
	struct mvdu_osd_area *osda;
	int areas_nr;
	struct fb_vblank blank;
	int hblanking, vblanking;
	int i, ret = 0;

	mutex_lock(&dev->fb_mutex);

	switch (cmd) {

	case FBIOPUT_BACKGROUND:
		if (!copy_from_user(&color, (void __user *)arg, sizeof(color)))
			ret = mvdu_set_bgcolor(dev, &color);
		else
			ret = -EFAULT;
		break;

	case FBIOGET_BACKGROUND:
		mvdu_get_bgcolor(dev, &color);
		if (copy_to_user((void __user *)arg, &color, sizeof(color)))
			ret = -EFAULT;
		break;

	case FBIOPUT_OSDAREAS:
		if (copy_from_user(&areas, (void __user *)arg, sizeof(areas))) {
			ret = -EFAULT;
			break;
		}

		if (areas.areas_nr < 0 || areas.areas_nr > dev->osd_areas_max) {
			ret = -EINVAL;
			break;
		}

		if (areas.areas_nr > 0) {
			if (copy_from_user(dev->fb_tmp_fb_areas,
				(void __user *)areas.areas,
				areas.areas_nr * sizeof(areas.areas[0]))) {
				ret = -EFAULT;
				break;
			}

			for (i = 0; i < areas.areas_nr; i++) {

				fba = &dev->fb_tmp_fb_areas[i];
				osda = &dev->fb_tmp_osd_areas[i];

				ret = check_fb_area_buf_pos(fba);
				if (ret)
					goto out;
				
				area_ext_to_int(dev, fba, osda);
			}
		}

		ret = mvdu_osd_set_areas(dev, areas.areas_nr,
							dev->fb_tmp_osd_areas);
		if (!ret) {
			if (areas.areas_nr != 1)
				dev->fb_fullscreen = 0;
			else
				dev->fb_fullscreen = is_area_for_var(dev,
						&dev->fb_tmp_fb_areas[0],
						&dev->fb_info->var);
		}
		break;

	case FBIOGET_OSDAREAS:
		if (copy_from_user(&areas, (void __user *)arg, sizeof(areas))) {
			ret = -EFAULT;
			break;
		}
		mvdu_osd_get_areas(dev, &areas_nr, dev->fb_tmp_osd_areas);
		areas.areas_nr = min(areas.areas_nr, areas_nr);
		if (copy_to_user((void __user *)arg, &areas, sizeof(areas))) {
			ret = -EFAULT;
			break;
		}
		if (areas.areas_nr > 0) {
			for (i = 0; i < areas.areas_nr; i++) {
				fba = &dev->fb_tmp_fb_areas[i];
				osda = &dev->fb_tmp_osd_areas[i];
				area_int_to_ext(dev, osda, fba);
			}
			if (copy_to_user((void __user *)areas.areas,
				dev->fb_tmp_fb_areas,
				areas.areas_nr * sizeof(areas.areas[0])))
				ret = -EFAULT;
		}
		break;

	case FBIO_WAITFORVSYNC:
		/* On this hardware, waiting for VSYNC is pointless
		   because DMA engine does aggressive prefetching.
		   So wait for DMA engine end reading current frame instead */
		ret = mvdu_wait_osd_fr_rd_end(dev);
		break;

	case FBIOGET_VBLANK:
		memset(&blank, 0, sizeof(blank));
		mvdu_get_blanking(dev, &hblanking, &vblanking);
		if (hblanking)
			blank.flags |= FB_VBLANK_HBLANKING;
		if (vblanking)
			blank.flags |= FB_VBLANK_VBLANKING;
		blank.flags = FB_VBLANK_HAVE_HBLANK | FB_VBLANK_HAVE_VBLANK;
		if (copy_to_user((void __user *)arg, &blank, sizeof(blank)))
			ret = -EFAULT;
		break;

	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&dev->fb_mutex);
	return ret;
}

static int module_vdu_fb_setcolreg(unsigned reg, unsigned r, unsigned g,
		unsigned b, unsigned transp, struct fb_info *info)
{
	if (reg >= 256)
		return -EINVAL;

	r >>= (16 - info->var.red.length);
	g >>= (16 - info->var.green.length);
	b >>= (16 - info->var.blue.length);
	transp >>= (16 - info->var.transp.length);

	((u32*)(info->pseudo_palette))[reg] =
		(r << info->var.red.offset) |
		(g << info->var.green.offset) |
		(b << info->var.blue.offset) |
		(transp << info->var.transp.offset);

	return 0;
}

static struct fb_ops module_vdu_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= module_vdu_fb_open,
	.fb_release	= module_vdu_fb_release,
	.fb_check_var	= module_vdu_fb_check_var,
	.fb_set_par	= module_vdu_fb_set_par,
	.fb_blank	= module_vdu_fb_blank,
	.fb_ioctl	= module_vdu_fb_ioctl,
	.fb_setcolreg	= module_vdu_fb_setcolreg,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_pan_display	= module_vdu_fb_pan_display,
};

static int module_vdu_fb_probe(struct mvdu_device *dev)
{

	struct fb_info *info;
	int ret;

	dev_info(dev->dev, "probing VDU framebuffer\n");
	mutex_init(&dev->fb_mutex);
	dev->fb_blanked = 0;
	dev->fb_open_count = 0;

	BUG_ON(dev->mode_notifier != 0);
	dev->mode_notifier = mode_notifier;

	info = framebuffer_alloc(0, NULL);
	if (!info) {
		dev_err(dev->dev, "failed to allocate fb_info\n");
		ret = -ENOMEM;
		goto err_return;
	}
	dev->fb_info = info;
	info->par = dev;

	dev->fb_tmp_fb_areas =
		kmalloc(dev->osd_areas_max * sizeof(struct mvdu_fb_area),
				GFP_KERNEL);
	if (!dev->fb_tmp_fb_areas) {
		dev_err(dev->dev, "failed to allocate area descriptors\n");
		ret = -ENOMEM;
		goto err_dealloc_info;
	}

	dev->fb_tmp_osd_areas =
		kmalloc(dev->osd_areas_max * sizeof(struct mvdu_osd_area),
				GFP_KERNEL);
	if (!dev->fb_tmp_osd_areas) {
		dev_err(dev->dev, "failed to allocate area descriptors\n");
		ret = -ENOMEM;
		goto err_dealloc_areas_half;
	}

	ret = (dev->allocate_framebuffer) ? dev->allocate_framebuffer(dev) : 0;
	if (!dev->fb_buffer_cpu) {
		dev_err(dev->dev, "failed to allocate frame buffer\n");
		if (!ret)
			ret = -ENOMEM;
		goto err_dealloc_areas;
	}

	info->screen_base = dev->fb_buffer_cpu;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->fbops = &module_vdu_fb_ops;

	info->fix.smem_start = dev->fb_buffer_dma;
	info->fix.smem_len = dev->fb_buffer_size;
	strncpy(info->fix.id, "module_vdu", sizeof(info->fix.id) - 1);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = MVDU_OSD_BUFFER_WIDTH * MVDU_OSD_BYTESPERPIXEL;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.xpanstep = info->fix.ypanstep = 1;

	mode_to_var(dev, dev->current_mode, &info->var);
	color_mode_to_var(MVDU_FB_DEFAULT_COLOR_MODE, &info->var);	/* read default from _core */
	info->var.xoffset = info->var.yoffset = 0;

	pr_info("Framebufer resolution is: %dx%d\n", xres, yres);

	info->var.xres = xres;
	info->var.yres = yres;
	dev->fb_fullscreen = !no_osd_areas;

	ret = module_vdu_fb_set_par(info);
	if (ret) {
		dev_err(dev->dev, "failed to initialize framebuffer params\n");
		goto err_dealloc_buffer;
	}

	/* This is required for fbcon */
	info->pseudo_palette = dev->pseudo_palette;
	ret = fb_alloc_cmap(&info->cmap, 256, 1);
	if (ret)
		goto err_dealloc_buffer;

	ret = register_framebuffer(info);
	if (ret) {
		dev_err(dev->dev, "failed to register framebuffer device\n");
		goto err_dealloc_cmap;
	}

	printk(KERN_INFO "fb%d: Module VDU frame buffer device\n", info->node);

	return 0;

err_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
err_dealloc_buffer:
	if (dev->free_framebuffer)
		dev->free_framebuffer(dev);
err_dealloc_areas:
	kfree(dev->fb_tmp_osd_areas);
err_dealloc_areas_half:
	kfree(dev->fb_tmp_fb_areas);
err_dealloc_info:
	framebuffer_release(info);
err_return:
	dev->mode_notifier = 0;
	return ret;
}

static void __exit module_vdu_fb_remove(struct mvdu_device *dev)
{
	struct fb_info *info;

	info = dev->fb_info;
	if (!info)
		return;		/* driver was not initialized for this device */

	BUG_ON(dev->fb_open_count != 0);

	mvdu_osd_set_areas(dev, 0, 0);
	dev->mode_notifier = 0;
	dev->fb_info = NULL;

	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	if (dev->free_framebuffer)
		dev->free_framebuffer(dev);
	kfree(dev->fb_tmp_osd_areas);
	kfree(dev->fb_tmp_fb_areas);
	framebuffer_release(info);
}

static int __init module_vdu_fb_init(void)
{
	struct mvdu_device *dev;
	int i = 0;

	while ((dev = mvdu_get_device(i)) != NULL) {
		pr_info("about to probe vdu dev %d\n", i);
		module_vdu_fb_probe(dev);
		i++;
	}
	return 0;
}

static void __exit module_vdu_fb_exit(void)
{
	struct mvdu_device *dev;
	int i = 0;

	while ((dev = mvdu_get_device(i)) != NULL) {
		module_vdu_fb_remove(dev);
		i--;
	}
	return;
}

module_init(module_vdu_fb_init);
module_exit(module_vdu_fb_exit);

MODULE_AUTHOR("Nikita Youshchenko <yoush@cs.msu.su>, "
	      "Alexander Gerasiov <gq@cs.msu.su>");
MODULE_DESCRIPTION("Module VDU OSD framebuffer device driver");
MODULE_LICENSE("GPL");
