/*
 * include/linux/module_vdu.h - Module VDU header
 *
 *	Copyright (C) 2008 Module
 *	Written by MSU, CMC dept., LVK lab http://lvk.cs.msu.su
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */
#ifndef __LINUX_MODULE_VDU_H__
#define __LINUX_MODULE_VDU_H__

#include <uapi/linux/rcm_vdu.h>

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <media/v4l2-device.h>
#include <linux/wait.h>

struct mvdu_mode {
	/* User-visible mode parameters */
	int width, height;		/* picture size, pixels */
	unsigned int vmode;		/* MVDU_MODE_(PROGRESSIVE|INTERLACED) */

	/* Required pixel clock, in Hz */
	unsigned int pixclock;

	/* Low level mode description */
	int mode;
	int h_blank;
	int h_active;
	int h_total;
	int hs_start;
	int hs_len;
	int hs_delay;
	int v_blank_beg;
	int v_active;
	int v_total;
	int v_blank_end;
	int vs_start;
	int vs_len;
	int vs_p;
};

/* Internal representation of an OSD area */
struct mvdu_osd_area {
	unsigned long buffer_pa;/* Buffer address of top-left area pixel */
	int linestep;		/* Buffer distance (in pixels) between line
				   beginnings */
	int x, y;		/* Area screen position */
	int w, h;		/* Area width and height */
	int color_mode;		/* Area color mode */
	int alpha;		/* Alpha value (for RGB565 and RGBA5551 modes) */
};

/* The osd_area_header is a DMA structure; it may require changes
   in case of endianness or similar issues */
struct osd_area_header {
	u32 osd_next;		/* 32-bit LE value at offset 0 */
	u32 osd_base;		/* 32-bit LE value at offset 4 */
	u16 osd_hor_size;	/* 16-bit LE value at offset 8 */
	u16 osd_ver_size;	/* 16-bit LE value at offset 10 */
	u16 osd_hor_start;	/* 16-bit LE value at offset 12 */
	u16 osd_ver_start;	/* 16-bit LE value at offset 14 */
	u16 full_line_size;	/* 16-bit LE value at offset 16 */
	u8 df;			/* Byte at offset 18 */
	u8 alpha;		/* Byte at offset 19 */
	u8 pad[12];		/* Pad at offsets 20..31 */
};

#define MVDU_OFLAG_SDTV_SEQ		0
#define MVDU_OFLAG_SDTV_PAR		MVDU_REG_DIF_CTRL_SDTV_FORM_MASK
#define MVDU_OFLAG_NO_EXT_SYNC		0
#define MVDU_OFLAG_EXT_SYNC		MVDU_REG_DIF_CTRL_EXT_SYNC_EN_MASK
#define MVDU_OFLAG_NO_INT_SYNC		0
#define MVDU_OFLAG_INT_SYNC		MVDU_REG_DIF_CTRL_INT_SYNC_EN_MASK

struct mvdu_video_buf {
	unsigned long planes[6];
#define MVDU_ADDR_Y(x)  ((x)->planes[0])
#define MVDU_ADDR_C(x)  ((x)->planes[1])
#define MVDU_ADDR_CB(x) ((x)->planes[1])
#define MVDU_ADDR_CR(x) ((x)->planes[2])
#define MVDU_ADDR_FR1_Y(x)  ((x)->planes[0])
#define MVDU_ADDR_FR1_C(x)  ((x)->planes[1])
#define MVDU_ADDR_FR1_CB(x) ((x)->planes[1])
#define MVDU_ADDR_FR1_CR(x) ((x)->planes[2])
#define MVDU_ADDR_FR2_Y(x)  ((x)->planes[3])
#define MVDU_ADDR_FR2_C(x)  ((x)->planes[4])
#define MVDU_ADDR_FR2_CB(x) ((x)->planes[4])
#define MVDU_ADDR_FR2_CR(x) ((x)->planes[5])

	/* Addresses to write to hardware */
	unsigned long odd_y_addr,  odd_cb_addr,  odd_cr_addr,
		      even_y_addr, even_cb_addr, even_cr_addr;

	int users;			/* how many pipeline entries do reference this buffer */
	unsigned int cr_valid : 1;	/* non-zero if cr addresses should be written */
	unsigned int single_use : 1;	/* non-zero if buffer is expected to be used for only one pipeline entry */
};

struct mvdu_mvl_pipeline_entry {
	struct mvdu_video_buf *buffer;	/* where to take frame data */
	unsigned int hungry : 1;	/* non-zero if used previous buffer because new one not available */
	unsigned int old_params : 1;	/* non-zero if used previous buffer because new one requires new params */
};

#define MVDU_MVL_PIPELINE_SIZE	8

struct mvdu_device {

	/* Video mode support information */
	u32 supported_modes;	/* bit mask */
	int default_mode;
	u32 output_flags;		/* low-level output flags - MVDU_OFLAG_* */

	/* Value for AXI_PARAM registers */
	u32 axi_mvl_param;
	u32 axi_osd_param;

	/* Callbacks to notify low-level about start/stop */
	void (*notify_started)(struct mvdu_device *dev);
	void (*notify_stopped)(struct mvdu_device *dev);

	/* Current background color in mode-independent representation */
	struct rgb888 bgcolor;
	unsigned int bgcolor_dirty : 1;	/* non-zero if background color write is pending */

	/* Video mode processing */
	int current_mode;
	void (*mode_notifier)(struct mvdu_device *);

	/* VDU enable/disable processing */
	int vdu_state;					/* VDU_STATE_* */
	unsigned int vdu_ena_req : 1;			/* non-zero if VDU was enabled by external interface */
	unsigned int vdu_restart_req : 1;		/* non-zero if VDU should be restarted */
	unsigned int vdu_delayed_restart_req : 1;	/* non-zero if VDU should be restarted after a delay */
	unsigned int write_osd_weights_on_ena : 1;	/* non-zero if OSD weight registers should be written on VDU start */
	unsigned int write_bgcolor_on_ena : 1;		/* non-zero if background color register should be written on VDU start */
	struct timer_list delayed_restart_timer;
	struct timer_list restart_watchdog_timer;
	int enable_counter;				/* interface-level enable counter */

	/* SA interrupt related */
	int sa_waiters;
	int sa_id;			/* increases when wakeup condition arises */
	wait_queue_head_t sa_wq;

	/* OSD_FR_RD_END interrupt related - similar to SA */
	int osd_fr_rd_end_waiters;
	int osd_fr_rd_end_id;
	wait_queue_head_t osd_fr_rd_end_wq;

	/* OSD enable/disable processing */
	int osd_state;				/* OSD_STATE_* */
	unsigned int osd_ena_req : 1;		/* non-zero if OSD was enabled by external interface */
	unsigned int osd_areas_ready : 1;	/* non-zero if OSD areas are ready */

	/* OSD area processing */
	int osd_areas_max;
	int osd_areas_nr;
	struct mvdu_osd_area *osd_areas;
	struct osd_area_header *osd_headers;
	dma_addr_t osd_headers_dma;
	int osd_prev_group, osd_cur_group, osd_free_group;
	int osd_last_change_avmp;	/* -1 if hardware definitly uses osd_cur_group, osd avmp bit at last header update otherwise */

	/* OSD header writing */
	unsigned int osd_reg0_is_odd : 1;	/* non-zero if reg0 is used for odd frames, reg1 for even frames */
	dma_addr_t osd_even_headers_dma, osd_odd_headers_dma;

	/* MVL */
	int mvl_state;			/* MVL_STATE_* */

	/* MVL current settings */
	struct mvdu_video_params mvl_params;
	unsigned int mvl_params_consistent : 1;	/* non-zero if parameters are consistent and may be used */
	unsigned int mvl_change_pending : 1;	/* non-zero if latest params not yet written to hardware */

	/* MVL new buffer */
	unsigned int mvl_new_buffer_offered : 1;
	struct mvdu_video_buf *mvl_new_buffer;

	/* MVL pipeline */
	struct mvdu_mvl_pipeline_entry mvl_pipeline[MVDU_MVL_PIPELINE_SIZE];
	int mvl_unfilled_entry;		/* entry to be filled next */
	int mvl_hw_curr_entry;		/* entry currently accessed by hardware */
	int mvl_unfreed_entry;
	unsigned int mvl_reg0_is_odd : 1;	/* non-zero if reg0 is used for odd frames, reg1 for even frames */

	/* Values to write to MVL registers when time comes */
	unsigned long dif_mvl_start,
		      mi_ctrl, mi_ctrl_saved,
		      mi_mvl_y_size, mi_mvl_c_size,
		      mi_mvl_full_size,
		      scaler_ctrl, scaler_ctrl_saved,
		      scaler_sch_y, scaler_scv_y, scaler_sch_c, scaler_scv_c,
		      scaler_size_y, scaler_size_c,
		      scaler_size_cut, scaler_size_cut_h,
		      scaler_mvl_clr_y, scaler_mvl_clr_cb, scaler_mvl_clr_cr,
		      scaler_flt_y0, scaler_flt_y1, scaler_flt_y2,
		      scaler_flt_c0, scaler_flt_c1, scaler_flt_c2;

	/* Offsets to add to addresses given by upper layer */
	int mvl_odd_y_offset,  mvl_odd_c_offset,
	    mvl_even_y_offset, mvl_even_c_offset;

	/* MVL callbacks */

	/*
	 * Fetch a new video buffer and optional video parameters change.
	 * Returns non-zero if more buffers are available.
	 */
	int (*mvl_fetch_buffer)(struct mvdu_device *dev,
				struct mvdu_video_buf **buf,
				struct mvdu_video_params **params);
	/*
	 * Called when mvdu_video_buf object, previously returned by
	 * mvl_fetch_buffer(), is no longer needed.
	 */
	void (*mvl_release_buffer)(struct mvdu_device *dev,
				   struct mvdu_video_buf *buf);

	/*
	 * Called when mvdu_video_params object, previously returned by
	 * mvl_fetch_buffer(), is no longer needed.
	 */
	void (*mvl_release_params)(struct mvdu_device *dev,
				   struct mvdu_video_params *params);

	/* Interrupt processing */
	u32 irq_mask;

	/* Locking */
	spinlock_t lock;

	/* List head used to link all available module_vdu devices */
	struct list_head devices_list;

	struct device *dev;	/* for dev_* macros */
	u32 __iomem *regs;
	unsigned long regs_phys;
	int irq;

#if defined(CONFIG_FB_RCM_VDU) || defined(CONFIG_FB_RCM_VDU_MODULE)
	/* Platform-specific methods to (de)allocate framebuffer */
	int (*allocate_framebuffer)(struct mvdu_device *dev);
	int (*free_framebuffer)(struct mvdu_device *dev);

	/* module_vdu_fb driver data */
	struct fb_info *fb_info;	/* framebuffer fb_info */
	struct mutex fb_mutex;		/* lock for safe counter changing */
	int fb_open_count;		/* open counter */
	unsigned int fb_buffer_size;	/* size of allocated framebuffer */
	void *fb_buffer_cpu;		/* framebuffer CPU address */
	dma_addr_t fb_buffer_dma;	/* framebuffer physical address */
	struct mvdu_fb_area *fb_tmp_fb_areas;	/* temporary buffer for ... */
	struct mvdu_osd_area *fb_tmp_osd_areas;	/* ... area operations */
	unsigned int fb_fullscreen : 1;	/* non-zero if auto-managed fullscreen area */
	int fb_blanked;			/* flag "OSD disabled by fb_bank" */
	u32 pseudo_palette[256];	/* required for fbcon and for logo */
#endif
#if defined(CONFIG_RCM_VDU_VIDEO) || defined(CONFIG_RCM_VDU_VIDEO_MODULE)

	/* Platform-specific methods to (de)allocate video buffer */
	int (*allocate_video_buffer)(struct mvdu_device *dev);
	int (*free_video_buffer)(struct mvdu_device *dev);

	/* module_vdu_video driver data */
	struct mvdu_video_data *mvd;
	int video_buffer_size;		/* video buffer size */
	void *video_buffer_cpu;		/* video buffer CPU address */
	dma_addr_t video_buffer_dma;	/* video buffer physical address */
	struct resource vpubuffer_res; /* vpubuffer resources */
	struct v4l2_device v4l2_dev;
#endif
};

/* VDU states */
#define VDU_STATE_OFF		0	/* VDU_ENA=0, VDU is stopped */
#define VDU_STATE_ON		1	/* VDU_ENA=1, VDU is working */
#define VDU_STATE_STOPPING	2	/* VDU_ENA=0, VDU is not stopped yet */

/* OSD states */
#define OSD_STATE_OFF		0	/* OSD_ENA=0, SW_ENA=0, not waiting for enable */
#define OSD_STATE_STARTING	1	/* OSD_ENA=0, SW_ENA=0, waiting for SA */
#define OSD_STATE_ON		2	/* OSD_ENA=1, SW_ENA=1, normal operation */
#define OSD_STATE_STOPPING	3	/* OSD_ENA=1, SW_ENA=0, waiting for FR_PRC_END */

/* MVL states */
#define MVL_STATE_OFF		0	/* no streaming and no streaming request */
#define MVL_STATE_READY		1	/* MVL_ENA=0, hardware completely prepared */
#define MVL_STATE_RUNNING	2	/* MVL_ENA=1, SW_ENA=1, in VDU_STATE_ON */
#define MVL_STATE_STOPPING_S	3	/* MVL_ENA=1, SW_ENA=0, precessing streaming stop request */
#define MVL_STATE_STOPPING_O	4	/* MVL_ENA=1, SW_ENA=0, left VDU_STATE_ON */
#define MVL_STATE_STOPPING_M	5	/* MVL_ENA=1, SW_ENA=0, processing mode change */

extern struct mvdu_device *mvdu_get_device(int num);

extern int mvdu_enable_vdu(struct mvdu_device *dev);
extern int mvdu_disable_vdu(struct mvdu_device *dev);

extern int mvdu_set_mode(struct mvdu_device *dev, int mode_num,
				int call_notifier_on_busy);
extern const struct mvdu_mode *mvdu_get_modeinfo(struct mvdu_device *dev,
				int mode_num);

extern int mvdu_enable_osd(struct mvdu_device *dev);
extern int mvdu_disable_osd(struct mvdu_device *dev);

extern int mvdu_osd_set_areas(struct mvdu_device *dev, int areas_nr,
		const struct mvdu_osd_area *areas);
extern int mvdu_osd_get_areas(struct mvdu_device *dev, int *areas_nr,
		struct mvdu_osd_area *areas);

extern int mvdu_set_bgcolor(struct mvdu_device *dev,
				const struct rgb888 *color);
extern int mvdu_get_bgcolor(struct mvdu_device *dev, struct rgb888 *color);

extern int mvdu_wait_notactive(struct mvdu_device *dev);
extern int mvdu_wait_osd_fr_rd_end(struct mvdu_device *dev);
extern void mvdu_get_blanking(struct mvdu_device *dev,
					int *hblanking, int *vblanking);


extern void mvdu_mvl_merge_params(struct mvdu_video_params *params,
		const struct mvdu_video_params *update);
extern int mvdu_mvl_check_params(struct mvdu_device *dev,
		const struct mvdu_video_params *params);

extern void mvdu_mvl_announce_buffer(struct mvdu_device *dev);
extern void mvdu_mvl_stop_streaming(struct mvdu_device *dev);


#ifdef DEBUG
extern void mvdu_dump_registers(struct mvdu_device *dev);
#endif

#endif /*__LINUX_MODULE_VDU_H__*/
