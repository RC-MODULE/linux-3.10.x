#ifndef RCM_VDU_GRABBER_H
#define RCM_VDU_GRABBER_H

#include <linux/ioctl.h>
#include <linux/watchdog.h>
#include <linux/version.h>

	// CONTROL
	#define ADDR_ID_REG           0x000
	#define ADDR_ENABLE           0x004
	#define ADDR_PR_RESET         0x008
	#define ADDR_REC_ENABLE       0x00c
	// STATUS
	#define ADDR_ACTIVE_FRAME     0x100
	#define ADDR_FRAME_SIZE       0x104
	#define ADDR_FRAME_PARAM      0x108
	#define ADDR_DMA0_STATUS      0x10c
	#define ADDR_DMA1_STATUS      0x110
	#define ADDR_DMA2_STATUS      0x114
	#define ADDR_DMA_ERROR        0x118
	// INTERRUPT
	#define ADDR_INT_STATUS       0x200
	#define ADDR_INT_MASK         0x204
	#define ADDR_TEST_INT         0x208
	#define ADDR_INTERRUPTION     0x20c
	// CONVERSION
	#define ADDR_GAM_ENABLE       0x300
	#define ADDR_CONV_ENABLE      0x304
	#define ADDR_C_0_0            0x308
	#define ADDR_C_0_1            0x30c
	#define ADDR_C_0_2            0x310
	#define ADDR_C_0_3            0x314
	#define ADDR_C_1_0            0x318
	#define ADDR_C_1_1            0x31c
	#define ADDR_C_1_2            0x320
	#define ADDR_C_1_3            0x324
	#define ADDR_C_2_0            0x328
	#define ADDR_C_2_1            0x32c
	#define ADDR_C_2_2            0x330
	#define ADDR_C_2_3            0x334
	#define ADDR_CH0_RANGE        0x338
	#define ADDR_CH1_RANGE        0x33c
	#define ADDR_CH2_RANGE        0x340
	// DMA
	#define ADDR_BASE_SW_ENA      0x400
	#define ADDR_DMA0_ADDR0       0x404
	#define ADDR_DMA0_ADDR1       0x408
	#define ADDR_DMA1_ADDR0       0x40c
	#define ADDR_DMA1_ADDR1       0x410
	#define ADDR_DMA2_ADDR0       0x414
	#define ADDR_DMA2_ADDR1       0x418
	#define ADDR_Y_SIZE           0x41c
	#define ADDR_C_SIZE           0x420
	#define ADDR_FULL_LINE_SIZE   0x424
	#define ADDR_MODE             0x428
	#define ADDR_LOCATION_DATA    0x42c
	#define ADDR_TRANSPARENCY     0x430
	#define ADDR_BASE_POINT       0x434
	#define ADDR_DMA_ID           0x438
	#define ADDR_AXI_PARAM        0x43c
	// GAMMA
	#define BASE_ADDR_TABLE_0     0xd00 // address of table gamma-correction
	#define BASE_ADDR_TABLE_1     0xe00 // address of table gamma-correction
	#define BASE_ADDR_TABLE_2     0xf00 // address of table gamma-correction

	#define INT_BIT_CH2_OVR		(1<<31)	// write to the full channel buffer 2
	#define INT_BIT_CH1_OVR		(1<<28)	// write to the full channel buffer 1
	#define INT_BIT_CH0_OVR		(1<<25)	// write to the full channel buffer 0
	#define INT_BIT_RCV_END		(1<<18)	// frame capture end
	#define INT_BIT_VSYNC		(1<<16)	// vertical sync signal front
	#define INT_BIT_HSYNC		(1<<15)	// horisontal sync signal front
	#define INT_BIT_ST_ERROR	(1<<14)	// discrepancy in the number of buses on which video data is transmitted in adjacent frames
	#define INT_BIT_VS_ERROR	(1<<13)	// mismatch of the number of lines in adjacent frames
	#define INT_BIT_HS_ERROR	(1<<12)	// mismatch of the number of points in adjacent frames
	#define INT_BIT_DONE		(1<<11)	// frame recognition end
	#define INT_BIT_GRB_OFF		(1<<8)	// video capture device activity state
	#define INT_BIT_END_WRITE	(1<<7)	// end of frame writing to memory
	#define INT_BIT_UNC_ERR		(1<<6)	// unrecoverable sync error EAV and SAV
	#define INT_BIT_REPL		(1<<5)	// EAV and SAV sync error correction
	#define INT_BIT_ERR_CODE	(1<<4)	// sync error EAV and SAV
	#define INT_BIT_DMA2_ERROR	(1<<3)	// AXI DMA error, channel 2
	#define INT_BIT_DMA1_ERROR	(1<<2)	// AXI DMA error, channel 1
	#define INT_BIT_DMA0_ERROR	(1<<1)	// AXI DMA error, channel 0
	#define INT_BIT_TEST		(1<<0)	// test interrupt

	#define EXTERNAL			0
	#define INTERNAL			1
	
	#define OFF					0
	#define ON					1
	
	#define YCBCR				0
	#define RGB					1
	
	#define D_FMT_YCBCR422		0
	#define D_FMT_YCBCR444		1
	#define D_FMT_RGB888		2
	
	#define STD_CLR_HD			0
	#define STD_CLR_SD			1

	#define V_IF_SERIAL			0
	#define V_IF_PARALLEL		1

	#define SYNC_EXTERNAL		0
	#define SYNC_INTERNAL		1

	#define RCM_GRB_CID_SYNC		(V4L2_CID_PRIVATE_BASE + 0)	// SYNC_xx..
	#define RCM_GRB_CID_STD_IN		(V4L2_CID_PRIVATE_BASE + 1)	// STD_CLR_xx
	#define RCM_GRB_CID_D_V_IF		(V4L2_CID_PRIVATE_BASE + 2)	// V_IF_xx..
	#define RCM_GRB_CID_D_FORMAT	(V4L2_CID_PRIVATE_BASE + 3)	// D_FMT_xx..
	#define RCM_GRB_CID_STD_OUT		(V4L2_CID_PRIVATE_BASE + 4)	// STD_CLR_xx

// custom code - todo standard
	#define VIDIOC_SET_GAMMA		_IOWR('v', BASE_VIDIOC_PRIVATE + 0, struct grb_gamma)
	#define VIDIOC_G_PARAMS			_IOWR('v', BASE_VIDIOC_PRIVATE + 1, struct grb_parameters)
	#define VIDIOC_S_PARAMS			_IOWR('v', BASE_VIDIOC_PRIVATE + 2, struct grb_parameters)
	#define VIDIOC_AUTO_DETECT		_IOWR('v', BASE_VIDIOC_PRIVATE + 3, struct grb_parameters)

	#define RCM_GRB_DEVICE_NAME "rcm_vdu_grb_dev"
	#define RCM_GRB_DRIVER_NAME "rcm_vdu_grb_drv"
	#define RCM_GRB_DRIVER_VERSION KERNEL_VERSION(1,0,0)

	#define RCM_GRB_DEVID 0xec176627

	#define PHYS_TO_DMA(A) ((A)|0x40000000)
	#define U16x2_TO_U32(H,L) (((H)<<16)|(L))
	#define U8x4_TO_U32(U0,U1,U2,U3) (((u32)U0<<0)+((u32)U1<<8)+((u32)U2<<16)+((u32)U3<<24))

	#define NUM_OUTPUT_FORMATS 4

	struct grb_parameters {
		u32 sync;
		u32 std_in;
		u32 v_if;
		u32 d_format;
		u32 std_out;
		u32 alpha;
	};

	struct input_format {
		u32 color;
		u32 color_std;
		u32	format_din;
	};

	struct output_format {
		u32 color;
		u32 color_std;
		u32 format_dout;
		u32 y_hor_size, y_ver_size, y_full_size;
		u32 c_hor_size, c_ver_size, c_full_size;
	};

	struct grb_gamma {
		u32 active_gamma;
		int table_Y_G[256];
		int table_C_R[256];
		int table_C_B[256];
	};

	struct coef_conv {
		u32	coef[3][4];
		u32	range[3];
	};

	static const struct coef_conv RGB_TO_YCBCR_SD = {
		{ { (0<<18)+16*256,  (0<<10)+129, (0<<10)+66,  (0<<10)+25 },
		  { (0<<18)+128*256, (1<<10)+94,  (0<<10)+112, (1<<10)+18 },
		  { (0<<18)+128*256, (1<<10)+74,  (1<<10)+38,  (0<<10)+112 } },
		{ (235<<16)+16, (240<<16)+16, (240<<16)+16 }
	};

	static const struct coef_conv RGB_TO_YCBCR_HD = {
		{ { (0<<18)+16*256,  (0<<10)+157, (0<<10)+47,  (0<<10)+16 },
		  { (0<<18)+128*256, (1<<10)+102, (0<<10)+112, (1<<10)+10 },
 		  { (0<<18)+128*256, (1<<10)+87,  (1<<10)+26,  (0<<10)+112 } },
		{ (235<<16)+16, (240<<16)+16, (240<<16)+16 }
	};

	static const struct coef_conv YCBCR_TO_RGB_SD = {
		{ { (0<<18)+34656, (0<<10)+298, (1<<10)+208, (1<<10)+100 },
		  { (1<<18)+57120, (0<<10)+298, (0<<10)+409, (0<<10)+0 },
		  { (1<<18)+70944, (0<<10)+298, (0<<10)+0,   (0<<10)+517 } },
		{ (255<<16)+0, (255<<16)+0, (255<<16)+0 }
	};

	static const struct coef_conv YCBCR_TO_RGB_HD = {
		{ { (0<<18)+19680, (0<<10)+298, (1<<10)+136, (1<<10)+55 },
		  { (1<<18)+63520, (0<<10)+298, (0<<10)+459, (0<<10)+0 },
		  { (1<<18)+74016, (0<<10)+298, (0<<10)+0,   (0<<10)+541 } },
		{ (255<<16)+0, (255<<16)+0, (255<<16)+0 }
	};

	static const struct coef_conv YCBCR_SD_TO_HD = {
		{ { (0<<18)+10624, (0<<10)+256, (1<<10)+53,  (1<<10)+30 },
		  { (1<<18)+3328,  (0<<10)+0,   (0<<10)+263, (0<<10)+19 },
		  { (1<<18)+4352,  (0<<10)+0,   (0<<10)+29,  (0<<10)+261 } },
		{ (235<<16)+16, (240<<16)+16, (240<<16)+16 }
	};

	static const struct coef_conv YCBCR_HD_TO_SD = {
		{ { (1<<18)+9472, (0<<10)+256, (0<<10)+49,  (0<<10)+25 },
		  { (0<<18)+2944, (0<<10)+0,   (0<<10)+252, (1<<10)+19 },
		  { (0<<18)+3968, (0<<10)+0,   (1<<10)+28,  (0<<10)+253 } },
		{ (235<<16)+16, (240<<16)+16, (240<<16)+16 }
	};


struct grb_info {
	struct device *dev;
	struct video_device video_dev;
	struct videobuf_queue videobuf_queue_grb;
	struct videobuf_buffer* cur_buf;
	struct list_head buffer_queue;
	unsigned int frame_count;
	unsigned int reqv_buf_cnt;
	unsigned int next_buf_num;
	int num_irq;
	//wait_queue_head_t wait_queue;
	struct completion cmpl;
	spinlock_t irq_lock;
	struct v4l2_device v4l2_device;
	struct v4l2_pix_format user_format;
	struct v4l2_rect cropping;
	struct v4l2_pix_format recognize_format;
	struct input_format in_f;
	struct output_format out_f;
	struct grb_parameters param;
	struct grb_gamma gam;
	const struct coef_conv* c_conv;
	phys_addr_t phys_addr_regs_grb;
	void __iomem *base_addr_regs_grb;
	phys_addr_t buff_phys_addr;
	dma_addr_t buff_dma_addr;
	void* kern_virt_addr;
	u32 mem_offset1, mem_offset2;		// fill set_register
	u32 buff_length;					// fiil buf_setup
};

#endif // RCM_VDU_GRABBER_H