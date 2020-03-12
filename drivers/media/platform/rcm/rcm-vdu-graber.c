// ****************************************************************************
// 
//						INCLUDES
//
// ****************************************************************************

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gfp.h>  //??? may be don"t need
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mm.h>  //??? may be don"t need
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/of_platform.h>

#include <media/videobuf-dma-contig.h>
#include <media/videobuf-core.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <asm/delay.h>
#include <asm/io.h>
#include <asm/page.h>
//#include <asm/segment.h>
#include <asm/uaccess.h>

#include "rcm-vdu-graber.h"

#define SUCCESS 0
#define DEVICE_NAME "grb_v2" /* Имя нашего устройства */
#define DRIVER_NAME "grb_driver"

#define V4L2_DEBUG_IOCTL     0x01
#define V4L2_DEBUG_IOCTL_ARG 0x02


#define ADD 0
#define DEL 1


static int debug = 0xFF;
module_param(debug, int, 0644);

MODULE_LICENSE			( "Dual BSD/GPL" );
MODULE_SUPPORTED_DEVICE ( "grb_v2" ); /* /dev/testdevice */

#define dprintk(level, fmt, arg...)						\
	do {												\
		if (debug >= level)								\
			printk(KERN_DEBUG "grb: " fmt, ## arg);	\
	} while (0)

struct input_format
{
	u32 color;
	u32 color_std;
	
	u32	format_din;
};

struct output_format
{
	u32 color;
	u32 color_std;
	
	u32	format_dout;
	
	u32	y_hor_size;
	u32	y_ver_size;
	u32	y_full_size;
	
	u32	c_hor_size;
	u32	c_ver_size;
	u32	c_full_size;
	
};

struct coef_conv
{
	u32	coef0_0;
	u32	coef0_1;
	u32	coef0_2;
	u32	coef0_3;
	
	u32	coef1_0;
	u32	coef1_1;
	u32	coef1_2;
	u32	coef1_3;
	
	u32	coef2_0;
	u32	coef2_1;
	u32	coef2_2;
	u32	coef2_3;
	
	u32	range_0;
	u32	range_1;
	u32	range_2;	
};

struct grb_info
{
	struct  file file_info;
	struct  device *dev;
	struct  video_device video_dev;
	
	struct  videobuf_queue videobuf_queue_grb;
	struct  list_head buffer_queue;	
	char	frame_count;	
	
	int num_irq;
	wait_queue_head_t queue;
	int irq_stat;
	
	spinlock_t irqlock ;

	struct 	v4l2_device v4l2_device;
	struct 	v4l2_pix_format	user_format;
	struct 	v4l2_rect		cropping;
	
	struct 	v4l2_pix_format	recognize_format;
	
	struct 	input_format in_f;
	struct 	output_format out_f;
	struct  grb_parameters param;
	u32 	active_gamma;
	struct 	grb_gamma *gam ;
	struct 	coef_conv c_conv;

	void __iomem *base_addr_regs_grb;
	void __iomem *base_addr_mem_area;
	
	u64		buff_phys_addr	;
	u32		mem_offset1		;
	u32		mem_offset2		;
	u32		buff_length		;
	
	u32		grb_phys_addr	;
	
	u32		frame_height	;
	u32		frame_width		;
};

void write_register(u32 __debug, u32 val, void __iomem *addr, u32 offset)
{
	iowrite32(val, addr + offset);
	dprintk(__debug, "set(0x%08X, 0x%08X)\n", offset, val);
}

u32 read_register(u32 __debug, void __iomem *addr, u32 offset)
{
	u32 r;
	
	r = ioread32(addr + offset);
	dprintk(__debug, "get(0x%08X, 0x%08X)\n", offset, r);
	return r;
}


static void set_mask(void __iomem *addr, u32 offset, int bit , u32 action)
{
	u32 mask;
	
	mask = read_register(2, addr, offset);
	
	if (action == ADD)
	{
		write_register(2,  mask + (1 << bit), addr, offset);
	}
	else // if (action == DEL)
	{
		write_register(2,  mask - (1 << bit), addr, offset);
	}
}

int set_input_format(struct grb_info *grb_info_ptr, struct grb_parameters *param)
{
	u32 active_bus  ;
	u32 format_data ;
	
//	active_bus = 0x0 ;
//	format_data = 0x0 ;
	
	grb_info_ptr->param = *param ;
	
	if (param->d_format == YCBCR422) {
		
		format_data  = 0x0 ; // YCbCr
		if (param->std_in == SD) {
			if (param->v_if == SERIAL)
				active_bus = 0x1 ;
		
			else if (param->v_if == PARALLEL)
				active_bus = 0x3 ;
			else {
				active_bus = 0x0 ;
				dprintk(1, "invalid parameter param.v_if!\n"); 
				return -EINVAL;
			}
		}
		else if (param->std_in == HD) {
			active_bus = 0x2 ;
		}
		else {
			active_bus = 0x0 ;
			dprintk(1, "invalid parameter param.std_in!\n"); 
			return -EINVAL;
		}
		
		grb_info_ptr->in_f.color = YCBCR ;
		dprintk(1, "input format is YCBCR 4:2:2.\n"); 
	}
	else if (param->d_format == YCBCR444) {
		format_data = 0x0 ; // YCbCr
		active_bus  = 0x3 ;
		
		grb_info_ptr->in_f.color = YCBCR ;
		dprintk(1, "input format is YCBCR 4:4:4.\n"); 
	} 
	else if (param->d_format == RGB888) {
		format_data = 0x1 ;  // RGB
		active_bus  = 0x3 ;
		
		grb_info_ptr->in_f.color = RGB ;
		dprintk(1, "input format is YCBCR 4:4:4.\n"); 
	}
	else {
		active_bus  = 0x0	;
		format_data = 0x0	;
		
		grb_info_ptr->in_f.color = 0x0 ;
		dprintk(1, "std_grb: unknown standard \n");
		return -EINVAL;
	}
	
	grb_info_ptr->in_f.format_din  = format_data ; // YCbCr
	grb_info_ptr->in_f.format_din += (active_bus << 1) ;
	grb_info_ptr->in_f.format_din += (param->std_in << 3) ;
	grb_info_ptr->in_f.format_din += (param->sync << 4) ;
	
	grb_info_ptr->in_f.color_std = param->std_in ;
	
	grb_info_ptr->out_f.color_std = param->std_out  ;
	
	if (param->alpha > 255) 
		return -1;

	return 0;
}

int set_output_format(
		struct grb_info *grb_info_ptr,
		struct v4l2_format *v4l2_format_ptr)
{

	grb_info_ptr->user_format = v4l2_format_ptr->fmt.pix;

	if (v4l2_format_ptr->fmt.pix.pixelformat == V4L2_PIX_FMT_BGR32) {
		grb_info_ptr->out_f.format_dout = 0x3 ;
		grb_info_ptr->out_f.color		= RGB ;
		dprintk(1, "pixelformat: ARGB8888. \n"); 
	}
	else if (v4l2_format_ptr->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32) {
		grb_info_ptr->out_f.format_dout = 0xe ;
		grb_info_ptr->out_f.color		= RGB ;
		dprintk(1, "pixelformat: RGB888. \n"); 
	}
	else if (v4l2_format_ptr->fmt.pix.pixelformat == V4L2_PIX_FMT_NV16) {
		grb_info_ptr->out_f.format_dout = 0x4	;
		grb_info_ptr->out_f.color		= YCBCR ;
		dprintk(1, "pixelformat: YCBCR422 two planes. \n");
	}
	else if (v4l2_format_ptr->fmt.pix.pixelformat == V4L2_PIX_FMT_NV61) {
		grb_info_ptr->out_f.format_dout = 0x6	;
		grb_info_ptr->out_f.color		= YCBCR ;
		dprintk(1, "pixelformat: YCBCR422 three planes.\n");
	}
	else if (v4l2_format_ptr->fmt.pix.pixelformat == V4L2_PIX_FMT_NV24) {
		grb_info_ptr->out_f.format_dout = 0xe	;
		grb_info_ptr->out_f.color		= YCBCR ;
		dprintk(1, "pixelformat: YCBCR444 three planes.\n");
	}
	else {
		grb_info_ptr->out_f.format_dout = 0x0	;
		dprintk(1, "pixelformat: unknown pixelformat! \n");
		return -EINVAL;
	}
	
	grb_info_ptr->out_f.y_hor_size = v4l2_format_ptr->fmt.pix.width;
	grb_info_ptr->out_f.y_ver_size = v4l2_format_ptr->fmt.pix.height;
	grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
	
	grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
	grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
	grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;

	if ( (grb_info_ptr->out_f.y_hor_size > grb_info_ptr->out_f.y_full_size) ||
		 (grb_info_ptr->out_f.c_hor_size > grb_info_ptr->out_f.c_full_size) )
		return -1;
	
	return 0;
}

int colour_conversion(struct grb_info *grb_info_ptr)
{
	u32 color_in;
	u32 color_std_in;
	
	u32 color_out;
	u32 color_std_out;

	color_in 		= grb_info_ptr->in_f.color ;
	color_std_in 	= grb_info_ptr->in_f.color_std ;
	
	color_out 		= grb_info_ptr->out_f.color ;
	color_std_out 	= grb_info_ptr->out_f.color_std ;
	
	if (color_in == RGB)
		dprintk(1, "color_input format is RGB.\n");
	else if (color_in == YCBCR)
		dprintk(1, "color_input format is YCBCR.\n");
	else 
		return -EINVAL;
		
	if (color_std_in == SD)
		dprintk(1, "color_input standard is SD.\n");
	else if (color_std_in == HD)
		dprintk(1, "color_input standard is HD.\n");
	else 
		return -EINVAL;
		
	if (color_out == RGB)
		dprintk(1, "color_output format is RGB.\n");
	else  if (color_out == YCBCR)
		dprintk(1, "color_output format is YCBCR.\n");
	else 
		return -EINVAL;
	
	if (color_std_out == SD)
		dprintk(1, "color_output standard is SD.\n");
	else  if (color_std_out == HD)
		dprintk(1, "color_output standard is HD.\n");
	else 
		return -EINVAL;
	
	if (color_in != color_out) { 
		if (color_in == RGB) {
		
			dprintk(1, "enable colour conversion from RGB to YCBCR.\n");
		
			if (color_std_out == SD) {
			
				dprintk(1, "output colour standard is SD.\n");
				
				grb_info_ptr->c_conv.coef0_0 = (0<<18) + 16*256 ;
				grb_info_ptr->c_conv.coef0_1 = (0<<10) + 129    ;
				grb_info_ptr->c_conv.coef0_2 = (0<<10) +  66    ;
				grb_info_ptr->c_conv.coef0_3 = (0<<10) +  25    ;
				
				grb_info_ptr->c_conv.coef1_0 = (0<<18) + 128*256 ;
				grb_info_ptr->c_conv.coef1_1 = (1<<10) +  94     ;
				grb_info_ptr->c_conv.coef1_2 = (0<<10) + 112     ;
				grb_info_ptr->c_conv.coef1_3 = (1<<10) +  18     ;
				
				grb_info_ptr->c_conv.coef2_0 = (0<<18) + 128*256 ;
				grb_info_ptr->c_conv.coef2_1 = (1<<10) +  74     ;
				grb_info_ptr->c_conv.coef2_2 = (1<<10) +  38     ;
				grb_info_ptr->c_conv.coef2_3 = (0<<10) + 112     ;
			}
			else {
			
				dprintk(1, "output colour standard is HD.\n");
				
				grb_info_ptr->c_conv.coef0_0 = (0<<18) + 16*256 ;
				grb_info_ptr->c_conv.coef0_1 = (0<<10) + 157    ;
				grb_info_ptr->c_conv.coef0_2 = (0<<10) +  47    ;
				grb_info_ptr->c_conv.coef0_3 = (0<<10) +  16    ;
				
				grb_info_ptr->c_conv.coef1_0 = (0<<18) + 128*256 ;
				grb_info_ptr->c_conv.coef1_1 = (1<<10) + 102     ;
				grb_info_ptr->c_conv.coef1_2 = (0<<10) + 112     ;
				grb_info_ptr->c_conv.coef1_3 = (1<<10) +  10     ;
				
				grb_info_ptr->c_conv.coef2_0 = (0<<18) + 128*256 ;
				grb_info_ptr->c_conv.coef2_1 = (1<<10) +  87     ;
				grb_info_ptr->c_conv.coef2_2 = (1<<10) +  26     ;
				grb_info_ptr->c_conv.coef2_3 = (0<<10) + 112     ;
			}
			grb_info_ptr->c_conv.range_0 = (235<<16) + 16 ;
			grb_info_ptr->c_conv.range_1 = (240<<16) + 16 ;
			grb_info_ptr->c_conv.range_2 = (240<<16) + 16 ;
	
			return 0;
		}
		else if (color_in == YCBCR) {
		
			dprintk(1, "enable colour conversion from YCBCR to RGB.\n");
		
			if (color_std_in == SD) {
			
				dprintk(1, "input colour standard is SD.\n");
				
				grb_info_ptr->c_conv.coef0_0 = (0<<18) + 34656 ;
				grb_info_ptr->c_conv.coef0_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef0_2 = (1<<10) + 208   ;
				grb_info_ptr->c_conv.coef0_3 = (1<<10) + 100   ;
				
				grb_info_ptr->c_conv.coef1_0 = (1<<18) + 57120 ;
				grb_info_ptr->c_conv.coef1_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef1_2 = (0<<10) + 409   ;
				grb_info_ptr->c_conv.coef1_3 = (0<<10) + 0     ;
	            
				grb_info_ptr->c_conv.coef2_0 = (1<<18) + 70944 ;
				grb_info_ptr->c_conv.coef2_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef2_2 = (0<<10) +   0   ;
				grb_info_ptr->c_conv.coef2_3 = (0<<10) + 517   ;
			}
			else {
			
				dprintk(1, "input colour standard is HD.\n");
				
				grb_info_ptr->c_conv.coef0_0 = (0<<18) + 19680 ;
				grb_info_ptr->c_conv.coef0_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef0_2 = (1<<10) + 136   ;
				grb_info_ptr->c_conv.coef0_3 = (1<<10) +  55   ;
								
				grb_info_ptr->c_conv.coef1_0 = (1<<18) + 63520 ;
				grb_info_ptr->c_conv.coef1_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef1_2 = (0<<10) + 459   ;
				grb_info_ptr->c_conv.coef1_3 = (0<<10) +   0   ;
								
				grb_info_ptr->c_conv.coef2_0 = (1<<18) + 74016 ;
				grb_info_ptr->c_conv.coef2_1 = (0<<10) + 298   ;
				grb_info_ptr->c_conv.coef2_2 = (0<<10) +   0   ;
				grb_info_ptr->c_conv.coef2_3 = (0<<10) + 541   ;
			}
			grb_info_ptr->c_conv.range_0 = (255<<16) + 0;
			grb_info_ptr->c_conv.range_1 = (255<<16) + 0;
			grb_info_ptr->c_conv.range_2 = (255<<16) + 0;
			
			return 0;
		}
	}
	else if ( (color_in == YCBCR) & (color_out == YCBCR) ) {
		
		dprintk(1, "enable conversion colour standard  for YCBCR.\n");
		
		if ( (color_std_in == SD) & (color_std_out == HD) ) {
			
			dprintk(1, "conversion from SD to HD.\n");
				
			grb_info_ptr->c_conv.coef0_0 = (0<<18) + 10624 ;
			grb_info_ptr->c_conv.coef0_1 = (0<<10) + 256   ;
			grb_info_ptr->c_conv.coef0_2 = (1<<10) +  53   ;
			grb_info_ptr->c_conv.coef0_3 = (1<<10) +  30   ;
						
			grb_info_ptr->c_conv.coef1_0 = (1<<18) + 3328  ;
			grb_info_ptr->c_conv.coef1_1 = (0<<10) +   0   ;
			grb_info_ptr->c_conv.coef1_2 = (0<<10) + 263   ;
			grb_info_ptr->c_conv.coef1_3 = (0<<10) +  19   ;
						
			grb_info_ptr->c_conv.coef2_0 = (1<<18) + 4352  ;
			grb_info_ptr->c_conv.coef2_1 = (0<<10) +   0   ;
			grb_info_ptr->c_conv.coef2_2 = (0<<10) +  29   ;
			grb_info_ptr->c_conv.coef2_3 = (0<<10) + 261   ;
		}
		else if ( (color_std_in == HD) & (color_std_out == SD) ) {
			
			dprintk(1, "conversion from HD to SD.\n");
			
			grb_info_ptr->c_conv.coef0_0 = (1<<18) + 9472 ;
			grb_info_ptr->c_conv.coef0_1 = (0<<10) + 256  ;
			grb_info_ptr->c_conv.coef0_2 = (0<<10) +  49  ;
			grb_info_ptr->c_conv.coef0_3 = (0<<10) +  25  ;
						
			grb_info_ptr->c_conv.coef1_0 = (0<<18) + 2944 ;
			grb_info_ptr->c_conv.coef1_1 = (0<<10) +   0  ;
			grb_info_ptr->c_conv.coef1_2 = (0<<10) + 252  ;
			grb_info_ptr->c_conv.coef1_3 = (1<<10) +  19  ;
						
			grb_info_ptr->c_conv.coef2_0 = (0<<18) + 3968 ;
			grb_info_ptr->c_conv.coef2_1 = (0<<10) +   0  ;
			grb_info_ptr->c_conv.coef2_2 = (1<<10) +  28  ;
			grb_info_ptr->c_conv.coef2_3 = (0<<10) + 253  ;
		}
		else {
			
			dprintk(1, "invalid colour conversion format.\n");
			
			return -EINVAL;
		}
		
		grb_info_ptr->c_conv.range_0 = (235<<16) + 16 ;
		grb_info_ptr->c_conv.range_1 = (240<<16) + 16 ;
		grb_info_ptr->c_conv.range_2 = (240<<16) + 16 ;
		
		return 0;
	}
	
	return -1;
}

int set_register (struct grb_info *grb_info_ptr)
{
	void __iomem *base_addr;
	
	u32 rd;
	
	u32 y_hor_size ;
	u32 y_ver_size ;
	u32 y_full_size;
	
	u32 c_ver_size ;
	u32 c_hor_size ;
	u32 c_full_size ;
	
	u32 mem_offset1, mem_offset2 ;
	
	u32 base_point_y ;
	u32 base_point_x ;
	
	u32 format_din  ;
	u32 format_dout ;

	struct grb_gamma *gam ;
	
	u32 value_ch0, value_ch1, value_ch2;
	
	u32 base_addr0_dma0 ;
	u32 base_addr1_dma0 ;
	u32 base_addr0_dma1 ;
	u32 base_addr1_dma1 ;
	u32 base_addr0_dma2 ;
	u32 base_addr1_dma2 ;
	
	u32 y_size         ;
	u32 c_size         ;
	u32 full_line_size ;
	
	u32 base_point ;
	u32 alpha ;

	int i, j;
	
	base_addr = grb_info_ptr->base_addr_regs_grb;
	
	format_din  = grb_info_ptr->in_f.format_din;
	
	format_dout = grb_info_ptr->out_f.format_dout;

	dprintk(1, "format_din: 0x%0x\n", format_din);
	dprintk(1, "format_dout: 0x%0x\n", format_dout);
	
	if (grb_info_ptr->cropping.width != 0 && grb_info_ptr->cropping.height !=0) {
		y_hor_size	= grb_info_ptr->cropping.width  ;
		y_ver_size	= grb_info_ptr->cropping.height ;
		
		c_hor_size	= grb_info_ptr->cropping.width  ;
		c_ver_size	= grb_info_ptr->cropping.height ;
	}
	else {
		y_hor_size	= grb_info_ptr->out_f.y_hor_size ;
		y_ver_size	= grb_info_ptr->out_f.y_ver_size ;
		
		c_hor_size	= grb_info_ptr->out_f.c_hor_size ;
		c_ver_size	= grb_info_ptr->out_f.c_ver_size ;
	}
	
	y_full_size = grb_info_ptr->out_f.y_full_size ;
	c_full_size	= grb_info_ptr->out_f.c_full_size ;
		
	if (format_dout == 0x3) {
		dprintk(1, "pixelformat: ARGB8888.\n"); 
	}
	else if (format_dout == 0x4) {
		dprintk(1, "pixelformat: YCBCR422 two planes.\n");
	}
	else if (format_dout == 0x6) {
		c_hor_size = c_hor_size/2;
		c_full_size = c_full_size/2;
		dprintk(1, "pixelformat: YCBCR422 three planes.\n");
	}
	else if (format_dout == 0xe) {
		dprintk(1, "pixelformat: YCBCR444 three planes.\n");
	}
	else {
		dprintk(1, "pixelformat: unknown pixelformat! \n");
		return -EINVAL;
	}

	mem_offset1 = y_full_size*y_ver_size;
	mem_offset2 = mem_offset1 + c_full_size*c_ver_size;
	
	grb_info_ptr->mem_offset1 = mem_offset1 ;
	grb_info_ptr->mem_offset2 = mem_offset2 ;
	
	dprintk(1, "y_hor_size: %d.\n", y_hor_size );
	dprintk(1, "y_ver_size: %d.\n", y_ver_size );

	dprintk(1, "c_hor_size: %d.\n", c_hor_size );
	dprintk(1, "c_ver_size: %d.\n", c_ver_size );
	
	dprintk(1, "y_full_size: %d.\n", y_full_size );
	dprintk(1, "c_full_size: %d.\n", c_full_size );
	
	dprintk(1, "mem_offset1: %d.\n", mem_offset1 );
	dprintk(1, "mem_offset2: %d.\n", mem_offset2 );
	
	base_point_y = grb_info_ptr->cropping.top	;
	base_point_x = grb_info_ptr->cropping.left	;
	
	alpha = grb_info_ptr->param.alpha	;

	dprintk(1, "alpha: %d.\n", alpha );
	
//	base_addr0_dma0 = 0x41001000	;
//	base_addr1_dma0 = 0x41001000	;
	
	base_addr0_dma0 = videobuf_to_dma_contig(grb_info_ptr->videobuf_queue_grb.bufs[0]);
	base_addr1_dma0 = videobuf_to_dma_contig(grb_info_ptr->videobuf_queue_grb.bufs[1]);
	
	base_addr0_dma1 = base_addr0_dma0 + mem_offset1 ;
	base_addr1_dma1 = base_addr0_dma0 + mem_offset1 ;
	
	base_addr0_dma2 = base_addr0_dma0 + mem_offset2 ;
	base_addr1_dma2 = base_addr0_dma0 + mem_offset2 ;

	y_size          = (y_ver_size << 16) + y_hor_size   ;
	c_size          = (c_ver_size << 16) + c_hor_size   ;
	full_line_size  = (c_full_size << 16) + y_full_size ;
	
	base_point		= (base_point_y << 16) + base_point_x ;
	
	write_register(1, 0x1				, base_addr, ADDR_PR_RESET		);
	
	do rd = read_register(1, base_addr, ADDR_PR_RESET);
	while (rd != 0);
	
	write_register(1, 0x1     			, base_addr, ADDR_BASE_SW_ENA		);
	write_register(1, base_addr0_dma0	, base_addr, ADDR_DMA0_ADDR0		);
	write_register(1, base_addr1_dma0	, base_addr, ADDR_DMA0_ADDR1		);
	write_register(1, base_addr0_dma1	, base_addr, ADDR_DMA1_ADDR0		);
	write_register(1, base_addr1_dma1	, base_addr, ADDR_DMA1_ADDR1		);
	write_register(1, base_addr0_dma2	, base_addr, ADDR_DMA2_ADDR0		);
	write_register(1, base_addr1_dma2	, base_addr, ADDR_DMA2_ADDR1		);

	write_register(1, y_size        	, base_addr, ADDR_Y_SIZE        	);
	write_register(1, c_size        	, base_addr, ADDR_C_SIZE        	);
	write_register(1, full_line_size	, base_addr, ADDR_FULL_LINE_SIZE	);
	
	write_register(1, format_din		, base_addr, ADDR_MODE         	);
	write_register(1, format_dout		, base_addr, ADDR_LOCATION_DATA	);
	write_register(1, alpha			    , base_addr, ADDR_TRANSPARENCY 	);
	write_register(1, base_point  		, base_addr, ADDR_BASE_POINT		);
		
	if (grb_info_ptr->active_gamma == OFF)
	{
		write_register(1,  0	, base_addr, ADDR_GAM_ENABLE	);
		dprintk(1, "VIDIOC_SET_GAMMA: active_gamma == OFF \n");
	}
	else if (grb_info_ptr->active_gamma == ON)
	{
		write_register(1,  0	, base_addr, ADDR_GAM_ENABLE	);
		
		gam = grb_info_ptr->gam ;
		
		for (i=0; i<256/4; i++)
		{
			value_ch0 = 0;
		    value_ch1 = 0;
		    value_ch2 = 0;
		
			for (j=0; j<4; j++)
			{
				value_ch0 = value_ch0 + ( gam->table_Y_G[4*i + j] << j*8 );
				value_ch1 = value_ch1 + ( gam->table_C_R[4*i + j] << j*8 );
				value_ch2 = value_ch2 + ( gam->table_C_B[4*i + j] << j*8 );
			}
			
			write_register(1,  value_ch0 , base_addr, BASE_ADDR_TABLE_0 + i*4 );
			write_register(1,  value_ch1 , base_addr, BASE_ADDR_TABLE_1 + i*4 );
			write_register(1,  value_ch2 , base_addr, BASE_ADDR_TABLE_2 + i*4 );
			
		}
		write_register(1,  1	, base_addr, ADDR_GAM_ENABLE	);
	}
	
	if (colour_conversion(grb_info_ptr) == 0)
	{
		write_register(1,  grb_info_ptr->c_conv.coef0_0 , base_addr, ADDR_C_0_0	);
		write_register(1,  grb_info_ptr->c_conv.coef0_1 , base_addr, ADDR_C_0_1	);
		write_register(1,  grb_info_ptr->c_conv.coef0_2 , base_addr, ADDR_C_0_2	);
		write_register(1,  grb_info_ptr->c_conv.coef0_3 , base_addr, ADDR_C_0_3	);
				
		write_register(1,  grb_info_ptr->c_conv.coef1_0 , base_addr, ADDR_C_1_0	);
		write_register(1,  grb_info_ptr->c_conv.coef1_1 , base_addr, ADDR_C_1_1	);
		write_register(1,  grb_info_ptr->c_conv.coef1_2 , base_addr, ADDR_C_1_2	);
		write_register(1,  grb_info_ptr->c_conv.coef1_3 , base_addr, ADDR_C_1_3	);
				
		write_register(1,  grb_info_ptr->c_conv.coef2_0 , base_addr, ADDR_C_2_0	);
		write_register(1,  grb_info_ptr->c_conv.coef2_1 , base_addr, ADDR_C_2_1	);
		write_register(1,  grb_info_ptr->c_conv.coef2_2 , base_addr, ADDR_C_2_2	);
		write_register(1,  grb_info_ptr->c_conv.coef2_3 , base_addr, ADDR_C_2_3	);
		
		write_register(1,  grb_info_ptr->c_conv.range_0 , base_addr, ADDR_CH0_RANGE);
		write_register(1,  grb_info_ptr->c_conv.range_1 , base_addr, ADDR_CH1_RANGE);
		write_register(1,  grb_info_ptr->c_conv.range_2 , base_addr, ADDR_CH2_RANGE);
	
		write_register(1,  1							, base_addr, ADDR_CONV_ENABLE);
	}
	else if (colour_conversion(grb_info_ptr) != -1)
		dprintk(1, "colour_conversion error.") ;
	
	write_register(1, 0xffffffff	, base_addr, ADDR_INT_STATUS	);	
	
	set_mask(base_addr, ADDR_INT_MASK , END_WRITE , ADD);
	
	write_register(1, 1				, base_addr, ADDR_ENABLE    	);
	
	dprintk(1, "set_registered finish successfully.") ;
	return 0;
}

// ****************************************************************************
// 
//						VIDEOBUF QUEUE OPERATIONS
//
// ****************************************************************************

static int buf_setup_grb (
			struct videobuf_queue *q,
			unsigned int *count,
			unsigned int *size)
{
	struct grb_info *grb_info_ptr = q->priv_data;

	int max_buff;
	
	*size = grb_info_ptr->user_format.sizeimage;
	
	max_buff = grb_info_ptr->buff_length / *size;
	
	dprintk(1, "buf_setup: max_buff = %d;\n", max_buff);
	
	if (*count < 2)
		*count = 2;
	else if (*count > max_buff)
		*count = max_buff;

	return 0;
}

static int buf_prepare_grb (
			struct videobuf_queue *q,
			struct videobuf_buffer *vb,
			enum v4l2_field field)
{
	struct grb_info *grb_info_ptr = q->priv_data;

	dprintk(1, "buf_prepare \n");
	
	vb->size	= grb_info_ptr->user_format.sizeimage;
	vb->width	= grb_info_ptr->user_format.bytesperline;
	vb->height	= grb_info_ptr->user_format.height;
	vb->field	= field;
	
	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		int ret = videobuf_iolock(q, vb, NULL);
		if (ret)
			return ret;
	}
	
	vb->state = VIDEOBUF_PREPARED;
	return 0;
}

static void buf_queue_grb (
			struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
	struct grb_info *grb_info_ptr = q->priv_data;

	dprintk(1, "buf_queue \n");
	
	//
	//Note that videobuf holds the lock when it calls
	//us, so we need not (indeed, cannot) take it here.
	//
	vb->state = VIDEOBUF_QUEUED;
	list_add_tail(&vb->queue, &grb_info_ptr->buffer_queue);
}

static void buf_release_grb (
			struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
//	struct grb_info *grb_info_ptr = q->priv_data;

	dprintk(1, "buf_release \n");

	videobuf_dma_contig_free(q, vb);
	
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static const struct videobuf_queue_ops videobuf_queue_ops_grb =
{
	.buf_setup		= buf_setup_grb,
	.buf_prepare	= buf_prepare_grb,
	.buf_queue		= buf_queue_grb,
	.buf_release	= buf_release_grb,
};


// ****************************************************************************
// 
//						FILE OPERATIONS
//
// ****************************************************************************


static int device_open(struct file *file_ptr)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	dprintk(1, "Open video4linux2 device. Name: %s \n" , grb_info_ptr->video_dev.name );	
	
	videobuf_queue_dma_contig_init(	&grb_info_ptr->videobuf_queue_grb,
							&videobuf_queue_ops_grb,
							grb_info_ptr->dev,
							&grb_info_ptr->irqlock,
							V4L2_BUF_TYPE_VIDEO_CAPTURE,
							V4L2_FIELD_NONE,
							sizeof(struct videobuf_buffer),
							grb_info_ptr,
							NULL);
	return 0;
}

static int device_release(struct file *file_ptr )
{
	struct video_device *video_dev = video_devdata(file_ptr);
	//struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	dprintk(1, "Close video4linux2 device. Name: %s \n" , video_dev->name );
	
	return 0;
}

static int device_mmap(struct file *file_ptr, struct vm_area_struct *vma)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	return videobuf_mmap_mapper(&grb_info_ptr->videobuf_queue_grb, vma);
}

static void video_dev_release(struct video_device *video_dev )
{
}

static struct v4l2_file_operations fops =
{
	.owner			= THIS_MODULE	,
	.open			= device_open	,
	.release		= device_release,
	.unlocked_ioctl	= video_ioctl2	,
	.mmap			= device_mmap
};

// ****************************************************************************
// 
//						IOCTR
//
// ****************************************************************************

static int vidioc_querycap_grb (
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_capability *v4l2_cap_ptr)
{
//	struct video_device *video_dev = video_devdata(file_ptr);
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
//	char base_addr[8];
//	
//	for (int i=0; i<8; i++)
//	base_addr[i] = (grb_info_ptr->grb_phys_addr & 0xff);
//	
//	strlcpy((char *)base_addr, grb_info_ptr->grb_phys_addr, 8);
	
	strlcpy((char *)v4l2_cap_ptr->driver, DRIVER_NAME, sizeof(v4l2_cap_ptr->driver));
	
	strlcpy((char *)v4l2_cap_ptr->card, DEVICE_NAME, sizeof(v4l2_cap_ptr->card));
	
	v4l2_cap_ptr->version = 1;
	
	strlcpy((char *)v4l2_cap_ptr->bus_info, "APB: 0x", 7);
	
//	strlcpy((char *)v4l2_cap_ptr->bus_info, grb_info_ptr->grb_phys_addr, 8);
	
	v4l2_cap_ptr->capabilities =
		V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE |
		V4L2_CAP_STREAMING;
	
	dprintk(1, "vidioc_querycap: driver: '%s';\n", v4l2_cap_ptr->driver);
	dprintk(1, "vidioc_querycap: card: '%s';\n", v4l2_cap_ptr->card);
	dprintk(1, "vidioc_querycap: bus_info: '%s';\n", v4l2_cap_ptr->bus_info);
	dprintk(1, "vidioc_querycap: version: '%d';\n", v4l2_cap_ptr->version);
	
	dprintk(1, "vidioc_querycap: grb_phys_addr: '0x%0x';\n", grb_info_ptr->grb_phys_addr);
	
	
//	v4l2_cap_ptr->device_caps =
//		V4L2_CAP_VIDEO_CAPTURE |
//		V4L2_CAP_READWRITE |
//		V4L2_CAP_STREAMING;
//
//	v4l2_cap_ptr->capabilities = v4l2_cap_ptr->device_caps ;
	
//	if (grb_info_ptr->video_dev.vfl_type == VFL_TYPE_GRABBER)
//		v4l2_cap_ptr->device_caps &=
//			~(V4L2_CAP_VBI_CAPTURE | V4L2_CAP_SLICED_VBI_OUTPUT);
//	else
//		v4l2_cap_ptr->device_caps &=
//			~(V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_AUDIO);

	return 0;
}
/*
static int vidioc_cropcap_grb (
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_cropcap *v4l2_cropcap_ptr)
{
//	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	return 0;
}

static int vidioc_g_crop_grb (
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_crop *v4l2_crop_ptr)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	v4l2_crop_ptr->c = grb_info_ptr->cropping;
		
	return 0;
}

static int vidioc_s_crop_grb (
			struct 	file *file_ptr,
			void 	*fh,
			const struct v4l2_crop *v4l2_crop_ptr)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
		
	grb_info_ptr->cropping = v4l2_crop_ptr->c;
	
	return 0;
}
*/
static int vidioc_g_fmt_vid_cap_grb (
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_format *v4l2_format_ptr)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
//	dprintk(1, "vidioc_g_fmt_vid_cap_grb: level 0 \n");
//	
//		set_mask(grb_info_ptr->base_addr_regs_grb + ADDR_INT_MASK , 11 , ADD);
//		
//		write_register(1, 1	, grb_info_ptr->base_addr_regs_grb, ADDR_REC_ENABLE	);
//		
//	dprintk(1, "vidioc_g_fmt_vid_cap_grb: gam_enable  = 0x%0x \n", read_register(1, grb_info_ptr->base_addr_regs_grb, ADDR_GAM_ENABLE	) );
//	dprintk(1, "vidioc_g_fmt_vid_cap_grb: frame_param = 0x%0x \n", read_register(1, grb_info_ptr->base_addr_regs_grb, ADDR_FRAME_PARAM	) );
		
	v4l2_format_ptr->fmt.pix = grb_info_ptr->user_format;
	
	return 0;
}

static int vidioc_s_fmt_vid_cap_grb (
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_format *v4l2_format_ptr)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	return set_output_format(grb_info_ptr, v4l2_format_ptr);
}

static int vidioc_reqbufs_grb (
			struct	file *file_ptr, 
			void	*fh,
			struct	v4l2_requestbuffers *req)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	return videobuf_reqbufs(&grb_info_ptr->videobuf_queue_grb, req);
}

static int vidioc_querybuf_grb(
			struct	file *file_ptr,
			void	*fh,
			struct	v4l2_buffer *buf)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	return videobuf_querybuf(&grb_info_ptr->videobuf_queue_grb, buf);
}

static int vidioc_qbuf_grb(
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_buffer *buf)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	return videobuf_qbuf(&grb_info_ptr->videobuf_queue_grb, buf);
}

static int vidioc_dqbuf_grb(
			struct 	file *file_ptr,
			void 	*fh,
			struct 	v4l2_buffer *buf)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	return videobuf_dqbuf(&grb_info_ptr->videobuf_queue_grb, buf, file_ptr->f_flags & O_NONBLOCK);
}

static int vidioc_streamon_grb(
			struct file *file_ptr,
			void *fh,
			enum v4l2_buf_type type)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
	int retval;
//	dprintk(1, "VIDIOC_STREAMON \n");
	
	
//	rd_data = read_register(1, grb_info_ptr->base_addr_regs_grb, ADDR_ENABLE);
//	dprintk(1, "Address %08x: value: %08x \n", grb_info_ptr->base_addr_regs_grb + ADDR_ENABLE , rd_data);
	
//	struct saa7146_fh *fh = __fh;
//	int err;
//
//	DEB_D("VIDIOC_STREAMON, type:%d\n", type);
//
//	err = video_begin(fh);
//	if (err)
//		return err;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
		
	retval = set_register(grb_info_ptr);
	if (retval <0) {
		dprintk(1, "set_register failed \n");
		return retval;
	}
	
	return videobuf_streamon(&grb_info_ptr->videobuf_queue_grb);
//	videobuf_streamon(&grb_info_ptr->videobuf_queue_grb);
//	
//	printk(KERN_ALERT "read: %08x\n", ioread32(grb_info_ptr->base_addr_regs_grb + ADDR_FRAME_PARAM) );
}

static int vidioc_streamoff_grb(
			struct file *file_ptr,
			void *__fh,
			enum v4l2_buf_type type)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
//	dprintk(1, "VIDIOC_STREAMOFF \n");

//	struct saa7146_fh *fh = __fh;
//	struct saa7146_dev *dev = fh->dev;
//	struct saa7146_vv *vv = dev->vv_data;
//	int err;
//
//	DEB_D("VIDIOC_STREAMOFF, type:%d\n", type);
//
//	// ugly: we need to copy some checks from video_end(),
//	// because videobuf_streamoff() relies on the capture running.
//	// check and fix this 
//	if ((vv->video_status & STATUS_CAPTURE) != STATUS_CAPTURE) {
//		DEB_S("not capturing\n");
//		return 0;
//	}
//
//	if (vv->video_fh != fh) {
//		DEB_S("capturing, but in another open\n");
//		return -EBUSY;
//	}
//
//	err = -EINVAL;
//	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		err = videobuf_streamoff(&fh->video_q);
//	else if (type == V4L2_BUF_TYPE_VBI_CAPTURE)
//		err = videobuf_streamoff(&fh->vbi_q);
//	if (0 != err) {
//		DEB_D("warning: videobuf_streamoff() failed\n");
//		video_end(fh, file);
//	} else {
//		err = video_end(fh, file);
//	}
//	return err;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	write_register(1, 0x1, grb_info_ptr->base_addr_regs_grb, ADDR_PR_RESET);
	return videobuf_streamoff(&grb_info_ptr->videobuf_queue_grb);
}

void drv_set_gamma (struct grb_info *grb_info_ptr, void *arg)
{
	struct grb_gamma *gam ;
	
	gam = (struct grb_gamma *) arg;
	
	grb_info_ptr->gam = gam;
	
	grb_info_ptr->active_gamma = gam->active_gamma;
}

void drv_vidioc_g_params (struct grb_info *grb_info_ptr, void *arg)
{
	struct grb_parameters *param ;
	
	param = (struct grb_parameters *) arg;
	
	param = &grb_info_ptr->param;
}

void drv_vidioc_s_params (struct grb_info *grb_info_ptr, void *arg)
{
	struct grb_parameters *param ;
	
	param = (struct grb_parameters *) arg;
	
	grb_info_ptr->param = *param;

	set_input_format(grb_info_ptr, param);

//	return set_input_format(grb_info_ptr, param);
}

int drv_vidioc_auto_detect (struct grb_info *grb_info_ptr, void *arg)
{
	struct 	v4l2_pix_format	*recognize_format;
	
	if (grb_info_ptr->recognize_format.bytesperline != 1) {
		write_register(1, 0x1 , grb_info_ptr->base_addr_regs_grb, ADDR_REC_ENABLE	);
		set_mask(grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK , INT_DONE  , ADD);
		grb_info_ptr->recognize_format.bytesperline = 1;
	}
	
	if (grb_info_ptr->recognize_format.priv != 1) {
		return -EAGAIN;
	}
	
	grb_info_ptr->recognize_format.priv = 0;
	
	recognize_format = (struct 	v4l2_pix_format *) arg;
	
//	*recognize_format  = grb_info_ptr->recognize_format  ;

	recognize_format->width      = grb_info_ptr->recognize_format.width      ;
	recognize_format->height     = grb_info_ptr->recognize_format.height     ;
	recognize_format->sizeimage  = grb_info_ptr->recognize_format.sizeimage  ;
	recognize_format->field = grb_info_ptr->recognize_format.field ;
	
	dprintk(1, "drv_vidioc_auto_detect: width = %d; height = %d;\n", grb_info_ptr->recognize_format.width, grb_info_ptr->recognize_format.height);
	dprintk(1, "drv_vidioc_auto_detect: sizeimage = %d; field = %d;\n", grb_info_ptr->recognize_format.sizeimage, grb_info_ptr->recognize_format.field);
	
	return 0;
}

static long vidioc_default_grb(
			struct file *file_ptr,
			void *fh,
			bool valid_prio,
			unsigned int cmd,
			void *arg)
{
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	
    switch (cmd) {
    case VIDIOC_SET_GAMMA:
  		drv_set_gamma(grb_info_ptr, arg);
		break;
    case VIDIOC_G_PARAMS:
  		drv_vidioc_g_params(grb_info_ptr, arg);
		break;
    case VIDIOC_S_PARAMS:
  		drv_vidioc_s_params(grb_info_ptr, arg);
		break;
    case VIDIOC_AUTO_DETECT:
  		return drv_vidioc_auto_detect(grb_info_ptr, arg);
    default:
		return -ENOTTY;
    }
	return 0;
}

	int vidioc_g_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
		struct grb_info *grb_info_ptr = video_drvdata(file);

		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;

		s->r = grb_info_ptr->cropping;
		return 0;
	}

	int vidioc_s_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
		struct grb_info *grb_info_ptr = video_drvdata(file);

		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;

		grb_info_ptr->cropping = s->r;
		return 0;
	}

// grb capture ioctl operations 
static const struct v4l2_ioctl_ops grb_ioctl_ops = {
	.vidioc_querycap			= vidioc_querycap_grb,
//	.vidioc_cropcap				= vidioc_cropcap_grb,					// VIDIOC_CROPCAP
//	.vidioc_g_crop				= vidioc_g_crop_grb,					// VIDIOC_G_CROP
//	.vidioc_s_crop				= vidioc_s_crop_grb,					// VIDIOC_S_CROP
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap_grb,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap_grb,
	.vidioc_reqbufs      		= vidioc_reqbufs_grb,
	.vidioc_querybuf     		= vidioc_querybuf_grb,
	.vidioc_qbuf         		= vidioc_qbuf_grb,
	.vidioc_dqbuf        		= vidioc_dqbuf_grb,
	.vidioc_streamon     		= vidioc_streamon_grb,
	.vidioc_streamoff    		= vidioc_streamoff_grb,
	.vidioc_default             = vidioc_default_grb,
	.vidioc_g_selection         = vidioc_g_selection_grb,
	.vidioc_s_selection         = vidioc_s_selection_grb
};

// ****************************************************************************
// 
//						IRQRETURN_T
//
// ****************************************************************************

static struct videobuf_buffer *grb_next_buffer(struct grb_info *grb_info_ptr)//(struct via_camera *cam)
{
//	unsigned long flags;
	struct videobuf_buffer *vb = NULL;

//	spin_lock_irqsave(&cam->viadev->reg_lock, flags);
//	if (cam->opstate != S_RUNNING)
//		goto out;
	if (list_empty(&grb_info_ptr->buffer_queue))
		goto out;
	
//	dprintk(1, "grb_next_buffer: level 0 \n");
	vb = list_entry(grb_info_ptr->buffer_queue.next, struct videobuf_buffer, queue);
	
//	dprintk(1, "grb_next_buffer: level 0 \n");
//	if (!waitqueue_active(&vb->done)) {/* Nobody waiting */
//		vb = NULL;
//		goto out;
//	}
	list_del(&vb->queue);
	
//	vb = list_entry(grb_info_ptr->buffer_queue.next, struct videobuf_buffer, queue);
//	vb->state = VIDEOBUF_ACTIVE;
out:
//	spin_unlock_irqrestore(&cam->viadev->reg_lock, flags);
	return vb;
}

static irqreturn_t proc_interrupt (struct grb_info *grb_info_ptr)
{
	int rd_data;
	
	void __iomem *base_addr;

	struct videobuf_buffer *vb;

	int height, width;

	u32 base_addr_dma0 ;
	u32 base_addr_dma1 ;
	u32 base_addr_dma2 ;

	u32 addr_dma0_addr ;
	u32 addr_dma1_addr ;
	u32 addr_dma2_addr ;

	char switch_page;

	base_addr = grb_info_ptr->base_addr_regs_grb;
	
	
	rd_data = read_register(2, base_addr, ADDR_INTERRUPTION);
	if (rd_data != 1)
		return IRQ_NONE;
	
	rd_data = read_register(2, base_addr, ADDR_INT_STATUS);
	
	if ( (rd_data >> TEST_INT) & 1)
	{
		write_register(2,  0 , base_addr, ADDR_TEST_INT	);
		
		set_mask(base_addr, ADDR_INT_MASK , 0 , DEL);
		
		write_register(2,  (1 << TEST_INT)	, base_addr, ADDR_INT_STATUS	);
		
		dprintk(2, "irq_handler: test interruption detected \n");
	}
	else if ( (rd_data >> END_WRITE) & 1)
	{
		write_register(2,  (1 << END_WRITE)	, base_addr, ADDR_INT_STATUS	);
		
		wake_up_all(&grb_info_ptr->queue);
		
		vb = grb_next_buffer(grb_info_ptr);
		
		if (vb == NULL) {
			return 1;
		}
		
		vb->state = VIDEOBUF_DONE;
	//	dprintk(2, "irq_handler: level 2 \n");
		wake_up(&vb->done);
		
		switch_page = grb_info_ptr->frame_count - (grb_info_ptr->frame_count/2)*2; // Check grb_info_ptr->frame_count%2;!
		
		dprintk(2, "irq_handler: frame_count = %d; switch_page = %d \n", grb_info_ptr->frame_count , switch_page);
		
		base_addr_dma0 = videobuf_to_dma_contig(vb);
		base_addr_dma1 = base_addr_dma0 + grb_info_ptr->mem_offset1 ;
		base_addr_dma2 = base_addr_dma0 + grb_info_ptr->mem_offset2 ;
		
		if (switch_page == 0)
		{
			addr_dma0_addr = ADDR_DMA0_ADDR0;
			addr_dma1_addr = ADDR_DMA1_ADDR0;
			addr_dma2_addr = ADDR_DMA2_ADDR0;
		}
		else // if (switch_page == 1)
		{
			addr_dma0_addr = ADDR_DMA0_ADDR1;
			addr_dma1_addr = ADDR_DMA1_ADDR1;
			addr_dma2_addr = ADDR_DMA2_ADDR1;
		}
		
		write_register(2, base_addr_dma0	, base_addr, addr_dma0_addr);
		write_register(2, base_addr_dma1 	, base_addr, addr_dma1_addr);
		write_register(2, base_addr_dma2	, base_addr, addr_dma2_addr);
		
		grb_info_ptr->frame_count = grb_info_ptr->frame_count + 1;
	}
	else if ( (rd_data >> INT_DONE) & 1)
	{
		write_register(2,  (1 << INT_DONE)	, base_addr, ADDR_INT_STATUS);
		
		rd_data = read_register(2, base_addr, ADDR_FRAME_SIZE);
		
		height = (rd_data >> 16) ;
		
		width = rd_data - (height << 16) ;
		
		width = width;
		
		grb_info_ptr->recognize_format.width  = width ;
		grb_info_ptr->recognize_format.height = height ;
		
		rd_data = read_register(2, base_addr, ADDR_FRAME_PARAM);
		
	//	grb_info_ptr->recognize_format.field = 14 ;
	//	grb_info_ptr->recognize_format.sizeimage  = 7 ;

	//	grb_info_ptr->recognize_format.sizeimage  = rd_data && 0x3 ;
	
		grb_info_ptr->recognize_format.sizeimage  = rd_data - (rd_data/4)*4 ;
		
		grb_info_ptr->recognize_format.field = rd_data/16 - (rd_data/32)*2 ;
		
	//	grb_info_ptr->recognize_format.field = (rd_data >> 4) && 1 ;

		dprintk(1, "irq_handler: rd_data = 0x%0x;\n", rd_data);
		
		dprintk(1, "irq_handler: width = %d; height = %d;\n", grb_info_ptr->recognize_format.width, grb_info_ptr->recognize_format.height);
		dprintk(1, "irq_handler: sizeimage = %d; field = %d;\n", grb_info_ptr->recognize_format.sizeimage, grb_info_ptr->recognize_format.field);
		grb_info_ptr->recognize_format.priv = 1;
		
	}
	else if ( (rd_data >> CH0_OVERFLOW) & 1)
	{
		dprintk(2, "irq_handler: overflow!\n");
		write_register(2,  1 , base_addr, ADDR_PR_RESET);
	}	
	else if ( (rd_data >> CH1_OVERFLOW) & 1)
	{	
		dprintk(2, "irq_handler: overflow!\n");
		write_register(2,  1 , base_addr, ADDR_PR_RESET);
	}	
	else if ( (rd_data >> CH2_OVERFLOW) & 1)
	{
		dprintk(2, "irq_handler: overflow!\n");
		write_register(2,  1 , base_addr, ADDR_PR_RESET);
	}
	
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler(int irq, void* dev)
{	
	struct grb_info *grb_info_ptr = (struct grb_info*) dev;

	irqreturn_t grb_irq;

	spin_lock(&grb_info_ptr->irqlock);

	grb_irq = proc_interrupt(grb_info_ptr);
	
	spin_unlock(&grb_info_ptr->irqlock);
	
	return grb_irq;
}

// ****************************************************************************
// 
//						PROBE
//
// ****************************************************************************

static int get_memory_buffer_address( const struct platform_device* grb_device,  const char* res_name, struct resource* res_ptr ) {
	unsigned int tmp[4];
	int ret;
	struct device_node* np;
	if( ( np  = of_parse_phandle( grb_device->dev.of_node, res_name, 0 ) ) == NULL ) {
		dprintk(1, "ERROR : can't get the device node of the memory-region"); 
		return -ENODEV;
	}
	if( ( ret = of_property_read_u32_array( np, "reg", tmp, 4 ) ) != 0 ) {
		dprintk(1, "ERROR : can't get the resource of the memory area");
		return ret; 
	}
	res_ptr->start = ((unsigned long long)tmp[0]<<32)|tmp[1];
	res_ptr->end = res_ptr->start + ( ((unsigned long long)tmp[2]<<32) | tmp[3] ) - 1;
	return 0;
}

static int get_resources (struct platform_device* grb_device , struct grb_info *grb_info_ptr)
{
	struct resource* grb_res_ptr;
	
	// 
	// Get grabber resource 
	//
	grb_res_ptr = platform_get_resource(grb_device, IORESOURCE_MEM, 0); // get the base address registers grabber
	if (!grb_res_ptr) {
		dprintk(1, "ERROR : can't get the base address registers grabber"); 
		return -1;
	}
	dprintk(1, "The base address registers grabber finish address (%08x,%08x)\n", (unsigned int)grb_res_ptr->start, (unsigned int)grb_res_ptr->end);	
	grb_info_ptr->grb_phys_addr = grb_res_ptr->start;
	grb_info_ptr->base_addr_regs_grb = devm_ioremap_resource(&grb_device->dev, grb_res_ptr);
    if (IS_ERR(grb_info_ptr->base_addr_regs_grb)) {
		dprintk(1, "ERROR : can't ioremap grabber resource");
    	return PTR_ERR(grb_info_ptr->base_addr_regs_grb);
	}
	

	if (read_register(1, grb_info_ptr->base_addr_regs_grb, ADDR_ID_REG) != 0xec176627) {
		dprintk(1, "invalid resources, device ""grabber"" not found! \n");
		return -1;
	}
	
	// 
	// Get buffer resource 
	//
	//grb_res_ptr = platform_get_resource(grb_device, IORESOURCE_MEM, 1); // get the base address of the memory area
	//if (!grb_res_ptr) {
	if( get_memory_buffer_address( grb_device, "memory-region", grb_res_ptr ) ) {
		dprintk(1, "ERROR : can't get the base address of the memory area"); 
		return -1;
	}

	grb_info_ptr->buff_phys_addr = grb_res_ptr->start ;	
	grb_info_ptr->buff_length = grb_res_ptr->end - grb_res_ptr->start + 1;


//	grb_info_ptr->base_addr_mem_area = devm_ioremap(&grb_device->dev, grb_info_ptr->buff_phys_addr, 4096);
//	if(!grb_info_ptr->base_addr_mem_area) {
//		dprintk(1, "ERROR : can't ioremap buffer resource");
//		return -1;
//	}
	dprintk(1, "The base address of the memory area and finish address (%llx,%llx)\n", grb_res_ptr->start, grb_res_ptr->end);
	dprintk(1, "grv_b2: buff_phys_addr = %llx\n", grb_info_ptr->buff_phys_addr);	
	
	// 
	// Get number interrupt 
	//
	grb_res_ptr = platform_get_resource(grb_device, IORESOURCE_IRQ, 0); // get irq resources
	if (!grb_res_ptr) {
		dprintk(1, "ERROR : can't get base irq");
		return -1;
	}
	dev_info(&grb_device->dev,"irq resource (%08x)\n", (unsigned int)grb_res_ptr->start);	
	grb_info_ptr->num_irq = grb_res_ptr->start;
	
	dprintk(1, "Number IRQ = %d\n", grb_info_ptr->num_irq);
	
	return 1;
}

static int device_probe (struct platform_device *grb_device)
{
	struct grb_info *grb_info_ptr;
	int res;
	
	dprintk(1, "Probe started!\n");

	grb_info_ptr = kzalloc(sizeof(struct grb_info), GFP_KERNEL); // memory allocation for inner structure and initialize by zeroes
	if (grb_info_ptr == NULL) {
		dprintk(1, "Can't allocated memory!\n");
		return -ENOMEM;
	}	
	grb_info_ptr->dev = &grb_device->dev;
	
	if(get_resources (grb_device , grb_info_ptr) < 0) {
		kfree(grb_info_ptr);
		return -1;
	}

	grb_device->dma_mask = DMA_BIT_MASK(32);
	grb_device->dev.archdata.dma_offset = - (grb_device->dev.dma_pfn_offset << PAGE_SHIFT);

	res = dma_declare_coherent_memory(&grb_device->dev, grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_length );//,  DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
	dprintk(1, "dma_declare_coherent_memory return %d for addr %llx and size %x\n" , res, grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_length);

	if (res != 0) {
		//dprintk(1, "dma_declare_coherent_memory error %d for addr %llx and size %x!\n" , res, grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_length);
		return -1;
	}
	//
	// Settings
	//
	
	platform_set_drvdata(grb_device, grb_info_ptr); 
	
	grb_info_ptr->video_dev.dev_parent = grb_info_ptr->dev;
	grb_info_ptr->video_dev.fops = &fops;
	grb_info_ptr->video_dev.ioctl_ops = &grb_ioctl_ops;
	strncpy(grb_info_ptr->video_dev.name, DRIVER_NAME, sizeof(grb_info_ptr->video_dev.name) - 1);
	grb_info_ptr->video_dev.release = video_dev_release;
	
	grb_info_ptr->video_dev.tvnorms = V4L2_STD_ATSC_8_VSB + V4L2_STD_ATSC_16_VSB;

	res = v4l2_device_register( grb_info_ptr->dev, &grb_info_ptr->v4l2_device );
	if (res) {
		dprintk( 1, "Failed v4l2_device register, error %d\n", res );
		return res;
	}
	grb_info_ptr->video_dev.v4l2_dev = &grb_info_ptr->v4l2_device;
	grb_info_ptr->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

	video_set_drvdata(&grb_info_ptr->video_dev , grb_info_ptr);
	
	res = video_register_device(&grb_info_ptr->video_dev, VFL_TYPE_GRABBER, -1);
	if (res) {
		dprintk( 1, "Failed video_dev register, error %d\n", res );
		video_device_release(&grb_info_ptr->video_dev); /* or kfree(my_vdev); */
		return res;
	}
	
//	dprintk(1, "video_device.dev.init_name: %s\n", &grb_info_ptr->video_dev.dev.init_name);
	
	init_waitqueue_head(&grb_info_ptr->queue);
	
	res = request_irq(grb_info_ptr->num_irq, irq_handler, IRQF_SHARED, DEVICE_NAME, grb_info_ptr);
	if (res)
	{
		dprintk(1, "ERROR : request_irq isn't availiable.");
		kfree(grb_info_ptr);
		return res;
	}
	
	//{ ШЛАК
	
	write_register(1, 0x1     			, grb_info_ptr->base_addr_regs_grb, ADDR_TEST_INT	);
	write_register(1, 0x1     			, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK	);
	
	INIT_LIST_HEAD(&grb_info_ptr->buffer_queue);
	//}	
	
	dprintk(1, "method ""probe"" completed successfully");
	return 0 ;
}

// ****************************************************************************
// 
//						REMOVE
//
// ****************************************************************************

static int device_remove (struct platform_device* grb_device)
{
	struct grb_info* grb_info_ptr;

	dprintk(1, "In grabber_remove function : %p\n", grb_device);
	grb_info_ptr = platform_get_drvdata(grb_device);

	write_register(1, 0x1, grb_info_ptr->base_addr_regs_grb, ADDR_PR_RESET);

	video_unregister_device(&grb_info_ptr->video_dev);

	if(grb_info_ptr != NULL) {
		kfree(grb_info_ptr);
	}
	
	if (grb_info_ptr->num_irq) {
		free_irq(grb_info_ptr->num_irq, grb_info_ptr);
	}
	return 0 ;
}

// ****************************************************************************
// 
//						MAIN
//
// ****************************************************************************

struct of_device_id grb_match[] = 
{
	{ .compatible = "rcm,vdu-graber" },
	{ }
};

static struct platform_driver pl_dr =
 {
	.driver = {
		.name = "grabber",
		.of_match_table = grb_match
	},
	.probe	= device_probe	,
	.remove	= device_remove
 };

static int grb_v2_init( void )
{
	dprintk(1, "Grabber driver mounted!\n");	
	if (!platform_driver_register(&pl_dr))
		dprintk(1, "platform_driver_register finished successfully \n");
	else
		return (-1);

	return 0;
}

static void grb_v2_exit( void )
{
	dprintk(1, "Exit from module grabber\n");
	platform_driver_unregister(&pl_dr);
	return;
}

module_init(grb_v2_init);
module_exit(grb_v2_exit);
