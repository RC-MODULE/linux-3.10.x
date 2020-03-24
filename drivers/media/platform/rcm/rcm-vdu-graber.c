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

static int debug = 1000;
module_param(debug, int, 0644);

#define dprintk(level, fmt, arg...)						\
	do {												\
		if (debug >= level)								\
			printk(KERN_DEBUG "grb: " fmt, ## arg);	\
	} while (0)

struct grb_info
{
	struct device *dev;
	struct video_device video_dev;
	struct videobuf_queue videobuf_queue_grb;
	struct list_head buffer_queue;
	char frame_count;
	int num_irq;
	wait_queue_head_t wait_queue;
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
	struct coef_conv c_conv;

	void __iomem *base_addr_regs_grb;
	phys_addr_t buff_phys_addr;
	dma_addr_t buff_dma_addr;
	void* kern_virt_addr;

	u32 mem_offset1, mem_offset2;		// заполняется set_register
	u32 buff_length;					// заполняется buf_setup
};

static inline void write_register( u32 val, void __iomem *addr, u32 offset ) {
	iowrite32( val, addr + offset );
}

static inline u32 read_register( void __iomem *addr, u32 offset ) {
	return ioread32( addr + offset );
}

static int reset_grab( void __iomem *addr ) {
	unsigned int i;
	write_register( 0x01 , addr, ADDR_PR_RESET );
	for( i = 0; i < 10000; i++ ) {
		if( read_register( addr, ADDR_PR_RESET ) == 0 ) {
			dprintk( 1, "Reset graber OK\n");
			return 0;
		}
	}
	dprintk( 1, "Reset graber failed\n");
	return -1;
}

static inline void print_register( void __iomem *addr, unsigned int begin, unsigned int end ) {
	unsigned int reg, val;
	for( reg=begin; reg<=end; reg+=4 ) {
		val = read_register( addr, reg );
		dprintk(1, C_GREY"get(0x%08X, 0x%08X)"C_CLEAR"\n", reg, val );
	}
}

static void print_videobuf_queue_param( struct videobuf_queue* queue, unsigned int num ) {
		struct videobuf_dma_contig_memory {
		u32 magic;
		void *vaddr;
		dma_addr_t dma_handle;
		unsigned long size;
	};
	unsigned int i=0 ;
	struct videobuf_dma_contig_memory* mem;
	struct videobuf_buffer* vb;
	for( i=0; i<num; i++ ) {
		vb = queue->bufs[i];		// буфер
		if( vb ) {
			mem = vb->priv;			// параметры области памяти
			dprintk( 1,
					 C_YELLOW"videobuf_buffer(%u): memory=%u,vaddr=%x,dma_handle=%x,size=%lx: %*ph"C_CLEAR"\n",
					 i, vb->memory, (u32)mem->vaddr, (u32)mem->dma_handle, mem->size, 16, mem->vaddr ); // V4L2_MEMORY_MMAP=1
			//memset( mem->vaddr, 0, mem->size );	// после вывода очистим буфер
		};
	}
}

static inline void print_frame_param( void __iomem *addr ) {
	dprintk( 1, "frame param: 0x%08X\n", read_register( addr, 0x108) );
}

static void set_mask( void __iomem *addr, u32 offset, int bit, bool add )
{
	u32 reg = read_register( addr, offset );
	write_register( add ? reg | (1<<bit) : reg & ~(1<<bit), addr, offset );
}

static int set_input_format( struct grb_info *grb_info_ptr, struct grb_parameters *param ) { // VIDIOC_S_PARAMS
	u32 active_bus;
	u32 format_data;

	if( param->d_format == YCBCR422 ) {
		format_data  = 0x0 ;							// YCbCr(0)
		if( param->std_in == SD ) {
			if( param->v_if == SERIAL )					// d0
				active_bus = 0x1;
			else if( param->v_if == PARALLEL )			// d0,d1,d2
				active_bus = 0x3;
			else {
				active_bus = 0x0;
				dprintk( 1, "invalid parameter param.v_if!\n" ); 
				return -EINVAL;
			}
		}
		else if( param->std_in == HD ) {
			active_bus = 0x2 ;							// d0,d1
		}
		else {
			active_bus = 0x0 ;
			dprintk(1, "invalid parameter param.std_in!\n"); 
			return -EINVAL;
		}
		grb_info_ptr->in_f.color = YCBCR;
		dprintk( 1, "input format is YCBCR 4:2:2\n" ); 
	}
	else if( param->d_format == YCBCR444 ) {			// 1
		format_data = 0x0 ;								// YCbCr
		active_bus  = 0x3 ;								// d0,d1,d2
		grb_info_ptr->in_f.color = YCBCR;
		dprintk(1, "input format is YCBCR 4:4:4\n"); 
	} 
	else if( param->d_format == RGB888 ) {				// 1
		format_data = 0x1 ;								// RGB
		active_bus  = 0x3 ;								// d0,d1,d2
		grb_info_ptr->in_f.color = RGB ;
		dprintk(1, "input format is YCBCR 4:4:4\n"); 
	}
	else {
		active_bus  = 0x0;
		format_data = 0x0;
		grb_info_ptr->in_f.color = 0x0;
		dprintk( 1, "std_grb: unknown standard \n" );
		return -EINVAL;
	}

	grb_info_ptr->in_f.format_din = format_data;					// 1–RGB,0-YCbCr
	grb_info_ptr->in_f.format_din |= (active_bus << 1) ;			// 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
	grb_info_ptr->in_f.format_din |= (param->std_in << 3) ;			// 0-дублирования нет;1–дублирование (SDTV)
	grb_info_ptr->in_f.format_din |= (param->sync << 4) ;			// 0–устройство работает по сигналам внешней синхронизации (hsync, vsync, field и data_enable); 1– по сигналам внутренней синхронизации (синхрокодам EAV и SAV)	
	grb_info_ptr->in_f.color_std = param->std_in;					// входной режим
	grb_info_ptr->out_f.color_std = param->std_out;					// выходной режим (SD,HD)

	if( param->alpha > 255 ) { 
		dprintk( 1, "alpha > 255\n" );
		return -EINVAL;
	}
	return 0;
}

static int set_output_format( struct grb_info *grb_info_ptr, struct v4l2_format *v4l2_format_ptr )
{
	grb_info_ptr->user_format = v4l2_format_ptr->fmt.pix;

	switch( v4l2_format_ptr->fmt.pix.pixelformat ) {
	case V4L2_PIX_FMT_BGR32:
		grb_info_ptr->out_f.format_dout = 0x03;
		grb_info_ptr->out_f.color = RGB;
		dprintk( 1, "pixelformat: ARGB8888\n" ); 
		break;
	case V4L2_PIX_FMT_RGB32:
		grb_info_ptr->out_f.format_dout = 0x0e;
		grb_info_ptr->out_f.color = RGB;
		dprintk( 1, "pixelformat: RGB888 \n" );
		break;
	case V4L2_PIX_FMT_NV16:
		grb_info_ptr->out_f.format_dout = 0x04;
		grb_info_ptr->out_f.color = YCBCR;
		dprintk( 1, "pixelformat: YCBCR422 two planes\n" );
		break;
	case V4L2_PIX_FMT_NV61:
		grb_info_ptr->out_f.format_dout = 0x06;
		grb_info_ptr->out_f.color = YCBCR;
		dprintk( 1, "pixelformat: YCBCR422 three planes\n" );
		break;
	case V4L2_PIX_FMT_NV24:
		grb_info_ptr->out_f.format_dout = 0x0e;
		grb_info_ptr->out_f.color = YCBCR;
		dprintk( 1, "pixelformat: YCBCR444 three planes.\n" );
		break;
	default:
		grb_info_ptr->out_f.format_dout = 0x00;
		dprintk( 1, "pixelformat: unknown pixelformat!\n" );
		return -EINVAL;
	}
	grb_info_ptr->out_f.y_hor_size = v4l2_format_ptr->fmt.pix.width;
	grb_info_ptr->out_f.y_ver_size = v4l2_format_ptr->fmt.pix.height;
	grb_info_ptr->out_f.y_full_size = v4l2_format_ptr->fmt.pix.bytesperline;
	grb_info_ptr->out_f.c_hor_size = v4l2_format_ptr->fmt.pix.width;
	grb_info_ptr->out_f.c_ver_size = v4l2_format_ptr->fmt.pix.height;
	grb_info_ptr->out_f.c_full_size = v4l2_format_ptr->fmt.pix.bytesperline;

	if ( (grb_info_ptr->out_f.y_hor_size > grb_info_ptr->out_f.y_full_size) ||
		 (grb_info_ptr->out_f.c_hor_size > grb_info_ptr->out_f.c_full_size) ) {
		dprintk( 1, "output size mismatch!\n" );
		return -EINVAL;
	} 
	return 0;
}

static int setup_color( struct grb_info *grb_info_ptr, void __iomem* base_addr, struct coef_conv* c_conv )
{
	//void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;

	u32 color_in = grb_info_ptr->in_f.color,
		color_std_in = grb_info_ptr->in_f.color_std,
		color_out = grb_info_ptr->out_f.color,
		color_std_out = grb_info_ptr->out_f.color_std;

	dprintk( 1, "color input format=%u,input standard=%u,color output format=%u,output standard=%u\n",
			color_in, color_std_in, color_out, color_std_out );

	if( ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != SD ) && ( color_std_in != HD ) ) ||
		  ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != SD ) && ( color_std_in != HD ) ) ) {
		dprintk( 1, "color format is wrong\n ");
		return -EINVAL;
	}

	if( color_in != color_out ) {
		if( color_in == RGB ) {
			dprintk( 1, "enable colour conversion from RGB to YCBCR.\n" );
			if( color_std_out == SD ) { // SD
				dprintk(1, "output colour standard is SD.\n");

				grb_info_ptr->c_conv.coef[0][0] = (0<<18) + 16*256;
				grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 129;
				grb_info_ptr->c_conv.coef[0][2] = (0<<10) + 66;
				grb_info_ptr->c_conv.coef[0][3] = (0<<10) + 25;

				grb_info_ptr->c_conv.coef[1][0] = (0<<18) + 128*256;
				grb_info_ptr->c_conv.coef[1][1] = (1<<10) + 94;
				grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 112;
				grb_info_ptr->c_conv.coef[1][3] = (1<<10) + 18;

				grb_info_ptr->c_conv.coef[2][0] = (0<<18) + 128*256;
				grb_info_ptr->c_conv.coef[2][1] = (1<<10) + 74;
				grb_info_ptr->c_conv.coef[2][2] = (1<<10) + 38;
				grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 112;
			} // SD
			else { // HD
				dprintk(1, "output colour standard is HD.\n");

				grb_info_ptr->c_conv.coef[0][0] = (0<<18) + 16*256;
				grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 157;
				grb_info_ptr->c_conv.coef[0][2] = (0<<10) + 47;
				grb_info_ptr->c_conv.coef[0][3] = (0<<10) + 16;

				grb_info_ptr->c_conv.coef[1][0] = (0<<18) + 128*256;
				grb_info_ptr->c_conv.coef[1][1] = (1<<10) + 102;
				grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 112;
				grb_info_ptr->c_conv.coef[1][3] = (1<<10) + 10;

				grb_info_ptr->c_conv.coef[2][0] = (0<<18) + 128*256;
				grb_info_ptr->c_conv.coef[2][1] = (1<<10) + 87;
				grb_info_ptr->c_conv.coef[2][2] = (1<<10) + 26;
				grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 112;
			} // HD
			grb_info_ptr->c_conv.range[0] = (235<<16) + 16;
			grb_info_ptr->c_conv.range[1] = (240<<16) + 16;
			grb_info_ptr->c_conv.range[2] = (240<<16) + 16;
		} // RGB
		else if( color_in == YCBCR ) { // YCBCR
			dprintk(1, "enable colour conversion from YCBCR to RGB.\n");
			if( color_std_in == SD ) { // SD
				dprintk(1, "input colour standard is SD.\n");

					grb_info_ptr->c_conv.coef[0][0] = (0<<18) + 34656;
					grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 298;
					grb_info_ptr->c_conv.coef[0][2] = (1<<10) + 208;
					grb_info_ptr->c_conv.coef[0][3] = (1<<10) + 100;
					
					grb_info_ptr->c_conv.coef[1][0] = (1<<18) + 57120;
					grb_info_ptr->c_conv.coef[1][1] = (0<<10) + 298;
					grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 409;
					grb_info_ptr->c_conv.coef[1][3] = (0<<10) + 0;

					grb_info_ptr->c_conv.coef[2][0] = (1<<18) + 70944;
					grb_info_ptr->c_conv.coef[2][1] = (0<<10) + 298;
					grb_info_ptr->c_conv.coef[2][2] = (0<<10) + 0;
					grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 517;
			} // SD
			else { // HD
				dprintk(1, "input colour standard is HD.\n");

				grb_info_ptr->c_conv.coef[0][0] = (0<<18) + 19680;
				grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 298;
				grb_info_ptr->c_conv.coef[0][2] = (1<<10) + 136;
				grb_info_ptr->c_conv.coef[0][3] = (1<<10) + 55;

				grb_info_ptr->c_conv.coef[1][0] = (1<<18) + 63520;
				grb_info_ptr->c_conv.coef[1][1] = (0<<10) + 298;
				grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 459;
				grb_info_ptr->c_conv.coef[1][3] = (0<<10) + 0;
				
				grb_info_ptr->c_conv.coef[2][0] = (1<<18) + 74016;
				grb_info_ptr->c_conv.coef[2][1] = (0<<10) + 298;
				grb_info_ptr->c_conv.coef[2][2] = (0<<10) + 0;
				grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 541;
			} // HD
			grb_info_ptr->c_conv.range[0] = (255<<16) + 0;
			grb_info_ptr->c_conv.range[1] = (255<<16) + 0;
			grb_info_ptr->c_conv.range[2] = (255<<16) + 0;
		} // YCBCR
	}
	else if( (color_in == YCBCR) && (color_out == YCBCR) ) {
		dprintk(1, "enable conversion colour standard  for YCBCR.\n");

		if( (color_std_in == SD) && (color_std_out == HD) ) {
			dprintk(1, "conversion from SD to HD.\n");

			grb_info_ptr->c_conv.coef[0][0] = (0<<18) + 10624;
			grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 256;
			grb_info_ptr->c_conv.coef[0][2] = (1<<10) + 53;
			grb_info_ptr->c_conv.coef[0][3] = (1<<10) + 30;

			grb_info_ptr->c_conv.coef[1][0] = (1<<18) + 3328;
			grb_info_ptr->c_conv.coef[1][1] = (0<<10) + 0;
			grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 263;
			grb_info_ptr->c_conv.coef[1][3] = (0<<10) + 19;

			grb_info_ptr->c_conv.coef[2][0] = (1<<18) + 4352;
			grb_info_ptr->c_conv.coef[2][1] = (0<<10) + 0;
			grb_info_ptr->c_conv.coef[2][2] = (0<<10) + 29;
			grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 261;
		}
		else if( (color_std_in == HD) & (color_std_out == SD) ) {
			dprintk(1, "conversion from HD to SD.\n");
			
			grb_info_ptr->c_conv.coef[0][0] = (1<<18) + 9472;
			grb_info_ptr->c_conv.coef[0][1] = (0<<10) + 256;
			grb_info_ptr->c_conv.coef[0][2] = (0<<10) + 49;
			grb_info_ptr->c_conv.coef[0][3] = (0<<10) + 25;
						
			grb_info_ptr->c_conv.coef[1][0] = (0<<18) + 2944;
			grb_info_ptr->c_conv.coef[1][1] = (0<<10) + 0;
			grb_info_ptr->c_conv.coef[1][2] = (0<<10) + 252;
			grb_info_ptr->c_conv.coef[1][3] = (1<<10) + 19;
						
			grb_info_ptr->c_conv.coef[2][0] = (0<<18) + 3968;
			grb_info_ptr->c_conv.coef[2][1] = (0<<10) + 0;
			grb_info_ptr->c_conv.coef[2][2] = (1<<10) + 28;
			grb_info_ptr->c_conv.coef[2][3] = (0<<10) + 253;
		}
		grb_info_ptr->c_conv.range[0] = (235<<16) + 16 ;
		grb_info_ptr->c_conv.range[1] = (240<<16) + 16 ;
		grb_info_ptr->c_conv.range[2] = (240<<16) + 16 ;
	} // YCBCR
	else { // RGB-RGB
		write_register( 0, base_addr, ADDR_CONV_ENABLE );
		return 0; // преобразовывать не нужно
	}

	write_register( 0, base_addr, ADDR_CONV_ENABLE );

	//if( c_conv->conv_on ) {
		write_register( c_conv->coef[0][0], base_addr, ADDR_C_0_0 );
		write_register( c_conv->coef[0][1], base_addr, ADDR_C_0_1 );
		write_register( c_conv->coef[0][2], base_addr, ADDR_C_0_2 );
		write_register( c_conv->coef[0][3], base_addr, ADDR_C_0_3 );

		write_register( c_conv->coef[1][0], base_addr, ADDR_C_1_0 );
		write_register( c_conv->coef[1][1], base_addr, ADDR_C_1_1 );
		write_register( c_conv->coef[1][2], base_addr, ADDR_C_1_2 );
		write_register( c_conv->coef[1][3], base_addr, ADDR_C_1_3 );

		write_register( c_conv->coef[2][0], base_addr, ADDR_C_2_0 );
		write_register( c_conv->coef[2][1], base_addr, ADDR_C_2_1 );
		write_register( c_conv->coef[2][2], base_addr, ADDR_C_2_2 );
		write_register( c_conv->coef[2][3], base_addr, ADDR_C_2_3 );

		write_register( c_conv->range[0], base_addr, ADDR_CH0_RANGE );
		write_register( c_conv->range[1], base_addr, ADDR_CH1_RANGE );
		write_register( c_conv->range[2], base_addr, ADDR_CH2_RANGE );
		write_register( 1, base_addr, ADDR_CONV_ENABLE );
	//}

	print_register( base_addr, ADDR_C_0_0, ADDR_CH2_RANGE );
	return 0;
}

static int check_and_correct_out_format( u32 format_dout, u32* c_hor_size, u32* c_full_size ) {
	switch( format_dout ) {
	case 0x03:
		dprintk( 1, "pixelformat: ARGB8888.\n" );
		return 0;
	case 0x04:
		dprintk( 1, "pixelformat: YCBCR422 two planes.\n" );
		return 0;
	case 0x06:
		*c_hor_size /= 2;	// общая панель для CB и CR
		*c_full_size /= 2;
		dprintk( 1, "pixelformat: YCBCR422 three planes.\n" );
		return 0;
	case 0x0e:
		dprintk( 1, "pixelformat: YCBCR444 three planes.\n" );
		return 0;
	default:
		dprintk( 1, "pixelformat: unknown pixelformat! \n" );
		return -EINVAL;
	}
}

static inline dma_addr_t videobuf_to_dma_contig_rcm( struct videobuf_buffer *buf ) {
	 dma_addr_t dma_addr = videobuf_to_dma_contig( buf );
	 return PHYS_TO_DMA( dma_addr );
}

static void setup_gamma( void __iomem* base_addr, struct grb_gamma *gam ) {
	int i;
	u32 vY_G, vC_R, vC_B;
	int *pY_G = gam->table_Y_G, *pC_R = gam->table_C_R, *pC_B = gam->table_C_B;

	write_register( 0, base_addr, ADDR_GAM_ENABLE );
	if( gam->active_gamma == ON ) {
		for ( i=0; i<256; i+=4 ) {											// todo check it!
				vY_G = U8x4_TO_U32( pY_G[i], pY_G[i+1], pY_G[i+2], pY_G[i+3] );
				vC_R = U8x4_TO_U32( pC_R[i], pC_R[i+1], pC_R[i+2], pC_R[i+3] );
				vC_B = U8x4_TO_U32( pC_B[i], pC_B[i+1], pC_B[i+2], pC_B[i+3] );
				write_register( vY_G, base_addr, BASE_ADDR_TABLE_0+i );		// dv0( Y,G)
				write_register( vC_R, base_addr, BASE_ADDR_TABLE_1+i );		// dv1(Cr,R)
				write_register( vC_B, base_addr, BASE_ADDR_TABLE_2+i );		// dv2(Cb,B)
				
		}
		write_register( 1, base_addr, ADDR_GAM_ENABLE );
	}
}

int set_register (struct grb_info *grb_info_ptr)
{
	void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;
	u32 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2;
	u32 y_hor_size, y_ver_size, y_full_size;
	u32 c_ver_size, c_hor_size, c_full_size;
	u32 base_point_y, base_point_x;
	u32 format_din;
	u32 format_dout;

	format_din  = grb_info_ptr->in_f.format_din;
	format_dout = grb_info_ptr->out_f.format_dout;
	dprintk( 1, "format_din: 0x%0x,format_dout: 0x%0x\n", format_din, format_dout );

	if( grb_info_ptr->cropping.width != 0 && grb_info_ptr->cropping.height !=0 ) {
		y_hor_size	= grb_info_ptr->cropping.width;
		y_ver_size	= grb_info_ptr->cropping.height;
		c_hor_size	= grb_info_ptr->cropping.width;
		c_ver_size	= grb_info_ptr->cropping.height;
	}
	else {
		y_hor_size	= grb_info_ptr->out_f.y_hor_size;
		y_ver_size	= grb_info_ptr->out_f.y_ver_size;
		c_hor_size	= grb_info_ptr->out_f.c_hor_size;
		c_ver_size	= grb_info_ptr->out_f.c_ver_size;
	}
	y_full_size = grb_info_ptr->out_f.y_full_size;
	c_full_size	= grb_info_ptr->out_f.c_full_size;
	base_point_y = grb_info_ptr->cropping.top;
	base_point_x = grb_info_ptr->cropping.left;

	if( check_and_correct_out_format( grb_info_ptr->out_f.format_dout, &c_hor_size, &c_full_size ) )
		return -EINVAL;

	grb_info_ptr->mem_offset1 = y_full_size*y_ver_size;											// плоскость цвета 1
	grb_info_ptr->mem_offset2 = grb_info_ptr->mem_offset1 + c_full_size*c_ver_size;				// плоскость цвета 2
	
	dprintk( 1, "y_hor_size=%d,y_ver_size=%d,c_hor_size=%d,c_ver_size=%d,y_full_size=%d,c_full_size=%d,mem_offset1=%d,mem_offset2=%d,alpha=%d\n",
			 y_hor_size, y_ver_size, c_hor_size, c_ver_size, y_full_size, c_full_size,
			 grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2, grb_info_ptr->param.alpha );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 2 ); // vaddr, dma_handle, size для каждого буфера

	base_addr0_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[0] );	// вернет dma_адрес
	base_addr1_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[1] );
	base_addr0_dma1 = base_addr0_dma0 + grb_info_ptr->mem_offset1;
	base_addr1_dma1 = base_addr1_dma0 + grb_info_ptr->mem_offset1;
	base_addr0_dma2 = base_addr0_dma0 + grb_info_ptr->mem_offset2;
	base_addr1_dma2 = base_addr1_dma0 + grb_info_ptr->mem_offset2;

	dprintk( 1, "base_addr0_dma0=%08x,base_addr1_dma0=%08x,base_addr0_dma1=%08x,base_addr1_dma1=%08x,base_addr0_dma2=%08x,base_addr1_dma2=%08x\n",
			 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2 );

	reset_grab( base_addr );
	print_frame_param( base_addr );
	write_register( 1, base_addr, ADDR_BASE_SW_ENA );											// разрешение переключения базовых адресов
	write_register( base_addr0_dma0, base_addr, ADDR_DMA0_ADDR0 );								// яркостная нечет
	write_register( base_addr1_dma0, base_addr, ADDR_DMA0_ADDR1 );								// яркостная чет
	write_register( base_addr0_dma1, base_addr, ADDR_DMA1_ADDR0 );								// цветоразностная нечет
	write_register( base_addr1_dma1, base_addr, ADDR_DMA1_ADDR1 );								// цветоразностная чет
	write_register( base_addr0_dma2, base_addr, ADDR_DMA2_ADDR0 );								// цветоразностная нечет
	write_register( base_addr1_dma2, base_addr, ADDR_DMA2_ADDR1 );								// цветоразностная чет
	write_register( U16x2_TO_U32( y_ver_size, y_hor_size ), base_addr, ADDR_Y_SIZE );			// 27:16-вертикальный размер,11:0-горизонтальный размер изображения для яркостной компоненты
	write_register( U16x2_TO_U32( c_ver_size, c_hor_size ), base_addr, ADDR_C_SIZE );			// 27:16-вертикальный размер,11:0-горизонтальный размер изображения для цветоразностной компоненты
	write_register( U16x2_TO_U32( c_full_size, y_full_size ), base_addr, ADDR_FULL_LINE_SIZE );	// 27:16-вертикальный размер,11:0-горизонтальный размер полной строки в памяти
	write_register( U16x2_TO_U32( base_point_y, base_point_x ), base_addr, ADDR_BASE_POINT );	// 27:16-вертикальная координата точки захвата видеоизображения,11:0-горизонтальная координата точки захвата видеоизображения
	write_register( grb_info_ptr->param.alpha, base_addr, ADDR_TRANSPARENCY );
// ADDR_MODE:
// бит 4: 0 – устройство работает по сигналам внешней синхронизации (hsync, vsync, field и data_enable),1– по сигналам внутренней синхронизации (синхрокодам EAV и SAV)
// бит 3: 0 – дублирования нет,1 – дублирование (SDTV)
// биты 2,1: 1–при передаче по линии dv0, 2 – при передаче данных по линиям dv0 и dv1, 3–при передаче данных по линиям dv0,dv1 и dv2
// бит 0: Данный регистр содержит информацию о формате данных, поступающих на вход УЗВИ: 0 – YCbCr,1 – RGB
	write_register( format_din , base_addr, ADDR_MODE );
// ADDR_LOCATION_DATA:
// бит 3: Данный регистр содержит информацию о формате данных (только для YCbCr): 0 – YCbCr 4:2:2; 1 – YCbCr 4:4:4
// биты 2,1: Данный регистр содержит информацию о количестве областей памяти, использующихся для записи: 1–для формата ARGB 8888,2–для формата YCbCr 4:2:2,3–для формата YCbCr 4:4:4,YCbCr 4:2:2 и RGB 888
// бит 0: Данный регистр содержит информацию о цветовой модели записываемых данных: 0 – YCbCr; 1 – RGB
	write_register( format_dout , base_addr, ADDR_LOCATION_DATA );
	write_register( 0, base_addr, ADDR_GAM_ENABLE );

	setup_gamma( base_addr, &grb_info_ptr->gam );

	setup_color( grb_info_ptr, base_addr, &grb_info_ptr->c_conv );

	print_frame_param( base_addr );
	set_mask( base_addr, ADDR_INT_MASK , END_WRITE, true );		// Разрешение генерации сигнала прерывания по событию: окончание записи кадра в память
	write_register( 1 , base_addr, ADDR_ENABLE );				// Разрешение захвата видеоизображения

	return 0;
}

static int buf_setup_grb ( struct videobuf_queue *q, unsigned int *count, unsigned int *size ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	int max_buff;
	dprintk( 1, "buf_setup_grb\n" );
	*size = grb_info_ptr->user_format.sizeimage;
	max_buff = grb_info_ptr->buff_length / *size;
	if( *count < 2 )
		*count = 2;
	else if( *count > max_buff )
		*count = max_buff;
	print_videobuf_queue_param( q, 4 );
	return 0;
}

static int buf_prepare_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb, enum v4l2_field field ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	dprintk( 1, "buf_prepare_grb\n" );
	vb->size = grb_info_ptr->user_format.sizeimage;
	vb->width = grb_info_ptr->user_format.bytesperline;
	vb->height = grb_info_ptr->user_format.height;
	vb->field = field;

	if( vb->state == VIDEOBUF_NEEDS_INIT ) {
		int ret = videobuf_iolock( q, vb, NULL );
		if( ret ) {
			dprintk( 1, "videobuf_iolock failed (%d)\n", ret );
			return ret;
		}
	}
	vb->state = VIDEOBUF_PREPARED;
	print_videobuf_queue_param( q, 4 );
	return 0;
}

static void buf_queue_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	dprintk( 1, "buf_queue_grb\n" );	
	list_add_tail( &vb->queue, &grb_info_ptr->buffer_queue );	// Note that videobuf holds the lock when it calls us, so we need not (indeed, cannot) take it here
	vb->state = VIDEOBUF_QUEUED;
	print_videobuf_queue_param( q, 4 );
}

static void buf_release_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	unsigned long flags;
	dprintk( 1, "buf_release_grb\n" );
	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );				// We need to flush the buffer from the dma queue since hey are de-allocated
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	videobuf_dma_contig_free( q, vb );
	vb->state = VIDEOBUF_NEEDS_INIT;
	print_videobuf_queue_param( q, 4 );
}

static const struct videobuf_queue_ops videobuf_queue_ops_grb =
{
	.buf_setup   = buf_setup_grb,	// Is called early in the I/O process, when streaming is being initiated, its purpose is to tell videobuf about the I/O stream.
									// The count parameter will be a suggested number of buffers to use; the driver should check it for rationality and adjust it if need be.
									// As a practical rule, a minimum of two buffers are needed for proper streaming, and there is usually a maximum (which cannot exceed 32) which makes sense for each device.
									// The size parameter should be set to the expected (maximum) size for each frame of data.
	.buf_prepare = buf_prepare_grb,	// Each buffer (in the form of a struct videobuf_buffer pointer) will be passed to buf_prepare().
									// which should set the buffer’s size, width, height, and field fields properly.
	.buf_queue   = buf_queue_grb,	// When a buffer is queued for I/O, it is passed to buf_queue(), which should put it onto the driver’s list of available buffers and set its state to VIDEOBUF_QUEUED.
									// Note also that videobuf may wait on the first buffer in the queue; placing other buffers in front of it could again gum up the works.
									// So use list_add_tail() to enqueue buffers.
	.buf_release = buf_release_grb,	// Finally, buf_release() is called when a buffer is no longer intended to be used.
};									// The driver should ensure that there is no I/O active on the buffer, then pass it to the appropriate free routine(s):


static int device_open( struct file *file_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr) ;

	dprintk(1, "Open video4linux2 device. Name: %s \n" , grb_info_ptr->video_dev.name );

	videobuf_queue_dma_contig_init( &grb_info_ptr->videobuf_queue_grb,
									&videobuf_queue_ops_grb,
									grb_info_ptr->dev,
									&grb_info_ptr->irq_lock,
									V4L2_BUF_TYPE_VIDEO_CAPTURE,
									V4L2_FIELD_NONE,
									sizeof(struct videobuf_buffer),
									grb_info_ptr,
									NULL );
	return 0;
}

static int device_release(struct file *file_ptr ) {
	struct video_device *video_dev = video_devdata( file_ptr );
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	int err;

	dprintk( 1, "Close video4linux2 device. Name: %s \n" , video_dev->name );

	videobuf_stop(  &grb_info_ptr->videobuf_queue_grb );			// The call to videobuf_stop() terminates any I/O in progress-though it is still up to the driver to stop the capture engine.
	err = videobuf_mmap_free( &grb_info_ptr->videobuf_queue_grb );	// The call to videobuf_mmap_free() will ensure that all buffers have been unmapped.
	if( err )														// If so, they will all be passed to the buf_release() callback. If buffers remain mapped, videobuf_mmap_free() returns an error code instead.
		dprintk( 1, "videobuf_mmap_free failed,err=%d\n", err );
	return 0;
}

static int device_mmap( struct file *file_ptr, struct vm_area_struct *vma ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	int ret = videobuf_mmap_mapper( &grb_info_ptr->videobuf_queue_grb, vma );
	// dprintk(1, C_YELLOW"device_mmap return: vm_start=%lx,vm_end=%lx"C_CLEAR"\n", vma->vm_start, vma->vm_end );
	return ret;
}

static void video_dev_release(struct video_device *video_dev ) {
}

static struct v4l2_file_operations fops =
{
	.owner          = THIS_MODULE,
	.open           = device_open,
	.release        = device_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = device_mmap
};

static int vidioc_querycap_grb ( struct file* file_ptr, void* fh, struct v4l2_capability* v4l2_cap_ptr )
{
	// struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	struct video_device *vfd = video_devdata( file_ptr );

	strlcpy( v4l2_cap_ptr->driver, DRIVER_NAME, sizeof(v4l2_cap_ptr->driver) );
	strlcpy( v4l2_cap_ptr->card, DEVICE_NAME, sizeof(v4l2_cap_ptr->card) );
	strlcpy( v4l2_cap_ptr->bus_info, "APB: 0x", sizeof(v4l2_cap_ptr->bus_info) ) ;
	v4l2_cap_ptr->version = 1;
	v4l2_cap_ptr->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;	
	v4l2_cap_ptr->device_caps = vfd->device_caps; // drivers must not change device_caps
	v4l2_cap_ptr->reserved[0] = v4l2_cap_ptr->reserved[1] = v4l2_cap_ptr->reserved[2] = 0;

	dprintk(1, "vidioc_querycap: driver:%s,card:%s,bus_info:%s,version:%d\n",
			v4l2_cap_ptr->driver, v4l2_cap_ptr->card, v4l2_cap_ptr->bus_info, v4l2_cap_ptr->version );
	return 0;
}

/*
static int vidioc_cropcap_grb ( struct file *file_ptr, void *fh, struct v4l2_cropcap *v4l2_cropcap_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	return 0;
}

static int vidioc_g_crop_grb ( struct file *file_ptr, void *fh, struct v4l2_crop *v4l2_crop_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	v4l2_crop_ptr->c = grb_info_ptr->cropping;
	return 0;
}

static int vidioc_s_crop_grb ( struct file *file_ptr,void *fh, const struct v4l2_crop *v4l2_crop_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	if (v4l2_crop_ptr->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	grb_info_ptr->cropping = v4l2_crop_ptr->c;
	return 0;
}
*/

static int vidioc_g_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *v4l2_format_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);

	struct v4l2_pix_format_mplane pix_mp = {0};				/* V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE */
	struct v4l2_window win = {{0}};							/* V4L2_BUF_TYPE_VIDEO_OVERLAY */
	struct v4l2_vbi_format vbi = {0};						/* V4L2_BUF_TYPE_VBI_CAPTURE */
	struct v4l2_sliced_vbi_format sliced = {0};				/* V4L2_BUF_TYPE_SLICED_VBI_CAPTURE */
	struct v4l2_sdr_format sdr = {0};						/* V4L2_BUF_TYPE_SDR_CAPTURE */
	struct v4l2_meta_format meta = {0};						/* V4L2_BUF_TYPE_META_CAPTURE */
 // todo
	v4l2_format_ptr->fmt.pix = grb_info_ptr->user_format;	/* V4L2_BUF_TYPE_VIDEO_CAPTURE */
	v4l2_format_ptr->fmt.pix_mp = pix_mp;
	v4l2_format_ptr->fmt.win = win;
	v4l2_format_ptr->fmt.vbi = vbi;
	v4l2_format_ptr->fmt.sliced = sliced;
	v4l2_format_ptr->fmt.sdr = sdr;
	v4l2_format_ptr->fmt.meta = meta;
	return 0;
}

/*negotiate the format of data (typically image format) exchanged between driver and application */
static int vidioc_s_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *v4l2_format_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	return set_output_format( grb_info_ptr, v4l2_format_ptr );
}

/*
	Функция VIDIOC_REQBUFS позволяет проинициализировать буфер памяти внутри устройства.
	struct v4l2_requestbuffers req:
	__u32 count;		// количество буферов
	__u32 type;			// тип или цель использования
	__u32 memory;		// режим работы с памятью.
	__u32 reserved[2];	// всегда в ноль
*/

static int vidioc_reqbufs_grb ( struct file *file_ptr, void *fh, struct v4l2_requestbuffers *req ) { // VIDIOC_REQBUFS
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	unsigned long flags;
	int ret;

	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );

	ret = videobuf_reqbufs( &grb_info_ptr->videobuf_queue_grb, req );
	dprintk( 1, "vidioc_reqbufs_grb return: count=%u,type=%u,memory=%08x\n", req->count, req->type, req->memory );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );	// для первых 4-х буферов,в очереди появляются 2 буфера с правильным типом,но адреса областей памяти 0
	return ret;
}

/*
	Функция VIDIOC_QUERYBUF позволяет считать параметры буфера, которые будут использоваться для создания memory-mapping области.
	struct v4l2_buffer:
		//до выполнения VIDIOC_QUERYBUF  устанавливаем следующие поля
		__u32 index;		// ноль или номер буфера (если v4l2_requestbuffers.cout > 1)
		__u32 type;			// тип  (совпадает со значением v4l2_requestbuffers.type)
	//после выполнения VIDIOC_QUERYBUF  используем эти поля в качестве параметров для memory-mapping
		__u32   offset;		// смещение буфера относительно начала памяти устройства
		__u32   length;		// размер  буфера
};
*/

static int vidioc_querybuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) { // VIDIOC_QUERYBUF
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_querybuf(&grb_info_ptr->videobuf_queue_grb, buf);
	dprintk( 1, "vidioc_querybuf_grb return: index=%u,type=%u,offset=%08x,length=%08x\n", buf->index, buf->type, buf->m.offset, buf->length );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );	// для первых 4-х буферов,в очереди по прежнему 2 буфера с правильным типом,но адреса областей памяти 0
	return ret;
}

/*
	Функция VIDIOC_QBUF ставит буфер в очередь обработки драйвером устройства.
	Поля используются такие же, как и для VIDIOC_REQBUFS или VIDIOC_QUERYBUF.
*/

static int vidioc_qbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) { // VIDIOC_QBUF
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_qbuf(&grb_info_ptr->videobuf_queue_grb, buf);
	dprintk( 1, "vidioc_qbuf_grb return: index=%u,type=%u,offset=%08x,length=%08x\n", buf->index, buf->type, buf->m.offset, buf->length );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );	// для первых 4-х буферов
	return ret;
}

/*
	Функция VIDIOC_DQBUF освобождает буфер из очереди обработки драйвера.
	В результате можем получить ошибку EAGAIN. Ничего опасного в этом нет, надо еще раз вызвать VIDIOC_DQBUF.
	Это происходит потому, что драйвер еще обрабатывает запрос и не может освободить буфер из очереди.
	При успешном выполнении этой функции, мы получаем в «руки» нашу картинку.
*/

static int vidioc_dqbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {	// VIDIOC_DQBUF
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	struct videobuf_queue* videobuf_queue = &grb_info_ptr->videobuf_queue_grb;
	int ret;
	dprintk( 1, "vidioc_dqbuf_grb entry,memory=%08x\n", buf->memory );
	ret = videobuf_dqbuf( videobuf_queue, buf, file_ptr->f_flags & O_NONBLOCK );
	dprintk( 1, "vidioc_dqbuf_grb return: ret=%d\n", ret );
	return ret;
}

/* Функция VIDIOC_STREAMON включает камеру в режим захвата */

static int vidioc_streamon_grb( struct file *file_ptr, void *fh, enum v4l2_buf_type type ) { // VIDIOC_STREAMON
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	dprintk( 1, "vidioc_streamon_grb\n" );
	if( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	retval = set_register( grb_info_ptr );
	if( retval < 0 ) {
		dprintk(1, "set_register failed \n");
		return retval;
	}
	return videobuf_streamon( &grb_info_ptr->videobuf_queue_grb );
}

static int vidioc_streamoff_grb( struct file *file_ptr, void *__fh, enum v4l2_buf_type type ) { // VIDIOC_STREAMOFF
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	dprintk( 1, "vidioc_streamoff_grb\n" );

	if ( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	retval = videobuf_streamoff( &grb_info_ptr->videobuf_queue_grb );
	reset_grab( grb_info_ptr->base_addr_regs_grb );
	return retval;
}

static void drv_set_color_conv( struct grb_info *grb_info_ptr, void *arg ) {
	struct coef_conv* conv = (struct coef_conv*)arg;
	grb_info_ptr->c_conv = *conv;
}

static void drv_set_gamma( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_gamma* gam = (struct grb_gamma*)arg;
	grb_info_ptr->gam = *gam;						// копируем таблицу
}

static void drv_vidioc_g_params( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_parameters* param = (struct grb_parameters*)arg;
	param = &grb_info_ptr->param;
}

static int drv_vidioc_s_params( struct grb_info *grb_info_ptr, void *arg )
{ // VIDIOC_S_PARAMS
	struct grb_parameters* param = (struct grb_parameters*)arg;
	grb_info_ptr->param = *param;
	return set_input_format( grb_info_ptr, param );
}

static int drv_vidioc_auto_detect( struct grb_info *grb_info_ptr, void *arg )
{
	struct v4l2_pix_format* recognize_format = (struct v4l2_pix_format*)arg;

	if( reset_grab( grb_info_ptr->base_addr_regs_grb ) )
		return -EIO;

	init_completion( &grb_info_ptr->cmpl );
	write_register( 1 , grb_info_ptr->base_addr_regs_grb, ADDR_REC_ENABLE );
	set_mask( grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK, INT_DONE, true );
	// print_register( grb_info_ptr->base_addr_regs_grb, 0x100, 0x108 );

	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, HZ ) == 0 ) {
		dprintk( 1 , "Vidioc autodetection: timeout\n" );
		return -ETIMEDOUT;
	}

	recognize_format->width = grb_info_ptr->recognize_format.width;			// Image width in pixels.
	recognize_format->height = grb_info_ptr->recognize_format.height;		// Image height in pixels. If field is one of V4L2_FIELD_TOP, V4L2_FIELD_BOTTOM or V4L2_FIELD_ALTERNATE then height refers to the number of lines
																			// in the field, otherwise it refers to the number of lines in the frame (which is twice the field height for interlaced formats)
	recognize_format->pixelformat = 0;										// This is a little endian four character code

	recognize_format->field = grb_info_ptr->recognize_format.field;			// Field order, from enum v4l2_field. Video images are typically interlaced
																			// Applications can request to capture or output only the top or bottom field, or both fields interlaced or sequentially
																			// stored in one buffer or alternating in separate buffers. Drivers return the actual field order selected
	recognize_format->bytesperline = 0;										// Distance in bytes between the leftmost pixels in two adjacent lines
	recognize_format->sizeimage = grb_info_ptr->recognize_format.sizeimage;
	recognize_format->colorspace = 0;
	recognize_format->priv = 0;
	recognize_format->flags = 0;
	recognize_format->ycbcr_enc = 0;
	recognize_format->quantization = 0;
	recognize_format->xfer_func = 0;
	return 0;
}

static long vidioc_default_grb( struct file *file_ptr, void *fh, bool valid_prio, unsigned int cmd, void *arg )
{
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	switch (cmd) {
	case VIDIOC_SET_GAMMA:
		drv_set_gamma( grb_info_ptr, arg );
		return 0;
	case VIDIOC_SET_COLOR_CONV:
		drv_set_color_conv( grb_info_ptr, arg );
		return 0;
	case VIDIOC_G_PARAMS:
		drv_vidioc_g_params( grb_info_ptr, arg );
		return 0;
    case VIDIOC_S_PARAMS:
		drv_vidioc_s_params( grb_info_ptr, arg );
		return 0;
	case VIDIOC_AUTO_DETECT:
		return drv_vidioc_auto_detect( grb_info_ptr, arg );
	default:
		return -ENOTTY;
	}
}

static int vidioc_g_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	s->r = grb_info_ptr->cropping; // rect
	return 0;
}

static int vidioc_s_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	grb_info_ptr->cropping = s->r; // rect
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

static struct videobuf_buffer *grb_next_buffer( struct grb_info *grb_info_ptr ) { // from context
	struct videobuf_buffer *vb = NULL;
//	unsigned long flags;

//	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
//	if (cam->opstate != S_RUNNING)
//		goto out;
	if( list_empty( &grb_info_ptr->buffer_queue ) )
		goto out;
//	dprintk(1, "grb_next_buffer: level 0 \n");
	vb = list_entry( grb_info_ptr->buffer_queue.next, struct videobuf_buffer, queue );
	
//	dprintk(1, "grb_next_buffer: level 0 \n");
//	if (!waitqueue_active(&vb->done)) {/* Nobody waiting */
//		vb = NULL;
//		goto out;
//	}
	list_del( &vb->queue );
	
//	vb = list_entry(grb_info_ptr->buffer_queue.next, struct videobuf_buffer, queue);
	vb->state = VIDEOBUF_ACTIVE;
out:
//	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	return vb;
}

static irqreturn_t proc_interrupt (struct grb_info *grb_info_ptr) {
	int rd_data;
	void __iomem *base_addr;
	struct videobuf_buffer *vb;
	u32 base_addr_dma0, base_addr_dma1, base_addr_dma2;
	u32 addr_dma0_addr, addr_dma1_addr, addr_dma2_addr;
	int switch_page;
	static unsigned int buf_num = 0;

	base_addr = grb_info_ptr->base_addr_regs_grb;
	rd_data = read_register( base_addr, ADDR_INTERRUPTION );
	if( rd_data != 1 )
		return IRQ_NONE;

	rd_data = read_register( base_addr, ADDR_INT_STATUS);

	if( (rd_data>>TEST_INT)&1 ) {
		write_register( 0 , base_addr, ADDR_TEST_INT );
		set_mask( base_addr, ADDR_INT_MASK, TEST_INT, false );
		write_register( (1<<TEST_INT), base_addr, ADDR_INT_STATUS );
		dprintk( 2, "irq_handler: test interruption detected \n" );
		complete_all( &grb_info_ptr->cmpl );
	}
	else if( (rd_data>>END_WRITE)&1 ) {
		write_register( (1<<END_WRITE), base_addr, ADDR_INT_STATUS );

		wake_up_all( &grb_info_ptr->wait_queue );
		
		if( ( vb = grb_next_buffer(grb_info_ptr) ) == NULL ) {
			dprintk( 2, "irq_handler: next buffer 0 (%u)\n", buf_num++ );
			write_register( 0, base_addr, ADDR_INT_MASK	);
			write_register( 0, base_addr, ADDR_ENABLE );
			return IRQ_HANDLED;
		}
		
		vb->state = VIDEOBUF_DONE;

		wake_up( &vb->done );

		switch_page = grb_info_ptr->frame_count % 2;
		
		dprintk(2, "irq_handler: frame_count = %d; switch_page = %d \n", grb_info_ptr->frame_count , switch_page);
		//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 2 );

		base_addr_dma0 = videobuf_to_dma_contig_rcm( vb );
		base_addr_dma1 = base_addr_dma0 + grb_info_ptr->mem_offset1 ;
		base_addr_dma2 = base_addr_dma0 + grb_info_ptr->mem_offset2 ;
		
		if (switch_page == 0)
		{
			addr_dma0_addr = ADDR_DMA0_ADDR0;	// 0x404
			addr_dma1_addr = ADDR_DMA1_ADDR0;	// 0x40c
			addr_dma2_addr = ADDR_DMA2_ADDR0;	// 0x414
		}
		else // if (switch_page == 1)
		{
			addr_dma0_addr = ADDR_DMA0_ADDR1;	// 0x408
			addr_dma1_addr = ADDR_DMA1_ADDR1;	// 0x410
			addr_dma2_addr = ADDR_DMA2_ADDR1;	// 0x418
		}
		
		write_register( base_addr_dma0, base_addr, addr_dma0_addr );
		write_register( base_addr_dma1, base_addr, addr_dma1_addr );
		write_register( base_addr_dma2, base_addr, addr_dma2_addr );
		grb_info_ptr->frame_count = grb_info_ptr->frame_count + 1;
	}
	else if( (rd_data>>INT_DONE)&1 ) {
		set_mask( base_addr, ADDR_INT_MASK, TEST_INT, false );
		rd_data = read_register( base_addr, ADDR_FRAME_SIZE );
		grb_info_ptr->recognize_format.height  = (rd_data >> 16) & 0xFFF;	// 27:16-вертикальный размер изображения
		grb_info_ptr->recognize_format.width = rd_data & 0xFFF;				// 11:0-горизонтальный размер изображения
		rd_data = read_register( base_addr, ADDR_FRAME_PARAM );
		grb_info_ptr->recognize_format.field = rd_data;						// несоответствие стандарту
		grb_info_ptr->recognize_format.sizeimage  = 0;
		complete_all( &grb_info_ptr->cmpl );
	}
	else
	{
		dprintk(2, "irq_handler: other, interrupt status %08x!\n", rd_data );
		//write_register( 1, base_addr, ADDR_PR_RESET );
	}
	return IRQ_HANDLED;
}

static irqreturn_t irq_handler( int irq, void* dev ) {
	struct grb_info *grb_info_ptr = (struct grb_info*) dev;
	irqreturn_t grb_irq;
	spin_lock( &grb_info_ptr->irq_lock );
	grb_irq = proc_interrupt( grb_info_ptr );
	spin_unlock( &grb_info_ptr->irq_lock );
	return grb_irq;
}

static int get_memory_buffer_address( const struct platform_device* grb_device,  const char* res_name, struct resource* res_ptr ) {
	int ret;
	struct device_node* np;
	if( ( np  = of_parse_phandle( grb_device->dev.of_node, res_name, 0 ) ) == NULL ) {
		dprintk(1, "ERROR : can't get the device node of the memory-region"); 
		return -ENODEV;
	}
	if( ( ret = of_address_to_resource( np, 0, res_ptr ) ) != 0 ) {
		dprintk(1, "ERROR : can't get the resource of the memory area");
	}
	return ret;
}

static int test_interrupt( struct grb_info *grb_info_ptr ) {				// дергаем тестовое прерывание,должен вызваться обработчик
	init_completion( &grb_info_ptr->cmpl );
	write_register( 0x1, grb_info_ptr->base_addr_regs_grb, ADDR_TEST_INT ); // 0x208
	write_register( 0x1, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK ); // 0x204
	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, HZ ) == 0 ) {
		dprintk( 1 , "Test interrupt: timeout\n" );
		return -ETIMEDOUT;
	}
	return 0;
}

static int get_resources( struct platform_device* grb_device , struct grb_info *grb_info_ptr ) {
	struct resource* grb_res_ptr;

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_MEM, 0 );
	if( !grb_res_ptr ) {
		dprintk( 1, "ERROR : can't get the base address registers grabber" ); 
		return -EINVAL;
	}

	grb_info_ptr->base_addr_regs_grb = devm_ioremap_resource( &grb_device->dev, grb_res_ptr );
	if( IS_ERR( grb_info_ptr->base_addr_regs_grb ) ) {
		dprintk(1, "ERROR : can't ioremap grabber resource");
		return PTR_ERR( grb_info_ptr->base_addr_regs_grb );
	}

	if ( read_register( grb_info_ptr->base_addr_regs_grb, ADDR_ID_REG) != GRB_DEVID ) {
		dprintk( 1, "invalid resources, device ""grabber"" not found! \n" );
		return -ENODEV;
	}

	if( get_memory_buffer_address( grb_device, "memory-region", grb_res_ptr ) ) {
		dprintk( 1, "ERROR : can't get the base address of the memory area" ); 
		return -EINVAL;
	}

	grb_info_ptr->buff_phys_addr = grb_res_ptr->start;
	grb_info_ptr->buff_length = grb_res_ptr->end - grb_res_ptr->start + 1;
	grb_info_ptr->buff_dma_addr = (dma_addr_t)(grb_info_ptr->buff_phys_addr - (grb_device->dev.dma_pfn_offset << PAGE_SHIFT));

	//dprintk(1, "The base address of the memory area and finish address (%llx,%llx)\n", grb_res_ptr->start, grb_res_ptr->end);
	//dprintk(1, "grv_b2: buff_phys_addr = %llx,buff_dma_addr = %llx,\n", grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_dma_addr );

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_IRQ, 0 );
	if( !grb_res_ptr ) {
		dprintk( 1, "ERROR : can't get base irq" );
		return -EINVAL;;
	}
	// dev_info(&grb_device->dev,"irq resource (%08x)\n", (unsigned int)grb_res_ptr->start);	
	grb_info_ptr->num_irq = grb_res_ptr->start;
	dprintk(1, "Number IRQ = %d\n", grb_info_ptr->num_irq);
	return 0;
}

static int device_probe (struct platform_device *grb_device)
{
	struct grb_info *grb_info_ptr;
	int res;

	dprintk( 1, "Probe started!\n" );

	grb_info_ptr = kzalloc(sizeof(struct grb_info), GFP_KERNEL); // memory allocation for inner structure and initialize by zeroes
	if( grb_info_ptr == NULL ) {
		dprintk(1, "Can't allocated memory!\n");
		return -ENOMEM;
	}
	grb_info_ptr->dev = &grb_device->dev;
	
	if( get_resources (grb_device , grb_info_ptr) < 0 ) {
		kfree(grb_info_ptr);
		return -1;
	}

	grb_device->dma_mask = DMA_BIT_MASK(32);
	grb_device->dev.archdata.dma_offset = 0;										// (grb_device->dev.dma_pfn_offset << PAGE_SHIFT);

	res = dma_declare_coherent_memory( &grb_device->dev,							// dev->dma_mem заполняется
									   grb_info_ptr->buff_phys_addr,
									   grb_info_ptr->buff_phys_addr,				// grb_info_ptr->buff_dma_addr,
									   grb_info_ptr->buff_length );					// DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);

	grb_info_ptr->kern_virt_addr = phys_to_virt( grb_info_ptr->buff_phys_addr );
	dprintk( 1,
			 "Declare coherent memory: for phys addr %llx, dma addr %llx, kern virt addr %x, size %x\n",
			 grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_dma_addr, (u32)grb_info_ptr->kern_virt_addr, grb_info_ptr->buff_length );

	if( res ) {
		dprintk( 1, "Dma dcm error %d\n" , res );
		return -ENOMEM;
	}
	
	platform_set_drvdata( grb_device, grb_info_ptr ); 

	grb_info_ptr->video_dev.dev_parent = grb_info_ptr->dev;
	grb_info_ptr->video_dev.fops = &fops;
	grb_info_ptr->video_dev.ioctl_ops = &grb_ioctl_ops;
	strlcpy( grb_info_ptr->video_dev.name, DRIVER_NAME, sizeof(grb_info_ptr->video_dev.name) );
	grb_info_ptr->video_dev.release = video_dev_release;
	
	grb_info_ptr->video_dev.tvnorms = V4L2_STD_ATSC_8_VSB + V4L2_STD_ATSC_16_VSB;

	res = v4l2_device_register( grb_info_ptr->dev, &grb_info_ptr->v4l2_device );
	if (res) {
		dprintk( 1, "Failed v4l2_device register, error %d\n", res );
		return res;
	}
	grb_info_ptr->video_dev.v4l2_dev = &grb_info_ptr->v4l2_device;
	grb_info_ptr->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS;

	video_set_drvdata( &grb_info_ptr->video_dev , grb_info_ptr );
	res = video_register_device( &grb_info_ptr->video_dev, VFL_TYPE_GRABBER, -1 );
	if( res ) {
		dprintk( 1, "Failed video_dev register, error %d\n", res );
		video_device_release( &grb_info_ptr->video_dev ); /* or kfree(my_vdev); */
		return res;
	}
	
//	dprintk(1, "video_device.dev.init_name: %s\n", &grb_info_ptr->video_dev.dev.init_name);
	
	init_waitqueue_head( &grb_info_ptr->wait_queue );

	res = request_irq( grb_info_ptr->num_irq, irq_handler, IRQF_SHARED, DEVICE_NAME, grb_info_ptr );
	if( res ) {
		dprintk( 1, "ERROR : request_irq isn't availiable\n" );
		kfree( grb_info_ptr );
		return res;
	}

	if( test_interrupt( grb_info_ptr ) ) {
		dprintk( 1, "ERROR : test interrupt failed\n" );
		kfree( grb_info_ptr );
		return -ETIMEDOUT;
	}

	spin_lock_init( &grb_info_ptr->irq_lock );
	return 0;
}

static int device_remove( struct platform_device* grb_device )
{
	struct grb_info* grb_info_ptr;

	dprintk( 1, "In grabber_remove function : %p\n", grb_device );

	grb_info_ptr = platform_get_drvdata(grb_device);
	reset_grab( grb_info_ptr->base_addr_regs_grb );

	video_unregister_device( &grb_info_ptr->video_dev );

	if( grb_info_ptr->num_irq )
		free_irq( grb_info_ptr->num_irq, grb_info_ptr );

	if(  grb_info_ptr )
		kfree( grb_info_ptr );

	return 0 ;
}

static struct of_device_id grb_match[] = 
{
	{ .compatible = "rcm,vdu-graber" },
	{}
};

static struct platform_driver module_grb_driver =
{
	.driver = {
		.owner	= THIS_MODULE,
		.name = "grabber",
		.of_match_table = grb_match
	},
	.probe	= device_probe	,
	.remove	= device_remove
};

module_platform_driver( module_grb_driver );

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, grb_match);


/*
static int grb_v2_init( void )
{
	dprintk( 1, "Grabber driver mounted!\n" );	
	if (!platform_driver_register(&pl_dr))
		dprintk( 1, "platform_driver_register finished successfully \n" );
	else
		return (-1);
	return 0;
}

static void grb_v2_exit( void )
{
	dprintk( 1, "Exit from module grabber\n" );
	platform_driver_unregister( &pl_dr );
	return;
}

module_init(grb_v2_init);
module_exit(grb_v2_exit);
*/