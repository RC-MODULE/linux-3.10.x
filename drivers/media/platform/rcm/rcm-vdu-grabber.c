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

#include "rcm-vdu-grabber.h"

#define RCM_VDU_GRB_DBG

#ifdef RCM_VDU_GRB_DBG
	#define GRB_DBG_PRINT(...) printk( KERN_DEBUG "[VDU_GRABER] " __VA_ARGS__ );
#else
	#define GRB_DBG_PRINT(...) while(0);
#endif

#define PI GRB_DBG_PRINT("%s\n",__FUNCTION__)

#ifdef RCM_VDU_GRB_DBG

struct videobuf_dma_contig_memory {
	u32 magic;
	void *vaddr;
	dma_addr_t dma_handle;
	unsigned long size;
};

static void print_videobuf_queue_param( struct videobuf_queue* queue, unsigned int num ) {
	unsigned int i=0 ;
	struct videobuf_dma_contig_memory* mem;
	struct videobuf_buffer* vb;
	for( i=0; i<num; i++ ) {
		vb = queue->bufs[i];		// buffer ptr
		if( vb ) {
			mem = vb->priv;			// memory area
			GRB_DBG_PRINT( "videobuf_buffer(%u): memory=%u,vaddr=%x,dma_handle=%x,size=%lx: %*ph\n",
						   i, vb->memory, (u32)mem->vaddr, (u32)mem->dma_handle, mem->size, 16, mem->vaddr )
		}
	}
}

static void print_videobuf_queue_param2( struct videobuf_queue* queue, int num, int off0, int off1, int off2 ) {
	struct videobuf_dma_contig_memory* mem;
	struct videobuf_buffer* vb = queue->bufs[num];
	if( vb ) {
		u32 p0, p1, p2;
		mem = vb->priv;
		p0 = (u32)(mem->vaddr+off0), p1 = (u32)(mem->vaddr+off1), p2 = (u32)(mem->vaddr+off2);
		GRB_DBG_PRINT( "videobuf_buffer: memory=%u,vaddr=%x,dma_handle=%x,size=%lx:\n%08x:%*ph\n%08x:%*ph\n%08x:%*ph\n",
				 	   vb->memory, (u32)mem->vaddr, (u32)mem->dma_handle, mem->size,
					   p0, 8, (void*)p0, p1, 8, (void*)p1, p2, 8, (void*)p2 )
	} 
}

static void print_four_cc( const char* info, unsigned int f ) {
	GRB_DBG_PRINT( "%s: '%c%c%c%c'", info, (u8)(f>>0), (u8)(f>>8), (u8)(f>>16), (u8)(f>>24) )
}

static void print_v4l2_selection( const char* info, int arg, const struct v4l2_selection* s ) {
		GRB_DBG_PRINT( "%s(%08x): target=%x,flag=%x,rect=%x,%x,%x,%x\n",
						info, arg, s->target, s->flags, s->r.left, s->r.top, s->r.width, s->r.height )
}

static void print_v4l2_format( const char* info, int arg, const struct v4l2_format* f ) {
	GRB_DBG_PRINT( "%s(%08x): type=%x,fmt: width=%x,height=%x,pixelformat=%x\n"
					"field=%x,bytesperline=%x,sizeimage=%x,colorspace=%x,priv=%x,flags=%x\n"
					"ycbcr_enc/hsv_enc=%x,quantization=%x,xfer_func=%x\n",
					info, arg, f->type, f->fmt.pix.width, f->fmt.pix.height, f->fmt.pix.pixelformat,
					f->fmt.pix.field, f->fmt.pix.bytesperline, f->fmt.pix.sizeimage, f->fmt.pix.colorspace,f->fmt.pix.priv, f->fmt.pix.flags,
					f->fmt.pix.ycbcr_enc, f->fmt.pix.quantization, f->fmt.pix.xfer_func )
}

static void print_v4l2_buffer( const char* info, const struct v4l2_buffer* b ) {
	GRB_DBG_PRINT( "%s: index=%x,type=%x,bytesused=%x,flags=%x,field=%x,sequence=%x,memory=%x,offset=%x,length=%x\n",
					info, b->index, b->type, b->bytesused, b->flags, b->field, b->sequence, b->memory, b->m.offset, b->length )
}

#endif // RCM_VDU_GRB_DBG

static inline void write_register( u32 val, void __iomem *addr, u32 offset ) {
	iowrite32( val, addr + offset );
}

static inline u32 read_register( void __iomem *addr, u32 offset ) {
	return ioread32( addr + offset );
}

static inline u32 clr_register( u32 mask, void __iomem *addr, u32 offset ) {
	u32 val = ioread32( addr + offset );
	iowrite32( val &= ~mask, addr + offset );
	return val;
}

static inline u32 set_register( u32 mask, void __iomem *addr, u32 offset ) {
	u32 val = ioread32( addr + offset );
	iowrite32( val |= mask, addr + offset );
	return val;
}

static inline void print_registers( void __iomem *addr, unsigned int begin, unsigned int end ) {
	unsigned int reg, val;
	for( reg=begin; reg<=end; reg+=4 ) {
		val = read_register( addr, reg );
		GRB_DBG_PRINT( "get(0x%08X, 0x%08X)\n", reg, val )
	}
}

static int reset_grab( void __iomem *addr ) {
	unsigned int i;
	write_register( 0x01 , addr, ADDR_PR_RESET );
	for( i = 0; i < 50000; i++ ) {
		if( read_register( addr, ADDR_PR_RESET ) == 0 ) {
			return 0;
		}
	}
	GRB_DBG_PRINT( "Grabber reset: timeout\n" )
	return -1;
}

static int set_input_format( struct grb_info *grb_info_ptr, struct grb_parameters *param ) {
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
				GRB_DBG_PRINT( "invalid parameter param.v_if!\n" ) 
				return -EINVAL;
			}
		}
		else if( param->std_in == HD ) {
			active_bus = 0x2 ;							// d0,d1
		}
		else {
			active_bus = 0x0 ;
			GRB_DBG_PRINT( "invalid parameter param.std_in!\n" )
			return -EINVAL;
		}
		grb_info_ptr->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:2:2\n" )
	}
	else if( param->d_format == YCBCR444 ) {			// 1
		format_data = 0x0 ;								// YCbCr
		active_bus  = 0x3 ;								// d0,d1,d2
		grb_info_ptr->in_f.color = YCBCR;
		GRB_DBG_PRINT( "input format is YCBCR 4:4:4\n" ) 
	} 
	else if( param->d_format == RGB888 ) {				// 1
		format_data = 0x1 ;								// RGB
		active_bus  = 0x3 ;								// d0,d1,d2
		grb_info_ptr->in_f.color = RGB ;
		GRB_DBG_PRINT( "input format is YCBCR 4:4:4\n" ) 
	}
	else {
		active_bus  = 0x0;
		format_data = 0x0;
		grb_info_ptr->in_f.color = 0x0;
		GRB_DBG_PRINT( "std_grb: unknown standard \n" )
		return -EINVAL;
	}

	grb_info_ptr->in_f.format_din = format_data;			// 1–RGB,0-YCbCr
	grb_info_ptr->in_f.format_din |= (active_bus << 1) ;	// 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
	grb_info_ptr->in_f.format_din |= (param->std_in << 3) ;	// 0-not duplication,1–duplication (SDTV)
	grb_info_ptr->in_f.format_din |= (param->sync << 4) ;	// synchronization: 0–external(hsync,vsync,field,data_enable); 1–internal(EAV,SAV)
	grb_info_ptr->in_f.color_std = param->std_in;			// input mode
	grb_info_ptr->out_f.color_std = param->std_out;			// output mode (SD,HD)

	if( param->alpha > 255 ) { 
		GRB_DBG_PRINT( "alpha > 255\n" )
		return -EINVAL;
	}
	return 0;
}

static int set_output_format( struct grb_info *grb_info_ptr, struct v4l2_format *v4l2_format_ptr ) {
	grb_info_ptr->user_format = v4l2_format_ptr->fmt.pix;
// 0:   0-YCBCR, 1-RGB
// 2,1: 01–ARGB8888, 10–YCbCr422, 11-YCbCr444,YCbCr422 too?,RGB888
// 3:   0–YCbCr422, 1–YCbCr444
	switch( v4l2_format_ptr->fmt.pix.pixelformat ) {
	case V4L2_PIX_FMT_RGB32:									// 0,'RGB4'
		grb_info_ptr->out_f.format_dout = 0x03;
		grb_info_ptr->out_f.color = RGB;
		GRB_DBG_PRINT( "pixelformat: ARGB8888\n" )
		break;
	case V4L2_PIX_FMT_NV16:										// 1,'NV16' These are two-plane versions of the YUV 4:2:2 format.
		grb_info_ptr->out_f.format_dout = 0x04;					// The Y plane has one byte per pixel. For V4L2_PIX_FMT_NV16,
		grb_info_ptr->out_f.color = YCBCR;						// a combined CbCr plane immediately follows the Y plane in memory.
		GRB_DBG_PRINT( "pixelformat: YCBCR422 two planes\n" )	// The CbCr plane is the same width and height, in bytes, as the Y plane (and of the image).
		break;
	case V4L2_PIX_FMT_YUV420:									// 'YU12' It's for check only. The three components are separated into three sub- images or planes.
	case V4L2_PIX_FMT_YUV422M:									// 2,'YM16' Three plane format. The Y plane is first. The Y plane has one byte per pixel.
		grb_info_ptr->out_f.format_dout = 0x06;					// For V4L2_PIX_FMT_YUV422M the Cb data constitutes the second plane which is half the width of the Y plane (and of the image).
		grb_info_ptr->out_f.color = YCBCR;						// Each Cb belongs to two pixels. For example, Cb0 belongs to Y’00, Y’01.
		GRB_DBG_PRINT( "pixelformat: YCBCR422 three planes\n" )	// The Cr data, just like the Cb plane, is in the third plane.
		break;
	case V4L2_PIX_FMT_YUV444M:									// 3,'YM24' The three components are separated into three sub-images or planes.The Y plane is first. The Y plane has one byte per pixel.
		grb_info_ptr->out_f.format_dout = 0x0e;					// For V4L2_PIX_FMT_YUV444M the Cb data constitutes the second plane
		grb_info_ptr->out_f.color = YCBCR;						// which is the same width and height as the Y plane (and as the image).
		GRB_DBG_PRINT( "pixelformat: YCBCR444 three planes.\n" )// The Cr data, just like the Cb plane, is in the third plane.
		break;
	case V4L2_PIX_FMT_RGB24:									// 'RGB3',but planar,no packet!!!Is's bad.
		grb_info_ptr->out_f.format_dout = 0x07;
		grb_info_ptr->out_f.color = RGB;
		GRB_DBG_PRINT( "pixelformat: RGB888\n" )
		break;
	default:
		grb_info_ptr->out_f.format_dout = 0x00;
		GRB_DBG_PRINT( "pixelformat: unknown pixelformat!\n" )
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
		GRB_DBG_PRINT( "output size mismatch!\n" )
		return -EINVAL;
	} 
	return 0;
}

static int setup_color( struct grb_info *grb_info_ptr, void __iomem* base_addr ) {
	//void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;

	u32 color_in = grb_info_ptr->in_f.color,
		color_std_in = grb_info_ptr->in_f.color_std,
		color_out = grb_info_ptr->out_f.color,
		color_std_out = grb_info_ptr->out_f.color_std;

	if( ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != SD ) && ( color_std_in != HD ) ) ||
		  ( ( color_in != RGB ) && ( color_in != YCBCR ) ) ||
		  ( ( color_std_in != SD ) && ( color_std_in != HD ) ) ) {
		GRB_DBG_PRINT( "color format is wrong: input-format=%u,standard=%u;output-format=%u,standard=%u\n",
					   color_in, color_std_in, color_out, color_std_out );
		return -EINVAL;
	}

	if( color_in != color_out ) {
		if( color_in == RGB ) {
			if( color_std_out == SD ) {
				GRB_DBG_PRINT( "RGB->YCBCR,SD\n" )
				grb_info_ptr->c_conv = &RGB_TO_YCBCR_SD;
			} 
			else {
				GRB_DBG_PRINT( "RGB->YCBCR,HD\n" )
				grb_info_ptr->c_conv = &RGB_TO_YCBCR_HD;
			}
		}
		else if( color_in == YCBCR ) {
			if( color_std_in == SD ) {
				GRB_DBG_PRINT( "YCBCR->RGB,SD\n" )
				grb_info_ptr->c_conv = &YCBCR_TO_RGB_SD;
			} 
			else {
				GRB_DBG_PRINT( "YCBCR->RGB,HD\n" )
				grb_info_ptr->c_conv = &YCBCR_TO_RGB_HD;
			}
		}
	}
	else if( (color_in == YCBCR) && (color_out == YCBCR) ) {
		GRB_DBG_PRINT( "enable conversion colour standard  for YCBCR.\n" )

		if( (color_std_in == SD) && (color_std_out == HD) ) {
			GRB_DBG_PRINT( "YCBCR SD->HD\n" )
			grb_info_ptr->c_conv = &YCBCR_SD_TO_HD;
		}
		else if( (color_std_in == HD) & (color_std_out == SD) ) {
			GRB_DBG_PRINT( "YCBCR HD->SD\n" )
			grb_info_ptr->c_conv = &YCBCR_HD_TO_SD;
		}
	}
	else // not need color conversion
		grb_info_ptr->c_conv = NULL;
	
	write_register( 0, base_addr, ADDR_CONV_ENABLE );

	if( grb_info_ptr->c_conv ) {
		write_register( grb_info_ptr->c_conv->coef[0][0], base_addr, ADDR_C_0_0 );
		write_register( grb_info_ptr->c_conv->coef[0][1], base_addr, ADDR_C_0_1 );
		write_register( grb_info_ptr->c_conv->coef[0][2], base_addr, ADDR_C_0_2 );
		write_register( grb_info_ptr->c_conv->coef[0][3], base_addr, ADDR_C_0_3 );

		write_register( grb_info_ptr->c_conv->coef[1][0], base_addr, ADDR_C_1_0 );
		write_register( grb_info_ptr->c_conv->coef[1][1], base_addr, ADDR_C_1_1 );
		write_register( grb_info_ptr->c_conv->coef[1][2], base_addr, ADDR_C_1_2 );
		write_register( grb_info_ptr->c_conv->coef[1][3], base_addr, ADDR_C_1_3 );

		write_register( grb_info_ptr->c_conv->coef[2][0], base_addr, ADDR_C_2_0 );
		write_register( grb_info_ptr->c_conv->coef[2][1], base_addr, ADDR_C_2_1 );
		write_register( grb_info_ptr->c_conv->coef[2][2], base_addr, ADDR_C_2_2 );
		write_register( grb_info_ptr->c_conv->coef[2][3], base_addr, ADDR_C_2_3 );

		write_register( grb_info_ptr->c_conv->range[0], base_addr, ADDR_CH0_RANGE );
		write_register( grb_info_ptr->c_conv->range[1], base_addr, ADDR_CH1_RANGE );
		write_register( grb_info_ptr->c_conv->range[2], base_addr, ADDR_CH2_RANGE );
		write_register( 1, base_addr, ADDR_CONV_ENABLE );
	}
	// print_registers( base_addr, ADDR_C_0_0, ADDR_CH2_RANGE );
	return 0;
}

static int check_and_correct_out_format( u32 format_dout, u32* c_hor_size, u32* c_full_size ) {
	switch( format_dout ) {
	case 0x03:
		GRB_DBG_PRINT( "pixelformat: ARGB8888\n" )
		return 0;
	case 0x07:
		GRB_DBG_PRINT( "pixelformat: RGB888\n" )
		return 0;
	case 0x04:
		GRB_DBG_PRINT( "pixelformat: YCBCR422 two planes\n" )
		return 0;
	case 0x06:
		*c_hor_size /= 2;
		*c_full_size /= 2;
		GRB_DBG_PRINT( "pixelformat: YCBCR422 three planes\n" )
		return 0;
	case 0x0e:
		GRB_DBG_PRINT( "pixelformat: YCBCR444 three planes\n" )
		return 0;
	default:
		GRB_DBG_PRINT( "pixelformat: unknown pixelformat\n" )
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
		//GRB_DBG_PRINT( "init gamma table, last word yg=%08x, cr=%08x, cb=%08x\n",
		//			   read_register( base_addr, BASE_ADDR_TABLE_0+252 ),
		//			   read_register( base_addr, BASE_ADDR_TABLE_1+252 ),
		//			   read_register( base_addr, BASE_ADDR_TABLE_2+252 ) )
		write_register( 1, base_addr, ADDR_GAM_ENABLE );
	}
}

int setup_registers( struct grb_info *grb_info_ptr ) {
	void __iomem* base_addr = grb_info_ptr->base_addr_regs_grb;
	u32 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2;
	u32 y_hor_size, y_ver_size, y_full_size;
	u32 c_ver_size, c_hor_size, c_full_size;
	u32 base_point_y, base_point_x;
	u32 format_din;
	u32 format_dout;

	format_din  = grb_info_ptr->in_f.format_din;
	format_dout = grb_info_ptr->out_f.format_dout;
	GRB_DBG_PRINT( "format_din: 0x%0x,format_dout: 0x%0x\n", format_din, format_dout )

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

	grb_info_ptr->mem_offset1 = y_full_size*y_ver_size;											// plane for color component 1
	grb_info_ptr->mem_offset2 = grb_info_ptr->mem_offset1 + c_full_size*c_ver_size;				// plane for color component 2
	
	GRB_DBG_PRINT( "y_hor_size=%d,y_ver_size=%d,c_hor_size=%d,c_ver_size=%d,y_full_size=%d,c_full_size=%d,mem_offset1=%d,mem_offset2=%d,alpha=%d\n",
			 y_hor_size, y_ver_size, c_hor_size, c_ver_size, y_full_size, c_full_size,
			 grb_info_ptr->mem_offset1, grb_info_ptr->mem_offset2, grb_info_ptr->param.alpha )
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 2 ); 						// print vaddr,dma_handle,size for each buffer

	base_addr0_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[0] );	// just return dma address
	base_addr0_dma1 = base_addr0_dma0 + grb_info_ptr->mem_offset1;
	base_addr0_dma2 = base_addr0_dma0 + grb_info_ptr->mem_offset2;	// even

	base_addr1_dma0 = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[1] );
	base_addr1_dma1 = base_addr1_dma0 + grb_info_ptr->mem_offset1;
	base_addr1_dma2 = base_addr1_dma0 + grb_info_ptr->mem_offset2;	// odd

	grb_info_ptr->frame_count = 0;
	grb_info_ptr->next_buf_num = 2;

	GRB_DBG_PRINT( "base_addr0_dma0=%08x,base_addr1_dma0=%08x,base_addr0_dma1=%08x,base_addr1_dma1=%08x,base_addr0_dma2=%08x,base_addr1_dma2=%08x\n",
			 base_addr0_dma0, base_addr1_dma0, base_addr0_dma1, base_addr1_dma1, base_addr0_dma2, base_addr1_dma2 )

	reset_grab( base_addr );
	write_register( 1, base_addr, ADDR_BASE_SW_ENA );											// enable switching base addresses
	write_register( base_addr0_dma0, base_addr, ADDR_DMA0_ADDR0 );								// luminance,odd
	write_register( base_addr0_dma1, base_addr, ADDR_DMA1_ADDR0 );								// color difference 1,odd
	write_register( base_addr0_dma2, base_addr, ADDR_DMA2_ADDR0 );								// color difference 2,odd
	write_register( base_addr1_dma0, base_addr, ADDR_DMA0_ADDR1 );								// luminance,even
	write_register( base_addr1_dma1, base_addr, ADDR_DMA1_ADDR1 );								// color difference 1,even
	write_register( base_addr1_dma2, base_addr, ADDR_DMA2_ADDR1 );								// color difference 2,even
	write_register( U16x2_TO_U32( y_ver_size, y_hor_size ), base_addr, ADDR_Y_SIZE );			// 27:16-height,11:0-width for luminance
	write_register( U16x2_TO_U32( c_ver_size, c_hor_size ), base_addr, ADDR_C_SIZE );			// 27:16-height,11:0-width color difference
	write_register( U16x2_TO_U32( c_full_size, y_full_size ), base_addr, ADDR_FULL_LINE_SIZE );	// 27:16-height,11:0-width full string
	write_register( U16x2_TO_U32( base_point_y, base_point_x ), base_addr, ADDR_BASE_POINT );	// 27:16-base point vert coord,11:0-ase point hor coord
	write_register( grb_info_ptr->param.alpha, base_addr, ADDR_TRANSPARENCY );
// ADDR_MODE:
// 4: 0-hsync, vsync, field and data_enable),1-EAV and SAV
// 3: 0-not duplication,1-duplication (SDTV)
// 2,1: 1–dv0;2–dv0,dv1;3–dv0,dv1,dv2
// 0: input format: 0 – YCbCr,1 – RGB
	write_register( format_din , base_addr, ADDR_MODE );
// ADDR_LOCATION_DATA:
// бит 3:  YCbCr only: 0 – YCbCr 4:2:2; 1 – YCbCr 4:4:4
// биты 2,1: output plane count: 1–ARGB8888; 2–YCbCr 4:2:2; 3–YCbCr 4:4:4,YCbCr 4:2:2 and RGB888
// бит 0: output format 0 – YCbCr; 1 – RGB
	write_register( format_dout , base_addr, ADDR_LOCATION_DATA );
	setup_gamma( base_addr, &grb_info_ptr->gam );
	setup_color( grb_info_ptr, base_addr );
	set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK  );
	write_register( 1 , base_addr, ADDR_ENABLE );

	return 0;
}

static int buf_setup_grb ( struct videobuf_queue *q, unsigned int *count, unsigned int *size ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	int max_buff;
	*size = grb_info_ptr->user_format.sizeimage;
	GRB_DBG_PRINT( "buf setup entry: buff_length=%u,size=%u,count=%u\n", grb_info_ptr->buff_length, *size, *count )
	if( *size == 0 )
		return -EINVAL;
	max_buff = grb_info_ptr->buff_length / *size;
	if( *count < 2 )
		*count = 2;
	else if( *count > 32 )
		*count = 32;
	if( *count > max_buff )
		*count = max_buff;
	grb_info_ptr->reqv_buf_cnt = *count;	// save available buffers count
	//print_videobuf_queue_param( q, 4 );
	GRB_DBG_PRINT( "buf setup return: buff_length=%u,size=%u,count=%u\n", grb_info_ptr->buff_length, *size, *count )
	return 0;
}

static int buf_prepare_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb, enum v4l2_field field ) {
	struct grb_info *grb_info_ptr = q->priv_data;
	PI
	vb->size = grb_info_ptr->user_format.sizeimage;
	vb->width = grb_info_ptr->user_format.bytesperline;
	vb->height = grb_info_ptr->user_format.height;
	vb->field = field;

	if( vb->state == VIDEOBUF_NEEDS_INIT ) {
		int ret = videobuf_iolock( q, vb, NULL );
		if( ret ) {
			GRB_DBG_PRINT( "videobuf_iolock failed (%d)\n", ret )
			return ret;
		}
	}
	vb->state = VIDEOBUF_PREPARED;
	//print_videobuf_queue_param( q, 4 );
	return 0;
}

static void buf_queue_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	PI
	list_add_tail( &vb->queue, &grb_info_ptr->buffer_queue );	// Note that videobuf holds the lock when it calls us, so we need not (indeed, cannot) take it here
	vb->state = VIDEOBUF_QUEUED;
	//print_videobuf_queue_param( q, 4 );
}

static void buf_release_grb ( struct videobuf_queue *q, struct videobuf_buffer *vb ) {
	struct grb_info* grb_info_ptr = q->priv_data;
	unsigned long flags;
	PI
	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );				// We need to flush the buffer from the dma queue since hey are de-allocated
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	videobuf_dma_contig_free( q, vb );
	vb->state = VIDEOBUF_NEEDS_INIT;
	//print_videobuf_queue_param( q, 4 );
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

static int drv_vidioc_auto_detect( struct grb_info *grb_info_ptr, void *arg );

static int device_open( struct file *file_ptr ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr) ;
#ifdef RCM_VDU_GRB_DBG
	GRB_DBG_PRINT(
#else
	dev_info( grb_info_ptr->dev,
#endif
			  "Open video4linux2 device. Name: %s, base: %x \n" ,
			  grb_info_ptr->video_dev.name,
			  (u32)grb_info_ptr->phys_addr_regs_grb );

	drv_vidioc_auto_detect( grb_info_ptr, NULL );

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

	dev_info( grb_info_ptr->dev, "Close video4linux2 device. Name: %s \n" , video_dev->name );

	videobuf_stop(  &grb_info_ptr->videobuf_queue_grb );			// The call to videobuf_stop() terminates any I/O in progress-though it is still up to the driver to stop the capture engine.
	err = videobuf_mmap_free( &grb_info_ptr->videobuf_queue_grb );	// The call to videobuf_mmap_free() will ensure that all buffers have been unmapped.
	if( err )														// If so, they will all be passed to the buf_release() callback. If buffers remain mapped, videobuf_mmap_free() returns an error code instead.
		dev_err( grb_info_ptr->dev, "videobuf_mmap_free failed,err=%d\n", err );
	return 0;
}

static int device_mmap( struct file *file_ptr, struct vm_area_struct *vma ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	int ret = videobuf_mmap_mapper( &grb_info_ptr->videobuf_queue_grb, vma );
	GRB_DBG_PRINT( "device_mmap return: vm_start=%lx,vm_end=%lx\n", vma->vm_start, vma->vm_end )
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
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	char bus_info[32];
	
	strlcpy( v4l2_cap_ptr->driver, RCM_GRB_DRIVER_NAME, sizeof(v4l2_cap_ptr->driver) );
	strlcpy( v4l2_cap_ptr->card, RCM_GRB_DEVICE_NAME, sizeof(v4l2_cap_ptr->card) );
	snprintf( bus_info, sizeof(bus_info), "APB: 0x%x", (u32)virt_to_phys(grb_info_ptr->base_addr_regs_grb) );
	strlcpy( v4l2_cap_ptr->bus_info, bus_info, sizeof(v4l2_cap_ptr->bus_info) ) ;
	v4l2_cap_ptr->version = RCM_GRB_DRIVER_VERSION;
	v4l2_cap_ptr->device_caps = grb_info_ptr->video_dev.device_caps;
	v4l2_cap_ptr->capabilities = v4l2_cap_ptr->device_caps | V4L2_CAP_DEVICE_CAPS;
	v4l2_cap_ptr->reserved[0] = v4l2_cap_ptr->reserved[1] = v4l2_cap_ptr->reserved[2] = 0;
	GRB_DBG_PRINT( "vidioc_querycap_grb return: device_caps=%x,capabilities=%x\n", v4l2_cap_ptr->device_caps, v4l2_cap_ptr->capabilities )
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

static int vidioc_fmt( struct grb_info *grb_info_ptr, struct v4l2_format *f ) {
	if( f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ) {
		f->fmt.pix.width = grb_info_ptr->recognize_format.width;
		f->fmt.pix.height = grb_info_ptr->user_format.height = grb_info_ptr->recognize_format.height;
		f->fmt.pix.field = grb_info_ptr->recognize_format.field;
		print_four_cc( "vidioc_fmt entry: ", f->fmt.pix.pixelformat );
		switch( f->fmt.pix.pixelformat  ) {
		default:
			f->fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; //  V4L2_PIX_FMT_YUV420 (‘YU12')
			f->fmt.pix.bytesperline = grb_info_ptr->user_format.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage = grb_info_ptr->user_format.sizeimage = f->fmt.pix.bytesperline*f->fmt.pix.height*3;
			break;
		case V4L2_PIX_FMT_RGB32:
			f->fmt.pix.bytesperline = grb_info_ptr->user_format.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage = grb_info_ptr->user_format.sizeimage = f->fmt.pix.bytesperline*f->fmt.pix.height*4;
			break;
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUV444M:
			f->fmt.pix.bytesperline = grb_info_ptr->user_format.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage = grb_info_ptr->user_format.sizeimage = f->fmt.pix.bytesperline*f->fmt.pix.height*3;
			break;
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_YUV422M:
			f->fmt.pix.bytesperline = grb_info_ptr->user_format.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage = grb_info_ptr->user_format.sizeimage = f->fmt.pix.bytesperline*f->fmt.pix.height*2;
			break;
		case V4L2_PIX_FMT_YUV420: // It's for check only
			f->fmt.pix.bytesperline = grb_info_ptr->user_format.bytesperline = f->fmt.pix.width;
			f->fmt.pix.sizeimage = grb_info_ptr->user_format.sizeimage = f->fmt.pix.bytesperline*f->fmt.pix.height*2;
			break;
		}
		f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;
		f->fmt.pix.priv = 0;
		f->fmt.pix.flags = 0;
		f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
		f->fmt.pix.xfer_func = 0;
		print_four_cc( "vidioc_fmt return: ", f->fmt.pix.pixelformat );
		return 0;
	}
	else
		return -EINVAL;
}

static int vidioc_enum_fmt_vid_out_grb( struct file *file, void *fh, struct v4l2_fmtdesc *fmt ) {
	int index = fmt->index;
	PI
	switch( index ) {
	case 0: // V4L2_PIX_FMT_RGB32
		fmt->flags = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		strlcpy( fmt->description, "ARGB8888", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'R','G','B', '4' );
		return 0;
	case 1: // V4L2_PIX_FMT_NV16,format with ½ horizontal chroma resolution, also known as YUV 4:2:2. One luminance and one chrominance plane.
		fmt->flags = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'N','V','1', '6' );
		return 0;
	case 2: // V4L2_PIX_FMT_YUV422M The three components are separated into three sub-images or planes. Cb,Cr two pixel.
		fmt->flags = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		strlcpy( fmt->description, "YUV 4:2:2", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'Y','M','1', '6' );
		return 0;
	case 3: // V4L2_PIX_FMT_YUV444M Planar formats with full horizontal resolution, also known as YUV and YVU 4:4:4
		fmt->flags = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		strlcpy( fmt->description, "YUV 4:4:4", sizeof(fmt->description) );
		fmt->pixelformat = v4l2_fourcc( 'Y','M','2', '4' );
		return 0;
	default:
		return -EINVAL;
	}
}

static int vidioc_g_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	print_v4l2_format( "Vidioc_g_fmt_vid_cap_grb entry", (int)grb_info_ptr->phys_addr_regs_grb, f );
	ret = vidioc_fmt( grb_info_ptr, f );
	print_v4l2_format( "Vidioc_g_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );	
	return ret;
}

static int vidioc_try_fmt_vid_cap_grb( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	print_v4l2_format( "Vidioc_try_fmt_vid_cap_grb entry", (int)grb_info_ptr->phys_addr_regs_grb, f );
	ret = vidioc_fmt( grb_info_ptr, f ) || set_output_format( grb_info_ptr, f );
	print_v4l2_format( "Vidioc_try_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );
	return ret;
}

static int vidioc_s_fmt_vid_cap_grb ( struct file *file_ptr, void *fh, struct v4l2_format *f ) {
	int ret;
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	print_v4l2_format( "Vidioc_s_fmt_vid_cap_grb entry", (int)grb_info_ptr->phys_addr_regs_grb, f );
	ret = vidioc_fmt( grb_info_ptr, f ) || set_output_format( grb_info_ptr, f );	//negotiate the format of data (typically image format) exchanged between driver and application
	print_v4l2_format( "Vidioc_s_fmt_vid_cap_grb return", (int)grb_info_ptr->phys_addr_regs_grb, f );
	return ret;
}

static int vidioc_reqbufs_grb ( struct file *file_ptr, void *fh, struct v4l2_requestbuffers *req ) {
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	unsigned long flags;
	int ret;
	PI
	spin_lock_irqsave( &grb_info_ptr->irq_lock, flags );
	INIT_LIST_HEAD( &grb_info_ptr->buffer_queue );
	spin_unlock_irqrestore( &grb_info_ptr->irq_lock, flags );
	ret = videobuf_reqbufs( &grb_info_ptr->videobuf_queue_grb, req );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_querybuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_querybuf(&grb_info_ptr->videobuf_queue_grb, buf);
	print_v4l2_buffer( "vidioc_querybuf_grb", buf );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_qbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int ret = videobuf_qbuf(&grb_info_ptr->videobuf_queue_grb, buf);
	print_v4l2_buffer( "vidioc_qbuf_grb", buf );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_dqbuf_grb( struct file *file_ptr, void *fh, struct v4l2_buffer *buf ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	struct videobuf_queue* videobuf_queue = &grb_info_ptr->videobuf_queue_grb;
	int ret = videobuf_dqbuf( videobuf_queue, buf, file_ptr->f_flags & O_NONBLOCK );
	print_v4l2_buffer( "vidioc_dqbuf_grb", buf );
	//print_videobuf_queue_param( &grb_info_ptr->videobuf_queue_grb, 4 );
	return ret;
}

static int vidioc_streamon_grb( struct file *file_ptr, void *fh, enum v4l2_buf_type type ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	PI
	if( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	retval = setup_registers( grb_info_ptr );
	if( retval < 0 ) {
		GRB_DBG_PRINT( "set_register failed \n")
		return retval;
	}
	return videobuf_streamon( &grb_info_ptr->videobuf_queue_grb );
}

static int vidioc_streamoff_grb( struct file *file_ptr, void *__fh, enum v4l2_buf_type type ) {
	struct grb_info *grb_info_ptr = video_drvdata(file_ptr);
	int retval;
	PI
	if ( type != V4L2_BUF_TYPE_VIDEO_CAPTURE )
		return -EINVAL;

	retval = videobuf_streamoff( &grb_info_ptr->videobuf_queue_grb );
	reset_grab( grb_info_ptr->base_addr_regs_grb );
	return retval;
}
/*
static void print_gamma_cs( struct grb_gamma* g ) {
	int i;
	unsigned int cs = 0;
	for( i=0; i<256; i++ )
		cs += (g->table_C_R[i]<<16) + (g->table_Y_G[i]<<8) + (g->table_C_B[i]<<0);
	GRB_DBG_PRINT( "Gamma table checksum=%08x", cs )
}
*/
static void drv_set_gamma( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_gamma* gam = (struct grb_gamma*)arg;
	grb_info_ptr->gam = *gam;
	//print_gamma_cs( &grb_info_ptr->gam );
}

static void drv_vidioc_g_params( struct grb_info *grb_info_ptr, void *arg ) {
	struct grb_parameters* param = (struct grb_parameters*)arg;
	PI
	param = &grb_info_ptr->param;
}

static int drv_vidioc_s_params( struct grb_info *grb_info_ptr, void *arg ) { // VIDIOC_S_PARAMS
	struct grb_parameters* param = (struct grb_parameters*)arg;
	PI
	grb_info_ptr->param = *param;
	return set_input_format( grb_info_ptr, param );
}

static int drv_vidioc_auto_detect( struct grb_info *grb_info_ptr, void *arg )
{ // todo mutex
	struct v4l2_pix_format* recognize_format = (struct v4l2_pix_format*)arg;

	if( reset_grab( grb_info_ptr->base_addr_regs_grb ) )
		return -EIO;

	init_completion( &grb_info_ptr->cmpl );
	set_register( INT_BIT_DONE, grb_info_ptr->base_addr_regs_grb, ADDR_INT_STATUS );
	set_register( INT_BIT_DONE, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK );
	write_register( 1 , grb_info_ptr->base_addr_regs_grb, ADDR_REC_ENABLE );
	//print_registers( grb_info_ptr->base_addr_regs_grb, 0x100, 0x108 );

	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, (HZ/10) ) == 0 ) {
		GRB_DBG_PRINT( "Vidioc autodetection: timeout\n" )
		return -ETIMEDOUT;
	}

	if( recognize_format )
		*recognize_format = grb_info_ptr->recognize_format;
	return 0;
}

static long vidioc_default_grb( struct file *file_ptr, void *fh, bool valid_prio, unsigned int cmd, void *arg )
{
	struct grb_info *grb_info_ptr = video_drvdata( file_ptr );
	GRB_DBG_PRINT( "Vidioc ioctl default: cmd=%08x\n", cmd )
	switch (cmd) {
	case VIDIOC_SET_GAMMA:
		drv_set_gamma( grb_info_ptr, arg );
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
// 0-target=2,flag=0,rect=0,0,0,0; 1-target=1,flag=2,rect=0,0,0x500, 0x2d0
static int vidioc_g_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	int ret = 0;
	struct grb_info *grb_info_ptr = video_drvdata(file);

	print_v4l2_selection( "Vidioc_g_selection_grb entry", (int)grb_info_ptr->phys_addr_regs_grb, s );

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ret = -EINVAL;
		goto exit;
	}

	if( s->target == V4L2_SEL_TGT_CROP ) {				// 0: Crop rectangle. Defines the cropped area.
		s->r = grb_info_ptr->cropping;
	}
	else if( s->target == V4L2_SEL_TGT_CROP_BOUNDS ) {	// 2: Bounds of the crop rectangle.
		s->r.left = 0;									// All valid crop rectangles fit inside the crop bounds rectangle.
		s->r.top = 0;
		s->r.width = grb_info_ptr->recognize_format.width;
		s->r.height = grb_info_ptr->recognize_format.height;
	}
	else if( s->target == V4L2_SEL_TGT_CROP_DEFAULT ) {	// 1:Suggested cropping rectangle that covers the “whole picture”.
		s->r.left = 0;									// This includes only active pixels and excludes other non-active pixels such as black pixels.
		s->r.top = 0;
		s->r.width = grb_info_ptr->recognize_format.width;
		s->r.height = grb_info_ptr->recognize_format.height;
	}
	else
		ret = -EINVAL;
exit:
	if( !ret ) memset( s->reserved, 0, sizeof(s->reserved) );
	print_v4l2_selection( "Vidioc_g_selection_grb return", (int)ret, s );
	return ret;
}

static int vidioc_s_selection_grb( struct file *file, void *fh, struct v4l2_selection *s ) {
	struct grb_info *grb_info_ptr = video_drvdata(file);
	PI
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
	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_vid_out_grb,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap_grb,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap_grb,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap_grb,
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

static struct videobuf_buffer* grb_next_buffer( struct list_head* buffer_queue ) { // from interrupt context,because without spinlock
	struct videobuf_buffer* vb = NULL;

	if( list_empty( buffer_queue ) )
		goto out;

	vb = list_entry( buffer_queue->next, struct videobuf_buffer, queue );
	list_del( &vb->queue );
	vb->state = VIDEOBUF_ACTIVE;
out:
	return vb;
}

static irqreturn_t proc_interrupt (struct grb_info *grb_info_ptr) {
	int rd_data;
	void __iomem *base_addr;
	struct videobuf_buffer *vb;
	u32 base_addr0_dma, base_addr1_dma, base_addr2_dma;
	int switch_page;

	base_addr = grb_info_ptr->base_addr_regs_grb;
	rd_data = read_register( base_addr, ADDR_INTERRUPTION );
	if( rd_data != 1 )
		return IRQ_NONE;

	rd_data = read_register( base_addr, ADDR_INT_STATUS );

	if( rd_data & INT_BIT_TEST ) {
		GRB_DBG_PRINT( "irq_handler: test detected \n" )

		write_register( 0 , base_addr, ADDR_TEST_INT );
		set_register( INT_BIT_TEST, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_TEST, base_addr, ADDR_INT_MASK );
		complete_all( &grb_info_ptr->cmpl );
	}
	else if( rd_data & INT_BIT_END_WRITE ) {
		GRB_DBG_PRINT( "irq_handler: end write detected\n" )

		set_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_STATUS );

		if( ( vb = grb_next_buffer( &grb_info_ptr->buffer_queue ) ) == NULL ) {
			GRB_DBG_PRINT( "irq_handler: next buffer 0\n" )
			clr_register( INT_BIT_END_WRITE, base_addr, ADDR_INT_MASK );
			write_register( 0, base_addr, ADDR_ENABLE );
			return IRQ_HANDLED;
		}

		vb->state = VIDEOBUF_DONE;
		wake_up( &vb->done );
		switch_page = grb_info_ptr->frame_count & 1;

		grb_info_ptr->frame_count = grb_info_ptr->frame_count + 1;

		// GRB_DBG_PRINT( "irq_handler: frame_count = %d; switch_page = %d\n", grb_info_ptr->frame_count , switch_page )
		print_videobuf_queue_param2( &grb_info_ptr->videobuf_queue_grb, grb_info_ptr->frame_count-1, 0, grb_info_ptr->mem_offset1/*-4*/, grb_info_ptr->mem_offset2/*-4*/ );
		//{ struct videobuf_dma_contig_memory* mem = vb->priv;
		//printk( "vb=%08x\n", (u32)mem->vaddr );
		//print_videobuf_queue_param(  &grb_info_ptr->videobuf_queue_grb, 8 ); }

		if( grb_info_ptr->next_buf_num < grb_info_ptr->reqv_buf_cnt ) { // prepare next buffer,if it avalaible
			base_addr0_dma = videobuf_to_dma_contig_rcm( grb_info_ptr->videobuf_queue_grb.bufs[grb_info_ptr->next_buf_num] );
			base_addr1_dma = base_addr0_dma + grb_info_ptr->mem_offset1;
			base_addr2_dma = base_addr0_dma + grb_info_ptr->mem_offset2;

			if( switch_page == 0 ) {	// even
				write_register( base_addr0_dma, base_addr, ADDR_DMA0_ADDR0 );
				write_register( base_addr1_dma, base_addr, ADDR_DMA1_ADDR0 );
				write_register( base_addr2_dma, base_addr, ADDR_DMA2_ADDR0 );
			}
			else {	// odd
				write_register( base_addr0_dma, base_addr, ADDR_DMA0_ADDR1 );
				write_register( base_addr1_dma, base_addr, ADDR_DMA1_ADDR1 );
				write_register( base_addr2_dma, base_addr, ADDR_DMA2_ADDR1 );
			}
			grb_info_ptr->next_buf_num = grb_info_ptr->next_buf_num + 1;
		}
	}
	else if( rd_data & INT_BIT_DONE ) {
		GRB_DBG_PRINT( "irq_handler: scan detected\n" )

		set_register( INT_BIT_DONE, base_addr, ADDR_INT_STATUS );
		clr_register( INT_BIT_DONE, base_addr, ADDR_INT_MASK );
		rd_data = read_register( base_addr, ADDR_FRAME_SIZE );
		grb_info_ptr->recognize_format.height  = (rd_data >> 16) & 0xFFF;	// 27:16-height
		grb_info_ptr->recognize_format.width = rd_data & 0xFFF;				// 11:0-width
		rd_data = read_register( base_addr, ADDR_FRAME_PARAM );
		grb_info_ptr->recognize_format.pixelformat = V4L2_PIX_FMT_NV16;		// set format default?
		grb_info_ptr->recognize_format.field = rd_data & 0x10 ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;

/*
		grb_info_ptr->recognize_format.bytesperline = grb_info_ptr->recognize_format.width * 2; // bytes per pixel;
		grb_info_ptr->recognize_format.sizeimage = grb_info_ptr->recognize_format.bytesperline * grb_info_ptr->recognize_format.height;
		grb_info_ptr->recognize_format.colorspace = V4L2_COLORSPACE_DEFAULT;
		grb_info_ptr->recognize_format.priv = 0;
		grb_info_ptr->recognize_format.flags = 0;
		grb_info_ptr->recognize_format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		grb_info_ptr->recognize_format.quantization = V4L2_QUANTIZATION_DEFAULT;
		grb_info_ptr->recognize_format.xfer_func = V4L2_XFER_FUNC_DEFAULT;
		*/

		complete_all( &grb_info_ptr->cmpl );
	}
	else {																	// we do not must be here
		GRB_DBG_PRINT( "irq_handler: unhandled status %08x\n", rd_data )
		write_register( rd_data, base_addr, ADDR_INT_STATUS );				// let it be now
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
		//GRB_DBG_PRINT( "ERROR : can't get the device node of the memory-region") 
		return -ENODEV;
	}
	if( ( ret = of_address_to_resource( np, 0, res_ptr ) ) != 0 ) {
		//GRB_DBG_PRINT( "ERROR : can't get the resource of the memory area")
		return ret;
	}
	return 0;
}

static int test_interrupt( struct grb_info *grb_info_ptr ) {	// test interrupt started,handler must be calling 
	init_completion( &grb_info_ptr->cmpl );
	set_register( INT_BIT_TEST, grb_info_ptr->base_addr_regs_grb, ADDR_INT_MASK );
	write_register( 1, grb_info_ptr->base_addr_regs_grb, ADDR_TEST_INT );
	if( wait_for_completion_timeout( &grb_info_ptr->cmpl, HZ ) == 0 ) {
		GRB_DBG_PRINT( "Test interrupt: timeout\n" )
		return -ETIMEDOUT;
	}
	return 0;
}

static int get_resources( struct platform_device *grb_device, struct grb_info *grb_info_ptr ) {
	struct device* dev = &grb_device->dev;
	struct resource* grb_res_ptr;

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_MEM, 0 );
	if( !grb_res_ptr ) {
		dev_err( dev, "can't get the base address registers grabber" );
		return -EINVAL;
	}
	grb_info_ptr->phys_addr_regs_grb = grb_res_ptr->start; // it is for information only

	grb_info_ptr->base_addr_regs_grb = devm_ioremap_resource( &grb_device->dev, grb_res_ptr );
	if( IS_ERR( grb_info_ptr->base_addr_regs_grb ) ) {
		dev_err( dev, "can't ioremap grabber resource");
		return PTR_ERR( grb_info_ptr->base_addr_regs_grb );
	}

	if( read_register( grb_info_ptr->base_addr_regs_grb, ADDR_ID_REG) != RCM_GRB_DEVID ) {
		dev_err( dev, "invalid identificator, device not found! \n" );
		return -ENODEV;
	}

	if( get_memory_buffer_address( grb_device, "memory-region", grb_res_ptr ) ) {
		dev_err( dev, "can't get the base address of the memory area" );
		return -EINVAL;
	}

	grb_info_ptr->buff_phys_addr = grb_res_ptr->start;
	grb_info_ptr->buff_length = grb_res_ptr->end - grb_res_ptr->start + 1;
	grb_info_ptr->buff_dma_addr = (dma_addr_t)(grb_info_ptr->buff_phys_addr - (grb_device->dev.dma_pfn_offset << PAGE_SHIFT));

	grb_res_ptr = platform_get_resource( grb_device, IORESOURCE_IRQ, 0 );
	if( !grb_res_ptr ) {
		dev_err( dev,  "can't get base irq" );
		return -EINVAL;;
	}
	grb_info_ptr->num_irq = grb_res_ptr->start;
	return 0;
}

static int device_probe( struct platform_device *grb_device ) {
	struct grb_info *grb_info_ptr;
	int err;

	grb_info_ptr = kzalloc(sizeof(struct grb_info), GFP_KERNEL);
	if( grb_info_ptr == NULL ) {
		dev_err( &grb_device->dev, "Can't allocated memory!\n" );
		return -ENOMEM;
	}
	grb_info_ptr->dev = &grb_device->dev;

	err = get_resources (grb_device , grb_info_ptr);
	if( err )
		goto err_free_mem;

	grb_device->dma_mask = DMA_BIT_MASK(32);
	grb_device->dev.archdata.dma_offset = 0;								// (grb_device->dev.dma_pfn_offset << PAGE_SHIFT);

	err = dma_declare_coherent_memory( &grb_device->dev,					// dev->dma_mem was filled
									   grb_info_ptr->buff_phys_addr,
									   grb_info_ptr->buff_phys_addr,		// grb_info_ptr->buff_dma_addr
									   grb_info_ptr->buff_length );			// DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE
	if( err ) {
		dev_err( grb_info_ptr->dev, "declare coherent memory %d\n" , err );
		goto err_free_mem;
	}
	grb_info_ptr->kern_virt_addr = phys_to_virt( grb_info_ptr->buff_phys_addr );
	GRB_DBG_PRINT( "Declare coherent memory: for phys addr %llx, dma addr %llx, kern virt addr %x, size %x\n",
			 	   grb_info_ptr->buff_phys_addr, grb_info_ptr->buff_dma_addr, (u32)grb_info_ptr->kern_virt_addr, grb_info_ptr->buff_length )

	platform_set_drvdata( grb_device, grb_info_ptr ); 

	grb_info_ptr->video_dev.dev_parent = grb_info_ptr->dev;
	grb_info_ptr->video_dev.fops = &fops;
	grb_info_ptr->video_dev.ioctl_ops = &grb_ioctl_ops;
	strlcpy( grb_info_ptr->video_dev.name, RCM_GRB_DRIVER_NAME, sizeof(grb_info_ptr->video_dev.name) );
	grb_info_ptr->video_dev.release = video_dev_release;
	//grb_info_ptr->video_dev.vfl_dir	= VFL_DIR_M2M;
	//grb_info_ptr->video_dev.tvnorms = V4L2_STD_ATSC_8_VSB + V4L2_STD_ATSC_16_VSB;

	grb_info_ptr->in_f.format_din = 0x04;	// it'default value

	err = v4l2_device_register( grb_info_ptr->dev, &grb_info_ptr->v4l2_device );
	if (err) {
		dev_err( grb_info_ptr->dev, "failed v4l2_device register %d\n", err );
		goto err_free_mem;
	}
	grb_info_ptr->video_dev.v4l2_dev = &grb_info_ptr->v4l2_device;
	grb_info_ptr->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	video_set_drvdata( &grb_info_ptr->video_dev , grb_info_ptr );
	err = video_register_device( &grb_info_ptr->video_dev, VFL_TYPE_GRABBER, -1 );
	if( err ) {
		dev_err( grb_info_ptr->dev, "failed video_dev register %d\n", err );
		goto err_release_dev;
	}

	err = request_irq( grb_info_ptr->num_irq, irq_handler, IRQF_SHARED, RCM_GRB_DEVICE_NAME, grb_info_ptr );
	if( err ) {
		dev_err( grb_info_ptr->dev, "request_irq %u isn't availiable\n", grb_info_ptr->num_irq );
		goto err_release_dev;
	}

	err = test_interrupt( grb_info_ptr );
	if( err ) {
		dev_err( grb_info_ptr->dev, "test interrupt failed\n" );
		goto err_release_dev;
	}

	spin_lock_init( &grb_info_ptr->irq_lock );

	dev_info( grb_info_ptr->dev, "probe succesfully completed (base %08x)\n", (u32)grb_info_ptr->phys_addr_regs_grb );
	return 0;

err_release_dev:
	video_unregister_device( &grb_info_ptr->video_dev ); 
err_free_mem:
	kfree( grb_info_ptr );
	return err;
}

static int device_remove( struct platform_device* grb_device )
{
	struct grb_info* grb_info_ptr;

	GRB_DBG_PRINT( "In grabber_remove function : %p\n", grb_device )

	grb_info_ptr = platform_get_drvdata(grb_device);
	reset_grab( grb_info_ptr->base_addr_regs_grb );

	video_unregister_device( &grb_info_ptr->video_dev );

	if( grb_info_ptr->num_irq )
		free_irq( grb_info_ptr->num_irq, grb_info_ptr );

	if(  grb_info_ptr )
		kfree( grb_info_ptr );

	return 0;
}

static struct of_device_id grb_match[] = 
{
	{ .compatible = "rcm,vdu-grabber" },
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

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Vladimir Shalyt <Vladimir.Shalyt@astrosoft.ru>");
MODULE_DESCRIPTION("RCM SoC video capture device driver");
MODULE_DEVICE_TABLE(of, grb_match);
