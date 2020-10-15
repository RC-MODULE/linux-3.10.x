#ifndef __LINUX_MSVDHD_H__
#define __LINUX_MSVDHD_H__

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif


#define MSVD_IO_MAGIC		'm'

#define MSVD_ENABLE_IRQ		_IO(MSVD_IO_MAGIC, 0)

#ifndef __KERNEL__
static inline int msvd_enable_irq(int fd)
{
	return ioctl(fd, MSVD_ENABLE_IRQ, 0);
}
#endif

#define MSVD_DISABLE_IRQ	_IO(MSVD_IO_MAGIC, 1)

#ifndef __KERNEL__
static inline int msvd_disable_irq(int fd)
{
	return ioctl(fd, MSVD_DISABLE_IRQ, 0);
}
#endif

struct msvd_wait_irq_params {
	unsigned int status;
};

#define MSVD_WAIT_IRQ		_IOR(MSVD_IO_MAGIC, 2, struct msvd_wait_irq_params)

#ifndef __KERNEL__
static inline int msvd_wait_for_irq(int fd, unsigned int *status)
{
	struct msvd_wait_irq_params params;
	int ret;

	ret = ioctl(fd, MSVD_WAIT_IRQ, &params);
	if (!ret)
		*status = params.status;
	return ret;
}
#endif

struct msvd_get_buf_phys_params {
	unsigned long addr;
};

#define MSVD_GET_BUF_PHYS	_IOR(MSVD_IO_MAGIC, 3, struct msvd_get_buf_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_buf_phys(int fd, unsigned long *addr)
{
	struct msvd_get_buf_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_BUF_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif

struct msvd_get_extmem_phys_params {
	unsigned long addr;
};

#define MSVD_GET_EXTMEM_PHYS	_IOR(MSVD_IO_MAGIC, 4, struct msvd_get_extmem_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_extmem_phys(int fd, unsigned long *addr)
{
	struct msvd_get_extmem_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_EXTMEM_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif

struct msvd_get_intmem_phys_params {
	unsigned long addr;
};

#define MSVD_GET_INTMEM_PHYS	_IOR(MSVD_IO_MAGIC, 5, struct msvd_get_intmem_phys_params)

#ifndef __KERNEL__
static inline int msvd_get_intmem_phys(int fd, unsigned long *addr)
{
	struct msvd_get_intmem_phys_params params;
	int ret;

	ret = ioctl(fd, MSVD_GET_INTMEM_PHYS, &params);
	if (!ret)
		*addr = params.addr;
	return ret;
}
#endif

struct msvd_check_buf {
	void* buf;
	size_t size;
};
#define MSVD_CHECK_BUFFER _IOW(MSVD_IO_MAGIC, 6, struct msvd_check_buf)

struct msvd_check_buffers {
	int n;
	struct msvd_check_buf buffers[32];	
};
#define MSVD_CHECK_BUFFERS _IOW(MSVD_IO_MAGIC, 7, struct msvd_check_buffers)


#define MSVD_MAP_REGS_OFFSET		0
#define MSVD_MAP_REGS_SIZE		0x20000

#define MSVD_MAP_BUF_OFFSET		0x100000
#define MSVD_MAP_BUF_SIZE		(4 * 1024 * 1024)

#define MSVD_MAP_EXTMEM_OFFSET		0x1000000
#define MSVD_MAP_EXTMEM_SIZE		(8 * 1024 * 1024)

#define MSVD_MAP_INTMEM_OFFSET		0x10000000
#define MSVD_MAP_INTMEM_SIZE		(256 * 1024)

#ifndef __KERNEL__
static inline void *msvd_map_registers(int fd)
{
	return mmap(0, MSVD_MAP_REGS_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_REGS_OFFSET);
}
static inline int msvd_unmap_registers(void *regs)
{
	return munmap(regs, MSVD_MAP_REGS_SIZE);
}

static inline void *msvd_map_buffer(int fd)
{
	return mmap(0, MSVD_MAP_BUF_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_BUF_OFFSET);
}
static inline int msvd_unmap_buffer(void *buf)
{
	return munmap(buf, MSVD_MAP_BUF_SIZE);
}

static inline void *msvd_map_extmem(int fd)
{
	return mmap(0, MSVD_MAP_EXTMEM_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_EXTMEM_OFFSET);
}
static inline int msvd_unmap_extmem(void *extmem)
{
	return munmap(extmem, MSVD_MAP_EXTMEM_SIZE);
}

static inline void *msvd_map_intmem(int fd)
{
	return mmap(0, MSVD_MAP_INTMEM_SIZE, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, MSVD_MAP_INTMEM_OFFSET);
}
static inline int msvd_unmap_intmem(void *intmem)
{
	return munmap(intmem, MSVD_MAP_INTMEM_SIZE);
}
#endif

struct msvd_buffer_geometry {
	uint32_t width;
	uint32_t height;
	
	uint32_t luma_offset;
	uint32_t chroma_offset;
};

enum msvd_picture_type {
	msvd_picture_type_top = 1,
	msvd_picture_type_bot = 2,
	msvd_picture_type_frame = 3
};

enum msvd_coding_type {
	msvd_coding_type_D = 0, 
	msvd_coding_type_I = 1, 
	msvd_coding_type_P = 2, 
	msvd_coding_type_B = 3
};

struct msvd_decode_result {
	int 		error_code;
	size_t 	num_of_decoded_mbs;
};

struct msvd_h264_scaling_lists {
	uint8_t _4x4[6][16];
	uint8_t _8x8[2][64];
};

struct msvd_h264_frame {
	uint32_t 	phys_addr;
	int16_t		poc_top;
	int16_t 	poc_bot;
};

struct msvd_h264_picture_reference {
	enum msvd_picture_type pt;
	size_t 						index;
	bool 							long_term;
	struct {
    int8_t weight;
    int8_t offset;
  } luma, cb, cr;
};

struct msvd_h264_decode_params {
 	struct msvd_buffer_geometry geometry;

  uint32_t hor_pic_size_in_mbs;
  uint32_t vert_pic_size_in_mbs;
	uint32_t max_num_ref_frames;
  uint32_t mb_mode;
  bool     frame_mbs_only_flag;
  bool     mbaff_frame_flag;
  bool     constr_intra_pred_flag;    
  bool     direct_8x8_inference_flag;
  bool     transform_8x8_mode_flag;
  bool     entropy_coding_mode_flag;
  uint32_t weight_mode;  
  int32_t  chroma_qp_index_offset;
  int32_t  second_chroma_qp_index_offset;

  struct msvd_h264_scaling_lists const* scaling_list;

 	enum msvd_picture_type picture_type;
  enum msvd_coding_type slice_type;

  uint32_t first_mb_in_slice;
  uint32_t cabac_init_idc;
  uint32_t disable_deblocking_filter_idc;
  uint32_t slice_qpy; 
  bool     direct_spatial_mv_pred_flag;
  uint32_t luma_log2_weight_denom;
  uint32_t chroma_log2_weight_denom;

	struct msvd_h264_frame curr_pic;

	struct msvd_h264_frame const* decoded_picture_buffer;
	size_t decoded_picture_buffer_size;

	struct {
		struct msvd_h264_picture_reference const* data;
		size_t size;
	} reflist[2];

  enum msvd_picture_type col_pic_type;
  bool      col_abs_diff_poc_flag; //  topAbsDiffPOC >= bottomAbsDiffPoc

	struct iovec const* slice_data;
	size_t slice_data_n;
	size_t slice_data_offset; // bitoffset of slice_data begining
};

#define MSVD_DECODE_H264_SLICE _IOW(MSVD_IO_MAGIC, 8, struct msvd_h264_decode_params)

struct msvd_mpeg_quantiser_matrix {
	uint8_t data[64];
};

struct msvd_mpeg_decode_params {
 	struct msvd_buffer_geometry geometry;

	uint32_t	hor_pic_size_in_mbs;
	uint32_t	ver_pic_size_in_mbs;

	const struct msvd_mpeg_quantiser_matrix* intra_quantiser_matrix;
	const struct msvd_mpeg_quantiser_matrix* non_intra_quantiser_matrix;

	bool 			full_pel_forward_vector;
	bool			full_pel_backward_vector;

	enum msvd_coding_type picture_coding_type;
	
	bool 			mpeg2;
	struct {
		uint8_t forward_f_code;
		uint8_t backward_f_code;		
	};
	uint8_t f_code[2][2];

	enum msvd_picture_type picture_structure;
	bool			top_field_first;
	bool			frame_pred_frame_dct;
	bool 			concealment_motion_vectors;
	bool 			q_scale_type;
	bool			intra_vlc_format;
	uint8_t		intra_dc_precision;
	bool			alternate_scan;
	bool			progressive_frame;

	uint32_t	curr_pic;
	uint32_t	refpic1;
	uint32_t	refpic2;

	struct iovec const* slice_data;
	size_t slice_data_n;
};

#define MSVD_DECODE_MPEG_FRAME _IOW(MSVD_IO_MAGIC, 9, struct msvd_mpeg_decode_params)

#endif
