#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/uio.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/mod_devicetable.h>

#include <linux/msvdhd.h>

#define DRIVER_NAME "msvdhd"

//#undef dev_dbg
//#define dev_dbg(device, format...) printk(KERN_DEBUG format);

enum status {
  mvde_cmd_rdy = 1 << 0,
  mvde_pic_rdy = 1 << 1,
  mvde_last_mb_rdy = 1 << 2,
  mvde_mvde_proc_delay = 1 << 3,
  mvde_proc_err = 1 << 4,
  sr1_cmd_rdy = 1 << 16,
  sr1_si_access_failed =  1 << 17,
  sr1_start_code_found = 1 << 18,
  sr1_pes_hd_found = 1 << 19,
  sr1_ts_extracted = 1 << 20,
  sr1_str_buf_changed =  1 << 21,
  sr1_str_buf_thresh = 1 << 22,
  sr1_str_buf_empty = 1 << 23,
  sr1_read_fail = 1 << 24,
  sr1_offs_end_missed = 1 << 25,
  sr1_offs_end_reached = 1 << 26,
  si_cmd_rdy = 1 << 27,
  si_start_dec = 1 << 28,
  si_parse_err = 1 << 29,
  si_data_err = 1 << 30
};

enum si_cmd {
  si_cmd_nop = 0,
  si_cmd_reset = 1,
  si_cmd_suspend_bus_transfers = 2,
  si_cmd_decode_until_return_point = 3,
  si_cmd_conceal_num_macroblocks = 4,
  si_cmd_remove_num_macroblocks = 5,
  si_cmd_stop_decoding = 6,
  si_cmd_release_bus_transfers = 7
};

enum mvde_cmd {
  mvde_cmd_nop = 0,
  mvde_cmd_reset = 1,
  mvde_cmd_suspend_bus_transfers = 2,
  mvde_cmd_start_continuous_decoding = 3,
  mvde_cmd_start_macroblock_decoding = 4,
  mvde_cmd_abort = 5,
  mvde_cmd_start_decode_until_return_point = 6,
  mvde_cmd_release_bus_transfers = 7
};

enum sr_cmd {
	sr_cmd_nop = 0,
	sr_cmd_reset = 1,
	sr_cmd_suspend_bus_transfers = 2,
	sr_cmd_byte_align = 3,
	sr_cmd_search_pattern = 4,
	sr_cmd_continue = 5,
	sr_cmd_load_buffer = 6,
	sr_cmd_search_start_code = 7,
	sr_cmd_release_bus_transfers = 8
};

//bit fields definitions

// ref_pic_info_tbl
#define ref_pic_id_to_ref_idx_l0 	(0,5)
#define ref_pic_id_l1 						(5,5)
#define frame_idx_l1 							(10,5)
#define ref_structure_l1 					(15,2)
#define long_term_ref_l1 					(17,1)
#define ref_pic_id_l0 						(18,5)
#define frame_idx_l0 							(23,5)
#define ref_structure_l0 					(28,2)
#define long_term_ref_l0 					(30,1)

// CT_HDR
#define ct_hdr_ctxt_type					(0,2)
#define ct_hdr_load_table_sel 		(2,8)
#define ct_hdr_table_scan 				(10,5)
#define ct_hdr_last_mb_in_picture (15,1)
#define ct_hdr_cp_mc_ip_en 				(24,1)
#define ct_hdr_lf_en 							(25,1)
#define ct_hdr_os_en 							(26,1)
#define ct_hdr_cp_mc_ip_last_mb 	(27,1)
#define ct_hdr_lf_last_mb 				(28,1)
#define ct_hdr_os_last_mb 				(29,1)
#define ct_hdr_cache_reset 				(30,1)
#define ct_hdr_always_1 					(31,1)

//ct_info
//word0
#define ct_info_structure (0,2)
#define ct_info_coding_type (2,2)
#define ct_info_vert_pic_size (4,8)
#define ct_info_hor_pic_size (12,8)
#define ct_info_mb_mode (20,3)
#define ct_info_mbaff_frame_flag (23,1)
#define ct_info_constr_intra_pred_flag (25,1)
#define ct_info_ipred_mode (26,1)

//word1
#define ct_info_pred_mode (0,2)
#define ct_info_trans_mode (2,2)
#define ct_info_iq_wt_en (4,1)
#define ct_info_iq_scale_en (5,1)
#define ct_info_iq_sat_en (6,1)
#define ct_info_iq_nadj_en (7,1)
#define ct_info_iq_div3 (8,3)
#define ct_info_iq_div_ctrl (11,1)
#define ct_info_def_dc_ctrl (12,2)
#define ct_info_iq_table_mask (14,4)
#define ct_info_mc_mode (18,3)
#define ct_info_scan_mode (22,6)

//word2
#define ct_info_weight_mode (0,2)

//word6
#define ct_info_lf_mode (13,3)
#define ct_info_os_mode (16,3)
#define ct_info_hor_fbuf_size (19,8)

//h264 si_info
//word0
#define h264_si_info_disable_deblocking_filter_idc (0,2)
#define h264_si_info_frame_mbs_only_flag (5,1)
#define h264_si_info_slice_qpy (8,6)
#define h264_si_info_num_ref_idx_l0_minus1 (14,5)
#define h264_si_info_num_ref_idx_l1_minus1 (19,5)
#define h264_si_info_direct_8x8_inference_flag (27,1)
#define h264_si_info_direct_spatial_mv_pred_flag (28,1)
#define h264_si_info_transform_8x8_flag (29,1)
#define h264_si_info_mb_adaptive_frame_field_flag (30,1)
#define h264_si_info_h4_new_slice (31,1)
//word1
#define h264_si_info_first_mb_in_slice (0,14)
#define h264_si_info_cabac_init_idc (14,2)
#define h264_si_info_entropy_coding_mode (16,1)
//word2
#define h264_si_info_chroma_qp_index_offset (0,5)
#define h264_si_info_second_chroma_qp_index_offset (5,5)
#define h264_si_info_luma_log2_weight_denom (10,3)
#define h264_si_info_chroma_log2_weight_denom (13,3)
#define h264_si_info_col_pic_mode (16,2)
#define h264_si_info_col_abs_diff_poc_flag (18,1)
//word3
#define h264_si_info_curr_bot_order_cnt (0,16)
#define h264_si_info_curr_top_order_cnt (16,16)

//mpeg2_si_info
//word0
#define mpeg_si_info_decoding_mode (2,3)
#define mpeg_si_info_progressive (5,1)
#define mpeg_si_info_mb_qp_luma (8,6)
#define mpeg_si_info_f_code00 (14,4)
#define mpeg_si_info_f_code01 (18,4)
#define mpeg_si_info_f_code10 (22,4)
#define mpeg_si_info_f_code11 (26,4)
#define mpeg_si_info_concealment_vectors (30,1)
#define mpeg_si_info_frame_pred_frame_dct (31,1)
//word1
#define mpeg_si_info_intra_vlc_format (16,1)
//word2
#define mpeg_si_info_full_pel_forward_flag (5,1)
#define mpeg_si_info_full_pel_backward_flag (6,1)
#define mpeg_si_info_q_scale_type (16,1)
#define mpeg_si_info_top_field_first (17,1)

#define dma_list_ad (0,14)
#define dma_list_en	(31,1)

#define vdmv_stat_ver_pos (24, 8)
#define vdmv_stat_hor_pos (16, 8)

#define PAIR_FIRST(x,y) x
#define PAIR_SECOND(x, y) y

#define set(field, value) ((((uint32_t)(value)) & ((1 << PAIR_SECOND field) - 1)) << (PAIR_FIRST field))
#define get(value, field) ((((uint32_t)(value)) >> (PAIR_FIRST field)) & ((1 << PAIR_SECOND field) - 1))

enum {
	CT_INFO = 1,
  CT_WT = 2,
  CT_BA = 3
};

enum load_table_sel {
	w_intra_8x8 = 1,
  w_inter_8x8 = 2,
  si_8x8_intra_y = 1,
  si_8x8_inter_y = 2,
  si_4x4_intra_y = 4,
  si_4x4_inter_y = 8,
  si_4x4_intra_cb = 16,
  si_4x4_inter_cb = 32,
  si_4x4_intra_cr = 64,
  si_4x4_inter_cr = 128
};

enum address_sel {
  frame_store_y_ba0 = 0,
  frame_store_c_ba0 = 1,
  frame_store_y_ba1 = 2,
  frame_store_c_ba1 = 3,
  frame_store_y_ba31 = frame_store_y_ba0 + 31*2,
  frame_store_c_ba31 = frame_store_y_ba0 + 31*2,
  cp_buf_ba = 0x40,
  ip_buf_ba = 0x41,
  os_buf_ba = 0x42,
  lf_buf_ba = 0x43,
  os_frame_store_y_ba = 0x44,
  os_frame_store_c_ba = 0x45,
  lf_frame_store_y_ba = 0x46,
  lf_frame_store_c_ba = 0x47
};

struct weighted_pred_tables {
  struct {
    uint32_t luma[16];
    uint32_t chroma[32];
  } list[2];

  struct {
    uint32_t luma;
    uint32_t chroma;
  } flags[2];
};

struct msvd_regs {
  uint32_t vd_clc; // Clock control (rwhh) MSVD_BASE + 0x0000
  uint32_t vd_id; // Revision identification (r) MSVD_BASE + 0x0004
  uint32_t vd_imsc; // Interrupt mask (rw) MSVD_BASE + 0x0008
  uint32_t vd_ris; // Raw interrupt status (r) MSVD_BASE + 0x000c
  uint32_t vd_mis; // Masked interrupt status (r) MSVD_BASE + 0x0010
  uint32_t vd_icr; // Interrupt status clear (w) MSVD_BASE + 0x0014
  uint32_t vd_isr; // Interrupt status set (w) MSVD_BASE + 0x0018
	uint32_t dummy0 [(0x8000-0x001c)/4]; // gap in address space
	uint32_t vdmv_cmd; // Command register of MVDE (w) MSVD_BASE + 0x8000
  uint32_t vdmv_stat; // Processing status of MVDE (r) MSVD_BASE + 0x8004
  uint32_t vdmv_ma_stat; // Status of memory access units of MVDE (r) MSVD_BASE + 0x8008
  uint32_t vdmv_cp_error; // MVDE error status (r) MSVD_BASE + 0x800c
  uint32_t vdmv_param_0; // Context header parameters (general and CP related) (rwhh) MSVD_BASE + 0x8010
  uint32_t vdmv_param_1; // Picture / Slice parameters (general and IP related) (rwhh) MSVD_BASE + 0x8014
  uint32_t vdmv_param_2; // Picture / Slice parameters (for CP and MC) (rwhh) MSVD_BASE + 0x8018
  uint32_t vdmv_param_3; // Picture / Slice parameters (for MC) (rwhh) MSVD_BASE + 0x801c
  uint32_t vdmv_param_4; // Picture / Slice parameters (for MC) (rwhh) MSVD_BASE + 0x8020
  uint32_t vdmv_param_5; // Picture / Slice parameters (for MC) (rwhh) MSVD_BASE + 0x8024
  uint32_t vdmv_param_6; // Picture / Slice parameters (for MC) (rwhh) MSVD_BASE + 0x8028
  uint32_t vdmv_param_7; // Picture / Slice parameters (for OS, LF and MC_MA) (rwhh) MSVD_BASE + 0x802c
  uint32_t vdmv_delay_thresh; // Threshold for delay detection (rw) MSVD_BASE + 0x8030
  uint32_t vdmv_ncyc_avg_mb; // Average number of cycles per macroblock (rw) MSVD_BASE + 0x8034
  uint32_t vdmv_delay_stat; // Processing delay status (r) MSVD_BASE + 0x8038
  uint32_t vdmv_ncyc1_pic; // Total cycles per picture (r) MSVD_BASE + 0x803c
  uint32_t vdmv_ncyc2_pic; // Effective cycles per picture (r) MSVD_BASE + 0x8040
  uint32_t vdmv_ncyc_max_mb; // Maximum number of cycles per macroblock (r) MSVD_BASE + 0x8044
  uint32_t vdmv_pos_max_mb; // Position of macroblock with maximum number of cycles (r) MSVD_BASE + 0x8048
  uint32_t vdmv_cp_buf_ba; // CP coefficient buffer base address (rwhh) MSVD_BASE + 0x804c
  uint32_t vdmv_ip_buf_ba; // IP line buffer base address (rwhh) MSVD_BASE + 0x8050
  uint32_t vdmv_os_buf_ba; // OS line buffer base address (rwhh) MSVD_BASE + 0x8054
  uint32_t vdmv_os_frame_store_y_ba; // OS luminance frame store base address (rwhh) MSVD_BASE + 0x8058
  uint32_t vdmv_os_frame_store_c_ba; // OS chrominance frame store base address (rwhh) MSVD_BASE + 0x805c
  uint32_t vdmv_lf_buf_ba; // LF line buffer base address (rwhh) MSVD_BASE + 0x8060
  uint32_t vdmv_lf_frame_store_y_ba; // LF luminance frame store base address (rwhh) MSVD_BASE + 0x8064
  uint32_t vdmv_lf_frame_store_c_ba; // LF chrominance frame store base address (rwhh) MSVD_BASE + 0x8068
  uint32_t _notused_1 [(0x8100-0x806c)/4]; // gap in address space
  uint32_t vdmv_frame_store_y_ba[32]; // base address of luminance frame store for MC (rwhh) 0x8100+n (n=0..31)
  uint32_t vdmv_frame_store_c_ba[32]; // base address of chrominance frame store for MC (rwhh) 0x8180+n (n=0..31)
  uint32_t vdmv_cp_data[128]; // CP RAM (rwhh) 0x8200+n (n=0..127)
  uint32_t _notused_2 [(0x10000-0x8400)/4]; // gap in address space
  uint32_t vdsr1_cmd; // Command register of SR1 (w) MSVD_BASE + 0x10000
  uint32_t vdsr1_stat; // Processing status of SR1 (r) MSVD_BASE + 0x10004
  uint32_t vdsr1_pes_hdr_data; // PES header data (r) MSVD_BASE + 0x10008
  uint32_t vdsr1_pts_l; // Presentation time stamp (r) MSVD_BASE + 0x1000c
  uint32_t vdsr1_dts_l; // Decoding time stamp (r) MSVD_BASE + 0x10010
  uint32_t vdsr1_search_range; // Search range for search commands (rw) MSVD_BASE + 0x10014
  uint32_t vdsr1_search_pattern_1; // Search pattern no. 1 (rw) MSVD_BASE + 0x10018
  uint32_t vdsr1_search_mask_1; // Search mask no. 1 (rw) MSVD_BASE + 0x1001c
  uint32_t vdsr1_cfg; // Configuration of SR1 (rw) MSVD_BASE + 0x10020
  uint32_t vdsr1_str_buf_ba; // Stream buffer base address (rw) MSVD_BASE + 0x10024
  uint32_t vdsr1_str_buf_size; // Stream buffer size (rw) MSVD_BASE + 0x10028
  uint32_t vdsr1_str_buf_blen; // Stream buffer block length (rw) MSVD_BASE + 0x1002c
  uint32_t vdsr1_str_buf_thresh; // Stream buffer threshold (rw) MSVD_BASE + 0x10030
  uint32_t vdsr1_str_offs; // Stream offset (rwhh) MSVD_BASE + 0x10034
  uint32_t vdsr1_str_offs_end; // Stream offset end (rw) MSVD_BASE + 0x10038
  uint32_t vdsr1_str_buf_rdptr; // Stream buffer read pointer (r) MSVD_BASE + 0x1003c
  uint32_t vdsr1_search_pattern_2; // Search pattern no. 2 (rw) MSVD_BASE + 0x10040
  uint32_t vdsr1_search_mask_2; // Search mask no. 2 (rw) MSVD_BASE + 0x10044
  uint32_t vdsr1_parse_stat; // Parse status of SR1 (rwhh) MSVD_BASE + 0x10048
  uint32_t _notused_3 [(0x10100-0x1004c)/4]; // gap in address space
  uint32_t vdsr1_get_bits[32]; // Stream data bits (get_bits) (r) 0x10100+n (n=0..31)
  uint32_t vdsr1_show_bits[32]; // Stream data bits (show_bits) (r) 0x10180+n (n=0..31)
  uint32_t vdsr1_flush_show[32]; // Stream data bits (flush_show) (r) 0x10200+n (n=0..31)
  uint32_t vdsr1_show_aligned; // Stream data from next byte boundary (r) MSVD_BASE + 0x10280
  uint32_t vdsr1_show_aligned_em; // Stream data with emulation bytes from next byte boundary (r) MSVD_BASE + 0x10284
  uint32_t _notused_4 [(0x10400-0x10288)/4]; // gap in address space
  uint32_t vdsr2_cmd; // Command register of SR2 (w) MSVD_BASE + 0x10400
  uint32_t vdsr2_stat; // Processing status of SR2 (r) MSVD_BASE + 0x10404
  uint32_t vdsr2_pes_hdr_data; // PES header data (r) MSVD_BASE + 0x10408
  uint32_t vdsr2_pts_l; // Presentation time stamp (r) MSVD_BASE + 0x1040c
  uint32_t vdsr2_dts_l; // Decoding time stamp (r) MSVD_BASE + 0x10410
  uint32_t vdsr2_search_range; // Search range for search commands (rw) MSVD_BASE + 0x10414
  uint32_t vdsr2_search_pattern_1; // Search pattern no. 1 (rw) MSVD_BASE + 0x10418
  uint32_t vdsr2_search_mask_1; // Search mask no. 1 (rw) MSVD_BASE + 0x1041c
  uint32_t vdsr2_cfg; // Configuration of SR2 (rw) MSVD_BASE + 0x10420
  uint32_t vdsr2_str_buf_ba; // Stream buffer base address (rw) MSVD_BASE + 0x10424
  uint32_t vdsr2_str_buf_size; // Stream buffer size (rw) MSVD_BASE + 0x10428
  uint32_t vdsr2_str_buf_blen; // Stream buffer block length (rw) MSVD_BASE + 0x1042c
  uint32_t vdsr2_str_buf_thresh; // Stream buffer threshold (rw) MSVD_BASE + 0x10430
  uint32_t vdsr2_str_offs; // Stream offset (rwhh) MSVD_BASE + 0x10434
  uint32_t vdsr2_str_offs_end; // Stream offset end (rw) MSVD_BASE + 0x10438
  uint32_t vdsr2_str_buf_rdptr; // Stream buffer read pointer (r) MSVD_BASE + 0x1043c
  uint32_t vdsr2_search_pattern_2; // Search pattern no. 2 (rw) MSVD_BASE + 0x10440
  uint32_t vdsr2_search_mask_2; // Search mask no. 2 (rw) MSVD_BASE + 0x10444
  uint32_t vdsr2_parse_stat; // Parse status of SR2 (rwhh) MSVD_BASE + 0x10448
  uint32_t _notused_5 [(0x10500-0x1044c)/4]; // gap in address space
  uint32_t vdsr2_get_bits[32]; // Stream data bits (get_bits) (r) 0x10500+n (n=0..31)
  uint32_t vdsr2_show_bits[32]; // Stream data bits (show_bits) (r) 0x10580+n (n=0..31)
  uint32_t vdsr2_flush_show[32]; // Stream data bits (flush_show) (r) 0x10600+n (n=0..31)
  uint32_t vdsr2_show_aligned; // Stream data from next byte boundary (r) MSVD_BASE + 0x10680
  uint32_t vdsr2_show_aligned_em; // Stream data with emulation bytes from next byte boundary (r) MSVD_BASE + 0x10684
  uint32_t _notused_6 [(0x10800-0x10688)/4]; // gap in address space
  uint32_t vdsi_cmd; // Command register of SI (w) MSVD_BASE + 0x10800
  uint32_t vdsi_stat; // Status register of SI (r) MSVD_BASE + 0x10804
  uint32_t vdsi_error_stat; // SI error status (r) MSVD_BASE + 0x10808
  uint32_t vdsi_cfg; // Configuration of SI (rw) MSVD_BASE + 0x1080c
  uint32_t vdsi_dma_list0_ad; // DMA list 0 start address (rw) MSVD_BASE + 0x10810
  uint32_t vdsi_dma_list1_ad; // DMA list 1 start address (rw) MSVD_BASE + 0x10814
  uint32_t vdsi_dma_list2_ad; // DMA list 2 start address (rw) MSVD_BASE + 0x10818
  uint32_t vdsi_conceal_cfg; // Concealment configuration (rw) MSVD_BASE + 0x1081c
  uint32_t vdsi_conceal_cnt; // Concealment count (r) MSVD_BASE + 0x10820
  uint32_t vdsi_mod_stat; // Status of SI modules (r) MSVD_BASE + 0x10824
  uint32_t vdsi_param_1; // Sequence / Picture parameters (for SI) (rw) MSVD_BASE + 0x10828
  uint32_t vdsi_param_2; // Sequence / Picture parameters (for SI) (rw) MSVD_BASE + 0x1082c
  uint32_t vdsi_param_3; // Sequence / Picture parameters (for SI) (rw) MSVD_BASE + 0x10830
  uint32_t vdsi_param_4; // Sequence / Picture parameters (for SI) (rw) MSVD_BASE + 0x10834
  uint32_t vdsi_param_5; // Sequence / Picture parameters (for SI) (rw) MSVD_BASE + 0x10838
  uint32_t vdsi_param_6; // General parameters (for SI and MVDE) (rw) MSVD_BASE + 0x1083c
  uint32_t vdsi_param_7; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x10840
  uint32_t vdsi_param_8; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x10844
  uint32_t vdsi_param_9; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x10848
  uint32_t vdsi_param_10; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x1084c
  uint32_t vdsi_param_11; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x10850
  uint32_t vdsi_param_12; // Picture / Slice parameters (for MVDE) (rw) MSVD_BASE + 0x10854
  uint32_t vdsi_pe_nsuma_ba; // PE_NSUMA buffer base address (rwhh) MSVD_BASE + 0x10858
  uint32_t vdsi_bpma_ba; // BPMA buffer base address (rwhh) MSVD_BASE + 0x1085c
  uint32_t vdsi_su_nsuma_ba; // SU_NSUMA buffer base address (rwhh) MSVD_BASE + 0x10860
  uint32_t vdsi_psuma_rd_top_ba; // PSUMA read buffer top field base address (rwhh) MSVD_BASE + 0x10864
  uint32_t vdsi_psuma_rd_bot_ba; // PSUMA read buffer bottom field base address (rwhh) MSVD_BASE + 0x10868
  uint32_t vdsi_psuma_wr_top_ba; // PSUMA write buffer top field base address (rwhh) MSVD_BASE + 0x1086c
  uint32_t vdsi_psuma_wr_bot_ba; // PSUMA write buffer bottom field base address (rwhh) MSVD_BASE + 0x10870
  uint32_t vdsi_fifo_error; // SI_FF FIFO error (rcw) MSVD_BASE + 0x10874
  uint32_t vdsi_fifo_fill_level; // Fill level of FIFOs (r) MSVD_BASE + 0x10878
  uint32_t vdsi_fifo_dma_stat; //
  uint32_t vdsi_fifo_clear; // Clear FIFOs (w) MSVD_BASE + 0x10880
  uint32_t vdsi_fifo_mb_ctxt; // Macroblock context FIFO data port (w) MSVD_BASE + 0x10884
  uint32_t vdsi_fifo_mv; // Motion vector FIFO data port (w) MSVD_BASE + 0x10888
  uint32_t vdsi_fifo_coeff; // Coefficient FIFO data port (w) MSVD_BASE + 0x1088c
  uint32_t vdsi_fifo_dma_cmd; // DMA command (w) MSVD_BASE + 0x10890
  uint32_t vdsi_fifo_dma_ad; // DMA address configuration (rw) MSVD_BASE + 0x10894
  uint32_t _notused_7 [(0x10900-0x10898)/4]; // gap in address space

 	uint32_t ref_pic_info_tbl[32];
  uint32_t cnt_tbl_l[2][32];
  struct weighted_pred_tables wt;

  uint32_t vdsi_rpi_data_rest[(0x11100-0x10900)/4 - 196]; // RPI RAM (rwhh) 0x10900+n (n=0..511)

  uint32_t vdsi_pe_nsuma_start_ad; // PE_NSUMA buffer start address (rwhh) MSVD_BASE + 0x11100
  uint32_t vdsi_pe_nsuma_end_ad; // PE_NSUMA buffer end address (rwhh) MSVD_BASE + 0x11104
  uint32_t vdsi_su_nsuma_start_ad; // SU_NSUMA buffer start address (rwhh) MSVD_BASE + 0x11108
  uint32_t vdsi_su_nsuma_end_ad; // SU_NSUMA buffer end address (rwhh) MSVD_BASE + 0x1110c
  uint32_t vdsi_bpma_start_ad; // BPMA buffer offset address (rwhh) MSVD_BASE + 0x11110
  uint32_t vdsi_bpma_start_pos; // BPMA buffer current position (rwhh) MSVD_BASE + 0x11114
  uint32_t _notused_8 [(0x11200-0x11118)/4]; // gap in address space
  uint32_t vdsi_su_cmd; // Command register of SI_SU (w) MSVD_BASE + 0x11200
  uint32_t vdsi_su_ctxt_data_0; // context data register 0 (rwhh) MSVD_BASE + 0x11204
  uint32_t vdsi_su_ctxt_data_1; // context data register 1 (rwhh) MSVD_BASE + 0x11208
  uint32_t _notused_9 [(0x11300-0x1120c)/4]; // gap in address space
  uint32_t vdsi_cabac_data[128]; // CABAC RAM (rwhh) 0x11300+n (n=0..127)
  uint32_t vdsi_pe_cmd; // Command register of SI_PE (w) MSVD_BASE + 0x11500
  uint32_t vdsi_pe_ctx1; // context data register 1 (rwhh) MSVD_BASE + 0x11504
  uint32_t vdsi_pe_ctx2; // context data register 2 (rwhh) MSVD_BASE + 0x11508
  uint32_t vdsi_pe_ctx3; // context data register 3 (rwhh) MSVD_BASE + 0x1150c
  uint32_t _notused_10 [(0x18000-0x11510)/4]; // gap in address space
  uint32_t vdsi_cfg_data[256]; // MSVD_HD setup data (rwhh) 0x18000+n (n=0..255)
};

struct continuation {
	void (*func)(struct continuation* this, uint32_t status);
};

typedef struct continuation continuation_t;


struct mem_range {
	uint32_t	begin;
	size_t 		length;
};

struct stream {
	continuation_t continuation;

	atomic_t refcount;

	uint32_t frame_ids[32];

	struct mem_range extmem;
	struct mem_range intmem;

	uint32_t mvde_ip_buffer;
	uint32_t mvde_lf_buffer;
	uint32_t pe_nsuma_buffer;
	uint32_t su_nsuma_buffer;
	uint32_t psuma_buffer;

	uint32_t hor_pic_size_in_mbs;

	spinlock_t lock;
	bool has_status;
	bool decoding;
	bool closing;
	struct msvd_decode_result status;
};

struct msvd_device {
	struct device *dev;
	struct miscdevice mdev;

	struct msvd_regs __iomem *regs;

	void *buf;
	unsigned long buf_phys;

	struct mem_range extmem;
	struct mem_range intmem;

	int irq;

	spinlock_t lock;
	continuation_t* continuation;

	bool busy;
	struct semaphore mx;
	wait_queue_head_t wq;
	struct timer_list watchdog;
} device;

static inline
struct mem_range alloc_extmem(size_t size, size_t alignment) { return device.extmem; }

static inline
struct mem_range alloc_intmem(size_t size, size_t alignment) { return device.intmem; }

static inline
void free_extmem(struct mem_range* range) { range->length = 0; }

static inline
void free_intmem(struct mem_range* range) { range->length = 0; }

static inline
void stream_addref(struct stream* stream) {
	dev_dbg(device.dev, "stream_addref: %p\n", stream);
	atomic_inc(&stream->refcount);
}

static inline
bool stream_release(struct stream* stream) {
	dev_dbg(device.dev, "stream_release: %p\n", stream);
	if(atomic_dec_return(&stream->refcount) == 0){
		dev_dbg(device.dev, "stream_free: %p\n", stream);
		free_extmem(&stream->extmem);
		free_intmem(&stream->intmem);

		kfree(stream);
		return true;
	}

	return false;
}

inline
struct stream* alloc_stream(void) {
	struct stream* stream = kmalloc(sizeof(struct stream), GFP_KERNEL);

	memset(stream,0, sizeof(*stream));
	spin_lock_init(&stream->lock);
	atomic_set(&stream->refcount, 0);

	stream_addref(stream);

	return stream;
}

static inline
void stream_set_status(struct stream* stream, struct msvd_decode_result status) {
	unsigned long flags;
	dev_dbg(device.dev, "stream_set_status %p", stream);
	spin_lock_irqsave(&stream->lock, flags);
	stream->status = status;
	stream->has_status = true;
	stream->decoding = false;
	spin_unlock_irqrestore(&stream->lock, flags);
}

static inline
bool stream_get_status(struct stream* stream, struct msvd_decode_result* status) {
	unsigned long flags;
	bool r = false;

	spin_lock_irqsave(&stream->lock, flags);

	if(stream->has_status) {
		r = true;
		*status = stream->status;
		stream->has_status = false;
		wake_up_interruptible(&device.wq);
	}

	spin_unlock_irqrestore(&stream->lock, flags);

	dev_dbg(device.dev, "stream_get_status -> %i", r);

	return r;
}

static inline
bool stream_has_status(struct stream* stream) {
	unsigned long flags;
	bool r = false;

	spin_lock_irqsave(&stream->lock, flags);

	r = stream->has_status;

  spin_unlock_irqrestore(&stream->lock, flags);

	return stream->has_status;
}

static inline
bool is_stream_readable(struct stream* stream) { return stream_has_status(stream); }

static inline
int stream_allocate_memory(struct stream* stream, size_t extmem_size, size_t extmem_alignment, size_t intmem_size, size_t intmem_alignement) {
	if((extmem_size && (stream->extmem.length >= extmem_size || stream->extmem.begin % extmem_alignment))
			|| stream->intmem.length >= intmem_size || stream->intmem.begin % intmem_alignement)
		return 0;

	free_extmem(&stream->extmem);
	free_intmem(&stream->intmem);

	stream->extmem = alloc_extmem(extmem_size, extmem_alignment);
	if(stream->extmem.length == 0) return -ENOMEM;

	stream->intmem = alloc_intmem(intmem_size, intmem_alignement);
	if(stream->intmem.length == 0) {
		free_extmem(&stream->extmem);
		return -ENOMEM;
	}


	return 0;
}

void set_continuation(continuation_t* c) {
	unsigned long flags;
	spin_lock_irqsave(&device.lock, flags);
	BUG_ON(device.continuation);
	device.continuation = c;
  spin_unlock_irqrestore(&device.lock, flags);
}

static inline
bool is_device_available(void) { return !device.busy; }

typedef uint8_t frame_id_t;
static const frame_id_t num_frame_ids = 32;
static const frame_id_t invalid_frame_id = 32;

static inline
frame_id_t find_frame_id(struct stream* stream, uint32_t frame) {
	frame_id_t i;
	for(i = 0; i != num_frame_ids; ++i)
		if(stream->frame_ids[i] == frame) break;

	dev_dbg(device.dev, "find_frame_id(%p,%x)->%d", stream, frame, i);
	return i;
}

int assign_frame_ids(struct stream* stream, struct msvd_h264_decode_params* p, uint8_t* curr_frame_id, uint8_t frame_ids[]) {
	size_t i;
	memset(frame_ids, 0, num_frame_ids);

	for(i = 0; i != 32; ++i) {
		dev_dbg(device.dev, "%p->frame_ids[%d] = %08x", stream, i, stream->frame_ids[i]);
	}

	*curr_frame_id = find_frame_id(stream, p->curr_pic.phys_addr);
	if(invalid_frame_id == *curr_frame_id)
		*curr_frame_id = find_frame_id(stream, 0);

	for(i = 0; i != p->decoded_picture_buffer_size; ++i) {
		if(p->decoded_picture_buffer[i].phys_addr != p->curr_pic.phys_addr) {
			frame_id_t id = find_frame_id(stream, p->decoded_picture_buffer[i].phys_addr);
			if(id == invalid_frame_id) return -EINVAL;
			frame_ids[i] = id;
		}
		else
			frame_ids[i] = *curr_frame_id;
	}

	memset(stream->frame_ids, 0, sizeof(stream->frame_ids));

	for(i = 0; i != p->decoded_picture_buffer_size; ++i)
		stream->frame_ids[frame_ids[i]] = p->decoded_picture_buffer[i].phys_addr;

	stream->frame_ids[*curr_frame_id] = p->curr_pic.phys_addr;

	return 0;
}

/* FixMe: kernel.h already has this macro
 * -   See: include/linux/kernel.h#L104
 *     -   -- Andrew
 *
-*/

#undef round_up
static inline
uint32_t round_up(uint32_t value, uint32_t round) {
	return value % round ? value + round - (value % round) : value;
}

static inline
uint32_t psuma_buffer_size(struct msvd_h264_decode_params* p) {
  return round_up(p->hor_pic_size_in_mbs*p->vert_pic_size_in_mbs*128u, 2*1024);
}

static inline
int assign_internal_buffers(struct stream* stream, struct msvd_h264_decode_params* p) {
	const uint32_t mvde_ip_buffer_size = p->hor_pic_size_in_mbs * 16 * 128;
	const size_t extmem_required = round_up(mvde_ip_buffer_size, 1024) + psuma_buffer_size(p) * (p->max_num_ref_frames + 1);
	const size_t intmem_required = round_up(2*16384, 16*1024) + round_up(2*16384, 16*1024) + p->hor_pic_size_in_mbs*256;
	int r;

	if((r = stream_allocate_memory(stream, extmem_required, 1024, intmem_required, 16*1024)) < 0) return r;

	stream->mvde_ip_buffer = round_up(stream->extmem.begin, 1024u);
	stream->psuma_buffer = round_up(stream->mvde_ip_buffer + mvde_ip_buffer_size, 1024);

	stream->pe_nsuma_buffer = round_up(stream->intmem.begin, 16*1024);
	stream->su_nsuma_buffer = round_up(stream->pe_nsuma_buffer + 2*16384, 16*1024);
	stream->mvde_lf_buffer  = round_up(stream->su_nsuma_buffer + 2*16384, 16*1024);

	dev_dbg(device.dev, "assign_internal_buffers(%p) -> {mvde_ip: %08x psuma: %08x, pe_nsuma: %08x, su_nsuma: %08x, mvde_lf: %08x}",
		stream, stream->mvde_ip_buffer, stream->psuma_buffer, stream->pe_nsuma_buffer, stream->su_nsuma_buffer, stream->mvde_lf_buffer);

	return 0;
}

static inline
uint32_t psuma_buffer_top(struct stream* stream, struct msvd_h264_decode_params* p, frame_id_t id) {
	return stream->psuma_buffer + id * psuma_buffer_size(p);
}

static inline
uint32_t psuma_buffer_bot(struct stream* stream, struct msvd_h264_decode_params* p, frame_id_t id) {
	return psuma_buffer_top(stream,p,id) + psuma_buffer_size(p)/2;
}

void reset(void) {
 	iowrite32(si_cmd_suspend_bus_transfers, &device.regs->vdsi_cmd);
	iowrite32(sr_cmd_suspend_bus_transfers, &device.regs->vdsr1_cmd);
  iowrite32(mvde_cmd_suspend_bus_transfers, &device.regs->vdmv_cmd);

	iowrite32(si_cmd_reset, &device.regs->vdsi_cmd);
	iowrite32(sr_cmd_reset, &device.regs->vdsr1_cmd);
	iowrite32(mvde_cmd_reset, &device.regs->vdmv_cmd);

	iowrite32(si_cmd_release_bus_transfers, &device.regs->vdsi_cmd);
	iowrite32(sr_cmd_release_bus_transfers, &device.regs->vdsr1_cmd);
	iowrite32(mvde_cmd_release_bus_transfers, &device.regs->vdmv_cmd);

	device.regs->vdmv_cmd = mvde_cmd_start_continuous_decoding;
}

static
bool acquire_device(void) {
	if(!down_trylock(&device.mx)) {
		dev_dbg(device.dev, "acquire_device: success\n");

		BUG_ON(device.busy);
		device.busy = true;
		return true;
	}

	dev_dbg(device.dev, "acquire_device: fail\n");

	return false;
}

static
void release_device(void) {
	dev_dbg(device.dev, "release_device");
	BUG_ON(!device.busy);
	device.busy = false;
	up(&device.mx);
	wake_up_interruptible(&device.wq);
}

static
void complete_decode(continuation_t* continuation, uint32_t irq_status) {
	struct stream* stream = (struct stream*)continuation;
	struct msvd_decode_result status = {-1,0};
	uint32_t mvde_status = ioread32(&device.regs->vdmv_stat);
	uint32_t vdsi_error_stat = ioread32(&device.regs->vdsi_error_stat);
	uint32_t vdsi_stat = ioread32(&device.regs->vdsi_stat);

	dev_dbg(device.dev, "complete_decode: stream=%p isr=%x vdmv_stat=%x vdsi_stat=%x vdsi_error_stat=%x\n", stream, irq_status, mvde_status, vdsi_stat, vdsi_error_stat);

	if(get(mvde_status, vdmv_stat_ver_pos) != 0)
		status.num_of_decoded_mbs = get(mvde_status, vdmv_stat_ver_pos) * stream->hor_pic_size_in_mbs + get(mvde_status, vdmv_stat_hor_pos) + 1;

	if((irq_status & (si_data_err | mvde_proc_err | mvde_mvde_proc_delay | si_parse_err | sr1_si_access_failed)) == 0)
		status.error_code = 0;

	stream_set_status(stream, status);
	stream_release(stream);

	//if(status.error_code)
		reset();
	release_device();
}

static void force_stop(void)
  {
	unsigned long flags;
  struct stream* stream;
  struct msvd_decode_result status = {-1,0};

  spin_lock_irqsave(&device.lock, flags);
  stream = (struct stream*)device.continuation;
  device.continuation = 0;
  spin_unlock_irqrestore(&device.lock, flags);

  if(stream) {
    iowrite32(0, &device.regs->vd_imsc);
    reset();

    stream_set_status(stream, status);
    release_device();
  }
}
/*
static
void decode_timeout(unsigned long not_used) {
	force_stop();
}
*/
static
void start_decode(struct stream* stream) {
 	unsigned long flags;
	stream->continuation.func = complete_decode;
	set_continuation(&stream->continuation);
	stream_addref(stream);

	//mod_timer(&device.watchdog, jiffies + msecs_to_jiffies(200));

  spin_lock_irqsave(&stream->lock, flags);

	if(!stream->closing) {
		stream->decoding = true;
		iowrite32(si_data_err | mvde_proc_err | mvde_last_mb_rdy | si_parse_err | sr1_si_access_failed, &device.regs->vd_imsc);
		iowrite32(si_cmd_decode_until_return_point, &device.regs->vdsi_cmd);
	}

  spin_unlock_irqrestore(&stream->lock, flags);
}

static
void config_sr1(int offset, long length, bool enable_sc_emulation_removal) {
	dev_dbg(device.dev, "config_sr1 0x%08lx %d %ld", device.buf_phys, offset, length);

	iowrite32(0x100000, 					&device.regs->vdsr1_search_range);
	iowrite32(0x00000100, 				&device.regs->vdsr1_search_pattern_1);
	iowrite32(device.buf_phys, 		&device.regs->vdsr1_str_buf_ba);
	iowrite32(MSVD_MAP_BUF_SIZE,	&device.regs->vdsr1_str_buf_blen);
	iowrite32(MSVD_MAP_BUF_SIZE,	&device.regs->vdsr1_str_buf_thresh);

	iowrite32(0x24 | (!!enable_sc_emulation_removal) << 1, &device.regs->vdsr1_cfg);
	iowrite32(offset,							&device.regs->vdsr1_str_offs);
	iowrite32(length * 8,					&device.regs->vdsr1_str_offs_end);

	iowrite32(sr_cmd_load_buffer,	&device.regs->vdsr1_cmd);
}

static inline
uint32_t ct_ba_element(uint32_t base, int sel, bool last) {
	return set((10, 22), base/(1 << 10)) | set((9, 1), last) | set((0, 7), sel);
}

static inline
volatile uint32_t* fill_ct_wt(uint8_t const* list, size_t n, volatile uint32_t* cfg) {
	size_t i;
	for(i = 0; i != n/2; ++i)
		*cfg++ = list[i*2] << 8 | list[i*2+1] | (i == ((n/2)-1)) << 30;
	return cfg;
}

static inline
volatile uint32_t* fill_ct_wt_default(size_t n,  volatile uint32_t* cfg) {
	size_t i;
  for(i = 0; i != n/2; ++i)
    *cfg++ = 0x1010 | (i == ((n/2)-1)) << 30;
  return cfg;
}

static inline
uint32_t volatile* dma_element(volatile uint32_t* begin, volatile uint32_t* end, uint32_t volatile* cfg, bool last) {
	*cfg++ = set((30,2), 0) | set((16,14), end-begin) | set((15,1), last) | set((2,12), begin - device.regs->vdsi_cfg_data);
  return cfg;
}

static inline
uint32_t volatile* dma_element_coeff(uint32_t volatile* begin, uint32_t volatile* end, uint32_t volatile* cfg, bool last) {
	*cfg++ = set((30,2), 2) | set((16,14), end-begin) | set((15,1), last) | set((2,12), begin - device.regs->vdsi_cfg_data);
  return cfg;
}

int configure_device_for_h264(struct stream* stream, struct msvd_h264_decode_params* p) {
	int err = -EINVAL;
	size_t i, l;
	frame_id_t frame_ids[32];
	frame_id_t curr_frame_id;
	uint32_t t;
	volatile uint32_t* cfg = device.regs->vdsi_cfg_data,
		*ct_ba[2], *ct_wt4[2], *ct_wt8[2], *ct_info[2], *dma[2];

	if((err = assign_internal_buffers(stream, p)) < 0) {
		dev_dbg(device.dev, "assign_internal_buffers: %p failed -> %x\n", stream, err);
		return err;
	}

	if((err = assign_frame_ids(stream, p, &curr_frame_id, frame_ids)) < 0) {
		dev_dbg(device.dev, "assign_frame_ids: %p -> %x\n", stream, err);
		return err;
	}

	ct_ba[0] = cfg;
	*cfg++ = set(ct_hdr_always_1, 1) | set(ct_hdr_ctxt_type, CT_BA);
	for(i = 0; i != p->decoded_picture_buffer_size; ++i) {
		*cfg++ = ct_ba_element(p->decoded_picture_buffer[i].phys_addr + p->geometry.luma_offset, frame_store_y_ba0 + i*2, false);
		*cfg++ = ct_ba_element(p->decoded_picture_buffer[i].phys_addr + p->geometry.chroma_offset, frame_store_c_ba0 + i*2, false);
	}

	*cfg++ = ct_ba_element(stream->mvde_ip_buffer, ip_buf_ba, false);
  *cfg++ = ct_ba_element(stream->mvde_lf_buffer, lf_buf_ba, false);
  *cfg++ = ct_ba_element(p->curr_pic.phys_addr + p->geometry.luma_offset, lf_frame_store_y_ba, false);
  *cfg++ = ct_ba_element(p->curr_pic.phys_addr + p->geometry.chroma_offset, lf_frame_store_c_ba, true);
	ct_ba[1] = cfg;

	ct_wt4[0] = cfg;
	*cfg++ = 0
    | set(ct_hdr_always_1, 1)
    | set(ct_hdr_table_scan, 17)
    | set(ct_hdr_load_table_sel, si_4x4_intra_y |si_4x4_inter_y | si_4x4_intra_cb |si_4x4_inter_cb | si_4x4_intra_cr |si_4x4_inter_cr)
    | set(ct_hdr_ctxt_type, CT_WT);

	if(p->scaling_list) {
		cfg = fill_ct_wt(p->scaling_list->_4x4[0], 16, cfg);
		cfg = fill_ct_wt(p->scaling_list->_4x4[3], 16, cfg);
		cfg = fill_ct_wt(p->scaling_list->_4x4[1], 16, cfg);
		cfg = fill_ct_wt(p->scaling_list->_4x4[4], 16, cfg);
		cfg = fill_ct_wt(p->scaling_list->_4x4[2], 16, cfg);
		cfg = fill_ct_wt(p->scaling_list->_4x4[5], 16, cfg);
	}
	else {
		for(i = 0; i != 6; ++i)
			cfg = fill_ct_wt_default(16, cfg);
	}

	ct_wt4[1] = cfg;

	ct_wt8[0] = cfg;
	*cfg++ = 0
    | set(ct_hdr_always_1, 1)
    | set(ct_hdr_table_scan, 16)
    | set(ct_hdr_load_table_sel, si_8x8_intra_y |si_8x8_inter_y)
    | set(ct_hdr_ctxt_type, CT_WT);

	if(p->scaling_list) {
		cfg = fill_ct_wt(p->scaling_list->_8x8[0], 64, cfg);
		cfg = fill_ct_wt(p->scaling_list->_8x8[1], 64, cfg);
	}
	else {
		cfg = fill_ct_wt_default(64, cfg);
		cfg = fill_ct_wt_default(64, cfg);
	}
	ct_wt8[1] = cfg;

	ct_info[0] = cfg;
	*cfg++ = set(ct_hdr_always_1, 1) | set(ct_hdr_ctxt_type, CT_INFO);

  t = *cfg++ = device.regs->vdsi_param_6 = 0
		| set(ct_info_structure,              p->picture_type)
    | set(ct_info_coding_type,            p->slice_type)
    | set(ct_info_vert_pic_size,         	p->vert_pic_size_in_mbs / (2 - (p->picture_type == msvd_picture_type_frame)))
    | set(ct_info_hor_pic_size,           p->hor_pic_size_in_mbs)
    | set(ct_info_mb_mode,                p->mb_mode)
    | set(ct_info_mbaff_frame_flag,       p->mbaff_frame_flag && p->picture_type == msvd_picture_type_frame)
    | set(ct_info_constr_intra_pred_flag, p->constr_intra_pred_flag)
    | set(ct_info_ipred_mode,             1);
	dev_dbg(device.dev, "vdsi_param_6: %08x\n", t);

	t = *cfg++ = device.regs->vdsi_param_7 = 0
   	| set(ct_info_trans_mode,             2)
    | set(ct_info_iq_wt_en,               1)
    | set(ct_info_iq_scale_en,            1)
    | set(ct_info_iq_nadj_en,             1)
    | set(ct_info_mc_mode,                3);
	dev_dbg(device.dev, "vdsi_param_7: %08x\n", t);

	t = *cfg++ = device.regs->vdsi_param_8 = 0
    | set(ct_info_weight_mode,            p->weight_mode);
	dev_dbg(device.dev, "vdsi_param_8: %08x\n", t);

  t = *cfg++ = device.regs->vdsi_param_9 = 0;
	dev_dbg(device.dev, "vdsi_param_9: %08x\n", t);
  t = *cfg++ = device.regs->vdsi_param_10 = 0;
	dev_dbg(device.dev, "vdsi_param_10: %08x\n", t);
	t = *cfg++ = device.regs->vdsi_param_11 = 0;
	dev_dbg(device.dev, "vdsi_param_11: %08x\n", t);
	t = *cfg++ = device.regs->vdsi_param_12 = 0
  	| set(ct_info_os_mode,        3)
    | set(ct_info_hor_fbuf_size,  p->geometry.width / 16);
	dev_dbg(device.dev, "vdsi_param_12: %08x\n", t);
	ct_info[1] = cfg;

	dma[0] = cfg;
  cfg = dma_element(ct_ba[0], ct_ba[1], cfg, false);
  cfg = dma_element(ct_wt4[0], ct_wt4[0]+1, cfg, false);
  cfg = dma_element_coeff(ct_wt4[0]+1, ct_wt4[1], cfg, false);
  cfg = dma_element(ct_wt8[0], ct_wt8[0]+1, cfg, false);
  cfg = dma_element_coeff(ct_wt8[0]+1, ct_wt8[1], cfg, false);
  cfg = dma_element(ct_info[0], ct_info[1], cfg, true);
  dma[1] = cfg;

	t = device.regs->vdsi_param_1 = 0
    | set(h264_si_info_disable_deblocking_filter_idc,  p->disable_deblocking_filter_idc)
    | set(h264_si_info_frame_mbs_only_flag,            p->frame_mbs_only_flag)
    | set(h264_si_info_slice_qpy,                      p->slice_qpy)
    | set(h264_si_info_num_ref_idx_l0_minus1,          p->reflist[0].size ? (p->reflist[0].size-1) : 0)
    | set(h264_si_info_num_ref_idx_l1_minus1,          p->reflist[1].size ? (p->reflist[1].size-1) : 0)
    | set(h264_si_info_direct_8x8_inference_flag,      p->direct_8x8_inference_flag)
    | set(h264_si_info_direct_spatial_mv_pred_flag,    p->direct_spatial_mv_pred_flag)
    | set(h264_si_info_transform_8x8_flag,             p->transform_8x8_mode_flag)
    | set(h264_si_info_mb_adaptive_frame_field_flag,   p->mbaff_frame_flag)
    | set(h264_si_info_h4_new_slice,                   1);
	dev_dbg(device.dev, "vdsi_param_1: %08x\n", t);

	t = device.regs->vdsi_param_2 = 0
    | set(h264_si_info_first_mb_in_slice,              p->first_mb_in_slice)
    | set(h264_si_info_cabac_init_idc,                 p->cabac_init_idc)
    | set(h264_si_info_entropy_coding_mode,            p->entropy_coding_mode_flag);
	dev_dbg(device.dev, "vdsi_param_2: %08x\n", t);

	t = device.regs->vdsi_param_3 = 0
    | set(h264_si_info_chroma_qp_index_offset,         p->chroma_qp_index_offset)
    | set(h264_si_info_second_chroma_qp_index_offset,  p->second_chroma_qp_index_offset)
    | set(h264_si_info_luma_log2_weight_denom,         p->luma_log2_weight_denom)
    | set(h264_si_info_chroma_log2_weight_denom,       p->chroma_log2_weight_denom)
    | set(h264_si_info_col_pic_mode,                   p->col_pic_type)
    | set(h264_si_info_col_abs_diff_poc_flag,          p->col_abs_diff_poc_flag);
	dev_dbg(device.dev, "vdsi_param_3: %08x\n", t);

  t = device.regs->vdsi_param_4 = 0
    | set(h264_si_info_curr_top_order_cnt,             p->curr_pic.poc_top)
    | set(h264_si_info_curr_bot_order_cnt,             p->curr_pic.poc_bot);
	dev_dbg(device.dev, "vdsi_param_4: %08x\n", t);

	t = device.regs->vdsi_param_5 = 0;
	dev_dbg(device.dev, "vdsi_param_5: %08x\n", t);

	device.regs->vdsi_pe_nsuma_ba = stream->pe_nsuma_buffer;
	device.regs->vdsi_su_nsuma_ba = stream->su_nsuma_buffer;

	if(p->slice_type == msvd_coding_type_B) {
    device.regs->vdsi_psuma_rd_top_ba = psuma_buffer_top(stream, p, frame_ids[p->reflist[1].data[0].index]);
    device.regs->vdsi_psuma_rd_bot_ba = psuma_buffer_bot(stream, p, frame_ids[p->reflist[1].data[0].index]);
  }
  else {
   	device.regs->vdsi_psuma_rd_top_ba = psuma_buffer_top(stream, p, curr_frame_id);
    device.regs->vdsi_psuma_rd_bot_ba = psuma_buffer_bot(stream, p, curr_frame_id);
  }
  device.regs->vdsi_psuma_wr_top_ba = psuma_buffer_top(stream, p, curr_frame_id);
  device.regs->vdsi_psuma_wr_bot_ba = psuma_buffer_bot(stream, p, curr_frame_id);

	// configuring rpi table
	for(i = 0; i != p->reflist[0].size; ++i) {
		uint32_t t = device.regs->ref_pic_info_tbl[i];
		device.regs->ref_pic_info_tbl[i] = (t & ~set((18, 13), -1))
			| set(ref_pic_id_l0, frame_ids[p->reflist[0].data[i].index] * 2 + (p->reflist[0].data[i].pt == msvd_picture_type_bot))
			| set(frame_idx_l0, p->reflist[0].data[i].index)
			| set(ref_structure_l0, p->reflist[0].data[i].pt)
			| set(long_term_ref_l0, p->reflist[0].data[i].long_term);
	}

	for(i = 0; i != p->reflist[1].size; ++i) {
		uint32_t t = device.regs->ref_pic_info_tbl[i];
		device.regs->ref_pic_info_tbl[i] = (t & ~set((5, 13), -1))
			| set(ref_pic_id_l1, frame_ids[p->reflist[1].data[i].index] * 2 + (p->reflist[1].data[i].pt == msvd_picture_type_bot))
			| set(frame_idx_l1, p->reflist[1].data[i].index)
			| set(ref_structure_l1, p->reflist[1].data[i].pt)
			| set(long_term_ref_l1, p->reflist[1].data[i].long_term);
	}

	for(i = 0; i != p->reflist[0].size; ++i) {
		frame_id_t f = frame_ids[p->reflist[0].data[i].index];
		if(p->picture_type != msvd_picture_type_frame) f = f * 2 + p->reflist[0].data[i].pt == msvd_picture_type_bot;

		device.regs->ref_pic_info_tbl[f] = (device.regs->ref_pic_info_tbl[f] & ~set((0,5),-1)) | set((0,5),i);
	}

	// configuring poc tables
	for(l = 0; l != 2; ++l) {
		for(i = 0; i != p->reflist[l].size; ++i) {
			struct msvd_h264_frame const* f = &p->decoded_picture_buffer[p->reflist[l].data[i].index];
			device.regs->cnt_tbl_l[l][i] = set((16,16), f->poc_top) | set((0,16), f->poc_bot);
		}
	}

	// configuring weighted prediction table
	for(l =0; l != 2; ++l) {
		uint32_t luma_flags = 0;
		uint32_t chroma_flags = 0;

		struct msvd_h264_picture_reference const* list = p->reflist[l].data;
		for(i = 0; i != p->reflist[l].size; ++i) {
			if(i%2 == 0)
				device.regs->wt.list[l].luma[i/2] = set((24,8), list[i].luma.weight) | set((16,8),list[i].luma.offset);
			else
				device.regs->wt.list[l].luma[i/2] |= set((8,8), list[i].luma.weight) | set((0,8), list[i].luma.offset);

			device.regs->wt.list[l].chroma[i] = set((24,8), list[i].cb.weight) | set((16,8), list[i].cb.offset) | set((8,8), list[i].cr.weight) | set((0,8),list[i].cr.offset);

			luma_flags 		|= (((int8_t)(1 << p->luma_log2_weight_denom)) != list[i].luma.weight || (0 != list[i].luma.offset)) << i;
			chroma_flags	|= (((int8_t)(1 << p->chroma_log2_weight_denom)) != list[i].cb.weight || (0 != list[i].cb.offset) ||
												((int8_t)(1 << p->chroma_log2_weight_denom)) != list[i].cr.weight || (0 != list[i].cr.offset)) << i;
		}
		device.regs->wt.flags[l].luma = luma_flags;
		device.regs->wt.flags[l].chroma = chroma_flags;
		dev_dbg(device.dev, "flags: %08x %08x", luma_flags, chroma_flags);
	}

	device.regs->vdsi_cfg = 0x80;
  device.regs->vdsi_conceal_cfg = 0x20000;

  device.regs->vdsi_dma_list0_ad = set(dma_list_en, 1) | set(dma_list_ad, sizeof(uint32_t)*(dma[0] - device.regs->vdsi_cfg_data));
  device.regs->vdsi_dma_list1_ad = 0;

	return err;
}

static irqreturn_t msvdhd_irq(int irq, void *dev_id) {
	unsigned int status;
	unsigned long flags;
	unsigned delay;
	continuation_t* continuation;

	status = ioread32(&device.regs->vd_mis);
	delay = ioread32(&device.regs->vdmv_delay_stat);

	iowrite32(status, &device.regs->vd_icr);
	iowrite32(0, &device.regs->vd_imsc);

	dev_dbg(device.dev, "interrrupt status %08X ris %08X vdmv_stat %08X vdsr1_stat %08x vdmv_ma_stat %08X vdmv_delay_stat %08X\n", status,
		ioread32(&device.regs->vd_ris), ioread32(&device.regs->vdmv_stat), ioread32(&device.regs->vdsr1_stat), ioread32(&device.regs->vdmv_ma_stat), delay);
	iowrite32(0, &device.regs->vd_ris);

	spin_lock_irqsave(&device.lock, flags);
	continuation = device.continuation;
	device.continuation = 0;
	spin_unlock_irqrestore(&device.lock, flags);

	if(continuation) continuation->func(continuation, status);

	return IRQ_HANDLED;
}

uint64_t* copy_swap(uint64_t const* first, uint64_t const* last, uint64_t* out) {
	while(first != last)
  	*out++ = __builtin_bswap64(*first++);
  return out;
}

int copy_swap_data(struct iovec const* data, size_t n) {
	static uint8_t swap_buffer[4*1024] __attribute__ ((aligned(8)));

	size_t i;
	size_t used = 0;
	int length = 0;
	uint64_t* out = device.buf;

	for(i = 0; i != n; ++i) {
		struct iovec current_buffer;
		if (0 != copy_from_user(&current_buffer, &data[i], sizeof(struct iovec))) return -EFAULT;
		if(!access_ok(/*VERIFY_READ,*/ current_buffer.iov_base, current_buffer.iov_len)) return -EFAULT; // The 5.0 kernel dropped the type argument to access_ok()

		while(current_buffer.iov_len) {
			size_t n = min(sizeof(swap_buffer) - used, current_buffer.iov_len);

			if(0 != __copy_from_user(swap_buffer + used, current_buffer.iov_base, n)) return -EFAULT;
			current_buffer.iov_len -= n;
			current_buffer.iov_base += n;
			used += n;
			length += n;

			if(used == sizeof(swap_buffer)) {
      	out = copy_swap((uint64_t*)swap_buffer, (uint64_t*)(swap_buffer + used), out);
        used = 0;
      }
		}
	}

	while(used % 8 != 0) swap_buffer[used++] = 0;

	out = copy_swap((uint64_t*)swap_buffer,(uint64_t*)(swap_buffer + used), out);
	swap_buffer[0] = 0;
	swap_buffer[1] = 0;
	swap_buffer[2] = 0;
	swap_buffer[3] = 1;

	out = copy_swap((uint64_t*)swap_buffer,(uint64_t*)(swap_buffer + 8), out);

	return (void*)out - device.buf;
}

struct msvd_h264_dpb {
	struct msvd_h264_frame frames[32];
};

struct msvd_h264_reflists {
	struct msvd_h264_picture_reference list[2][32];
};

static
int copy_h264_params(struct msvd_h264_decode_params __user* args,
	struct msvd_h264_decode_params* params,
	struct msvd_h264_scaling_lists* scaling_list,
	struct msvd_h264_dpb* dpb,
	struct msvd_h264_reflists* reflist)
{
	int err = 0;
	size_t i,k;

	if(copy_from_user(params, args, sizeof(*params))) return -EFAULT;

	if(params->scaling_list) {
		if(copy_from_user(scaling_list, params->scaling_list, sizeof(*scaling_list))) return -EFAULT;
		params->scaling_list = scaling_list;
	}

	if(params->decoded_picture_buffer_size > sizeof(dpb->frames) / sizeof(dpb->frames[0])) return -EINVAL;
	if(copy_from_user(dpb, params->decoded_picture_buffer, sizeof(dpb->frames[0]) * params->decoded_picture_buffer_size))
		return -EFAULT;
	params->decoded_picture_buffer = (struct msvd_h264_frame*)dpb;

	for(k = 0; k != 2; ++k) {
		if(copy_from_user(reflist->list[k], params->reflist[k].data, sizeof(reflist->list[0][0]) * params->reflist[k].size)) return -EFAULT;

		params->reflist[k].data = reflist->list[k];

		for(i = 0; i != params->reflist[k].size; ++i)
			if(params->reflist[k].data[i].index >= params->decoded_picture_buffer_size) return -EINVAL;
	}

	return err;
}

static int ioctl_start_decode_slice(struct inode* inode, struct file* file, struct msvd_h264_decode_params __user* args) {
	int err;

	//all of this accessed only after device acquisition
	static struct msvd_h264_decode_params params;
	static struct msvd_h264_scaling_lists scaling_list;
	static struct msvd_h264_dpb dpb;
	static struct msvd_h264_reflists reflists;

	struct stream* stream = file->private_data;

	if(is_stream_readable(stream) || !acquire_device()) return -EAGAIN;

	if((err = copy_h264_params(args, &params, &scaling_list, &dpb, &reflists)) < 0) goto release_and_return;

	dev_dbg(device.dev, "stream %p, params copied", stream);

	if((err = configure_device_for_h264(stream, &params)) < 0) goto release_and_return;

	if((err = copy_swap_data(params.slice_data, params.slice_data_n)) < 0) goto release_and_return;

	if(params.entropy_coding_mode_flag)
    params.slice_data_offset = round_up(params.slice_data_offset, 8);

	config_sr1(params.slice_data_offset, err, true);

	stream->hor_pic_size_in_mbs = params.hor_pic_size_in_mbs;

	start_decode(stream);

	return 0;
release_and_return:
	release_device();
	return err;
}

int assign_internal_buffers_for_mpeg(struct stream* stream, struct msvd_mpeg_decode_params* params) {
	int r;
	if((r = stream_allocate_memory(stream, 0, 0, 4 * params->hor_pic_size_in_mbs, 16*1024))) return r;
	stream->su_nsuma_buffer = stream->intmem.begin;
	return 0;
}

static const uint8_t mpeg_default_intra_quantiser_matrix[] = {
   8, 16, 19, 22, 26, 27, 29, 34,
  16, 16, 22, 24, 27, 29, 34, 37,
  19, 22, 26, 27, 29, 34, 34, 38,
  22, 22, 26, 27, 29, 34, 37, 40,
  22, 26, 27, 29, 32, 35, 40, 48,
  26, 27, 29, 32, 35, 40, 48, 58,
  26, 27, 29, 34, 38, 46, 56, 69,
  27, 29, 35, 38, 46, 56, 69, 83
};

static const uint8_t mpeg_default_non_intra_quantiser_matrix[] = {
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16,
  16, 16, 16, 16, 16, 16, 16, 16
};

int configure_device_for_mpeg(struct stream* stream, struct msvd_mpeg_decode_params* p) {
	uint32_t volatile* cfg = device.regs->vdsi_cfg_data, *ct_ba[2], *ct_wt[2], *ct_info[2], *dma[2];
 	int r;

	if((r = assign_internal_buffers_for_mpeg(stream, p)) < 0) return r;

	ct_ba[0] = cfg;
  *cfg++ = set(ct_hdr_always_1, 1) | set(ct_hdr_ctxt_type, CT_BA);

  if(p->refpic1) {
    if(p->refpic2) {
			*cfg++ = ct_ba_element(p->refpic1 + p->geometry.luma_offset, 	frame_store_y_ba1, false);
			*cfg++ = ct_ba_element(p->refpic1 + p->geometry.chroma_offset,frame_store_c_ba1, false);
      *cfg++ = ct_ba_element(p->refpic2 + p->geometry.luma_offset, 	frame_store_y_ba0, false);
      *cfg++ = ct_ba_element(p->refpic2 + p->geometry.chroma_offset,frame_store_c_ba0, false);
    }
		else {
    	*cfg++ = ct_ba_element(p->refpic1 + p->geometry.luma_offset, 	frame_store_y_ba0, false);
    	*cfg++ = ct_ba_element(p->refpic1 + p->geometry.chroma_offset,frame_store_c_ba0, false);
    }
 	}

  *cfg++ = ct_ba_element(p->curr_pic + p->geometry.luma_offset, lf_frame_store_y_ba, 	false);
  *cfg++ = ct_ba_element(p->curr_pic + p->geometry.chroma_offset, lf_frame_store_c_ba,true);

  ct_ba[1] = cfg;

	ct_wt[0] = cfg;

	*cfg++ = 0
    | set(ct_hdr_always_1, 1)
    | set(ct_hdr_table_scan, (p->mpeg2 && p->alternate_scan) ? 14 : 13)
    | set(ct_hdr_load_table_sel, w_intra_8x8 | w_inter_8x8)
    | set(ct_hdr_ctxt_type, CT_WT);

	cfg = fill_ct_wt(p->intra_quantiser_matrix ? p->intra_quantiser_matrix->data : mpeg_default_intra_quantiser_matrix, 64, cfg);
	cfg = fill_ct_wt(p->non_intra_quantiser_matrix ? p->non_intra_quantiser_matrix->data : mpeg_default_non_intra_quantiser_matrix, 64, cfg);
	ct_wt[1] = cfg;

	ct_info[0] = cfg;
  *cfg++ = set(ct_hdr_always_1, 1) | set(ct_hdr_ctxt_type, CT_INFO);
  device.regs->vdsi_param_6 = *cfg++ = 0
		| set(ct_info_structure,      p->mpeg2 ? p->picture_structure : 3)
    | set(ct_info_coding_type,    p->picture_coding_type)
    | set(ct_info_vert_pic_size,  p->ver_pic_size_in_mbs)
    | set(ct_info_hor_pic_size,   p->hor_pic_size_in_mbs);

  device.regs->vdsi_param_7 = *cfg++ = 0
    | set(ct_info_pred_mode,      1)
    | set(ct_info_iq_wt_en,       1)
    | set(ct_info_iq_sat_en,      1)
    | set(ct_info_iq_div3,        p->mpeg2 ? 5 : 4)
		| set(ct_info_iq_div_ctrl,    1)
    | set(ct_info_def_dc_ctrl,    p->mpeg2 ? p->intra_dc_precision : 0)
    | set(ct_info_iq_table_mask,  0xE)
    | set(ct_info_mc_mode,        p->mpeg2 ? 1 : 0)
    | set(ct_info_scan_mode,      p->mpeg2 && p->alternate_scan ? 3 : 2);

	device.regs->vdsi_param_8 = *cfg++ = 0;
  device.regs->vdsi_param_9 = *cfg++ = 0;
  device.regs->vdsi_param_10 = *cfg++ = 0;
  device.regs->vdsi_param_11 = *cfg++ = 0;
  device.regs->vdsi_param_12 = *cfg++ = 0
		| set(ct_info_lf_mode,        3)
    | set(ct_info_os_mode,        3)
    | set(ct_info_hor_fbuf_size,	p->geometry.width / 16);
	ct_info[1] = cfg;

	dma[0] = cfg;
  cfg = dma_element(ct_ba[0], ct_ba[1], cfg, false);
  cfg = dma_element(ct_wt[0], ct_wt[0]+1, cfg, false);
  cfg = dma_element_coeff(ct_wt[0]+1, ct_wt[1], cfg, false);
  cfg = dma_element(ct_info[0], ct_info[1], cfg, true);
  dma[1] =cfg;

  device.regs->vdsi_param_1 = 0
    | set(mpeg_si_info_decoding_mode,  					p->mpeg2 ? 2 : 1)
    | set(mpeg_si_info_progressive,    					p->mpeg2 && p->progressive_frame)
    | set(mpeg_si_info_mb_qp_luma,     					p->mpeg2 ? (1 << (3 - p->intra_dc_precision)) : 8)
    | set(mpeg_si_info_f_code00,       					p->mpeg2 ? p->f_code[0][0] : p->forward_f_code)
    | set(mpeg_si_info_f_code01,       					p->mpeg2 ? p->f_code[0][1] : p->forward_f_code)
    | set(mpeg_si_info_f_code10,       					p->mpeg2 ? p->f_code[1][0] : p->backward_f_code)
    | set(mpeg_si_info_f_code11,       					p->mpeg2 ? p->f_code[1][1] : p->backward_f_code)
    | set(mpeg_si_info_concealment_vectors, 		p->mpeg2 && p->concealment_motion_vectors)
    | set(mpeg_si_info_frame_pred_frame_dct,		p->mpeg2 && p->frame_pred_frame_dct);

	device.regs->vdsi_param_2 = set(mpeg_si_info_intra_vlc_format, p && p->intra_vlc_format);
  device.regs->vdsi_param_3 = 0
    | set(mpeg_si_info_full_pel_forward_flag,  p->full_pel_forward_vector)
    | set(mpeg_si_info_full_pel_backward_flag, p->full_pel_backward_vector)
    | set(mpeg_si_info_q_scale_type,           p->mpeg2 && p->q_scale_type)
    | set(mpeg_si_info_top_field_first,        p->mpeg2 && p->top_field_first);

  device.regs->vdsi_su_nsuma_ba = stream->su_nsuma_buffer;

  device.regs->vdsi_cfg = 0x80;
  device.regs->vdsi_conceal_cfg = 1 << 17;

  device.regs->vdsi_dma_list0_ad = set(dma_list_en, 1) | set(dma_list_ad, sizeof(uint32_t)*(dma[0] - device.regs->vdsi_cfg_data));
  device.regs->vdsi_dma_list1_ad = 0;

	return 0;
}

static
int copy_mpeg_params(struct msvd_mpeg_decode_params __user* args,
	struct msvd_mpeg_decode_params* params,
	struct msvd_mpeg_quantiser_matrix* intra_quantiser_matrix,
	struct msvd_mpeg_quantiser_matrix* non_intra_quantiser_matrix)
{
	if(copy_from_user(params, args, sizeof(*params))) return -EFAULT;

	if(params->intra_quantiser_matrix) {
		if(copy_from_user(intra_quantiser_matrix, params->intra_quantiser_matrix, sizeof(*intra_quantiser_matrix))) return -EFAULT;
		params->intra_quantiser_matrix = intra_quantiser_matrix;
	}
	if(params->non_intra_quantiser_matrix) {
		if(copy_from_user(non_intra_quantiser_matrix, params->non_intra_quantiser_matrix, sizeof(*non_intra_quantiser_matrix))) return -EFAULT;
		params->non_intra_quantiser_matrix = non_intra_quantiser_matrix;
	}

	return 0;
}

static int ioctl_start_decode_mpeg_frame(struct inode* inode, struct file* file, struct msvd_mpeg_decode_params __user* args) {
	int err;
	struct stream* stream = file->private_data;

	//all of this accessed only after device acquisition
	static struct msvd_mpeg_decode_params params;
	static struct msvd_mpeg_quantiser_matrix intra_qm, non_intra_qm;

	if(is_stream_readable(stream) || !acquire_device()) return -EAGAIN;

	if((err = copy_mpeg_params(args, &params, &intra_qm, &non_intra_qm)) < 0) goto release_and_return;

	if((err = configure_device_for_mpeg(stream, &params)) < 0) goto release_and_return;

	if((err = copy_swap_data(params.slice_data, params.slice_data_n)) < 0) goto release_and_return;

	config_sr1(0, err, false);

	stream->hor_pic_size_in_mbs = params.hor_pic_size_in_mbs;

	start_decode(stream);

	return 0;
release_and_return:
	release_device();
	return err;
}

static long ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	dev_dbg(device.dev, "ioctl: %x, %lx\n", cmd, arg);
	switch(cmd) {
	case MSVD_DECODE_H264_SLICE:
		return ioctl_start_decode_slice(0, filp, (struct msvd_h264_decode_params*)arg);
	case MSVD_DECODE_MPEG_FRAME:
		return ioctl_start_decode_mpeg_frame(0, filp, (struct msvd_mpeg_decode_params*)arg);
	default:
		return -EINVAL;
	}
}

static int read(struct file* file, char __user* buffer, size_t size, loff_t* offset) {
	struct stream* stream = file->private_data;
	struct msvd_decode_result status;

	dev_dbg(device.dev, "read: %p\n", stream);

	if(size < sizeof(stream->status)) return -EINVAL;

	if(!stream_get_status(stream, &status)) return -EAGAIN;

	if(copy_to_user(buffer, &status, sizeof(status))) return -EFAULT;

	return sizeof(status);
}

static unsigned int poll(struct file* file, struct poll_table_struct* wait) {
	struct stream* stream = file->private_data;
	bool a = is_device_available();
	bool r = is_stream_readable(stream);

	dev_dbg(device.dev, "poll: %p, %s, %s\n", stream, is_device_available() ? "avail" : "busy", is_stream_readable(stream) ? "readable" : "non-readable");
	poll_wait(file, &device.wq, wait);


	return ((a && !r)  ? POLLOUT : 0) | (r ? POLLIN : 0);
}

static int open(struct inode *inode, struct file *filp) {
	filp->private_data = alloc_stream();
	if(!filp->private_data) return -ENOMEM;

	return 0;
}

static int release(struct inode *inode, struct file *filp) {
  struct stream* stream = filp->private_data;
	unsigned long flags;

	spin_lock_irqsave(&stream->lock, flags);
	stream->closing = true;
	//if(stream->decoding) {
	//	dev_dbg(device.dev, "release: si_cmd_stop_decoding\n");
  //		iowrite32(si_cmd_stop_decoding, &device.regs->vdsi_cmd);
	//}
	spin_unlock_irqrestore(&stream->lock, flags);

	force_stop();
	stream_release(stream);

	return 0;
}

static struct file_operations msvdhd_ops = {
	owner:		THIS_MODULE,
	unlocked_ioctl:	 	ioctl,
	open:			open,
	release:	release,
	read:			read,
	poll:			poll
};

static int probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	unsigned int val;

	device.dev = &pdev->dev;

	dev_err(device.dev, "probe %x\n",(void*)&device.regs - (void*)&device);

	device.busy = 0;

	spin_lock_init(&device.lock);
	sema_init(&device.mx, 1);
	init_waitqueue_head(&device.wq);
/*	init_timer(&device.watchdog);
	setup_timer(&device.watchdog, decode_timeout, 0);*/

	platform_set_drvdata(pdev, &device);

	device.irq = platform_get_irq_byname(pdev, "irq");
	if(device.irq < 0) {
		dev_err(device.dev, "irq not defined\n");
		ret = -ENODEV;
		goto failed_get_irq;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "registers");
	if(!res) {
		dev_err(device.dev, "registers not defined\n");
		ret = -ENODEV;
		goto failed_get_regs;
	}

	device.regs = ioremap(res->start, res->end - res->start + 1);
	if(!device.regs) {
		dev_err(device.dev, "failed to map registers\n");
		ret = -ENOMEM;
		goto failed_remap;
	}

	val = ioread32(&device.regs->vd_id);
	if(((val >> 12) & 0xFF) != 0x53) {
		dev_err(device.dev, "no MSVD-HD signature\n");
		ret = -ENODEV;
		goto failed_read_reg;
	}

	device.buf = alloc_pages_exact(MSVD_MAP_BUF_SIZE, GFP_KERNEL);
	if(!device.buf) {
		dev_err(device.dev, "failed to allocate input buffer");
		ret = -ENOMEM;
		goto failed_alloc_buf;
	}
	device.buf_phys = __virt_to_phys((unsigned long)device.buf);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "extmem");
	if (!res) {
		dev_err(device.dev, "extmem not defined properly\n");
		ret = -ENODEV;
		goto failed_get_extmem;
	}
	device.extmem.begin = res->start;
	device.extmem.length = MSVD_MAP_EXTMEM_SIZE;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intmem");
	if (!res || resource_size(res) != MSVD_MAP_INTMEM_SIZE) {
		dev_err(device.dev, "intmem not defined properly\n");
		ret = -ENODEV;
		goto failed_get_intmem;
	}
	device.intmem.begin = res->start;
	device.intmem.length = MSVD_MAP_INTMEM_SIZE;

	device.mdev.minor = MISC_DYNAMIC_MINOR;
	device.mdev.name = DRIVER_NAME;
	device.mdev.parent = device.dev;
	device.mdev.fops = &msvdhd_ops;
	ret = misc_register(&device.mdev);
	if (ret) {
		dev_err(device.dev, "failed to register\n");
		goto failed_register_dev;
	}

	ret = request_irq(device.irq, msvdhd_irq, 0, DRIVER_NAME, &device);
  if(ret) {
  	dev_err(device.dev, "failed to set irq handler\n");
		goto failed_request_irq;
	}

	if(!acquire_device()) BUG_ON(true);

	reset();
	release_device();

	return 0;
failed_request_irq:
failed_register_dev:
failed_get_intmem:
failed_get_extmem:
	free_pages_exact(device.buf, MSVD_MAP_BUF_SIZE);
failed_alloc_buf:
failed_read_reg:
	iounmap(device.regs);
failed_remap:
failed_get_regs:
failed_get_irq:
	platform_set_drvdata(pdev, 0);
	return ret;
}

static int remove(struct platform_device *pdev) {
	misc_deregister(&device.mdev);
	free_pages_exact(device.buf, MSVD_MAP_BUF_SIZE);
	iounmap(device.regs);
	platform_set_drvdata(pdev, 0);
	return 0;
}

static const struct of_device_id msvd_of_match_table[] = {
	{ .compatible = "module,msvdhd", },
	{ /* end of list */ }
};

MODULE_DEVICE_TABLE(of, msvd_of_match_table);
static struct platform_driver driver = {
	.probe = probe,
	.remove = remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msvd_of_match_table
	}
};

static int __init init(void) {
	return platform_driver_register(&driver);
}

static void __exit deinit(void) {
	platform_driver_unregister(&driver);
}

module_init(init);
module_exit(deinit);

MODULE_DESCRIPTION("MSVD-HD driver");
MODULE_LICENSE("GPL");
