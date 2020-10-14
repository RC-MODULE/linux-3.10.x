#ifndef REGS_PCIE_H
#define REGS_PCIE_H

/* --- SCTL --- */

#define SCTL_PCIE_RST   0x400
#define SCTL_PCIE_REG_0 0x404
#define SCTL_PCIE_REG_1 0x408
#define SCTL_PCIE_REG_2 0x40C

/**
 * \defgroup regs_pcie Cadence PCIe
 * \ingroup regs
 *
 * Register offsets and constants
 * \code{.c}
 * #include <regs/regs_pcie.h>
 * \endcode
 *
 * \defgroup regs_pcie_constants Constants
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_constants
 * @{
 */

#define PCIe_Core_FuncRPConfig  0x00000000
#define PCIe_Core_LocalMgmt     0x00001000
#define PCIe_Core_AXIConfig     0x00004000
#define PCIe_Core_DMAConfig     0x00006000

/**
 * @}
 * \defgroup regs_pcie_ep_regs PCIe Core EndPoint mode Register offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_ep_regs
 * @{
 */

#define PCIe_EP_i_vendor_id_device_id                    0x0000
#define PCIe_EP_i_command_status                         0x0004
#define PCIe_EP_i_revision_id_class_code                 0x0008
#define PCIe_EP_i_bist_header_latency_cache_line         0x000C
#define PCIe_EP_i_base_addr_0                            0x0010
#define PCIe_EP_i_base_addr_1                            0x0014
#define PCIe_EP_i_base_addr_2                            0x0018
#define PCIe_EP_i_base_addr_3                            0x001C
#define PCIe_EP_i_base_addr_4                            0x0020
#define PCIe_EP_i_base_addr_5                            0x0024
#define PCIe_EP_i_subsystem_vendor_id_subsystem_i        0x002C
#define PCIe_EP_i_capabilities_pointer                   0x0034
#define PCIe_EP_i_intrpt_line_intrpt_pin                 0x003C
#define PCIe_EP_i_pwr_mgmt_cap                           0x0080
#define PCIe_EP_i_pwr_mgmt_ctrl_stat_rep                 0x0084
#define PCIe_EP_i_msi_ctrl_reg                           0x0090
#define PCIe_EP_i_msi_msg_low_addr                       0x0094
#define PCIe_EP_i_msi_msg_hi_addr                        0x0098
#define PCIe_EP_i_msi_msg_data                           0x009C
#define PCIe_EP_i_msi_mask                               0x00A0
#define PCIe_EP_i_msi_pending_bits                       0x00A4
#define PCIe_EP_i_msix_ctrl                              0x00B0
#define PCIe_EP_i_msix_tbl_offset                        0x00B4
#define PCIe_EP_i_msix_pending_intrpt                    0x00B8
#define PCIe_EP_i_pcie_cap_list                          0x00C0
#define PCIe_EP_i_pcie_dev_cap                           0x00C4
#define PCIe_EP_i_pcie_dev_ctrl_status                   0x00C8
#define PCIe_EP_i_link_cap                               0x00CC
#define PCIe_EP_i_link_ctrl_status                       0x00D0
#define PCIe_EP_i_pcie_dev_cap_2                         0x00E4
#define PCIe_EP_i_pcie_dev_ctrl_status_2                 0x00E8
#define PCIe_EP_i_link_cap_2_reg                         0x00EC
#define PCIe_EP_i_link_ctrl_status_2                     0x00F0
#define PCIe_EP_i_AER_enhanced_cap_hdr                   0x0100
#define PCIe_EP_i_uncorr_err_status                      0x0104
#define PCIe_EP_i_uncorr_err_mask                        0x0108
#define PCIe_EP_i_uncorr_err_severity                    0x010C
#define PCIe_EP_i_corr_err_status                        0x0110
#define PCIe_EP_i_corr_err_mask                          0x0114
#define PCIe_EP_i_advcd_err_cap_ctrl                     0x0118
#define PCIe_EP_i_hdr_log_0                              0x011C
#define PCIe_EP_i_hdr_log_1                              0x0120
#define PCIe_EP_i_hdr_log_2                              0x0124
#define PCIe_EP_i_hdr_log_3                              0x0128

/**
 * @}
 * \defgroup regs_pcie_rp_regs PCIe Core RootPort mode Register offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_rp_regs
 * @{
 */

#define PCIe_RP_i_vendor_id_device_id                    0x0000
#define PCIe_RP_i_command_status                         0x0004
#define PCIe_RP_i_revision_id_class_code                 0x0008
#define PCIe_RP_i_bist_header_latency_cache_line         0x000C
#define PCIe_RP_i_RC_BAR_0                               0x0010
#define PCIe_RP_i_RC_BAR_1                               0x0014
#define PCIe_RP_i_pcie_bus_numbers                       0x0018
#define PCIe_RP_i_pcie_io_base_limit                     0x001C
#define PCIe_RP_i_pcie_mem_base_limit                    0x0020
#define PCIe_RP_i_pcie_prefetch_base_limit               0x0024
#define PCIe_RP_i_pcie_prefetch_base_upper               0x0028
#define PCIe_RP_i_pcie_prefetch_limit_upper              0x002C
#define PCIe_RP_i_pcie_io_base_limit_upper               0x0030
#define PCIe_RP_i_capabilities_pointer                   0x0034
#define PCIe_RP_i_intrpt_line_intrpt_pin                 0x003C
#define PCIe_RP_i_pwr_mgmt_cap                           0x0080
#define PCIe_RP_i_pwr_mgmt_ctrl_stat_rep                 0x0084
#define PCIe_RP_i_msi_ctrl_reg                           0x0090
#define PCIe_RP_i_msi_msg_low_addr                       0x0094
#define PCIe_RP_i_msi_msg_hi_addr                        0x0098
#define PCIe_RP_i_msi_msg_data                           0x009C
#define PCIe_RP_i_msi_mask                               0x00A0
#define PCIe_RP_i_msi_pending_bits                       0x00A4
#define PCIe_RP_i_msix_ctrl                              0x00B0
#define PCIe_RP_i_msix_tbl_offset                        0x00B4
#define PCIe_RP_i_msix_pending_intrpt                    0x00B8
#define PCIe_RP_i_pcie_cap_list                          0x00C0
#define PCIe_RP_i_pcie_cap                               0x00C4
#define PCIe_RP_i_pcie_dev_ctrl_status                   0x00C8
#define PCIe_RP_i_link_cap                               0x00CC
#define PCIe_RP_i_link_ctrl_status                       0x00D0
#define PCIe_RP_i_slot_ctrl_status                       0x00D8
#define PCIe_RP_i_root_ctrl_cap                          0x00DC
#define PCIe_RP_i_root_status                            0x00E0
#define PCIe_RP_i_pcie_cap_2                             0x00E4
#define PCIe_RP_i_pcie_dev_ctrl_status_2                 0x00E8
#define PCIe_RP_i_link_cap_2                             0x00EC
#define PCIe_RP_i_link_ctrl_status_2                     0x00F0
#define PCIe_RP_i_AER_enhncd_cap                         0x0100
#define PCIe_RP_i_uncorr_err_status                      0x0104
#define PCIe_RP_i_uncorr_err_mask                        0x0108
#define PCIe_RP_i_uncorr_err_severity                    0x010C
#define PCIe_RP_i_corr_err_status                        0x0110
#define PCIe_RP_i_corr_err_mask                          0x0114
#define PCIe_RP_i_adv_err_cap_ctl                        0x0118
#define PCIe_RP_i_hdr_log_0                              0x011C
#define PCIe_RP_i_hdr_log_1                              0x0120
#define PCIe_RP_i_hdr_log_2                              0x0124
#define PCIe_RP_i_hdr_log_3                              0x0128
#define PCIe_RP_i_root_err_cmd                           0x012C
#define PCIe_RP_i_root_err_stat                          0x0130
#define PCIe_RP_i_err_src_id                             0x0134

/**
 * @}
 * \defgroup regs_pcie_locmgmt_regs PCIe Core Local Management Register offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_locmgmt_regs
 * @{
 */

#define PCIe_LocMgmt_i_pl_config_0_reg                         0x0000
#define PCIe_LocMgmt_i_pl_config_1_reg                         0x0004
#define PCIe_LocMgmt_i_dll_tmr_config_reg                      0x0008
#define PCIe_LocMgmt_i_rcv_cred_lim_0_reg                      0x000C
#define PCIe_LocMgmt_i_rcv_cred_lim_1_reg                      0x0010
#define PCIe_LocMgmt_i_transm_cred_lim_0_reg                   0x0014
#define PCIe_LocMgmt_i_transm_cred_lim_1_reg                   0x0018
#define PCIe_LocMgmt_i_transm_cred_update_int_config_0_reg     0x001C
#define PCIe_LocMgmt_i_transm_cred_update_int_config_1_reg     0x0020
#define PCIe_LocMgmt_i_L0S_timeout_limit_reg                   0x0024
#define PCIe_LocMgmt_i_transmit_tlp_count_reg                  0x0028
#define PCIe_LocMgmt_i_transmit_tlp_payload_dword_count_reg    0x002C
#define PCIe_LocMgmt_i_receive_tlp_count_reg                   0x0030
#define PCIe_LocMgmt_i_receive_tlp_payload_dword_count_reg     0x0034
#define PCIe_LocMgmt_i_compln_tmout_lim_0_reg                  0x0038
#define PCIe_LocMgmt_i_compln_tmout_lim_1_reg                  0x003C
#define PCIe_LocMgmt_i_L1_st_reentry_delay_reg                 0x0040
#define PCIe_LocMgmt_i_vendor_id_reg                           0x0044
#define PCIe_LocMgmt_i_aspm_L1_entry_tmout_delay_reg           0x0048
#define PCIe_LocMgmt_i_pme_turnoff_ack_delay_reg               0x004C
#define PCIe_LocMgmt_i_linkwidth_control_reg                   0x0050
#define PCIe_LocMgmt_i_sris_control_reg                        0x0074
#define PCIe_LocMgmt_i_shdw_hdr_log_0_reg                      0x0100
#define PCIe_LocMgmt_i_shdw_hdr_log_1_reg                      0x0104
#define PCIe_LocMgmt_i_shdw_hdr_log_2_reg                      0x0108
#define PCIe_LocMgmt_i_shdw_hdr_log_3_reg                      0x010C
#define PCIe_LocMgmt_i_shdw_func_num_reg                       0x0110
#define PCIe_LocMgmt_i_shdw_ur_err_reg                         0x0114
#define PCIe_LocMgmt_i_debug_dllp_count_gen1_reg               0x0144
#define PCIe_LocMgmt_i_debug_dllp_count_gen2_reg               0x0148
#define PCIe_LocMgmt_i_debug_dllp_count_gen3_reg               0x014C
#define PCIe_LocMgmt_i_negotiated_lane_map_reg                 0x0200
#define PCIe_LocMgmt_i_receive_fts_count_reg                   0x0204
#define PCIe_LocMgmt_i_debug_mux_control_reg                   0x0208
#define PCIe_LocMgmt_i_local_error_status_register             0x020C
#define PCIe_LocMgmt_i_local_intrpt_mask_reg                   0x0210
#define PCIe_LocMgmt_i_lcrc_err_count_reg                      0x0214
#define PCIe_LocMgmt_i_ecc_corr_err_count_reg                  0x0218
#define PCIe_LocMgmt_i_pme_service_timeout_delay_reg           0x0224
#define PCIe_LocMgmt_i_root_port_requestor_id_reg              0x0228
#define PCIe_LocMgmt_i_ep_bus_device_number_reg                0x022C
#define PCIe_LocMgmt_i_pf_0_BAR_config_0_reg                   0x0240
#define PCIe_LocMgmt_i_pf_0_BAR_config_1_reg                   0x0244
#define PCIe_LocMgmt_i_pf_config_reg                           0x02C0
#define PCIe_LocMgmt_i_rc_BAR_config_reg                       0x0300
#define PCIe_LocMgmt_i_ecc_corr_err_count_reg_axi              0x0C80
#define PCIe_LocMgmt_low_power_debug_and_control0              0x0C88
#define PCIe_LocMgmt_low_power_debug_and_control1              0x0C8C
#define PCIe_LocMgmt_tl_internal_control                       0x0C94
#define PCIe_LocMgmt_i_ld_ctrl                                 0x0DA0
#define PCIe_LocMgmt_rx_elec_idle_filter_control               0x0DA4

/**
 * @}
 * \defgroup regs_pcie_axi_regs PCIe AXI port Registers offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_axi_regs
 * @{
 */

#define PCIe_AXI_outregion_0_addr_translation_0                0x0000
#define PCIe_AXI_outregion_0_addr_translation_1                0x0004
#define PCIe_AXI_outregion_0_header_descriptor_0               0x0008
#define PCIe_AXI_outregion_0_header_descriptor_1               0x000C
#define PCIe_AXI_outregion_0_region_base_addr                  0x0018
#define PCIe_AXI_outregion_1_addr_translation_0                0x0020
#define PCIe_AXI_outregion_1_addr_translation_1                0x0024
#define PCIe_AXI_outregion_1_header_descriptor_0               0x0028
#define PCIe_AXI_outregion_1_header_descriptor_1               0x002C
#define PCIe_AXI_outregion_1_region_base_addr                  0x0038
#define PCIe_AXI_outregion_2_addr_translation_0                0x0040
#define PCIe_AXI_outregion_2_addr_translation_1                0x0044
#define PCIe_AXI_outregion_2_header_descriptor_0               0x0048
#define PCIe_AXI_outregion_2_header_descriptor_1               0x004C
#define PCIe_AXI_outregion_2_region_base_addr                  0x0058
#define PCIe_AXI_outregion_3_addr_translation_0                0x0060
#define PCIe_AXI_outregion_3_addr_translation_1                0x0064
#define PCIe_AXI_outregion_3_header_descriptor_0               0x0068
#define PCIe_AXI_outregion_3_header_descriptor_1               0x006C
#define PCIe_AXI_outregion_3_region_base_addr                  0x0078
#define PCIe_AXI_outregion_4_addr_translation_0                0x0080
#define PCIe_AXI_outregion_4_addr_translation_1                0x0084
#define PCIe_AXI_outregion_4_header_descriptor_0               0x0088
#define PCIe_AXI_outregion_4_header_descriptor_1               0x008C
#define PCIe_AXI_outregion_4_region_base_addr                  0x0098
#define PCIe_AXI_outregion_5_addr_translation_0                0x00A0
#define PCIe_AXI_outregion_5_addr_translation_1                0x00A4
#define PCIe_AXI_outregion_5_header_descriptor_0               0x00A8
#define PCIe_AXI_outregion_5_header_descriptor_1               0x00AC
#define PCIe_AXI_outregion_5_region_base_addr                  0x00B8
#define PCIe_AXI_outregion_6_addr_translation_0                0x00C0
#define PCIe_AXI_outregion_6_addr_translation_1                0x00C4
#define PCIe_AXI_outregion_6_header_descriptor_0               0x00C8
#define PCIe_AXI_outregion_6_header_descriptor_1               0x00CC
#define PCIe_AXI_outregion_6_region_base_addr                  0x00D8
#define PCIe_AXI_outregion_7_addr_translation_0                0x00E0
#define PCIe_AXI_outregion_7_addr_translation_1                0x00E4
#define PCIe_AXI_outregion_7_header_descriptor_0               0x00E8
#define PCIe_AXI_outregion_7_header_descriptor_1               0x00EC
#define PCIe_AXI_outregion_7_region_base_addr                  0x00F8
#define PCIe_AXI_outregion_8_addr_translation_0                0x0100
#define PCIe_AXI_outregion_8_addr_translation_1                0x0104
#define PCIe_AXI_outregion_8_header_descriptor_0               0x0108
#define PCIe_AXI_outregion_8_header_descriptor_1               0x010C
#define PCIe_AXI_outregion_8_region_base_addr                  0x0118
#define PCIe_AXI_outregion_9_addr_translation_0                0x0120
#define PCIe_AXI_outregion_9_addr_translation_1                0x0124
#define PCIe_AXI_outregion_9_header_descriptor_0               0x0128
#define PCIe_AXI_outregion_9_header_descriptor_1               0x012C
#define PCIe_AXI_outregion_9_region_base_addr                  0x0138
#define PCIe_AXI_outregion_10_addr_translation_0               0x0140
#define PCIe_AXI_outregion_10_addr_translation_1               0x0144
#define PCIe_AXI_outregion_10_header_descriptor_0              0x0148
#define PCIe_AXI_outregion_10_header_descriptor_1              0x014C
#define PCIe_AXI_outregion_10_region_base_addr                 0x0158
#define PCIe_AXI_outregion_11_addr_translation_0               0x0160
#define PCIe_AXI_outregion_11_addr_translation_1               0x0164
#define PCIe_AXI_outregion_11_header_descriptor_0              0x0168
#define PCIe_AXI_outregion_11_header_descriptor_1              0x016C
#define PCIe_AXI_outregion_11_region_base_addr                 0x0178
#define PCIe_AXI_outregion_12_addr_translation_0               0x0180
#define PCIe_AXI_outregion_12_addr_translation_1               0x0184
#define PCIe_AXI_outregion_12_header_descriptor_0              0x0188
#define PCIe_AXI_outregion_12_header_descriptor_1              0x018C
#define PCIe_AXI_outregion_12_region_base_addr                 0x0198
#define PCIe_AXI_outregion_13_addr_translation_0               0x01A0
#define PCIe_AXI_outregion_13_addr_translation_1               0x01A4
#define PCIe_AXI_outregion_13_header_descriptor_0              0x01A8
#define PCIe_AXI_outregion_13_header_descriptor_1              0x01AC
#define PCIe_AXI_outregion_13_region_base_addr                 0x01B8
#define PCIe_AXI_outregion_14_addr_translation_0               0x01C0
#define PCIe_AXI_outregion_14_addr_translation_1               0x01C4
#define PCIe_AXI_outregion_14_header_descriptor_0              0x01C8
#define PCIe_AXI_outregion_14_header_descriptor_1              0x01CC
#define PCIe_AXI_outregion_14_region_base_addr                 0x01D8
#define PCIe_AXI_outregion_15_addr_translation_0               0x01E0
#define PCIe_AXI_outregion_15_addr_translation_1               0x01E4
#define PCIe_AXI_outregion_15_header_descriptor_0              0x01E8
#define PCIe_AXI_outregion_15_header_descriptor_1              0x01EC
#define PCIe_AXI_outregion_15_region_base_addr                 0x01F8
#define PCIe_AXI_outregion_16_addr_translation_0               0x0200
#define PCIe_AXI_outregion_16_addr_translation_1               0x0204
#define PCIe_AXI_outregion_16_header_descriptor_0              0x0208
#define PCIe_AXI_outregion_16_header_descriptor_1              0x020C
#define PCIe_AXI_outregion_16_region_base_addr                 0x0218
#define PCIe_AXI_outregion_17_addr_translation_0               0x0220
#define PCIe_AXI_outregion_17_addr_translation_1               0x0224
#define PCIe_AXI_outregion_17_header_descriptor_0              0x0228
#define PCIe_AXI_outregion_17_header_descriptor_1              0x022C
#define PCIe_AXI_outregion_17_region_base_addr                 0x0238
#define PCIe_AXI_outregion_18_addr_translation_0               0x0240
#define PCIe_AXI_outregion_18_addr_translation_1               0x0244
#define PCIe_AXI_outregion_18_header_descriptor_0              0x0248
#define PCIe_AXI_outregion_18_header_descriptor_1              0x024C
#define PCIe_AXI_outregion_18_region_base_addr                 0x0258
#define PCIe_AXI_outregion_19_addr_translation_0               0x0260
#define PCIe_AXI_outregion_19_addr_translation_1               0x0264
#define PCIe_AXI_outregion_19_header_descriptor_0              0x0268
#define PCIe_AXI_outregion_19_header_descriptor_1              0x026C
#define PCIe_AXI_outregion_19_region_base_addr                 0x0278
#define PCIe_AXI_outregion_20_addr_translation_0               0x0280
#define PCIe_AXI_outregion_20_addr_translation_1               0x0284
#define PCIe_AXI_outregion_20_header_descriptor_0              0x0288
#define PCIe_AXI_outregion_20_header_descriptor_1              0x028C
#define PCIe_AXI_outregion_20_region_base_addr                 0x0298
#define PCIe_AXI_outregion_21_addr_translation_0               0x02A0
#define PCIe_AXI_outregion_21_addr_translation_1               0x02A4
#define PCIe_AXI_outregion_21_header_descriptor_0              0x02A8
#define PCIe_AXI_outregion_21_header_descriptor_1              0x02AC
#define PCIe_AXI_outregion_21_region_base_addr                 0x02B8
#define PCIe_AXI_outregion_22_addr_translation_0               0x02C0
#define PCIe_AXI_outregion_22_addr_translation_1               0x02C4
#define PCIe_AXI_outregion_22_header_descriptor_0              0x02C8
#define PCIe_AXI_outregion_22_header_descriptor_1              0x02CC
#define PCIe_AXI_outregion_22_region_base_addr                 0x02D8
#define PCIe_AXI_outregion_23_addr_translation_0               0x02E0
#define PCIe_AXI_outregion_23_addr_translation_1               0x02E4
#define PCIe_AXI_outregion_23_header_descriptor_0              0x02E8
#define PCIe_AXI_outregion_23_header_descriptor_1              0x02EC
#define PCIe_AXI_outregion_23_region_base_addr                 0x02F8
#define PCIe_AXI_outregion_24_addr_translation_0               0x0300
#define PCIe_AXI_outregion_24_addr_translation_1               0x0304
#define PCIe_AXI_outregion_24_header_descriptor_0              0x0308
#define PCIe_AXI_outregion_24_header_descriptor_1              0x030C
#define PCIe_AXI_outregion_24_region_base_addr                 0x0318
#define PCIe_AXI_outregion_25_addr_translation_0               0x0320
#define PCIe_AXI_outregion_25_addr_translation_1               0x0324
#define PCIe_AXI_outregion_25_header_descriptor_0              0x0328
#define PCIe_AXI_outregion_25_header_descriptor_1              0x032C
#define PCIe_AXI_outregion_25_region_base_addr                 0x0338
#define PCIe_AXI_outregion_26_addr_translation_0               0x0340
#define PCIe_AXI_outregion_26_addr_translation_1               0x0344
#define PCIe_AXI_outregion_26_header_descriptor_0              0x0348
#define PCIe_AXI_outregion_26_header_descriptor_1              0x034C
#define PCIe_AXI_outregion_26_region_base_addr                 0x0358
#define PCIe_AXI_outregion_27_addr_translation_0               0x0360
#define PCIe_AXI_outregion_27_addr_translation_1               0x0364
#define PCIe_AXI_outregion_27_header_descriptor_0              0x0368
#define PCIe_AXI_outregion_27_header_descriptor_1              0x036C
#define PCIe_AXI_outregion_27_region_base_addr                 0x0378
#define PCIe_AXI_outregion_28_addr_translation_0               0x0380
#define PCIe_AXI_outregion_28_addr_translation_1               0x0384
#define PCIe_AXI_outregion_28_header_descriptor_0              0x0388
#define PCIe_AXI_outregion_28_header_descriptor_1              0x038C
#define PCIe_AXI_outregion_28_region_base_addr                 0x0398
#define PCIe_AXI_outregion_29_addr_translation_0               0x03A0
#define PCIe_AXI_outregion_29_addr_translation_1               0x03A4
#define PCIe_AXI_outregion_29_header_descriptor_0              0x03A8
#define PCIe_AXI_outregion_29_header_descriptor_1              0x03AC
#define PCIe_AXI_outregion_29_region_base_addr                 0x03B8
#define PCIe_AXI_outregion_30_addr_translation_0               0x03C0
#define PCIe_AXI_outregion_30_addr_translation_1               0x03C4
#define PCIe_AXI_outregion_30_header_descriptor_0              0x03C8
#define PCIe_AXI_outregion_30_header_descriptor_1              0x03CC
#define PCIe_AXI_outregion_30_region_base_addr                 0x03D8
#define PCIe_AXI_outregion_31_addr_translation_0               0x03E0
#define PCIe_AXI_outregion_31_addr_translation_1               0x03E4
#define PCIe_AXI_outregion_31_header_descriptor_0              0x03E8
#define PCIe_AXI_outregion_31_header_descriptor_1              0x03EC
#define PCIe_AXI_outregion_31_region_base_addr                 0x03F8
#define PCIe_AXI_inregion_rc_bar_0_addr_translation            0x0800
#define PCIe_AXI_inregion_rc_bar_1_addr_translation            0x0808
#define PCIe_AXI_inregion_rc_bar_2_addr_translation            0x0810
#define PCIe_AXI_l0                                            0x0824
#define PCIe_AXI_inregion_ep_bar_0_addr_translation            0x0840
#define PCIe_AXI_inregion_ep_bar_1_addr_translation            0x0848
#define PCIe_AXI_inregion_ep_bar_2_addr_translation            0x0850
#define PCIe_AXI_inregion_ep_bar_3_addr_translation            0x0858
#define PCIe_AXI_inregion_ep_bar_4_addr_translation            0x0860
#define PCIe_AXI_inregion_ep_bar_5_addr_translation            0x0868
#define PCIe_AXI_inregion_ep_bar_6_addr_translation            0x0870

/**
 * @}
 * \defgroup regs_pcie_dma_regs PCIe DMA Registers offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_dma_regs
 * @{
 */

#define PCIe_DMA_channel_0_ctrl                                0x0000
#define PCIe_DMA_channel_0_sp_l                                0x0004
#define PCIe_DMA_channel_0_sp_u                                0x0008
#define PCIe_DMA_channel_0_attr_l                              0x000C
#define PCIe_DMA_channel_0_attr_u                              0x0010
#define PCIe_DMA_channel_1_ctrl                                0x0014
#define PCIe_DMA_channel_1_sp_l                                0x0018
#define PCIe_DMA_channel_1_sp_u                                0x001C
#define PCIe_DMA_channel_1_attr_l                              0x0020
#define PCIe_DMA_channel_1_attr_u                              0x0024
#define PCIe_DMA_common_udma_int                               0x00A0
#define PCIe_DMA_common_udma_int_ena                           0x00A4
#define PCIe_DMA_common_udma_int_dis                           0x00A8
#define PCIe_DMA_common_udma_ib_ecc_uncorrectable_errors       0x00AC
#define PCIe_DMA_common_udma_ib_ecc_correctable_errors         0x00B0
#define PCIe_DMA_common_udma_ob_ecc_uncorrectable_errors       0x00B4
#define PCIe_DMA_common_udma_ob_ecc_correctable_errors         0x00B8
#define PCIe_DMA_common_udma_cap_ver                           0x00F8
#define PCIe_DMA_common_udma_config                            0x00FC

/**
 * @}
 * \defgroup regs_pcie_phy_regs PCIe Phy Registers offsets (not full list)
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_phy_regs
 * @{
 */
// paddr [11] was excluded, so it paddr forms next way
#define PCIe_Phy_PCS_CMN_CTRL1             0xC000*2 + 0x000*4
#define PCIe_Phy_PCS_CMN_CTRL2             0xC000*2 + 0x001*4
#define PCIe_Phy_PCS_COM_LOCK_CFG1         0xC000*2 + 0x002*4
#define PCIe_Phy_PCS_COM_LOCK_CFG2         0xC000*2 + 0x003*4
#define PCIe_Phy_PCS_GEN3_EIE_LOCK_CFG     0xC000*2 + 0x004*4
#define PCIe_Phy_PCS_LANE_DSBL             0xC000*2 + 0x005*4
#define PCIe_Phy_PCS_RCV_DET_INH           0xC000*2 + 0x006*4
#define PCIe_Phy_PCS_ISO_CMN_CTRL          0xC000*2 + 0x008*4
#define PCIe_Phy_EQ_TRAIN_CTRL             0xC000*2 + 0x008*4
#define PCIe_Phy_ISO_LN_CTRL               0xD000*2 + 0x000*4
#define PCIe_Phy_EQ_COEF_TRAIN_CTRL        0xD000*2 + 0x001*4
#define PCIe_Phy_PCS_ISO_TX_CTRL           0xD000*2 + 0x000*4
#define PCIe_Phy_PCS_ISO_TX_LPC_LO         0xD000*2 + 0x001*4
#define PCIe_Phy_PCS_ISO_TX_LPC_HI         0xD000*2 + 0x002*4
#define PCIe_Phy_PCS_ISO_TX_DMPH_LO        0xD000*2 + 0x003*4
#define PCIe_Phy_PCS_ISO_TX_DMPH_HI        0xD000*2 + 0x004*4
#define PCIe_Phy_PCS_ISO_TX_FSLF           0xD000*2 + 0x005*4
#define PCIe_Phy_PCS_ISO_TX_DATA_LO        0xD000*2 + 0x006*4
#define PCIe_Phy_PCS_ISO_TX_DATA_HI        0xD000*2 + 0x007*4
#define PCIe_Phy_PCS_ISO_RX_CTRL           0xD000*2 + 0x008*4
#define PCIe_Phy_PCS_ISO_RX_EQ_EVAL        0xD000*2 + 0x009*4
#define PCIe_Phy_PCS_ISO_RX_DATA_LO        0xD000*2 + 0x00E*4
#define PCIe_Phy_PCS_ISO_RX_DATA_HI        0xD000*2 + 0x00F*4
#define PCIe_Phy_PMA_CMN_CTRL              0xE000*2 + 0x000*4
#define PCIe_Phy_PMA_SSM_STATE             0xE000*2 + 0x001*4
#define PCIe_Phy_PMA_PSM_STATE_LO          0xE000*2 + 0x002*4
#define PCIe_Phy_PMA_PSM_STATE_HI          0xE000*2 + 0x003*4
#define PCIe_Phy_PMA_ISO_CMN_CTRL          0xE000*2 + 0x004*4
#define PCIe_Phy_PMA_ISO_MODE              0xE000*2 + 0x005*4
#define PCIe_Phy_PMA_ISO_PWRST_CTRL        0xE000*2 + 0x006*4
#define PCIe_Phy_PMA_ISOLATION_CTRL        0xE000*2 + 0x00F*4
#define PCIe_Phy_PMA_PMA_CMN_SSM_DIAG      0x0000*2 + 0x039*4
#define PCIe_Phy_PMA_XCVR_CTRL             0xF000*2 + 0x000*4
#define PCIe_Phy_PMA_XCVR_LPBK             0xF000*2 + 0x001*4
#define PCIe_Phy_PMA_PI_POS                0xF000*2 + 0x002*4
#define PCIe_Phy_PMA_ISO_XCVR_CTRL         0xF000*2 + 0x003*4
#define PCIe_Phy_PMA_ISO_TX_LPC_LO         0xF000*2 + 0x004*4
#define PCIe_Phy_PMA_ISO_TX_LPC_HI         0xF000*2 + 0x005*4
#define PCIe_Phy_PMA_ISO_TX_DMPH_LO        0xF000*2 + 0x006*4
#define PCIe_Phy_PMA_ISO_TX_DMPH_HI        0xF000*2 + 0x007*4
#define PCIe_Phy_PMA_ISO_TX_FSLF           0xF000*2 + 0x008*4
#define PCIe_Phy_PMA_ISO_TX_MGN            0xF000*2 + 0x009*4
#define PCIe_Phy_PMA_ISO_RX_EQ_CTRL        0xF000*2 + 0x00D*4
#define PCIe_Phy_PMA_ISO_DATA_LO           0xF000*2 + 0x00E*4
#define PCIe_Phy_PMA_ISO_DATA_HI           0xF000*2 + 0x00F*4
#define PCIe_Phy_PMA_TX_BIST_CTRL          0x4000*2 + 0x040*4
#define PCIe_Phy_PMA_TX_BIST_UDDWR         0x4000*2 + 0x041*4
#define PCIe_Phy_PMA_RX_BIST_CTRL          0x8000*2 + 0x120*4
#define PCIe_Phy_PMA_RX_BIST_SYNCCNT       0x8000*2 + 0x121*4
#define PCIe_Phy_PMA_RX_BIST_UDDWR         0x8000*2 + 0x122*4
#define PCIe_Phy_PMA_RX_BIST_ERRCNT        0x8000*2 + 0x123*4
#define PCIe_Phy_PMA_CMN_PID_TYPE          0x0000*2 + 0x000*4
#define PCIe_Phy_PMA_CMN_PID_NUM           0x0000*2 + 0x003*4
#define PCIe_Phy_PMA_CMN_PID_REV           0x0000*2 + 0x004*4
#define PCIe_Phy_PMA_CMN_PID_MFG           0x0000*2 + 0x008*4
#define PCIe_Phy_PMA_CMN_PID_NODE          0x0000*2 + 0x009*4
#define PCIe_Phy_PMA_CMN_PID_FLV           0x0000*2 + 0x00A*4
#define PCIe_Phy_PMA_CMN_PID_IOV           0x0000*2 + 0x00C*4
#define PCIe_Phy_PMA_CMN_PID_METAL         0x0000*2 + 0x00D*4
#define PCIe_Phy_PMA_CMN_PID_LANES         0x0000*2 + 0x010*4
#define PCIe_Phy_RX_EYESURF_CTRL           0x8000*2 + 0x0E0*4
#define PCIe_Phy_RX_EYESURF_TMR_DELLOW     0x8000*2 + 0x0E4*4
#define PCIe_Phy_RX_EYESURF_TMR_DELHIGH    0x8000*2 + 0x0E5*4
#define PCIe_Phy_RX_EYESURF_TMR_TESTLOW    0x8000*2 + 0x0E6*4
#define PCIe_Phy_RX_EYESURF_TMR_TESTHIGH   0x8000*2 + 0x0E7*4
#define PCIe_Phy_RX_EYESURF_NS_COORD       0x8000*2 + 0x0E8*4
#define PCIe_Phy_RX_EYESURF_EW_COORD       0x8000*2 + 0x0E9*4
#define PCIe_Phy_RX_EYESURF_ERRCNT         0x8000*2 + 0x0EA*4

/* --- This registers are used for netlist simspeedup --- */
#define PCIe_Phy_MASSWR_TX_RCVDET_CTRL                    0x6000*2 + 0x020*4
#define PCIe_Phy_CMN_SSM_BANDGAP_TMR                      0x0000*2 + 0x020*4
#define PCIe_Phy_CMN_SSM_BIAS_TMR                         0x0000*2 + 0x021*4
#define PCIe_Phy_CMN_SSM_PLLEN_TMR                        0x0000*2 + 0x022*4
#define PCIe_Phy_CMN_SSM_PLLPRE_TMR                       0x0000*2 + 0x023*4
#define PCIe_Phy_CMN_SSM_PLLVREF_TMR                      0x0000*2 + 0x024*4
#define PCIe_Phy_CMN_SSM_PLLLOCK_TMR                      0x0000*2 + 0x025*4
#define PCIe_Phy_CMN_SSM_PLLCLKDIS_TMR                    0x0000*2 + 0x026*4
#define PCIe_Phy_CMN_SSM_LANECAL_TMR                      0x0000*2 + 0x028*4
#define PCIe_Phy_CMN_SSM_A0IN_TMR                         0x0000*2 + 0x029*4
#define PCIe_Phy_CMN_SSM_A1IN_TMR                         0x0000*2 + 0x02A*4
#define PCIe_Phy_CMN_SSM_A2IN_TMR                         0x0000*2 + 0x02B*4
#define PCIe_Phy_CMN_SSM_A3IN_TMR                         0x0000*2 + 0x02C*4
#define PCIe_Phy_CMN_BGCAL_OVRD                           0x0000*2 + 0x061*4
#define PCIe_Phy_CMN_PLL_CPI_OVRD                         0x0000*2 + 0x081*4
#define PCIe_Phy_MASSWR_CMN_SDCAL0_OVRD                   0xA000*2 + 0x181*4
#define PCIe_Phy_MASSWR_CMN_SDCAL1_OVRD                   0xA000*2 + 0x191*4
#define PCIe_Phy_CMN_PLL_VCOCAL_OVRD                      0x0000*2 + 0x0A2*4
#define PCIe_Phy_CMN_PLL_VCOCAL_CNT_START                 0x0000*2 + 0x0A7*4
#define PCIe_Phy_CMN_PLL_LOCK_CNT_START                   0x0000*2 + 0x0A5*4
#define PCIe_Phy_MASSWR_RX_IQPI_ILL_CAL_OVRD              0xA000*2 + 0x022*4
#define PCIe_Phy_MASSWR_RX_IQPI_ILL_LOCK_REFCNT_START     0xA000*2 + 0x025*4
#define PCIe_Phy_MASSWR_RX_IQPI_ILL_LOCK_CALCNT_START0    0xA000*2 + 0x026*4
#define PCIe_Phy_MASSWR_RX_IQPI_ILL_LOCK_CALCNT_START1    0xA000*2 + 0x027*4
#define PCIe_Phy_MASSWR_RX_EPI_ILL_CAL_OVRD               0xA000*2 + 0x042*4
#define PCIe_Phy_MASSWR_RX_EPI_ILL_LOCK_REFCNT_START      0xA000*2 + 0x045*4
#define PCIe_Phy_MASSWR_RX_EPI_ILL_LOCK_CALCNT_START0     0xA000*2 + 0x046*4
#define PCIe_Phy_MASSWR_RX_EPI_ILL_LOCK_CALCNT_START1     0xA000*2 + 0x047*4
/* #define PCIe_Phy_LANE_CAL_RESET_TIME_VALUE            register wasn`t found */
#define PCIe_Phy_CMN_TXPUCAL_OVRD                         0x0000*2 + 0x0C1*4
#define PCIe_Phy_CMN_TXPDCAL_OVRD                         0x0000*2 + 0x0E1*4
#define PCIe_Phy_CMN_RXCAL_OVRD                           0x0000*2 + 0x101*4
#define PCIe_Phy_MASSWR_RX_SLC_INIT_TMR                   0xA000*2 + 0x14D*4
#define PCIe_Phy_MASSWR_RX_SLC_RUN_TMR                    0xA000*2 + 0x14E*4
/* #define PCIe_Phy_CMN_PRPLL_LOCK_CNT_START             register wasn`t found */

#define PCIe_Phy_PMA_CMN_PID_TYPE                     0x0000*2 + 0x000*4
#define PCIe_Phy_PMA_CMN_PID_NUM                      0x0000*2 + 0x003*4
#define PCIe_Phy_PMA_CMN_PID_REV                      0x0000*2 + 0x004*4
#define PCIe_Phy_PMA_CMN_PID_MFG                      0x0000*2 + 0x008*4
#define PCIe_Phy_PMA_CMN_PID_NODE                     0x0000*2 + 0x009*4
#define PCIe_Phy_PMA_CMN_PID_FLV                      0x0000*2 + 0x00A*4
#define PCIe_Phy_PMA_CMN_PID_IOV                      0x0000*2 + 0x00C*4
#define PCIe_Phy_PMA_CMN_PID_METAL                    0x0000*2 + 0x00D*4
#define PCIe_Phy_PMA_CMN_PID_LANES                    0x0000*2 + 0x010*4
#define PCIe_Phy_PMA_CMN_SSM_BANDGAP_TMR              0x0000*2 + 0x020*4
#define PCIe_Phy_PMA_CMN_SSM_BIAS_TMR                 0x0000*2 + 0x021*4
#define PCIe_Phy_PMA_CMN_SSM_PLLEN_TMR                0x0000*2 + 0x022*4
#define PCIe_Phy_PMA_CMN_SSM_PLLVREF_TMR              0x0000*2 + 0x024*4
#define PCIe_Phy_PMA_CMN_SSM_LANECAL_TMR              0x0000*2 + 0x028*4
#define PCIe_Phy_PMA_CMN_SSM_A1OUT_TMR                0x0000*2 + 0x030*4
#define PCIe_Phy_PMA_CMN_PSC_A0                       0x0000*2 + 0x040*4
#define PCIe_Phy_PMA_CMN_PSC_A1                       0x0000*2 + 0x041*4
#define PCIe_Phy_PMA_CMN_PSC_A2                       0x0000*2 + 0x042*4
#define PCIe_Phy_PMA_CMN_PSC_A4                       0x0000*2 + 0x044*4
#define PCIe_Phy_PMA_CMN_PLLCPICAL_CTRL               0x0000*2 + 0x080*4
#define PCIe_Phy_PMA_CMN_PLLCPICAL_OVRD               0x0000*2 + 0x081*4
#define PCIe_Phy_PMA_CMN_PLLCPICAL_INIT_TMR           0x0000*2 + 0x082*4
#define PCIe_Phy_PMA_CMN_RXCAL_CTRL                   0x0000*2 + 0x100*4
#define PCIe_Phy_PMA_CMN_RXCAL_OVRD                   0x0000*2 + 0x101*4
#define PCIe_Phy_PMA_CMN_RXCAL_INIT_TMR               0x0000*2 + 0x102*4
#define PCIe_Phy_PMA_CMN_TXPUCAL_CTRL                 0x0000*2 + 0x0C0*4
#define PCIe_Phy_PMA_CMN_TXPUCAL_OVRD                 0x0000*2 + 0x0C1*4
#define PCIe_Phy_PMA_CMN_TXPUCAL_INIT_TMR             0x0000*2 + 0x0C2*4
#define PCIe_Phy_PMA_CMN_TXPDCAL_CTRL                 0x0000*2 + 0x0E0*4
#define PCIe_Phy_PMA_CMN_TXPDCAL_OVRD                 0x0000*2 + 0x0E1*4
#define PCIe_Phy_PMA_CMN_TXPDCAL_INIT_TMR             0x0000*2 + 0x0E2*4
#define PCIe_Phy_PMA_TX_HBDC_PRE_OVRD                 0x4000*2 + 0x000*4
#define PCIe_Phy_PMA_TX_HBDC_MAIN_OVRD                0x4000*2 + 0x001*4
#define PCIe_Phy_PMA_TX_HBDC_POST_OVRD                0x4000*2 + 0x002*4
#define PCIe_Phy_PMA_TX_HBDC_PRE_PREM_OVRD            0x4000*2 + 0x004*4
#define PCIe_Phy_PMA_TX_HBDC_MAIN_INITM_OVRD          0x4000*2 + 0x008*4
#define PCIe_Phy_PMA_TX_RCVDET_CTRL                   0x4000*2 + 0x020*4
#define PCIe_Phy_PMA_TX_BIST_CTRL                     0x4000*2 + 0x040*4
#define PCIe_Phy_PMA_TX_BIST_UDDWR                    0x4000*2 + 0x041*4
#define PCIe_Phy_PMA_RX_IQPI_ILL_CAL_CTRL             0x8000*2 + 0x020*4
#define PCIe_Phy_PMA_RX_IQPI_ILL_CAL_START            0x8000*2 + 0x021*4
#define PCIe_Phy_PMA_RX_IQPI_ILL_CAL_INIT_TMR         0x8000*2 + 0x023*4
#define PCIe_Phy_PMA_RX_IQPI_ILL_CAL_ITER_TMR         0x8000*2 + 0x024*4
#define PCIe_Phy_PMA_RX_IQPI_ILL_LOCK_CALCNT_START0   0x8000*2 + 0x026*4
#define PCIe_Phy_PMA_RX_EPI_ILL_CAL_CTRL              0x8000*2 + 0x040*4
#define PCIe_Phy_PMA_RX_EPI_ILL_CAL_START             0x8000*2 + 0x041*4
#define PCIe_Phy_PMA_RX_EPI_ILL_CAL_INIT_TMR          0x8000*2 + 0x043*4
#define PCIe_Phy_PMA_RX_EPI_ILL_CAL_ITER_TMR          0x8000*2 + 0x044*4
#define PCIe_Phy_PMA_RX_EPI_ILL_LOCK_CALCNT_START0    0x8000*2 + 0x046*4
#define PCIe_Phy_PMA_RX_CDRLF_CNFG                    0x8000*2 + 0x080*4
#define PCIe_Phy_PMA_RX_CDRLF_CNFG2                   0x8000*2 + 0x081*4
#define PCIe_Phy_PMA_RX_CDRLF_MGN_DIAG                0x8000*2 + 0x082*4
#define PCIe_Phy_PMA_RX_SIGDET_FILT_TMR               0x8000*2 + 0x160*4
#define PCIe_Phy_PMA_RX_SIGDET_DLY_TMR                0x8000*2 + 0x161*4
#define PCIe_Phy_PMA_RX_SIGDET_MIN_TMR                0x8000*2 + 0x162*4


/**
 * @}
 * \defgroup regs_pcie_addr_trans_slv_regs PCIe Address Translator Slave Registers offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_addr_trans_slv_regs
 * @{
 */

#define ADDR_TRANS_SLV_ctrl                     0x0000
#define ADDR_TRANS_SLV_region0_base             0x0004
#define ADDR_TRANS_SLV_region1_base             0x0010
#define ADDR_TRANS_SLV_region2_base             0x001C
#define ADDR_TRANS_SLV_region3_base             0x0028
#define ADDR_TRANS_SLV_region4_base             0x0034
#define ADDR_TRANS_SLV_region5_base             0x0040
#define ADDR_TRANS_SLV_region6_base             0x004C
#define ADDR_TRANS_SLV_region7_base             0x0058
#define ADDR_TRANS_SLV_region8_base             0x0064
#define ADDR_TRANS_SLV_region9_base             0x0070
#define ADDR_TRANS_SLV_region10_base            0x007C
#define ADDR_TRANS_SLV_region11_base            0x0088
#define ADDR_TRANS_SLV_region12_base            0x0094
#define ADDR_TRANS_SLV_region13_base            0x00A0
#define ADDR_TRANS_SLV_region14_base            0x00AC
#define ADDR_TRANS_SLV_region15_base            0x00B8
#define ADDR_TRANS_SLV_region16_base            0x00C4
#define ADDR_TRANS_SLV_region17_base            0x00D0
#define ADDR_TRANS_SLV_region18_base            0x00DC
#define ADDR_TRANS_SLV_region19_base            0x00E8
#define ADDR_TRANS_SLV_region20_base            0x00F4
#define ADDR_TRANS_SLV_region21_base            0x0100
#define ADDR_TRANS_SLV_region22_base            0x010C
#define ADDR_TRANS_SLV_region23_base            0x0118
#define ADDR_TRANS_SLV_region24_base            0x0124
#define ADDR_TRANS_SLV_region25_base            0x0130
#define ADDR_TRANS_SLV_region26_base            0x013C
#define ADDR_TRANS_SLV_region27_base            0x0148
#define ADDR_TRANS_SLV_region28_base            0x0154
#define ADDR_TRANS_SLV_region29_base            0x0160
#define ADDR_TRANS_SLV_region30_base            0x016C
#define ADDR_TRANS_SLV_region31_base            0x0178
#define ADDR_TRANS_SLV_region32_base            0x0184
#define ADDR_TRANS_SLV_region33_base            0x0190
#define ADDR_TRANS_SLV_region34_base            0x019C
#define ADDR_TRANS_SLV_region35_base            0x01A8
#define ADDR_TRANS_SLV_region36_base            0x01B4
#define ADDR_TRANS_SLV_region37_base            0x01C0
#define ADDR_TRANS_SLV_region38_base            0x01CC
#define ADDR_TRANS_SLV_region39_base            0x01D8
#define ADDR_TRANS_SLV_region40_base            0x01E4
#define ADDR_TRANS_SLV_region41_base            0x01F0
#define ADDR_TRANS_SLV_region42_base            0x01FC
#define ADDR_TRANS_SLV_region43_base            0x0208
#define ADDR_TRANS_SLV_region44_base            0x0214
#define ADDR_TRANS_SLV_region45_base            0x0220
#define ADDR_TRANS_SLV_region46_base            0x022C
#define ADDR_TRANS_SLV_region47_base            0x0238
#define ADDR_TRANS_SLV_region48_base            0x0244
#define ADDR_TRANS_SLV_region49_base            0x0250
#define ADDR_TRANS_SLV_region50_base            0x025C
#define ADDR_TRANS_SLV_region51_base            0x0268
#define ADDR_TRANS_SLV_region52_base            0x0274
#define ADDR_TRANS_SLV_region53_base            0x0280
#define ADDR_TRANS_SLV_region54_base            0x028C
#define ADDR_TRANS_SLV_region55_base            0x0298
#define ADDR_TRANS_SLV_region56_base            0x02A4
#define ADDR_TRANS_SLV_region57_base            0x02B0
#define ADDR_TRANS_SLV_region58_base            0x02BC
#define ADDR_TRANS_SLV_region59_base            0x02C8
#define ADDR_TRANS_SLV_region60_base            0x02D4
#define ADDR_TRANS_SLV_region61_base            0x02E0
#define ADDR_TRANS_SLV_region62_base            0x02EC
#define ADDR_TRANS_SLV_region63_base            0x02F8
#define ADDR_TRANS_SLV_region64_base            0x0304
#define ADDR_TRANS_SLV_region65_base            0x0310
#define ADDR_TRANS_SLV_region66_base            0x031C
#define ADDR_TRANS_SLV_region67_base            0x0328
#define ADDR_TRANS_SLV_region68_base            0x0334
#define ADDR_TRANS_SLV_region69_base            0x0340
#define ADDR_TRANS_SLV_region70_base            0x034C
#define ADDR_TRANS_SLV_region71_base            0x0358
#define ADDR_TRANS_SLV_region72_base            0x0364
#define ADDR_TRANS_SLV_region73_base            0x0370
#define ADDR_TRANS_SLV_region74_base            0x037C
#define ADDR_TRANS_SLV_region75_base            0x0388
#define ADDR_TRANS_SLV_region76_base            0x0394
#define ADDR_TRANS_SLV_region77_base            0x03A0
#define ADDR_TRANS_SLV_region78_base            0x03AC
#define ADDR_TRANS_SLV_region79_base            0x03B8
#define ADDR_TRANS_SLV_region80_base            0x03C4
#define ADDR_TRANS_SLV_region81_base            0x03D0
#define ADDR_TRANS_SLV_region82_base            0x03DC
#define ADDR_TRANS_SLV_region83_base            0x03E8
#define ADDR_TRANS_SLV_region84_base            0x03F4
#define ADDR_TRANS_SLV_region85_base            0x0400
#define ADDR_TRANS_SLV_region86_base            0x040C
#define ADDR_TRANS_SLV_region87_base            0x0418
#define ADDR_TRANS_SLV_region88_base            0x0424
#define ADDR_TRANS_SLV_region89_base            0x0430
#define ADDR_TRANS_SLV_region90_base            0x043C
#define ADDR_TRANS_SLV_region91_base            0x0448
#define ADDR_TRANS_SLV_region92_base            0x0454
#define ADDR_TRANS_SLV_region93_base            0x0460
#define ADDR_TRANS_SLV_region94_base            0x046C
#define ADDR_TRANS_SLV_region95_base            0x0478
#define ADDR_TRANS_SLV_region96_base            0x0484
#define ADDR_TRANS_SLV_region97_base            0x0490
#define ADDR_TRANS_SLV_region98_base            0x049C
#define ADDR_TRANS_SLV_region99_base            0x04A8
#define ADDR_TRANS_SLV_region100_base           0x04B4
#define ADDR_TRANS_SLV_region101_base           0x04C0
#define ADDR_TRANS_SLV_region102_base           0x04CC
#define ADDR_TRANS_SLV_region103_base           0x04D8
#define ADDR_TRANS_SLV_region104_base           0x04E4
#define ADDR_TRANS_SLV_region105_base           0x04F0
#define ADDR_TRANS_SLV_region106_base           0x04FC
#define ADDR_TRANS_SLV_region107_base           0x0508
#define ADDR_TRANS_SLV_region108_base           0x0514
#define ADDR_TRANS_SLV_region109_base           0x0520
#define ADDR_TRANS_SLV_region110_base           0x052C
#define ADDR_TRANS_SLV_region111_base           0x0538
#define ADDR_TRANS_SLV_region112_base           0x0544
#define ADDR_TRANS_SLV_region113_base           0x0550
#define ADDR_TRANS_SLV_region114_base           0x055C
#define ADDR_TRANS_SLV_region115_base           0x0568
#define ADDR_TRANS_SLV_region116_base           0x0574
#define ADDR_TRANS_SLV_region117_base           0x0580
#define ADDR_TRANS_SLV_region118_base           0x058C
#define ADDR_TRANS_SLV_region119_base           0x0598
#define ADDR_TRANS_SLV_region120_base           0x05A4
#define ADDR_TRANS_SLV_region121_base           0x05B0
#define ADDR_TRANS_SLV_region122_base           0x05BC
#define ADDR_TRANS_SLV_region123_base           0x05C8
#define ADDR_TRANS_SLV_region124_base           0x05D4
#define ADDR_TRANS_SLV_region125_base           0x05E0
#define ADDR_TRANS_SLV_region126_base           0x05EC
#define ADDR_TRANS_SLV_region127_base           0x05F8
#define ADDR_TRANS_SLV_region0_end              0x0008
#define ADDR_TRANS_SLV_region1_end              0x0014
#define ADDR_TRANS_SLV_region2_end              0x0020
#define ADDR_TRANS_SLV_region3_end              0x002C
#define ADDR_TRANS_SLV_region4_end              0x0038
#define ADDR_TRANS_SLV_region5_end              0x0044
#define ADDR_TRANS_SLV_region6_end              0x0050
#define ADDR_TRANS_SLV_region7_end              0x005C
#define ADDR_TRANS_SLV_region8_end              0x0068
#define ADDR_TRANS_SLV_region9_end              0x0074
#define ADDR_TRANS_SLV_region10_end             0x0080
#define ADDR_TRANS_SLV_region11_end             0x008C
#define ADDR_TRANS_SLV_region12_end             0x0098
#define ADDR_TRANS_SLV_region13_end             0x00A4
#define ADDR_TRANS_SLV_region14_end             0x00B0
#define ADDR_TRANS_SLV_region15_end             0x00BC
#define ADDR_TRANS_SLV_region16_end             0x00C8
#define ADDR_TRANS_SLV_region17_end             0x00D4
#define ADDR_TRANS_SLV_region18_end             0x00E0
#define ADDR_TRANS_SLV_region19_end             0x00EC
#define ADDR_TRANS_SLV_region20_end             0x00F8
#define ADDR_TRANS_SLV_region21_end             0x0104
#define ADDR_TRANS_SLV_region22_end             0x0110
#define ADDR_TRANS_SLV_region23_end             0x011C
#define ADDR_TRANS_SLV_region24_end             0x0128
#define ADDR_TRANS_SLV_region25_end             0x0134
#define ADDR_TRANS_SLV_region26_end             0x0140
#define ADDR_TRANS_SLV_region27_end             0x014C
#define ADDR_TRANS_SLV_region28_end             0x0158
#define ADDR_TRANS_SLV_region29_end             0x0164
#define ADDR_TRANS_SLV_region30_end             0x0170
#define ADDR_TRANS_SLV_region31_end             0x017C
#define ADDR_TRANS_SLV_region32_end             0x0188
#define ADDR_TRANS_SLV_region33_end             0x0194
#define ADDR_TRANS_SLV_region34_end             0x01A0
#define ADDR_TRANS_SLV_region35_end             0x01AC
#define ADDR_TRANS_SLV_region36_end             0x01B8
#define ADDR_TRANS_SLV_region37_end             0x01C4
#define ADDR_TRANS_SLV_region38_end             0x01D0
#define ADDR_TRANS_SLV_region39_end             0x01DC
#define ADDR_TRANS_SLV_region40_end             0x01E8
#define ADDR_TRANS_SLV_region41_end             0x01F4
#define ADDR_TRANS_SLV_region42_end             0x0200
#define ADDR_TRANS_SLV_region43_end             0x020C
#define ADDR_TRANS_SLV_region44_end             0x0218
#define ADDR_TRANS_SLV_region45_end             0x0224
#define ADDR_TRANS_SLV_region46_end             0x0230
#define ADDR_TRANS_SLV_region47_end             0x023C
#define ADDR_TRANS_SLV_region48_end             0x0248
#define ADDR_TRANS_SLV_region49_end             0x0254
#define ADDR_TRANS_SLV_region50_end             0x0260
#define ADDR_TRANS_SLV_region51_end             0x026C
#define ADDR_TRANS_SLV_region52_end             0x0278
#define ADDR_TRANS_SLV_region53_end             0x0284
#define ADDR_TRANS_SLV_region54_end             0x0290
#define ADDR_TRANS_SLV_region55_end             0x029C
#define ADDR_TRANS_SLV_region56_end             0x02A8
#define ADDR_TRANS_SLV_region57_end             0x02B4
#define ADDR_TRANS_SLV_region58_end             0x02C0
#define ADDR_TRANS_SLV_region59_end             0x02CC
#define ADDR_TRANS_SLV_region60_end             0x02D8
#define ADDR_TRANS_SLV_region61_end             0x02E4
#define ADDR_TRANS_SLV_region62_end             0x02F0
#define ADDR_TRANS_SLV_region63_end             0x02FC
#define ADDR_TRANS_SLV_region64_end             0x0308
#define ADDR_TRANS_SLV_region65_end             0x0314
#define ADDR_TRANS_SLV_region66_end             0x0320
#define ADDR_TRANS_SLV_region67_end             0x032C
#define ADDR_TRANS_SLV_region68_end             0x0338
#define ADDR_TRANS_SLV_region69_end             0x0344
#define ADDR_TRANS_SLV_region70_end             0x0350
#define ADDR_TRANS_SLV_region71_end             0x035C
#define ADDR_TRANS_SLV_region72_end             0x0368
#define ADDR_TRANS_SLV_region73_end             0x0374
#define ADDR_TRANS_SLV_region74_end             0x0380
#define ADDR_TRANS_SLV_region75_end             0x038C
#define ADDR_TRANS_SLV_region76_end             0x0398
#define ADDR_TRANS_SLV_region77_end             0x03A4
#define ADDR_TRANS_SLV_region78_end             0x03B0
#define ADDR_TRANS_SLV_region79_end             0x03BC
#define ADDR_TRANS_SLV_region80_end             0x03C8
#define ADDR_TRANS_SLV_region81_end             0x03D4
#define ADDR_TRANS_SLV_region82_end             0x03E0
#define ADDR_TRANS_SLV_region83_end             0x03EC
#define ADDR_TRANS_SLV_region84_end             0x03F8
#define ADDR_TRANS_SLV_region85_end             0x0404
#define ADDR_TRANS_SLV_region86_end             0x0410
#define ADDR_TRANS_SLV_region87_end             0x041C
#define ADDR_TRANS_SLV_region88_end             0x0428
#define ADDR_TRANS_SLV_region89_end             0x0434
#define ADDR_TRANS_SLV_region90_end             0x0440
#define ADDR_TRANS_SLV_region91_end             0x044C
#define ADDR_TRANS_SLV_region92_end             0x0458
#define ADDR_TRANS_SLV_region93_end             0x0464
#define ADDR_TRANS_SLV_region94_end             0x0470
#define ADDR_TRANS_SLV_region95_end             0x047C
#define ADDR_TRANS_SLV_region96_end             0x0488
#define ADDR_TRANS_SLV_region97_end             0x0494
#define ADDR_TRANS_SLV_region98_end             0x04A0
#define ADDR_TRANS_SLV_region99_end             0x04AC
#define ADDR_TRANS_SLV_region100_end            0x04B8
#define ADDR_TRANS_SLV_region101_end            0x04C4
#define ADDR_TRANS_SLV_region102_end            0x04D0
#define ADDR_TRANS_SLV_region103_end            0x04DC
#define ADDR_TRANS_SLV_region104_end            0x04E8
#define ADDR_TRANS_SLV_region105_end            0x04F4
#define ADDR_TRANS_SLV_region106_end            0x0500
#define ADDR_TRANS_SLV_region107_end            0x050C
#define ADDR_TRANS_SLV_region108_end            0x0518
#define ADDR_TRANS_SLV_region109_end            0x0524
#define ADDR_TRANS_SLV_region110_end            0x0530
#define ADDR_TRANS_SLV_region111_end            0x053C
#define ADDR_TRANS_SLV_region112_end            0x0548
#define ADDR_TRANS_SLV_region113_end            0x0554
#define ADDR_TRANS_SLV_region114_end            0x0560
#define ADDR_TRANS_SLV_region115_end            0x056C
#define ADDR_TRANS_SLV_region116_end            0x0578
#define ADDR_TRANS_SLV_region117_end            0x0584
#define ADDR_TRANS_SLV_region118_end            0x0590
#define ADDR_TRANS_SLV_region119_end            0x059C
#define ADDR_TRANS_SLV_region120_end            0x05A8
#define ADDR_TRANS_SLV_region121_end            0x05B4
#define ADDR_TRANS_SLV_region122_end            0x05C0
#define ADDR_TRANS_SLV_region123_end            0x05CC
#define ADDR_TRANS_SLV_region124_end            0x05D8
#define ADDR_TRANS_SLV_region125_end            0x05E4
#define ADDR_TRANS_SLV_region126_end            0x05F0
#define ADDR_TRANS_SLV_region127_end            0x05FC
#define ADDR_TRANS_SLV_region0_tran             0x000C
#define ADDR_TRANS_SLV_region1_tran             0x0018
#define ADDR_TRANS_SLV_region2_tran             0x0024
#define ADDR_TRANS_SLV_region3_tran             0x0030
#define ADDR_TRANS_SLV_region4_tran             0x003C
#define ADDR_TRANS_SLV_region5_tran             0x0048
#define ADDR_TRANS_SLV_region6_tran             0x0054
#define ADDR_TRANS_SLV_region7_tran             0x0060
#define ADDR_TRANS_SLV_region8_tran             0x006C
#define ADDR_TRANS_SLV_region9_tran             0x0078
#define ADDR_TRANS_SLV_region10_tran            0x0084
#define ADDR_TRANS_SLV_region11_tran            0x0090
#define ADDR_TRANS_SLV_region12_tran            0x009C
#define ADDR_TRANS_SLV_region13_tran            0x00A8
#define ADDR_TRANS_SLV_region14_tran            0x00B4
#define ADDR_TRANS_SLV_region15_tran            0x00C0
#define ADDR_TRANS_SLV_region16_tran            0x00CC
#define ADDR_TRANS_SLV_region17_tran            0x00D8
#define ADDR_TRANS_SLV_region18_tran            0x00E4
#define ADDR_TRANS_SLV_region19_tran            0x00F0
#define ADDR_TRANS_SLV_region20_tran            0x00FC
#define ADDR_TRANS_SLV_region21_tran            0x0108
#define ADDR_TRANS_SLV_region22_tran            0x0114
#define ADDR_TRANS_SLV_region23_tran            0x0120
#define ADDR_TRANS_SLV_region24_tran            0x012C
#define ADDR_TRANS_SLV_region25_tran            0x0138
#define ADDR_TRANS_SLV_region26_tran            0x0144
#define ADDR_TRANS_SLV_region27_tran            0x0150
#define ADDR_TRANS_SLV_region28_tran            0x015C
#define ADDR_TRANS_SLV_region29_tran            0x0168
#define ADDR_TRANS_SLV_region30_tran            0x0174
#define ADDR_TRANS_SLV_region31_tran            0x0180
#define ADDR_TRANS_SLV_region32_tran            0x018C
#define ADDR_TRANS_SLV_region33_tran            0x0198
#define ADDR_TRANS_SLV_region34_tran            0x01A4
#define ADDR_TRANS_SLV_region35_tran            0x01B0
#define ADDR_TRANS_SLV_region36_tran            0x01BC
#define ADDR_TRANS_SLV_region37_tran            0x01C8
#define ADDR_TRANS_SLV_region38_tran            0x01D4
#define ADDR_TRANS_SLV_region39_tran            0x01E0
#define ADDR_TRANS_SLV_region40_tran            0x01EC
#define ADDR_TRANS_SLV_region41_tran            0x01F8
#define ADDR_TRANS_SLV_region42_tran            0x0204
#define ADDR_TRANS_SLV_region43_tran            0x0210
#define ADDR_TRANS_SLV_region44_tran            0x021C
#define ADDR_TRANS_SLV_region45_tran            0x0228
#define ADDR_TRANS_SLV_region46_tran            0x0234
#define ADDR_TRANS_SLV_region47_tran            0x0240
#define ADDR_TRANS_SLV_region48_tran            0x024C
#define ADDR_TRANS_SLV_region49_tran            0x0258
#define ADDR_TRANS_SLV_region50_tran            0x0264
#define ADDR_TRANS_SLV_region51_tran            0x0270
#define ADDR_TRANS_SLV_region52_tran            0x027C
#define ADDR_TRANS_SLV_region53_tran            0x0288
#define ADDR_TRANS_SLV_region54_tran            0x0294
#define ADDR_TRANS_SLV_region55_tran            0x02A0
#define ADDR_TRANS_SLV_region56_tran            0x02AC
#define ADDR_TRANS_SLV_region57_tran            0x02B8
#define ADDR_TRANS_SLV_region58_tran            0x02C4
#define ADDR_TRANS_SLV_region59_tran            0x02D0
#define ADDR_TRANS_SLV_region60_tran            0x02DC
#define ADDR_TRANS_SLV_region61_tran            0x02E8
#define ADDR_TRANS_SLV_region62_tran            0x02F4
#define ADDR_TRANS_SLV_region63_tran            0x0300
#define ADDR_TRANS_SLV_region64_tran            0x030C
#define ADDR_TRANS_SLV_region65_tran            0x0318
#define ADDR_TRANS_SLV_region66_tran            0x0324
#define ADDR_TRANS_SLV_region67_tran            0x0330
#define ADDR_TRANS_SLV_region68_tran            0x033C
#define ADDR_TRANS_SLV_region69_tran            0x0348
#define ADDR_TRANS_SLV_region70_tran            0x0354
#define ADDR_TRANS_SLV_region71_tran            0x0360
#define ADDR_TRANS_SLV_region72_tran            0x036C
#define ADDR_TRANS_SLV_region73_tran            0x0378
#define ADDR_TRANS_SLV_region74_tran            0x0384
#define ADDR_TRANS_SLV_region75_tran            0x0390
#define ADDR_TRANS_SLV_region76_tran            0x039C
#define ADDR_TRANS_SLV_region77_tran            0x03A8
#define ADDR_TRANS_SLV_region78_tran            0x03B4
#define ADDR_TRANS_SLV_region79_tran            0x03C0
#define ADDR_TRANS_SLV_region80_tran            0x03CC
#define ADDR_TRANS_SLV_region81_tran            0x03D8
#define ADDR_TRANS_SLV_region82_tran            0x03E4
#define ADDR_TRANS_SLV_region83_tran            0x03F0
#define ADDR_TRANS_SLV_region84_tran            0x03FC
#define ADDR_TRANS_SLV_region85_tran            0x0408
#define ADDR_TRANS_SLV_region86_tran            0x0414
#define ADDR_TRANS_SLV_region87_tran            0x0420
#define ADDR_TRANS_SLV_region88_tran            0x042C
#define ADDR_TRANS_SLV_region89_tran            0x0438
#define ADDR_TRANS_SLV_region90_tran            0x0444
#define ADDR_TRANS_SLV_region91_tran            0x0450
#define ADDR_TRANS_SLV_region92_tran            0x045C
#define ADDR_TRANS_SLV_region93_tran            0x0468
#define ADDR_TRANS_SLV_region94_tran            0x0474
#define ADDR_TRANS_SLV_region95_tran            0x0480
#define ADDR_TRANS_SLV_region96_tran            0x048C
#define ADDR_TRANS_SLV_region97_tran            0x0498
#define ADDR_TRANS_SLV_region98_tran            0x04A4
#define ADDR_TRANS_SLV_region99_tran            0x04B0
#define ADDR_TRANS_SLV_region100_tran           0x04BC
#define ADDR_TRANS_SLV_region101_tran           0x04C8
#define ADDR_TRANS_SLV_region102_tran           0x04D4
#define ADDR_TRANS_SLV_region103_tran           0x04E0
#define ADDR_TRANS_SLV_region104_tran           0x04EC
#define ADDR_TRANS_SLV_region105_tran           0x04F8
#define ADDR_TRANS_SLV_region106_tran           0x0504
#define ADDR_TRANS_SLV_region107_tran           0x0510
#define ADDR_TRANS_SLV_region108_tran           0x051C
#define ADDR_TRANS_SLV_region109_tran           0x0528
#define ADDR_TRANS_SLV_region110_tran           0x0534
#define ADDR_TRANS_SLV_region111_tran           0x0540
#define ADDR_TRANS_SLV_region112_tran           0x054C
#define ADDR_TRANS_SLV_region113_tran           0x0558
#define ADDR_TRANS_SLV_region114_tran           0x0564
#define ADDR_TRANS_SLV_region115_tran           0x0570
#define ADDR_TRANS_SLV_region116_tran           0x057C
#define ADDR_TRANS_SLV_region117_tran           0x0588
#define ADDR_TRANS_SLV_region118_tran           0x0594
#define ADDR_TRANS_SLV_region119_tran           0x05A0
#define ADDR_TRANS_SLV_region120_tran           0x05AC
#define ADDR_TRANS_SLV_region121_tran           0x05B8
#define ADDR_TRANS_SLV_region122_tran           0x05C4
#define ADDR_TRANS_SLV_region123_tran           0x05D0
#define ADDR_TRANS_SLV_region124_tran           0x05DC
#define ADDR_TRANS_SLV_region125_tran           0x05E8
#define ADDR_TRANS_SLV_region126_tran           0x05F4
#define ADDR_TRANS_SLV_region127_tran           0x0600

/**
 * @}
 * \defgroup regs_pcie_addr_trans_mst_regs PCIe Address Translator Master Registers offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_addr_trans_mst_regs
 * @{
 */

#define ADDR_TRANS_MST_ctrl                     0x0000
#define ADDR_TRANS_MST_region0_base             0x0004
#define ADDR_TRANS_MST_region1_base             0x0010
#define ADDR_TRANS_MST_region2_base             0x001C
#define ADDR_TRANS_MST_region3_base             0x0028
#define ADDR_TRANS_MST_region4_base             0x0034
#define ADDR_TRANS_MST_region5_base             0x0040
#define ADDR_TRANS_MST_region6_base             0x004C
#define ADDR_TRANS_MST_region7_base             0x0058
#define ADDR_TRANS_MST_region8_base             0x0064
#define ADDR_TRANS_MST_region0_end              0x0008
#define ADDR_TRANS_MST_region1_end              0x0014
#define ADDR_TRANS_MST_region2_end              0x0020
#define ADDR_TRANS_MST_region3_end              0x002C
#define ADDR_TRANS_MST_region4_end              0x0038
#define ADDR_TRANS_MST_region5_end              0x0044
#define ADDR_TRANS_MST_region6_end              0x0050
#define ADDR_TRANS_MST_region7_end              0x005C
#define ADDR_TRANS_MST_region8_end              0x0068
#define ADDR_TRANS_MST_region0_tran             0x000C
#define ADDR_TRANS_MST_region1_tran             0x0018
#define ADDR_TRANS_MST_region2_tran             0x0024
#define ADDR_TRANS_MST_region3_tran             0x0030
#define ADDR_TRANS_MST_region4_tran             0x003C
#define ADDR_TRANS_MST_region5_tran             0x0048
#define ADDR_TRANS_MST_region6_tran             0x0054
#define ADDR_TRANS_MST_region7_tran             0x0060
#define ADDR_TRANS_MST_region8_tran             0x006C

/**
 * @}
 * \defgroup regs_pcie_ext_irq_gen_regs PCIe External IRQ Generator Registers offsets
 * \ingroup regs_pcie
 *
 * \addtogroup regs_pcie_ext_irq_gen_regs
 * @{
 */

#define EXT_IRQ_GEN_Ctrl                     0x0000
#define EXT_IRQ_GEN_Global_IRQ_Status_l      0x0004
#define EXT_IRQ_GEN_Global_IRQ_Status_h      0x0008
#define EXT_IRQ_GEN_Global_IRQ_Mask_l        0x000C
#define EXT_IRQ_GEN_Global_IRQ_Mask_h        0x0010
#define EXT_IRQ_GEN_Global_IRQ_Map_0         0x0014
#define EXT_IRQ_GEN_Global_IRQ_Map_1         0x0018
#define EXT_IRQ_GEN_Global_IRQ_Map_2         0x001C
#define EXT_IRQ_GEN_Global_IRQ_Map_3         0x0020
#define EXT_IRQ_GEN_Global_IRQ_Map_4         0x0024
#define EXT_IRQ_GEN_Global_IRQ_Map_5         0x0028
#define EXT_IRQ_GEN_Global_IRQ_Map_6         0x002C
#define EXT_IRQ_GEN_Global_IRQ_Map_7         0x0030
#define EXT_IRQ_GEN_Global_IRQ_Map_8         0x0034
#define EXT_IRQ_GEN_Global_IRQ_Map_9         0x0038
#define EXT_IRQ_GEN_Global_IRQ_Map_10        0x003C
#define EXT_IRQ_GEN_Global_IRQ_Map_11        0x0040
#define EXT_IRQ_GEN_Global_IRQ_Map_12        0x0044
#define EXT_IRQ_GEN_Global_IRQ_Map_13        0x0048
#define EXT_IRQ_GEN_Global_IRQ_Map_14        0x004C
#define EXT_IRQ_GEN_Global_IRQ_Map_15        0x0050
#define EXT_IRQ_GEN_v0_Message_Address       0x0800
#define EXT_IRQ_GEN_v0_Message_Upper_Address 0x0804
#define EXT_IRQ_GEN_v0_Message_Data          0x0808
#define EXT_IRQ_GEN_v0_Vector_Control        0x080C
#define EXT_IRQ_GEN_Global_PBA_Entries_l     0x0C00
#define EXT_IRQ_GEN_Global_PBA_Entries_h     0x0C04
#define EXT_IRQ_GEN_Global_AXI_param         0x0C08

/**
 * @}
 * \defgroup TODO
 * \ingroup regs_pcie
 *
 * \addtogroup TODO
 * @{
 */
 
/**
 * @}
 */

#endif /* end of include guard: REGS_PCIE_H */
