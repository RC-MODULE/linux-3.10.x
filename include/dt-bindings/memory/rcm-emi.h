/*
 * RCM EMI device tree definitions
 *
 * Copyright (C) 2020 MIR
 *	Mikhail.Petrov@mir.dev
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef DT_BINDINGS_RCM_EMI_H_INCLUDED
#define DT_BINDINGS_RCM_EMI_H_INCLUDED

#define RCM_EMI_SSx_BTYP_SRAM (0 << 0)
#define RCM_EMI_SSx_BTYP_NOR (1 << 0)
#define RCM_EMI_SSx_BTYP_SSRAM (2 << 0)
#define RCM_EMI_SSx_BTYP_PIPERDY (4 << 0)
#define RCM_EMI_SSx_BTYP_SDRAM (6 << 0)
#define RCM_EMI_SSx_BTYP_MASK (7 << 0)

#define RCM_EMI_SSx_PTYP_NO_PAGES (0 << 3)
#define RCM_EMI_SSx_PTYP_USE_PAGES (1 << 3)

#define RCM_EMI_SSx_SRDY_EXT_RDY_NOT_USE (0 << 4)
#define RCM_EMI_SSx_SRDY_EXT_RDY_USE (1 << 4)

#define RCM_EMI_SSx_TWR_0 (0 << 5)
#define RCM_EMI_SSx_TWR_1 (1 << 5)

#define RCM_EMI_SSx_SST_FLOW_THROUGH (0 << 6)
#define RCM_EMI_SSx_SST_FLOW_PIPELINED (1 << 6)

#define RCM_EMI_SSx_TSSOE_1 (0 << 7)
#define RCM_EMI_SSx_TSSOE_2 (1 << 7)

#define RCM_EMI_SSx_TSOE_1 (0 << 8)
#define RCM_EMI_SSx_TSOE_2 (1 << 8)

#define RCM_EMI_SSx_TCYC_2 (15 << 9)
#define RCM_EMI_SSx_TCYC_3 (14 << 9)
#define RCM_EMI_SSx_TCYC_4 (13 << 9)
#define RCM_EMI_SSx_TCYC_5 (12 << 9)
#define RCM_EMI_SSx_TCYC_6 (11 << 9)
#define RCM_EMI_SSx_TCYC_7 (10 << 9)
#define RCM_EMI_SSx_TCYC_8 (9 << 9)
#define RCM_EMI_SSx_TCYC_9 (8 << 9)
#define RCM_EMI_SSx_TCYC_10 (7 << 9)
#define RCM_EMI_SSx_TCYC_11 (6 << 9)
#define RCM_EMI_SSx_TCYC_12 (5 << 9)
#define RCM_EMI_SSx_TCYC_13 (4 << 9)
#define RCM_EMI_SSx_TCYC_14 (3 << 9)
#define RCM_EMI_SSx_TCYC_15 (2 << 9)
#define RCM_EMI_SSx_TCYC_16 (1 << 9)
#define RCM_EMI_SSx_TCYC_17 (0 << 9)

#define RCM_EMI_SSx_RDY(x) (x << 13)

#define RCM_EMI_SSx_TDEL_0 (0 << 23)
#define RCM_EMI_SSx_TDEL_1 (3 << 23)
#define RCM_EMI_SSx_TDEL_2 (2 << 23)
#define RCM_EMI_SSx_TDEL_3 (1 << 23)


#define RCM_EMI_SDx_CSP_256 (0 << 1)
#define RCM_EMI_SDx_CSP_512 (1 << 1)
#define RCM_EMI_SDx_CSP_1024 (3 << 1)
#define RCM_EMI_SDx_CSP_2048 (5 << 1)
#define RCM_EMI_SDx_CSP_4096 (7 << 1)

#define RCM_EMI_SDx_SDS_2M (0 << 4)
#define RCM_EMI_SDx_SDS_4M (1 << 4)
#define RCM_EMI_SDx_SDS_8M (2 << 4)
#define RCM_EMI_SDx_SDS_16M (3 << 4)
#define RCM_EMI_SDx_SDS_32M (4 << 4)
#define RCM_EMI_SDx_SDS_64M (5 << 4)
#define RCM_EMI_SDx_SDS_128M (6 << 4)
#define RCM_EMI_SDx_SDS_256M (7 << 4)

#define RCM_EMI_SDx_CL_1 (2 << 7)
#define RCM_EMI_SDx_CL_2 (1 << 7)
#define RCM_EMI_SDx_CL_3 (0 << 7)

#define RCM_EMI_SDx_TRDL_1 (0 << 9)
#define RCM_EMI_SDx_TRDL_2 (1 << 9)

#define RCM_EMI_SDx_SI_EXT_INIT (0 << 10)
#define RCM_EMI_SDx_SI_CPU_INIT (1 << 10)

#define RCM_EMI_SDx_T_RCD_2 (3 << 11)
#define RCM_EMI_SDx_T_RCD_3 (2 << 11)
#define RCM_EMI_SDx_T_RCD_4 (1 << 11)
#define RCM_EMI_SDx_T_RCD_5 (0 << 11)

#define RCM_EMI_SDx_T_RAS_4 (5 << 13)
#define RCM_EMI_SDx_T_RAS_5 (4 << 13)
#define RCM_EMI_SDx_T_RAS_6 (3 << 13)
#define RCM_EMI_SDx_T_RAS_7 (2 << 13)
#define RCM_EMI_SDx_T_RAS_8 (1 << 13)
#define RCM_EMI_SDx_T_RAS_9 (0 << 13)
#define RCM_EMI_SDx_T_RAS_10 (7 << 13)
#define RCM_EMI_SDx_T_RAS_11 (6 << 13)


#define RCM_EMI_RFC_TRFC_6 (3 << 1)
#define RCM_EMI_RFC_TRFC_7 (2 << 1)
#define RCM_EMI_RFC_TRFC_8 (1 << 1)
#define RCM_EMI_RFC_TRFC_9 (0 << 1)
#define RCM_EMI_RFC_TRFC_10 (7 << 1)
#define RCM_EMI_RFC_TRFC_11 (6 << 1)
#define RCM_EMI_RFC_TRFC_12 (5 << 1)
#define RCM_EMI_RFC_TRFC_13 (4 << 1)

#define RCM_EMI_RFC_RP(x) (x << 4)


/* ??? #define RCM_EMI_HSTSR_HEN_0 (1 << 0)
#define RCM_EMI_HSTSR_HEN_1 (1 << 1)
#define RCM_EMI_HSTSR_HEN_2 (1 << 2)
#define RCM_EMI_HSTSR_HEN_3 (1 << 3)
#define RCM_EMI_HSTSR_HEN_4 (1 << 4)
#define RCM_EMI_HSTSR_HEN_5 (1 << 5)*/

#endif // !DT_BINDINGS_RCM_EMI_H_INCLUDED
