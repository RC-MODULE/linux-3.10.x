/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

#define CRG_VDU_PLL_STATE 0x000
#define CRG_VDU_PLL_CTRL 0x004
#define CRG_VDU_WR_LOCK 0x00C
#define CRG_VDU_RST_MON 0x010
#define CRG_VDU_RST_CTRL 0x014
#define CRG_VDU_RST_EN 0x018
#define CRG_VDU_UPD_CK 0x01C
#define CRG_VDU_CKALIGN 0x020
#define CRG_VDU_CKDIVSTAT 0x024
#define CRG_VDU_PLL_PRDIV 0x030
#define CRG_VDU_PLL_FBDIV 0x034
#define CRG_VDU_PLL_PSDIV 0x038

#define CRG_VDU_CKDIVMODE_VOUT0 0x100
#define CRG_VDU_CKEN_VOUT0 0x104

#define CRG_VDU_CKDIVMODE_VOUT1 0x110
#define CRG_VDU_CKEN_VOUT1 0x114

#define CRG_VDU_CKDIVMODE_CODEC 0x120
#define CRG_VDU_CKEN_CODEC 0x124

#define CRG_VDU_PLL_STATE_PLL_RDY (1U << 0)

#define CRG_VDU_PLL_CTRL_PLL_RESTART (1U << 0)
#define CRG_VDU_PLL_CTRL_PLL_ENABLE (1U << 1)
#define CRG_VDU_PLL_CTRL_PLL_BYPASS (1U << 4)

#define CRG_VDU_WR_LOCK_LOCKED 0x00000001
#define CRG_VDU_WR_LOCK_UNLOCKED 0x00000000
#define CRG_VDU_WR_LOCK_LOCK 0x00000001
#define CRG_VDU_WR_LOCK_UNLOCK 0x1ACCE551

#define CRG_VDU_UPD_CK_UPD_CKDIV (1U << 0)
