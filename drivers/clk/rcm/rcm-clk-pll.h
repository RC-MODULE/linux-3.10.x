/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#ifndef _RCM_CLK_PLL_H
#define _RCM_CLK_PLL_H

struct rcm_pll_freq_mode {
	unsigned int fbdiv; // 8..511
	unsigned int prdiv; // 1..31
	unsigned int psdiv; // 2'b00 - 1; 2'b01 - 2; 2'b10 - 4; 2'b11 - 8
};

struct rcm_pll {
	struct clk_hw hw;
	spinlock_t lock;
	struct regmap *regmap;
	struct device *dev;
	struct rcm_pll_freq_mode *freqs;
	int freq_num;
	unsigned long current_freq;
};

#define RCM_PLL_STATE 0x0000
#define RCM_PLL_CTRL 0x0004
#define RCM_PLL_WR_LOCK 0x000C
#define RCM_PLL_RST_MON 0x0010
#define RCM_PLL_RST_CTRL 0x0014
#define RCM_PLL_RST_EN 0x0018
#define RCM_PLL_UPD_CK 0x001C
#define RCM_PLL_CKALIGN 0x0020
#define RCM_PLL_CKDIVSTAT 0x0024
#define RCM_PLL_PRDIV 0x0030
#define RCM_PLL_FBDIV 0x0034
#define RCM_PLL_PSDIV 0x0038
#define RCM_PLL_DIVMODE0 0x0100
#define RCM_PLL_CKEN0 0x0104
#define RCM_PLL_DIVMODEX 0x0190
#define RCM_PLL_CKENX 0x0194

#define RCM_PLL_WRUNLOCK 0x1ACCE551

// PLL stabilizing timeout uS
#define RCM_PLL_STABILITY_TIMEOUT 500

#endif