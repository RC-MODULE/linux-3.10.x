/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#ifndef _RCM_I2S_H_
#define _RCM_I2S_H_

#define RCM_I2S_REG_CTRL0	0x00
#define RCM_I2S_REG_CTRL1	0x08
#define RCM_I2S_REG_STAT0	0x10
#define RCM_I2S_REG_STAT1	0x18
#define RCM_I2S_REG_FIFO0	0x20
#define RCM_I2S_REG_FIFO1	0x28
#define RCM_I2S_REG_FIFO2	0x30
#define RCM_I2S_REG_FIFO3	0x38

struct rcm_i2s_control {
	spinlock_t lock;
    struct resource *res;
	struct regmap *regmap;
    int started_count;
    int opened_count;
    int hw_params[2];
    int (*rcm_i2s_reset)(struct rcm_i2s_control *i2s);
};

#endif