/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/clk-provider.h>
#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <asm/mach/arch.h>
#include <asm/system_misc.h> 

static void rcm_pm_idle(void)
{
	asm volatile (
		"nop\n\t"
		"nop\n\t");
}

static void __init rcm_dt_timer_init(void)
{
	of_clk_init(NULL);
	timer_probe();
}

static const char *rcm_1879vm8ya_dt_match[] __initconst = {
	"rcm,1879VM8YA",
	NULL
};

static int __init rcm_init(void)
{
	arm_pm_idle = rcm_pm_idle;
	return 0;
}

arch_initcall(rcm_init); 

DT_MACHINE_START(RCM_1879VM8YA, "RCM 1879VM8YA Module")
	//.init_time      = timer_probe,
	.init_time      = rcm_dt_timer_init,
	.l2c_aux_mask   = 0xffffffff,
	.l2c_aux_val    =  0x00000000,
	.dt_compat      = rcm_1879vm8ya_dt_match,
MACHINE_END

