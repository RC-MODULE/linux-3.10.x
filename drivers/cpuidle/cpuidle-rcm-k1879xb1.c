/*
 * CPU idle for RC Module K1879XB1YA SoC
 *
 * Copyright (C) 2015 RC Module.
 * http://www.module.ru/
 *
 * Andrew Andrianov <andrew@ncrmnt.org>
 *
 * Based on Davinci CPU idle code
 * (arch/arm/mach-davinci/cpuidle.c)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <asm/cpuidle.h>

/*
 * We only enable WFI here, since DDR will enter self-refresh
 * on it's own when it can (bootloader takes care to configure that)
 */

static struct cpuidle_driver rcm_idle_driver = {
	.name			= "rcm_k1879xb1_idle",
	.states[0]		= ARM_CPUIDLE_WFI_STATE,
	.state_count		= 1,
};

static int __init rcm_cpuidle_probe(struct platform_device *pdev)
{
	return cpuidle_register(&rcm_idle_driver, NULL);
}

static struct platform_driver rcm_cpuidle_driver = {
	.driver = {
		.name	= "cpuidle-rcm-k1879xb1",
	},
};

static int __init rcm_cpuidle_init(void)
{
	return platform_driver_probe(&rcm_cpuidle_driver, rcm_cpuidle_probe);
}
device_initcall(rcm_cpuidle_init);
