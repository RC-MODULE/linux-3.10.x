// SPDX-License-Identifier: GPL-2.0+
/*
 * rcmodule-cpufreq.c - RC-Module PPC470 Core frequency driver.
 *
 * Copyright (C) 2018 by AstroSoft
 * Alexey Spirkov <alexeis@astrosoft.ru>
 */

#undef DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/of_device.h>
#include <linux/io.h>


#define DBG(fmt...) pr_debug(fmt)

#define CPUFREQ_PLLSTATE_OFFSET 0x000
#define CPUFREQ_PLLCTRL_OFFSET 0x004
#define CPUFREQ_PLLDIV_OFFSET 0x008
#define CPUFREQ_WRLOCK_OFFSET 0x00C
#define CPUFREQ_WRUNLOCK 0x1ACCE551


static struct cpufreq_frequency_table rcmodule_cpu_freqs[] = {
	{0, 0x03010050, 250000},
	{0, 0x02010050, 500000},
	{0, 0x0101003C, 750000},
//	{0, 0x01010048, 900000},		// todo enable on actual HW and extend with required values
//	{0, 0x01010050, 1000000},
	{0, 0, CPUFREQ_TABLE_END},
};


struct rcmodule_cpufreq_data {
	void __iomem *regs;
	int curr_mode;
};

static int rcmodule_switch_cpufreq(struct rcmodule_cpufreq_data* drv_data, unsigned int freqkHz, unsigned int hwvalue)
{
	unsigned long flags;

	local_irq_save(flags);

	writel(CPUFREQ_WRUNLOCK, drv_data->regs + CPUFREQ_WRLOCK_OFFSET);
	writel(hwvalue, drv_data->regs + CPUFREQ_PLLDIV_OFFSET);
	writel(0x1, drv_data->regs + CPUFREQ_PLLCTRL_OFFSET);

	// wait while PLL will be stabilized enough 
	// just estimated behavior - no manual for a while
	while(!(readl(drv_data->regs + CPUFREQ_PLLSTATE_OFFSET) & 0x1));

	local_irq_restore(flags);
	ppc_proc_freq = freqkHz*1000;

	return 0;
}
static struct cpufreq_driver rcmodule_cpufreq_driver;

static int rcmodule_cpufreq_target(struct cpufreq_policy *policy,
	unsigned int index)
{
	int res;
	struct rcmodule_cpufreq_data* data = rcmodule_cpufreq_driver.driver_data;
	res = rcmodule_switch_cpufreq(data, rcmodule_cpu_freqs[index].frequency, rcmodule_cpu_freqs[index].driver_data);
	if(!res)
		// just for get function...
		data->curr_mode = index;
	return res;
}

static int rcmodule_cpufreq_init(struct cpufreq_policy *policy)
{
	return cpufreq_generic_init(policy, rcmodule_cpu_freqs, 12000);		// to check latency
}

static unsigned int rcmodule_get_cpu_freq(unsigned int cpu)
{
	struct rcmodule_cpufreq_data* data = rcmodule_cpufreq_driver.driver_data;
	return rcmodule_cpu_freqs[data->curr_mode].frequency;
}

static struct cpufreq_driver rcmodule_cpufreq_driver = {
	.name		= "rcmodule",
	.flags		= CPUFREQ_CONST_LOOPS,
	.init		= rcmodule_cpufreq_init,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= rcmodule_cpufreq_target,
	.get		= rcmodule_get_cpu_freq,
	.attr		= cpufreq_generic_attr,
};


static int rcmodule_cpufreq_probe(struct platform_device *pdev)
{
	struct rcmodule_cpufreq_data *data;
	struct resource *res;
	struct device_node *cpunode;
	int rc = -ENODEV, index, i;
	unsigned int min, max;
	const u32 *valp;
	struct cpufreq_frequency_table* table;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->regs)) {
		pr_err("Can't find registers information for CPU frequency driver\n");
		goto err0;
	}

	/* Get first CPU node */
	cpunode = of_cpu_device_node_get(0);
	if (cpunode == NULL) {
		pr_err("Can't find any CPU 0 node\n");
		goto err1;
	}

	valp = of_get_property(cpunode, "clock-frequency", NULL);
	if (!valp)
	{
		pr_err("Can't find desired frequency for CPU 0 node\n");
		goto err1;
	}

	index = -1;
	min = 0xffffffff;
	max = 0;
	// find desired index
	for(i = 0, table = rcmodule_cpu_freqs; table->frequency != CPUFREQ_TABLE_END; table++, i++)
	{
		if(table->frequency == (*valp)/1000)
			index = i;
		if(table->frequency < min)
			min = table->frequency;
		if(table->frequency > max)
			max = table->frequency;
	}
	if(index == -1)	
	{
		index = 0;
		pr_warn("Requested CPU frequency not found set to lowest");
	}
	pr_info("Registering RC Module PPC 470s CPU frequency driver\n");
	pr_info("Low: %d Mhz, High: %d Mhz, Cur: %d MHz\n",
		min/1000,
		max/1000,
		rcmodule_cpu_freqs[index].frequency/1000);

	rcmodule_switch_cpufreq(data, rcmodule_cpu_freqs[index].frequency, rcmodule_cpu_freqs[index].driver_data);
	data->curr_mode = index;

	rcmodule_cpufreq_driver.driver_data = data;

	rc = cpufreq_register_driver(&rcmodule_cpufreq_driver);

	of_node_put(cpunode);

	return rc;

err1:
	devm_iounmap(&pdev->dev, data->regs);

err0:
	of_node_put(cpunode);

	return rc;
}

static int rcmodule_cpufreq_remove(struct platform_device *pdev)
{
	struct rcmodule_cpufreq_data *data = platform_get_drvdata(pdev);
	cpufreq_unregister_driver(&rcmodule_cpufreq_driver);
	devm_iounmap(&pdev->dev, data->regs);
	return 0;
}

static const struct of_device_id rcmodule_cpufreq_of_match[] = {
	{ .compatible = "rc-module,cpu-freq", },
	{ }
};
MODULE_DEVICE_TABLE(of, rcmodule_cpufreq_of_match);

static struct platform_driver rcmodule_cpufreq_platform_driver = {
	.driver = {
		.name = "rcmodule-cpufreq",
		.of_match_table = rcmodule_cpufreq_of_match,
	},
	.probe = rcmodule_cpufreq_probe,
	.remove = rcmodule_cpufreq_remove,
};
module_platform_driver(rcmodule_cpufreq_platform_driver);

MODULE_DESCRIPTION("RC-Module PPC 470 core frequency driver");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_LICENSE("GPL v2");
