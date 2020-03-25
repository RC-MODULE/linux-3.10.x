// SPDX-License-Identifier: GPL-2.0+
/*
 * rcm-cpufreq.c - RCM PPC470 Core frequency driver.
 *
 * Copyright (C) 2019 by AstroSoft
 * Alexey Spirkov <dev@alsp.net>
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
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define DBG(fmt...) pr_debug(fmt)

#define CPUFREQ_PLLSTATE_OFFSET 0x000
#define CPUFREQ_PLLCTRL_OFFSET 0x004
#define CPUFREQ_PRDIV_OFFSET 0x0030
#define CPUFREQ_FBDIV_OFFSET 0x0034
#define CPUFREQ_PSDIV_OFFSET 0x0038
#define CPUFREQ_CKDIVMODE_CPU_OFFSET 0x0100
#define CPUFREQ_WRLOCK_OFFSET 0x00C
#define CPUFREQ_WRUNLOCK 0x1ACCE551

struct rcm_freq_mode {
	unsigned int freq_khz;
	unsigned int fbdiv; // 8..511
	unsigned int prediv; // 1..31
	unsigned int postdiv; // 2'b00 - 1; 2'b01 - 2; 2'b10 - 4; 2'b11 - 8
	unsigned int cpudiv; // 0 - 255; Fclk = Fpll/(cpudiv+1)
};

struct rcm_cpufreq_data {
	struct regmap *control;
	struct cpufreq_frequency_table *cpu_freqs;
	struct rcm_freq_mode *cpu_modes;
	int curr_mode;
};

static int rcm_switch_cpufreq(struct rcm_cpufreq_data *drv_data,
				   unsigned int index)
{
	unsigned long flags;
	unsigned int val;
	int retval = 0;
	unsigned int fbdiv, prdiv, postdiv, cpudiv;

	// first check that we really need to change frequency
	if(regmap_read(drv_data->control, CPUFREQ_FBDIV_OFFSET, &fbdiv))
	{
		pr_err("Unable to access control regmap\n");
		return 2;
	}
	regmap_read(drv_data->control, CPUFREQ_PRDIV_OFFSET, &prdiv);
	regmap_read(drv_data->control, CPUFREQ_PSDIV_OFFSET, &postdiv);
	regmap_read(drv_data->control, CPUFREQ_CKDIVMODE_CPU_OFFSET, &cpudiv);

	if (drv_data->cpu_modes[index].fbdiv != fbdiv ||
	    drv_data->cpu_modes[index].prediv != prdiv ||
	    drv_data->cpu_modes[index].postdiv != postdiv ||
	    drv_data->cpu_modes[index].cpudiv != cpudiv) {
		local_irq_save(flags);

		if (!regmap_write(drv_data->control, CPUFREQ_WRLOCK_OFFSET,
				  CPUFREQ_WRUNLOCK)) // unlock
		{
			// write new data to PLL control
			regmap_write(drv_data->control, CPUFREQ_FBDIV_OFFSET,
				     drv_data->cpu_modes[index].fbdiv);
			regmap_write(drv_data->control, CPUFREQ_PRDIV_OFFSET,
				     drv_data->cpu_modes[index].prediv);
			regmap_write(drv_data->control, CPUFREQ_PSDIV_OFFSET,
				     drv_data->cpu_modes[index].postdiv);
			regmap_write(drv_data->control,
				     CPUFREQ_CKDIVMODE_CPU_OFFSET,
				     drv_data->cpu_modes[index].cpudiv);

			regmap_update_bits(drv_data->control,
					   CPUFREQ_PLLCTRL_OFFSET, BIT(0),
					   1); // restart PLL

			regmap_write(drv_data->control, CPUFREQ_WRLOCK_OFFSET,
				     0); // lock back

			// wait while PLL will be stabilized enough
			// just estimated behavior - no manual for a while
			do {
				if (regmap_read(drv_data->control,
						CPUFREQ_PLLSTATE_OFFSET,
						&val)) {
					pr_err("Unable to access control regmap\n");
					retval = 2;
				}
			} while (!(val & 0x1));

			if (!retval)
				ppc_proc_freq =
					drv_data->cpu_modes[index].freq_khz *
					1000;

		} else {
			pr_err("Unable to access control regmap\n");
			retval = 1;
		}

		local_irq_restore(flags);
	}

	return retval;
}

static struct cpufreq_driver rcm_cpufreq_driver;

static int rcm_cpufreq_target(struct cpufreq_policy *policy,
				   unsigned int index)
{
	int res;
	struct rcm_cpufreq_data *data =
		rcm_cpufreq_driver.driver_data;
	res = rcm_switch_cpufreq(data, data->cpu_freqs[index].driver_data);
	if (!res)
		// just for get function...
		data->curr_mode = index;
	return res;
}

static int rcm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct rcm_cpufreq_data *data =
		rcm_cpufreq_driver.driver_data;
	cpufreq_generic_init(policy, data->cpu_freqs,
					12000); // to check latency
	return 0;
}

static unsigned int rcm_get_cpu_freq(unsigned int cpu)
{
	struct rcm_cpufreq_data *data =
		rcm_cpufreq_driver.driver_data;
	return data->cpu_modes[data->curr_mode].freq_khz;
}

static struct cpufreq_driver rcm_cpufreq_driver = {
	.name = "rcm-cpufreq",
	.flags = CPUFREQ_CONST_LOOPS,
	.init = rcm_cpufreq_init,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = rcm_cpufreq_target,
	.get = rcm_get_cpu_freq,
	.attr = cpufreq_generic_attr,
};

static int rcm_cpufreq_probe(struct platform_device *pdev)
{
	struct rcm_cpufreq_data *data;
	struct device_node *cpunode, *tmp;
	int rc = -ENODEV, index, i;
	unsigned int min, max;
	const u32 *valp;
	struct cpufreq_frequency_table *table;
	struct property *prop;
	int sz, count;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	prop = of_find_property(pdev->dev.of_node, "freqs", &sz);

	if (!prop) {
		pr_err("failed to find frequency table\n");
		goto err0;
	}
	count = sz / sizeof(struct rcm_freq_mode);

	data->cpu_modes = devm_kzalloc(&pdev->dev, sz, GFP_KERNEL);
	data->cpu_freqs = devm_kzalloc(
		&pdev->dev,
		(count + 1) * sizeof(struct cpufreq_frequency_table),
		GFP_KERNEL);

	if (of_property_read_u32_array(pdev->dev.of_node, "freqs",
				       (u32 *)data->cpu_modes,
				       sz / sizeof(unsigned int))) {
		pr_err("failed to read frequency table\n");
		goto err0;
	}

	for (i = 0; i < count; i++) {
		data->cpu_freqs[i].flags = 0;
		data->cpu_freqs[i].driver_data = i;
		data->cpu_freqs[i].frequency = data->cpu_modes[i].freq_khz;
	}
	data->cpu_freqs[i].flags = 0;
	data->cpu_freqs[i].driver_data = 0;
	data->cpu_freqs[i].frequency = CPUFREQ_TABLE_END;

	tmp = of_parse_phandle(pdev->dev.of_node, "control", 0);
	if (!tmp) {
		pr_err("failed to find control register reference\n");
		goto err0;
	}
	data->control = syscon_node_to_regmap(tmp);

	/* Get first CPU node */
	cpunode = of_cpu_device_node_get(0);
	if (cpunode == NULL) {
		pr_err("Can't find any CPU 0 node\n");
		goto err0;
	}

	valp = of_get_property(cpunode, "clock-frequency", NULL);
	if (!valp) {
		pr_err("Can't find desired frequency for CPU 0 node\n");
		goto err0;
	}

	index = -1;
	min = 0xffffffff;
	max = 0;
	// find desired index
	for (i = 0, table = data->cpu_freqs;
	     table->frequency != CPUFREQ_TABLE_END; table++, i++) {
		if (table->frequency == (*valp) / 1000)
			index = i;
		if (table->frequency < min)
			min = table->frequency;
		if (table->frequency > max)
			max = table->frequency;
	}
	if (index == -1) {
		index = 0;
		pr_warn("Requested CPU frequency not found set to lowest");
	}
	pr_info("Registering RC Module PPC 470s CPU frequency driver\n");
	pr_info("Low: %d Mhz, High: %d Mhz, Cur: %d MHz\n", min / 1000,
		max / 1000, data->cpu_freqs[index].frequency / 1000);

	rcm_switch_cpufreq(data, data->cpu_freqs[index].driver_data);
	data->curr_mode = index;

	rcm_cpufreq_driver.driver_data = data;

	rc = cpufreq_register_driver(&rcm_cpufreq_driver);

	of_node_put(cpunode);

	return rc;

err0:
	of_node_put(cpunode);

	return rc;
}

static int rcm_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&rcm_cpufreq_driver);
	return 0;
}

static const struct of_device_id rcm_cpufreq_of_match[] = {
	{
		.compatible = "rcm,cpu-freq",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_cpufreq_of_match);

static struct platform_driver rcm_cpufreq_platform_driver = {
	.driver =
		{
			.name = "rcm-cpufreq",
			.of_match_table = rcm_cpufreq_of_match,
		},
	.probe = rcm_cpufreq_probe,
	.remove = rcm_cpufreq_remove,
};
module_platform_driver(rcm_cpufreq_platform_driver);

MODULE_DESCRIPTION("RCM PPC 470 core frequency driver");
MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_LICENSE("GPL v2");
