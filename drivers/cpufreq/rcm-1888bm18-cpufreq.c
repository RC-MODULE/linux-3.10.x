/* RCM 1888BM18 Core frequency driver
 *
 * Copyright (C) 2019 by AstroSoft
 *	Alexey Spirkov <dev@alsp.net>
 *
 * Copyright (C) 2020 MIR
 *	Mikhail.Petrov@mir.dev
 * 
 * SPDX-License-Identifier: GPL-2.0+
 */

#undef DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/of_device.h>
#include <asm/dcr.h>

#define CPUFREQ_WRLOCK 0x03C
#define CPUFREQ_CKUPDATE 0x0060
#define CPUFREQ_CKDIVMODE_CPU 0x0100

#define CPUFREQ_WRLOCK_LOCKED 0x00000001
#define CPUFREQ_WRLOCK_UNLOCKED 0x00000000
#define CPUFREQ_WRLOCK_WRUNLOCK 0x1ACCE551

#define CPUFREQ_CKUPDATE_UPDCKDIV_MASK (1 << 0)

struct rcm_freq_data {
	u32 freq_khz;
	u32 divider;
};

struct rcm_cpufreq_data {
	struct cpufreq_frequency_table *freq_table;
	struct rcm_freq_data *freq_data;
	int curr_freq_index;
	bool dcr_host_mapped;
	dcr_host_t dcr_host;
	unsigned dcr_n;
	unsigned dcr_c;
	bool cpufreq_driver_registered;
};

static int rcm_cpufreq_init(struct cpufreq_policy *policy);
static int rcm_cpufreq_target_index(struct cpufreq_policy *policy, unsigned int index);
static unsigned int rcm_cpufreq_get(unsigned int cpu);

static struct cpufreq_driver rcm_cpufreq_driver = {
	.name = "rcm-cpufreq",
	.flags = CPUFREQ_CONST_LOOPS,
	.init = rcm_cpufreq_init,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = rcm_cpufreq_target_index,
	.get = rcm_cpufreq_get,
	.attr = cpufreq_generic_attr
};

static int set_cpu_divider_internal(struct rcm_cpufreq_data *data, u32 divider)
{
	u32 old_value;
	u32 new_value;
	u32 update;
	unsigned long timeout;

	old_value = dcr_read(data->dcr_host, CPUFREQ_CKDIVMODE_CPU);
	pr_debug("previous divider value = 0x%08X\n", old_value);

	new_value = divider - 1;
	if (new_value != old_value) {
		dcr_write(data->dcr_host, CPUFREQ_CKDIVMODE_CPU, new_value);
		pr_debug("new divider value = 0x%08X\n", dcr_read(data->dcr_host, CPUFREQ_CKDIVMODE_CPU));
		dcr_write(data->dcr_host, CPUFREQ_CKUPDATE, CPUFREQ_CKUPDATE_UPDCKDIV_MASK);

		// wait until the PLL controller is ready
		timeout = jiffies + HZ / 10;
		while (true) {
			update = dcr_read(data->dcr_host, CPUFREQ_CKUPDATE);
			if ((update & CPUFREQ_CKUPDATE_UPDCKDIV_MASK) == 0)
				break;
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
		}
	}

	return 0;
}

static int set_cpu_divider(struct rcm_cpufreq_data *data, u32 divider)
{
	u32 locked;
	int ret;

	// unlock the PLL controller if it is locked
	locked = dcr_read(data->dcr_host, CPUFREQ_WRLOCK);
	pr_debug("locked = %u\n", locked);
	if (locked == CPUFREQ_WRLOCK_LOCKED)
		dcr_write(data->dcr_host, CPUFREQ_WRLOCK, CPUFREQ_WRLOCK_WRUNLOCK);

	ret = set_cpu_divider_internal(data, divider);

	// lock the PLL controller if needed
	if (locked == CPUFREQ_WRLOCK_LOCKED)
		dcr_write(data->dcr_host, CPUFREQ_WRLOCK, CPUFREQ_WRLOCK_LOCKED);

	return ret;
}

static int rcm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct rcm_cpufreq_data *data = rcm_cpufreq_driver.driver_data;
	cpufreq_generic_init(policy, data->freq_table, 100000 /* 100 us*/);

	return 0;
}

static int rcm_cpufreq_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	struct rcm_cpufreq_data *data = rcm_cpufreq_driver.driver_data;
	int ret;

	ret = set_cpu_divider(data, data->freq_data[index].divider);
	if (ret != 0) {
		pr_err("cannot switch to the new CPU frequency\n");
		return ret;
	}

	data->curr_freq_index = index;

	return 0;
}

static unsigned int rcm_cpufreq_get(unsigned int cpu)
{
	struct rcm_cpufreq_data *data =	rcm_cpufreq_driver.driver_data;

	return data->freq_table[data->curr_freq_index].frequency;
}

static int probe_internal(struct platform_device *pdev)
{
	struct rcm_cpufreq_data *data;
	struct property *prop;
	struct device_node *cpunode;
	u32 curr_freq;
	u32 min_freq;
	u32 max_freq;
	int prop_size;
	int freq_count;
	int curr_freq_index;
	int i;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, data);

	prop = of_find_property(pdev->dev.of_node, "freqs", &prop_size);
	if (prop == NULL) {
		pr_err("failed to find frequency table\n");
		return -EINVAL;
	}
	if ((prop_size == 0) || ((prop_size % sizeof(struct rcm_freq_data)) != 0)) {
		pr_err("frequency table error\n");
		return -EINVAL;
	}
	freq_count = prop_size / sizeof(struct rcm_freq_data);

	data->freq_data = devm_kzalloc(&pdev->dev,
		(freq_count + 1) * sizeof(struct rcm_freq_data), GFP_KERNEL);
	if (data->freq_data == NULL)
		return -ENOMEM;

	data->freq_table = devm_kzalloc(&pdev->dev,
		(freq_count + 1) * sizeof(struct cpufreq_frequency_table), GFP_KERNEL);
	if (data->freq_table == NULL)
		return -ENOMEM;
	data->freq_table[freq_count].frequency = CPUFREQ_TABLE_END;

	if (of_property_read_u32_array(pdev->dev.of_node, "freqs",
		(u32*)data->freq_data, prop_size / sizeof(u32))) {
		pr_err("failed to read frequency table\n");
		return -EINVAL;
	}

	for (i = 0; i < freq_count; ++i) {
		data->freq_table[i].flags = 0;
		data->freq_table[i].driver_data = i;
		data->freq_table[i].frequency = data->freq_data[i].freq_khz;
		pr_debug("index: %i, freq %u kHz, div %u\n",
			i, data->freq_data[i].freq_khz, data->freq_data[i].divider);
	}

	data->dcr_n = dcr_resource_start(pdev->dev.of_node, 0);
	data->dcr_c = dcr_resource_len(pdev->dev.of_node, 0);
	if ((data->dcr_n == 0) || (data->dcr_c == 0)) {
		pr_err("failed to read dcr-reg value\n");
		return -EINVAL;
	}

	data->dcr_host = dcr_map(&pdev->dev, data->dcr_n, data->dcr_c);
	if (!DCR_MAP_OK(data->dcr_host)) {
		pr_err("failed to map dcr area\n");
		return -EFAULT;
	}
	data->dcr_host_mapped = true;

	cpunode = of_cpu_device_node_get(0);
	if (cpunode == NULL) {
		pr_err("can't find any CPU 0 node\n");
		return -EFAULT;
	}
	ret =  of_property_read_u32(cpunode, "clock-frequency", &curr_freq);
	of_node_put(cpunode);
	if (ret != 0) {
		pr_err("can't find desired frequency for CPU 0 node\n");
		return -EFAULT;
	}

	curr_freq_index = -1;
	min_freq = data->freq_table[0].frequency;
	max_freq = data->freq_table[0].frequency;
	for (i = 0; i < freq_count; ++i) {
		if (curr_freq == data->freq_table[i].frequency * 1000)
			curr_freq_index = i;
		min_freq = min(min_freq, data->freq_table[i].frequency);
		max_freq = max(max_freq, data->freq_table[i].frequency);
	}
	if (curr_freq_index == -1) {
		curr_freq_index = 0;
		pr_warn("Requested CPU frequency not found, set to lowest\n");
	}
	data->curr_freq_index =curr_freq_index;

	pr_info("Low: %d Mhz, High: %d Mhz, Cur: %d MHz\n",
		min_freq / 1000,
		max_freq / 1000,
		data->freq_table[curr_freq_index].frequency / 1000);

	ret = set_cpu_divider(data, data->freq_data[curr_freq_index].divider);
	if (ret != 0) {
		pr_err("cannot switch to the new CPU frequency\n");
		return ret;
	}

	rcm_cpufreq_driver.driver_data = data;
	ret = cpufreq_register_driver(&rcm_cpufreq_driver);
	if (ret != 0) {
		pr_err("cannot register the cpufreq driver\n");
		return ret;
	}
	data->cpufreq_driver_registered = true;

	return 0;
}

static void cleanup(struct platform_device *pdev)
{
	struct rcm_cpufreq_data *data = dev_get_drvdata(&pdev->dev);

	if (data == NULL)
		return;

	if (data->cpufreq_driver_registered)
		cpufreq_unregister_driver(&rcm_cpufreq_driver);

	if (data->dcr_host_mapped)
		dcr_unmap(data->dcr_host, data->dcr_c);
}

static int rcm_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("probing...\n");

	ret = probe_internal(pdev);
	if (ret != 0) {
		pr_err("driver probe error %i\n", ret);
		cleanup(pdev);
		return ret;
	}

	pr_debug("probed successfully\n");

	return 0;
}

static int rcm_cpufreq_remove(struct platform_device *pdev)
{
	cleanup(pdev);

	return 0;
}

static const struct of_device_id rcm_cpufreq_of_match[] = {
	{
		.compatible = "rcm,1888bm18-cpufreq",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_cpufreq_of_match);

static struct platform_driver rcm_cpufreq_platform_driver = {
	.driver =
		{
			.name = "rcm-1888bm18-cpufreq",
			.of_match_table = rcm_cpufreq_of_match,
		},
	.probe = rcm_cpufreq_probe,
	.remove = rcm_cpufreq_remove,
};
module_platform_driver(rcm_cpufreq_platform_driver);

MODULE_DESCRIPTION("RCM 1888BM18 core frequency driver");
MODULE_AUTHOR("Mikhail Petrov<Mikhail.Petrov@mir.dev>");
MODULE_LICENSE("GPL v2");
