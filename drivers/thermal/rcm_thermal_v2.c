/*
 * RCM temperature thermal.
 *
 * Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define RCM_THERMAL_OVERHEAT_POLL_DELAY 1000
#define RCM_THERMAL_DATA_PREPARE_DELAY 5

#pragma pack(push)
#pragma pack(4)

struct rcm_thermal_regs {
	u32 pwdn;
	u32 start;
	u32 ir;
	u32 im;
	u32 clk_div;
	u32 level;
	u32 data;
	u32 enzc;
};

#pragma pack(pop)

#define RCM_THERMAL_DATA_MASK 0xFFF

#define read_term_reg(X) le32_to_cpu(__raw_readl(&(X)))
#define write_term_reg(Y, X) __raw_writel(cpu_to_le32(Y), &(X))

struct rcm_thermal_data {
	struct device              *dev;
	struct rcm_thermal_regs    *regs;
	struct thermal_zone_device *tz_device;
	int                         irq_overheat;
	int                         temp_overheat;
	int                         hist_overheat;
};

static const int temp_table[] = {
	3795	,	-40	,
	3787	,	-35	,
	3779	,	-30	,
	3769	,	-25	,
	3761	,	-20	,
	3751	,	-15	,
	3743	,	-10	,
	3733	,	-5	,
	3723	,	0	,
	3713	,	5	,
	3703	,	10	,
	3693	,	15	,
	3683	,	20	,
	3673	,	25	,
	3663	,	30	,
	3651	,	35	,
	3641	,	40	,
	3629	,	45	,
	3617	,	50	,
	3605	,	55	,
	3593	,	60	,
	3581	,	65	,
	3569	,	70	,
	3555	,	75	,
	3543	,	80	,
	3529	,	85	,
	3515	,	90	,
	3501	,	95	,
	3487	,	100	,
	3471	,	105	,
	3457	,	110	,
	3441	,	115	,
	3425	,	120	,
	3409	,	125
};

static const int size_temp_table = sizeof(temp_table) / sizeof(temp_table[0]) / 2;

static int rcm_thermal_to_temp(u32 val)
{
	int i;

	for(i = 0; i < size_temp_table; ++i)
	{
		if(val >= temp_table[2 * i])
			break;
	}

	if (i == size_temp_table)
		--i;

	return temp_table[2 * i + 1] * 1000;
}

static u32 rcm_thermal_from_temp(int temp)
{
	int i;

	for(i = 0; i < size_temp_table; ++i)
	{
		if(temp <= temp_table[2 * i + 1] * 1000)
			break;
	}

	if (i == size_temp_table)
		--i;

	return temp_table[2 * i];
}

/**
 * rcm_thermal_read_temp: Read temperatue.
 * @_data:	Device specific data.
 * @temp:	Temperature in millidegrees Celsius
 *
 * Return 0 on success otherwise error number to show reason of failure.
 */
static int rcm_thermal_read_temp(void *_data, int *temp)
{
	struct rcm_thermal_data *data = _data;
	u32 val;

	val = read_term_reg(data->regs->data) & RCM_THERMAL_DATA_MASK;

	pr_debug("%s: data = %u\n", __func__, val);

	*temp = rcm_thermal_to_temp(val);

	return 0;
}

static int rcm_thermal_configure_overheat_interrupt(struct rcm_thermal_data *data)
{
	const struct thermal_trip *trips = of_thermal_get_trip_points(data->tz_device);
	int i;

	if (!trips)
		return -EINVAL;

	for (i = 0; i < of_thermal_get_ntrips(data->tz_device); i++)
		if (trips[i].type == THERMAL_TRIP_CRITICAL)
			break;

	if (i == of_thermal_get_ntrips(data->tz_device))
		return -EINVAL;


	data->temp_overheat = trips[i].temperature;
	data->hist_overheat = trips[i].hysteresis;

	write_term_reg(rcm_thermal_from_temp(data->temp_overheat), data->regs->level);

	write_term_reg(1, data->regs->im);

	return 0;
} 

static const struct thermal_zone_of_device_ops rcm_thermal_ops = {
	.get_temp = rcm_thermal_read_temp
};

static irqreturn_t rcm_thermal_overheat_isr(int irq, void *dev_id)
{
	disable_irq_nosync(irq);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rcm_thermal_overheat_isr_thread(int irq, void *dev_id)
{
	struct rcm_thermal_data *data = dev_id;
	int low = data->temp_overheat - data->hist_overheat;
	int temp;

	pr_debug("%s >>>\n", __func__);

	thermal_zone_device_update(data->tz_device,
	                           THERMAL_EVENT_UNSPECIFIED);

	do {
		msleep(RCM_THERMAL_OVERHEAT_POLL_DELAY);
		rcm_thermal_read_temp(data, &temp);
	} while (temp >= low);

	thermal_zone_device_update(data->tz_device,
	                           THERMAL_EVENT_UNSPECIFIED);

	write_term_reg(0, data->regs->ir);

	enable_irq(irq);

	pr_debug("%s <<<\n", __func__);

	return IRQ_HANDLED;
}

static int rcm_thermal_probe(struct platform_device *pdev)
{
	struct rcm_thermal_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	data->regs = devm_ioremap_resource(data->dev, pdev->resource);
	if (IS_ERR(data->regs)) {
		dev_err(&pdev->dev, "Failed to allocate registers\n");
		return -ENODEV;
	}

	data->irq_overheat = platform_get_irq(pdev, 0);

	if (data->irq_overheat > 0) {
		write_term_reg(0, data->regs->level);
		write_term_reg(0, data->regs->im);
		write_term_reg(0, data->regs->ir);

		ret = devm_request_threaded_irq(data->dev, data->irq_overheat,
		                                rcm_thermal_overheat_isr, 
		                                rcm_thermal_overheat_isr_thread,
		                                0, "rcm-thermal", data);

		if (ret)
		{
			dev_err(&pdev->dev, "Unable to request thermal IRQ\n");
			return ret;
		}
	}

	write_term_reg(0, data->regs->pwdn);
	write_term_reg(1, data->regs->start);

	// It's needed to wait until data became valid (at least 7 ADC-cycles)
	msleep(RCM_THERMAL_DATA_PREPARE_DELAY);

	data->tz_device = devm_thermal_zone_of_sensor_register(&pdev->dev, 0,
				data, &rcm_thermal_ops);

	if (IS_ERR(data->tz_device)) {
		dev_err(&pdev->dev, "Failed to register thermal zone: %d\n",
		        ret);
		return PTR_ERR(data->tz_device);
	}

	platform_set_drvdata(pdev, data);

	if (data->irq_overheat > 0)
		rcm_thermal_configure_overheat_interrupt(data);

	dev_dbg(&pdev->dev, "probe succeeded\n");

	return 0;
}

static int rcm_thermal_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id rcm_thermal_id_table[] = {
	{ .compatible = "rcm,thermal-v2" },
	{},
};

MODULE_DEVICE_TABLE(of, rcm_thermal_id_table);

static struct platform_driver rcm_thermal_driver = {
	.driver = {
		.name = "rcm-thermal-v2",
		.of_match_table = rcm_thermal_id_table,
	},
	.probe = rcm_thermal_probe,
	.remove = rcm_thermal_remove,
};

module_platform_driver(rcm_thermal_driver);

MODULE_DESCRIPTION("RCM temperature Thermal driver (v2)");
MODULE_AUTHOR("Alexander Shtreys");
MODULE_LICENSE("GPL v2");
