/*
 * RCM temperature thermal.
 *
 * Copyright (C) 2020 mir.dev, driver was originally written by:
 * 
 * - Alexander Shtreys <alexander.shtreys@mir.dev>
 * - Nadezhda Kharlamova <nadezhda.kharlamova@mir.dev>
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
#define RCM_THERMAL_DATA_PREPARE_DELAY 5 // probe: need to wait until data became valid 

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

struct rcm_thermal_data {
	struct device			*dev;
	struct rcm_thermal_regs		*regs;
	struct thermal_zone_device	*tz_device;
	int				irq_overheat;
	int				temp_overheat;
	int				hist_overheat;
	struct delayed_work		work;
};

struct adc_temp {
	u32 adc;
	int temp; // millidegree Celsius
};

static const struct adc_temp temp_table[] = {
	{ 3795	,	-40000	},
	{ 3787	,	-35000	},
	{ 3779	,	-30000	},
	{ 3769	,	-25000	},
	{ 3761	,	-20000	},
	{ 3751	,	-15000	},
	{ 3743	,	-10000	},
	{ 3733	,	-5000	},
	{ 3723	,	0	},
	{ 3713	,	5000	},
	{ 3703	,	10000	},
	{ 3693	,	15000	},
	{ 3683	,	20000	},
	{ 3673	,	25000	},
	{ 3663	,	30000	},
	{ 3651	,	35000	},
	{ 3641	,	40000	},
	{ 3629	,	45000	},
	{ 3617	,	50000	},
	{ 3605	,	55000	},
	{ 3593	,	60000	},
	{ 3581	,	65000	},
	{ 3569	,	70000	},
	{ 3555	,	75000	},
	{ 3543	,	80000	},
	{ 3529	,	85000	},
	{ 3515	,	90000	},
	{ 3501	,	95000	},
	{ 3487	,	100000  },
	{ 3471	,	105000	},
	{ 3457	,	110000	},
	{ 3441	,	115000	},
	{ 3425	,	120000	},
	{ 3409	,	125000  }
};

static int adc_to_temp(u32 val)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(temp_table); ++i)
	{
		if(val >= temp_table[i].adc)
			break;
	}

	if (i > 0) 
		i--;
	if (i == 0)
		return temp_table[i].temp;

	if (i < ARRAY_SIZE(temp_table) - 1) {
		u32 adc_begin = temp_table[i].adc;
		u32 adc_end = temp_table[i+1].adc;
		int temp_begin = temp_table[i].temp;
		int temp_end = temp_table[i+1].temp;
		return temp_begin + (temp_end - temp_begin) * (s32)(val - adc_begin) / (s32)(adc_end - adc_begin);
	}
	else
		return temp_table[i].temp;
}

static u32 temp_to_adc(int temp)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(temp_table); ++i)
	{
		if(temp <= temp_table[i].temp)
			break;
	}

	if (i > 0)
		i--;
	if (i == 0)
		return temp_table[i].adc;

	if (i < ARRAY_SIZE(temp_table) - 1) {
		u32 adc_begin = temp_table[i].adc;
		u32 adc_end = temp_table[i+1].adc;
		int temp_begin = temp_table[i].temp;
		int temp_end = temp_table[i+1].temp;
		return (s32)(adc_begin - (temp - temp_begin) * (s32)(adc_begin - adc_end) / (temp_end - temp_begin));
	}
	else
		return temp_table[i].adc;
}

static int read_term_reg(void __iomem *addr) 
{
	u32 val;
	val = ioread32(addr);
	return val;
}

static void write_term_reg(u32 val, void __iomem *addr) 
{
	iowrite32(val, addr);
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

	val = read_term_reg(&data->regs->data);

	pr_debug("%s: data = %u\n", __func__, val);

	*temp = adc_to_temp(val);

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

	write_term_reg(temp_to_adc(data->temp_overheat), &data->regs->level);

	write_term_reg(0, &data->regs->ir);
	write_term_reg(1, &data->regs->im);

	return 0;
} 


void rcm_thermal_work_handler(struct work_struct *work)
{
	struct rcm_thermal_data *data;
	int temp, low;

	data = container_of(work, struct rcm_thermal_data, work.work);

	rcm_thermal_read_temp(data, &temp);

	low = data->temp_overheat - data->hist_overheat;

	if (temp >= low) {
		queue_delayed_work(system_wq, &data->work, msecs_to_jiffies(RCM_THERMAL_OVERHEAT_POLL_DELAY));
	}
	else {
		write_term_reg(0, &data->regs->ir);
		write_term_reg(1, &data->regs->im);
		thermal_zone_device_update(data->tz_device, THERMAL_EVENT_UNSPECIFIED);
	}
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

	pr_debug("%s >>>\n", __func__);

	write_term_reg(0, &data->regs->im);
	thermal_zone_device_update(data->tz_device, THERMAL_EVENT_UNSPECIFIED);

	queue_delayed_work(system_wq, &data->work, msecs_to_jiffies(RCM_THERMAL_OVERHEAT_POLL_DELAY));

	enable_irq(irq);

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
		return PTR_ERR(data->regs);
	}

	INIT_DELAYED_WORK(&data->work, rcm_thermal_work_handler);

	data->irq_overheat = platform_get_irq(pdev, 0);

	if (data->irq_overheat > 0) {
		write_term_reg(0, &data->regs->level);
		write_term_reg(0, &data->regs->ir);
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

	// register clk_div has default value 0xFFF
	write_term_reg(0, &data->regs->pwdn);
	write_term_reg(1, &data->regs->start);

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
MODULE_AUTHOR("Alexander Shtreys, Nadezhda Kharlamova");
MODULE_LICENSE("GPL v2");
