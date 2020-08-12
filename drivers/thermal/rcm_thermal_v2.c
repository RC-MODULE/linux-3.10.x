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
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define TS_PWDN     0x00
#define TS_START    0x04
#define TS_IR       0x08
#define TS_IM       0x0C
#define TS_CLK_DIV  0x10
#define TS_LEVEL    0x14
#define TS_DATA     0x18
#define TS_ENZC     0x1C

#define RCM_THERMAL_OVERHEAT_POLL_DELAY 1000
#define RCM_THERMAL_DATA_PREPARE_DELAY 5

#define RCM_THERMAL_DATA_MASK 0xFFF

struct rcm_thermal_data {
	struct device              *dev;
	struct regmap              *tvsens;
	struct thermal_zone_device *tz_device;
	int                         irq_overheat;
	int                         temp_overheat;
	int                         hist_overheat;
};

typedef struct _temp_tbl_entry
{
	u16 val;
        s16 temp;
} temp_tbl_entry;

static const temp_tbl_entry temp_tbl[] =
{
	{ 3795,	-40 },
	{ 3787,	-35 },
	{ 3779,	-30 },
	{ 3769,	-25 },
	{ 3761,	-20 },
	{ 3751,	-15 },
	{ 3743,	-10 },
	{ 3733,	-5  },
	{ 3723,	0   },
	{ 3713,	5   },
	{ 3703,	10  },
	{ 3693,	15  },
	{ 3683,	20  },
	{ 3673,	25  },
	{ 3663,	30  },
	{ 3651,	35  },
	{ 3641,	40  },
	{ 3629,	45  },
	{ 3617,	50  },
	{ 3605,	55  },
	{ 3593,	60  },
	{ 3581,	65  },
	{ 3569,	70  },
	{ 3555,	75  },
	{ 3543,	80  },
	{ 3529,	85  },
	{ 3515,	90  },
	{ 3501,	95  },
	{ 3487,	100 },
	{ 3471,	105 },
	{ 3457,	110 },
	{ 3441,	115 },
	{ 3425,	120 },
	{ 3409,	125 }
};

static int rcm_thermal_to_temp(u32 val)
{
	u32 i;
        int temp;

	for (i = 0; i < ARRAY_SIZE(temp_tbl); i ++)
		if (val >= temp_tbl[i].val)
			break;
	if (i == 0)
                temp = temp_tbl[i].temp * 1000;
        else
        {
		if (i == ARRAY_SIZE(temp_tbl))
			temp = temp_tbl[i - 1].temp * 1000;
                else
                        temp = temp_tbl[i - 1].temp * 1000 +
                                (((temp_tbl[i - 1].val - val) * 1000) /
                                (temp_tbl[i - 1].val - temp_tbl[i].val)) *
                                (temp_tbl[i].temp - temp_tbl[i - 1].temp);
        }

	return temp;
}

static u32 rcm_thermal_from_temp(int temp)
{
	u32 i;
        u32 val;

	for (i = 0; i < ARRAY_SIZE(temp_tbl); i ++)
		if (temp <= temp_tbl[i].temp * 1000)
			break;
	if (i == 0)
                val = temp_tbl[i].val;
        else
        {
		if (i == ARRAY_SIZE(temp_tbl))
                        val = temp_tbl[i - 1].val;
                else
                        val = temp_tbl[i - 1].val -
                                ((((temp - temp_tbl[i - 1].temp) * 1000) /
                                (temp_tbl[i].temp - temp_tbl[i - 1].temp)) *
                                (temp_tbl[i - 1].val - temp_tbl[i].val)) / 1000;
        }

	return val;
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
	struct regmap *map = data->tvsens;
	u32 val;

        regmap_read(map, TS_DATA, &val);
        val &= RCM_THERMAL_DATA_MASK;

	*temp = rcm_thermal_to_temp(val);

	dev_dbg(data->dev, "%s: data=%xH/%u => temp=%d\n", __func__, val, val, *temp);

	return 0;
}

static int rcm_thermal_configure_overheat_interrupt(struct rcm_thermal_data *data)
{
	struct regmap *map = data->tvsens;
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

        regmap_write(map, TS_LEVEL, rcm_thermal_from_temp(data->temp_overheat));

        regmap_write(map, TS_IM, 1);

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
	struct regmap *map = data->tvsens;
	int low = data->temp_overheat - data->hist_overheat;
	int temp;

	dev_dbg(data->dev, "%s >>>\n", __func__);

	thermal_zone_device_update(data->tz_device,
	                           THERMAL_EVENT_UNSPECIFIED);

	do {
		msleep(RCM_THERMAL_OVERHEAT_POLL_DELAY);
		rcm_thermal_read_temp(data, &temp);
	} while (temp >= low);

	thermal_zone_device_update(data->tz_device,
	                           THERMAL_EVENT_UNSPECIFIED);

        regmap_write(map, TS_IR, 0);

	enable_irq(irq);

	dev_dbg(data->dev, "%s <<<\n", __func__);

	return IRQ_HANDLED;
}

static int rcm_thermal_probe(struct platform_device *pdev)
{
	struct rcm_thermal_data *data;
	struct regmap *map;
        int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;

	map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "rcm,tempmon");
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&pdev->dev, "failed to get tempmon regmap: %d\n", ret);
		return ret;
	}
	data->tvsens = map;

	data->irq_overheat = platform_get_irq(pdev, 0);

	if (data->irq_overheat > 0) {
        	regmap_write(map, TS_LEVEL, 0);
        	regmap_write(map, TS_IM, 0);
        	regmap_write(map, TS_IR, 0);

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

	regmap_write(map, TS_PWDN, 0);
	regmap_write(map, TS_START, 1);

	// It's needed to wait until data became valid (at least 7 ADC-cycles)
	msleep(RCM_THERMAL_DATA_PREPARE_DELAY);

	data->tz_device = devm_thermal_zone_of_sensor_register(&pdev->dev, 0,
				data, &rcm_thermal_ops);

	if (IS_ERR(data->tz_device)) {
		ret = PTR_ERR(data->tz_device);
		dev_err(&pdev->dev, "Failed to register thermal zone: %d\n",
		        ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	if (data->irq_overheat > 0)
		rcm_thermal_configure_overheat_interrupt(data);

	dev_dbg(&pdev->dev, "probe succeeded\n");

	return 0;
}

static int rcm_thermal_remove(struct platform_device *pdev)
{
	struct rcm_thermal_data *data = platform_get_drvdata(pdev);
	struct regmap *map = data->tvsens;

	regmap_write(map, TS_START, 0);
	regmap_write(map, TS_PWDN, 1);

	thermal_zone_device_unregister(data->tz_device);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rcm_thermal_suspend(struct device *dev)
{
	struct rcm_thermal_data *data = dev_get_drvdata(dev);
	struct regmap *map = data->tvsens;

	regmap_write(map, TS_START, 0);
	regmap_write(map, TS_PWDN, 1);

	return 0;
}

static int rcm_thermal_resume(struct device *dev)
{
	struct rcm_thermal_data *data = dev_get_drvdata(dev);
	struct regmap *map = data->tvsens;

	regmap_write(map, TS_PWDN, 0);
	regmap_write(map, TS_START, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(rcm_thermal_pm_ops,
			 rcm_thermal_suspend, rcm_thermal_resume);

static struct of_device_id rcm_thermal_id_table[] = {
	{ .compatible = "rcm,thermal-v2" },
	{},
};

MODULE_DEVICE_TABLE(of, rcm_thermal_id_table);

static struct platform_driver rcm_thermal_driver = {
	.driver = {
		.name = "rcm-thermal-v2",
		.pm	= &rcm_thermal_pm_ops,
		.of_match_table = rcm_thermal_id_table,
	},
	.probe = rcm_thermal_probe,
	.remove = rcm_thermal_remove,
};

module_platform_driver(rcm_thermal_driver);

MODULE_DESCRIPTION("RCM temperature Thermal driver (v2)");
MODULE_AUTHOR("Alexander Shtreys");
MODULE_AUTHOR("Nikolai Vasilev <wasko@mail.ru>");
MODULE_LICENSE("GPL v2");
