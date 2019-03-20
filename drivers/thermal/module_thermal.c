/*
 * RC-Module temperature thermal.
 *
 * Copyright (c) 2018, AstroSoft.  All rights reserved.
 *
 * Author: Alexey Spirkov <alexeis@astrosoft.ru>
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


struct termal_regs {
	u32 tavg;
	u32 tctrl_avg;
	u32 tgrad;
	u32 thigh;
	u32 tlow;
	u32 tint;
	u32 tint_mask;
	u32 tsens;
	u32 ttmode;
	u32 ttint;
	u32 tid;
	u32 tstart;
	u32 skip[4];
	u32 vavg;
	u32 vctrl_avg;
	u32 vgrad;
	u32 vhigh;
	u32 vlow;
	u32 vint;
	u32 vint_mask;
	u32 vsens;
	u32 vtmode;
	u32 vtint;
	u32 vid;
	u32 vstart;
};

#define read_term_reg(X) le32_to_cpu(__raw_readl(&(X)))
#define write_term_reg(Y, X) __raw_writel(cpu_to_le32(Y), &(X))

struct rcmodule_therm_info {
	struct device			*dev;
	struct termal_regs		*regs;
	struct thermal_zone_device	*tz_device;
	int temp_irq;
	int voltage_irq;
};

static const int temp_table[] = {
	650	,	-40	,
	642	,	-35	,
	635	,	-30	,
	627	,	-25	,
	619	,	-20	,
	611	,	-15	,
	603	,	-10	,
	595	,	-5	,
	588	,	0	,
	580	,	5	,
	572	,	10	,
	564	,	15	,
	556	,	20	,
	548	,	25	,
	541	,	30	,
	533	,	35	,
	525	,	40	,
	517	,	45	,
	509	,	50	,
	501	,	55	,
	493	,	60	,
	485	,	65	,
	477	,	70	,
	469	,	75	,
	462	,	80	,
	454	,	85	,
	446	,	90	,
	438	,	95	,
	430	,	100	,
	422	,	105	,
	414	,	110	,
	406	,	115	,
	398	,	120	,
	390	,	125	,
};




/**
 * rcmodule_thermal_read_temp: Read temperatue.
 * @data:	Device specific data.
 * temp:	Temperature in millidegrees Celsius
 *
 * Return 0 on success otherwise error number to show reason of failure.
 */

#undef TABLE_METHOD

static int rcmodule_thermal_read_temp(void *data, int *temp)
{
	struct rcmodule_therm_info *therminfo = data;
	unsigned int val, val0;

	val0 = read_term_reg(therminfo->regs->tavg);

	//  take avarage by 3 sensors 
	val = ((val0 & 0x3FF) + ((val0 >> 10) & 0x3FF) + ((val0 >> 20) & 0x3FF));

#ifdef DEBUG
	printk("tvag: %d, %d, %d\n", (val0 & 0x3FF), ((val0 >> 10) & 0x3FF), ((val0 >> 20) & 0x3FF));
#endif

	if(val < 390*3)
		*temp = 125000;
	else if(val > 650*3)
		*temp = -40000;
	else
	{
#ifdef TABLE_METHOD
		int i;
		for(i = 0; i < (sizeof(temp_table) - 1); i+=2)
		{
			if(val > temp_table[i])
				break;
		}
		*temp = temp_table[i+1]*1000;
#else		

		*temp = ((65000000*3 - val*100000)/473 + -40000);
#endif		
	}

#ifdef DEBUG
	printk("--- temp*1000: %d, from (%x)", *temp, val);
	printk("tctrl_avg: %x", read_term_reg(therminfo->regs->tctrl_avg));
	printk("tgrad:     %x", read_term_reg(therminfo->regs->tgrad));
	printk("thigh:     %x", read_term_reg(therminfo->regs->thigh));
	printk("tlow:      %x", read_term_reg(therminfo->regs->tlow));
	printk("tint:      %x", read_term_reg(therminfo->regs->tint));
	printk("tint_mask: %x", read_term_reg(therminfo->regs->tint_mask));
	printk("tsens:     %x", read_term_reg(therminfo->regs->tsens));
	printk("ttmode:    %x", read_term_reg(therminfo->regs->ttmode));
	printk("ttint:     %x", read_term_reg(therminfo->regs->ttint));
	printk("tid:       %x", read_term_reg(therminfo->regs->tid));
	printk("tstart:    %x", read_term_reg(therminfo->regs->tstart));
#endif

	return 0;
}

static const struct thermal_zone_of_device_ops rcmodule_thermal_ops = {
	.get_temp = rcmodule_thermal_read_temp,
};

static irqreturn_t rcmodule_thermal_irq(int irq, void *data)
{
	struct rcmodule_therm_info *therminfo = data;

	thermal_zone_device_update(therminfo->tz_device,
				   THERMAL_EVENT_UNSPECIFIED);

	return IRQ_HANDLED;
}

static int rcmodule_thermal_probe(struct platform_device *pdev)
{
	struct rcmodule_therm_info *therminfo;
	int ret;

	therminfo = devm_kzalloc(&pdev->dev, sizeof(*therminfo), GFP_KERNEL);
	if (!therminfo)
		return -ENOMEM;

	therminfo->dev = &pdev->dev;
	therminfo->regs = devm_ioremap_resource(therminfo->dev, pdev->resource);
	if (IS_ERR(therminfo->regs)) {
		dev_err(&pdev->dev, "Failed to allocate registers\n");
		return -ENODEV;
	}

	// first check tid
	if(read_term_reg(therminfo->regs->tid) != 1) 
	{
		printk("Temp sensor ID == %x, skip", read_term_reg(therminfo->regs->tid));
		goto errMemMapped;
	}

	therminfo->temp_irq = platform_get_irq(pdev, 0);
	therminfo->voltage_irq = platform_get_irq(pdev, 1);

	if(request_irq(therminfo->temp_irq, rcmodule_thermal_irq, 0, "rcmodule-thermal", therminfo))
	{
		dev_err(&pdev->dev, "Unable to request thermal IRQ\n");
		goto errMemMapped;
	}

	if(request_irq(therminfo->voltage_irq, rcmodule_thermal_irq, 0, "rcmodule-voltage", therminfo))
	{
		dev_err(&pdev->dev, "Unable to request voltage IRQ\n");
		goto errIrq0Requested;
	}

	// avegage by 16 count
	write_term_reg(16, therminfo->regs->tctrl_avg);
	
	// start measuring temperature
	write_term_reg(0x1, therminfo->regs->tstart);

	// allow irq for temperature
	// TODO AstroSoft: fix on workable HW
	 write_term_reg(0x0, therminfo->regs->tint_mask);

	therminfo->tz_device = devm_thermal_zone_of_sensor_register(&pdev->dev, 0,
				therminfo, &rcmodule_thermal_ops);

	if (IS_ERR(therminfo->tz_device)) {
		ret = PTR_ERR(therminfo->tz_device);
		dev_err(&pdev->dev, "Failed to register thermal zone: %d\n",
			ret);
		return ret;
	}

	platform_set_drvdata(pdev, therminfo);

	return 0;

errIrq0Requested:
	free_irq(therminfo->temp_irq, (void *)therminfo->dev);

errMemMapped:
	devm_iounmap(therminfo->dev, therminfo->regs);

	return -EINVAL;
}

static int rcmodule_thermal_remove(struct platform_device *pdev)
{
	struct rcmodule_therm_info *therminfo = platform_get_drvdata(pdev);

	free_irq(therminfo->voltage_irq, (void *)therminfo->dev);
	free_irq(therminfo->temp_irq, (void *)therminfo->dev);
	devm_iounmap(therminfo->dev, therminfo->regs);

	return 0;	
}

static struct of_device_id rcmodule_thermal_id_table[] = {
	{ .compatible = "rc-module,thermal" },
	{},
};

MODULE_DEVICE_TABLE(of, rcmodule_thermal_id_table);

static struct platform_driver rcmodule_thermal_driver = {
	.driver = {
		.name = "rcmodule-thermal",
		.of_match_table = rcmodule_thermal_id_table,
	},
	.probe = rcmodule_thermal_probe,
	.remove = rcmodule_thermal_remove,
};

module_platform_driver(rcmodule_thermal_driver);

MODULE_DESCRIPTION("RC-Module temperature Thermal driver");
MODULE_AUTHOR("Alexey Spirkov <alexeis@astrosoft.ru>");
MODULE_LICENSE("GPL v2");
