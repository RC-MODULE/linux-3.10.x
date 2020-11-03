/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Mikhail Petrov <Mikhail.Petrov@astrosoft.ru>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>

#define MAIN_REG_CHIP_REVISION 0x00
#define MAIN_REG_I2C_FREQ_ID_CFG 0x15
#define MAIN_REG_VIDEO_INPUT_CFG_1 0x16
#define MAIN_REG_VIDEO_INPUT_CFG_2 0x17
#define MAIN_REG_CSC_UPPER 0x18
#define MAIN_REG_POWER 0x41
#define MAIN_REG_STATUS 0x42
#define MAIN_REG_AVI_INFOFRAME_0 0x55
#define MAIN_REG_INT_0 0x96
#define MAIN_REG_FIXED_98 0x98
#define MAIN_REG_FIXED_9A 0x9A
#define MAIN_REG_FIXED_9C 0x9C
#define MAIN_REG_INPUT_CLK_DIV 0x9D
#define MAIN_REG_FIXED_A2 0xA2
#define MAIN_REG_FIXED_A3 0xA3
#define MAIN_REG_HDCP_HDMI_CFG 0xAF
#define MAIN_REG_FIXED_E0 0xE0
#define MAIN_REG_FIXED_F9 0xF9

#define MAIN_REG_POWER_POWER_DOWN_MASK (1U << 6)

#define MAIN_REG_STATUS_HPD_STATE_MASK (1U << 6)

#define MAIN_REG_INT_0_HPD_MASK (1U << 7)

#define TEST_HPD_PERIOD (2 * HZ)

struct adv7513_common
{
	struct i2c_client *i2c_main;
	struct regmap *regmap_main;
	struct delayed_work test_HPD_work;
	bool initialized;
	bool active;
};

static const struct regmap_config reg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int read_HPD_state(struct adv7513_common *adv7513, bool *active, bool *changed)
{
	int ret;
	unsigned data;

	*active = false;
	*changed = false;

	while (true) {
		ret = regmap_read(adv7513->regmap_main, MAIN_REG_STATUS, &data);
		if (ret != 0)
			return ret;
		*active = ((data & MAIN_REG_STATUS_HPD_STATE_MASK) != 0);

		ret = regmap_read(adv7513->regmap_main, MAIN_REG_INT_0, &data);
		if (ret != 0)
			return ret;
		if ((data & MAIN_REG_INT_0_HPD_MASK) == 0)
			break;

		*changed = true;
		ret = regmap_write(adv7513->regmap_main, MAIN_REG_INT_0, MAIN_REG_INT_0_HPD_MASK);
		if (ret != 0)
			return ret;
	}

	return 0;
}

static int power_on(struct adv7513_common *adv7513)
{
	int ret;
	unsigned data;

	dev_info(&adv7513->i2c_main->dev, "truning power on\n");

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_POWER, &data);
	if (ret != 0)
		return ret;
	data &= ~MAIN_REG_POWER_POWER_DOWN_MASK;
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_POWER, data);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_98, 0x03);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_FIXED_9A, &data);
	if (ret != 0)
		return ret;
	data |= 0xE0;
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_9A, data);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_9C, 0x30);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_INPUT_CLK_DIV, &data);
	if (ret != 0)
		return ret;
	data &= ~0x03;
	data |= 0x01;
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_INPUT_CLK_DIV, data);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_A2, 0xA4);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_A3, 0xA4);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_E0, 0xD0);
	if (ret != 0)
		return ret;

	ret = regmap_write(adv7513->regmap_main, MAIN_REG_FIXED_F9, 0x00);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_I2C_FREQ_ID_CFG, &data);
	if (ret != 0)
		return ret;
	data &= ~0x0F;
	data |= 0x01; // 16, 20, 24 bit YCbCr 4:2:2 (separate syncs)
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_I2C_FREQ_ID_CFG, data);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_VIDEO_INPUT_CFG_1, &data);
	if (ret != 0)
		return ret;
	data &= ~0xBD;
	data |= 0xB9; // 4:2:2, 8 bit, style 1, black image YCbCr
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_VIDEO_INPUT_CFG_1, data);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_VIDEO_INPUT_CFG_2, &data);
	if (ret != 0)
		return ret;
	data &= ~0x02;
	data |= 0x00; // 16:9 Aspect Ratio
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_VIDEO_INPUT_CFG_2, data);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_CSC_UPPER, &data);
	if (ret != 0)
		return ret;
	data &= ~0x80;
	data |= 0x00; // CSC Disabled
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_CSC_UPPER, data);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_HDCP_HDMI_CFG, &data);
	if (ret != 0)
		return ret;
	data &= ~0x02;
	data |= 0x02; // HDMI Mode
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_HDCP_HDMI_CFG, data);
	if (ret != 0)
		return ret;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_AVI_INFOFRAME_0, &data);
	if (ret != 0)
		return ret;
	data &= ~0x60;
	data |= 0x20; // 01 = YCbCr 4:2:2
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_AVI_INFOFRAME_0, data);
	if (ret != 0)
		return ret;

	dev_info(&adv7513->i2c_main->dev, "successfully activated\n");

	return 0;
}

static int power_off(struct adv7513_common *adv7513)
{
	int ret;
	unsigned data;

	dev_info(&adv7513->i2c_main->dev, "truning power off\n");

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_POWER, &data);
	if (ret != 0)
		return ret;

	data |= MAIN_REG_POWER_POWER_DOWN_MASK;
	ret = regmap_write(adv7513->regmap_main, MAIN_REG_POWER, data);
	if (ret != 0)
		return ret;

	return 0;
}

void test_HPD_work(struct work_struct *work)
{
	int ret;
	bool active;
	bool changed;
	struct delayed_work *dwork = to_delayed_work(work);
	struct adv7513_common *adv7513 = container_of(dwork, struct adv7513_common, test_HPD_work);

	schedule_delayed_work(&adv7513->test_HPD_work, TEST_HPD_PERIOD);

	ret = read_HPD_state(adv7513, &active, &changed);
	if (ret != 0)
		return;

	if (!active && changed) {
		adv7513->active = false;
		power_off(adv7513);
	} else if (active && (changed || !adv7513->active)) {
		adv7513->active = true;
		power_on(adv7513);
	}
}

static int probe_internal(struct adv7513_common *adv7513)
{
	int ret;
	unsigned data;
	struct regmap *regmap_main;
	
	regmap_main = devm_regmap_init_i2c(adv7513->i2c_main, &reg_regmap_config);
	if (IS_ERR(regmap_main))
		return PTR_ERR(regmap_main);
	adv7513->regmap_main = regmap_main;

	ret = regmap_read(adv7513->regmap_main, MAIN_REG_CHIP_REVISION, &data);
	if (ret != 0)
		return ret;
	if (data != 0x13) {
		dev_err(&adv7513->i2c_main->dev, "wrong chip id 0x%02X (must be 0x13)\n", data);
		return -ENODEV;
	}

	ret = power_off(adv7513);
	if (ret != 0)
		return ret;

	INIT_DELAYED_WORK(&adv7513->test_HPD_work, test_HPD_work);

	return 0;
}

static int free_internal(struct adv7513_common *adv7513)
{
	if (adv7513->initialized)
		cancel_delayed_work_sync(&adv7513->test_HPD_work);

	if (adv7513->active)
		power_off(adv7513);

	return 0;
}

static int rcm_adv7513_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	struct adv7513_common *adv7513;
	struct device *dev = &i2c->dev;

	adv7513 = devm_kzalloc(dev, sizeof(*adv7513), GFP_KERNEL);
	if (!adv7513)
		return -ENOMEM;

	adv7513->i2c_main = i2c;
	i2c_set_clientdata(i2c, adv7513);

	ret = probe_internal(adv7513);
	if (ret == 0) {
		adv7513->initialized = true;
		dev_info(&adv7513->i2c_main->dev, "probed ok\n");
		schedule_delayed_work(&adv7513->test_HPD_work, TEST_HPD_PERIOD);
	}
	else {
		dev_err(&adv7513->i2c_main->dev, "probe error %i\n", ret);
		free_internal(adv7513);
	}

	return ret;
}

static int rcm_adv7513_remove(struct i2c_client *i2c)
{
	struct adv7513_common *adv7513 = i2c_get_clientdata(i2c);
	return free_internal(adv7513);
}

static const struct of_device_id rcm_adv7513_of_ids[] = {
	{ .compatible = "rcm-adv513" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcm_adv7513_of_ids);

static const struct i2c_device_id rcm_adv7513_i2c_ids[] = {
	{ "rcm-adv7513" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rcm_adv7513_i2c_ids);

static struct i2c_driver rcm_adv7513_driver = {
	.driver = {
		.name = "rcm-adv7513",
		.owner = THIS_MODULE,
		.of_match_table = rcm_adv7513_of_ids,
	},
	.id_table = rcm_adv7513_i2c_ids,
	.probe = rcm_adv7513_probe,
	.remove = rcm_adv7513_remove,
};

static int __init rcm_adv7513_init(void)
{
	return i2c_add_driver(&rcm_adv7513_driver);
}

static void __exit rcm_adv7513_exit(void)
{
	i2c_del_driver(&rcm_adv7513_driver);
}

module_init(rcm_adv7513_init);
module_exit(rcm_adv7513_exit);

MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@astrosoft.ru>");
MODULE_DESCRIPTION("ADV7513 HDMI transmitter driver");
MODULE_LICENSE("GPL");
