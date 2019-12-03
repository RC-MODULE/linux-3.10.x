/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define CS4384_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |		\
			     SNDRV_PCM_FMTBIT_S24_LE)

#define CS4384_PCM_RATES   (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100  | \
			     SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200  | \
			     SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define CS4384_CHIP_REVISION    0x01
#define CS4384_MODE_CONTROL     0x02
#define CS4384_PCM_CONTROL      0x03
#define CS4384_DSD_CONTROL      0x04
#define CS4384_FILER_CONTROL    0x05
#define CS4384_INVERT_CONTROL   0x06
#define CS4384_GROUP_CONTROL    0x07
#define CS4384_RAMP_AND_MUTE    0x08
#define CS4384_MUTE_CONTROL     0x09
#define CS4384_MIXING_CONTROL_1 0x0a
#define CS4384_VOL_CONTROL_A1   0x0b
#define CS4384_VOL_CONTROL_B1   0x0c
#define CS4384_MIXING_CONTROL_2 0x0d
#define CS4384_VOL_CONTROL_A2   0x0e
#define CS4384_VOL_CONTROL_B2   0x0f
#define CS4384_MIXING_CONTROL_3 0x10
#define CS4384_VOL_CONTROL_A3   0x11
#define CS4384_VOL_CONTROL_B3   0x12
#define CS4384_MIXING_CONTROL_4 0x13
#define CS4384_VOL_CONTROL_A4   0x14
#define CS4384_VOL_CONTROL_B4   0x15
#define CS4384_PCM_CLOCK_MODE   0x16


static const struct reg_default cs4384_reg_defaults[] = {
	{ CS4384_CHIP_REVISION,	    0x00 },
	{ CS4384_MODE_CONTROL,	    0x01 },
	{ CS4384_PCM_CONTROL,	    0x03 },
	{ CS4384_DSD_CONTROL,	    0x08 },
	{ CS4384_FILER_CONTROL,	    0x00 },
	{ CS4384_INVERT_CONTROL,    0x00 },
	{ CS4384_GROUP_CONTROL,	    0x00 },
	{ CS4384_RAMP_AND_MUTE,	    0xBC },
	{ CS4384_MUTE_CONTROL,	    0x00 },
	{ CS4384_MIXING_CONTROL_1,	0x09 },
	{ CS4384_VOL_CONTROL_A1,	0x00 },
	{ CS4384_VOL_CONTROL_B1,	0x00 },
	{ CS4384_MIXING_CONTROL_2,	0x09 },
	{ CS4384_VOL_CONTROL_A2,	0x09 },
	{ CS4384_VOL_CONTROL_B2,	0x09 },
	{ CS4384_MIXING_CONTROL_3,	0x09 },
	{ CS4384_VOL_CONTROL_A3,	0x00 },
	{ CS4384_VOL_CONTROL_B3,	0x00 },
	{ CS4384_MIXING_CONTROL_4,	0x09 },
	{ CS4384_VOL_CONTROL_A4,	0x00 },
	{ CS4384_VOL_CONTROL_B4,	0x00 },
	{ CS4384_PCM_CLOCK_MODE,	0x00 },
};

static bool cs4384_accessible_reg(struct device *dev, unsigned int reg)
{
	return !((reg == 0x00));
}

static bool cs4384_writeable_reg(struct device *dev, unsigned int reg)
{
	return cs4384_accessible_reg(dev, reg) &&
		(reg != 1);
}

struct cs4384_private {
	struct regmap *regmap;
	unsigned int format;
	/* Current deemphasis status */
	unsigned int deemph;
	/* Current rate for deemphasis control */
	unsigned int rate;
};


static int cs4384_suspend(struct snd_soc_component *component)
{
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);

    // enable PDN
    return regmap_update_bits(priv->regmap, CS4384_MODE_CONTROL,
                0x01, 1);
}

static int cs4384_resume(struct snd_soc_component *component)
{
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);
	int ret; 

    // disabe PDN
    return regmap_update_bits(priv->regmap, CS4384_MODE_CONTROL,
                0x01, 0);
}



static const int cs4384_deemph[] = { 44100, 48000, 32000 };

static int cs4384_set_deemph(struct snd_soc_component *component)
{
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);
    int res;
    u8 val = 0;

	if (priv->deemph) {
        switch(priv->rate)
        {
            case 44100:
                val = 0x1;
                break;
            case 48000:
                val = 0x2;
                break;
            case 32000:
                val = 0x3;
                break;
            default:
                break;
                // disable
        }
	}

    res = regmap_update_bits(priv->regmap, CS4384_MIXING_CONTROL_1,
                0x60, val << 6);
    if(res)
        return res;

    res = regmap_update_bits(priv->regmap, CS4384_MIXING_CONTROL_2,
                0x60, val << 6);
    if(res)
        return res;

    res = regmap_update_bits(priv->regmap, CS4384_MIXING_CONTROL_3,
                0x60, val << 6);
    if(res)
        return res;

    res = regmap_update_bits(priv->regmap, CS4384_MIXING_CONTROL_4,
                0x60, val << 6);
	return res;
}

static int cs4384_get_deemph(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = priv->deemph;

	return 0;
}

static int cs4384_put_deemph(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);

	priv->deemph = ucontrol->value.integer.value[0];

	return cs4384_set_deemph(component);
}

static int cs4384_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int format)
{
	struct snd_soc_component *component = codec_dai->component;
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);

	/* The CS4384 can only be slave to all clocks */
	if ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(component->dev, "Invalid clocking mode\n");
		return -EINVAL;
	}

	priv->format = format;

	return 0;
}

static int cs4384_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);
	int val = 0, ret;

	priv->rate = params_rate(params);

	switch (priv->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		val = 0x00;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = 0x01;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		switch (params_width(params)) {
		case 16:
			val = 0x02;
			break;
		case 24:
			val = 0x03;
			break;
		case 20:
			val = 0x04;
			break;
		case 18:
			val = 0x05;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		dev_err(component->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(priv->regmap, CS4384_PCM_CONTROL, 0xf0, val << 4);
	if (ret < 0)
		return ret;

	return cs4384_set_deemph(component);
}

static int cs4384_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_component *component = dai->component;
	struct cs4384_private *priv = snd_soc_component_get_drvdata(component);
	int val;

	if (mute)
		val = 0xff;
	else
		val = 0;

	return regmap_write(priv->regmap, CS4384_MUTE_CONTROL, val);
}

static const struct snd_soc_dai_ops cs4384_dai_ops = {
	.set_fmt	= cs4384_set_dai_fmt,
	.hw_params	= cs4384_hw_params,
	.digital_mute	= cs4384_digital_mute,
};

static const struct snd_soc_dapm_widget cs4384_dapm_widgets[] = {
    SND_SOC_DAPM_OUTPUT("AOUT1"),
    SND_SOC_DAPM_OUTPUT("AOUT2"),
    SND_SOC_DAPM_OUTPUT("AOUT3"),
    SND_SOC_DAPM_OUTPUT("AOUT4"),
    SND_SOC_DAPM_OUTPUT("AOUT5"),
    SND_SOC_DAPM_OUTPUT("AOUT6"),
    SND_SOC_DAPM_OUTPUT("AOUT7"),
    SND_SOC_DAPM_OUTPUT("AOUT8"),
};

static const struct snd_soc_dapm_route cs4384_dapm_routes[] = {
	{ "AOUT1", NULL, "Playback" },
	{ "AOUT2", NULL, "Playback" },
	{ "AOUT3", NULL, "Playback" },
	{ "AOUT4", NULL, "Playback" },
	{ "AOUT5", NULL, "Playback" },
	{ "AOUT6", NULL, "Playback" },
	{ "AOUT7", NULL, "Playback" },
	{ "AOUT8", NULL, "Playback" },
};

static const DECLARE_TLV_DB_SCALE(cs4384_dac_tlv, -12750, 50, 0);

static const struct snd_kcontrol_new cs4384_controls[] = {
	SOC_DOUBLE_R_TLV("Channel 1/2 Playback Volume",
			CS4384_VOL_CONTROL_A1, CS4384_VOL_CONTROL_B1, 0,
			0xff, 1, cs4384_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 3/4 Playback Volume",
			CS4384_VOL_CONTROL_A2, CS4384_VOL_CONTROL_B2, 0,
			0xff, 1, cs4384_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 5/6 Playback Volume",
			CS4384_VOL_CONTROL_A3, CS4384_VOL_CONTROL_B3, 0,
			0xff, 1, cs4384_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 7/8 Playback Volume",
			CS4384_VOL_CONTROL_A4, CS4384_VOL_CONTROL_B4, 0,
			0xff, 1, cs4384_dac_tlv),
	SOC_SINGLE_BOOL_EXT("De-emphasis Switch", 0,
			    cs4384_get_deemph, cs4384_put_deemph),
};

static struct snd_soc_dai_driver cs4384_dai = {
	.name = "cs4384-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = CS4384_PCM_RATES,
		.formats = CS4384_PCM_FORMATS,
	},
	.ops = &cs4384_dai_ops,
};


#ifdef CONFIG_OF
static const struct of_device_id cs4384_dt_ids[] = {
	{ .compatible = "cirrus,cs4384", },
	{ }
};
MODULE_DEVICE_TABLE(of, cs4384_dt_ids);
#endif

static const struct regmap_config cs4384_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= 0x16,
	.reg_defaults		= cs4384_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(cs4384_reg_defaults),
	.writeable_reg		= cs4384_writeable_reg,
	.readable_reg		= cs4384_accessible_reg,
};

static const struct snd_soc_component_driver soc_component_dev_cs4384 = {
	.suspend		= cs4384_suspend,
	.resume			= cs4384_resume,
	.controls		= cs4384_controls,
	.num_controls		= ARRAY_SIZE(cs4384_controls),
	.dapm_widgets		= cs4384_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(cs4384_dapm_widgets),
	.dapm_routes		= cs4384_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(cs4384_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};


static const struct i2c_device_id cs4384_i2c_id[] = {
	{"cs4384", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs4384_i2c_id);

static int cs4384_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int ret;
    u32 val;
	struct cs4384_private *priv;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(client, &cs4384_regmap);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, priv);

    // first read CHIPREVISION
    ret = regmap_read(priv->regmap, CS4384_CHIP_REVISION, &val);
    if (ret) {
		dev_err(&client->dev, "Failed to read chip revision\n");
		return ret;
    }
    dev_info(&client->dev, "cs4384 chipid=%02x", val);

    // enable CPEN
    ret = regmap_update_bits(priv->regmap, CS4384_MODE_CONTROL,
                0x80, 1 << 7);
    if (ret) {
		dev_err(&client->dev, "Failed to set CPEN\n");
		return ret;
    }

    // disabe PDN
    ret = regmap_update_bits(priv->regmap, CS4384_MODE_CONTROL,
                0x01, 0);
    if (ret) {
		dev_err(&client->dev, "Failed to set PDN to false\n");
		return ret;
    }

    // set i2s mode by default
    ret = regmap_update_bits(priv->regmap, CS4384_PCM_CONTROL, 0xf0, 0x1 << 4);

    if (ret) {
		dev_err(&client->dev, "Failed to set I2S mode\n");
		return ret;
    }

	return devm_snd_soc_register_component(&client->dev,
		&soc_component_dev_cs4384,
		&cs4384_dai, 1);
}

static struct i2c_driver cs4384_i2c_driver = {
	.driver = {
		.name	= "cs4384",
		.of_match_table = of_match_ptr(cs4384_dt_ids),
	},
	.id_table	= cs4384_i2c_id,
	.probe		= cs4384_i2c_probe,
};

module_i2c_driver(cs4384_i2c_driver);

MODULE_DESCRIPTION("Cirrus Logic CS4384 ALSA SoC Codec Driver");
MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_LICENSE("GPL");