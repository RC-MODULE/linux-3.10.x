/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/dmaengine.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#define RCM_I2S_REG_CTRL0	0x00
#define RCM_I2S_REG_CTRL1	0x08
#define RCM_I2S_REG_STAT0	0x10
#define RCM_I2S_REG_STAT1	0x18
#define RCM_I2S_REG_FIFO0	0x20
#define RCM_I2S_REG_FIFO1	0x28
#define RCM_I2S_REG_FIFO2	0x30
#define RCM_I2S_REG_FIFO3	0x38


struct rcm_i2s {
	struct regmap *regmap;
	struct snd_soc_dai_driver dai_driver;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
};

static const struct regmap_config rcm_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 8,
	.val_bits = 32,
	.max_register = RCM_I2S_REG_FIFO3,
};


static int rcm_i2s_reset(struct rcm_i2s *i2s)
{
	unsigned int status;
	regmap_update_bits(i2s->regmap, RCM_I2S_REG_CTRL0, 1, 1);
	return regmap_read_poll_timeout(i2s->regmap, RCM_I2S_REG_CTRL0, status, (status&1) == 0, 100, 10000);
}


static int rcm_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	printk("TRACE: rcm_i2s_dai_probe");

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
		0);

	return 0;
}

static int rcm_i2s_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret;
	printk("TRACE: rcm_i2s_startup");

	ret = rcm_i2s_reset(i2s);
	if(ret)
		return ret;

    return 0;
}


static void rcm_i2s_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	
	rcm_i2s_reset(i2s);

	printk("TRACE: rcm_i2s_shutdown");

}

static int rcm_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	printk("TRACE: rcm_i2s_trigger, %d", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = regmap_update_bits(i2s->regmap, RCM_I2S_REG_CTRL1, 0x200, 0x200);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = regmap_update_bits(i2s->regmap, RCM_I2S_REG_CTRL1, 0x200, 0x0);
		break;
	default:
		return -EINVAL;
	}

    return ret;
}

static int rcm_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int width = snd_pcm_format_width(params_format(params));
	int rate = params_rate(params);
	int is_big_endian = snd_pcm_format_big_endian(params_format(params));
	int div_value = 24576 / (rate / 1000) / width / 4;
	int ctrl0 = 0;
	switch(width)
	{
		case 16:
			ctrl0 |= 0xF0008;
			break;
		case 20:
			ctrl0 |= 0x130000;
			break;
		case 24:
			ctrl0 |= 0x170000;
			break;
		case 32:
			ctrl0 |= 0x1F0000;
			break;
		default:
			return -EINVAL;
	}

	// allow interrupts
	ctrl0 |= 0xFF0;

	if(is_big_endian)
		ctrl0 |= 0x4;

	printk("TRACE: rcm_i2s_hw_params %d, %d, %d", params_channels(params), params_rate(params), params_format(params));

	regmap_write(i2s->regmap, RCM_I2S_REG_CTRL0, ctrl0);
	regmap_write(i2s->regmap, RCM_I2S_REG_CTRL1, div_value);

    return 0;
}


static const struct snd_soc_dai_ops rcm_i2s_dai_ops = {
	.startup = rcm_i2s_startup,
	.shutdown = rcm_i2s_shutdown,
	.trigger = rcm_i2s_trigger,
	.hw_params = rcm_i2s_hw_params,
};

#define RCM_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |\
			SNDRV_PCM_FMTBIT_S20_LE | SNDRV_PCM_FMTBIT_S20_BE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |\
			SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)


static struct snd_soc_dai_driver rcm_i2s_dai = {
	.probe = rcm_i2s_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000, // SNDRV_PCM_RATE_8000_192000,
		.formats = RCM_I2S_FORMATS,
	},
	.ops = &rcm_i2s_dai_ops,
	.symmetric_rates = 1,
};


static int i2s_pcm_channel_info(struct snd_pcm_substream *stream,
				struct snd_pcm_channel_info *info)
{
	struct snd_pcm_runtime *runtime = stream->runtime;
	size_t size = frames_to_bytes(runtime,
	                              runtime->buffer_size) / 
	                              (runtime->channels / 2);
	size_t width = snd_pcm_format_physical_width(runtime->format);

	info->offset = 0;
	info->first =
	    (info->channel / 2) * size * 8 + (info->channel % 2) * width;
	info->step = width * 2;

	printk("TRACE: i2s_pcm_channel_info channel %d offset %lu first %d step %d",
	      info->channel, info->offset, info->first, info->step);

	return 0;
}

static int i2s_pcm_ioctl(struct snd_pcm_substream *stream,
			 unsigned int cmd, void *arg)
{
//	printk("TRACE: i2s_pcm_ioctl %d", cmd);

	switch (cmd) {
	case SNDRV_PCM_IOCTL1_CHANNEL_INFO:
		return i2s_pcm_channel_info(stream, arg);
	default:
		return snd_pcm_lib_ioctl(stream, cmd, arg);
	}
}


static const struct snd_pcm_ops rcm_i2s_pcm_ops = {
	.ioctl		= i2s_pcm_ioctl,
};

static const struct snd_soc_component_driver rcm_i2s_component = {
	.name = "rcm-i2s",
	.ops		= &rcm_i2s_pcm_ops,
};

static irqreturn_t i2s_interrupt_handler(int irq, void *data)
{
	struct rcm_i2s *i2s = (struct rcm_i2s *)data;
	unsigned int stat0;
	unsigned int stat1;
	regmap_read(i2s->regmap, RCM_I2S_REG_STAT0, &stat0);
	regmap_read(i2s->regmap, RCM_I2S_REG_STAT1, &stat1);

	printk("TRACE: i2s_interrupt_handler: %08X, %08X", stat0, stat1);

	return IRQ_HANDLED;
}


int rcm_dmaengine_pcm_prepare_slave_config(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct dma_slave_config *slave_config)
{
	int ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, slave_config);
	printk("TRACE: rcm_dmaengine_pcm_prepare_slave_config replace dst_maxburst from %d to %d", slave_config->dst_maxburst, params_channels(params)/2);

	// use dst_maxburst to indicate number of channels to be used
	slave_config->dst_maxburst = params_channels(params)/2;
	
	return ret;
}

static const struct snd_pcm_hardware i2s_playback_hw =
{
	.info =			(SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_COMPLEX |
				SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		RCM_I2S_FORMATS,
	.rates =		SNDRV_PCM_RATE_8000_192000,
	.rate_min =		8000,
	.rate_max =		192000,
	.channels_min =		2,
	.channels_max =		8,
	.buffer_bytes_max	= SIZE_MAX,
	.period_bytes_min	= 256,
	.period_bytes_max	= SIZE_MAX,
	.periods_min =		2,
	.periods_max =		2,	// number of DMA descriptors
};

static const struct snd_dmaengine_pcm_config pcm_conf = {
	.chan_names[SNDRV_PCM_STREAM_PLAYBACK] = "rx",
	.prepare_slave_config = rcm_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &i2s_playback_hw
};


static int rcm_i2s_probe(struct platform_device *pdev)
{
	struct rcm_i2s *i2s;
	void __iomem *base;
    int ret, irq;
	struct resource *res;

	printk("TRACE: rcm_i2s_probe");

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2s);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, base,
		&rcm_i2s_regmap_config);
	if (IS_ERR(i2s->regmap))
		return PTR_ERR(i2s->regmap);

	if(rcm_i2s_reset(i2s))
	{
        dev_err(&pdev->dev, "unable to find i2s hardware");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
        dev_err(&pdev->dev, "unable to get interrupt property");
		return -ENODEV;
    }
	ret = devm_request_irq(&pdev->dev, irq, i2s_interrupt_handler, 0,
			       pdev->name, i2s);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return  -ENODEV;
	};

	i2s->playback_dma_data.addr = res->start + RCM_I2S_REG_FIFO1;
	i2s->playback_dma_data.addr_width = 4;
	i2s->playback_dma_data.maxburst = 1;


	ret = devm_snd_soc_register_component(&pdev->dev, &rcm_i2s_component,
					 &rcm_i2s_dai, 1);

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &pcm_conf, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		return ret;
	}

	printk("TRACE: rcm_i2s_probe return %d", ret);

	return ret;

}


static int rcm_i2s_dev_remove(struct platform_device *pdev)
{
	//struct rcm_i2s *i2s = platform_get_drvdata(pdev);
	printk("TRACE: rcm_i2s_dev_remove");
	return 0;
}

static const struct of_device_id rcm_i2s_of_match[] = {
	{ .compatible = "rcm,i2s", },
	{},
};
MODULE_DEVICE_TABLE(of, rcm_i2s_of_match);

static struct platform_driver rcm_i2s_driver = {
	.driver = {
		.name = "rcm-i2s",
		.of_match_table = rcm_i2s_of_match,
	},
	.probe = rcm_i2s_probe,
	.remove = rcm_i2s_dev_remove,
};
module_platform_driver(rcm_i2s_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM I2S driver");
MODULE_LICENSE("GPL");
