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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/dmaengine.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include "rcm-i2s.h"

static int g_i2s_debug = 0;

module_param_named(debug, g_i2s_debug, int, 0);

#define TRACE(format, ...)                                                     \
	do {                                                                   \
		if (g_i2s_debug) {                                             \
			printk("TRACE: rcm-i2s/%s:%d: " format "\n", __func__, \
			       __LINE__, ##__VA_ARGS__);                       \
		}                                                              \
	} while (0)

#define RCM_I2S_CLK_24 24576000
#define RCM_I2S_CLK_22 22579200

struct rcm_i2s {
	struct device *dev;
	struct rcm_i2s_control *ctrl;
	struct snd_soc_dai_driver dai_driver;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	int id;
	char dai_name[16];
};

static int rcm_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	TRACE("%d", i2s->id);

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data, 0);

	return 0;
}

static int rcm_i2s_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct rcm_i2s_control *i2s_ctrl = i2s->ctrl;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;

	runtime->private_data = (void *)(uintptr_t)i2s_ctrl;

	if (i2s_ctrl->opened_count == 0) {
		i2s_ctrl->rcm_i2s_reset(i2s_ctrl);
		clk_prepare_enable(i2s_ctrl->clock);
	}

	spin_lock_irqsave(&i2s_ctrl->lock, flags);
	i2s_ctrl->opened_count++;
	spin_unlock_irqrestore(&i2s_ctrl->lock, flags);

	TRACE("%d, %d", i2s->id, i2s_ctrl->opened_count);
	return 0;
}

static void rcm_i2s_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct rcm_i2s_control *i2s_ctrl = i2s->ctrl;
	unsigned long flags;

	spin_lock_irqsave(&i2s_ctrl->lock, flags);
	i2s_ctrl->opened_count--;
	spin_unlock_irqrestore(&i2s_ctrl->lock, flags);

	if (i2s_ctrl->opened_count == 0) {
		i2s_ctrl->rcm_i2s_reset(i2s_ctrl);
		clk_disable_unprepare(i2s_ctrl->clock);
	}

	TRACE("%d %d", i2s->id, i2s_ctrl->opened_count);
}

static int rcm_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct rcm_i2s_control *i2s_ctrl = i2s->ctrl;
	unsigned long flags;

	int ret = 0;

	TRACE("%d, %d, %d, %d", i2s->id, cmd, i2s_ctrl->opened_count,
	      i2s_ctrl->started_count);

	spin_lock_irqsave(&i2s_ctrl->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		i2s_ctrl->started_count++;
		if (i2s_ctrl->started_count ==
		    i2s_ctrl->opened_count) // sync start
		{
			ret = regmap_update_bits(i2s_ctrl->regmap,
						 RCM_I2S_REG_CTRL1, 0x200,
						 0x200);
			TRACE("on");
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		i2s_ctrl->started_count--;
		if (i2s_ctrl->started_count ==
		    0) // don't need sync stop to use channels separately
		{
			ret = regmap_update_bits(i2s_ctrl->regmap,
						 RCM_I2S_REG_CTRL1, 0x200, 0x0);
			TRACE("off");
		}
		break;
	default:
		return -EINVAL;
	}

	spin_unlock_irqrestore(&i2s_ctrl->lock, flags);

	//	snd_pcm_group_for_each_entry(s, substream) {
	//		if (snd_pcm_substream_chip(s) == chip) {
	//			snd_pcm_trigger_done(s, substream);
	//		}
	//	}

	return ret;
}

static int rcm_i2s_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct rcm_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	struct rcm_i2s_control *i2s_ctrl = i2s->ctrl;
	int width = snd_pcm_format_width(params_format(params));
	int rate = params_rate(params);
	int is_big_endian = snd_pcm_format_big_endian(params_format(params));
	int div_value;
	unsigned long req_clock = RCM_I2S_CLK_24;
	int ctrl0 = 0;
	unsigned long flags;
	if ((rate % 22050) == 0)
		req_clock = RCM_I2S_CLK_22;

	div_value = req_clock / (rate) / width / 4;
	switch (width) {
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

	if (is_big_endian)
		ctrl0 |= 0x4;

	//	ctrl0 |= 0x2;

	TRACE("%d, %d, %d", params_channels(params), params_rate(params),
	      params_format(params));

	if ((i2s_ctrl->hw_params[0] == ctrl0) &&
	    (i2s_ctrl->hw_params[1] == div_value)) {
		// already changed just skip
		return 0;
	}

	if (i2s_ctrl->hw_params[0] == 0) {
		int ret = 0;
		spin_lock_irqsave(&i2s_ctrl->lock, flags);

		// set clock
		if (i2s_ctrl->clock) {
			if (clk_get_rate(i2s_ctrl->clock) != req_clock) {
				unsigned long freq = clk_round_rate(
					i2s_ctrl->clock, req_clock);
				if (req_clock == freq)
					ret = clk_set_rate(i2s_ctrl->clock,
							   freq);
			}
			if (!ret) {
				i2s_ctrl->hw_params[0] = ctrl0;
				i2s_ctrl->hw_params[1] = div_value;

				regmap_write(i2s_ctrl->regmap,
					     RCM_I2S_REG_CTRL0, ctrl0);
				regmap_write(i2s_ctrl->regmap,
					     RCM_I2S_REG_CTRL1, div_value);
			} else
				dev_err(i2s->dev,
					"Failed to set clock rate %ld\n",
					req_clock);
		}
		spin_unlock_irqrestore(&i2s_ctrl->lock, flags);
		return ret;
	} else {
		dev_err(i2s->dev, "requested conflict i2s parameters");
		return -EBUSY;
	}
}

static const struct snd_soc_dai_ops rcm_i2s_dai_ops = {
	.startup = rcm_i2s_startup,
	.shutdown = rcm_i2s_shutdown,
	.trigger = rcm_i2s_trigger,
	.hw_params = rcm_i2s_hw_params,
};

#define RCM_I2S_FORMATS                                                        \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |                   \
	 SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)

// do not support theese modes because it seems kernel unable to organize samples
// per words
/*	 SNDRV_PCM_FMTBIT_S20_LE | SNDRV_PCM_FMTBIT_S20_BE |                   \
	 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE |                   \
*/

static const struct snd_soc_component_driver rcm_i2s_component = {
	.name = "rcm-i2s",
};

static const struct snd_pcm_hardware i2s_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_SYNC_START),
	.formats = RCM_I2S_FORMATS,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = SIZE_MAX,
	.period_bytes_min = 256,
	.period_bytes_max = SIZE_MAX,
	.periods_min = 2,
	.periods_max = 2, // number of DMA descriptors
};

static const struct snd_dmaengine_pcm_config pcm_conf = {
	.chan_names[SNDRV_PCM_STREAM_PLAYBACK] = "rx",
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &i2s_playback_hw
};

#define SNDRV_PCM_RATE_8000_192000_SKIP_11025                                  \
	SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |    \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 |                  \
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000

static int rcm_i2s_probe(struct platform_device *pdev)
{
	struct rcm_i2s *i2s;
	int ret, id;
	struct platform_device *i2s_ctrl_pdev;
	struct of_phandle_args args;
	struct rcm_i2s_control *ctrl;
	struct snd_soc_dai_driver *i2s_dai;
	struct snd_soc_pcm_stream i2s_playback;

	TRACE("");

	if (of_parse_phandle_with_args(pdev->dev.of_node, "ctrl", "#i2s-cells",
				       0, &args)) {
		dev_err(&pdev->dev, "unable to find i2s control hardware");
		return -ENODEV;
	}
	i2s_ctrl_pdev = of_find_device_by_node(args.np);
	id = args.args[0];
	of_node_put(args.np);

	ctrl = platform_get_drvdata(i2s_ctrl_pdev);
	if (!ctrl)
		return -ENODEV;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s_dai = devm_kzalloc(&pdev->dev, sizeof(*i2s_dai), GFP_KERNEL);
	if (!i2s_dai)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2s);

	i2s->ctrl = ctrl;
	i2s->id = id;
	i2s->dev = &pdev->dev;

	i2s->playback_dma_data.addr =
		ctrl->res->start + RCM_I2S_REG_FIFO0 + id * 8;
	i2s->playback_dma_data.addr_width = 4;
	i2s->playback_dma_data.maxburst = 1;

	i2s_playback.channels_min = 2;
	i2s_playback.channels_max = 2;
	i2s_playback.rates = i2s->ctrl->disable_11025 ?
				     SNDRV_PCM_RATE_8000_192000_SKIP_11025 :
				     SNDRV_PCM_RATE_8000_192000;
	i2s_playback.formats = RCM_I2S_FORMATS;

	snprintf(&i2s->dai_name[0], 16, "rcm-i2s%1d", id);
	i2s_dai->name = &i2s->dai_name[0];
	i2s_dai->id = id;
	i2s_dai->probe = rcm_i2s_dai_probe;
	i2s_dai->playback = i2s_playback;
	i2s_dai->ops = &rcm_i2s_dai_ops;
	i2s_dai->symmetric_rates = 1;

	ret = devm_snd_soc_register_component(&pdev->dev, &rcm_i2s_component,
					      i2s_dai, 1);

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &pcm_conf, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		return ret;
	}

	TRACE("return %d", ret);

	return ret;
}

static int rcm_i2s_dev_remove(struct platform_device *pdev)
{
	//struct rcm_i2s *i2s = platform_get_drvdata(pdev);
	TRACE("");
	return 0;
}

static const struct of_device_id rcm_i2s_of_match[] = {
	{
		.compatible = "rcm,i2s",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rcm_i2s_of_match);

static struct platform_driver rcm_i2s_driver = {
	.driver =
		{
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
