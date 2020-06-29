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
#include <sound/asoundef.h>

static int g_spdif_debug = 0;

module_param_named(debug, g_spdif_debug, int, 0);

#define TRACE(format, ...)                                                     \
	do {                                                                   \
		if (g_spdif_debug) {                                           \
			printk("TRACE: rcm-spdif/%s:%d: " format "\n",         \
			       __func__, __LINE__, ##__VA_ARGS__);             \
		}                                                              \
	} while (0)

#define RCM_SPDIF_CLK_24 24576000
#define RCM_SPDIF_CLK_22 22579200

struct rcm_spdif {
	spinlock_t lock;
	struct regmap *regmap;
	struct clk *clock;
	struct device *dev;
	struct snd_soc_dai_driver dai_driver;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	unsigned char channel_status[24];
};

#define RCM_SPDIF_CTRL 0x00
#define RCM_SPDIF_STAT 0x08
#define RCM_SPDIF_MASK 0x10
#define RCM_SPDIF_NONLIN_PARAMS 0x18
#define RCM_SPDIF_BUF_CS1 0x20
#define RCM_SPDIF_BUF_CS2 0x28
#define RCM_SPDIF_BUF_UD 0x30
#define RCM_SPDIF_FIFO 0x38

enum { SPDIF_CTRL_FREQ_22_5792 = 0 << 7,
       SPDIF_CTRL_FREQ_24_576 = 1 << 7,
       SPDIF_CTRL_PROG_LEVEL_QUARTER = 0 << 14,
       SPDIF_CTRL_PROG_LEVEL_HALF = 1 << 14,
       SPDIF_CTRL_PROG_LEVEL_THREE_QUARTERS = 2 << 14,
       SPDIF_CTRL_BE = 1 << 22,
       SPDIF_CTRL_LE = 0 << 22,
       SPDIF_CTRL_DATA_PACK = 1 << 23,
       SPDIF_CTRL_MONO = 1 << 28,
       SPDIF_CTRL_COMPRESSED = 1 << 29,
       SPDIF_CTRL_BIT_RESET = 1 << 30,
       SPDIF_CTRL_START = 1 << 31 };

#define RCM_SPDIF_RATES SNDRV_PCM_RATE_KNOT

#define RCM_SPDIF_FORMATS                                                      \
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |                    \
		SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE

static const struct snd_ratnum clocks[2] = { {
						     .num = 192000,
						     .den_min = 1,
						     .den_max = 8,
						     .den_step = 1,
					     },
					     {
						     .num = 176400,
						     .den_min = 1,
						     .den_max = 8,
						     .den_step = 1,
					     } };

static const struct snd_pcm_hw_constraint_ratnums hw_constraints_clocks = {
	.nrats = 2,
	.rats = clocks,
};

static const struct regmap_config rcm_spdif_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 8,
	.val_bits = 32,
	.max_register = RCM_SPDIF_FIFO,
};

static int rcm_spdif_reset(struct rcm_spdif *spdif)
{
	unsigned int val;
	int ret;
	unsigned long flags;
	TRACE("");

	spin_lock_irqsave(&spdif->lock, flags);

	ret = regmap_update_bits(spdif->regmap, RCM_SPDIF_CTRL,
				 SPDIF_CTRL_BIT_RESET, SPDIF_CTRL_BIT_RESET);
	if (!ret)
		ret = regmap_read_poll_timeout(
			spdif->regmap, RCM_SPDIF_CTRL, val,
			(val & SPDIF_CTRL_BIT_RESET) == 0, 0, 200);

	spin_unlock_irqrestore(&spdif->lock, flags);

	return ret;
}

static int rcm_spdif_out_get_status_mask(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.iec958.status[0] = 0xff;
	ucontrol->value.iec958.status[1] = 0xff;
	ucontrol->value.iec958.status[2] = 0xff;
	ucontrol->value.iec958.status[3] = 0xff;
	ucontrol->value.iec958.status[4] = 0xff;

	return 0;
}

static int rcm_spdif_out_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int rcm_spdif_out_get_status(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(cpu_dai);

	TRACE("");
	memcpy(ucontrol->value.iec958.status, spdif->channel_status,
	       sizeof(ucontrol->value.iec958.status));
	return 0;
}

static int rcm_spdif_out_set_status(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(cpu_dai);

	TRACE("AES0 0x%02X AES1 0x%02X AES2 0x%02X AES3 0x%02X",
	      ucontrol->value.iec958.status[0],
	      ucontrol->value.iec958.status[1],
	      ucontrol->value.iec958.status[2],
	      ucontrol->value.iec958.status[3]);

	if (memcmp(spdif->channel_status, ucontrol->value.iec958.status,
		   sizeof(ucontrol->value.iec958.status))) {
		memcpy(spdif->channel_status, ucontrol->value.iec958.status,
		       sizeof(ucontrol->value.iec958.status));
		return 1;
	}
	return 0;
}

static struct snd_kcontrol_new rcm_spdif_out_controls[] = {
	{ .access = SNDRV_CTL_ELEM_ACCESS_READ,
	  .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	  .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
	  .info = rcm_spdif_out_info,
	  .get = rcm_spdif_out_get_status_mask },
	{ .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	  .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	  .info = rcm_spdif_out_info,
	  .get = rcm_spdif_out_get_status,
	  .put = rcm_spdif_out_set_status }
};

static int rcm_spdif_dai_probe(struct snd_soc_dai *dai)
{
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	TRACE("");

	snd_soc_dai_init_dma_data(dai, &spdif->playback_dma_data, 0);

	snd_soc_add_dai_controls(dai, rcm_spdif_out_controls,
				 ARRAY_SIZE(rcm_spdif_out_controls));

	return 0;
}

static int rcm_spdif_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = snd_pcm_hw_constraint_ratnums(substream->runtime, 0,
					    SNDRV_PCM_HW_PARAM_RATE,
					    &hw_constraints_clocks);

	TRACE("%d", ret);

	if (ret)
		return ret;

	ret = clk_prepare_enable(spdif->clock);

	return ret;
}

static void rcm_spdif_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	TRACE("");
	clk_disable_unprepare(spdif->clock);
}

static int rcm_spdif_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;

	TRACE("");
	spin_lock_irqsave(&spdif->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		regmap_update_bits(spdif->regmap, RCM_SPDIF_CTRL,
				   SPDIF_CTRL_START, SPDIF_CTRL_START);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		regmap_update_bits(spdif->regmap, RCM_SPDIF_CTRL,
				   SPDIF_CTRL_START, 0);
		break;
	default:
		return -EINVAL;
	}
	spin_unlock_irqrestore(&spdif->lock, flags);

	return 0;
}

static int rcm_spdif_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct rcm_spdif *spdif = snd_soc_dai_get_drvdata(dai);

	int width = snd_pcm_format_width(params_format(params));
	int rate = params_rate(params);
	int ctrl = 0, ret = -EINVAL;
	unsigned long req_clock = RCM_SPDIF_CLK_24;
	unsigned long flags;
	uint32_t *cs = (uint32_t *)&spdif->channel_status[0];
	int i;
	TRACE("%d, %d, %d", params_channels(params), params_rate(params),
	      params_format(params));

	if ((rate % 22050) == 0)
		req_clock = RCM_SPDIF_CLK_22;
	else
		ctrl |= SPDIF_CTRL_FREQ_24_576;

	if (spdif->clock) {
		if (clk_get_rate(spdif->clock) != req_clock) {
			unsigned long freq =
				clk_round_rate(spdif->clock, req_clock);
			if (req_clock == freq)
				ret = clk_set_rate(spdif->clock, freq);
		} else
			ret = 0;
	}

	if (ret) {
		dev_err(spdif->dev, "unable to set-up requirec clock");
		return ret;
	}

	ctrl |= (req_clock / 128 / rate) - 1;
	TRACE("div: %08X", ctrl);

	ctrl |= SPDIF_CTRL_PROG_LEVEL_HALF; //???

	if (snd_pcm_format_big_endian(params_format(params)))
		ctrl |= SPDIF_CTRL_BE;

	if (width == 16)
		ctrl |= SPDIF_CTRL_DATA_PACK;

	if ((spdif->channel_status[0] & IEC958_AES0_NONAUDIO) != 0)
		ctrl |= SPDIF_CTRL_COMPRESSED;

	TRACE("ctrl: %08X", ctrl);

	spin_lock_irqsave(&spdif->lock, flags);

	regmap_update_bits(spdif->regmap, RCM_SPDIF_CTRL, 0x3FFFFFF, ctrl);
#ifndef CONFIG_ARCH_RCM_K1879XB1
	regmap_write(spdif->regmap, RCM_SPDIF_MASK, 0x3f7fffff /*0x18000000*/);
#endif
	regmap_write(spdif->regmap, RCM_SPDIF_NONLIN_PARAMS,
		     0x1); // todo investigate

	for (i = 0; i != sizeof(spdif->channel_status) /
				 sizeof(spdif->channel_status[0]) / sizeof(*cs);
	     ++i, ++cs) {
		regmap_write(spdif->regmap, RCM_SPDIF_BUF_CS1, (*cs));
		regmap_write(spdif->regmap, RCM_SPDIF_BUF_CS2, (*cs));
	}
	spin_unlock_irqrestore(&spdif->lock, flags);

	return 0;
}

static const struct snd_soc_dai_ops rcm_spdif_dai_ops = {
	.startup = rcm_spdif_startup,
	.shutdown = rcm_spdif_shutdown,
	.trigger = rcm_spdif_trigger,
	.hw_params = rcm_spdif_hw_params,
};

static struct snd_soc_dai_driver rcm_spdif_dai = {
	.probe = rcm_spdif_dai_probe,
	.playback =
		{
			.rates = RCM_SPDIF_RATES,
			.formats = RCM_SPDIF_FORMATS,
			.channels_min = 2,
			.channels_max = 2,
		},
	.ops = &rcm_spdif_dai_ops,
};

static const struct snd_pcm_hardware spdif_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = RCM_SPDIF_FORMATS,
	.rates = RCM_SPDIF_RATES,
	.rate_min = 22050,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = SIZE_MAX,
	.period_bytes_min = 256,
	.period_bytes_max = SIZE_MAX,
	.periods_min = 2,
	.periods_max = 2, // number of DMA descriptors
};

static const struct snd_soc_component_driver rcm_spdif_component = {
	.name = "rcm-spdif",
};

static const struct snd_dmaengine_pcm_config pcm_conf = {
	.chan_names[SNDRV_PCM_STREAM_PLAYBACK] = "rx",
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &spdif_playback_hw
};

static irqreturn_t spdif_interrupt_handler(int irq, void *data)
{
	struct rcm_spdif *spdif = (struct rcm_spdif *)data;
	unsigned int stat;
	regmap_read(spdif->regmap, RCM_SPDIF_STAT, &stat);
	regmap_write(spdif->regmap, RCM_SPDIF_STAT, 0xffffffff);
	TRACE("stat: %08X", stat);

	return IRQ_HANDLED;
}

static int rcm_spdif_probe(struct platform_device *pdev)
{
	struct rcm_spdif *spdif;

	void __iomem *base;
	int ret, irq;
	struct resource *res;

	TRACE("");

	spdif = devm_kzalloc(&pdev->dev, sizeof(*spdif), GFP_KERNEL);
	if (!spdif)
		return -ENOMEM;

	platform_set_drvdata(pdev, spdif);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	spdif->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					      &rcm_spdif_regmap_config);
	if (IS_ERR(spdif->regmap))
		return PTR_ERR(spdif->regmap);

	spin_lock_init(&spdif->lock);

	if (rcm_spdif_reset(spdif)) {
		dev_err(&pdev->dev, "unable to find spdif hardware");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get interrupt property");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, irq, spdif_interrupt_handler, 0,
			       pdev->name, spdif);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return -ENODEV;
	};

	spdif->clock = devm_clk_get(&pdev->dev, 0);
	if (IS_ERR(spdif->clock)) {
		dev_err(&pdev->dev, "Cannot find clock source\n");
		return -ENODEV;
	}

	spdif->playback_dma_data.addr =
		res->start + RCM_SPDIF_FIFO; // +1 means SPDIF mode for FIFO
	spdif->playback_dma_data.addr_width = 4;
	spdif->playback_dma_data.maxburst = 1;

	clk_prepare_enable(spdif->clock);

	ret = devm_snd_soc_register_component(&pdev->dev, &rcm_spdif_component,
					      &rcm_spdif_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &pcm_conf, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		goto err_clk_disable;
	}

	return 0;

err_clk_disable:
	clk_disable_unprepare(spdif->clock);
	TRACE("return %d", ret);

	return ret;
}

static const struct of_device_id rcm_spdif_of_match[] = {
	{
		.compatible = "rcm,spdif",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rcm_spdif_of_match);

static struct platform_driver rcm_spdif_driver = {
	.driver =
		{
			.name = "rcm-spdif",
			.of_match_table = rcm_spdif_of_match,
		},
	.probe = rcm_spdif_probe,
};
module_platform_driver(rcm_spdif_driver);

MODULE_AUTHOR("Alexey Spirkov <dev@alsp.net>");
MODULE_DESCRIPTION("RCM SPDIF driver");
MODULE_LICENSE("GPL");
