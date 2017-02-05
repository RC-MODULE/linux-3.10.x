/*
 * sound/arm/maudio.c - Module SPDIF and I2S driver
 *
 *	Copyright (C) 2010 Module
 *	Written by Kirill Mikhailov
 *
 *	This file is subject to the terms and conditions of the GNU General Public
 *	License. See the file COPYING in the main directory of this archive for
 *	more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <sound/soc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <sound/asoundef.h>

#define DRIVER_NAME "maudio"

#define MAUDIO_DECLARE_PARAM(name, def) \
	static int g_maudio_##name = def; \
	module_param_named(name, g_maudio_##name, int, 0);

MAUDIO_DECLARE_PARAM(debug, 0);

#define TRACE(format, ... ) do { \
		if(g_maudio_debug) { \
			printk(KERN_DEBUG DRIVER_NAME "/%s:%d: " format "\n", \
				__func__, __LINE__, ##__VA_ARGS__); \
		} \
	} while(0)

#define ERROR(format, ... ) do { \
		printk(KERN_ERR DRIVER_NAME "/%s:%d: " format "\n", \
			__func__, __LINE__, ##__VA_ARGS__); \
	} while(0)

struct mem_resource {
	struct resource res;
	void __iomem *io;
};

#define MAUDIO_DEBUGFS_BLOB_SIZE 50

struct maudio_chip {
	struct platform_device *dev;

	struct snd_card *card;
	struct device_node *dmac_of_node;
	struct device_node *i2s_of_node;
	struct device_node *spdif_of_node;

	struct mem_resource dmac;
	struct mem_resource i2s;
	struct mem_resource spdif;
	struct mem_resource control;

	int irq;

	dma_addr_t dma_handle;
	size_t dma_size;
	void *dma_area;

	spinlock_t lock;

	enum {
		MAUDIO_CHIP_STATE_IDLE,
		MAUDIO_CHIP_STATE_I2S,
		MAUDIO_CHIP_STATE_SPDIF
	} state;

	int channels;

	struct snd_pcm_substream *stream;
	snd_pcm_uframes_t hw_ptr;

	unsigned char spdif_channel_status[24];

#ifdef CONFIG_SND_MAUDIO_DEBUG
	struct dentry *debugfs_dir;

	uint32_t dmac_dma_addr_page0;
	uint32_t dmac_dma_addr_page1;

	struct debugfs_blob_wrapper debugfs_irqstat_blob;
	uint32_t debugfs_irqstat_data[MAUDIO_DEBUGFS_BLOB_SIZE];
	uint32_t debugfs_irqstat_index;
#endif
};

struct maudio_chip *g_chip;

static inline uint32_t i2s_read_stat0(struct maudio_chip *chip);
static inline uint32_t i2s_read_stat1(struct maudio_chip *chip);
static inline uint32_t dmac_read_dmaaddr(struct maudio_chip *chip);
static irqreturn_t maudio_interrupt_handler(int irq, void *data);

/* Chip init/free {{{ */
static inline struct maudio_chip *card_chip(struct snd_card *card)
{
	g_chip = card->private_data;
	return card->private_data;
}

static int maudio_chip_init(struct maudio_chip *chip,
			    struct platform_device *dev,
			    struct snd_card *card, size_t dma_size)
{
	chip->dev = dev;
	chip->card = card;

	spin_lock_init(&chip->lock);

	chip->state = MAUDIO_CHIP_STATE_IDLE;

	chip->stream = 0;
	chip->hw_ptr = 0;

	memset(chip->spdif_channel_status, 0,
	       sizeof(chip->spdif_channel_status));

	chip->dma_size = dma_size;
	chip->dma_area = dma_alloc_writecombine(
		&dev->dev, dma_size, &chip->dma_handle, GFP_KERNEL);

	if (!chip->dma_area)
		return -ENOMEM;

	return 0;
}

static void maudio_chip_free(struct maudio_chip *chip)
{
	if (chip->dma_area) {
		dma_free_writecombine(&chip->dev->dev, chip->dma_size,
				      chip->dma_area, chip->dma_handle);
	}

	memset(chip, 0, sizeof(chip));
}
/* }}} */

/* Chip debugging stuff {{{ */
#ifdef CONFIG_SND_MAUDIO_DEBUG
static void maudio_chipdebug_reset(struct maudio_chip *chip)
{
	chip->dmac_dma_addr_page0 = 0;
	chip->dmac_dma_addr_page1 = 0;
	chip->debugfs_irqstat_index = 0;
	memset(&chip->debugfs_irqstat_data, 0,
		sizeof(chip->debugfs_irqstat_data));
}

static void maudio_statuswatch_check(struct maudio_chip *chip,
	uint32_t dmac_int_status)
{
	uint32_t next;
	uint32_t *pdata = chip->debugfs_irqstat_data;
	if(dmac_int_status != pdata[chip->debugfs_irqstat_index]) {
		next = (chip->debugfs_irqstat_index + 1);
		next %= MAUDIO_DEBUGFS_BLOB_SIZE;
		pdata[next] = dmac_int_status;
		chip->debugfs_irqstat_index = next;
	}
}

static void maudio_dmaaddr_check(struct maudio_chip *chip, 
	struct snd_pcm_substream *stream, uint32_t expected_hw_ptr)
{
	uint32_t addr;
	uint32_t bufsz;

	if(!snd_pcm_running(stream)) {
		/* Somebody have just closed the stream */
		return;
	}

	bufsz = frames_to_bytes(stream->runtime, 
		stream->runtime->period_size);

	expected_hw_ptr = frames_to_bytes(stream->runtime, expected_hw_ptr) +
		chip->dma_handle;

	addr = dmac_read_dmaaddr(chip);
	if(addr == 0) {
		/* dmaadr is undocumented debug register. From time to time it
		 * is read as 0. Skip checking for those cases.*/
		return;
	}

	if(expected_hw_ptr == chip->dma_handle) {
		chip->dmac_dma_addr_page0 = addr;
	} else {
		chip->dmac_dma_addr_page1 = addr;
	}

	if((addr < expected_hw_ptr) || (addr >= expected_hw_ptr + bufsz)) {
		ERROR("DMA out of bounds: expected 0x%08X-0x%08X got 0x%08X",
			expected_hw_ptr, expected_hw_ptr+bufsz, addr);
	}
}

static int maudio_debugfs_init(struct maudio_chip *chip)
{
	chip->debugfs_dir = debugfs_create_dir("maudio", NULL);
	if(IS_ERR(chip->debugfs_dir)) {
		chip->debugfs_dir = NULL;
		return -EINVAL;
	}
	debugfs_create_u32("dma_addr_page0", 0400,
		chip->debugfs_dir, &chip->dmac_dma_addr_page0);
	debugfs_create_u32("dma_addr_page1", 0400,
		chip->debugfs_dir, &chip->dmac_dma_addr_page1);

	chip->debugfs_irqstat_blob.data = &chip->debugfs_irqstat_data[0];
	chip->debugfs_irqstat_blob.size = sizeof(chip->debugfs_irqstat_data);
	debugfs_create_blob("irq_status_data", 0400,
		chip->debugfs_dir, &chip->debugfs_irqstat_blob);
	debugfs_create_u32("irq_status_index", 0400,
		chip->debugfs_dir, &chip->debugfs_irqstat_index);
	return 0;
}

static void maudio_debugfs_free(struct maudio_chip *chip)
{
	if(chip->debugfs_dir)
		debugfs_remove_recursive(chip->debugfs_dir);
}
#else
static inline void maudio_chipdebug_reset(struct maudio_chip *chip) {}
static inline void maudio_statuswatch_check(struct maudio_chip *chip,
	uint32_t dmac_int_status) {}
static inline void maudio_dmaaddr_check(struct maudio_chip *chip, 
	struct snd_pcm_substream *stream, uint32_t expected_hw_ptr) {}
static inline int maudio_debugfs_init(struct maudio_chip *chip) { return 0; }
static inline void maudio_debugfs_free(struct maudio_chip *chip) { }
#endif /* CONFIG_SND_MAUDIO_DEBUG */
/*}}}*/

/* {{{ DMAC stuff */
static inline void dmac_write_register(struct maudio_chip *chip,
				       uint32_t offset, uint32_t data)
{
	TRACE("reg[0x%08X] <- 0x%08X", offset, data);
	iowrite32(data, chip->dmac.io + offset);
}

static inline void dmac_write_ena(struct maudio_chip *chip, uint32_t data)
{
	dmac_write_register(chip, 0, data);
}


static inline uint32_t dmac_read_ena(struct maudio_chip *chip)
{
	return ioread32(chip->dmac.io);
}

static inline void dmac_write_chbase(struct maudio_chip *chip, int channel,
				     int page, dma_addr_t address)
{
	BUG_ON(channel < 0 || channel > 3);
	BUG_ON(page < 0 || page > 1);
	dmac_write_register(chip,
			    0x4 + (channel * 2 + page) * sizeof(uint32_t),
			    address);
}

static inline uint32_t dmac_read_chbase(struct maudio_chip *chip,
					int channel, int page)
{
	return ioread32(chip->dmac.io + 0x4 +
			(channel * 2 + page) * sizeof(uint32_t));
}

static inline void dmac_write_chend(struct maudio_chip *chip, int channel,
				    int page, dma_addr_t address)
{
	BUG_ON(channel < 0 || channel > 3);
	BUG_ON(page < 0 || page > 1);
	dmac_write_register(chip,
			    0x24 + (channel * 2 + page) * sizeof(uint32_t),
			    address);
}

static inline void dmac_write_slvbase(struct maudio_chip *chip,
				      int channel, dma_addr_t address)
{
	BUG_ON(channel < 0 || channel > 3);
	dmac_write_register(chip, 0x44 + channel * sizeof(uint32_t), address);
}

static inline void dmac_write_trw(struct maudio_chip *chip, int channel,
				  uint32_t data)
{
	BUG_ON(channel < 0 || channel > 3);
	TRACE("trw[%d] <- 0x%08X", channel, data);
	dmac_write_register(chip, 0x54 + channel * sizeof(uint32_t), data);
}

static inline void dmac_write_slvovrh(struct maudio_chip *chip,
				      uint32_t data)
{
	dmac_write_register(chip, 0x64, data);
}

/* Size of buffer of slave devices (i2s or spdif) in 32-bit words minus 1. */
static inline void dmac_write_slvbsize(struct maudio_chip *chip,
				       int channel, uint32_t data)
{
	BUG_ON(channel < 0 || channel > 3);
	TRACE("slvbsize[%d] <- 0x%08X", channel, data);
	dmac_write_register(chip, 0x68 + channel * sizeof(uint32_t), data);
}

static inline void dmac_write_axiparam(struct maudio_chip *chip,
				       uint32_t data)
{
	dmac_write_register(chip, 0x78, data);
}

static inline void dmac_write_int_mask(struct maudio_chip *chip,
				       uint32_t data)
{
	dmac_write_register(chip, 0x7c, data);
}

static inline uint32_t dmac_read_int_mask(struct maudio_chip *chip)
{
	return ioread32(chip->dmac.io + 0x7c);
}

static inline uint32_t dmac_read_bufstat(struct maudio_chip *chip)
{
	return ioread32(chip->dmac.io + 0x84);
}

static inline void dmac_write_int(struct maudio_chip *chip, uint32_t data)
{
	dmac_write_register(chip, 0x80, data);
}

static inline uint32_t dmac_read_int(struct maudio_chip *chip)
{
	return ioread32(chip->dmac.io + 0x80);
}

/* Debug-mode undocumented register, containing DMA address currently used by
 * device. May be read as 0 from time to time. */
static inline uint32_t dmac_read_dmaaddr(struct maudio_chip *chip)
{
	return ioread32(chip->dmac.io + 0x9C);
}

enum {
	DMAC_ENA_CHANNEL0_ACTIVE = 0x1,
	DMAC_ENA_CHANNEL1_ACTIVE = 0x2,
	DMAC_ENA_CHANNEL2_ACTIVE = 0x4,
	DMAC_ENA_CHANNEL3_ACTIVE = 0x8,
	DMAC_ENA_SPDIF = 1 << 4,
	DMAC_ENA_CHANNEL0_SW = 1 << 8,
	DMAC_ENA_CHANNEL1_SW = 1 << 9,
	DMAC_ENA_CHANNEL2_SW = 1 << 10,
	DMAC_ENA_CHANNEL3_SW = 1 << 11
};

enum {
	DMAC_INT_CHANNEL0_WREND = 1 << 0,
	DMAC_INT_CHANNEL1_WREND = 1 << 1,
	DMAC_INT_CHANNEL2_WREND = 1 << 2,
	DMAC_INT_CHANNEL3_WREND = 1 << 3,
	DMAC_INT_CHANNEL0_STOP = 1 << 16,
	DMAC_INT_CHANNELS_STOP = 0xF << 16
};

enum {
	DMAC_INT_MASK_CHANNEL0_WREND = 1 << 0,
	DMAC_INT_MASK_CHANNEL0_STOP = 1 << 16
};

enum {
	MAUDIO_CONTROL_DMA_OFFSET = 0x28
};

static void dmac_reset(struct maudio_chip *chip)
{
	dmac_write_ena(chip, 0x8000);
	dmac_write_ena(chip, 0);
	iowrite32(1, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
	iowrite32(0, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
	iowrite32(1, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
}

static int dmac_hardware_init(struct platform_device *pdev,
			      struct maudio_chip *chip)
{
	iowrite32(1, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
	iowrite32(0, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
	iowrite32(1, chip->control.io + MAUDIO_CONTROL_DMA_OFFSET);
	dmac_reset(chip);
	return 0;
}

static void dmac_hardware_term(struct platform_device *pdev,
			       struct maudio_chip *chip)
{
}

static void dmac_start(struct maudio_chip *chip, int channels)
{
	struct snd_pcm_runtime *runtime = chip->stream->runtime;
	size_t size;
	int32_t mask;
	uint32_t ena;
	int i;

	maudio_chipdebug_reset(chip);
		
	ena = chip->state == MAUDIO_CHIP_STATE_SPDIF ? DMAC_ENA_SPDIF : 0;
	size = frames_to_bytes(runtime, runtime->buffer_size) / channels;

	BUG_ON(chip->state == MAUDIO_CHIP_STATE_IDLE);

	chip->channels = channels;

	memset(chip->dma_area, 0, chip->dma_size);

	for (i = 0; i < channels; ++i) {
		dma_addr_t page0_start = runtime->dma_addr + size * i;
		dma_addr_t page0_end = page0_start + size / 2;
		dma_addr_t page1_start = page0_end;
		dma_addr_t page1_end = page0_start + size;
		TRACE("ch%d: dma 0x%08X-0x%08X 0x%08X-0x%08X", i,
			page0_start, page0_end, page1_start, page1_end);

		/* dma starts from page #1 */
		dmac_write_chbase(chip, i, 0, page1_start);
		dmac_write_chbase(chip, i, 1, page0_start);
		dmac_write_chend(chip, i, 0, page1_end);
		dmac_write_chend(chip, i, 1, page0_end);

		if (chip->state == MAUDIO_CHIP_STATE_I2S)
			dmac_write_slvbase(chip, i,
				chip->i2s.res.start + (0x10 + i * 4) * 2);
		else
			dmac_write_slvbase(chip, i,
				chip->spdif.res.start + 0x1C * 2);

		dmac_write_slvbsize(chip, i, 31);

		dmac_write_trw(chip, i, (size / 2 / sizeof(uint32_t)) - 1);

		ena |= ((DMAC_ENA_CHANNEL0_ACTIVE | DMAC_ENA_CHANNEL0_SW) << i);
	}

	dmac_write_ena(chip, ena);
	dmac_write_int(chip, -1);

	/* enabling interrupt from the last channel */
	mask = (DMAC_INT_MASK_CHANNEL0_WREND) << (channels-1);
	dmac_write_int_mask(chip, mask);
}

static void dmac_request_stop(struct maudio_chip *chip)
{
	if (chip->state != MAUDIO_CHIP_STATE_IDLE) {
		dmac_write_ena(chip,
			       chip->state ==
			       MAUDIO_CHIP_STATE_SPDIF ? DMAC_ENA_SPDIF :
			       0);
	}
}

static void dmac_wait_stop(struct maudio_chip *chip)
{
	while ((dmac_read_int(chip) & DMAC_INT_CHANNELS_STOP) !=
	       DMAC_INT_CHANNELS_STOP) {
		schedule();
		TRACE("dma_status 0x%08X i2s0_status 0x%08X i2s1_status 0x%08X",
		      dmac_read_int(chip), i2s_read_stat0(chip), 
		      i2s_read_stat1(chip));
	}

	dmac_write_ena(chip, 0);
	dmac_reset(chip);
}
/* }}} */

/* I2S stuff {{{ */
static int i2s_hardware_init(struct platform_device *pdev,
			     struct maudio_chip *chip)
{
	return 0;
}

static void i2s_hardware_term(struct platform_device *pdev,
			      struct maudio_chip *chip)
{
}

static inline void i2s_write_register(struct maudio_chip *chip,
				      uint32_t reg, uint32_t data)
{
	TRACE("reg[0x%X] <- 0x%08X", reg, data);
	iowrite32(data, chip->i2s.io + reg);
}

static inline uint32_t i2s_read_register(struct maudio_chip *chip,
					 uint32_t reg)
{
	uint32_t t = ioread32(chip->i2s.io + reg);
	TRACE("reg[0x%X] -> 0x%08X", reg, t);
	return t;
}

static inline void i2s_write_ctrl0(struct maudio_chip *chip, uint32_t data)
{
	i2s_write_register(chip, 0, data);
}

static inline uint32_t i2s_read_ctrl0(struct maudio_chip *chip)
{
	return i2s_read_register(chip, 0);
}

static inline void i2s_write_ctrl1(struct maudio_chip *chip, uint32_t data)
{
	i2s_write_register(chip, 0x4 * 2, data);
}

static inline uint32_t i2s_read_ctrl1(struct maudio_chip *chip)
{
	return i2s_read_register(chip, 0x4 * 2);
}

static inline uint32_t i2s_read_stat0(struct maudio_chip *chip)
{
	return i2s_read_register(chip, 0x08 * 2);
}

static inline uint32_t i2s_read_stat1(struct maudio_chip *chip)
{
	return i2s_read_register(chip, 0xc * 2);
}

enum {
	I2S_PERIOD_SIZE = 1024 * 24 * 2
};

static int i2s_pcm_open(struct snd_pcm_substream *stream)
{
	static struct snd_pcm_hardware hw = {
		.info = (SNDRV_PCM_INFO_MMAP
			 | SNDRV_PCM_INFO_MMAP_VALID
			 | SNDRV_PCM_INFO_COMPLEX),
		.formats = SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_S32
		    /*| SNDRV_PCM_FMTBIT_S24 */ ,
		.rates =
		    SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_32000 |
		    SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
		    SNDRV_PCM_RATE_192000,
		.rate_min = 32000,
		.rate_max = 192000,
		.channels_min = 2,
		.channels_max = 8,
		.buffer_bytes_max = I2S_PERIOD_SIZE * 2,
		.period_bytes_min = I2S_PERIOD_SIZE,
		.period_bytes_max = I2S_PERIOD_SIZE,
		.periods_min = 2,
		.periods_max = 2
	};

	TRACE("pcm open");

	stream->runtime->hw = hw;
	return 0;
}

static int i2s_pcm_close(struct snd_pcm_substream *stream)
{
	unsigned long flags;
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);

	TRACE("pcm_close");

	dmac_request_stop(chip);
	dmac_wait_stop(chip);

	spin_lock_irqsave(&chip->lock, flags);

	chip->stream = 0;
	chip->state = MAUDIO_CHIP_STATE_IDLE;

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int i2s_pcm_prepare(struct snd_pcm_substream *stream)
{
	unsigned long flags;
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);

	TRACE("period_size %lu buffer_size %lu frame_bits %d "
	      "sample_bits %d nchannels %d boundary 0x%lX",
	      stream->runtime->period_size, stream->runtime->buffer_size,
	      stream->runtime->frame_bits, stream->runtime->sample_bits,
	      stream->runtime->channels, stream->runtime->boundary);

	dmac_wait_stop(snd_pcm_substream_chip(stream));

	spin_lock_irqsave(&chip->lock, flags);

	if (chip->state != MAUDIO_CHIP_STATE_IDLE
	    && chip->stream != stream) {
		spin_unlock_irqrestore(&chip->lock, flags);
		return -EBUSY;
	}

	chip->state = MAUDIO_CHIP_STATE_I2S;
	chip->stream = stream;

	spin_unlock_irqrestore(&chip->lock, flags);
	return 0;
}

static void i2s_reset(struct maudio_chip *chip)
{
	i2s_write_ctrl0(chip, 0x1);
	while (i2s_read_ctrl0(chip) != 0);
}

static int i2s_pcm_trigger(struct snd_pcm_substream *stream, int cmd)
{
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);
	uint32_t ctrl0;
	uint32_t ctrl1;

	TRACE("action %d stat0 0x%08X stat1 0x%08X", cmd,
	      i2s_read_stat0(chip), i2s_read_stat1(chip));
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ctrl0 = 0;
		ctrl1 = 0;

		switch (snd_pcm_format_width(stream->runtime->format)) {
		case 16:
			ctrl0 |= 0xF0008;
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

		ctrl1 =
		    24576 / (stream->runtime->rate / 1000) /
		    snd_pcm_format_width(stream->runtime->format) / 4;

		i2s_reset(chip);
		chip->hw_ptr = 0;

		dmac_start(chip, stream->runtime->channels / 2);
		i2s_write_ctrl0(chip, ctrl0);
		i2s_write_ctrl1(chip, ctrl1 | 0x200);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		dmac_request_stop(chip);
		return 0;
	}

	return -EINVAL;
}

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

	TRACE("channel %d offset %lu first %d step %d",
	      info->channel, info->offset, info->first, info->step);

	return 0;
}

static int i2s_pcm_ioctl(struct snd_pcm_substream *stream,
			 unsigned int cmd, void *arg)
{
	TRACE("ioctl %d", cmd);

	switch (cmd) {
	case SNDRV_PCM_IOCTL1_CHANNEL_INFO:
		return i2s_pcm_channel_info(stream, arg);
	default:
		return snd_pcm_lib_ioctl(stream, cmd, arg);
	}
}
/* I2S stuff }}} */

/* SPDIF stuff {{{ */
static inline uint32_t spdif_read_ctrl(struct maudio_chip *chip)
{
	return ioread32(chip->spdif.io);
}

static inline void spdif_write_ctrl(struct maudio_chip *chip,
				    uint32_t data)
{
	TRACE("0x%08X", data);
	iowrite32(data, chip->spdif.io);
}

static inline uint32_t spdif_read_stat(struct maudio_chip *chip)
{
	return ioread32(chip->spdif.io + 8);
}

static inline void spdif_write_stat(struct maudio_chip *chip,
				    uint32_t data)
{
	TRACE("0x%08X", data);
	iowrite32(data, chip->spdif.io + 8);
}

static inline uint32_t spdif_read_int_en(struct maudio_chip *chip)
{
	return ioread32(chip->spdif.io + 0x10);
}

static inline void spdif_write_int_en(struct maudio_chip *chip,
				      uint32_t data)
{
	TRACE("0x%08X", data);
	iowrite32(data, chip->spdif.io + 0x10);
}

static inline uint32_t spdif_read_non_linear_params(struct maudio_chip
						    *chip)
{
	return ioread32(chip->spdif.io + 0x18);
}

static inline void spdif_write_non_linear_params(struct maudio_chip *chip,
						 uint32_t data)
{
	TRACE("0x%08X", data);
	iowrite32(data, chip->spdif.io + 0x18);
}

static inline void spdif_write_channel_status1(struct maudio_chip *chip,
					       uint32_t data)
{
	TRACE("0x%08X", data);
	iowrite32(data, chip->spdif.io + 0x20);
}

static inline void spdif_write_channel_status2(struct maudio_chip *chip,
					       uint32_t data)
{
	iowrite32(data, chip->spdif.io + 0x28);
}

static inline void spdif_write_samples(struct maudio_chip *chip,
				       void *data, int count)
{
	iowrite32_rep(chip->spdif.io + 0x38, data, count);
}

enum {
	SPDIF_FIFO_SIZE = 32
};

enum {
	SPDIF_CTRL_FREQ_22_5792 = 0 << 7,
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
	SPDIF_CTRL_START = 1 << 31
};

enum {
	SPDIF_INT_EN_UNDERFLOW
};

#define SPDIF_CTRL_FREQ_BASE(base) ((base & 7) << 3)
#define SPDIF_CTRL_FREQ_DIV(div) ((div & 7))

void spdif_write_channel_status(struct maudio_chip *chip)
{
	uint32_t *cs = (uint32_t *) & chip->spdif_channel_status[0];
	int i;
	for (i = 0; i != sizeof(chip->spdif_channel_status)
	     / sizeof(chip->spdif_channel_status[0])
	     / sizeof(*cs); ++i, ++cs) {
		spdif_write_channel_status1(chip, *cs);	// litle endian
		spdif_write_channel_status2(chip, *cs);
	}
}

static int spdif_reset(struct maudio_chip *chip)
{
	int i;

	spdif_write_ctrl(chip, SPDIF_CTRL_BIT_RESET);

	for (i = 0;; ++i) {
		if (i > 4) {
			ERROR("reset timeout");
			return -EIO;
		}

		if ((spdif_read_ctrl(chip) & SPDIF_CTRL_BIT_RESET) == 0)
			break;
	}
	return 0;
}

enum {
	SPDIF_CLOCK_VDW = 56,
	SPDIF_CLOCK_RDW = 23,
	SPDIF_CLOCK_OD = 4,
	SPDIF_FREQ_BASE = 0
};

static int spdif_hardware_init(struct platform_device *pdev,
			       struct maudio_chip *chip)
{
	int err;

#if 0
	/* Setting clock. Required for RealView EB */
	TRACE("setting clock 0x%08X", 0
		| (SPDIF_CLOCK_OD << 16)
		| (SPDIF_CLOCK_RDW << 9)
		| SPDIF_CLOCK_VDW);
	iowrite32(0
		| (SPDIF_CLOCK_OD << 16) 
		| (SPDIF_CLOCK_RDW << 9) 
		| SPDIF_CLOCK_VDW
		| 0, chip->control.io + MAUDIO_CONTROL_CLOCK_OFFSET);
#endif

	if ((err = spdif_reset(chip)))
		return err;

	return 0;
}

static void spdif_hardware_term(struct platform_device *pdev,
				struct maudio_chip *chip)
{
}

enum {
	SPDIF_PERIOD_SIZE = I2S_PERIOD_SIZE
};

static int spdif_pcm_open(struct snd_pcm_substream *stream)
{

	static struct snd_pcm_hardware hw = {
		.info = ( SNDRV_PCM_INFO_MMAP
			| SNDRV_PCM_INFO_MMAP_VALID 
			| SNDRV_PCM_INFO_INTERLEAVED ),
		.formats = SNDRV_PCM_FMTBIT_S16,
		.rates =
		    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
		    SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
		.rate_min = 32000,
		.rate_max = 192000,
		.channels_min = 2,
		.channels_max = 2,
		.buffer_bytes_max = SPDIF_PERIOD_SIZE * 2,
		.period_bytes_min = SPDIF_PERIOD_SIZE,
		.period_bytes_max = SPDIF_PERIOD_SIZE,
		.periods_min = 2,
		.periods_max = 2
	};


	TRACE("pcm open");

	stream->runtime->hw = hw;

#if 0
	do {
		unsigned long flags;

		spin_lock_irqsave(&chip->lock, flags);	 

		if(chip->state != MAUDIO_CHIP_STATE_IDLE) {
			spin_unlock_irqrestore(&chip->lock, flags);
			return -EBUSY;
		}

		chip->stream = stream;
		chip->state = MAUDIO_CHIP_STATE_SPDIF;

		spin_unlock_irqrestore(&chip->lock, flags);
	} while(0);
#endif
	return 0;
};

static int spdif_pcm_close(struct snd_pcm_substream *stream)
{
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);
	unsigned long flags;

	dmac_wait_stop(chip);

	spin_lock_irqsave(&chip->lock, flags);

	chip->stream = 0;
	chip->state = MAUDIO_CHIP_STATE_IDLE;

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int spdif_pcm_prepare(struct snd_pcm_substream *stream)
{
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);
	unsigned long flags;

	TRACE("period_size %lu buffer_size %lu "
	      "frame_bits %d boundary 0x%lX",
	      stream->runtime->period_size, stream->runtime->buffer_size,
	      stream->runtime->frame_bits, stream->runtime->boundary);

	dmac_wait_stop(chip);

	spin_lock_irqsave(&chip->lock, flags);

	if ((chip->state != MAUDIO_CHIP_STATE_IDLE) && 
	    (chip->stream != stream)) {
		spin_unlock_irqrestore(&chip->lock, flags);
		return -EBUSY;
	}

	chip->stream = stream;
	chip->state = MAUDIO_CHIP_STATE_SPDIF;

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static inline int spdif_rate_to_divisor(size_t rate)
{
	switch (rate) {
	case 32000:
		return 5;
	case 48000:
		return 3;
	case 96000:
		return 1;
	case 192000:
		return 0;
	default:
		TRACE("Frequency %d is not supported", rate);
		return 3;
	}
}

static int spdif_is_data_compressed(struct maudio_chip *chip)
{
	return (chip->spdif_channel_status[0] & IEC958_AES0_NONAUDIO) != 0;
}

static inline void spdif_wait_full_fifo(struct maudio_chip *chip)
{
	int i = 0;
	spdif_write_ctrl(chip, SPDIF_CTRL_PROG_LEVEL_HALF);
	TRACE("0x%08X", spdif_read_stat(chip));

	for (i = 0; i < 10000000; ++i) {
		if ((spdif_read_stat(chip) & (1 << 14)) == 0) {
			TRACE("spdif fifo signal received (on %d cycle)", i);
			break;
		}
	}

	TRACE("0x84 0x%08X", ioread32(chip->dmac.io + 0x84));
	TRACE("0x88 0x%08X", ioread32(chip->dmac.io + 0x88));
	TRACE("0x8C 0x%08X", ioread32(chip->dmac.io + 0x8C));
	TRACE("0x90 0x%08X", ioread32(chip->dmac.io + 0x90));
	TRACE("0x94 0x%08X", ioread32(chip->dmac.io + 0x94));
	TRACE("0x98 0x%08X", ioread32(chip->dmac.io + 0x98));
	TRACE("0x9C 0x%08X", ioread32(chip->dmac.io + 0x9C));
	TRACE("0xA0 0x%08X", ioread32(chip->dmac.io + 0xA0));
	TRACE("0xA4 0x%08X", ioread32(chip->dmac.io + 0xA4));
	TRACE("0xA8 0x%08X", ioread32(chip->dmac.io + 0xA8));
	TRACE("0xAC 0x%08X", ioread32(chip->dmac.io + 0xAC));
	TRACE("0xB0 0x%08X", ioread32(chip->dmac.io + 0xB0));
	TRACE("0xB4 0x%08X", ioread32(chip->dmac.io + 0xB4));
	TRACE("0xB8 0x%08X", ioread32(chip->dmac.io + 0xB8));
	TRACE("0xBC 0x%08X", ioread32(chip->dmac.io + 0xBC));
	TRACE("0xC0 0x%08X", ioread32(chip->dmac.io + 0xC0));
	TRACE("0xC4 0x%08X", ioread32(chip->dmac.io + 0xC4));
	TRACE("0x%08X", spdif_read_stat(chip));
}

static int spdif_pcm_trigger(struct snd_pcm_substream *stream, int cmd)
{
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);

	TRACE("action %d", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		spdif_reset(chip);

		chip->hw_ptr = 0;

		spdif_write_stat(chip, 0xFFFFFFFF);
		spdif_write_int_en(chip, 0);
		spdif_write_channel_status(chip);
		spdif_write_non_linear_params(chip, 1);

		dmac_start(chip, 1);

		spdif_wait_full_fifo(chip);

		spdif_write_ctrl(chip, 0
			| (spdif_is_data_compressed(chip) ?  SPDIF_CTRL_COMPRESSED : 0)
			| SPDIF_CTRL_PROG_LEVEL_HALF
#ifdef __BIG_ENDIAN
			| SPDIF_CTRL_BE
#else
			| SPDIF_CTRL_LE
#endif
			| SPDIF_CTRL_DATA_PACK
			| SPDIF_CTRL_FREQ_BASE(SPDIF_FREQ_BASE)
			| SPDIF_CTRL_FREQ_DIV(spdif_rate_to_divisor(stream->runtime->rate))
			| SPDIF_CTRL_FREQ_24_576 
			| SPDIF_CTRL_START);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		dmac_request_stop(chip);
		return 0;
	}

	return -EINVAL;
}

static int spdif_ctl_info(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	TRACE("");
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int spdif_ctl_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct maudio_chip *chip = snd_kcontrol_chip(kcontrol);
	TRACE("");
	memcpy(ucontrol->value.iec958.status, chip->spdif_channel_status,
	       sizeof(ucontrol->value.iec958.status));
	return 0;
}

static int spdif_ctl_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct maudio_chip *chip = snd_kcontrol_chip(kcontrol);
	TRACE("AES0 0x%02X AES1 0x%02X AES2 0x%02X AES3 0x%02X",
	      ucontrol->value.iec958.status[0],
	      ucontrol->value.iec958.status[1],
	      ucontrol->value.iec958.status[2],
	      ucontrol->value.iec958.status[3]);

	if (memcmp(chip->spdif_channel_status, ucontrol->value.iec958.status,
	    sizeof(ucontrol->value.iec958.status))) {
		memcpy(chip->spdif_channel_status,
		       ucontrol->value.iec958.status,
		       sizeof(ucontrol->value.iec958.status));
		return 1;
	}
	return 0;
}
/* }}} */

/* PCM stuff {{{ */
static int maudio_pcm_hw_params(struct snd_pcm_substream *stream,
				struct snd_pcm_hw_params *params)
{
	TRACE("period_size %d buffer_size %d buffer_bytes %d",
	      params_period_size(params),
	      params_buffer_size(params),
	      params_buffer_bytes(params));
#if 0
	TRACE("period_size %lu buffer_size %lu frame_bits %d buffer_bytes %u",
	      stream->runtime->period_size, stream->runtime->buffer_size,
	      stream->runtime->frame_bits, params_buffer_bytes(params));
#endif

	snd_pcm_set_runtime_buffer(stream, &stream->dma_buffer);

	return 0;
}

static int maudio_pcm_hw_free(struct snd_pcm_substream *stream)
{
	snd_pcm_set_runtime_buffer(stream, NULL);
	return 0;
}

static snd_pcm_uframes_t maudio_pcm_pointer(struct snd_pcm_substream
					    *stream)
{
	struct maudio_chip *chip = snd_pcm_substream_chip(stream);
	return chip->hw_ptr;
}

static int acquire_mem_resource(struct device_node *const np, char *name,
				struct mem_resource *const ret)
{
	struct resource res;

	if (of_address_to_resource(np, 0, &res))
		return -ENOMEM;

	ret->res = res;

    ret->io = ioremap(res.start, resource_size(&res));
	if (!ret->io) {
		TRACE("ioremap_nocache(0x%08X, 0x%08X) failed",
		      res.start, resource_size(&res));
		return -ENOMEM;
	};

	TRACE("io (%s) %p", name, ret->io);

	return 0;
}
/* }}} */

/* Generic init/free {{{ */
static void release_mem_resource(struct mem_resource *ret)
{
	iounmap(ret->io);
	TRACE("releasing mem region(0x%08X,0x%08X)",
	      ret->res.start, resource_size(&ret->res));
}

static int maudio_acquire_resources(struct platform_device *pdev,
				    struct maudio_chip *chip)
{
	int err;

	TRACE("acquring resources");

	err = acquire_mem_resource(chip->dmac_of_node, "dmac", &chip->dmac);
	if (err) {
		TRACE("cannot acquire dmac resources");
		goto error_dmac;
	}

	err = acquire_mem_resource(chip->i2s_of_node, "i2s", &chip->i2s);
	if (err) {
		TRACE("cannot acquire i2s resources");
		goto error_i2s;
	}

	err = acquire_mem_resource(chip->spdif_of_node, "spdif", &chip->spdif);
	if (err) {
		TRACE("cannot acquire spdif resources");
		goto error_spdif;
	}

	err = acquire_mem_resource(pdev->dev.of_node, "control", &chip->control);
	if (err) {
		TRACE("cannot acquire control resources");
		goto error_control;
	}
	return 0;

error_control:
	release_mem_resource(&chip->spdif);
error_spdif:
	release_mem_resource(&chip->i2s);
error_i2s:
	release_mem_resource(&chip->dmac);
error_dmac:
	return err;
}

static void maudio_release_resources(struct platform_device *pdev,
				     struct maudio_chip *chip)
{
	release_mem_resource(&chip->control);
	release_mem_resource(&chip->spdif);
	release_mem_resource(&chip->i2s);
	release_mem_resource(&chip->dmac);
}

static int maudio_hardware_init(struct platform_device *pdev,
	struct maudio_chip *chip)
{
	struct device_node *const np = pdev->dev.of_node;
	int ret, irq;

	chip->dmac_of_node = of_parse_phandle(np,
			"maudio,dmac", 0);
	if (!chip->dmac_of_node) {
		dev_err(&pdev->dev,
			"Property 'module,dmac' missing or invalid\n");
		ret = -EINVAL;
		goto error_acquire_resource;
	};

	chip->i2s_of_node = of_parse_phandle(np,
			"maudio,i2s", 0);
	if (!chip->i2s_of_node) {
		dev_err(&pdev->dev,
			"Property 'module,i2s' missing or invalid\n");
		ret = -EINVAL;
		goto error_acquire_resource;
	};

	chip->spdif_of_node = of_parse_phandle(np,
			"maudio,spdif", 0);
	if (!chip->spdif_of_node) {
		dev_err(&pdev->dev,
			"Property 'module,spdif' missing or invalid\n");
		ret = -EINVAL;
		goto error_acquire_resource;
	};

	ret = maudio_acquire_resources(pdev, chip);
	if (ret) {
		goto error_acquire_resource;
	};

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENODEV;
		goto error_irq;
	}

	TRACE("irq# %d", irq);
	ret = devm_request_irq(&pdev->dev, irq, maudio_interrupt_handler, 0,
			       pdev->name, chip);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto error_irq;
	};
	chip->irq = irq;

	ret = dmac_hardware_init(pdev, chip);
	if (ret)
		goto error_dmac;

	ret = i2s_hardware_init(pdev, chip);
	if (ret)
		goto error_i2s;

	ret = spdif_hardware_init(pdev, chip);
	if (ret)
		goto error_spdif;
	return 0;

error_spdif:
	i2s_hardware_term(pdev, chip);
error_i2s:
	free_irq(chip->irq, chip);
error_irq:
	dmac_hardware_term(pdev, chip);
error_dmac:
	maudio_release_resources(pdev, chip);
error_acquire_resource:
	return ret;
}

static void maudio_hardware_term(struct platform_device *pdev, 
	struct maudio_chip *chip)
{
	TRACE("entering");
	spdif_hardware_term(pdev, chip);
	i2s_hardware_term(pdev, chip);
	dmac_hardware_term(pdev, chip);
	free_irq(chip->irq, chip);
	maudio_release_resources(pdev, chip);
}

void maudio_card_free(struct snd_card *card)
{
	maudio_chip_free(card_chip(card));
}

int maudio_dma_mmap(struct snd_pcm_substream *stream,
		    struct vm_area_struct *area)
{
	struct snd_pcm_runtime *runtime = stream->runtime;
	return dma_mmap_writecombine(stream->pcm->card->dev, area,
				     runtime->dma_area, runtime->dma_addr,
				     runtime->dma_bytes);
}

static void maudio_pcm_private_free(struct snd_pcm *pcm)
{
	TRACE("");
	pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->dma_buffer.
	    area = NULL;
}

static int maudio_pcm_new(struct snd_card *card, char *name, int id,
			  struct snd_pcm_ops *ops)
{
	struct maudio_chip *chip = card_chip(card);
	struct snd_pcm *pcm;
	struct snd_pcm_substream *stream;
	struct snd_dma_buffer *buf;
	int err;

	if ((err = snd_pcm_new(card, name, id, 1, 0, &pcm)))
		return err;

	pcm->private_data = chip;
	pcm->private_free = maudio_pcm_private_free;

	strcpy(pcm->name, name);

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, ops);

	stream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	buf = &stream->dma_buffer;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = &chip->dev->dev;
	buf->private_data = NULL;
	buf->addr = chip->dma_handle;
	buf->area = chip->dma_area;
	buf->bytes = chip->dma_size;

	TRACE("area %p addr 0x%08X size %u", buf->area,
	      buf->addr, buf->bytes);

	return 0;
}
/*}}}*/

/* Interrupt routines {{{ */
static void maudio_move_hw_ptr(struct snd_pcm_substream *stream,
			       snd_pcm_uframes_t delta)
{
	snd_pcm_uframes_t old_hw_ptr;
	struct maudio_chip *chip =
	    snd_pcm_substream_chip(stream->pcm->card);
	int frame_bytes;

	frame_bytes = stream->runtime->frame_bits/8;
	old_hw_ptr = chip->hw_ptr;

	chip->hw_ptr += delta;
	chip->hw_ptr %= stream->runtime->buffer_size;
	TRACE("hw_ptr 0x%08lX -> 0x%08lX delta %lu", 
		old_hw_ptr, chip->hw_ptr, delta);

	maudio_dmaaddr_check(chip, stream, chip->hw_ptr);
}


static irqreturn_t maudio_interrupt_handler(int irq, void *data)
{
	struct maudio_chip *chip = (struct maudio_chip *)data;
	unsigned long flags;
	uint32_t dmac_wrend;
	uint32_t dmac_int_status;

	dmac_int_status = dmac_read_int(chip);
	dmac_wrend = DMAC_INT_CHANNEL0_WREND << (chip->channels - 1);

	maudio_statuswatch_check(chip, dmac_int_status);

	if (dmac_int_status & dmac_wrend) {
		dmac_write_int(chip, dmac_wrend);

		spin_lock_irqsave(&chip->lock, flags);

		if (chip->stream) {
			struct snd_pcm_substream* chip_stream = chip->stream;
			maudio_move_hw_ptr(chip_stream,
					   chip_stream->runtime->period_size);

			spin_unlock_irqrestore(&chip->lock, flags);
			snd_pcm_period_elapsed(chip_stream);
			spin_lock_irqsave(&chip->lock, flags);
		}

		spin_unlock_irqrestore(&chip->lock, flags);
	}

	return IRQ_HANDLED;
}
/* }}} */

static struct snd_pcm_ops i2s_pcm_ops = {
	.open = i2s_pcm_open,
	.close = i2s_pcm_close,
	.trigger = i2s_pcm_trigger,
	.hw_params = maudio_pcm_hw_params,
	.hw_free = maudio_pcm_hw_free,
	.prepare = i2s_pcm_prepare,
	.ioctl = i2s_pcm_ioctl,
	.pointer = maudio_pcm_pointer,
	.mmap = maudio_dma_mmap
};

static struct snd_pcm_ops spdif_pcm_ops = {
	.open = spdif_pcm_open,
	.close = spdif_pcm_close,
	.trigger = spdif_pcm_trigger,
	.hw_params = maudio_pcm_hw_params,
	.hw_free = maudio_pcm_hw_free,
	.prepare = spdif_pcm_prepare,
	.ioctl = snd_pcm_lib_ioctl,
	.pointer = maudio_pcm_pointer,
	.mmap = maudio_dma_mmap
};

static struct snd_kcontrol_new spdif_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	.info = spdif_ctl_info,
	.get = spdif_ctl_get,
	.put = spdif_ctl_put
};

static int maudio_driver_probe(struct platform_device *pdev)
{
	struct snd_card *card = NULL;
	struct maudio_chip *chip = NULL;
	struct device_node *np = pdev->dev.of_node;
	int err;
	const char *name = "maudio";

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if ((err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
				THIS_MODULE, sizeof(struct maudio_chip), &card))) {
		ERROR("Unable create sound card, error %d", err);
		goto error;
	}

	chip = card_chip(card);

	maudio_chip_init(chip, pdev, card, 128 * 1024);

	card->private_free = maudio_card_free;

	if ((err = maudio_pcm_new(card, "i2s", 0, &i2s_pcm_ops)))
		goto error;

	if ((err = maudio_pcm_new(card, "iec958", 1, &spdif_pcm_ops)))
		goto error;

	if ((err = snd_ctl_add(card, snd_ctl_new1(&spdif_ctl, chip)))) {
		ERROR("Unable to add sound control, error %d", err);
		goto error;
	}

	platform_set_drvdata(pdev, card);
	snd_card_set_dev(card, &pdev->dev);

	spin_lock_init(&chip->lock);

	if ((err = maudio_hardware_init(pdev, chip))) {
		ERROR("Failed to initialize hardware, error %d", err);
		goto error;
	}

	strcpy(card->driver, DRIVER_NAME);
	
	strcpy(card->shortname, name);
 	(void) of_property_read_string_index(np, "maudio,model", 0, &name);
	strcpy(card->longname, name);

	if ((err = snd_card_register(card))) {
		ERROR("Unable to register sound card, error %d", err);
		goto error_hw;
	}

	maudio_debugfs_init(chip);
	printk("PROBE SUCCESS\n");
	TRACE("probe success");
	return 0;

error_hw:
	maudio_hardware_term(pdev, chip);
error:
	if (platform_get_drvdata(pdev))
		platform_set_drvdata(pdev, NULL);

	if (card)
		snd_card_free(card);

	return err;
}

static int maudio_driver_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct maudio_chip *chip = card_chip(card);

	TRACE("entering");
	maudio_debugfs_free(chip);
	maudio_hardware_term(pdev, chip);
	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id maudio_of_match[] = {
	{ .compatible = "rcm,audio", },
	{},
};

static struct platform_driver maudio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = maudio_of_match,
	},
	.probe = maudio_driver_probe,
	.remove = maudio_driver_remove,
};
module_platform_driver(maudio_driver);


MODULE_AUTHOR("Kirill Mikhailov");
MODULE_DESCRIPTION("RC Module audio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DEVICE_TABLE(of, maudio_of_match);
