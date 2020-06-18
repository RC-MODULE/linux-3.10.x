// SPDX-License-Identifier: GPL-2.0
/**
 * Test driver to test RCM DMA functionality
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#define DEBUG

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>

#include <linux/dmaengine.h>

static DEFINE_IDA(rcm_dma_test_ida);

struct rcm_dma_test {
	struct device         *dev;
	struct miscdevice      miscdev;

	struct dma_chan       *tx;
	struct dma_chan       *rx;

	struct completion      dma_completion;

	char                   devname[20];
	int                    id;

	struct mutex           lock;
};

#define to_dma_test(priv) container_of((priv), struct rcm_dma_test, miscdev)

static void rcm_dma_test_dma_callback(void *param)
{
	struct rcm_dma_test *data = param;

	complete(&data->dma_completion);
}

int rcm_dma_test_dma_memcpy(struct rcm_dma_test *data,
                            dma_addr_t src_addr, dma_addr_t dest_addr,
                            size_t size)
{
	struct dma_chan *chan = data->tx;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret;

	pr_debug("%s: src_addr = 0x%llX, dest_addr = 0x%llX, size = %u\n",
	         __func__, (u64)src_addr, (u64)dest_addr, (u32)size);

	if (!chan) {
		pr_debug("%s: chan is NULL\n", __func__);
	} else if (!chan->device) {
		pr_debug("%s: chan->device is NULL\n", __func__);
	} else if (!chan->device->device_prep_dma_memcpy) {
		pr_debug("%s: chan->device->device_prep_dma_memcpy is NULL\n",
		         __func__);
	}

	desc = dmaengine_prep_dma_memcpy(chan, dest_addr, src_addr, size,
	                                 DMA_PREP_INTERRUPT);

	if (!desc) {
		pr_err("%s: Failed prepare dma operation.\n", __func__);
		return -EFAULT;
	}

	reinit_completion(&data->dma_completion);

	desc->callback = rcm_dma_test_dma_callback;
	desc->callback_param = data;

	cookie = dmaengine_submit(desc);
	dma_async_issue_pending(chan); 

	ret = 0;

	if (!wait_for_completion_timeout(&data->dma_completion,
	                                 msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;

	if (dma_async_is_tx_complete(chan, cookie, NULL, NULL) != DMA_COMPLETE)
		ret = -ETIMEDOUT;

	if (ret) {
		pr_err("%s: DMA Error %i\n", __func__, ret);
		dmaengine_terminate_all(chan);
	}

	return ret;
}

static int rcm_dma_test_0(struct rcm_dma_test *data, unsigned size)
{
	void *src = NULL;
	void *dst = NULL;
	dma_addr_t src_dma_addr;
	dma_addr_t dst_dma_addr;
	int ret;
	int i;
#ifdef DEBUG
	char mess[256];
#endif

	src = dma_alloc_coherent(data->dev, size, &src_dma_addr, GFP_KERNEL);
	if (!src) {
		pr_err("%s: Failed to allocate %u bytes of DMA-memory.\n",
		       __func__, size);
		ret = -ENOMEM;
		goto err;
	}

	dst = dma_alloc_coherent(data->dev, size, &dst_dma_addr, GFP_KERNEL);
	if (!dst) {
		pr_err("%s: Failed to allocate %u bytes of DMA-memory.\n",
		       __func__, size);
		ret = -ENOMEM;
		goto err;
	}

	get_random_bytes(src, size);
	memset(dst, 0xCC, size);

#ifdef DEBUG
	for (i = 0; i < size; ++i) {
		if (i * 3 + 10 > sizeof(mess)) {
			sprintf(&mess[i * 3], "...");
			break;
		}

		sprintf(&mess[i * 3], "%02X ", (unsigned)((char*)src)[i]);
	}

	pr_debug("%s: src: %s\n", __func__, mess);
#endif

	ret = rcm_dma_test_dma_memcpy(data, src_dma_addr, dst_dma_addr, size);

	if (ret)
		goto err;

	if (memcmp(src, dst, size) != 0) {
		pr_info("%s: Test failed (arrays does not match).\n", __func__);
#ifdef DEBUG
		for (i = 0; i < size; ++i) {
			if (i * 3 + 10 > sizeof(mess)) {
				sprintf(&mess[i * 3], "...");
				break;
			}

			sprintf(&mess[i * 3], "%02X ", 
			        (unsigned)((char*)dst)[i]);
		}

		pr_debug("%s: dst: %s\n", __func__, mess);
#endif
	} else {
		pr_info("%s: Test completed successfully.\n", __func__);
	}

	return 0;

err:
	if (src) {
		dma_free_coherent(data->dev, size, src, src_dma_addr);
	}

	if (dst) {
		dma_free_coherent(data->dev, size, dst, dst_dma_addr);
	}

	return ret;
}

static long rcm_dma_test_ioctl(struct file *file, unsigned int cmd,
                               unsigned long arg)
{
	int ret = -EINVAL;
	struct rcm_dma_test *data = to_dma_test(file->private_data);

	mutex_lock(&data->lock);
	switch (cmd) {
	case 0:
		ret = rcm_dma_test_0(data, arg);
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static const struct file_operations rcm_dma_test_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rcm_dma_test_ioctl,
};

static int rcm_dma_test_probe(struct platform_device *pdev)
{
	struct rcm_dma_test *data;
	struct device *dev = &pdev->dev;
	int err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;

	data->tx = dma_request_chan(dev, "tx");

	if (IS_ERR(data->tx)) {
		dev_warn(dev, "DMA TX channel is not available.\n");
		data->tx = NULL;
	}

	data->rx = dma_request_chan(dev, "rx");

	if (IS_ERR(data->rx)) {
		dev_warn(dev, "DMA RX channel is not available.\n");
		data->rx = NULL;
	}

	init_completion(&data->dma_completion);
	mutex_init(&data->lock);

	dev_set_drvdata(dev, data);

	data->id = ida_simple_get(&rcm_dma_test_ida, 0, 0, GFP_KERNEL);
	if (data->id < 0) {
		dev_err(dev, "Unable to get id\n");
		err = data->id;
		goto err_release;
	}

	snprintf(data->devname, sizeof(data->devname), "rcm-dma-test-%d",
	         data->id);
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = data->devname;
	data->miscdev.fops = &rcm_dma_test_fops,

	err = misc_register(&data->miscdev);
	if (err) {
		dev_err(dev, "Failed to register device\n");
		goto err_ida_remove;
	}

	return 0;

err_ida_remove:
	ida_simple_remove(&rcm_dma_test_ida, data->id);

err_release:
	if (data->tx) {
		dma_release_channel(data->tx);
	}

	if (data->rx) {
		dma_release_channel(data->rx);
	}

	return err;
}

static int rcm_dma_test_remove(struct platform_device *pdev)
{
	struct rcm_dma_test *data = dev_get_drvdata(&pdev->dev);

	misc_deregister(&data->miscdev);

	ida_simple_remove(&rcm_dma_test_ida, data->id);

	if (data->tx) {
		dma_release_channel(data->tx);
		data->tx = NULL;
	}

	if (data->rx) {
		dma_release_channel(data->rx);
		data->rx = NULL;
	}

	return 0;
}

static const struct of_device_id rcm_dma_test_ids[] = {
	{
		.compatible = "rcm,dma-test",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_dma_test_ids);

static struct platform_driver rcm_dma_test_driver = {
	.driver =
		{
			.name = "rcm-dma-test",
			.of_match_table = rcm_dma_test_ids,
		},
	.probe = rcm_dma_test_probe,
	.remove = rcm_dma_test_remove,
};

module_platform_driver(rcm_dma_test_driver);

MODULE_DESCRIPTION("RCM DMA TEST DRIVER");
MODULE_AUTHOR("Alexander Shtreys <alexander.shtreys@mir.dev>");
MODULE_LICENSE("GPL v2");
