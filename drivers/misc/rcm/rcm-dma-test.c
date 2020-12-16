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
#include <linux/vmalloc.h>
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

static int rcm_dma_test_sg_alloc_from_pages(
	struct sg_table* sgt, struct page **pages, unsigned int n_pages,
	unsigned int offset, unsigned long size, unsigned int max_segment,
	gfp_t gfp_mask)
{
	unsigned int chunks, cur_page, seg_len, i;
	struct scatterlist *s;

	chunks = 1;
	seg_len = 0;
	for (i = 1; i < n_pages; i++) {
		seg_len += PAGE_SIZE;
		if ((seg_len >= max_segment) ||
		    (page_to_pfn(pages[i]) != page_to_pfn(pages[i - 1]) + 1)) {
			chunks++;
			seg_len = 0;
		}
	}

	sgt->sgl = kmalloc(chunks * sizeof(struct scatterlist), gfp_mask);

	if (sgt->sgl == NULL)
		return -ENOMEM;

	sg_init_table(sgt->sgl, chunks);
	sgt->nents = sgt->orig_nents = chunks;

	/* merging chunks and putting them into the scatterlist */
	cur_page = 0;
	for_each_sg(sgt->sgl, s, sgt->orig_nents, i) {
		unsigned int j, chunk_size;

		/* look for the end of the current chunk */
		seg_len = 0;
		for (j = cur_page + 1; j < n_pages; j++) {
			seg_len += PAGE_SIZE;
			if (seg_len >= max_segment ||
			    page_to_pfn(pages[j]) !=
			    page_to_pfn(pages[j - 1]) + 1)
				break;
		}

		chunk_size = ((j - cur_page) << PAGE_SHIFT) - offset;
		sg_set_page(s, pages[cur_page],
			    min_t(unsigned long, size, chunk_size), offset);
		size -= chunk_size;
		offset = 0;
		cur_page = j;
	}

	return 0;
}

static void rcm_dma_test_dma_callback(void *param)
{
	struct rcm_dma_test *data = param;

	complete(&data->dma_completion);
}

int rcm_dma_test_dma_memcpy_slave(struct rcm_dma_test *data,
                                  bool from_sg, phys_addr_t phys_addr,
                                  void *buf, size_t size)
{
	struct dma_chan *chan = (from_sg) ? data->rx : data->tx;
	struct dma_slave_config config; 
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	struct page **pages;
	struct sg_table sgt;
//	struct scatterlist* sg;
	const void *p; 
	int nr_pages; 
	int ret;
	int i;

	pr_debug("%s: dir: %s, phys_addr = 0x%llX, size = %u\n",
	         __func__, (from_sg) ? "FROM_SG" : "TO_SG",
	         (u64)phys_addr, (u32)size);

	if (!chan) {
		pr_debug("%s: chan is NULL\n", __func__);
		return -EFAULT;
	} else if (!chan->device) {
		pr_debug("%s: chan->device is NULL\n", __func__);
		return -EFAULT;
	} else if (!chan->device->device_prep_slave_sg) {
		pr_debug("%s: chan->device->device_prep_slave_sg is NULL\n",
		         __func__);
		return -EFAULT;
	}

	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr = phys_addr;
	config.src_addr = phys_addr;
	config.src_maxburst = 1;
	config.dst_maxburst = 1;
	config.device_fc = false;

	config.direction = DMA_MEM_TO_MEM;

	ret = dmaengine_slave_config(chan, &config);

	if (ret) {
		pr_err("%s: Failed to configure DMA channel.\n", __func__);
		return ret;
	}

	nr_pages = DIV_ROUND_UP((unsigned long)buf + size, PAGE_SIZE) -
	           (unsigned long)buf / PAGE_SIZE;
	pages = kmalloc_array(nr_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %d pages.\n",
		       __func__, nr_pages);
		return -ENOMEM;
	}

	pr_debug("%s: buf = 0x%px, %d pages, offset = %u\n",
	         __func__, buf, nr_pages, (u32)(offset_in_page(buf)));

	p = buf - offset_in_page(buf);
	for (i = 0; i < nr_pages; i++) {
		if (is_vmalloc_addr(p))
			pages[i] = vmalloc_to_page(p);
		else
			pages[i] = virt_to_page((void *)p);
		if (!pages[i]) {
			kfree(pages);
			pr_err("%s: Failed to get page for address 0x%px.\n",
			       __func__, p);
			ret = -EFAULT;
			goto err_free_pages;
		}

		p += PAGE_SIZE;
	}

	ret = rcm_dma_test_sg_alloc_from_pages(&sgt, pages, nr_pages,
	                                       offset_in_page(buf), size,
	                                       PAGE_SIZE, GFP_KERNEL);

	if (ret) {
		pr_err("%s: Failed to allocate SG-table (%d pages).\n",
		       __func__, nr_pages);
		ret = -EFAULT;
		goto err_sg_free_table;
	}

	ret = dma_map_sg(chan->device->dev, sgt.sgl, sgt.nents,
	                 (from_sg) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (ret == 0) {
		pr_err("%s: Failed to map SG-table.\n", __func__);
		ret = -EFAULT;
		goto err_sg_free_table;
	}
/*
	for_each_sg(sgt.sgl, sg, sgt.nents, i)
	{
		dma_addr_t addr = sg_dma_address(sg);
		u32        len  = sg_dma_len(sg);

		pr_debug("%s: addr = 0x%08X, len = %u\n",
		         __func__, (u32)addr, len);
	}
*/
	desc = dmaengine_prep_slave_sg(chan, sgt.sgl, sgt.nents, 
	                               config.direction,
	                               DMA_PREP_INTERRUPT);

	if (!desc) {
		pr_err("%s: Failed prepare dma operation.\n", __func__);
		ret = -EFAULT;
		goto err_dma_unmap;
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

err_dma_unmap:
	dma_unmap_sg(chan->device->dev, sgt.sgl, sgt.nents,
	             (from_sg) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
err_sg_free_table:
	kfree(sgt.sgl);
err_free_pages:
	kfree(pages);

	return ret;
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
		return -EFAULT;
	} else if (!chan->device) {
		pr_debug("%s: chan->device is NULL\n", __func__);
		return -EFAULT;
	} else if (!chan->device->device_prep_dma_memcpy) {
		pr_debug("%s: chan->device->device_prep_dma_memcpy is NULL\n",
		         __func__);
		return -EFAULT;
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

static int rcm_dma_test_memcpy(struct rcm_dma_test *data, unsigned size)
{
	void *src = NULL;
	void *dst = NULL;
	dma_addr_t src_dma_addr;
	dma_addr_t dst_dma_addr;
	int ret = 0;
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

err:
	if (src) {
		dma_free_coherent(data->dev, size, src, src_dma_addr);
	}

	if (dst) {
		dma_free_coherent(data->dev, size, dst, dst_dma_addr);
	}

	return ret;
}

static int rcm_dma_test_memcpy_sg(struct rcm_dma_test *data, unsigned size,
                                  bool from_sg)
{
	void *src = NULL;
	void *dst = NULL;
	dma_addr_t dma_addr;
	int ret = 0;
	int i;
#ifdef DEBUG
	char mess[256];
#endif

	src = vmalloc(size);
	if (!src) {
		pr_err("%s: Failed to allocate memory (%u bytes).\n", __func__,
		       size);
		return -ENOMEM;
	}

	dst = dma_alloc_coherent(data->dev, size, &dma_addr, GFP_KERNEL);
	if (!dst) {
		pr_err("%s: Failed to allocate %u bytes of DMA-memory.\n",
		       __func__, size);
		ret = -ENOMEM;
		goto err;
	}

	if (!from_sg)
		swap(src, dst);

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

	ret = rcm_dma_test_dma_memcpy_slave(data, from_sg, dma_addr, 
	                                    (from_sg) ? src : dst, size);

	if (ret)
		goto err_swap;

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

err_swap:
	if (!from_sg)
		swap(src, dst);
err:
	if (src) {
		vfree(src);
	}

	if (dst) {
		dma_free_coherent(data->dev, size, dst, dma_addr);
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
		ret = rcm_dma_test_memcpy(data, arg);
		break;
	case 1:
		ret = rcm_dma_test_memcpy_sg(data, arg, true);
		break;
	case 2:
		ret = rcm_dma_test_memcpy_sg(data, arg, false);
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
