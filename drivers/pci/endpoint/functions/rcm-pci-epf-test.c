// SPDX-License-Identifier: GPL-2.0
/**
 * Test driver to test RCM-endpoint functionality
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/of.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>

#include <linux/dmaengine.h>

#define IRQ_TYPE_LEGACY			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

#define COMMAND_RAISE_LEGACY_IRQ	BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ			BIT(3)
#define COMMAND_WRITE			BIT(4)
#define COMMAND_COPY			BIT(5)

#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)

#define TIMER_RESOLUTION		1

static struct workqueue_struct *kpcitest_workqueue;

struct pci_epf_test {
	void                          *reg[PCI_STD_NUM_BARS];
	struct pci_epf                *epf;
	enum pci_barno                 test_reg_bar;
	struct delayed_work            cmd_handler;
	const struct pci_epc_features *epc_features;

	struct dma_chan               *tx;
	struct dma_chan               *rx;

	struct completion              dma_completion;

	int                            use_dma;
	int                            memcpy_workaround;
};

struct pci_epf_test_reg {
	u32	magic;
	u32	command;
	u32	status;
	u64	src_addr;
	u64	dst_addr;
	u32	size;
	u32	checksum;
	u32	irq_type;
	u32	irq_number;
} __packed;

static struct pci_epf_header test_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576 };

static void pci_epf_test_dma_callback(void *param)
{
	struct pci_epf_test *epf_test = param;

	complete(&epf_test->dma_completion);
}

int pci_epf_test_dma_memcpy(struct pci_epf_test *epf_test,
                            enum dma_data_direction dir, phys_addr_t phys_addr,
                            void *buf, size_t size)
{
	struct dma_chan *chan = (dir == DMA_TO_DEVICE) ? epf_test->tx : 
	                                                 epf_test->rx;
	struct dma_slave_config config; 
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	struct page **pages;
	struct sg_table sgt;
	struct scatterlist* sg;
	const void *p; 
	int nr_pages; 
	int ret;
	int i;

	if ((dir != DMA_TO_DEVICE) && (dir != DMA_FROM_DEVICE)) {
		pr_err("%s: wrong direction of dma-operation (dir = %d).\n",
		       __func__, dir);
		return -EFAULT;
	}

	pr_debug("%s: dir: %s, phys_addr = 0x%llX, size = %u\n",
	         __func__, (dir == DMA_TO_DEVICE) ? "TO_DEVICE" : "FROM_DEVICE",
	         (u64)phys_addr, (u32)size);

	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr = phys_addr;
	config.src_addr = phys_addr;
	config.src_maxburst = 1;
	config.dst_maxburst = 1;
	config.device_fc = false;

	config.direction = (dir == DMA_TO_DEVICE) ? DMA_MEM_TO_DEV :
	                                            DMA_DEV_TO_MEM;

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
/*
	ret = __sg_alloc_table_from_pages(&sgt, pages, nr_pages,
	                                  offset_in_page(buf), size,
	                                  PAGE_SIZE, GFP_KERNEL);
*/
	ret = sg_alloc_table_from_pages(&sgt, pages, nr_pages,
	                                offset_in_page(buf), size, GFP_KERNEL);


	if (ret) {
		pr_err("%s: Failed to allocate SG-table (%d pages).\n",
		       __func__, nr_pages);
		ret = -EFAULT;
		goto err_sg_free_table;
	}

	ret = dma_map_sg(chan->device->dev, sgt.sgl, sgt.nents, dir);
	if (ret == 0) {
		pr_err("%s: Failed to map SG-table.\n", __func__);
		ret = -EFAULT;
		goto err_sg_free_table;
	}

	for_each_sg(sgt.sgl, sg, sgt.nents, i)
	{
		dma_addr_t addr = sg_dma_address(sg);
		u32        len  = sg_dma_len(sg);

		pr_debug("%s: addr = 0x%08X, len = %u\n",
		         __func__, (u32)addr, len);
	}

	desc = dmaengine_prep_slave_sg(chan, sgt.sgl, sgt.nents, 
	                               config.direction,
	                               DMA_PREP_INTERRUPT);

	if (!desc) {
		pr_err("%s: Failed prepare dma operation.\n", __func__);
		ret = -EFAULT;
		goto err_dma_unmap;
	}

	reinit_completion(&epf_test->dma_completion);

	desc->callback = pci_epf_test_dma_callback;
	desc->callback_param = epf_test;

	cookie = dmaengine_submit(desc);
	dma_async_issue_pending(chan); 

	ret = 0;

	if (!wait_for_completion_timeout(&epf_test->dma_completion,
	                                 msecs_to_jiffies(1000)))
		ret = -ETIMEDOUT;

	if (dma_async_is_tx_complete(chan, cookie, NULL, NULL) != DMA_COMPLETE)
		ret = -ETIMEDOUT;

	if (ret) {
		pr_err("%s: DMA Error %i\n", __func__, ret);
		dmaengine_terminate_all(chan);
	}

err_dma_unmap:
	dma_unmap_sg(chan->device->dev, sgt.sgl, sgt.nents, dir);
err_sg_free_table:
	sg_free_table(&sgt);
err_free_pages:
	kfree(pages);

	return ret;
}

static int pci_epf_test_copy(struct pci_epf_test *epf_test)
{
	int ret;
	void __iomem *src_addr = NULL;
	void __iomem *dst_addr = NULL;
	phys_addr_t src_phys_addr;
	phys_addr_t dst_phys_addr;
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
	int i;
	int cnt;

	if ((!epf_test->rx) || (!epf_test->tx) || (!epf_test->use_dma)) {
		src_addr = pci_epc_mem_alloc_addr(epc, &src_phys_addr,
		                                  reg->size);
		if (!src_addr) {
			dev_err(dev, "Failed to allocate source address\n");
			reg->status = STATUS_SRC_ADDR_INVALID;
			ret = -ENOMEM;
			goto err;
		}

		ret = pci_epc_map_addr(epc, epf->func_no, src_phys_addr,
		                       reg->src_addr, reg->size);
		if (ret) {
			dev_err(dev, "Failed to map source address\n");
			reg->status = STATUS_SRC_ADDR_INVALID;
			goto err_src_addr;
		}

		dst_addr = pci_epc_mem_alloc_addr(epc, &dst_phys_addr,
		                                  reg->size);
		if (!dst_addr) {
			dev_err(dev, "Failed to allocate destination address\n");
			reg->status = STATUS_DST_ADDR_INVALID;
			ret = -ENOMEM;
			goto err_src_map_addr;
		}

		ret = pci_epc_map_addr(epc, epf->func_no, dst_phys_addr,
		                       reg->dst_addr, reg->size);
		if (ret) {
			dev_err(dev, "Failed to map destination address\n");
			reg->status = STATUS_DST_ADDR_INVALID;
			goto err_dst_addr;
		}
	}

	if ((epf_test->use_dma) && (epf_test->rx) && (epf_test->tx)) {
		void *buf = kzalloc(reg->size, GFP_KERNEL);
		if (!buf) {
			ret = -ENOMEM;
			goto err_dst_map_addr;
		}

		pci_epf_test_dma_memcpy(epf_test, DMA_FROM_DEVICE,
		                        reg->src_addr, buf, reg->size);
		pci_epf_test_dma_memcpy(epf_test, DMA_TO_DEVICE,
		                        reg->dst_addr, buf, reg->size);
	} else if (epf_test->memcpy_workaround) {
		i = 0;

		while (i < reg->size) {
			cnt = (reg->size - i > 16) ? 16 : (reg->size - i);
			memcpy(dst_addr + i, src_addr + i, cnt);
			i += cnt;
		}
	} else {
		memcpy(dst_addr, src_addr, reg->size);
	}

err_dst_map_addr:
	if (dst_addr)
		pci_epc_unmap_addr(epc, epf->func_no, dst_phys_addr);

err_dst_addr:
	if (dst_addr)
		pci_epc_mem_free_addr(epc, dst_phys_addr, dst_addr, reg->size);

err_src_map_addr:
	if (src_addr)
		pci_epc_unmap_addr(epc, epf->func_no, src_phys_addr);

err_src_addr:
	if (src_addr)
		pci_epc_mem_free_addr(epc, src_phys_addr, src_addr, reg->size);

err:
	return ret;
}

static int pci_epf_test_read(struct pci_epf_test *epf_test)
{
	int ret;
	void __iomem *src_addr = NULL;
	void *buf;
	u32 crc32;
	phys_addr_t phys_addr;
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
#ifdef DEBUG
	char mess[256];
#endif
	int i;
	int cnt;

	if ((!epf_test->rx) || (!epf_test->use_dma)) {
		src_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->size);
		if (!src_addr) {
			dev_err(dev, "Failed to allocate address\n");
			reg->status = STATUS_SRC_ADDR_INVALID;
			ret = -ENOMEM;
			goto err;
		}

		ret = pci_epc_map_addr(epc, epf->func_no, phys_addr,
		                       reg->src_addr, reg->size);
		if (ret) {
			dev_err(dev, "Failed to map address\n");
			reg->status = STATUS_SRC_ADDR_INVALID;
			goto err_addr;
		}
	}

	buf = kzalloc(reg->size, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_map_addr;
	}

	memset(buf, 0xCC, reg->size);

	if ((epf_test->use_dma) && (epf_test->rx)) {
		pci_epf_test_dma_memcpy(epf_test, DMA_FROM_DEVICE,
		                        reg->src_addr, buf, reg->size);
	} else if (epf_test->memcpy_workaround) {
		i = 0;

		while (i < reg->size) {
			cnt = (reg->size - i > 16) ? 16 : (reg->size - i);
			memcpy_fromio(buf + i, src_addr + i, cnt);
			i += cnt;
		}
	} else {
		memcpy_fromio(buf, src_addr, reg->size);
	}

	crc32 = crc32_le(~0, buf, reg->size);
	if (crc32 != reg->checksum)
		ret = -EIO;

#ifdef DEBUG
	for (i = 0; i < reg->size; ++i)
	{
		if (i * 3 + 10 > sizeof(mess))
		{
			sprintf(&mess[i * 3], "...");
			break;
		}

		sprintf(&mess[i * 3], "%02X ", (unsigned)((char*)buf)[i]);
	}

	pr_debug("%s: data: %s\n", __func__, mess);

	pr_debug("%s: crc_reg = 0x%08X, crc_calc = 0x%08X\n", __func__,
	         reg->checksum, crc32);
#endif

	kfree(buf);

err_map_addr:
	if (src_addr)
		pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	if (src_addr)
		pci_epc_mem_free_addr(epc, phys_addr, src_addr, reg->size);

err:
	return ret;
}

static int pci_epf_test_write(struct pci_epf_test *epf_test)
{
	int ret;
	void __iomem *dst_addr = NULL;
	void *buf;
	phys_addr_t phys_addr;
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
#ifdef DEBUG
	char mess[256];
#endif
	int i;
	int cnt;

	pr_debug("%s: size = %u, dst_addr = 0x%llX\n", __func__,
	         reg->size, reg->dst_addr);

	if ((!epf_test->tx) || (!epf_test->use_dma)) {
		dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->size);
		if (!dst_addr) {
			dev_err(dev, "Failed to allocate address\n");
			reg->status = STATUS_DST_ADDR_INVALID;
			ret = -ENOMEM;
			goto err;
		}

		pr_debug("%s: alloc_addr -> phys_addr = 0x%X, dst_addr = 0x%X\n",
		         __func__, (u32)phys_addr, (u32)dst_addr);

		ret = pci_epc_map_addr(epc, epf->func_no, phys_addr, reg->dst_addr,
				       reg->size);
		if (ret) {
			dev_err(dev, "Failed to map address\n");
			reg->status = STATUS_DST_ADDR_INVALID;
			goto err_addr;
		}
	}

	buf = kzalloc(reg->size, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_map_addr;
	}

	get_random_bytes(buf, reg->size);
	reg->checksum = crc32_le(~0, buf, reg->size);

#ifdef DEBUG
	for (i = 0; i < reg->size; ++i) {
		if (i * 3 + 10 > sizeof(mess)) {
			sprintf(&mess[i * 3], "...");
			break;
		}

		sprintf(&mess[i * 3], "%02X ", (unsigned)((char*)buf)[i]);
	}

	pr_debug("%s: data: %s\n", __func__, mess);

	pr_debug("%s: checksum = 0x%08X\n", __func__, reg->checksum);
#endif

	if ((epf_test->use_dma) && (epf_test->tx)) {
		pci_epf_test_dma_memcpy(epf_test, DMA_TO_DEVICE, reg->dst_addr,
		                        buf, reg->size);
	} else if (epf_test->memcpy_workaround) {
		i = 0;

		while (i < reg->size) {
			cnt = (reg->size - i > 16) ? 16 : (reg->size - i);
			memcpy_toio(dst_addr + i, buf + i, cnt);
			i += cnt;
		}
	} else {
		memcpy_toio(dst_addr, buf, reg->size);
	}


	/*
	 * wait 1ms inorder for the write to complete. Without this delay L3
	 * error in observed in the host system.
	 */
	usleep_range(1000, 2000);

	kfree(buf);

err_map_addr:
	if (dst_addr)
		pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	if (dst_addr)
		pci_epc_mem_free_addr(epc, phys_addr, dst_addr, reg->size);

err:
	return ret;
}

static void pci_epf_test_raise_irq(struct pci_epf_test *epf_test, u8 irq_type,
				   u16 irq)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];

	pr_debug("%s >>> (type = %u, irq = %u)\n", __func__,
	         (unsigned)irq_type, (unsigned)irq);

	reg->status |= STATUS_IRQ_RAISED;

	switch (irq_type) {
	case IRQ_TYPE_LEGACY:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		break;
	case IRQ_TYPE_MSI:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI, irq);
		break;
	case IRQ_TYPE_MSIX:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX, irq);
		break;
	default:
		dev_err(dev, "Failed to raise IRQ, unknown type\n");
		break;
	}
}

static void pci_epf_test_cmd_handler(struct work_struct *work)
{
	int ret;
	int count;
	u32 command;
	struct pci_epf_test *epf_test = container_of(work, struct pci_epf_test,
						     cmd_handler.work);
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];

	command = reg->command;
	if (!command)
		goto reset_handler;

	pr_debug("%s: cmd = 0x%08X\n", __func__, reg->command);

	reg->command = 0;
	reg->status = 0;

	if (reg->irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_LEGACY_IRQ) {
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		goto reset_handler;
	}

	if (command & COMMAND_WRITE) {
		ret = pci_epf_test_write(epf_test);
		if (ret)
			reg->status |= STATUS_WRITE_FAIL;
		else
			reg->status |= STATUS_WRITE_SUCCESS;
		pci_epf_test_raise_irq(epf_test, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_READ) {
		ret = pci_epf_test_read(epf_test);
		if (!ret)
			reg->status |= STATUS_READ_SUCCESS;
		else
			reg->status |= STATUS_READ_FAIL;
		pci_epf_test_raise_irq(epf_test, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_COPY) {
		ret = pci_epf_test_copy(epf_test);
		if (!ret)
			reg->status |= STATUS_COPY_SUCCESS;
		else
			reg->status |= STATUS_COPY_FAIL;
		pci_epf_test_raise_irq(epf_test, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSI_IRQ) {
		count = pci_epc_get_msi(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI,
				  reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSIX_IRQ) {
		count = pci_epc_get_msix(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX,
				  reg->irq_number);
		goto reset_handler;
	}

reset_handler:
	queue_delayed_work(kpcitest_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));
}

static void pci_epf_test_linkup(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);

	queue_delayed_work(kpcitest_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));
}

static void pci_epf_test_unbind(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	cancel_delayed_work(&epf_test->cmd_handler);
	pci_epc_stop(epc);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (epf_test->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf_bar);
			pci_epf_free_space(epf, epf_test->reg[bar], bar);
		}
	}
}

static int pci_epf_test_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	const struct pci_epc_features *epc_features;

	epc_features = epf_test->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf_bar);
		if (ret) {
			pci_epf_free_space(epf, epf_test->reg[bar], bar);
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == test_reg_bar)
				return ret;
		}
	}

	return 0;
}

static int pci_epf_test_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	void *base;
	int bar, add;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	const struct pci_epc_features *epc_features;
	size_t test_reg_size;

	epc_features = epf_test->epc_features;

	if (epc_features->bar_fixed_size[test_reg_bar])
		test_reg_size = bar_size[test_reg_bar];
	else
		test_reg_size = sizeof(struct pci_epf_test_reg);

	base = pci_epf_alloc_space(epf, test_reg_size,
				   test_reg_bar, epc_features->align);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}
	epf_test->reg[test_reg_bar] = base;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (bar == test_reg_bar)
			continue;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		base = pci_epf_alloc_space(epf, bar_size[bar], bar,
					   epc_features->align);
		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n",
				bar);
		epf_test->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_configure_bar(struct pci_epf *epf,
				  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			bar_size[i] = epc_features->bar_fixed_size[i];
	}
}

static const struct of_device_id pci_epf_test_of_match_table[] = {
	{ .compatible = "rcm,pci_epf_test", },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, pci_epf_test_of_match_table);

static int pci_epf_test_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	enum pci_barno test_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool linkup_notifier = false;
	bool msix_capable = false;
	bool msi_capable = true;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		linkup_notifier = epc_features->linkup_notifier;
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
		test_reg_bar = pci_epc_get_first_free_bar(epc_features);
		pci_epf_configure_bar(epf, epc_features);
	}

	epf_test->test_reg_bar = test_reg_bar;
	epf_test->epc_features = epc_features;

	ret = pci_epc_write_header(epc, epf->func_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}

	ret = pci_epf_test_alloc_space(epf);
	if (ret)
		return ret;

	ret = pci_epf_test_set_bar(epf);
	if (ret)
		return ret;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->msix_interrupts);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	if (!linkup_notifier)
		queue_work(kpcitest_workqueue, &epf_test->cmd_handler.work);

	return 0;
}

static const struct pci_epf_device_id pci_epf_test_ids[] = {
	{
		.name = "pci_epf_test",
	},
	{},
};

static ssize_t pci_epf_test_show_use_dma(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct pci_epf_test *epf_test = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", epf_test->use_dma);
}

static ssize_t pci_epf_test_set_use_dma(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	struct pci_epf_test *epf_test = dev_get_drvdata(dev);
	int ret;
	unsigned int i;

	ret = kstrtouint(buf, 10, &i);
	if (ret)
		return ret;

	epf_test->use_dma = i;

	return count;
}
static DEVICE_ATTR(use_dma, 0644, pci_epf_test_show_use_dma, 
                   pci_epf_test_set_use_dma);

static ssize_t
pci_epf_test_show_memcpy_workaround(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct pci_epf_test *epf_test = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", epf_test->memcpy_workaround);
}

static ssize_t
pci_epf_test_set_memcpy_workaround(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	struct pci_epf_test *epf_test = dev_get_drvdata(dev);
	int ret;
	unsigned int i;

	ret = kstrtouint(buf, 10, &i);
	if (ret)
		return ret;

	epf_test->memcpy_workaround = i;

	return count;
}
static DEVICE_ATTR(memcpy_workaround, 0644, pci_epf_test_show_memcpy_workaround,
                   pci_epf_test_set_memcpy_workaround);

static int pci_epf_test_probe(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test;
	struct device *dev = &epf->dev;

	struct device_node *node = 
		of_find_matching_node(NULL, pci_epf_test_of_match_table);

	dev->of_node = node;

	epf_test = devm_kzalloc(dev, sizeof(*epf_test), GFP_KERNEL);
	if (!epf_test)
		return -ENOMEM;

	epf_test->tx = dma_request_chan(dev, "tx");

	if (IS_ERR(epf_test->tx)) {
		dev_warn(dev, "DMA TX channel is not available.\n");
		epf_test->tx = NULL;
	}

	epf_test->rx = dma_request_chan(dev, "rx");

	if (IS_ERR(epf_test->rx)) {
		dev_warn(dev, "DMA RX channel is not available.\n");
		epf_test->tx = NULL;
	}

	epf_test->use_dma = ((epf_test->tx) && (epf_test->rx)) ? 1 : 0;

	if (device_create_file(dev, &dev_attr_use_dma)) {
		dev_warn(dev, "Failed to create \"use_dma\" file.\n");
	}

	epf_test->memcpy_workaround = 1;

	if (device_create_file(dev, &dev_attr_memcpy_workaround)) {
		dev_warn(dev, "Failed to create \"memcpy_workaround\" file.\n");
	}

	init_completion(&epf_test->dma_completion);

	epf->header = &test_header;
	epf_test->epf = epf;

	INIT_DELAYED_WORK(&epf_test->cmd_handler, pci_epf_test_cmd_handler);

	epf_set_drvdata(epf, epf_test);

	return 0;
}

static int pci_epf_test_remove(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);

	device_remove_file(&epf->dev, &dev_attr_use_dma);

	device_remove_file(&epf->dev, &dev_attr_memcpy_workaround);

	if (epf_test->tx) {
		dma_release_channel(epf_test->tx);
		epf_test->tx = NULL;
	}

	if (epf_test->rx) {
		dma_release_channel(epf_test->rx);
		epf_test->rx = NULL;
	}

	return 0;
}

static struct pci_epf_ops ops = {
	.unbind	= pci_epf_test_unbind,
	.bind	= pci_epf_test_bind,
	.linkup = pci_epf_test_linkup,
};

static struct pci_epf_driver test_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pci_epf_test",
		.of_match_table = of_match_ptr(pci_epf_test_of_match_table),
	},
	.probe		= pci_epf_test_probe,
	.remove		= pci_epf_test_remove,
	.id_table	= pci_epf_test_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_test_init(void)
{
	int ret;

	kpcitest_workqueue = alloc_workqueue("kpcitest",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpcitest_workqueue) {
		pr_err("Failed to allocate the kpcitest work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&test_driver);
	if (ret) {
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_test_init);

static void __exit pci_epf_test_exit(void)
{
	pci_epf_unregister_driver(&test_driver);
}
module_exit(pci_epf_test_exit);

MODULE_DESCRIPTION("PCI EPF TEST DRIVER");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
