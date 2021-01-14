/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Mikhail Petrov <Mikhail.Petrov@mir.dev>
 */

#define pr_fmt(fmt) "rcm-rmace: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/syscon.h>
#include <linux/rcm-rmace.h>
#include <asm/io.h>

#define HSIF_REG_HSIF_ID_REG 0x000
#define HSIF_REG_RMACE_RADDR_EXTEND 0x018
#define HSIF_REG_RMACE_WADDR_EXTEND 0x01C

#define RMACE_REG_ID 0x000
#define RMACE_REG_VERSION 0x004
#define RMACE_REG_SW_RST 0x008

#define RMACE_REG_ADMA_SW_RST 0x808
#define RMACE_REG_ADMA_CH_STATUS 0x80C

#define RMACE_REG_RDMA_SETTINGS 0x900
#define RMACE_REG_RDMA_STATUS 0x904
#define RMACE_REG_RDMA_SYS_ADDR 0x908
#define RMACE_REG_RDMA_TBL_SIZE 0x90C
#define RMACE_REG_RDMA_DESC_ADDR 0x918

#define RMACE_REG_WDMA_SETTINGS 0xA00
#define RMACE_REG_WDMA_STATUS 0xA04
#define RMACE_REG_WDMA_SYS_ADDR 0xA08
#define RMACE_REG_WDMA_TBL_SIZE 0xA0C
#define RMACE_REG_WDMA_DESC_ADDR 0xA18

#define DMA_CH_STATUS_RDMA_IRQ_MASK BIT(0)
#define DMA_CH_STATUS_WDMA_IRQ_MASK BIT(16)

#define DMA_SETTINGS_BAD_DESC_MASK BIT(2)
#define DMA_SETTINGS_AXI_ERROR_MASK BIT(5)
#define DMA_SETTINGS_EN_DMA_MASK BIT(28)
#define DMA_SETTINGS_EN_DMA_DESC_TBL_MASK BIT(29)
#define DMA_SETTINGS_DMA_LONG_LEN_MASK BIT(30)

#define DMA_STATUS_BAD_DESC_MASK BIT(2)
#define DMA_STATUS_AXI_ERROR_MASK BIT(5)

struct rcm_rmace_dev *rcm_rmace_dev_single_ptr;

static inline u32 rmace_read_reg(struct rcm_rmace_dev *rmace, unsigned reg)
{
	return ioread32(rmace->base + reg);
}

static inline void rmace_write_reg(struct rcm_rmace_dev *rmace, unsigned reg, u32 val)
{
	iowrite32(val, rmace->base + reg);
}

static void reset_rmace(struct rcm_rmace_dev *rmace)
{
	rmace_write_reg(rmace, RMACE_REG_SW_RST, 0x1);
	while ((rmace_read_reg(rmace, RMACE_REG_SW_RST) & 0x1) != 0) {
		// wait loop
	}
}

static void reset_dma(struct rcm_rmace_dev *rmace)
{
	rmace_write_reg(rmace, RMACE_REG_ADMA_SW_RST, 0x1);
	while ((rmace_read_reg(rmace, RMACE_REG_ADMA_SW_RST) & 0x1) != 0) {
		// wait loop
	}
}

static void schedule_next_ctx(struct rcm_rmace_dev *rmace)
{
	struct rcm_rmace_ctx *ctx;
	dma_addr_t phys_addr;
	unsigned long irq_flags;
	unsigned i;

	spin_lock_irqsave(&rmace->scheduled_lock, irq_flags);
	if (rmace->cur_ctx != NULL) {
		spin_unlock_irqrestore(&rmace->scheduled_lock, irq_flags);
		return;
	}

	rmace->cur_ctx = list_first_entry_or_null(&rmace->scheduled_ctx_list,
		struct rcm_rmace_ctx, scheduled_list);
	if (rmace->cur_ctx != NULL)
		list_del(&rmace->cur_ctx->scheduled_list);
	spin_unlock_irqrestore(&rmace->scheduled_lock, irq_flags);

	if (rmace->cur_ctx == NULL)
		return;

	ctx = rmace->cur_ctx;
	ctx->status = RCM_RMACE_CTX_STATUS_EXECUTING;

	for (i = 0; i < ctx->src_desc_count; ++i)
		rmace->dma_data->src_hw_descs[i] = ctx->src_descs[i];
	rmace->dma_data->src_hw_descs[ctx->src_desc_count].data = 0; // stop descriptor

	for (i = 0; i < ctx->dst_desc_count; ++i)
		rmace->dma_data->dst_hw_descs[i] = ctx->dst_descs[i];
	rmace->dma_data->dst_hw_descs[ctx->dst_desc_count].data = 0; // stop descriptor

	spin_lock_irqsave(&rmace->reg_lock, irq_flags);

	// program the source DMA channel
	phys_addr = rmace->dma_data_phys_addr + offsetof(struct rcm_rmace_dma_data, src_hw_descs);
	rmace_write_reg(rmace, RMACE_REG_RDMA_SYS_ADDR, phys_addr);
	rmace_write_reg(rmace, RMACE_REG_RDMA_TBL_SIZE, sizeof(struct rcm_rmace_hw_desc) * RCM_RMACE_HW_DESC_COUNT);
	rmace_write_reg(rmace, RMACE_REG_RDMA_DESC_ADDR, phys_addr);
	rmace_read_reg(rmace, RMACE_REG_RDMA_STATUS); // clear status register

	// program the destination DMA channel
	phys_addr = rmace->dma_data_phys_addr + offsetof(struct rcm_rmace_dma_data, dst_hw_descs);
	rmace_write_reg(rmace, RMACE_REG_WDMA_SYS_ADDR, phys_addr);
	rmace_write_reg(rmace, RMACE_REG_WDMA_TBL_SIZE, sizeof(struct rcm_rmace_hw_desc) * RCM_RMACE_HW_DESC_COUNT);
	rmace_write_reg(rmace, RMACE_REG_WDMA_DESC_ADDR, phys_addr);
	rmace_read_reg(rmace, RMACE_REG_WDMA_STATUS); // clear status register

	// start the channels
	rmace_write_reg(rmace, RMACE_REG_RDMA_SETTINGS,
		DMA_SETTINGS_AXI_ERROR_MASK
		| DMA_SETTINGS_EN_DMA_MASK
		| DMA_SETTINGS_EN_DMA_DESC_TBL_MASK
		| DMA_SETTINGS_DMA_LONG_LEN_MASK);
	rmace_write_reg(rmace, RMACE_REG_WDMA_SETTINGS,
		DMA_SETTINGS_BAD_DESC_MASK
		| DMA_SETTINGS_AXI_ERROR_MASK
		| DMA_SETTINGS_EN_DMA_MASK
		| DMA_SETTINGS_EN_DMA_DESC_TBL_MASK
		| DMA_SETTINGS_DMA_LONG_LEN_MASK);

	spin_unlock_irqrestore(&rmace->reg_lock, irq_flags);
}

static irqreturn_t rmace_irq(int irq, void *arg)
{
	struct rcm_rmace_dev *rmace = arg;
	u32 irq_ch_status = rmace_read_reg(rmace, RMACE_REG_ADMA_CH_STATUS);

	if ((irq_ch_status & (DMA_CH_STATUS_RDMA_IRQ_MASK | DMA_CH_STATUS_WDMA_IRQ_MASK)) != 0)
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static irqreturn_t rmace_irq_thread(int irq, void *arg)
{
	struct rcm_rmace_dev *rmace = arg;
	struct rcm_rmace_ctx *ctx;
	u32 rdma_status, wdma_status;
	unsigned long irq_flags;

	spin_lock_irqsave(&rmace->scheduled_lock, irq_flags);
	ctx = rmace->cur_ctx;
	rmace->cur_ctx = NULL;
	spin_unlock_irqrestore(&rmace->scheduled_lock, irq_flags);
	BUG_ON(ctx == NULL);

	spin_lock_irqsave(&rmace->reg_lock, irq_flags);

	rdma_status = rmace_read_reg(rmace, RMACE_REG_RDMA_STATUS);
	wdma_status = rmace_read_reg(rmace, RMACE_REG_WDMA_STATUS);

	if (((rdma_status & DMA_STATUS_AXI_ERROR_MASK) != 0)
		|| ((wdma_status & DMA_STATUS_AXI_ERROR_MASK) != 0))
	{
		dev_err(&rmace->pdev->dev,
			"AXI error: rdma_status = 0x%08X, wdma_status = 0x%08X\n",
			rdma_status,
			wdma_status);
		reset_rmace(rmace);
		reset_dma(rmace);
		ctx->status = RCM_RMACE_CTX_STATUS_FINISHED;
	}
	else if ((wdma_status & DMA_STATUS_BAD_DESC_MASK) != 0)
		ctx->status = RCM_RMACE_CTX_STATUS_FINISHED | RCM_RMACE_CTX_STATUS_SUCCESS;
	else
		panic("unknow interrupt");

	spin_unlock_irqrestore(&rmace->reg_lock, irq_flags);

	if (ctx->callback != NULL)
		ctx->callback(ctx, ctx->callback_arg);

	schedule_next_ctx(rmace);

	return IRQ_HANDLED;
}

void rcm_rmace_ctx_init(struct rcm_rmace_dev *rmace, struct rcm_rmace_ctx *ctx)
{
	memset(ctx, 0, sizeof *ctx);
	ctx->rmace = rmace;
	INIT_LIST_HEAD(&ctx->scheduled_list);
}
EXPORT_SYMBOL(rcm_rmace_ctx_init);

void rcm_rmace_ctx_schedule(struct rcm_rmace_ctx *ctx)
{
	unsigned long irq_flags;

	BUG_ON((ctx->status & (RCM_RMACE_CTX_STATUS_SCHEDULED | RCM_RMACE_CTX_STATUS_EXECUTING)) != 0);
	BUG_ON(ctx->src_desc_count > RCM_RMACE_HW_DESC_COUNT - 1);
	BUG_ON(ctx->src_desc_count > RCM_RMACE_HW_DESC_COUNT - 1);
	ctx->status = RCM_RMACE_CTX_STATUS_SCHEDULED;

	spin_lock_irqsave(&ctx->rmace->scheduled_lock, irq_flags);
	list_add_tail(&ctx->scheduled_list, &ctx->rmace->scheduled_ctx_list);
	spin_unlock_irqrestore(&ctx->rmace->scheduled_lock, irq_flags);

	schedule_next_ctx(ctx->rmace);
}
EXPORT_SYMBOL(rcm_rmace_ctx_schedule);

static int rcm_rmace_probe(struct platform_device *pdev)
{
	struct rcm_rmace_dev *rmace;
	struct device_node *syscon_np;
	u32 control_id;
	u32 device_id;
	int irq;
	int ret;

#ifdef CONFIG_1888TX018
	pdev->dev.archdata.dma_offset = - (pdev->dev.dma_pfn_offset << PAGE_SHIFT);
#endif

	if (rcm_rmace_dev_single_ptr != NULL) {
		dev_err(&pdev->dev, "only one device is supported\n");
		return -ENODEV;
	}

	rmace = kzalloc(sizeof *rmace, GFP_KERNEL);
	if (rmace == NULL) {
		dev_err(&pdev->dev, "insufficient memory\n");
		return -ENOMEM;
	}

	rmace->pdev = pdev;
	spin_lock_init(&rmace->reg_lock);
	spin_lock_init(&rmace->scheduled_lock);
	INIT_LIST_HEAD(&rmace->scheduled_ctx_list);
	platform_set_drvdata(pdev, rmace);

	syscon_np = of_parse_phandle(pdev->dev.of_node, "control", 0);
	if (!syscon_np) {
		dev_err(&pdev->dev, "no control property\n");
		return -ENODEV;
	}
	
	rmace->control = syscon_node_to_regmap(syscon_np);
	of_node_put(syscon_np);
	if (IS_ERR(rmace->control)) {
		dev_err(&pdev->dev, "cannot get control regmap\n");
		return -ENODEV;
	}

	ret = regmap_read(rmace->control, HSIF_REG_HSIF_ID_REG, &control_id);
	if (ret) {
		dev_err(&pdev->dev, "control ID register read error\n");
		return -ENODEV;
	}
	if (control_id != 0x46495348) {
		dev_err(&pdev->dev, "unknow control device ID: %08X\n", control_id);
		return -ENODEV;
	}

	// [***]
	ret = regmap_write(rmace->control, HSIF_REG_RMACE_RADDR_EXTEND, 0x0);
	if (ret) {
		dev_err(&pdev->dev, "HSIF RDMA setup error\n");
		return -ENODEV;
	}
	ret = regmap_write(rmace->control, HSIF_REG_RMACE_WADDR_EXTEND, 0x0);
	if (ret) {
		dev_err(&pdev->dev, "HSIF WDMA setup error\n");
		return -ENODEV;
	}

	rmace->base = devm_ioremap_resource(&pdev->dev, pdev->resource);
	if (IS_ERR(rmace->base)) {
		dev_err(&pdev->dev, "register allocation failed\n");
		return -ENODEV;
	}

	reset_rmace(rmace);
	reset_dma(rmace);

	device_id = rmace_read_reg(rmace, RMACE_REG_ID);
	if (device_id != 0x4543414D) {
		dev_err(&pdev->dev, "unknow device ID: %08X\n", device_id);
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "get IRQ resource error\n");
		return -ENODEV;
	}
	ret = devm_request_threaded_irq(&pdev->dev, irq,
		rmace_irq, rmace_irq_thread, IRQF_ONESHOT, NULL, rmace);
	if (ret) {
		dev_err(&pdev->dev, "grab IRQ error\n");
		return -ENODEV;
	}

	rmace->device_version = rmace_read_reg(rmace, RMACE_REG_VERSION);

	rmace->dma_data = dmam_alloc_coherent(
		&pdev->dev,
		sizeof *rmace->dma_data,
		&rmace->dma_data_phys_addr,
		GFP_KERNEL);
	if (rmace->dma_data == NULL) {
		dev_err(&pdev->dev, "cannot allocate DMA buffers");
		return -ENODEV;
	}

#ifdef CONFIG_CRYPTO_RCM_RMACE
	ret = rcm_rmace_crypto_register(rmace);
	if (ret != 0)
		return ret;
#endif

#ifdef CONFIG_RCM_RMACE_DMA
	ret = rcm_rmace_dma_register(rmace);
	if (ret != 0)
		goto dma_register_error;
#endif

	rcm_rmace_dev_single_ptr = rmace;
	dev_info(&pdev->dev, "initialized (hardware version=%u.%u)\n",
		(rmace->device_version >> 8) & 0xFF,
		(rmace->device_version >> 0) & 0xFF);

	return 0;

#ifdef CONFIG_RCM_RMACE_DMA
dma_register_error:
#ifdef CONFIG_CRYPTO_RCM_RMACE
	rcm_rmace_crypto_unregister(rmace);
#endif
	return ret;
#endif
}

static int rcm_rmace_remove(struct platform_device *pdev)
{
	struct rcm_rmace_dev *rmace = platform_get_drvdata(pdev);

#ifdef CONFIG_CRYPTO_RCM_RMACE
	rcm_rmace_crypto_unregister(rmace);
#endif

#ifdef CONFIG_RCM_RMACE_DMA
	rcm_rmace_dma_unregister(rmace);
#endif

	rcm_rmace_dev_single_ptr = NULL;
	kfree(rmace);

	return 0;
}

static struct of_device_id rcm_rmace_id_table[] = {
	{ .compatible = "rcm,rmace" },
	{ },
};

MODULE_DEVICE_TABLE(of, rcm_rmace_id_table);

static struct platform_driver rcm_rmace_driver = {
	.driver = {
		.name = "rcm-rmace",
		.owner = THIS_MODULE,
		.of_match_table = rcm_rmace_id_table,
	},
	.probe = rcm_rmace_probe,
	.remove = rcm_rmace_remove,
};

module_platform_driver(rcm_rmace_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikhail Petrov <Mikhail.Petrov@mir.dev>");
MODULE_DESCRIPTION("RCM RMACE platform driver");
