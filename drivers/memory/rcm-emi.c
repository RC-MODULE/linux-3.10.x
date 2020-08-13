/* RCM EMI memory controller driver
 *
 * Copyright (C) 2020 MIR
 *	Mikhail.Petrov@mir.dev
 * 
 * SPDX-License-Identifier: GPL-2.0+
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <asm/dcr.h>
#include <dt-bindings/memory/rcm-emi.h>

#define BANK_COUNT 6

#define RCM_EMI_SS0 0x00
#define RCM_EMI_SD0 0x04
#define RCM_EMI_SS1 0x08
#define RCM_EMI_SD1 0x0C
#define RCM_EMI_SS2 0x10
#define RCM_EMI_SD2 0x14
#define RCM_EMI_SS3 0x18
#define RCM_EMI_SD3 0x1C
#define RCM_EMI_SS4 0x20
#define RCM_EMI_SD4 0x24
#define RCM_EMI_SS5 0x28
#define RCM_EMI_SD5 0x2C
#define RCM_EMI_RFC 0x30
#define RCM_EMI_HSTSR 0x34
#define RCM_EMI_ECNT20 0x38
#define RCM_EMI_ECNT53 0x3C
#define RCM_EMI_H1ADR 0x40
#define RCM_EMI_H2ADR 0x44
#define RCM_EMI_RREQADR 0x48
#define RCM_EMI_WREQADR 0x4C
#define RCM_EMI_WDADR 0x50
#define RCM_EMI_BUSEN 0x54
#define RCM_EMI_WECR 0x58
#define RCM_EMI_FLCNTRL 0x5C
#define RCM_EMI_IMR 0x60
#define RCM_EMI_IMR_SET 0x64
#define RCM_EMI_IMR_RST 0x68
#define RCM_EMI_IRR 0x70
#define RCM_EMI_IRR_RST 0x74
#define RCM_EMI_ECCWRR 0x78
#define RCM_EMI_ECCRDR 0x7C
#define RCM_EMI_H1CMR 0xC0
#define RCM_EMI_H2CMR 0xC4
#define RCM_EMI_RREQCMR 0xC8
#define RCM_EMI_WREQCMR 0xCC
#define RCM_EMI_WDCMR 0xD0

struct bank_config
{
	bool configured;
	u32 ss;
	u32 sd;
	bool ecc;
	u32 base;
	u32 size;
};

struct emi_config
{
	struct bank_config banks[BANK_COUNT];
};

struct emi_data
{
	struct emi_config config;
	bool dcr_host_mapped;
	dcr_host_t dcr_host;
	unsigned dcr_n;
	unsigned dcr_c;
	bool ecc_irq_err_single_allocated;
	int ecc_irq_err_single;
	bool ecc_irq_err_double_allocated;
	int ecc_irq_err_double;
};

static const u32 BANK_BASE_ADDRESSES[BANK_COUNT] = {
	0x00000000, 0x20000000, 0x40000000, 0x50000000, 0x60000000, 0x70000000
};

static const u32 BANK_DEFAULT_SIZES[BANK_COUNT] = {
	0x20000000, 0x20000000, 0x10000000, 0x10000000, 0x10000000, 0x10000000
};

static int read_config(struct platform_device *pdev)
{
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);
	char prop_name[128];
	int i;
	int ret;

	for (i = 0; i < BANK_COUNT; ++i) {
		sprintf(prop_name, "bank-%i-ss", i);
		ret = of_property_read_u32(pdev->dev.of_node, prop_name, &emi->config.banks[i].ss);
		if (ret == -EINVAL)
			continue;
		if (ret != 0)
			return ret;

		sprintf(prop_name, "bank-%i-sd", i);
		ret = of_property_read_u32(pdev->dev.of_node, prop_name, &emi->config.banks[i].sd);
		if (ret != 0)
			return ret;

		sprintf(prop_name, "bank-%i-ecc", i);
		emi->config.banks[i].ecc = of_property_read_bool(pdev->dev.of_node, prop_name);

		emi->config.banks[i].base = BANK_BASE_ADDRESSES[i];

		sprintf(prop_name, "bank-%i-size", i);
		ret = of_property_read_u32(pdev->dev.of_node, prop_name, &emi->config.banks[i].size);
		if (ret != 0)
			return ret;

		emi->config.banks[i].configured = true;
	}

	return 0;
}

static irqreturn_t ecc_err_single_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device*)dev_id;
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);
	u32 irr = dcr_read(emi->dcr_host, RCM_EMI_IRR);
	u32 h1adr = dcr_read(emi->dcr_host, RCM_EMI_H1ADR);

	dev_err(&pdev->dev, "single ECC error: EMI_IRR = 0x%08X, EMI_H1ADR = 0x%08X\n",
		irr, h1adr);

	// clear the interrupt and the error address
	dcr_write(emi->dcr_host, RCM_EMI_H1CMR, 0);
	dcr_write(emi->dcr_host, RCM_EMI_IRR_RST, (irr & 0x5A5));

	return IRQ_HANDLED;
}

static irqreturn_t ecc_err_double_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device*)dev_id;
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);
	u32 irr = dcr_read(emi->dcr_host, RCM_EMI_IRR);
	u32 h2adr = dcr_read(emi->dcr_host, RCM_EMI_H2ADR);

	dev_err(&pdev->dev, "double ECC error: EMI_IRR = 0x%08X, EMI_H2ADR = 0x%08X\n",
		irr, h2adr);

	dcr_write(emi->dcr_host, RCM_EMI_IMR, 0); // mask interrupts
	panic("Memory double ECC error\n");

	return IRQ_HANDLED;
}

static int fill_for_ecc(struct platform_device *pdev, struct bank_config *bank)
{
	u32 base;
	u32 size;
	u32 page_size;
	void *p;

	dev_info(&pdev->dev, "Filling memory region 0x%08x-0x%08x for ECC\n",
		bank->base, bank->base + bank->size);

	base = bank->base;
	size = bank->size;

	while (size != 0) {
		page_size = min_t(u32, size, PAGE_SIZE);

		p = devm_memremap(&pdev->dev, base, PAGE_SIZE, MEMREMAP_WC);
		if (IS_ERR(p))
			return PTR_ERR(p);
		memset(p, 0, page_size);
		devm_memunmap(&pdev->dev, p);

		base += page_size;
		size -= page_size;
	}

	return 0;
}

static int setup_hardware(struct platform_device *pdev)
{
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);
	struct bank_config *bank;
	u32 hstsr;
	int i;
	int ret;

	dcr_write(emi->dcr_host, RCM_EMI_IMR, 0); // mask interrupts
	hstsr = dcr_read(emi->dcr_host, RCM_EMI_HSTSR);

	dev_dbg(&pdev->dev, "bank cfg    ss         sd    ecc   base       size\n");

	for (i = 0; i < BANK_COUNT; ++i)
	{
		bank = &emi->config.banks[i];

		if (bank->configured) {
			dcr_write(emi->dcr_host, RCM_EMI_SS0 + (RCM_EMI_SS1 - RCM_EMI_SS0) * i, bank->ss);
			dcr_write(emi->dcr_host, RCM_EMI_SD0 + (RCM_EMI_SD1 - RCM_EMI_SD0) * i, bank->sd);
			if (bank->ecc)
				hstsr |= (1 << i);
			else
				hstsr &= ~(1 << i);
		} else {
			bank->ss = dcr_read(emi->dcr_host, RCM_EMI_SS0 + (RCM_EMI_SS1 - RCM_EMI_SS0) * i);
			bank->sd = dcr_read(emi->dcr_host, RCM_EMI_SD0 + (RCM_EMI_SD1 - RCM_EMI_SD0) * i);
			bank->ecc = ((hstsr & (1 << i)) != 0);
			bank->base = BANK_BASE_ADDRESSES[i];
			bank->size = BANK_DEFAULT_SIZES[i];
		}

		dev_dbg(&pdev->dev, " %i    %i 0x%08X 0x%08X %i 0x%08X 0x%08X\n",
			i, (int)bank->configured, bank->ss, bank->sd, (int)bank->ecc, bank->base, bank->size);
	}

	dcr_write(emi->dcr_host, RCM_EMI_HSTSR, hstsr);

	// clear state and unmask interrupts
	dcr_write(emi->dcr_host, RCM_EMI_H1CMR, 0);
	dcr_write(emi->dcr_host, RCM_EMI_H2CMR, 0);
	dcr_write(emi->dcr_host, RCM_EMI_IRR_RST, 0xFFFFFFFF);
	dcr_write(emi->dcr_host, RCM_EMI_IMR, 0xFFF);

	mb();

	// clear banks with ECC
	for (i = 0; i < BANK_COUNT; ++i)
	{
		bank = &emi->config.banks[i];
		if (bank->configured && bank->ecc) {
			switch (bank->ss & RCM_EMI_SSx_BTYP_MASK) {
			case RCM_EMI_SSx_BTYP_SRAM:
			case RCM_EMI_SSx_BTYP_SSRAM:
			case RCM_EMI_SSx_BTYP_SDRAM:
				ret = fill_for_ecc(pdev, bank);
				if (ret != 0)
					return ret;
				break;
			}
		}
	}

	return 0;
}

static int probe_internal(struct platform_device *pdev)
{
	struct emi_data *emi;
	int ret;

	emi = devm_kzalloc(&pdev->dev, sizeof(*emi), GFP_KERNEL);
	if (emi == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, emi);

	emi->dcr_n = dcr_resource_start(pdev->dev.of_node, 0);
	emi->dcr_c = dcr_resource_len(pdev->dev.of_node, 0);
	if ((emi->dcr_n == 0) || (emi->dcr_c == 0)) {
		dev_err(&pdev->dev, "failed to read dcr-reg value\n");
		return -EINVAL;
	}

	emi->dcr_host = dcr_map(&pdev->dev, emi->dcr_n, emi->dcr_c);
	if (!DCR_MAP_OK(emi->dcr_host)) {
		dev_err(&pdev->dev, "failed to map dcr area\n");
		return -EFAULT;
	}
	emi->dcr_host_mapped = true;

	ret = read_config(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to read DT configuration\n");
		return ret;
	}

	emi->ecc_irq_err_single = platform_get_irq(pdev, 0);
	if (emi->ecc_irq_err_single <= 0)
		return emi->ecc_irq_err_single;
	ret = request_irq(emi->ecc_irq_err_single, ecc_err_single_interrupt, 0, "emi", pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "could not allocate interrupt for single ECC errors\n");
		return ret;
	}
	emi->ecc_irq_err_single_allocated = true;

	emi->ecc_irq_err_double = platform_get_irq(pdev, 1);
	if (emi->ecc_irq_err_double <= 0)
		return emi->ecc_irq_err_double;
	ret = request_irq(emi->ecc_irq_err_double, ecc_err_double_interrupt, 0, "emi", pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "could not allocate interrupt for double ECC errors\n");
		return ret;
	}
	emi->ecc_irq_err_double_allocated = true;

	ret = setup_hardware(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to setup hardware\n");
		return ret;
	}

	return 0;
}

static void cleanup(struct platform_device *pdev)
{
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);

	if (emi == NULL)
		return;

	if (emi->ecc_irq_err_single_allocated)
		free_irq(emi->ecc_irq_err_single, pdev);

	if (emi->ecc_irq_err_double_allocated)
		free_irq(emi->ecc_irq_err_double, pdev);

	if (emi->dcr_host_mapped)
		dcr_unmap(emi->dcr_host, emi->dcr_c);
}

static int rcm_emi_probe(struct platform_device *pdev)
{
	int ret;

	dev_dbg(&pdev->dev, "probing...\n");

	ret = probe_internal(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "driver probe error %i\n", ret);
		cleanup(pdev);
		return ret;
	}

	dev_info(&pdev->dev, "probed successfully\n");

	// populate child devices
	if (pdev->dev.of_node)
		of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);

	return 0;
}

static int rcm_emi_remove(struct platform_device *pdev)
{
	cleanup(pdev);

	return 0;
}

static const struct of_device_id rcm_emi_of_match[] = {
	{
		.compatible = "rcm,emi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rcm_emi_of_match);

static struct platform_driver rcm_emi_platform_driver = {
	.driver =
		{
			.name = "rcm-emi",
			.of_match_table = rcm_emi_of_match,
		},
	.probe = rcm_emi_probe,
	.remove = rcm_emi_remove,
};
module_platform_driver(rcm_emi_platform_driver);

MODULE_DESCRIPTION("RCM EMI memory controller driver");
MODULE_AUTHOR("Mikhail Petrov<Mikhail.Petrov@mir.dev>");
MODULE_LICENSE("GPL v2");
