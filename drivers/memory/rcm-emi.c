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
#include <asm/dcr.h>

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

		emi->config.banks[i].configured = true;
	}

	return 0;
}

static int setup_hardware(struct platform_device *pdev)
{
	struct emi_data *emi = dev_get_drvdata(&pdev->dev);
	int i;
	u32 hstsr;

	dev_dbg(&pdev->dev, "bank conf     ss         sd\n");

	hstsr = dcr_read(emi->dcr_host, RCM_EMI_HSTSR);

	for (i = 0; i < BANK_COUNT; ++i)
	{
		struct bank_config *bank = &emi->config.banks[i];

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
		}

		dev_dbg(&pdev->dev, " %i    %i   0x%08x 0x%08x\n",
			i, (int)bank->configured, bank->ss, bank->sd);
	}

	dcr_write(emi->dcr_host, RCM_EMI_HSTSR, hstsr);

	mb();

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
