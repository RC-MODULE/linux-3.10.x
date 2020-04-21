/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexander Shtreys <alexander.shtreys@mir.dev>
 */
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include "pcie-cadence-rcm.h"

/**
 * struct cdns_plat_pcie - private data for this PCIe platform driver
 * @pcie: Cadence PCIe controller
 */
struct rcm_cdns_plat_pcie {
	struct cdns_pcie        *pcie;
};

static const struct of_device_id rcm_cdns_plat_pcie_of_match[];

static int rcm_cdns_plat_pcie_probe(struct platform_device *pdev)
{
	struct rcm_cdns_plat_pcie *cdns_plat_pcie;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct rcm_cdns_pcie_ep *ep;
	int phy_count;
	int ret;

	match = of_match_device(rcm_cdns_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	pr_debug(" Started %s\n", __func__);
	cdns_plat_pcie = devm_kzalloc(dev, sizeof(*cdns_plat_pcie), GFP_KERNEL);
	if (!cdns_plat_pcie)
		return -ENOMEM;

	platform_set_drvdata(pdev, cdns_plat_pcie);

	if (!IS_ENABLED(CONFIG_RCM_PCIE_CADENCE_PLAT_EP))
		return -ENODEV;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pcie.dev = dev;
	cdns_plat_pcie->pcie = &ep->pcie;

	ret = cdns_pcie_init_phy(dev, cdns_plat_pcie->pcie);
	if (ret) {
		dev_err(dev, "failed to init phy\n");
		return ret;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync() failed\n");
		goto err_get_sync;
	}

	ret = rcm_cdns_pcie_ep_setup(ep);
	if (ret)
		goto err_init;

 err_init:
	pm_runtime_put_sync(dev);

 err_get_sync:
	pm_runtime_disable(dev);
	cdns_pcie_disable_phy(cdns_plat_pcie->pcie);
	phy_count = cdns_plat_pcie->pcie->phy_count;
	while (phy_count--)
		device_link_del(cdns_plat_pcie->pcie->link[phy_count]);

	return 0;
}

static void rcm_cdns_plat_pcie_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cdns_pcie *pcie = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_put_sync(dev);
	if (ret < 0)
		dev_dbg(dev, "pm_runtime_put_sync failed\n");

	pm_runtime_disable(dev);

	cdns_pcie_disable_phy(pcie);
}

static const struct of_device_id rcm_cdns_plat_pcie_of_match[] = {
	{
		.compatible = "rcm,cdns-pcie-ep",
	},
	{},
};

static struct platform_driver rcm_cdns_plat_pcie_driver = {
	.driver = {
		.name = "rcm-cdns-pcie",
		.of_match_table = rcm_cdns_plat_pcie_of_match,
		.pm	= &cdns_pcie_pm_ops,
	},
	.probe = rcm_cdns_plat_pcie_probe,
	.shutdown = rcm_cdns_plat_pcie_shutdown,
};
builtin_platform_driver(rcm_cdns_plat_pcie_driver);
