// SPDX-License-Identifier: GPL-2.0
/*
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
 * @is_rc: Set to 1 indicates the PCIe controller mode is Root Complex,
 *         if 0 it is in Endpoint mode.
 */
struct rcm_cdns_plat_pcie {
	struct cdns_pcie        *pcie;
	bool is_rc;
};

struct rcm_cdns_plat_pcie_of_data {
	bool is_rc;
};

static const struct of_device_id rcm_cdns_plat_pcie_of_match[];

static int rcm_cdns_plat_pcie_probe(struct platform_device *pdev)
{
	const struct rcm_cdns_plat_pcie_of_data *data;
	struct rcm_cdns_plat_pcie *cdns_plat_pcie;
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct rcm_cdns_pcie_ep *ep;
	struct cdns_pcie_rc *rc;
	int phy_count;
	bool is_rc;
	int ret;

	match = of_match_device(rcm_cdns_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = (struct rcm_cdns_plat_pcie_of_data *)match->data;
	is_rc = data->is_rc;

	pr_debug(" Started %s with is_rc: %d\n", __func__, is_rc);
	cdns_plat_pcie = devm_kzalloc(dev, sizeof(*cdns_plat_pcie), GFP_KERNEL);
	if (!cdns_plat_pcie)
		return -ENOMEM;

	platform_set_drvdata(pdev, cdns_plat_pcie);

	if (is_rc) {
		if (!IS_ENABLED(CONFIG_RCM_PCIE_CADENCE_PLAT_HOST))
			return -ENODEV;

		bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
		if (!bridge)
			return -ENOMEM;

		rc = pci_host_bridge_priv(bridge);
		rc->pcie.dev = dev;
		cdns_plat_pcie->pcie = &rc->pcie;
		cdns_plat_pcie->is_rc = is_rc;

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

		ret = rcm_cdns_pcie_host_setup(rc);
		if (ret)
			goto err_init;
	} else {
		if (!IS_ENABLED(CONFIG_RCM_PCIE_CADENCE_PLAT_EP))
			return -ENODEV;

		ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
		if (!ep)
			return -ENOMEM;

		ep->pcie.dev = dev;
		cdns_plat_pcie->pcie = &ep->pcie;
		cdns_plat_pcie->is_rc = is_rc;

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
	}

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

static const struct rcm_cdns_plat_pcie_of_data rcm_cdns_plat_pcie_host_of_data = {
	.is_rc = true,
};

static const struct rcm_cdns_plat_pcie_of_data rcm_cdns_plat_pcie_ep_of_data = {
	.is_rc = false,
};

static const struct of_device_id rcm_cdns_plat_pcie_of_match[] = {
	{
		.compatible = "rcm,cdns-pcie-host",
		.data = &rcm_cdns_plat_pcie_host_of_data,
	},
	{
		.compatible = "rcm,cdns-pcie-ep",
		.data = &rcm_cdns_plat_pcie_ep_of_data,
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
