#include <linux/mfd/syscon.h>

#include "pcie-cadence-rcm.h"

int rcm_cdns_pcie_ep_send_legacy_irq(struct rcm_cdns_pcie_ep *ep, u8 fn, 
                                     u8 intx)
{
	u16 cmd;
	u32 reg;

	cmd = cdns_pcie_ep_fn_readw(&ep->pcie, fn, PCI_COMMAND);
	if (cmd & PCI_COMMAND_INTX_DISABLE)
	{
		dev_info(ep->pcie.dev, "%s: INTx Message Disabled\n", __func__);
		return -EINVAL;
	}

	regmap_write_bits(ep->csc, 0x18, 0x100, 0x100);

	regmap_write_bits(ep->csc, 0x20, 0x01, 0x01);

	do
	{
		regmap_read(ep->csc, 0x20, &reg);
	} while((reg & 0x04) == 0);

	regmap_write(ep->csc, 0x20, reg | 0x02);

	do
	{
		regmap_read(ep->csc, 0x20, &reg);
	} while((reg & 0x04) == 0);

	regmap_write_bits(ep->csc, 0x18, 0x100, 0x000);

	return 0;
}

int rcm_cdns_pcie_ep_setup_priv(struct rcm_cdns_pcie_ep *ep)
{
	struct device *dev = ep->pcie.dev;
	struct device_node *np = dev->of_node;
	struct device_node* node;

	node = of_parse_phandle(np, "csc", 0);
	if (!node) {
		dev_err(dev, "failed to find csc node\n");
		return -EFAULT;
	}

	ep->csc = syscon_node_to_regmap(node);

	return 0;
}