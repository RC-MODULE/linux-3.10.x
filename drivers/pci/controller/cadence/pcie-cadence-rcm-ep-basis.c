#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "../../../clocksource/timer-sp.h"
#include "pcie-cadence-rcm.h"

#define LEGACY_IRQ_DELAY (50)

int rcm_cdns_pcie_ep_send_legacy_irq(struct rcm_cdns_pcie_ep *ep, u8 fn, 
                                     u8 intx)
{
	u16 cmd;
	u16 status;
	u32 reg;
	struct irq_data *irq_data;
	int num_irq;

	cmd = cdns_pcie_ep_fn_readw(&ep->pcie, fn, PCI_COMMAND);
	if (cmd & PCI_COMMAND_INTX_DISABLE)
	{
		pr_info("%s: INTx Message Disabled\n", __func__);
		return -EINVAL;
	}

	irq_data = irq_get_irq_data(ep->irq_timer);
	num_irq = irq_data->hwirq - 32;

	if ((!ep->ext_irq_gen_base) || (!ep->timer_base)) {
		dev_info(ep->pcie.dev,
		         "Can't generate legacy interrupt "
		         "(EXT_IRQ_GEN and disabled timer needed)\n");
		return -EINVAL;
	}

	do {
		status = cdns_pcie_ep_fn_readw(&ep->pcie, 0, PCI_STATUS);
	} while (status & PCI_STATUS_INTERRUPT);

	writel(0x8, ep->ext_irq_gen_base + 0x0);
	if (num_irq < 32)
		writel(~BIT(num_irq), ep->ext_irq_gen_base + 0x0C);
	else
		writel(~BIT(num_irq - 32), ep->ext_irq_gen_base + 0x10);

	reg  = TIMER_CTRL_32BIT | TIMER_CTRL_IE;

	writel(reg,                     ep->timer_base + TIMER_CTRL);
	writel(0,                       ep->timer_base + TIMER_LOAD);
	writel(reg | TIMER_CTRL_ENABLE, ep->timer_base + TIMER_CTRL);

	return 0;
}

static irqreturn_t rcm_cdns_pcie_ep_timer_irq(int irq, void *dev_id)
{
	struct rcm_cdns_pcie_ep *ep = dev_id;

	writel(1, ep->timer_base + TIMER_INTCLR);
	writel(0, ep->timer_base + TIMER_CTRL);

	tasklet_schedule(&ep->tasklet);;

	return IRQ_HANDLED;
}

static void rcm_cdns_pcie_ep_tasklet(unsigned long data)
{
	struct rcm_cdns_pcie_ep *ep = (struct rcm_cdns_pcie_ep *)data;
	struct irq_data *irq_data = irq_get_irq_data(ep->irq_timer);
	int num_irq = irq_data->hwirq - 32;

	udelay(LEGACY_IRQ_DELAY);

	if (num_irq < 32)
		writel(BIT(num_irq), ep->ext_irq_gen_base + 0x04);
	else
		writel(BIT(num_irq - 32), ep->ext_irq_gen_base + 0x08);
}

static const struct of_device_id sp804_of_match[] = {
	{
		.compatible = "arm,sp804",
	},
	{},
};

int rcm_cdns_pcie_ep_setup_priv(struct rcm_cdns_pcie_ep *ep)
{
	struct device *dev = ep->pcie.dev;
	struct device_node* node;
	const struct of_device_id *match;
	int ret;

	node = of_find_node_by_name(NULL, "ext_irq_gen");
	if (!node) {
		dev_err(dev, "EXT_IRQ_GEN not found.\n");
	} else {
		ep->ext_irq_gen_base = of_iomap(node, 0);

		if (!ep->ext_irq_gen_base)
			dev_err(dev, "Failed to map EXT_IRQ_GEN memory.\n");
	}

	for_each_matching_node_and_match(node, sp804_of_match, &match) {
		if (!of_device_is_available(node)) {
			ep->timer_base = of_iomap(node, 0);
			if (!ep->timer_base) {
				dev_err(dev, "Failed to map timer memory\n");
				continue;
			}

			ep->irq_timer = of_irq_get(node, 0);

			if (ep->irq_timer < 0) {
				iounmap(ep->timer_base);
				ep->timer_base = NULL;
				dev_err(dev, "Failed to get timer IRQ\n");
				continue;
			}

			ret = devm_request_irq(dev, ep->irq_timer,
			                       rcm_cdns_pcie_ep_timer_irq,
			                       IRQF_SHARED, "rcm_pcie_timer",
			                       ep);

			if (ret) {
				iounmap(ep->timer_base);
				ep->timer_base = NULL;
				dev_err(dev, "Failed to allocate timer IRQ\n");
				continue;
			}
			tasklet_init(&ep->tasklet,
			             rcm_cdns_pcie_ep_tasklet, (ulong)ep);

			break;
		}
	}

	if (!ep->timer_base)
		dev_err(dev, "No timer for legacy interrupts were found.\n");

	return 0;
}