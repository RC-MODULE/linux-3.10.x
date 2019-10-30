// SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe host controller driver for NWL PCIe Bridge
 * Based on pcie-xilinx-nwl.c
 *
 * Copyright (C) 2019 by AstroSoft
 * Alexey Spirkov <alexeis@astrosoft.ru>
*/

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/irqchip/chained_irq.h>
#include <asm/pci-bridge.h>

#include "../pci.h"

/* Bridge core config registers */
#define BRCFG_PCIE_RX0			0x00000000
#define BRCFG_INTERRUPT			0x00000010
#define BRCFG_PCIE_RX_MSG_FILTER	0x00000020

/* Egress - Bridge translation registers */
#define E_BREG_CAPABILITIES		0x00000200
#define E_BREG_CONTROL			0x00000208
#define E_BREG_BASE_LO			0x00000210
#define E_BREG_BASE_HI			0x00000214
#define E_ECAM_CAPABILITIES		0x00000220
#define E_ECAM_CONTROL			0x00000228
#define E_ECAM_BASE_LO			0x00000230
#define E_ECAM_BASE_HI			0x00000234

#define	E_DREG_CAPABILITIES			(0x00000280 + 0x00)
#define	E_DREG_STATUS				(0x00000280 + 0x04)
#define	E_DREG_CONTROL				(0x00000280 + 0x08)
#define	E_DREG_BASE_LO				(0x00000280 + 0x10)
#define	E_DREG_BASE_HI				(0x00000280 + 0x14)

/* Ingress - address translations */
#define I_MSII_CAPABILITIES		0x00000300
#define I_MSII_CONTROL			0x00000308
#define I_MSII_BASE_LO			0x00000310
#define I_MSII_BASE_HI			0x00000314

#define I_ISUB_CONTROL			0x000003E8
#define SET_ISUB_CONTROL		BIT(0)
#define I_ESUB_CONTROL			0x000002E8
#define SET_ESUB_CONTROL		BIT(0)

/* Rxed msg fifo  - Interrupt status registers */
#define MSGF_MISC_STATUS		0x00000400
#define MSGF_MISC_MASK			0x00000404
#define MSGF_MISC_MASTER_ID		0x0000040c
#define MSGF_LEG_STATUS			0x00000420
#define MSGF_LEG_MASK			0x00000424
#define MSGF_MSI_STATUS_LO		0x00000440
#define MSGF_MSI_STATUS_HI		0x00000444
#define MSGF_MSI_MASK_LO		0x00000448
#define MSGF_MSI_MASK_HI		0x0000044C

/* Msg filter mask bits */
#define CFG_ENABLE_PM_MSG_FWD		BIT(1)
#define CFG_ENABLE_INT_MSG_FWD		BIT(2)
#define CFG_ENABLE_ERR_MSG_FWD		BIT(3)
#define CFG_ENABLE_MSG_FILTER_MASK	(CFG_ENABLE_PM_MSG_FWD | \
					CFG_ENABLE_INT_MSG_FWD | \
					CFG_ENABLE_ERR_MSG_FWD)

/* Misc interrupt status mask bits */
#define MSGF_MISC_SR_RXMSG_AVAIL	BIT(0)
#define MSGF_MISC_SR_RXMSG_OVER		BIT(1)
#define MSGF_MISC_SR_SLAVE_ERR		BIT(4)
#define MSGF_MISC_SR_MASTER_ERR		BIT(5)
#define MSGF_MISC_SR_I_ADDR_ERR		BIT(6)
#define MSGF_MISC_SR_E_ADDR_ERR		BIT(7)
#define MSGF_MISC_SR_FATAL_AER		BIT(16)
#define MSGF_MISC_SR_NON_FATAL_AER	BIT(17)
#define MSGF_MISC_SR_CORR_AER		BIT(18)
#define MSGF_MISC_SR_UR_DETECT		BIT(20)
#define MSGF_MISC_SR_NON_FATAL_DEV	BIT(22)
#define MSGF_MISC_SR_FATAL_DEV		BIT(23)
#define MSGF_MISC_SR_LINK_DOWN		BIT(24)
#define MSGF_MSIC_SR_LINK_AUTO_BWIDTH	BIT(25)
#define MSGF_MSIC_SR_LINK_BWIDTH	BIT(26)

#define MSGF_MISC_SR_MASKALL		(MSGF_MISC_SR_RXMSG_AVAIL | \
					MSGF_MISC_SR_RXMSG_OVER | \
					MSGF_MISC_SR_SLAVE_ERR | \
					MSGF_MISC_SR_MASTER_ERR | \
					MSGF_MISC_SR_I_ADDR_ERR | \
					MSGF_MISC_SR_E_ADDR_ERR | \
					MSGF_MISC_SR_FATAL_AER | \
					MSGF_MISC_SR_NON_FATAL_AER | \
					MSGF_MISC_SR_CORR_AER | \
					MSGF_MISC_SR_UR_DETECT | \
					MSGF_MISC_SR_NON_FATAL_DEV | \
					MSGF_MISC_SR_FATAL_DEV | \
					MSGF_MISC_SR_LINK_DOWN | \
					MSGF_MSIC_SR_LINK_AUTO_BWIDTH | \
					MSGF_MSIC_SR_LINK_BWIDTH)

/* Legacy interrupt status mask bits */
#define MSGF_LEG_SR_INTA		BIT(0)
#define MSGF_LEG_SR_INTB		BIT(1)
#define MSGF_LEG_SR_INTC		BIT(2)
#define MSGF_LEG_SR_INTD		BIT(3)
#define MSGF_LEG_SR_MASKALL		(MSGF_LEG_SR_INTA | MSGF_LEG_SR_INTB | \
					MSGF_LEG_SR_INTC | MSGF_LEG_SR_INTD)

/* MSI interrupt status mask bits */
#define MSGF_MSI_SR_LO_MASK		GENMASK(31, 0)
#define MSGF_MSI_SR_HI_MASK		GENMASK(31, 0)

#define MSII_PRESENT			BIT(0)
#define MSII_ENABLE			BIT(0)
#define MSII_STATUS_ENABLE		BIT(15)

/* Bridge config interrupt mask */
#define BRCFG_INTERRUPT_MASK		BIT(0)
#define BREG_PRESENT			BIT(0)
#define BREG_ENABLE			BIT(0)
#define BREG_ENABLE_FORCE		BIT(1)

/* E_ECAM status mask bits */
#define E_ECAM_PRESENT			BIT(0)
#define E_ECAM_CR_ENABLE		BIT(0)
#define E_ECAM_SIZE_LOC			GENMASK(20, 16)
#define E_ECAM_SIZE_SHIFT		16
#define ECAM_BUS_LOC_SHIFT		20
#define ECAM_DEV_LOC_SHIFT		12
#define NWL_ECAM_VALUE_DEFAULT		12

#define CFG_DMA_REG_BAR			GENMASK(2, 0)

#define INT_PCI_MSI_NR			(2 * 32)

/* Readin the PS_LINKUP */
#define PS_LINKUP_OFFSET		0x0000001c
#define PHY_RDY_LINKUP_BIT		BIT(0)

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES          10
#define LINK_WAIT_USLEEP_MIN           90000
#define LINK_WAIT_USLEEP_MAX           100000

struct nwl_msi {			/* MSI information */
	struct irq_domain *msi_domain;
	unsigned long *bitmap;
	struct irq_domain *dev_domain;
	struct mutex lock;		/* protect bitmap variable */
	int irq_msi0;
	int irq_msi1;
};

struct nwl_pcie {
	struct device *dev;
	void __iomem *breg_base;
	void __iomem *pcireg_base;
	void __iomem *ecam_base;
	void __iomem *serdes_base;
	phys_addr_t phys_breg_base;	/* Physical Bridge Register Base */
	phys_addr_t phys_pcie_reg_base;	/* Physical PCIe Controller Base */
	phys_addr_t phys_ecam_base;	/* Physical Configuration Base */
	u32 breg_size;
	u32 pcie_reg_size;
	u32 ecam_size;
	int irq_intx;
	int irq_misc;
	u32 ecam_value;
	u8 last_busno;
	u8 root_busno;
	struct nwl_msi msi;
	struct irq_domain *legacy_irq_domain;
	raw_spinlock_t leg_mask_lock;
};

static void __pcie_writel(u32 val, void __iomem * iomem, u32 off)
{
	__raw_writel(cpu_to_le32(val), iomem + off);
}

static inline u32 __pcie_readl(void __iomem * iomem, u32 off)
{
	return le32_to_cpu(__raw_readl(iomem + off));
}


static inline u32 nwl_bridge_readl(struct nwl_pcie *pcie, u32 off)
{
	return le32_to_cpu(__raw_readl(pcie->breg_base + off));
}

static inline void nwl_bridge_writel(struct nwl_pcie *pcie, u32 val, u32 off)
{
    __raw_writel(cpu_to_le32(val), pcie->breg_base + off);
}

/* bus device and function of bridge */
#define 	PCI_NWL_RP_ID			(0x0000 + 0x00 + 0x0)

/* mgmt_pcie_status[31:0] */
#define PCI_NWL_MGMT_PCIE_STATUS_DW_0x00                        0x600
#define         PCI_NWL_PHY_LAYER_UP_MASK                       0x00000001
#define         PCI_NWL_DL_LAYER_UP_MASK                        0x00000002
#define         PCI_NWL_LTSSM_STATE_MASK                        0x000000fc
#define                 PCI_NWL_LTSSM_STATE_L0                  0x0000000c


#define	PCI_NWL_MGMT_CFG_CONST_DW_0x10			0x10
#define		PCI_NWL_MGMT_INTERRUPT_ENABLE_MASK	0x00000001
#define		PCI_NWL_MGMT_INTERRUPT_ENABLE		0x00000000
#define		PCI_NWL_MGMT_EXP_CAPABILITIES_EN_MASK	0x00000002
#define		PCI_NWL_MGMT_EXP_CAPABILITIES_EN	0x00000000
#define		PCI_NWL_MGMT_TARGET_ONLY_MASK		0x00000004
#define		PCI_NWL_MGMT_TARGET_ONLY		0x00000000
#define		PCI_NWL_MGMT_T1_TX_BYPASS_MSG_DEC_MASK	0x00000020
#define		PCI_NWL_MGMT_T1_TX_BYPASS_MSG_DEC	0x00000000
#define		PCI_NWL_MGMT_T1_TX_BYPASS_ADDR_DEC_MASK	0x00000040
#define		PCI_NWL_MGMT_T1_TX_BYPASS_ADDR_DEC	0x00000000
#define		PCI_NWL_MGMT_BAR0_CFG_LO_MASK		0xffff0000
#define		PCI_NWL_MGMT_BAR0_CFG_LO_OFFSET		16

/* mgmt_cfg_constants[191:160] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x14			0x14
#define		PCI_NWL_MGMT_BAR0_CFG_HI_MASK		0x0000ffff
#define		PCI_NWL_MGMT_BAR0_CFG_HI_OFFSET		0
#define		PCI_NWL_MGMT_BAR1_CFG_LO_MASK		0xffff0000
#define		PCI_NWL_MGMT_BAR1_CFG_LO_OFFSET		16

/* mgmt_cfg_constants[223:192] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x18			0x18
#define		PCI_NWL_MGMT_BAR1_CFG_HI_MASK		0x0000ffff
#define		PCI_NWL_MGMT_BAR1_CFG_HI_OFFSET		0
#define		PCI_NWL_MGMT_BAR2_CFG_LO_MASK		0xffff0000
#define		PCI_NWL_MGMT_BAR2_CFG_LO_OFFSET		16

/* mgmt_cfg_constants[255:224] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x1c			0x1c
#define		PCI_NWL_MGMT_BAR2_CFG_HI_MASK		0x0000ffff
#define		PCI_NWL_MGMT_BAR2_CFG_HI_OFFSET		0
#define		PCI_NWL_MGMT_BAR3_CFG_LO_MASK		0xffff0000
#define		PCI_NWL_MGMT_BAR3_CFG_LO_OFFSET		16

/* mgmt_cfg_constants[383:352] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x2c			0x2c
#define		PCI_NWL_MGMT_EN_L_POWER_MGMT_MASK	0x07000000
#define		PCI_NWL_MGMT_EN_L_POWER_MGMT		0x00000000	//0x07000000 - enable, 0x00000000 - disable

/* mgmt_cfg_constants[447:416] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x34			0x34
#define		PCI_NWL_RP_ID_7_0			(PCI_NWL_RP_ID & 0xff)
#define		PCI_NWL_RP_ID_7_0_MASK			0xff000000

/* mgmt_cfg_constants[479:448] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x38				0x38
#define		PCI_NWL_RP_ID_15_8_MASK				0x000000ff
#define		PCI_NWL_RP_ID_15_8				(PCI_NWL_RP_ID >> 8)
#define		PCI_NWL_CONFIG_REQ_RETRY_STATUS_EN_BIT_MASK	0x00400000
#define		PCI_NWL_CONFIG_REQ_RETRY_STATUS_EN_BIT		0x00000000	//0x00000000 -  allow Type 0 Configuration Writes and Reads, 0x00400000 -  Return Configuration Request Retry Status for Type 0 Configuration Reads and Writes
#define		PCI_NWL_RP_SW_MODE_MASK				0x00000900
//TODO: вернуть значение 0x00000800
#define		PCI_NWL_RP_SW_MODE_AS_RP			0x00000800
// #define		PCI_NWL_RP_SW_MODE_AS_RP		0x00000800
// #define		PCI_NWL_RP_SW_MODE_AS_RP		0x00000000
#define		PCI_NWL_RP_SW_MODE_AS_EP			0x00000000
#define		PCI_NWL_SUPPORT_DIRECT_5GTS_MASK		0x00030000
#define		PCI_NWL_SUPPORT_DIRECT_5GTS			0x00000000	//0x00030000 - support and direct to 4 gt/s, 0x00000000 - otherwise
#define		PCI_NWL_RP_MODE_FORCE_MASK			0x00000400
#define		PCI_NWL_RP_MODE_FORCE				0x00000000	//always must be 0
#define		PCI_NWL_T1_RX_BYPASS_MSG_DEC_MASK		0x00800000
#define		PCI_NWL_T1_RX_BYPASS_MSG_DEC			0x00000000

/* mgmt_cfg_constants[607:576] */
#define	PCI_NWL_MGMT_CFG_CONST_DW_0x48				0x48
#define		PCI_NWL_DISABLE_INFER_8g_5g_2_5g_MASK		0x00000038
#define		PCI_NWL_DISABLE_INFER_8g_5g_2_5g		0x00000038	//0x00000038 - disable infer 8g, 5g, 2.5g, 0x00000000 - otherwise


#define	XHSIFCTRL_WRITE_ENABLE_OFFSET				(0x5000 + 0xf00)
#define	XHSIFCTRL_FREQ_SETTINGS_OFFSET				(0x5000 + 0x44)
#define	XHSIFCTRL_WRITE_ENABLE_MAGIC				0x98fa2c45

#define	XHSIFCTRL_SERDES_MUX_CONTROL0_OFFSET 			(0x5000 + 0x030)
#define	XHSIFCTRL_SERDES_MUX_CONTROL1_OFFSET 			(0x5000 + 0x034)

#define	PCI_INNO_SOFT_RST_OFFSET				0x4000
#define		PCI_INNO_SOFT_RST_MASK				0x00000001
#define		PCI_INNO_SOFT_RST_ASSERT			0x00000000
#define		PCI_INNO_SOFT_RST_DEASSERT			0x00000001

/* Readin the PS_LINKUP */
#define	XHSIF_CTRL_PCIE_STATUS0_OFFSET 				(0x5000 + 0x1c)
#define	PHY_RDY_LINKUP_BIT					BIT(0)


/* mgmt_cfg_control[191:160] */
#define	PCI_NWL_MGMT_CFG_CONTROL_DW_0x14			(0x200 + 0x14)
#define		PCI_NWL_READ_COMPLETION_BOUNDARY_MASK		0x00020000
#define		PCI_NWL_READ_COMPLETION_BOUNDARY		0x00000000	//0x00000000 - 64 bytes, 0x00020000 - 128 bytes
#define		PCI_NWL_DEVICE_PORT_TYPE_MASK			0x03c00000
#define		PCI_NWL_DEVICE_PORT_TYPE			0x01000000	//0x01000000 - root port of pci express root complex
#define		PCI_NWL_SLOT_IMPLEMENTED_MASK			0x04000000
#define		PCI_NWL_SLOT_IMPLEMENTED			0x00000000	//0x04000000 - slot Implemented, 0x00000000 - Otherwise

#define	PCIE_CFG_CMD_BUSM_EN			0x00000004  /* Bus master enable */
#define	PCIE_CFG_CMD_MEM_EN			0x00000002  /* Memory access enable */
#define	PCIE_CFG_CMD_IO_EN			0x00000001 	/* I/O access enable */
#define	PCIE_CFG_CMD_PARITY			0x00000040 	/* parity errors response */
#define	PCIE_CFG_CMD_SERR_EN			0x00000100 	/* SERR report enable */


static void enable_write_to_xhsif_ctrl_registers (struct nwl_pcie *pcie) 
{
	__pcie_writel(XHSIFCTRL_WRITE_ENABLE_MAGIC, pcie->pcireg_base, XHSIFCTRL_WRITE_ENABLE_OFFSET);
}


static int nwl_pcie_core_init(struct nwl_pcie *pcie)
{
    /* установка mgmt_cfg_constants */

    /* set Device ID and Vendor ID*/
    //__pcie_writel(0xBBBBAAAA, pcie->pcireg_base, 0x00);

	__pcie_writel((__pcie_readl(pcie->pcireg_base, 0x04 ) & 0x000000FF) | 0x6040000, pcie->pcireg_base, 0x04);	

    /* clear BARs 0-1 */
    __pcie_writel(__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x10) & (~PCI_NWL_MGMT_BAR0_CFG_LO_MASK),
		 pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x10);
    __pcie_writel(__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x14) & (~PCI_NWL_MGMT_BAR0_CFG_HI_MASK),
		 pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x14);
    __pcie_writel(__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x14) & (~PCI_NWL_MGMT_BAR1_CFG_LO_MASK),
		 pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x14);
    __pcie_writel(__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x18) & (~PCI_NWL_MGMT_BAR1_CFG_HI_MASK),
		pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x18);

    /* Clear "Enable L2 Power Mgmt", "Enable L1 Power Mgmt", "Enable L0s Power Mgmt" */
    __pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x2c) & (~PCI_NWL_MGMT_EN_L_POWER_MGMT_MASK)) |
		PCI_NWL_MGMT_EN_L_POWER_MGMT, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x2c);
	
	/* root port mode {mgmt_sw_mode,mgmt_rp_mode}=b10 */
	__pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38)&(~PCI_NWL_RP_SW_MODE_MASK))|
		PCI_NWL_RP_SW_MODE_AS_RP, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38);

	/* root port type */
	__pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONTROL_DW_0x14)&(~PCI_NWL_DEVICE_PORT_TYPE_MASK))|
		PCI_NWL_DEVICE_PORT_TYPE, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONTROL_DW_0x14);

	/* root port id[7:0] */
	__pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x34)&(~PCI_NWL_RP_ID_7_0_MASK))|
		PCI_NWL_RP_ID_7_0, pcie->pcireg_base,  PCI_NWL_MGMT_CFG_CONST_DW_0x34);

	/* root port id[15:8] */
	__pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38)&(~PCI_NWL_RP_ID_15_8_MASK))|
		PCI_NWL_RP_ID_15_8, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38);

    /* "Support_5GT/s", "Direct to 5GT/s" */
    __pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38)&(~PCI_NWL_SUPPORT_DIRECT_5GTS_MASK))|
		PCI_NWL_SUPPORT_DIRECT_5GTS, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38);

// limit 1 lane
    //__pcie_writel(__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x48) & 0xfffffff8 | 1, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x48);


    /* infer 8g, 5g, 2.5g */
    __pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x48) & (~PCI_NWL_DISABLE_INFER_8g_5g_2_5g_MASK)) |
		PCI_NWL_DISABLE_INFER_8g_5g_2_5g, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x48);
		
    /* Configuration Request Retry Status Enable */
    __pcie_writel((__pcie_readl(pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38)&(~PCI_NWL_CONFIG_REQ_RETRY_STATUS_EN_BIT_MASK))|
		PCI_NWL_CONFIG_REQ_RETRY_STATUS_EN_BIT, pcie->pcireg_base, PCI_NWL_MGMT_CFG_CONST_DW_0x38);

    return 0;
}

static void inno_phy_soft_reset(struct nwl_pcie *pcie) 
{

    __pcie_writel((__pcie_readl(pcie->serdes_base, 0x00000 + PCI_INNO_SOFT_RST_OFFSET) & (~PCI_INNO_SOFT_RST_MASK)) | 
		PCI_INNO_SOFT_RST_DEASSERT, pcie->serdes_base, 0x00000 + PCI_INNO_SOFT_RST_OFFSET);
    __pcie_writel((__pcie_readl(pcie->serdes_base, 0x10000 + PCI_INNO_SOFT_RST_OFFSET) & (~PCI_INNO_SOFT_RST_MASK)) | 
		PCI_INNO_SOFT_RST_DEASSERT, pcie->serdes_base, 0x10000 + PCI_INNO_SOFT_RST_OFFSET);
    __pcie_writel((__pcie_readl(pcie->serdes_base, 0x20000 + PCI_INNO_SOFT_RST_OFFSET) & (~PCI_INNO_SOFT_RST_MASK)) | 
		PCI_INNO_SOFT_RST_DEASSERT, pcie->serdes_base, 0x20000 + PCI_INNO_SOFT_RST_OFFSET);
    __pcie_writel((__pcie_readl(pcie->serdes_base, 0x30000 + PCI_INNO_SOFT_RST_OFFSET) & (~PCI_INNO_SOFT_RST_MASK)) | 
		PCI_INNO_SOFT_RST_DEASSERT, pcie->serdes_base, 0x30000 + PCI_INNO_SOFT_RST_OFFSET);
    
	msleep(150);
    
    __pcie_writel(0x11100000, pcie->pcireg_base, XHSIFCTRL_SERDES_MUX_CONTROL1_OFFSET);
    __pcie_writel(__pcie_readl(pcie->pcireg_base, XHSIFCTRL_SERDES_MUX_CONTROL0_OFFSET) | 0x80000000, pcie->pcireg_base, XHSIFCTRL_SERDES_MUX_CONTROL0_OFFSET);
}


static void inno_phy_init(struct nwl_pcie *pcie)
{
	    //***************************************************************
    //  Initialize SERDES for PCIe mode
    //***************************************************************
    __pcie_writel(0x19, pcie->serdes_base, 0x00000 + 0x3028);  //  tx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x00000 + 0x311c);  //  rx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x10000 + 0x3028);  //  tx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x10000 + 0x311c);  //  rx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x20000 + 0x3028);  //  tx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x20000 + 0x311c);  //  rx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x30000 + 0x3028);  //  tx pll mult
    __pcie_writel(0x19, pcie->serdes_base, 0x30000 + 0x311c);  //  rx pll mult


    __pcie_writel(0x0aa0, pcie->pcireg_base, XHSIFCTRL_FREQ_SETTINGS_OFFSET);	// magic with freq

    __pcie_writel(0x58, pcie->serdes_base, 0x00000 + 0x3160);
    __pcie_writel(0x58, pcie->serdes_base, 0x10000 + 0x3160);
    __pcie_writel(0x58, pcie->serdes_base, 0x20000 + 0x3160);
    __pcie_writel(0x58, pcie->serdes_base, 0x30000 + 0x3160);
}

static bool nwl_phy_link_up(struct nwl_pcie *pcie)
{
    if (__pcie_readl(pcie->pcireg_base, XHSIF_CTRL_PCIE_STATUS0_OFFSET) & PHY_RDY_LINKUP_BIT)
		return true;
	return false;
}

static bool nwl_pcie_link_up(struct nwl_pcie *pcie)
{
   uint32_t pcie_status = __pcie_readl(pcie->pcireg_base,PCI_NWL_MGMT_PCIE_STATUS_DW_0x00);
    if ((pcie_status & PCI_NWL_PHY_LAYER_UP_MASK) && 
		(pcie_status & PCI_NWL_DL_LAYER_UP_MASK) && 
		nwl_phy_link_up(pcie) && 
		(pcie_status & PCI_NWL_LTSSM_STATE_MASK)==PCI_NWL_LTSSM_STATE_L0)
		return true;
    return false;
}

static int nwl_wait_for_link(struct nwl_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int retries;

	/* check if the link is up or not */
	for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		if (nwl_phy_link_up(pcie))
			return 0;
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	dev_err(dev, "PHY link never came up\n");
	return -ETIMEDOUT;
}

static bool nwl_pcie_valid_device(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_controller *hose = bus->sysdata;
	struct nwl_pcie *pcie = hose->private_data;

	/* Check link before accessing downstream ports */
	if (bus->number != pcie->root_busno) {
		if (!nwl_pcie_link_up(pcie))
			return false;
	}

	/* Only one device down on each root port */
	if (bus->number == pcie->root_busno && devfn > 0)
		return false;

	return true;
}

/**
 * nwl_pcie_map_bus - Get configuration base
 *
 * @bus: Bus structure of current bus
 * @devfn: Device/function
 * @where: Offset from base
 *
 * Return: Base address of the configuration space needed to be
 *	   accessed.
 */
static void __iomem *nwl_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
				      int where)
{
	struct pci_controller *hose = bus->sysdata;
	struct nwl_pcie *pcie = hose->private_data;
	int relbus;

	if (!nwl_pcie_valid_device(bus, devfn))
		return NULL;

	relbus = (bus->number << ECAM_BUS_LOC_SHIFT) |
			(devfn << ECAM_DEV_LOC_SHIFT);

	return pcie->ecam_base + relbus + where;
}


static int nwl_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
			    int where, int size, u32 *val)
{
	void __iomem *addr;

	addr = bus->ops->map_bus(bus, devfn, where);
	if (!addr) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (size == 1)
		*val = readb(addr);
	else if (size == 2)
		*val = (readw(addr));
	else
		*val = (readl(addr));

	return PCIBIOS_SUCCESSFUL;
}


static int nwl_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 val)
{
	void __iomem *addr;

	addr = bus->ops->map_bus(bus, devfn, where);
	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size == 1)
		writeb(val, addr);
	else if (size == 2)
		writew((val), addr);
	else
		writel((val), addr);

	return PCIBIOS_SUCCESSFUL;
}



/* PCIe operations */
static struct pci_ops nwl_pcie_ops = {
	.map_bus = nwl_pcie_map_bus,
	.read  = nwl_pcie_config_read,
	.write = nwl_pcie_config_write,
};

static irqreturn_t nwl_pcie_dummy_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}


static irqreturn_t nwl_pcie_misc_handler(int irq, void *data)
{
	struct nwl_pcie *pcie = data;
	struct device *dev = pcie->dev;
	u32 misc_stat;

	/* Checking for misc interrupts */
	misc_stat = nwl_bridge_readl(pcie, MSGF_MISC_STATUS) &
				     MSGF_MISC_SR_MASKALL;
	if (!misc_stat)
		return IRQ_NONE;

	if (misc_stat & MSGF_MISC_SR_RXMSG_OVER)
		dev_err(dev, "Received Message FIFO Overflow\n");

	if (misc_stat & MSGF_MISC_SR_SLAVE_ERR)
		dev_err(dev, "Slave error\n");

	if (misc_stat & MSGF_MISC_SR_MASTER_ERR)
	{
		u32 master_err;
		master_err = nwl_bridge_readl(pcie, MSGF_MISC_MASTER_ID);

		dev_err(dev, "Master error %08X\n", master_err);

	}

	if (misc_stat & MSGF_MISC_SR_I_ADDR_ERR)
		dev_err(dev, "In Misc Ingress address translation error\n");

	if (misc_stat & MSGF_MISC_SR_E_ADDR_ERR)
		dev_err(dev, "In Misc Egress address translation error\n");

	if (misc_stat & MSGF_MISC_SR_FATAL_AER)
		dev_err(dev, "Fatal Error in AER Capability\n");

	if (misc_stat & MSGF_MISC_SR_NON_FATAL_AER)
		dev_err(dev, "Non-Fatal Error in AER Capability\n");

	if (misc_stat & MSGF_MISC_SR_CORR_AER)
		dev_err(dev, "Correctable Error in AER Capability\n");

	if (misc_stat & MSGF_MISC_SR_UR_DETECT)
		dev_err(dev, "Unsupported request Detected\n");

	if (misc_stat & MSGF_MISC_SR_NON_FATAL_DEV)
		dev_err(dev, "Non-Fatal Error Detected\n");

	if (misc_stat & MSGF_MISC_SR_FATAL_DEV)
		dev_err(dev, "Fatal Error Detected\n");

	if (misc_stat & MSGF_MSIC_SR_LINK_AUTO_BWIDTH)
		dev_info(dev, "Link Autonomous Bandwidth Management Status bit set\n");

	if (misc_stat & MSGF_MSIC_SR_LINK_BWIDTH)
		dev_info(dev, "Link Bandwidth Management Status bit set\n");

	/* Clear misc interrupt status */
	nwl_bridge_writel(pcie, misc_stat, MSGF_MISC_STATUS);

	return IRQ_HANDLED;
}

static void nwl_pcie_leg_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct nwl_pcie *pcie;
	unsigned long status;
	u32 bit;
	u32 virq;

	chained_irq_enter(chip, desc);
	pcie = irq_desc_get_handler_data(desc);

	while ((status = nwl_bridge_readl(pcie, MSGF_LEG_STATUS) &
				MSGF_LEG_SR_MASKALL) != 0) {
		for_each_set_bit(bit, &status, PCI_NUM_INTX) {
			virq = irq_find_mapping(pcie->legacy_irq_domain, bit);
			if (virq)
				generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

static void nwl_pcie_handle_msi_irq(struct nwl_pcie *pcie, u32 status_reg)
{
	struct nwl_msi *msi;
	unsigned long status;
	u32 bit;
	u32 virq;

	msi = &pcie->msi;

	while ((status = nwl_bridge_readl(pcie, status_reg)) != 0) {
		for_each_set_bit(bit, &status, 32) {
			nwl_bridge_writel(pcie, 1 << bit, status_reg);
			virq = irq_find_mapping(msi->dev_domain, bit);
			if (virq)
				generic_handle_irq(virq);
		}
	}
}

static void nwl_pcie_msi_handler_high(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct nwl_pcie *pcie = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);
	nwl_pcie_handle_msi_irq(pcie, MSGF_MSI_STATUS_HI);
	chained_irq_exit(chip, desc);
}

static void nwl_pcie_msi_handler_low(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct nwl_pcie *pcie = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);
	nwl_pcie_handle_msi_irq(pcie, MSGF_MSI_STATUS_LO);
	chained_irq_exit(chip, desc);
}

static void nwl_mask_leg_irq(struct irq_data *data)
{
	struct irq_desc *desc = irq_to_desc(data->irq);
	struct nwl_pcie *pcie;
	unsigned long flags;
	u32 mask;
	u32 val;

	pcie = irq_desc_get_chip_data(desc);
	mask = 1 << (data->hwirq - 1);
	raw_spin_lock_irqsave(&pcie->leg_mask_lock, flags);
	val = nwl_bridge_readl(pcie, MSGF_LEG_MASK);
	nwl_bridge_writel(pcie, (val & (~mask)), MSGF_LEG_MASK);
	raw_spin_unlock_irqrestore(&pcie->leg_mask_lock, flags);
}

static void nwl_unmask_leg_irq(struct irq_data *data)
{
	struct irq_desc *desc = irq_to_desc(data->irq);
	struct nwl_pcie *pcie;
	unsigned long flags;
	u32 mask;
	u32 val;

	pcie = irq_desc_get_chip_data(desc);
	mask = 1 << (data->hwirq - 1);
	raw_spin_lock_irqsave(&pcie->leg_mask_lock, flags);
	val = nwl_bridge_readl(pcie, MSGF_LEG_MASK);
	nwl_bridge_writel(pcie, (val | mask), MSGF_LEG_MASK);
	raw_spin_unlock_irqrestore(&pcie->leg_mask_lock, flags);
}

static struct irq_chip nwl_leg_irq_chip = {
	.name = "nwl_pcie:legacy",
	.irq_enable = nwl_unmask_leg_irq,
	.irq_disable = nwl_mask_leg_irq,
	.irq_mask = nwl_mask_leg_irq,
	.irq_unmask = nwl_unmask_leg_irq,
};

static int nwl_legacy_map(struct irq_domain *domain, unsigned int irq,
			  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &nwl_leg_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

static const struct irq_domain_ops legacy_domain_ops = {
	.map = nwl_legacy_map,
	.xlate = pci_irqd_intx_xlate,
};

#ifdef CONFIG_PCI_MSI
static struct irq_chip nwl_msi_irq_chip = {
	.name = "nwl_pcie:msi",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,

};

static struct msi_domain_info nwl_msi_domain_info = {
	.flags = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_MULTI_PCI_MSI),
	.chip = &nwl_msi_irq_chip,
};
#endif

static void nwl_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct nwl_pcie *pcie = irq_data_get_irq_chip_data(data);
	phys_addr_t msi_addr = pcie->phys_pcie_reg_base;

	msg->address_lo = lower_32_bits(msi_addr);
	msg->address_hi = 0 /*upper_32_bits(msi_addr)*/;
	msg->data = data->hwirq;
}

static int nwl_msi_set_affinity(struct irq_data *irq_data,
				const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static struct irq_chip nwl_irq_chip = {
	.name = "RC-Module MSI",
	.irq_compose_msi_msg = nwl_compose_msi_msg,
	.irq_set_affinity = nwl_msi_set_affinity,
};

static int nwl_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *args)
{
	struct nwl_pcie *pcie = domain->host_data;
	struct nwl_msi *msi = &pcie->msi;
	int bit;
	int i;

	mutex_lock(&msi->lock);
	bit = bitmap_find_next_zero_area(msi->bitmap, INT_PCI_MSI_NR, 0,
					 nr_irqs, 0);
	if (bit >= INT_PCI_MSI_NR) {
		mutex_unlock(&msi->lock);
		return -ENOSPC;
	}

	bitmap_set(msi->bitmap, bit, nr_irqs);

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i, &nwl_irq_chip,
				domain->host_data, handle_simple_irq,
				NULL, NULL);
	}
	mutex_unlock(&msi->lock);
	return 0;
}

static void nwl_irq_domain_free(struct irq_domain *domain, unsigned int virq,
					unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct nwl_pcie *pcie = irq_data_get_irq_chip_data(data);
	struct nwl_msi *msi = &pcie->msi;

	mutex_lock(&msi->lock);
	bitmap_clear(msi->bitmap, data->hwirq, nr_irqs);
	mutex_unlock(&msi->lock);
}

static const struct irq_domain_ops dev_msi_domain_ops = {
	.alloc  = nwl_irq_domain_alloc,
	.free   = nwl_irq_domain_free,
};

static int nwl_pcie_init_msi_irq_domain(struct nwl_pcie *pcie)
{
#ifdef CONFIG_PCI_MSI
	struct device *dev = pcie->dev;
	struct fwnode_handle *fwnode = of_node_to_fwnode(dev->of_node);
	struct nwl_msi *msi = &pcie->msi;

	msi->dev_domain = irq_domain_add_linear(NULL, INT_PCI_MSI_NR,
						&dev_msi_domain_ops, pcie);
	if (!msi->dev_domain) {
		dev_err(dev, "failed to create dev IRQ domain\n");
		return -ENOMEM;
	}
	msi->msi_domain = pci_msi_create_irq_domain(fwnode,
						    &nwl_msi_domain_info,
						    msi->dev_domain);
	if (!msi->msi_domain) {
		dev_err(dev, "failed to create msi IRQ domain\n");
		irq_domain_remove(msi->dev_domain);
		return -ENOMEM;
	}
#endif
	return 0;
}

static int nwl_pcie_init_irq_domain(struct nwl_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *legacy_intc_node;

	legacy_intc_node = of_get_next_child(node, NULL);
	if (!legacy_intc_node) {
		dev_err(dev, "No legacy intc node found\n");
		return -EINVAL;
	}

	pcie->legacy_irq_domain = irq_domain_add_linear(legacy_intc_node,
							PCI_NUM_INTX,
							&legacy_domain_ops,
							pcie);
	of_node_put(legacy_intc_node);
	if (!pcie->legacy_irq_domain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	raw_spin_lock_init(&pcie->leg_mask_lock);
	nwl_pcie_init_msi_irq_domain(pcie);
	return 0;
}

static int nwl_pcie_enable_msi(struct nwl_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct nwl_msi *msi = &pcie->msi;
	phys_addr_t base;
	int ret;
	int size = BITS_TO_LONGS(INT_PCI_MSI_NR) * sizeof(long);

	mutex_init(&msi->lock);

	msi->bitmap = kzalloc(size, GFP_KERNEL);
	if (!msi->bitmap)
		return -ENOMEM;

	/* Get msi_1 IRQ number */
	msi->irq_msi1 = platform_get_irq_byname(pdev, "msi1");
	if (msi->irq_msi1 < 0) {
		dev_err(dev, "failed to get IRQ#%d\n", msi->irq_msi1);
		ret = -EINVAL;
		goto err;
	}

	irq_set_chained_handler_and_data(msi->irq_msi1,
					 nwl_pcie_msi_handler_high, pcie);

	/* Get msi_0 IRQ number */
	msi->irq_msi0 = platform_get_irq_byname(pdev, "msi0");
	if (msi->irq_msi0 < 0) {
		dev_err(dev, "failed to get IRQ#%d\n", msi->irq_msi0);
		ret = -EINVAL;
		goto err;
	}

	irq_set_chained_handler_and_data(msi->irq_msi0,
					 nwl_pcie_msi_handler_low, pcie);

	/* Check for msii_present bit */
	ret = nwl_bridge_readl(pcie, I_MSII_CAPABILITIES) & MSII_PRESENT;
	if (!ret) {
		dev_err(dev, "MSI not present\n");
		ret = -EIO;
		goto err;
	}

	/* Enable MSII */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, I_MSII_CONTROL) |
			  MSII_ENABLE, I_MSII_CONTROL);

	/* Enable MSII status */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, I_MSII_CONTROL) |
			  MSII_STATUS_ENABLE, I_MSII_CONTROL);

	/* setup AFI/FPCI range */
	base = pcie->phys_pcie_reg_base;

	nwl_bridge_writel(pcie, lower_32_bits(base), I_MSII_BASE_LO);
	nwl_bridge_writel(pcie, 0 /*upper_32_bits(base)*/, I_MSII_BASE_HI);	// AXI mapping - toDo

	/*
	 * For high range MSI interrupts: disable, clear any pending,
	 * and enable
	 */
	nwl_bridge_writel(pcie, 0, MSGF_MSI_MASK_HI);

	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie,  MSGF_MSI_STATUS_HI) &
			  MSGF_MSI_SR_HI_MASK, MSGF_MSI_STATUS_HI);

	nwl_bridge_writel(pcie, MSGF_MSI_SR_HI_MASK, MSGF_MSI_MASK_HI);

	/*
	 * For low range MSI interrupts: disable, clear any pending,
	 * and enable
	 */
	nwl_bridge_writel(pcie, 0, MSGF_MSI_MASK_LO);

	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, MSGF_MSI_STATUS_LO) &
			  MSGF_MSI_SR_LO_MASK, MSGF_MSI_STATUS_LO);

	nwl_bridge_writel(pcie, MSGF_MSI_SR_LO_MASK, MSGF_MSI_MASK_LO);

	return 0;
err:
	kfree(msi->bitmap);
	msi->bitmap = NULL;
	return ret;
}


static int nwl_pcie_base_init(struct nwl_pcie *pcie)
{
    enable_write_to_xhsif_ctrl_registers(pcie);
    nwl_pcie_core_init(pcie);
	inno_phy_init(pcie);
	inno_phy_soft_reset(pcie);
	return 0;
}


#define	TRAN_EGRESS_0_BASE			0xc00
#define	TRAN_EGRESS_1_BASE			0xc20
#define	TRAN_EGRESS_2_BASE			0xc40
#define	TRAN_EGRESS_3_BASE			0xc60
#define	TRAN_EGRESS_4_BASE			0xc80
#define	TRAN_EGRESS_5_BASE			0xca0
#define	TRAN_EGRESS_6_BASE			0xcc0
#define	TRAN_EGRESS_7_BASE			0xce0

#define	TRAN_EGRESS_CAPABILITIES		0x00
#define	TRAN_EGRESS_STATUS			0x04
#define	TRAN_EGRESS_CONTROL			0x08
#define	TRAN_EGRESS_CTRL_SIZE_SHIFT		16
#define	TRAN_EGRESS_CTRL_SIZE_LOC		0x001f0000
#define	TRAN_EGRESS_SRC_BASE_LO			0x10
#define	TRAN_EGRESS_SRC_BASE_HI			0x14
#define	TRAN_EGRESS_DST_BASE_LO			0x18
#define	TRAN_EGRESS_DST_BASE_HI			0x1c

#define	TRAN_INGRESS_0_BASE			0x800
#define	TRAN_INGRESS_1_BASE			0x820
#define	TRAN_INGRESS_2_BASE			0x840
#define	TRAN_INGRESS_3_BASE			0x860
#define	TRAN_INGRESS_4_BASE			0x880
#define	TRAN_INGRESS_5_BASE			0x8a0
#define	TRAN_INGRESS_6_BASE			0x8c0
#define	TRAN_INGRESS_7_BASE			0x8e0

#define	TRAN_INGRESS_CAPABILITIES	0x00
#define	TRAN_INGRESS_STATUS			0x04
#define	TRAN_INGRESS_CONTROL		0x08
#define	TRAN_INGRESS_CTRL_SIZE_SHIFT 16
#define	TRAN_INGRESS_CTRL_SIZE_LOC	0x001f0000
#define	TRAN_INGRESS_SRC_BASE_LO	0x10
#define	TRAN_INGRESS_SRC_BASE_HI	0x14
#define	TRAN_INGRESS_DST_BASE_LO	0x18
#define	TRAN_INGRESS_DST_BASE_HI	0x1c


static void egress_translate(struct nwl_pcie *pcie, unsigned long long src_addr, unsigned long long dst_addr, uint32_t egress_base, uint32_t egress_size) {
	uint32_t breg_val;
	printk("TRACE: egress_translate: %x, %llx, %llx, %x, %x", 
		nwl_bridge_readl(pcie, egress_base + TRAN_EGRESS_CAPABILITIES), 
		src_addr, dst_addr, egress_base, egress_size);

	nwl_bridge_writel(pcie, upper_32_bits(src_addr), egress_base + TRAN_EGRESS_SRC_BASE_HI);
	nwl_bridge_writel(pcie, lower_32_bits(src_addr), egress_base + TRAN_EGRESS_SRC_BASE_LO);
	
	nwl_bridge_writel(pcie, upper_32_bits(dst_addr), egress_base + TRAN_EGRESS_DST_BASE_HI);
	nwl_bridge_writel(pcie, lower_32_bits(dst_addr), egress_base + TRAN_EGRESS_DST_BASE_LO);
	
	breg_val = nwl_bridge_readl(pcie, egress_base + TRAN_EGRESS_CONTROL);
	breg_val |= 0x1;
	breg_val = breg_val & (~TRAN_EGRESS_CTRL_SIZE_LOC);
	breg_val = breg_val | (egress_size << TRAN_EGRESS_CTRL_SIZE_SHIFT);
	nwl_bridge_writel(pcie, breg_val, egress_base + TRAN_EGRESS_CONTROL);
}


static void ingress_translate(struct nwl_pcie *pcie, unsigned long long src_addr, unsigned long long dst_addr, uint32_t ingress_base, uint32_t ingress_size) {
	uint32_t breg_val;
	printk("TRACE: ingress_translate: %x, %llx, %llx, %x, %x", 
		nwl_bridge_readl(pcie, ingress_base + TRAN_INGRESS_CAPABILITIES), 
		src_addr, dst_addr, ingress_base, ingress_size);

	nwl_bridge_writel(pcie, upper_32_bits(src_addr), ingress_base + TRAN_INGRESS_SRC_BASE_HI);
	nwl_bridge_writel(pcie, lower_32_bits(src_addr), ingress_base + TRAN_INGRESS_SRC_BASE_LO);
	
	nwl_bridge_writel(pcie, upper_32_bits(dst_addr), ingress_base + TRAN_INGRESS_DST_BASE_HI);
	nwl_bridge_writel(pcie, lower_32_bits(dst_addr), ingress_base + TRAN_INGRESS_DST_BASE_LO);
	
	breg_val = nwl_bridge_readl(pcie, ingress_base + TRAN_INGRESS_CONTROL);
	breg_val |= 0x1;
	breg_val = breg_val & (~TRAN_INGRESS_CTRL_SIZE_LOC);
	breg_val = breg_val | (ingress_size << TRAN_INGRESS_CTRL_SIZE_SHIFT);
	nwl_bridge_writel(pcie, breg_val, ingress_base + TRAN_INGRESS_CONTROL);
}

static int nwl_pcie_bridge_init(struct nwl_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	u32 breg_val, ecam_val, first_busno = 0;
	int err;

	breg_val = nwl_bridge_readl(pcie, E_BREG_CAPABILITIES) & BREG_PRESENT;
	if (!breg_val) {
		dev_err(dev, "BREG is not present\n");
		return breg_val;
	}
	/* Write bridge_off to breg base */
	nwl_bridge_writel(pcie, lower_32_bits(pcie->phys_breg_base),
			  E_BREG_BASE_LO);
	nwl_bridge_writel(pcie, /* upper_32_bits(pcie->phys_breg_base) */ 0,		// AXI mapping ToDo
			  E_BREG_BASE_HI);

	nwl_bridge_writel(pcie, 0x33, 0x4);
	nwl_bridge_writel(pcie, 0x22, 0x8);


	/* Enable BREG */
	nwl_bridge_writel(pcie, (~BREG_ENABLE_FORCE & BREG_ENABLE),
			  E_BREG_CONTROL);
    
	/* Disable DMA channel registers */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, BRCFG_PCIE_RX0) |
			  CFG_DMA_REG_BAR, BRCFG_PCIE_RX0);

	/* Disable Ingress subtractive decode translation */
	nwl_bridge_writel(pcie, 0 /*SET_ISUB_CONTROL*/, I_ISUB_CONTROL);

	nwl_bridge_writel(pcie, SET_ESUB_CONTROL, I_ESUB_CONTROL);

	// egress_translate(pcie, 0x1200000000, 0x300000000, TRAN_EGRESS_0_BASE, 20);

	/* Enable msg filtering details */
	nwl_bridge_writel(pcie, CFG_ENABLE_MSG_FILTER_MASK,
			  BRCFG_PCIE_RX_MSG_FILTER);
   
    err = nwl_wait_for_link(pcie);
	if (err)
		return err;

	ecam_val = nwl_bridge_readl(pcie, E_ECAM_CAPABILITIES) & E_ECAM_PRESENT;
	if (!ecam_val) {
		dev_err(dev, "ECAM is not present\n");
		return ecam_val;
	}

	/* Enable ECAM */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, E_ECAM_CONTROL) |
			  E_ECAM_CR_ENABLE, E_ECAM_CONTROL);

	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, E_ECAM_CONTROL) |
			  (pcie->ecam_value << E_ECAM_SIZE_SHIFT),
			  E_ECAM_CONTROL);

	nwl_bridge_writel(pcie, lower_32_bits(pcie->phys_ecam_base),
			  E_ECAM_BASE_LO);
	nwl_bridge_writel(pcie, /* upper_32_bits(pcie->phys_ecam_base) */ 0,		// AXI mapping todo
			  E_ECAM_BASE_HI);

	/* Get bus range */
	ecam_val = nwl_bridge_readl(pcie, E_ECAM_CONTROL);
	pcie->last_busno = (ecam_val & E_ECAM_SIZE_LOC) >> E_ECAM_SIZE_SHIFT;
	/* Write primary, secondary and subordinate bus numbers */
	ecam_val = first_busno;
	ecam_val |= (first_busno + 1) << 8;
	ecam_val |= (pcie->last_busno << E_ECAM_SIZE_SHIFT);
	__pcie_writel(ecam_val, pcie->ecam_base, PCI_PRIMARY_BUS);

	if (nwl_pcie_link_up(pcie))
		dev_info(dev, "Link is UP\n");
	else
		dev_info(dev, "Link is DOWN\n");

	/* Get misc IRQ number */
	pcie->irq_misc = platform_get_irq_byname(pdev, "misc");
	if (pcie->irq_misc < 0) {
		dev_err(dev, "failed to get misc IRQ %d\n",
			pcie->irq_misc);
		return -EINVAL;
	}

	err = devm_request_irq(dev, pcie->irq_misc,
			       nwl_pcie_misc_handler, IRQF_SHARED,
			       "nwl_pcie:misc", pcie);
	if (err) {
		dev_err(dev, "fail to register misc IRQ#%d\n",
			pcie->irq_misc);
		return err;
	}

	err = devm_request_irq(dev, platform_get_irq_byname(pdev, "dummy"),
			       nwl_pcie_dummy_handler, IRQF_SHARED,
			       "nwl_pcie:dummy", pcie);

	/* Disable all misc interrupts */
	nwl_bridge_writel(pcie, (u32)~MSGF_MISC_SR_MASKALL, MSGF_MISC_MASK);

	/* Clear pending misc interrupts */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, MSGF_MISC_STATUS) &
			  MSGF_MISC_SR_MASKALL, MSGF_MISC_STATUS);

	/* Enable all misc interrupts */
	nwl_bridge_writel(pcie, MSGF_MISC_SR_MASKALL, MSGF_MISC_MASK);


	/* Disable all legacy interrupts */
	nwl_bridge_writel(pcie, (u32)~MSGF_LEG_SR_MASKALL, MSGF_LEG_MASK);

	/* Clear pending legacy interrupts */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, MSGF_LEG_STATUS) &
			  MSGF_LEG_SR_MASKALL, MSGF_LEG_STATUS);

	/* Enable all legacy interrupts */
	nwl_bridge_writel(pcie, MSGF_LEG_SR_MASKALL, MSGF_LEG_MASK);

	/* Enable the bridge config interrupt */
	nwl_bridge_writel(pcie, nwl_bridge_readl(pcie, BRCFG_INTERRUPT) |
			  BRCFG_INTERRUPT_MASK, BRCFG_INTERRUPT);

	return 0;
}


static int nwl_pcie_apply_ingress_range(struct nwl_pcie *pcie,
				    struct of_pci_range *range,
				    int *index)
{
	u64 size;
	u32 size_mask;
	int idx = *index;
	u32 ingress_base = TRAN_INGRESS_0_BASE + 0x20*idx;
	u32 capabilities, size_max, size_offset;

	if(idx > 7)
	{
		dev_err(pcie->dev, "ingress translation index %d out of range", idx);
		return -EINVAL;		
	}

	capabilities = nwl_bridge_readl(pcie, ingress_base + TRAN_INGRESS_CAPABILITIES);

	if(!(capabilities & BIT(0)))
	{
		dev_err(pcie->dev, "ingress translation module %d not found", idx);
		return -EINVAL;		
	}

	size_max = (capabilities >> 24) & 0xFF;
	size_offset = (capabilities >> 16) & 0xFF;

	/*
	 * If the size of the range is larger than the alignment of the start
	 * address, we have to use multiple entries to perform the mapping.
	 */
	if (range->cpu_addr > 0) {
		unsigned long nr_zeros = __ffs64(range->cpu_addr);
		u64 alignment = 1ULL << nr_zeros;

		size = min(range->size, alignment);
	} else {
		size = range->size;
	}
	/* Hardware supports max 4GiB inbound region */
	size = min(size, 1ULL << 32);

	size_mask = ilog2(size) - size_offset;
	if(size_mask > size_max)
	{
		dev_err(pcie->dev, "size [%llx] for dma_regions too big, %x, %x", size, (unsigned int) (ilog2(size)), size_offset);
		return -EINVAL;
	}

	ingress_translate(pcie, range->pci_addr, range->cpu_addr, ingress_base, size_mask);
	*index = idx+1;

	return 0;
}


static int nwl_pcie_parse_map_dma_ranges(struct nwl_pcie *pcie,
					  struct device_node *np)
{
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	int index = 0;
	int err;

	if (of_pci_dma_range_parser_init(&parser, np))
		return -EINVAL;

	/* Get the dma-ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		u64 end = range.cpu_addr + range.size - 1;

		dev_dbg(pcie->dev, "0x%08x 0x%016llx..0x%016llx -> 0x%016llx\n",
			range.flags, range.cpu_addr, end, range.pci_addr);

		err = nwl_pcie_apply_ingress_range(pcie, &range, &index);
		if (err)
			return err;
	}

	return 0;
}



static int nwl_pcie_parse_dt(struct nwl_pcie *pcie,
			     struct platform_device *pdev)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;
	const char *type;

	/* Check for device type */
	type = of_get_property(node, "device_type", NULL);
	if (!type || strcmp(type, "pci")) {
		dev_err(dev, "invalid \"device_type\" %s\n", type);
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "breg");
	pcie->breg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->breg_base))
		return PTR_ERR(pcie->breg_base);
	pcie->phys_breg_base = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcireg");
	pcie->pcireg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->pcireg_base))
		return PTR_ERR(pcie->pcireg_base);
	pcie->phys_pcie_reg_base = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	pcie->ecam_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pcie->ecam_base))
		return PTR_ERR(pcie->ecam_base);
	pcie->phys_ecam_base = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "serdes");
	pcie->serdes_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->serdes_base))
		return PTR_ERR(pcie->serdes_base);
		
	/* Get intx IRQ number */
	pcie->irq_intx = platform_get_irq_byname(pdev, "legacy");
	if (pcie->irq_intx < 0) {
		dev_err(dev, "failed to get intx IRQ %d\n", pcie->irq_intx);
		return pcie->irq_intx;
	}

	irq_set_chained_handler_and_data(pcie->irq_intx,
					 nwl_pcie_leg_handler, pcie);

	return 0;
}

static const struct of_device_id nwl_pcie_of_match[] = {
	{ .compatible = "rcm,nwl-pcie-2.11", },
	{}
};

static int nwl_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nwl_pcie *pcie;
	struct pci_bus *bus;
	struct pci_bus *child;
	struct pci_host_bridge *bridge;
	struct pci_controller *hose = NULL;

	int err;
	resource_size_t iobase = 0;
	LIST_HEAD(res);

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENODEV;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;
	pcie->ecam_value = NWL_ECAM_VALUE_DEFAULT;

	err = nwl_pcie_parse_dt(pcie, pdev);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	err = nwl_pcie_base_init(pcie);
	if (err) {
		dev_err(dev, "Initial setup failed\n");
		return err;
	}

	err = nwl_pcie_bridge_init(pcie);
	if (err) {
		dev_err(dev, "HW Initialization failed\n");
		return err;
	}

	err = devm_of_pci_get_host_bridge_resources(dev, 0, 0xff, &res,
						    &iobase);
	if (err) {
		dev_err(dev, "Getting bridge resources failed\n");
		return err;
	}

	err = devm_request_pci_bus_resources(dev, &res);
	if (err)
		goto error;

	err = nwl_pcie_parse_map_dma_ranges(pcie, dev->of_node);
	if (err)
		goto error;

	err = nwl_pcie_init_irq_domain(pcie);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		goto error;
	}

	// init pcie_controller
	hose = pcibios_alloc_controller(pdev->dev.of_node);
	if (!hose)
		goto error;

	hose->private_data = pcie;
	

	list_splice_init(&res, &bridge->windows);
	bridge->dev.parent = dev;
	bridge->sysdata = hose;
	bridge->busnr = pcie->root_busno;
	bridge->ops = &nwl_pcie_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->swizzle_irq = pci_common_swizzle;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = nwl_pcie_enable_msi(pcie);
		if (err < 0) {
			dev_err(dev, "failed to enable MSI support: %d\n", err);
			goto error;
		}
	}

	err = pci_scan_root_bus_bridge(bridge);
	if (err)
		goto error;

	bus = bridge->bus;

	pci_assign_unassigned_bus_resources(bus);
	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);
	pci_bus_add_devices(bus);

	return 0;

error:
	pci_free_resource_list(&res);
	return err;
}

static struct platform_driver nwl_pcie_driver = {
	.driver = {
		.name = "nwl-pcie",
		.suppress_bind_attrs = true,
		.of_match_table = nwl_pcie_of_match,
	},
	.probe = nwl_pcie_probe,
};
builtin_platform_driver(nwl_pcie_driver);
