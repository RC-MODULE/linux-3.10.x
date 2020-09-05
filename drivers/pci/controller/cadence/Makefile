# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_PCIE_CADENCE) += pcie-cadence.o
obj-$(CONFIG_PCIE_CADENCE_HOST) += pcie-cadence-host.o
obj-$(CONFIG_PCIE_CADENCE_EP) += pcie-cadence-ep.o
obj-$(CONFIG_PCIE_CADENCE_PLAT) += pcie-cadence-plat.o
obj-$(CONFIG_RCM_PCIE_CADENCE) += pcie-cadence-rcm.o
obj-$(CONFIG_RCM_PCIE_CADENCE_HOST) += pcie-cadence-rcm-host.o
ifeq ($(CONFIG_TARGET_1879VM8YA),y)
obj-$(CONFIG_RCM_PCIE_CADENCE_EP) += pcie-cadence-rcm-ep.o pcie-cadence-rcm-ep-nm6408.o
endif
ifeq ($(CONFIG_TARGET_1888BC048),y)
obj-$(CONFIG_RCM_PCIE_CADENCE_EP) += pcie-cadence-rcm-ep.o pcie-cadence-rcm-ep-basis.o
endif
obj-$(CONFIG_RCM_PCIE_CADENCE_PLAT) += pcie-cadence-rcm-plat.o
