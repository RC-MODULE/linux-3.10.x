#
# Makefile for the linux kernel.
#

obj-$(CONFIG_MACH_UEMD)	+= uemd.o
ifdef CONFIG_UEMD_GENERIC_TIME
obj-$(CONFIG_MACH_UEMD)	+= clocksource.o
else
obj-$(CONFIG_MACH_UEMD)	+= timer.o
endif
obj-$(CONFIG_MACH_UEMD) += mdevice.o
obj-$(CONFIG_MODULE_HDMI_BI) += module_hdmi.o
obj-$(CONFIG_MACH_UEMD_NMC3) += nmc3.o
