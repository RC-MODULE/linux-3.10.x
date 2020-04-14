obj-$(CONFIG_FB_RCM_VDU_CORE) += rcm-vdu-core.o
# There is common driver for adv7513,but we are using our for mb115
ifeq ($(CONFIG_1888TX018),y)
obj-$(CONFIG_FB_RCM_VDU_CORE) += rcm-adv7513.o
endif
obj-$(CONFIG_FB_RCM_VDU) += rcm-vdu-fb.o
