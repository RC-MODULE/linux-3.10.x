/*
 * EasyNMC Dummy Core Driver
 * (c) RC Module 2014
 *
 * This driver provides a simple interface to userspace apps and 
 * is designed to be as simple as possible. 
 * Cores are registered with this framework by calling 
 *
 * easynmc_register_core() from respective platform drivers. 
 * 
 * A  
 *
 * 
 * License terms: GNU General Public License (GPL) version 2
 * Author: Andrew Andrianov <andrew@ncrmnt.org>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/easynmc.h>

#define DRVNAME "easynmc-coretest"
#define DUMMYMEMSIZE 4096

static char imem[DUMMYMEMSIZE];

void dummy_reset(struct nmc_core *self) {
	printk("nmcdummy: Resetting dummy core\n");
}

void dummy_interrupt(struct nmc_core *self, enum nmc_irq n) {
	printk("nmcdummy: Sending IRQ: %d\n", n);
}

static struct nmc_core core = { 
	.name           = "DummyCore",
	.type           = "noop",
	.reset          = dummy_reset,
	.send_interrupt = dummy_interrupt,
	.imem_virt      = imem,
	.imem_size      = DUMMYMEMSIZE,
};


static int __init easynmc_init(void)
{
	int err;
	err = easynmc_register_core(&core);
	printk("nmcdummy: registered with code %d\n", err);
	return 0;
}

static void __exit easynmc_exit(void)
{
	easynmc_deregister_core(&core);
}


MODULE_AUTHOR("Andrew Andrianov <andrew@ncrmnt.org>");
MODULE_DESCRIPTION("EasyNMC Dummy Core");
MODULE_LICENSE("GPL v2");

module_init(easynmc_init);
module_exit(easynmc_exit);
