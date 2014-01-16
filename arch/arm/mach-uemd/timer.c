/*
 *  arch/arm/mach-uemd/time.c
 *
 *  Copyright (C) 2010  RC Module
 *
 *  Sergey Mironov <ierton@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach/time.h>

#include "uemd.h"

/* USE timer0 */
#define REG(x) (UEMD_TIMER0_VIRT_BASE + (x))

#define TIMER_LOAD                0x00
#define TIMER_VALUE               0x04
#define TIMER_CONTROL             0x08
#define TIMER_ICLEAR              0x0C
#define TIMER_RIS                 0x10
#define TIMER_MIS                 0x14
#define TIMER_BGLOAD              0x18

#define TIMER_IRQ                 UEMD_TIMER0_IRQ

#define TIMER_CONTROL_ENABLE      (1<<7)
#define TIMER_CONTROL_PERIODIC    (1<<6)
#define TIMER_CONTROL_FREERUNNING (0<<6)
#define TIMER_CONTROL_IEN         (1<<5)
#define TIMER_CONTROL_IDIS        (0<<5)
#define TIMER_CONTROL_P1          (0<<2) /* Prescaler 1 */
#define TIMER_CONTROL_P16         (1<<2) /* Prescaler 16 */
#define TIMER_CONTROL_P256        (2<<2) /* Prescaler 256 */
#define TIMER_CONTROL_SZ32        (1<<1)
#define TIMER_CONTROL_WRAPPING    (1<<0)

#define TIMER_INITIAL_VALUE       ((UEMD_FREQ_HZ/16)/HZ)

#define TIMER_MODE (\
	TIMER_CONTROL_ENABLE | \
	TIMER_CONTROL_SZ32 | \
	TIMER_CONTROL_PERIODIC | \
	TIMER_CONTROL_P16 | \
	TIMER_CONTROL_IEN | \
	0) 

static irqreturn_t uemd_timer_interrupt(int irq, void *dev_id)
{
	uemd_raw_writel(0, REG(TIMER_ICLEAR));
	timer_tick();
	return IRQ_HANDLED;
}

static struct irqaction uemd_timer_irq = {
	.name		= "UEMD Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= uemd_timer_interrupt,
};

static void __init uemd_timer_init (void)
{
	int ret;

	printk(KERN_WARNING "Timer: using old-style timer implementation\n");

	uemd_raw_writel(0, REG(TIMER_CONTROL));

	ret = setup_irq(TIMER_IRQ, &uemd_timer_irq);
	if(ret != 0)
		printk(KERN_WARNING "Timer: unable to register IRQ handler (%d)\n", ret);

	printk(KERN_INFO "Timer: using initial value %d\n", TIMER_INITIAL_VALUE);

	uemd_raw_writel(TIMER_INITIAL_VALUE, REG(TIMER_LOAD));
	uemd_raw_writel(TIMER_MODE, REG(TIMER_CONTROL));
}

struct sys_timer uemd_timer = {
	.init		= &uemd_timer_init,
};

