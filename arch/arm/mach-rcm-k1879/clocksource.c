/*
 *  arch/arm/mach-uemd/clocksource.c
 *
 *  Copyright (C) 2011  RC Module
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
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/sched_clock.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach/time.h>

#include <mach/platform.h>

/* USE timer0 */
#define TIMER0(x) ((void __iomem*) (RCM_K1879_TIMER0_VIRT_BASE + (x)))
#define TIMER1(x) ((void __iomem*) (RCM_K1879_TIMER0_VIRT_BASE + 0x20 + (x)))

#define TIMER_LOAD                0x00
#define TIMER_VALUE               0x04
#define TIMER_CONTROL             0x08
#define TIMER_CTRL                TIMER_CONTROL
#define TIMER_ICLEAR              0x0C
#define TIMER_RIS                 0x10
#define TIMER_MIS                 0x14
#define TIMER_BGLOAD              0x18

#define TIMER_CTRL_ENABLE      (1<<7)
#define TIMER_CTRL_DISABLE     (0<<7)

/* 
 * TIMER_CONTROL_PERIODIC:
 * The counter generates an interrupt at a constant interval, reloading the
 * original value after wrapping past zero.
 */
#define TIMER_CTRL_PERIODIC    (1<<6)
/* 
 * TIMER_CONTROL_FREERUNNING:
 * The counter wraps after reaching its zero value, and continues to count down
 * from the maximum value. This is the default mode.
 */
#define TIMER_CTRL_FREERUNNING (0<<6)
/* Use interrupts */
#define TIMER_CTRL_IE             (1<<5)
/* No interrupts */
#define TIMER_CTRL_IDIS        (0<<5)
/* Prescalers */
#define TIMER_CTRL_P1          (0<<2) /* Prescaler 1 */
#define TIMER_CTRL_P16         (1<<2) /* Prescaler 16 */
#define TIMER_CTRL_P256        (2<<2) /* Prescaler 256 */
/* Counter register size is 32 Bit long */
#define TIMER_CTRL_32BIT           (1<<1)
/* 
 * TIMER_CONTROL_ONESHOT:
 * The counter generates an interrupt once. When the counter reaches zero, it
 * halts until reprogrammed by the user. This can be achieved by either clearing
 * the One Shot Count bit in the control register (in which case the count will
 * proceed according to the selection of Free-running or Periodic mode), or by
 * writing a new value to the Load Value register.
 */
#define TIMER_CTRL_ONESHOT        (1<<0)
#define TIMER_CTRL_WRAPPING       (0<<0)


static cycle_t k1879_timer1_read(struct clocksource *cs)
{
	return ~readl(TIMER1(TIMER_VALUE));
}

static struct clocksource k1879_clocksource = {
	.name		= "k1879_timer1",
	.rating		= 200,
	.read		= k1879_timer1_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static u64 notrace k1879_read_sched_clock(void)
{
	return 0xffffffffu - (~readl(TIMER1(TIMER_VALUE)));
}

static void k1879_clocksource_init(void)
{
	int ret;
	int mode;

	sched_clock_register(k1879_read_sched_clock, 32, RCM_K1879_FREQ_HZ);

	writel(0, TIMER1(TIMER_CONTROL));
	writel(0xffffffff, TIMER1(TIMER_LOAD));
	writel(0xffffffff, TIMER1(TIMER_VALUE));

	mode = 	TIMER_CTRL_ENABLE 		|
		TIMER_CTRL_32BIT 		| 
		TIMER_CTRL_PERIODIC 		|
		TIMER_CTRL_P1 			|
		TIMER_CTRL_IDIS 		|
		0;
	writel(mode, TIMER1(TIMER_CONTROL));

	k1879_clocksource.mult =
		clocksource_hz2mult(RCM_K1879_FREQ_HZ, k1879_clocksource.shift);
	printk(KERN_INFO "Clocksource: rate %d mult %d shift %d\n", 
		RCM_K1879_FREQ_HZ, k1879_clocksource.mult, k1879_clocksource.shift);
	ret = clocksource_register(&k1879_clocksource);
	if(ret != 0) {
		printk(KERN_EMERG "Failed to register clocksource! Error %d\n", ret);
	}
}

static int k1879_timer0_next_event(unsigned long cycles, struct clock_event_device *evt)
{
	unsigned long ctrl = readl( TIMER0(TIMER_CTRL) );
	writel(cycles, TIMER0(TIMER_LOAD));
	writel(ctrl | TIMER_CTRL_ENABLE, TIMER0(TIMER_CTRL));
	return 0;
}

static void k1879_timer0_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	unsigned long ctrl;
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk("set mode to periodic\n");
		writel((RCM_K1879_FREQ_HZ/HZ)-1, TIMER0(TIMER_LOAD));

		ctrl = TIMER_CTRL_PERIODIC;
		ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_IE | TIMER_CTRL_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk("set mode to oneshot\n");
		/* period set, and timer enabled in 'next_event' hook */
		ctrl = TIMER_CTRL_ONESHOT;
		ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_IE;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
	default:
		ctrl = 0;
		break;
	}

	writel(ctrl, TIMER0(TIMER_CONTROL));
}

static struct clock_event_device k1879_clockevent = {
	.name		= "k1879_timer0",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= k1879_timer0_set_mode,
	.set_next_event	= k1879_timer0_next_event,
	.rating 	= 300,
};

static irqreturn_t k1879_timer0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &k1879_clockevent;

	writel(1, TIMER0(TIMER_ICLEAR));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction k1879_timer0_irq = {
	.name		= "k1879_timer0_irq",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= k1879_timer0_interrupt,
};

static void k1879_clockevent_init(void)
{
	int ret;

	k1879_clockevent.irq = RCM_K1879_TIMER0_IRQ;
	k1879_clockevent.mult =
		div_sc(RCM_K1879_FREQ_HZ, NSEC_PER_SEC, k1879_clockevent.shift);
	k1879_clockevent.cpumask = cpumask_of(0);
	k1879_clockevent.max_delta_ns = clockevent_delta2ns(0xffffffff, &k1879_clockevent);
	k1879_clockevent.min_delta_ns = clockevent_delta2ns(0xf, &k1879_clockevent);

	printk(KERN_INFO "Clockevent: rate %d mult %d shift %d\n", 
		RCM_K1879_FREQ_HZ, k1879_clockevent.mult, k1879_clockevent.shift);

	writel(0, TIMER0(TIMER_CONTROL));

	ret = setup_irq(RCM_K1879_TIMER0_IRQ, &k1879_timer0_irq);
	if(ret != 0) {
		printk(KERN_EMERG "Clockevent: unable to register IRQ %d handler (%d)\n", 
		       RCM_K1879_TIMER0_IRQ, ret);
	}

	clockevents_config_and_register(&k1879_clockevent, RCM_K1879_FREQ_HZ,
					0xf, 0xfffffffe);
}

/*
 * This sets up the system timers, clock source and clock event.
 */
void __init k1879_timer_init(void)
{
	printk("UEMD: Firing up timer system\n");
	k1879_clocksource_init();
	k1879_clockevent_init();
}

