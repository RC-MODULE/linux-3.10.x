/*
 *  drivers/clocksource/arm_dit_timer.c
 *
 *  Copyright (C) 2011-2015  RC Module
 *
 *  Sergey Mironov <ierton@gmail.com>
 *  Andrew Andrianov <andrew@ncrmnt.org>
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
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/mach/time.h>
#include <linux/slab.h>


#define TIMER0(x)      (0x0  + (x))
#define TIMER1(x)      (0x20 + (x))
#define TIMER_NUM(n, x) ((n * 0x20) + (x))

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


struct dit_timer {
	void __iomem *base;
	void __iomem *timer_ctrl;
	void __iomem *timer_load;
	void __iomem *timer_iclr;
	u32           freq;
	struct clock_event_device evt;
	struct irqaction act;
};

static void __iomem *system_clock __read_mostly;

static u64 notrace dit_sched_read(void)
{
	return ~readl_relaxed(system_clock);
}

static int dit_next_event(unsigned long cycles,
			  struct clock_event_device *evt_dev)
{
	struct dit_timer *timer = container_of(evt_dev,
		struct dit_timer, evt);
	unsigned long ctrl = readl(timer->timer_ctrl);

	writel_relaxed(cycles, timer->timer_load);
	writel_relaxed(ctrl | TIMER_CTRL_ENABLE, timer->timer_ctrl);
	return 0;
}

static void dit_set_mode(enum clock_event_mode mode,
			 struct clock_event_device *evt_dev)
{
	struct dit_timer *timer = container_of(evt_dev,
		struct dit_timer, evt);
	unsigned long ctrl;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel_relaxed((timer->freq / HZ) - 1, timer->timer_load);
		ctrl = TIMER_CTRL_PERIODIC;
		ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_IE | TIMER_CTRL_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
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

	writel(ctrl, timer->timer_ctrl);
}

static irqreturn_t dit_interrupt(int irq, void *dev_id)
{
	struct dit_timer *timer = dev_id;

	writel(1, timer->timer_iclr);
	timer->evt.event_handler(&timer->evt);
	return IRQ_HANDLED;
}

static void __init dit_time_init(struct device_node *node)
{
	struct dit_timer *timer;
	void __iomem     *base;
	u32               freq, mode;
	int               irq;

	/* Just in case */
	BUG_ON(system_clock);

	/*  Timer 1 is used as a clocksource */
	base = of_iomap(node, 0);
	if (!base)
		panic("DIT: Can't remap registers\n");

	if (of_property_read_u32(node, "clock-frequency", &freq))
		panic("DIT: Can't read clock-frequency\n");

	system_clock = base + TIMER1(TIMER_VALUE);
	sched_clock_register(dit_sched_read, 32, freq);
	clocksource_mmio_init(base + TIMER1(TIMER_VALUE), node->name,
		freq, 300, 32, clocksource_mmio_readl_down);

	/* Fire it now ! */
	writel_relaxed(0,           base + TIMER1(TIMER_CONTROL));
	writel_relaxed(0xffffffff,  base + TIMER1(TIMER_LOAD));
	writel_relaxed(0xffffffff,  base + TIMER1(TIMER_VALUE));

	mode =  TIMER_CTRL_ENABLE	|
		TIMER_CTRL_32BIT	|
		TIMER_CTRL_PERIODIC	|
		TIMER_CTRL_P1		|
		TIMER_CTRL_IDIS		|
		0;
	writel_relaxed(mode, base + TIMER1(TIMER_CONTROL));

	/* Timer 0 is used for clockevent system */
	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("DIT: Can't parse IRQ\n");

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer)
		panic("DIT: Can't allocate timer struct\n");

	/* Good, now let's fill our struct with sensible data */
	timer->base = base;
	timer->freq = freq;

	timer->timer_ctrl   = base + TIMER0(TIMER_CONTROL);
	timer->timer_load   = base + TIMER0(TIMER_LOAD);
	timer->timer_iclr   = base + TIMER0(TIMER_ICLEAR);

	timer->evt.name           = node->name;
	timer->evt.rating         = 300;
	timer->evt.features       =	CLOCK_EVT_FEAT_ONESHOT |
					CLOCK_EVT_FEAT_PERIODIC;
	timer->evt.set_mode       = dit_set_mode;
	timer->evt.set_next_event = dit_next_event;
	timer->evt.cpumask        = cpumask_of(0);

	timer->act.name    = node->name;
	timer->act.flags   = IRQF_TIMER | IRQF_SHARED;
	timer->act.dev_id  = timer;
	timer->act.handler = dit_interrupt;

	if (setup_irq(irq, &timer->act))
		panic("DIT: Can't set up timer IRQ\n");

	clockevents_config_and_register(&timer->evt, freq, 0xf, 0xffffffff);
}

CLOCKSOURCE_OF_DECLARE(arm_dit, "arm,arm-dit-timer",
			dit_time_init);
