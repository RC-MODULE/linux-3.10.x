/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/poll.h>

#include "pseudo_spi_serial.h"

extern struct tty_driver* pseudo_spi_serial_tty_driver;

int pseudo_spi_serial_do_write(struct pseudo_spi_serial_data* data,
							   const unsigned char *buffer, int count);

static int pseudo_spi_console_setup(struct console *co, char *options)
{
	return 0;
}

static void pseudo_spi_console_write(struct console *co, const char *buf, unsigned count)
{
	struct pseudo_spi_serial_data* data = co->data;
	int retval = 0;

	while (count)
	{
		unsigned int i;
		unsigned int lf;
		
		/* search for LF so we can insert CR if necessary */
		for (i = 0, lf = 0 ; i < count ; i++)
		{
			if (*(buf + i) == 10)
			{
				lf = 1;
				i++;
				break;
			}
		}
		/* pass on to the driver specific version of this function if
		   it is available */
		retval = pseudo_spi_serial_do_write(data, buf, i);

		if (retval <= 0)
		{
			break;
		}

		buf += retval;
		count -= retval;

		if (retval == i)
		{
			if (lf)
			{
				/* append CR after LF */
				unsigned char cr = 13;

				pseudo_spi_serial_do_write(data, &cr, 1);
			}
		}
	}
}

static struct tty_driver* pseudo_spi_console_device(struct console *co, int *index)
{
	*index = co->index;
	return pseudo_spi_serial_tty_driver;
}

static struct console pseudo_spi_console =
 {
	.name =		"ttySPI",
	.write =	pseudo_spi_console_write,
	.device =	pseudo_spi_console_device,
	.setup =	pseudo_spi_console_setup,
	.flags =	CON_PRINTBUFFER,
	.index =	-1,
};

void pseudo_spi_console_init(struct pseudo_spi_serial_data* data)
{
	pseudo_spi_console.data = data;
	register_console(&pseudo_spi_console);
}

void pseudo_spi_console_exit(void)
{
	unregister_console(&pseudo_spi_console);
}

