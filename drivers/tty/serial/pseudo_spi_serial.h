/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2019 Alexey Spirkov <dev@alsp.net>
 */

#include <linux/tty.h>
#include <linux/kfifo.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>
#include <linux/dma-mapping.h>

#define PSEUDO_SPI_SERIAL_TTY_MAJOR		240
#define PSEUDO_SPI_SERIAL_TTY_MINORS	1

struct pseudo_spi_serial_data
{
	struct spi_device*		spi;
	struct tty_struct*		tty;
	struct tty_port 		tty_port;
	struct device*			tty_dev;
	spinlock_t 				lock;
	struct kfifo 			tx_fifo;
	unsigned char* 			tx_buffer;
	dma_addr_t				tx_dma;
	unsigned char* 			rx_buffer;
	dma_addr_t				rx_dma;
	size_t					size_buffer; 
	struct timer_list		timer;
	struct tasklet_struct 	io_tasklet;
	struct spi_message		spi_msg;
	struct spi_transfer		spi_xfer;
};

