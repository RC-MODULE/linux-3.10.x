/*
 * udbg for PL011 compatible serial ports
 *
 * Copyright (C) 2018 AstroSoft
 * 		Alexey Spirkov <alexeis@astrosft.ru>
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */
#include <linux/types.h>
#include <asm/udbg.h>
#include <asm/io.h>
#include <platforms/44x/44x.h>
#include <asm/reg_a2.h>


#define     PL011_UARTFR          0x018
#define     PL011_UARTRIS         0x03C
#define     PL011_UARTDR          0x000
#define 	PL011_RXFE_i     	  4
#define 	PL011_TXFE_i     	  7
#define 	PL011_TXRIS_i    	  5
#define 	PL011_RXFF_i     	  6
#define 	PL011_TXFF_i     	  5
#define 	PL011_RXRIS_i    	  4


static volatile uint32_t _ioread32(uint32_t const base_addr)
{
    return le32_to_cpu(*((volatile uint32_t*)(base_addr)));
}

static  void _iowrite32(uint32_t const value, uint32_t const base_addr)
{
    *((volatile uint32_t*)(base_addr)) = cpu_to_le32(value);
}

static int tx_fifo_ready(uint32_t base_addr)
{
  if (_ioread32(base_addr + PL011_UARTFR) & (1 << PL011_TXFE_i))
  return 0;

  if (_ioread32(base_addr + PL011_UARTRIS) & (1 << PL011_TXRIS_i))
  return 0;

  do{
    if (!(_ioread32(base_addr + PL011_UARTFR) & (1 << PL011_TXFF_i)))
    return 0; //FIFO is not full, i.e. at least 1 byte can be pushed
  }
  while(1);

  return -1;
}

static u32 pl011_get_fifo_state(void)
{
	return _ioread32(PPC44x_EARLY_DEBUG_VIRTADDR + PL011_UARTFR);
}

static void pl011_uart_flush(void)
{
	/* wait for idle */
	while ((pl011_get_fifo_state() & (1 << PL011_TXFE_i)) == 0)
		cpu_relax();
}

static void pl011_uart_putc(char c)
{
	if (c == '\n')
		pl011_uart_putc('\r');
	pl011_uart_flush();

	tx_fifo_ready(PPC44x_EARLY_DEBUG_VIRTADDR);
	_iowrite32((char)c, PPC44x_EARLY_DEBUG_VIRTADDR + PL011_UARTDR);
}

static int pl011_uart_getc_poll(void)
{

	if (pl011_get_fifo_state() & (1 << PL011_RXFE_i))
		return _ioread32(PPC44x_EARLY_DEBUG_VIRTADDR + PL011_UARTDR) & 0xff;

	return -1;
}

static int pl011_uart_getc(void)
{
	/* wait for char */
	while (!(pl011_get_fifo_state() & (1 << PL011_RXFE_i)))
		cpu_relax();
	return _ioread32(PPC44x_EARLY_DEBUG_VIRTADDR + PL011_UARTDR) & 0xff;
}

void __init udbg_init_44x_pl011(void)
{
	udbg_putc = pl011_uart_putc;
	udbg_flush = pl011_uart_flush;
	udbg_getc = pl011_uart_getc;
	udbg_getc_poll = pl011_uart_getc_poll;
}
