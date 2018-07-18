/*
 * Copyright © 2018 Alexey Spirkov AstroSoft
 *
 * Based on earlier code:
 *   Copyright (C) Paul Mackerras 1997.
 *
 *   Matt Porter <mporter@kernel.crashing.org>
 *   Copyright 2002-2005 MontaVista Software Inc.
 *
 *   Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>
 *   Copyright (c) 2003, 2004 Zultys Technologies
 *
 *    Copyright 2007 David Gibson, IBM Corporation.
 *    Copyright 2010 Ben. Herrenschmidt, IBM Corporation.
 *    Copyright © 2011 David Kleikamp IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "elf.h"
#include "string.h"
#include "stdio.h"
#include "page.h"
#include "ops.h"
#include "reg.h"
#include "io.h"
#include "dcr.h"
#include "4xx.h"
#include "44x.h"
#include "libfdt.h"

BSS_STACK(4096);

#define MAX_RANKS	0x4
#define DDR3_MR0CF	0x80050040U

static unsigned long long mpw7705_memsize;
static unsigned long long mpw7705_detect_memsize(void)
{
	u32 reg;
	unsigned i;
	unsigned long long memsize = 0;

	for(i = 0; i < MAX_RANKS; i++){
		reg = mfdcrx(DDR3_MR0CF + i);

		if (!(reg & 1))
			continue;

		reg &= 0x0000f000;
		reg >>= 12;
		memsize += (0x800000ULL << reg);
	}

	return memsize;
}

static void mpw7705_fixups(void)
{
}

#define SPRN_PIR	0x11E	/* Processor Identification Register */
void platform_init(void)
{
	unsigned long end_of_ram, avail_ram;
	u32 pir_reg;
	int node, size;
	const u32 *timebase;

	mpw7705_memsize = mpw7705_detect_memsize();
	if (mpw7705_memsize >> 32)
		end_of_ram = ~0UL;
	else
		end_of_ram = mpw7705_memsize;
	avail_ram = end_of_ram - (unsigned long)_end;

	simple_alloc_init(_end, avail_ram, 128, 64);
	platform_ops.fixups = mpw7705_fixups;
	platform_ops.exit = ibm44x_dbcr_reset;
	pir_reg = mfspr(SPRN_PIR);

	/* Make sure FDT blob is sane */
	if (fdt_check_header(_dtb_start) != 0)
		fatal("Invalid device tree blob\n");

	node = fdt_node_offset_by_prop_value(_dtb_start, -1, "device_type",
	                                     "cpu", sizeof("cpu"));
	if (!node)
		fatal("Cannot find cpu node\n");
	timebase = fdt_getprop(_dtb_start, node, "timebase-frequency", &size);
	if (timebase && (size == 4))
		timebase_period_ns = 1000000000 / *timebase;

	fdt_set_boot_cpuid_phys(_dtb_start, pir_reg);
	fdt_init(_dtb_start);

	serial_console_init();
}
