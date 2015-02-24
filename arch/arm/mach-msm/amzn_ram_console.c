/*
 *
 * Copyright (C) 2012 Amazon Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/pstore_ram.h>
#include "amzn_ram_console.h"

static struct ramoops_platform_data ramoops_data = {
        .mem_size	= AMZN_RAM_TOTAL_SIZE,
        .mem_address	= AMZN_RAM_CONSOLE_ADDR,
        .record_size	= AMZN_RAM_OOPS_RECORD_SIZE,
        .console_size	= AMZN_RAM_CONSOLE_SIZE,
        .ftrace_size	= AMZN_RAM_FTRACE_SIZE,
        .dump_oops	= 1,
        .ecc_info	= {
		.ecc_size	= 16,
        },
};

static struct platform_device ramoops_dev = {
        .name = "ramoops",
        .dev = {
                .platform_data = &ramoops_data,
        },
};

/* device_initcall to register ramconsole device */
static int __init amzn_ram_console_register(void)
{
	int ret;

	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		pr_err("%s: unable to register ram console device:"
			"start=0x%08x, size=0x%08x, ret=%d\n",
			__func__, (u32)ramoops_data.mem_address,
			(u32)ramoops_data.mem_size, ret);
		return ret;
	}
	memblock_add(ramoops_data.mem_address, ramoops_data.mem_size);

	return ret;
}
device_initcall(amzn_ram_console_register);
