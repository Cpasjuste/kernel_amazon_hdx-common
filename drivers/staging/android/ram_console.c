/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/pstore_ram.h>
#include <linux/compiler.h>
#include "ram_console.h"

static int last_kmsg_proc_show(struct seq_file *m, void *v)
{
	char *p_last_kmsg = get_last_kmsg_buf();
	if (p_last_kmsg) {
		if (strlen(p_last_kmsg) > 0)
			seq_puts(m, p_last_kmsg);
		else
			seq_puts(m,
				"There is no lask kmsg due to no ramdump from last reboot.\n");
	}
	return 0;
}

static int last_kmsg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, last_kmsg_proc_show, NULL);
}

static const struct file_operations ram_console_file_ops = {
	.open = last_kmsg_proc_open,
	.read = seq_read,
	.release = single_release,
};

static int __init ram_console_late_init(void)
{
	proc_create("last_kmsg", 0, NULL, &ram_console_file_ops);

	return 0;
}

module_init(ram_console_late_init);
