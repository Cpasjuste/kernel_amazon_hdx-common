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

#ifndef __AMZN_RAM_CONSOLE_H
#define __AMZN_RAM_CONSOLE_H

#define AMZN_RAM_CONSOLE_ADDR		(UL(0x50000000))
/*
 * The pstore rounds down the separate sizes and the total size to be power of
 * two, so we can't simply add the sizes together to get the total size.
 */
#define AMZN_RAM_TOTAL_SIZE		(SZ_2M)
#define AMZN_RAM_OOPS_RECORD_COUNT	(63)
#define AMZN_RAM_OOPS_RECORD_SIZE	(SZ_16K)
#define AMZN_RAM_FTRACE_SIZE		(SZ_16K)
#define AMZN_RAM_CONSOLE_SIZE		(SZ_1M)

#ifdef CONFIG_AMZN_RAM_CONSOLE
extern int amzn_ram_console_init(void);
#else
static inline int amzn_ram_console_init(void)
{
	return 0;
}
#endif /* CONFIG_AMZN_RAM_CONSOLE */

#endif /* __AMZN_RAM_CONSOLE_H */
