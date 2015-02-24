/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _LINUX_SLIMPORT_H
#define _LINUX_SLIMPORT_H

#include <linux/err.h>

#ifdef CONFIG_SLIMPORT_ANX3618
int slimport_read_edid_block(int block, uint8_t *edid_buf);
void slimport_set_edid_complete(void);
bool slimport_link_ready(void);
static inline bool slimport_support_enabled(void)
{
	return true;
}
#else
static inline int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	return -ENOSYS;
}
static inline void slimport_set_edid_complete(void)
{
}
static inline bool slimport_link_ready(void)
{
	return true;
}

static inline bool slimport_support_enabled(void)
{
	return false;
}

#endif
#endif
