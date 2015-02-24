/*
 * bwan.h  --  WAN hardware control driver header file
 *
 * Copyright (c) 2005-2014 Lab126, Inc./Amazon Technologies Inc.
 *   All rights reserved.
 *
 */

#ifndef _BWAN_H
#define _BWAN_H

#define WAN_USB_RESUME_GPIO    15

struct bwan_platform_data {
	void (*init)(void);
	int wan_on_gpio;
	int wan_shutdown_gpio;
	int usb_en_gpio;
	int fw_rdy_gpio;
	int sim_present_gpio;
};

#endif
