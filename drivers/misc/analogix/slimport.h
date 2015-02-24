/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#include <linux/gpio.h>
#include <linux/delay.h>
#define LOG_TAG "anx3618"

//#include <mach/mt_gpio.h>

//#define SSC_1
//#define HDCP_EN


#define AUX_ERR  1
#define AUX_OK   0


void Colorado2_work_func(struct work_struct * work);
unsigned char sp_write_reg(unsigned char dev, unsigned char dev_offset, unsigned char d);
unsigned char sp_read_reg(unsigned char dev, unsigned char dev_offset, unsigned char *d);
unsigned char __i2c_read_byte(unsigned char dev, unsigned char dev_offset);

#define cable_detected()  gpio_get_value(SLIMPORT_CABLE_DETECT)


#define ENABLE_READ_EDID

#define idata
#define code
#define TX_P0 0x70
#define TX_P1 0x7A
#define TX_P2 0x72

#define RX_P0 0x7e
#define RX_P1 0x80

#define debug_puts(fmt) pr_debug(fmt)
#define debug_printf(fmt, arg...) pr_debug("%s: " fmt, __func__, ##arg)

#define delay_ms(msec) mdelay(msec)

#define __read_reg(dev, dev_offset) __i2c_read_byte(dev, dev_offset)

void sp_tx_hardware_poweron(void);
void sp_tx_hardware_powerdown(void);
int slimport_read_edid_block(int block, uint8_t *edid_buf);
unsigned char slimport_get_link_bw(void);
unsigned char sp_get_ds_cable_type(void);
#ifdef CONFIG_SLIMPORT_FAST_CHARGE
enum CHARGING_STATUS sp_get_ds_charge_type(void);
#endif
bool slimport_is_connected(void);
int get_slimport_hdcp_status(void);

#ifndef XTAL_CLK_DEF
#define XTAL_CLK_DEF XTAL_27M
#endif

#define XTAL_CLK_M10 pXTAL_data[XTAL_CLK_DEF].xtal_clk_m10
#define XTAL_CLK pXTAL_data[XTAL_CLK_DEF].xtal_clk


#endif
