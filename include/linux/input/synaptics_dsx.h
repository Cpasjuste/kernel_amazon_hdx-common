/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#if !defined(CONFIG_ARCH_MSM8974_THOR) && !defined(CONFIG_ARCH_MSM8974_APOLLO) && !defined(CONFIG_ARCH_APQ8084_LOKI) && !defined(CONFIG_ARCH_APQ8084_SATURN)
/*
 * struct synaptics_rmi4_capacitance_button_map - 0d button map
 * @nbuttons: number of buttons
 * @map: button map
 */
struct synaptics_rmi4_capacitance_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

/*
 * struct synaptics_rmi4_platform_data - rmi4 platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @i2c_pull_up: pull up i2c bus with regulator
 * @power_down_enable: enable complete regulator shutdown in suspend
 * @irq_gpio: attention interrupt gpio
 * @irq_flags: flags used by the irq
 * @reset_flags: flags used by reset line
 * @reset_gpio: reset gpio
 * @panel_x: panel maximum values on the x
 * @panel_y: panel maximum values on the y
 * @disp_maxx: display panel maximum values on the x
 * @disp_maxy: display panel maximum values on the y
 * @disp_minx: display panel minimum values on the x
 * @disp_miny: display panel minimum values on the y
 * @panel_maxx: touch panel maximum values on the x
 * @panel_maxy: touch panel maximum values on the y
 * @panel_minx: touch panel minimum values on the x
 * @panel_miny: touch panel minimum values on the y
 * @reset_delay: reset delay
 * @gpio_config: pointer to gpio configuration function
 * @capacitance_button_map: pointer to 0d button map
 */
struct synaptics_rmi4_platform_data {
	bool x_flip;
	bool y_flip;
	bool regulator_en;
	bool i2c_pull_up;
	bool power_down_enable;
	bool disable_gpios;
	bool do_lockdown;
	unsigned irq_gpio;
	u32 irq_flags;
	u32 reset_flags;
	unsigned reset_gpio;
	unsigned panel_minx;
	unsigned panel_miny;
	unsigned panel_maxx;
	unsigned panel_maxy;
	unsigned disp_minx;
	unsigned disp_miny;
	unsigned disp_maxx;
	unsigned disp_maxy;
	unsigned reset_delay;
	const char *fw_image_name;
	int (*gpio_config)(unsigned gpio, bool configure);
	struct synaptics_rmi4_capacitance_button_map *capacitance_button_map;
};

#elif defined(CONFIG_ARCH_MSM8974_THOR) || defined(CONFIG_ARCH_MSM8974_APOLLO)
/*
 * struct synaptics_dsx_cap_button_map - 0d button map
 * @nbuttons: number of 0d buttons
 * @map: pointer to array of button types
 */
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

/*
 * struct synaptics_dsx_platform_data - dsx platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @regulator_en: regulator enable flag
 * @irq_gpio: attention interrupt gpio
 * @irq_flags: flags used by the irq
 * @reset_gpio: reset gpio
 * @gpio_config: pointer to gpio configuration function
 * @cap_button_map: pointer to 0d button map
 */
struct synaptics_dsx_platform_data {
	bool x_flip;
	bool y_flip;
	bool regulator_en;
	bool i2c_pull_up;
	unsigned irq_gpio;
	u32 irq_flags;
	u32 reset_flags;
	unsigned reset_gpio;
	unsigned reset_gpio_flags;
	bool use_id_gpio;
	unsigned id0_gpio;
	unsigned id1_gpio;
	unsigned panel_x;
	unsigned panel_y;
	const char *fw_image_name;
	int (*gpio_config)(unsigned gpio, bool configure);
	struct synaptics_dsx_cap_button_map *cap_button_map;
};

#elif defined(CONFIG_ARCH_APQ8084_LOKI) || defined(CONFIG_ARCH_APQ8084_SATURN)
#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#define SPI_DRIVER_NAME "synaptics_dsx_spi"
/*
 * struct synaptics_dsx_cap_button_map - 0d button map
 * @nbuttons: number of 0d buttons
 * @map: pointer to array of button types
 */
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

/*
 * struct synaptics_dsx_platform_data - dsx platform data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @regulator_en: regulator enable flag
 * @irq_gpio: attention interrupt gpio
 * @irq_flags: flags used by the irq
 * @reset_gpio: reset gpio
 * @gpio_config: pointer to gpio configuration function
 * @cap_button_map: pointer to 0d button map
 */
struct synaptics_dsx_board_data {
        bool x_flip;
        bool y_flip;
        bool swap_axes;
        bool regulator_en;
        bool i2c_pull_up;
        unsigned irq_gpio;
	 int irq_on_state;
        int power_gpio;
        int power_on_state;
        u32 irq_flags;
        u32 reset_flags;
        unsigned reset_gpio;
        unsigned reset_on_state;
        unsigned reset_gpio_flags;
        bool use_id_gpio;
        int id0_gpio;
        int id1_gpio;
        unsigned panel_x;
        unsigned panel_y;
        unsigned int power_delay_ms;
        unsigned int reset_delay_ms;
        unsigned int reset_active_ms;
        unsigned int byte_delay_us;
        unsigned int block_delay_us;
        unsigned char *regulator_name;
        const char *fw_image_name;
        int (*gpio_config)(int gpio, bool configure, int dir, int state);
	struct synaptics_dsx_cap_button_map *cap_button_map;
};
#endif /* For apollo and thor Or loki and saturn */

#endif
