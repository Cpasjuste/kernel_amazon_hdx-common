/*
 * lab126_sensor_sysfs.c
 * sysfs node to power on and off camera sensors (Only root user access)
 *
 * Copyright 2014 Amazon Lab126, Inc.
 *
 * Author: Karthik Poduval <kpoduval@lab126.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include "msm_sensor.h"


#define to_dev(obj) container_of(obj, struct device, kobj)

static char state[10]= "off";
struct msm_sensor_ctrl_t;

static ssize_t power_func_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t n)
{
	//struct device *dev = to_dev(kobj);
	struct msm_sensor_ctrl_t* s_ctrl = (struct msm_sensor_ctrl_t*)dev_get_drvdata(dev);
	char cmd[10];
	int rc;

	printk("s_ctrl[%p]\n", s_ctrl);

	sscanf(buf,"%s",cmd);
	if(strcmp(cmd,"on")==0)
	{
		printk("CMD to turn ON power\n");
		if(s_ctrl->func_tbl->sensor_power_up)
		{
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if(rc)
			{
				printk("Could not power UP sensor rc=%d\n", rc);
				return rc;
			}
			else
				strcpy(state,"on");

		}
		else
			printk("No power up sequence defined for this sensor\n");

	}
	if(strcmp(cmd,"off")==0)
	{
		printk("CMD to turn OFF power\n");
		if(s_ctrl->func_tbl->sensor_power_down)
		{
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if(rc)
			{
				printk("Could not power DOWN sensor rc=%d\n", rc);
				return rc;
			}
			else
				strcpy(state,"off");

		}
		else
			printk("No power down sequence defined for this sensor\n");
	}

	return n;
}

static ssize_t power_func_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", state);

}


static struct device_attribute power = {
	.attr = {.name = "sensor_power",.mode = 0644},
	.store = power_func_store,
	.show = power_func_show,
};


int lab126_sensor_sysfs_file_create(struct device *dev)
{
	int ret;
	printk("[%s]\n", __func__);

	ret = device_create_file(dev, &power);

	return ret;
}
