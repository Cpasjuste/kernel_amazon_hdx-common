/*
 * WAN over HSIC
 *
 * Copyright 2013-2014 Lab126, Inc. All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <asm/current.h>
#include <mach/gpiomux.h>


static struct platform_device *xhci_hcd_pdev;
static struct device_node *xhci_hcd_node;
static struct kobject *wan_kobj;
static struct wake_lock wan_lock;
static int wan_power;
static int wan_power_gpio;
static int wan_reset_gpio;

static ssize_t wan_power_show(struct kobject *kobj,
		    struct kobj_attribute *attr,
		    char *buf)
{
	if (strcmp(attr->attr.name, "power")) {
		return -1;
	}

	return sprintf(buf, "%d", wan_power);
}

#define WAN_POWER_OFF	0
#define WAN_POWER_ON	1
#define WAN_POWER_RESET 2
static ssize_t wan_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf,
				size_t count)
{
	int var;

	sscanf(buf, "%du", &var);

	if (strcmp(attr->attr.name, "power")) {
		return -1;
	}

	pr_info("%s: Received option %d from %s\n", __func__, var,current->comm);
	wake_lock(&wan_lock);
	switch (var) {
		case WAN_POWER_RESET:
			if (wan_power != WAN_POWER_ON)
				break;

			gpio_direction_output(wan_reset_gpio,1);
			msleep(1000);
			gpio_direction_output(wan_reset_gpio,0);
			device_del(&xhci_hcd_pdev->dev);
			wan_power = WAN_POWER_RESET;
			break;
		case WAN_POWER_ON:
			if (wan_power == WAN_POWER_ON)
				break;
			gpio_direction_output(wan_power_gpio,1);
			msleep(500);
			gpio_direction_output(wan_power_gpio,0);

			/* FIXME: This is arbitray now  */
			msleep(5000);
			of_device_add(xhci_hcd_pdev);
			wan_power = WAN_POWER_ON;
			break;
		case WAN_POWER_OFF:
			if (wan_power != WAN_POWER_ON)
				break;

			/* FIXME: Graceful shutdown. Timing to be changed */
			gpio_direction_output(wan_power_gpio,1);
			msleep(3000);
			gpio_direction_output(wan_power_gpio,0);
			msleep(10000);

			/* FIXME: This should not be necessary, but it is for now */
			gpio_direction_output(wan_reset_gpio,1);
			msleep(1000);
			gpio_direction_output(wan_reset_gpio,0);
			device_del(&xhci_hcd_pdev->dev);
			wan_power = WAN_POWER_OFF;
			break;
		default:
			pr_err ("Input error\n");
	}
	wake_unlock(&wan_lock);
	return count;
}

static struct kobj_attribute wan_power_attribute =
	__ATTR(power, 0644, wan_power_show, wan_power_store);

static struct attribute *wan_attrs[] = {
	&wan_power_attribute.attr,
	NULL,
};

static struct attribute_group wan_attr_group = {
	.name = NULL,
	.attrs = wan_attrs,
};

static struct of_device_id wan_of_match[] = {
	{.compatible = "lab126,wan-hsic", },
	{ },
};

static int wan_request_gpio(void)
{
	int ret;

	if ( (ret = gpio_request(wan_power_gpio, "WAN_Power_On")) != 0 )
		pr_err("%s: can't request wan power gpio %d\n",__func__,ret);
	else if ( (ret = gpio_request(wan_reset_gpio, "WAN_Power_Reset")) != 0 ) {
		pr_err("%s: can't request wan reset gpio %d\n",__func__,ret);
		gpio_free(wan_power_gpio);
	}

	return ret;
}

static int wan_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int retval;

	xhci_hcd_node = of_find_node_by_name(NULL, "qcom,xhci-msm-hsic");
	if(xhci_hcd_node){
		xhci_hcd_pdev = of_find_device_by_node(xhci_hcd_node);
		if (!xhci_hcd_pdev)
		{
			pr_err("%s: No HCD reference\n",__func__);
			return -ENODEV;
		}
	}
	else {
		pr_err("%s: can't find the node\n",__func__);
		return -ENODEV;
	}

	wan_power_gpio = of_get_named_gpio(node, "hsic,wan-pwr-gpio", 0);
	wan_reset_gpio = of_get_named_gpio(node, "hsic,wan-reset-gpio", 0);
	if (!wan_power_gpio || !wan_reset_gpio) {
		pr_err("%s: Invalid GPIOs - %d/%d\n",__func__,wan_power_gpio,wan_reset_gpio);
		return -ENODEV;
	}
	if (wan_request_gpio() != 0)
		return -ENODEV;

	wake_lock_init(&wan_lock, WAKE_LOCK_SUSPEND,"wan_wake_lock");

	wan_kobj = kobject_create_and_add("wan", NULL);

	if (!wan_kobj) {
		pr_err ("Failed to create wan object\n");
		retval = -ENOMEM;
		goto destory_wakelock;
	}
	if ((retval = sysfs_create_group(wan_kobj, &wan_attr_group)) != 0)
		goto release_kobj;

	/* remove the platform device to keep the bus in proper state */
	device_del(&xhci_hcd_pdev->dev);
	pr_info("WAN driver for xHCI/HSIC loaded\n");
	return 0;

release_kobj:
	kobject_put(wan_kobj);
destory_wakelock:
	wake_lock_destroy(&wan_lock);

	gpio_free(wan_power_gpio);
	gpio_free(wan_reset_gpio);

	return retval;
}
static int  wan_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&wan_lock);
	sysfs_remove_group(wan_kobj, &wan_attr_group);
	kobject_put(wan_kobj);
	gpio_free(wan_power_gpio);
	gpio_free(wan_reset_gpio);

	return 0;
}

static struct platform_driver wan_driver = {
	.driver = {
		.name = "wan-hsic",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(wan_of_match),
		},
	.probe   = wan_probe,
	.remove  = wan_remove,
};

static int __init wan_init(void)
{
	if(platform_driver_register(&wan_driver) != 0) {
		pr_err("can't register wan driver\n");
                return -1;
        }

	return 0;
}

static void wan_exit(void)
{
	platform_driver_unregister(&wan_driver);
}

module_init(wan_init);
module_exit(wan_exit);

MODULE_DESCRIPTION("WAN driver for xHCI/HSIC");
MODULE_LICENSE("GPL");
