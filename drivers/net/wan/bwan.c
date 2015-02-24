/*
 * bwan.c  --  WAN hardware control driver
 *
 * Copyright (c) 2005-2014 Lab126, Inc/Amazon Technologies Inc.
 *   All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/bwan.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <mach/gpiomux.h>


#define HIGH				1
#define LOW				0

#define INTERRUPT_DEBOUNCE_TIME		30
#define POWER_ON_HOLD_TIME		600
#define POWER_OFF_HOLD_TIME		2500
#define FW_RDY_TIMEOUT			6000

#define GPIO_INVALID			-1

#define MAX_RETRY_COUNT 		3
#define BWAN_WAKE_LOCK_TIMEOUT_SEC	(30 * HZ)

/* IOCTL command codes and macros */
#define BWAN_CODE	0xAA
#define BWAN_EHCI_RESET	_IO(BWAN_CODE, 1)

#define PRODUCT_BOARD_ID_LEN 16
#define PRODUCT_BOARD_REV_BYTE_INDEX 8
#define PRODUCT_BOARD_DVT_BYTE_VALUE '5'

static DEFINE_MUTEX(bwan_status_lock);

static int bwan_power = LOW;
static int bwan_usb_en = LOW;

static int gpio_wan_on = GPIO_INVALID;
static int gpio_wan_shutdown = GPIO_INVALID;
static int gpio_wan_usb_en = GPIO_INVALID;
static int gpio_wan_fw_rdy = GPIO_INVALID;
static int bwan_fw_rdy_status = -1;
static int gpio_wan_sim_present = GPIO_INVALID;
static int bwan_sim_present_status = -1;
static int bwan_sim_present_invert = 1;
static int bwan_power_gpio_invert = 1;

static struct kobject *bwan_kobj;
static struct kset *bwan_kset;

static wait_queue_head_t bwan_waitq;

static struct wake_lock bwan_lock;

#define bwan_pulse_gpio_wan_on(hold_time)			\
do {								\
	bwan_gpio_wan_on(LOW);					\
	msleep(hold_time);					\
	bwan_gpio_wan_on(HIGH);					\
} while (0)							\

#define bwan_pulse_gpio_wan_shutdown(hold_time)			\
do {								\
	bwan_gpio_wan_shutdown(LOW);				\
	msleep(hold_time);					\
	bwan_gpio_wan_shutdown(HIGH);				\
} while (0)							\

#define bwan_fw_rdy_wait(waitq, condition, timeout, __ret)	\
do {								\
	__ret = wait_event_interruptible_timeout(waitq,		\
				condition,			\
				timeout);			\
} while (0)

#define bwan_interrupt_handler(intr_name, work_queue)		\
do {								\
	disable_irq_nosync(gpio_to_irq(gpio_wan_##intr_name));	\
	schedule_delayed_work(&bwan_##work_queue,		\
		msecs_to_jiffies(INTERRUPT_DEBOUNCE_TIME));	\
} while (0)

#define bwan_request_irq(intr_name, intr_handler, __ret)	\
do {								\
	__ret = request_irq(gpio_to_irq(gpio_wan_##intr_name),	\
			bwan_##intr_handler,			\
			(IRQF_TRIGGER_RISING | 			\
			IRQF_TRIGGER_FALLING),			\
			#intr_name, NULL);			\
	if (__ret) {						\
		printk ("Unable to request irq %d for %s "	\
		"(gpio %d)\n", gpio_to_irq(gpio_wan_##intr_name),\
		#intr_name, gpio_wan_##intr_name);		\
	}							\
} while (0)

#if defined(CONFIG_ARCH_APQ8084_LOKI) || defined(CONFIG_ARCH_APQ8084_SATURN)
#define SIM_DETECT_GPIO_CFG_IDX 0

static struct gpiomux_setting gpio_sim_present = {
        .func = GPIOMUX_FUNC_GPIO,
        .drv  = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config bwan_gpio_configs[] = {
        {
                .gpio =  GPIO_INVALID,
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_sim_present,
                        [GPIOMUX_SUSPENDED] = &gpio_sim_present,
                },
        }
};
#endif


bool is_sim_detection_invert(void)
{
	struct device_node *ap;
	int len;

	ap = of_find_node_by_path("/idme/board_id");
	if (ap) {
	const char *boardid = of_get_property(ap, "value", &len);
		if (len >= PRODUCT_BOARD_ID_LEN) {
			if (boardid[PRODUCT_BOARD_REV_BYTE_INDEX] < PRODUCT_BOARD_DVT_BYTE_VALUE)
				return true;
		}
	}
	return false;
}

static void bwan_request_gpio(void)
{
#if defined(CONFIG_ARCH_APQ8084_LOKI) || defined(CONFIG_ARCH_APQ8084_SATURN)
        bwan_gpio_configs[SIM_DETECT_GPIO_CFG_IDX].gpio = gpio_wan_sim_present;
        msm_gpiomux_install(bwan_gpio_configs, ARRAY_SIZE(bwan_gpio_configs));
#endif
	gpio_request(gpio_wan_on, "Power_On");
	gpio_request(gpio_wan_shutdown, "Power_Shutdown");
	gpio_request(gpio_wan_usb_en, "USB_EN");
	gpio_request(gpio_wan_fw_rdy, "FW_RDY");
	gpio_request(gpio_wan_sim_present, "Sim_Present");

	return;
}

void bwan_free_gpio(void)
{
	gpio_free(gpio_wan_sim_present);
	gpio_free(gpio_wan_fw_rdy);
	gpio_free(gpio_wan_usb_en);
	gpio_free(gpio_wan_shutdown);
	gpio_free(gpio_wan_on);
	return;
}

void bwan_gpio_wan_on(int value)
{
	value = bwan_power_gpio_invert ? !value : value;
	gpio_direction_output(gpio_wan_on, value);
	return;
}

void bwan_gpio_wan_shutdown(int value)
{
	value = bwan_power_gpio_invert ? !value : value;
	gpio_direction_output(gpio_wan_shutdown, value);
	return;
}

void bwan_gpio_wan_usb_en(int value)
{
	gpio_direction_output(gpio_wan_usb_en, value);
	return;
}


static inline int bwan_on(int retry_count)
{
	int retval = 0, count = retry_count;
	bwan_fw_rdy_status = 0;

	bwan_pulse_gpio_wan_on(POWER_ON_HOLD_TIME);

	do {
		bwan_fw_rdy_wait(bwan_waitq,
				bwan_fw_rdy_status,
				msecs_to_jiffies(FW_RDY_TIMEOUT),
				retval);

		if (retval > 0) {
			if (!bwan_fw_rdy_status) {
				printk ("fw_rdy_status is not set\n");
			} else {
				printk ("Received FW ready - BWAN_HMI=0\n");
			}
			break;
		} else {
			printk ("Yet to receive FW ready..\n");
			--count;
		}
	} while (count);

	return retval;
}

static ssize_t bwan_power_show(struct kobject *kobj,
		    struct kobj_attribute *attr,
		    char *buf)
{
	if (strcmp(attr->attr.name, "power")) {
		return -1;
	}

	return sprintf(buf, "%d", bwan_power);
}

static ssize_t bwan_power_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf,
				size_t count)
{
	int var, retval, retry_count = MAX_RETRY_COUNT;

	sscanf(buf, "%du", &var);

	if (strcmp(attr->attr.name, "power")) {
		return -1;
	}

	printk("%s: Received option %d from wankit\n", __func__, var);

	switch (var) {
		case 2:
			/* Restart modem */
			wake_lock_timeout(&bwan_lock, BWAN_WAKE_LOCK_TIMEOUT_SEC);
			bwan_pulse_gpio_wan_shutdown(POWER_OFF_HOLD_TIME);

			/* At this point, the modem is shutdown.
			   Initiate the power up sequence */
			retval = bwan_on(retry_count);
			if (retval <= 0) {
				printk ("Modem reset was not successful\n");
			}

			/*
			 * Set the status to ON even though we didn't receive
			 * fw ready from the modem. The USB port *might* have
			 * enumerated
			 */
			bwan_power = HIGH;
			break;

		case 1:
			/* Initiate the power up sequence */
			if (bwan_power == HIGH) {
				break;
			}

			wake_lock_timeout(&bwan_lock, BWAN_WAKE_LOCK_TIMEOUT_SEC);

			if (bwan_fw_rdy_status == -1) {
				bwan_fw_rdy_status = 0;
			}

			retval = bwan_on(retry_count);

			/*
			 * Setting status to ON even though we didn't receive
			 * fw ready from the modem
			 */
			bwan_power = HIGH;

			break;
		case 0:
			if (bwan_power == LOW) {
				break;
			}

			bwan_pulse_gpio_wan_shutdown(POWER_OFF_HOLD_TIME);
			bwan_power = LOW;
			break;
		default:
			printk ("Input error\n");
	}

	return count;
}

static ssize_t bwan_usb_en_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	if (strcmp(attr->attr.name, "usben")) {
		return -1;
	}

	return sprintf(buf, "%d", bwan_usb_en);
}

static ssize_t bwan_usb_en_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf,
				size_t count)
{
	int var;

	sscanf(buf, "%du", &var);

	if (strcmp(attr->attr.name, "usben")) {
		return -1;
	}

	printk("%s: Received option %d from wankit\n", __func__, var);

	switch (var) {
		case 1:
			if (bwan_usb_en == HIGH) {
				break;
			}
			bwan_gpio_wan_usb_en(HIGH);
			bwan_usb_en = HIGH;
			break;
		case 0:
			if (bwan_usb_en == LOW) {
				break;
			}
			bwan_gpio_wan_usb_en(LOW);
			bwan_usb_en = LOW;
			break;
		default:
			printk("Invalid input\n");
	}

	return count;
}

static ssize_t bwan_fw_rdy_show(struct kobject *kobj,
		    struct kobj_attribute *attr,
		    char *buf)
{
	if (strcmp(attr->attr.name, "fw_rdy")) {
		return -1;
	}

	return sprintf(buf, "%d", bwan_fw_rdy_status);
}

static ssize_t bwan_sim_present_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	if (strcmp(attr->attr.name, "sim_present")) {
		return -1;
	}

	return sprintf(buf, "%d", bwan_sim_present_status);
}

static ssize_t bwan_sim_invert_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	if (strcmp(attr->attr.name, "sim_invert")) {
		return -1;
	}

	return sprintf(buf, "%d", bwan_sim_present_invert);
}

static ssize_t bwan_sim_invert_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf,
				size_t count)
{
	int var;
	sscanf(buf, "%du", &var);

	if (strcmp(attr->attr.name, "sim_invert")) {
		return -1;
	}

	switch (var) {
		case 1:
		case 0:
			bwan_sim_present_invert = var;
			if(bwan_sim_present_invert) bwan_sim_present_status = !bwan_sim_present_status;
		default:
			printk("Invalid input\n");
	}

	return count;
}

/*
 * Expose attributes - power, usben and fw_rdy as regular files
 */
static struct kobj_attribute bwan_power_attribute =
	__ATTR(power, 0666, bwan_power_show, bwan_power_store);

static struct kobj_attribute bwan_usb_en_attribute =
	__ATTR(usben, 0666, bwan_usb_en_show, bwan_usb_en_store);

static struct kobj_attribute bwan_fw_rdy_attribute =
	__ATTR(fw_rdy, 0666, bwan_fw_rdy_show, NULL);

static struct kobj_attribute bwan_sim_present_attribute =
	__ATTR(sim_present, 0666, bwan_sim_present_show, NULL);

static struct kobj_attribute bwan_sim_invert_attribute =
	__ATTR(sim_invert, 0666, bwan_sim_invert_show, bwan_sim_invert_store);

static struct attribute *attrs[] = {
	&bwan_power_attribute.attr,
	&bwan_usb_en_attribute.attr,
	&bwan_fw_rdy_attribute.attr,
	&bwan_sim_present_attribute.attr,
	&bwan_sim_invert_attribute.attr,
	NULL,
};

/*
 * Create an attribute group with no attribute name (an attribute name
 * would be taken as a subdirectory)
 */
static struct attribute_group attr_group = {
	.name = NULL,
	.attrs = attrs,
};

/*
 * We get here if we receive an interrupt. This could be
 * on a rising or falling edge.
 */
static void bwan_fw_rdy(struct work_struct *dummy)
{
	char *envp[] = { NULL, NULL};

	if (bwan_fw_rdy_status == gpio_get_value(gpio_wan_fw_rdy)) {

		/* Spurious interrupt */
		enable_irq(gpio_to_irq(gpio_wan_fw_rdy));
		return;
	}

	/* Should have been initialized to 0 during power up*/
	if (bwan_fw_rdy_status == -1) {
		enable_irq(gpio_to_irq(gpio_wan_fw_rdy));
		return;
	}

	/* Change status before enabling the irq */
	bwan_fw_rdy_status = !!gpio_get_value(gpio_wan_fw_rdy);

	wake_up(&bwan_waitq);

	enable_irq(gpio_to_irq(gpio_wan_fw_rdy));

	/* Send uevent */
	bwan_fw_rdy_status ? (envp[0] = "FW_RDY=1") :
			(envp[0] = "FW_RDY=0");
	kobject_uevent_env(bwan_kobj, KOBJ_CHANGE, envp);

	return;
}

/*
 * We get here if we receive an interrupt. This could be
 * on a rising or falling edge.
 */
static void bwan_sim_present(struct work_struct *dummy)
{
	char *envp[] = { NULL, NULL};
	int sim_gpio_v = gpio_get_value(gpio_wan_sim_present);

	if (bwan_sim_present_invert) sim_gpio_v = !sim_gpio_v;
	if (bwan_sim_present_status ==
		sim_gpio_v) {

		/* Spurious interrupt */
		enable_irq(gpio_to_irq(gpio_wan_sim_present));
		return;
	}

	/* Should have been initialized to 0 during power up*/
	if (bwan_sim_present_status == -1) {
		enable_irq(gpio_to_irq(gpio_wan_sim_present));
		return;
	}

	bwan_sim_present_status = sim_gpio_v;

	enable_irq(gpio_to_irq(gpio_wan_sim_present));

	bwan_sim_present_status ? (envp[0] = "SIM_PRESENT=1") :
			(envp[0] = "SIM_PRESENT=0");
	kobject_uevent_env(bwan_kobj, KOBJ_CHANGE, envp);

	return;
}

static DECLARE_DELAYED_WORK(bwan_fw_rdy_work, bwan_fw_rdy);
static DECLARE_DELAYED_WORK(bwan_sim_present_work, bwan_sim_present);

static irqreturn_t bwan_fw_rdy_handler(int irq, void *devid)
{
       /*
	* debounce for 30 ms, adds the function bwan_fw_ready
	* to the timer queue and returns.
	*/
	bwan_interrupt_handler(fw_rdy, fw_rdy_work);
	return IRQ_HANDLED;
}

static irqreturn_t bwan_sim_present_handler(int irq, void *devid)
{
	bwan_interrupt_handler(sim_present, sim_present_work);
	return IRQ_HANDLED;
}

static int bwan_gpio_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	gpio_wan_fw_rdy      = of_get_gpio(np, 0);
	gpio_wan_usb_en      = of_get_gpio(np, 1);
	gpio_wan_shutdown    = of_get_gpio(np, 2);
	gpio_wan_on          = of_get_gpio(np, 3);
	gpio_wan_sim_present = of_get_gpio(np, 4);
	if ((gpio_wan_on == GPIO_INVALID)
		|| (gpio_wan_shutdown == GPIO_INVALID)
		|| (gpio_wan_fw_rdy == GPIO_INVALID)
		|| (gpio_wan_usb_en == GPIO_INVALID)
		|| (gpio_wan_sim_present == GPIO_INVALID)
		) {
			printk ("WAN gpio's not initialized.\n");
			return -1;
	}

	bwan_request_gpio();

	return 0;
}

static void bwan_gpio_deinit(void)
{
	bwan_free_gpio();
}

static int bwan_probe(struct platform_device *pdev)
{
	int retval=0, irq;

	retval = bwan_gpio_init(pdev);
	if (retval) {
		return retval;
	}

	/*
	 * create a "wan" directory in sysfs.
	 * The first argument specifies the name of the kernel object
	 * (and hence the directory) to be created. The second argument
	 * specifies the kernel object associated with the parent directory.
	 */
	bwan_kobj = kobject_create_and_add("wan", NULL);

	if (!bwan_kobj) {
		printk ("Failed to create wan object\n");
		return -ENOMEM;
	}

	/*
	 * this would create the attribute group with the files as the
	 * attributes - power, usb_en, fw_rdy.
	 */
	retval = sysfs_create_group(bwan_kobj, &attr_group);

	if (retval) {
		printk ("Failed to create wan attributes\n");
		goto error;
	}

	bwan_kset = kset_create_and_add("bwan_kset", NULL, NULL);

	if (!bwan_kset) {
		retval = -1;
		goto error;
	}

	bwan_kobj->kset = bwan_kset;

	init_waitqueue_head(&bwan_waitq);

	if (gpio_direction_input(gpio_wan_fw_rdy)) {
		retval = -1;
		goto error;
	}

	/*
	 * request an irq for fw_rdy
	 */
	irq = platform_get_irq(pdev, 0);
	retval = request_irq(irq,
			bwan_fw_rdy_handler,
			(IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING),
			"fw_rdy_irq", NULL);
	if (retval) {
		printk ("Unable to request irq %d for fw_rdy "
			"(gpio %d)\n", irq, gpio_wan_fw_rdy);
		goto error;
	}

	/*
	 * initialize sim present gpio
	 */
	if (gpio_direction_input(gpio_wan_sim_present)) {
		retval = -1;
		goto error;
	}

	bwan_sim_present_status = gpio_get_value(gpio_wan_sim_present);
	if (bwan_sim_present_invert) bwan_sim_present_status = !bwan_sim_present_status;

	/*
	 * request an irq for sim_present
	 */
	irq = platform_get_irq(pdev, 1);
	retval = request_irq(irq,
			bwan_sim_present_handler,
			(IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING),
			"sim_present_irq", NULL);

	if (retval) {
		printk ("Unable to request irq %d for sim_present "
			"(gpio %d)\n", irq, gpio_wan_sim_present);
		goto error;
	}

	bwan_gpio_wan_shutdown(HIGH);
	bwan_gpio_wan_on(HIGH);

	/* Make sure the modem is powered down */
	if (bwan_fw_rdy_status) {
		bwan_pulse_gpio_wan_shutdown(POWER_OFF_HOLD_TIME);
	}

	/* Init wakelock */
	wake_lock_init(&bwan_lock,
			WAKE_LOCK_SUSPEND,
			"bwan_wake_lock");
	return 0;

	wake_lock_destroy(&bwan_lock);
error:
	bwan_gpio_deinit();
	if (bwan_kobj && bwan_kset) {
		kset_unregister(bwan_kset);
	}
	kobject_put(bwan_kobj);

	return retval;
}

static int bwan_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&bwan_lock);
	/* Make sure the modem is powered down */
	if (bwan_fw_rdy_status) {
		bwan_pulse_gpio_wan_shutdown(POWER_OFF_HOLD_TIME);
	}

	free_irq(platform_get_irq(pdev, 0), NULL);
	free_irq(platform_get_irq(pdev, 1), NULL);

	bwan_gpio_wan_shutdown(LOW);
	bwan_gpio_deinit();
	kset_unregister(bwan_kset);
	kobject_put(bwan_kobj);
	return 0;
}

static void bwan_shutdown(struct platform_device *pdev)
{
	printk(KERN_INFO "bwan shutdown\n");
	gpio_direction_output(gpio_wan_usb_en, 0);
	bwan_pulse_gpio_wan_shutdown(POWER_OFF_HOLD_TIME);
}

static int bwan_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	/* Disable fw_rdy? */
	return 0;
}

static int bwan_resume(struct platform_device *pdev)
{
	/* Send an uevent to the user */
	char *device_resume[] = {"DEVICE RESUME", NULL};
	kobject_uevent_env(bwan_kobj, KOBJ_CHANGE, device_resume);
	/* enable fw_rdy? */
	return 0;
}

static struct of_device_id bwan_of_match[] = {
	{.compatible = "lab126,bwan", },
	{ },
};

static struct platform_driver bwan_driver = {
	.driver = {
		.name = "bwan",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(bwan_of_match),
		},
	.suspend = bwan_suspend,
	.resume  = bwan_resume,
	.probe   = bwan_probe,
	.remove  = bwan_remove,
	.shutdown = bwan_shutdown,
};

static int __init bwan_init(void)
{
	bwan_sim_present_invert = is_sim_detection_invert();
	if (platform_driver_register(&bwan_driver) != 0) {
		printk("driver_reg::can not register bwan driver\n");
                return -1;
        }

	return 0;
}

static void bwan_exit(void)
{
	platform_driver_unregister(&bwan_driver);
}

module_init(bwan_init);
module_exit(bwan_exit);

MODULE_DESCRIPTION("WAN hardware driver");
MODULE_AUTHOR("Shrivatsan Vasudhevan <shrivats@lab126.com>");
MODULE_LICENSE("GPL");
