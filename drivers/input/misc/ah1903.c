/*
 * Hall sensor driver capable of dealing with more than one sensor.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input/ah1903.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/input.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input/ah1903_ioctl.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
//----------------------------------------------------
#define KEY_PRESSED 1
#define KEY_RELEASED 0
#define TIMEOUT_VALUE 1900
#define ENABLE_REGULATOR

struct regulator *vcc_reg = NULL;
struct timer_list hall_timer;

struct ah1903_platform_data *g_ah1903_data = NULL;

static const char *ah1903_irqs[] = {
	"ah1903_irq1",
	"ah1903_irq2",
	"ah1903_irq3",
};

static const char *ah1903_name[] = {
	"ah1903_1",
	"ah1903_2",
	"ah1903_3",
};

static const char *ah1903_phys[] = {
	"ah1903/input1",
	"ah1903/input2",
	"ah1903/input3",
};

enum backlight_status {
	BL_OFF = 0,
	BL_ON
};

enum hall_status {
	HALL_CLOSED = 0,
	HALL_OPENED
};

enum hall_type {
	POWER = 0,
	CAMERA,
	COVER,
};

enum device_state {
	BOOT = 0,
	UP
};

struct hall_event_info {
	enum backlight_status bl_status;
	enum hall_status hall_current_status;
	unsigned int ignore_hall_event;
	enum hall_type type;
	enum backlight_status previous_bl_status;
};

static struct hall_event_info gHallEventInfo[] = {
	{BL_ON, HALL_CLOSED, 0, POWER,  BL_OFF},
	{BL_ON, HALL_CLOSED, 0, COVER,  BL_OFF},
	{BL_ON, HALL_CLOSED, 0, CAMERA, BL_OFF},
};

static const char *hall_name[] = {
	"POWER",
	"COVER",
	"CAMERA",
};

static const char *WAKELOCK = "HallSensor 2000000000";
static enum device_state mState = BOOT;
//----------------------------------------------------
#define AH1903_FTM_PORTING

#ifdef AH1903_FTM_PORTING
#include <linux/miscdevice.h>
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#ifdef	CONFIG_PROC_FS      // Proc function
#define	AH1903_PROC_FILE	"driver/hall_sensor"
#define	AH1903_COUNT_PROC_FILE       "driver/hall_sensor_count"

static unsigned int *ah1903_count[] = {
	0,
	0,
	0,
};

static int ah1903_proc_show(struct seq_file *m, void *v) {
	u8 reg_val=2;
	int i;
	for(i=0; i<MAX_SENSORS; i++) {
		if(i == 0) {
			reg_val= reg_val | (gHallEventInfo[i].hall_current_status << i);
		} else {
			reg_val= reg_val |(gHallEventInfo[i].hall_current_status << (i+1));
		}
		printk(KERN_DEBUG "%s: hall_%d_status=%d count = %d \n", __func__,
				i, gHallEventInfo[i].hall_current_status,
				(int)ah1903_count[i]);
	}
	seq_printf(m, "0x%x \n",reg_val);
	return 0;
}

static int ah1903_proc_show_count(struct seq_file *m, void *v) {
	int i;
	seq_puts(m, "Stats ");
	for(i=0; i<MAX_SENSORS; i++) {
		seq_printf(m, " Hall[%d]=%d", i+1, (int)ah1903_count[i]);
	}
	seq_puts(m, "\n");
	return 0;
}

static int ah1903_proc_open(struct inode *inode, struct  file *file) {
	return single_open(file, ah1903_proc_show, NULL);
}

static int ah1903_proc_open_count(struct inode *inode, struct  file *file) {
	return single_open(file, ah1903_proc_show_count, NULL);
}

static const struct file_operations ah1903_proc_fops = {
	.owner = THIS_MODULE,
	.open = ah1903_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ah1903_count_fops = {
        .owner = THIS_MODULE,
        .open = ah1903_proc_open_count,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

static void create_ah1903_proc_file(void) {
	struct proc_dir_entry *proc;
	proc = proc_create(AH1903_PROC_FILE, 0644, NULL, &ah1903_proc_fops);
	if (proc) {
		if (!proc_create(AH1903_COUNT_PROC_FILE, 0600, NULL,
				&ah1903_count_fops))
			printk(KERN_ERR "%s: Error creating %s \n", __func__,
						AH1903_COUNT_PROC_FILE);
	} else {
		printk(KERN_ERR "%s Error creating %s\n", __func__, AH1903_PROC_FILE);
	}
}

static void remove_ah1903_proc_file(void)
{
	remove_proc_entry(AH1903_PROC_FILE, NULL);
	remove_proc_entry(AH1903_COUNT_PROC_FILE, NULL);
}
#endif      //Proc function

#ifdef AH1903_FTM_PORTING
static int ah1903_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long ah1903_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg) {
	int ret = 0;
	int i;
	struct ioctl_cmd data;
	memset(&data, 0, sizeof(data));
	switch(cmd) {
	case IOCTL_VALSET:
		if (!access_ok(VERIFY_READ,(void __user *)arg, _IOC_SIZE(cmd))) {
			ret = -EFAULT;
			goto done;
		}
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			ret = -EFAULT;
			goto done;
		}
		printk(KERN_DEBUG "ah1903_ioctl received data = %d\n",data.halt_key);
		for (i=0; i<MAX_SENSORS; i++) {
			gHallEventInfo[i].ignore_hall_event = data.halt_key;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
done:
	printk(KERN_DEBUG "ah1903_ioctl DONE \n");
	return ret;
}

static ssize_t ah1903_read(struct file *file, char __user * buffer,
			size_t size, loff_t * f_pos) {
	unsigned int val=2;
	int i;
	for (i=0; i<MAX_SENSORS; i++) {
		if (i == 0) {
			val = val | (gHallEventInfo[i].hall_current_status << i);
		} else {
			val = val | (gHallEventInfo[i].hall_current_status << (i+1));
		}
		printk(KERN_DEBUG "Hall sensor %d state:%d\n",
				i, gHallEventInfo[i].hall_current_status);
	}
	printk(KERN_DEBUG "Hall sensor state: 0x%x\n", val);

	if(copy_to_user(buffer, &val, sizeof(val))){
		printk(KERN_ERR "[line:%d] copy_to_user failed\n",  __LINE__ );
		return -EFAULT;
	}
	return 0;
}

static ssize_t ah1903_write(struct file *file, const char __user *buffer,
			size_t size, loff_t *f_ops) {
	return 0;
}
static int ah1903_release(struct inode *inode, struct file *filp) {
	return 0;
}
#endif //AH1903_FTM_PORTING

static void hall_handle_event(int i) {
	char *envp[2];
	char buf[120];
	struct kobject *kobj = &g_ah1903_data->pdev->kobj;
	struct input_dev *dev = g_ah1903_data->dev[i];

	gHallEventInfo[i].hall_current_status = gpio_get_value(
		g_ah1903_data->irq_gpio[i]);

#ifdef CONFIG_PROC_FS
	ah1903_count[i]++;
#endif

	printk(KERN_DEBUG "AH1903: %s hall_%d_current_status = %s\n",
		hall_name[i], i,
		gHallEventInfo[i].hall_current_status ? "HALL_OPENED" : "HALL_CLOSED");
	printk(KERN_DEBUG "AH1903: %s gHallEventInfo[%d].bl_status = %s\n",
		hall_name[i], i,
		gHallEventInfo[i].bl_status ? "BL_ON" : "BL_OFF");

	if (gHallEventInfo[i].ignore_hall_event == false) {
		switch(gHallEventInfo[i].type) {
		case POWER:
			if (gHallEventInfo[1].hall_current_status == HALL_CLOSED) {
				/*Hall sensor State Machine: only two cases need to send
				power key event:1.close book-cover in Normal mode(BL_ON)
				2.open book-cover in Suspend mode(BL_OFF) */
				if (((gHallEventInfo[i].bl_status == BL_ON) &&
					(gHallEventInfo[i].hall_current_status == HALL_CLOSED)) ||
					((gHallEventInfo[i].bl_status == BL_OFF) &&
					(gHallEventInfo[i].hall_current_status == HALL_OPENED))) {

					if ((gHallEventInfo[i].previous_bl_status !=
						gHallEventInfo[i].bl_status) || (mState == BOOT)) {
						printk(KERN_DEBUG "AH1903: KEY_POWER = %s\n",
							gHallEventInfo[i].bl_status ? "OFF" : "ON");
						gHallEventInfo[i].previous_bl_status =
							gHallEventInfo[i].bl_status;
						input_report_key(dev, KEY_POWER, KEY_PRESSED);
						input_sync(dev);
						mdelay(20);
						input_report_key(dev, KEY_POWER, KEY_RELEASED);
						input_sync(dev);
					}
				}
			}
		break;

		case COVER:
			if (gHallEventInfo[i].bl_status == BL_OFF) {
				input_report_key(dev, KEY_POWER, KEY_PRESSED);
				input_sync(dev);
				mdelay(20);
				input_report_key(dev, KEY_POWER, KEY_RELEASED);
				input_sync(dev);
				mdelay(20);
			}
			snprintf(buf, sizeof(buf),"COVER_STATE=%s",
				gHallEventInfo[i].hall_current_status ? "1" : "0");
			envp[0] = buf;
			envp[1] = NULL;
			kobject_uevent_env(kobj, KOBJ_CHANGE, envp);
			input_event(dev, EV_MSC, MSC_RAW,
				gHallEventInfo[i].hall_current_status);
			input_sync(dev);
		break;

		case CAMERA:
			snprintf(buf, sizeof(buf),"CAMERA_STATE=%s",
				gHallEventInfo[i].hall_current_status ? "1" : "0");
			envp[0] = buf;
			envp[1] = NULL;
			kobject_uevent_env(kobj, KOBJ_CHANGE, envp);
			input_event(dev, EV_MSC, MSC_RAW,
				gHallEventInfo[i].hall_current_status);
			input_sync(dev);
		break;

		default:
			printk(KERN_ERR "AH1903: %s Unknown HALL TYPE \n", __func__);
		break;
		}
	}
}

static void hall_timeout_report(unsigned long arg) {
	int i=0;
	struct input_dev *dev = g_ah1903_data->dev[i];

	for (i=0; i<MAX_SENSORS; i++) {
		gHallEventInfo[i].hall_current_status = gpio_get_value(g_ah1903_data->irq_gpio[i]);
	}

	i = 0;
	printk(KERN_DEBUG "AH1903: %s %s hall_%d_current_status = %s\n",
		__func__, hall_name[i], i,
		gHallEventInfo[i].hall_current_status ? "HALL_OPENED" : "HALL_CLOSED");
	printk(KERN_DEBUG "AH1903: %s %s gHallEventInfo[%d].bl_status = %s\n",
		__func__, hall_name[i], i,
		gHallEventInfo[i].bl_status ? "BL_ON" : "BL_OFF");

	if (gHallEventInfo[i].ignore_hall_event == false) {
		if (gHallEventInfo[1].hall_current_status == HALL_CLOSED) {
			/*Hall sensor State Machine: only two cases need to send
			power key event:
			1.close book-cover in Normal mode(BL_ON)
			2.open book-cover in Suspend mode(BL_OFF) */
			if (((gHallEventInfo[i].bl_status == BL_ON) &&
				(gHallEventInfo[i].hall_current_status == HALL_CLOSED)) ||
				((gHallEventInfo[i].bl_status == BL_OFF) &&
				(gHallEventInfo[i].hall_current_status == HALL_OPENED))) {
				printk(KERN_DEBUG "AH1903: %s KEY_POWER = %s\n",
					__func__, gHallEventInfo[i].bl_status ? "OFF" : "ON");
				gHallEventInfo[i].previous_bl_status = gHallEventInfo[i].bl_status;
				input_report_key(dev, KEY_POWER, KEY_PRESSED);
				input_sync(dev);
				mdelay(20);
				input_report_key(dev, KEY_POWER, KEY_RELEASED);
				input_sync(dev);
			}
		}
	}
}

static void hall_init_timer(void) {
	init_timer(&hall_timer);
	hall_timer.function = hall_timeout_report;
	hall_timer.data = 0;
	hall_timer.expires = jiffies + ((TIMEOUT_VALUE*HZ)/1000);
	add_timer(&hall_timer);
	printk(KERN_DEBUG "AH1903 hall_init_timer Done\n");
}

static void ah1903_irq_work(struct work_struct *work) {
	int i;
	for (i=0; i<MAX_SENSORS; i++) {
		if (&g_ah1903_data->irq_work[i] == work) {
			hall_handle_event(i);
			if(i == 0){
				mod_timer(&hall_timer, jiffies + ((TIMEOUT_VALUE*HZ)/1000));
			}
			break;
		}
	}
}

static irqreturn_t ah1903_interrupt(int irq, void *dev_id) {
	int i;
	printk(KERN_DEBUG "AH1903 ah1903_interrupt irq = %d \n", irq);

#ifdef CONFIG_PM_WAKELOCKS
	pm_wake_lock(WAKELOCK);
#endif

	for (i=0; i<MAX_SENSORS; i++) {
		if (irq == g_ah1903_data->irq[i]) {
			schedule_work(&g_ah1903_data->irq_work[i]);
			break;
		}
	}
	return IRQ_HANDLED;
}

static int ah1903_input_open(struct input_dev *dev) {
	return 0;
}
static void ah1903_input_close(struct input_dev *dev) {
}

#ifdef ENABLE_REGULATOR
static int ah1903_config_regulator(struct device dev, bool on) {
	int rc = 0;
	if (on) {
		vcc_reg = regulator_get(&dev, "vcc");
		if (IS_ERR(vcc_reg)) {
			rc = PTR_ERR(vcc_reg);
			printk(KERN_ERR "ah1903 regulator_get failed return = %d\n", rc);
			vcc_reg = NULL;
			return rc;
		}

		if (regulator_count_voltages(vcc_reg) > 0) {
			rc = regulator_set_voltage(vcc_reg, 1800000, 1800000);
			if (rc) {
				printk(KERN_ERR "ah1903 regulator_setvoltage failed %d\n", rc);
				regulator_put(vcc_reg);
				vcc_reg = NULL;
				return rc;
			}
		}

		rc = regulator_enable(vcc_reg);
		if (rc) {
			printk(KERN_ERR "ah1903 regulator_enable failed, return %d\n", rc);
			if (regulator_count_voltages(vcc_reg) > 0) {
				regulator_set_voltage(vcc_reg, 0, 1800000);
			}
			regulator_put(vcc_reg);
			vcc_reg = NULL;
			return rc;
		}
		printk(KERN_DEBUG "ah1903 regulator_enabled return %d\n", rc);
	} else {
		if (!IS_ERR_OR_NULL(vcc_reg)) {
			if (regulator_count_voltages(vcc_reg) > 0) {
				regulator_set_voltage(vcc_reg, 0, 1800000);
			}
			regulator_disable(vcc_reg);
			regulator_put(vcc_reg);
			vcc_reg = NULL;
			printk(KERN_DEBUG "ah1903 regulator_disabled return %d\n", rc);
		}
	}
	return rc;
}
#endif

#ifdef AH1903_FTM_PORTING
static const struct file_operations ah1903_dev_fops = {
	.owner = THIS_MODULE,
	.open = ah1903_open,
	.read = ah1903_read,
	.write = ah1903_write,
	.release = ah1903_release,
	.unlocked_ioctl = ah1903_ioctl,
};

static struct miscdevice ah1903_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ah1903",
	.fops = &ah1903_dev_fops,
};
#endif //AH1903_FTM_PORTING

static int ah1903_probe(struct platform_device *pdev) {
	//struct input_dev *input_dev;
	int ret;
	enum of_gpio_flags flags;
	int i;
	struct device_node *node;
	unsigned long irqflags;

	node = pdev->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	g_ah1903_data= kzalloc(sizeof(struct ah1903_platform_data), GFP_KERNEL);
	if (!g_ah1903_data) {
		ret = -ENOMEM;
		printk(KERN_ERR "[%s:%d] allocate g_ah1903_data fail!\n",
			__func__, __LINE__);
		goto fail1;
	}

#ifdef CONFIG_PROC_FS
	create_ah1903_proc_file();
#endif

	/* input device */
	for (i=0; i<MAX_SENSORS; i++) {
		g_ah1903_data->dev[i] = input_allocate_device();
		if (!g_ah1903_data->dev[i]) {
			ret = -ENOMEM;
			printk(KERN_ERR "[%s:%d] allocate input device fail!\n",
				__func__, __LINE__);
			goto fail2;
		}

		/*input device settings*/
		g_ah1903_data->dev[i]->name = ah1903_name[i];
		g_ah1903_data->dev[i]->phys = ah1903_phys[i];

		__set_bit(EV_KEY, g_ah1903_data->dev[i]->evbit);
		__set_bit(EV_MSC, g_ah1903_data->dev[i]->evbit);
		__set_bit(MSC_RAW, g_ah1903_data->dev[i]->mscbit);
		__set_bit(KEY_POWER, g_ah1903_data->dev[i]->keybit);

		g_ah1903_data->dev[i]->open = ah1903_input_open;
		g_ah1903_data->dev[i]->close = ah1903_input_close;

		ret = input_register_device(g_ah1903_data->dev[i]);
		if (ret) {
			printk(KERN_ERR "[%s:%d]ah1903 input register device fail!\n",
				__func__, __LINE__);
			goto fail2;
		}

		g_ah1903_data->irq_gpio[i] = of_get_named_gpio_flags(node,
									"gpios", i, &flags);
		if (gpio_is_valid(g_ah1903_data->irq_gpio[i])) {
			ret = gpio_request(g_ah1903_data->irq_gpio[i], ah1903_irqs[i]);
			if (ret) {
				printk(KERN_ERR "[%s:%d] failed gpio %d request \n",
					__func__, __LINE__, g_ah1903_data->irq_gpio[i]);
				goto fail3;
			}
			ret = gpio_direction_input(g_ah1903_data->irq_gpio[i]);
			if (ret) {
				printk(KERN_ERR "[%s:%d] failed set gpio %d directiont \n",
					__func__, __LINE__, g_ah1903_data->irq_gpio[i]);
				goto fail3;
			}
		}
		g_ah1903_data->irq[i] = platform_get_irq(pdev, i);
		printk(KERN_ERR "[%s:%d] gpio %d setup properly irq %d \n",
			__func__, __LINE__, g_ah1903_data->irq_gpio[i], g_ah1903_data->irq[i]);

		INIT_WORK(&g_ah1903_data->irq_work[i], ah1903_irq_work);
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_NO_SUSPEND | IRQF_FORCE_RESUME | IRQF_EARLY_RESUME |
			IRQF_ONESHOT;
		ret = request_threaded_irq(g_ah1903_data->irq[i], NULL,
				ah1903_interrupt, irqflags, ah1903_irqs[i], NULL);
		if (ret) {
			printk(KERN_ERR "ah1903 request_irq %d failed, return:%d\n",
				g_ah1903_data->irq[i], ret);
			goto fail3;
		}

		enable_irq_wake(g_ah1903_data->irq[i]);

		gHallEventInfo[i].hall_current_status =
			gpio_get_value(g_ah1903_data->irq_gpio[i]);

	}

	hall_init_timer();

#ifdef CONFIG_FB
	g_ah1903_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&g_ah1903_data->fb_notif);
	if (ret)
		printk(KERN_ERR "Unable to register fb_notifier: %d\n", ret);
#endif

#ifdef ENABLE_REGULATOR
	ret = ah1903_config_regulator(pdev->dev, 1);
	if (ret) {
		printk(KERN_ERR "[%s:%d] AH1903 ah1903_config_regulator fail!\n", __func__, __LINE__);
		goto fail3;
	}
#endif

	g_ah1903_data->pdev = (&pdev->dev);
	mState = BOOT;
	printk(KERN_DEBUG "AH1903 Probe OK\n");
	return 0;

fail3:
	for (i=0; i<MAX_SENSORS; i++) {
		if (gpio_is_valid(g_ah1903_data->irq_gpio[i])) {
			gpio_free(g_ah1903_data->irq_gpio[i]);
		}
		if(g_ah1903_data->irq[i]) {
			free_irq(g_ah1903_data->irq[i], NULL);
		}
	}
fail2:
	for (i=0; i<MAX_SENSORS; i++) {
		if(g_ah1903_data->dev[i]) {
			input_free_device(g_ah1903_data->dev[i]);
		}
	}
	kfree(g_ah1903_data);
fail1:
	return ret;
}

static int ah1903_remove(struct platform_device *pdev) {
	int i;
	for (i=0; i<MAX_SENSORS; i++) {
		free_irq(g_ah1903_data->irq[i], NULL);
		input_unregister_device(g_ah1903_data->dev[i]);
		input_free_device(g_ah1903_data->dev[i]);
		gpio_free(g_ah1903_data->irq_gpio[i]);
	}

#ifdef CONFIG_PROC_FS
	remove_ah1903_proc_file();
#endif

#ifdef CONFIG_FB
	fb_unregister_client(&g_ah1903_data->fb_notif);
#endif
	del_timer_sync(&hall_timer);

#ifdef CONFIG_PM_WAKELOCKS
	pm_wake_unlock(WAKELOCK);
#endif

#ifdef ENABLE_REGULATOR
	ah1903_config_regulator(pdev->dev, 0);
#endif

	kfree(g_ah1903_data);

	return 0;
}

static void ah1903_shutdown(struct platform_device *pdev) {
	int i;
	printk(KERN_DEBUG "ah1903: shutdown\n");
	for (i=0; i<MAX_SENSORS; i++)
		cancel_work_sync(&g_ah1903_data->irq_work[i]);
}

static int ah1903_suspend(struct device *dev) {
	int i;
	for(i=0; i<MAX_SENSORS; i++) {
		gHallEventInfo[i].bl_status = BL_OFF;
		gHallEventInfo[i].previous_bl_status = BL_ON;
		cancel_work_sync(&g_ah1903_data->irq_work[i]);
	}
	mState = UP;
	printk(KERN_DEBUG "AH1903 ah1903_suspend\n");
	return 0;
}

static int ah1903_resume(struct device *dev) {
	int i = 0;
	for (i = 0; i < MAX_SENSORS; i++) {
		gHallEventInfo[i].bl_status = BL_ON;
		gHallEventInfo[i].previous_bl_status = BL_OFF;
                check_irq_resend(irq_to_desc(g_ah1903_data->irq[i]),
					g_ah1903_data->irq[i]);
		if (i == 1)
			hall_handle_event(i);
	}
	printk(KERN_DEBUG "AH1903 ah1903_resume\n");
	return 0;
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data) {
	struct fb_event *evdata = data;
	int *blank;
	if ((evdata && evdata->data) && (event == FB_EVENT_BLANK)) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ah1903_resume(g_ah1903_data->pdev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ah1903_suspend(g_ah1903_data->pdev);
	}

	return 0;
}
#endif

static const struct dev_pm_ops ah1903_dev_pm_ops = {
	.suspend  = ah1903_suspend,
	.resume   = ah1903_resume,
};

static struct of_device_id ah1903_of_match[] = {
	{.compatible = "diode,ah1903", },
	{ },
};
MODULE_DEVICE_TABLE(of, ah1903_of_match);

static struct platform_driver ah1903_device_driver = {
	.probe    = ah1903_probe,
	.remove   = ah1903_remove,
	.shutdown = ah1903_shutdown,
	.driver   = {
		.name   = "ah1903",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ah1903_of_match),
	}
};

static int __init ah1903_init(void) {
	printk(KERN_DEBUG "AH1903 sensors driver: init\n");

#ifdef AH1903_FTM_PORTING
	if (0 != misc_register(&ah1903_dev)) {
		printk(KERN_ERR "ah1903_dev register failed.\n");
		return 0;
	} else {
		printk(KERN_DEBUG "ah1903_dev register ok.\n");
	}
#endif

	return platform_driver_register(&ah1903_device_driver);
}

static void __exit ah1903_exit(void) {
	printk(KERN_DEBUG "AH1903 sensors driver: exit\n");

#ifdef AH1903_FTM_PORTING
	misc_deregister(&ah1903_dev);
#endif
	platform_driver_unregister(&ah1903_device_driver);
}

module_init(ah1903_init);
module_exit(ah1903_exit);

MODULE_DESCRIPTION("hall sensor driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
