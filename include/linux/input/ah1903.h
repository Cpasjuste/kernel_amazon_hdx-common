#ifndef __AH1903_H__
#define __AH1903_H__
#include <linux/types.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <asm/atomic.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if defined(CONFIG_ARCH_APQ8084_LOKI)
#define MAX_SENSORS 3
#elif defined(CONFIG_ARCH_APQ8084_SATURN)
#define MAX_SENSORS 3
#else
#define MAX_SENSORS 3
#endif

#ifdef CONFIG_PM_WAKELOCKS
/* kernel/power/wakelock.c */
extern ssize_t pm_show_wakelocks(char *buf, bool show_active);
extern int pm_wake_lock(const char *buf);
extern int pm_wake_unlock(const char *buf);
#endif /* !CONFIG_PM_WAKELOCKS */

struct ah1903_platform_data {
	struct input_dev *dev[MAX_SENSORS];
	atomic_t used;
	struct work_struct irq_work[MAX_SENSORS];
	unsigned int irq[MAX_SENSORS];
	unsigned int irq_gpio[MAX_SENSORS];
	int (*init_irq)(void);
	int (*get_gpio_value)(void);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#else
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
#endif
	struct device *pdev;
};
#endif // __AH1903_H__

