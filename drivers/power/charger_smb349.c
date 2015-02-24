/*
 * charger_smb349.c
 *
 * Summit SMB349 Charger detection Driver
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 * Donald Chan (hoiho@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/workqueue.h>
#include <linux/qpnp/power-on.h>
#include <linux/wakelock.h>

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#define METRICS_STR_LEN 128
#endif

#define SMB349_I2C_ADDRESS	0x5F

#define DRIVER_NAME			"smb349"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Donald Chan"

#define SMB349_CURRENT_LIMIT		0x0
#define SMB349_OTHER_CHARGE_CURRENT	0x1
#define SMB349_FUNCTIONS		0x2
#define SMB349_FLOAT_VOLTAGE		0x3
#define SMB349_CHARGE_CONTROL		0x4
#define SMB349_STAT_TIMERS		0x5
#define SMB349_ENABLE_CONTROL		0x6
#define SMB349_THERMAL_CONTROL		0x7
#define SMB349_SYSOK_USB30		0x8
#define SMB349_OTHER_CONTROL_A		0x9
#define SMB349_OTG_THERM_CONTROL	0xA
#define SMB349_CELL_TEMP		0xB
#define SMB349_FAULT_INTERRUPT	0xC
#define SMB349_INTERRUPT_STAT	0xD
#define SMB349_SLAVE_ADDR	0xE
#define SMB349_I2C_ADDR		0x12

#define SMB349_COMMAND_REG_A	0x30
#define SMB349_COMMAND_REG_B	0x31
#define SMB349_COMMAND_REG_C	0x33
#define SMB349_INTSTAT_REG_A	0x35
#define SMB349_INTSTAT_REG_B	0x36
#define SMB349_INTSTAT_REG_C	0x37
#define SMB349_INTSTAT_REG_D	0x38
#define SMB349_INTSTAT_REG_E	0x39
#define SMB349_INTSTAT_REG_F	0x3A
#define SMB349_STATUS_REG_A	0x3B
#define SMB349_STATUS_REG_B	0x3C
#define SMB349_STATUS_REG_C	0x3D
#define SMB349_STATUS_REG_D	0x3E
#define SMB349_STATUS_REG_E	0x3F

#define SMB349_TLOW_THRESHOLD	37
#define SMB349_THIGH_THRESHOLD	113
#define SMB349_VHI_THRESHOLD	4350
#define SMB349_VLO_THRESHOLD	2500

#define SMB349_CHG_TIMEOUT_MASK 0x0C
#define SMB349_CHG_TIMEOUT_OFF  0x0C
#define SMB349_CHG_TIMEOUT_MAX  0x08
#define SMB349_CHG_TIMEOUT_MED  0x04
#define SMB349_CHG_TIMEOUT_MIN  0x00

#define SMB349_FUNCTIONS_APSD_EN_MASK	0x04
#define SMB349_FUNCTIONS_AICL_EN_MASK	0x10

#define SMB349_IS_APSD_DONE(value)	((value) & (1 << 6))
#define SMB349_IS_POWER_OK(value) ((value) & (1 << 0))

#define SMB349_APSD_RESULT_OTHER	(1 << 0)
#define SMB349_APSD_RESULT_SDP		(1 << 1)
#define SMB349_APSD_RESULT_DCP		(1 << 2)
#define SMB349_APSD_RESULT_CDP		(1 << 3)
#define SMB349_APSD_RESULT_ACA0		(1 << 4)
#define SMB349_APSD_RESULT_ACA1		(1 << 5)
#define SMB349_APSD_RESULT_ACA2		(1 << 6)
#define SMB349_APSD_RESULT_ACA3		(1 << 7)

#define SMB349_CHARGING_STATUS(value)	(((value) >> 1) & 0x3)

#define SMB349_CHARGING_STATUS_NOT_CHARGING	(0)
#define SMB349_CHARGING_STATUS_PRE_CHARGING	(1)
#define SMB349_CHARGING_STATUS_FAST_CHARGING	(2)
#define SMB349_CHARGING_STATUS_TAPER_CHARGING	(3)

#define SMB349_USB5CS_MODE(value)	(((value) >> 5) & 0x3)

#define SMB349_HC_MODE			(0)
#define SMB349_USB1_MODE		(1)
#define SMB349_USB5_MODE		(2)

#define SMB349_IS_AICL_DONE(value)	((value) & (1 << 4))

#define SMB349_AICL_RESULT(value)	((value) & 0xf)

#define SMB349_THERMAL_CONTROL_MONITOR	(1<<4)
#define SMB349_ENABLE_CHG_FAULT_IRQ	(1<<2)

#define SMB349_EN_CTRL_MASK		(0x60)
#define SMB349_USBCS_EN_CTRL_MASK	(0x10)
#define SMB349_COMMAND_A_CHRG_EN	(1<<1)
#define SMB349_COMMAND_A_OTG_EN        (1<<4)
#define SMB349_OTG_CUR_LIMIT_MASK	(0x0c)
#define SMB349_DEFAULT_CUR_LIMIT	(1000)
#define SMB349_OVERCURRENT_IRQ	(1<<7)

#define SMB349_INTSTAT_A_HOT_HARD		(1<<7)
#define SMB349_INTSTAT_A_COLD_HARD		(1<<5)
#define SMB349_INTSTAT_A_COLD_SOFT		(1<<1)
#define SMB349_INTSTAT_A_COLD_SOFT_STATUS	(1<<0)
#define SMB349_INTSTAT_B_BATT_OV		(1<<7)
#define SMB349_INTSTAT_B_BATT_MISSING		(1<<5)
#define SMB349_INTSTAT_B_BATT_LV		(1<<3)
#define SMB349_INTSTAT_C_INT_TEMP		(1<<7)
#define SMB349_INTSTAT_D_CHG_TIMEOUT		(1<<1)
#define SMB349_INTSTAT_D_APSD_CMPL		(1<<7)
#define SMB349_INTSTAT_D_AICL_CMPL		(1<<5)
#define SMB349_INTSTAT_E_UV			(1<<1)
#define SMB349_INTSTAT_E_UV_STATUS		(1<<0)

#define SMB349_FLOAT_VOLTAGE_MIN	346
#define SMB349_FLOAT_VOLTAGE_MAX	472
#define SMB349_FLOAT_VOLTAGE_MASK	0x3F


enum {
	SMB349_USB_MODE_1,
	SMB349_USB_MODE_5,
	SMB349_USB_MODE_HC,
};

struct smb349_regulator {
       struct regulator_desc   rdesc;
       struct regulator_dev    *rdev;
};

struct smb349_priv {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct power_supply ac;
	struct power_supply usb;
	struct power_supply *dwc3_usb;
	struct power_supply *bq27x41;
	atomic_t usb_online;
	atomic_t ac_online;
	struct mutex lock;
	struct delayed_work irq_work;
	struct delayed_work charge_work;
	struct delayed_work otg_work;
#ifdef CONFIG_AMAZON_METRICS_LOG
	struct work_struct disconnect_log_work;
#endif
	struct smb349_regulator otg_vreg;
	struct smb349_regulator apsd_ctrl;
	int apsd_disabled;
	int chrg_stat;
	int chrg_en;
	int apsd_trigger;
	int chrg_hcs;
        int chrg_susp;
	int bad_battery;
	int chg_health;
	int current_limit;
	int otg_current_limit;
	int otg_enabled;
	int usb_boot_det;
	int polling_mode;
	int charge_mode;
	int irq;

	atomic_t suspended;
	int handle_irq;
	struct wake_lock irq_wake_lock;
};
/* DEBUG */
//#define SMB349_DEBUG

#if defined(CONFIG_PM_WAKELOCKS)
extern int pm_wake_lock(const char *string);
extern int pm_wake_unlock(const char *string);
#endif

static int smb349_APSD_enable(struct smb349_priv *chip, int enable);
static int smb349_modify_charge_current(struct smb349_priv *priv,
					int fast_charge_current,
					int precharge_current);
static int smb349_get_current_limit_index(int limit);

#define FLOAT_VOLTAGE_NORMAL 4350000
#define FLOAT_VOLTAGE_COLD   4100000
#define FLOAT_VOLTAGE_MIN    3000000

enum {
	CHARGE_MODE_NORMAL,
	CHARGE_MODE_900MA_435V,
	CHARGE_MODE_500MA_435V,
};

/*
 * FIXME
 *
 * 1. Use regmap APIs
 * 2. Adjust pre-charge/fast-charge currents
 * 3. Support for re-trigger APSD, AICL
 */

static const struct i2c_device_id smb349_id[] =  {
	{ "smb349", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, smb349_id);

static const int smb349_aicl_results[] = {
				 500,  900, 1000, 1100,
				1200, 1300, 1500, 1600,
				1700, 1800, 2000, 2200,
				2400, 2500, 3000, 3500
			};

static const int smb349_otg_current_limits[] = {
				250, 500, 750, 1000
			};

static const int smb349_fast_charge_currents[] = {
				1000, 1200, 1400, 1600,
				1800, 2000, 2200, 2400
			};

static const int smb349_input_current_limits[] = {
				 500,  900, 1000, 1100,
				1200, 1300, 1500, 1600,
				1700, 1800, 2000
			};

static const int smb349_precharge_currents[] = {
				200, 300, 400, 500, 600,
				700, 100
			};

static int smb349_i2c_read(struct i2c_client *client,
			u8 reg_num, u8 *value);

static int smb349_i2c_write(struct i2c_client *client,
			u8 reg_num, u8 value);

static int smb349_change_current_limit(struct smb349_priv *priv, int idx);

static int apsd_triggered = 0;
static void enable_apsd_trigger(struct smb349_priv *priv)
{
	if (priv->apsd_trigger == -1) {
		dev_err(priv->dev, "Wave generate gpio is not ready.\n");
		return;
	}
	gpio_set_value(priv->apsd_trigger, 1);
	msleep(30);
	gpio_set_value(priv->apsd_trigger, 0);

	msleep(25);

	gpio_set_value(priv->apsd_trigger, 1);
	msleep(30);
	gpio_set_value(priv->apsd_trigger, 0);
}

static const char *smb349_apsd_result_string(u8 value)
{
	switch (value) {
	case SMB349_APSD_RESULT_OTHER:
		return "Other Downstream Port";
		break;
	case SMB349_APSD_RESULT_SDP:
		return "SDP";
		break;
	case SMB349_APSD_RESULT_DCP:
		return "DCP";
		break;
	case SMB349_APSD_RESULT_CDP:
		return "CDP";
		break;
	case SMB349_APSD_RESULT_ACA0:
		return "ACA-A charging port";
		break;
	case SMB349_APSD_RESULT_ACA1:
		return "ACA-B charging port";
		break;
	case SMB349_APSD_RESULT_ACA2:
		return "ACA-C charging port";
		break;
	case SMB349_APSD_RESULT_ACA3:
		return "ACA dock";
		break;
	default:
		return "unknown";
		break;
	}
}

static inline int smb349_aicl_current(int reg)
{
	return smb349_aicl_results[SMB349_AICL_RESULT(reg)];
}

static void smb349_aicl_complete(struct smb349_priv *priv)
{
	u8 value = 0xff;
	int ret = -1;

	mutex_lock(&priv->lock);

	ret = smb349_i2c_read(priv->i2c_client, SMB349_STATUS_REG_E, &value);

	if (ret) {
		dev_err(priv->dev,
			"Failed to read SMB349_STATUS_REG_E: %d\n", ret);
		goto done;
	}

	dev_info(priv->dev, "AICL result: %d mA\n",
			smb349_aicl_results[value & 0xf]);

done:
	mutex_unlock(&priv->lock);
	return;
}

static int smb349_config(struct smb349_priv *priv, int enable)
{
	int ret = 0;
	unsigned char value = 0xff;

	if (!priv->i2c_client)
		return -EINVAL;

	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_COMMAND_REG_A, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		if (enable) {
			value |= 0x80;
		} else {
			value &= ~0x80;
		}

		if ((ret = smb349_i2c_write(priv->i2c_client,
				SMB349_COMMAND_REG_A, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_COMMAND_REG_A: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static int smb349_set_float_voltage(struct smb349_priv *priv, int value)
{
	int ret = -1;
	u8 reg_value = 0xff;
	int fv_value = 0;

	/* convert to decivolts from microvolts */
	value = value/10000;

	if ((value > SMB349_FLOAT_VOLTAGE_MAX) ||
		(value < SMB349_FLOAT_VOLTAGE_MIN)) {
		dev_err(priv->dev, "Invalid float voltage value: %d\n",
			value);
		return ret;
	}

	mutex_lock(&priv->lock);
	if (smb349_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	ret = smb349_i2c_read(priv->i2c_client,
		SMB349_FLOAT_VOLTAGE, &reg_value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_FLOAT_VOLTAGE: %d\n",
			__func__, ret);
		goto done;
	}

	fv_value = (value - SMB349_FLOAT_VOLTAGE_MIN) / 2;

	reg_value &= ~SMB349_FLOAT_VOLTAGE_MASK;
	reg_value |= (fv_value & SMB349_FLOAT_VOLTAGE_MASK);

	dev_info(priv->dev, "Setting float voltage to 0x%02x\n", reg_value);

	ret = smb349_i2c_write(priv->i2c_client,
		SMB349_FLOAT_VOLTAGE, reg_value);

done:
	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}
	mutex_unlock(&priv->lock);
	return ret;
}

static int smb349_otg_current_limit(struct smb349_priv *priv, int limit)
{
	int ret = -1;
	u8 value = 0xff;
	int i;

	if (smb349_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	ret = smb349_i2c_read(priv->i2c_client,
			SMB349_OTG_THERM_CONTROL, &value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_OTG_THERM_CONTROL: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	/* Locate the first smaller otg current limit */
	for (i = ARRAY_SIZE(smb349_otg_current_limits) - 1; i >= 1; i--)
		if (smb349_otg_current_limits[i] <= limit)
			break;

	dev_info(priv->dev, "Change otg_current_limit to [%d]\n", smb349_otg_current_limits[i]);

	value &= ~SMB349_OTG_CUR_LIMIT_MASK;
	value |= (i << 2);

	ret = smb349_i2c_write(priv->i2c_client,
			SMB349_OTG_THERM_CONTROL, value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to write SMB349_OTG_THERM_CONTROL: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	priv->otg_current_limit =  smb349_otg_current_limits[i];


	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}
done:
	return ret;
}

static int smb349_fast_charge_current_limit(struct smb349_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	if (smb349_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_CURRENT_LIMIT, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_CURRENT_LIMIT: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	dev_info(priv->dev, "Fast charge current limit = %d mA, "
			"Input Current Limit = %d mA",
			smb349_fast_charge_currents[value >> 4],
			smb349_input_current_limits[value & 0xf]);

	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}
done:
	return ret;
}

static int smb349_config_fixup(struct smb349_priv *priv)
{
	int ret = -1, i = 0;
#if defined(CONFIG_ARCH_MSM8974_THOR) || defined(CONFIG_ARCH_MSM8974_APOLLO)
	int fixup_values[] = {
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		0x07, 0xa3, -1,   -1,
		-1
	};
#else
	int fixup_values[] = {
		-1,     -1,   -1,   -1,
		-1,     -1, 0x7a,   0xc3,
		-1,     -1,   -1, 0xa1,
		0x57, 0xa3,   -1,   -1,
		-1
	};
#endif

	u8 value = 0xff;

	if (smb349_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	for (i = 0; i < ARRAY_SIZE(fixup_values); i++) {
		if (fixup_values[i] == -1)
			continue;

		if ((ret = smb349_i2c_read(priv->i2c_client,
				i, &value))) {
			dev_err(priv->dev,
				"%s: Unable to read from reg %02x: %d\n",
				__FUNCTION__, i, ret);
			goto done;
		}

		/* Skip write if the value matches */
		if (value == fixup_values[i])
			continue;

		dev_info(priv->dev,
			"Writing %02x to reg %02x\n",
			fixup_values[i], i);

		if ((ret = smb349_i2c_write(priv->i2c_client,
				i, fixup_values[i]))) {
			dev_err(priv->dev,
				"%s: Unable to write to reg %02x: %d\n",
				__FUNCTION__, i, ret);
			goto done;
		}
	}

	ret = 0;
done:
	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		ret = -1;
	}

	/* Make sure that APSD is still disabled */
	if (priv->apsd_disabled) {
		smb349_APSD_enable(priv, 0);
	}

	priv->charge_mode = CHARGE_MODE_NORMAL;

	/* Set the appropriate fast charge current limit */
	smb349_fast_charge_current_limit(priv);

	return ret;
}

static int smb349_enable_charger_fault(struct smb349_priv *priv, int enable,
				int chg_timeout)
{
	int ret = -1;
	u8 value = 0xff;

	if (smb349_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	ret = smb349_i2c_read(priv->i2c_client,	SMB349_ENABLE_CONTROL, &value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_ENABLE_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	if (enable)
		value |= SMB349_ENABLE_CHG_FAULT_IRQ;
	else
		value &= ~SMB349_ENABLE_CHG_FAULT_IRQ;

	ret = smb349_i2c_write(priv->i2c_client, SMB349_ENABLE_CONTROL, value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to write SMB349_ENABLE_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	ret = smb349_i2c_read(priv->i2c_client,	SMB349_STAT_TIMERS, &value);
	if (ret) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_ENABLE_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	/* Set charge timeout properly */
	value &= ~SMB349_CHG_TIMEOUT_MASK;
	value |= chg_timeout;

	ret = smb349_i2c_write(priv->i2c_client, SMB349_STAT_TIMERS, value);
	if (ret) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_STAT_TIMERS: %d\n",
				__func__, ret);
			goto done;
	}

	ret = 0;

done:
	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		ret = -1;
	}

	return ret;
}

static int smb349_switch_mode(struct smb349_priv *priv, int mode)
{
	int ret = 0;
	unsigned char value = 0xff;

	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_COMMAND_REG_B, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_COMMAND_REG_B: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		switch (mode) {
		case SMB349_USB_MODE_1:
			dev_info(priv->dev, "Switching to USB1 mode\n");
			value &= ~0x01;
			value &= ~0x02;
			break;
		case SMB349_USB_MODE_5:
			dev_info(priv->dev, "Switching to USB5 mode\n");
			value &= ~0x01;
			value |= 0x02;
			break;
		case SMB349_USB_MODE_HC:
			dev_info(priv->dev, "Switching to HC mode\n");
			value |= 0x01;
			break;
		default:
			dev_err(priv->dev, "Unknown USB mode: %d\n", mode);
			return -1;
		}

		if ((ret = smb349_i2c_write(priv->i2c_client,
				SMB349_COMMAND_REG_B, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_COMMAND_REG_B: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static void smb349_apsd_complete(struct smb349_priv *priv)
{
	u8 value = 0xff;
	int ret = -1;
	int type = POWER_SUPPLY_TYPE_USB;

	mutex_lock(&priv->lock);

	ret = smb349_i2c_read(priv->i2c_client, SMB349_STATUS_REG_D, &value);

	if (ret) {
		dev_err(priv->dev,
			"Failed to read SMB349_STATUS_REG_D: %d\n", ret);
		mutex_unlock(&priv->lock);
		return;
	}

	dev_info(priv->dev, "Detected charger: %s\n",
			smb349_apsd_result_string(value));

#ifdef CONFIG_AMAZON_METRICS_LOG
	{
		char buf[METRICS_STR_LEN];

		snprintf(buf, sizeof(buf),
		"smb349:def:chgevent=1;CT;1,connected=1;CT;1,type=%s;DV;1:NR",
			smb349_apsd_result_string(value));

		log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
	}
#endif

	switch (value) {
	case SMB349_APSD_RESULT_SDP:
	case SMB349_APSD_RESULT_CDP:
#if defined(CONFIG_PM_WAKELOCKS)
		pm_wake_lock("smb349");
#endif
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 1);
		smb349_enable_charger_fault(priv, 1, SMB349_CHG_TIMEOUT_MAX);

		if (value == SMB349_APSD_RESULT_CDP) {
			type = POWER_SUPPLY_TYPE_USB_CDP;
		} else {
			type = POWER_SUPPLY_TYPE_USB;
		}
		if (!apsd_triggered) {
			apsd_triggered = 1;
			enable_apsd_trigger(priv);
		}

		break;
	case SMB349_APSD_RESULT_DCP:
	case SMB349_APSD_RESULT_OTHER:
#if defined(CONFIG_PM_WAKELOCKS)
		/* FIXME: Do not hold wakelocks for DCP/Other endpoints for now */
		//pm_wake_lock("smb349");
#endif
		atomic_set(&priv->ac_online, 1);
		atomic_set(&priv->usb_online, 0);
		smb349_enable_charger_fault(priv, 1, SMB349_CHG_TIMEOUT_MED);

		type = POWER_SUPPLY_TYPE_USB_DCP;

		if (!apsd_triggered) {
			apsd_triggered = 1;
			enable_apsd_trigger(priv);
		}

		break;
	default:
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 0);

		break;
	}

	power_supply_changed(&priv->usb);
	power_supply_changed(&priv->ac);

	if (value == SMB349_APSD_RESULT_SDP
			|| value == SMB349_APSD_RESULT_CDP) {
		power_supply_set_present(priv->dwc3_usb, 1);
	}

	mutex_unlock(&priv->lock);

	return;
}

static void smb349_handle_charger_error(struct smb349_priv *priv, int error_id)
{
	char *error_str;
#ifdef CONFIG_AMAZON_METRICS_LOG
	char buf[METRICS_STR_LEN];
#endif

	/* Ignore errors if no cable */
	if (!atomic_read(&priv->ac_online) &&
		!atomic_read(&priv->usb_online)) {
		return;
	}

	priv->chg_health = error_id;
	power_supply_changed(&priv->usb);
	power_supply_changed(&priv->ac);

	switch (error_id) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		error_str = "overheat";
		break;
	case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
		error_str = "overvoltage";
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		error_str = "cold";
		break;
	default:
		error_str = "unknown";
		break;
	}

	printk(KERN_WARNING
		"smb349: charger error: %s, shutting down...\n", error_str);

#ifdef CONFIG_AMAZON_METRICS_LOG

	snprintf(buf, sizeof(buf),
		"smb349:def:charger_error=1;CT;1,type=%s;DV;1:NR", error_str);

	log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
#endif
}

#define SOFT_COLD_LIMIT                 15
#define HOT_HARD_LIMIT                  60
#define LOW_TEMP_CHARGE_WORK_INTERVAL   60000
#define HIGH_TEMP_CHARGE_WORK_INTERVAL  300000

static void smb349_check_temp(struct smb349_priv *priv)
{
	union power_supply_propval ret = {0,};
	int battery_temp;
	int charge_mode = CHARGE_MODE_NORMAL;
	int voltage;
	int idx;

	cancel_delayed_work(&priv->charge_work);

	/* Check battery temp */
	priv->bq27x41->get_property(priv->bq27x41,
				POWER_SUPPLY_PROP_TEMP, &ret);
	battery_temp = ret.intval / 10; /* round down */

	dev_dbg(priv->dev, "%s: begin temp=%d\n", __func__, battery_temp);

	/* Manually check the Hot limit to avoid spurious interrupts */
	if (battery_temp >= HOT_HARD_LIMIT) {
		smb349_handle_charger_error(priv,
				POWER_SUPPLY_HEALTH_OVERHEAT);
	}

	/* Do nothing if DCP is not connected */
	if (!atomic_read(&priv->ac_online))
		return;

	/* Check to see if the device is 0-15C and the current limit is high */
	if ((battery_temp <= SOFT_COLD_LIMIT)) {

		/* Check battery level */
		priv->bq27x41->get_property(priv->bq27x41,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		voltage = ret.intval;

		dev_dbg(priv->dev, "Device cold, voltage=%d\n", voltage);

		if (voltage >= FLOAT_VOLTAGE_NORMAL) {
			charge_mode = CHARGE_MODE_NORMAL;
		} else if (voltage >= FLOAT_VOLTAGE_COLD) {
			charge_mode = CHARGE_MODE_500MA_435V;
		} else if (voltage >= FLOAT_VOLTAGE_MIN) {
			charge_mode = CHARGE_MODE_900MA_435V;
		} else {
			charge_mode = CHARGE_MODE_NORMAL;
		}

		schedule_delayed_work(&priv->charge_work,
				msecs_to_jiffies(LOW_TEMP_CHARGE_WORK_INTERVAL));

		/* Battery voltage will droop when switching to the lower
		   current, make sure that we do not get into a loop.  Once
		   the device has entered the lowest current state, do not leave
		   until the device warms up or the charger is unplugged */
		if (priv->charge_mode == CHARGE_MODE_500MA_435V) {
			dev_dbg(priv->dev,
				"Voltage droop, do not increase current\n");
			return;
		}

	} else {
		/* Schedule a lower polling interval at high temp since we
		   do not always get soft cold IRQs from smb349 */
		schedule_delayed_work(&priv->charge_work,
			      msecs_to_jiffies(HIGH_TEMP_CHARGE_WORK_INTERVAL));
	}

	if (priv->charge_mode != charge_mode) {
		dev_dbg(priv->dev, "charge mode changed: %d -> %d\n",
			priv->charge_mode, charge_mode);

		switch (charge_mode) {
		case CHARGE_MODE_900MA_435V:
			dev_info(priv->dev,
				"Device cold, current limit=900mA\n");

			/* In cold we need to limit charge to .9A */
			smb349_change_current_limit(priv, 1);
			break;
		case CHARGE_MODE_500MA_435V:
			dev_info(priv->dev,
				"Device cold, current limit=500mA\n");

			/* After 4.1V we need to limit charge to .5A */
			smb349_change_current_limit(priv, 0);
			break;
		default:
			/* reset current limit */
			idx = smb349_get_current_limit_index(priv->current_limit);
			if (idx >= 0) {
				dev_info(priv->dev,
					"Change current_limit to [%d]\n",
					priv->current_limit);
				if (smb349_change_current_limit(priv, idx)) {
					dev_err(priv->dev,
						"Unable to change current limit\n");
				}
			}
			break;
		}
		priv->charge_mode = charge_mode;
		dev_dbg(priv->dev, "charge mode set: %d\n", priv->charge_mode);
	}

}

static void smb349_charge_worker(struct work_struct *work)
{

	struct smb349_priv *priv = container_of(work,
				struct smb349_priv, charge_work.work);
#ifdef SMB349_DEBUG
	int error;
	u8 status_a = 0xFF, status_e = 0xFF;

	if ((error = smb349_i2c_read(priv->i2c_client,
						SMB349_STATUS_REG_A,
						&status_a))) {
		printk(KERN_ERR "couldn't read status_a\n");
	}

	if ((error = smb349_i2c_read(priv->i2c_client,
						SMB349_STATUS_REG_E,
						&status_e))) {
		printk(KERN_ERR "couldn't read status_e\n");
	}

	dev_err(priv->dev, "charge: mode=%d status_a=0x%02x status_e=0x%02x\n",
		priv->charge_mode, status_a, status_e);
#endif

	smb349_check_temp(priv);
}

/* IRQ worker */
static void smb349_irq_worker(struct work_struct *work)
{
	u8 value = 0xff;
	int ret = -1;
	int idx = 0;
	int disconnected = 0;

	struct smb349_priv *priv = container_of(work,
				struct smb349_priv, irq_work.work);
	BUG_ON(!priv);

	dev_dbg(priv->dev, "%s: begin\n", __func__);

	/* Check interrupt status E (disconnect) register first */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_E, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_E: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
			"INTSTAT_REG_E is %02x\n", value);

		disconnected = (value & SMB349_INTSTAT_E_UV_STATUS);

		/* If the cable is not present, run disconnect routine
		   if the irq is still enabled or if we have not already
		   run the disconnect routine */
		if (((value & SMB349_INTSTAT_E_UV) |
				atomic_read(&priv->ac_online) |
				atomic_read(&priv->usb_online)) &&
			disconnected) {
			dev_info(priv->dev,
				"USB disconnected\n");
			apsd_triggered = 0;

			atomic_set(&priv->ac_online, 0);
			atomic_set(&priv->usb_online, 0);
			power_supply_changed(&priv->usb);
			power_supply_changed(&priv->ac);

			if (!priv->apsd_disabled)
				power_supply_set_present(priv->dwc3_usb, 0);

			/* newer parts go back to defaults after unplug */
			smb349_config_fixup(priv);

			/* reset current limit */
			idx = smb349_get_current_limit_index(priv->current_limit);
			if (idx >= 0) {
				dev_info(priv->dev, "Change current_limit to [%d]\n",
					priv->current_limit);
				if (smb349_change_current_limit(priv, idx)) {
					dev_err(priv->dev,
						"Unable to change input current limit\n");
				}
			}

#if defined(CONFIG_PM_WAKELOCKS)
			pm_wake_unlock("smb349");
#endif
#ifdef CONFIG_AMAZON_METRICS_LOG
			schedule_work(&priv->disconnect_log_work);
#endif
		}
	}

	/* Check interrupt status A register */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_A, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_A: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
			"INTSTAT_REG_A is %02x\n", value);

		if (value & SMB349_INTSTAT_A_COLD_HARD) {
			smb349_handle_charger_error(priv,
						POWER_SUPPLY_HEALTH_COLD);
		}

		smb349_check_temp(priv);
	}

	/* Check interrupt status B register */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_B, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_B: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
			"INTSTAT_REG_B is %02x\n", value);

		if (value & SMB349_INTSTAT_B_BATT_OV) {
			smb349_handle_charger_error(priv,
					     POWER_SUPPLY_HEALTH_OVERVOLTAGE);
		}
		if (value & SMB349_INTSTAT_B_BATT_MISSING) {
			dev_err(priv->dev,
				"SMB349: No battery, disabling charger\n");
			smb349_handle_charger_error(priv,
					  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		}
	}

	/* Check interrupt status C register */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_C, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_C: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
			"INTSTAT_REG_C is %02x\n", value);

		if (value & SMB349_INTSTAT_C_INT_TEMP) {
			smb349_handle_charger_error(priv,
						POWER_SUPPLY_HEALTH_OVERHEAT);
		}
	}

	/* Check interrupt status D register */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_D, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_D: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
			"INTSTAT_REG_D is %02x\n", value);

		/* Check for charge timeout interrupt */
		if (value & SMB349_INTSTAT_D_CHG_TIMEOUT) {
#ifdef CONFIG_AMAZON_METRICS_LOG
			char buf[METRICS_STR_LEN];
			snprintf(buf, sizeof(buf),
				"smb349:def:chg_timeout=1;CT;1:NR");

			log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
#endif
			dev_err(priv->dev,
				"SMB349: Charge timeout, disabling\n");

			power_supply_changed(&priv->usb);
			power_supply_changed(&priv->ac);
		}

		/* Don't handle charger detect IRQs if cable has already been
		   unplugged */
		if (!disconnected) {

			if (!priv->apsd_disabled) {
				/* Check for APSD status */
				if (value & SMB349_INTSTAT_D_APSD_CMPL) {
					if (!SMB349_IS_APSD_DONE(value)) {
						dev_warn(priv->dev,
							"Spurious APSD IRQ!\n");
					} else {
						smb349_apsd_complete(priv);
					}
				}
			}

			/* Check for AICL status */
			if ((value & SMB349_INTSTAT_D_AICL_CMPL) &&
				SMB349_IS_AICL_DONE(value)) {
				smb349_aicl_complete(priv);
			}
		}
	}

	/* Check interrupt status F register */
	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_F, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB349_INTSTAT_REG_F: %d\n", ret);
	} else {
		char *envp[2] = {"OTG_OVERCURRENT_IRQ=0", NULL};

#ifdef SMB349_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_F is %02x\n", value);
#endif
		/* check bit 7 of INTSTAT_REG_F, for the OTG over-current limit IRQ */
		if (value & SMB349_OVERCURRENT_IRQ) {
			sprintf(envp[0], "OTG_OVERCURRENT_IRQ=%d",
				(value & SMB349_OVERCURRENT_IRQ) !=0);
			kobject_uevent_env(&priv->dev->kobj, KOBJ_CHANGE, envp);
			dev_info(priv->dev, "%s: Send uevent %s", __func__, envp[0]);
		}
	}

	if (priv->polling_mode) {
		schedule_delayed_work(&priv->irq_work, msecs_to_jiffies(1000));
	} else if (priv->handle_irq) {
		priv->handle_irq = 0;
		enable_irq(priv->irq);
	}

	ret = smb349_i2c_read(priv->i2c_client, SMB349_INTSTAT_REG_F, &value);

	if (ret) {
		dev_warn(priv->dev,
				"Failed to read SMB349_INTSTAT_REG_F: %d\n", ret);
	} else {
		dev_dbg(priv->dev,
				"INTSTAT_REG_F is %02x\n", value);
		if (!disconnected) {
			if (!SMB349_IS_POWER_OK(value)) {
				atomic_set(&priv->ac_online, 0);
				atomic_set(&priv->usb_online, 0);
				power_supply_changed(&priv->usb);
				power_supply_changed(&priv->ac);

				if (!priv->apsd_disabled)
					power_supply_set_present(priv->dwc3_usb, 0);

				/* newer parts go back to defaults after unplug */
				smb349_config_fixup(priv);

				/* reset current limit */
				idx = smb349_get_current_limit_index(priv->current_limit);
				if (idx >= 0) {
					dev_info(priv->dev, "Change current_limt to [%d]\n",
						priv->current_limit);
					if (smb349_change_current_limit(priv, idx)) {
						dev_err(priv->dev,
							"Unable to change input current limit\n");
					}
				}

#if defined(CONFIG_PM_WAKELOCKS)
				pm_wake_unlock("smb349");
#endif
#ifdef CONFIG_AMAZON_METRICS_LOG
				schedule_work(&priv->disconnect_log_work);
#endif

			} else if (priv->apsd_disabled) {
				if (!atomic_read(&priv->ac_online)) {

					dev_info(priv->dev, "Detected charger present\n");

					atomic_set(&priv->ac_online, 1);
					atomic_set(&priv->usb_online, 0);
					power_supply_changed(&priv->usb);
					power_supply_changed(&priv->ac);
				}
			}
		} else {
			if (SMB349_IS_POWER_OK(value)) {
				smb349_apsd_complete(priv);
			}
		}
	}

	if (wake_lock_active(&priv->irq_wake_lock))
		wake_unlock(&priv->irq_wake_lock);
}

static irqreturn_t smb349_irq(int irq, void *data)
{
	struct smb349_priv *priv = (struct smb349_priv *)data;

	disable_irq_nosync(priv->irq);

	priv->handle_irq = 1;

	/* Need to wait until i2c susbsytem is resumed */
	if (!atomic_read(&priv->suspended)) {
		/* Scrub through the registers to ack any interrupts */
		schedule_delayed_work(&priv->irq_work, 0);
	} else {
		/* Make sure we stay awake until the irq has been handled */
		wake_lock(&priv->irq_wake_lock);
	}

	return IRQ_HANDLED;
}

static int smb349_i2c_read(struct i2c_client *i2c_client,
				u8 reg_num, u8 *value)
{
	struct smb349_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_read_byte_data(i2c_client, reg_num);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
		return error;
	}

	*value = (unsigned char) (error & 0xff);
	return 0;
}

static int smb349_i2c_write(struct i2c_client *i2c_client,
				u8 reg_num, u8 value)
{
	struct smb349_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_write_byte_data(i2c_client, reg_num, value);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
	}

	return error;
}

static int smb349_read_id(struct smb349_priv *priv, int *id)
{
	int error = 0;
	unsigned char value = 0xff;

	error = smb349_i2c_read(priv->i2c_client,
				SMB349_I2C_ADDR, &value);

	if (!error) {
		*id = value >> 4 ;
	}

	return error;
}

/* Enable/disable charging */
static int smb349_enable_charging(struct smb349_priv *priv, int enable)
{
	int ret = -1;
	unsigned char value;

	if (priv->chrg_en != -1) {
		/* Use EN pin to control charging */
		if (enable) {
			gpio_set_value(priv->chrg_en, 0);
		} else {
			gpio_set_value(priv->chrg_en, 1);
		}

		ret = 0;
	} else {
		/* Control charging in sw */
		if (smb349_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			return ret;
		}

		ret = smb349_i2c_read(priv->i2c_client,
				SMB349_ENABLE_CONTROL, &value);
		if (ret) {
			dev_err(priv->dev,
				"%s: Unable to read ENABLE_CONTROL: %d\n",
				__func__, ret);
			goto done;
		}

		if (enable) {
			/* Switch to pin control (pull down) */
			value |= SMB349_EN_CTRL_MASK;
		} else {
			/* Switch to i2c control (0 disables chrg) */
			value &= ~SMB349_EN_CTRL_MASK;
		}

		ret = smb349_i2c_write(priv->i2c_client,
				SMB349_ENABLE_CONTROL, value);
		if (ret) {
			dev_err(priv->dev,
				"%s: Unable to write ENABLE_CONTROL: %d\n",
				__func__, ret);
			goto done;
		}

		if (!enable) {
			ret = smb349_i2c_read(priv->i2c_client,
					SMB349_COMMAND_REG_A, &value);
			if (ret) {
				dev_err(priv->dev,
					"Couldn't read CMD reg A ret=%d\n", ret);
				goto done;
			}
			value &= ~SMB349_COMMAND_A_CHRG_EN;

			ret = smb349_i2c_write(priv->i2c_client,
					SMB349_COMMAND_REG_A, value);
			if (ret) {
				dev_err(priv->dev,
					"Couldn't disable charging ret=%d\n",
					ret);
			}
		}
	done:
		if (smb349_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
		}
	}

	return ret;
}

static enum power_supply_property smb349_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property smb349_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
};

static int smb349_enable_otg(struct smb349_priv *chip, int enable)
{
	int rc = 0;
	u8 value = 0;

	rc = smb349_i2c_read(chip->i2c_client, SMB349_COMMAND_REG_A, &value);
	if (rc) {
		dev_err(chip->dev, "Couldn't read OTG mode rc=%d\n", rc);
		return rc;
	}

	if (enable)
		value |= SMB349_COMMAND_A_OTG_EN;
	else
		value &= ~SMB349_COMMAND_A_OTG_EN;

	rc = smb349_i2c_write(chip->i2c_client, SMB349_COMMAND_REG_A, value);
	if (rc) {
		dev_err(chip->dev, "Couldn't set OTG mode rc=%d\n", rc);
		return rc;
	}

	if (enable) {
		/* Set OTG current limit */
		rc = smb349_otg_current_limit(chip, chip->otg_current_limit);
	}

	return rc;
}

#define OTG_WORK_INTERVAL   500

static void smb349_otg_worker(struct work_struct *work)
{
	int rc = 0;
	u8 value = 0;
	struct smb349_priv *chip = container_of(work,
				struct smb349_priv, otg_work.work);

	/* Check to see if smb349 has reset */
	if (chip->otg_enabled) {
		rc = smb349_i2c_read(chip->i2c_client,
				SMB349_COMMAND_REG_A, &value);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read OTG mode rc=%d\n", rc);
			return;
		}

		dev_dbg(chip->dev, "%s: value=0x%x\n", __func__, value);

		/* Fix up smb349 registers if mismatch detected */
		if (!(value & SMB349_COMMAND_A_OTG_EN)) {
			dev_info(chip->dev, "USB OTG inrush reset!\n");

			smb349_config_fixup(chip);
			smb349_enable_otg(chip, 1);
		}

		schedule_delayed_work(&chip->otg_work,
				msecs_to_jiffies(OTG_WORK_INTERVAL));
	}
}

static int smb349_chg_otg_host_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	rc = smb349_enable_otg(chip, 1);
	if (!rc) {
		chip->otg_enabled = true;

		/* HACK - workaround Saturn USB OTG inrush current bug */
		schedule_delayed_work(&chip->otg_work,
				msecs_to_jiffies(OTG_WORK_INTERVAL));
	}

	return rc;
}

static int smb349_chg_otg_host_regulator_disable(struct regulator_dev *rdev)
{
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	chip->otg_enabled = false;
	cancel_delayed_work_sync(&chip->otg_work);

	return smb349_enable_otg(chip, 0);
}

static int smb349_chg_otg_host_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	return chip->otg_enabled;
}


struct regulator_ops smb349_chg_otg_host_reg_ops = {
	.enable		= smb349_chg_otg_host_regulator_enable,
	.disable	= smb349_chg_otg_host_regulator_disable,
	.is_enabled	= smb349_chg_otg_host_regulator_is_enable,
};


static int smb349_regulator_otg_host_init(struct smb349_priv *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *of_node;

	of_node = of_get_child_by_name(chip->dev->of_node, "smb349_otg_supply");
	init_data = of_get_regulator_init_data(chip->dev, of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb349_chg_otg_host_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static int smb349_APSD_enable(struct smb349_priv *chip, int enable)
{
	int ret = -1;
        u8 value = 0;

	mutex_lock(&chip->lock);

	dev_dbg(chip->dev, "%s: begin en=%d\n", __func__, enable);

	chip->apsd_disabled = !enable;

	/* We will modify config after i2c is up */
	if (atomic_read(&chip->suspended)) {
		ret = 0;
		goto err1;
	}

        if (smb349_config(chip, 1)) {
                dev_err(chip->dev, "Unable to enable writes to CONFIG regs\n");
                goto err1;
        }

        if ((ret = smb349_i2c_read(chip->i2c_client,SMB349_FUNCTIONS, &value))) {
                dev_err(chip->dev, "%s: Unable to read SMB349_FUNCTIONS: %d\n",__FUNCTION__, ret);
                goto err0;
        }

	if (enable)
		value |= (SMB349_FUNCTIONS_APSD_EN_MASK | SMB349_FUNCTIONS_AICL_EN_MASK);
	else
		value &= ~(SMB349_FUNCTIONS_APSD_EN_MASK | SMB349_FUNCTIONS_AICL_EN_MASK);

        if ((ret = smb349_i2c_write(chip->i2c_client, SMB349_FUNCTIONS, value))) {
                dev_err(chip->dev, "%s: Unable to write SMB349_FUNCTIONS: %d\n", __FUNCTION__, ret);
                goto err0;
        }

        ret = 0;

err0:
        if (smb349_config(chip, 1))
                dev_err(chip->dev, "Unable to enable writes to CONFIG regs\n");
err1:
        mutex_unlock(&chip->lock);
        return ret;
}

static int smb349_chg_apsd_host_regulator_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	ret = smb349_APSD_enable(chip, 1);

        return ret;
}

static int smb349_chg_apsd_host_regulator_disable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	ret = smb349_APSD_enable(chip, 0);

	return ret;
}

static int smb349_chg_apsd_host_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	mutex_lock(&chip->lock);
	ret = !chip->apsd_disabled;
	mutex_unlock(&chip->lock);

	return ret;
}

static int smb349_chg_apsd_set_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	int ret = -1;
	u8 value = 0;
	struct smb349_priv *chip = rdev_get_drvdata(rdev);

	mutex_lock(&chip->lock);

	if (smb349_config(chip, 1)) {
		dev_err(chip->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	/* Re-enable AICL */
	if ((ret = smb349_i2c_read(chip->i2c_client,SMB349_FUNCTIONS, &value))) {
                dev_err(chip->dev, "%s: Unable to read SMB349_FUNCTIONS: %d\n",__FUNCTION__, ret);
                goto done;
        }

	value |= SMB349_FUNCTIONS_AICL_EN_MASK;

        if ((ret = smb349_i2c_write(chip->i2c_client, SMB349_FUNCTIONS, value))) {
                dev_err(chip->dev, "%s: Unable to write SMB349_FUNCTIONS: %d\n", __FUNCTION__, ret);
                goto done;
        }

	/* Switch to charging i2c control */
	ret = smb349_i2c_read(chip->i2c_client,
			SMB349_ENABLE_CONTROL, &value);
	if (ret) {
		dev_err(chip->dev,
			"%s: Unable to read ENABLE_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	value &= ~SMB349_USBCS_EN_CTRL_MASK;

	ret = smb349_i2c_write(chip->i2c_client,
			SMB349_ENABLE_CONTROL, value);
	if (ret) {
		dev_err(chip->dev,
			"%s: Unable to write ENABLE_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	/* Switch to High Current mode, this will retrigger AICL */
	ret = smb349_switch_mode(chip, SMB349_USB_MODE_HC);
	if (ret) {
		dev_warn(chip->dev,
			"Unable to modify charge current mode\n");
	}

	if (smb349_config(chip, 0)) {
		dev_err(chip->dev,
			"Unable to disable writes to CONFIG regs\n");
		goto done;
	}

done:
	mutex_unlock(&chip->lock);

	return ret;
}

struct regulator_ops smb349_chg_apsd_host_reg_ops = {
	.enable		= smb349_chg_apsd_host_regulator_enable,
	.disable	= smb349_chg_apsd_host_regulator_disable,
	.set_current_limit = smb349_chg_apsd_set_current_limit,
	.is_enabled	= smb349_chg_apsd_host_regulator_is_enable,
};

static int smb349_regulator_apsd_host_init(struct smb349_priv *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *of_node;

	of_node = of_get_child_by_name(chip->dev->of_node, "smb349_apsd_supply");
	init_data = of_get_regulator_init_data(chip->dev, of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->apsd_ctrl.rdesc.owner = THIS_MODULE;
		chip->apsd_ctrl.rdesc.type = REGULATOR_VOLTAGE;
		chip->apsd_ctrl.rdesc.ops = &smb349_chg_apsd_host_reg_ops;
		chip->apsd_ctrl.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = of_node;

		init_data->constraints.valid_ops_mask
			|= (REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_CURRENT);
		init_data->constraints.min_uA = 0;
		init_data->constraints.max_uA = 2400000;

		chip->apsd_ctrl.rdev = regulator_register(
						&chip->apsd_ctrl.rdesc, &cfg);
		if (IS_ERR(chip->apsd_ctrl.rdev)) {
			rc = PTR_ERR(chip->apsd_ctrl.rdev);
			chip->apsd_ctrl.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

#ifdef CONFIG_AMAZON_METRICS_LOG
static void smb349_disconnect_log_worker(struct work_struct *work)
{
	char buf[METRICS_STR_LEN];
	snprintf(buf, sizeof(buf),
		"smb349:def:chgevent=1;CT;1,disconnected=1;CT;1:NR");

	log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
}

#endif


static int smb349_get_usb_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb349_priv *priv = container_of(ps, struct smb349_priv, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->usb_online);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = priv->chg_health;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* AC property */
static int smb349_get_ac_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb349_priv *priv = container_of(ps, struct smb349_priv, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->ac_online);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = priv->chg_health;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t smb349_status_a_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, voltage = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_A, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB349_STATUS_REG_A: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB349_STATUS_REG_A = 0x%02x\n\n", value);
	len += sprintf(buf + len, "Thermal Regulation Status: %s\n",
			(value & (1 << 7)) ? "Active" : "Inactive");
        len += sprintf(buf + len, "THERM Soft Limit Regulation Status: %s\n",
			(value & (1 << 6)) ? "Active" : "Inactive");

	voltage = 3460 + (value & 0x3f) * 20;

	/* Max out at 4500 mV */
	if (voltage > 4500) voltage = 4500;

	len += sprintf(buf + len,
		"Actual Float Voltage after compensation: %d mV\n", voltage);
done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_a, S_IRUGO, smb349_status_a_show, NULL);

static inline int smb349_charging_current(struct smb349_priv *priv)
{
	int curr = -1, ret = -1;
	int state = SMB349_CHARGING_STATUS_NOT_CHARGING;
	unsigned char value = 0xff;

	/* Check actual charging status */
	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_STATUS_REG_C, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_STATUS_REG_C: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	state = SMB349_CHARGING_STATUS(value);

	switch (state) {
	case SMB349_CHARGING_STATUS_PRE_CHARGING:
		if ((ret = smb349_i2c_read(priv->i2c_client,
				SMB349_OTHER_CHARGE_CURRENT, &value))) {
			dev_err(priv->dev,
				"%s: Unable to read SMB349_OTHER_CHARGE_CURRENT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		curr = smb349_precharge_currents[(value >> 5) & 0x07];
		break;
	case SMB349_CHARGING_STATUS_FAST_CHARGING:
	case SMB349_CHARGING_STATUS_TAPER_CHARGING:
		if ((ret = smb349_i2c_read(priv->i2c_client,
				SMB349_CURRENT_LIMIT, &value))) {
			dev_err(priv->dev,
				"%s: Unable to read SMB349_CURRENT_LIMIT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		curr = smb349_fast_charge_currents[(value >> 4) & 0xf];
		break;
	default:
		curr = 0;
		break;
	}

done:
	return curr;
}

static ssize_t smb349_status_b_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_B, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB349_STATUS_REG_B: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB349_STATUS_REG_B = 0x%02x\n\n", value);

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_b, S_IRUGO, smb349_status_b_show, NULL);

static ssize_t smb349_status_c_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_C, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB349_STATUS_REG_C: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB349_STATUS_REG_C = 0x%02x\n\n", value);

	len += sprintf(buf + len, "Charging Enable/Disable: %s\n",
			(value & 0x1) ? "Enabled" : "Disabled");

	switch (SMB349_CHARGING_STATUS(value)) {
	case SMB349_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "Charging Status: Not charging\n");
		break;
	case SMB349_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "Charging Status: Pre-charging\n");
		break;
	case SMB349_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "Charging Status: Fast-charging\n");
		break;
	case SMB349_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "Charging Status: Taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "Charging Status: Unknown\n");
		break;
	}

	len += sprintf(buf + len, "Charger %s hold-off status\n",
			(value & (1 << 3)) ? "in" : "not in");

	len += sprintf(buf + len, "Vbatt %c 2.1 V\n",
			(value & (1 << 4)) ? '<' : '>');

	if (value & (1 << 5)) {
		len += sprintf(buf + len,
			"At least one charging cycle has terminated\n");
	} else {
		len += sprintf(buf + len,
			"No full charge cycle has occurred\n");
	}

	if (value & (1 << 6))
		len += sprintf(buf + len,
			"Charger has encountered an error\n");

	len += sprintf(buf + len, "Charger error %s an IRQ signal\n",
			(value & (1 << 7)) ? "asserts" : "does not assert");

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_c, S_IRUGO, smb349_status_c_show, NULL);

static ssize_t smb349_status_d_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_D, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB349_STATUS_REG_D: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB349_STATUS_REG_D = 0x%02x\n\n", value);

	len += sprintf(buf + len, "APSD result = %s\n", smb349_apsd_result_string(value));

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_d, S_IRUGO, smb349_status_d_show, NULL);

static ssize_t smb349_status_e_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_E, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB349_STATUS_REG_E: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB349_STATUS_REG_E = 0x%02x\n\n", value);
	len += sprintf(buf + len, "Suspend Mode: %s\n",
		(value & (1 << 7)) ? "Yes" : "No");

	switch (SMB349_USB5CS_MODE(value)) {
	case SMB349_HC_MODE:
		len += sprintf(buf + len, "In HC mode\n");
		break;
	case SMB349_USB1_MODE:
		len += sprintf(buf + len, "In USB1 mode\n");
		break;
	case SMB349_USB5_MODE:
		len += sprintf(buf + len, "In USB5 mode\n");
		break;
	}

	curr = smb349_aicl_current(value);

	len += sprintf(buf + len,
			"AICL Result = %d mA\n", curr);

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_e, S_IRUGO, smb349_status_e_show, NULL);

static ssize_t smb349_charge_current_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_B, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	curr = smb349_charging_current(priv);

	if (curr != -1) {
		len += sprintf(buf + len, "%d\n", curr);
	} else {
		len += sprintf(buf + len, "-1\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static int smb349_force_precharge(struct smb349_priv *priv, int flag)
{
	int ret = -1;
	unsigned char value = 0xff;

	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_COMMAND_REG_A, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB349_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		if (flag) {
			value &= ~0x40;
		} else {
			value |= 0x40;
		}

		if ((ret = smb349_i2c_write(priv->i2c_client,
				SMB349_COMMAND_REG_A, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_COMMAND_REG_A: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static int smb349_modify_charge_current(struct smb349_priv *priv,
			int fast_charge_current, int precharge_current)
{
	int ret = -1;
	unsigned char value = 0xff;

	if (fast_charge_current != -1) {
		/* Adjust fast charge current */
		if ((ret = smb349_i2c_read(priv->i2c_client,
				SMB349_CURRENT_LIMIT, &value))) {
			dev_err(priv->dev,
				"%s: Unable to read SMB349_CURRENT_LIMIT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		value &= ~0xf0;
		value |= (fast_charge_current << 4);

		if ((ret = smb349_i2c_write(priv->i2c_client,
				SMB349_CURRENT_LIMIT, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_CURRENT_LIMIT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

#ifdef CONFIG_AMAZON_METRICS_LOG
		{
			char buf[METRICS_STR_LEN];

			snprintf(buf, sizeof(buf),
			  "smb349:def:chgevent=1;CT;1,charge_current=%d;CT;1:NR",
			smb349_fast_charge_currents[value >> 4]);

			log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
		}
#endif
		ret = 0;

	} else if (precharge_current != -1) {
		/* Adjust pre-charge current */
		if ((ret = smb349_i2c_read(priv->i2c_client,
				SMB349_OTHER_CHARGE_CURRENT, &value))) {
			dev_err(priv->dev,
				"%s: Unable to read SMB349_OTHER_CHARGE_CURRENT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		value &= ~0x0f;
		value |= (precharge_current & 0xf0);

		if ((ret = smb349_i2c_write(priv->i2c_client,
				SMB349_OTHER_CHARGE_CURRENT, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB349_OTHER_CHARGE_CURRENT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static ssize_t smb349_charge_current_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int i = 0, value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (value < 0) {
		dev_err(priv->dev, "Invalid charge current %d mA\n", value);
		goto done;
	}

	/* Locate the first smaller current in fast charge current */
	for (i = ARRAY_SIZE(smb349_fast_charge_currents) - 1; i >= 0; i--)
		if (smb349_fast_charge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Disable force precharge, set fast charge current */
		if (smb349_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Enable writes to CONFIG regs\n");

		if (smb349_force_precharge(priv, 0)) {
			dev_warn(priv->dev,
				"Unable to disable force pre-charge\n");
		} else if (smb349_modify_charge_current(priv, i, -1)) {
			dev_warn(priv->dev,
				"Unable to modify fast charge current\n");
		}

		if (smb349_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Disabled writes to CONFIG regs\n");

		goto done;
	}

	/* Locate the first smaller current in precharge current */
	for (i = ARRAY_SIZE(smb349_precharge_currents) - 1; i >= 0; i--)
		if (smb349_precharge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Force precharge, set pre-charge current */
		if (smb349_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Enable writes to CONFIG regs\n");

		if (smb349_force_precharge(priv, 1)) {
			dev_warn(priv->dev,
				"Unable to force pre-charge\n");
		} else if (smb349_modify_charge_current(priv, -1, i)) {
			dev_warn(priv->dev,
				"Unable to modify pre-charge current\n");
		}

		if (smb349_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

		dev_dbg(priv->dev, "Disabled writes to CONFIG regs\n");

		goto done;
	}

	dev_warn(priv->dev,
		"Unable to find a valid charge current setting for %d mA\n",
		value);

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_current, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_charge_current_show,
			smb349_charge_current_store);

static ssize_t smb349_current_limit_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	u8 value = 0xff;
	int ret = -1;
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&priv->lock);

	if ((ret = smb349_i2c_read(priv->i2c_client, SMB349_CURRENT_LIMIT, &value))) {
		dev_err(priv->dev, "%s: Failed to read SMB349_CURRENT_LIMIT: %d\n", __FUNCTION__, ret);
		goto done;
	}

	value &= 0xf;

	if (value >= ARRAY_SIZE(smb349_input_current_limits))
		value = ARRAY_SIZE(smb349_input_current_limits) - 1;

	len += sprintf(buf + len, "%d\n", smb349_input_current_limits[value & 0xf]);

done:
	mutex_unlock(&priv->lock);

	return len;
}

static int smb349_get_current_limit_index(int limit)
{
	int idx;

	/* Locate the first smaller input current limit*/
	for (idx = ARRAY_SIZE(smb349_input_current_limits) - 1; idx >= 0; idx--)
		if (smb349_input_current_limits[idx] <= limit)
			break;

	return idx;
}

static int smb349_change_current_limit(struct smb349_priv *priv, int idx)
{
	int ret = -1;
	unsigned char value = 0xff;
	unsigned char oldvalue;

	if (smb349_config(priv, 1)) {
		dev_err(priv->dev, "Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	if ((ret = smb349_i2c_read(priv->i2c_client, SMB349_CURRENT_LIMIT, &value))) {
		dev_err(priv->dev,
				"%s: Unable to read SMB349_CURRENT_LIMIT: %d\n",
				__FUNCTION__, ret);
		goto done;
	}

	oldvalue = value & 0xf;

	/* Need to disable AICL to increase the input current */
	if (idx > oldvalue) {
		unsigned char func_value;

		/* Disable AICL */
		if ((ret = smb349_i2c_read(priv->i2c_client,SMB349_FUNCTIONS, &func_value))) {
			dev_err(priv->dev, "%s: Unable to read SMB349_FUNCTIONS: %d\n",
				__func__, ret);
			goto done;
		}

		func_value &= ~SMB349_FUNCTIONS_AICL_EN_MASK;

		if ((ret = smb349_i2c_write(priv->i2c_client, SMB349_FUNCTIONS, func_value))) {
			dev_err(priv->dev, "%s: Unable to write SMB349_FUNCTIONS: %d\n",
				__func__, ret);
			goto done;
		}
	}

	value &= ~0x0f;
	value |= idx;

	/* Adjust input current limit */
	if ((ret = smb349_i2c_write(priv->i2c_client, SMB349_CURRENT_LIMIT, value))) {
		dev_err(priv->dev,
				"%s: Unable to write SMB349_CURRENT_LIMIT: %d\n",
				__FUNCTION__, ret);
		goto done;
	}

	mdelay(200);

#ifdef CONFIG_AMAZON_METRICS_LOG
	if (oldvalue != idx) {
		char buf[METRICS_STR_LEN];

		snprintf(buf, sizeof(buf),
			"smb349:def:chgevent=1;CT;1,current_limit=%d;CT;1:NR",
			smb349_input_current_limits[value >> 4]);

		log_to_metrics(ANDROID_LOG_INFO, "charger", buf);
	}
#endif

	ret = 0;

done:
	if (smb349_config(priv, 0)) {
		dev_err(priv->dev,
			"%s: Unable to enable writes to CONFIG regs\n", __FUNCTION__);
		ret = -1;
	}

	return ret;
}

static ssize_t smb349_current_limit_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int idx = 0, value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	idx = smb349_get_current_limit_index(value);
	if (idx < 0) {
		dev_err(priv->dev, "Invalid current limit value\n");
		goto done;
	}

	if (smb349_change_current_limit(priv, idx)) {
		dev_err(priv->dev, "Unable to change input current limit\n");
		goto done;
	}

	priv->current_limit = smb349_input_current_limits[idx];
done:

	mutex_unlock(&priv->lock);

	return len;
}

static DEVICE_ATTR(current_limit, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_current_limit_show,
			smb349_current_limit_store);



static ssize_t smb349_float_voltage_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;
	int float_voltage = 0;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_FLOAT_VOLTAGE, &value))) {
		dev_err(priv->dev,
			"Error reading SMB349_FLOAT_VOLTAGE: %d\n", error);

		goto done;
	}

	value &= SMB349_FLOAT_VOLTAGE_MASK;
	float_voltage = (SMB349_FLOAT_VOLTAGE_MIN + (2 * value)) * 10000;

	len += sprintf(buf, "%d\n", float_voltage);

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb349_float_voltage_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtoul(buf, NULL, 10);

	if (smb349_set_float_voltage(priv, value)) {
		dev_err(priv->dev, "Unable to change float voltage\n");
		return len;
	}

	return len;
}

static DEVICE_ATTR(float_voltage, S_IRUGO | S_IWUSR | S_IWGRP,
				smb349_float_voltage_show,
				smb349_float_voltage_store);


static ssize_t smb349_otg_current_limit_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&priv->lock);

	len += sprintf(buf + len, "%d\n", priv->otg_current_limit);

	mutex_unlock(&priv->lock);

	return len;
}

static ssize_t smb349_otg_current_limit_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (smb349_otg_current_limit(priv, value)) {
		dev_err(priv->dev, "Unable to change otg current limit\n");
		goto done;
	}
done:
	mutex_unlock(&priv->lock);

	return len;
}

static DEVICE_ATTR(otg_current_limit, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_otg_current_limit_show,
			smb349_otg_current_limit_store);

static ssize_t smb349_charge_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_STATUS_REG_C, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x01) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb349_charge_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (value) {
		smb349_enable_charging(priv, 1);
	} else {
		smb349_enable_charging(priv, 0);
	}

	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_charge_enable_show,
			smb349_charge_enable_store);

static ssize_t smb349_suspend_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_COMMAND_REG_A, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x04) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static int smb349_suspend_mode(struct smb349_priv *priv, int enable)
{
	int ret = -1;
	unsigned char value = 0xff;

        if (!enable && gpio_is_valid(priv->chrg_susp)){
                gpio_set_value(priv->chrg_susp, 1);
                msleep(10);
	}

	if ((ret = smb349_i2c_read(priv->i2c_client,
			SMB349_COMMAND_REG_A, &value))) {
		printk("%s: Unable to read SMB349_COMMAND_REG_A: %d\n",__FUNCTION__, ret);
		goto done;
	}

	if (enable) {
		value |= 0x04;
	} else {
		value &= ~(0x04);
	}

	if ((ret = smb349_i2c_write(priv->i2c_client,
			SMB349_COMMAND_REG_A, value))) {
		printk("%s: Unable to write SMB349_COMMAND_REG_A: %d\n", __FUNCTION__, ret);
		goto done;
	}

        if (enable && gpio_is_valid(priv->chrg_susp))
                gpio_set_value(priv->chrg_susp, 0);

	ret = 0;

done:
	return ret;
}

static ssize_t smb349_suspend_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int enable = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	smb349_suspend_mode(priv, enable);

	mutex_unlock(&priv->lock);

	return len;
}

static DEVICE_ATTR(suspend_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_suspend_mode_show,
			smb349_suspend_mode_store);

static ssize_t smb349_bad_battery_show(struct device *dev,
                       struct device_attribute *attr, char *buf)
{
       struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
       ssize_t len = 0;

       mutex_lock(&priv->lock);
       len += sprintf(buf + len, "%d\n", priv->bad_battery);
       mutex_unlock(&priv->lock);
       return len;
}

static ssize_t smb349_bad_battery_store(struct device *dev,
                       struct device_attribute *attr, const char *buf, size_t len)
{
    struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
    int status = simple_strtoul(buf, NULL, 10);

    mutex_lock(&priv->lock);
    if (status)
        priv->bad_battery = 1;
    else
        priv->bad_battery = 0;
    mutex_unlock(&priv->lock);
    return len;
}

static DEVICE_ATTR(bad_battery, S_IRUGO | S_IWUSR | S_IWGRP,
                       smb349_bad_battery_show,
                       smb349_bad_battery_store);

static ssize_t smb349_precharge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x01) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(precharge_timeout, S_IRUGO,
			smb349_precharge_timeout_show, NULL);

static ssize_t smb349_complete_charge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
				SMB349_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x04) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(complete_charge_timeout, S_IRUGO,
			smb349_complete_charge_timeout_show, NULL);

static ssize_t smb349_charge_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb349_i2c_read(priv->i2c_client,
			SMB349_STATUS_REG_C, &value))) {
		len += sprintf(buf + len, "error\n");
		goto done;
	}

	switch (SMB349_CHARGING_STATUS(value)) {
	case SMB349_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "not-charging\n");
		break;
	case SMB349_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "pre-charging\n");
		break;
	case SMB349_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "fast-charging\n");
		break;
	case SMB349_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "unknown\n");
		break;
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(charge_status, S_IRUGO,
			smb349_charge_status_show, NULL);

static ssize_t smb349_therm_monitor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	error = smb349_i2c_read(priv->i2c_client,
				SMB349_THERMAL_CONTROL, &value);
	if (error) {
		len += snprintf(buf + len, 10, "-1\n");
		goto done;
	}

	if (value & SMB349_THERMAL_CONTROL_MONITOR) {
		len += snprintf(buf + len, 10, "0\n");
	} else {
		len += snprintf(buf + len, 10, "1\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb349_therm_monitor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long enable;
	int ret = -1;
	unsigned char value = 0xff;

	ret = kstrtoul(buf, 10, &enable);
	if (ret) {
		printk(KERN_ERR "%s: unable to parse value\n", __func__);
		return ret;
	}

	mutex_lock(&priv->lock);

	ret = smb349_i2c_read(priv->i2c_client,
			SMB349_THERMAL_CONTROL, &value);
	if (ret) {
		printk(KERN_ERR
			"%s: Unable to write SMB349_THERMAL_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	ret = smb349_config(priv, 1);
	if (ret) {
		goto done;
	}

	if (enable) {
		value &= ~SMB349_THERMAL_CONTROL_MONITOR;
	} else {
		value |= SMB349_THERMAL_CONTROL_MONITOR;
	}

	ret = smb349_i2c_write(priv->i2c_client,
			SMB349_THERMAL_CONTROL, value);
	if (ret) {
		printk(KERN_ERR
			"%s: Unable to write SMB349_THERMAL_CONTROL: %d\n",
			__func__, ret);
		goto done;
	}

	ret = smb349_config(priv, 0);
	if (ret) {
		goto done;
	}

	ret = len;
done:
	mutex_unlock(&priv->lock);
	return ret;
}
static DEVICE_ATTR(therm_monitor, S_IRUGO | S_IWUSR | S_IWGRP,
			smb349_therm_monitor_show,
			smb349_therm_monitor_store);

static ssize_t smb349_usb_boot_det_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;

	len += snprintf(buf + len, 10, "%d",
			(priv->usb_boot_det < 0) ?
			-1 : gpio_get_value(priv->usb_boot_det));

	return len;
}

static DEVICE_ATTR(usb_boot_det, S_IRUGO,
			smb349_usb_boot_det_show, NULL);

static ssize_t smb349_APSD_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int ret = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((ret = smb349_i2c_read(priv->i2c_client, SMB349_FUNCTIONS, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	len += sprintf(buf + len, (value & SMB349_FUNCTIONS_APSD_EN_MASK) ? "1\n" : "0\n");

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb349_APSD_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int enable = simple_strtoul(buf, NULL, 10);

	if (smb349_APSD_enable(priv, enable))
		return -1;

	return len;
}

static DEVICE_ATTR(APSD_enable, S_IRUGO | S_IWUGO,
			smb349_APSD_show,
			smb349_APSD_store);

#ifdef SMB349_DEBUG
static ssize_t smb349_config_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb349_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int ret, i = 0;
	u8 value = 0;
	ssize_t len = 0;

	/* Dump config, command and status registers */
	for (i = 0; i <= 0xe; i++) {
		ret = smb349_i2c_read(priv->i2c_client, i, &value);
		if (ret)
			return len;

		len += snprintf(buf + len, 50,
				"cfg_reg=0x%x, value=0x%x\n", i, value);
	}

	for (i = 0x30; i <= 0x33; i++) {
		ret = smb349_i2c_read(priv->i2c_client, i, &value);
		if (ret)
			return len;

		len += snprintf(buf + len, 50,
				"cmd_reg=0x%x, value=0x%x\n", i, value);
	}


	for (i = 0x35; i <= 0x3a; i++) {
		ret = smb349_i2c_read(priv->i2c_client, i, &value);
		if (ret)
			return len;

		len += snprintf(buf + len, 50,
				"int_stat_reg=0x%x, value=0x%x\n", i, value);
	}


	for (i = 0x3b; i <= 0x3f; i++) {
		ret = smb349_i2c_read(priv->i2c_client, i, &value);
		if (ret)
			return len;

		len += snprintf(buf + len, 50,
				"stats_reg=0x%x, value=0x%x\n", i, value);
	}

	return len;
}

static DEVICE_ATTR(config, S_IRUGO,
			smb349_config_show, NULL);

#endif

static struct attribute *smb349_attrs[] = {
	&dev_attr_status_a.attr,
	&dev_attr_status_b.attr,
	&dev_attr_status_c.attr,
	&dev_attr_status_d.attr,
	&dev_attr_status_e.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_current_limit.attr,
	&dev_attr_float_voltage.attr,
	&dev_attr_otg_current_limit.attr,
	&dev_attr_charge_enable.attr,
	&dev_attr_suspend_mode.attr,
	&dev_attr_complete_charge_timeout.attr,
	&dev_attr_precharge_timeout.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_therm_monitor.attr,
	&dev_attr_bad_battery.attr,
	&dev_attr_usb_boot_det.attr,
	&dev_attr_APSD_enable.attr,
#ifdef SMB349_DEBUG
	&dev_attr_config.attr,
#endif
	NULL,
};

static struct attribute_group smb349_attrs_group = {
	.attrs = smb349_attrs,
};

static int smb_parse_dt(struct smb349_priv *priv)
{
	int rc;
	struct device_node *node = priv->dev->of_node;

	if (!node) {
		dev_err(priv->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "chrg_susp-gpio", &(priv->chrg_susp));
	if (rc) {
		dev_err(priv->dev, "Invalid chrg_susp-gpio\n");
		priv->chrg_susp = -1;
	}

	return 0;
}

static int smb349_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct smb349_priv *priv = NULL;
	struct power_supply *dwc3_usb = NULL;
	struct power_supply *bq27x41 = NULL;
	int ret = 0, chip_id = 0;

	dwc3_usb = power_supply_get_by_name("usb");

	if (!dwc3_usb) {
		pr_err("USB supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto err5;
	}


	bq27x41 = power_supply_get_by_name("bq27x41");
	if (!bq27x41) {
		pr_err("Battery supply not found deferring probe\n");
		ret = -EPROBE_DEFER;
		goto err5;
	}

	if (!(priv = kzalloc(sizeof(*priv), GFP_KERNEL))) {
		pr_err("Out of memory\n");
		ret = -ENOMEM;
		goto err5;
	}

	/* Set up I2C structs */
	priv->i2c_client = client;
	i2c_set_clientdata(client, priv);

	/* Set up mutex */
	mutex_init(&priv->lock);

	/* Set up dev pointer */
	priv->dev = &client->dev;

	ret = smb_parse_dt(priv);
	if (ret) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", ret);
		goto err4;
	}

	if (gpio_is_valid(priv->chrg_susp)){
		ret = gpio_request(priv->chrg_susp, "smb349_chrg_susp");
		if (ret) {
			dev_err(&client->dev,
				"gpio_request for chrg_susp-gpio %d failed rc=%d\n",
				priv->chrg_susp, ret);
			goto err4;
		}
		gpio_direction_output(priv->chrg_susp, 1);
		msleep(10);
	}

	/* Check for device ID */
	if (smb349_read_id(priv, &chip_id) < 0) {
		pr_err("Unable to detect device ID\n");
		ret = -ENODEV;
		goto err3;
	}

	dev_info(priv->dev, "SMB349 detected, addr=0x%02x chip_id=0x%0x\n",
		 client->addr, chip_id);

	priv->dwc3_usb = dwc3_usb;
	priv->bq27x41 = bq27x41;
	priv->charge_mode = CHARGE_MODE_NORMAL;

	/* Set up and register the power_supply structs */
	priv->ac.name = "smb349_ac";
	priv->ac.type = POWER_SUPPLY_TYPE_MAINS;
	priv->ac.get_property = smb349_get_ac_property;
	priv->ac.properties = smb349_charger_props;
	priv->ac.num_properties = ARRAY_SIZE(smb349_charger_props);

	priv->usb.name = "smb349_usb";
	priv->usb.type = POWER_SUPPLY_TYPE_USB;
	priv->usb.get_property = smb349_get_usb_property;
	priv->usb.properties = smb349_usb_props;
	priv->usb.num_properties = ARRAY_SIZE(smb349_usb_props);
	if ((ret = power_supply_register(&client->dev, &priv->ac))) {
		dev_err(priv->dev,
			"failed to register ac power supply: %d\n", ret);
		goto err3;
	}

	if ((ret = power_supply_register(&client->dev, &priv->usb))) {
		dev_err(priv->dev,
			"failed to register usb power supply: %d\n", ret);
		goto err2;
	}

	/* Register the sysfs nodes */
	if ((ret = sysfs_create_group(&priv->dev->kobj,
					&smb349_attrs_group))) {
		dev_err(priv->dev, "Unable to create sysfs group\n");
		goto err1;
	}

	priv->otg_current_limit = SMB349_DEFAULT_CUR_LIMIT;

	/* Fixup any registers we want to */
	smb349_config_fixup(priv);

	ret = smb349_regulator_otg_host_init(priv);
	if  (ret) {
		dev_err(priv->dev,"Couldn't initialize smb349 regulator for USB host mode. rc=%d\n", ret);
		goto err1;
	}
	ret = smb349_regulator_apsd_host_init(priv);
	if  (ret) {
		dev_err(priv->dev, "Couldn't initialize smb349 regulator for APSD contral. rc=%d\n", ret);
		goto err1;
	}

	priv->current_limit = smb349_input_current_limits[ARRAY_SIZE(smb349_input_current_limits) - 1];

	/* Init work */
	INIT_DELAYED_WORK(&priv->irq_work, smb349_irq_worker);
	INIT_DELAYED_WORK(&priv->charge_work, smb349_charge_worker);
	INIT_DELAYED_WORK(&priv->otg_work, smb349_otg_worker);
	wake_lock_init(&priv->irq_wake_lock, WAKE_LOCK_SUSPEND,
			"smb349_irq");

#ifdef CONFIG_AMAZON_METRICS_LOG
	INIT_WORK(&priv->disconnect_log_work, smb349_disconnect_log_worker);
#endif

	/* Get the charger stat IRQ */
	priv->chrg_stat = of_get_named_gpio_flags(priv->dev->of_node,
				"summit,smb349-chrg-stat-gpio", 0, NULL);

	/* Get the charger EN pin */
	priv->chrg_en = of_get_named_gpio_flags(priv->dev->of_node,
				"summit,smb349-chrg-en-gpio", 0, NULL);

	if (!gpio_is_valid(priv->chrg_en)) {
		dev_warn(priv->dev,
			"Unable to determine charge enable GPIO\n");
		priv->chrg_en = -1;
	} else {
		gpio_request(priv->chrg_en, "chrg_en");
		gpio_direction_output(priv->chrg_en, 0);
	}

	priv->apsd_trigger = of_get_named_gpio_flags(priv->dev->of_node, "summit,apsd-trigger", 0, NULL);
	if (!gpio_is_valid(priv->apsd_trigger)) {
		dev_warn(priv->dev, "Unable to determine waveform GPIO\n");
		priv->apsd_trigger = -1;
	} else {
		gpio_request(priv->apsd_trigger, "apsd_trigger");
		gpio_direction_output(priv->apsd_trigger, 0);
	}

	priv->chrg_hcs = of_get_named_gpio_flags(priv->dev->of_node,
				"summit,smb349-chrg-hcs-gpio", 0, NULL);

	if (!gpio_is_valid(priv->chrg_hcs)) {
		dev_warn(priv->dev,
			"Unable to determine charge HCS GPIO\n");
		priv->chrg_hcs = -1;
	} else {
		gpio_request(priv->chrg_hcs, "chrg_hcs");
		gpio_direction_output(priv->chrg_hcs, 0);
	}

	priv->usb_boot_det = of_get_named_gpio_flags(priv->dev->of_node,
				"summit,smb349-usb-boot-gpio", 0, NULL);

	if (!gpio_is_valid(priv->usb_boot_det)) {
		dev_warn(priv->dev,
			"Unable to determine USB Boot GPIO\n");
		priv->usb_boot_det = -1;
	} else {
		gpio_request(priv->usb_boot_det, "usb_boot_detect");
		gpio_direction_input(priv->usb_boot_det);
	}

	priv->chg_health = POWER_SUPPLY_HEALTH_GOOD;

	if (!gpio_is_valid(priv->chrg_stat))
		priv->polling_mode = 1;

	if (priv->polling_mode) {
		/* Scrub through the registers to ack any interrupts */
		smb349_irq_worker(&priv->irq_work.work);
	} else {
		gpio_request(priv->chrg_stat, "chrg_stat");
		gpio_direction_input(priv->chrg_stat);

		priv->irq = gpio_to_irq(priv->chrg_stat);

		if (request_irq(priv->irq,
				smb349_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb349_irq", priv)) {
			dev_err(priv->dev, "Unable to set up threaded IRQ\n");
			goto err0;
		}

		enable_irq_wake(priv->irq);  /* this will make the irq wakeupable */

		/* Check APSD status */
		smb349_apsd_complete(priv);
	}

	return 0;

err0:
	sysfs_remove_group(&priv->dev->kobj, &smb349_attrs_group);
err1:
	power_supply_unregister(&priv->usb);
err2:
	power_supply_unregister(&priv->ac);
err3:
	if (gpio_is_valid(priv->chrg_susp))
		gpio_free(priv->chrg_susp);
err4:
	i2c_set_clientdata(client, NULL);
	kfree(priv);
err5:
	return ret;
}

static int smb349_remove(struct i2c_client *client)
{
	struct smb349_priv *priv = i2c_get_clientdata(client);

	/* Disable suspend mode */
	smb349_suspend_mode(priv, 0);

	/* Reset charge current to maximum */
	smb349_config(priv, 1);
	smb349_force_precharge(priv, 0);
	smb349_modify_charge_current(priv,
		ARRAY_SIZE(smb349_fast_charge_currents) - 1, -1);
	smb349_config(priv, 0);

	/* Reset input current limit to maximum */
	smb349_change_current_limit(priv,
		ARRAY_SIZE(smb349_input_current_limits) - 1);

	/* Free IRQ and stop any pending IRQ work */
	if (priv->polling_mode == 0)
		free_irq(priv->irq, priv);

	cancel_delayed_work_sync(&priv->irq_work);
	cancel_delayed_work_sync(&priv->charge_work);
	cancel_delayed_work_sync(&priv->otg_work);

	wake_lock_destroy(&priv->irq_wake_lock);

	sysfs_remove_group(&priv->dev->kobj, &smb349_attrs_group);

	power_supply_unregister(&priv->usb);
	power_supply_unregister(&priv->ac);

	i2c_set_clientdata(client, NULL);

	kfree(priv);

	return 0;
}

static void smb349_shutdown(struct i2c_client *client)
{
	struct smb349_priv *priv = i2c_get_clientdata(client);

	dev_info(priv->dev, "shutdown\n");

	if (priv->otg_vreg.rdev)
		smb349_chg_otg_host_regulator_disable(priv->otg_vreg.rdev);

	if (priv->apsd_ctrl.rdev &&
		smb349_chg_apsd_host_regulator_is_enable(priv->apsd_ctrl.rdev))
		smb349_chg_apsd_host_regulator_disable(priv->apsd_ctrl.rdev);

	/* Trigger charge mode if cable is plugged in */
	if (atomic_read(&priv->ac_online) ||
		atomic_read(&priv->usb_online)) {
		dev_info(priv->dev, "Going to charge mode\n");
		qpnp_pon_charge_mode_enable();
	}

	smb349_remove(client);

	return;
}

static int smb349_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct smb349_priv *priv = i2c_get_clientdata(client);

	dev_dbg(priv->dev, "Entering suspend, event = 0x%04x\n", mesg.event);

	if (priv->handle_irq) {
		dev_info(priv->dev, "%s: Pending interrupt handler, exiting\n",
			__func__);
		return -EBUSY;
	}

	cancel_delayed_work_sync(&priv->irq_work);

	/* Don't suspend chip if charger is attached (to handle interrupts) */
	if (gpio_is_valid(priv->chrg_susp) && !atomic_read(&priv->ac_online)) {
		dev_dbg(priv->dev, "Suspending smb349\n");
		gpio_set_value(priv->chrg_susp, 0);
	}

	atomic_inc(&priv->suspended);

	dev_dbg(priv->dev, "Finishing suspend\n");

	return 0;
}

static int smb349_resume(struct i2c_client *client)
{
	struct smb349_priv *priv = i2c_get_clientdata(client);

	atomic_dec(&priv->suspended);

        if (gpio_is_valid(priv->chrg_susp)) {
		gpio_set_value(priv->chrg_susp, 1);
		msleep(10);
	}

	if (priv->apsd_disabled) {
		smb349_APSD_enable(priv, 0);
	}

	if (priv->polling_mode)
		schedule_delayed_work(&priv->irq_work, msecs_to_jiffies(1000));
	else if (priv->handle_irq || atomic_read(&priv->ac_online))
		schedule_delayed_work(&priv->irq_work, 0);

	dev_dbg(priv->dev, "Finishing resume\n");

	return 0;
}

static unsigned short normal_i2c[] = { SMB349_I2C_ADDRESS, I2C_CLIENT_END };

static const struct of_device_id smb349_match[] = {
	{ .compatible = "qcom,smb349-charger", },
	{ .compatible = "summit,smb349", },
	{ },
};

static struct i2c_driver smb349_i2c_driver = {
	.driver = {
		.name = "smb349",
		.of_match_table = smb349_match,
	},
	.probe = smb349_probe,
	.remove = smb349_remove,
	.id_table = smb349_id,
	.address_list = normal_i2c,
	.suspend = smb349_suspend,
	.resume = smb349_resume,
	.shutdown = smb349_shutdown,
};

static int __init smb349_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Summit SMB349 Driver\n");

	if ((ret = i2c_add_driver(&smb349_i2c_driver))) {
		printk(KERN_ERR "smb349: Could not add driver\n");
		return ret;
	}

	printk(KERN_INFO "SMB349 Driver added\n");

	return 0;
}

static void __exit smb349_exit(void)
{
	i2c_del_driver(&smb349_i2c_driver);
}

module_init(smb349_init);
module_exit(smb349_exit);

MODULE_DESCRIPTION("Summit SMB349 Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
