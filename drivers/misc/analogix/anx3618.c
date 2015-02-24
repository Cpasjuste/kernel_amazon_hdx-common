/*
 * ANX3618 driver
 *
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

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include "slimport_tx_drv.h"
#include "sp_tx_reg.h"
#include "slimport.h"
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/dma-mapping.h>

#define EDID_VIDEO_INPUT 0x14

/*
 * HDCP switch for external block
 * external_block_en = 1: enable, 0: disable
 */
int external_block_en = 1;

/* to access global platform data */
static struct anx3618_platform_data *g_pdata;

/* For Slimport swing&pre-emphasis test */
unchar val_SP_TX_LT_CTRL_REG0;
unchar val_SP_TX_LT_CTRL_REG2;
unchar val_SP_TX_LT_CTRL_REG1;
unchar val_SP_TX_LT_CTRL_REG6;
unchar val_SP_TX_LT_CTRL_REG16;
unchar val_SP_TX_LT_CTRL_REG5;
unchar val_SP_TX_LT_CTRL_REG8;
unchar val_SP_TX_LT_CTRL_REG15;
unchar val_SP_TX_LT_CTRL_REG18;

#define TRUE 1
#define FALSE 0

#define CHARGE_HIGH_CURRENT_LIMIT 2000000

struct i2c_client *anx3618_client;

struct anx3618_platform_data {
	int gpio_p_dwn;
	int gpio_reset;
	int gpio_cable_det;
	int gpio_v10_ctrl;
	int gpio_v33_ctrl;
	int external_ldo_control;
	struct regulator *avdd_10;
	struct regulator *dvdd;
	struct regulator *apsd_regulator;
	struct regulator *id_pullup;
	spinlock_t lock;
};

struct anx3618_data {
	struct anx3618_platform_data    *pdata;
	struct delayed_work    work;
	struct work_struct    cable_detect_work;
	struct workqueue_struct    *workqueue;
	struct mutex    lock;
	struct wake_lock slimport_lock;
	int regnum;
	int regval;
};

bool slimport_is_connected(void)
{
	struct anx3618_platform_data *pdata = NULL;
	bool result = false;

	if (!anx3618_client)
		return false;

#ifdef CONFIG_OF
	pdata = g_pdata;
#else
	pdata = anx3618_client->dev.platform_data;
#endif

	if (!pdata)
		return false;

	if (gpio_get_value_cansleep(pdata->gpio_cable_det)) {
		mdelay(10);
		if (gpio_get_value_cansleep(pdata->gpio_cable_det)) {
			pr_info("%s %s : Slimport Dongle is detected\n",
					LOG_TAG, __func__);
			result = true;
		}
	}

	return result;
}
EXPORT_SYMBOL(slimport_is_connected);

/*sysfs  interface : HDCP switch for VGA dongle*/
static ssize_t sp_external_block_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", external_block_en);
}

static ssize_t sp_external_block_store(struct device *dev, struct device_attribute *attr,
		 const char *buf,  size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	external_block_en = val;
	return count;
}

/*
 * anx7730 addr id:
 *         DP_rx(0x50:0, 0x8c:1) HDMI_tx(0x72:5, 0x7a:6, 0x70:7)
 *
 *         ex:read ) 05df   = read:0  id:5 reg:0xdf
 *         ex:write) 15df5f = write:1 id:5 reg:0xdf val:0x5f
 */
static ssize_t anx7730_write_reg_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret = 0;
	char op, i;
	char r[3];
	char v[3];
	unchar tmp;
	int id, reg, val = 0;
	unchar sp_state = sp_tx_cur_states();

	if (!(sp_state >= STATE_WAITTING_CABLE_PLUG &&
			sp_state <= STATE_PLAY_BACK)) {
		dev_err(dev, "Not a valid state\n");
	}

	if (sp_tx_cur_cable_type() != DWN_STRM_IS_HDMI_7730) {
		dev_err(dev, "rx is not anx7730\n");
		return -EINVAL;
	}

	if (count != 7 && count != 5) {
		dev_err(dev, "cnt:%d, invalid input!\n", count-1);
		dev_err(dev, "ex) 05df   -> op:0(read)  id:5 reg:0xdf\n");
		dev_err(dev, "ex) 15df5f -> op:1(wirte) id:5 reg:0xdf val:0x5f\n");
		return -EINVAL;
	}

	ret = snprintf(&op, 2, buf);
	ret = snprintf(&i, 2, buf+1);
	ret = snprintf(r, 3, buf+2);

	id = simple_strtoul(&i, NULL, 10);
	reg = simple_strtoul(r, NULL, 16);

	if ((id != 0 && id != 1 && id != 5 && id != 6 && id != 7)) {
		dev_err(dev, "invalid addr id! (id:0,1,5,6,7)\n");
		return -EINVAL;
	}

	switch (op) {
	case 0x30: /* "0" -> read */
		i2c_master_read_reg(id, reg, &tmp);
		dev_warn(dev, "anx7730 read(%d,0x%x)= 0x%x\n", id, reg, tmp);
		break;

	case 0x31: /* "1" -> write */
		ret = snprintf(v, 3, buf+4);
		val = simple_strtoul(v, NULL, 16);

		i2c_master_write_reg(id, reg, val);
		i2c_master_read_reg(id, reg, &tmp);
		dev_warn(dev, "anx7730 write(%d,0x%x,0x%x)\n", id, reg, tmp);
		break;

	default:
		dev_warn(dev, "invalid operation code! (0:read, 1:write)\n");
		return -EINVAL;
	}

	return count;
}

/*
 * anx3618 addr id:
 *         HDMI_rx(0x7e:0, 0x80:1) DP_tx(0x72:5, 0x7a:6, 0x70:7)
 *
 *         ex:read ) 05df   = read:0  id:5 reg:0xdf
 *         ex:write) 15df5f = write:1 id:5 reg:0xdf val:0x5f
 */
static ssize_t anx3618_id_change(int id)
{
	int chg_id = 0;

	switch (id) {
	case 0:
		chg_id = RX_P0;
		break;
	case 1:
		chg_id = RX_P1;
		break;
	case 5:
		chg_id = TX_P2;
		break;
	case 6:
		chg_id = TX_P1;
		break;
	case 7:
		chg_id = TX_P0;
		break;
	}
	return chg_id;
}

static ssize_t anx3618_reg_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct anx3618_data *anx = dev_get_drvdata(dev);
	int ret = 0;
	char op, i;
	char r[3];
	char v[3];
	unchar tmp;
	int id, reg, val = 0;
	unchar sp_state = sp_tx_cur_states();

	if (!(sp_state >= STATE_WAITTING_CABLE_PLUG &&
			sp_state <= STATE_PLAY_BACK)) {
		dev_err(dev, "Not a valid state\n");
	}

	if (count != 7 && count != 5) {
		dev_err(dev, "cnt:%d, invalid input!\n", count-1);
		dev_err(dev, "ex) 05df   -> op:0(read)  id:5 reg:0xdf\n");
		dev_err(dev, "ex) 15df5f -> op:1(wirte) id:5 reg:0xdf val:0x5f\n");
		return -EINVAL;
	}

	ret = snprintf(&op, 2, buf);
	ret = snprintf(&i, 2, buf+1);
	ret = snprintf(r, 3, buf+2);

	id = simple_strtoul(&i, NULL, 10);
	reg = simple_strtoul(r, NULL, 16);

	if ((id != 0 && id != 1 && id != 5 && id != 6 && id != 7)) {
		dev_err(dev, "invalid addr id! (id:0,1,5,6,7)\n");
		return -EINVAL;
	}

	id = anx3618_id_change(id); /* ex) 5 -> 0x72 */

	switch (op) {
	case 0x30: /* "0" -> read */
		sp_read_reg(id, reg, &tmp);
		anx->regnum = reg;
		anx->regval = (int)tmp;
		dev_dbg(dev, "anx3618 read(0x%x,0x%x)= 0x%x\n", id, reg, tmp);
		break;

	case 0x31: /* "1" -> write */
		ret = snprintf(v, 3, buf+4);
		val = simple_strtoul(v, NULL, 16);

		sp_write_reg(id, reg, val);
		sp_read_reg(id, reg, &tmp);
		dev_dbg(dev, "anx3618 write(0x%x,0x%x,0x%x)\n", id, reg, tmp);
		break;

	default:
		dev_warn(dev, "invalid operation code! (0:read, 1:write)\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t anx3618_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct anx3618_data *anx = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%02x: %02x\n", anx->regnum, anx->regval);
}

/* For Slimport swing&pre-emphasis test */
static ssize_t ctrl_reg0_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG0);
}

static ssize_t ctrl_reg0_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG0 = val;
	return count;
}

static ssize_t ctrl_reg2_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG2);
}

static ssize_t ctrl_reg2_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG2 = val;
	return count;
}

static ssize_t ctrl_reg1_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG1);
}

static ssize_t ctrl_reg1_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG1 = val;
	return count;
}

static ssize_t ctrl_reg6_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG6);
}

static ssize_t ctrl_reg6_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG6 = val;
	return count;
}

static ssize_t ctrl_reg16_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG16);
}

static ssize_t ctrl_reg16_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG16 = val;
	return count;
}

static ssize_t ctrl_reg5_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG5);
}

static ssize_t ctrl_reg5_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG5 = val;
	return count;
}

static ssize_t ctrl_reg8_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG8);
}

static ssize_t ctrl_reg8_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG8 = val;
	return count;
}

static ssize_t ctrl_reg15_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG15);
}

static ssize_t  ctrl_reg15_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG15 = val;
	return count;
}

static ssize_t  ctrl_reg18_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", val_SP_TX_LT_CTRL_REG18);
}

static ssize_t ctrl_reg18_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG18 = val;
	return count;
}

/* for debugging */
static struct device_attribute slimport_device_attrs[] = {
	__ATTR(hdcp_switch, S_IRUGO | S_IWUSR, sp_external_block_show, sp_external_block_store),
	__ATTR(anx7730, S_IWUSR, NULL, anx7730_write_reg_store),
	__ATTR(anx3618, S_IRUGO | S_IWUSR, anx3618_reg_show, anx3618_reg_store),
	/* slimport test */
	__ATTR(ctrl_reg0, S_IRUGO | S_IWUSR, ctrl_reg0_show, ctrl_reg0_store),
	__ATTR(ctrl_reg2, S_IRUGO | S_IWUSR, ctrl_reg2_show, ctrl_reg2_store),
	__ATTR(ctrl_reg1, S_IRUGO | S_IWUSR, ctrl_reg1_show, ctrl_reg1_store),
	__ATTR(ctrl_reg6, S_IRUGO | S_IWUSR, ctrl_reg6_show, ctrl_reg6_store),
	__ATTR(ctrl_reg16, S_IRUGO | S_IWUSR, ctrl_reg16_show, ctrl_reg16_store),
	__ATTR(ctrl_reg5, S_IRUGO | S_IWUSR, ctrl_reg5_show, ctrl_reg5_store),
	__ATTR(ctrl_reg8, S_IRUGO | S_IWUSR, ctrl_reg8_show, ctrl_reg8_store),
	__ATTR(ctrl_reg15, S_IRUGO | S_IWUSR, ctrl_reg15_show, ctrl_reg15_store),
	__ATTR(ctrl_reg18, S_IRUGO | S_IWUSR, ctrl_reg18_show, ctrl_reg18_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		if (device_create_file(dev, &slimport_device_attrs[i]))
			goto error;
	return 0;
error:
	for (; i >= 0; i--)
		device_remove_file(dev, &slimport_device_attrs[i]);
	pr_err("%s %s: Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}


static int anx3618_init_gpio(struct anx3618_data *anx3618)
{
	int ret = 0;

	/*  gpio for chip power down  */
	ret = gpio_request(anx3618->pdata->gpio_p_dwn, "anx3618_p_dwn_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				anx3618->pdata->gpio_p_dwn);
		goto err0;
	}
	gpio_direction_output(anx3618->pdata->gpio_p_dwn, 1);
	/*  gpio for chip reset  */
	ret = gpio_request(anx3618->pdata->gpio_reset, "anx3618_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				anx3618->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(anx3618->pdata->gpio_reset, 0);
	/*  gpio for slimport cable detect  */
	ret = gpio_request(anx3618->pdata->gpio_cable_det, "anx3618_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				anx3618->pdata->gpio_cable_det);
		goto err2;
	}
	gpio_direction_input(anx3618->pdata->gpio_cable_det);


	gpio_export(anx3618->pdata->gpio_p_dwn, 0);
	gpio_export(anx3618->pdata->gpio_reset, 0);
	gpio_export(anx3618->pdata->gpio_cable_det, 0);

	/*  gpios for power control */
	if (anx3618->pdata->external_ldo_control) {
		/* V10 power control */
		ret = gpio_request(anx3618->pdata->gpio_v10_ctrl,
							"anx3618_v10_ctrl");
			if (ret) {
				pr_err("%s : failed to request gpio %d\n",
						__func__,
						anx3618->pdata->gpio_v10_ctrl);
			goto err3;
		}
		gpio_direction_output(anx3618->pdata->gpio_v10_ctrl, 0);
		/* V33 power control */
		ret = gpio_request(anx3618->pdata->gpio_v33_ctrl,
							"anx3618_v33_ctrl");
			if (ret) {
				pr_err("%s : failed to request gpio %d\n",
						__func__,
						anx3618->pdata->gpio_v33_ctrl);
			goto err4;
		}
		gpio_direction_output(anx3618->pdata->gpio_v33_ctrl, 1);
		gpio_export(anx3618->pdata->gpio_v10_ctrl, 0);
		gpio_export(anx3618->pdata->gpio_v33_ctrl, 0);

	}

	goto out;

err4:
	gpio_free(anx3618->pdata->gpio_v33_ctrl);
err3:
	gpio_free(anx3618->pdata->gpio_v10_ctrl);
err2:
	gpio_free(anx3618->pdata->gpio_cable_det);
err1:
	gpio_free(anx3618->pdata->gpio_reset);
err0:
	gpio_free(anx3618->pdata->gpio_p_dwn);
out:
	return ret;
}

unsigned char sp_read_reg(unsigned char slave_addr, unsigned char offset, unsigned char *buf)
{
	unsigned char ret = 0;

	anx3618_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx3618_client, offset);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
			__func__, slave_addr);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

unsigned char sp_write_reg(unsigned char slave_addr, unsigned char offset, unsigned char value)
{
	unsigned char ret = 0;

	anx3618_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx3618_client, offset, value);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
			__func__, slave_addr);
	}
	return ret;
}

int  slimport_system_init (struct anx3618_platform_data *pdata);
void slimport_work_func(struct work_struct * work);

int get_slimport_hdcp_status(void)
{
	int hdcp_en = edid_blocks[EDID_VIDEO_INPUT] >> 7;
	debug_printf("HDCP enabled: %d\n", hdcp_en);
	return hdcp_en;
}

void sp_tx_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct anx3618_platform_data *pdata = g_pdata;
#else
	struct anx3618_platform_data *pdata = anx3618_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	msleep(1);
	/* Needed for Saturn */
	if (regulator_enable(pdata->dvdd))
		pr_err("failed to enable regulator\n");
	msleep(1);

	if (pdata->external_ldo_control) {
		/* Enable 1.0V LDO */
		gpio_set_value(pdata->gpio_v10_ctrl, 1);
		msleep(1);
	}
	gpio_set_value(pdata->gpio_p_dwn, 0);
	msleep(1);
	gpio_set_value(pdata->gpio_reset, 1);
}

void sp_tx_hardware_powerdown(void)
{

#ifdef CONFIG_OF
	struct anx3618_platform_data *pdata = g_pdata;
#else
	struct anx3618_platform_data *pdata = anx3618_client->dev.platform_data;
#endif
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(1);
	/* Needed for Saturn */
	if (regulator_disable(pdata->dvdd))
		pr_err("failed to disable regulator\n");
	msleep(1);
	if (pdata->external_ldo_control) {
		gpio_set_value(pdata->gpio_v10_ctrl, 0);
		msleep(2);
	}
	gpio_set_value(pdata->gpio_p_dwn, 1);
	msleep(1);
}

int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, edid_blocks, 128 * sizeof(char));
	} else if (block == 1) {
		memcpy(edid_buf, (edid_blocks + 128), 128 * sizeof(char));
	} else {
		pr_err("%s %s: block number %d is invalid\n",
			   LOG_TAG, __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

bool slimport_link_ready(void)
{
	unchar sp_tx_state = sp_tx_cur_states();
	return (sp_tx_state >= STATE_HDCP_AUTH);
}
EXPORT_SYMBOL(slimport_link_ready);

static void anx3618_free_gpio(struct anx3618_data *anx3618)
{
	gpio_free(anx3618->pdata->gpio_cable_det);
	gpio_free(anx3618->pdata->gpio_reset);
	gpio_free(anx3618->pdata->gpio_p_dwn);
	if (anx3618->pdata->external_ldo_control) {
		gpio_free(anx3618->pdata->gpio_v10_ctrl);
		gpio_free(anx3618->pdata->gpio_v33_ctrl);
	}
}

unchar sp_get_link_bw(void)
{
	return sp_tx_cur_bw();
}
EXPORT_SYMBOL(sp_get_link_bw);

void sp_set_link_bw(unchar link_bw)
{
	sp_tx_set_bw(link_bw);
}
EXPORT_SYMBOL(sp_set_link_bw);
unsigned char sp_get_ds_cable_type(void)
{
	return  sp_tx_cur_cable_type();
}
EXPORT_SYMBOL(sp_get_ds_cable_type);

#ifdef CONFIG_SLIMPORT_FAST_CHARGE
enum CHARGING_STATUS sp_get_ds_charge_type(void)
{
	/*
	0x02: fast charge
	0x01: slow charge
	0x00: no charge
	*/
	return downstream_charging_status;
}
EXPORT_SYMBOL(sp_get_ds_charge_type);
#endif

int sp_set_ds_charge_type(enum CHARGING_STATUS status)
{
	struct anx3618_platform_data *pdata = NULL;
	int ret = 0;

#ifdef CONFIG_OF
	pdata = g_pdata;
#else
	pdata = anx3618_client->dev.platform_data;
#endif
	if (!pdata)
		return -1;

	if (!IS_ERR(pdata->apsd_regulator)) {
		debug_printf("Setting high current\n");
		ret = regulator_set_current_limit(
			pdata->apsd_regulator, 0, CHARGE_HIGH_CURRENT_LIMIT);
		if (ret < 0)
			dev_err(&anx3618_client->dev,
				"apsd_regulator set current limit error: %d\n",
				ret);
	}

	return ret;
}

static void cable_detect_work_func(struct work_struct *work)
{
	struct anx3618_data *anx3618 = container_of(work,
						struct anx3618_data,
						cable_detect_work);
	int ret;

	if (gpio_get_value(anx3618->pdata->gpio_cable_det)) {
		wake_lock(&anx3618->slimport_lock);

		debug_printf("cable got connected\n");

		/* Disable charger APSD */
		if (!IS_ERR(anx3618->pdata->apsd_regulator)) {
			debug_printf("slimport plug in, disable apsd_regulator");
			ret = regulator_disable(anx3618->pdata->apsd_regulator);
			if (ret < 0)
				dev_err(&anx3618_client->dev,
					"disable apsd_regulator error\n");
		}

		queue_delayed_work(anx3618->workqueue, &anx3618->work, 0);

	} else {
		debug_printf("cable got disconnected\n");

		sp_tx_clean_state_machine();
		cancel_delayed_work_sync(&anx3618->work);
		system_power_down();

		/* Re-enable charger APSD */
		if (!IS_ERR(anx3618->pdata->apsd_regulator)) {
			debug_printf("slimport remove, enable apsd_regulator");
			ret = regulator_enable(anx3618->pdata->apsd_regulator);
			if (ret < 0)
				dev_err(&anx3618_client->dev,
					"enable apsd_regulator error\n");
		}

		wake_unlock(&anx3618->slimport_lock);
	}
}

static irqreturn_t cable_detect_isr(int irq,void *data)
{
	struct anx3618_data *anx3618 = data;

	queue_work(anx3618->workqueue, &anx3618->cable_detect_work);

	return IRQ_HANDLED;
}

static int anx3618_system_init(void)
{
	int ret = 0;

	ret = slimport_chip_detect();
	if (ret == 0) {
		hardware_power_ctl(0);
		pr_err("%s : failed to detect anx3618\n", __func__);
		return -ENODEV;
	}

	slimport_chip_initial();
	return 0;
}

#ifdef CONFIG_OF
int anx3618_regulator_configure(
	struct device *dev, struct anx3618_platform_data *pdata)
{
	int rc = 0;

	pdata->id_pullup = regulator_get(dev, "id_pullup");
	if (IS_ERR(pdata->id_pullup)) {
		rc = PTR_ERR(pdata->id_pullup);
		pr_err("%s : Regulator get failed id_pullup rc=%d\n",
			   __func__, rc);
		return rc;
	}

	pdata->dvdd = regulator_get(dev, "dvdd");
	if (IS_ERR(pdata->dvdd)) {
		rc = PTR_ERR(pdata->dvdd);
		pr_err("%s : Regulator get failed dvdd rc=%d\n",
			   __func__, rc);
		goto err0;
	}

	pdata->apsd_regulator = regulator_get(dev, "apsd_ctrl");
	if (IS_ERR(pdata->apsd_regulator)) {
		rc = PTR_ERR(pdata->apsd_regulator);
		if (rc == -EPROBE_DEFER)
			dev_err(dev, "%s: apsd_regulator get defer!\n",
					__func__);
		else
			pr_err("%s : Regulator get failed apsd_ctrl rc=%d\n",
					__func__, rc);
		goto err1;
	}

	return rc;
err1:
	regulator_put(pdata->dvdd);
err0:
	regulator_put(pdata->id_pullup);

	return rc;
}

static int anx3618_parse_dt(
	struct device *dev, struct anx3618_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;

	pdata->gpio_p_dwn = of_get_named_gpio_flags(
		np, "slimport,chip_pd-gpio", 0, NULL);

	pdata->gpio_reset = of_get_named_gpio_flags(
		np, "slimport,reset-gpio", 0, NULL);

	pdata->gpio_cable_det = of_get_named_gpio_flags(
		np, "slimport,cable_detect-gpio", 0, NULL);

	dev_dbg(dev, " gpio p_dwn : %d, reset : %d,  gpio_cable_det %d\n",
			pdata->gpio_p_dwn,
			pdata->gpio_reset,
			pdata->gpio_cable_det);
	/*
	 * if "external-ldo-control" property is not exist, we
	 * assume that it is used in board.
	 * if don't use external ldo control,
	 * please use "external-ldo-control=<0>" in dtsi
	 */
	rc = of_property_read_u32(np, "slimport,external-ldo-control",
		&pdata->external_ldo_control);
	if (rc == -EINVAL)
		pdata->external_ldo_control = 1;

	if (pdata->external_ldo_control) {
		pdata->gpio_v10_ctrl = of_get_named_gpio_flags(
			np, "slimport,1v-gpio", 0, NULL);

		pdata->gpio_v33_ctrl = of_get_named_gpio_flags(
			np, "slimport,vdd3v3", 0, NULL);

		dev_dbg(dev, " gpio_v10_ctrl %d avdd33-en-gpio %d\n",
			pdata->gpio_v10_ctrl, pdata->gpio_v33_ctrl);
	}

	rc = anx3618_regulator_configure(dev, pdata);
	if (rc < 0)
		pr_err("%s %s: parsing dt for anx3618 is failed.\n",
				LOG_TAG, __func__);

	return rc;
}
#else
static int anx3618_parse_dt(
	struct device *dev, struct anx3618_platform_data *pdata)
{
	return -ENODEV;
}
#endif


static int anx3618_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct anx3618_data *anx3618;
	struct anx3618_platform_data *pdata;
	int ret = 0;

	/* Default values for slimport swing&pre-emphasis */
	val_SP_TX_LT_CTRL_REG0 = 0x19;
	val_SP_TX_LT_CTRL_REG2 = 0x36;
	val_SP_TX_LT_CTRL_REG1 = 0x26;
	val_SP_TX_LT_CTRL_REG6 = 0x3c;
	val_SP_TX_LT_CTRL_REG16 = 0x18;
	val_SP_TX_LT_CTRL_REG5 = 0x28;
	val_SP_TX_LT_CTRL_REG8 = 0x2F;
	val_SP_TX_LT_CTRL_REG15 = 0x10;
	val_SP_TX_LT_CTRL_REG18 = 0x1F;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s: i2c bus does not support the anx3618\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx3618 = kzalloc(sizeof(struct anx3618_data), GFP_KERNEL);
	if (!anx3618) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				 sizeof(struct anx3618_platform_data),
				 GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		client->dev.platform_data = pdata;
		/* device tree parsing function call */
		ret = anx3618_parse_dt(&client->dev, pdata);
		if (ret != 0) /* if occurs error */
			goto err0;

		ret = regulator_enable(pdata->apsd_regulator);
		if (ret < 0)
			dev_err(&client->dev, "enable apsd_regulator error!\n");
		else
			dev_info(&client->dev, "enable apsd_regulator\n");

		ret = regulator_enable(pdata->id_pullup);
		if (ret < 0)
			dev_err(&client->dev, "enable id_pullup error!\n");
		else
			dev_info(&client->dev, "enable id_pullup\n");


		anx3618->pdata = pdata;
	} else {
		anx3618->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = anx3618->pdata;

	anx3618_client = client;

	mutex_init(&anx3618->lock);

	if (!anx3618->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = anx3618_init_gpio(anx3618);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	INIT_DELAYED_WORK(&anx3618->work, slimport_work_func);
	INIT_WORK(&anx3618->cable_detect_work, cable_detect_work_func);

	anx3618->workqueue = create_singlethread_workqueue("anx3618_work");
	if (anx3618->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	ret = anx3618_system_init();
	if (ret) {
		pr_err("%s: failed to initialize anx3618\n", __func__);
		goto err2;
	}

	client->irq = gpio_to_irq(anx3618->pdata->gpio_cable_det);
	if (client->irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err2;
	}

	wake_lock_init(&anx3618->slimport_lock,
				WAKE_LOCK_SUSPEND,
				"slimport_wake_lock");


	ret = request_threaded_irq(client->irq, NULL, cable_detect_isr,
						IRQF_TRIGGER_RISING
						| IRQF_TRIGGER_FALLING
						| IRQF_ONESHOT,
						"anx3618", anx3618);
	if (ret  < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err2;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret  < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err3;
	}

	ret = enable_irq_wake(client->irq);
	if (ret  < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err3;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err3;
	}

	i2c_set_clientdata(client, anx3618);
	goto exit;

err3:
	free_irq(client->irq, anx3618);
err2:
	destroy_workqueue(anx3618->workqueue);
err1:
	anx3618_free_gpio(anx3618);
err0:
	if (pdata)
		devm_kfree(&client->dev, pdata);
	anx3618_client = NULL;
	kfree(anx3618);
exit:
	return ret;
}

static int anx3618_i2c_remove(struct i2c_client *client)
{
	struct anx3618_data *anx3618 = i2c_get_clientdata(client);
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		device_remove_file(&client->dev, &slimport_device_attrs[i]);

	free_irq(client->irq, anx3618);
	anx3618_free_gpio(anx3618);
	destroy_workqueue(anx3618->workqueue);
	wake_lock_destroy(&anx3618->slimport_lock);
	kfree(anx3618);
	return 0;
}

bool is_slimport_vga(void)
{
	return ((sp_tx_cur_cable_type() == DWN_STRM_IS_VGA_9832)
		|| (sp_tx_cur_cable_type() == DWN_STRM_IS_ANALOG)) ? 1 : 0;
}
/*
 * 0x01: hdmi device is attached
 * 0x02: DP device is attached
 * 0x03: Old VGA device is attached - RX_VGA_9832
 * 0x04: new combo VGA device is attached - RX_VGA_GEN
 * 0x00: unknown device
 */
EXPORT_SYMBOL(is_slimport_vga);
bool is_slimport_dp(void)
{
	return (sp_tx_cur_cable_type() == DWN_STRM_IS_DIGITAL) ? TRUE : FALSE;
}
EXPORT_SYMBOL(is_slimport_dp);

#ifdef CONFIG_PM
static int anx3618_suspend(struct device *dev)
{
	//TODO Implement
	return 0;
}

static int anx3618_resume(struct device *dev)
{
	//TODO Implement
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void anx3618_early_suspend(struct early_suspend *es)
{
	//TODO Implement
}

static void anx3618_late_resume(struct early_suspend *es)
{
	//TODO Implement
}

#else

static const struct dev_pm_ops anx3618_pm_ops = {
	.suspend	= anx3618_suspend,
	.resume		= anx3618_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM */

static const struct i2c_device_id anx3618_id[] = {
	{ "anx3618", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, anx3618_id);

#ifdef CONFIG_OF
static struct of_device_id anx3618_match_table[] = {
	{ .compatible = "slimport, anx3618",},
	{ },
};
#endif

static struct i2c_driver anx3618_driver = {
	.driver = {
		.name	= "anx3618",
		.owner	= THIS_MODULE,
#if (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
		.pm	= &anx3618_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = anx3618_match_table,
#endif
	},
	.probe	=	anx3618_i2c_probe,
	.remove	=	anx3618_i2c_remove,
	.id_table	= anx3618_id,
};

void slimport_work_func(struct work_struct * work)
{
	struct anx3618_data *td = container_of(work, struct anx3618_data,
								work.work);
	int workqueu_timer = 0;
	unchar sp_tx_state = sp_tx_cur_states();

	if (sp_tx_state >= STATE_PLAY_BACK)
		workqueu_timer = 500;
	else
		workqueu_timer = 100;

	mutex_lock(&td->lock);
	slimport_main_process();
	mutex_unlock(&td->lock);

	//debug_printf("state = %d, timer = %d\n", sp_tx_state, workqueu_timer);
	if (gpio_get_value(td->pdata->gpio_cable_det))
		queue_delayed_work(td->workqueue, &td->work,
			msecs_to_jiffies(workqueu_timer));
}

static int __init anx3618_init(void)
{
	int ret = 0;

	ret =  i2c_add_driver(&anx3618_driver);
	if (ret) {
		debug_puts("i2c_add_driver error!\n");
		goto error;
	}
	return 0;

error:
	i2c_del_driver(&anx3618_driver);
	return ret;
}

int slimport_send_msg(unsigned char addr, unsigned char *buf, unsigned short len, unsigned short rw_flag)
{
	int rc;

	anx3618_client->addr = addr;

	if(rw_flag) {
		rc = i2c_smbus_read_byte_data(anx3618_client, *buf);
		*buf = rc;
	} else {
		rc = i2c_smbus_write_byte_data(anx3618_client, buf[0], buf[1]);
	}

	return 0;
}

unsigned char __i2c_read_byte(unsigned char dev,unsigned char offset)
{
	unsigned char c;
	int ret;
	c = offset;

	ret = slimport_send_msg(dev >> 1, &c, 1, 1);
	if(ret < 0)
		debug_puts("mydp: Colorado_send_msg err!\n");

	return c;
}

static void __exit anx3618_exit(void)
{
	struct anx3618_data *anx3618 = i2c_get_clientdata(anx3618_client);

	disable_irq(anx3618_client->irq);

	sp_tx_clean_state_machine();
	drain_workqueue(anx3618->workqueue);

	hardware_power_ctl(0);

	destroy_workqueue(anx3618->workqueue);
	wake_lock_destroy(&anx3618->slimport_lock);
	i2c_del_driver(&anx3618_driver);
}

module_init(anx3618_init);
module_exit(anx3618_exit);

MODULE_DESCRIPTION("Slimport  transmitter anx3618 driver");
MODULE_AUTHOR("Junhua Xia <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
