/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2012  Pali Rohár <pali.rohar@gmail.com>
 * Copyright (C) 2015  Rudolf Tammekivi <rtammekivi@gmail.com>
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Datasheets:
 * http://www.ti.com/product/bq24150
 * http://www.ti.com/product/bq24150a
 * http://www.ti.com/product/bq24152
 * http://www.ti.com/product/bq24153
 * http://www.ti.com/product/bq24153a
 * http://www.ti.com/product/bq24155
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include "bq2415x_boost.h"

/* timeout for resetting chip timer */
#define BQ2415X_TIMER_TIMEOUT		10

#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CONTROL		0x01
#define BQ2415X_REG_VOLTAGE		0x02
#define BQ2415X_REG_VENDER		0x03
#define BQ2415X_REG_CURRENT		0x04

/* reset state for all registers */
#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

/* control register */
#define BQ2415X_MASK_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_LIMIT		6
#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4
#define BQ2415X_BIT_TE			3
#define BQ2415X_BIT_CE			2
#define BQ2415X_BIT_HZ_MODE		1
#define BQ2415X_BIT_OPA_MODE		0

/* voltage register */
#define BQ2415X_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2
#define BQ2415X_BIT_OTG_PL		1
#define BQ2415X_BIT_OTG_EN		0

/* vender register */
#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5
#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3
#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/* current register */
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define BQ2415X_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_VI_TERM		0


enum bq2415x_command {
	BQ2415X_TIMER_RESET,
	BQ2415X_OTG_STATUS,
	BQ2415X_STAT_PIN_STATUS,
	BQ2415X_STAT_PIN_ENABLE,
	BQ2415X_STAT_PIN_DISABLE,
	BQ2415X_CHARGE_STATUS,
	BQ2415X_BOOST_STATUS,
	BQ2415X_FAULT_STATUS,

	BQ2415X_CHARGE_TERMINATION_STATUS,
	BQ2415X_CHARGE_TERMINATION_ENABLE,
	BQ2415X_CHARGE_TERMINATION_DISABLE,
	BQ2415X_CHARGER_STATUS,
	BQ2415X_CHARGER_ENABLE,
	BQ2415X_CHARGER_DISABLE,
	BQ2415X_HIGH_IMPEDANCE_STATUS,
	BQ2415X_HIGH_IMPEDANCE_ENABLE,
	BQ2415X_HIGH_IMPEDANCE_DISABLE,
	BQ2415X_BOOST_MODE_STATUS,
	BQ2415X_BOOST_MODE_ENABLE,
	BQ2415X_BOOST_MODE_DISABLE,

	BQ2415X_OTG_LEVEL,
	BQ2415X_OTG_ACTIVATE_HIGH,
	BQ2415X_OTG_ACTIVATE_LOW,
	BQ2415X_OTG_PIN_STATUS,
	BQ2415X_OTG_PIN_ENABLE,
	BQ2415X_OTG_PIN_DISABLE,

	BQ2415X_VENDER_CODE,
	BQ2415X_PART_NUMBER,
	BQ2415X_REVISION,
};

enum bq2415x_chip {
	BQUNKNOWN,
	BQ24150,
	BQ24150A,
	BQ24151,
	BQ24151A,
	BQ24152,
	BQ24153,
	BQ24153A,
	BQ24155,
	BQ24156,
	BQ24156A,
	BQ24158,
};

struct bq2415x_device {
	struct device *dev;
	struct delayed_work work;
	int autotimer;
	struct wake_lock wakelock;
	enum bq2415x_mode mode;
};

static struct bq2415x_device *bq2415x_data = NULL;

static DEFINE_MUTEX(bq2415x_timer_mutex);
static DEFINE_MUTEX(bq2415x_i2c_mutex);

/* read value from register */
static int bq2415x_i2c_read(struct bq2415x_device *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[2];
	u8 val;
	int ret;
	int tries;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	for (tries = 0; tries < 5; tries++) {
		mutex_lock(&bq2415x_i2c_mutex);
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		mutex_unlock(&bq2415x_i2c_mutex);

		if (ret != ARRAY_SIZE(msg))
			msleep(10);
		else
			break;
	}


	if (ret < 0)
		return ret;

	return val;
}

/* read value from register, apply mask and right shift it */
static int bq2415x_i2c_read_mask(struct bq2415x_device *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int bq2415x_i2c_read_bit(struct bq2415x_device *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/* write value to register */
static int bq2415x_i2c_write(struct bq2415x_device *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;
	int tries;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	for (tries = 0; tries < 5; tries++) {
		mutex_lock(&bq2415x_i2c_mutex);
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		mutex_unlock(&bq2415x_i2c_mutex);

		if (ret != ARRAY_SIZE(msg))
			msleep(10);
		else
			break;
	}

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int bq2415x_i2c_write_mask(struct bq2415x_device *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = bq2415x_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return bq2415x_i2c_write(bq, reg, ret);
}

/* change only one bit in register */
static int bq2415x_i2c_write_bit(struct bq2415x_device *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return bq2415x_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

static int bq2415x_exec_command(struct bq2415x_device *bq,
				enum bq2415x_command command)
{
	int ret;

	switch (command) {
	case BQ2415X_TIMER_RESET:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS,
				1, BQ2415X_BIT_TMR_RST);
	case BQ2415X_OTG_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_OTG);
	case BQ2415X_STAT_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 1,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_STAT_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_STATUS, 0,
				BQ2415X_BIT_EN_STAT);
	case BQ2415X_CHARGE_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
				BQ2415X_MASK_STAT, BQ2415X_SHIFT_STAT);
	case BQ2415X_BOOST_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_STATUS,
				BQ2415X_BIT_BOOST);
	case BQ2415X_FAULT_STATUS:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
			BQ2415X_MASK_FAULT, BQ2415X_SHIFT_FAULT);

	case BQ2415X_CHARGE_TERMINATION_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_TE);
	case BQ2415X_CHARGE_TERMINATION_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_TE);
	case BQ2415X_CHARGER_STATUS:
		ret = bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
			BQ2415X_BIT_CE);
		if (ret < 0)
			return ret;
		else
			return ret > 0 ? 0 : 1;
	case BQ2415X_CHARGER_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_CE);
	case BQ2415X_CHARGER_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_CE);
	case BQ2415X_HIGH_IMPEDANCE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_HIGH_IMPEDANCE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_HZ_MODE);
	case BQ2415X_BOOST_MODE_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				1, BQ2415X_BIT_OPA_MODE);
	case BQ2415X_BOOST_MODE_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
				0, BQ2415X_BIT_OPA_MODE);

	case BQ2415X_OTG_LEVEL:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_HIGH:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_ACTIVATE_LOW:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_PL);
	case BQ2415X_OTG_PIN_STATUS:
		return bq2415x_i2c_read_bit(bq, BQ2415X_REG_VOLTAGE,
				BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_ENABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				1, BQ2415X_BIT_OTG_EN);
	case BQ2415X_OTG_PIN_DISABLE:
		return bq2415x_i2c_write_bit(bq, BQ2415X_REG_VOLTAGE,
				0, BQ2415X_BIT_OTG_EN);

	case BQ2415X_VENDER_CODE:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_VENDER, BQ2415X_SHIFT_VENDER);
	case BQ2415X_PART_NUMBER:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
				BQ2415X_MASK_PN, BQ2415X_SHIFT_PN);
	case BQ2415X_REVISION:
		return bq2415x_i2c_read_mask(bq, BQ2415X_REG_VENDER,
			BQ2415X_MASK_REVISION, BQ2415X_SHIFT_REVISION);
	}
	return -EINVAL;
}

static void bq2415x_set_autotimer(struct bq2415x_device *bq, int state);

int bq2415x_set_mode(enum bq2415x_mode mode)
{
	int ret = 0;
	struct bq2415x_device *bq;

	if (!bq2415x_data)
		return -EPERM;

	bq = bq2415x_data;

	switch (mode) {
	case BQ2415X_MODE_BOOST_OFF:
		dev_info(bq->dev, "Turning off boost mode\n");
		bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);
		bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_ENABLE);
		break;
	case BQ2415X_MODE_BOOST_ON:
		dev_info(bq->dev, "Turning on boost mode\n");
		bq2415x_exec_command(bq, BQ2415X_HIGH_IMPEDANCE_DISABLE);
		bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_ENABLE);
		break;
	default:
		return -EINVAL;
	}

	bq->mode = mode;

	/* Safe to turn off safety timer when charger is in turned off mode. */
	bq2415x_set_autotimer(bq, (mode != BQ2415X_MODE_BOOST_OFF));

	return ret;
}
EXPORT_SYMBOL(bq2415x_set_mode);

static void bq2415x_set_autotimer(struct bq2415x_device *bq, int state)
{
	mutex_lock(&bq2415x_timer_mutex);

	if (bq->autotimer == state) {
		mutex_unlock(&bq2415x_timer_mutex);
		return;
	}

	bq->autotimer = state;

	if (state) {
		wake_lock(&bq->wakelock);
		schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
		bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
	} else {
		cancel_delayed_work_sync(&bq->work);
		wake_unlock(&bq->wakelock);
	}

	mutex_unlock(&bq2415x_timer_mutex);
}

static void bq2415x_timer_work(struct work_struct *work)
{
	struct bq2415x_device *bq = container_of(work, struct bq2415x_device,
						 work.work);
	int ret;
	int error;
	int boost;
	int status;

	if (!bq->autotimer)
		return;

	ret = bq2415x_exec_command(bq, BQ2415X_TIMER_RESET);
	if (ret < 0) {
		dev_err(bq->dev, "Resetting timer failed\n");
		goto reschedule;
	}

	status = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
	if (status < 0) {
		dev_err(bq->dev, "Unknown error\n");
		goto reschedule;
	}

	boost = bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_STATUS);
	if (boost < 0) {
		dev_err(bq->dev, "Unknown error\n");
		goto reschedule;
	}

	error = bq2415x_exec_command(bq, BQ2415X_FAULT_STATUS);
	if (error < 0) {
		dev_err(bq->dev, "Unknown error\n");
		goto reschedule;
	}

	if (boost) {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			bq2415x_set_mode(bq->mode);
			break;
		case 3: /* Battery voltage too low */
			dev_err(bq->dev, "Battery voltage to low\n");
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			dev_err(bq->dev,
				"Overvoltage protection (chip fried)\n");
			break;
		case 2: /* Overload */
			dev_err(bq->dev, "Overload\n");
			break;
		case 4: /* Battery overvoltage protection */
			dev_err(bq->dev, "Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			dev_err(bq->dev, "Thermal shutdown (too hot)\n");
			break;
		case 7: /* N/A */
			dev_err(bq->dev, "Unknown error\n");
			break;
		}
	} else {
		switch (error) {
		/* Non fatal errors, chip is OK */
		case 0: /* No error */
			break;
		case 2: /* Sleep mode */
			dev_err(bq->dev, "Sleep mode\n");
			break;
		case 3: /* Poor input source */
			dev_err(bq->dev, "Poor input source\n");
			break;
		case 6: /* Timer expired */
			dev_err(bq->dev, "Timer expired\n");
			bq2415x_set_mode(bq->mode);
			break;
		case 7: /* No battery */
			dev_err(bq->dev, "No battery\n");
			bq2415x_set_mode(bq->mode);
			break;

		/* Fatal errors, disable and reset chip */
		case 1: /* Overvoltage protection (chip fried) */
			dev_err(bq->dev,
				"Overvoltage protection (chip fried)\n");
			break;
		case 4: /* Battery overvoltage protection */
			dev_err(bq->dev, "Battery overvoltage protection\n");
			break;
		case 5: /* Thermal shutdown (too hot) */
			dev_err(bq->dev, "Thermal shutdown (too hot)\n");
			break;
		}
	}

reschedule:
	schedule_delayed_work(&bq->work, BQ2415X_TIMER_TIMEOUT * HZ);
}

static int bq2415x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct bq2415x_device *bq;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "failed to allocate device data\n");
		ret = -ENOMEM;
		goto error;
	}

	i2c_set_clientdata(client, bq);

	bq->dev = &client->dev;
	bq->autotimer = 0;
	bq->mode = BQ2415X_MODE_BOOST_OFF;

	wake_lock_init(&bq->wakelock, WAKE_LOCK_SUSPEND, "bq2415x_timer");

	INIT_DELAYED_WORK(&bq->work, bq2415x_timer_work);

	bq2415x_data = bq;

	return 0;

error:
	return ret;
}

static int bq2415x_remove(struct i2c_client *client)
{
	struct bq2415x_device *bq = i2c_get_clientdata(client);

	if (bq->mode == BQ2415X_MODE_BOOST_ON)
		bq2415x_set_mode(BQ2415X_MODE_BOOST_OFF);

	wake_lock_destroy(&bq->wakelock);

	bq2415x_data = NULL;

	return 0;
}

static const struct i2c_device_id bq2415x_i2c_id_table[] = {
	{ "bq2415x", BQUNKNOWN },
	{ "bq24150", BQ24150 },
	{ "bq24150a", BQ24150A },
	{ "bq24151", BQ24151 },
	{ "bq24151a", BQ24151A },
	{ "bq24152", BQ24152 },
	{ "bq24153", BQ24153 },
	{ "bq24153a", BQ24153A },
	{ "bq24155", BQ24155 },
	{ "bq24156", BQ24156 },
	{ "bq24156a", BQ24156A },
	{ "bq24158", BQ24158 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2415x_i2c_id_table);

static void bq2415x_shutdown(struct i2c_client *client)
{
	struct bq2415x_device *bq = i2c_get_clientdata(client);

	if (bq->mode == BQ2415X_MODE_BOOST_ON)
		bq2415x_set_mode(BQ2415X_MODE_BOOST_OFF);
}

static struct i2c_driver bq2415x_driver = {
	.driver = {
		.name = "bq2415x-boost",
	},
	.probe = bq2415x_probe,
	.remove = bq2415x_remove,
	.id_table = bq2415x_i2c_id_table,
	.shutdown = bq2415x_shutdown,
};
module_i2c_driver(bq2415x_driver);

MODULE_AUTHOR("Pali Rohár <pali.rohar@gmail.com>");
MODULE_DESCRIPTION("bq2415x charger driver");
MODULE_LICENSE("GPL");
