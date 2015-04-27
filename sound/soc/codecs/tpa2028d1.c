/*
 * TPA2028D1 Audio Amplifier Driver
 *
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <sound/tpa2028d1.h>

#define REG_IC_FUNC_CONTROL	0x01
#define REG_AGC_ATT_CONTROL	0x02
#define REG_AGC_REL_CONTROL	0x03
#define REG_AGC_HOLD_CONTROL	0x04
#define REG_AGC_GAIN_CONTROL	0x05
#define REG_AGC_CONTROL1	0x06
#define REG_AGC_CONTROL2	0x07

#define IC_EN_BIT		6
#define IC_SWS_BIT		5
#define IC_FAULT_BIT		3
#define IC_THERMAL_BIT		2
#define IC_NG_BIT		0

#define AGC_TIME_MASK		(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5))
#define AGC_TIME_SHIFT		0

#define AGC1_OUTLIM_BIT		7
#define AGC1_NG_MASK		(BIT(5)|BIT(6))
#define AGC1_NG_SHIFT		5
#define AGC1_OUTLIM_LVL_MASK	(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4))
#define AGC1_OUTLIM_LVL_SHIFT	0

#define AGC2_MAX_GAIN_MASK	(BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define AGC2_MAX_GAIN_SHIFT	4
#define AGC2_COMP_MASK		(BIT(0)|BIT(1))
#define AGC2_COMP_SHIFT		0

enum tpa2028d1_command {
	/* REG_IC_FUNC_CONTROL */
	TPA2028D1_EN_STATUS,
	TPA2028D1_EN_ENABLE,
	TPA2028D1_EN_DISABLE,
	TPA2028D1_SWS_STATUS,
	TPA2028D1_SWS_ENABLE,
	TPA2028D1_SWS_DISABLE,
	TPA2028D1_FAULT_STATUS,
	TPA2028D1_FAULT_CLEAR,
	TPA2028D1_THERMAL_STATUS,
	TPA2028D1_THERMAL_CLEAR,
	TPA2028D1_NG_STATUS,
	TPA2028D1_NG_ENABLE,
	TPA2028D1_NG_DISABLE,

	/* REG_AGC_ATT_CONTROL */
	TPA2028D1_AGC_ATT_SET,
	TPA2028D1_AGC_ATT_GET,

	/* REG_AGC_REL_CONTROL */
	TPA2028D1_AGC_REL_SET,
	TPA2028D1_AGC_REL_GET,

	/* REG_AGC_HOLD_CONTROL */
	TPA2028D1_AGC_HOLD_SET,
	TPA2028D1_AGC_HOLD_GET,

	/* REG_AGC_GAIN_CONTROL */
	TPA2028D1_AGC_GAIN_SET,
	TPA2028D1_AGC_GAIN_GET,

	/* REG_AGC_CONTROL1 */
	TPA2028D1_AGC_OUTLIM_STATUS,
	TPA2028D1_AGC_OUTLIM_ENABLE,
	TPA2028D1_AGC_OUTLIM_DISABLE,
	TPA2028D1_AGC_NG_THRESHOLD_SET,
	TPA2028D1_AGC_NG_THRESHOLD_GET,
	TPA2028D1_AGC_OUTLIM_LVL_SET,
	TPA2028D1_AGC_OUTLIM_LVL_GET,

	/* REG_AGC_CONTROL2 */
	TPA2028D1_AGC_MAX_GAIN_SET,
	TPA2028D1_AGC_MAX_GAIN_GET,
	TPA2028D1_AGC_COMP_RATIO_SET,
	TPA2028D1_AGC_COMP_RATIO_GET,
};

struct tpa2028d1_data {
	struct i2c_client *client;
	const struct tpa2028d1_platform_data *pdata;
	struct tpa2028d1_config config;
	struct tpa2028d1_callbacks callbacks;
	int pwr_count;

};

static int tpa2028d1_read(struct i2c_client *client, uint8_t reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int tpa2028d1_write(struct i2c_client *client, uint8_t reg, uint8_t val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int tpa2028d1_read_mask(struct i2c_client *client,
	uint8_t reg, uint8_t mask, uint8_t shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = tpa2028d1_read(client, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}

static int tpa2028d1_write_mask(struct i2c_client *client,
	uint8_t reg, uint8_t val, uint8_t mask, uint8_t shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = tpa2028d1_read(client, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return tpa2028d1_write(client, reg, ret);
}

static int tpa2028d1_read_bit(struct i2c_client *client,
	uint8_t reg, uint8_t bit)
{
	if (bit > 8)
		return -EINVAL;
	return tpa2028d1_read_mask(client, reg, BIT(bit), bit);
}

static int tpa2028d1_write_bit(struct i2c_client *client,
	uint8_t reg, uint8_t val, uint8_t bit)
{
	if (bit > 8)
		return -EINVAL;
	return tpa2028d1_write_mask(client, reg, val, BIT(bit), bit);
}

static int tpa2028d1_exec_command(struct tpa2028d1_data *data,
	enum tpa2028d1_command command)
{
	struct i2c_client *client = data->client;
	switch (command) {
	case TPA2028D1_EN_STATUS:
		return tpa2028d1_read_bit(client,
			REG_IC_FUNC_CONTROL, IC_EN_BIT);
	case TPA2028D1_EN_ENABLE:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 1, IC_EN_BIT);
	case TPA2028D1_EN_DISABLE:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 0, IC_EN_BIT);
	case TPA2028D1_SWS_STATUS:
		return tpa2028d1_read_bit(client,
			REG_IC_FUNC_CONTROL, IC_SWS_BIT);
	case TPA2028D1_SWS_ENABLE:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 1, IC_SWS_BIT);
	case TPA2028D1_SWS_DISABLE:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 0, IC_SWS_BIT);
	case TPA2028D1_FAULT_STATUS:
		return tpa2028d1_read_bit(client,
			REG_IC_FUNC_CONTROL, IC_FAULT_BIT);
	case TPA2028D1_FAULT_CLEAR:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 0, IC_FAULT_BIT);
	case TPA2028D1_THERMAL_STATUS:
		return tpa2028d1_read_bit(client,
			REG_IC_FUNC_CONTROL, IC_THERMAL_BIT);
	case TPA2028D1_THERMAL_CLEAR:
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 0, IC_THERMAL_BIT);
	case TPA2028D1_NG_STATUS:
		return tpa2028d1_read_bit(client,
			REG_IC_FUNC_CONTROL, IC_NG_BIT);
	case TPA2028D1_NG_ENABLE:
		data->config.ng_enabled = 1;
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 1, IC_NG_BIT);
	case TPA2028D1_NG_DISABLE:
		data->config.ng_enabled = 0;
		return tpa2028d1_write_bit(client,
			REG_IC_FUNC_CONTROL, 0, IC_NG_BIT);
	case TPA2028D1_AGC_OUTLIM_STATUS:
		return tpa2028d1_read_bit(client,
			REG_AGC_CONTROL1, AGC1_OUTLIM_BIT);
	case TPA2028D1_AGC_OUTLIM_ENABLE:
		data->config.out_lim_disabled = 0;
		return tpa2028d1_write_bit(client,
			REG_AGC_CONTROL1, 0, AGC1_OUTLIM_BIT);
	case TPA2028D1_AGC_OUTLIM_DISABLE:
		data->config.out_lim_disabled = 1;
		return tpa2028d1_write_bit(client,
			REG_AGC_CONTROL1, 1, AGC1_OUTLIM_BIT);
	default:
		break;
	}
	return -EINVAL;
}

static int tpa2028d1_read_value(struct tpa2028d1_data *data,
	enum tpa2028d1_command command)
{
	struct i2c_client *client = data->client;
	int reg;
	int mask;
	int shift;

	switch (command) {
	case TPA2028D1_AGC_ATT_GET:
		reg = REG_AGC_ATT_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		break;
	case TPA2028D1_AGC_REL_GET:
		reg = REG_AGC_REL_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		break;
	case TPA2028D1_AGC_HOLD_GET:
		reg = REG_AGC_HOLD_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		break;
	case TPA2028D1_AGC_GAIN_GET:
		reg = REG_AGC_GAIN_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		break;
	case TPA2028D1_AGC_NG_THRESHOLD_GET:
		reg = REG_AGC_CONTROL1;
		mask = AGC1_NG_MASK;
		shift = AGC1_NG_SHIFT;
		break;
	case TPA2028D1_AGC_OUTLIM_LVL_GET:
		reg = REG_AGC_CONTROL1;
		mask = AGC1_OUTLIM_LVL_MASK;
		shift = AGC1_OUTLIM_LVL_SHIFT;
		break;
	case TPA2028D1_AGC_MAX_GAIN_GET:
		reg = REG_AGC_CONTROL2;
		mask = AGC2_MAX_GAIN_MASK;
		shift = AGC2_MAX_GAIN_SHIFT;
		break;
	case TPA2028D1_AGC_COMP_RATIO_GET:
		reg = REG_AGC_CONTROL2;
		mask = AGC2_COMP_MASK;
		shift = AGC2_COMP_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	return tpa2028d1_read_mask(client, reg, mask, shift);
}

static int tpa2028d1_write_value(struct tpa2028d1_data *data,
	enum tpa2028d1_command command, int val)
{
	struct i2c_client *client = data->client;
	int reg;
	int mask;
	int shift;

	switch (command) {
	case TPA2028D1_AGC_ATT_SET:
		reg = REG_AGC_ATT_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		data->config.atk_time = val;
		break;
	case TPA2028D1_AGC_REL_SET:
		reg = REG_AGC_REL_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		data->config.rel_time = val;
		break;
	case TPA2028D1_AGC_HOLD_SET:
		reg = REG_AGC_HOLD_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		data->config.hold_time = val;
		break;
	case TPA2028D1_AGC_GAIN_SET:
		reg = REG_AGC_GAIN_CONTROL;
		mask = AGC_TIME_MASK;
		shift = AGC_TIME_SHIFT;
		data->config.fixed_gain = val;
		break;
	case TPA2028D1_AGC_NG_THRESHOLD_SET:
		reg = REG_AGC_CONTROL1;
		mask = AGC1_NG_MASK;
		shift = AGC1_NG_SHIFT;
		data->config.ng_threshold = val;
		break;
	case TPA2028D1_AGC_OUTLIM_LVL_SET:
		reg = REG_AGC_CONTROL1;
		mask = AGC1_OUTLIM_LVL_MASK;
		shift = AGC1_OUTLIM_LVL_SHIFT;
		data->config.out_lim_level = val;
		break;
	case TPA2028D1_AGC_MAX_GAIN_SET:
		reg = REG_AGC_CONTROL2;
		mask = AGC2_MAX_GAIN_MASK;
		shift = AGC2_MAX_GAIN_SHIFT;
		data->config.max_gain = val;
		break;
	case TPA2028D1_AGC_COMP_RATIO_SET:
		reg = REG_AGC_CONTROL2;
		mask = AGC2_COMP_MASK;
		shift = AGC2_COMP_SHIFT;
		data->config.comp_ratio = val;
		break;
	default:
		return -EINVAL;
	}

	return tpa2028d1_write_mask(client, reg, val, mask, shift);
}

static int tpa2028d1_write_config(struct tpa2028d1_data *data)
{
	const struct tpa2028d1_config *config = &data->config;
	int ret;
	uint8_t val = 0;

	/* REG_IC_FUNC_CONTROL */
	val |= (0x0 << IC_EN_BIT) & BIT(IC_EN_BIT);
	val |= (config->ng_enabled << IC_NG_BIT) & BIT(IC_NG_BIT);
	ret = tpa2028d1_write(data->client, REG_IC_FUNC_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_ATT_CONTROL */
	val |= (config->atk_time << AGC_TIME_SHIFT) & AGC_TIME_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_ATT_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_REL_CONTROL */
	val |= (config->rel_time << AGC_TIME_SHIFT) & AGC_TIME_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_REL_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_HOLD_CONTROL */
	val |= (config->hold_time << AGC_TIME_SHIFT) & AGC_TIME_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_HOLD_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_GAIN_CONTROL */
	val |= (config->fixed_gain << AGC_TIME_SHIFT) & AGC_TIME_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_GAIN_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_CONTROL1 */
	val |= (config->out_lim_disabled << AGC1_OUTLIM_BIT)
		& BIT(AGC1_OUTLIM_BIT);
	val |= (config->ng_threshold << AGC1_NG_SHIFT) & AGC1_NG_MASK;
	val |= (config->out_lim_level << AGC1_OUTLIM_LVL_SHIFT)
		& AGC1_OUTLIM_LVL_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_CONTROL1, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_AGC_CONTROL2 */
	val |= (config->max_gain << AGC2_MAX_GAIN_SHIFT) & AGC2_MAX_GAIN_MASK;
	val |= (config->comp_ratio << AGC2_COMP_SHIFT) & AGC2_COMP_MASK;
	ret = tpa2028d1_write(data->client, REG_AGC_CONTROL2, val);
	if (ret)
		goto err;
	val = 0;

	/* REG_IC_FUNC_CONTROL */
	val |= (0x1 << IC_EN_BIT) & BIT(IC_EN_BIT);
	val |= (config->ng_enabled << IC_NG_BIT) & BIT(IC_NG_BIT);
	ret = tpa2028d1_write(data->client, REG_IC_FUNC_CONTROL, val);
	if (ret)
		goto err;
	val = 0;

	return 0;
err:
	dev_err(&data->client->dev, "failed to write config ret=%d\n", ret);
	return ret;
}

static int tpa2028d1_power(struct tpa2028d1_data *data, bool enable)
{
	if (data->pdata->amp_en_gpio == -1)
		return 0;

	if (enable)
		data->pwr_count++;
	else if (!enable && data->pwr_count > 0)
		data->pwr_count--;

	if ((enable && data->pwr_count == 1) ||
		(!enable && data->pwr_count == 0)) {

		/* Need to wait 1ms before last I2C transaction. */
		if (!enable)
			msleep(1);

		gpio_set_value(data->pdata->amp_en_gpio, enable ? 1 : 0);

		if (enable) {
			/* Need to wait at least 15ms before I2C transactions.
			 * Write config after enabling amplifier. */
			msleep(15);
			tpa2028d1_write_config(data);
		}
	}

	return 0;
}

static int tpa2028d1_cb_enable(struct tpa2028d1_callbacks *cb, bool enable)
{
	struct tpa2028d1_data *data =
		container_of(cb, struct tpa2028d1_data, callbacks);

	dev_vdbg(&data->client->dev, "%s %d\n", __func__, enable);

	tpa2028d1_power(data, enable);

	return 0;
}

/* Sysfs */
static ssize_t tpa2028d1_sysfs_regs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int i;
	ssize_t ret = 0;

	tpa2028d1_power(data, true);

	for (i = 1; i < REG_AGC_CONTROL2 + 1; i++) {
		int val = tpa2028d1_read(client, i);
		if (val < 0) {
			ret += sprintf(buf+ret, "0x%02x:error,ret=%d\n",
				i, val);
		} else {
			ret += sprintf(buf+ret, "0x%02x:0x%02x\n",
				i, val);
		}
	}

	tpa2028d1_power(data, false);

	return ret;
}

static ssize_t tpa2028d1_sysfs_regs_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	int reg, val;
	int ret;

	if (sscanf(buf, "0x%x 0x%x", &reg, &val) != 2)
		return -EINVAL;

	tpa2028d1_power(data, true);
	ret = tpa2028d1_write(data->client, reg, val);
	tpa2028d1_power(data, false);

	if (ret < 0)
		return ret;
	return count;
}

static ssize_t tpa2028d1_sysfs_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	int status;
	enum tpa2028d1_command command;

	tpa2028d1_power(data, true);

	if (!strcmp(attr->attr.name, "en_status"))
		command = TPA2028D1_EN_STATUS;
	else if (!strcmp(attr->attr.name, "sws_status"))
		command = TPA2028D1_SWS_STATUS;
	else if (!strcmp(attr->attr.name, "fault_status"))
		command = TPA2028D1_FAULT_STATUS;
	else if (!strcmp(attr->attr.name, "thermal_status"))
		command = TPA2028D1_THERMAL_STATUS;
	else if (!strcmp(attr->attr.name, "ng_status"))
		command = TPA2028D1_NG_STATUS;
	else if (!strcmp(attr->attr.name, "out_lim_disabled"))
		command = TPA2028D1_AGC_OUTLIM_STATUS;
	else {
		status = -EINVAL;
		goto end;
	}

	status = tpa2028d1_exec_command(data, command);

end:
	tpa2028d1_power(data, false);

	return sprintf(buf, "%d\n", status);
}

static ssize_t tpa2028d1_sysfs_status_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	enum tpa2028d1_command command;
	int val;
	int ret;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (!strcmp(attr->attr.name, "en_status"))
		command = val ? TPA2028D1_EN_ENABLE : TPA2028D1_EN_DISABLE;
	else if (!strcmp(attr->attr.name, "sws_status"))
		command = val ? TPA2028D1_SWS_ENABLE : TPA2028D1_SWS_DISABLE;
	else if (!strcmp(attr->attr.name, "fault_status"))
		command = TPA2028D1_FAULT_CLEAR;
	else if (!strcmp(attr->attr.name, "thermal_status"))
		command = TPA2028D1_THERMAL_CLEAR;
	else if (!strcmp(attr->attr.name, "ng_status"))
		command = val ? TPA2028D1_NG_ENABLE : TPA2028D1_NG_DISABLE;
	else if (!strcmp(attr->attr.name, "out_lim_disabled"))
		command = val ? TPA2028D1_AGC_OUTLIM_DISABLE :
			TPA2028D1_AGC_OUTLIM_ENABLE;
	else
		return -EINVAL;

	tpa2028d1_power(data, true);
	ret = tpa2028d1_exec_command(data, command);
	tpa2028d1_power(data, false);

	if (ret < 0)
		return ret;
	return count;
}

static ssize_t tpa2028d1_sysfs_val_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	int val;
	enum tpa2028d1_command command;

	tpa2028d1_power(data, true);

	if (!strcmp(attr->attr.name, "atk_time"))
		command = TPA2028D1_AGC_ATT_GET;
	else if (!strcmp(attr->attr.name, "rel_time"))
		command = TPA2028D1_AGC_REL_GET;
	else if (!strcmp(attr->attr.name, "hold_time"))
		command = TPA2028D1_AGC_HOLD_GET;
	else if (!strcmp(attr->attr.name, "fixed_gain"))
		command = TPA2028D1_AGC_GAIN_GET;
	else if (!strcmp(attr->attr.name, "ng_threshold"))
		command = TPA2028D1_AGC_NG_THRESHOLD_GET;
	else if (!strcmp(attr->attr.name, "out_lim_level"))
		command = TPA2028D1_AGC_OUTLIM_LVL_GET;
	else if (!strcmp(attr->attr.name, "max_gain"))
		command = TPA2028D1_AGC_MAX_GAIN_GET;
	else if (!strcmp(attr->attr.name, "comp_ratio"))
		command = TPA2028D1_AGC_COMP_RATIO_GET;
	else {
		val = -EINVAL;
		goto end;
	}

	val = tpa2028d1_read_value(data, command);

end:
	tpa2028d1_power(data, false);

	return sprintf(buf, "%d\n", val);
}

static ssize_t tpa2028d1_sysfs_val_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tpa2028d1_data *data = dev_get_drvdata(dev);
	enum tpa2028d1_command command;
	int val;
	int ret;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (!strcmp(attr->attr.name, "atk_time"))
		command = TPA2028D1_AGC_ATT_SET;
	else if (!strcmp(attr->attr.name, "rel_time"))
		command = TPA2028D1_AGC_REL_SET;
	else if (!strcmp(attr->attr.name, "hold_time"))
		command = TPA2028D1_AGC_HOLD_SET;
	else if (!strcmp(attr->attr.name, "fixed_gain"))
		command = TPA2028D1_AGC_GAIN_SET;
	else if (!strcmp(attr->attr.name, "ng_threshold"))
		command = TPA2028D1_AGC_NG_THRESHOLD_SET;
	else if (!strcmp(attr->attr.name, "out_lim_level"))
		command = TPA2028D1_AGC_OUTLIM_LVL_SET;
	else if (!strcmp(attr->attr.name, "max_gain"))
		command = TPA2028D1_AGC_MAX_GAIN_SET;
	else if (!strcmp(attr->attr.name, "comp_ratio"))
		command = TPA2028D1_AGC_COMP_RATIO_SET;
	else
		return -EINVAL;

	tpa2028d1_power(data, true);
	ret = tpa2028d1_write_value(data, command, val);
	tpa2028d1_power(data, false);

	if (ret < 0)
		return ret;
	return count;
}

static DEVICE_ATTR(dump_regs, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_regs_show, tpa2028d1_sysfs_regs_store);

static DEVICE_ATTR(en_status, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);
static DEVICE_ATTR(sws_status, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);
static DEVICE_ATTR(fault_status, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);
static DEVICE_ATTR(thermal_status, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);
static DEVICE_ATTR(ng_status, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);
static DEVICE_ATTR(out_lim_disabled, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_status_show, tpa2028d1_sysfs_status_store);

static DEVICE_ATTR(atk_time, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(rel_time, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(hold_time, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(fixed_gain, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(ng_threshold, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(out_lim_level, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(max_gain, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);
static DEVICE_ATTR(comp_ratio, S_IRUGO | S_IWUSR,
	tpa2028d1_sysfs_val_show, tpa2028d1_sysfs_val_store);

static struct attribute *tpa2028d1_sysfs_attributes[] = {
	&dev_attr_dump_regs.attr,

	&dev_attr_en_status.attr,
	&dev_attr_sws_status.attr,
	&dev_attr_fault_status.attr,
	&dev_attr_thermal_status.attr,
	&dev_attr_ng_status.attr,
	&dev_attr_out_lim_disabled.attr,

	&dev_attr_atk_time.attr,
	&dev_attr_rel_time.attr,
	&dev_attr_hold_time.attr,
	&dev_attr_fixed_gain.attr,
	&dev_attr_ng_threshold.attr,
	&dev_attr_out_lim_level.attr,
	&dev_attr_max_gain.attr,
	&dev_attr_comp_ratio.attr,
	NULL,
};

static const struct attribute_group tpa2028d1_sysfs_attr_group = {
	.attrs = tpa2028d1_sysfs_attributes,
};
/* Sysfs end */

static int __devinit tpa2028d1_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	const struct tpa2028d1_platform_data *pdata = client->dev.platform_data;
	struct tpa2028d1_data *data;

	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		ret = -EPERM;
		goto err;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err;
	}

	data->client = client;
	data->pdata = pdata;
	data->config = pdata->config;

	i2c_set_clientdata(client, data);

	if (pdata->amp_en_gpio != -1) {
		ret = gpio_request(pdata->amp_en_gpio, "tpa2028d1");
		if (ret) {
			dev_err(&client->dev,
				"Failed to get gpio ret=%d\n", ret);
			goto err_free_mem;
		}
	}

	data->callbacks.enable = tpa2028d1_cb_enable;
	if (pdata->register_callbacks) {
		pdata->register_callbacks(&data->callbacks);
	}

	ret = sysfs_create_group(&client->dev.kobj,
		&tpa2028d1_sysfs_attr_group);
	if (ret) {
		dev_err(&client->dev, "Failed to register sysfs ret=%d\n", ret);
		goto err_unregister;
	}

	return 0;
err_unregister:
	pdata->unregister_callbacks();
	if (pdata->amp_en_gpio != -1)
		gpio_free(pdata->amp_en_gpio);
err_free_mem:
	kfree(data);
err:
	return ret;
}

static int __devexit tpa2028d1_remove(struct i2c_client *client)
{
	struct tpa2028d1_data *data = i2c_get_clientdata(client);

	data->pdata->unregister_callbacks();

	/* Make sure power gets turned off. */
	data->pwr_count = 1;
	tpa2028d1_power(data, false);

	sysfs_remove_group(&client->dev.kobj, &tpa2028d1_sysfs_attr_group);

	gpio_free(data->pdata->amp_en_gpio);

	kfree(data);

	return 0;
}

static const struct i2c_device_id tpa2028d1_id[] = {
	{ "tpa2028d1", 0 },
	{ },
};

static struct i2c_driver tpa2028d1_driver = {
	.probe		= tpa2028d1_probe,
	.remove		= tpa2028d1_remove,
	.id_table	= tpa2028d1_id,
	.driver		= {
		.name	= "tpa2028d1",
		.owner	= THIS_MODULE,
	},
};

module_i2c_driver(tpa2028d1_driver);

MODULE_DESCRIPTION("TPA2028D1 Audio Amplifier Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("Rudolf Tammekivi <rtammekivi@gmail.com>");
