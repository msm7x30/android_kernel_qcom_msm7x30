/*
 * Copyright (c) 2013-2014, Rudolf Tammekivi.
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

/*
 * aps-12d.c
 * APS-12D Ambient Light & Proximity Sensor Driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/input/aps-12d.h>

#define CTRL_CMD1	0x00
#define CTRL_CMD2	0x01
#define DATA_LSB	0x02
#define DATA_MSB	0x03

/* Minimum operating voltage @2.25V */
#define APS_MINIMUM_UV 2250000
/* Maximum operating voltage @3.5V */
#define APS_MAXIMUM_UV 3500000

struct aps_12d_sensor_info {
	enum aps_12d_sensor_type type;
	bool enabled;
	int64_t poll_delay;
	struct delayed_work input_work;
};

struct aps_12d_data {
	const struct aps_12d_platform_data *pdata;

	struct i2c_client *client;
	struct input_dev *input_device;
	struct mutex sensor_mutex;

	struct aps_12d_settings settings;
	enum aps_12d_status status;
	struct aps_12d_sensor_info sensors[APS_12D_SENSOR_MAX];

	int last_adc_count[APS_12D_SENSOR_MAX];

	/* Keep index so we don't have to loop it through every time. */
	int sensors_enabled;
	struct regulator *vcc_regulator;
};

/* Global pointer for miscdevice. */
struct aps_12d_data *_aps_data;

static int aps_12d_read_reg(struct i2c_client *client, uint8_t reg)
{
	int ret;
	struct i2c_msg msg[2];
	u8 val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);

	/* Read register data. */
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "Failed to read 0x%02x reg\n", reg);
		return -EIO;
	}

	return val;
}

static int aps_12d_write_reg(struct i2c_client *client,
	uint8_t reg, uint8_t val)
{
	int ret;
	struct i2c_msg msg[1];
	u8 data[2];

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	/* Write register data. */
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "Failed to write 0x%02x reg\n", reg);
		return -EIO;
	}

	return 0;
}

static uint16_t aps_12d_8bit_to_12bit(uint8_t lsb, uint8_t msb)
{
	/* Move 8-bit LSB and 4-bit MSB into one. */
	return (msb << 8) | lsb;
}

static int aps_12d_adc_count(struct i2c_client *client)
{
	uint8_t lsb, msb;
	int adc_count;

	/* Read the LSB and MSB registers. */
	lsb = aps_12d_read_reg(client, DATA_LSB);
	if (lsb < 0)
		return lsb;
	msb = aps_12d_read_reg(client, DATA_MSB);
	if (msb < 0)
		return msb;

	adc_count = aps_12d_8bit_to_12bit(lsb, msb);

	return adc_count;
}

static void aps_12d_set_sensor_defaults(struct aps_12d_sensor_info *sensors)
{
	int i;
	for (i = 0; i < APS_12D_SENSOR_MAX; i++) {
		sensors[i].type = i;
		sensors[i].enabled = false;
		sensors[i].poll_delay = MINIMUM_DELAY_NS;
	}
}

static void aps_12d_set_defaults(struct aps_12d_data *data)
{
	const struct aps_12d_platform_data *pdata = data->pdata;
	struct aps_12d_settings *settings = &data->settings;

	if (pdata->range < 0)
		settings->range = APS_12D_RANGE_0P24_TO_1000;
	else
		settings->range = pdata->range;

	if (pdata->irdr_current < 0)
		settings->irdr_current = APS_12D_IRDR_50;
	else
		settings->irdr_current = pdata->irdr_current;

	if (pdata->mod_freq < 0)
		settings->mod_freq = APS_12D_MOD_FREQ_DC;
	else
		settings->mod_freq = pdata->mod_freq;

	if (pdata->resolution < 0)
		settings->resolution = APS_12D_RES_12;
	else
		settings->resolution = pdata->resolution;

	if (pdata->allow_reconfig < 0)
		settings->allow_reconfig = false;
	else
		settings->allow_reconfig = pdata->allow_reconfig;

	aps_12d_set_sensor_defaults(data->sensors);
}

static int aps_12d_set_settings(struct i2c_client *client,
	struct aps_12d_settings *settings)
{
	int ret;

	uint8_t reg_data = 0;

	/* Make a config byte. */
	reg_data |= settings->irdr_current << 6;
	reg_data |= settings->mod_freq << 4;
	reg_data |= settings->resolution << 2;
	reg_data |= settings->range;

	dev_dbg(&client->dev,
		"LED Current: 0x%02x, MOD Freq: 0x%02x, Res: 0x%02x, "
		"Range: 0x%02x. Final: 0x%02x\n",
		settings->irdr_current, settings->mod_freq,
		settings->resolution, settings->range, reg_data);

	ret = aps_12d_write_reg(client, CTRL_CMD2, reg_data);

	return ret;
}

/* Directly sets the sensor function. */
static int _aps_12d_set_state(struct i2c_client *client,
	enum aps_12d_state state)
{
	int ret;

	uint8_t reg_data = state << 5;

	ret = aps_12d_write_reg(client, CTRL_CMD1, reg_data);

	if (!ret)
		dev_vdbg(&client->dev, "Operating mode set to %d\n", state);

	return ret;
}

/* Sets the status used in the driver. */
static int aps_12d_set_status(struct aps_12d_data *data,
	enum aps_12d_status status)
{
	int ret;
	struct i2c_client *client = data->client;
	int i;

	/* If we turn the chip off, make sure every sensor is turned off. */
	if (status == APS_12D_STATUS_NONE) {
		data->sensors_enabled = 0;
		for (i = 0; i < APS_12D_SENSOR_MAX; i++)
			data->sensors[i].enabled = false;
	}

	ret = _aps_12d_set_state(client, STATUS_TO_STATE(status));

	if (ret) {
		dev_err(&client->dev, "Failed to set status ret=%d\n", ret);
		return ret;
	}

	data->status = status;

	return ret;
}

static int aps_12d_power(struct aps_12d_data *data, bool on)
{
	int ret = 0;

	if (!data->vcc_regulator)
		return 0;

	if (on)
		ret = regulator_enable(data->vcc_regulator);
	else
		ret = regulator_disable(data->vcc_regulator);

	return ret;
}

/* Set sensor info inside the driver. */
static void aps_12d_set_sensor(struct aps_12d_data *data,
	enum aps_12d_sensor_type type, bool enabled)
{
	/* Count the enabled sensors. */
	if (enabled && !data->sensors[type].enabled)
		data->sensors_enabled++;
	else if (!enabled && data->sensors[type].enabled)
		data->sensors_enabled--;

	data->sensors[type].enabled = enabled;

	/* No sensors enabled, turn off the sensor. */
	if (!data->sensors_enabled) {
		mutex_lock(&data->sensor_mutex);
		aps_12d_set_status(data, APS_12D_STATUS_NONE);
		mutex_unlock(&data->sensor_mutex);
	}
}

static void aps_12d_set_delay(struct aps_12d_data *data,
	enum aps_12d_sensor_type type, int64_t delay)
{
	struct aps_12d_sensor_info *sensor = &data->sensors[type];

	if (delay <= MINIMUM_DELAY_NS)
		sensor->poll_delay = MINIMUM_DELAY_NS;
	else
		sensor->poll_delay = delay;
}

static int aps_12d_reset(struct aps_12d_data *data)
{
	int ret;
	struct i2c_client *client = data->client;
	struct aps_12d_settings *settings = &data->settings;

	ret = aps_12d_set_settings(client, settings);

	/* Make sure the sensor is turned off. */
	mutex_lock(&data->sensor_mutex);
	ret = aps_12d_set_status(data, APS_12D_STATUS_NONE);
	mutex_unlock(&data->sensor_mutex);

	return ret;
}

static void aps_12d_report(struct aps_12d_data *data,
	enum aps_12d_sensor_type type, int adc_count)
{
	struct input_dev *input_device = data->input_device;

	mutex_lock(&input_device->mutex);

	/* Reporting negative is not allowed. */
	if (adc_count < 0)
		adc_count = 0;

	switch (type) {
	case APS_12D_SENSOR_LIGHT:
		input_report_abs(input_device, ABS_MISC, adc_count);
		break;
	case APS_12D_SENSOR_PROXIMITY:
		/* Hack: Userspace needs to sample this value. If it is
		 * same value, it gets filtered. */
		if (data->last_adc_count[type] == adc_count)
			adc_count += 1;

		input_report_abs(input_device, ABS_DISTANCE, adc_count);
		break;
	default:
		break;
	}

	data->last_adc_count[type] = adc_count;

	input_sync(input_device);

	mutex_unlock(&input_device->mutex);
}

static void aps_12d_schedule(struct aps_12d_data *data,
	struct aps_12d_sensor_info *sensor)
{
	if (sensor->enabled)
		schedule_delayed_work(&sensor->input_work,
			nsecs_to_jiffies(
				sensor->poll_delay - MINIMUM_DELAY_NS));
	else
		cancel_delayed_work(&sensor->input_work);
}

/* Light set function, only used in work func. */
static int aps_12d_write_light(struct aps_12d_data *data)
{
	int ret;

	struct aps_12d_sensor_info *light_sensor =
		&data->sensors[APS_12D_SENSOR_LIGHT];
	enum aps_12d_status new_status;

	/* We can set CONT mode if no other sensor is enabled. */
	if (data->sensors_enabled == 1 &&
		light_sensor->poll_delay <= ONCE_THRESHOLD_NS)
		new_status = TYPE_TO_STATUS_CONT(light_sensor->type);
	else
		new_status = TYPE_TO_STATUS_ONCE(light_sensor->type);

	ret = aps_12d_set_status(data, new_status);

	return ret;
}

/* Proximity set function, only used in work func. */
static int aps_12d_write_proximity(struct aps_12d_data *data, bool ir)
{
	int ret;

	enum aps_12d_status new_status;

	if (ir)
		new_status = APS_12D_STATUS_SURROUND_ONCE;
	else
		new_status = APS_12D_STATUS_PROXIMITY_ONCE;

	ret = aps_12d_set_status(data, new_status);

	return ret;
}

static void aps_12d_input_light_work_func(struct work_struct *work)
{
	struct aps_12d_data *data = container_of((struct delayed_work *)work,
		struct aps_12d_data, sensors[APS_12D_SENSOR_LIGHT].input_work);

	int adc_count;

	mutex_lock(&data->sensor_mutex);

	/* First, do the als. */
	aps_12d_write_light(data);

	msleep(MINIMUM_DELAY_MS);

	adc_count = aps_12d_adc_count(data->client);
	if (adc_count < 0)
		goto work_end;

	/* Check for sensor enable before returning new data. */
	if (!data->sensors[APS_12D_SENSOR_LIGHT].enabled)
		goto work_end;

	/* Second, report the data. */
	aps_12d_report(data, APS_12D_SENSOR_LIGHT, adc_count);

	/* Finally, reschedule. */
	aps_12d_schedule(data, &data->sensors[APS_12D_SENSOR_LIGHT]);

work_end:
	mutex_unlock(&data->sensor_mutex);

}

static void aps_12d_input_prox_work_func(struct work_struct *work)
{
	struct aps_12d_data *data = container_of((struct delayed_work *)work,
		struct aps_12d_data,
		sensors[APS_12D_SENSOR_PROXIMITY].input_work);

	int surround_adc, proximity_adc, final_adc;

	mutex_lock(&data->sensor_mutex);

	/* First, do the surround IR. */
	aps_12d_write_proximity(data, true);

	msleep(MINIMUM_DELAY_MS);

	surround_adc = aps_12d_adc_count(data->client);
	if (surround_adc < 0)
		goto work_end;

	/* Second, do the proximity IR. */
	aps_12d_write_proximity(data, false);

	msleep(MINIMUM_DELAY_MS);

	proximity_adc = aps_12d_adc_count(data->client);
	if (proximity_adc < 0)
		goto work_end;

	/* Check for sensor enable before returning new data. */
	if (!data->sensors[APS_12D_SENSOR_PROXIMITY].enabled)
		goto work_end;

	/* Third, calculate the ADC. */
	final_adc = proximity_adc - surround_adc;

	/* Fourth, report the data. */
	aps_12d_report(data, APS_12D_SENSOR_PROXIMITY, final_adc);

	/* Finally, reschedule. */
	aps_12d_schedule(data, &data->sensors[APS_12D_SENSOR_PROXIMITY]);

work_end:
	mutex_unlock(&data->sensor_mutex);
}

static int aps_12d_handle_enable(struct aps_12d_data *data,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *) arg;

	struct aps_12d_sensor_info *sensors = data->sensors;
	bool enabled;

	switch (cmd) {
	case APS_IOCTL_GET_LIGHT_ENABLE:
		enabled = sensors[APS_12D_SENSOR_LIGHT].enabled;
		break;
	case APS_IOCTL_GET_PROXIMITY_ENABLE:
		enabled = sensors[APS_12D_SENSOR_PROXIMITY].enabled;
		break;
	/* If the command does not match, check if we need to set. */
	default:
		goto set_enable;
		break;
	}

	if (copy_to_user(argp, &enabled, sizeof(enabled)))
		return -EFAULT;

	return 0;

set_enable:
	if (copy_from_user(&enabled, argp, sizeof(enabled)))
		return -EFAULT;

	/* When a sensor gets enabled, the data->sensors_enabled is not yet
	 * updated. Update the registry to make sure config is correct. */
	if (data->sensors_enabled == 0 && enabled)
		aps_12d_set_settings(data->client, &data->settings);

	switch (cmd) {
	case APS_IOCTL_SET_LIGHT_ENABLE:
		aps_12d_set_sensor(data, APS_12D_SENSOR_LIGHT, enabled);
		aps_12d_schedule(data, &sensors[APS_12D_SENSOR_LIGHT]);
		break;
	case APS_IOCTL_SET_PROXIMITY_ENABLE:
		aps_12d_set_sensor(data, APS_12D_SENSOR_PROXIMITY, enabled);
		aps_12d_schedule(data, &sensors[APS_12D_SENSOR_PROXIMITY]);
		break;
	default:
		break;
	}

	return 0;
}

static int aps_12d_handle_delay(struct aps_12d_data *data,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *) arg;

	struct aps_12d_sensor_info *sensors = data->sensors;
	int64_t poll_delay;

	switch (cmd) {
	case APS_IOCTL_GET_LIGHT_DELAY:
		poll_delay = sensors[APS_12D_SENSOR_LIGHT].poll_delay;
		break;
	case APS_IOCTL_GET_PROXIMITY_DELAY:
		poll_delay = sensors[APS_12D_SENSOR_PROXIMITY].poll_delay;
		break;
	/* If the command does not match, check if we need to set. */
	default:
		goto set_delay;
		break;
	}

	if (copy_to_user(argp, &poll_delay, sizeof(poll_delay)))
		return -EFAULT;

	return 0;

set_delay:

	if (copy_from_user(&poll_delay, argp, sizeof(poll_delay)))
		return -EFAULT;

	switch (cmd) {
	case APS_IOCTL_SET_LIGHT_DELAY:
		aps_12d_set_delay(data, APS_12D_SENSOR_LIGHT, poll_delay);
		break;
	case APS_IOCTL_SET_PROXIMITY_DELAY:
		aps_12d_set_delay(data, APS_12D_SENSOR_PROXIMITY, poll_delay);
		break;
	default:
		break;
	}

	return 0;
}


static long aps_12d_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *) arg;
	struct aps_12d_data *data = file->private_data;
	int ret = 0;

	switch (cmd) {
	case APS_IOCTL_GET_SETTINGS:
		if (copy_to_user(argp, &data->settings, sizeof(data->settings)))
			return -EFAULT;
		break;
	case APS_IOCTL_SET_SETTINGS:
		if (copy_from_user(&data->settings, argp,
			sizeof(data->settings)))
			return -EFAULT;
		if (data->settings.allow_reconfig)
			ret = aps_12d_set_settings(data->client,
				&data->settings);
		else
			ret = -EPERM;
		break;
	case APS_IOCTL_GET_STATUS:
		if (copy_to_user(argp, &data->status, sizeof(data->status)))
			return -EFAULT;
		break;
	case APS_IOCTL_GET_LIGHT_ENABLE:
	case APS_IOCTL_SET_LIGHT_ENABLE:
	case APS_IOCTL_GET_PROXIMITY_ENABLE:
	case APS_IOCTL_SET_PROXIMITY_ENABLE:
		ret = aps_12d_handle_enable(data, cmd, arg);
		break;
	case APS_IOCTL_GET_LIGHT_DELAY:
	case APS_IOCTL_SET_LIGHT_DELAY:
	case APS_IOCTL_GET_PROXIMITY_DELAY:
	case APS_IOCTL_SET_PROXIMITY_DELAY:
		ret = aps_12d_handle_delay(data, cmd, arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int aps_12d_fops_open(struct inode *ip, struct file *file)
{
	file->private_data = _aps_data;
	return 0;
}

static const struct file_operations aps_12d_fops = {
	.owner = THIS_MODULE,
	.open = aps_12d_fops_open,
	.unlocked_ioctl = aps_12d_ioctl,
};

static struct miscdevice aps_12d_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aps-12d",
	.fops = &aps_12d_fops,
};

static int __devinit aps_12d_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	const struct aps_12d_platform_data *pdata = client->dev.platform_data;
	struct aps_12d_data *data;
	struct input_dev *input_device;

	if (!pdata) {
		dev_err(&client->dev, "No plaform data\n");
		return -EPERM;
	}

	data = kzalloc(sizeof(struct aps_12d_data), GFP_KERNEL);
	input_device = input_allocate_device();
	if (!data || !input_device) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}

	input_device->name = id->name;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;

	input_set_capability(input_device, EV_ABS, ABS_MISC);
	input_set_capability(input_device, EV_ABS, ABS_DISTANCE);

	/* Ambient Light Sensor. */
	input_set_abs_params(input_device,
		ABS_MISC, 0, APS_12D_MAX_VALUE, 0, 0);
	/* Proximity Sensor. */
	input_set_abs_params(input_device,
		ABS_DISTANCE, 0, APS_12D_MAX_VALUE, 0, 0);

	data->client = client;
	data->input_device = input_device;
	data->pdata = pdata;

	aps_12d_set_defaults(data);

	input_set_drvdata(input_device, data);
	i2c_set_clientdata(client, data);

	mutex_init(&data->sensor_mutex);

	if (data->pdata->vcc_regulator) {
		data->vcc_regulator = regulator_get(&data->client->dev,
			data->pdata->vcc_regulator);

		if (IS_ERR(data->vcc_regulator)) {
			dev_err(&client->dev,
				"Failed to configure regulator\n");
			data->vcc_regulator = NULL;
		} else {
			ret = regulator_set_voltage(data->vcc_regulator,
				APS_MINIMUM_UV, APS_MAXIMUM_UV);
			if (ret) {
				dev_err(&client->dev, "Failed to configure "
					"regulator voltage\n");
				data->vcc_regulator = NULL;
			}
		}
	}

	ret = aps_12d_power(data, true);
	if (ret) {
		dev_err(&client->dev, "Failed to turn on power ret=%d\n", ret);
		goto err_power;
	}

	ret = aps_12d_reset(data);
	if (ret) {
		dev_err(&client->dev, "Failed to reset the device ret=%d\n",
			ret);
		goto err_power;
	}

	ret = input_register_device(input_device);
	if (ret) {
		dev_err(&client->dev, "Failed to register input device "
			"ret=%d\n", ret);
		goto err_unregister_device;
	}

	ret = misc_register(&aps_12d_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to initialize devfs ret=%d\n",
			ret);
		goto err_unregister_device;
	}

	INIT_DELAYED_WORK(&data->sensors[APS_12D_SENSOR_LIGHT].input_work,
		aps_12d_input_light_work_func);
	INIT_DELAYED_WORK(&data->sensors[APS_12D_SENSOR_PROXIMITY].input_work,
		aps_12d_input_prox_work_func);

	_aps_data = data;

	return 0;

err_unregister_device:
	input_unregister_device(input_device);
	input_device = NULL;
err_power:
	aps_12d_power(data, false);
	regulator_put(data->vcc_regulator);
err_free_mem:
	input_free_device(input_device);
	kfree(data);
	return ret;
}

static int aps_12d_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct aps_12d_data *data = i2c_get_clientdata(client);

	aps_12d_power(data, false);

	return 0;
}

static int aps_12d_resume(struct i2c_client *client)
{
	struct aps_12d_data *data = i2c_get_clientdata(client);

	aps_12d_power(data, true);

	return 0;
}

static int __devexit aps_12d_remove(struct i2c_client *client)
{
	int ret = 0;
	struct aps_12d_data *data = i2c_get_clientdata(client);

	input_unregister_device(data->input_device);
	input_free_device(data->input_device);

	misc_deregister(&aps_12d_dev);

	mutex_lock(&data->sensor_mutex);
	aps_12d_set_status(data, APS_12D_STATUS_NONE);
	aps_12d_power(data, false);
	mutex_unlock(&data->sensor_mutex);

	regulator_put(data->vcc_regulator);

	kfree(data);

	return ret;
}

static const struct i2c_device_id aps_12d_id[] = {
	{ "aps-12d", 0 },
	{ },
};

static struct i2c_driver aps_12d_driver = {
	.probe = aps_12d_probe,
	.remove = aps_12d_remove,
	.id_table = aps_12d_id,
	.driver = {
		.name = "aps-12d",
		.owner = THIS_MODULE,
	},
	.suspend    = aps_12d_suspend,
	.resume     = aps_12d_resume,
};

module_i2c_driver(aps_12d_driver);

MODULE_DESCRIPTION("APS-12D Ambient Light & Proximity Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("Rudolf Tammekivi <rtammekivi@gmail.com>");
