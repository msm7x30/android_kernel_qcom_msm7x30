/*
 * Huawei Battery Driver
 * Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014  Rudolf Tammekivi <rtammekivi@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see [http://www.gnu.org/licenses/].
 */

#define DEBUG

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <mach/msm_rpcrouter.h>

#include <linux/huawei_battery.h>

#define BATTERY_RPC_PROG			0x30000089
#define BATTERY_RPC_VERS			0x00050001

#define BATTERY_CB_TYPE_PROC			1
#define BATTERY_REGISTER_PROC			2
#define BATTERY_MODIFY_CLIENT_PROC		4
#define BATTERY_DEREGISTER_CLIENT_PROC		5
#define BATTERY_READ_MV_PROC			12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC	14
#define BATTERY_SET_DELTA_PROC			17

#define BATTERY_CB_ID_ALL_ACTIVITY		1
#define BATTERY_CB_ID_LOW_VOLTAGE		2

#define CHG_RPC_PROG				0x3000001a
#define CHG_RPC_VERS				0x00040001

#define CHG_SET_CHARGER_CONNECTED		7
#define CHG_SET_CHARGER_DISCONNECTED		8
#define CHG_SET_CURRENT_AVAILABLE		9
#define CHG_SET_CURRENT_UNAVAILABLE		10
#define CHG_GET_GENERAL_STATUS_PROC		12
#define CHG_GET_CAPACITY_PROC			19

#define PM_LIB_RPC_PROG				0x30000061
#define PM_LIB_RPC_VERS				0x00030005

#define PM_LIB_CONSUMER_NOTIFY_PROC		200

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

struct huawei_bat_batt {
	uint8_t capacity;
	uint16_t voltage;
	int8_t temperature;
};

struct huawei_bat_data {
	struct device *dev;
	const struct huawei_bat_platform_data *pdata;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;
	struct msm_rpc_endpoint *pm_ep;

	uint32_t batt_handle;

	struct workqueue_struct *workqueue;

	struct power_supply battery_ps;
	struct power_supply ac_ps;
	struct power_supply usb_ps;

	struct huawei_bat_batt battery;
	enum chg_type charger_type;
};
static struct huawei_bat_data *bat_data = NULL;

struct work_entry {
	struct work_struct work;
	struct huawei_bat_data *data;
	enum huawei_bat_consumer consumer;
	union huawei_bat_state state;
};

/* Corresponding RPC entry for the consumer. */
static const uint32_t rpc_consumer_entry[HW_BAT_CONSUMER_MAX] = {
	[HW_BAT_CONSUMER_LCD]			= 0,
	[HW_BAT_CONSUMER_FRONT_CAMERA]		= 1,
	[HW_BAT_CONSUMER_BACK_CAMERA]		= 2,
	[HW_BAT_CONSUMER_CAMERA]		= 3,
	[HW_BAT_CONSUMER_WIFI]			= 4,
	[HW_BAT_CONSUMER_BT]			= 5,
	[HW_BAT_CONSUMER_FM]			= 6,
	[HW_BAT_CONSUMER_ADSP]			= 7,
	[HW_BAT_CONSUMER_CAMERA_FLASH]		= 8,
	[HW_BAT_CONSUMER_KEYPAD]		= 20,
	[HW_BAT_CONSUMER_VIBRATOR]		= 21,
	[HW_BAT_CONSUMER_GSM850_GSM900]		= 22,
	[HW_BAT_CONSUMER_GSM1800_GSM1900]	= 23,
	[HW_BAT_CONSUMER_WCDMA]			= 24,
	[HW_BAT_CONSUMER_CDMA1X]		= 25,
	[HW_BAT_CONSUMER_SPEAKER]		= 26,
	[HW_BAT_CONSUMER_CPU]			= 27,
	[HW_BAT_CONSUMER_GPS]			= 28,

	[HW_BAT_CONSUMER_NONE]			= 0xff,
};

static int huawei_bat_notify_rpc(uint32_t consumer,
	uint32_t state)
{
	int ret;
	struct huawei_bat_data *data = bat_data;
	struct device *dev = data->dev;

	struct {
		struct rpc_request_hdr hdr;
		u32 consumer;
		u32 state;
	} req;

	if (!data || IS_ERR_OR_NULL(data->pm_ep))
		return -EPERM;

	req.consumer = cpu_to_be32(consumer);
	req.state = cpu_to_be32(state);

	ret = msm_rpc_call(data->pm_ep, PM_LIB_CONSUMER_NOTIFY_PROC,
		&req, sizeof(req), msecs_to_jiffies(5000));
	if (ret) {
		dev_err(dev, "failed to do rpc call for consumer %d ret=%d\n",
			consumer, ret);
		return ret;
	}

	return 0;
}

static void huawei_bat_work(struct work_struct *work)
{
	int ret = 0;
	struct work_entry *entry = (struct work_entry *)work;
	struct huawei_bat_data *data = entry->data;
	struct device *dev = data->dev;
	uint32_t rpc_consumer = rpc_consumer_entry[entry->consumer];
	union huawei_bat_state state = entry->state;

	switch(entry->consumer) {
	case HW_BAT_CONSUMER_LCD:
		dev_vdbg(dev, "Notifying LCD level %d\n",
			state.backlight_level);
		ret = huawei_bat_notify_rpc(rpc_consumer,
			state.backlight_level);
		break;
	case HW_BAT_CONSUMER_FRONT_CAMERA:
	case HW_BAT_CONSUMER_BACK_CAMERA:
	case HW_BAT_CONSUMER_CAMERA:
		dev_vdbg(dev, "Notifying camera on %d\n", state.on);
		ret = huawei_bat_notify_rpc(rpc_consumer, state.on);
		break;
	case HW_BAT_CONSUMER_WIFI:
	case HW_BAT_CONSUMER_BT:
	case HW_BAT_CONSUMER_FM:
	case HW_BAT_CONSUMER_ADSP:
		break;
	case HW_BAT_CONSUMER_CAMERA_FLASH:
		dev_vdbg(dev, "Notifying flash current %d\n",
			state.flash_current);
		ret = huawei_bat_notify_rpc(rpc_consumer, state.flash_current);
		break;
	case HW_BAT_CONSUMER_KEYPAD:
	case HW_BAT_CONSUMER_VIBRATOR:
	case HW_BAT_CONSUMER_GSM850_GSM900:
	case HW_BAT_CONSUMER_GSM1800_GSM1900:
	case HW_BAT_CONSUMER_WCDMA:
	case HW_BAT_CONSUMER_CDMA1X:
	case HW_BAT_CONSUMER_SPEAKER:
	case HW_BAT_CONSUMER_CPU:
	case HW_BAT_CONSUMER_GPS:
	default:
		break;
	}

	if (ret) {
		dev_err(dev, "failed to notify rpc ret=%d\n", ret);
	}

	kfree(work);
}

int huawei_bat_notify(enum huawei_bat_consumer consumer,
	union huawei_bat_state state)
{
	int ret;
	struct huawei_bat_data *data = bat_data;
	struct device *dev;
	struct work_entry *work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (!work)
		return -ENOMEM;

	INIT_WORK((struct work_struct *)work, huawei_bat_work);

	if (!data || IS_ERR_OR_NULL(data->pm_ep) || !data->workqueue)
		return -EPERM;
	work->data = data;

	dev = data->dev;

	if (consumer < 0 || consumer > HW_BAT_CONSUMER_MAX) {
		dev_err(dev, "invalid consumer %d\n", consumer);
		return -EINVAL;
	}
	work->consumer = consumer;
	work->state = state;

	ret = queue_work(data->workqueue, (struct work_struct *)work);
	if (ret < 0) {
		dev_err(dev, "failed to queue work ret=%d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(huawei_bat_notify);

void huawei_bat_charger_connected(enum chg_type chgtype)
{
	int ret;
	struct huawei_bat_data *data = bat_data;

	pr_debug("%s: %d\n", __func__, chgtype);

	data->charger_type = chgtype;

	if (chgtype != USB_CHG_TYPE__INVALID) {
		struct {
			struct rpc_request_hdr hdr;
			uint32_t otg_dev;
		} req;

		memset(&req, 0, sizeof(req));

		req.otg_dev = cpu_to_be32(chgtype);

		ret = msm_rpc_call(data->chg_ep, CHG_SET_CHARGER_CONNECTED,
			&req, sizeof(req), msecs_to_jiffies(5000));
		if (ret)
			pr_err("%s: failed to do rpc call ret=%d",
				__func__, ret);
	} else {
		struct {
			struct rpc_request_hdr hdr;
		} req;

		memset(&req, 0, sizeof(req));

		ret = msm_rpc_call(data->chg_ep, CHG_SET_CHARGER_DISCONNECTED,
			&req, sizeof(req), msecs_to_jiffies(5000));
		if (ret)
			pr_err("%s: failed to do rpc call ret=%d",
				__func__, ret);

		/* Notify no current available. */
		huawei_bat_charger_draw(0);
	}

	power_supply_changed(&data->ac_ps);
	power_supply_changed(&data->usb_ps);
	power_supply_changed(&data->battery_ps);
}
EXPORT_SYMBOL(huawei_bat_charger_connected);

void huawei_bat_charger_draw(unsigned int ma)
{
	int ret;
	struct huawei_bat_data *data = bat_data;

	pr_debug("%s: %d\n", __func__, ma);

	if (ma != 0) {
		struct {
			struct rpc_request_hdr hdr;
			uint32_t i_ma;
		} req;

		memset(&req, 0, sizeof(req));

		req.i_ma = cpu_to_be32(ma);

		ret = msm_rpc_call(data->chg_ep, CHG_SET_CURRENT_AVAILABLE,
			&req, sizeof(req), msecs_to_jiffies(5000));
		if (ret)
			pr_err("%s: failed to do rpc call ret=%d",
				__func__, ret);
	} else {
		struct {
			struct rpc_request_hdr hdr;
		} req;

		memset(&req, 0, sizeof(req));

		ret = msm_rpc_call(data->chg_ep, CHG_SET_CURRENT_UNAVAILABLE,
			&req, sizeof(req), msecs_to_jiffies(5000));
		if (ret)
			pr_err("%s: failed to do rpc call ret=%d",
				__func__, ret);
	}
}
EXPORT_SYMBOL(huawei_bat_charger_draw);

static int huawei_bat_get_capacity(struct huawei_bat_data *data,
	uint8_t *capacity)
{
	int ret;
	int8_t ret_capacity;

	struct {
		struct rpc_request_hdr hdr;
	} req;
	struct {
		struct rpc_reply_hdr hdr;
		u32 battery_level;
	} rep;

	memset(&req, 0, sizeof(req));
	memset(&rep, 0, sizeof(rep));

	ret = msm_rpc_call_reply(data->chg_ep, CHG_GET_CAPACITY_PROC,
		&req, sizeof(req), &rep, sizeof(rep), msecs_to_jiffies(5000));
	if (ret < 0)
		return ret;

	ret_capacity = be32_to_cpu(rep.battery_level);

	if (ret_capacity < 0)
		return ret_capacity;

	*capacity = ret_capacity;

	return 0;
}

static uint8_t huawei_bat_calculate_capacity(struct huawei_bat_data *data)
{
	uint32_t low_voltage = data->pdata->voltage_min_design;
	uint32_t high_voltage = data->pdata->voltage_max_design;
	uint32_t current_voltage = data->battery.voltage;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static int huawei_bat_batt_get(struct huawei_bat_data *data,
	struct huawei_bat_batt *battery)
{
	int ret;

	struct {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req;
	struct {
		struct rpc_reply_hdr hdr;
		u32 more_data;

		u32 charger_status;
		u32 charger_type;
		u32 battery_status;
		u32 battery_level;
		u32 battery_voltage;
		u32 battery_temp;

		u32 charger_valid;
		u32 charger_charging;
		u32 battery_valid;
		u32 ui_event;
	} rep;

	memset(&req, 0, sizeof(req));
	memset(&rep, 0, sizeof(rep));

	req.more_data = cpu_to_be32(1);

	ret = msm_rpc_call_reply(data->chg_ep, CHG_GET_GENERAL_STATUS_PROC,
		&req, sizeof(req), &rep, sizeof(rep), msecs_to_jiffies(5000));
	if (ret < 0)
		return ret;

	battery->voltage = be32_to_cpu(rep.battery_voltage);
	battery->temperature = be32_to_cpu(rep.battery_temp);

	ret = huawei_bat_get_capacity(data, &(battery->capacity));
	if (ret) {
		battery->capacity = huawei_bat_calculate_capacity(data);
	}

	return 0;
}

static int huawei_bat_update(struct huawei_bat_data *data)
{
	int ret;
	struct huawei_bat_batt *battery = &data->battery;

	ret = huawei_bat_batt_get(data, battery);
	if (ret) {
		pr_err("%s: Failed to get battery info ret=%d\n",
			__func__, ret);
		return ret;
	}

	power_supply_changed(&data->battery_ps);

	pr_debug("%s: C:%d V:%d T:%d\n", __func__,
		battery->capacity, battery->voltage, battery->temperature);

	return 0;
}

static int huawei_bat_batt_cb(struct msm_rpc_client *client, void *buffer,
	int in_size)
{
	int ret;
	struct huawei_bat_data *data = bat_data;
	struct rpc_request_hdr *req = buffer;
	uint32_t procedure = be32_to_cpu(req->procedure);
	uint32_t accept_status;

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;
	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		break;
	}

	msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
		accept_status);

	ret = msm_rpc_send_accepted_reply(client, 0);
	if (ret) {
		pr_err("%s: failed to send accepted reply ret=%d\n",
			__func__, ret);
		return ret;
	}

	huawei_bat_update(data);

	return 0;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

static int huawei_bat_batt_register_arg(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req = data;

	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->more_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_error);
	size += sizeof(u32);

	return size;
}

static int huawei_bat_batt_register_ret(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;

	data_ptr = data;
	buf_ptr = buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

static int huawei_bat_batt_register(struct huawei_bat_data *data)
{
	int ret;
	struct batt_client_registration_req req;
	struct batt_client_registration_rep rep;

	memset(&req, 0, sizeof(req));
	memset(&rep, 0, sizeof(rep));

	req.desired_batt_voltage = data->pdata->voltage_min_design;
	req.voltage_direction = BATTERY_ALL_ACTIVITY;
	req.batt_cb_id = BATTERY_CB_ID_ALL_ACTIVITY;
	req.cb_data = BATTERY_ALL_ACTIVITY;
	req.more_data = 1;
	req.batt_error = 0;

	ret = msm_rpc_client_req(data->batt_client, BATTERY_REGISTER_PROC,
		huawei_bat_batt_register_arg, &req,
		huawei_bat_batt_register_ret, &rep,
		msecs_to_jiffies(5000));
	if (ret)
		return ret;

	data->batt_handle = rep.batt_handle;

	return 0;
}

static int huawei_bat_connect_rpc(struct huawei_bat_data *data)
{
	int ret;
	struct device *dev = data->dev;

	/* Connect to Battery RPC service. */
	data->batt_client = msm_rpc_register_client("battery",
		BATTERY_RPC_PROG, BATTERY_RPC_VERS, 1, huawei_bat_batt_cb);
	if (IS_ERR(data->batt_client)) {
		ret = PTR_ERR(data->batt_client);
		dev_err(dev, "failed to connect to batt rpc service ret=%d\n",
			ret);
		goto err;
	}

	/* Connect to CHG RPC service. */
	data->chg_ep = msm_rpc_connect(CHG_RPC_PROG, CHG_RPC_VERS, 0);
	if (IS_ERR(data->chg_ep)) {
		ret = PTR_ERR(data->chg_ep);
		dev_err(dev, "failed to connect to chg rpc service ret=%d\n",
			ret);
		goto err_close_battery;
	}

	/* Connect to PM LIB RPC service. */
	data->pm_ep = msm_rpc_connect(PM_LIB_RPC_PROG, PM_LIB_RPC_VERS, 0);
	if (IS_ERR(data->pm_ep)) {
		ret = PTR_ERR(data->pm_ep);
		dev_err(dev, "failed to connect to pm lib rpc service ret=%d\n",
			ret);
		goto err_close_chg;
	}

	return 0;

err_close_chg:
	msm_rpc_close(data->chg_ep);
	data->chg_ep = NULL;
err_close_battery:
	msm_rpc_unregister_client(data->batt_client);
	data->batt_client = NULL;
err:
	return ret;
}

static void huawei_bat_disconnect_rpc(struct huawei_bat_data *data)
{
	msm_rpc_close(data->pm_ep);
	data->pm_ep = NULL;

	msm_rpc_close(data->chg_ep);
	data->chg_ep = NULL;

	msm_rpc_unregister_client(data->batt_client);
	data->batt_client = NULL;
}

static enum power_supply_property huawei_bat_ps_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
};

static int huawei_bat_ps_get_status(struct huawei_bat_data *data)
{
	if (data->charger_type == USB_CHG_TYPE__INVALID)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (data->battery.capacity == 100)
		return POWER_SUPPLY_STATUS_FULL;

	return POWER_SUPPLY_STATUS_CHARGING;
}

static int huawei_bat_ps_get_health(struct huawei_bat_data *data)
{
	struct huawei_bat_batt *battery = &data->battery;

	/* Check for min/max battery temperatures. */
	if (battery->temperature > 45)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (battery->temperature < 0)
		return POWER_SUPPLY_HEALTH_COLD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int huawei_bat_ps_get_present(struct huawei_bat_data *data)
{
	struct huawei_bat_batt *battery = &data->battery;

	/* Battery voltage should be more than 3V in all batteries. */
	return (battery->voltage >= 3000);
}

static int huawei_bat_ps_get_online(struct huawei_bat_data *data)
{
	/* If battery is present then it is online aswell. */
	return huawei_bat_ps_get_present(data);
}

static int huawei_ac_ps_get_online(struct huawei_bat_data *data)
{
	return data->charger_type == USB_CHG_TYPE__WALLCHARGER;
}

static int huawei_usb_ps_get_online(struct huawei_bat_data *data)
{
	return (data->charger_type == USB_CHG_TYPE__SDP ||
		data->charger_type == USB_CHG_TYPE__CARKIT);
}

static int huawei_bat_ps_get_capacity_level(struct huawei_bat_data *data)
{
	struct huawei_bat_batt *battery = &data->battery;

	if (battery->capacity < 5)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (battery->capacity < 25)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (battery->capacity < 75)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (battery->capacity < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (battery->capacity == 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;

	return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static int huawei_ps_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct huawei_bat_data *data = NULL;

	if (psy->type == POWER_SUPPLY_TYPE_BATTERY)
		data = container_of(psy, struct huawei_bat_data, battery_ps);
	else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		data = container_of(psy, struct huawei_bat_data, ac_ps);
	else if (psy->type == POWER_SUPPLY_TYPE_USB)
		data = container_of(psy, struct huawei_bat_data, usb_ps);
	else
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = huawei_bat_ps_get_status(data);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = huawei_bat_ps_get_health(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = huawei_bat_ps_get_present(data);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_BATTERY)
			val->intval = huawei_bat_ps_get_online(data);
		else if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = huawei_ac_ps_get_online(data);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = huawei_usb_ps_get_online(data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = data->pdata->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = data->pdata->voltage_max_design * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = data->pdata->voltage_min_design * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = data->battery.voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = data->battery.capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = huawei_bat_ps_get_capacity_level(data);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = data->battery.temperature * 10;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *huawei_bat_chg_supplied_to[] = {
	"huawei-battery",
};

static enum power_supply_property huawei_chg_ps_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int huawei_bat_register_ps(struct huawei_bat_data *data)
{
	int ret;
	struct device *dev = data->dev;

	data->battery_ps.name = "huawei-battery";
	data->battery_ps.type = POWER_SUPPLY_TYPE_BATTERY;
	data->battery_ps.properties = huawei_bat_ps_properties;
	data->battery_ps.num_properties = ARRAY_SIZE(huawei_bat_ps_properties);
	data->battery_ps.get_property = huawei_ps_get_property;

	ret = power_supply_register(dev, &data->battery_ps);
	if (ret) {
		dev_err(dev, "failed to register battery ps ret=%d\n", ret);
		goto err;
	}

	data->ac_ps.name = "huawei-ac";
	data->ac_ps.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac_ps.supplied_to = huawei_bat_chg_supplied_to,
	data->ac_ps.num_supplicants = ARRAY_SIZE(huawei_bat_chg_supplied_to),
	data->ac_ps.properties = huawei_chg_ps_properties;
	data->ac_ps.num_properties = ARRAY_SIZE(huawei_chg_ps_properties);
	data->ac_ps.get_property = huawei_ps_get_property;

	ret = power_supply_register(dev, &data->ac_ps);
	if (ret) {
		dev_err(dev, "failed to register ac ps ret=%d\n", ret);
		goto err_free_bat_ps;
	}

	data->usb_ps.name = "huawei-usb";
	data->usb_ps.type = POWER_SUPPLY_TYPE_USB;
	data->usb_ps.supplied_to = huawei_bat_chg_supplied_to,
	data->usb_ps.num_supplicants = ARRAY_SIZE(huawei_bat_chg_supplied_to),
	data->usb_ps.properties = huawei_chg_ps_properties;
	data->usb_ps.num_properties = ARRAY_SIZE(huawei_chg_ps_properties);
	data->usb_ps.get_property = huawei_ps_get_property;

	ret = power_supply_register(dev, &data->usb_ps);
	if (ret) {
		dev_err(dev, "failed to register ac ps ret=%d\n", ret);
		goto err_free_ac_ps;
	}

	return 0;

err_free_ac_ps:
	power_supply_unregister(&data->ac_ps);
err_free_bat_ps:
	power_supply_unregister(&data->battery_ps);
err:
	return ret;
}

static void huawei_bat_unregister_ps(struct huawei_bat_data *data)
{
	power_supply_unregister(&data->usb_ps);
	power_supply_unregister(&data->ac_ps);
	power_supply_unregister(&data->battery_ps);
}

static int huawei_bat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct huawei_bat_platform_data *pdata = dev_get_platdata(dev);
	struct huawei_bat_data *data;
	int ret = 0;

	if (pdev->id != -1) {
		dev_err(dev, "multiple devices not supported\n");
		goto err_exit;
	}

	if (!pdata) {
		dev_err(dev, "no platform data\n");
		goto err_exit;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	bat_data = data;

	platform_set_drvdata(pdev, data);

	data->pdata = pdata;
	data->dev = dev;
	data->charger_type = USB_CHG_TYPE__INVALID;

	/* Connect to RPC services. */
	ret = huawei_bat_connect_rpc(data);
	if (ret) {
		dev_err(dev, "failed to connect rpc ret=%d\n", ret);
		goto err_free;
	}

	ret = huawei_bat_batt_register(data);
	if (ret) {
		dev_err(dev, "failed to register batt ret=%d\n", ret);
		goto err_disconnect;
	}

	data->workqueue = create_workqueue("huawei_bat_queue");
	if (!data->workqueue) {
		dev_err(dev, "failed to create workqueue\n");
		ret = -EPERM;
		goto err_disconnect;
	}

	ret = huawei_bat_register_ps(data);
	if (ret) {
		dev_err(dev, "failed to register ps ret=%d\n", ret);
		goto err_workqueue;
	}

	huawei_bat_update(data);

	return 0;

err_workqueue:
	flush_workqueue(data->workqueue);
	destroy_workqueue(data->workqueue);
err_disconnect:
	huawei_bat_disconnect_rpc(data);
err_free:
	bat_data = NULL;
	kfree(data);
err_exit:
	return ret;
}

static int huawei_bat_remove(struct platform_device *pdev)
{
	struct huawei_bat_data *data = platform_get_drvdata(pdev);

	huawei_bat_unregister_ps(data);

	flush_workqueue(data->workqueue);
	destroy_workqueue(data->workqueue);

	huawei_bat_disconnect_rpc(data);

	bat_data = NULL;
	kfree(data);

	return 0;
}

static int huawei_bat_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int huawei_bat_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver huawei_bat_driver = {
	.probe		= huawei_bat_probe,
	.remove		= huawei_bat_remove,
	.suspend	= huawei_bat_suspend,
	.resume		= huawei_bat_resume,
	.driver		= {
		.name	= "huawei_battery",
		.owner	= THIS_MODULE,
	},
};

static int __init huawei_bat_init(void)
{
	return platform_driver_register(&huawei_bat_driver);
}

static void __exit huawei_bat_exit(void)
{
	platform_driver_unregister(&huawei_bat_driver);
}

module_init(huawei_bat_init);
module_exit(huawei_bat_exit);

MODULE_DESCRIPTION("Huawei Battery Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("Rudolf Tammekivi <rtammekivi@gmail.com>");
