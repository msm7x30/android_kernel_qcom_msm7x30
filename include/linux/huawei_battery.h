/*
 * Huawei Battery Driver
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

#ifndef __HUAWEI_BATTERY_H__
#define __HUAWEI_BATTERY_H__

#include <mach/msm_hsusb.h>

enum huawei_bat_consumer {
	HW_BAT_CONSUMER_LCD,
	HW_BAT_CONSUMER_FRONT_CAMERA,
	HW_BAT_CONSUMER_BACK_CAMERA,
	HW_BAT_CONSUMER_CAMERA,
	HW_BAT_CONSUMER_WIFI,
	HW_BAT_CONSUMER_BT,
	HW_BAT_CONSUMER_FM,
	HW_BAT_CONSUMER_ADSP,
	HW_BAT_CONSUMER_CAMERA_FLASH,
	HW_BAT_CONSUMER_KEYPAD,
	HW_BAT_CONSUMER_VIBRATOR,
	HW_BAT_CONSUMER_GSM850_GSM900,
	HW_BAT_CONSUMER_GSM1800_GSM1900,
	HW_BAT_CONSUMER_WCDMA,
	HW_BAT_CONSUMER_CDMA1X,
	HW_BAT_CONSUMER_SPEAKER,
	HW_BAT_CONSUMER_CPU,
	HW_BAT_CONSUMER_GPS,

	HW_BAT_CONSUMER_NONE,

	HW_BAT_CONSUMER_MAX
};

union huawei_bat_state {
	int backlight_level;
	int flash_current;
	bool on;
};

#ifdef CONFIG_BATTERY_HUAWEI
int huawei_bat_notify(enum huawei_bat_consumer consumer,
	union huawei_bat_state state);
void huawei_bat_charger_connected(enum chg_type chgtype);
void huawei_bat_charger_draw(unsigned int ma);
void huawei_bat_charger_charged(bool charged);
#else
static inline int huawei_bat_notify(enum huawei_bat_consumer consumer,
	union huawei_bat_state state)
{
	/* stub */
	return 0;
}
static inline void huawei_bat_charger_connected(enum chg_type chgtype)
{
	/* stub */
}
static inline void huawei_bat_charger_draw(unsigned int ma)
{
	/* stub */
}
#endif

struct huawei_bat_platform_data {
	int technology;
	int voltage_min_design;
	int voltage_max_design;
};

#endif /* __HUAWEI_BATTERY_H__ */
