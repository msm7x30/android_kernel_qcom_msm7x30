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
 * aps-12d.h
 * APS-12D Ambient Light & Proximity Sensor Header File
 */

#ifndef __APS_12D_H__
#define __APS_12D_H__

/* 12-bit ADC value. */
#define APS_12D_MAX_VALUE	4095
/* APS-12D has a minimum delay of 52.5ms.
 * Increase the delay to 53ms to be safe. */
#define MINIMUM_DELAY_NS	53000000
#define MINIMUM_DELAY_US	(MINIMUM_DELAY_NS / 1000)
#define MINIMUM_DELAY_MS	(MINIMUM_DELAY_US / 1000)
/* The minimum theshold between CONT to ONCE is 1 second. */
#define ONCE_THRESHOLD_NS	1000000000


/* Convert type to status. */
#define TYPE_TO_STATUS_ONCE(type)	(type << 0x1)
#define TYPE_TO_STATUS_CONT(type)	((type << 0x1) + 0x1)

// (if 6 -> 0 : (if cont -> cont : once))
#define STATUS_TO_STATE(status)	\
 	((status == 0x6) ? 0 : \
	((status & 0x1) ? ((status >> 0x1) + 0x5) : ((status >> 0x1) + 0x1)))

#define STATUS_IS_CONTINUOUS(status)	(status & 0x1)

/* The real state in the device. */
enum aps_12d_state {
	APS_12D_STATE_POWER_DOWN = 0,
	APS_12D_STATE_ALS_ONCE,
	APS_12D_STATE_SUR_ONCE,
	APS_12D_STATE_PROX_ONCE,
	APS_12D_STATE_RESERVED,
	APS_12D_STATE_ALS_CONT,
	APS_12D_STATE_SUR_CONT,
	APS_12D_STATE_PROX_CONT,
};

enum aps_12d_range {
	APS_12D_RANGE_0P24_TO_1000 = 0,
	APS_12D_RANGE_0P96_TO_4000,
	APS_12D_RANGE_3P84_TO_16000,
	APS_12D_RANGE_15P36_TO_64000,
};

enum aps_12d_irdr_current {
	APS_12D_IRDR_50 = 0,
	APS_12D_IRDR_25,
	APS_12D_IRDR_12P5,
	APS_12D_IRDR_6P25,
};

enum aps_12d_mod_freq {
	APS_12D_MOD_FREQ_DC = 0,
	APS_12D_MOD_FREQ_NA0,
	APS_12D_MOD_FREQ_NA1,
	APS_12D_MOD_FREQ_39KHZ,
};

enum aps_12d_resolution {
	APS_12D_RES_12 = 0,
};

/* The status used in the driver. */
enum aps_12d_status {
	APS_12D_STATUS_LIGHT_ONCE = 0,
	APS_12D_STATUS_LIGHT_CONT,
	APS_12D_STATUS_SURROUND_ONCE,
	APS_12D_STATUS_SURROUND_CONT,
	APS_12D_STATUS_PROXIMITY_ONCE,
	APS_12D_STATUS_PROXIMITY_CONT,
	APS_12D_STATUS_NONE,
};

/* The type sent from userspace. */
enum aps_12d_sensor_type {
	APS_12D_SENSOR_LIGHT,
	APS_12D_SENSOR_PROXIMITY,
	APS_12D_SENSOR_MAX
};

struct aps_12d_settings {
	enum aps_12d_range range;
	enum aps_12d_irdr_current irdr_current;
	enum aps_12d_mod_freq mod_freq;
	enum aps_12d_resolution resolution;

	bool allow_reconfig;
};

#define APS_MAGIC 'A' | 'P' | 'S'

#define APS_IOCTL_GET_SETTINGS	_IOR(APS_MAGIC, 0x10, struct aps_12d_settings)
#define APS_IOCTL_SET_SETTINGS	_IOW(APS_MAGIC, 0x11, struct aps_12d_settings)

#define APS_IOCTL_GET_STATUS	_IOR(APS_MAGIC, 0x12, enum aps_12d_status)

#define APS_IOCTL_GET_LIGHT_ENABLE		_IOR(APS_MAGIC, 0x20, bool)
#define APS_IOCTL_SET_LIGHT_ENABLE		_IOW(APS_MAGIC, 0x21, bool)
#define APS_IOCTL_GET_PROXIMITY_ENABLE		_IOR(APS_MAGIC, 0x22, bool)
#define APS_IOCTL_SET_PROXIMITY_ENABLE		_IOW(APS_MAGIC, 0x23, bool)

/* Returns nanoseconds. */
#define APS_IOCTL_GET_LIGHT_DELAY		_IOR(APS_MAGIC, 0x30, int64_t)
#define APS_IOCTL_SET_LIGHT_DELAY		_IOW(APS_MAGIC, 0x31, int64_t)
#define APS_IOCTL_GET_PROXIMITY_DELAY		_IOR(APS_MAGIC, 0x32, int64_t)
#define APS_IOCTL_SET_PROXIMITY_DELAY		_IOW(APS_MAGIC, 0x33, int64_t)

#ifdef __KERNEL__
/* struct aps_12d_platform_data
 * @range - ADC data input range min-max
 * @irdr_current - IR led current output
 * @mod_freq - modulation frequency (DC or 39kHz)
 * @resolution - n-bit data output (only 12-bit allowed)
 * @allow_reconfig - allow userspace to modify settings (range for example)
 * @vcc_regulator - the VCC analog power supply
 */
struct aps_12d_platform_data {
	enum aps_12d_range range;
	enum aps_12d_irdr_current irdr_current;
	enum aps_12d_mod_freq mod_freq;
	enum aps_12d_resolution resolution;

	bool allow_reconfig;
	const char *vcc_regulator;
};
#endif /* __KERNEL__ */

#endif /* __APS_12D_H__ */
