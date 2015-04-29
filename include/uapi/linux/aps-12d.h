/*
 * Copyright (c) 2015, Rudolf Tammekivi.
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

#ifndef __UAPI_LINUX_APS_12D_H__
#define __UAPI_LINUX_APS_12D_H__

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

#endif /* __UAPI_LINUX_APS_12D_H__ */
