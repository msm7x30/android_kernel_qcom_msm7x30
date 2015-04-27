/*
 * include/linux/synaptics_i2c_rmi4_hw.h
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_SYNAPTICS_I2C_RMI4_HW_H__
#define __LINUX_SYNAPTICS_I2C_RMI4_HW_H__

#define SYNAPTICS_I2C_RMI4_NAME "synaptics_i2c_rmi4"

struct synaptics_i2c_rmi4_hw_platform_data {
	int irq_gpio;
	unsigned int display_x;
	unsigned int display_y;
	unsigned int panel_x;
	unsigned int panel_y;
	int (*power)(bool enable);
	int (*reset)(void);
	int (*init)(void);
	void (*exit)(void);
};

#endif /* __LINUX_SYNAPTICS_I2C_RMI4_HW_H__ */

