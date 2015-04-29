/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lsm303dlh.h
* Authors            : MSH - Motion Mems BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 1.0.1
* Date               : 19/03/2010
* Description        : LSM303DLH 6D module sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

#ifndef __LSM303DLH_H__
#define __LSM303DLH_H__

#include <uapi/linux/lsm303dlh.h>

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Operating Mode */
#define LSM303DLH_ACC_PM_OFF		0x00
#define LSM303DLH_ACC_PM_NORMAL		0x20
#define LSM303DLH_ACC_ENABLE_ALL_AXES	0x07

/* Accelerometer output data rate  */
#define LSM303DLH_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define LSM303DLH_ACC_ODR1		0x60	/* 1Hz output data rate */
#define LSM303DLH_ACC_ODR2		0x80	/* 2Hz output data rate */
#define LSM303DLH_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define LSM303DLH_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define LSM303DLH_ACC_ODR50		0x00	/* 50Hz output data rate */
#define LSM303DLH_ACC_ODR100		0x08	/* 100Hz output data rate */
#define LSM303DLH_ACC_ODR400		0x10	/* 400Hz output data rate */
#define LSM303DLH_ACC_ODR1000		0x18	/* 1000Hz output data rate */

/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetic Sensor Operating Mode */
#define LSM303DLH_MAG_NORMAL_MODE	0x00
#define LSM303DLH_MAG_POS_BIAS		0x01
#define LSM303DLH_MAG_NEG_BIAS		0x02
#define LSM303DLH_MAG_CC_MODE		0x00
#define LSM303DLH_MAG_SC_MODE		0x01
#define LSM303DLH_MAG_SLEEP_MODE	0x03

/* Magnetometer output data rate  */
#define LSM303DLH_MAG_ODR_75		0x00	/* 0.75Hz output data rate */
#define LSM303DLH_MAG_ODR1_5		0x04	/* 1.5Hz output data rate */
#define LSM303DLH_MAG_ODR3_0		0x08	/* 3Hz output data rate */
#define LSM303DLH_MAG_ODR7_5		0x09	/* 7.5Hz output data rate */
#define LSM303DLH_MAG_ODR15		0x10	/* 15Hz output data rate */
#define LSM303DLH_MAG_ODR30		0x14	/* 30Hz output data rate */
#define LSM303DLH_MAG_ODR75		0x18	/* 75Hz output data rate */

struct lsm303dlh_acc_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};

struct lsm303dlh_mag_platform_data {

	int poll_interval;
	int min_interval;

	u8 h_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};

#endif  /* __LSM303DLH_H__ */
