/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HW7X30_H__
#define __ARCH_ARM_MACH_MSM_BOARD_HW7X30_H__

/* board-hw7x30-devices.c */
extern struct platform_device asoc_msm_dai0;
extern struct platform_device asoc_msm_dai1;
extern struct platform_device asoc_msm_pcm;
extern struct platform_device msm7x30_device_acpuclk;
extern struct platform_device msm_adsp_device;
extern struct platform_device msm_device_dmov;
extern struct platform_device msm_device_gadget_peripheral;
extern struct platform_device msm_device_i2c;
extern struct platform_device msm_device_i2c_2;
extern struct platform_device msm_device_otg;
extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_ssbi7;
extern struct platform_device msm_device_ssbi_pmic1;
extern struct platform_device msm_device_uart2;
extern struct platform_device msm_device_uart_dm1;
extern struct platform_device msm_device_vidc_720p;
extern struct platform_device msm_ebi0_thermal;
extern struct platform_device msm_ebi1_thermal;
extern struct platform_device *msm_footswitch_devices[];
extern unsigned msm_num_footswitch_devices;
extern struct platform_device msm_kgsl_2d0;
extern struct platform_device msm_kgsl_3d0;
extern struct platform_device msm_rotator_device;
extern struct platform_device qup_device_i2c;
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
extern struct platform_device asoc_mvs_dai0;
extern struct platform_device asoc_mvs_dai1;
extern struct platform_device asoc_msm_mvs;
#endif

void __init msm_fb_register_device(char *name, void *data);

/* board-hw7x30-regulator.c */
extern struct proccomm_regulator_platform_data msm7x30_proccomm_regulator_data;

#if defined(CONFIG_MACH_HUAWEI_U8800PRO) || defined(CONFIG_MACH_HUAWEI_U8860)
extern void (*wifi_status_cb)(int card_present, void *dev_id);
extern void *wifi_status_cb_devid;
extern int bcm_wifi_cd; /* WIFI virtual 'card detect' status */
#endif

/* Common GPIOs */
#define GPIO_MDP_IRQ	30

#define GPIO_AMP_EN	82

#define GPIO_TS_RST	85
#define GPIO_TS_IRQ	148

#define GPIO_DCDC_SCL	149
#define GPIO_DCDC_SDA	150

/* U8860 GPIOs */
#ifdef CONFIG_MACH_HUAWEI_U8860
#define GPIO_APDS_IRQ	89
#define GPIO_AKM_IRQ	132
#endif

#endif /* __ARCH_ARM_MACH_MSM_BOARD_HW7X30_H__ */
