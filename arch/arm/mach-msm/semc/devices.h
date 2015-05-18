/* linux/arch/arm/mach-msm/devices.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_DEVICES_H
#define __ARCH_ARM_MACH_MSM_DEVICES_H

/* board-semc7x30-devices.c */
extern struct platform_device asoc_msm_dai0;
extern struct platform_device asoc_msm_dai1;
extern struct platform_device asoc_msm_pcm;
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
extern struct platform_device asoc_mvs_dai0;
extern struct platform_device asoc_mvs_dai1;
extern struct platform_device asoc_msm_mvs;
#endif
extern struct platform_device msm7x30_device_acpuclk;
extern struct platform_device msm_adsp_device;
extern struct platform_device msm_device_dmov;
extern struct platform_device msm_device_gadget_peripheral;
extern struct platform_device msm_device_i2c;
extern struct platform_device msm_device_i2c_2;
extern struct platform_device msm_device_nand;
extern struct platform_device msm_device_otg;
extern struct platform_device msm_device_smd;
extern struct platform_device msm_device_ssbi7;
extern struct platform_device msm_device_ssbi_pmic1;
extern struct platform_device msm_device_uart3;
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

void __init msm_fb_register_device(char *name, void *data);

/* board-<device>.c */
#ifdef CONFIG_PSTORE_RAM
void __init reserve_ramoops_memory(void);
extern struct platform_device ramoops_dev;
#endif

#endif /* __ARCH_ARM_MACH_MSM_DEVICES_H */
