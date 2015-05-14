/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_SPM_H
#define __ARCH_ARM_MACH_MSM_SPM_H
enum {
	MSM_SPM_MODE_DISABLED,
	MSM_SPM_MODE_CLOCK_GATING,
	MSM_SPM_MODE_POWER_RETENTION,
	MSM_SPM_MODE_POWER_COLLAPSE,
	MSM_SPM_MODE_NR
};

enum {
	MSM_SPM_L2_MODE_DISABLED = MSM_SPM_MODE_DISABLED,
	MSM_SPM_L2_MODE_RETENTION,
	MSM_SPM_L2_MODE_GDHS,
	MSM_SPM_L2_MODE_POWER_COLLAPSE,
};

#if defined(CONFIG_MSM_SPM_V1)

enum {
	MSM_SPM_REG_SAW_AVS_CTL,
	MSM_SPM_REG_SAW_CFG,
	MSM_SPM_REG_SAW_SPM_CTL,
	MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY,
	MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY,
	MSM_SPM_REG_SAW_SLP_CLK_EN,
	MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN,
	MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN,
	MSM_SPM_REG_SAW_SLP_CLMP_EN,
	MSM_SPM_REG_SAW_SLP_RST_EN,
	MSM_SPM_REG_SAW_SPM_MPM_CFG,
	MSM_SPM_REG_NR_INITIALIZE,

	MSM_SPM_REG_SAW_VCTL = MSM_SPM_REG_NR_INITIALIZE,
	MSM_SPM_REG_SAW_STS,
	MSM_SPM_REG_SAW_SPM_PMIC_CTL,
	MSM_SPM_REG_NR
};

struct msm_spm_platform_data {
	void __iomem *reg_base_addr;
	uint32_t reg_init_values[MSM_SPM_REG_NR_INITIALIZE];

	uint8_t awake_vlevel;
	uint8_t retention_vlevel;
	uint8_t collapse_vlevel;
	uint8_t retention_mid_vlevel;
	uint8_t collapse_mid_vlevel;

	uint32_t vctl_timeout_us;
};

/* Public functions */

/**
 * msm_spm_set_low_power_mode() - Configure SPM start address for low power mode
 * @mode: SPM LPM mode to enter
 * @notify_rpm: Notify RPM in this mode
 */
int msm_spm_set_low_power_mode(unsigned int mode, bool notify_rpm);

/**
 * msm_spm_set_vdd(): Set core voltage
 * @cpu: core id
 * @vlevel: Encoded PMIC data.
 */
int msm_spm_set_vdd(unsigned int cpu, unsigned int vlevel);

/**
 * msm_spm_get_vdd(): Get core voltage
 * @cpu: core id
 * @return: Returns encoded PMIC data.
 */
unsigned int msm_spm_get_vdd(unsigned int cpu);

/**
 * msm_spm_turn_on_cpu_rail(): Power on cpu rail before turning on core
 * @cpu: core id
 */
int msm_spm_turn_on_cpu_rail(unsigned int cpu);

/* Internal low power management specific functions */

/**
 * msm_spm_reinit(): Reinitialize SPM registers
 */
void msm_spm_reinit(void);

/**
 * msm_spm_init(): Board initalization function
 * @data: platform specific SPM register configuration data
 * @nr_devs: Number of SPM devices being initialized
 */
int msm_spm_init(struct msm_spm_platform_data *data, int nr_devs);

/**
 * msm_spm_device_init(): Device tree initialization function
 */
int msm_spm_device_init(void);

#else /* defined(CONFIG_MSM_SPM_V1) */

static inline int msm_spm_set_low_power_mode(unsigned int mode, bool notify_rpm)
{
	return -ENOSYS;
}

static inline int msm_spm_set_vdd(unsigned int cpu, unsigned int vlevel)
{
	return -ENOSYS;
}

static inline unsigned int msm_spm_get_vdd(unsigned int cpu)
{
	return 0;
}

static inline void msm_spm_reinit(void)
{
	/* empty */
}

static inline int msm_spm_turn_on_cpu_rail(unsigned int cpu)
{
	return -ENOSYS;
}

static inline int msm_spm_device_init(void)
{
	return -ENOSYS;
}

#endif  /*defined(CONFIG_MSM_SPM_V1) */
#endif  /* __ARCH_ARM_MACH_MSM_SPM_H */
