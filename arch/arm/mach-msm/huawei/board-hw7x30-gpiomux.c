/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"

#include "board-hw7x30.h"

static struct gpiomux_setting mdp_vsync_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting audio_pamp_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting ts_resout_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting ts_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting i2c_dcdc_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config msm7x30_standard_configs[] __initdata = {
	{	/* MDP_VSYNC */
		.gpio = GPIO_MDP_IRQ,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdp_vsync_cfg,
		},
	},
	{	/* AUDIO_PAMP */
		.gpio = GPIO_AMP_EN,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_pamp_cfg,
		},
	},
	{	/* TOUCH_RESET */
		.gpio = GPIO_TS_RST,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ts_resout_sus_cfg,
		},
	},
	{	/* TOUCH_INT */
		.gpio = GPIO_TS_IRQ,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ts_int_sus_cfg,
		},
	},
	{	/* I2C_SCL_DCDC */
		.gpio = GPIO_DCDC_SCL,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_dcdc_sus_cfg,
		},
	},
	{	/* I2C_SDA_DCDC */
		.gpio = GPIO_DCDC_SDA,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_dcdc_sus_cfg,
		},
	},
};

#ifdef CONFIG_MACH_HUAWEI_U8860
static struct gpiomux_setting apds_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting akm_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config u8860_standard_configs[] __initdata = {
	{
		/* APDS_INT */
		.gpio = GPIO_APDS_IRQ,
		.settings = {
			[GPIOMUX_SUSPENDED] = &apds_int_sus_cfg,
		},
	},
	{
		/* AKM_INT */
		.gpio = GPIO_AKM_IRQ,
		.settings = {
			[GPIOMUX_SUSPENDED] = &akm_int_sus_cfg,
		},
	},
};
#endif

static int __init msm7x30_init_gpiomux(void)
{
	int rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "%s failed %d\n", __func__, rc);
		return rc;
	}

	msm_gpiomux_install(msm7x30_standard_configs,
		ARRAY_SIZE(msm7x30_standard_configs));

#ifdef CONFIG_MACH_HUAWEI_U8860
	msm_gpiomux_install(u8860_standard_configs,
		ARRAY_SIZE(u8860_standard_configs));
#endif

	return rc;
}

postcore_initcall(msm7x30_init_gpiomux);
