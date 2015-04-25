/*
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/delay.h>
#include <linux/pwm.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#ifdef CONFIG_BATTERY_HUAWEI
#include <linux/huawei_battery.h>
#endif

#define ENTER_SLEEP_MODE			0x1000
#define EXIT_SLEEP_MODE				0x1100

#define NT35582_PWM_LEVEL	255
#define NT35582_PWM_FREQ_HZ	21480
#define NT35582_PWM_PERIOD_NSEC	(NSEC_PER_SEC / NT35582_PWM_FREQ_HZ)
#define NT35582_PWM_DUTY_LEVEL	(NT35582_PWM_PERIOD_NSEC / NT35582_PWM_LEVEL)

static struct mddi_panel_platform_data *pdata;
static struct pwm_device *bl_pwm;

static int mddi_nt35582_panel_on(struct platform_device *pdev)
{
	mddi_host_client_cnt_reset();

	mddi_queue_register_write(EXIT_SLEEP_MODE, 0, TRUE, 0);
	mddi_wait(125);

	return 0;
}

static int mddi_nt35582_panel_off(struct platform_device *pdev)
{
	mddi_queue_register_write(ENTER_SLEEP_MODE, 0, TRUE, 0);
	mddi_wait(10);

	return 0;
}

static void mddi_nt35582_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	static bool first_set = true;

	if (first_set) {
		if (pdata) {
			if (pdata->pwm_config)
				pdata->pwm_config();
			bl_pwm = pwm_request(pdata->pwm_channel, "backlight");
			if (bl_pwm == NULL || IS_ERR(bl_pwm)) {
				pr_err("%s: pwm request failed\n", __func__);
				bl_pwm = NULL;
			}
		}
		first_set = false;
	}

	if (!bl_pwm)
		return;

	pwm_config(bl_pwm,
		(NT35582_PWM_DUTY_LEVEL * mfd->bl_level) / NSEC_PER_USEC,
		NT35582_PWM_PERIOD_NSEC / NSEC_PER_USEC);

	if (mfd->bl_level > 0)
		pwm_enable(bl_pwm);
	else
		pwm_disable(bl_pwm);

#ifdef CONFIG_BATTERY_HUAWEI
	{
		union huawei_bat_state state;
		state.backlight_level = mfd->bl_level;
		huawei_bat_notify(HW_BAT_CONSUMER_LCD, state);
	}
#endif
}

static int __devinit nt35582_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		pdata = pdev->dev.platform_data;
		return 0;
	}

	mddi_queue_register_write(0xB101, 0x01, TRUE, 0);
	mddi_queue_register_write(0xB102, 0x82, TRUE, 0);
	mddi_queue_register_write(0xB107, 0x5A, TRUE, 0);
	mddi_queue_register_write(0x4400, 0x00, TRUE, 0);
	mddi_queue_register_write(0x4401, 0xC8, TRUE, 0);

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe = nt35582_probe,
	.driver = {
		.name = "mddi_nt35582_wvga",
	},
};

static struct msm_fb_panel_data nt35582_panel_data = {
	.on = mddi_nt35582_panel_on,
	.off = mddi_nt35582_panel_off,
	.set_backlight = mddi_nt35582_panel_set_backlight,
};

static struct platform_device this_device = {
	.name = "mddi_nt35582_wvga",
	.id = 1,
	.dev = {
		.platform_data = &nt35582_panel_data,
	}
};

static int __init mddi_nt35582_wvga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("mddi_nt35582_wvga"))
		return -ENODEV;

	if ((0xb9f6 << 16 | 0x5582) != mddi_get_client_id())
		return -ENODEV;
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &nt35582_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = MDDI_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo->mddi.is_type1 = TRUE;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 16;
	pinfo->lcd.refx100 = 5400;
	pinfo->lcd.v_back_porch = 6;
	pinfo->lcd.v_front_porch = 0;
	pinfo->lcd.v_pulse_width = 0;
	pinfo->lcd.rev = 1;
	pinfo->lcd.vsync_enable = TRUE;
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = (1 * HZ);
	pinfo->bl_max = 255;
	pinfo->bl_min = 6;
	pinfo->clk_rate = 192000000;
	pinfo->clk_min = 192000000;
	pinfo->clk_max = 192000000;
	pinfo->fb_num = 2;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			__func__);
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

module_init(mddi_nt35582_wvga_init);
