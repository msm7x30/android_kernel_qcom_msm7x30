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
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#ifdef CONFIG_BATTERY_HUAWEI
#include <linux/huawei_battery.h>
#endif

#define ENTER_SLEEP_MODE			0x1000
#define EXIT_SLEEP_MODE				0x1100
#define WRDISBV					0x5100
#define WRCTRLD					0x5500

static bool cabc_enabled = true;

static int mddi_nt35560_panel_on(struct platform_device *pdev)
{
	mddi_host_client_cnt_reset();

	mddi_queue_register_write(EXIT_SLEEP_MODE, 0, TRUE, 0);
	mddi_wait(15);

	return 0;
}

static int mddi_nt35560_panel_off(struct platform_device *pdev)
{
	mddi_queue_register_write(ENTER_SLEEP_MODE, 0, TRUE, 0);

	return 0;
}

static void mddi_nt35560_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	mddi_queue_register_write(WRDISBV, mfd->bl_level, TRUE, 0);
#ifdef CONFIG_BATTERY_HUAWEI
	{
		union huawei_bat_state state;
		state.backlight_level = mfd->bl_level;
		huawei_bat_notify(HW_BAT_CONSUMER_LCD, state);
	}
#endif
}

static ssize_t mddi_nt35560_sysfs_cabc_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cabc_enabled);
}

static ssize_t mddi_nt35560_sysfs_cabc_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int cabc_status = 0;

	ret = kstrtoint(buf, 0, &cabc_status);
	if (ret)
		return ret;

	if (cabc_status) {
		/* CABC_COND[1:0] = 1 */
		mddi_queue_register_write(WRCTRLD, 1, TRUE, 0);
	} else {
		/* CABC_COND[1:0] = 0 */
		mddi_queue_register_write(WRCTRLD, 0, TRUE, 0);
	}

	cabc_enabled = !!cabc_status;

	return count;
}

static struct kobj_attribute mddi_nt35560_sysfs_cabc_attr = {
	.attr = {
		.name = "cabc",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = mddi_nt35560_sysfs_cabc_show,
	.store = mddi_nt35560_sysfs_cabc_store,
};

static struct attribute *mddi_nt35560_sysfs_attrs[] = {
	&mddi_nt35560_sysfs_cabc_attr.attr,
	NULL,
};

static struct attribute_group mddi_nt35560_sysfs_attr_group = {
	.attrs = mddi_nt35560_sysfs_attrs,
};

static int __devinit nt35560_probe(struct platform_device *pdev)
{
	int ret;
	struct platform_device *pthisdev = NULL;
	struct msm_fb_data_type *mfd = NULL;

	if (pdev->id == 0)
		return 0;

	/* CABC_COND[1:0] = 0 */
	mddi_queue_register_write(WRCTRLD, 0, TRUE, 0);
	cabc_enabled = false;

	pthisdev = msm_fb_add_device(pdev);

	mfd = platform_get_drvdata(pthisdev);
	if (!mfd) {
		pr_err("%s: mfd not found\n", __func__);
		return -ENODEV;
	}
	if (!mfd->fbi) {
		pr_err("%s: mfd->fbi not found\n", __func__);
		return -ENODEV;
	}
	if (!mfd->fbi->dev) {
		pr_err("%s: mfd->fbi->dev not found\n", __func__);
		return -ENODEV;
	}

	ret = sysfs_create_group(&mfd->fbi->dev->kobj,
		&mddi_nt35560_sysfs_attr_group);
	if (ret) {
		pr_err("%s: sysfs group creation failed, ret=%d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static struct platform_driver this_driver = {
	.probe = nt35560_probe,
	.driver = {
		.name = "mddi_nt35560_fwvga",
	},
};

static struct msm_fb_panel_data nt35560_panel_data = {
	.on = mddi_nt35560_panel_on,
	.off = mddi_nt35560_panel_off,
	.set_backlight = mddi_nt35560_panel_set_backlight,
};

static struct platform_device this_device = {
	.name = "mddi_nt35560_fwvga",
	.id = 1,
	.dev = {
		.platform_data = &nt35560_panel_data,
	}
};

static int __init mddi_nt35560_fwvga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("mddi_nt35560_fwvga"))
		return -ENODEV;

	if ((0xb9f6 << 16 | 0x5560) != mddi_get_client_id())
		return -ENODEV;
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &nt35560_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 854;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = MDDI_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo->mddi.is_type1 = FALSE;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->lcd.refx100 = 6000;
	pinfo->lcd.v_back_porch = 0;
	pinfo->lcd.v_front_porch = 0;
	pinfo->lcd.v_pulse_width = 22;
	pinfo->lcd.rev = 2;
	pinfo->lcd.vsync_enable = TRUE;
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = (1 * HZ);
	pinfo->bl_max = 255;
	pinfo->bl_min = 12;
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

module_init(mddi_nt35560_fwvga_init);
