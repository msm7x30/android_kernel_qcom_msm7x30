/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <mach/msm_iomap.h>
#include <mach/socinfo.h>
#include <asm/mach-types.h>
#include <asm/sizes.h>
#include "idle.h"
#include "pm-boot.h"

static uint32_t *msm_pm_reset_vector;
static uint32_t saved_vector[2];
static void (*msm_pm_boot_before_pc)(unsigned int cpu, unsigned long entry);
static void (*msm_pm_boot_after_pc)(unsigned int cpu);

static int msm_pm_boot_reset_vector_init(uint32_t *reset_vector)
{
	if (!reset_vector)
		return -ENODEV;
	msm_pm_reset_vector = reset_vector;
	mb();

	return 0;
}

static void msm_pm_config_rst_vector_before_pc(unsigned int cpu,
		unsigned long entry)
{
	saved_vector[0] = msm_pm_reset_vector[0];
	saved_vector[1] = msm_pm_reset_vector[1];
	msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
	msm_pm_reset_vector[1] = entry;
}

static void msm_pm_config_rst_vector_after_pc(unsigned int cpu)
{
	msm_pm_reset_vector[0] = saved_vector[0];
	msm_pm_reset_vector[1] = saved_vector[1];
}

void msm_pm_boot_config_before_pc(unsigned int cpu, unsigned long entry)
{
	if (msm_pm_boot_before_pc)
		msm_pm_boot_before_pc(cpu, entry);
}

void msm_pm_boot_config_after_pc(unsigned int cpu)
{
	if (msm_pm_boot_after_pc)
		msm_pm_boot_after_pc(cpu);
}
#define BOOT_REMAP_ENABLE  BIT(0)

int msm_pm_boot_init(struct msm_pm_boot_platform_data *pdata)
{
	int ret = 0;
	void *remapped;

	switch (pdata->mode) {
	case MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS:
		pdata->v_addr = ioremap(pdata->p_addr, PAGE_SIZE);
		/* Fall through */
	case MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT:

		if (!pdata->v_addr)
			return -ENODEV;

		ret = msm_pm_boot_reset_vector_init(pdata->v_addr);
		msm_pm_boot_before_pc = msm_pm_config_rst_vector_before_pc;
		msm_pm_boot_after_pc = msm_pm_config_rst_vector_after_pc;
		break;
	case MSM_PM_BOOT_CONFIG_REMAP_BOOT_ADDR:
		/*
		 * Set the boot remap address and enable remapping of
		 * reset vector
		 */
		if (!pdata->p_addr || !pdata->v_addr)
			return -ENODEV;

		remapped = ioremap_nocache(pdata->p_addr, SZ_8);
		ret = msm_pm_boot_reset_vector_init(remapped);

		__raw_writel((pdata->p_addr | BOOT_REMAP_ENABLE),
				pdata->v_addr);

		msm_pm_boot_before_pc = msm_pm_config_rst_vector_before_pc;
		msm_pm_boot_after_pc = msm_pm_config_rst_vector_after_pc;
		break;
	default:
		__WARN();
	}

	return ret;
}

static int msm_pm_boot_probe(struct platform_device *pdev)
{
	struct msm_pm_boot_platform_data pdata;
	char *key = NULL;
	uint32_t val = 0;
	int ret = 0;
	int flag = 0;

	key = "qcom,mode";
	ret = of_property_read_u32(pdev->dev.of_node, key, &val);
	if (ret) {
		pr_err("Unable to read boot mode Err(%d).\n", ret);
		return -ENODEV;
	}
	pdata.mode = val;

	key = "qcom,phy-addr";
	ret = of_property_read_u32(pdev->dev.of_node, key, &val);
	if (ret && pdata.mode == MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS)
		goto fail;
	if (!ret) {
		pdata.p_addr = val;
		flag++;
	}

	key = "qcom,virt-addr";
	ret = of_property_read_u32(pdev->dev.of_node, key, &val);
	if (ret && pdata.mode == MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT)
		goto fail;
	if (!ret) {
		pdata.v_addr = (void *)val;
		flag++;
	}

	if (pdata.mode == MSM_PM_BOOT_CONFIG_REMAP_BOOT_ADDR && (flag != 2)) {
		key = "addresses for boot remap";
		goto fail;
	}

	return msm_pm_boot_init(&pdata);

fail:
	pr_err("Error reading %s\n", key);
	return -EFAULT;
}

static struct of_device_id msm_pm_match_table[] = {
	{.compatible = "qcom,pm-boot"},
	{},
};

static struct platform_driver msm_pm_boot_driver = {
	.probe = msm_pm_boot_probe,
	.driver = {
		.name = "pm-boot",
		.owner = THIS_MODULE,
		.of_match_table = msm_pm_match_table,
	},
};

static int __init msm_pm_boot_module_init(void)
{
	return platform_driver_register(&msm_pm_boot_driver);
}
module_init(msm_pm_boot_module_init);
