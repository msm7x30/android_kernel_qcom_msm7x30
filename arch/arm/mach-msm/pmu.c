/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <asm/pmu.h>
#include <mach/irqs.h>
#include <mach/socinfo.h>

/*
 * Types of PMUs that can be accessed directly and require mutual
 * exclusion between profiling tools.
 */
enum arm_pmu_type {
	ARM_PMU_DEVICE_CPU	= 0,
	ARM_PMU_DEVICE_L2CC	= 1,
	ARM_NUM_PMU_DEVICES,
};

static struct resource cpu_pmu_resource[] = {
	{
		.start = INT_ARMQC_PERFMON,
		.end = INT_ARMQC_PERFMON,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device cpu_pmu_device = {
	.name		= "cpu-arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= cpu_pmu_resource,
	.num_resources	= ARRAY_SIZE(cpu_pmu_resource),
};


static struct platform_device *pmu_devices[] = {
	&cpu_pmu_device,
};

static int __init msm_pmu_init(void)
{
	return platform_add_devices(pmu_devices, ARRAY_SIZE(pmu_devices));
}

arch_initcall(msm_pmu_init);
