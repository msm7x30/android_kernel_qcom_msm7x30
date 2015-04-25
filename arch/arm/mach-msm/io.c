/* arch/arm/mach-msm/io.c
 *
 * MSM7X30 io support
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, The Linux Foundation. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/export.h>

#include <asm/page.h>
#include <mach/msm_iomap.h>
#include <asm/mach/map.h>
#include <linux/dma-mapping.h>

#include <mach/board.h>

#define MSM_CHIP_DEVICE(name, chip) { \
		.virtual = (unsigned long) MSM_##name##_BASE, \
		.pfn = __phys_to_pfn(chip##_##name##_PHYS), \
		.length = chip##_##name##_SIZE, \
		.type = MT_DEVICE, \
	 }

#define MSM_DEVICE(name) MSM_CHIP_DEVICE(name, MSM)

/* msm_shared_ram_phys default value of 0x00100000 is the most common value
 * and should work as-is for any target without stacked memory.
 */
unsigned int msm_shared_ram_phys = 0x00100000;

static void __init msm_map_io(struct map_desc *io_desc, int size)
{
	int i;

	BUG_ON(!size);
	for (i = 0; i < size; i++)
		if (io_desc[i].virtual == (unsigned long)MSM_SHARED_RAM_BASE)
			io_desc[i].pfn = __phys_to_pfn(msm_shared_ram_phys);

	iotable_init(io_desc, size);
}

#ifdef CONFIG_ARCH_MSM7X30
static struct map_desc msm7x30_io_desc[] __initdata = {
	MSM_CHIP_DEVICE(VIC, MSM7X30),
	MSM_CHIP_DEVICE(CSR, MSM7X30),
	MSM_CHIP_DEVICE(TMR, MSM7X30),
	MSM_CHIP_DEVICE(GPIO1, MSM7X30),
	MSM_CHIP_DEVICE(GPIO2, MSM7X30),
	MSM_CHIP_DEVICE(CLK_CTL, MSM7X30),
	MSM_CHIP_DEVICE(CLK_CTL_SH2, MSM7X30),
	MSM_CHIP_DEVICE(AD5, MSM7X30),
	MSM_CHIP_DEVICE(MDC, MSM7X30),
	MSM_CHIP_DEVICE(ACC0, MSM7X30),
	MSM_CHIP_DEVICE(SAW0, MSM7X30),
	MSM_CHIP_DEVICE(APCS_GCC, MSM7X30),
	MSM_CHIP_DEVICE(TCSR, MSM7X30),
#if defined(CONFIG_DEBUG_MSM_UART1) || defined(CONFIG_DEBUG_MSM_UART2) || \
	defined(CONFIG_DEBUG_MSM_UART3)
	MSM_DEVICE(DEBUG_UART),
#endif
	{
		.virtual =  (unsigned long) MSM_SHARED_RAM_BASE,
		.length =   MSM_SHARED_RAM_SIZE,
		.type =     MT_DEVICE,
	},
};

void __init msm_map_msm7x30_io(void)
{
	msm_map_io(msm7x30_io_desc, ARRAY_SIZE(msm7x30_io_desc));
}
#endif /* CONFIG_ARCH_MSM7X30 */
