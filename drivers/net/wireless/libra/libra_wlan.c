/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/libra_sdioif.h>
#ifdef CONFIG_LIBRA_MEM_PRE_ALLOC
#include "libra_prealloc.h"
#endif

#define DEVICE "libra_wlan"
#define VERSION "1.00"

static int __init libra_wlan_init(void)
{
	int ret = 0;

#ifdef CONFIG_LIBRA_MEM_PRE_ALLOC
	ret = libra_prealloc_init();
	if (ret < 0)
		pr_err("libra: pre-allocation failed\n");
#endif

	return ret;
}

static void __exit libra_wlan_exit(void)
{
#ifdef CONFIG_LIBRA_MEM_PRE_ALLOC
	libra_prealloc_deinit();
#endif
}

module_init(libra_wlan_init);
module_exit(libra_wlan_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION(DEVICE "Driver");
