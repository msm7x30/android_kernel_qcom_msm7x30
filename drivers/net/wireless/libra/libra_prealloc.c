/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/libra_sdioif.h>

static DEFINE_MUTEX(alloc_lock);

struct libra_prealloc {
	int occupied;
	unsigned int size;
	void *ptr;
};

/* pre-alloced mem for WLAN driver */
static struct libra_prealloc libra_allocs[] = {
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 8  * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
};

int libra_prealloc_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(libra_allocs); i++) {
		libra_allocs[i].occupied = 0;
		libra_allocs[i].ptr = kmalloc(libra_allocs[i].size, GFP_KERNEL);
		if (libra_allocs[i].ptr == NULL)
			return -ENOMEM;
	}

	return 0;
}

void libra_prealloc_deinit(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(libra_allocs); i++)
		kfree(libra_allocs[i].ptr);
}

void *libra_prealloc_get(unsigned int size)
{
	int i = 0;

	mutex_lock(&alloc_lock);
	for (i = 0; i < ARRAY_SIZE(libra_allocs); i++) {
		if (libra_allocs[i].occupied)
			continue;

		if (libra_allocs[i].size > size) {
			/* we found the slot */
			libra_allocs[i].occupied = 1;
			mutex_unlock(&alloc_lock);
			return libra_allocs[i].ptr;
		}
	}
	pr_err("libra: %s: prealloc not available\n", __func__);
	mutex_unlock(&alloc_lock);

	return NULL;
}
EXPORT_SYMBOL(libra_prealloc_get);

int libra_prealloc_put(void *ptr)
{
	int i = 0;

	mutex_lock(&alloc_lock);
	for (i = 0; i < ARRAY_SIZE(libra_allocs); i++) {
		if (libra_allocs[i].ptr == ptr) {
			libra_allocs[i].occupied = 0;
			mutex_unlock(&alloc_lock);
			return 1;
		}
	}
	mutex_unlock(&alloc_lock);

	return 0;
}
EXPORT_SYMBOL(libra_prealloc_put);
