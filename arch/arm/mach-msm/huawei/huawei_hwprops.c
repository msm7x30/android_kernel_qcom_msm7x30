/*
 * Copyright (c) 2014, Rudolf Tammekivi.
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

#include <asm/setup.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <mach/rpc_nv.h>
#include <asm/processor.h>

#define SERIALNO_SIZE	17
#define MACADDR_SIZE	6

struct hwprops_data {
	bool opened;

	uint8_t bt_mac_address[MACADDR_SIZE];
	uint8_t wlan_mac_address[MACADDR_SIZE];
	char serialno[SERIALNO_SIZE];
};

static struct hwprops_data *data = NULL;

struct factory_info {
	char meid[SERIALNO_SIZE];	/* Not null-terminated. */
	uint64_t unknown1;
	char serialno[SERIALNO_SIZE];	/* Not null-terminated. */
	uint64_t unknown2;
	char test[50];			/* Factory test info. */
} __attribute__((packed));
static void hwprops_get_rpc_serialno(void)
{
	int rc;
	struct factory_info finfo;
	nv_cmd_item_type item;
	char serialno[SERIALNO_SIZE];

	/* Read factory info from NV. */
	rc = msm_nv_read(NV_FACTORY_INFO_I, &item);
	if (rc) {
		pr_err("%s: failed to read nv %d %d\n", __func__,
			NV_FACTORY_INFO_I, rc);
		return;
	}

	memcpy(&finfo, &item.factory_info, sizeof(finfo));

	/* Copy over to properly null-terminate. */
	strncpy(serialno, finfo.serialno, SERIALNO_SIZE);
	serialno[SERIALNO_SIZE-1] = '\0';

	strncpy(data->serialno, serialno, SERIALNO_SIZE);

	pr_debug("%s: %s\n", __func__, data->serialno);
}

static void hwprops_get_rpc_btmac(void)
{
	int rc;
	nv_cmd_item_type item;
	int i;

	/* Read BT MAC address from NV. */
	rc = msm_nv_read(NV_BD_ADDR_I, &item);
	if (rc) {
		pr_err("%s: failed to read nv %d %d\n", __func__,
			NV_FACTORY_INFO_I, rc);
		return;
	}

	for (i = 0; i < MACADDR_SIZE; i++)
		data->bt_mac_address[(MACADDR_SIZE - 1) - i] = item.bd_addr[i];

	pr_debug("%s: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		data->bt_mac_address[0], data->bt_mac_address[1],
		data->bt_mac_address[2], data->bt_mac_address[3],
		data->bt_mac_address[4], data->bt_mac_address[5]);
}

static void hwprops_get_rpc_wlanmac(void)
{
	int rc;
	nv_cmd_item_type item;
	int i;

	/* Read WLAN MAC address from NV. */
	rc = msm_nv_read(NV_WLAN_MAC_ADDRESS_I, &item);
	if (rc) {
		printk("%s: failed to read nv %d %d\n", __func__,
			NV_FACTORY_INFO_I, rc);
		return;
	}

	for (i = 0; i < MACADDR_SIZE; i++) {
		data->wlan_mac_address[(MACADDR_SIZE - 1) - i] =
			item.wlan_mac_address[i];
	}

	pr_debug("%s: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		data->wlan_mac_address[0], data->wlan_mac_address[1],
		data->wlan_mac_address[2], data->wlan_mac_address[3],
		data->wlan_mac_address[4], data->wlan_mac_address[5]);
}

static int __init hwprops_fixup_serialno(void)
{
	/* Some bootloaders provide it.*/
	if (strstr(saved_command_line, "androidboot.serialno") != NULL)
		return 0;

	/* Add serialno to the command line. */
	strlcat(saved_command_line, " androidboot.serialno=",
		COMMAND_LINE_SIZE);
	strlcat(saved_command_line, data->serialno, COMMAND_LINE_SIZE);

	return 0;
}

static char *replace_str(char *str, char *orig, char *rep)
{
	static char buffer[4096];
	char *p;

	/* Is 'orig' even in 'str'? */
	if(!(p = strstr(str, orig)))
		return str;

	/* Copy characters from 'str' start to 'orig' st$ */
	strncpy(buffer, str, p-str);
	buffer[p-str] = '\0';

	sprintf(buffer+(p-str), "%s%s", rep, p+strlen(orig));

	return buffer;
}

#if defined(CONFIG_MACH_HUAWEI_U8800) || defined(CONFIG_MACH_HUAWEI_U8800PRO)
#define ATAG_CHARGE_FLAG  0x4d534D77
#elif defined(CONFIG_MACH_HUAWEI_U8860)
#define ATAG_CHARGE_FLAG  0x4d534D78
#endif

static int charge_tag = 0;
static int __init parse_tag_charge(const struct tag *tags)
{
	struct tag *t = (struct tag *)tags;
	charge_tag = t->u.revision.rev;
	return 0;
}
__tagtable(ATAG_CHARGE_FLAG, parse_tag_charge);

static int __init hwprops_fixup_charger(void)
{
	/* Bootloader boots recovery when phone boots due to charger. */
	if (charge_tag) {
		saved_command_line = replace_str(saved_command_line,
			"androidboot.mode=recovery","androidboot.mode=charger");
		saved_command_line = replace_str(saved_command_line,
			"androidboot.mode=user","androidboot.mode=charger");
	}
	return 0;
}

static ssize_t hwprops_sysfs_attr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (!strcmp(attr->attr.name, "serialno")) {
		return sprintf(buf, "%s\n", data->serialno);
	} else if (!strcmp(attr->attr.name, "btmac")) {
		return sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			data->bt_mac_address[0], data->bt_mac_address[1],
			data->bt_mac_address[2], data->bt_mac_address[3],
			data->bt_mac_address[4], data->bt_mac_address[5]);
	} else if (!strcmp(attr->attr.name, "wlanmac")) {
		return sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			data->wlan_mac_address[0], data->wlan_mac_address[1],
			data->wlan_mac_address[2], data->wlan_mac_address[3],
			data->wlan_mac_address[4], data->wlan_mac_address[5]);
	}

	return -EINVAL;
}

static ssize_t hwprops_sysfs_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (!strcmp(attr->attr.name, "serialno")) {
		char serialno[SERIALNO_SIZE];
		if (sscanf(buf, "%16s", serialno) != 1)
			return -EINVAL;
		strncpy(data->serialno, serialno, SERIALNO_SIZE);
	} else if (!strcmp(attr->attr.name, "btmac")) {
		uint8_t bt_mac_address[MACADDR_SIZE];
		if (sscanf(buf, "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
			&bt_mac_address[0], &bt_mac_address[1],
			&bt_mac_address[2], &bt_mac_address[3],
			&bt_mac_address[4], &bt_mac_address[5])
			!= 6)
			return -EINVAL;
		memcpy(data->bt_mac_address, bt_mac_address,
			sizeof(bt_mac_address));
	} else if (!strcmp(attr->attr.name, "wlanmac")) {
		uint8_t wlan_mac_address[MACADDR_SIZE];
		if (sscanf(buf, "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
			&wlan_mac_address[0], &wlan_mac_address[1],
			&wlan_mac_address[2], &wlan_mac_address[3],
			&wlan_mac_address[4], &wlan_mac_address[5])
			!= 6)
			return -EINVAL;
		memcpy(data->wlan_mac_address, wlan_mac_address,
			sizeof(wlan_mac_address));
	} else
		return -EINVAL;
	return count;
}

static struct kobj_attribute hwprops_sysfs_serialno_attr = {
	.attr = {
		.name = "serialno",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = hwprops_sysfs_attr_show,
	.store = hwprops_sysfs_attr_store,
};

static struct kobj_attribute hwprops_sysfs_btmac_attr = {
	.attr = {
		.name = "btmac",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = hwprops_sysfs_attr_show,
	.store = hwprops_sysfs_attr_store,
};

static struct kobj_attribute hwprops_sysfs_wlanmac_attr = {
	.attr = {
		.name = "wlanmac",
		.mode = S_IRUGO | S_IWUSR,
	},
	.show = hwprops_sysfs_attr_show,
	.store = hwprops_sysfs_attr_store,
};

static ssize_t hwprops_sysfs_reset_attr_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	/* Initialize NV RPC client. */
	ret = msm_nv_rpc_connect();
	if (ret) {
		pr_err("%s: failed to connect ret=%d\n", __func__, ret);
		return ret;
	}
	data->opened = true;

	/* Get all NV RPC props. */
	hwprops_get_rpc_serialno();
	hwprops_get_rpc_btmac();
	hwprops_get_rpc_wlanmac();

	/* Exit NV RPC client. */
	ret = msm_nv_rpc_close();
	if (ret) {
		pr_err("%s: failed to close ret=%d\n", __func__, ret);
		return ret;
	}
	data->opened = false;

	return count;
}

static struct kobj_attribute hwprops_sysfs_reset_attr = {
	.attr = {
		.name = "reset",
		.mode = S_IWUSR,
	},
	.store = hwprops_sysfs_reset_attr_store,
};

static struct attribute *hwprops_sysfs_attrs[] = {
	&hwprops_sysfs_serialno_attr.attr,
	&hwprops_sysfs_btmac_attr.attr,
	&hwprops_sysfs_wlanmac_attr.attr,
	&hwprops_sysfs_reset_attr.attr,
	NULL
};

static struct attribute_group hwprops_sysfs_attr_group = {
	.attrs = hwprops_sysfs_attrs,
};

static int __init hwprops_init_sysfs(void)
{
	int ret = 0;
	static struct kobject *hwprops_kobj;

	/* Already registered. */
	if (hwprops_kobj)
		return ret;

	hwprops_kobj = kobject_create_and_add("hwprops", NULL);
	if (!hwprops_kobj)
		return -ENODEV;

	ret = sysfs_create_group(hwprops_kobj, &hwprops_sysfs_attr_group);

	return ret;
}

static int __init hwprops_init(void)
{
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	/* Initialize NV RPC client. */
	ret = msm_nv_rpc_connect();
	if (ret) {
		pr_err("%s: failed to connect ret=%d\n", __func__, ret);
		return ret;
	}
	data->opened = true;

	/* Get all NV RPC props. */
	hwprops_get_rpc_serialno();
	hwprops_get_rpc_btmac();
	hwprops_get_rpc_wlanmac();

	ret = hwprops_fixup_serialno();
	if (ret) {
		pr_err("%s: failed to fixup serialno ret=%d\n", __func__, ret);
		goto err;
	}

	ret = hwprops_fixup_charger();
	if (ret) {
		pr_err("%s: failed to fixup charger ret=%d\n", __func__, ret);
		goto err;
	}

	ret = hwprops_init_sysfs();
	if (ret) {
		pr_err("%s: failed to init sysfs ret=%d\n", __func__, ret);
		goto err;
	}

	/* Exit NV RPC client. */
	ret = msm_nv_rpc_close();
	if (ret) {
		pr_err("%s: failed to close ret=%d\n", __func__, ret);
		return ret;
	}
	data->opened = false;

	return 0;
err:
	msm_nv_rpc_close();
	data->opened = false;
	return ret;
}

late_initcall(hwprops_init);

int hwprops_get_wlanmac(uint8_t *wlanmac)
{
	uint8_t cmp_zero[6] = {0};
	if (!data)
		return -EINVAL;
	/* Check if MAC is zero. */
	if (!memcmp(data->wlan_mac_address, &cmp_zero,
		sizeof(data->wlan_mac_address)))
		return -EINVAL;
	memcpy(wlanmac, data->wlan_mac_address, sizeof(data->wlan_mac_address));
	return 0;
}
EXPORT_SYMBOL(hwprops_get_wlanmac);
