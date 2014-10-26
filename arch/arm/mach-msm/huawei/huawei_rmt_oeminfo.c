/* arch/arm/mach-msm/huawei_rmt_oeminfo.c
 *
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2014  Rudolf Tammekivi <rtammekivi@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <mach/msm_rpcrouter.h>
#include <asm-generic/uaccess.h>

#include "smd_private.h"

#define OEMINFO_BUFFER_SIZE		(64 * 1024)
#define RMT_OEMINFO_WAIT_FOR_REQ	0x5555
#define RMT_OEMINFO_SEND_STATUS		0x6666

struct rmt_oeminfo_data {
	int open_excl;
	atomic_t total_events;
	wait_queue_head_t event_q;
	struct list_head event_list;
	struct list_head data_list;
	spinlock_t lock;
};

struct rmt_oeminfo_type {
	int ver;
	int func_type;
	int oeminfo_type;
	int total_size;
	int return_status;
	char buffer[OEMINFO_BUFFER_SIZE];
};

struct rmt_oeminfo_kevent {
	struct list_head list;
	struct rmt_oeminfo_type *event;
};

struct rmt_oeminfo_kdata {
	struct list_head list;
	uint32_t handle;
	struct msm_rpc_client_info cinfo;
	struct rmt_oeminfo_type *data;
};

static struct rmt_oeminfo_data *_rmt_data = NULL;

static void rmt_oeminfo_put_event(struct rmt_oeminfo_data *data,
	struct rmt_oeminfo_kevent *kevent)
{
	spin_lock(&data->lock);
	list_add_tail(&kevent->list, &data->event_list);
	spin_unlock(&data->lock);
}

static struct rmt_oeminfo_kevent *rmt_oeminfo_get_event(
	struct rmt_oeminfo_data *data)
{
	struct rmt_oeminfo_kevent *kevent = NULL;

	spin_lock(&data->lock);
	if (!list_empty(&data->event_list)) {
		kevent = list_first_entry(&data->event_list,
			struct rmt_oeminfo_kevent, list);
		list_del(&kevent->list);
	}
	spin_unlock(&data->lock);
	return kevent;
}


static void rmt_oeminfo_put_data(struct rmt_oeminfo_data *data,
	struct rmt_oeminfo_kdata *kdata)
{
	spin_lock(&data->lock);
	list_add_tail(&kdata->list, &data->data_list);
	spin_unlock(&data->lock);
}

static struct rmt_oeminfo_kdata *rmt_oeminfo_get_data(
	struct rmt_oeminfo_data *data)
{
	struct rmt_oeminfo_kdata *kdata = NULL;

	spin_lock(&data->lock);
	if (!list_empty(&data->data_list)) {
		kdata = list_first_entry(&data->data_list,
			struct rmt_oeminfo_kdata, list);
		list_del(&kdata->list);
	}
	spin_unlock(&data->lock);
	return kdata;
}

#ifdef DEBUG
static void rmt_oeminfo_print_data(void *data)
{
	char all_log_string[512 + 4];
	char sub_string[12];
	int *my_int_array = NULL;
	int ii;

	memset(all_log_string, 0, sizeof(all_log_string));
	memset(sub_string, 0, sizeof(sub_string));

	my_int_array = (int *)data;

	for (ii = 0; ii < 40; ii++) {
		snprintf(sub_string, sizeof(sub_string),
			"%08X ", my_int_array[ii]);
		strncat(all_log_string, sub_string, sizeof(sub_string));
	}

	pr_debug("%s\n", all_log_string);
}
#endif

int rmt_oeminfo_handle_key(uint32_t key_parm)
{
	struct rmt_oeminfo_data *data = _rmt_data;
	struct rmt_oeminfo_kevent *kevent = NULL;
	struct rmt_oeminfo_kdata *kdata = NULL;
	struct rmt_oeminfo_type *share_ptr = NULL;

	pr_debug("%s: key_parm 0x%x\n", __func__, key_parm);

	kevent = kmalloc(sizeof(*kevent), GFP_KERNEL);
	if (!kevent) {
		pr_err("%s: Failed to allocate kevent memory\n", __func__);
		return -ENOMEM;
	}

	kdata = kmalloc(sizeof(*kdata), GFP_KERNEL);
	if (!kdata) {
		pr_err("%s: Failed to allocate kdata memory\n", __func__);
		return -ENOMEM;
	}

	share_ptr = smem_alloc(SMEM_LCD_CUR_PANEL, sizeof(*share_ptr));
#ifdef DEBUG
	rmt_oeminfo_print_data(share_ptr);
#endif

	kevent->event = share_ptr;

	msm_rpc_server_get_requesting_client(&kdata->cinfo);
	kdata->data = share_ptr;

	rmt_oeminfo_put_event(data, kevent);
	rmt_oeminfo_put_data(data, kdata);
	atomic_inc(&data->total_events);
	wake_up(&data->event_q);

	return 0;
}
EXPORT_SYMBOL(rmt_oeminfo_handle_key);

static long rmt_oeminfo_ioctl(struct file *fp, unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *) arg;
	struct rmt_oeminfo_data *data = _rmt_data;

	struct rmt_oeminfo_kdata *kdata;
	struct rmt_oeminfo_type *share_ptr = NULL;

	pr_debug("%s: cmd 0x%x\n", __func__, cmd);

	switch (cmd) {
	case RMT_OEMINFO_WAIT_FOR_REQ: {
		struct rmt_oeminfo_kevent *kevent = NULL;

		if (atomic_read(&data->total_events) == 0) {
			ret = wait_event_interruptible(data->event_q,
				atomic_read(&data->total_events) != 0);
		}
		if (ret < 0)
			break;
		atomic_dec(&data->total_events);

		kevent = rmt_oeminfo_get_event(data);

		WARN_ON(kevent == NULL);
		if (copy_to_user(argp, kevent->event,
			sizeof(struct rmt_oeminfo_type))) {
			ret = -EFAULT;
		}
		kfree(kevent);
		break;
	}
	case RMT_OEMINFO_SEND_STATUS:
		kdata = rmt_oeminfo_get_data(data);
		share_ptr = smem_alloc(SMEM_LCD_CUR_PANEL,
			sizeof(*share_ptr));

		if (copy_from_user(kdata->data, argp,
			sizeof(struct rmt_oeminfo_type))) {
			ret = -EFAULT;
			break;
		}

#ifdef DEBUG
		rmt_oeminfo_print_data(kdata->data);
		rmt_oeminfo_print_data(share_ptr);
#endif

		kfree(kdata);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rmt_oeminfo_open(struct inode *ip, struct file *fp)
{
	int ret = 0;

	spin_lock(&_rmt_data->lock);

	if (!_rmt_data->open_excl)
		_rmt_data->open_excl = 1;
	else
		ret = -EBUSY;

	spin_unlock(&_rmt_data->lock);
	return ret;
}

static int rmt_oeminfo_release(struct inode *ip, struct file *fp)
{
	spin_lock(&_rmt_data->lock);
	_rmt_data->open_excl = 0;
	spin_unlock(&_rmt_data->lock);
	return 0;
}

static const struct file_operations rmt_oeminfo_fops = {
	.owner		= THIS_MODULE,
	.open		= rmt_oeminfo_open,
	.unlocked_ioctl	= rmt_oeminfo_ioctl,
	.release	= rmt_oeminfo_release,
};

static struct miscdevice rmt_oeminfo_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "rmt_oeminfo",
	.fops	= &rmt_oeminfo_fops,
};

static int rmt_oeminfo_probe(struct platform_device *pdev)
{
	int ret;
	struct rmt_oeminfo_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_exit;
	}

	_rmt_data = data;
	platform_set_drvdata(pdev, data);

	init_waitqueue_head(&data->event_q);
	spin_lock_init(&data->lock);
	atomic_set(&data->total_events, 0);
	INIT_LIST_HEAD(&data->event_list);
	INIT_LIST_HEAD(&data->data_list);

	ret = misc_register(&rmt_oeminfo_device);
	if (ret) {
		pr_err("%s: Failed to register misc device ret=%d\n",
			__func__, ret);
		goto err_free_mem;
	}

	return 0;

err_free_mem:
	_rmt_data = NULL;
	kfree(data);
err_exit:
	return ret;
}

static int rmt_oeminfo_remove(struct platform_device *pdev)
{
	struct rmt_oeminfo_data *data = platform_get_drvdata(pdev);

	misc_deregister(&rmt_oeminfo_device);
	_rmt_data = NULL;
	kfree(data);

	return 0;
}

static struct platform_driver rmt_oeminfo_driver = {
	.probe		= rmt_oeminfo_probe,
	.remove		= rmt_oeminfo_remove,
	.driver		= {
		.name	= "rmt_oeminfo",
		.owner	= THIS_MODULE,
	},
};

static int __init rmt_oeminfo_init(void)
{
	return platform_driver_register(&rmt_oeminfo_driver);
}

static void __exit rmt_oeminfo_exit(void)
{
	platform_driver_unregister(&rmt_oeminfo_driver);
}

module_init(rmt_oeminfo_init);
module_exit(rmt_oeminfo_exit);

MODULE_DESCRIPTION("Huawei Remote Oeminfo Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("Rudolf Tammekivi <rtammekivi@gmail.com>");
