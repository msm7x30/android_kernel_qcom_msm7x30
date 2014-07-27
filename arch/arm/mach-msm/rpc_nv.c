/* linux/arch/arm/mach-msm/rpc_nv.c
 *
 * Copyright (c) 2008-2009, PANTECH, Co. Ltd. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <mach/rpc_nv.h>

#define	MSM_RPC_NV_PROG		0x3000000e
#define NV_VERS_9_2		0x00090002
#define NV_VERS_6_1   		0x00060001
#define	NV_VERS_2_1		0x00020001
#define	NV_VERS_1_1		0x00010001

struct msm_nv_rpc_ids {
	unsigned long prog;
	unsigned long vers_comp;
	unsigned long cmd_remote;
};

static struct msm_rpc_endpoint *nv_ep;
static struct msm_nv_rpc_ids nv_rpc_ids;

static int msm_nv_init_rpc_ids(unsigned int vers)
{
	if (vers == NV_VERS_6_1) {
		nv_rpc_ids.prog		= 0x3000000e;
		nv_rpc_ids.vers_comp	= NV_VERS_6_1;
		nv_rpc_ids.cmd_remote	= 9;
		return 0;
	} else if (vers == NV_VERS_2_1) {
		nv_rpc_ids.prog		= 0x3000000e;
		nv_rpc_ids.vers_comp	= NV_VERS_2_1;
		nv_rpc_ids.cmd_remote	= 9;
		return 0;
	} else if (vers == NV_VERS_1_1) {
		nv_rpc_ids.prog		= 0x3000000e;
		nv_rpc_ids.vers_comp	= NV_VERS_1_1;
		nv_rpc_ids.cmd_remote	= 9;
		return 0;
	} else if (vers == NV_VERS_9_2) {
		nv_rpc_ids.prog		= 0x3000000e;
		nv_rpc_ids.vers_comp	= NV_VERS_9_2;
		nv_rpc_ids.cmd_remote	= 9;
		return 0;
	} else {
		pr_err("%s: no matches found for version (0x%x)\n",
			__func__, vers);
		return -ENODATA;
	}
}

/* rpc connect for nv */
int msm_nv_rpc_connect(void)
{
	if (!IS_ERR_OR_NULL(nv_ep)) {
		pr_debug("%s: nv_ep already connected\n", __func__);
		return 0;
	}

	/* Initialize rpc ids */
	if (msm_nv_init_rpc_ids(NV_VERS_9_2)) {
		pr_err("%s: rpc ids(0x%x) initialization failed\n",
			__func__, NV_VERS_9_2);
		return -ENODATA;
	}

	nv_ep = msm_rpc_connect_compatible(nv_rpc_ids.prog,
		nv_rpc_ids.vers_comp, MSM_RPC_UNINTERRUPTIBLE);

	if (IS_ERR_OR_NULL(nv_ep)) {
		pr_err("%s: connect compatible failed vers = %lx\n",
			__func__, nv_rpc_ids.vers_comp);
		if (msm_nv_init_rpc_ids(NV_VERS_1_1)) {
			pr_err("%s: rpc ids(0x%x) initialization failed\n",
				__func__, NV_VERS_1_1);
			return -ENODATA;
		}
		nv_ep = msm_rpc_connect_compatible(nv_rpc_ids.prog,
			nv_rpc_ids.vers_comp, MSM_RPC_UNINTERRUPTIBLE);
	}

	if (IS_ERR_OR_NULL(nv_ep)) {
		pr_err("%s: connect compatible failed vers = %lx\n",
			__func__, nv_rpc_ids.vers_comp);
		return -EAGAIN;
	} else
		pr_debug("%s: rpc connect success vers = %lx\n",
			__func__, nv_rpc_ids.vers_comp);

	return 0;
}
EXPORT_SYMBOL(msm_nv_rpc_connect);

static int msm_nv_check_validity(uint32_t item)
{
	int rc = -EINVAL;

	switch(item) {
	case NV_FACTORY_INFO_I:
	case NV_BD_ADDR_I:
	case NV_WLAN_MAC_ADDRESS_I:
		rc = 0;
		break;
	}

	return rc;
}

static void msm_nv_set_req_data(int item, nv_cmd_generic_cmd *cmd,
	nv_cmd_item_type *data, int *reqsz, int *repsz)
{
	nv_cmd_item_type *ptr = (nv_cmd_item_type *)cmd->req.data;
	int req_hdr_sz = sizeof(nv_cmd_req_header);
	int rep_hdr_sz = sizeof(nv_cmd_rep_header);

	switch (item) {
	case NV_FACTORY_INFO_I:
		*reqsz = req_hdr_sz + sizeof(ptr->factory_info);
		*repsz = rep_hdr_sz + sizeof(ptr->factory_info);
		break;
	case NV_BD_ADDR_I:
		*reqsz = req_hdr_sz + sizeof(ptr->bd_addr);
		*repsz = rep_hdr_sz + sizeof(ptr->bd_addr);
		break;
	case NV_WLAN_MAC_ADDRESS_I:
		*reqsz = req_hdr_sz + sizeof(ptr->wlan_mac_address);
		*repsz = rep_hdr_sz + sizeof(ptr->wlan_mac_address);
		break;
	default:
		pr_err("%s: no matches found for item (%d)\n",
			__func__, item);
		break;
	}
}

static void msm_nv_set_rep_data(int item, nv_cmd_generic_cmd *cmd,
	nv_cmd_item_type *data)
{
	nv_cmd_item_type *ptr = (nv_cmd_item_type *)cmd->rep.data;

	switch (item) {
	case NV_FACTORY_INFO_I:
		memcpy(data->factory_info, ptr->factory_info,
			sizeof(ptr->factory_info));
		break;
	case NV_BD_ADDR_I:
		data->integer = ptr->integer;
		break;
	case NV_WLAN_MAC_ADDRESS_I:
		data->integer = ptr->integer;
		break;
	default:
		pr_err("%s: no matches found for item (%d)\n",
			__func__, item);
		break;
	}
}

int msm_nv_cmd_remote(uint32_t cmd, uint32_t item, nv_cmd_item_type *data_ptr)
{
	int rc = 0;
	int reqsz = 0, repsz = 0;
	int more_data;
	nv_cmd_generic_cmd *cmdptr;

	if (msm_nv_check_validity(item) < 0) {
		pr_err("%s : cmd(%d) is NOT supported!\n",
			__func__, item);
		return	-EINVAL;
	}

	if (IS_ERR_OR_NULL(nv_ep)) {
		pr_err("%s: rpc connect failed: rc = %ld\n",
			__func__, PTR_ERR(nv_ep));
		return -EAGAIN;
	}

	cmdptr = kmalloc(sizeof(nv_cmd_generic_cmd), GFP_KERNEL);
	if (!cmdptr) {
		pr_err("%s: malloc failed\n", __func__);
		return -EBUSY;
	}

	cmdptr->req.common.cmd = cpu_to_be32(cmd);
	cmdptr->req.common.item = cpu_to_be32(item);
	more_data = (data_ptr != NULL);
	cmdptr->req.common.more_data = cpu_to_be32(more_data);
	cmdptr->req.common.disc = cpu_to_be32(item);

	msm_nv_set_req_data(item, cmdptr, data_ptr, &reqsz, &repsz);
	pr_debug("%s: reqhdr(%d), rephdr(%d)\n",
		__func__, sizeof(struct rpc_request_hdr),
		sizeof(struct rpc_reply_hdr));
	pr_debug("%s: reqsz(%d), repsz(%d)\n", __func__, reqsz, repsz);

	rc = msm_rpc_call_reply(nv_ep, nv_rpc_ids.cmd_remote,
		&cmdptr->req, reqsz, &cmdptr->rep, repsz, 5 * HZ);

	if (rc < 0) {
		kfree(cmdptr);
		pr_debug("%s: rpc call failed! error: %d\n", __func__, rc);
		return rc;
	} else {
		pr_debug("%s: rpc call success\n" , __func__);
	}

	rc = be32_to_cpu(cmdptr->rep.common.result);
	if (rc == NV_DONE_S) {
		msm_nv_set_rep_data(item, cmdptr, data_ptr);
	}

	kfree(cmdptr);

	return rc;
}
EXPORT_SYMBOL(msm_nv_cmd_remote);

int msm_nv_read(uint32_t item, nv_cmd_item_type *data_ptr)
{
	return msm_nv_cmd_remote(NV_READ_F, item, data_ptr);
}
EXPORT_SYMBOL(msm_nv_read);

int msm_nv_write(uint32_t item, nv_cmd_item_type *data_ptr)
{
	return msm_nv_cmd_remote(NV_WRITE_F, item, data_ptr);
}
EXPORT_SYMBOL(msm_nv_write);

int msm_nv_rpc_close(void)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(nv_ep)) {
		pr_err("%s: rpc_close failed before call, rc = %ld\n",
			__func__, PTR_ERR(nv_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_close(nv_ep);
	nv_ep = NULL;

	if (rc < 0) {
		pr_err("%s: close rpc failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	} else
		pr_debug("rpc close success\n");

	return rc;
}
EXPORT_SYMBOL(msm_nv_rpc_close);
