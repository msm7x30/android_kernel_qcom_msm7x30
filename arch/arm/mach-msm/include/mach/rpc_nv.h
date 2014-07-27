/* linux/include/mach/rpc_nv.h
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

#ifndef __ASM_ARCH_MSM_RPC_NV_H
#define __ASM_ARCH_MSM_RPC_NV_H

#include <mach/msm_rpcrouter.h>

typedef enum {
	NV_READ_F,			/* Read item */
	NV_WRITE_F,			/* Write item */
	NV_PEEK_F,			/* Peek at a location */
	NV_POKE_F,			/* Poke into a location */
	NV_FREE_F,			/* Free an nv item's memory allocation */
	NV_CHKPNT_DIS_F,		/* Disable cache checkpointing for glitch recovery */
	NV_CHKPNT_ENA_F,		/* Enable cache checkpointing for glitch recovery */
	NV_OTASP_COMMIT_F,		/* Commit (write) OTASP parameters to nv */
	NV_REPLACE_F,			/* Replace (overwrite) a dynamic pool item */
	NV_INCREMENT_F,			/* Increment the rental timer item */
	NV_RTRE_OP_CONFIG_F,		/* Set the operational configuration of RTRE */
	NV_FUNC_ENUM_PAD = 0x7FFF	/* Pad to 16 bits on ARM */
} nv_func_enum_type;

typedef enum {
	NV_DONE_S,
	NV_BUSY_S,
	NV_BADCMD_S,
	NV_FULL_S,
	NV_FAIL_S,
	NV_NOTACTIVE_S,
	NV_BADPARM_S,
	NV_READONLY_S,
	NV_BADTG_S,
	NV_NOMEM_S,
	NV_NOTALLOC_S,
	NV_STAT_ENUM_PAD = 32767
} nv_stat_enum_type;

typedef enum {
	NV_FACTORY_INFO_I		= 114,
	NV_BD_ADDR_I			= 447,
	NV_WLAN_MAC_ADDRESS_I		= 4678,
} nv_items_enum_type;

typedef struct {
	struct rpc_request_hdr hdr;
	uint32_t cmd;
	uint32_t item;
	uint32_t more_data;
	uint32_t disc;
} nv_cmd_req_header;

typedef struct {
	struct rpc_reply_hdr hdr;
	uint32_t result;
	uint32_t more_data;
	uint32_t disc;
} nv_cmd_rep_header;

typedef struct {
	nv_cmd_req_header common;
	uint8_t data[1024];
} nv_cmd_generic_req;

typedef struct {
	nv_cmd_rep_header common;
	uint8_t data[1024];
} nv_cmd_generic_rep;

typedef struct {
	nv_cmd_generic_req req;
	nv_cmd_generic_rep rep;
} nv_cmd_generic_cmd;

typedef union {
	uint8_t bd_addr[8];
	uint8_t wlan_mac_address[8];
	uint64_t integer;
	char factory_info[100];
} nv_cmd_item_type;

#ifdef CONFIG_MSM_ONCRPCROUTER
int msm_nv_rpc_connect(void);
int msm_nv_read(uint32_t item, nv_cmd_item_type *data);
int msm_nv_write(uint32_t item, nv_cmd_item_type *data);
int msm_nv_cmd_remote(uint32_t cmd, uint32_t item, nv_cmd_item_type *data_ptr);
int msm_nv_rpc_close(void);
#else
static int msm_nv_rpc_connect(void)
{
	return -EPERM;
}
static int msm_nv_read(uint32_t item, nv_cmd_item_type *data)
{
	return -EPERM;
}
static int msm_nv_write(uint32_t item, nv_cmd_item_type *data)
{
	return -EPERM;
}
static int msm_nv_cmd_remote(uint32_t cmd, uint32_t item,
	nv_cmd_item_type *data_ptr)
{
	return -EPERM;
}
static int msm_nv_rpc_close(void)
{
	return -EPERM;
}
#endif

#endif
