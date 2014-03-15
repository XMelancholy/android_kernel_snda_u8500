/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Marten Olsson <marten.xm.olsson@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef TEE_RPC_H
#define TEE_RPC_H

#include "tee_mem.h"

#define TEE_RPC_ICMD_ALLOCATE 0x1001
#define TEE_RPC_ICMD_FREE     0x1002
#define TEE_RPC_ICMD_INVOKE   0x1003

#define TEE_RPC_LOAD_TA 0x10000001
#define TEE_RPC_WAIT    0x10000002
#define TEE_RPC_CANCEL  0x10000003

#define TEE_RPC_NBR_BUFF 1
#define TEE_RPC_DATA_SIZE 64

/**
 * struct tee_rpc_bf - Contains definition of the tee com buffer
 * @state: Buffer state
 * @data: Command data
 */
struct tee_rpc_bf {
	uint32_t state;
	uint8_t data[TEE_RPC_DATA_SIZE];
};

struct tee_rpc_alloc {
	size_t size; /* size of block */
	void *data; /* pointer to data */
};

struct tee_rpc_mem {
	struct tee_mem *tee_mem; /* memory handle */
	int32_t gid;
	uint32_t size;
};

struct tee_rpc_invoke {
	uint32_t res;
	uint32_t nbr_bf;
};

struct tee_rpc_cmd {
	void *buffer;
	uint32_t size;
	uint32_t type;
};

enum teec_rpc_result {
	TEEC_RPC_FAIL = 0,
	TEEC_RPC_OK = 1
};

struct tee_supp_comm {
	uint32_t cmd;
	uint32_t payloadlength;
	uint8_t payload[0];
};

#endif
