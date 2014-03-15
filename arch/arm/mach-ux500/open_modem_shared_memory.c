/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Marten Olsson <marten.xm.olsson@stericsson.com> for ST-Ericsson
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/kernel.h>
#include <linux/tee.h>
#include <linux/string.h>

#include "id.h"

#define STATIC_TEE_TA_START_LOW	 0xBC765EDE
#define STATIC_TEE_TA_START_MID	 0x6724
#define STATIC_TEE_TA_START_HIGH	0x11DF
#define STATIC_TEE_TA_START_CLOCKSEQ  \
	{0x8E, 0x12, 0xEC, 0xDB, 0xDF, 0xD7, 0x20, 0x85}

int open_modem_shared_memory(void)
{
	int err = TEED_ERROR_GENERIC;
	int origin_err;
	struct tee_operation operation;
	struct tee_context context;
	struct tee_session session;
	u32 status;
	/* Selects trustzone application needed for the job. */
	struct tee_uuid static_uuid = {
		STATIC_TEE_TA_START_LOW,
		STATIC_TEE_TA_START_MID,
		STATIC_TEE_TA_START_HIGH,
		STATIC_TEE_TA_START_CLOCKSEQ,
	};

	status = false;

	/*
	 * It seems like shrm driver is calling this function before tee driver
	 * is ready, therefore we need to delay this call.
	 */
	while (is_teec_ready() != true)
		cpu_relax();

	err = teec_initialize_context(NULL, &context);
	if (err) {
		pr_err("%s: unable to initialize tee context, err = %d\n",
		       __func__, err);
		goto error0;
	}

	err = teec_open_session(&context, &session, &static_uuid,
			TEEC_LOGIN_PUBLIC, NULL, NULL, &origin_err);
	if (err) {
		pr_err("%s: unable to open tee session, tee error = %d, origin error = %d\n",
		       __func__, err, origin_err);
		goto error1;
	}

	memset(&operation, 0, sizeof(struct tee_operation));
	if (cpu_is_u8500_family()) {
		operation.shm[0].buffer = &status;
		operation.shm[0].size = sizeof(u32 *);
		operation.shm[0].flags = TEEC_MEM_OUTPUT;
		operation.flags = TEEC_MEMREF_0_USED;
	} else if (cpu_is_ux540_family()) {
		operation.param[0].tmpref.buffer = &status;
		operation.param[0].tmpref.size = sizeof(u32 *);
		operation.types = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_OUTPUT,
						   TEEC_NONE, TEEC_NONE,
						   TEEC_NONE);
	} else {
		pr_err("%s: incorrect memref\n", __func__);
		err = -EINVAL;
		goto error1;
	}

	err = teec_invoke_command(&session, TEE_STA_OPEN_SHARED_MEMORY,
			&operation, &origin_err);
	if (err) {
		pr_err("%s: open shared memory failed, err=%d",
		       __func__, err);
		goto error2;
	}

error2:
	(void) teec_close_session(&session);
error1:
	(void) teec_finalize_context(&context);
error0:
	if (status == false)
		err = -1;
	return err;
}

