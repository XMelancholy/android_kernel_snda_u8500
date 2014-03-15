/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Jens Wiklander <jens.wiklander@stericsson.com>
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com>
 * Author: Mian Yousaf Kaukab <mian.yousaf.kaukab@stericsson.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/tee.h>
#include <linux/module.h>
#include <linux/string.h>

#include <mach/hardware.h>

#include "id.h"

#define STATIC_TEE_TA_START_LOW	0xBC765EDE
#define STATIC_TEE_TA_START_MID	0x6724
#define STATIC_TEE_TA_START_HIGH	0x11DF
#define STATIC_TEE_TA_START_CLOCKSEQ  \
	{0x8E, 0x12, 0xEC, 0xDB, 0xDF, 0xD7, 0x20, 0x85}

#define PRCMU_DBG_PWRCTRL_A9DBGCLKEN    (1 << 4)

static struct tee_product_config product_config;

bool ux500_jtag_enabled(void)
{
#ifdef CONFIG_UX500_DEBUG_NO_LAUTERBACH
	return false;
#else
	if (cpu_is_u8500_family() || cpu_is_ux540_family())
		return (product_config.rt_flags & TEE_RT_FLAGS_JTAG_ENABLED) ==
			TEE_RT_FLAGS_JTAG_ENABLED;

	return true;
#endif
}

static int __init product_detect(void)
{
	int err;
	int origin_err;
	struct tee_operation operation = {0};
	struct tee_context context;
	struct tee_session session;

	/* Selects trustzone application needed for the job. */
	struct tee_uuid static_uuid = {
		STATIC_TEE_TA_START_LOW,
		STATIC_TEE_TA_START_MID,
		STATIC_TEE_TA_START_HIGH,
		STATIC_TEE_TA_START_CLOCKSEQ,
	};

	err = teec_initialize_context(NULL, &context);
	if (err) {
		pr_err("ux500-product: unable to initialize tee context,"
			" err = %d\n", err);
		err = -EINVAL;
		goto error0;
	}

	err = teec_open_session(&context, &session, &static_uuid,
				TEEC_LOGIN_PUBLIC, NULL, NULL, &origin_err);
	if (err) {
		pr_err("ux500-product: unable to open tee session,"
			" tee error = %d, origin error = %d\n",
			err, origin_err);
		err = -EINVAL;
		goto error1;
	}

	memset(&operation, 0, sizeof(struct tee_operation));
	if (cpu_is_u8500_family()) {
		operation.shm[0].buffer = &product_config;
		operation.shm[0].size = sizeof(product_config);
		operation.shm[0].flags = TEEC_MEM_OUTPUT;
		operation.flags = TEEC_MEMREF_0_USED;
	} else if (cpu_is_ux540_family()) {
		operation.param[0].tmpref.buffer = &product_config;
		operation.param[0].tmpref.size = sizeof(product_config);
		operation.types = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_OUTPUT,
						   TEEC_NONE, TEEC_NONE,
						   TEEC_NONE);
	} else {
		pr_err("ux500-product: incorrect memref\n");
		err = -EINVAL;
		goto error1;
	}

	err = teec_invoke_command(&session,
				  TEE_STA_GET_PRODUCT_CONFIG,
				  &operation, &origin_err);
	if (err) {
		pr_err("ux500-product: fetching product settings failed, err=%d",
		       err);
		err = -EINVAL;
		goto error1;
	}

	switch (product_config.product_id) {
	case TEE_PRODUCT_ID_8400:
		pr_info("ux500-product: u8400 detected\n");
		break;
	case TEE_PRODUCT_ID_8500B:
		pr_info("ux500-product: u8500B detected\n");
		break;
	case TEE_PRODUCT_ID_9500:
		pr_info("ux500-product: a9500 detected\n");
		break;
	case TEE_PRODUCT_ID_7400:
		pr_info("ux500-product: u7400 detected\n");
		break;
	case TEE_PRODUCT_ID_8500C:
		pr_info("ux500-product: u8500C detected\n");
		break;
	case TEE_PRODUCT_ID_8500A:
		pr_info("ux500-product: u8500A detected\n");
		break;
	case TEE_PRODUCT_ID_8500E:
		pr_info("ux500-product: u8500E detected\n");
		break;
	case TEE_PRODUCT_ID_8520F:
		pr_info("ux500-product: u8520F detected\n");
		break;
	case TEE_PRODUCT_ID_8520H:
		pr_info("ux500-product: u8520H detected\n");
		break;
	case TEE_PRODUCT_ID_9540:
		pr_info("ux500-product: u9540 detected\n");
		break;
	case TEE_PRODUCT_ID_9500C:
		pr_info("ux500-product: a9500C detected\n");
		break;
	case TEE_PRODUCT_ID_8500F:
		pr_info("ux500-product: u8500F detected\n");
		break;
	case TEE_PRODUCT_ID_8540APE:
		pr_info("ux500-product: u8540APE detected\n");
		break;
	case TEE_PRODUCT_ID_8540XMIP:
		pr_info("ux500-product: u8540XMIP detected\n");
		break;
	case TEE_PRODUCT_ID_8520E:
		pr_info("ux500-product: u8520E detected\n");
		break;
	case TEE_PRODUCT_ID_8520J:
		pr_info("ux500-product: u8520J detected\n");
		break;
	case TEE_PRODUCT_ID_UNKNOWN:
	default:
		pr_info("ux500-product: UNKNOWN! (0x%x) detected\n",
			product_config.product_id);
		break;
	}
	pr_info("ux500-product: JTAG is %s\n",
		ux500_jtag_enabled() ? "enabled" : "disabled");
error1:
	(void) teec_finalize_context(&context);
error0:
	return err;
}
device_initcall(product_detect);
