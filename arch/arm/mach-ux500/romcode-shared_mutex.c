/*
 * Copyright (C) ST-Ericsson SA 2012
 * License terms:  GNU General Public License (GPL), version 2
 * Author:  David Paris <david.paris@stericsson.com> for ST-Ericsson.
 *
 * ROM code Mutex Configuration for A9 exclusif load and store operation
 * for l2 cache maintenance.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/tee.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

#define BASS_ROMCODE_SHARED_MUTEX           0

#define UUID_TEE_TA_START_LOW	0xBC765EDE
#define UUID_TEE_TA_START_MID	0x6724
#define UUID_TEE_TA_START_HIGH	0x11DF
#define UUID_TEE_TA_START_CLOCKSEQ  \
	{0x8E, 0x12, 0xEC, 0xDB, 0xDF, 0xD7, 0x20, 0x85}

static uint32_t ROMCODE_DDR_addr;
static void __iomem *ROMCODE_DDR_addr_ptr;

static int __devinit shared_mutex_config_probe(struct platform_device *pdev)
{
	int err;
	int origin_err;
	struct tee_operation op;
	uint32_t mutex_type = BASS_ROMCODE_SHARED_MUTEX;
	struct tee_session session;
	struct tee_context context;

	/* Selects trustzone application needed for the job. */
	struct tee_uuid static_uuid = {
		UUID_TEE_TA_START_LOW,
		UUID_TEE_TA_START_MID,
		UUID_TEE_TA_START_HIGH,
		UUID_TEE_TA_START_CLOCKSEQ,
	};

	err = teec_initialize_context(NULL, &context);
	if (err) {
		pr_err("%s: unable to initialize tee context err=%d\n",
				__func__, err);
		err = -EINVAL;
		goto error0;
	}

	err = teec_open_session(&context, &session, &static_uuid,
			TEEC_LOGIN_PUBLIC, NULL, NULL, &origin_err);
	if (err) {
		pr_err("%s: unable to open tee session, err=%d, ori err=%d\n",
				__func__, err, origin_err);
		err = -EINVAL;
		goto error1;
	}

	memset(&op, 0, sizeof(op));

	ROMCODE_DDR_addr = 0;
	ROMCODE_DDR_addr_ptr = (void __iomem *)__virt_to_phys(
			(uint32_t)&ROMCODE_DDR_addr);

	/* ret_val */
	op.param[0].tmpref.buffer = (void *)&ROMCODE_DDR_addr_ptr;
	op.param[0].tmpref.size = sizeof(uint32_t *);
	op.param[1].tmpref.buffer = (void *)&mutex_type;
	op.param[1].tmpref.size = sizeof(uint32_t);
	op.types = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
			TEEC_MEMREF_TEMP_INPUT,
			TEEC_NONE, TEEC_NONE);

	err = teec_invoke_command(&session, TEE_STA_CONF_SHARED_MUTEX, &op,
			&origin_err);

	if (err) {
		pr_err("romcode-shared_mutex: init failed, err=%d",
				err);
		err = -EINVAL;
		goto error1;
	}
	(void) teec_close_session(&session);
	(void) teec_finalize_context(&context);

	pr_info("romcode-shared_mutex: initialized.\n");

	return 0;

error1:
	(void)teec_finalize_context(&context);
error0:
	return err;
}

static struct platform_driver romcode_sm_driver = {
	.driver = {
		.name = "romcode-sm",
		.owner = THIS_MODULE,
	},
	.probe = shared_mutex_config_probe,
};

static int __init shared_mutex_config_init(void)
{
	return platform_driver_register(&romcode_sm_driver);
}

/* Wait for TEE driver to be initialized. */
late_initcall(shared_mutex_config_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ROMCODE shared mutex config");
