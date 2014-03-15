/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/tee.h>

#define TEED_PFX "TEE: "

static bool teec_ready;
static struct tee_platform_data *tee_pdata;

bool is_teec_ready(void)
{
	return teec_ready;
}

int teec_initialize_context(const char *name, struct tee_context *context)
{
	if (tee_pdata)
		return tee_pdata->eops.initialize(name, context);

	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_initialize_context);

int teec_finalize_context(struct tee_context *context)
{
	if (tee_pdata)
		return tee_pdata->eops.finalize(context);

	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_finalize_context);

int teec_open_session(struct tee_context *context,
		      struct tee_session *session,
		      const struct tee_uuid *destination,
		      unsigned int connection_method,
		      void *connection_data, struct tee_operation *operation,
		      unsigned int *error_origin)
{
	if (tee_pdata)
		return tee_pdata->eops.open_session(context, session,
				destination, connection_method,
				connection_data, operation, error_origin);

	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_open_session);

int teec_close_session(struct tee_session *session)
{
	if (tee_pdata)
		return tee_pdata->eops.close_session(session);
	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_close_session);

int teec_invoke_command(
	struct tee_session *session, unsigned int command_id,
	struct tee_operation *operation,
	unsigned int *error_origin)
{
	if (tee_pdata)
		return tee_pdata->eops.invoke(session, command_id, operation,
					      error_origin);
	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_invoke_command);

int teec_allocate_shared_memory(struct tee_context *context,
				struct tee_sharedmemory *shared_memory)
{
	if (tee_pdata)
		return tee_pdata->eops.allocate_shared_memory(context,
							      shared_memory);

	return TEED_ERROR_GENERIC;
}
EXPORT_SYMBOL(teec_allocate_shared_memory);

void teec_release_shared_memory(struct tee_sharedmemory *shared_memory)
{
	if (tee_pdata)
		tee_pdata->eops.release_shared_memory(shared_memory);
}
EXPORT_SYMBOL(teec_release_shared_memory);

static int tee_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;

	dev_info(&pdev->dev, "[%s]\n", __func__);

	tee_pdata = (struct tee_platform_data *)pdev->dev.platform_data;

	/*
	 * Call the specific init function used by the version of Global
	 * Platform API defined by the platform in use.
	 */
	switch (tee_pdata->api_version) {
	case TEE_VERSION_017:
		ret = tee_init_017(pdev);
		break;

	case TEE_VERSION_100:
		ret = tee_init_100(pdev);
		break;

	default:
		dev_info(&pdev->dev, "[%s] No default init function called\n",
			 __func__);
	}

	if (ret)
		goto exit;

	memset(&tee_pdata->misc_dev, 0, sizeof(struct miscdevice));
	tee_pdata->misc_dev.parent = &pdev->dev;
	tee_pdata->misc_dev.minor = MISC_DYNAMIC_MINOR;
	tee_pdata->misc_dev.name = pdev->name;
	tee_pdata->misc_dev.fops = &tee_pdata->fops;
	ret = misc_register(&tee_pdata->misc_dev);

	if (ret)
		dev_err(&pdev->dev, "[%s] error register device: %d\n",
			__func__, ret);
	else
		teec_ready = true;
exit:
	return ret;
}

static int tee_remove(struct platform_device *pdev)
{
	int ret = -EINVAL;

	if (tee_pdata) {
		/* Call the architecture specific exit function. */
		tee_pdata->tops.exit();
		ret = misc_deregister(&tee_pdata->misc_dev);
	}

	if (ret)
		dev_err(&pdev->dev, "[%s] error deregister device: %d\n",
			__func__, ret);

	return ret;
}

static struct platform_driver tee_driver = {
	.probe  = tee_probe,
	.remove = tee_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = "tee"
	}
};

static int __init tee_mod_init(void)
{
	teec_ready = false;
	pr_debug(TEED_PFX ": %s\n", __func__);
	return platform_driver_register(&tee_driver);
}

static void __exit tee_mod_exit(void)
{
	pr_debug(TEED_PFX ": %s\n", __func__);
	platform_driver_unregister(&tee_driver);
}

subsys_initcall(tee_mod_init);
module_exit(tee_mod_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Trusted Execution Enviroment driver");
