/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 *
 * Author: Mathieu Poirier <mathieu.poirier@linaro.org> for ST-Ericsson
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ux500_wdt.h>

#include <linux/mfd/dbx500-prcmu.h>

#define WATCHDOG_TIMEOUT 600 /* 10 minutes */
#define WATCHDOG_MIN	0
#define WATCHDOG_MAX28	268435  /* 28 bit resolution in ms == 268435.455 s */
#define WATCHDOG_MAX32  4294967 /* 32 bit resolution in ms == 4294967.295 s */

static int timeout = WATCHDOG_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout,
	"Watchdog timeout in seconds. default="
				__MODULE_STRING(WATCHDOG_TIMEOUT) ".");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
static u8 wdog_id;
static bool wdt_en;
static bool wdt_auto_off;

static int ux500_wdt_start(struct watchdog_device *wdd)
{
	wdt_en = true;
	return prcmu_enable_a9wdog(0);
}

static int ux500_wdt_stop(struct watchdog_device *wdd)
{
	wdt_en = false;
	return	prcmu_disable_a9wdog(0);
}

static int ux500_wdt_keepalive(struct watchdog_device *wdd)
{
	return prcmu_kick_a9wdog(0);
}

static int ux500_wdt_set_timeout(struct watchdog_device *wdd,
						unsigned int timeout)
{
	ux500_wdt_stop(wdd);
	prcmu_load_a9wdog(0, timeout * 1000);
	ux500_wdt_start(wdd);

	return 0;
}

static const struct watchdog_info ux500_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Ux500 WDT",
	.firmware_version = 1,
};

static struct watchdog_ops ux500_wdt_ops = {
	.owner = THIS_MODULE,
	.start = ux500_wdt_start,
	.stop  = ux500_wdt_stop,
	.ping  = ux500_wdt_keepalive,
	.set_timeout = ux500_wdt_set_timeout,
};

static struct watchdog_device ux500_wdt = {
	.info = &ux500_wdt_info,
	.ops = &ux500_wdt_ops,
	.min_timeout = WATCHDOG_MIN,
	.max_timeout = WATCHDOG_MAX32,
};

#ifdef CONFIG_UX500_WATCHDOG_DEBUG
enum wdog_dbg {
	WDOG_DBG_CONFIG,
	WDOG_DBG_LOAD,
	WDOG_DBG_KICK,
	WDOG_DBG_EN,
	WDOG_DBG_DIS,
};

static ssize_t wdog_dbg_write(struct file *file,
			      const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	unsigned long val;
	int err;
	enum wdog_dbg v = (enum wdog_dbg)((struct seq_file *)
					  (file->private_data))->private;

	switch(v) {
	case WDOG_DBG_CONFIG:
		err = kstrtoul_from_user(user_buf, count, 0, &val);

		if (!err) {
			wdt_auto_off = val != 0;
			prcmu_config_a9wdog(1, wdt_auto_off);
		}
		else {
			pr_err("ux500_wdt:dbg: unknown value\n");
		}
		break;
	case WDOG_DBG_LOAD:
		err = kstrtoul_from_user(user_buf, count, 0, &val);

		if (!err) {
			timeout = val;
			/* Convert seconds to ms */
			prcmu_disable_a9wdog(wdog_id);
			prcmu_load_a9wdog(wdog_id, timeout * 1000);
			prcmu_enable_a9wdog(wdog_id);
		}
		else {
			pr_err("ux500_wdt:dbg: unknown value\n");
		}
		break;
	case WDOG_DBG_KICK:
		prcmu_kick_a9wdog(wdog_id);
		break;
	case WDOG_DBG_EN:
		wdt_en = true;
		prcmu_enable_a9wdog(wdog_id);
		break;
	case WDOG_DBG_DIS:
		wdt_en = false;
		prcmu_disable_a9wdog(wdog_id);
		break;
	}

	return count;
}

static int wdog_dbg_read(struct seq_file *s, void *p)
{
	enum wdog_dbg v = (enum wdog_dbg)s->private;

	switch(v) {
	case WDOG_DBG_CONFIG:
		seq_printf(s,"wdog is on id %d, auto off on sleep: %s\n",
			   (int)wdog_id,
			   wdt_auto_off ? "enabled": "disabled");
		break;
	case WDOG_DBG_LOAD:
		/* In 1s */
		seq_printf(s, "wdog load is: %d s\n",
			   timeout);
		break;
	case WDOG_DBG_KICK:
		break;
	case WDOG_DBG_EN:
	case WDOG_DBG_DIS:
		seq_printf(s, "wdog is %sabled\n",
			       wdt_en ? "en" : "dis");
		break;
	}
	return 0;
}

static int wdog_dbg_open(struct inode *inode,
			struct file *file)
{
	return single_open(file, wdog_dbg_read, inode->i_private);
}

static const struct file_operations wdog_dbg_fops = {
	.open		= wdog_dbg_open,
	.write		= wdog_dbg_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int __init wdog_dbg_init(void)
{
	struct dentry *wdog_dir;

	wdog_dir = debugfs_create_dir("wdog", NULL);
	if (IS_ERR_OR_NULL(wdog_dir))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_u8("id",
					     S_IWUGO | S_IRUGO, wdog_dir,
					     &wdog_id)))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_file("config",
					       S_IWUGO | S_IRUGO, wdog_dir,
					       (void *)WDOG_DBG_CONFIG,
					       &wdog_dbg_fops)))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_file("load",
					       S_IWUGO | S_IRUGO, wdog_dir,
					       (void *)WDOG_DBG_LOAD,
					       &wdog_dbg_fops)))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_file("kick",
					       S_IWUGO, wdog_dir,
					       (void *)WDOG_DBG_KICK,
					       &wdog_dbg_fops)))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_file("enable",
					       S_IWUGO | S_IRUGO, wdog_dir,
					       (void *)WDOG_DBG_EN,
					       &wdog_dbg_fops)))
		goto fail;

	if (IS_ERR_OR_NULL(debugfs_create_file("disable",
					       S_IWUGO | S_IRUGO, wdog_dir,
					       (void *)WDOG_DBG_DIS,
					       &wdog_dbg_fops)))
		goto fail;

	return 0;
fail:
	pr_err("ux500:wdog: Failed to initialize wdog dbg\n");
	debugfs_remove_recursive(wdog_dir);

	return -EFAULT;
}

#else
static inline int __init wdog_dbg_init(void)
{
	return 0;
}
#endif

static int __init ux500_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct ux500_wdt_data *pdata = pdev->dev.platform_data;
	bool has_28bit_resolution = false;

	if (pdata) {
		if (pdata->timeout > 0)
			timeout = pdata->timeout;
		if(pdata->nowayout >= 0)
			nowayout = pdata->nowayout;
		has_28bit_resolution = pdata->has_28_bits_resolution;
	}

	watchdog_set_nowayout(&ux500_wdt, nowayout);

	/* update max timeout according to platform capabilities */
	if (has_28bit_resolution)
		ux500_wdt.max_timeout = WATCHDOG_MAX28;

	/* auto off on sleep */
	prcmu_config_a9wdog(1, wdt_auto_off);

	/* set HW initial value */
	prcmu_load_a9wdog(0, timeout * 1000);

	ret = watchdog_register_device(&ux500_wdt);
	if (ret)
		return ret;

	ret = wdog_dbg_init();
	if (ret < 0)
		goto fail;

	dev_info(&pdev->dev, "initialized\n");

	return 0;
fail:
	watchdog_unregister_device(&ux500_wdt);
	return ret;
}

static int __exit ux500_wdt_remove(struct platform_device *dev)
{
	watchdog_unregister_device(&ux500_wdt);

	return 0;
}

#ifdef CONFIG_PM
static int ux500_wdt_suspend(struct platform_device *pdev,
			     pm_message_t state)
{
	if (wdt_en && !wdt_auto_off) {
		ux500_wdt_stop(&ux500_wdt);
		prcmu_config_a9wdog(1, true);

		prcmu_load_a9wdog(0, timeout * 1000);
		ux500_wdt_start(&ux500_wdt);
	}
	return 0;
}

static int ux500_wdt_resume(struct platform_device *pdev)
{
	if (wdt_en && !wdt_auto_off) {
		ux500_wdt_stop(&ux500_wdt);
		prcmu_config_a9wdog(1, wdt_auto_off);

		prcmu_load_a9wdog(0, timeout * 1000);
		ux500_wdt_start(&ux500_wdt);
	}
	return 0;
}

#else
#define ux500_wdt_suspend NULL
#define ux500_wdt_resume NULL
#endif
static struct platform_driver ux500_wdt_driver = {
	.remove		= __exit_p(ux500_wdt_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ux500_wdt",
	},
	.suspend	= ux500_wdt_suspend,
	.resume		= ux500_wdt_resume,
};

static int __init ux500_wdt_init(void)
{
	return platform_driver_probe(&ux500_wdt_driver, ux500_wdt_probe);
}
module_init(ux500_wdt_init);

MODULE_AUTHOR("Jonas Aaberg <jonas.aberg@stericsson.com>");
MODULE_DESCRIPTION("Ux500 Watchdog Driver");
MODULE_LICENSE("GPL");
