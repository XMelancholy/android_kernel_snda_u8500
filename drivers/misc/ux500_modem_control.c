/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License Terms: GNU General Public License v2
 * Author: Aurelien Gerault <aurelien.gerault-nonst@stericsson.com>.
 *
 * Adding 3 sysfs entries to enable / monitor ux500 modem boot.
 *
 * modem_power (R/W): start (1) or stop (0) modem
 * resout2     (RO) : is modem ready? Polling notification awared.
 * resout0     (RO) : has to trigger a modem reboot?
 *
 * sysfs entries are added to /sys/devices/platform/modem_control
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/ux500_modem_control.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/c2c_genio.h>
#include <linux/notifier.h>
#include <linux/delay.h>

struct modem_control {
	int resout2;
	int resout0;
	struct device *dev;
	int (*power_set)(struct modem_control *, enum modpwr_states);
	int (*power_get)(struct modem_control *);
	int (*show_resout2)(struct modem_control *);
	int (*show_resout0)(struct modem_control *);
	int (*show_service_n)(struct modem_control *);
	int (*store_service_n)(struct modem_control *, int);
	int (*show_reset_n)(struct modem_control *);
	int (*store_reset_n)(struct modem_control *, int);
	int (*init)(struct platform_device *, struct modem_control *);
	struct notifier_block resout0n_nb;
	struct notifier_block resout2n_nb;
};

static int ap9540_set_modem_power(struct modem_control *mctrl,
		enum modpwr_states state)
{
	return abx540_set_modem_power(state);
}

static int ap9540_get_modem_power(struct modem_control *mctrl)
{
	return abx540_get_modem_power();
}

static int ap9540_show_resout2(struct modem_control *mctrl)
{
	if (!mctrl->resout2)
		return -EINVAL;

	return !!gpio_get_value_cansleep(mctrl->resout2);
}

static int ap9540_show_resout0(struct modem_control *mctrl)
{
	if (!mctrl->resout0)
		return -EINVAL;

	return !!gpio_get_value_cansleep(mctrl->resout0);
}

static irqreturn_t ap9540_resout0_irq_handler(int irq, void *data)
{
	struct platform_device *pdev = data;

	sysfs_notify(&pdev->dev.kobj, NULL, "resout0");
	return IRQ_HANDLED;
}

static irqreturn_t ap9540_resout2_irq_handler(int irq, void *data)
{
	struct platform_device *pdev = data;

	sysfs_notify(&pdev->dev.kobj, NULL, "resout2");
	return IRQ_HANDLED;
}

static int ap9540_modem_control_init(struct platform_device *pdev,
		struct modem_control *mctrl)
{
	int ret;
	struct modem_control_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata->gpio_resout2)
		return -EINVAL;

	mctrl->resout2 = pdata->gpio_resout2;

	ret = devm_request_threaded_irq(&pdev->dev,
		gpio_to_irq(mctrl->resout2),
		NULL, ap9540_resout2_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
		IRQF_SHARED | IRQF_NO_SUSPEND,
		"resout2", pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq for resout2 gpio\n");
		return ret;
	}

	ret = devm_gpio_request(&pdev->dev, mctrl->resout2, "CAIF_READY");
	if (ret) {
		dev_err(&pdev->dev, "failed to request resout2 gpio\n");
		return ret;
	}

	ret = gpio_direction_input(mctrl->resout2);
	if (ret) {
		dev_err(&pdev->dev
			, "failed to set input direction on resout2 gpio\n");
		return ret;
	}

	if (!pdata->gpio_resout0) {
		ret = -EINVAL;
		return ret;
	}

	mctrl->resout0 = pdata->gpio_resout0;

	ret = devm_request_threaded_irq(&pdev->dev,
		gpio_to_irq(mctrl->resout0),
		NULL, ap9540_resout0_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_SHARED | IRQF_NO_SUSPEND,
		"resout0", pdev);
	if (ret) {
		dev_err(&pdev->dev
			, "failed to request irq for resout0 gpio\n");
		return ret;
	}

	ret = devm_gpio_request(&pdev->dev, mctrl->resout0, "RESOUT0");
	if (ret) {
		dev_err(&pdev->dev, "failed to request resout0 gpio\n");
		return ret;
	}

	ret = gpio_direction_input(mctrl->resout0);
	if (ret) {
		dev_err(&pdev->dev
			, "failed to set input direction on resout0 gpio\n");
		return ret;
	}

	return ret;
}

static int l8540_get_modem_power(struct modem_control *mctrl)
{
	int ab_val = abx540_get_modem_power();
	if (ab_val < 0)
		dev_err(mctrl->dev, "%s: abx540_get_modem_power failed (%d)",
				__func__, ab_val);
	return ab_val;
}

static int l8540_set_modem_power(struct modem_control *mctrl,
		enum modpwr_states state)
{
	int ret = 0;
	int i = 0;

	if (state) {
		ret = abx540_set_modem_power(1);
		if (ret < 0)
			return ret;

		mdelay(10);
		prcmu_set_xmip_reset_n(1);
		while (!prcmu_get_modem_resout0_n()) {
			mdelay(1);
			i++;
			if (i > 50)
				return -1;
		}
	} else {
		prcmu_set_xmip_reset_n(0);
		ret = abx540_set_modem_power(0);
		if (ret < 0)
			return ret;

		mdelay(10);
		while (!prcmu_get_modem_resout0_n()) {
			mdelay(1);
			i++;
			if (i > 50)
				return -1;
		}
	}
	return ret;
}

static int l8540_show_service_n(struct modem_control *mctrl)
{
	return prcmu_get_service_n();
}

static int l8540_store_service_n(struct modem_control *mctrl, int state)
{
	return prcmu_set_service_n(state);
}

static int l8540_show_reset_n(struct modem_control *mctrl)
{
	return prcmu_get_xmip_reset_n();
}

static int l8540_store_reset_n(struct modem_control *mctrl, int state)
{
	return prcmu_set_xmip_reset_n(state);
}

static int l8540_show_resout2_n(struct modem_control *mctrl)
{
	return prcmu_get_modem_resout2_n();
}

static int l8540_show_resout0_n(struct modem_control *mctrl)
{
	return prcmu_get_modem_resout0_n();
}

int l8540_notify_resout2_n(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct modem_control *mctrl =
		container_of(nb, struct modem_control, resout2n_nb);

	sysfs_notify(&mctrl->dev->kobj, NULL, "resout2_n");
	return 0;
}

int l8540_notify_resout0_n(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct modem_control *mctrl =
		container_of(nb, struct modem_control, resout0n_nb);


	sysfs_notify(&mctrl->dev->kobj, NULL, "resout0_n");
	return 0;
}

static ssize_t show_modem_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;

	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (mctrl->power_get != NULL) {
		ret = mctrl->power_get(mctrl);
		if (ret < 0)
			return ret;
	} else {
		dev_dbg(dev, "%s: failed access to power_get function\n",
				__func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_modem_power(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (count > 0) {
		if (mctrl->power_set != NULL) {
			int ret;

			/* reset C2C before asserting Modem Power/Reset */
			if (!strcmp(genio_drvname(), "c2c") && (buf[0] == '0'))
				prcmu_c2c_request_reset();
			ret = mctrl->power_set(mctrl, (buf[0] == '0') ? 0 : 1);
			if (ret < 0)
				return ret;
		} else {
			dev_err(dev, "%s: failed access to power_set function\n"
					, __func__);
			return -ENODEV;
		}
	}

	return count;
}

static ssize_t show_resout2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (mctrl->show_resout2 != NULL) {
		ret = mctrl->show_resout2(mctrl);
		if (ret < 0)
			return ret;
	} else {
		dev_dbg(dev, "%s: failed access to show_resout2 function\n"
				, __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_resout0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (mctrl->show_resout0 != NULL)
		ret = mctrl->show_resout0(mctrl);
		if (ret < 0)
			return ret;
	else
		dev_dbg(dev, "failed access to show_resout0 function\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_mod_service_n(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (mctrl->show_service_n != NULL) {
		ret = mctrl->show_service_n(mctrl);
		if (ret < 0)
			return ret;
	} else {
		dev_dbg(dev, "%s: failed access to show_mod_service_n function\n"
				, __func__);
		return -ENODEV;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_mod_service_n(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (count <= 0)
		return count;

	if (mctrl->store_service_n != NULL) {
		if (mctrl->store_service_n(mctrl, (buf[0] ==  '0') ? 0 : 1))
			return 0;
	} else {
		dev_dbg(dev, "%s: failed access to store_mod_service_n function\n"
				, __func__);
		return -ENODEV;
	}

	return count;
}

static ssize_t show_modem_reset_n(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (mctrl->show_reset_n != NULL) {
		ret = mctrl->show_reset_n(mctrl);
		if (ret < 0)
			return ret;
	} else {
		dev_dbg(dev, "%s: failed access to xmip_rst_n function\n"
				, __func__);
		return -ENODEV;

	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_modem_reset_n(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_control *mctrl = dev_get_drvdata(dev);

	if (count <= 0)
		return count;

	if (mctrl->store_reset_n != NULL) {
		int ret = mctrl->store_reset_n(mctrl, (buf[0] ==  '0') ? 0 : 1);
		if (ret < 0)
			return ret;
	} else {
		dev_dbg(dev, "%s: failed access to xmip_rst_n function\n"
				, __func__);
		return -ENODEV;
	}

	return count;
}

static DEVICE_ATTR(modem_power, S_IRUGO | S_IWUSR,
	show_modem_power, store_modem_power);
static DEVICE_ATTR(resout2, S_IRUGO, show_resout2, NULL);
static DEVICE_ATTR(resout0, S_IRUGO, show_resout0, NULL);
static DEVICE_ATTR(resout2_n, S_IRUGO, show_resout2, NULL);
static DEVICE_ATTR(resout0_n, S_IRUGO, show_resout0, NULL);
static DEVICE_ATTR(modem_service_n, S_IRUGO | S_IWUSR,
		show_mod_service_n, store_mod_service_n);
static DEVICE_ATTR(modem_reset_n, S_IRUGO | S_IWUSR,
		show_modem_reset_n, store_modem_reset_n);

static struct attribute *l9540_modem_control_entries[] = {
	&dev_attr_modem_power.attr,
	&dev_attr_resout2.attr,
	&dev_attr_resout0.attr,
	NULL
};

static struct attribute *l8540_modem_control_entries[] = {
	&dev_attr_modem_power.attr,
	&dev_attr_resout2_n.attr,
	&dev_attr_resout0_n.attr,
	&dev_attr_modem_service_n.attr,
	&dev_attr_modem_reset_n.attr,
	NULL
};

static struct attribute_group modem_control_attr_group[] = {
	[0] = {
			.attrs = l9540_modem_control_entries,
		},
	[1] = {
			.attrs = l8540_modem_control_entries,
		},
};

static int ux500_modem_control_probe(struct platform_device *pdev)
{
	int ret;
	struct modem_control *mctrl;
	struct modem_control_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "invalid modem control platform data\n");
		return -EINVAL;
	}

	mctrl = devm_kzalloc(&pdev->dev, sizeof(struct modem_control),
			GFP_KERNEL);
	if (!mctrl)
		return -ENOMEM;

	/* store callbacks functions, depending on detected hardware */
	switch (pdata->version) {
	case MODCTRL_VERSION_AP9540:
		mctrl->power_set = ap9540_set_modem_power;
		mctrl->power_get = ap9540_get_modem_power;
		mctrl->init  = ap9540_modem_control_init;
		mctrl->show_resout0 = ap9540_show_resout0;
		mctrl->show_resout2 = ap9540_show_resout2;
		break;
	case MODCTRL_VERSION_AP8540:
		mctrl->power_set = l8540_set_modem_power;
		mctrl->power_get = l8540_get_modem_power;
		mctrl->show_service_n = l8540_show_service_n;
		mctrl->store_service_n = l8540_store_service_n;
		mctrl->show_reset_n = l8540_show_reset_n;
		mctrl->store_reset_n = l8540_store_reset_n;
		mctrl->show_resout0 = l8540_show_resout0_n;
		mctrl->show_resout2 = l8540_show_resout2_n;
		mctrl->resout0n_nb.notifier_call = l8540_notify_resout0_n;
		mctrl->resout2n_nb.notifier_call = l8540_notify_resout2_n;

		upap_register_notifier(UPAP_NFYID_XMIP_RESOUT0N,
				&mctrl->resout0n_nb);
		upap_register_notifier(UPAP_NFYID_XMIP_RESOUT2N,
				&mctrl->resout2n_nb);
		break;
	default:
		dev_err(&pdev->dev, "Can't match version\n");
		return -ENODEV;
		break;
	}

	mctrl->dev = &pdev->dev;

	ret = sysfs_create_group(&pdev->dev.kobj,
			&modem_control_attr_group[pdata->version]);

	if (ret) {
		dev_err(&pdev->dev, "%s: error on creating sysfs\n", __func__);
		return ret;
	}

	platform_set_drvdata(pdev, mctrl);

	if (mctrl->init) {
		ret = mctrl->init(pdev, mctrl);
		if (ret)
			goto out_noinit;
	}
	return ret;

out_noinit:
	sysfs_remove_group(&pdev->dev.kobj,
		&modem_control_attr_group[pdata->version]);
	return ret;
}

static int __devexit ux500_modem_control_remove(struct platform_device *pdev)
{
	struct modem_control_platform_data *pdata = pdev->dev.platform_data;
	struct modem_control *mctrl = dev_get_drvdata(&pdev->dev);

	if (pdata && (pdata->version == MODCTRL_VERSION_AP8540)) {
		upap_unregister_notifier(UPAP_NFYID_XMIP_RESOUT0N,
				&mctrl->resout0n_nb);
		upap_unregister_notifier(UPAP_NFYID_XMIP_RESOUT2N,
				&mctrl->resout2n_nb);
	}

	sysfs_remove_group(&pdev->dev.kobj, modem_control_attr_group);
	return 0;
}

static struct platform_driver ux500_modem_control_driver = {
	.driver = {
		.name = "modem_control",
		.owner = THIS_MODULE,
	},
	.probe = ux500_modem_control_probe,
	.remove = __devexit_p(ux500_modem_control_remove),
};

static int __init ux500_modem_control_init(void)
{
	int ret;
	ret = platform_driver_register(&ux500_modem_control_driver);
	if (ret)
		printk(KERN_ERR "%s: modem control registation failed %d.\n",
			__func__, ret);

	return ret;
}

static void __exit ux500_modem_control_exit(void)
{
	platform_driver_unregister(&ux500_modem_control_driver);
}

module_init(ux500_modem_control_init);
module_exit(ux500_modem_control_exit);

MODULE_AUTHOR("Aurelien Gerault <aurelien.gerault-nonst@stericsson.com>");
MODULE_DESCRIPTION("Ux500 Modem Control Driver");
MODULE_ALIAS("Ux500 Modem Control Driver");
MODULE_LICENSE("GPL v2");
