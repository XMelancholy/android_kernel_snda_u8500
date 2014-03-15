/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com> for ST Ericsson.
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/abx500/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ab9540-modctrl.h>
#include <linux/regulator/ab8500.h>
#include <linux/gpio.h>

struct abx540_modctrl {
	struct device *dev;
	struct ab8500 *parent;
	int pwren_gpio;
	int pwren_reg_bank;
	int pwren_reg_addr;
	int pwren_reg_mask;
};

struct abx540_modctrl *modctrl;

int abx540_get_modem_power(void)
{
	int ret;
	u8 value;

	if (!modctrl)
		return -ENODEV;

	if (modctrl->pwren_gpio != -ENOSYS) {
		return !!gpio_get_value_cansleep(modctrl->pwren_gpio);
	} else if (modctrl->pwren_reg_bank != -ENOSYS) {
		ret = abx500_get_register_interruptible(modctrl->dev,
				modctrl->pwren_reg_bank,
				modctrl->pwren_reg_addr, &value);
		if (ret < 0) {
			dev_err(modctrl->dev,
				"Failed to get Modem power: err %d\n", ret);
			return ret;
		}
		return (value & (modctrl->pwren_reg_mask)) ? 1 : 0;
	} else {
		return -ENOSYS;
	}
}

int abx540_set_modem_power(enum modpwr_states state)
{
	int ret;
	u8 value;

	if (!modctrl)
		return -ENODEV;

	if (modctrl->pwren_gpio != -ENOSYS) {
		gpio_set_value(modctrl->pwren_gpio, state);
		return 0;
	} else if (modctrl->pwren_reg_bank) {

		if (state == MODPWR_RSTN_SET)
			value = modctrl->pwren_reg_mask;
		else if (state == MODPWR_RSTN_CLR)
			value = 0;
		else
			return -EINVAL;
		ret = abx500_mask_and_set_register_interruptible(modctrl->dev,
				modctrl->pwren_reg_bank,
				modctrl->pwren_reg_addr,
				modctrl->pwren_reg_mask, value);
		if (ret)
			dev_err(modctrl->dev,
				"Failed to set modem power to %d, err %d\n",
				state, ret);
		return ret;
	} else {
		return -ENOSYS;
	}
}

static int __devinit abx540_modctrl_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ab8500_platform_data *plat;
	struct abx540_modctrl_platform_data *pdata;

	modctrl = devm_kzalloc(&pdev->dev, sizeof(struct abx540_modctrl),
			GFP_KERNEL);
	if (!modctrl) {
		dev_err(&pdev->dev, "Probe error: No memory\n");
		return -ENOMEM;
	}

	modctrl->dev = &pdev->dev;
	modctrl->parent = dev_get_drvdata(pdev->dev.parent);

	plat = dev_get_platdata(pdev->dev.parent);
	pdata = plat->modctrl;

	modctrl->pwren_gpio = -ENOSYS;
	modctrl->pwren_reg_bank = -ENOSYS;

	if (pdata != NULL) {
		if (gpio_is_valid(pdata->gpio_power)) {

			ret = devm_gpio_request(&pdev->dev, pdata->gpio_power,
					"mod_pwr_en");

			if (ret == 0) {
				ret = gpio_direction_output(pdata->gpio_power,
						1);
			} else {
				dev_err(modctrl->dev, "gpio request failed\n");
				return ret;
			}
			if (ret == 0) {
				modctrl->pwren_gpio = pdata->gpio_power;
			} else {
				dev_err(modctrl->dev,
						"gpio direction failed\n");
				return ret;
			}
		} else if (pdata->reg_power->reg_pwr_bank != 0) {
			modctrl->pwren_reg_bank = pdata->reg_power->reg_pwr_bank;
			modctrl->pwren_reg_addr = pdata->reg_power->reg_pwr_addr;
			modctrl->pwren_reg_mask = pdata->reg_power->reg_pwr_mask;
		} else {
			dev_err(modctrl->dev,
				"gpio_power and reg_pwr_bank are not valid\n");
			ret = -ENODEV;
		}
	} else {
		dev_err(modctrl->dev, "no platform data defined\n");
		ret = -ENODEV;
	}

	return ret;
}

static struct platform_driver abx540_modctrl_driver = {
	.driver = {
		.name = "abx540-modctrl",
		.owner = THIS_MODULE,
	},
	.probe = abx540_modctrl_probe,
};

static int __init abx540_modctrl_init(void)
{
	return platform_driver_register(&abx540_modctrl_driver);
}
device_initcall(abx540_modctrl_init);

MODULE_AUTHOR("Maxime Coquelin <maxime.coquelin@stericsson.com>");
MODULE_DESCRIPTION("AB8500 modem control driver");
MODULE_LICENSE("GPL v2");
