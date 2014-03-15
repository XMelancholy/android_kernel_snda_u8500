/*
 * Copyright (C) ST-Ericsson SA 2012
 * Authors: Hanumath Prasad <hanumath.prasad@stericsson.com> for ST-Ericsson.
 *	    Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/amba/mmci.h>

#include "board-common-sdi.h"

/* GPIO pins used by the sdi0 level shifter */
static int gpio_en = -1;
static int gpio_vsel = -1;

static int ux500_common_sdi0_ios_handler(struct device *dev,
		struct mmc_ios *ios, enum rpm_status pm)
{
	static unsigned char power_mode = MMC_POWER_ON;
	static unsigned char signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	if (signal_voltage == ios->signal_voltage)
		goto do_power;

	/*
	 * We need to re-init the levelshifter when switching I/O voltage level.
	 * Max discharge time according to ST6G3244ME spec is 1 ms.
	 */
	if (power_mode == MMC_POWER_ON) {
		power_mode = MMC_POWER_OFF;
		gpio_direction_output(gpio_en, 0);
		usleep_range(1000, 2000);
	}

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		gpio_direction_output(gpio_vsel, 0);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		/* Time between VCLK pass from 3.3V to 1.8V is minimum 5ms */
		gpio_direction_output(gpio_vsel, 1);
		usleep_range(5000, 6000);
		break;
	default:
		pr_warning("Non supported signal voltage for levelshifter.\n");
		break;
	}

	signal_voltage = ios->signal_voltage;

do_power:
	if (power_mode == ios->power_mode)
		goto do_pm;

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		break;
	case MMC_POWER_ON:
		gpio_direction_output(gpio_en, 1);
		/* Max settling time according to ST6G3244ME spec is 100 us. */
		udelay(100);
		break;
	case MMC_POWER_OFF:
		gpio_direction_output(gpio_en, 0);
		break;
	}

	power_mode = ios->power_mode;

do_pm:
	if ((pm == RPM_SUSPENDING) && (power_mode == MMC_POWER_ON)) {
		/* Disable levelshifter to save power */
		gpio_direction_output(gpio_en, 0);
	} else if ((pm == RPM_RESUMING) && (power_mode == MMC_POWER_ON)) {
		/* Re-enable levelshifter. */
		gpio_direction_output(gpio_en, 1);
		/* Max settling time according to ST6G3244ME spec is 100 us. */
		udelay(100);
	}

	return 0;
}

int ux500_common_sdi0_ios_handler_init(struct mmci_platform_data *sdi0,
		struct sdi_ios_pins *pins)
{
	int ret;

	gpio_en = pins->enable;
	gpio_vsel = pins->vsel;

	ret = gpio_request(gpio_en, "level shifter enable");
	if (ret < 0) {
		pr_err("%s: Fail to request gpio%d (enable) (%d).\n",
				__func__, gpio_en, ret);
		return ret;
	}

	ret = gpio_request(gpio_vsel, "level shifter 1v8-3v select");
	if (ret < 0) {
		pr_err("%s: Fail to request gpio%d (vsel) (%d).\n",
				__func__, gpio_vsel, ret);
		return ret;
	}

	/* Select the default 2.9V and enable level shifter */
	ret = gpio_direction_output(gpio_vsel, 0);
	if (ret < 0) {
		pr_err("%s: Fail to set direction for gpio%d (enable) (%d).\n",
				__func__, gpio_en, ret);
		return ret;
	}

	gpio_direction_output(gpio_en, 1);
	if (ret < 0) {
		pr_err("%s: Fail to set direction for gpio%d (vsel) (%d).\n",
				__func__, gpio_vsel, ret);
		return ret;
	}

	sdi0->ios_handler = ux500_common_sdi0_ios_handler;

	return 0;
}
