/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Alexandre Torgue <alexandre.torgue@stericsson.com> for ST Ericsson.
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef UX500_MODEM_CONTROL_H
#define UX500_MODEM_CONTROL_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mfd/abx500/ab9540-modctrl.h>
#include <linux/mfd/dbx500-prcmu.h>

enum modctrl_version {
	MODCTRL_VERSION_AP9540 = 0x0,
	MODCTRL_VERSION_AP8540,
	MODCTRL_VERSION_UNDEFINED,
};

struct modem_control_platform_data {
	int gpio_resout2;
	int gpio_resout0;
	enum modctrl_version version;
};

#endif /* UX500_MODEM_CONTROL_H */
