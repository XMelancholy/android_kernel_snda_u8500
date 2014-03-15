/*
 * Copyright (C) 2008-2009 ST-Ericsson
 *
 * Author: Alexandre Torgue <alexandre.torgue@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include "board-ccu9540-modem.h"

/*
 * Modem Control Signals
 */
static struct modem_control_platform_data u9540_modem_control_data = {
	.gpio_resout2		= GPIO_RESOUT2,
	.gpio_resout0		= GPIO_RESOUT0,
	.version		= MODCTRL_VERSION_AP9540,
};

struct platform_device ap9540_modem_control_device = {
	.name	= "modem_control",
	.id	= -1,
	.dev	= {
		.platform_data = &u9540_modem_control_data,
	},
};
