/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *
 * Author: Alexandre Torgue <alexandre.torgue@stericsson.com> for ST-Ericsson
 *
 */

#ifndef __BOARD_CCU9540_MODEM_H
#define __BOARD_CCU9540_MODEM_H

#include <linux/ux500_modem_control.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mfd/abx500/ab9540-modctrl.h>

/* modem control ressource */
#define GPIO_RESOUT0 84
#define GPIO_RESOUT2 85

extern struct platform_device ap9540_modem_control_device;

#endif
