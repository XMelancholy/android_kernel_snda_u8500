/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __BOARD_CCU9540_H
#define __BOARD_CCU9540_H
#include "board-common-uib.h"
#include "board-ccu9540-pins.h"

/* U9540 generic GPIOs */
#define U9540_HDMI_RST_GPIO		196
#define UIB_9540_DISP1_RST_GPIO		143

#define PRCM_DEBUG_NOPWRDOWN_VAL	0x194
#define ARM_DEBUG_NOPOWER_DOWN_REQ	1

/* CCU GPIO for MMC Card */
#define CCU9540_SDMMC_EN_GPIO		21
#define CCU9540_SDMMC_1V8_3V_GPIO	20
#define CCU9540_SDMMC_CD_GPIO		230

/* GPIO for PM2301 */
#define GPIO_171 171
#define GPIO_38 38

/* AB GPIO 60 located at offset 54 */
#define GPIO_60 54
/* AB GPIO 50 use in C2C instead of 60 for HSI */
#define GPIO_50 50

struct i2c_board_info;

void __init ccu9540_sdi_init(struct device *parent);
void __init mop500_msp_init(struct device *parent);
void __init mop500_vibra_init(void);

void accessory_detect_config(int ccu_id);

#endif
