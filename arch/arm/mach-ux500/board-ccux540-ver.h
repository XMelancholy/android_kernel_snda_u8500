/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __BOARD_CCU9540_VER_H
#define __BOARD_CCU9540_VER_H

enum ccx540_ver_family {
	CCUX540_VER_U9540_FAMILY,
	CCUX540_VER_L8540_FAMILY,
};


enum ccux540_ver {
	CCUX540_NOT_SET,
	CCU9540_HSI_US_V1,
	CCU9540_HSI_US_V1_05,
	CCU9540_HSI_US_V1_1,
	CCU9540_C2C_US_V1,
	CCU9540_C2C_US_V1_1, /*Board ID for C2C V1.1 HW */
	CCU9540_HSI_US_V2,
	CCU9540_HSI_EU_V2,
	CCU9540_C2C_US_V2,
	CCU9540_C2C_EU_V2,
	CCU8550_US_V1,
	CCU8540_V1,
};

struct ccux540_version_pdata {
	enum ccx540_ver_family family;
	enum ccux540_ver version;
	int rev_pin0;
	int rev_pin1;
	int rev_pin2;
	int rev_pin3;
};

#endif
