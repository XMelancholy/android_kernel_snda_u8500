/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Maxime Coquelin <maxime.coquelin@stericsson> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __BOARD_MOP500_UIB_H
#define __BOARD_MOP500_UIB_H

enum mop500_uib {
	STUIB,
	U8500UIB,
	U8500UIB_R3,
	U9540UIBS_V1,
	U9540UIBS_V2,
	U9540UIBS_V3,
	U9540UIBT_V1,
	U8540UIBS_V2,
	U8540UIBS_V3,
	NO_UIB,
	NO_UIB_SELECTED,
};

#ifdef CONFIG_UIB
extern enum mop500_uib type_of_uib;
#else
#define type_of_uib NO_UIB
#endif

/* CCU GPIO for Camera */
#define CCU9540_PRIMARY_SENSOR_POWER_EN   MOP500_EGPIO(3)
#define CCU9540_SECONDARY_SENSOR_POWER_EN MOP500_EGPIO(4)
#define CCU9540_CAMERA_FLASH_READY        6
#define CCU9540_CAMERA_FLASH_EN1          MOP500_EGPIO(2)

#define CCU8540_PRIMARY_SENSOR_POWER_EN   MOP500_EGPIO(29)
#define CCU8540_SECONDARY_SENSOR_POWER_EN MOP500_EGPIO(30)
#define CCU8540_CAMERA_FLASH_READY        5
#define CCU8540_CAMERA_FLASH_EN1          MOP500_EGPIO(26)

#define uib_is_stuib() (type_of_uib == STUIB)
#define uib_is_not_selected() (type_of_uib == NO_UIB_SELECTED)
#define uib_is_u8500uib() (type_of_uib == U8500UIB)
#define uib_is_u8500uibr3() (type_of_uib == U8500UIB_R3)
#define uib_is_u9540uibs_v1() (type_of_uib == U9540UIBS_V1)
#define uib_is_u9540uibs_v2() (type_of_uib == U9540UIBS_V2)
#define uib_is_u9540uibs_v3() (type_of_uib == U9540UIBS_V3)
#define uib_is_u9540uibt_v1() (type_of_uib == U9540UIBT_V1)
#define uib_is_u9540uibs() (uib_is_u9540uibs_v1() || uib_is_u9540uibs_v2() \
	|| uib_is_u9540uibs_v3())
#define uib_is_u9540uibt() uib_is_u9540uibt_v1()

#define uib_is_u8540uibs_v2() (type_of_uib == U8540UIBS_V2)
#define uib_is_u8540uibs_v3() (type_of_uib == U8540UIBS_V3)
#define uib_is_u8540uibt_v3() (type_of_uib == U8540UIBT_V3)
#define uib_is_u8540uibs() (uib_is_u8540uibs_v3() || uib_is_u8540uibs_v2())

#endif
