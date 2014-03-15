/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Alexandre Torgue <alexandre.torgue@st.com>
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef _AP9540_C2C_PLAT_H_
#define _AP9540_C2C_PLAT_H_

/**
 * struct c2c_platform_data
 * @wumod_gpio: gpio used to wake up modem from AP
 *
 * This structure is required for configuration of exynos4_tmu driver.
 */

struct c2c_platform_data {
	int wumod_gpio;
};

#endif
