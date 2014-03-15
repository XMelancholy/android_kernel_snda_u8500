/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author:  Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __BOARD_COMMON_SDI_H
#define __BOARD_COMMON_SDI_H

#include <linux/amba/mmci.h>

struct sdi_ios_pins {
	int enable;
	int vsel;
};

int ux500_common_sdi0_ios_handler_init(struct mmci_platform_data *sdi0,
		struct sdi_ios_pins *pins);

#endif
