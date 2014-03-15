/*
 * service.c
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Patrice Chotard <patrice.chotard@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/module.h>
#include <linux/pasr.h>

#include "init.h"

int pasr_get_interleave_info(struct interleave_info *info)
{
	if (pasr_info.nr_int) {
		info->nr_int = pasr_info.nr_int;
		info->int_area = &pasr_info.int_area[0];
		info->granularity = pasr_info.int_area[0].granularity;
		return 0;
	} else {
		info->nr_int = 0;
		info->int_area = NULL;
		info->granularity = GRANULARITY_NOT_SET;
		/* return ENODATA if no interleave info are available */
		return -ENODATA;
	}
}
EXPORT_SYMBOL(pasr_get_interleave_info);
