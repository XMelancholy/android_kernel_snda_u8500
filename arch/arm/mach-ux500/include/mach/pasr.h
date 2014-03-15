/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Maxime Coquelin <maxime.coquelin@stericsson> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __ASM_ARCH_PASR_H
#define __ASM_ARCH_PASR_H

#define PASR_SECTION_SZ_BITS		26 /* 64MB sections */
#define PASR_SECTION_SZ			(1 << PASR_SECTION_SZ_BITS)
#define PASR_MAX_DIE_NR			4
#define PASR_MAX_SECTION_NR_PER_DIE	8 /* 32 * 64MB = 2GB */

#endif
