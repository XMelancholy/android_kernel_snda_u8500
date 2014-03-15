/*
 * init.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Patrice Chotard <patrice.chotard@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef _PASR_INIT_H
#define _PASR_INIT_H

#define NR_DIES 8
#define NR_INT 8

struct ddr_die {
	phys_addr_t addr;
	unsigned long size;
};

struct pasr_info {
	int nr_dies;
	struct ddr_die die[NR_DIES];

	int nr_int;
	struct interleaved_area int_area[NR_INT];
};

extern struct pasr_info pasr_info;

#endif /* _PASR_INIT_H */
