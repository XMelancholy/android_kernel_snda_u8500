/*
 * hx170dec.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author: <benjamin.gaignard@stericsson.org> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __HX170_H__
#define __HX170_H__

#define HX170_IOCTL_GET_REGSIZE _IOR('h', 0, unsigned long)
#define HX170_IOCTL_DECODE      _IOWR('h', 1, struct hx170_reglist_t *)

#define HX170_MAX_REGS  (60+40+10)
struct hx170_reglist_t {
	int n;
	int offset[HX170_MAX_REGS];
	int data[HX170_MAX_REGS];
};

#endif
