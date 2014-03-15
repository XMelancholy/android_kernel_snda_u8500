/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Michel JAOUEN <michel.jaouen@stericsson.con> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __MACH_UX500_USECASE_GOV
#define __MACH_UX500_USECASE_GOV

struct usecase_config {
	char *name;
	unsigned int min_arm;
	unsigned int max_arm;
	unsigned long cpuidle_multiplier;
	bool second_cpu_online;
	bool l2_prefetch_en;
	unsigned int cpuidle_force_state; /* Forced cpu idle state. */
	bool vc_override; /* QOS override for voice-call. */
};

enum ux500_uc {
	UX500_UC_NORMAL	= 0,
	UX500_UC_VC,
	UX500_UC_LPA,
	UX500_UC_MAX, /* Add use case above this. */
};

#endif
