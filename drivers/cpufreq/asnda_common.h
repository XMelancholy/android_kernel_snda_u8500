#ifndef __ASNDA_COMMON__
#define __ASNDA_COMMON__

#include <linux/kallsyms.h>

#define DEVICE_NAME "bambook s1"
// for get proc address
typedef unsigned long (*kallsyms_lookup_name_type)(const char *name);
static kallsyms_lookup_name_type kallsyms_lookup_name_ax = kallsyms_lookup_name;


#endif


