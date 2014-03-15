/*
 * Copyright (C) ST-Ericsson SA 2010-2012
 *
 * Author: Mattias Wallin <mattias.wallin@stericsson.com> for ST-Ericsson
 *
 * License Terms: GNU General Public License v2
 *
 */
#include <linux/kernel.h>
#include <linux/rtc.h>

#define U8500_RTC_MASTER_DEV "ab8500-rtc"
#define RTC_DEV_STR_LEN 6

/* Add a ux500 specific hook from rtc/hctosys.c */
void rtc_hctohc(struct rtc_time tm) {
	/*
	 * RTC_ALARM_DEV_NAME is not the same as rtc device name: rtc0 and rtc1
	 * The rtc numbering can not be configured instead it
	 * will be decided at boot time so for u8500
	 * rtc-ab8500 will get rtc0 becase it is an platform device and
	 * rtc-pl031 will get rtc1 because it is an amba device.
	 * hctosys device is default rtc0
	 * */
	struct rtc_device *rtc = rtc_class_open("rtc1");

	if (rtc == NULL) {
		pr_warn("%s: unable to open 2nd rtc device (%s)\n",
			__FILE__, "rtc1");
		return;
	}

	if (rtc_set_time(rtc, &tm)) {
		dev_err(rtc->dev.parent, ": unable to set the 2nd rtc\n");
		return;
	}

	dev_dbg(rtc->dev.parent, "setting rtc1 to "
		"%d-%02d-%02d %02d:%02d:%02d UTC\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);
}

void rtc_sync_slaves_if_master(struct rtc_device *rtc,
			       int max_rtc_devs)
{
	struct rtc_time rtc_master_tm;
	char rtc_i_str[RTC_DEV_STR_LEN];
	struct rtc_device *rtc_i;
	int i;

	if (strcmp(rtc->name, U8500_RTC_MASTER_DEV) != 0)
		return;

	dev_dbg(rtc->dev.parent,
		"is the master RTC, now syncing slaves...");

	for (i = 0; i < max_rtc_devs; i++) {
		sprintf(rtc_i_str, "rtc%d", i);
		rtc_i = rtc_class_open(rtc_i_str);

		if (rtc_i == NULL || strcmp(rtc->name, rtc_i->name) == 0)
			continue;

		if (rtc_read_time(rtc, &rtc_master_tm)) {
			dev_err(rtc->dev.parent,
				": unable to get the master rtc\n");
			return;
		}

		if (rtc_set_time(rtc_i, &rtc_master_tm)) {
			dev_err(rtc->dev.parent,
				": unable to set the rtc slave %s\n",
				rtc_i->name);
			return;
		} else {
			dev_dbg(rtc->dev.parent,
				"%s slave has been synced\n",
				rtc_i->name);
		}
	}
}
