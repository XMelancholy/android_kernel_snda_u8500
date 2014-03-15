#ifndef _FT5306_H
#define _FT5306_H

#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CFG_SUPPORT_AUTO_UPG
#define FT5306_NAME "tp_ft5306"
#define DATA_LENGTH (3 + 6*DATA_FINGERS_MAX)
#define DATA_FINGERS_MAX   10
#define DATA_XY_START 3

#define PMODE_HIBERNATE 0x3
#define PMODE_MONITOR   0x1
#define PMODE_ACTIVE    0x0

#define ID_G_PMODE 0xA5
#define ID_G_FIRMWARE_ID 0xA6

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct ft5306_platform_data {
	int (*wake_up)(void);
	int res_x;
	int res_y;
	int touch_x_max;
	int touch_y_max;
};

#endif
