/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Chris Blair <chris.blair@stericsson.com> for ST-Ericsson
 *	   Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson
 *
 * License terms: GNU General Public License (GPL), version 2
 *
 * Version specific initialization for U9540 CCU boards.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <plat/gpio-nomadik.h>
#include <plat/pincfg.h>
#include <mach/hardware.h>
#include <mach/devices.h>
#include <linux/l3g4200d.h>
#include <linux/i2c.h>

#include "pins-db8500.h"
#include "pins.h"
#include "board-mop500.h"
#include "board-ccu9540.h"
#include "board-ccu9540-modem.h"
#include "board-ccux540-ver.h"

static struct platform_device *ccu9540_hsi_us_v1_platform_devs[] __initdata = {
#ifdef CONFIG_HSI
	&u8500_hsi_device,
	&ap9540_modem_control_device,
#endif
};

static struct platform_device *ccu9540_c2c_us_v1_platform_devs[] __initdata = {
#ifdef CONFIG_C2C
	&ux500_c2c_device,
	&ap9540_modem_control_device,
#endif
};

static struct l3g4200d_gyr_platform_data l3g4200d_pdata_u9540 = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negative_x = 1,
	.negative_y = 0,
	.negative_z = 1,
};

static struct i2c_board_info __initdata ccu9540_i2c2_devices_gyro_68[] = {
	{
		/* L3G4200d Gyroscope */
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &l3g4200d_pdata_u9540,
	},
};

static struct i2c_board_info __initdata ccu9540_i2c2_devices_gyro_6a[] = {
	/*
	 * Gyroscope chip changed for C2C v1.1 and HSI v2
	 * older versions use l3g4200d(0x68) newer version uses l3G20(0x6a)
	 */
	{
		/* L3G4200d Gyroscope */
		I2C_BOARD_INFO("l3g4200d", 0x6a),
		.platform_data = &l3g4200d_pdata_u9540,
	},
};

static const pin_cfg_t u9540_pins_hsi[] = {
	GPIO219_HSIR_FLA0 | PIN_INPUT_PULLDOWN,
	GPIO220_HSIR_DAT0 | PIN_INPUT_PULLDOWN,
	GPIO221_HSIR_RDY0 | PIN_OUTPUT_LOW,
	GPIO222_HSIT_FLA0 | PIN_OUTPUT_LOW,
};

static const pin_cfg_t u9540_pins_c2c[] = {
	GPIO219_MC3_CLK,
	GPIO220_MC3_FBCLK,
	GPIO221_MC3_CMD,
	GPIO222_MC3_DAT0,
};

static const pin_cfg_t u9540_c2c_us_V1_1_pins[] = {
	GPIO128_GPIO | PIN_INPUT_PULLDOWN,
	GPIO223_GPIO | PIN_INPUT_PULLUP,
	GPIO224_GPIO | PIN_INPUT_PULLUP,
};

enum ipc_type {
	IPC_NONE,
	IPC_HSI,
	IPC_C2C,
	IPC_XMIP,
};

const char *ipc_name[] = {
	"none",
	"hsi",
	"c2c",
	"xmip",
};

enum audio_type {
	AUDIO_NONE,
	AUDIO_SLIM,
	AUDIO_HALF_SLIM,
	AUDIO_FAT,
};

const char *audio_name[] = {
	"none",
	"slim",
	"half-slim",
	"fat",
};

struct ccu {
	const char *name;
	void (*init)(struct ccu *);
	enum ccux540_ver id;
	enum ipc_type ipc;
	enum audio_type audio;
	u8 modem_version;
	u8 rcf;
};

static void __init ccu9540_hsi_init(struct ccu *ccu)
{
	nmk_config_pins((pin_cfg_t *)u9540_pins_hsi,
			ARRAY_SIZE(u9540_pins_hsi));

	platform_add_devices(ccu9540_hsi_us_v1_platform_devs,
		ARRAY_SIZE(ccu9540_hsi_us_v1_platform_devs));
}

static void __init ccu9540_hsi_v1_init(struct ccu *ccu)
{
	mop500_uib_i2c_add(2, ccu9540_i2c2_devices_gyro_68,
			ARRAY_SIZE(ccu9540_i2c2_devices_gyro_68));
	ccu9540_hsi_init(ccu);
}

static void __init ccu9540_hsi_v2_init(struct ccu *ccu)
{
	mop500_uib_i2c_add(2, ccu9540_i2c2_devices_gyro_6a,
			ARRAY_SIZE(ccu9540_i2c2_devices_gyro_6a));
	ccu9540_hsi_init(ccu);
}

#define U8500_CR_REG1			0x04
#define U8500_CR_REG1_MSPR4ACTIVE	(1<<15)

static void __init ccu9540_c2c_init(struct ccu *ccu)
{
	void __iomem *crbase;

	nmk_config_pins((pin_cfg_t *)u9540_pins_c2c,
			ARRAY_SIZE(u9540_pins_c2c));

	/* config Pressure sensor interrupt gpios on X9540 C2C/US V1.1 Board, */
	if (ccu->id == CCU9540_C2C_US_V1_1) {
		nmk_config_pins((pin_cfg_t *)u9540_c2c_us_V1_1_pins,
				ARRAY_SIZE(u9540_c2c_us_V1_1_pins));
	}

	platform_add_devices(ccu9540_c2c_us_v1_platform_devs,
		ARRAY_SIZE(ccu9540_c2c_us_v1_platform_devs));

	/* set bit Msp4active to 1 (MSP4 selected) in the ConfigRegister 1 */
	crbase = ioremap(U8500_CR_BASE, PAGE_SIZE);
	if (!crbase) {
		pr_err("Failed to map U8500_CR_BASE\n");
		return;
	}
	writel(readl(crbase + U8500_CR_REG1) | U8500_CR_REG1_MSPR4ACTIVE,
		crbase + U8500_CR_REG1);
	iounmap(crbase);
}

static void __init ccu9540_c2c_v1_init(struct ccu *ccu)
{
	mop500_uib_i2c_add(2, ccu9540_i2c2_devices_gyro_68,
			ARRAY_SIZE(ccu9540_i2c2_devices_gyro_68));
	ccu9540_c2c_init(ccu);
}

static void __init ccu9540_c2c_v1_1_and_upper_init(struct ccu *ccu)
{
	mop500_uib_i2c_add(2, ccu9540_i2c2_devices_gyro_6a,
			ARRAY_SIZE(ccu9540_i2c2_devices_gyro_6a));
	ccu9540_c2c_init(ccu);
}

static struct ccu __initdata ccus[] = {
	[CCU9540_HSI_US_V1] = {
		.name	= "CCU9540-HSI-US-V1",
		.id	= CCU9540_HSI_US_V1,
		.init	= ccu9540_hsi_v1_init,
		.ipc	= IPC_HSI,
		.audio	= AUDIO_HALF_SLIM,
	},
	[CCU9540_HSI_US_V1_05] = {
		.name	= "CCU9540-HSI-US-V1_05",
		.id	= CCU9540_HSI_US_V1_05,
		.init	= ccu9540_hsi_v1_init,
		.ipc	= IPC_HSI,
		.audio	= AUDIO_HALF_SLIM,
	},
	[CCU9540_HSI_US_V1_1] = {
		.name	= "CCU9540-HSI-US-V1_1",
		.id	= CCU9540_HSI_US_V1_1,
		.init	= ccu9540_hsi_v1_init,
		.ipc	= IPC_HSI,
		.audio	= AUDIO_HALF_SLIM,
		.modem_version = 2,
	},
	[CCU9540_HSI_EU_V2] = {
		.name	= "CCU9540-HSI-EU-V2",
		.id	= CCU9540_HSI_EU_V2,
		.init	= ccu9540_hsi_v2_init,
		.ipc	= IPC_HSI,
		.audio	= AUDIO_HALF_SLIM,
		.rcf	= 1,
		.modem_version = 2,
	},
	[CCU9540_C2C_US_V1] = {
		.name	= "CCU9540-C2C-US-V1",
		.id	= CCU9540_C2C_US_V1,
		.init	= ccu9540_c2c_v1_init,
		.ipc	= IPC_C2C,
		.audio	= AUDIO_FAT,
	},
	[CCU9540_C2C_US_V1_1] = {
		.name	= "CCU9540-C2C-US-V1_1",
		.id	= CCU9540_C2C_US_V1_1,
		.init	= ccu9540_c2c_v1_1_and_upper_init,
		.ipc	= IPC_C2C,
		.audio	= AUDIO_FAT,
		.rcf	= 1,
	},
	[CCU9540_C2C_EU_V2] = {
		.name	= "CCU9540-C2C-EU-V2",
		.id	= CCU9540_C2C_EU_V2,
		.init	= ccu9540_c2c_v1_1_and_upper_init,
		.ipc	= IPC_C2C,
		.audio	= AUDIO_FAT,
		.rcf	= 1,
		.modem_version = 2,
	},
	[CCU8550_US_V1] = {
		.name	= "CCU8550-US-V1",
		.id	= CCU8550_US_V1,
		.init	= ccu9540_c2c_v1_1_and_upper_init,
		.ipc	= IPC_C2C,
		.audio	= AUDIO_FAT,
		.rcf	= 1,
	},
	[CCU8540_V1] = {
		.name	= "CCU8540-V1",
		.id	= CCU8540_V1,
		.ipc	= IPC_XMIP,
		.audio	= AUDIO_FAT,
	}
};

static u8 __init ccux540_read_revision_id(struct ccux540_version_pdata *pdata)
{
	u8 revision = 0xFF;

	if (gpio_request(pdata->rev_pin0, __func__))
		goto err;
	if (gpio_request(pdata->rev_pin1, __func__))
		goto err0;
	if (gpio_request(pdata->rev_pin2, __func__))
		goto err1;
	if (gpio_request(pdata->rev_pin3, __func__))
		goto err2;

	if (gpio_direction_input(pdata->rev_pin0) ||
			gpio_direction_input(pdata->rev_pin1) ||
			gpio_direction_input(pdata->rev_pin2) ||
			gpio_direction_input(pdata->rev_pin3))
		goto err3;

	revision = (!!gpio_get_value_cansleep(pdata->rev_pin0)) |
			(!!gpio_get_value_cansleep(pdata->rev_pin1) << 1) |
			(!!gpio_get_value_cansleep(pdata->rev_pin2) << 2) |
			(!!gpio_get_value_cansleep(pdata->rev_pin3) << 3);
err3:
	gpio_free(pdata->rev_pin3);
err2:
	gpio_free(pdata->rev_pin2);
err1:
	gpio_free(pdata->rev_pin1);
err0:
	gpio_free(pdata->rev_pin0);
err:
	return revision;
}

static ssize_t show_hw_detect_hsi(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	pr_warning("%s is deprecated, use config_ipc/audio instead\n"
			, attr->attr.name);
	return sprintf(buf, "%d\n", ccu->ipc == IPC_HSI);
}

static ssize_t show_hw_detect_c2c(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	pr_warning("%s is deprecated, use config_ipc/audio instead\n"
			, attr->attr.name);

	return sprintf(buf, "%d\n", ccu->ipc == IPC_C2C);
}

static ssize_t show_config_ipc(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ipc_name[ccu->ipc]);
}

static ssize_t show_config_audio(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", audio_name[ccu->audio]);
}

static ssize_t show_hw_detect_rcfilter(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ccu->rcf);
}

static ssize_t show_hw_detect_modem_version(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ccu *ccu = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ccu->modem_version);
}

/* Deprecated attributes */
static DEVICE_ATTR(config_hsi, S_IRUGO, show_hw_detect_hsi, NULL);
static DEVICE_ATTR(config_c2c, S_IRUGO, show_hw_detect_c2c, NULL);
static DEVICE_ATTR(audio_half_slim, S_IRUGO, show_hw_detect_hsi, NULL);
static DEVICE_ATTR(audio_fat, S_IRUGO, show_hw_detect_c2c, NULL);


static DEVICE_ATTR(config_ipc, S_IRUGO, show_config_ipc, NULL);
static DEVICE_ATTR(config_audio, S_IRUGO, show_config_audio, NULL);
static DEVICE_ATTR(rcfilter, S_IRUGO, show_hw_detect_rcfilter, NULL);
static DEVICE_ATTR(modem_version, S_IRUGO, show_hw_detect_modem_version, NULL);

static const struct attribute *ccuver_attr[] = {
	&dev_attr_config_hsi.attr,
	&dev_attr_config_c2c.attr,
	&dev_attr_config_ipc.attr,
	&dev_attr_config_audio.attr,
	&dev_attr_audio_half_slim.attr,
	&dev_attr_audio_fat.attr,
	&dev_attr_rcfilter.attr,
	&dev_attr_modem_version.attr,
	NULL,
};

static const struct attribute_group ccuver_attr_group = {
	.attrs = (struct attribute **)ccuver_attr,
};

static struct ccu * __init ccu9540_get_revision(struct platform_device *pdev)
{
	struct ccux540_version_pdata *pdata = pdev->dev.platform_data;
	u8 rev;
	struct ccu *ccu = NULL;

	rev = ccux540_read_revision_id(pdata);
	switch (rev) {
	case 0x00:
		ccu = &ccus[CCU9540_HSI_US_V1];
		break;
	case 0x02:
		ccu = &ccus[CCU9540_C2C_US_V1];
		break;
	case 0x03:
		ccu = &ccus[CCU9540_C2C_EU_V2];
		break;
	case 0x04:
		ccu = &ccus[CCU9540_HSI_US_V1_1];
		break;
	case 0x05:
		ccu = &ccus[CCU9540_HSI_EU_V2];
		break;
	case 0x06:
	case 0x07:
		ccu = &ccus[CCU9540_C2C_US_V1_1];
		break;
	case 0x08:
		ccu = &ccus[CCU9540_HSI_US_V1_05];
		break;
	case 0x0a:
		ccu = &ccus[CCU8550_US_V1];
		break;
	case 0xFF:
		dev_err(&pdev->dev, "failed to read CCU revision\n");
		break;
	default:
		dev_err(&pdev->dev, "u9540 CCU ID 0x%02x not supported!\n", rev);
	}

	return ccu;
}

static struct ccu * __init ccu8540_get_revision(struct platform_device *pdev)
{
	struct ccux540_version_pdata *pdata = pdev->dev.platform_data;
	u8 rev;
	struct ccu *ccu = NULL;

	rev = ccux540_read_revision_id(pdata);
	switch (rev) {
	case 0x00:
		ccu = &ccus[CCU8540_V1];
		break;
	case 0xFF:
		dev_err(&pdev->dev, "failed to read CCU revision\n");
		break;
	default:
		dev_err(&pdev->dev, "u8540 CCU ID 0x%02x not supported!\n", rev);
	}

	return ccu;
}

static int __init ccux540_version_probe(struct platform_device *pdev)
{
	struct ccu *drv_dat, *ccu = NULL;
	struct ccux540_version_pdata *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data passed\n");
		return -EINVAL;
	}

	/* Board detection can be forced when passed in platform data */
	if (pdata->version != CCUX540_NOT_SET) {
		ccu = &ccus[pdata->version];
		goto out;
	}

	switch (pdata->family) {
	case CCUX540_VER_U9540_FAMILY:
		ccu = ccu9540_get_revision(pdev);
		break;
	case CCUX540_VER_L8540_FAMILY:
		ccu = ccu8540_get_revision(pdev);
		break;
	default:
		dev_err(&pdev->dev, "Unknown family %d\n", pdata->family);
		return -EINVAL;
	}

	if (!ccu) {
		dev_err(&pdev->dev, "No board rev. found for family (%d)\n"
				, pdata->family);
		return -EINVAL;
	}

out:
	dev_info(&pdev->dev, "%s (identified)\n", ccu->name);

	if (ccu->init)
		ccu->init(ccu);

	accessory_detect_config(ccu->id);

	drv_dat = kzalloc(sizeof(struct ccu), GFP_KERNEL);
	if (!drv_dat)
		return -ENOMEM;

	memcpy(drv_dat, ccu, sizeof(struct ccu));
	dev_set_drvdata(&pdev->dev, drv_dat);

	return sysfs_create_group(&pdev->dev.kobj, &ccuver_attr_group);
}

static struct platform_driver ccux540_version_driver = {
	.driver = {
		.name = "modem-hwcfg",
		.owner = THIS_MODULE,
	},
};

static int __init ccux540_version_init(void)
{
	return platform_driver_probe(&ccux540_version_driver,
			ccux540_version_probe);
}
module_init(ccux540_version_init);
