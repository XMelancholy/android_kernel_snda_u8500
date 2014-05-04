/*
 * Copyright (C) 2011 ST-Ericsson SA
 * Author: Kerello c <christophe.kerello@stericsson.com> for ST-Ericsson
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/adp1653_plat.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds-lp5521.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/amba/pl022.h>
#include <linux/platform_data/i2c-nomadik.h>
#include <asm/mach-types.h>
#include <plat/gpio-nomadik.h>
#include <plat/pincfg.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/irqs-db8500.h>
#include "pins-db8500.h"
#include "board-mop500.h"
#include "devices-db8500.h"
#include "pins.h"

static struct adp1653_platform_data __initdata adp1653_pdata_u8540_uib = {
	.irq_no = CCU8540_CAMERA_FLASH_READY,
	.enable_gpio = CCU8540_CAMERA_FLASH_EN1,
};

static struct lp5521_led_config lp5521_pri_led[] = {
	[0] = {
		.chan_nr = 0,
		.led_current = 0x2f,
		.max_current = 0x5f,
	},
	[1] = {
		.chan_nr = 1,
		.led_current = 0x2f,
		.max_current = 0x5f,
	},
	[2] = {
		.chan_nr = 2,
		.led_current = 0x2f,
		.max_current = 0x5f,
	},
};

static struct lp5521_platform_data __initdata lp5521_pri_data = {
	.label		= "lp5521_pri",
	.led_config	= &lp5521_pri_led[0],
	.num_channels	= 3,
	.clock_mode	= LP5521_CLOCK_EXT,
};

static struct i2c_board_info __initdata mop500_i2c2_devices_u8540[] = {
	{
		/* Light sensor Rohm BH1780GLI */
		I2C_BOARD_INFO("bh1780", 0x29),
	},
	{
		/* lp5521 LED driver */
		I2C_BOARD_INFO("lp5521", 0x32),
		.platform_data = &lp5521_pri_data,
	},
	{
		/* Flash LED driver */
		I2C_BOARD_INFO("adp1653", 0x30),
		.platform_data = &adp1653_pdata_u8540_uib,
	}
};

static struct i2c_board_info __initdata i2c5_devices_uibs[] = {
	{
		/* STMT05 touchscreen */
		I2C_BOARD_INFO("st-ftk", 0x4b),
		.irq = NOMADIK_GPIO_TO_IRQ(146),
	},
};

#define U8540_UIB_I2C_SEL MOP500_EGPIO(34)

void u8540_uib_touchscreen_init(void)
{
	int ret;

	ret = gpio_request(U8540_UIB_I2C_SEL, __func__);
	if (ret < 0) {
		pr_err("%s: failed to request UIB I2C SEL GPIO\n", __func__);
		goto out;
	}

	ret = gpio_direction_output(U8540_UIB_I2C_SEL, 0);
	if (ret < 0) {
		pr_err("%s: failed to configure UIB I2C SEL GPIO\n", __func__);
		goto out_free_pin;
	}

out_free_pin:
	gpio_free(U8540_UIB_I2C_SEL);
out:
	return;
}

void __init mop500_u8540uibs_v2_init(void)
{
	u8540_uib_touchscreen_init();

	mop500_uib_i2c_add(2, mop500_i2c2_devices_u8540,
			ARRAY_SIZE(mop500_i2c2_devices_u8540));
	mop500_uib_i2c_add(5, i2c5_devices_uibs,
			ARRAY_SIZE(i2c5_devices_uibs));
}

void __init mop500_u8540uibs_v3_init(void)
{
	u8540_uib_touchscreen_init();

	mop500_uib_i2c_add(2, mop500_i2c2_devices_u8540,
			ARRAY_SIZE(mop500_i2c2_devices_u8540));
	mop500_uib_i2c_add(5, i2c5_devices_uibs,
			ARRAY_SIZE(i2c5_devices_uibs));
}
