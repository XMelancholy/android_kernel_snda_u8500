/*
 * Copyright (C) 2011 ST-Ericsson SA
 * Author: Avinash A <avinash.a@stericsson.com> for ST-Ericsson
 * License terms:GNU General Public License (GPL) version 2
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input/cyttsp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_data/i2c-nomadik.h>
#include <linux/i2c/adp1653_plat.h>
#include <linux/input/matrix_keypad.h>
#include <linux/input/st-ftk.h>
#include <linux/leds-lp5521.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/amba/pl022.h>
#include <linux/pinctrl/machine.h>
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

#define NUM_SSP_CLIENTS 10

/* These simply sets bias for pins */
#define BIAS(a, b) static unsigned long a[] = { b }
BIAS(in_pd, PIN_INPUT_PULLDOWN);
BIAS(out_lo, PIN_OUTPUT_LOW);
BIAS(in_nopull, PIN_INPUT_NOPULL);
/* These also force them into GPIO mode */
BIAS(gpio_out_hi, PIN_OUTPUT_HIGH|PIN_GPIOMODE_ENABLED);
/* Sleep modes */
BIAS(slpm_out_hi, PIN_SLEEPMODE_ENABLED|PIN_SLPM_OUTPUT_HIGH);
BIAS(slpm_in_pu, PIN_SLEEPMODE_ENABLED|PIN_SLPM_INPUT_PULLUP);
BIAS(slpm_out_lo, PIN_SLEEPMODE_ENABLED|PIN_SLPM_OUTPUT_LOW);
BIAS(slpm_in_nopull_wkup, PIN_SLEEPMODE_ENABLED|PIN_INPUT_NOPULL|PIN_SLPM_WAKEUP_ENABLE);

/* These are default states associated with device and changed runtime */
#define DB8500_MUX(group, func, dev) \
	PIN_MAP_MUX_GROUP_DEFAULT(dev, "pinctrl-db8500", group, func)
#define DB8500_PIN(pin, conf, dev) \
	PIN_MAP_CONFIGS_PIN_DEFAULT(dev, "pinctrl-db8500", pin, conf)
#define DB8500_PIN_SLEEP(pin, conf, dev) \
	PIN_MAP_CONFIGS_PIN(dev, PINCTRL_STATE_SLEEP, "pinctrl-db8500",	\
			    pin, conf)
static struct adp1653_platform_data __initdata adp1653_pdata_u9540_uib = {
	.irq_no = CCU9540_CAMERA_FLASH_READY,
	.enable_gpio = CCU9540_CAMERA_FLASH_EN1,
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

static struct i2c_board_info __initdata mop500_i2c2_devices_u9540[] = {
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
		.platform_data = &adp1653_pdata_u9540_uib,
	}
};

static struct i2c_board_info __initdata mop500_i2c2_devices_tablet_v1[] = {
	{
		I2C_BOARD_INFO("dsi2lvds_i2c", 0x0F),
	}
};

/* cyttsp_gpio_board_init : configures the touch panel. */
static int cyttsp_plat_init(void)
{
	int ret;

	/*
	 * Enable the alternative C function
	 * in the PRCMU register
	 */
	prcmu_enable_spi2();

	ret = gpio_request(CYPRESS_SLAVE_SELECT_GPIO, "slave_select_gpio");
	if (ret < 0)
		pr_err("slave select gpio failed\n");

	ret = gpio_direction_output(CYPRESS_SLAVE_SELECT_GPIO, 1);
	if (ret < 0) {
		pr_err("slave select gpio direction failed\n");
		gpio_free(CYPRESS_SLAVE_SELECT_GPIO);
		return ret;
	}
	return 0;
}

static int cyttsp_plat_exit(void)
{
	prcmu_disable_spi2();
	gpio_free(CYPRESS_SLAVE_SELECT_GPIO);
	return 0;
}

static struct pl022_ssp_controller mop500_spi2_data = {
	.bus_id         = SPI023_2_CONTROLLER,
	.num_chipselect = NUM_SSP_CLIENTS,
};

static int cyttsp_wakeup(void)
{
	int ret;

	ret = gpio_request(CYPRESS_TOUCH_9540_INT_PIN, "Wakeup_pin");
	if (ret < 0) {
		pr_err("touch gpio failed\n");
		return ret;
	}
	ret = gpio_direction_output(CYPRESS_TOUCH_9540_INT_PIN, 1);
	if (ret < 0) {
		pr_err("touch gpio direction failed\n");
		goto out;
	}
	gpio_set_value(CYPRESS_TOUCH_9540_INT_PIN, 0);
	gpio_set_value(CYPRESS_TOUCH_9540_INT_PIN, 1);
	/*
	 * To wake up the controller from sleep
	 * state the interrupt pin needs to be
	 * pulsed twice with a delay greater
	 * than 2 micro seconds.
	 */
	udelay(3);
	gpio_set_value(CYPRESS_TOUCH_9540_INT_PIN, 0);
	gpio_set_value(CYPRESS_TOUCH_9540_INT_PIN, 1);
	ret = gpio_direction_input(CYPRESS_TOUCH_9540_INT_PIN);
	if (ret < 0) {
		pr_err("touch gpio direction IN config failed\n");
		goto out;
	}
out:
	gpio_free(CYPRESS_TOUCH_9540_INT_PIN);
	return 0;
}

struct cyttsp_platform_data cyttsp_9540_platdata = {
	.maxx = 480,
	.maxy = 854,
	.use_hndshk = 0,
	.use_sleep = 1,
	.act_intrvl = CY_ACT_INTRVL_DFLT,  /* Active refresh interval; ms */
	.tch_tmout = CY_TCH_TMOUT_DFLT,   /* Active touch timeout; ms */
	.lp_intrvl = CY_LP_INTRVL_DFLT,   /* Low power refresh interval; ms */
	.init = cyttsp_plat_init,
	.exit = cyttsp_plat_exit,
	.name = CY_SPI_NAME,
	.wakeup = cyttsp_wakeup,
	.rst_gpio = CYPRESS_TOUCH_9540_RST_GPIO,
};

static void cyttsp_spi_cs_control(u32 command)
{
	if (command == SSP_CHIP_SELECT)
		gpio_set_value(CYPRESS_SLAVE_SELECT_GPIO, 0);
	else if (command == SSP_CHIP_DESELECT)
		gpio_set_value(CYPRESS_SLAVE_SELECT_GPIO, 1);
}

static struct pl022_config_chip cyttsp_ssp_config_chip = {
	.com_mode = INTERRUPT_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	/* we can act as master only */
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = 0,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_16_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_16,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = cyttsp_spi_cs_control,
};

static struct spi_board_info cypress_spi_9540_devices[] = {
	{
		.modalias = CY_SPI_NAME,
		.controller_data = &cyttsp_ssp_config_chip,
		.platform_data = &cyttsp_9540_platdata,
		.max_speed_hz = 1000000,
		.bus_num = SPI023_2_CONTROLLER,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.irq = NOMADIK_GPIO_TO_IRQ(CYPRESS_TOUCH_9540_INT_PIN),
	}
};

static struct pinctrl_map __initdata u9540uibs_v1_pinmap[] = {
	/* Mux in SPI2 pins on the "other C1" altfunction */
	DB8500_MUX("spi2_oc1_2", "spi2", "spi2"),
	DB8500_PIN("GPIO216_AG12", gpio_out_hi, "spi2"),
	DB8500_PIN("GPIO218_AH11", in_pd, "spi2"), /* RXD */
	DB8500_PIN("GPIO215_AH13", out_lo, "spi2"), /* TXD */
	DB8500_PIN("GPIO217_AH12", out_lo, "spi2"), /* CLK */
	/* SPI2 sleep state */
	DB8500_PIN_SLEEP("GPIO216_AG12", slpm_out_hi, "spi2"),
	DB8500_PIN_SLEEP("GPIO218_AH11", slpm_in_pu, "spi2"),
	DB8500_PIN_SLEEP("GPIO215_AH13", slpm_out_lo, "spi2"),
	DB8500_PIN_SLEEP("GPIO217_AH12", slpm_out_lo, "spi2"),
};

static struct pinctrl_map __initdata u9540uibs_v2_pinmap[] = {
	/* Mux in I2C3 */
	DB8500_MUX("i2c3_c_1", "i2c3", "i2c3"),
	DB8500_PIN("GPIO216_AG12", in_nopull, "i2c3"), /* SDA */
	DB8500_PIN("GPIO218_AH11", in_nopull, "i2c3"), /* SCL */
	/* I2C3 sleep state */
	DB8500_PIN_SLEEP("GPIO216_AG12", slpm_in_nopull_wkup, "i2c3"), /* SDA */
	DB8500_PIN_SLEEP("GPIO218_AH11", slpm_in_nopull_wkup, "i2c3"), /* SCL */
};

static UX500_PINS(ccu9540_pins_i2c3,
	GPIO216_I2C3_SDA | PIN_INPUT_NOPULL,
		PIN_SLPM_GPIO | PIN_SLPM_INPUT_NOPULL,
	GPIO218_I2C3_SCL | PIN_INPUT_NOPULL,
		PIN_SLPM_GPIO | PIN_SLPM_INPUT_NOPULL,
);

static struct ux500_pin_lookup pins_uibsv2[] = {
	PIN_LOOKUP("nmk-i2c.3", &ccu9540_pins_i2c3),
};

struct ftk_platform_data ftk_platdata_uibsv2 = {
	.gpio_rst = 145,
	.x_min = 0,
	.x_max = 720,
	.y_min = 0,
	.y_max = 1280,
	.p_min = 0,
	.p_max = 256,
	.portrait = 1,
	.patch_file = "st-ftks.bin",
};

static struct i2c_board_info __initdata i2c3_devices_uibsv2[] = {
	{
		/* STMT05 touchscreen */
		I2C_BOARD_INFO("st-ftk", 0x4b),
		.irq = NOMADIK_GPIO_TO_IRQ(146),
		.platform_data = &ftk_platdata_uibsv2,
	},
};

struct ftk_platform_data ftk_platdata_uibt = {
	.gpio_rst = 145,
	.x_min = 0,
	.x_max = 1280,
	.y_min = 0,
	.y_max = 800,
	.p_min = 0,
	.p_max = 256,
	.portrait = 0,
	.patch_file = "st-ftkt.bin",
};

static struct i2c_board_info __initdata i2c3_devices_tablet_v1[] = {
	{
		/* STMT07 touchscreen */
		I2C_BOARD_INFO("st-ftk", 0x4b),
		.irq = NOMADIK_GPIO_TO_IRQ(146),
		.platform_data = &ftk_platdata_uibt,
	},
};

static void __init mop500_u9540_cyttsp_init(void)
{
		spi_register_board_info(cypress_spi_9540_devices,
			ARRAY_SIZE(cypress_spi_9540_devices));
}

#ifdef CONFIG_UX500_GPIO_KEYS
static struct gpio_keys_button gpio_keys[] = {
	{
		.desc = "SFH7741 Proximity Sensor",
		.type = EV_SW,
		.code = SW_FRONT_PROXIMITY,
		.active_low = 0,
		.can_disable = 1,
		.gpio = 229,
	},
};

static struct regulator *gpio_keys_regulator;
static int gpio_keys_activate(struct device *dev);
static void gpio_keys_deactivate(struct device *dev);

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons = gpio_keys,
	.nbuttons = ARRAY_SIZE(gpio_keys),
	.enable = gpio_keys_activate,
	.disable = gpio_keys_deactivate,
};

static struct platform_device gpio_keys_device = {
	.name = "gpio-keys",
	.id = 0,
	.dev = {
		.platform_data = &gpio_keys_data,
	},
};

static int gpio_keys_activate(struct device *dev)
{
	gpio_keys_regulator = regulator_get(&gpio_keys_device.dev, "vcc");
	if (IS_ERR(gpio_keys_regulator)) {
		dev_err(&gpio_keys_device.dev, "no regulator\n");
		return PTR_ERR(gpio_keys_regulator);
	}
	regulator_enable(gpio_keys_regulator);

	/*
	 * Please be aware that the start-up time of the SFH7741 is
	 * 120 ms and during that time the output is undefined.
	 */
	return 0;
}

static void gpio_keys_deactivate(struct device *dev)
{
	if (!IS_ERR(gpio_keys_regulator)) {
		regulator_disable(gpio_keys_regulator);
		regulator_put(gpio_keys_regulator);
	}
}

static __init void gpio_keys_init(void)
{
	struct ux500_pins *gpio_keys_pins = ux500_pins_get("gpio-keys.0");

	if (gpio_keys_pins == NULL) {
		pr_err("gpio_keys: Failed to get pins\n");
		return;
	}

	ux500_pins_enable(gpio_keys_pins);
}
#else
static inline void gpio_keys_init(void) { }
#endif

/* UIB specific platform devices */
static struct platform_device *uibs_v1_platform_devs[] __initdata = {
#ifdef CONFIG_UX500_GPIO_KEYS
	&gpio_keys_device,
#endif
};

void __init mop500_u9540uibs_v1_init(void)
{
	pinctrl_register_mappings(u9540uibs_v1_pinmap,
			ARRAY_SIZE(u9540uibs_v1_pinmap));

	mop500_u9540_cyttsp_init();
	db8500_add_spi2(NULL, &mop500_spi2_data);

	mop500_uib_i2c_add(2, mop500_i2c2_devices_u9540,
			ARRAY_SIZE(mop500_i2c2_devices_u9540));

	gpio_keys_init();
	platform_add_devices(uibs_v1_platform_devs,
			ARRAY_SIZE(uibs_v1_platform_devs));
}

void __init mop500_u9540uibs_v2_init(void)
{
	ux500_pins_add(pins_uibsv2, ARRAY_SIZE(pins_uibsv2));
	pinctrl_register_mappings(u9540uibs_v2_pinmap,
			ARRAY_SIZE(u9540uibs_v2_pinmap));

	db8500_add_i2c3(NULL, NULL);

	mop500_uib_i2c_add(2, mop500_i2c2_devices_u9540,
			ARRAY_SIZE(mop500_i2c2_devices_u9540));
	mop500_uib_i2c_add(3, i2c3_devices_uibsv2,
			ARRAY_SIZE(i2c3_devices_uibsv2));
}

void __init mop500_u9540uibs_v3_init(void)
{
	ux500_pins_add(pins_uibsv2, ARRAY_SIZE(pins_uibsv2));
	pinctrl_register_mappings(u9540uibs_v2_pinmap,
			ARRAY_SIZE(u9540uibs_v2_pinmap));

	db8500_add_i2c3(NULL, NULL);

	mop500_uib_i2c_add(2, mop500_i2c2_devices_u9540,
			ARRAY_SIZE(mop500_i2c2_devices_u9540));
	mop500_uib_i2c_add(3, i2c3_devices_uibsv2,
			ARRAY_SIZE(i2c3_devices_uibsv2));
}

void __init mop500_u9540uibt_v1_init(void)
{
	ux500_pins_add(pins_uibsv2, ARRAY_SIZE(pins_uibsv2));
	pinctrl_register_mappings(u9540uibs_v2_pinmap,
			ARRAY_SIZE(u9540uibs_v2_pinmap));

	db8500_add_i2c3(NULL, NULL);

	mop500_uib_i2c_add(2, mop500_i2c2_devices_tablet_v1,
				ARRAY_SIZE(mop500_i2c2_devices_tablet_v1));
	mop500_uib_i2c_add(3, i2c3_devices_tablet_v1,
				ARRAY_SIZE(i2c3_devices_tablet_v1));
}
