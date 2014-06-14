/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author:  Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/pinctrl/machine.h>

#include <asm/mach-types.h>

#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>

#include <mach/hardware.h>
#include <mach/suspend.h>

#include "pins-db8500.h"
#include "pins.h"

/* These simply sets bias for pins */
#define BIAS(a, b) static unsigned long a[] = { b }

BIAS(in_pu, PIN_INPUT_PULLUP);
BIAS(in_pd, PIN_INPUT_PULLDOWN);
BIAS(in_nopull, PIN_INPUT_NOPULL);
/* These also force them into GPIO mode */
BIAS(gpio_in_pu, PIN_INPUT_PULLUP|PIN_GPIOMODE_ENABLED);
BIAS(gpio_in_nopull, PIN_INPUT_NOPULL|PIN_GPIOMODE_ENABLED);
BIAS(gpio_in_pd, PIN_INPUT_PULLDOWN|PIN_GPIOMODE_ENABLED);
BIAS(gpio_out_lo, PIN_OUTPUT_LOW|PIN_GPIOMODE_ENABLED);

/* We use these to define hog settings that are always done on boot */
#define DB8540_MUX_HOG(group, func) \
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("pinctrl-db8540", group, func)
#define DB8540_PIN_HOG(pin, conf) \
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("pinctrl-db8540", pin, conf)

/* These are default states associated with device and changed runtime */
#define DB8540_MUX(group, func, dev) \
	PIN_MAP_MUX_GROUP_DEFAULT(dev, "pinctrl-db8540", group, func)
#define DB8540_PIN(pin, conf, dev) \
	PIN_MAP_CONFIGS_PIN_DEFAULT(dev, "pinctrl-db8540", pin, conf)

static struct pinctrl_map __initdata ccu8540_pinmap[] = {
	/* UART0 - GBFN_UART (CG2905) */
	DB8540_MUX_HOG("u0_a_1", "u0"),
	DB8540_PIN_HOG("GPIO0_AH5", in_pu),
	DB8540_PIN_HOG("GPIO1_AG7", in_nopull),
	DB8540_PIN_HOG("GPIO2_AF2", in_pu),
	DB8540_PIN_HOG("GPIO3_AD3", in_nopull),

	/* KEYPAD_EXPENDER_IRQ */
	DB8540_PIN_HOG("GPIO4_AF6", gpio_in_pu),
	/* FLASH_READY */
	DB8540_MUX_HOG("ipgpio6_c_1", "ipgpio"),
	DB8540_PIN_HOG("GPIO5_AG6", in_pu),
	/* LAN_PME */
	DB8540_PIN_HOG("GPIO6_AD5", gpio_in_pu),
	/* PRESS_SENSOR_INT1 */
	DB8540_PIN_HOG("GPIO7_AF7", gpio_in_nopull),
	/* I2C Camera */
	DB8540_MUX_HOG("ipi2c_a_1", "ipi2c"),
	DB8540_PIN_HOG("GPIO8_AG5", in_pu),
	DB8540_PIN_HOG("GPIO9_AH5", in_pu),
	/* GBFN PCM Nluetooth (CG2905) */
	DB8540_MUX_HOG("msp0txrx_a_1", "msp0"),
	DB8540_PIN_HOG("GPIO12_AD2", in_nopull), /* TXD */
	DB8540_PIN_HOG("GPIO15_AC3", in_nopull), /* RXD */
	DB8540_MUX_HOG("msp0tfstck_a_1", "msp0"),
	DB8540_PIN_HOG("GPIO13_AC2", in_nopull), /* TFS */
	DB8540_PIN_HOG("GPIO14_AC4", in_nopull), /* TCK */
	/* Service pin */
	DB8540_PIN_HOG("GPIO22_AF8", gpio_in_pu),
	/*  DB <-> AB Audio Interface */
	DB8540_MUX_HOG("msp1txrx_b_1", "msp1"),
	DB8540_PIN_HOG("GPIO33_AD4", in_nopull), /* RXD */
	DB8540_PIN_HOG("GPIO36_AG4", in_nopull), /* TXD */
	DB8540_MUX_HOG("msp1_a_1", "msp1"),
	DB8540_PIN_HOG("GPIO34_AF3", in_nopull), /* TFS */
	DB8540_PIN_HOG("GPIO35_AF5", in_nopull), /* TCK */
	 /* GPS_MOD_CLKOUT */
	DB8540_MUX_HOG("modobsclk_a_1", "modobs"),
	DB8540_PIN_HOG("GPIO37_AF9", in_nopull),
	DB8540_PIN_HOG("GPIO38_AE8", gpio_in_pu), /* EXT_CHG_INT (PM2130 IRQ) */

	/* STM APE */
	DB8540_MUX_HOG("stmape_c_1", "stmape"),
	DB8540_PIN_HOG("GPIO70_M28", in_nopull), /* clk */
	DB8540_PIN_HOG("GPIO71_N26", in_nopull), /* dat3 */
	DB8540_PIN_HOG("GPIO72_M22", in_nopull), /* dat2 */
	DB8540_PIN_HOG("GPIO73_N22", in_nopull), /* dat1 */
	DB8540_PIN_HOG("GPIO74_N27", in_nopull), /* dat0 */

	/*  STM Modem */
	DB8540_MUX_HOG("stmmod_oc4_1", "stmmod"),
	DB8540_PIN_HOG("GPIO161_AH20", in_nopull), /* clk */
	DB8540_PIN_HOG("GPIO162_AG19", in_nopull), /* dat3 */
	DB8540_PIN_HOG("GPIO163_AF22", in_nopull), /* dat2 */
	DB8540_PIN_HOG("GPIO164_AJ21", in_nopull), /* dat1 */
	DB8540_PIN_HOG("GPIO86_M27", in_nopull), /* dat0 */

	/* Unused GPIOs */
	DB8540_PIN_HOG("GPIO64_M26", gpio_in_pd),
	DB8540_PIN_HOG("GPIO65_M25", gpio_in_pd),
	DB8540_PIN_HOG("GPIO66_M27", gpio_in_pd),
	DB8540_PIN_HOG("GPIO67_N25", gpio_in_pd),
	DB8540_PIN_HOG("GPIO75_N28", gpio_in_pu),
	DB8540_PIN_HOG("GPIO76_P22", gpio_in_pd),
	DB8540_PIN_HOG("GPIO77_P28", gpio_in_pd),
	DB8540_PIN_HOG("GPIO78_P26", gpio_in_pd),
	DB8540_PIN_HOG("GPIO79_T22", gpio_in_pd),
	DB8540_PIN_HOG("GPIO80_R27", gpio_in_pd),
	DB8540_PIN_HOG("GPIO81_P27", gpio_in_pd),
	DB8540_PIN_HOG("GPIO82_R26", gpio_in_pd),
	DB8540_PIN_HOG("GPIO83_R25", gpio_in_pd),
	DB8540_PIN_HOG("GPIO87_T26", gpio_in_pu),
	DB8540_PIN_HOG("GPIO116_AF20", gpio_in_pd), /* SPI touchscreen backup*/
	DB8540_PIN_HOG("GPIO117_AG21", gpio_in_pd), /* SPI touchscreen backup*/
	DB8540_PIN_HOG("GPIO131_AG26", gpio_in_pd),
	DB8540_PIN_HOG("GPIO132_AF25", gpio_in_pd),
	DB8540_PIN_HOG("GPIO133_AE27", gpio_in_pd),
	DB8540_PIN_HOG("GPIO134_AF27", gpio_in_pd),
	DB8540_PIN_HOG("GPIO135_AG28", gpio_in_pd),
	DB8540_PIN_HOG("GPIO136_AF28", gpio_in_pd),
	DB8540_PIN_HOG("GPIO137_AG25", gpio_in_pd),
	DB8540_PIN_HOG("GPIO138_AG24", gpio_in_pd),
	DB8540_PIN_HOG("GPIO153_AG22", gpio_in_pd),
	DB8540_PIN_HOG("GPIO154_AF21", gpio_in_pd),
	DB8540_PIN_HOG("GPIO157_AJ23", gpio_in_pd),
	DB8540_PIN_HOG("GPIO158_AH21", gpio_in_pd),
	DB8540_PIN_HOG("GPIO159_AG20", gpio_in_pd),
	DB8540_PIN_HOG("GPIO160_AE23", gpio_in_pd),

	/*  HSI 2nd Modem - Unused by default */
	DB8540_PIN_HOG("GPIO219_B9", gpio_in_pd),
	DB8540_PIN_HOG("GPIO220_A10", gpio_in_pd),
	DB8540_PIN_HOG("GPIO221_D9", gpio_in_pd),
	DB8540_PIN_HOG("GPIO222_B11", gpio_in_pd),
	DB8540_PIN_HOG("GPIO223_B10", gpio_in_pd),
	DB8540_PIN_HOG("GPIO224_E10", gpio_in_pd),
	DB8540_PIN_HOG("GPIO225_B12", gpio_in_pd),
	DB8540_PIN_HOG("GPIO226_D10", gpio_in_pd),

	/* Modem UART 1 */
	DB8540_MUX_HOG("moduart1txrx_oc4_1", "moduart"),
	DB8540_PIN_HOG("GPIO84_U22", in_pu), /* RXD */
	DB8540_PIN_HOG("GPIO85_T27", in_nopull), /* TXD */

	/* Modem Access UART */
	DB8540_MUX_HOG("modaccuarttxrx_oc4_1", "modaccuart"),
	DB8540_PIN_HOG("GPIO155_AF24", in_pu), /* RXD */
	DB8540_PIN_HOG("GPIO156_AH22", in_nopull), /* TXD */

	/* UART2 : APE console (CP2105) */
	DB8540_MUX_HOG("u2txrx_a_1", "u2"),
	DB8540_PIN_HOG("GPIO120_AG18", in_pu), /* RXD */
	DB8540_PIN_HOG("GPIO121_AH17", in_nopull), /* TXD */

	/* ACCEL_MAGNETOMETER_INT1_DRDY */
	DB8540_PIN_HOG("GPIO124_AE18", gpio_in_pd),
	DB8540_PIN_HOG("GPIO125_AG17", gpio_out_lo), /* DISP0_RST */
	DB8540_PIN_HOG("GPIO126_AF17", gpio_in_pd), /* USBHUB_INT */
	DB8540_PIN_HOG("GPIO127_AE17", gpio_in_pu), /* LAN_IRQ */
	DB8540_PIN_HOG("GPIO128_AC27", gpio_out_lo), /* HDMI_PWRN */
	DB8540_PIN_HOG("GPIO129_AD27", gpio_in_pu), /* HDMI_INT_N */
	DB8540_PIN_HOG("GPIO130_AE28", gpio_in_pu), /* NFC_IRQ */
	DB8540_PIN_HOG("GPIO139_AD25", gpio_in_pd), /* GYROSCOPE_INT */

	DB8540_MUX_HOG("ipgpio7_b_1", "ipgpio"),
	DB8540_PIN_HOG("GPIO140_AH25", in_nopull), /* FLASH_LED_SYNC */

	DB8540_PIN_HOG("GPIO143_AG23", gpio_in_pd), /* ACCEL_MANETOMETER_INT2 */
	DB8540_PIN_HOG("GPIO144_AE25", gpio_in_pd), /* GYROSCOPE_DRDY_INT2 */
	DB8540_PIN_HOG("GPIO145_AH24", gpio_out_lo), /* TOUCH_RST */
	DB8540_PIN_HOG("GPIO146_AJ25", gpio_in_pu), /* TOUCH_INT */

	DB8540_MUX_HOG("lcdvsi0_a_1_pins", "lcd"),
	DB8540_PIN_HOG("GPIO151_AJ24", in_pd), /* LCD Tearing effect */

	/* FAT Modem audio IF */
	DB8540_MUX_HOG("modi2s_a_1", "modi2s"),
	DB8540_PIN_HOG("GPIO165_AD26", in_nopull), /* TCK */
	DB8540_PIN_HOG("GPIO166_AD28", in_nopull), /* TFS */
	DB8540_PIN_HOG("GPIO167_AC28", in_nopull), /* TXD */
	DB8540_PIN_HOG("GPIO168_AC26", in_nopull), /* RXD */

	/* 2nd Modem FAT audio */
	DB8540_MUX_HOG("msp4_a_1_pins", "msp4"),
	DB8540_PIN_HOG("GPIO231_B14", in_nopull), /* TCK */
	DB8540_PIN_HOG("GPIO232_E11", in_nopull), /* TXD */
	DB8540_MUX_HOG("msp4_b_1_pins", "msp4"),
	DB8540_PIN_HOG("GPIO196_H3", in_nopull), /* RXD */
	DB8540_MUX_HOG("msp4_c_1_pins", "msp4"),
	DB8540_PIN_HOG("GPIO192_J3", in_nopull), /* TFS */

	/* HDMI Audio */
	DB8540_MUX_HOG("msp2txdtcktfs_a_1", "msp2"),
	DB8540_PIN_HOG("GPIO193_H1", in_nopull), /* TXD */
	DB8540_PIN_HOG("GPIO194_J2", in_nopull), /* TCK */
	DB8540_PIN_HOG("GPIO195_H2", in_nopull), /* TFS */

	/* CLK CAM */
	DB8540_MUX_HOG("clkout_a_1", "clkout"),
	DB8540_PIN_HOG("GPIO227_D11", in_nopull), /* CLK CAM0 */
	DB8540_PIN_HOG("GPIO228_AJ6", in_nopull), /* CLK CAM1 */

	/* CAM0 */
	DB8540_MUX_HOG("ipgpio2_b_1", "ipgpio"),
	DB8540_PIN_HOG("GPIO141_AF26", in_nopull), /* RES */
	DB8540_MUX_HOG("ipgpio0_a_1_pins", "ipgpio"),
	DB8540_PIN_HOG("GPIO149_AE26", in_nopull), /* SD */

	/* CAM1 */
	DB8540_MUX_HOG("ipgpio3_b_1", "ipgpio"),
	DB8540_PIN_HOG("GPIO142_P27", in_nopull), /* CAM1_RES */
	DB8540_MUX_HOG("ipgpio1_a_1", "ipgpio"),
	DB8540_PIN_HOG("GPIO150_AE24", in_nopull), /* CAM1_SD */

	DB8540_PIN_HOG("GPIO230_C12", gpio_in_pu), /* SDMMC_CD */

	/* Mux in I2C blocks */
	/* I2C0 : HDMI, External StepUp Audio 4.5V, IO Expander, KP Expander */
	/* default state */
	DB8540_MUX("i2c0_a_1", "i2c0", "nmk-i2c.0"),
	DB8540_PIN("GPIO147_AG27", in_pu, "nmk-i2c.0"), /* SCL */
	DB8540_PIN("GPIO148_AH23", in_pu, "nmk-i2c.0"), /* SDA */

	/* I2C1 : Battery charging; GPIO Expander, Keypad Expander */
	/* default state */
	DB8540_MUX("i2c1_b_2", "i2c1", "nmk-i2c.1"),
	DB8540_PIN("GPIO16_AH7", in_pu, "nmk-i2c.1"), /* SCL */
	DB8540_PIN("GPIO17_AE7", in_pu, "nmk-i2c.1"), /* SDA */

	/* I2C2 : UIB */
	/* default state */
	DB8540_MUX("i2c2_b_2", "i2c2", "nmk-i2c.2"),
	DB8540_PIN("GPIO10_AE4", in_pu, "nmk-i2c.2"), /* SCL */
	DB8540_PIN("GPIO11_AD1", in_pu, "nmk-i2c.2"), /* SDA */

	/* I2C4 : Misc devices */
	/* default state */
	DB8540_MUX("i2c4_b_2", "i2c4", "nmk-i2c.4"),
	DB8540_PIN("GPIO122_AF19", in_pu, "nmk-i2c.4"), /* SCL */
	DB8540_PIN("GPIO123_AF18", in_pu, "nmk-i2c.4"), /* SDA */

	/* I2C5 : Touchscreen */
	/* default state */
	DB8540_MUX("i2c5_c_2", "i2c5", "nmk-i2c.5"),
	DB8540_PIN("GPIO118_AH19", in_pu, "nmk-i2c.5"), /* SCL */
	DB8540_PIN("GPIO119_AE19", in_pu, "nmk-i2c.5"), /* SDA */

	DB8540_MUX("lcdvsi1_a_1", "lcd", "0-0070"),
	DB8540_PIN("GPIO152_AE21", in_pd, "0-0070"),
};

/* SDI0 : Removable MMC/SD/SDIO cards */
static UX500_PINS(ccu8540_pins_sdi0,
	GPIO23_MC0_CLK		| PIN_PULL_NONE,
	GPIO24_MC0_CMD		| PIN_PULL_NONE,
	GPIO25_MC0_DAT0		| PIN_PULL_NONE,
	GPIO26_MC0_DAT1		| PIN_PULL_NONE,
	GPIO27_MC0_DAT2		| PIN_PULL_NONE,
	GPIO28_MC0_DAT3		| PIN_PULL_NONE,
);

/* SDI1 : SDIO (WLAN CW1200) */
static UX500_PINS(ccu8540_pins_sdi1,
	GPIO208_MC1_CLK		| PIN_PULL_DOWN,
	GPIO209_GPIO		| PIN_INPUT_PULLUP,
	GPIO210_MC1_CMD		| PIN_PULL_UP,
	GPIO211_MC1_DAT0	| PIN_PULL_UP,
	GPIO212_MC1_DAT1	| PIN_PULL_UP,
	GPIO213_MC1_DAT2	| PIN_PULL_UP,
	GPIO214_MC1_DAT3	| PIN_PULL_UP,
);

/* SDI4 : PCB eMMC */
static UX500_PINS(ccu8540_pins_sdi4,
	GPIO197_MC4_DAT3	| PIN_PULL_UP,
	GPIO198_MC4_DAT2	| PIN_PULL_UP,
	GPIO199_MC4_DAT1	| PIN_PULL_UP,
	GPIO200_MC4_DAT0	| PIN_PULL_UP,
	GPIO201_MC4_CMD		| PIN_PULL_UP,
	GPIO202_MC4_RSTN	| PIN_PULL_UP,
	GPIO203_MC4_CLK		| PIN_PULL_DOWN,
	GPIO204_MC4_DAT7	| PIN_PULL_UP,
	GPIO205_MC4_DAT6	| PIN_PULL_UP,
	GPIO206_MC4_DAT5	| PIN_PULL_UP,
	GPIO207_MC4_DAT4	| PIN_PULL_UP,
);

/* USB */
static UX500_PINS(ccu8540_pins_usb,
	GPIO256_USB_NXT		| PIN_PULL_NONE,
	GPIO257_USB_STP		| PIN_PULL_NONE,
	GPIO258_USB_XCLK	| PIN_PULL_NONE,
	GPIO259_USB_DIR		| PIN_PULL_NONE,
	GPIO260_USB_DAT7	| PIN_PULL_NONE,
	GPIO261_USB_DAT6	| PIN_PULL_NONE,
	GPIO262_USB_DAT5	| PIN_PULL_NONE,
	GPIO263_USB_DAT4	| PIN_PULL_NONE,
	GPIO264_USB_DAT3	| PIN_PULL_NONE,
	GPIO265_USB_DAT2	| PIN_PULL_NONE,
	GPIO266_USB_DAT1	| PIN_PULL_NONE,
	GPIO267_USB_DAT0	| PIN_PULL_NONE,
);

static UX500_PINS(ccu8540_pins_sensors1p,
	GPIO229_GPIO		| PIN_INPUT_PULLUP,
);

static struct ux500_pin_lookup ccu8540_pins_lookup[] = {
	PIN_LOOKUP("sdi0", &ccu8540_pins_sdi0),
	PIN_LOOKUP("sdi1", &ccu8540_pins_sdi1),
	PIN_LOOKUP("sdi4", &ccu8540_pins_sdi4),
	PIN_LOOKUP("musb-ux500.0", &ccu8540_pins_usb),
	PIN_LOOKUP("gpio-keys.0", &ccu8540_pins_sensors1p),
};

void __init ccu8540_pinmaps_init(void)
{
	pinctrl_register_mappings(ccu8540_pinmap, ARRAY_SIZE(ccu8540_pinmap));
	ux500_pins_add(ccu8540_pins_lookup, ARRAY_SIZE(ccu8540_pins_lookup));
}
