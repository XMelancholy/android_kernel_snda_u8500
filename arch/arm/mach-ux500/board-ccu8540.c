/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Jean-Nicolas Graux <jean-nicolas.graux@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/input/abx500-accdet.h>
#include <linux/gpio.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/amba/pl022.h>
#include <linux/amba/serial.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/pm2301_charger.h>
#include <linux/mfd/stmpe.h>
#include <linux/mfd/abx500/ab8500-gpio.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/input/matrix_keypad.h>
#include <linux/platform_data/crypto-ux500.h>
#include <linux/ux500_modem_control.h>

#include <plat/pincfg.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <plat/gpio-nomadik.h>

#include <mach/ste-dma40-db8500.h>
#include <mach/devices.h>
#include <mach/setup.h>
#include <mach/reboot_reasons.h>

#include "devices-db8500.h"
#include "cpu-db8500.h"

#include "board-ccu8540-regulators.h"
#include "board-mop500-bm.h"
#include "board-common-uib.h"
#include "board-ccu8540-pins.h"
#include "board-ccu8540-sdi.h"
#include "board-ccux540-ver.h"

#ifdef CONFIG_INPUT_AB8500_ACCDET
static struct abx500_accdet_platform_data ab8540_accdet_pdata = {
	.btn_keycode = KEY_MEDIA,
	.accdet1_dbth = ACCDET1_TH_1200mV | ACCDET1_DB_70ms,
	.accdet2122_th = ACCDET21_TH_1000mV | ACCDET22_TH_1000mV,
	.video_ctrl_gpio = AB8500_PIN_GPIO(54),
	.is_detection_inverted = true,
	.video_ctrl_gpio_inverted = true,
};
#endif

static struct ccux540_version_pdata ccu8540_version_pdata = {
	.family = CCUX540_VER_L8540_FAMILY,
	.rev_pin0 = MOP500_EGPIO(12),
	.rev_pin1 = MOP500_EGPIO(13),
	.rev_pin2 = MOP500_EGPIO(14),
	.rev_pin3 = MOP500_EGPIO(15),
};

static struct platform_device ccu8540_version_dev = {
	.name           = "modem-hwcfg",
	.id		= -1,
	.dev            = {
		.platform_data = &ccu8540_version_pdata,
	},
};

static struct cryp_platform_data u8540_cryp1_platform_data = {
	.mem_to_engine = {
		.dir = STEDMA40_MEM_TO_PERIPH,
		.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
		.dst_dev_type = DB8500_DMA_DEV48_CAC1_TX,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	},
	.engine_to_mem = {
		.dir = STEDMA40_PERIPH_TO_MEM,
		.src_dev_type = DB8500_DMA_DEV48_CAC1_RX,
		.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	}
};

static struct stedma40_chan_cfg u8540_hash_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV50_HAC1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.mode = STEDMA40_MODE_LOGICAL,
	.src_info.psize = STEDMA40_PSIZE_LOG_16,
	.dst_info.psize = STEDMA40_PSIZE_LOG_16,
};

static struct hash_platform_data u8540_hash1_platform_data = {
	.mem_to_engine = &u8540_hash_dma_cfg_tx,
	.dma_filter = stedma40_filter,
};

static void __init u8540_cryp1_hash1_init(struct device *parent)
{
	db8500_add_cryp1(parent, &u8540_cryp1_platform_data);
	db8500_add_hash1(parent, &u8540_hash1_platform_data);
}

static struct platform_device *u8540_platform_devs[] __initdata = {
};

static void __init u8540_i2c_init(struct device *parent)
{
	db8500_add_i2c0(parent, NULL);
	db8500_add_i2c1(parent, NULL);
	db8500_add_i2c2(parent, NULL);
	db8500_add_i2c4(parent, NULL);
	db8540_add_i2c5(parent, NULL);
}

static void __init u8540_uart_init(struct device *parent)
{
	db8500_add_uart0(parent, NULL);
	db8500_add_uart1(parent, NULL);
	db8500_add_uart2(parent, NULL);
}

/* The AB8540 GPIO support is based on AB9540 but also has additional GPIO. */
static struct ab8500_gpio_platform_data ab8540_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO(1),
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/*
	 * initial_pin_config is the initial configuration of ab9540 pins.
	 * The pins can be configured as GPIO or alt functions based
	 * on value present in GpioSel1 to GpioSel7 and AlternatFunction
	 * register. This is the array of 8 configuration settings.
	 * These settings are decided at compile time, and are explained below.
	 * GpioSel1 = 0x04 => Pin SysClkReq2
	 *                    Pin SysClkReq3
	 *                    Pin GPIO3
	 *                    Pin SysClkReq6
	 * GpioSel2 = 0xD0 => Pin Alternate function
	 *                    Pin PDMCLK
	 *                    Pin GPIO13
	 *                    Pin PWMOut1
	 *                    Pin GPIO15
	 *                    Pin GPIO16
	 * GpioSel3 = 0x70 => Pin AD_Data2
	 *                    Pin DA_Data2
	 *                    Pin Fsync2
	 *                    Pin BitClk2
	 *                    Pin GPIO21
	 *                    Pin GPIO22
	 *                    Pin GPIO23
	 *                    Pin SysClkReq7
	 * GpioSel4 = 0x00 => Pin SysClkReq8
	 *                    Pin Dmic12Clk
	 *                    Pin Dmic12Dat
	 *                    Pin Dmic34Clk
	 *                    Pin Dmic34Dat
	 *                    Pin Dmic56Clk
	 *                    Pin Dmic56Dat
	 * GpioSel5 = 0x82 => Pin GPIO34
	 *                    Pin GPIO40
	 * GpioSel6 = 0x01 => Pin GPIO41
	 *                    Pin SysClkReq5
	 * GpioSel7 = 0x3A => Pin GPIO50
	 *                    Pin BATTREM
	 *                    Pin GPIO52
	 *                    Pin GPIO53
	 *                    Pin GPIO60
	 *
	 * GpioSel2 bit 1 is set as alternate function.
	 *     Alternate Function: PDMDAT on pad GPIO10 = 0x00
	 */
	.config_reg = {0x04, 0xD0, 0x70, 0x00, 0x82, 0x01, 0x3A, 0x20},

	/*
	 * initial_pin_direction allows for the initial GPIO direction to
	 * be set.
	 * GpioDir7 = 0x24 ==> change to 0x26,
	 * change gpio50 driect to output ; gpio60 is output;
	 */
	.config_direction = {0x06, 0xF6, 0x72, 0x54, 0x00, 0x02, 0x26},

	/*
	 * initial_pin_pullups allows for the initial configuration of the
	 * GPIO pullup/pulldown configuration.
	 * GpioPud7 = 0xE5 ==> change to 0xE7,
	 * gpio50 pulldown is disabed, gpio60 pulldown is disabled
	 */
	.config_pullups = {0xFF, 0xFF, 0x7F, 0xFE, 0x7D, 0xFE, 0xE7},
	/*
	 * initial_pin_vinsel allows for the voltage selection.
	 * GpioVinSel = 0x55 ==> GPIO1_VBAT(GPIO51) to GPIO4_VBAT(GPIO54)
	 * select VIO voltage (1.8V)
	 */
	.config_vinsel = 0x55,
	/*
	 * initial_pin_pullupdown allows for the initial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	.config_pullupdown = 0x00,
};

static struct ab8500_sysctrl_platform_data ab8540_sysctrl_pdata = {
	/*
	 * SysClkReq1RfClkBuf - SysClkReq8RfClkBuf
	 * The initial values should not be changed because of the way
	 * the system works today
	 */
	.initial_req_buf_config
			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	.reboot_reason_code = reboot_reason_code,
};

static struct stmpe_gpio_platform_data u8540_stmpe1_gpio_data = {
	/* pins MOP500_EGPIO(8) => MOP500_EGPIO(15) */
	.gpio_base = MOP500_EGPIO(8),
};

static struct stmpe_platform_data u8540_stmpe1_platform_data = {
	.id = 0,
	.blocks = STMPE_BLOCK_GPIO,
	.gpio = &u8540_stmpe1_gpio_data,
};

static struct stmpe_gpio_platform_data u8540_stmpe2_gpio_data = {
	/* pins MOP500_EGPIO(24) => MOP500_EGPIO(41) */
	.gpio_base = MOP500_EGPIO(24),
	/* GPIO[0-2] & GPIO[8-9] reserved for keypad */
	.norequest_mask = 0x707,
};

static const uint32_t u8540_stmpe1801_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_F1),
	KEY(0, 2, KEY_F2),

	KEY(1, 0, KEY_VOLUMEDOWN),
	KEY(1, 1, KEY_F3),
	KEY(1, 2, KEY_F4),

	/*KEY(2, 0, xxx),*/
	KEY(2, 1, KEY_CAMERA_FOCUS),
	KEY(2, 2, KEY_CAMERA),
};

static const struct matrix_keymap_data u8540_stmpe1801_matrix_keymap_data = {
	.keymap = u8540_stmpe1801_keymap,
	.keymap_size = ARRAY_SIZE(u8540_stmpe1801_keymap),
};

static struct stmpe_keypad_platform_data u8540_stmpe2_keypad_data = {
	.keymap_data = &u8540_stmpe1801_matrix_keymap_data,
	.debounce_ms = 64,
	.scan_count = 8,
	.no_autorepeat = true,
};

static struct stmpe_platform_data u8540_stmpe2_platform_data = {
	.id = 2,
	.blocks = STMPE_BLOCK_GPIO|STMPE_BLOCK_KEYPAD,
	.gpio = &u8540_stmpe2_gpio_data,
	.keypad = &u8540_stmpe2_keypad_data,
	.irq_trigger    = IRQF_TRIGGER_FALLING,
	.irq_base       = MOP500_STMPE1801_IRQ(0),
};

static struct modctrl_reg_pwr ab8540_modctrl_reg_pwr = {
	.reg_pwr_bank = AB8500_REGU_CTRL2,
	.reg_pwr_addr = AB9540_MODEM_CTRL2_REG,
	.reg_pwr_mask = AB8540_MODEM_CTRL2_ONSWN_BIT,
};

static struct abx540_modctrl_platform_data ab8540_modctrl_plat_data = {
	.reg_power  = &ab8540_modctrl_reg_pwr,
	.gpio_power = -1,
};

static struct ab8500_platform_data ab8540_platdata = {
	.irq_base	= MOP500_AB8500_IRQ_BASE,
	.regulator	= &ab8540_regulator_plat_data,
	.battery	= &ab8540_bm_data,
	.charger	= &ab8540_charger_plat_data,
	.btemp		= &ab8500_btemp_plat_data,
	.fg		= &ab8500_fg_plat_data,
	.chargalg	= &ab8500_chargalg_plat_data,
	.gpio		= &ab8540_gpio_pdata,
	.sysctrl	= &ab8540_sysctrl_pdata,
	.pwmled		= &ab8500_pwmled_plat_data,
#ifdef CONFIG_INPUT_AB8500_ACCDET
	.accdet = &ab8540_accdet_pdata,
#endif
#ifdef CONFIG_PM
	.pm_power_off = true,
#endif
	.thermal_time_out = 20, /* seconds */
#ifdef CONFIG_INPUT_AB8505_MICRO_USB_DETECT
	.iddet = &iddet_adc_val_list,
#endif
	.modctrl	= &ab8540_modctrl_plat_data,
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start	= IRQ_DB8500_AB8500,
		.end	= IRQ_DB8500_AB8500,
		.flags	= IORESOURCE_IRQ
	}
};
struct platform_device ab8540_device = {
	.name = "ab8540-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8540_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

static struct i2c_board_info __initdata u8540_i2c0_devices[] = {

};

static struct pm2xxx_platform_data ccu8540_pm2xxx_platdata = {
	.wall_charger	= &ccu8540_pm2xxx_charger_plat_data,
	.battery	= &pm2xxx_bm_data,
};

static struct i2c_board_info __initdata u8540_i2c1_devices[] = {
	{
		/* stmpe801 GPIO expander: Core and UIB ID */
		I2C_BOARD_INFO("stmpe801", 0x41),
		.platform_data = &u8540_stmpe1_platform_data,
		.irq = -1,
	},
	{
		/* stmpe1801 GPIO/KEYPAD expander */
		I2C_BOARD_INFO("stmpe1801", 0x40),
		.platform_data = &u8540_stmpe2_platform_data,
		.irq = NOMADIK_GPIO_TO_IRQ(4),
	},
	{
		I2C_BOARD_INFO("pm2301", 0x2C),
		.platform_data = &ccu8540_pm2xxx_platdata,
	},
};

static struct i2c_board_info __initdata u8540_i2c2_devices[] = {
	/* To Be added: I2C2 : UIB */
};

static struct i2c_board_info __initdata u8540_i2c4_devices[] = {
	/* To Be added: I2C4 : Misc devices */
};

static struct i2c_board_info __initdata u8540_i2c5_devices[] = {
	/* To Be added: I2C5 : Touchscreen */
};

static struct modem_control_platform_data db8540_modem_control_data = {
	.version		= MODCTRL_VERSION_AP8540,
};

struct platform_device db8540_modem_control_device = {
	.name	= "modem_control",
	.id	= -1,
	.dev	= {
		.platform_data = &db8540_modem_control_data,
	},
};

static void __init ccu8540_init_irq(void)
{
	prcmu_early_init(&db8540_tcdm_map, true);
	ux500_init_irq();
}

static void __init u8540_init_machine(void)
{
	struct device *parent;
	int i;

	parent = u8500_init_devices();

	ccu8540_pinmaps_init();
	dbx500_add_pinctrl(parent, "pinctrl-db8540");

	platform_device_register(&dbx540_prcmu_device);
	ccu8540_sdi_init(parent);

	for (i = 0; i < ARRAY_SIZE(u8540_platform_devs); i++)
		u8540_platform_devs[i]->dev.parent = parent;

	u8540_cryp1_hash1_init(parent);
	u8540_uart_init(parent);
	u8540_i2c_init(parent);
	platform_device_register(&ab8540_device);
	platform_device_register(&db8540_modem_control_device);

	i2c_register_board_info(0, u8540_i2c0_devices,
		ARRAY_SIZE(u8540_i2c0_devices));

	i2c_register_board_info(1, u8540_i2c1_devices,
		ARRAY_SIZE(u8540_i2c1_devices));

	platform_device_register(&ccu8540_version_dev);

	i2c_register_board_info(2, u8540_i2c2_devices,
		ARRAY_SIZE(u8540_i2c2_devices));

	i2c_register_board_info(4, u8540_i2c4_devices,
		ARRAY_SIZE(u8540_i2c4_devices));

	i2c_register_board_info(5, u8540_i2c5_devices,
		ARRAY_SIZE(u8540_i2c5_devices));

	platform_device_register(&db8540_xmip_device);
}

MACHINE_START(U8540, "ST-Ericsson 8540 platform")
	.atag_offset	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= ccu8540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= u8540_init_machine,
	.restart	= ux500_restart,
MACHINE_END

#ifdef CONFIG_MACH_UX500_DT

struct of_dev_auxdata u8540_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("arm,pl011", 0x80120000, "uart0", NULL),
	OF_DEV_AUXDATA("arm,pl011", 0x80121000, "uart1", NULL),
	OF_DEV_AUXDATA("arm,pl011", 0x80007000, "uart2", NULL),
	{},
};

static const struct of_device_id u8540_soc_node[] = {
	/* only create devices below soc node */
	{ .compatible = "stericsson,db8500", },
	{ },
};

static void __init u8540_dt_init_machine(void)
{
	struct device *parent;
	int i;

	parent = u8500_init_devices();

	ccu8540_pinmaps_init();
	dbx500_add_pinctrl(parent, "pinctrl-db8540");

	platform_device_register(&dbx540_prcmu_device);
	ccu8540_sdi_init(parent);

	for (i = 0; i < ARRAY_SIZE(u8540_platform_devs); i++)
		u8540_platform_devs[i]->dev.parent = parent;

	/* automatically probe child nodes of db8500 device */
	of_platform_populate(NULL, u8540_soc_node,
			u8540_auxdata_lookup, parent);

	u8540_cryp1_hash1_init(parent);
	u8540_i2c_init(parent);
	platform_device_register(&ab8540_device);
	platform_device_register(&db8540_modem_control_device);

	i2c_register_board_info(0, u8540_i2c0_devices,
		ARRAY_SIZE(u8540_i2c0_devices));

	i2c_register_board_info(1, u8540_i2c1_devices,
		ARRAY_SIZE(u8540_i2c1_devices));

	platform_device_register(&db8540_xmip_device);

	platform_device_register(&ccu8540_version_dev);

	i2c_register_board_info(2, u8540_i2c2_devices,
		ARRAY_SIZE(u8540_i2c2_devices));

	i2c_register_board_info(4, u8540_i2c4_devices,
		ARRAY_SIZE(u8540_i2c4_devices));

	i2c_register_board_info(5, u8540_i2c5_devices,
		ARRAY_SIZE(u8540_i2c5_devices));
}

static const char *u8540_dt_board_compat[] = {
	"st-ericsson,u8540",
	NULL,
};

DT_MACHINE_START(U8540_DT, "ST-Ericsson 8540 platform (Device Tree Support)")
	.atag_offset	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= ccu8540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= u8540_dt_init_machine,
	.dt_compat	= u8540_dt_board_compat,
MACHINE_END
#endif
