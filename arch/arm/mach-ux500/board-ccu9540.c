/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Alexandre Torgue <alexandre.torgue@stericsson.com> for ST-Ericsson
 *
 * License terms: GNU General Public License (GPL), version 2
 *
 * specific initialization U9540 CCU boards.
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/input/abx500-accdet.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/hsi/hsi.h>
#include <linux/leds_pwm.h>
#include <linux/leds.h>
#include <linux/smsc911x.h>
#include <linux/amba/pl022.h>
#include <linux/spi/stm_msp.h>
#include <linux/mfd/abx500/ux500_sysctrl.h>
#include <linux/pm2301_charger.h>
#include <linux/mfd/stmpe.h>
#include <linux/mfd/abx500/ab8500-gpio.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/input/ab8505_micro_usb_iddet.h>
#include <linux/amba/serial.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/tee.h>
#include <linux/lsm303dlh.h>
#include <linux/input/lps331ap.h>
#include <linux/platform_data/i2c-nomadik.h>
#include <linux/platform_data/crypto-ux500.h>
#include <linux/platform_data/keypad-nomadik-ske.h>
#include <linux/mfd/abx500/ab9540-modctrl.h>
#include <linux/mfd/abx500/ab8500.h>

#include <video/av8100.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>

#include <mach/ste-dma40-db8500.h>
#include <mach/devices.h>
#include <mach/setup.h>
#include <mach/reboot_reasons.h>

#include "pins-db8500.h"
#include "devices-db8500.h"
#include "cpu-db8500.h"
#include "pm/cpuidle.h"
#include "board-ccu9540-regulators.h"
#include "board-mop500-bm.h"
#include "board-ccu9540.h"
#include "board-mop500-wlan.h"
#include "pins.h"
#include "board-mop500.h"
#include "board-ccux540-ver.h"

#ifdef CONFIG_INPUT_AB8500_ACCDET
static struct abx500_accdet_platform_data ab8500_accdet_pdata = {
	.btn_keycode = KEY_MEDIA,
	.accdet1_dbth = ACCDET1_TH_1200mV | ACCDET1_DB_70ms,
	.accdet2122_th = ACCDET21_TH_1000mV | ACCDET22_TH_1000mV,
	.video_ctrl_gpio = AB8500_PIN_GPIO(35),
};
#endif

void accessory_detect_config(int ccu_id)
{
#ifdef CONFIG_INPUT_AB8500_ACCDET
	ab8500_accdet_pdata.is_detection_inverted = true;

	ab8500_accdet_pdata.video_ctrl_gpio = AB8500_PIN_GPIO(13);
	ab8500_accdet_pdata.mic_ctrl = AB8500_PIN_GPIO(15);
	if (ccu_id < CCU9540_C2C_US_V1_1) {
		/* AB GPIO 60 located at offset 54 */
		ab8500_accdet_pdata.nahj_ctrl = AB8500_PIN_GPIO(GPIO_60);
	} else {
		/* Newer boards use GPIO 50 for nahj_ctrl */
		ab8500_accdet_pdata.nahj_ctrl = AB8500_PIN_GPIO(GPIO_50);
	}
	ab8500_accdet_pdata.video_ctrl_gpio_inverted = true;
#endif
}

static struct ccux540_version_pdata ccu9540_version_pdata = {
	.family = CCUX540_VER_U9540_FAMILY,
	.rev_pin0 = MOP500_EGPIO(12),
	.rev_pin1 = MOP500_EGPIO(13),
	.rev_pin2 = MOP500_EGPIO(14),
	.rev_pin3 = MOP500_EGPIO(15),
};

static struct platform_device ccu9540_version_dev = {
	.name           = "modem-hwcfg",
	.id		= -1,
	.dev            = {
		.platform_data = &ccu9540_version_pdata,
	},
};

static struct cryp_platform_data u8500_cryp1_platform_data = {
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

static struct stedma40_chan_cfg u8500_hash_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV50_HAC1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.mode = STEDMA40_MODE_LOGICAL,
	.src_info.psize = STEDMA40_PSIZE_LOG_16,
	.dst_info.psize = STEDMA40_PSIZE_LOG_16,
};

static struct hash_platform_data u8500_hash1_platform_data = {
	.mem_to_engine = &u8500_hash_dma_cfg_tx,
	.dma_filter = stedma40_filter,
};

static void __init u8500_cryp1_hash1_init(struct device *parent)
{
	db8500_add_cryp1(parent, &u8500_cryp1_platform_data);
	db8500_add_hash1(parent, &u8500_hash1_platform_data);
}

static struct tee_platform_data ccu9540_tee_platform_data = {
	.api_version = TEE_VERSION_100,
	.bridge_func_addr =
		(u32)IO_ADDRESS_DB9540_ROM(U9540_BOOT_ROM_BASE + 0x17300)
};

static void __init ccu9540_tee_init(void)
{
	db8500_add_tee(&ccu9540_tee_platform_data);
}

#ifdef CONFIG_HSI
static struct hsi_board_info __initdata u8500_hsi_devices[] = {
	{
		.name = "hsi_char",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 1,
			.speed = 200000,
			{.arb_mode = HSI_ARB_RR},
			.dma_burst = 1,
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 1,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
			.dma_burst = 1,
		},
	},
	{
		.name = "hsi_test",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 100000,
			{.arb_mode = HSI_ARB_RR},
			.dma_burst = 1,
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
			.dma_burst = 1,
		},
	},
	{
		.name = "cfhsi_v3_driver",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 200000,
			{.arb_mode = HSI_ARB_RR},
			.dma_burst = 8,
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_PIPE},
			.dma_burst = 8,
		},
	},
};
#endif

/* Force feedback vibrator device */
static struct platform_device ste_ff_vibra_device = {
	.name = "ste_ff_vibra"
};

#ifdef CONFIG_U9540_MLOADER
static struct platform_device u9540_mloader_device = {
	.name = "u9540-mloader",
	.id = -1,
	.num_resources = 0,
};
#endif

static struct smsc911x_platform_config u9540_ethernet_config = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags = SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.shift = 1,
};

static struct resource u9540_ethernet_res[] = {
	{
		.name = "smsc911x-memory",
		.start = (0x5000 << 16),
		.end  =  (0x5000 << 16) + 0xffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = NOMADIK_GPIO_TO_IRQ(7),
		.end = NOMADIK_GPIO_TO_IRQ(7),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device u9540_ethernet_dev = {
	.name           = "smsc911x",
	.num_resources  = ARRAY_SIZE(u9540_ethernet_res),
	.resource       = u9540_ethernet_res,
	.dev            = {
		.platform_data = &u9540_ethernet_config,
	},
};

#ifdef CONFIG_STE_HVA
#define HVA_IO_AREA "HVA I/O REGS"
#define HVA_IRQ_ITS "HVA IRQ ITS"
#define HVA_IRQ_ERR "HVA IRQ ERR"
#define HVA_IO_SIZE (24*4)
static struct resource hva_resources[] = {
	[0] = {
		.name  = HVA_IO_AREA,
		.start = U9540_HVA_BASE,
		.end   = U9540_HVA_BASE + HVA_IO_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = HVA_IRQ_ITS,
		.start = IRQ_AP9540_HVA_ITS,
		.end   = IRQ_AP9540_HVA_ITS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name  = HVA_IRQ_ERR,
		.start = IRQ_AP9540_HVA_ERR,
		.end   = IRQ_AP9540_HVA_ERR,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device u9540_hva_device = {
	.name = "hva",
	.id = -1,
	.num_resources = ARRAY_SIZE(hva_resources),
	.resource = hva_resources,
};
#endif

#ifdef CONFIG_HX170_DEC
#define HX170_IO_AREA "HX170 I/O REGS"
#define HX170_IRQ "HX170 IRQ"
#define HX170_IO_SIZE (101*4)

static struct resource hx170_resources[] = {
	[0] = {
		.name  = HX170_IO_AREA,
		.start = U9540_G1_BASE,
		.end   = U9540_G1_BASE + HX170_IO_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = HX170_IRQ,
		.start = IRQ_AP9540_C2C_G1,
		.end   = IRQ_AP9540_C2C_G1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device u9540_hx170_device = {
	.name = "g1",
	.id = -1,
	.num_resources = ARRAY_SIZE(hx170_resources),
	.resource = hx170_resources,
};
#endif

#ifdef CONFIG_UX500_ROMCODE_SHARED_MUTEX
static struct platform_device db9540_romcode_sm_device = {
	.name = "romcode-sm",
	.id = -1,
};
#endif

static struct platform_device *u9540_platform_devs[] __initdata = {
	&ste_ff_vibra_device,
#ifdef CONFIG_U8500_MMIO
	&ux500_mmio_device,
	&ux500_mmio_raw_device,
	&ux500_mmio_yuv_device,
#endif
	&ux500_hwmem_device,
	&ux500_mcde_device,
#ifdef CONFIG_MCDE_DISPLAY_DSI
	&u8500_dsilink_device[0],
	&u8500_dsilink_device[1],
	&u8500_dsilink_device[2],
#endif
	&ux500_b2r2_device,
	&ux500_b2r2_1_device,
	&ux500_b2r2_blt_device,
#ifdef CONFIG_U9540_MLOADER
	&u9540_mloader_device,
#endif
	&u9540_ethernet_dev,
#ifdef CONFIG_STE_HVA
	&u9540_hva_device,
#endif
#ifdef CONFIG_HX170_DEC
	&u9540_hx170_device,
#endif
#ifdef CONFIG_UX500_ROMCODE_SHARED_MUTEX
	&db9540_romcode_sm_device,
#endif
};

static void __init ccu9540_i2c_init(struct device *parent)
{
	db8500_add_i2c0(parent, NULL);
	db8500_add_i2c1(parent, NULL);
	db8500_add_i2c2(parent, NULL);
}

/*
 * SSP
 */

#define NUM_SSP_CLIENTS 10

#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ssp0_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV8_SSP0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};

static struct stedma40_chan_cfg ssp0_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV8_SSP0_TX,
	.src_info.data_width = STEDMA40_BYTE_WIDTH,
	.dst_info.data_width = STEDMA40_BYTE_WIDTH,
};
#endif

static struct pl022_ssp_controller ssp0_platform_data = {
	.bus_id = 4,
#ifdef CONFIG_STE_DMA40
	.enable_dma = 1,
	.dma_filter = stedma40_filter,
	.dma_rx_param = &ssp0_dma_cfg_rx,
	.dma_tx_param = &ssp0_dma_cfg_tx,
#endif
	/* on this platform, gpio 31,142,144,214 &
	 * 224 are connected as chip selects
	 */
	.num_chipselect = NUM_SSP_CLIENTS,
};

/*
 * MSP-SPI
 */

#define NUM_MSP_CLIENTS 10

static struct stm_msp_controller ccu9540_msp2_spi_data = {
	.id		= 2,
	.num_chipselect	= NUM_MSP_CLIENTS,
	.base_addr	= U8500_MSP2_BASE,
	.device_name	= "msp2",
};

static void __init ccu9540_spi_init(struct device *parent)
{
	db8500_add_ssp0(parent, &ssp0_platform_data);
	db8500_add_msp2_spi(parent, &ccu9540_msp2_spi_data);
}

static pin_cfg_t ccu9540_pins_cam_en[] = {
	MOP500_EGPIO(3) | PIN_OUTPUT_LOW,
	MOP500_EGPIO(4) | PIN_OUTPUT_LOW
};

static void __init ccu9540_cam_en_init(void)
{
	int ret;

	ret = nmk_config_pins(ccu9540_pins_cam_en,
			ARRAY_SIZE(ccu9540_pins_cam_en));
	if (ret < 0)
		pr_err("cam_en config failed\n");
}

static pin_cfg_t ccu9540_pins_uart0[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH,
};

static pin_cfg_t ccu9540_pins_uart1[] = {
	GPIO4_U1_RXD  | PIN_INPUT_PULLUP,
	GPIO5_U1_TXD  | PIN_OUTPUT_HIGH,
	GPIO6_U1_CTSn | PIN_INPUT_PULLUP,
	GPIO7_U1_RTSn | PIN_OUTPUT_HIGH,
};

static pin_cfg_t ccu9540_pins_uart1_off[] = {
	GPIO4_GPIO | PIN_INPUT_PULLUP,
	GPIO5_GPIO | PIN_OUTPUT_LOW,
	GPIO6_GPIO | PIN_INPUT_PULLUP,
	GPIO7_GPIO | PIN_OUTPUT_HIGH,
};

static void ux500_uart0_init(void)
{
	int ret;

	ret = nmk_config_pins(ccu9540_pins_uart0,
			ARRAY_SIZE(ccu9540_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart1_init(void)
{
	int ret = 0;

	ret = nmk_config_pins(ccu9540_pins_uart1,
				ARRAY_SIZE(ccu9540_pins_uart1));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart0_exit(void)
{
	int ret;

	ret = nmk_config_pins_sleep(ccu9540_pins_uart0,
			ARRAY_SIZE(ccu9540_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_disable failed\n");
}

static void ux500_uart1_exit(void)
{
	int ret = 0;

	ret = nmk_config_pins(ccu9540_pins_uart1_off,
				ARRAY_SIZE(ccu9540_pins_uart1_off));
	if (ret < 0)
		pr_err("pl011: uart pins_disable failed\n");
}

static struct amba_pl011_data uart0_plat = {
	.init = ux500_uart0_init,
	.exit = ux500_uart0_exit,
};

static struct amba_pl011_data uart1_plat = {
	.init = ux500_uart1_init,
	.exit = ux500_uart1_exit,
};

static void __init ccu9540_uart_init(struct device *parent)
{
	db8500_add_uart0(parent, &uart0_plat);
	db8500_add_uart1(parent, &uart1_plat);
	db8500_add_uart2(parent, NULL);
}

/* The AB9540 GPIO support is based on AB8500 but also has additional GPIO. */
static struct ab8500_gpio_platform_data ab9540_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO(1),
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/* initial_pin_config is the initial configuration of ab9540 pins.
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

	/* initial_pin_direction allows for the initial GPIO direction to
	 * be set.
	 */
	/* GpioDir7 = 0x24 ==> change to 0x26,
	 * change gpio50 driect to output ; gpio60 is output;
	 */
	.config_direction = {0x06, 0xF6, 0x72, 0x54, 0x00, 0x02, 0x26},

	/*
	 * initial_pin_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	/* GpioPud7 = 0xE5 ==> change to 0xE7,
	 * gpio50 pulldown is disabed, gpio60 pulldown is disabled
	 */
	.config_pullups = {0xFF, 0xFF, 0x7F, 0xFE, 0x7D, 0xFE, 0xE7},
};

static struct ab8500_sysctrl_platform_data ab8500_sysctrl_pdata = {
	/*
	 * SysClkReq1RfClkBuf - SysClkReq8RfClkBuf
	 * The initial values should not be changed because of the way
	 * the system works today
	 */
	.initial_req_buf_config
			= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	.reboot_reason_code = reboot_reason_code,
};

static struct modctrl_reg_pwr ab9540_modctrl_reg_pwr = {
	.reg_pwr_bank = AB8500_REGU_CTRL2,
	.reg_pwr_addr = AB9540_MODEM_CTRL2_REG,
	.reg_pwr_mask = AB9540_MODEM_CTRL2_SWDBBRSTN_BIT,
};

static struct abx540_modctrl_platform_data ab9540_modctrl_plat_data = {
	.reg_power  = &ab9540_modctrl_reg_pwr,
	.gpio_power = -1,
};

#ifdef CONFIG_KEYBOARD_NOMADIK_SKE
#define U9540_ROW_PIN_I0 67
#define U9540_ROW_PIN_I1 66
#define U9540_ROW_PIN_I6 79
#define U9540_COL_PIN_O0 65
#define U9540_COL_PIN_O1 64
#define U9540_COL_PIN_O6 78

static int u9540_ske_kp_rows[] = {
	U9540_ROW_PIN_I0, U9540_ROW_PIN_I1, U9540_ROW_PIN_I6
};
static int u9540_ske_kp_cols[] = {
	U9540_COL_PIN_O0, U9540_COL_PIN_O1, U9540_COL_PIN_O6
};

static int u9540_ske_kp_init(void)
{
	struct ux500_pins *pins;
	int i;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_enable(pins);

	/* disable any pull up/down on outputs (columns) */
	for (i = 0; i < ARRAY_SIZE(u9540_ske_kp_cols); i++)
		nmk_gpio_set_pull(u9540_ske_kp_cols[i], NMK_GPIO_PULL_NONE);

	return 0;
}

static int u9540_ske_kp_exit(void)
{
	struct ux500_pins *pins;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_disable(pins);

	return 0;
}

static const unsigned int u9540_ske_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_HOME),
	KEY(0, 6, KEY_F2),

	KEY(1, 0, KEY_VOLUMEDOWN),
	KEY(1, 1, KEY_F3),
	KEY(1, 6, KEY_F4),

	/* KEY(6, 0, xxx) not used */
	KEY(6, 1, KEY_CAMERA_FOCUS),
	KEY(6, 6, KEY_CAMERA),
};

static struct matrix_keymap_data u9540_ske_keymap_data = {
	.keymap      = u9540_ske_keymap,
	.keymap_size = ARRAY_SIZE(u9540_ske_keymap),
};

static struct ske_keypad_platform_data u9540_ske_keypad_data = {
	.init           = u9540_ske_kp_init,
	.exit           = u9540_ske_kp_exit,
	.gpio_input_pins = u9540_ske_kp_rows,
	.gpio_output_pins = u9540_ske_kp_cols,
	.keymap_data    = &u9540_ske_keymap_data,
	.no_autorepeat  = true,
	.krow           = 7,
	.kcol           = 7,
	.kconnected_rows = ARRAY_SIZE(u9540_ske_kp_rows),
	.kconnected_cols = ARRAY_SIZE(u9540_ske_kp_cols),
	.debounce_ms    = 20,                   /* in timeout period */
	.switch_delay	= 200,			/* in jiffies */
};
#endif /* CONFIG_KEYBOARD_NOMADIK_SKE */

static struct ab8500_platform_data ab9540_platdata = {
	.irq_base	= MOP500_AB8500_IRQ_BASE,
	.regulator	= &ab9540_regulator_plat_data,
	.battery	= &ab9540_bm_data,
	.charger	= &ab9540_charger_plat_data,
	.btemp		= &ab8500_btemp_plat_data,
	.fg		= &ab8500_fg_plat_data,
	.chargalg	= &ab8500_chargalg_plat_data,
	.gpio		= &ab9540_gpio_pdata,
	.sysctrl	= &ab8500_sysctrl_pdata,
	.pwmled		= &ab8500_pwmled_plat_data,

#ifdef CONFIG_INPUT_AB8500_ACCDET
	.accdet = &ab8500_accdet_pdata,
#endif
#ifdef CONFIG_PM
	.pm_power_off = true,
#endif
	.thermal_time_out = 20, /* seconds */
#ifdef CONFIG_INPUT_AB8505_MICRO_USB_DETECT
	.iddet = &iddet_adc_val_list,
#endif
	.modctrl	= &ab9540_modctrl_plat_data,
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start	= IRQ_DB8500_AB8500,
		.end	= IRQ_DB8500_AB8500,
		.flags	= IORESOURCE_IRQ
	}
};

/*  device name fixes ab type till detection possible */
struct platform_device ab9540_device = {
	.name = "ab9540-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab9540_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

static struct av8100_platform_data av8100_u9540_plat_data = {
	.irq			= NOMADIK_GPIO_TO_IRQ(192),
	.reset			= U9540_HDMI_RST_GPIO,
	.inputclk_id		= "ab9540-sysclk12Buf2",
	.regulator_pwr_id	= "hdmi_1v8",
	.alt_powerupseq		= true,
	.mclk_freq		= 1, /* MCLK_RNG_22_27 */
};


static struct i2c_board_info __initdata ccu9540_i2c0_devices[] = {
	{
		I2C_BOARD_INFO("av8100", 0x70),
		.platform_data = &av8100_u9540_plat_data,
	},
};

static struct pm2xxx_platform_data ccu9540_pm2xxx_platdata = {
	.wall_charger	= &ccu9540_pm2xxx_charger_plat_data,
	.battery	= &pm2xxx_bm_data,
};

static struct i2c_board_info __initdata ccu9540_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("pm2301", 0x2C),
		.platform_data = &ccu9540_pm2xxx_platdata,
	},
};

static struct stmpe_gpio_platform_data ccu9540_stmpe0_gpio_data = {
	/* pins MOP500_EGPIO(0) => MOP500_EGPIO(7) */
	.gpio_base = MOP500_EGPIO(0),
};

static struct stmpe_platform_data ccu9540_stmpe0_platform_data = {
	.id = 0,
	.blocks = STMPE_BLOCK_GPIO,
	.gpio = &ccu9540_stmpe0_gpio_data,
};

static struct stmpe_gpio_platform_data ccu9540_stmpe1_gpio_data = {
	/* pins MOP500_EGPIO(8) => MOP500_EGPIO(15) */
	.gpio_base = MOP500_EGPIO(8),
};

static struct stmpe_platform_data ccu9540_stmpe1_platform_data = {
	.id = 1,
	.blocks = STMPE_BLOCK_GPIO,
	.gpio = &ccu9540_stmpe1_gpio_data,
};

/*
 * LSM303DLH accelerometer + magnetometer & L3G4200D Gyroscope sensors
 */
#define LSM303DLHC_CHIP_ID 51
static struct lsm303dlh_platform_data lsm303dlh_pdata_u9540 = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negative_x = 1,
	.negative_y = 1,
	.negative_z = 0,
	.irq_a1 = HREFV60_ACCEL_INT1_GPIO,
	.irq_a2 = HREFV60_ACCEL_INT2_GPIO,
	.chip_id = LSM303DLHC_CHIP_ID,
};

static struct lps331ap_platform_data __initdata lps331ap_pdata = {
	.irq = 0,
};

static struct i2c_board_info __initdata ccu9540_i2c2_devices[] = {
	{
		/* LSM303DLH Accelerometer */
		I2C_BOARD_INFO("lsm303dlhc_a", 0x19),
		.platform_data = &lsm303dlh_pdata_u9540,
	},
	{
		/* LSM303DLH Magnetometer */
		I2C_BOARD_INFO("lsm303dlh_m", 0x1E),
		.platform_data = &lsm303dlh_pdata_u9540,
	},
	{
		/* LPS331AP Barometer */
		I2C_BOARD_INFO("lps331ap", 0x5C),
		.platform_data = &lps331ap_pdata,
	},
	/* Note: Gyroscope init is now handled in board-ccu9540-ver */
	{
		/* stmpe801 GPIO expander */
		I2C_BOARD_INFO("stmpe801", 0x44),
		.platform_data = &ccu9540_stmpe0_platform_data,
		.irq = -1,
	},
	{
		/* stmpe801 GPIO expander: Core and UIB ID */
		I2C_BOARD_INFO("stmpe801", 0x41),
		.platform_data = &ccu9540_stmpe1_platform_data,
		.irq = -1,
	},
};

static void __init ccu9540_init_irq(void)
{
	prcmu_early_init(&db9540_tcdm_map, true);
	ux500_init_irq();
}

static void __init u9540_init_machine(void)
{
	struct device *parent;
	int i;

	((struct dbx500_cpuidle_platform_data *)
	 db8500_cpuidle_device.dev.platform_data)->wakeups |=
		PRCMU_WAKEUP(HSI0);

	parent = u8500_init_devices();

	ccu9540_pinmaps_init();
	dbx500_add_pinctrl(parent, "pinctrl-db8500");

	platform_device_register(&dbx540_prcmu_device);
	platform_device_register(&u9540_usecase_gov_device);
	ccu9540_tee_init();

	for (i = 0; i < ARRAY_SIZE(u9540_platform_devs); i++)
		u9540_platform_devs[i]->dev.parent = parent;

	u8500_cryp1_hash1_init(parent);

#ifdef CONFIG_HSI
	hsi_register_board_info(u8500_hsi_devices,
			ARRAY_SIZE(u8500_hsi_devices));
#endif
	platform_add_devices(u9540_platform_devs,
				ARRAY_SIZE(u9540_platform_devs));
	ccu9540_i2c_init(parent);
	ccu9540_sdi_init(parent);
	mop500_msp_init(parent);
	ccu9540_spi_init(parent);
	ccu9540_uart_init(parent);
	mop500_wlan_init();
	ccu9540_cam_en_init();

#ifdef CONFIG_KEYBOARD_NOMADIK_SKE
	/*
	 * If a hw debugger is detected, do not load the ske driver
	 * since the gpio usage collides.
	 */
	if (!(prcmu_read(PRCM_DEBUG_NOPWRDOWN_VAL) &
	      ARM_DEBUG_NOPOWER_DOWN_REQ))
		db8500_add_ske_keypad(NULL, &u9540_ske_keypad_data,
				sizeof(u9540_ske_keypad_data));
#endif

#ifdef CONFIG_ANDROID_STE_TIMED_VIBRA
	mop500_vibra_init();
#endif
	platform_device_register(&ccu9540_version_dev);
	platform_device_register(&ab9540_device);

	i2c_register_board_info(0, ccu9540_i2c0_devices,
			ARRAY_SIZE(ccu9540_i2c0_devices));
	i2c_register_board_info(1, ccu9540_i2c1_devices,
			ARRAY_SIZE(ccu9540_i2c1_devices));
	i2c_register_board_info(2, ccu9540_i2c2_devices,
			ARRAY_SIZE(ccu9540_i2c2_devices));

	/* This board has full regulator constraints */
	regulator_has_full_constraints();
}

MACHINE_START(U9540, "ST-Ericsson 9540 platform")
	.atag_offset	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= ccu9540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= u9540_init_machine,
	.restart	= ux500_restart,
MACHINE_END

#ifdef CONFIG_MACH_UX500_DT

struct of_dev_auxdata u9540_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("arm,pl011", 0x80120000, "uart0", &uart0_plat),
	OF_DEV_AUXDATA("arm,pl011", 0x80121000, "uart1", &uart1_plat),
	OF_DEV_AUXDATA("arm,pl011", 0x80007000, "uart2", NULL),
	{},
};

static const struct of_device_id u9540_soc_node[] = {
	/* only create devices below soc node */
	{ .compatible = "stericsson,db8500", },
	{ },
};

static void __init u9540_dt_init_machine(void)
{
	struct device *parent;
	int i;

	((struct dbx500_cpuidle_platform_data *)
	 db8500_cpuidle_device.dev.platform_data)->wakeups |=
		PRCMU_WAKEUP(HSI0);

	parent = u8500_init_devices();

	ccu9540_pinmaps_init();
	dbx500_add_pinctrl(parent, "pinctrl-db8500");

	platform_device_register(&dbx540_prcmu_device);
	platform_device_register(&u9540_usecase_gov_device);

	for (i = 0; i < ARRAY_SIZE(u9540_platform_devs); i++)
		u9540_platform_devs[i]->dev.parent = parent;

	/* automatically probe child nodes of db8500 device */
	of_platform_populate(NULL, u9540_soc_node,
			u9540_auxdata_lookup, parent);

	u8500_cryp1_hash1_init(parent);

#ifdef CONFIG_HSI
	hsi_register_board_info(u8500_hsi_devices,
			ARRAY_SIZE(u8500_hsi_devices));
#endif
	platform_add_devices(u9540_platform_devs,
				ARRAY_SIZE(u9540_platform_devs));
	ccu9540_i2c_init(parent);
	ccu9540_sdi_init(parent);
	mop500_msp_init(parent);
	ccu9540_spi_init(parent);
	mop500_wlan_init();

#ifdef CONFIG_KEYBOARD_NOMADIK_SKE
	/*
	 * If a hw debugger is detected, do not load the ske driver
	 * since the gpio usage collides.
	 */
	if (!(prcmu_read(PRCM_DEBUG_NOPWRDOWN_VAL) &
	      ARM_DEBUG_NOPOWER_DOWN_REQ))
		db8500_add_ske_keypad(NULL, &u9540_ske_keypad_data,
				sizeof(u9540_ske_keypad_data));
#endif

#ifdef CONFIG_ANDROID_STE_TIMED_VIBRA
	mop500_vibra_init();
#endif
	platform_device_register(&ccu9540_version_dev);
	platform_device_register(&ab9540_device);

	i2c_register_board_info(0, ccu9540_i2c0_devices,
			ARRAY_SIZE(ccu9540_i2c0_devices));
	i2c_register_board_info(1, ccu9540_i2c1_devices,
			ARRAY_SIZE(ccu9540_i2c1_devices));
	i2c_register_board_info(2, ccu9540_i2c2_devices,
			ARRAY_SIZE(ccu9540_i2c2_devices));

	/* This board has full regulator constraints */
	regulator_has_full_constraints();

}

static const char *u9540_dt_board_compat[] = {
	"st-ericsson,u9540",
	NULL,
};


DT_MACHINE_START(U9540_DT, "ST-Ericsson U9540 platform (Device Tree Support)")
	.map_io		= u8500_map_io,
	.init_irq	= ccu9540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= u9540_dt_init_machine,
	.dt_compat      = u9540_dt_board_compat,
MACHINE_END
#endif
