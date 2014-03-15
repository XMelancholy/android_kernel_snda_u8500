
/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Authors: Maxime Coquelin <maxime.coquelin@stericsson.com>
 *	    Christophe Guibout <christophe.guibout@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/amba/pl022.h>
#include <linux/amba/serial.h>
#include <linux/platform_data/crypto-ux500.h>

#include <plat/pincfg.h>
#include <plat/gpio-nomadik.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <mach/ste-dma40-db8500.h>
#include <mach/devices.h>
#include <mach/setup.h>

#include "pins-db8500.h"
#include "devices-db8500.h"
#include "cpu-db8500.h"

#include "board-common-uib.h"
#include "board-palladiumx540-sdi.h"
#include "board-ccu9540-pins.h"

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

static pin_cfg_t mop500_pins_uart0[] = {
	GPIO0_U0_CTSn   | PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn   | PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD    | PIN_INPUT_PULLUP,
	GPIO3_U0_TXD    | PIN_OUTPUT_HIGH,
};

static pin_cfg_t palladium9540_pins_uart1[] = {
	GPIO4_U1_RXD  | PIN_INPUT_PULLUP,
	GPIO5_U1_TXD  | PIN_OUTPUT_HIGH,
	GPIO6_U1_CTSn | PIN_INPUT_PULLUP,
	GPIO7_U1_RTSn | PIN_OUTPUT_HIGH,
};

static pin_cfg_t palladium9540_pins_uart1_off[] = {
	GPIO4_GPIO | PIN_INPUT_PULLUP,
	GPIO5_GPIO | PIN_OUTPUT_LOW,
	GPIO6_GPIO | PIN_INPUT_PULLUP,
	GPIO7_GPIO | PIN_OUTPUT_HIGH,
};

static void ux500_uart0_init(void)
{
	int ret;

	ret = nmk_config_pins(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart1_init(void)
{
	int ret = 0;

	ret = nmk_config_pins(palladium9540_pins_uart1,
				ARRAY_SIZE(palladium9540_pins_uart1));
	if (ret < 0)
		pr_err("pl011: uart pins_enable failed\n");
}

static void ux500_uart0_exit(void)
{
	int ret;

	ret = nmk_config_pins_sleep(mop500_pins_uart0,
			ARRAY_SIZE(mop500_pins_uart0));
	if (ret < 0)
		pr_err("pl011: uart pins_disable failed\n");
}

static void ux500_uart1_exit(void)
{
	int ret = 0;

	ret = nmk_config_pins(palladium9540_pins_uart1_off,
				ARRAY_SIZE(palladium9540_pins_uart1_off));
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

static void __init palladium9540_uart_init(struct device *parent)
{
	db8500_add_uart0(parent, &uart0_plat);
	db8500_add_uart1(parent, &uart1_plat);
	db8500_add_uart2(parent, NULL);
}

static void __init palladium9540_init_irq(void)
{
	prcmu_early_init(&db9540_tcdm_map, true);
	ux500_init_irq();
}

static void __init palladium9540_init_machine(void)
{
	struct device *parent;

	parent = u8500_init_devices();
	platform_device_register(&dbx540_prcmu_device);
	ccu9540_pinmaps_init();
	u8500_cryp1_hash1_init(parent);
	palladiumx540_sdi_init(parent);
	palladium9540_uart_init(parent);
}

MACHINE_START(U9540, "ST-Ericsson 9540 platform")
	.atag_offset	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= palladium9540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= palladium9540_init_machine,
	.restart	= ux500_restart,
MACHINE_END
