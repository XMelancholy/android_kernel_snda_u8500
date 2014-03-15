
/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com>
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
#include <linux/ux500_modem_control.h>

#include <plat/pincfg.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <plat/gpio-nomadik.h>

#include <mach/ste-dma40-db8500.h>
#include <mach/devices.h>
#include <mach/setup.h>

#include "pins-db8500.h"
#include "devices-db8500.h"
#include "cpu-db8500.h"

#include "board-common-uib.h"
#include "board-palladiumx540-sdi.h"
#include "board-ccu8540-pins.h"

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

static void __init palladium8540_uart_init(struct device *parent)
{
	db8500_add_uart0(parent, NULL);
	db8500_add_uart1(parent, NULL);
	db8500_add_uart2(parent, NULL);
}

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

static void __init palladium8540_init_irq(void)
{
	prcmu_early_init(&db8540_tcdm_map, true);
	ux500_init_irq();
}

static void __init palladium8540_init_machine(void)
{
	struct device *parent;

	dbx500_add_pinctrl(NULL, "pinctrl-db8540");
	ccu8540_pinmaps_init();
	parent = u8500_init_devices();
	platform_device_register(&dbx540_prcmu_device);
	u8500_cryp1_hash1_init(parent);
	palladiumx540_sdi_init(parent);
	palladium8540_uart_init(parent);
	platform_device_register(&db8540_modem_control_device);
	platform_device_register(&db8540_xmip_device);
}

MACHINE_START(U8540, "ST-Ericsson L8540 Palladium platform")
	.atag_offset	= 0x100,
	.map_io		= u8500_map_io,
	.init_irq	= palladium8540_init_irq,
	.timer		= &ux500_timer,
	.handle_irq	= gic_handle_irq,
	.init_machine	= palladium8540_init_machine,
	.restart	= ux500_restart,
MACHINE_END
