/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * Authors: Maxime Coquelin <maxime.coquelin@stericsson.com>
 *	    Christophe Guibout <christophe.guibout@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <plat/ste_dma40.h>
#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/ste-dma40-db8500.h>

#include "devices-db8500.h"

#define SDI_PERIPHID_9540 0x10480180
#define SDI_PERIPHID_8540 0x20480180

/*
 * SDI 4 (on-board eMMC)
 */

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg palladiumx540_sdi4_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV42_SD_MM4_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg palladiumx540_sdi4_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV42_SD_MM4_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data palladium8540_sdi4_data = {
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 200000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED,
	.capabilities2	= MMC_CAP2_BOOTPART_NOACC,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &palladiumx540_sdi4_dma_cfg_rx,
	.dma_tx_param	= &palladiumx540_sdi4_dma_cfg_tx,
#endif
};

static struct mmci_platform_data palladium9540_sdi4_data = {
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED,
	.capabilities2	= MMC_CAP2_BOOTPART_NOACC,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &palladiumx540_sdi4_dma_cfg_rx,
	.dma_tx_param	= &palladiumx540_sdi4_dma_cfg_tx,
#endif
};

void __init palladiumx540_sdi_init(struct device *parent)
{
	if (machine_is_u9540()) {
		/* On-board eMMC */
		db8500_add_sdi4(parent, &palladium9540_sdi4_data,
				SDI_PERIPHID_9540);
	} else if (machine_is_u8540()) {
		/* On-board eMMC */
		db8500_add_sdi4(parent, &palladium8540_sdi4_data,
				SDI_PERIPHID_8540);
	} else {
		pr_err("palladiumx540_sdi_init: machine unknown");
	}
}
