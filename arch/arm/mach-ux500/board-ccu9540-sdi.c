/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Authors: Hanumath Prasad <hanumath.prasad@stericsson.com>
 *	    Maxime Coquelin <maxime.coquelin@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <plat/ste_dma40.h>
#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/ste-dma40-db8500.h>

#include "board-common-sdi.h"
#include "devices-db8500.h"
#include "board-ccu9540.h"

#define SDI_PERIPHID 0x10480180

/*
 * SDI 0 (MicroSD slot)
 */

/* GPIO pins used by the sdi0 level shifter */
static struct sdi_ios_pins sdi0_ios_pins = {
	.enable = CCU9540_SDMMC_EN_GPIO,
	.vsel = CCU9540_SDMMC_1V8_3V_GPIO,
};


#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ccu9540_sdi0_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV1_SD_MMC0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};

static struct stedma40_chan_cfg ccu9540_sdi0_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV1_SD_MMC0_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};
#endif

static struct mmci_platform_data ccu9540_sdi0_data = {
	.f_max		= 100000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_UHS_SDR12 |
				MMC_CAP_UHS_SDR25 |
				MMC_CAP_UHS_DDR50,
	.capabilities2	= MMC_CAP2_DETECT_ON_ERR,
	.gpio_wp	= -1,
	.gpio_cd	= CCU9540_SDMMC_CD_GPIO,
	.cd_invert	= true,
	.levelshifter   = true,
	.sigdir		= MCI_ST_FBCLKEN |
				MCI_ST_CMDDIREN |
				MCI_ST_DATA0DIREN,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &ccu9540_sdi0_dma_cfg_rx,
	.dma_tx_param	= &ccu9540_sdi0_dma_cfg_tx,
#endif
};

/*
 * SDI1 (SDIO WLAN)
 */
#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg sdi1_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV32_SD_MM1_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg sdi1_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV32_SD_MM1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data ccu9540_sdi1_data = {
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi1_dma_cfg_rx,
	.dma_tx_param	= &sdi1_dma_cfg_tx,
#endif
};

/*
 * SDI 4 (on-board eMMC)
 */

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg ccu9540_sdi4_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV42_SD_MM4_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg ccu9540_sdi4_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV42_SD_MM4_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data ccu9540_sdi4_data = {
	.f_max		= 100000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_1_8V_DDR |
				MMC_CAP_UHS_DDR50,
	.capabilities2	= MMC_CAP2_BOOTPART_NOACC,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &ccu9540_sdi4_dma_cfg_rx,
	.dma_tx_param	= &ccu9540_sdi4_dma_cfg_tx,
#endif
};

void __init ccu9540_sdi_init(struct device *parent)
{
	int ret;

	ret = ux500_common_sdi0_ios_handler_init(&ccu9540_sdi0_data,
			&sdi0_ios_pins);
	if (ret)
		pr_warn("%s: SDI0 ios hanfler init failed (%d)", __func__, ret);

	db8500_add_sdi4(parent, &ccu9540_sdi4_data, SDI_PERIPHID);
	db8500_add_sdi0(parent, &ccu9540_sdi0_data, SDI_PERIPHID);
	db8500_add_sdi1(parent, &ccu9540_sdi1_data, SDI_PERIPHID);
}
