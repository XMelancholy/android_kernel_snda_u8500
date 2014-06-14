/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Bin ZHOU <bin.zhou@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mfd/dbx500-prcmu.h>

#include <asm/mach-types.h>
#include <plat/ste_dma40.h>
#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/ste-dma40-db8500.h>

#include "board-mop500-sdi.h"
#include "devices-db8500.h"
#include "board-ccu8540-sdi.h"

#define SDI_PERIPHID 0x20480180

/*
 * SDI 0 (MicroSD slot)
 */
static struct regulator *regulator_vmmc_io;

#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ccu8540_sdi0_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV1_SD_MMC0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};

static struct stedma40_chan_cfg ccu8540_sdi0_dma_cfg_tx = {
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

static struct mmci_platform_data ccu8540_sdi0_data = {
	.f_max		= 100000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_UHS_SDR12 |
				MMC_CAP_UHS_SDR25 |
				MMC_CAP_UHS_DDR50,
	.capabilities2	= MMC_CAP2_DETECT_ON_ERR,
	.gpio_wp	= -1,
	.gpio_cd	= CCU8540_SDMMC_CD_GPIO,
	.cd_invert	= true,
	.levelshifter	= true,
	.sigdir		= MCI_ST_FBCLKEN |
				MCI_ST_CMDDIREN |
				MCI_ST_DATA0DIREN,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &ccu8540_sdi0_dma_cfg_rx,
	.dma_tx_param	= &ccu8540_sdi0_dma_cfg_tx,
#endif
};

/*
 * SDI1 (SDIO WLAN)
 */
#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ccu8540_sdi1_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV32_SD_MM1_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg ccu8540_sdi1_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV32_SD_MM1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data ccu8540_sdi1_data = {
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 200000000,
	.capabilities	= MMC_CAP_4_BIT_DATA,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &ccu8540_sdi1_dma_cfg_rx,
	.dma_tx_param	= &ccu8540_sdi1_dma_cfg_tx,
#endif
};

/*
 * SDI 4 (on-board eMMC)
 */

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg ccu8540_sdi4_dma_cfg_rx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type =  DB8500_DMA_DEV42_SD_MM4_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg ccu8540_sdi4_dma_cfg_tx = {
	.mode = STEDMA40_MODE_LOGICAL,
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV42_SD_MM4_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data ccu8540_sdi4_data = {
	.f_max		= 200000000,
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
	.dma_rx_param	= &ccu8540_sdi4_dma_cfg_rx,
	.dma_tx_param	= &ccu8540_sdi4_dma_cfg_tx,
#endif
};

static int ccu8540_sdi0_ios_handler(struct device *dev, struct mmc_ios *ios,
					enum rpm_status pm)
{
	static unsigned char power_mode = MMC_POWER_ON;
	static unsigned char signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	if (signal_voltage == ios->signal_voltage)
		goto do_power;

	if (power_mode == MMC_POWER_ON) {
		power_mode = MMC_POWER_OFF;
		regulator_disable(regulator_vmmc_io);
	}

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		prcmu_set_sdmmc_psw(true);
		regulator_set_voltage(regulator_vmmc_io, 2750000, 3000000);
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		regulator_set_voltage(regulator_vmmc_io, 1650000, 1900000);
		prcmu_set_sdmmc_psw(false);
		break;
	default:
		pr_warning("Non supported signal voltage for levelshifter.\n");
		break;
	}

	signal_voltage = ios->signal_voltage;

do_power:
	if (power_mode == ios->power_mode)
		goto do_pm;

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		break;
	case MMC_POWER_ON:
		regulator_enable(regulator_vmmc_io);
		usleep_range(3000, 4000);
		break;
	case MMC_POWER_OFF:
		regulator_disable(regulator_vmmc_io);
		break;
	default:
		pr_warning("Non supported power mode.\n");
		break;
	}

	power_mode = ios->power_mode;

do_pm:
	if ((pm == RPM_SUSPENDING) && (power_mode == MMC_POWER_ON)) {
		/* Disable Vsdio to save power */
		regulator_disable(regulator_vmmc_io);
	} else if ((pm == RPM_RESUMING) && (power_mode == MMC_POWER_ON)) {
		/* Re-enable Vsdio. */
		regulator_enable(regulator_vmmc_io);
		usleep_range(3000, 4000);
	}

	return 0;
}

static int ccu8540_sdi0_ios_handler_init(struct mmci_platform_data *sdi0)
{
	int error = 0;

	regulator_vmmc_io = regulator_get(NULL, "vmmc_io");
	if (IS_ERR(regulator_vmmc_io)) {
		error = PTR_ERR(regulator_vmmc_io);
		return error;
	}

	regulator_disable(regulator_vmmc_io);
	prcmu_set_sdmmc_psw(true);
	regulator_set_voltage(regulator_vmmc_io, 2750000, 3000000);
	error = regulator_enable(regulator_vmmc_io);
	if (error)
		return error;
	usleep_range(3000, 4000);

	sdi0->ios_handler = ccu8540_sdi0_ios_handler;

	return error;
}

void __init ccu8540_sdi_init(struct device *parent)
{
	int ret;

	ret = ccu8540_sdi0_ios_handler_init(&ccu8540_sdi0_data);
	if (ret)
		pr_warn("%s: SDI0 ios hanfler init failed (%d)", __func__, ret);

	db8500_add_sdi4(parent, &ccu8540_sdi4_data, SDI_PERIPHID);
	db8500_add_sdi0(parent, &ccu8540_sdi0_data, SDI_PERIPHID);
	db8500_add_sdi1(parent, &ccu8540_sdi1_data, SDI_PERIPHID);
}
