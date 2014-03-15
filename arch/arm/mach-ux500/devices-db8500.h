/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __DEVICES_DB8500_H
#define __DEVICES_DB8500_H

#include "devices-common.h"
#include <mach/pm.h>

struct ske_keypad_platform_data;
struct pl022_ssp_controller;

static inline struct platform_device *
db8500_add_ske_keypad(struct device *parent,
		      struct ske_keypad_platform_data *pdata,
		      size_t size)
{
	struct resource resources[] = {
		DEFINE_RES_MEM(U8500_SKE_BASE, SZ_4K),
		DEFINE_RES_IRQ(IRQ_DB8500_KB),
	};
	struct platform_device *dev =
		platform_device_register_resndata(parent, "nmk-ske-keypad", -1,
						  resources, 2, pdata, size);
	dev->dev.pm_domain = &ux500_dev_power_domain;
	return dev;
}

static inline struct amba_device *
db8500_add_ssp(struct device *parent, const char *name, resource_size_t base,
	       int irq, struct pl022_ssp_controller *pdata)
{
	struct amba_device *dev = amba_ahb_device_add(parent, name, base,
						       SZ_4K, irq,
						       0, pdata, 0);
	return dev;
}


#define db8500_add_i2c0(parent, pdata) \
	dbx500_add_i2c(parent, 0, U8500_I2C0_BASE, IRQ_DB8500_I2C0, pdata)
#define db8500_add_i2c1(parent, pdata) \
	dbx500_add_i2c(parent, 1, U8500_I2C1_BASE, IRQ_DB8500_I2C1, pdata)
#define db8500_add_i2c2(parent, pdata) \
	dbx500_add_i2c(parent, 2, U8500_I2C2_BASE, IRQ_DB8500_I2C2, pdata)
#define db8500_add_i2c3(parent, pdata) \
	dbx500_add_i2c(parent, 3, U8500_I2C3_BASE, IRQ_DB8500_I2C3, pdata)
#define db8500_add_i2c4(parent, pdata) \
	dbx500_add_i2c(parent, 4, U8500_I2C4_BASE, IRQ_DB8500_I2C4, pdata)
#define db8540_add_i2c5(parent,pdata) \
	dbx500_add_i2c(parent, 5, U8540_I2C5_BASE, IRQ_DB8540_I2C5, pdata)
#define db8540_add_i2c6(parent,pdata) \
	dbx500_add_i2c(parent, 6, U8540_I2C6_BASE , IRQ_DB8540_I2C6, pdata)

#define db8500_add_msp0_i2s(parent, pdata) \
	dbx500_add_msp_i2s(parent, 0, U8500_MSP0_BASE, IRQ_DB8500_MSP0, pdata)
#define db8500_add_msp1_i2s(parent, pdata) \
	dbx500_add_msp_i2s(parent, 1, U8500_MSP1_BASE, IRQ_DB8500_MSP1, pdata)
#define db8500_add_msp2_i2s(parent, pdata) \
	dbx500_add_msp_i2s(parent, 2, U8500_MSP2_BASE, IRQ_DB8500_MSP2, pdata)
#define db8500_add_msp3_i2s(parent, pdata) \
	dbx500_add_msp_i2s(parent, 3, U8500_MSP3_BASE, IRQ_DB8500_MSP1, pdata)
#define db9540_add_msp4_i2s(parent, pdata) \
	dbx500_add_msp_i2s(parent, 4, U9540_MSP4_BASE, IRQ_DB9540_MSP4, pdata)

#define db8500_add_msp0_spi(parent, pdata) \
	dbx500_add_msp_spi(parent, "msp0", U8500_MSP0_BASE, IRQ_DB8500_MSP0, pdata)
#define db8500_add_msp1_spi(parent, pdata) \
	dbx500_add_msp_spi(parent, "msp1", U8500_MSP1_BASE, IRQ_DB8500_MSP1, pdata)
#define db8500_add_msp2_spi(parent, pdata) \
	dbx500_add_msp_spi(parent, "msp2", U8500_MSP2_BASE, IRQ_DB8500_MSP2, pdata)
#define db8500_add_msp3_spi(parent, pdata) \
	dbx500_add_msp_spi(parent, "msp3", U8500_MSP3_BASE, IRQ_DB8500_MSP1, pdata)
#define db9540_add_msp4_spi(parent, pdata) \
	dbx500_add_msp_spi(parent, "msp4", U9540_MSP4_BASE, IRQ_DB9540_MSP4, pdata)

#define db8500_add_rtc(parent) \
	dbx500_add_rtc(parent, U8500_RTC_BASE, IRQ_DB8500_RTC);

#define db8500_add_usb(parent, rx_cfg, tx_cfg) \
	ux500_add_usb(parent, U8500_USBOTG_BASE, IRQ_DB8500_USBOTG, rx_cfg, tx_cfg)

#define db8500_add_sdi0(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi0", U8500_SDI0_BASE, IRQ_DB8500_SDMMC0, pdata, pid)
#define db8500_add_sdi1(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi1", U8500_SDI1_BASE, IRQ_DB8500_SDMMC1, pdata, pid)
#define db8500_add_sdi2(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi2", U8500_SDI2_BASE, IRQ_DB8500_SDMMC2, pdata, pid)
#define db8500_add_sdi3(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi3", U8500_SDI3_BASE, IRQ_DB8500_SDMMC3, pdata, pid)
#define db8500_add_sdi4(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi4", U8500_SDI4_BASE, IRQ_DB8500_SDMMC4, pdata, pid)
#define db8500_add_sdi5(parent, pdata, pid) \
	dbx500_add_sdi(parent, "sdi5", U8500_SDI5_BASE, IRQ_DB8500_SDMMC5, pdata, pid)

#define db8500_add_ssp0(parent, pdata) \
	db8500_add_ssp(parent, "ssp0", U8500_SSP0_BASE, IRQ_DB8500_SSP0, pdata)
#define db8500_add_ssp1(parent, pdata) \
	db8500_add_ssp(parent, "ssp1", U8500_SSP1_BASE, IRQ_DB8500_SSP1, pdata)

#define db8500_add_spi0(parent, pdata) \
	dbx500_add_spi(parent, "spi0", U8500_SPI0_BASE, IRQ_DB8500_SPI0, pdata, 0)
#define db8500_add_spi1(parent, pdata) \
	dbx500_add_spi(parent, "spi1", U8500_SPI1_BASE, IRQ_DB8500_SPI1, pdata, 0)
#define db8500_add_spi2(parent, pdata) \
	dbx500_add_spi(parent, "spi2", U8500_SPI2_BASE, IRQ_DB8500_SPI2, pdata, 0)
#define db8500_add_spi3(parent, pdata) \
	dbx500_add_spi(parent, "spi3", U8500_SPI3_BASE, IRQ_DB8500_SPI3, pdata, 0)

#define db8500_add_uart0(parent, pdata) \
	dbx500_add_uart(parent, "uart0", U8500_UART0_BASE, IRQ_DB8500_UART0, pdata)
#define db8500_add_uart1(parent, pdata) \
	dbx500_add_uart(parent, "uart1", U8500_UART1_BASE, IRQ_DB8500_UART1, pdata)
#define db8500_add_uart2(parent, pdata) \
	dbx500_add_uart(parent, "uart2", U8500_UART2_BASE, IRQ_DB8500_UART2, pdata)
#define db8500_add_uart3(parent, pdata) \
	dbx500_add_uart(parent, "uart3", U8500_UART3_BASE, IRQ_DB8500_UART3, pdata)
#define db8500_add_uart4(parent, pdata) \
	dbx500_add_uart(parent, "uart4", U8500_UART4_BASE, IRQ_DB8500_UART4, pdata)

#define db8500_add_cryp1(parent, pdata) \
	dbx500_add_cryp1(parent, -1, U8500_CRYP1_BASE, IRQ_DB8500_CRYP1, pdata)
#define db8500_add_hash1(parent, pdata) \
	dbx500_add_hash1(parent, -1, U8500_HASH1_BASE, pdata)

#define db8500_add_tee(pdata) \
	dbx5x0_add_tee(NULL, -1, pdata)

#endif
