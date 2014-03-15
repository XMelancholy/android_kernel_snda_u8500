/*
 * ste_hsi.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author:  Cedric Madianga <cedric.madianga@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef STE_HSI_H
#define STE_HSI_H

#include <linux/wakelock.h>

#define STE_HSI_LOW_SPEED	50000 /* bps */
#define STE_HSI_MIDDLE_SPEED	100000
#define STE_HSI_HIGH_SPEED	200000
#define STE_HSI_MAX_FREQ	200000000 /* Hz */
#define STE_HSI_MIN_FREQ	100000000
#define STE_HSI_MIDDLE_OPP	50
#define STE_HSI_HIGH_OPP	100

#define DB8500_PRCM_LINE_VALUE			0x170
#define DB8500_PRCM_LINE_VALUE_HSI_CAWAKE0	BIT(3)

/*
 * Copy of HSIR/HSIT context for restoring after HW reset (Vape power off).
 */
struct ste_hsi_hw_context {
	unsigned int tx_mode;
	unsigned int tx_divisor;
	unsigned int tx_channels;
	unsigned int tx_priority;
	unsigned int rx_mode;
	unsigned int rx_channels;
	unsigned int tx_dma_burst;
	unsigned int rx_dma_burst;
};

/**
 * struct ste_hsi_controller - STE HSI controller data
 * @dev: device associated to STE HSI controller
 * @tx_dma_base: HSI TX peripheral physical address
 * @rx_dma_base: HSI RX peripheral physical address
 * @rx_base: HSI RX peripheral virtual address
 * @tx_base: HSI TX peripheral virtual address
 * @regulator: STE HSI Vape consumer regulator
 * @context: copy of client-configured HSI TX / HSI RX registers
 * @tx_clk: HSI TX core clock (HSITXCLK)
 * @rx_clk: HSI RX core clock (HSIRXCLK)
 * @ssitx_clk: HSI TX host clock (HCLK)
 * @ssirx_clk: HSI RX host clock (HCLK)
 * @clk_work: structure for delayed HSI clock disabling
 * @overrun_irq: HSI channels overrun IRQ table
 * @ck_refcount: reference count for clock enable operation
 * @ck_lock: locking primitive for HSI clocks
 * @lock: locking primitive for HSI controller
 * @use_dma: flag for DMA enabled
 * @ck_on: flag for HSI clocks enabled
 * @ddbg_dir_hsit: ptr to hsit deep debug dentry
 * @ddbg_dir_hsir: ptr to hsir deep debug dentry
 */
struct ste_hsi_controller {
	struct device *dev;
	dma_addr_t tx_dma_base;
	dma_addr_t rx_dma_base;
	unsigned char __iomem *rx_base;
	unsigned char __iomem *tx_base;
	struct regulator *regulator;
	struct ste_hsi_hw_context *context;
	struct clk *tx_clk;
	struct clk *rx_clk;
	struct clk *ssitx_clk;
	struct clk *ssirx_clk;
	struct delayed_work clk_work;
	int overrun_irq[STE_HSI_MAX_CHANNELS];
	int ck_refcount;
	spinlock_t ck_lock;
	spinlock_t lock;
	unsigned int use_dma:1;
	unsigned int ck_on:1;
	unsigned int suspend_ongoing;
	struct wake_lock suspend_wakelock;
	unsigned int ca_status;
};

enum {
	CA_SLEEP,
	CA_START_RX,
	CA_STOP_RX,
};

#ifdef CONFIG_STE_DMA40
struct ste_hsi_channel_dma {
	struct dma_chan *dma_chan;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
};
#endif

struct ste_hsi_port {
	struct device *dev;
	struct list_head txqueue[STE_HSI_MAX_CHANNELS];
	struct list_head rxqueue[STE_HSI_MAX_CHANNELS];
	struct list_head brkqueue;
	int cawake_irq;
	unsigned int pending_ca_wake_isr;
	int acwake_gpio;
	int tx_irq;
	int rx_irq;
	int excep_irq;
	struct tasklet_struct cawake_tasklet;
	struct tasklet_struct rx_tasklet;
	struct tasklet_struct tx_tasklet;
	struct tasklet_struct exception_tasklet;
	struct tasklet_struct overrun_tasklet;
	unsigned char channels;
	struct work_struct ca_wake_work;
#ifdef CONFIG_STE_DMA40
	struct ste_hsi_channel_dma tx_dma[STE_HSI_MAX_CHANNELS];
	struct ste_hsi_channel_dma rx_dma[STE_HSI_MAX_CHANNELS];
#endif
};

#define hsi_to_ste_port(port) (hsi_port_drvdata(port))
#define hsi_to_ste_controller(con) (hsi_controller_drvdata(con))
#define client_to_ste_port(cl) (hsi_port_drvdata(hsi_get_port(cl)))
#define client_to_hsi(cl) \
	(to_hsi_controller(hsi_get_port(cl)->device.parent))
#define client_to_ste_controller(cl)  \
	(hsi_controller_drvdata(client_to_hsi(cl)))
#define ste_port_to_ste_controller(port) \
	((struct ste_hsi_controller *)hsi_controller_drvdata(	\
		to_hsi_controller(port->dev->parent)))

int ste_hsi_clock_enable(struct hsi_controller *hsi);
void ste_hsi_clock_disable(struct hsi_controller *hsi);

#ifdef CONFIG_STE_HSI_DEEP_DEBUG
int ste_hsi_deep_debug_exit(void);
int ste_hsi_deep_debug_init(struct hsi_controller *hsi);
#else
int ste_hsi_deep_debug_exit(void) {return 0; }
int ste_hsi_deep_debug_init(struct hsi_controller *hsi) {return 0; }
#endif /* STE_HSI_DEEP_DEBUG */

#endif /* STE_HSI_H */
