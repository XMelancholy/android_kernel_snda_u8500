/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Rickard Andersson <rickard.andersson@stericsson.com> for
 *         ST-Ericsson.
 * License terms: GNU General Public License (GPL) version 2
 *
 */

#include <linux/io.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/nomadik.h>
#include <linux/mfd/dbx500-prcmu.h>

#include <asm/hardware/gic.h>
#include <asm/processor.h>

#include <mach/hardware.h>
#include <mach/pm.h>
#include <mach/irqs.h>

#include <mach/id.h>

#define STABILIZATION_TIME 30 /* us */
#define GIC_FREEZE_DELAY 1 /* us */

#define PRCM_ARM_WFI_STANDBY_CPU0_WFI 0x8
#define PRCM_ARM_WFI_STANDBY_CPU1_WFI 0x10

/* Dual A9 core interrupt management unit registers */
#define PRCM_A9_MASK_REQ	0x328
#define PRCM_A9_MASK_REQ_PRCM_A9_MASK_REQ	0x1
#define PRCM_A9_MASK_ACK	0x32c

#define PRCM_ARMITMSK31TO0	0x11c
#define PRCM_ARMITMSK63TO32	0x120
#define PRCM_ARMITMSK95TO64	0x124
#define PRCM_ARMITMSK127TO96	0x128
#define PRCM_ARMITMSK159TO128	0x184
#define PRCM_ARMITMSK191TO160	0x188
#define PRCM_ARMITMSK223TO192	0x18C
#define PRCM_POWER_STATE_VAL	0x25C
#define PRCM_ARMITVAL31TO0	0x260
#define PRCM_ARMITVAL63TO32	0x264
#define PRCM_ARMITVAL95TO64	0x268
#define PRCM_ARMITVAL127TO96	0x26C
#define PRCM_ARMITVAL159TO128	0x5DC
#define PRCM_ARMITVAL191TO160	0x5E0
#define PRCM_ARMITVAL223TO192	0x5E4

/* ARM WFI Standby signal register */
#define PRCM_ARM_WFI_STANDBY    0x130

/* IO force */
#define PRCM_IOCR		0x310
#define PRCM_IOCR_IOFORCE			0x1

#ifdef CONFIG_UX500_SUSPEND_DBG_WAKE_ON_UART
int ux500_console_uart_gpio_pin = CONFIG_UX500_CONSOLE_UART_GPIO_PIN;
#endif
static u32 u8500_gpio_banks[] = {U8500_GPIOBANK0_BASE,
				 U8500_GPIOBANK1_BASE,
				 U8500_GPIOBANK2_BASE,
				 U8500_GPIOBANK3_BASE,
				 U8500_GPIOBANK4_BASE,
				 U8500_GPIOBANK5_BASE,
				 U8500_GPIOBANK6_BASE,
				 U8500_GPIOBANK7_BASE,
				 U8500_GPIOBANK8_BASE};

static u32 ux500_gpio_wks[ARRAY_SIZE(u8500_gpio_banks)];

struct ux500_pm_resources {
	int nb_gic_regs;
	bool (*prcmu_pending_interrupt)(u32 *pending_irq);
	void (*prcmu_copy_gic_settings)(void);
};
static struct ux500_pm_resources *ux500_pm;

inline int ux500_pm_arm_on_ext_clk(bool leave_arm_pll_on)
{
	return 0;
}

/* Decouple GIC from the interrupt bus */
void ux500_pm_gic_decouple(void)
{
	prcmu_write_masked(PRCM_A9_MASK_REQ,
			   PRCM_A9_MASK_REQ_PRCM_A9_MASK_REQ,
			   PRCM_A9_MASK_REQ_PRCM_A9_MASK_REQ);

	(void)prcmu_read(PRCM_A9_MASK_REQ);

	/* TODO: Use the ack bit when possible */
	udelay(GIC_FREEZE_DELAY); /* Wait for the GIC to freeze */
}

/* Recouple GIC with the interrupt bus */
void ux500_pm_gic_recouple(void)
{
	prcmu_write_masked(PRCM_A9_MASK_REQ,
			   PRCM_A9_MASK_REQ_PRCM_A9_MASK_REQ,
			   0);

	/* TODO: Use the ack bit when possible */
}

/* return true if a GIC interrupt is enabled and pending */
bool ux500_pm_gic_pending_interrupt(u32 *pending_irq)
{
	u32 pr; /* Pending register */
	u32 er; /* Enable register */
	int i;

	BUG_ON(unlikely(!ux500_pm));

	/* 5 registers. STI & PPI not skipped */
	for (i = 0; i < ux500_pm->nb_gic_regs; i++) {

		pr = readl_relaxed(__io_address(U8500_GIC_DIST_BASE) +
				   GIC_DIST_PENDING_SET + i * 4);
		er = readl_relaxed(__io_address(U8500_GIC_DIST_BASE) +
				   GIC_DIST_ENABLE_SET + i * 4);

		if (pr & er) { /* Return first pending interrupt */
			if (pending_irq)
				*pending_irq = i * 32 + __ffs(pr & er);
			return true;
		}
	}
	return false;
}

/* return true if a PRCMU interrupt for A9s is enabled and pending */
bool ux500_pm_prcmu_pending_interrupt(u32 *pending_irq)
{
	BUG_ON(unlikely(!ux500_pm || !ux500_pm->prcmu_pending_interrupt));
	return ux500_pm->prcmu_pending_interrupt(pending_irq);
}
static bool u8500_pm_prcmu_pending_interrupt(u32 *pending_irq)
{
	u32 it;
	u32 im;
	int i;

	for (i = 0; i < U8500_GIC_DIST_SPI_REGS_NB; i++) { /* There are 4 registers */

		it = prcmu_read(PRCM_ARMITVAL31TO0 + i * 4);
		im = prcmu_read(PRCM_ARMITMSK31TO0 + i * 4);

		if (it & im) { /* Return first pending interrupt */
			if (pending_irq)
				*pending_irq = i * 32 + __ffs(it & im);
			return true;
		}
	}
	return false;
}
static bool ux540_pm_prcmu_pending_interrupt(u32 *pending_irq)
{
	u32 it;
	u32 im;
	int i, j;

	if (u8500_pm_prcmu_pending_interrupt(pending_irq))
		return true;

	i = U8500_GIC_DIST_SPI_REGS_NB;
	for (j = 0; i < U9540_GIC_DIST_SPI_REGS_NB; i++, j++) {

		it = prcmu_read(PRCM_ARMITVAL159TO128 + j * 4);
		im = prcmu_read(PRCM_ARMITMSK159TO128 + j * 4);
		if (it & im) { /* Return first pending interrupt */
			if (pending_irq)
				*pending_irq = i * 32 + __ffs(it & im);
			return true;
		}
	}
	return false;
}

void ux500_pm_prcmu_set_ioforce(bool enable)
{
	if (enable)
		prcmu_write_masked(PRCM_IOCR,
				   PRCM_IOCR_IOFORCE,
				   PRCM_IOCR_IOFORCE);
	else
		prcmu_write_masked(PRCM_IOCR,
				   PRCM_IOCR_IOFORCE,
				   0);
}

void ux500_pm_prcmu_copy_gic_settings(void)
{
	BUG_ON(unlikely(!ux500_pm || !ux500_pm->prcmu_copy_gic_settings));
	ux500_pm->prcmu_copy_gic_settings();
}
static void u8500_pm_prcmu_copy_gic_settings(void)
{
	u32 er; /* Enable register */
	int i;

	for (i = 0; i < U8500_GIC_DIST_SPI_REGS_NB; i++) { /* 4*32 SPI interrupts */
		/* +1 due to skip STI and PPI */
		er = readl_relaxed(__io_address(U8500_GIC_DIST_BASE) +
			   GIC_DIST_ENABLE_SET + (i + 1) * 4);
		prcmu_write(PRCM_ARMITMSK31TO0 + i * 4, er);
	}
}
static void ux540_pm_prcmu_copy_gic_settings(void)
{
	u32 er; /* Enable register */
	int i, j;

	u8500_pm_prcmu_copy_gic_settings();

	i = U8500_GIC_DIST_SPI_REGS_NB;
	for (j = 0; i < U9540_GIC_DIST_SPI_REGS_NB; i++, j++) {
		/* +1 due to skip STI and PPI */
		er = readl_relaxed(__io_address(U8500_GIC_DIST_BASE) +
			   GIC_DIST_ENABLE_SET + (i + 1) * 4);
		prcmu_write(PRCM_ARMITMSK159TO128 + j * 4, er);
	}
}

void ux500_pm_gpio_save_wake_up_status(void)
{
	int num_banks;
	u32 *banks;
	int i;

	num_banks = ARRAY_SIZE(u8500_gpio_banks);
	banks = u8500_gpio_banks;

	nmk_gpio_clocks_enable();

	for (i = 0; i < num_banks; i++)
		ux500_gpio_wks[i] = readl(__io_address(banks[i]) + NMK_GPIO_WKS);

	nmk_gpio_clocks_disable();
}

u32 ux500_pm_gpio_read_wake_up_status(unsigned int bank_num)
{
	if (bank_num >= ARRAY_SIZE(u8500_gpio_banks))
		return 0;

	return ux500_gpio_wks[bank_num];
}

/* Check if the other CPU is in WFI */
bool ux500_pm_other_cpu_wfi(void)
{
	if (smp_processor_id()) {
		/* We are CPU 1 => check if CPU0 is in WFI */
		if (prcmu_read(PRCM_ARM_WFI_STANDBY) &
		    PRCM_ARM_WFI_STANDBY_CPU0_WFI)
			return true;
	} else {
		/* We are CPU 0 => check if CPU1 is in WFI */
		if (prcmu_read(PRCM_ARM_WFI_STANDBY) &
		    PRCM_ARM_WFI_STANDBY_CPU1_WFI)
			return true;
	}

	return false;
}

static struct ux500_pm_resources u8500_pm_res = {
	.nb_gic_regs = U8500_GIC_DIST_REGS_NB,
	.prcmu_pending_interrupt = u8500_pm_prcmu_pending_interrupt,
	.prcmu_copy_gic_settings = u8500_pm_prcmu_copy_gic_settings,
};
static struct ux500_pm_resources ux540_pm_res = {
	.nb_gic_regs = U9540_GIC_DIST_REGS_NB,
	.prcmu_pending_interrupt = ux540_pm_prcmu_pending_interrupt,
	.prcmu_copy_gic_settings = ux540_pm_prcmu_copy_gic_settings,
};

static int __init ux500_pm_init(void)
{
	if (cpu_is_ux540_family())
		ux500_pm = &ux540_pm_res;
	else
		ux500_pm = &u8500_pm_res;
	return 0;
}
subsys_initcall(ux500_pm_init);
