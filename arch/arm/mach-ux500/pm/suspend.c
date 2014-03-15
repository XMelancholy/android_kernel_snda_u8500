/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010-2011
 *
 * License Terms: GNU General Public License v2
 *
 * Authors: Rickard Andersson <rickard.andersson@stericsson.com>,
 *	    Jonas Aaberg <jonas.aberg@stericsson.com>,
 *          Sundar Iyer for ST-Ericsson.
 */

#include <linux/suspend.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/ab8500-debug.h>
#include <linux/regulator/dbx500-prcmu.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>

#include <plat/gpio-nomadik.h>

#include <mach/context.h>
#include <mach/pm.h>

#include "../id.h"

#include "suspend_dbg.h"

#define UX500_SUSPEND_NO_IRQ UINT_MAX

static void (*pins_suspend_force)(void);
static void (*pins_suspend_force_mux)(void);

static suspend_state_t suspend_state = PM_SUSPEND_ON;
static u32 suspend_wakeups;
static u32 running_wakeups;
static u32 wake_cause_interrupt = UX500_SUSPEND_NO_IRQ;

void suspend_set_pins_force_fn(void (*force)(void), void (*force_mux)(void))
{
	pins_suspend_force = force;
	pins_suspend_force_mux = force_mux;
}

static atomic_t block_sleep = ATOMIC_INIT(0);

void suspend_block_sleep(void)
{
	atomic_inc(&block_sleep);
}

void suspend_unblock_sleep(void)
{
	atomic_dec(&block_sleep);
}

bool suspend_sleep_is_blocked(void)
{
	return (atomic_read(&block_sleep) != 0);
}

bool suspend_test_wake_cause_interrupt_is_mine(u32 my_int)
{
	if (my_int == wake_cause_interrupt) {
		wake_cause_interrupt = UX500_SUSPEND_NO_IRQ;
		return true;
	}
	return false;
}

static int suspend(bool do_deepsleep)
{
	bool pins_force = pins_suspend_force_mux && pins_suspend_force;
	int ret = 0;
	u32 pending_irq;
	bool keep_ulp_clk = false, keep_ap_pll = false;
	u8 state;
	if (suspend_sleep_is_blocked()) {
		pr_info("suspend/resume: interrupted by modem.\n");
		return -EBUSY;
	}

	if (has_wake_lock(WAKE_LOCK_SUSPEND)) {
		pr_info("suspend/resume: aborted. wakelock has been taken.\n");
		return -EBUSY;
	}

	nmk_gpio_clocks_enable();

	ux500_suspend_dbg_add_wake_on_uart();

	nmk_gpio_wakeups_suspend();

	/* configure the prcm for a sleep wakeup */
	prcmu_enable_wakeups(suspend_wakeups);

	ux500_suspend_dbg_test_set_wakeup();

	/*
	 * In deep debug mode, vape can be kept switch on
	 * in order to simulate idle and deep idle state
	 */
	if (ux500_suspend_ape())
		context_vape_save();

	if (pins_force && ux500_suspend_ape()) {
		/*
		 * Save GPIO settings before applying power save
		 * settings
		 */
		context_gpio_save();

		/* Apply GPIO power save mux settings */
		context_gpio_mux_safe_switch(true);
		pins_suspend_force_mux();
		context_gpio_mux_safe_switch(false);

		/* Apply GPIO power save settings */
		pins_suspend_force();
	}

	ux500_pm_gic_decouple();

	/* Copy GIC interrupt settings to PRCMU interrupt settings */
	ux500_pm_prcmu_copy_gic_settings();

	ret = ux500_pm_gic_pending_interrupt(&pending_irq);
	if (ret) {
		pr_info("suspend/resume: pending interrupt %u\n",
			pending_irq);

		/* Recouple GIC with the interrupt bus */
		ux500_pm_gic_recouple();
		ret = -EBUSY;

		goto exit;
	}

	if (ux500_pm_prcmu_pending_interrupt(&pending_irq)) {
		pr_info("suspend/resume: pending interrupt prcmu: %u\n",
				pending_irq);

		/* Recouple GIC with the interrupt bus */
		ux500_pm_gic_recouple();
		ret = -EBUSY;

		goto exit;
	}

	if (ux500_suspend_ape())
		ux500_pm_prcmu_set_ioforce(true);

	if (do_deepsleep) {
		context_varm_save_common();
		context_varm_save_core();
		context_gic_dist_disable_unneeded_irqs();
		context_save_cpu_registers();

		/*
		 * Due to we have only 100us between requesting a powerstate
		 * and wfi, we clean the cache before as well to assure the
		 * final cache clean before wfi has as little as possible to
		 * do.
		 */
		context_clean_l1_cache_all();
		state = PRCMU_AP_DEEP_SLEEP;
		/*  modify condition if hats test */
		ux500_suspend_dbg_cond(&state, &keep_ulp_clk, &keep_ap_pll);
		(void) prcmu_set_power_state(state, keep_ulp_clk, keep_ap_pll);
		context_save_to_sram_and_wfi(true);

		context_restore_cpu_registers();
		context_varm_restore_core();
		context_varm_restore_common();

	} else {
		context_clean_l1_cache_all();
		state = PRCMU_AP_SLEEP;
		/*  modify condition if hats test */
		ux500_suspend_dbg_cond(&state, &keep_ulp_clk, &keep_ap_pll);
		(void) prcmu_set_power_state(state, keep_ulp_clk, keep_ap_pll);
		dsb();
		__asm__ __volatile__("wfi\n\t" : : : "memory");
	}

	if (ux500_suspend_ape())
		context_vape_restore();

	/* If GPIO woke us up then save the pins that caused the wake up */
	ux500_pm_gpio_save_wake_up_status();

	ux500_suspend_dbg_sleep_status(do_deepsleep);

	/* APE was turned off, restore IO ring */
	if (ux500_suspend_ape())
		ux500_pm_prcmu_set_ioforce(false);

	wake_cause_interrupt = UX500_SUSPEND_NO_IRQ;
	ux500_pm_gic_pending_interrupt(&wake_cause_interrupt);

exit:
	if (pins_force && ux500_suspend_ape()) {
		/* Restore gpio settings */
		context_gpio_mux_safe_switch(true);
		context_gpio_restore_mux();
		context_gpio_mux_safe_switch(false);
		context_gpio_restore();
	}

	/* Configure the prcmu with the wake-ups that cpuidle needs */
	prcmu_enable_wakeups(running_wakeups);

	nmk_gpio_wakeups_resume();

	ux500_suspend_dbg_remove_wake_on_uart();

	nmk_gpio_clocks_disable();

	return ret;
}

static int ux500_suspend_enter(suspend_state_t state)
{
	if (ux500_suspend_enabled()) {
		if (ux500_suspend_deepsleep_enabled() &&
		    state == PM_SUSPEND_MEM)
			return suspend(true);
		if (ux500_suspend_sleep_enabled())
			return suspend(false);
	}

	ux500_suspend_dbg_add_wake_on_uart();

	ux500_pm_prcmu_set_ioforce(true);

	dsb();
	__asm__ __volatile__("wfi\n\t" : : : "memory");

	ux500_pm_prcmu_set_ioforce(false);
	ux500_suspend_dbg_remove_wake_on_uart();

	return 0;
}

static int ux500_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY;
}

static int ux500_suspend_prepare(void)
{
	int ret;

	ret = regulator_suspend_prepare(suspend_state);
	if (ret < 0)
		return ret;

	return 0;
}

static int ux500_suspend_prepare_late(void)
{
	/* ESRAM to retention instead of OFF until ROM is fixed */
	(void) prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);

	ab8500_regulator_debug_force();
	ux500_regulator_suspend_debug();
	return 0;
}

static void ux500_suspend_wake(void)
{
	ux500_regulator_resume_debug();
	ab8500_regulator_debug_restore();
	(void) prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);
}

static void ux500_suspend_finish(void)
{
	(void)regulator_suspend_finish();
}

static int ux500_suspend_begin(suspend_state_t state)
{
	(void) prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					    "suspend",
					    PRCMU_QOS_ARM_KHZ_MAX);

	suspend_state = state;
	return ux500_suspend_dbg_begin(state);
}

static void ux500_suspend_end(void)
{
	(void) prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					    "suspend",
					    PRCMU_QOS_DEFAULT_VALUE);
	suspend_state = PM_SUSPEND_ON;

	ux500_suspend_dbg_end();
}

static struct platform_suspend_ops ux500_suspend_ops = {
	.enter	      = ux500_suspend_enter,
	.valid	      = ux500_suspend_valid,
	.prepare      = ux500_suspend_prepare,
	.prepare_late = ux500_suspend_prepare_late,
	.wake	      = ux500_suspend_wake,
	.finish       = ux500_suspend_finish,
	.begin	      = ux500_suspend_begin,
	.end	      = ux500_suspend_end,
};

static __init int ux500_suspend_init(void)
{
	if (machine_is_a9500() || machine_is_u9540()) {
		suspend_wakeups = (PRCMU_WAKEUP(ABB) | PRCMU_WAKEUP(RTC) |
				   PRCMU_WAKEUP(HSI0));
		running_wakeups = (PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) |
				   PRCMU_WAKEUP(ABB) | PRCMU_WAKEUP(HSI0));
	} else {
		suspend_wakeups = PRCMU_WAKEUP(ABB) | PRCMU_WAKEUP(RTC);
		running_wakeups = (PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) |
				   PRCMU_WAKEUP(ABB));
	}

	ux500_suspend_dbg_init();
	prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ,
				  "suspend",
				  PRCMU_QOS_DEFAULT_VALUE);
	suspend_set_ops(&ux500_suspend_ops);
	return 0;
}
device_initcall(ux500_suspend_init);
