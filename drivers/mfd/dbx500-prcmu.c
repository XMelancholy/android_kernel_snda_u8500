/*
 * Copyright (C) ST-Ericsson SA 2010. All rights reserved.
 * This code is ST-Ericsson proprietary and confidential.
 * Any use of the code for whatever purpose is subject to
 * specific written permission of ST-Ericsson SA.
 *
 * Author: Michel Jaouen <michel.jaouen@stericsson.com> for
 * ST-Ericsson.
 * License terms: GNU Gereral Public License (GPL) version 2
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/hwmon.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/io.h>

#include <mach/hardware.h>

#include "db8500-prcmu.h"
#include "dbx540-prcmu.h"
#include "dbx500-prcmu.h"

/* Offset for the firmware version within the TCPM */
#define DB8500_PRCMU_FW_VERSION_OFFSET 0xA4
#define DBX540_PRCMU_FW_VERSION_OFFSET 0xA8

static struct {
	bool valid;
	struct prcmu_fw_version version;
} fw_info;

static __init char *fw_project_name(u32 project)
{
	switch (project) {
	case PRCMU_FW_PROJECT_U8500:
		return "U8500";
	case PRCMU_FW_PROJECT_U8400:
		return "U8400";
	case PRCMU_FW_PROJECT_U9500:
		return "U9500";
	case PRCMU_FW_PROJECT_U8500_MBB:
		return "U8500 MBB";
	case PRCMU_FW_PROJECT_U8500_C1:
		return "U8500 C1";
	case PRCMU_FW_PROJECT_U8500_C2:
		return "U8500 C2";
	case PRCMU_FW_PROJECT_U8500_C3:
		return "U8500 C3";
	case PRCMU_FW_PROJECT_U8500_C4:
		return "U8500 C4";
	case PRCMU_FW_PROJECT_U9500_MBL:
		return "U9500 MBL";
	case PRCMU_FW_PROJECT_U8500_MBL:
		return "U8500 MBL";
	case PRCMU_FW_PROJECT_U8500_MBL2:
		return "U8500 MBL2";
	case PRCMU_FW_PROJECT_U8520:
		return "U8520 MBL";
	case PRCMU_FW_PROJECT_U8420:
		return "U8420";
	case PRCMU_FW_PROJECT_U9540:
		return "U9540";
	case PRCMU_FW_PROJECT_A9420:
		return "A9420";
	default:
		return "Unknown";
	}
}

struct prcmu_fw_version *prcmu_get_fw_version(void)
{
	return fw_info.valid ? &fw_info.version : NULL;
}

#define dbx500_prcmu_warn(a) do {\
	printk(KERN_WARN "%s : dbx500-prcmu driver %s",\
	__func__, a);\
	} while (0)


#define dbx500_prcmu_error(a) do {\
	printk(KERN_ERR "%s : dbx500-prcmu driver %s",\
	__func__, a);\
	} while (0)

#define dbx500_prcmu_early_trap_void do {\
	printk(KERN_ERR "%s called :dbx500-prcmu driver not initialized",\
		__func__); \
	} while (0)


#define dbx500_prcmu_early_trap(a) do {\
	printk(KERN_ERR "%s called :dbx500-prcmu driver not initialized",\
		__func__);\
	return a;\
	} while (0)

#define dbx500_prcmu_trap(a)	do {\
	printk(KERN_ERR "%s called : dbx500-prcmu driver not probed",\
	__func__);\
	return a;\
	} while (0)

#define dbx500_prcmu_trap_void do {\
	printk(KERN_ERR "%s called : dbx500-prcmu driver not probed",\
	__func__);\
	} while (0)

/* dummy handler */

static int dummy_set_power_state(u8 state, bool keep_ulp_clk,
		bool keep_ap_pll) {
	dbx500_prcmu_trap(-EINVAL);
}

static u8  dummy_get_power_state_result(void)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_config_clkout(u8 clkout, u8 source, u8 div)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static int dummy_request_clock(u8 clock, bool enable)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static long dummy_round_clock_rate(u8 clock, unsigned long rate)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static int dummy_set_clock_rate(u8 clock, unsigned long rate)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	unsigned long dummy_clock_rate(u8 clock)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	int dummy_set_val(enum prcmu_val type, u8 value)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	int dummy_get_val(enum prcmu_val type)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	void dummy_system_reset(u16 reset_code)
{
	dbx500_prcmu_early_trap_void;
}
static	u16 dummy_get_reset_code(void)
{
	dbx500_prcmu_trap(-EINVAL);
}
static	u32 dummy_get_reset_status(void)
{
	dbx500_prcmu_trap(-EINVAL);
}

static	void dummy_enable_wakeups(u32 wakeups)
{
	dbx500_prcmu_trap_void;
}

static	bool dummy_is_ac_wake_requested(void)
{
	dbx500_prcmu_trap(false);
}

static	int  dummy_disable(enum prcmu_out out)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	int  dummy_enable(enum prcmu_out out)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	u32  dummy_read(unsigned int reg)
{
	dbx500_prcmu_early_trap(-EINVAL);
}

static	void  dummy_write(unsigned int reg, u32 value)
{
	dbx500_prcmu_early_trap_void;
}

static bool dummy_check_ape_age(void)
{
	return true;
}

static	void  default_write_masked(unsigned int reg, u32 mask, u32 value)
{
	u32 val;
	val = readl(_PRCMU_BASE + reg);
	val = ((val & ~mask) | (value & mask));
	writel(val, (_PRCMU_BASE + reg));
}

static	int  dummy_config_esram0_deep_sleep(u8 state)
{
	dbx500_prcmu_trap(-EINVAL);
}

static	void  dummy_config_abb_event_readout(u32 abb_events)
{
	dbx500_prcmu_trap_void;
}

static	void  dummy_get_abb_event_buffer(void __iomem **buf)
{
	dbx500_prcmu_trap_void;
}

static int  dummy_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int  dummy_abb_read_no_irq(u8 slave, u8 reg, u8 *value, u8 size)
{
	dbx500_prcmu_trap(-EINVAL);
}

static	int  dummy_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	dbx500_prcmu_trap(-EINVAL);
}

static	int  dummy_abb_write_masked(u8 slave, u8 reg, u8 *value,
		u8 *mask, u8 size)
{
	dbx500_prcmu_trap(-EINVAL);
}

static	void  dummy_modem_reset(void)
{
	dbx500_prcmu_trap_void;
}

static int dummy_set_epod(u16 epod_id, u8 epod_state)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_request_ape_opp_100_voltage(bool enable)
{
	dbx500_prcmu_trap(-EINVAL);
}

static void dummy_configure_auto_pm(struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle)
{
	dbx500_prcmu_trap_void;
}

static int dummy_config_a9wdog(u8 num, bool sleep_auto_off)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_enable_a9wdog(u8 id)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_disable_a9wdog(u8 id)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_kick_a9wdog(u8 id)
{
	dbx500_prcmu_trap(-EINVAL);
}

static int dummy_load_a9wdog(u8 id, u32 timeout)
{
	dbx500_prcmu_trap(-EINVAL);
}

static void dummy_vc(bool enable)
{
	dbx500_prcmu_early_trap_void;
}

static bool dummy_trace_pins_enabled(int bank)
{
	dbx500_prcmu_trap(-EINVAL);
}

struct prcmu_probe_data dummy_fops = {
	/* sysfs soc inf */
	.get_reset_code = dummy_get_reset_code,

	/* pm/suspend.c/cpu freq */
	.config_esram0_deep_sleep = dummy_config_esram0_deep_sleep,
	.set_power_state = dummy_set_power_state,
	.get_power_state_result = dummy_get_power_state_result,
	.enable_wakeups = dummy_enable_wakeups,
	.is_ac_wake_requested = dummy_is_ac_wake_requested,
	.trace_pins_enabled = dummy_trace_pins_enabled,

	/* modem */
	.modem_reset = dummy_modem_reset,

	/*  regulator */
	.set_epod = dummy_set_epod,

	/* no used at all */
	.config_abb_event_readout = dummy_config_abb_event_readout,
	.get_abb_event_buffer = dummy_get_abb_event_buffer,

	/* abb access */
	.abb_read = dummy_abb_read,
	.abb_read_no_irq = dummy_abb_read_no_irq,
	.abb_write = dummy_abb_write,
	.get_reset_status = dummy_get_reset_status,
	/*  other u8500 specific */
	.request_ape_opp_100_voltage = dummy_request_ape_opp_100_voltage,
	.configure_auto_pm = dummy_configure_auto_pm,

	/* abb specific access */
	.abb_write_masked = dummy_abb_write_masked,

	/* watchdog */
	.config_a9wdog = dummy_config_a9wdog,
	.enable_a9wdog = dummy_enable_a9wdog,
	.disable_a9wdog = dummy_disable_a9wdog,
	.kick_a9wdog = dummy_kick_a9wdog,
	.load_a9wdog = dummy_load_a9wdog,
};

static struct prcmu_early_data default_early_fops = {
	/*  system reset  */
	.system_reset = dummy_system_reset,

	/*  clock service */
	.config_clkout = dummy_config_clkout,
	.request_clock = dummy_request_clock,

	/*  direct register access */
	.read = dummy_read,
	.write =  dummy_write,
	.write_masked = default_write_masked,
	/* others */
	.round_clock_rate = dummy_round_clock_rate,
	.set_clock_rate = dummy_set_clock_rate,
	.clock_rate = dummy_clock_rate,
	.vc = dummy_vc,
};

static int dummy_return_null(void)
{
	return 0;
}

static struct prcmu_probe_ux540_data dummy_fops_ux540 = {
	. stay_in_wfi_check = dummy_return_null,
	. replug_cpu1 = dummy_return_null,
	. unplug_cpu1 = dummy_return_null,
};

static struct {
	struct prcmu_early_data *pearly;
	struct prcmu_probe_data *pprobe;
	struct prcmu_probe_ux540_data *pprobeux540;
	struct prcmu_val_data tab_val[PRCMU_VAL_MAX];
	int (*set_val)(enum prcmu_val type, u8 val);
	int (*get_val) (enum prcmu_val type);
	struct prcmu_out_data tab_out[PRCMU_OUT_MAX];
	int (*disable) (enum prcmu_out out);
	int (*enable) (enum prcmu_out out);
	bool (*check_ape_age)(void);
} dbx500_prcmu_context = {
	.pearly = &default_early_fops,
	.pprobe = &dummy_fops,
	.pprobeux540 = &dummy_fops_ux540,
	.set_val = dummy_set_val,
	.get_val = dummy_get_val,
	.disable = dummy_disable,
	.enable = dummy_enable,
	.check_ape_age = dummy_check_ape_age,
};

/* early service */

void prcmu_system_reset(u16 reset_code)
{
	dbx500_prcmu_context.pearly->system_reset(reset_code);
}

u32 prcmu_read(unsigned int reg)
{
	return	dbx500_prcmu_context.pearly->read(reg);
}

void prcmu_write(unsigned int reg, u32 value)
{
	return	dbx500_prcmu_context.pearly->write(reg, value);
}

void prcmu_write_masked(unsigned int reg, u32 mask, u32 value)
{
	 dbx500_prcmu_context.pearly->write_masked(reg, mask, value);
}

int prcmu_config_clkout(u8 clkout, u8 source, u8 div)
{
	return dbx500_prcmu_context.pearly->config_clkout(clkout, source, div);
}

int prcmu_request_clock(u8 clock, bool enable)
{
	return  dbx500_prcmu_context.pearly->request_clock(clock, enable);
}

unsigned long prcmu_clock_rate(u8 clock)
{
	return   dbx500_prcmu_context.pearly->clock_rate(clock);
}

long prcmu_round_clock_rate(u8 clock, unsigned long rate)
{
	return  dbx500_prcmu_context.pearly->round_clock_rate(clock, rate);
}

int prcmu_set_clock_rate(u8 clock, unsigned long rate)
{
	return  dbx500_prcmu_context.pearly->set_clock_rate(clock, rate);
}

int prcmu_set_val(enum prcmu_val type, u32 value)
{
	return dbx500_prcmu_context.set_val(type, value);
}

int prcmu_get_val(enum prcmu_val type)
{
	return dbx500_prcmu_context.get_val(type);
}

int prcmu_enable_out(enum prcmu_out out)
{
	return dbx500_prcmu_context.enable(out);
}

int prcmu_disable_out(enum prcmu_out out)
{
	return dbx500_prcmu_context.disable(out);
}

int prcmu_set_ddr_opp(u8 opp)
{
	return dbx500_prcmu_context.set_val(DDR_OPP, opp);
}

int prcmu_get_ddr_opp(enum prcmu_val type)
{
	return dbx500_prcmu_context.get_val(type);
}

int prcmu_get_arm_opp(void)
{
	return dbx500_prcmu_context.get_val(ARM_OPP);
}

int prcmu_set_ape_opp(u8 opp)
{
	return dbx500_prcmu_context.set_val(APE_OPP, opp);
}

int prcmu_get_ape_opp(void)
{
	return dbx500_prcmu_context.get_val(APE_OPP);
}

bool prcmu_check_ape_age(void)
{
	return dbx500_prcmu_context.check_ape_age();
}

/*  other service available after the probe */

int prcmu_set_power_state(u8 state, bool keep_ulp_clk,
		bool keep_ap_pll)
{
	return	 dbx500_prcmu_context.pprobe->set_power_state(state,
			keep_ulp_clk,
			keep_ap_pll);
}

u8 prcmu_get_power_state_result(void)
{
	return	dbx500_prcmu_context.pprobe->get_power_state_result();
}

void prcmu_enable_wakeups(u32 wakeups)
{
	dbx500_prcmu_context.pprobe->enable_wakeups(wakeups);
}

void prcmu_disable_wakeups(void)
{
	dbx500_prcmu_context.pprobe->enable_wakeups(0);
}

void prcmu_config_abb_event_readout(u32 abb_events)
{
	dbx500_prcmu_context.pprobe->config_abb_event_readout(abb_events);
}

void prcmu_get_abb_event_buffer(void __iomem **buf)
{
	dbx500_prcmu_context.pprobe->get_abb_event_buffer(buf);
}

u16 prcmu_get_reset_code(void)
{
	return dbx500_prcmu_context.pprobe->get_reset_code();
}

void prcmu_modem_reset(void)
{
	dbx500_prcmu_context.pprobe->modem_reset();
}

int  prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	return dbx500_prcmu_context.pprobe->abb_read(slave, reg, value, size);
}

int  prcmu_abb_read_no_irq(u8 slave, u8 reg, u8 *value, u8 size)
{
	return dbx500_prcmu_context.pprobe->abb_read_no_irq(slave, reg, \
			value, size);
}

int  prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	return dbx500_prcmu_context.pprobe->abb_write(slave, reg, value, size);
}

int  prcmu_abb_write_masked(u8 slave, u8 reg, u8 *value,
		u8 *mask, u8 size)
{
	return dbx500_prcmu_context.pprobe->abb_write_masked(
			slave, reg, value, mask, size);
}

u32  prcmu_get_reset_status(void)
{
	return dbx500_prcmu_context.pprobe->get_reset_status();
}

int  prcmu_replug_cpu1(void)
{
	return dbx500_prcmu_context.pprobeux540->replug_cpu1();
}

int prcmu_stay_in_wfi_check(void)
{
	return dbx500_prcmu_context.pprobeux540->stay_in_wfi_check();
}

int prcmu_unplug_cpu1(void)
{
	return dbx500_prcmu_context.pprobeux540->unplug_cpu1();
}

bool prcmu_is_ac_wake_requested(void)
{
	return dbx500_prcmu_context.pprobe->is_ac_wake_requested();
}

int prcmu_config_esram0_deep_sleep(u8 state)
{
	 return dbx500_prcmu_context.pprobe->config_esram0_deep_sleep(state);
}

int prcmu_set_epod(u16 epod_id, u8 epod_state)
{
	return dbx500_prcmu_context.pprobe->set_epod(epod_id, epod_state);
}

int prcmu_config_a9wdog(u8 num, bool sleep_auto_off)
{
	return dbx500_prcmu_context.pprobe->config_a9wdog(num, sleep_auto_off);
}

int prcmu_enable_a9wdog(u8 id)
{
	return dbx500_prcmu_context.pprobe->enable_a9wdog(id);
}

int prcmu_disable_a9wdog(u8 id)
{
	return dbx500_prcmu_context.pprobe->disable_a9wdog(id);
}

int prcmu_kick_a9wdog(u8 id)
{
	return dbx500_prcmu_context.pprobe->kick_a9wdog(id);
}

int prcmu_load_a9wdog(u8 id, u32 timeout)
{
	return dbx500_prcmu_context.pprobe->load_a9wdog(id, timeout);
}

/**
 * prcmu_enable_spi2 - Enables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_enable_spi2(void)
{
	 dbx500_prcmu_context.enable(SPI2_MUX);
}

/**
 * prcmu_disable_spi2 - Disables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_disable_spi2(void)
{
	dbx500_prcmu_context.disable(SPI2_MUX);
}

/**
 * prcmu_enable_stm_mod_uart - Enables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
void prcmu_enable_stm_mod_uart(void)
{
	dbx500_prcmu_context.enable(STM_MOD_UART_MUX);
}

/**
 * prcmu_disable_stm_mod_uart - Disables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
void prcmu_disable_stm_mod_uart(void)
{
	dbx500_prcmu_context.disable(STM_MOD_UART_MUX);
}

/**
 * prcmu_enable_stm_ape - Enables pin muxing for STM APE on OtherAlternateC1.
 */
void prcmu_enable_stm_ape(void)
{
	dbx500_prcmu_context.enable(STM_APE_MUX);
}

/**
 * prcmu_disable_stm_ape - Disables pin muxing for STM APE on OtherAlternateC1.
 */
void prcmu_disable_stm_ape(void)
{
	dbx500_prcmu_context.disable(STM_APE_MUX);
}

bool prcmu_trace_pins_enabled(int bank)
{
	return dbx500_prcmu_context.pprobe->trace_pins_enabled(bank);
}

void prcmu_configure_auto_pm(struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle)
{
	dbx500_prcmu_context.pprobe->configure_auto_pm(sleep, idle);
}
EXPORT_SYMBOL(prcmu_configure_auto_pm);

int prcmu_request_ape_opp_100_voltage(bool enable)
{
	return	dbx500_prcmu_context.
		pprobe->request_ape_opp_100_voltage(enable);
}
void prcmu_vc(bool enable)
{
	dbx500_prcmu_context.pearly->vc(enable);
}

static int dbx500_prcmu_set_val(enum prcmu_val type, u8 value)
{
	if (type < PRCMU_VAL_MAX)
		return dbx500_prcmu_context.tab_val[type].set_val(value);
	dbx500_prcmu_error("request out of range");
		return -EIO;

}

static int dbx500_prcmu_get_val(enum prcmu_val type)
{
	if (type < PRCMU_VAL_MAX)
		return dbx500_prcmu_context.tab_val[type].get_val();
	dbx500_prcmu_error("request out of range");
		return -EIO;

}

static int dbx500_prcmu_enable_out(enum prcmu_out out)
{
	if (out < PRCMU_OUT_MAX)
		return dbx500_prcmu_context.tab_out[out].enable();
	dbx500_prcmu_error("request out of range");
		return -EIO;
}

static int dbx500_prcmu_disable_out(enum prcmu_out out)
{
	if (out < PRCMU_OUT_MAX)
		return	dbx500_prcmu_context.tab_out[out].disable();
	dbx500_prcmu_error("request out of range");
		return -EIO;
}

int prcmu_set_display_clocks(void)
{
	return db8500_prcmu_set_display_clocks();
}

int prcmu_disable_dsipll(void)
{
	return db8500_prcmu_disable_dsipll();
}

int prcmu_enable_dsipll(void)
{
	return db8500_prcmu_enable_dsipll();
}

/*  used for enable , disable and get */
static int dbx500_default_handler(void)
{
	return 0;
}

static int dbx500_default_getval(void)
{
	return -EIO;
}

static int dbx500_default_set(u8 val)
{
	return 0;
}
static int default_get_ape_opp(void)
{
	return APE_100_OPP;
}

static int default_get_ddr_opp(void)
{
	return DDR_100_OPP;
}

static struct prcmu_val_data default_ape = {
	.set_val = dbx500_default_set,
	.get_val = default_get_ape_opp,
};


static struct prcmu_val_data default_ddr = {
	.set_val = dbx500_default_set,
	.get_val = default_get_ddr_opp,
};

static struct prcmu_out_data default_out = {
	.enable = dbx500_default_handler,
	.disable = dbx500_default_handler,
};

static struct prcmu_val_data default_val = {
	.set_val = dbx500_default_set,
	.get_val = dbx500_default_getval,
};

static void dbx500_prcmu_init_ctx(void)
{
	int i;
	struct prcmu_val_data *pdefault;
	for (i = 0; i < PRCMU_VAL_MAX; i++) {
		switch (i) {
		case DDR_OPP:
			pdefault = &default_ddr;
			break;
		case APE_OPP:
			pdefault = &default_ape;
			break;
		default:
			pdefault = &default_val;
		}

		memcpy(&dbx500_prcmu_context.tab_val[i], pdefault,
				sizeof(struct prcmu_val_data));
	}

	for (i = 0; i < PRCMU_OUT_MAX; i++)
		memcpy(&dbx500_prcmu_context.tab_out[i], &default_out,
				sizeof(struct prcmu_out_data));
	dbx500_prcmu_context.enable = dbx500_prcmu_enable_out;
	dbx500_prcmu_context.disable = dbx500_prcmu_disable_out;
	dbx500_prcmu_context.set_val = dbx500_prcmu_set_val;
	dbx500_prcmu_context.get_val = dbx500_prcmu_get_val;
}

static void dbx500_prcmu_register_pout(struct prcmu_out_data *data, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (data[i].out < PRCMU_OUT_MAX)
			memcpy(&dbx500_prcmu_context.tab_out[data[i].out],
				&data[i], sizeof(struct prcmu_out_data));
		else
			dbx500_prcmu_error("ops out of range");
}

static void dbx500_prcmu_register_pval(struct prcmu_val_data *data, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (data[i].val < PRCMU_VAL_MAX)
			memcpy(&dbx500_prcmu_context.tab_val[data[i].val],
				&data[i], sizeof(struct prcmu_val_data));
		else
			dbx500_prcmu_error("registering ops out of range");
}

/**
 * @brief register prcmu handler
 *
 * @param fops
 */
void __init prcmu_early_init(struct prcmu_tcdm_map *map, bool is_ux540_family)
{
	int i, ret = 0;
	struct prcmu_fops_register_data *data;

	void *tcpm_base = ioremap_nocache(U8500_PRCMU_TCPM_BASE, SZ_4K);

	if (tcpm_base != NULL) {
		u32 version;
		int project_shift = 0;

		if (is_ux540_family) {
			project_shift = 8;
			version = readl(tcpm_base +
					DBX540_PRCMU_FW_VERSION_OFFSET);
		} else {
			version = readl(tcpm_base +
					DB8500_PRCMU_FW_VERSION_OFFSET);
		}

		fw_info.version.project = (version & 0xFF) << project_shift;
		fw_info.version.api_version = (version >> 8) & 0xFF;
		fw_info.version.func_version = (version >> 16) & 0xFF;
		fw_info.version.errata = (version >> 24) & 0xFF;

		strncpy(fw_info.version.project_name,
			fw_project_name(fw_info.version.project),
			PRCMU_FW_PROJECT_NAME_LEN);

		fw_info.valid = true;
		pr_info("PRCMU firmware: %s(%d), version %d.%d.%d\n",
			fw_info.version.project_name,
			fw_info.version.project >> project_shift,
			fw_info.version.api_version,
			fw_info.version.func_version,
			fw_info.version.errata);

		iounmap(tcpm_base);
	}

	if (is_ux540_family)
		data =  dbx540_prcmu_early_init(map);
	else
		data = db8500_prcmu_early_init(map);

	if (data == NULL)
		return;

	dbx500_prcmu_init_ctx();

	for (i = 0; i < data->size; i++) {
		switch (data->tab[i].fops) {
		case PRCMU_EARLY:
			dbx500_prcmu_context.pearly = data->tab[i].data.pearly;
			break;
		case PRCMU_VAL:
			dbx500_prcmu_register_pval(data->tab[i].data.pval,
					data->tab[i].size);
			break;
		case PRCMU_OUT:
			dbx500_prcmu_register_pout(data->tab[i].data.pout,
					data->tab[i].size);
			break;
		default:
			dbx500_prcmu_error("ops out of range");
			ret = -EIO;
		}
	}
	return;
}

/**
 * @brief dbx500-prcmu probe function
 *
 * @param pdev
 *
 * @return
 */
static int __devinit dbx500_prcmu_probe(struct platform_device *pdev)
{
	struct prcmu_fops_register_data *data = dev_get_platdata(&pdev->dev);
	int i, ret = 0;
	for (i = 0; i < data->size; i++) {
		switch (data->tab[i].fops) {
		case PRCMU_VAL:
			dbx500_prcmu_register_pval(data->tab[i].data.pval,
					data->tab[i].size);
			break;
		case PRCMU_OUT:
			dbx500_prcmu_register_pout(data->tab[i].data.pout,
					data->tab[i].size);
			break;
		case PRCMU_PROBE:
			dbx500_prcmu_context.pprobe =
				data->tab[i].data.pprobe;
			break;
		case PRCMU_PROBE_UX540:
			dbx500_prcmu_context.pprobeux540 =
				data->tab[i].data.pprobeux540;
			break;
		case PRCMU_APE_AGE:
			dbx500_prcmu_context.check_ape_age =
				data->tab[i].data.check_ape_age;
			break;
		default:
			dbx500_prcmu_error("ops out of range");
			ret = -EIO;
		}
	}
	return ret;
}

/* No action required in suspend/resume, thus the lack of functions */
static struct platform_driver dbx500_prcmu_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "dbx500-prcmu",
	},
	.probe = dbx500_prcmu_probe,
};

static int __init dbx500_prcmu_init(void)
{
	return platform_driver_register(&dbx500_prcmu_driver);
}

MODULE_AUTHOR("Michel JAOUEN <michel.jaouen@stericsson.com>");
MODULE_DESCRIPTION("DBX500 PRCMU DRIVER");
MODULE_LICENSE("GPL");

arch_initcall(dbx500_prcmu_init);

