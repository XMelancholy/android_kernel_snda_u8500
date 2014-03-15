/*
 * Copyright (C) ST Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 *
 * STE Ux500 PRCMU API
 */
#ifndef __MACH_PRCMU_H
#define __MACH_PRCMU_H

#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/err.h>

/* PRCMU Wakeup defines */
enum prcmu_wakeup_index {
	PRCMU_WAKEUP_INDEX_RTC,
	PRCMU_WAKEUP_INDEX_RTT0,
	PRCMU_WAKEUP_INDEX_RTT1,
	PRCMU_WAKEUP_INDEX_HSI0,
	PRCMU_WAKEUP_INDEX_HSI1,
	PRCMU_WAKEUP_INDEX_USB,
	PRCMU_WAKEUP_INDEX_ABB,
	PRCMU_WAKEUP_INDEX_ABB_FIFO,
	PRCMU_WAKEUP_INDEX_ARM,
	PRCMU_WAKEUP_INDEX_CD_IRQ,
	NUM_PRCMU_WAKEUP_INDICES
};
#define PRCMU_WAKEUP(_name) (BIT(PRCMU_WAKEUP_INDEX_##_name))

/* EPOD (power domain) IDs */

/*
 * DB8500 EPODs
 * - EPOD_ID_SVAMMDSP: power domain for SVA MMDSP
 * - EPOD_ID_SVAPIPE: power domain for SVA pipe
 * - EPOD_ID_SIAMMDSP: power domain for SIA MMDSP
 * - EPOD_ID_SIAPIPE: power domain for SIA pipe
 * - EPOD_ID_SGA: power domain for SGA
 * - EPOD_ID_B2R2_MCDE: power domain for B2R2 and MCDE
 * - EPOD_ID_ESRAM12: power domain for ESRAM 1 and 2
 * - EPOD_ID_ESRAM34: power domain for ESRAM 3 and 4
 * - NUM_EPOD_ID: number of power domains
 *
 * TODO: These should be prefixed.
 */
#define EPOD_ID_SVAMMDSP	0
#define EPOD_ID_SVAPIPE		1
#define EPOD_ID_SIAMMDSP	2
#define EPOD_ID_SIAPIPE		3
#define EPOD_ID_SGA		4
#define EPOD_ID_B2R2_MCDE	5
#define EPOD_ID_ESRAM12		6
#define EPOD_ID_ESRAM34		7
#define NUM_EPOD_ID		8

/*
 * state definition for EPOD (power domain)
 * - EPOD_STATE_NO_CHANGE: The EPOD should remain unchanged
 * - EPOD_STATE_OFF: The EPOD is switched off
 * - EPOD_STATE_RAMRET: The EPOD is switched off with its internal RAM in
 *                         retention
 * - EPOD_STATE_ON_CLK_OFF: The EPOD is switched on, clock is still off
 * - EPOD_STATE_ON: Same as above, but with clock enabled
 */
#define EPOD_STATE_NO_CHANGE	0x00
#define EPOD_STATE_OFF		0x01
#define EPOD_STATE_RAMRET	0x02
#define EPOD_STATE_ON_CLK_OFF	0x03
#define EPOD_STATE_ON		0x04

/*
 * CLKOUT sources
 */
#define PRCMU_CLKSRC_CLK38M		0x00
#define PRCMU_CLKSRC_ACLK		0x01
#define PRCMU_CLKSRC_SYSCLK		0x02
#define PRCMU_CLKSRC_LCDCLK		0x03
#define PRCMU_CLKSRC_SDMMCCLK		0x04
#define PRCMU_CLKSRC_TVCLK		0x05
#define PRCMU_CLKSRC_TIMCLK		0x06
#define PRCMU_CLKSRC_CLK009		0x07
/* These are only valid for CLKOUT1: */
#define PRCMU_CLKSRC_SIAMMDSPCLK	0x40
#define PRCMU_CLKSRC_I2CCLK		0x41
#define PRCMU_CLKSRC_MSP02CLK		0x42
#define PRCMU_CLKSRC_ARMPLL_OBSCLK	0x43
#define PRCMU_CLKSRC_HSIRXCLK		0x44
#define PRCMU_CLKSRC_HSITXCLK		0x45
#define PRCMU_CLKSRC_ARMCLKFIX		0x46
#define PRCMU_CLKSRC_HDMICLK		0x47

/*
 * Clock identifiers.
 */
enum prcmu_clock {
	PRCMU_SGACLK,
	PRCMU_UARTCLK,
	PRCMU_MSP02CLK,
	PRCMU_MSP1CLK,
	PRCMU_I2CCLK,
	PRCMU_SDMMCCLK,
	PRCMU_SPARE1CLK,
	PRCMU_SLIMCLK,
	PRCMU_PER1CLK,
	PRCMU_PER2CLK,
	PRCMU_PER3CLK,
	PRCMU_PER5CLK,
	PRCMU_PER6CLK,
	PRCMU_PER7CLK,
	PRCMU_LCDCLK,
	PRCMU_BMLCLK,
	PRCMU_HSITXCLK,
	PRCMU_HSIRXCLK,
	PRCMU_HDMICLK,
	PRCMU_APEATCLK,
	PRCMU_APETRACECLK,
	PRCMU_MCDECLK,
	PRCMU_IPI2CCLK,
	PRCMU_DSIALTCLK,
	PRCMU_DMACLK,
	PRCMU_B2R2CLK,
	PRCMU_TVCLK,
	PRCMU_SSPCLK,
	PRCMU_RNGCLK,
	PRCMU_UICCCLK,
	PRCMU_PWMCLK,
	PRCMU_IRDACLK,
	PRCMU_IRRCCLK,
	PRCMU_SIACLK,
	PRCMU_SVACLK,
	PRCMU_ACLK,
	/* HVA & G1 - U9540 only */
	PRCMU_HVACLK,
	PRCMU_G1CLK,
	PRCMU_SDMMCHCLK,
	PRCMU_CAMCLK,
	PRCMU_NUM_REG_CLOCKS,
	PRCMU_SYSCLK = PRCMU_NUM_REG_CLOCKS,
	PRCMU_CDCLK,
	PRCMU_TIMCLK,
	PRCMU_PLLSOC0,
	PRCMU_PLLSOC1,
	PRCMU_ARMSS,
	PRCMU_ARMCLK,
	PRCMU_PLLDDR,
	PRCMU_PLLDSI,
	PRCMU_DSI0CLK,
	PRCMU_DSI1CLK,
	PRCMU_DSI0ESCCLK,
	PRCMU_DSI1ESCCLK,
	PRCMU_DSI2ESCCLK,
	/* LCD DSI PLL - U9540 only */
	PRCMU_PLLDSI_LCD,
	PRCMU_DSI0CLK_LCD,
	PRCMU_DSI1CLK_LCD,
	PRCMU_DSI0ESCCLK_LCD,
	PRCMU_DSI1ESCCLK_LCD,
	PRCMU_DSI2ESCCLK_LCD,
};

/**
 * enum ape_opp - APE OPP states definition
 * @APE_OPP_INIT:
 * @APE_NO_CHANGE: The APE operating point is unchanged
 * @APE_100_OPP: The new APE operating point is ape100opp
 * @APE_50_OPP: 50%
 * @APE_50_PARTLY_25_OPP: 50%, except some clocks at 25%.
 */
enum ape_opp {
	APE_OPP_INIT = 0x00,
	APE_NO_CHANGE = 0x01,
	APE_100_OPP = 0x02,
	APE_50_OPP = 0x03,
	APE_50_PARTLY_25_OPP = 0xFF,
};

/**
 * enum arm_opp - ARM OPP states definition
 * @ARM_OPP_INIT:
 * @ARM_NO_CHANGE: The ARM operating point is unchanged
 * @ARM_100_OPP: The new ARM operating point is arm100opp
 * @ARM_50_OPP: The new ARM operating point is arm50opp
 * @ARM_MAX_OPP: Operating point is "max" (more than 100)
 * @ARM_MAX_FREQ100OPP: Set max opp if available, else 100
 * @ARM_EXTCLK: The new ARM operating point is armExtClk
 */
enum arm_opp {
	ARM_OPP_INIT = 0x00,
	ARM_NO_CHANGE = 0x01,
	ARM_100_OPP = 0x02,
	ARM_50_OPP = 0x03,
	ARM_MAX_OPP = 0x04,
	ARM_MAX_FREQ100OPP = 0x05,
	ARM_EXTCLK = 0x07
};

/**
 * enum ddr_opp - DDR OPP states definition
 * @DDR_100_OPP: The new DDR operating point is ddr100opp
 * @DDR_50_OPP: The new DDR operating point is ddr50opp
 * @DDR_25_OPP: The new DDR operating point is ddr25opp
 */
enum ddr_opp {
	DDR_100_OPP = 0x00,
	DDR_50_OPP = 0x01,
	DDR_25_OPP = 0x02,
};

/**
 * enum vsafe_opp - VSAFE OPP states definition
 * @VSAFE_100_OPP: The new VSAFE operating point is vsafe100opp
 * @VSAFE_50_OPP: The new DDR operating point is vsafe50opp
 */
enum vsafe_opp {
	VSAFE_OPP_INIT = 0x00,
	VSAFE_50_OPP = 0x01,
	VSAFE_100_OPP = 0x02,
};

/*
 * Definitions for controlling ESRAM0 in deep sleep.
 */
#define ESRAM0_DEEP_SLEEP_STATE_OFF 1
#define ESRAM0_DEEP_SLEEP_STATE_RET 2

/**
 * enum ddr_pwrst - DDR power states definition
 * @DDR_PWR_STATE_UNCHANGED: SDRAM and DDR controller state is unchanged
 * @DDR_PWR_STATE_ON:
 * @DDR_PWR_STATE_OFFLOWLAT:
 * @DDR_PWR_STATE_OFFHIGHLAT:
 */
enum ddr_pwrst {
	DDR_PWR_STATE_UNCHANGED     = 0x00,
	DDR_PWR_STATE_ON            = 0x01,
	DDR_PWR_STATE_OFFLOWLAT     = 0x02,
	DDR_PWR_STATE_OFFHIGHLAT    = 0x03
};

/**
 * enum ap_pwrst_trans - Transition states defined in PRCMU firmware
 * @NO_TRANSITION: No power state transition
 * @APEXECUTE_TO_APSLEEP: Power state transition from ApExecute to ApSleep
 * @APIDLE_TO_APSLEEP: Power state transition from ApIdle to ApSleep
 * @APEXECUTE_TO_APDEEPSLEEP: Power state transition from ApExecute to
 *                          ApDeepSleep
 * @APEXECUTE_TO_APIDLE: Power state transition from ApExecute to ApIdle
 */
enum ap_pwrst_trans {
	PRCMU_AP_NO_CHANGE		= 0x00,
	PRCMU_AP_SLEEP,
	PRCMU_AP_DEEP_SLEEP,
	PRCMU_AP_IDLE,
	PRCMU_AP_DEEP_IDLE,
};

/**
 * enum prcmu_power_status - results from set_power_state
 * @PRCMU_SLEEP_OK: Sleep went ok
 * @PRCMU_DEEP_SLEEP_OK: DeepSleep went ok
 * @PRCMU_IDLE_OK: Idle went ok
 * @PRCMU_DEEPIDLE_OK: DeepIdle went ok
 * @PRCMU_PRCMU2ARMPENDINGIT_ER: Pending interrupt detected
 * @PRCMU_ARMPENDINGIT_ER: Pending interrupt detected
 *
 */
enum prcmu_power_status {
	PRCMU_SLEEP_OK			= 0xf3,
	PRCMU_DEEP_SLEEP_OK		= 0xf6,
	PRCMU_IDLE_OK			= 0xf0,
	PRCMU_DEEPIDLE_OK		= 0xe3,
	PRCMU_PRCMU2ARMPENDINGIT_ER	= 0x91,
	PRCMU_ARMPENDINGIT_ER		= 0x93,
};

struct prcmu_tcdm_map {
	u32 tcdm_size;
	u32 legacy_offset;
};

struct db8500_prcmu_pdata
{
	struct cpufreq_frequency_table *cpufreq;
	int cpufreq_size;
	bool enable_set_ddr_opp;
};

struct dbx540_prcmu_pdata
{
	struct cpufreq_frequency_table *cpufreq;
	int cpufreq_size;
	bool enable_ape_opp_100_voltage;
};

#define PRCMU_FW_PROJECT_U8500		2
#define PRCMU_FW_PROJECT_U8400		3
#define PRCMU_FW_PROJECT_U9500		4 /* Customer specific */
#define PRCMU_FW_PROJECT_U8500_MBB	5
#define PRCMU_FW_PROJECT_U8500_C1	6
#define PRCMU_FW_PROJECT_U8500_C2	7
#define PRCMU_FW_PROJECT_U8500_C3	8
#define PRCMU_FW_PROJECT_U8500_C4	9
#define PRCMU_FW_PROJECT_U9500_MBL	10
#define PRCMU_FW_PROJECT_U8500_MBL	11 /* Customer specific */
#define PRCMU_FW_PROJECT_U8500_MBL2	12 /* Customer specific */
#define PRCMU_FW_PROJECT_U8520		13
#define PRCMU_FW_PROJECT_U8420		14

/* ux540 family prcmu fw version shall be shifted by 8 to differ with u8500 */
#define PRCMU_FW_PROJECT_U9540		(6<<8)
#define PRCMU_FW_PROJECT_A9420		20


#define PRCMU_FW_PROJECT_NAME_LEN	20
struct prcmu_fw_version {
	u32 project; /* Notice, project shifted with 8 on ux540 */
	u8 api_version;
	u8 func_version;
	u8 errata;
	char project_name[PRCMU_FW_PROJECT_NAME_LEN];
};

/*
 * Definitions for autonomous power management configuration.
 */

#define PRCMU_AUTO_PM_OFF 0
#define PRCMU_AUTO_PM_ON 1

#define PRCMU_AUTO_PM_POWER_ON_HSEM BIT(0)
#define PRCMU_AUTO_PM_POWER_ON_ABB_FIFO_IT BIT(1)

enum prcmu_auto_pm_policy {
	PRCMU_AUTO_PM_POLICY_NO_CHANGE,
	PRCMU_AUTO_PM_POLICY_DSP_OFF_HWP_OFF,
	PRCMU_AUTO_PM_POLICY_DSP_OFF_RAMRET_HWP_OFF,
	PRCMU_AUTO_PM_POLICY_DSP_CLK_OFF_HWP_OFF,
	PRCMU_AUTO_PM_POLICY_DSP_CLK_OFF_HWP_CLK_OFF,
};

/**
 * struct prcmu_auto_pm_config - Autonomous power management configuration.
 * @sia_auto_pm_enable: SIA autonomous pm enable. (PRCMU_AUTO_PM_{OFF,ON})
 * @sia_power_on:       SIA power ON enable. (PRCMU_AUTO_PM_POWER_ON_* bitmask)
 * @sia_policy:         SIA power policy. (enum prcmu_auto_pm_policy)
 * @sva_auto_pm_enable: SVA autonomous pm enable. (PRCMU_AUTO_PM_{OFF,ON})
 * @sva_power_on:       SVA power ON enable. (PRCMU_AUTO_PM_POWER_ON_* bitmask)
 * @sva_policy:         SVA power policy. (enum prcmu_auto_pm_policy)
 */
struct prcmu_auto_pm_config {
	u8 sia_auto_pm_enable;
	u8 sia_power_on;
	u8 sia_policy;
	u8 sva_auto_pm_enable;
	u8 sva_power_on;
	u8 sva_policy;
};


/* keep increasing ID from 0: used as table indices */
enum upap_nfy_id {
	UPAP_NFYID_C2C_NOTIF = 0,
	UPAP_NFYID_XMIP_RESOUT0N,
	UPAP_NFYID_XMIP_RESOUT2N,
	UPAP_NFYID_XMIP_MODAPP,
	UPAP_NFYID_MAX,
};

struct upap_modapp_evt {
	u32 modapp_itstatus;
	u32 modapp_linelevel;
};

struct upap_resoutxnlevel_evt {
	u32 resoutxn_level;
};

#ifdef CONFIG_MFD_DBX540_PRCMU

int upap_register_notifier(enum upap_nfy_id id, struct notifier_block *nb);
int upap_unregister_notifier(enum upap_nfy_id id, struct notifier_block *nb);

int prcmu_get_xmip_reset_n(void);
int prcmu_set_xmip_reset_n(int  state);
int prcmu_get_modem_resout2_n(void);
int prcmu_get_modem_resout0_n(void);
int prcmu_get_service_n(void);
int prcmu_set_service_n(int  state);
void prcmu_xmip_modapp_notif(u32 bitmask, u32 edgel, u32 edgeh);

#else

static inline int upap_register_notifier(enum upap_nfy_id id,
	struct notifier_block *nb)
{
	return -EINVAL;
}

static inline int upap_unregister_notifier(enum upap_nfy_id id,
	struct notifier_block *nb)
{
	return -EINVAL;
}

static inline int prcmu_get_xmip_reset_n(void)
{
	return -EINVAL;
}

static inline int prcmu_set_xmip_reset_n(int  state)
{
	return -EINVAL;
}

static inline int prcmu_get_modem_resout0_n(void)
{
	return -EINVAL;
}

static inline int prcmu_get_modem_resout2_n(void)
{
	return -EINVAL;
}
static inline int prcmu_get_service_n(void)
{
	return -EINVAL;
}
static inline int prcmu_set_service_n(int  state)
{
	return -EINVAL;
}
static inline void prcmu_xmip_modapp_notif(u32 bitmask, u32 edgel, u32 edgeh) {}
#endif

#if defined(CONFIG_UX500_SOC_DB8500)

void prcmu_early_init(struct prcmu_tcdm_map *map, bool is_ux540_family);

int prcmu_set_power_state(u8 state, bool keep_ulp_clk, bool keep_ap_pll);
u8 prcmu_get_power_state_result(void);
void prcmu_enable_wakeups(u32 wakeups);
void prcmu_disable_wakeups(void);

void prcmu_config_abb_event_readout(u32 abb_events);

void prcmu_get_abb_event_buffer(void __iomem **buf);

int prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size);
int prcmu_abb_read_no_irq(u8 slave, u8 reg, u8 *value, u8 size);
int prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size);
int prcmu_abb_write_masked(u8 slave, u8 reg, u8 *value, u8 *mask, u8 size);

int prcmu_config_a9wdog(u8 num, bool sleep_auto_off);
int prcmu_enable_a9wdog(u8 id);
int prcmu_disable_a9wdog(u8 id);
int prcmu_kick_a9wdog(u8 id);
int prcmu_load_a9wdog(u8 id, u32 timeout);

int prcmu_config_clkout(u8 clkout, u8 source, u8 div);

int prcmu_request_clock(u8 clock, bool enable);

unsigned long prcmu_clock_rate(u8 clock);
long prcmu_round_clock_rate(u8 clock, unsigned long rate);
int prcmu_set_clock_rate(u8 clock, unsigned long rate);

int prcmu_set_ddr_opp(u8 opp);

int prcmu_get_arm_opp(void);

int prcmu_set_ape_opp(u8 opp);

int prcmu_get_ape_opp(void);

int prcmu_set_vsafe_opp(u8 opp);

void prcmu_set_sdmmc_psw(bool status);

void prcmu_c2c_request_notif_up(void);

int prcmu_register_modem(char *name);

int prcmu_unregister_modem(char *name);

void prcmu_c2c_request_reset(void);

int prcmu_ac_wake_req(void);

void prcmu_ac_sleep_req(void);

int prcmu_get_vsafe_opp(void);

bool prcmu_check_ape_age(void);

void prcmu_system_reset(u16 reset_code);

u16 prcmu_get_reset_code(void);

u32 prcmu_get_reset_status(void);

void prcmu_modem_reset(void);

int prcmu_set_epod(u16 epod_id, u8 epod_state);

bool prcmu_is_ac_wake_requested(void);

int prcmu_set_display_clocks(void);

int prcmu_disable_dsipll(void);

int prcmu_enable_dsipll(void);

int prcmu_disable_dsipll(void);

int prcmu_enable_dsipll(void);

int prcmu_config_esram0_deep_sleep(u8 state);

u32 prcmu_read(unsigned int reg);

void prcmu_write(unsigned int reg, u32 value);

void prcmu_write_masked(unsigned int reg, u32 mask, u32 value);

int prcmu_stay_in_wfi_check(void);

int prcmu_unplug_cpu1(void);

int prcmu_replug_cpu1(void);



/* prcmu_get_val /prcmu_set_val */
enum prcmu_val {
	DDR_OPP,
	DDR1_OPP,
	EFF_DDR_OPP,
	EFF_DDR1_OPP,
	ARM_OPP,
	APE_OPP,

	PRCMU_VAL_MAX /*  used for dimensioning */
};

int prcmu_set_val(enum prcmu_val type, u32 value);
int prcmu_get_val(enum prcmu_val type);
int prcmu_get_ddr_opp(enum prcmu_val type);

/*  prcmu_enable/prcmu_disable */
enum prcmu_out {
	SPI2_MUX,
	STM_MOD_UART_MUX,
	STM_APE_MUX,

	PRCMU_OUT_MAX /* used for dimensioning  */
};

int prcmu_enable(enum prcmu_out out);
int prcmu_disable(enum prcmu_out out);


struct prcmu_out_data {
	enum prcmu_out out;
	int (*enable)(void);
	int (*disable)(void);
};

struct prcmu_val_data {
	enum prcmu_val val;
	int (*get_val)(void);
	int (*set_val)(u8 value);
};


/**
 * @brief mfd device dbx500-prmcu early fops
 */
struct prcmu_early_data {
	/* reset */
	void (*system_reset) (u16 reset_code);

	/*  clock api  */
	int (*config_clkout) (u8 clkout, u8 source, u8 div);
	int (*request_clock) (u8 clock, bool enable);

	/* direct access to prcmu reg */
	u32 (*read) (unsigned int reg);
	void (*write) (unsigned int reg, u32 value);
	void (*write_masked) (unsigned int reg, u32 mask, u32 value);


	/*  other specific 8500 */
	long (*round_clock_rate) (u8 clock, unsigned long rate);
	int (*set_clock_rate) (u8 clock, unsigned long rate);
	unsigned long (*clock_rate) (u8 clock);
	/*  clock specific */
	void (*vc) (bool enable);
};

/**
 * @brief mfd device dbx500-prmcu platform data
 */
struct prcmu_probe_data {
	/* ux500 soc sysfs */
	u16 (*get_reset_code) (void);


	/* pm/suspend.c  */
	int (*config_esram0_deep_sleep) (u8 state);
	void (*enable_wakeups)(u32 wakeups);
	bool (*is_ac_wake_requested) (void);
	int (*set_power_state) (u8 state, bool keep_ulp_clk,
			bool keep_ap_pll);
	u8  (*get_power_state_result) (void);
	bool (*trace_pins_enabled) (int bank);

	/* modem */
	void (*modem_reset)(void);

	/*  regulator */
	int (*set_epod)(u16 epod_id, u8 epod_state);

	/* no used at all */
	void (*config_abb_event_readout) (u32 abb_events);
	void (*get_abb_event_buffer) (void __iomem **buf);

	/* abb access */
	int (*abb_read) (u8 slave, u8 reg, u8 *value, u8 size);
	int (*abb_read_no_irq) (u8 slave, u8 reg, u8 *value, u8 size);
	int (*abb_write) (u8 slave, u8 reg, u8 *value, u8 size);

	u32 (*get_reset_status)(void);
	/*  other u8500 specific */
	int (*request_ape_opp_100_voltage) (bool enable);
	void (*configure_auto_pm) (struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle);
	/* abb specific */
	int (*abb_write_masked) (u8 slave, u8 reg, u8 *value,
		u8 *mask, u8 size);
	/* watchdog */
	int (*config_a9wdog) (u8 num, bool sleep_auto_off);
	int (*enable_a9wdog) (u8 id);
	int (*disable_a9wdog) (u8 id);
	int (*kick_a9wdog) (u8 id);
	int (*load_a9wdog) (u8 id, u32 val);
};

/* on u8500 default behaviour return 0 */
struct prcmu_probe_ux540_data {
	int (*stay_in_wfi_check)(void);
	int (*replug_cpu1) (void);
	int (*unplug_cpu1) (void);
};


enum prcmu_fops_type {
	PRCMU_VAL,
	PRCMU_OUT,
	PRCMU_EARLY,
	PRCMU_PROBE,
	PRCMU_PROBE_UX540,
	PRCMU_APE_AGE,
};

struct prcmu_fops_register {
	enum prcmu_fops_type fops;
	int size;
	union {
		struct prcmu_val_data *pval;
		struct prcmu_out_data *pout;
		struct prcmu_early_data *pearly;
		struct prcmu_probe_data *pprobe;
		struct prcmu_probe_ux540_data *pprobeux540;
		bool (*check_ape_age)(void);
	} data;
};
/**
 * @brief mfd device dbx500-prcmu platform data
 */
struct prcmu_fops_register_data {
	int size;
	struct prcmu_fops_register *tab;
};

/**
 * struct dbx500_regulator_init_data - mfd device prcmu-regulators data
 *
 */
struct dbx500_regulator_init_data {
	int (*set_epod) (u16 epod_id, u8 epod_state);
	void *regulators;
	int reg_num;
};

void prcmu_vc(bool enable);

struct prcmu_fw_version *prcmu_get_fw_version(void);
int prcmu_request_ape_opp_100_voltage(bool enable);
void prcmu_configure_auto_pm(struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle);

int dbx500_prcmu_early_init(struct prcmu_fops_register_data *data);

#else

static inline void __init prcmu_early_init(struct prcmu_tcdm_map *map) {}


#endif


#if defined(CONFIG_UX500_SOC_DB8500)

/*
 * prcmu_enable_spi2 - Enables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_enable_spi2(void);

/**
 * prcmu_disable_spi2 - Disables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_disable_spi2(void);
/**
 * prcmu_enable_stm_mod_uart - Enables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
void prcmu_enable_stm_mod_uart(void);

/**
 * prcmu_disable_stm_mod_uart - Disables pin muxing for STMMOD
 * and UARTMOD on OtherAlternateC3.
 */
void prcmu_disable_stm_mod_uart(void);

/**
 * prcmu_enable_stm_ape - Enables pin muxing for STM APE on OtherAlternateC1.
 */
void prcmu_enable_stm_ape(void);

/**
 * prcmu_disable_stm_ape - Disables pin muxing for STM APE on OtherAlternateC1.
 */
void prcmu_disable_stm_ape(void);

#endif

/**
 * prcmu_trace_pins_enabled - Check if debugger trace is enabled for a certain
 * GPIO bank.
 */
bool prcmu_trace_pins_enabled(int bank);

/* PRCMU QoS APE OPP class */
#define PRCMU_QOS_APE_OPP 1
#define PRCMU_QOS_DDR_OPP 2
#define PRCMU_QOS_ARM_KHZ 3
#define PRCMU_QOS_VSAFE_OPP 4
#define PRCMU_QOS_ARM_MAX_FREQ  5
#define PRCMU_QOS_DEFAULT_VALUE -1
#define PRCMU_QOS_MAX_VALUE -2
#define PRCMU_QOS_ARM_KHZ_MAX PRCMU_QOS_MAX_VALUE
#define PRCMU_QOS_DDR_OPP_MAX PRCMU_QOS_MAX_VALUE
#define PRCMU_QOS_APE_OPP_MAX PRCMU_QOS_MAX_VALUE

#ifdef CONFIG_DBX500_PRCMU_QOS_POWER

unsigned long prcmu_qos_get_cpufreq_opp_delay(void);
void prcmu_qos_set_cpufreq_opp_delay(unsigned long);
void prcmu_qos_force_opp(int, s32);
void prcmu_qos_show_requirement(struct seq_file *s, u32 verbose);
int prcmu_qos_requirement(int pm_qos_class);
int prcmu_qos_add_requirement(int pm_qos_class, const char *name, s32 value);
int prcmu_qos_update_requirement(int pm_qos_class, const char *name, s32 new_value);
void prcmu_qos_remove_requirement(int pm_qos_class, const char *name);
int prcmu_qos_add_notifier(int prcmu_qos_class,
			   struct notifier_block *notifier);
int prcmu_qos_remove_notifier(int prcmu_qos_class,
			      struct notifier_block *notifier);
void prcmu_qos_voice_call_override(bool enable);
int prcmu_qos_lpa_override(bool enable);

#else

static inline unsigned long prcmu_qos_get_cpufreq_opp_delay(void)
{
	return 0;
}

static inline void prcmu_qos_set_cpufreq_opp_delay(unsigned long n) {}

static inline void prcmu_qos_force_opp(int prcmu_qos_class, s32 i) {}

static inline void prcmu_qos_show_requirement(struct seq_file *s,
					u32 verbose) {}

static inline int prcmu_qos_requirement(int prcmu_qos_class)
{
	return 0;
}

static inline int prcmu_qos_add_requirement(int prcmu_qos_class,
					const char *name, s32 value)
{
	return 0;
}

static inline int prcmu_qos_update_requirement(int prcmu_qos_class,
					const char *name, s32 new_value)
{
	return 0;
}

static inline void prcmu_qos_remove_requirement(int prcmu_qos_class,
					const char *name)
{
}

static inline int prcmu_qos_add_notifier(int prcmu_qos_class,
					 struct notifier_block *notifier)
{
	return 0;
}
static inline int prcmu_qos_remove_notifier(int prcmu_qos_class,
					    struct notifier_block *notifier)
{
	return 0;
}
static inline void prcmu_qos_voice_call_override(bool enable) {}

static inline int prcmu_qos_lpa_override(bool enable)
{
	return 0;
}
#endif

#endif /* __MACH_PRCMU_H */
