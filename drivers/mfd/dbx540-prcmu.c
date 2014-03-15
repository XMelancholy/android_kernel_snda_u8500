/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License Terms: GNU General Public License v2
 * Author: Michel Jaouen <michel.jaouen@stericsson.com>
 * Author: Alexandre Torgue <alexandre.torgues@stericsson.com>
 * Author: David Paris <david.paris@stericsson.com>
 * Author: Etienne Carriere <etienne.carriere@stericsson.com>
 * Author: Guillaume KOUADIO CARRY <guillaume.kouadio-carry@stericsson.com>
 * DBX540 PRCM Unit interface driver
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/mfd/core.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mfd/dbx500_temp.h>
#include <linux/regulator/db8500-prcmu.h>
#include <linux/regulator/machine.h>
#include <linux/ux500-pasr.h>
#include <linux/mfd/abx500.h>
#include <linux/platform_data/ux500_wdt.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/db8500-regs.h>
#include <mach/hardware.h>
#include <mach/prcmu-debug.h>
#include "dbx500-prcmu.h"
#include "dbx540-prcmu.h"
#include "dbx500-prcmu-regs.h"
#include "dbx540-prcmu-regs.h"
#include "dbx500-prcmu-trace.h"

/* Global var to runtime determine TCDM base for v2 or v1 */
static __iomem void *tcdm_legacy_base;
static __iomem void *tcdm_base;

/* mailbox definition */
static struct mb0_transfer mb0;
static struct mb2_transfer mb2;
static struct mb3_transfer mb3;
static struct mb4_transfer mb4;
static struct mb5_transfer mb5;

/* storage of currently supported modem type */
enum modem_type {
	MODEM_NONE,
	MODEM_C2C,
	MODEM_XMIP,
};
static enum modem_type connected_modem;

#define PRCM_BOOT_STATUS	0xFFF

#define PRCM_SW_RST_REASON 0xFF8 /* 2 bytes */

#define PRCM_TCDM_VOICE_CALL_FLAG 0xDD4 /* 4 bytes */


/*  TODO: check size from xp70 API */
#define UPAP_REQ_DATA_SZ (4 * 6)
#define UPAP_NFY_DATA_SZ (4 * 3)

/*
 * UniqPAP (Request/Response/Notify) - U9540
 */
struct upap_arm_opp_req_data {
	u32 freq;
	u16 volt;
	u16 bias;
	u16 vbbp;
	u16 vbbn;
};

struct upap_sprcmuapi_xmip_notifrequest {
	u32 resout0n_notif_req;
	u32 resout2n_notif_req;
	u32 modapp_enablebits;
	u32 modapp_edgesens_h;
	u32 modapp_edgesens_l;
};

struct upap_sprcmuapi_xmip_resoutxnlevel {
	u32 resoutxn_level;
};

struct upap_sprcmuapi_xmip_notifdata {
	u32 modapp_itstatus;
	u32 modapp_linelevel;
};

struct upap_req {
	u32 req_state;
	u32 service_id;
	u32 command_id;
	u32 status;
	union {
		u32 data; /*  default: single 32bit data */
		struct upap_sprcmuapi_xmip_notifrequest xmip_evt_data;
		struct upap_arm_opp_req_data arm_opp;
		u8 full_data_buf[UPAP_REQ_DATA_SZ];
	} data;
};

struct upap_nfy {
	u32 nfy_state;
	u32 service_id;
	u32 command_id;
	union {
		u32 data; /*  default: single 32bit data */
		u8 full_data_buf[UPAP_NFY_DATA_SZ];
	} data;
};

/*  UniqPAP timeout */
#define UPAP_TIM (HZ/10)

/* UniqPAP Configuration */
static struct upap_configuration {
	struct upap_req *req;
	struct upap_nfy *nfy;
	u8 mbox_nb;
} upap_conf;

enum upap_req_state {
	UX540_PRCM_UPAP_REQ_STATE_REQ_IDLE = 0,
	UX540_PRCM_UPAP_REQ_STATE_REQ_SENT,
	UX540_PRCM_UPAP_REQ_STATE_REQ_EXECUTING,
	UX540_PRCM_UPAP_REQ_STATE_ACK_SENT,
};

enum upap_nfy_state {
	UX540_PRCM_UPAP_NFY_STATE_IDLE = 0,
	UX540_PRCM_UPAP_NFY_STATE_ONGOING,
	UX540_PRCM_UPAP_NFY_STATE_SENT,
};

enum upap_service {
	UX540_PRCM_UPAP_SERVICE_DDR = 0,
	UX540_PRCM_UPAP_SERVICE_DVFS,
	UX540_PRCM_UPAP_SERVICE_MODEM,
	UX540_PRCM_UPAP_SERVICE_USB,
	UX540_PRCM_UPAP_SERVICE_CLOCK,
	UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
	UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG,
	UX540_PRCM_UPAP_SERVICE_THSENSOR,
	UPAP_SERVICES_NB,
};

enum upap_command {
	/* req/resp commands */
	U9540_PRCM_UPAP_COMMAND_SET_ARM_OPP = 0x1002,
	U9540_PRCM_UPAP_COMMAND_SET_APE_OPP = 0x1003,
	U9540_PRCM_UPAP_COMMAND_SET_SAFE_OPP = 0x1004,
	U9540_PRCM_UPAP_COMMAND_DDR_SLEEP_STRAT = 0x2005,
	U9540_PRCM_UPAP_COMMAND_RESET_MODEM = 0x3001,
	U9540_PRCM_UPAP_COMMAND_USB_WAKEUP_REL = 0x4001,
	U9540_PRCM_UPAP_COMMAND_PLL_ON_OFF = 0x5001,
	U9540_PRCM_UPAP_COMMAND_C2CINIT = 0x6001,
	U8540_PRCM_UPAP_COMMAND_XMIPMODEM_STARTING = 0x6001,
	U9540_PRCM_UPAP_COMMAND_C2CNOTIFYME = 0x6002,
	U8540_PRCM_UPAP_COMMAND_XMIP_NOTIFICATION = 0x6002,
	U9540_PRCM_UPAP_COMMAND_C2CTESTWAKEUP = 0x6003,
	U9540_PRCM_UPAP_COMMAND_C2CTESTSLEEP = 0x6004,
	U9540_PRCM_UPAP_COMMAND_C2CRESET = 0x6005,
	U9540_PRCM_UPAP_COMMAND_CPU1_UNPLUG = 0x7001,
	U9540_PRCM_UPAP_COMMAND_CPU1_REPLUG = 0x7002,
	U9540_PRCM_UPAP_COMMAND_THSENSOR_GET_TEMP = 0x8001,
	/* nfy commands */
	U9540_PRCM_UPAP_COMMAND_C2CNOTIFICATION = 0x601,
	U8540_PRCM_UPAP_COMMAND_XMIPRESOUT0N = 0x601,
	U8540_PRCM_UPAP_COMMAND_XMIPRESOUT2N = 0x602,
	U8540_PRCM_UPAP_COMMAND_XMIPMODAPP = 0x603,
};

/*
 * Table of convertion between index in registered notifiers table
 * and related PRCMU fmw UPAP-Nfy IDs (serviceID/commandID).
 * Table indices based on "enum upap_nfy_id".
 */
static u32 upap_nfy_cmds[UPAP_NFYID_MAX][2] = {
	[UPAP_NFYID_C2C_NOTIF] = {
		UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
		U9540_PRCM_UPAP_COMMAND_C2CNOTIFICATION
	},
	[UPAP_NFYID_XMIP_RESOUT0N] = {
		UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
		U8540_PRCM_UPAP_COMMAND_XMIPRESOUT0N
	},
	[UPAP_NFYID_XMIP_RESOUT2N] = {
		UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
		U8540_PRCM_UPAP_COMMAND_XMIPRESOUT2N
	},
	[UPAP_NFYID_XMIP_MODAPP] = {
		UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
		U8540_PRCM_UPAP_COMMAND_XMIPMODAPP
	},
};

/*
 * Notifier list for PRCMU events.
 * Index based on "enum upap_nfy_id".
 */
struct atomic_notifier_head prcmu_nfy_notifiers[UPAP_NFYID_MAX];

enum upap_status {
	U9540_PRCM_UPAP_STATUS_OK = 0,
	/* all non-0 IDs below report an error */
	U9540_PRCM_UPAP_STATUS_UNKNOWN_CMD_ID,
	U9540_PRCM_UPAP_STATUS_BAD_PARAM,
	U9540_PRCM_UPAP_STATUS_PARTIAL_SELF_REFRESH_DDR_EXEC,
	U9540_PRCM_UPAP_STATUS_QOS_DDR_EXEC,
	U9540_PRCM_UPAP_STATUS_SET_ARM_OPP_EXEC,
	U9540_PRCM_UPAP_STATUS_SET_ARM_OPP_INVAL,
	U9540_PRCM_UPAP_STATUS_SET_APE_OPP_EXEC,
	U9540_PRCM_UPAP_STATUS_SET_APE_OPP_INVAL,
	U9540_PRCM_UPAP_STATUS_SET_SAFE_OPP_EXEC,
	U9540_PRCM_UPAP_STATUS_SET_SAFE_OPP_INVAL,
	U9540_PRCM_UPAP_STATUS_DVFS_PLL_NOT_LOCKED,
	U9540_PRCM_UPAP_STATUS_C2C_UNKNOWN_ERR,
	U9540_PRCM_UPAP_STATUS_BAD_STATE,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_ALRDY_UNPLUGED,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_NOT_UNPLUGED,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_SECURE_ROM_ERR,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_UNKNOWN_ERR,
	U9540_PRCM_UPAP_STATUS_INVALID_STATE,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_ARMVOK_TIMEOUT,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_ROMCODESAVEOWNCTX_ERR,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_WAKEUPNORESP_ROM_ERR,
	U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_RESPLSNOTDSTOREADY,
	U9540_PRCM_UPAP_STATUS_OVERFLOW,
	U9540_PRCM_UPAP_STATUS_BUSY,
	U9540_PRCM_UPAP_STATUS_SET_ARM_OPP_FREQ_ERR,
	U9540_PRCM_UPAP_STATUS_THSENSOR_ALL_READY,
};

enum upap_ape_opp_ids {
	U9540_PRCM_REQ_UPAP_APE_OPP_1 = 1,
	U9540_PRCM_REQ_UPAP_APE_OPP_2,
};

enum upap_pll_on_off_ids {
	U9540_PRCM_REQ_UPAP_PLL_SOC0_OFF = 1,
	U9540_PRCM_REQ_UPAP_PLL_SOC0_ON	= 2,
	U9540_PRCM_REQ_UPAP_PLL_SOC1_OFF = 4,
	U9540_PRCM_REQ_UPAP_PLL_SOC1_ON	= 8,
};

enum upap_vsafe_opp_ids {
	U9540_PRCM_REQ_UPAP_VSAFE_OPP0 = 0,
	U9540_PRCM_REQ_UPAP_VSAFE_OPP1,
	U9540_PRCM_REQ_UPAP_VSAFE_OPP2,
};

enum uupap_c2c_ids {
	U9540_PRCM_UPAP_NFYDAT_C2CNOTIF_OK = 0x601,
	U9540_PRCM_REQ_DATA_C2C_NOTIFYME = 0x601,
};

/* UniqPAP Acknowledgement data: data copied from upap buffer */
struct upap_ack {
	u32 service_id;
	u32 command_id;
	u32 status;
	u32 arm_freq;
	u32 sensor_read;
};

enum upap_resout0n_ids {
	U8540_PRCM_UPAP_XMIPRESOUT0N_NOTIFNOTREQ = 0,
	U8540_PRCM_UPAP_XMIPRESOUT0N_NOTIFREQUESTED,
};

enum upap_resout2n_ids {
	U8540_PRCM_UPAP_XMIPRESOUT2N_NOTIFNOTREQ = 0,
	U8540_PRCM_UPAP_XMIPRESOUT2N_NOTIFREQUESTED,
};

/*
 * upap_transfer - state needed for UniqPAP communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 * @ape_opp:	The current APE OPP.
 * @arm_freq:	The current ARM Freq (U9540 only)
 * @ack:	Reply ("acknowledge") data. Structure used selected at run-
 *		time based on chip-set detected.
 */
static struct {
	struct mutex lock;
	struct completion work;
	struct upap_ack *ack;
} upap_transfer;

/*
 * dvfs_transfer - PRCMU need to save some dvfs context
 * @ape_opp:	The current APE OPP.
 * @arm_freq:	The current ARM Freq (U9540 only)
 * @vsafe_opp:  The current Vsafe state (U9540 only)
 */
struct {
	u8 ape_opp;
	u32 arm_freq;
	u8 vsafe_opp;
} dvfs_context;

static void (*upap_read_services[UPAP_SERVICES_NB])(struct upap_req *req,
		struct upap_ack *ack);

static int cpu1_unplug_ongoing;
static int prcmu_driver_initialised;
static int set_arm_freq(u32 freq);
static int get_arm_freq(void);

static unsigned long latest_armss_rate;

static bool enable_ape_opp_100_voltage;

/*The timer time-base is in nano-seconde*/
#define TIME_NS 1000000000ULL
/* profiling cycle time (in second)*/
#define PROFILING_CYCLE_TIME 4ULL
/* STORE_CYCLE = TIME_NS*PROFILING_CYCLE_TIME in NS*/
#define STORE_CYCLE (TIME_NS * PROFILING_CYCLE_TIME)
/* 9540 aging in second (8 years by default)*/
#define	DB9540_AGING 252288000ULL
/* 9540 aging in nano-second*/
#define	DB9540_AGING_TRADE (DB9540_AGING * TIME_NS)

/* SecMap is at 0x300 from tcdm_legacy_base adress*/
#define PRCMU_SECMAP 0x0300
/* InitOppData is at 0x598 from SecMap */
#define PRCM_INIT_OPP_DATA (PRCMU_SECMAP + 0x0598)
/* OPP0 table is at 0x60 from InitOppData */
#define PRCMU_OPP0_TABLE (PRCM_INIT_OPP_DATA + 0x0060)
/* OPP0 enable/disable is at 0x8 from OPP0 table*/
#define PRCMU_OPP0_IS_ENABLE (PRCMU_OPP0_TABLE + 0x0008)

/*
 * define 1G5 for the processor age threshold
 * Processor age counter must be updated for any frequency
 * higher than 1500000KHZ
 */
#define FREQ_1G5 1500000

struct prcmu_fw_freq_config {
	u32 arm_freq;
	u16 varm;
	u16 varm_transistor;
	u16 enable;
	u16 vbbp;
	u16 vbbn;
	u16 padding;
};

struct max_opp_profile {
	u32 last_arm_opp;
	u64 max_opp_cnt;
	u64 secure_memory;
	u64 cumul;
	u64 start;
};

static struct max_opp_profile arm_max_opp_profile = {
	.last_arm_opp = 0,
	.max_opp_cnt = 0,
	.secure_memory = 0,
	.cumul = 0,
	.start = 0,
};

static atomic_t ac_wake_req_state = ATOMIC_INIT(0);

/* Spinlocks */
static DEFINE_SPINLOCK(prcmu_lock);
static DEFINE_SPINLOCK(clkout_lock);
static DEFINE_SPINLOCK(spare_out_lock);

/*
 * Copies of the startup values of the reset status register and the SW reset
 * code.
 */
static u32 reset_status_copy;
static u16 reset_code_copy;

static DEFINE_SPINLOCK(clk_mgt_lock);

#define CLK_MGT_ENTRY(_name, _branch, _clk38div)[PRCMU_##_name] = \
	{ (PRCM_##_name##_MGT), 0 , _branch, _clk38div}
static struct clk_mgt clk_mgt[PRCMU_NUM_REG_CLOCKS] = {
	CLK_MGT_ENTRY(SGACLK, PLL_DIV, false),
	CLK_MGT_ENTRY(UARTCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(MSP02CLK, PLL_FIX, true),
	CLK_MGT_ENTRY(MSP1CLK, PLL_FIX, true),
	CLK_MGT_ENTRY(I2CCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(SDMMCCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(SLIMCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(PER1CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER2CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER3CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER5CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER6CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(PER7CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(LCDCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(BMLCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HSITXCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HSIRXCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(HDMICLK, PLL_FIX, false),
	CLK_MGT_ENTRY(APEATCLK, PLL_DIV, true),
	CLK_MGT_ENTRY(APETRACECLK, PLL_DIV, true),
	CLK_MGT_ENTRY(MCDECLK, PLL_DIV, true),
	CLK_MGT_ENTRY(IPI2CCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(DSIALTCLK, PLL_FIX, false),
	CLK_MGT_ENTRY(DMACLK, PLL_DIV, true),
	CLK_MGT_ENTRY(ACLK, PLL_DIV, true),
	CLK_MGT_ENTRY(B2R2CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(TVCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(SSPCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(RNGCLK, PLL_FIX, true),
	CLK_MGT_ENTRY(UICCCLK, PLL_FIX, false),
	CLK_MGT_ENTRY(HVACLK, PLL_DIV, true),
	CLK_MGT_ENTRY(G1CLK, PLL_DIV, true),
	CLK_MGT_ENTRY(SPARE1CLK, PLL_FIX, true),
	CLK_MGT_ENTRY(SDMMCHCLK, PLL_DIV, false),
	CLK_MGT_ENTRY(CAMCLK, PLL_FIX, true),
};

struct dsiclk {
	u32 divsel_mask;
	u32 divsel_shift;
	u32 divsel;
	u32 divsel_lcd_mask; /* For LCD DSI PLL supported by U9540 */
};

static struct dsiclk u9540_dsiclk[2] = {
	{
		.divsel_mask =
			U9540_PRCM_DSI_PLLOUT_SEL_DSI0_PLLOUT_DIVSEL_MASK,
		.divsel_shift = PRCM_DSI_PLLOUT_SEL_DSI0_PLLOUT_DIVSEL_SHIFT,
		.divsel = PRCM_DSI_PLLOUT_SEL_PHI,
		.divsel_lcd_mask = BIT(3),
	},
	{
		.divsel_mask =
			U9540_PRCM_DSI_PLLOUT_SEL_DSI1_PLLOUT_DIVSEL_MASK,
		.divsel_shift = PRCM_DSI_PLLOUT_SEL_DSI1_PLLOUT_DIVSEL_SHIFT,
		.divsel = PRCM_DSI_PLLOUT_SEL_PHI,
		.divsel_lcd_mask = BIT(11),
	}
};

struct dsiescclk {
	u32 en;
	u32 div_mask;
	u32 div_shift;
};

static struct dsiescclk dsiescclk[3] = {
	{
		.en = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI0_ESC_CLK_DIV_SHIFT,
	},
	{
		.en = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI1_ESC_CLK_DIV_SHIFT,
	},
	{
		.en = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_EN,
		.div_mask = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_DIV_MASK,
		.div_shift = PRCM_DSITVCLK_DIV_DSI2_ESC_CLK_DIV_SHIFT,
	}
};

static u32 ddr_sleep_strat_policy[PRCMU_DDR_SLEEP_STRAT_LP_MODE_NB]
					[PRCMU_DDR_SLEEP_STRAT_DDRCTRL_NB] =
{
	{
		DDRCTRLSTATE_ON, /* Ctrl0ApIdle */
		DDRCTRLSTATE_ON  /* Ctrl1ApIdle */
	},
	{
		DDRCTRLSTATE_ON, /* Ctrl0ApDeepIdle */
		DDRCTRLSTATE_ON  /* Ctrl1ApDeepIdle */
	},
	{
		DDRCTRLSTATE_OFFHIGHLAT, /* Ctrl0ApSleep */
		DDRCTRLSTATE_OFFHIGHLAT  /* Ctrl1ApSleep */
	}
};

/*
* Used by MCDE to setup all necessary PRCMU registers
*/
#define PRCMU_RESET_DSIPLLTV		0x00004000
#define PRCMU_RESET_DSIPLLLCD		0x00008000
#define PRCMU_UNCLAMP_DSIPLL		0x00400800

#define PRCMU_CLK_PLL_DIV_SHIFT		0
#define PRCMU_CLK_PLL_SW_SHIFT		5
#define PRCMU_CLK_38			(1 << 9)
#define PRCMU_CLK_38_SRC		(1 << 10)
#define PRCMU_CLK_38_DIV		(1 << 11)

/* PLLDIV=12, PLLSW=4 (PLLDDR) */
#define PRCMU_DSI_CLOCK_SETTING		0x0000008C
/* PLLDIV = 12, PLLSW=1 (PLLSOC0) */
#define U9540_PRCMU_DSI_CLOCK_SETTING	0x0000002C

/* DPI 50000000 Hz */
#define PRCMU_DPI_CLOCK_SETTING		((1 << PRCMU_CLK_PLL_SW_SHIFT) | \
					  (16 << PRCMU_CLK_PLL_DIV_SHIFT))
#define PRCMU_DSI_LP_CLOCK_SETTING	0x00000E00

/* D=101, N=1, R=4, SELDIV2=0 */
#define PRCMU_PLLDSI_FREQ_SETTING	0x00040165

#define PRCMU_ENABLE_PLLDSI		0x00000001
#define PRCMU_DISABLE_PLLDSI		0x00000000
#define PRCMU_RELEASE_RESET_DSS		0x0000400C
#define PRCMU_TV_DSI_PLLOUT_SEL_SETTING	0x00000202
#define PRCMU_LCD_DSI_PLLOUT_SEL_SETTING	0x00000A0A
/* ESC clk, div0=1, div1=1, div2=3 */
#define PRCMU_ENABLE_ESCAPE_CLOCK_DIV	0x07030101
#define PRCMU_DISABLE_ESCAPE_CLOCK_DIV	0x00030101
#define PRCMU_DSI_RESET_SW		0x00000007

#define PRCMU_PLLDSI_LOCKP_LOCKED	0x3

/**
 * upap_init
 * initialization UniqPAP link
 */
static void upap_init(void)
{
	int upap_offset;
	struct prcmu_fw_version *fw_vers = prcmu_get_fw_version();

	if (fw_vers->api_version >= 5)
		/* TODO: Use defines instead of magic numbers */
		upap_offset = 0x2D00;
	else if (fw_vers->api_version >= 4)
		/* TODO: Use defines instead of magic numbers */
		upap_offset = 0xA30;
	else
		BUG();

	upap_conf.req = (struct upap_req *)(tcdm_base + upap_offset);
	upap_conf.nfy = (struct upap_nfy *)(tcdm_base + upap_offset +
			sizeof(struct upap_req));
	upap_conf.mbox_nb = 1;

	mutex_init(&upap_transfer.lock);
	init_completion(&upap_transfer.work);
}

/**
 * db9540_prcmu_upap_wait_released
 * Utility function which blocks until Mailbox is released.
 */
static inline void db9540_prcmu_upap_wait_released(void)
{
	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(upap_conf.mbox_nb))
		cpu_relax();
}

/**
 * db9540_prcmu_upap_wait_for_idle
 * Utility function which blocks until uniqPAP is in its Idle state.
 */
static inline void db9540_prcmu_upap_wait_for_idle(void)
{
	while (upap_conf.req->req_state != UX540_PRCM_UPAP_REQ_STATE_REQ_IDLE)
		cpu_relax();
}

/**
 * upap_send_request
 * Generic to send UniqPAP request to PRCMU
 */
static void upap_send_request(struct upap_req *req, struct upap_ack *ack,
		int data_size)
{
	mutex_lock(&upap_transfer.lock);

	/* save ack structure */
	upap_transfer.ack = ack;

	/* Wait for MBOX to become idle */
	db9540_prcmu_upap_wait_released();
	/* Ensure UPAP is in Idle state */
	db9540_prcmu_upap_wait_for_idle();

	/* Write to TCDM (header and data, then req_state) */
	upap_conf.req->service_id = req->service_id;
	upap_conf.req->command_id = req->command_id;
	upap_conf.req->req_state = req->req_state;
	if (data_size)
		memcpy(&upap_conf.req->data.data, &req->data.data, data_size);

	/* Set interrupt ARM -> PRCMU */
	writel(MBOX_BIT(upap_conf.mbox_nb), PRCM_MBOX_CPU_SET);
	WARN_ON(wait_for_completion_timeout(&upap_transfer.work, UPAP_TIM)== 0);

	mutex_unlock(&upap_transfer.lock);
}

/**
 * upap_register_ack_service
 * dynamic service acknoledge registering
 */
static int upap_register_ack_service(u32 service_id,
		void (*service_ack)(struct upap_req *req, struct upap_ack *ack))
{
	if(service_id >= UPAP_SERVICES_NB)
		return -EINVAL;

	if(upap_read_services[service_id] == NULL)
		upap_read_services[service_id] = service_ack;
	else
		return -EBUSY;
	return 0;
}

/**
 * upap_register_notifier
 * dynamic notifier registering on dedicated PRMCU services
 */
int upap_register_notifier(enum upap_nfy_id id,
		struct notifier_block *nb)
{
	if ((id < 0) || (id >= UPAP_NFYID_MAX))
		return -EINVAL;
	return atomic_notifier_chain_register(&prcmu_nfy_notifiers[id], nb);
}
EXPORT_SYMBOL_GPL(upap_register_notifier);

/**
 * upap_unregister_notifier
 * dynamic notifier unregistering on dedicated PRMCU services
 */
int upap_unregister_notifier(enum upap_nfy_id id,
		struct notifier_block *nb)
{
	if ((id < 0) || (id >= UPAP_NFYID_MAX))
		return -EINVAL;
	return atomic_notifier_chain_unregister(&prcmu_nfy_notifiers[id], nb);
}
EXPORT_SYMBOL_GPL(upap_unregister_notifier);

/**
 * upap_unregister_ack_service
 * dynamic service acknoledge registering
 */
static int upap_unregister_ack_service(u32 service_id)
{
	if(service_id >= UPAP_SERVICES_NB)
		return -EINVAL;

	if(upap_read_services[service_id] != NULL)
		upap_read_services[service_id] = NULL;
	else
		return -EINVAL;
	return 0;
}

/**
 * unplug_cpu1 - Power gate OFF CPU1 for U9540
 * * void:
 * Returns:
 */
static int unplug_cpu1(void)
{
	int r = 0;
#if defined (CONFIG_UX500_ROMCODE_SHARED_MUTEX) && \
				defined(CONFIG_UX500_PRCMU_CPU_HOTPLUG)
	struct upap_req req;
	struct upap_ack ack;

	/* Set flag start Hotplug sequence */
	cpu1_unplug_ongoing = 1;

	/* Fill request (header and data, then req_state) */
	req.service_id = UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG;
	req.command_id = U9540_PRCM_UPAP_COMMAND_CPU1_UNPLUG;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, 0);

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_CPU1_UNPLUG)) {
		switch (ack.status) {
		case U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_UNKNOWN_ERR:
			pr_err("PRCMU: %s, unknown error\n", __func__);
			WARN_ON(1);
			break;
		case U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_ROMCODESAVEOWNCTX_ERR:
			pr_err("PRCMU: %s, CPU1 ROM code err: save own context error\n"
			, __func__);
			break;
	}
	} else {
		r = -EIO;
		pr_err("PRCMU - bad ack in %s. 0x%x 0x%x %u\n", __func__,
		ack.service_id, ack.command_id, ack.status);
	}
	/* set flag HotPlug sequence end */
	cpu1_unplug_ongoing = 0;
#endif
	return r;
}

/**
 * replug_cpu1 - Power gate ON CPU1 for U9540
 * * void
 * * Returns:
 */
static int replug_cpu1(void)
{
	int r = 0;
#if defined (CONFIG_UX500_ROMCODE_SHARED_MUTEX) && \
		defined(CONFIG_UX500_PRCMU_CPU_HOTPLUG)
	struct upap_req req;
	struct upap_ack ack;

	if (prcmu_driver_initialised == 0) {
		pr_info("PRCMU: %s, PRCMU DRIVER NOT INITIALISED\n", __func__);
		return 0;
	}

	/* Fill request (header and data, then req_state) */
	req.service_id = UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG;
	req.command_id = U9540_PRCM_UPAP_COMMAND_CPU1_REPLUG;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, 0);

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_CPU1_REPLUG)) {
		switch (ack.status) {
		case U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_UNKNOWN_ERR:
			pr_err("PRCMU: %s, unknown error\n", __func__);
			WARN_ON(1);
			break;
		case U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_WAKEUPNORESP_ROM_ERR:
			pr_err("PRCMU: %s, CPU1 Rom code err: no resp at wake up\n"
					, __func__);
			WARN_ON(1);
			break;
		case U9540_PRCM_UPAP_STATUS_CPUHOTPLUG_RESPLSNOTDSTOREADY:
			pr_err("PRCMU: %s, CPU1 Rom code err: no Ds to Rdy\n"
					, __func__);
			WARN_ON(1);
			break;
	}
	} else {
		r = -EIO;
		pr_err("PRCMU - bad ack in %s. 0x%x 0x%x %u\n", __func__,
		ack.service_id, ack.command_id, ack.status);
	}
#endif
	return r;
}

static struct cpufreq_frequency_table *freq_table;

static bool db9540_check_ap9540_age(void);

bool has_arm_maxopp(void)
{
	if ((!db9540_check_ap9540_age()) ||
			(readw(tcdm_base+PRCMU_OPP0_IS_ENABLE) != 1))
		return false;
	else
		return true;
}

static void update_freq_table(struct cpufreq_frequency_table *table)
{
	struct prcmu_fw_freq_config
		*arm_freq_config = tcdm_base + PRCM_INIT_OPP_DATA;
	u32 freq;
	int j = 0;
	int nb_freq;
	struct prcmu_fw_version *fw_vers = prcmu_get_fw_version();

	/*
	 * i = 0 => retention should not fill in frequency table
	 * i = 1 => 266 Mhz. remove 266 MHz frequency that creates
	 *                  re-entering condition
	 */
#ifdef CONFIG_MFD_DBX540_FREQ_LIMITATION
	int i = 2;
#else
	int i = 1;
#endif
	/*
	 * There are 6 OPPs Before PRCMU_FW version 5
	 * and them 10 OPPs from
	 * [Retention][266000][400000][800000][1200000][1500000]
	 * [1850000][New_freq1][New_freq2][New_freq3][New_freq4]
	 * frequency table should not be fill with  [Retention]
	 */

	if (fw_vers->api_version < 5)
		nb_freq = 7;
	else
		nb_freq = 11;

	for (; i < nb_freq; i++) {
		if (readw(&(arm_freq_config[i].enable)) != 0) {
			freq = readl(&(arm_freq_config[i].arm_freq));
			if ((freq <= FREQ_1G5) || (db9540_check_ap9540_age())) {
				table[j].frequency = freq;
				table[j].index = j;
				j++;
			}
		}
	}
	for ( ; j < i; j++) {
		table[j].frequency = CPUFREQ_TABLE_END;
		table[j].index = j;
	}
}

/**
 * config_clkout - Configure one of the programmable clock outputs.
 * @clkout:	The CLKOUT number (0 or 1).
 * @source:	The clock to be used (one of the PRCMU_CLKSRC_*).
 * @div:	The divider to be applied.
 *
 * Configures one of the programmable clock outputs (CLKOUTs).
 * @div should be in the range [1,63] to request a configuration, or 0 to
 * inform that the configuration is no longer requested.
 */
static int config_clkout(u8 clkout, u8 source, u8 div)
{
	static int requests[2];
	int r = 0;
	unsigned long flags;
	u32 val;
	u32 bits;
	u32 mask;
	u32 div_mask;

	BUG_ON(clkout > 1);
	BUG_ON(div > 63);
	BUG_ON((clkout == 0) && (source > PRCMU_CLKSRC_CLK009));

	if (!div && !requests[clkout])
		return -EINVAL;

	switch (clkout) {
	case 0:
		div_mask = PRCM_CLKOCR_CLKODIV0_MASK;
		mask = (PRCM_CLKOCR_CLKODIV0_MASK | PRCM_CLKOCR_CLKOSEL0_MASK);
		bits = ((source << PRCM_CLKOCR_CLKOSEL0_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV0_SHIFT));
		break;
	case 1:
		div_mask = PRCM_CLKOCR_CLKODIV1_MASK;
		mask = (PRCM_CLKOCR_CLKODIV1_MASK | PRCM_CLKOCR_CLKOSEL1_MASK |
			PRCM_CLKOCR_CLK1TYPE);
		bits = ((source << PRCM_CLKOCR_CLKOSEL1_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV1_SHIFT));
		break;
	}
	bits &= mask;

	spin_lock_irqsave(&clkout_lock, flags);

	val = readl(PRCM_CLKOCR);
	if (val & div_mask) {
		if (div) {
			if ((val & mask) != bits) {
				r = -EBUSY;
				goto unlock_and_return;
			}
		} else {
			if ((val & mask & ~div_mask) != bits) {
				r = -EINVAL;
				goto unlock_and_return;
			}
		}
	}
	writel((bits | (val & ~mask)), PRCM_CLKOCR);
	requests[clkout] += (div ? 1 : -1);

unlock_and_return:
	spin_unlock_irqrestore(&clkout_lock, flags);

	return r;
}

/*  transition translation table to FW magic number */
static u8 dbx540_fw_trans[] = {
	0x00,/* PRCMU_AP_NO_CHANGE */
	0x10,/* PRCMU_AP_SLEEP */
	0x43,/* PRCMU_AP_DEEP_SLEEP */
	0x50,/* PRCMU_AP_IDLE */
	0x73,/*	PRCMU_AP_DEEP_IDLE */
};

static int stay_in_wfi_check(void)
{
	int stay_in_wfi = 0;
	u8 status;

	status = readb(tcdm_legacy_base + PRCM_ACK_MB0_AP_PWRSTTR_STATUS);

	if ((status == EXECUTETODEEPSLEEP)
			|| (status == EXECUTETODEEPIDLE)) {
		stay_in_wfi = 1;
	}
	if (cpu1_unplug_ongoing == 1)
		stay_in_wfi = 1;

	return stay_in_wfi;
}

/*
 * db9540_write_arm_max_opp : mission profile
 * write accumulated arm_max_opp value
 * The parameter val is in second
 */
static void db9540_write_arm_max_opp(u64 val)
{
	/*Call the routine to write arm_max_opp value  */
	arm_max_opp_profile.secure_memory = val;
}

/*
 * db9540_read_arm_max_opp_counter : mission profile
 * read accumulated arm_max_opp
 * The return value is in second
 */
u64 db9540_read_arm_max_opp_counter(void)
{
	/*
	 * Call the routine to read arm_max_opp value
	 The return value should replace "arm_max_opp_profile.max_opp_cnt"
	 */
	return arm_max_opp_profile.secure_memory;
}

/*
 * return false if AP9540 current age is higher than the max allowed
 */
static bool db9540_check_ap9540_age(void)
{
	return ((arm_max_opp_profile.max_opp_cnt >=
			DB9540_AGING_TRADE) ? false : true);
}

/*
 * db9540_memorize_arm_max_opp accumulate the time spent at arm_opp_max
 * If the store time is grater than "STORE_CYCLE",
 * call the function .... to memorize in FLASH Mem
 * The timer time-base is in nano-second
 */
void db9540_memorize_arm_max_opp(u32 freq)
{
	if (arm_max_opp_profile.last_arm_opp > FREQ_1G5) {
		bool save;
		u64 this_time , delta_time;

		this_time = sched_clock();
		delta_time = this_time - arm_max_opp_profile.start;
		arm_max_opp_profile.cumul += delta_time;
		arm_max_opp_profile.max_opp_cnt += delta_time;
		arm_max_opp_profile.start = this_time;
		arm_max_opp_profile.last_arm_opp = freq;
		save = false;
		while (arm_max_opp_profile.cumul >= STORE_CYCLE) {
			save = true;
			arm_max_opp_profile.cumul -=  STORE_CYCLE;
		}
		/* call to save the counter content*/
		if (save)
			db9540_write_arm_max_opp(
					arm_max_opp_profile.max_opp_cnt);
	 }
	else if (freq > FREQ_1G5) {
		arm_max_opp_profile.start = sched_clock();
		arm_max_opp_profile.last_arm_opp = freq;
	}
}

/*
 * set_arm_freq - set the appropriate ARM frequency for U9540
 * @freq: The new ARM frequency to which transition is to be made (kHz)
 * Returns: 0 on success, non-zero on failure
 */
static int set_arm_freq(u32 freq)
{
	struct upap_req req;
	struct upap_ack ack;
	int r = 0;

	if (dvfs_context.arm_freq == freq)
		return 0;

	/* Prepare request (header and data, then req_state) */
	req.service_id = UX540_PRCM_UPAP_SERVICE_DVFS;
	req.command_id = U9540_PRCM_UPAP_COMMAND_SET_ARM_OPP;
	req.data.arm_opp.freq = freq;
	req.data.arm_opp.volt = 0;
	req.data.arm_opp.bias = 0;
	req.data.arm_opp.vbbp = 0;
	req.data.arm_opp.vbbn = 0;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(struct upap_arm_opp_req_data));

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_DVFS) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_SET_ARM_OPP) &&
		(ack.status == U9540_PRCM_UPAP_STATUS_OK)) {
		dvfs_context.arm_freq = freq;
		latest_armss_rate = freq;
		db9540_memorize_arm_max_opp(freq);
		prcmu_debug_arm_opp_log(freq);
	} else {
		r = -EIO;
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u %u %u\n", __func__,
		ack.service_id, ack.command_id, ack.status, ack.arm_freq,
		freq);
	}

	return r;
}

/**
 * get_arm_freq - get the current ARM freq
 *
 * Returns: the current ARM freq (kHz).
 * Not supported by U8500
 */
static int get_arm_freq(void)
{
	u32 val;
	/*
	 * U9540 is not able to read ARM OPP value from TCDM. Therefore
	 * determine if the ARM OPP has been set, or not.
	 */
	if (dvfs_context.arm_freq != 0)
		return dvfs_context.arm_freq;

	/* ARM OPP value not yet initialised. Read value from register. */
	val = readl(PRCM_POWER_STATE_VAL);
	val &= PRCM_POWER_STATE_VAL_VARM_STATE_OPP_MASK;
	val >>= PRCM_POWER_STATE_VAL_VARM_STATE_OPP_SHIFT;

	switch (val) {
	case 0x00:
		return 1850000;
	case 0x01:
		return 1500000;
	case 0x02:
		return 1200000;
	case 0x03:
		return 800000;
	case 0x04:
		return 400000;
	case 0x05:
		return 266000;
	default:
		pr_warn("prcmu: %s Unknown ARM OPP val %d\n", __func__, val);
		/* Return fastest non-"speed-binned" frequency */
		return 1500000;
	}
}

/**
 * prcmu_get_vsafe_opp - get the current VSAFE OPP
 *
 * Returns: the current VSAFE OPP
 */
int prcmu_get_vsafe_opp(void)
{
	/*
	 * U9540 is not able to read VSAFE OPP value from TCDM. Therefore
	 * determine if the VSAFE OPP has been set, or not.
	 */
	if (dvfs_context.vsafe_opp != 0) {
		return dvfs_context.vsafe_opp;
	} else {
		/*
		 * VSAFE OPP value not yet initialised.
		 * Return default (reset) value.
		 */
		return VSAFE_100_OPP;
	}
}

/**
 * prcmu_set_vsafe_opp - set the appropriate VSAFE OPP
 * @opp: The new VSAFE operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the VSAFE.
 */
int prcmu_set_vsafe_opp(u8 opp)
{
	struct upap_req req;
	struct upap_ack ack;
	int r = 0;
	u32 prcmu_opp;

	switch (opp) {
	case VSAFE_50_OPP:
	case VSAFE_100_OPP:
		prcmu_opp = U9540_PRCM_REQ_UPAP_VSAFE_OPP2;
		break;
	default:
		/* Do nothing */
		return 0;
	}

	/* Prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_DVFS;
	req.command_id = U9540_PRCM_UPAP_COMMAND_SET_SAFE_OPP;
	req.data.data = prcmu_opp;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(u32));

	/*
	 * Check response from PRCMU. U9540 TCDM does not contain current OPP
	 * so we cannot check its value.
	 */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_DVFS) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_SET_SAFE_OPP) &&
		(ack.status == U9540_PRCM_UPAP_STATUS_OK)) {
		dvfs_context.vsafe_opp = prcmu_opp;
	} else {
		r = -EIO;
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u %u\n", __func__,
		ack.service_id, ack.command_id, ack.status, opp);
	}

	return r;
}

/**
 * get_ddr_opp - get the current DDR OPP
 *
 * Returns: the current DDR OPP
 */
int get_ddr_opp(void)
{
	return readb(PRCM_DDR_SUBSYS_APE_MINBW);
}

/**
 * get_effective_ddr_opp - get the current effective DDR OPP
 *
 * Returns: the current effective DDR OPP
 */
int get_effective_ddr_opp(void)
{
	return ((readb(PRCM_DDRSUBSYS_STATUS)&
			PRCM_DDRSUBSYS_STATUS_MINBW_MASK)>>
			PRCM_DDRSUBSYS_STATUS_SHIFT);
}

/**
 * get_ddr1_opp - get the current DDR1 OPP
 *
 * Returns: the current DDR1 OPP
 */
int get_ddr1_opp(void)
{
	return readb(PRCM_DDR1_SUBSYS_APE_MINBW);
}

/**
 * get_effective_ddr1_opp - get the current effective DDR1 OPP
 *
 * Returns: the current effective DDR1 OPP
 */
int get_effective_ddr1_opp(void)
{
	return ((readb(PRCM_DDR1SUBSYS_STATUS)&
			PRCM_DDRSUBSYS_STATUS_MINBW_MASK)>>
			PRCM_DDRSUBSYS_STATUS_SHIFT);
}

/**
 * set_ddr_opp - set the appropriate DDR OPP
 * @opp: The new DDR operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the DDR.
 */
int set_ddr_opp(u8 opp)
{
	if (opp < DDR_100_OPP || opp > DDR_25_OPP)
		return -EINVAL;
	/* Changing the DDR OPP can hang the hardware pre-v21 */
	writeb(opp, PRCM_DDR_SUBSYS_APE_MINBW);
	writeb(opp, PRCM_DDR1_SUBSYS_APE_MINBW);

	trace_u8500_set_ddr_opp(opp);
	return 0;
}

/* Divide the frequency of certain clocks by 2 for APE_50_PARTLY_25_OPP. */
static void request_even_slower_clocks(bool enable)
{
	void __iomem *clock_reg[] = {
		PRCM_ACLK_MGT,
		PRCM_DMACLK_MGT
	};
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	for (i = 0; i < ARRAY_SIZE(clock_reg); i++) {
		u32 val;
		u32 div;

		val = readl(clock_reg[i]);
		div = (val & PRCM_CLK_MGT_CLKPLLDIV_MASK);
		if (enable) {
			if ((div <= 1) || (div > 15)) {
				pr_err("prcmu: Bad clock divider %d in %s\n",
					div, __func__);
				goto unlock_and_return;
			}
			div <<= 1;
		} else {
			if (div <= 2)
				goto unlock_and_return;
			div >>= 1;
		}
		val = ((val & ~PRCM_CLK_MGT_CLKPLLDIV_MASK) |
			(div & PRCM_CLK_MGT_CLKPLLDIV_MASK));
		writel(val, clock_reg[i]);
	}

unlock_and_return:
	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);
}

static int db9540_prcmu_write_ape_opp(u8 opp)
{
	struct upap_req req;
	struct upap_ack ack;
	int r = 0;
	u32 prcmu_opp;

	switch (opp) {
	case APE_50_OPP:
	case APE_50_PARTLY_25_OPP:
		prcmu_opp = U9540_PRCM_REQ_UPAP_APE_OPP_1;
		break;
	case APE_100_OPP:
		prcmu_opp = U9540_PRCM_REQ_UPAP_APE_OPP_2;
		break;
	case APE_OPP_INIT:
	case APE_NO_CHANGE:
	default:
		/* Do nothing */
		return 0;
	}

	/* Prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_DVFS;
	req.command_id = U9540_PRCM_UPAP_COMMAND_SET_APE_OPP;
	req.data.data = prcmu_opp;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(u32));

	/*
	 * Check response from PRCMU. U9540 TCDM does not contain current OPP
	 * so we cannot check its value.
	 */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_DVFS) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_SET_APE_OPP) &&
		(ack.status == U9540_PRCM_UPAP_STATUS_OK)) {
		r = 0;
	} else {
		r = -EIO;
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u %u\n", __func__,
		ack.service_id, ack.command_id, ack.status, prcmu_opp);
	}

	return r;
}

/**
 * set_ape_opp - set the appropriate APE OPP
 * @opp: The new APE operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the APE.
 */
static int set_ape_opp(u8 opp)
{
	int r = 0;

	trace_u8500_set_ape_opp(opp);
	if (opp == dvfs_context.ape_opp)
		return 0;

	/* Exit APE_50_PARTLY_25_OPP */
	if (dvfs_context.ape_opp == APE_50_PARTLY_25_OPP)
		request_even_slower_clocks(false);

	if ((opp != APE_100_OPP) && (dvfs_context.ape_opp != APE_100_OPP))
		goto skip_message;

	r = db9540_prcmu_write_ape_opp(opp);
skip_message:
	if ((!r && (opp == APE_50_PARTLY_25_OPP)) ||
			/* Set APE_50_PARTLY_25_OPP back in case new opp failed */
			(r && (dvfs_context.ape_opp == APE_50_PARTLY_25_OPP)))
		request_even_slower_clocks(true);
	if (!r)
		dvfs_context.ape_opp = opp;

	return r;
}

/**
 * get_ape_opp - get the current APE OPP
 *
 * Returns: the current APE OPP
 */
static int get_ape_opp(void)
{
	u32 val;
	/*
	 * U9540 is not able to read APE OPP value from TCDM. Therefore
	 * determine if the APE OPP has been set, or not.
	 */
	if (dvfs_context.ape_opp != APE_OPP_INIT)
		return dvfs_context.ape_opp;

	/*
	 * APE OPP value not yet initialised. Read value from
	 * register.
	 */
	val = readl(PRCM_POWER_STATE_VAL);
	val &= PRCM_POWER_STATE_VAL_VAPE_STATE_OPP_MASK;
	val >>= PRCM_POWER_STATE_VAL_VAPE_STATE_OPP_SHIFT;
	switch (val) {
	case 0x00:
		return APE_100_OPP;
	case 0x01:
		return APE_50_OPP;
	default:
		pr_warn("prcmu: %s Unknown APE OPP val %d\n", __func__, val);
		return APE_OPP_INIT;
	}
}

/**
 * request_ape_opp_100_voltage - Request APE OPP 100% voltage
 * @enable: true to request the higher voltage, false to drop a request.
 *
 * Calls to this function to enable and disable requests must be balanced.
 * Not supported by U9540. u8540 needs to supoort it.
 */
static int request_ape_opp_100_voltage(bool enable)
{
	if (!enable_ape_opp_100_voltage) {
		pr_debug("prcmu: %s not supported\n", __func__);
		return 0;
	}

	/* this function for 8540 need check with PRCMU team. */
	return 0;
}

static int db9540_prcmu_release_usb_wakeup_state(void)
{
	struct upap_req req;
	struct upap_ack ack;
	int r = 0;

	/* Write to TCDM */
	req.service_id = UX540_PRCM_UPAP_SERVICE_USB;
	req.command_id = U9540_PRCM_UPAP_COMMAND_USB_WAKEUP_REL;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, 0);

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_USB) &&
			(ack.command_id ==
			 U9540_PRCM_UPAP_COMMAND_USB_WAKEUP_REL) &&
			(ack.status == U9540_PRCM_UPAP_STATUS_OK)) {
		r = 0;
	} else {
		r = -EIO;
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u\n", __func__,
				ack.service_id, ack.command_id, ack.status);
	}

	return r;
}

/**
 * dbx540_prcmu_release_usb_wakeup_state - release the state required by a USB wakeup
 *
 * This function releases the power state requirements of a USB wakeup.
 */
int dbx540_prcmu_release_usb_wakeup_state(void)
{
	return (db9540_prcmu_release_usb_wakeup_state());
}

static int db9540_request_pll(u8 clock, bool enable)
{
	int r;
	u32 prcmu_clock;
	struct upap_req req;
	struct upap_ack ack;

	if (clock == PRCMU_PLLSOC0)
		prcmu_clock = (enable ? U9540_PRCM_REQ_UPAP_PLL_SOC0_ON :
				U9540_PRCM_REQ_UPAP_PLL_SOC0_OFF);
	else if (clock == PRCMU_PLLSOC1)
		prcmu_clock = (enable ? U9540_PRCM_REQ_UPAP_PLL_SOC1_ON :
				U9540_PRCM_REQ_UPAP_PLL_SOC1_OFF);

	/* Prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_CLOCK;
	req.command_id = U9540_PRCM_UPAP_COMMAND_PLL_ON_OFF;
	req.data.data = prcmu_clock;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(u32));

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_CLOCK) &&
			(ack.command_id == U9540_PRCM_UPAP_COMMAND_PLL_ON_OFF)
			&& (ack.status == U9540_PRCM_UPAP_STATUS_OK))
		r = 0;
	else {
		r = -EIO;
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u\n", __func__,
				ack.service_id, ack.command_id, ack.status);
	}

	return r;
}

static int request_pll(u8 clock, bool enable)
{
	if (clock != PRCMU_PLLSOC1)
		return -EINVAL;

	 return db9540_request_pll(clock, enable);
}

static int request_sysclk(bool enable)
{
	int r = 0;
	unsigned long flags;

	mutex_lock(&mb3.sysclk_lock);

	spin_lock_irqsave(&mb3.lock, flags);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(3))
		cpu_relax();

	writeb((enable ? 1 : 0), tcdm_legacy_base + PRCM_REQ_MB3_SYSCLK_MGT);

	writeb(MB3H_SYSCLK, (tcdm_legacy_base + PRCM_MBOX_HEADER_REQ_MB3));
	writel(MBOX_BIT(3), PRCM_MBOX_CPU_SET);

	spin_unlock_irqrestore(&mb3.lock, flags);

	/*
	 * The firmware only sends an ACK if we want to enable the
	 * SysClk, and it succeeds.
	 */
	if (enable && !wait_for_completion_timeout(&mb3.sysclk_work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		db8500_prcmu_debug_dump(__func__, true, true);
	}

	mutex_unlock(&mb3.sysclk_lock);

	return r;
}

static int request_timclk(bool enable)
{
	u32 val = (PRCM_TCR_DOZE_MODE | PRCM_TCR_TENSEL_MASK);

	if (!enable)
		val |= PRCM_TCR_STOP_TIMERS;
	writel(val, PRCM_TCR);

	return 0;
}

static int request_clock(u8 clock, bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(clk_mgt[clock].reg);
	if (enable) {
		val |= (PRCM_CLK_MGT_CLKEN | clk_mgt[clock].pllsw);
	} else {
		clk_mgt[clock].pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);
		val &= ~(PRCM_CLK_MGT_CLKEN | PRCM_CLK_MGT_CLKPLLSW_MASK);
	}
	writel(val, clk_mgt[clock].reg);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}

static int request_sga_clock(u8 clock, bool enable)
{
	u32 val;
	int ret;

	if (enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val | PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	ret = request_clock(clock, enable);

	if (!ret && !enable) {
		val = readl(PRCM_CGATING_BYPASS);
		writel(val & ~PRCM_CGATING_BYPASS_ICN2, PRCM_CGATING_BYPASS);
	}

	return ret;
}

static inline bool plldsi_tv_locked(void)
{
	return (readl(PRCM_PLLDSITV_LOCKP) &
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3)) ==
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3);
}

static inline bool plldsi_lcd_locked(void)
{
	return (readl(PRCM_PLLDSILCD_LOCKP) &
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3)) ==
		(PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP10 |
		 PRCM_PLLDSI_LOCKP_PRCM_PLLDSI_LOCKP3);
}

static int request_plldsi(bool enable, bool lcd)
{
	int r = 0;
	u32 val;
	void __iomem *pll_dsi_enable_reg;
	u32 pll_dsi_resetn_bit;
	bool (*plldsi_locked)(void);

	if (lcd) {
		pll_dsi_enable_reg = PRCM_PLLDSILCD_ENABLE;
		pll_dsi_resetn_bit = PRCM_APE_RESETN_DSIPLL_LCD_RESETN;
		plldsi_locked = plldsi_lcd_locked;
	} else {
		pll_dsi_enable_reg = PRCM_PLLDSITV_ENABLE;
		pll_dsi_resetn_bit = PRCM_APE_RESETN_DSIPLL_TV_RESETN;
		plldsi_locked = plldsi_tv_locked;
	}

	if (enable) {
		/* Only clamp for enable if both are unlocked */
		if (!plldsi_lcd_locked() && !plldsi_tv_locked())
			writel((PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMP |
				PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMPI),
							PRCM_MMIP_LS_CLAMP_CLR);
	} else {
		/* Only clamp for disable if one are locked */
		bool tv_locked = plldsi_tv_locked();
		bool lcd_locked = plldsi_lcd_locked();
		if ((!lcd_locked && tv_locked) || (lcd_locked && !tv_locked))
			writel((PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMP |
				PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMPI),
							PRCM_MMIP_LS_CLAMP_SET);
	}

	val = readl(pll_dsi_enable_reg);
	if (enable)
		val |= PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
	else
		val &= ~PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
	writel(val, pll_dsi_enable_reg);

	if (enable) {
		unsigned int i;
		bool locked = plldsi_locked();

		for (i = 10; !locked && (i > 0); --i) {
			udelay(100);
			locked = plldsi_locked();
		}
		if (locked) {
			writel(pll_dsi_resetn_bit,
				PRCM_APE_RESETN_SET);
		} else {
			writel((PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMP |
				PRCM_MMIP_LS_CLAMP_DSIPLL_CLAMPI),
				PRCM_MMIP_LS_CLAMP_SET);
			val &= ~PRCM_PLLDSI_ENABLE_PRCM_PLLDSI_ENABLE;
			writel(val, pll_dsi_enable_reg);
			r = -EAGAIN;
		}
	} else {
		writel(pll_dsi_resetn_bit, PRCM_APE_RESETN_CLR);
	}

	return r;
}

#define NO_LCD false
#define LCD true

static int request_dsiclk(u8 n, bool enable, bool lcd)
{
	u32 val;
	struct dsiclk *dsiclk;

	dsiclk = u9540_dsiclk;

	val = readl(PRCM_DSI_PLLOUT_SEL);
	val &= ~dsiclk[n].divsel_mask;
	val |= ((enable ? dsiclk[n].divsel : PRCM_DSI_PLLOUT_SEL_OFF) <<
			dsiclk[n].divsel_shift);
	if (lcd)
		val |= dsiclk[n].divsel_lcd_mask;
	writel(val, PRCM_DSI_PLLOUT_SEL);
	return 0;
}

static int request_dsiescclk(u8 n, bool enable)
{
	u32 val;

	val = readl(PRCM_DSITVCLK_DIV);
	enable ? (val |= dsiescclk[n].en) : (val &= ~dsiescclk[n].en);
	writel(val, PRCM_DSITVCLK_DIV);
	return 0;
}

/**
 * dbx540_request_clock() - Request for a clock to be enabled or disabled.
 * @clock:      The clock for which the request is made.
 * @enable:     Whether the clock should be enabled (true) or disabled (false).
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
static int dbx540_prcmu_request_clock(u8 clock, bool enable)
{
	trace_u8500_request_clock(clock, enable);
	if (clock == PRCMU_SGACLK)
		return request_sga_clock(clock, enable);
	else if (clock < PRCMU_NUM_REG_CLOCKS)
		return request_clock(clock, enable);
	else if (clock == PRCMU_TIMCLK)
		return request_timclk(enable);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return request_dsiclk((clock - PRCMU_DSI0CLK), enable, NO_LCD);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return request_dsiescclk((clock - PRCMU_DSI0ESCCLK), enable);
	else if (clock == PRCMU_PLLDSI)
		return request_plldsi(enable, false);
	else if ((clock == PRCMU_DSI0CLK_LCD) || (clock == PRCMU_DSI1CLK_LCD))
		return request_dsiclk((clock - PRCMU_DSI0CLK_LCD),
			enable, LCD);
	else if (clock == PRCMU_PLLDSI_LCD)
		return request_plldsi(enable, true);
	else if (clock == PRCMU_SYSCLK)
		return request_sysclk(enable);
	else if ((clock == PRCMU_PLLSOC0) || (clock == PRCMU_PLLSOC1))
		return request_pll(clock, enable);
	else
		return -EINVAL;
}

static unsigned long pll_rate(void __iomem *reg, unsigned long src_rate,
	int branch)
{
	u64 rate;
	u32 val;
	u32 d;
	u32 div = 1;

	val = readl(reg);

	rate = src_rate;
	rate *= ((val & PRCM_PLL_FREQ_D_MASK) >> PRCM_PLL_FREQ_D_SHIFT);

	d = ((val & PRCM_PLL_FREQ_N_MASK) >> PRCM_PLL_FREQ_N_SHIFT);
	if (d > 1)
		div *= d;

	d = ((val & PRCM_PLL_FREQ_R_MASK) >> PRCM_PLL_FREQ_R_SHIFT);
	if (d > 1)
		div *= d;

	if (val & PRCM_PLL_FREQ_SELDIV2)
		div *= 2;

	if ((branch == PLL_FIX) || ((branch == PLL_DIV) &&
		(val & PRCM_PLL_FREQ_DIV2EN) &&
		((reg == PRCM_PLLSOC0_FREQ) ||
		 (reg == PRCM_PLLDDR_FREQ))))
		div *= 2;

	(void)do_div(rate, div);

	return (unsigned long)rate;
}

#define ROOT_CLOCK_RATE 38400000

static unsigned long clock_rate(u8 clock)
{
	u32 val;
	u32 pllsw;
	unsigned long rate = ROOT_CLOCK_RATE;

	val = readl(clk_mgt[clock].reg);

	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div && (val & PRCM_CLK_MGT_CLK38DIV))
			rate /= 2;
		return rate;
	}

	val |= clk_mgt[clock].pllsw;
	pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);

	if (pllsw == PRCM_CLK_MGT_CLKPLLSW_SOC0)
		rate = pll_rate(PRCM_PLLSOC0_FREQ, rate, clk_mgt[clock].branch);
	else if (pllsw == PRCM_CLK_MGT_CLKPLLSW_SOC1)
		rate = pll_rate(PRCM_PLLSOC1_FREQ, rate, clk_mgt[clock].branch);
	else if (pllsw == PRCM_CLK_MGT_CLKPLLSW_DDR)
		rate = pll_rate(PRCM_PLLDDR_FREQ, rate, clk_mgt[clock].branch);
	else
		return 0;

	if ((clock == PRCMU_SGACLK) &&
		(val & PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN)) {
		u64 r = (rate * 10);

		(void)do_div(r, 25);
		return (unsigned long)r;
	}
	val &= PRCM_CLK_MGT_CLKPLLDIV_MASK;
	if (val)
		return rate / val;
	else
		return 0;
}

static unsigned long armss_rate(void)
{
	return latest_armss_rate;
}

static unsigned long dsiclk_rate(u8 n, bool lcd)
{
	u32 divsel;
	u32 div = 1;
	struct dsiclk *dsiclk;

	dsiclk = u9540_dsiclk;

	divsel = readl(PRCM_DSI_PLLOUT_SEL);
	divsel = ((divsel & dsiclk[n].divsel_mask) >> dsiclk[n].divsel_shift);

	if (divsel == PRCM_DSI_PLLOUT_SEL_OFF)
		divsel = dsiclk[n].divsel;

	switch (divsel) {
	case PRCM_DSI_PLLOUT_SEL_PHI_4:
		div *= 2;
	case PRCM_DSI_PLLOUT_SEL_PHI_2:
		div *= 2;
	case PRCM_DSI_PLLOUT_SEL_PHI:
		if (lcd)
			return pll_rate(PRCM_PLLDSILCD_FREQ,
					clock_rate(PRCMU_SPARE1CLK), PLL_RAW) / div;
		else
			return pll_rate(PRCM_PLLDSITV_FREQ,
					clock_rate(PRCMU_HDMICLK), PLL_RAW) / div;
	default:
		return 0;
	}
}

static unsigned long dsiescclk_rate(u8 n)
{
	u32 div;

	div = readl(PRCM_DSITVCLK_DIV);
	div = ((div & dsiescclk[n].div_mask) >> (dsiescclk[n].div_shift));
	return clock_rate(PRCMU_TVCLK) / max((u32)1, div);
}

static unsigned long dbx540_prcmu_clock_rate(u8 clock)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		return clock_rate(clock);
	else if (clock == PRCMU_TIMCLK)
		return ROOT_CLOCK_RATE / 16;
	else if (clock == PRCMU_SYSCLK)
		return ROOT_CLOCK_RATE;
	else if (clock == PRCMU_PLLSOC0)
		return pll_rate(PRCM_PLLSOC0_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLSOC1)
		return pll_rate(PRCM_PLLSOC1_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLDDR)
		return pll_rate(PRCM_PLLDDR_FREQ, ROOT_CLOCK_RATE, PLL_RAW);
	else if (clock == PRCMU_PLLDSI)
		return pll_rate(PRCM_PLLDSITV_FREQ, clock_rate(PRCMU_HDMICLK),
			PLL_RAW);
	else if (clock == PRCMU_ARMSS)
		return KHZ_TO_HZ(armss_rate());
	else if (clock == PRCMU_ARMCLK)
		return KHZ_TO_HZ(get_arm_freq());
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return dsiclk_rate(clock - PRCMU_DSI0CLK, false);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return dsiescclk_rate(clock - PRCMU_DSI0ESCCLK);
	else if (clock == PRCMU_PLLDSI_LCD)
		return pll_rate(PRCM_PLLDSILCD_FREQ,
					clock_rate(PRCMU_SPARE1CLK), PLL_RAW);
	else if ((clock == PRCMU_DSI0CLK_LCD) || (clock == PRCMU_DSI1CLK_LCD))
		return dsiclk_rate(clock - PRCMU_DSI0CLK_LCD, true);
	else
		return 0;
}

static unsigned long clock_source_rate(u32 clk_mgt_val, int branch)
{
	if (clk_mgt_val & PRCM_CLK_MGT_CLK38)
		return ROOT_CLOCK_RATE;
	clk_mgt_val &= PRCM_CLK_MGT_CLKPLLSW_MASK;
	if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_SOC0)
		return pll_rate(PRCM_PLLSOC0_FREQ, ROOT_CLOCK_RATE, branch);
	else if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_SOC1)
		return pll_rate(PRCM_PLLSOC1_FREQ, ROOT_CLOCK_RATE, branch);
	else if (clk_mgt_val == PRCM_CLK_MGT_CLKPLLSW_DDR)
		return pll_rate(PRCM_PLLDDR_FREQ, ROOT_CLOCK_RATE, branch);
	else
		return 0;
}

static u32 clock_divider(unsigned long src_rate, unsigned long rate)
{
	u32 div;

	div = (src_rate / rate);
	if (div == 0)
		return 1;
	if (rate < (src_rate / div))
		div++;
	return div;
}

static long round_clock_rate(u8 clock, unsigned long rate)
{
	u32 val;
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	val = readl(clk_mgt[clock].reg);
	src_rate = clock_source_rate((val | clk_mgt[clock].pllsw),
		clk_mgt[clock].branch);
	div = clock_divider(src_rate, rate);
	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div) {
			if (div > 2)
				div = 2;
		} else {
			div = 1;
		}
	} else if ((clock == PRCMU_SGACLK) && (div == 3)) {
		u64 r = (src_rate * 10);

		(void)do_div(r, 25);
		if (r <= rate)
			return (unsigned long)r;
	}
	rounded_rate = (src_rate / min(div, (u32)31));

	return rounded_rate;
}

#define MIN_PLL_VCO_RATE 600000000ULL
#define MAX_PLL_VCO_RATE 2000000000ULL

static long round_plldsi_rate(unsigned long rate)
{
	long rounded_rate = 0;
	unsigned long src_rate;
	unsigned long rem;
	u32 r;

	src_rate = clock_rate(PRCMU_HDMICLK);
	rem = rate;

	for (r = 7; (rem > 0) && (r > 0); r--) {
		u64 d;

		d = (r * rate);
		(void)do_div(d, src_rate);
		if (d < 6)
			d = 6;
		else if (d > 255)
			d = 255;
		d *= src_rate;
		if (((2 * d) < (r * MIN_PLL_VCO_RATE)) ||
			((r * MAX_PLL_VCO_RATE) < (2 * d)))
			continue;
		(void)do_div(d, r);
		if (rate < d) {
			if (rounded_rate == 0)
				rounded_rate = (long)d;
			break;
		}
		if ((rate - d) < rem) {
			rem = (rate - d);
			rounded_rate = (long)d;
		}
	}
	return rounded_rate;
}

static long round_dsiclk_rate(unsigned long rate, bool lcd)
{
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	if (lcd)
		src_rate = pll_rate(PRCM_PLLDSILCD_FREQ,
			clock_rate(PRCMU_SPARE1CLK), PLL_RAW);
	else
		src_rate = pll_rate(PRCM_PLLDSITV_FREQ,
			clock_rate(PRCMU_HDMICLK), PLL_RAW);
	div = clock_divider(src_rate, rate);
	rounded_rate = (src_rate / ((div > 2) ? 4 : div));

	return rounded_rate;
}

static long round_dsiescclk_rate(unsigned long rate)
{
	u32 div;
	unsigned long src_rate;
	long rounded_rate;

	src_rate = clock_rate(PRCMU_TVCLK);
	div = clock_divider(src_rate, rate);
	rounded_rate = (src_rate / min(div, (u32)255));

	return rounded_rate;
}

static long dbx540_prcmu_round_clock_rate(u8 clock, unsigned long rate)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		return round_clock_rate(clock, rate);
	else if (clock == PRCMU_PLLDSI)
		return round_plldsi_rate(rate);
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		return round_dsiclk_rate(rate, false);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		return round_dsiescclk_rate(rate);
	else if (clock == PRCMU_PLLDSI_LCD)
		return round_plldsi_rate(rate);
	else if ((clock == PRCMU_DSI0CLK_LCD) || (clock == PRCMU_DSI1CLK_LCD))
		return round_dsiclk_rate(rate, true);
	else
		return (long)prcmu_clock_rate(clock);
}

static void set_clock_rate(u8 clock, unsigned long rate)
{
	u32 val;
	u32 div;
	unsigned long src_rate;
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(clk_mgt[clock].reg);
	src_rate = clock_source_rate((val | clk_mgt[clock].pllsw),
		clk_mgt[clock].branch);
	div = clock_divider(src_rate, rate);
	if (val & PRCM_CLK_MGT_CLK38) {
		if (clk_mgt[clock].clk38div) {
			if (div > 1)
				val |= PRCM_CLK_MGT_CLK38DIV;
			else
				val &= ~PRCM_CLK_MGT_CLK38DIV;
		}
	} else if (clock == PRCMU_SGACLK) {
		val &= ~(PRCM_CLK_MGT_CLKPLLDIV_MASK |
			PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN);
		if (div == 3) {
			u64 r = (src_rate * 10);

			(void)do_div(r, 25);
			if (r <= rate) {
				val |= PRCM_SGACLK_MGT_SGACLKDIV_BY_2_5_EN;
				div = 0;
			}
		}
		val |= min(div, (u32)31);
	} else {
		val &= ~PRCM_CLK_MGT_CLKPLLDIV_MASK;
		val |= min(div, (u32)31);
	}

	writel(val, clk_mgt[clock].reg);

	/* Release the HW semaphore. */
	writel(0, PRCM_SEM);

	spin_unlock_irqrestore(&clk_mgt_lock, flags);
}

static int set_plldsi_rate(unsigned long rate, bool lcd)
{
	unsigned long src_rate;
	unsigned long rem;
	u32 pll_freq = 0;
	u32 r;

	if (lcd)
		src_rate = clock_rate(PRCMU_SPARE1CLK);
	else
		src_rate = clock_rate(PRCMU_HDMICLK);

	rem = rate;

	for (r = 7; (rem > 0) && (r > 0); r--) {
		u64 d;
		u64 hwrate;

		d = (r * rate);
		(void)do_div(d, src_rate);
		if (d < 6)
			d = 6;
		else if (d > 255)
			d = 255;
		hwrate = (d * src_rate);
		if (((2 * hwrate) < (r * MIN_PLL_VCO_RATE)) ||
			((r * MAX_PLL_VCO_RATE) < (2 * hwrate)))
			continue;
		(void)do_div(hwrate, r);
		if (rate < hwrate) {
			if (pll_freq == 0)
				pll_freq = (((u32)d << PRCM_PLL_FREQ_D_SHIFT) |
					(r << PRCM_PLL_FREQ_R_SHIFT));
			break;
		}
		if ((rate - hwrate) < rem) {
			rem = (rate - hwrate);
			pll_freq = (((u32)d << PRCM_PLL_FREQ_D_SHIFT) |
				(r << PRCM_PLL_FREQ_R_SHIFT));
		}
	}
	if (pll_freq == 0)
		return -EINVAL;

	pll_freq |= (1 << PRCM_PLL_FREQ_N_SHIFT);
	writel(pll_freq, lcd ? PRCM_PLLDSILCD_FREQ : PRCM_PLLDSITV_FREQ);

	return 0;
}

static void set_dsiclk_rate(u8 n, unsigned long rate, bool lcd)
{
	unsigned long src_rate;
	u32 val;
	u32 div;
	struct dsiclk *dsiclk;

	dsiclk = u9540_dsiclk;

	if (lcd)
		src_rate = clock_rate(PRCMU_SPARE1CLK);
	else
		src_rate = clock_rate(PRCMU_HDMICLK);

	div = clock_divider(pll_rate(
				lcd ? PRCM_PLLDSILCD_FREQ : PRCM_PLLDSITV_FREQ,
				src_rate, PLL_RAW), rate);

	dsiclk[n].divsel = (div == 1) ? PRCM_DSI_PLLOUT_SEL_PHI :
		(div == 2) ? PRCM_DSI_PLLOUT_SEL_PHI_2 :
		/* else */	PRCM_DSI_PLLOUT_SEL_PHI_4;

	val = readl(PRCM_DSI_PLLOUT_SEL);
	val &= ~dsiclk[n].divsel_mask;
	val |= (dsiclk[n].divsel << dsiclk[n].divsel_shift);
	if (lcd)
		val |= dsiclk[n].divsel_lcd_mask;
	writel(val, PRCM_DSI_PLLOUT_SEL);
}

static void set_dsiescclk_rate(u8 n, unsigned long rate)
{
	u32 val;
	u32 div;

	div = clock_divider(clock_rate(PRCMU_TVCLK), rate);
	val = readl(PRCM_DSITVCLK_DIV);
	val &= ~dsiescclk[n].div_mask;
	val |= (min(div, (u32)255) << dsiescclk[n].div_shift);
	writel(val, PRCM_DSITVCLK_DIV);
}

static int dbx540_prcmu_set_clock_rate(u8 clock, unsigned long rate)
{
	if (clock < PRCMU_NUM_REG_CLOCKS)
		set_clock_rate(clock, rate);
	else if (clock == PRCMU_PLLDSI)
		return set_plldsi_rate(rate, false);
	else if (clock == PRCMU_ARMCLK)
		return set_arm_freq(HZ_TO_KHZ(rate));
	else if ((clock == PRCMU_DSI0CLK) || (clock == PRCMU_DSI1CLK))
		set_dsiclk_rate((clock - PRCMU_DSI0CLK), rate, false);
	else if ((PRCMU_DSI0ESCCLK <= clock) && (clock <= PRCMU_DSI2ESCCLK))
		set_dsiescclk_rate((clock - PRCMU_DSI0ESCCLK), rate);
	else if (clock == PRCMU_PLLDSI_LCD)
		return set_plldsi_rate(rate, true);
	else if ((clock == PRCMU_DSI0CLK_LCD) || (clock == PRCMU_DSI1CLK_LCD))
		set_dsiclk_rate((clock - PRCMU_DSI0CLK_LCD), rate, true);
	trace_u8500_set_clock_rate(clock, rate);

	return 0;
}

static int config_esram0_deep_sleep(u8 state)
{
	return 0;
}

int prcmu_set_ddr_sleep_strat_policy(u8 ddr_ctrl_nb, u8 lp_mode,
		u8 ddr_ctrl_mode)
{
	struct upap_req req;
	struct upap_ack ack;
	int r = 0;

	if ((ddr_ctrl_nb > DDR_SLEEP_STRAT_DDRCTRL1) ||
			(ddr_ctrl_nb < DDR_SLEEP_STRAT_DDRCTRL0))
		goto error;
	if ((lp_mode > DDR_SLEEP_STRAT_AP_SLEEP_INDEX) ||
			(ddr_ctrl_nb < DDR_SLEEP_STRAT_AP_IDLE_INDEX))
		goto error;
	if ((ddr_ctrl_mode > DDRCTRLSTATE_ON) ||
			(ddr_ctrl_mode < DDRCTRLSTATE_OFFHIGHLAT))
		goto error;

	/* Set policy for DDRCtrlNb[cs0] */
	ddr_sleep_strat_policy[lp_mode][ddr_ctrl_nb] = ddr_ctrl_mode;

	/* Write to TCDM (header and data, then req_state) */
	req.service_id = UX540_PRCM_UPAP_SERVICE_DDR;
	req.command_id = U9540_PRCM_UPAP_COMMAND_DDR_SLEEP_STRAT;
	memcpy(req.data.full_data_buf, ddr_sleep_strat_policy,
					sizeof(ddr_sleep_strat_policy));
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(ddr_sleep_strat_policy));

	return r;

error:
	return -EINVAL;
}

static int config_hotdog(u8 threshold)
{
	mutex_lock(&mb4.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(threshold, (tcdm_legacy_base + PRCM_REQ_MB4_HOTDOG_THRESHOLD));
	writeb(MB4H_HOTDOG, (tcdm_legacy_base + PRCM_MBOX_HEADER_REQ_MB4));

	writel(MBOX_BIT(4), PRCM_MBOX_CPU_SET);
	WARN_ON(wait_for_completion_timeout(&mb4.work, MB4TIM) == 0);

	mutex_unlock(&mb4.lock);

	return 0;
}

static int config_hotmon(u8 low, u8 high)
{
	mutex_lock(&mb4.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(low, (tcdm_legacy_base + PRCM_REQ_MB4_HOTMON_LOW));
	writeb(high, (tcdm_legacy_base + PRCM_REQ_MB4_HOTMON_HIGH));
	writeb((HOTMON_CONFIG_LOW | HOTMON_CONFIG_HIGH),
		(tcdm_legacy_base + PRCM_REQ_MB4_HOTMON_CONFIG));
	writeb(MB4H_HOTMON, (tcdm_legacy_base + PRCM_MBOX_HEADER_REQ_MB4));

	writel(MBOX_BIT(4), PRCM_MBOX_CPU_SET);
	WARN_ON(wait_for_completion_timeout(&mb4.work, MB4TIM) == 0);

	mutex_unlock(&mb4.lock);

	return 0;
}

static int config_hot_period(u16 val)
{
	mutex_lock(&mb4.lock);

	while (readl(PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writew(val, (tcdm_legacy_base + PRCM_REQ_MB4_HOT_PERIOD));
	writeb(MB4H_HOT_PERIOD, (tcdm_legacy_base + PRCM_MBOX_HEADER_REQ_MB4));

	writel(MBOX_BIT(4), PRCM_MBOX_CPU_SET);
	WARN_ON(wait_for_completion_timeout(&mb4.work, MB4TIM) == 0);

	mutex_unlock(&mb4.lock);

	return 0;
}

static int start_temp_sense(u16 cycles32k)
{
	if (cycles32k == 0xFFFF)
		return -EINVAL;

	return config_hot_period(cycles32k);
}

static int stop_temp_sense(void)
{
	return config_hot_period(0xFFFF);
}

/**
* db9540_prcmu_thsensor_get_temp - get AP9540 currrent temperature
* Returns: current temperature value on success, non-zero on failure
*/
static int thsensor_get_temp(void)
{
	struct upap_req req;
	struct upap_ack ack;
	int val = 0;

	/* Prepare request (header and req_state) */
	req.service_id = UX540_PRCM_UPAP_SERVICE_THSENSOR;
	req.command_id = U9540_PRCM_UPAP_COMMAND_THSENSOR_GET_TEMP;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, 0);

	/* Check response from PRCMU */
	if ((ack.service_id == UX540_PRCM_UPAP_SERVICE_THSENSOR) &&
		(ack.command_id == U9540_PRCM_UPAP_COMMAND_THSENSOR_GET_TEMP)
		&& (ack.status == U9540_PRCM_UPAP_STATUS_OK)) {
		pr_debug("PRCMU sensor read: %d\n", ack.sensor_read);
		val = ack.sensor_read;
	} else {
		pr_info("PRCMU - bad ack in %s. 0x%x 0x%x %u %u\n", __func__,
		ack.service_id, ack.command_id,
		ack.status, ack.sensor_read);
		return -EIO;
	}

	return val;
}

static bool is_ac_wake_requested(void)
{
	return (atomic_read(&ac_wake_req_state) != 0);
}

void prcmu_reset_hva(void)
{
	writel(PRCM_C2C_RESETN_HVA_MEM | PRCM_C2C_RESETN_HVA_LOGIC,
			PRCM_C2C_RESETN_CLR);
	writel(PRCM_C2C_RESETN_HVA_MEM | PRCM_C2C_RESETN_HVA_LOGIC,
			PRCM_C2C_RESETN_SET);
}
EXPORT_SYMBOL(prcmu_reset_hva);

void prcmu_reset_hx170(void)
{
	writel(PRCM_C2C_RESETN_G1_MEM | PRCM_C2C_RESETN_G1_LOGIC,
			PRCM_C2C_RESETN_CLR);
	writel(PRCM_C2C_RESETN_G1_MEM | PRCM_C2C_RESETN_G1_LOGIC,
			PRCM_C2C_RESETN_SET);
}
EXPORT_SYMBOL(prcmu_reset_hx170);

/**
 * get_reset_code - Retrieve SW reset reason code
 *
 * Retrieves the reset reason code stored by prcmu_system_reset() before
 * last restart.
 */
static u16 get_reset_code(void)
{
	return reset_code_copy;
}

/**
 * get_reset_status - Retrieve reset status
 *
 * Retrieves the value of the reset status register as read at startup.
 */
u32 get_reset_status(void)
{
	return reset_status_copy;
}

static void prcmu_modem_reset_db9540(void)
{
	struct upap_req req;
	struct upap_ack ack;

	/* prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_MODEM;
	req.command_id = U9540_PRCM_UPAP_COMMAND_RESET_MODEM;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, 0);

	/*
	 * No need to check return from PRCMU as modem should go in reset state
	 * This state is already managed by upper layer
	 */
}

/**
 * prcmu_reset_modem - ask the PRCMU to reset modem
 */
void modem_reset(void)
{
	trace_u8500_modem_reset(0);

	prcmu_modem_reset_db9540();
}

void prcmu_c2c_request_notif_up(void)
{
	struct upap_req req;
	struct upap_ack ack;

	/* prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_C2C_XMIP;
	req.command_id = U9540_PRCM_UPAP_COMMAND_C2CNOTIFYME;
	req.data.data = U9540_PRCM_REQ_DATA_C2C_NOTIFYME;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;

	upap_send_request(&req, &ack, sizeof(u32));
}
EXPORT_SYMBOL(prcmu_c2c_request_notif_up);

void prcmu_c2c_request_reset(void)
{
	struct upap_req req;
	struct upap_ack ack;

	/* prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_C2C_XMIP;
	req.command_id = U9540_PRCM_UPAP_COMMAND_C2CRESET;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;
	upap_send_request(&req, &ack, 0);
}
EXPORT_SYMBOL(prcmu_c2c_request_reset);

static inline void print_unknown_header_warning(u8 n, u8 header)
{
	pr_warning("prcmu: Unknown message header (%d) in mailbox %d.\n",
		header, n);
}

static void upap_print_unknown_header_warning(
	u32 service, u32 command, u32 status)
{
	pr_warning("prcmu: Unknown service (0x%x) and command (0x%x) in UniqPAP."
			"Returned status (%u)\n",
		service, command, status);
}

static void upap_read_service_dvfs(struct upap_req *req,
		struct upap_ack *ack)
{
	switch (ack->command_id) {
	case U9540_PRCM_UPAP_COMMAND_SET_ARM_OPP:
		ack->arm_freq = req->data.data;
		break;

	case U9540_PRCM_UPAP_COMMAND_SET_APE_OPP:
		/* No response data for this service ID and command ID. */
		break;

	case U9540_PRCM_UPAP_COMMAND_SET_SAFE_OPP:
		/* No response data for this service ID and command ID. */
		break;

	default:
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
		break;
	}
}

static void upap_read_service_usb(struct upap_req *req,
		struct upap_ack *ack)
{
	/* No response data for this service ID. Just check command ID is OK */
	if (unlikely(ack->command_id != U9540_PRCM_UPAP_COMMAND_USB_WAKEUP_REL))
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
}

static void upap_read_service_clock(struct upap_req *req,
		struct upap_ack *ack)
{
	/* No response data for this service ID. Just check command ID is OK */
	if (unlikely(ack->command_id != U9540_PRCM_UPAP_COMMAND_PLL_ON_OFF))
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
}

static void upap_read_service_modem(struct upap_req *req,
		struct upap_ack *ack)
{
	/* No response data for this service ID. Just check command ID is OK */
	if (unlikely(ack->command_id != U9540_PRCM_UPAP_COMMAND_RESET_MODEM))
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
}

static void upap_read_service_cpuhotplug(struct upap_req *req,
		struct upap_ack *ack)
{
	/* No response data for this service ID. Just check command ID is OK */
	if (unlikely((ack->command_id != U9540_PRCM_UPAP_COMMAND_CPU1_UNPLUG) &&
			(ack->command_id != U9540_PRCM_UPAP_COMMAND_CPU1_REPLUG)
			))
	  upap_print_unknown_header_warning(ack->service_id,
	  ack->command_id, ack->status);
}

static void upap_read_service_ddr(struct upap_req *req,
		struct upap_ack *ack)
{
	/* No response data for this service ID. Just check command ID is OK */
	if (unlikely(ack->command_id !=
				U9540_PRCM_UPAP_COMMAND_DDR_SLEEP_STRAT))
		upap_print_unknown_header_warning(ack->service_id,
				ack->command_id, ack->status);
}

static void upap_read_service_c2c(struct upap_req *req,
		struct upap_ack *ack)
{
	if ((ack->command_id == U9540_PRCM_UPAP_COMMAND_C2CNOTIFYME) ||
		(ack->command_id == U9540_PRCM_UPAP_COMMAND_C2CRESET)) {
		if (ack->status != 0)
			pr_warning("prcmu: C2C service (0x%x) / command (0x%x) in "
				"upap. Error status (%u)\n",
				ack->service_id, ack->command_id, ack->status);
	} else {
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
	}
}

static void upap_read_service_thsensor(struct upap_req *req,
		struct upap_ack *ack)
{
	switch (ack->command_id) {
	case U9540_PRCM_UPAP_COMMAND_THSENSOR_GET_TEMP:
		ack->sensor_read = req->data.data;
		break;

	default:
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
		break;
	}
}

static void upap_read_ack(struct upap_req *req)
{
	struct upap_ack *ack = upap_transfer.ack;

	ack->service_id = req->service_id;
	ack->command_id = req->command_id;
	ack->status = req->status;

	if ((ack->service_id < UPAP_SERVICES_NB) &&
		(upap_read_services[ack->service_id] != NULL))
		upap_read_services[ack->service_id](req, ack);
	else
		upap_print_unknown_header_warning(ack->service_id,
				ack->command_id, ack->status);

	/* Update mailbox state */
	req->req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_IDLE;

	complete(&upap_transfer.work);
}

static void upap_read_service_xmip(struct upap_req *req,
		struct upap_ack *ack)
{
	if (ack->command_id == U8540_PRCM_UPAP_COMMAND_XMIP_NOTIFICATION) {
		if (ack->status != 0)
			pr_warning("prcmu: XMIP service (0x%x) / command (0x%x) in "
				"upap. Error status (%u)\n", ack->service_id,
				ack->command_id, ack->status);
	} else {
		upap_print_unknown_header_warning(ack->service_id,
			ack->command_id, ack->status);
	}
}

int prcmu_set_service_n(int  state)
{
	if (state)
		writel(PRCM_APE2_SERVICE_N_BIT, PRCM_APE2_RESETN_SET);
	else
		writel(PRCM_APE2_SERVICE_N_BIT, PRCM_APE2_RESETN_CLR);
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_set_service_n);

int prcmu_get_service_n(void)
{
	if (readl(PRCM_APE2_RESETN_VAL) & PRCM_APE2_SERVICE_N_BIT)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_get_service_n);

int prcmu_get_xmip_reset_n(void)
{
	if (readl(PRCM_APE2_RESETN_VAL) & PRCM_APE2_XMIP_RESETN_BIT)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_get_xmip_reset_n);

int prcmu_set_xmip_reset_n(int  state)
{
	if (state)
		writel(PRCM_APE2_XMIP_RESETN_BIT, PRCM_APE2_RESETN_SET);
	else
		writel(PRCM_APE2_XMIP_RESETN_BIT, PRCM_APE2_RESETN_CLR);
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_set_xmip_reset_n);

int prcmu_get_modem_resout2_n(void)
{
	if (readl(PRCM_LINE_VALUE) & PRCM_RESOUT2_N_BIT)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_get_modem_resout2_n);

int prcmu_get_modem_resout0_n(void)
{
	if (readl(PRCM_LINE_VALUE) & PRCM_RESOUT0_N_BIT)
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(prcmu_get_modem_resout0_n);

static void upap_read_nfy(struct upap_nfy *nfy)
{
	int i, found;
	u8 data[UPAP_NFY_DATA_SZ];

	/* Recopy nfy data to limit risk of TCDM corruption */
	memcpy(data, nfy->data.full_data_buf, UPAP_NFY_DATA_SZ);

	/* several notifiers can register to the same event */
	for (i = UPAP_NFYID_MAX - 1, found = 0; i >= 0; i--) {
		if ((nfy->service_id == upap_nfy_cmds[i][0]) &&
			(nfy->command_id == upap_nfy_cmds[i][1])) {
			atomic_notifier_call_chain(&prcmu_nfy_notifiers[i],
					0, (void *)data);
			found = 1;
		}
	}
	if (!found) {
		pr_warning("prcmu: unexpected upap/nfy: ID=0x%x/0x%x\n",
				nfy->service_id, nfy->command_id);
	}

	/* Update mailbox state */
	nfy->nfy_state = UX540_PRCM_UPAP_NFY_STATE_IDLE;
}

static bool upap_handler(void)
{
	/* ack interuption */
	writel(MBOX_BIT(upap_conf.mbox_nb), PRCM_ARM_IT1_CLR);

	if (upap_conf.req->req_state == UX540_PRCM_UPAP_REQ_STATE_ACK_SENT)
		upap_read_ack(upap_conf.req);

	if (upap_conf.nfy->nfy_state == UX540_PRCM_UPAP_NFY_STATE_SENT)
		upap_read_nfy(upap_conf.nfy);

	return false;
}

void prcmu_xmip_modapp_notif(u32 bitmask, u32 edgel, u32 edgeh)
{
	struct upap_req req;
	struct upap_ack ack;

	/* prepare request */
	req.service_id = UX540_PRCM_UPAP_SERVICE_C2C_XMIP;
	req.command_id = U8540_PRCM_UPAP_COMMAND_XMIP_NOTIFICATION;
	req.req_state = UX540_PRCM_UPAP_REQ_STATE_REQ_SENT;
	req.data.xmip_evt_data.resout0n_notif_req =
		U8540_PRCM_UPAP_XMIPRESOUT0N_NOTIFREQUESTED;
	req.data.xmip_evt_data.resout2n_notif_req =
		U8540_PRCM_UPAP_XMIPRESOUT2N_NOTIFREQUESTED;
	req.data.xmip_evt_data.modapp_enablebits = bitmask;
	req.data.xmip_evt_data.modapp_edgesens_h = edgeh;
	req.data.xmip_evt_data.modapp_edgesens_l = edgel;
	upap_send_request(&req, &ack, sizeof(u32));
}
EXPORT_SYMBOL(prcmu_xmip_modapp_notif);

int prcmu_register_modem(char *name)
{
	if (!name)
		return -EINVAL;

	if (connected_modem == MODEM_NONE) {
		if (!strcmp(name, "c2c")) {
			connected_modem = MODEM_C2C;
			upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
				upap_read_service_c2c);
		} else if (!strcmp(name, "xmip")) {
			connected_modem = MODEM_XMIP;
			upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_C2C_XMIP,
				upap_read_service_xmip);
		} else {
			pr_warn("%s: invalid modem\n", __func__);
			return -ENXIO;
		}
	} else {
		pr_err("%s: modem already registered\n", __func__);
		return -EBUSY;
	}
	return 0;
}
EXPORT_SYMBOL(prcmu_register_modem);

int prcmu_unregister_modem(char *name)
{
	u32 modem;

	if (!name)
		return -EINVAL;

	if (!strcmp(name, "c2c")) {
		modem = MODEM_C2C;
	} else if (!strcmp(name, "xmip")) {
		modem = MODEM_XMIP;
	} else {
		pr_warn("%s: invalid modem\n", __func__);
		return -ENXIO;
	}

	if ((connected_modem == MODEM_NONE) || (connected_modem != modem)) {
		pr_warn("%s: modem not registered\n", __func__);
		return -ENODEV;
	}
	else {
		upap_unregister_ack_service(UX540_PRCM_UPAP_SERVICE_C2C_XMIP);
		connected_modem = MODEM_NONE;
	}
	return 0;
}
EXPORT_SYMBOL(prcmu_unregister_modem);

static bool (*dbx540_read_mailbox[NUM_MB])(void) = {
	db8500_prcmu_read_mailbox_0,
	upap_handler,
	db8500_prcmu_read_mailbox_2,
	db8500_prcmu_read_mailbox_3,
	db8500_prcmu_read_mailbox_4,
	db8500_prcmu_read_mailbox_5,
	db8500_prcmu_read_mailbox_6,
	db8500_prcmu_read_mailbox_7
};


static  struct prcmu_val_data val_tab[] = {
	{
		.val = APE_OPP,
		.set_val = set_ape_opp,
		.get_val = get_ape_opp,
	},
	{
		.val = DDR_OPP,
		.set_val = set_ddr_opp,
		.get_val = get_ddr_opp,
	},
	{
		.val = DDR1_OPP,
		.get_val = get_ddr1_opp,
	},
	{
		.val = EFF_DDR_OPP,
		.get_val = get_effective_ddr_opp,
	},
	{
		.val = EFF_DDR1_OPP,
		.get_val = get_effective_ddr1_opp,
	},

};

static struct prcmu_out_data out_tab[] = {
	{
		.out = SPI2_MUX,
		.enable =  db8500_prcmu_enable_spi2,
		.disable = db8500_prcmu_disable_spi2,
	},
	{
		.out = STM_APE_MUX,
		.enable = db8500_prcmu_enable_stm_ape,
		.disable = db8500_prcmu_disable_stm_ape,
	},
	{
		.out = STM_MOD_UART_MUX,
		.enable = db8500_prcmu_enable_stm_mod_uart,
		.disable = db8500_prcmu_disable_stm_mod_uart,
	}
};

static struct prcmu_early_data early_fops = {
	/*  system reset  */
	.system_reset = db8500_prcmu_system_reset,

	/*  clock service */
	.config_clkout = config_clkout,
	.request_clock = dbx540_prcmu_request_clock,

	/*  direct register access */
	.read = db8500_prcmu_read,
	.write =  db8500_prcmu_write,
	.write_masked = db8500_prcmu_write_masked,
	/* others */
	.round_clock_rate = dbx540_prcmu_round_clock_rate,
	.set_clock_rate = dbx540_prcmu_set_clock_rate,
	.clock_rate = dbx540_prcmu_clock_rate,
	.vc = db8500_prcmu_vc,
};

static struct prcmu_fops_register early_tab[] = {
	{
		.fops = PRCMU_EARLY,
		.data.pearly = &early_fops
	},
	{
		.fops = PRCMU_VAL,
		.size = ARRAY_SIZE(val_tab),
		.data.pval = val_tab
	},
	{
		.fops = PRCMU_OUT,
		.size = ARRAY_SIZE(out_tab),
		.data.pout = out_tab
	}
};

static struct prcmu_fops_register_data early_data = {
	.size = ARRAY_SIZE(early_tab),
	.tab = early_tab
};

struct prcmu_probe_data probe_fops = {
	/* sysfs soc inf */
	.get_reset_code = get_reset_code,

	/* pm/suspend.c/cpu freq */
	.config_esram0_deep_sleep = config_esram0_deep_sleep,
	.set_power_state = db8500_prcmu_set_power_state,
	.get_power_state_result = db8500_prcmu_get_power_state_result,
	.enable_wakeups = db8500_prcmu_enable_wakeups,
	.is_ac_wake_requested = is_ac_wake_requested,

	/* modem */
	.modem_reset = modem_reset,

	/*  regulators */
	.set_epod = db8500_prcmu_set_epod,

	/* no used at all */
	.config_abb_event_readout = db8500_prcmu_config_abb_event_readout,
	.get_abb_event_buffer = db8500_prcmu_get_abb_event_buffer,

	/* abb access */
	.abb_read = db8500_prcmu_abb_read,
	.abb_read_no_irq = db8500_prcmu_abb_read_no_irq,
	.abb_write = db8500_prcmu_abb_write,
	.get_reset_status = get_reset_status,
	/*  other u8500 specific */
	.request_ape_opp_100_voltage = request_ape_opp_100_voltage,
	.configure_auto_pm = db8500_prcmu_configure_auto_pm,

	/* abb specific access */
	.abb_write_masked = db8500_prcmu_abb_write_masked,

	/* watchdog */
	.config_a9wdog = db8500_prcmu_config_a9wdog,
	.enable_a9wdog = db8500_prcmu_enable_a9wdog,
	.disable_a9wdog = db8500_prcmu_disable_a9wdog,
	.kick_a9wdog = db8500_prcmu_kick_a9wdog,
	.load_a9wdog = db8500_prcmu_load_a9wdog,
};

struct prcmu_probe_ux540_data probex540_fops = {

	.stay_in_wfi_check = stay_in_wfi_check,
	.replug_cpu1 = replug_cpu1,
	.unplug_cpu1 = unplug_cpu1,
};

static struct prcmu_fops_register probe_tab[] = {
	{
		.fops = PRCMU_PROBE,
		.data.pprobe = &probe_fops,
	},
	{
		.fops = PRCMU_PROBE_UX540,
		.data.pprobeux540 =&probex540_fops,
	},
	{
		.fops = PRCMU_APE_AGE,
		.data.check_ape_age = db9540_check_ap9540_age,
	}
};

struct prcmu_fops_register_data probe_data = {
	.size = ARRAY_SIZE(probe_tab),
	.tab = probe_tab,
};

struct prcmu_fops_register_data *__init
			dbx540_prcmu_early_init(struct prcmu_tcdm_map *map)
{
	void __iomem *sec_base;
	struct prcmu_context context;

	tcdm_base = ioremap_nocache(U8500_PRCMU_TCDM_BASE, map->tcdm_size);
	context.tcdm_base = tcdm_base;
	tcdm_legacy_base = context.tcdm_base + map->legacy_offset;
	context.tcdm_legacy_base = tcdm_legacy_base;

	/* read current max opp counter */
	arm_max_opp_profile.max_opp_cnt =
		arm_max_opp_profile.secure_memory;
	/*
	 * Copy the value of the reset status register and if needed also
	 * the software reset code.
	 */
	sec_base = ioremap_nocache(U8500_PRCMU_SEC_BASE, SZ_4K);
	if (sec_base != NULL) {
		reset_status_copy = readl(sec_base +
				DB8500_SEC_PRCM_RESET_STATUS);
		iounmap(sec_base);
	}
	if (reset_status_copy & DB8500_SEC_PRCM_RESET_STATUS_APE_SOFTWARE_RESET)
		reset_code_copy = readw(context.tcdm_legacy_base + PRCM_SW_RST_REASON);

	context.fw_trans = dbx540_fw_trans;
	context.fw_trans_nb = ARRAY_SIZE(dbx540_fw_trans);
	context.read_mbox = dbx540_read_mailbox;
	db8500_prcmu_context_init(&context);

	db8500_prcmu_init_mb0(&mb0);
	/*  mailbox 1 used by UniqPAP */
	db8500_prcmu_init_mb2(&mb2);
	db8500_prcmu_init_mb3(&mb3);
	db8500_prcmu_init_mb4(&mb4);
	db8500_prcmu_init_mb5(&mb5);

	/* initialize UniqPAP */
	upap_init();
	/* register UniqPAP services */
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_DVFS,
			upap_read_service_dvfs);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_USB,
			upap_read_service_usb);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_CLOCK,
			upap_read_service_clock);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_MODEM,
			upap_read_service_modem);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_CPUHOTPLUG,
			upap_read_service_cpuhotplug);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_THSENSOR,
			upap_read_service_thsensor);
	upap_register_ack_service(UX540_PRCM_UPAP_SERVICE_DDR,
			upap_read_service_ddr);
	/* UniqPAP Modem service are dynamically registered */
	connected_modem = MODEM_NONE;

	dvfs_context.ape_opp = APE_OPP_INIT;

	/* Initalize irqs. */
	db8500_prcmu_init_irq();

	/*  fixed it according to soc settings knowledge */
	latest_armss_rate = 1500000;
	return &early_data;
}

static void __init init_prcm_registers(void)
{
	u32 val;

	val = readl(PRCM_A9PL_FORCE_CLKEN);
	val &= ~(PRCM_A9PL_FORCE_CLKEN_PRCM_A9PL_FORCE_CLKEN |
		PRCM_A9PL_FORCE_CLKEN_PRCM_A9AXI_FORCE_CLKEN);
	writel(val, (PRCM_A9PL_FORCE_CLKEN));
}

void prcmu_set_sdmmc_psw(bool status)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&spare_out_lock, flags);
	val = readl(PRCM_SPARE_OUT);
	if (status)
		val |= PRCM_SPARE_OUT_PSW_SDMMC;
	else
		val &= ~PRCM_SPARE_OUT_PSW_SDMMC;
	writel(val, PRCM_SPARE_OUT);
	spin_unlock_irqrestore(&spare_out_lock, flags);
}

void prcmu_pullup_tdo(bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&prcmu_lock, flags);
	val = readl(PRCM_IOCR);
	if (enable)
		val &= ~PRCM_IOCR_TDO_PULLUP_ENABLE;
	else
		val |= PRCM_IOCR_TDO_PULLUP_ENABLE;
	writel(val, PRCM_IOCR);
	spin_unlock_irqrestore(&prcmu_lock, flags);
}

/*
 * Power domain switches (ePODs) modeled as regulators for the DB8500 SoC
 */
static struct regulator_consumer_supply db8500_vape_consumers[] = {
	REGULATOR_SUPPLY("v-ape", NULL),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.0"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.1"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.2"),
	REGULATOR_SUPPLY("v-i2c", "nmk-i2c.3"),
	/* "v-mmc" changed to "vcore" in the mainline kernel */
	REGULATOR_SUPPLY("vcore", "sdi0"),
	REGULATOR_SUPPLY("vcore", "sdi1"),
	REGULATOR_SUPPLY("vcore", "sdi2"),
	REGULATOR_SUPPLY("vcore", "sdi3"),
	REGULATOR_SUPPLY("vcore", "sdi4"),
	REGULATOR_SUPPLY("v-dma", "dma40.0"),
	REGULATOR_SUPPLY("v-ape", "ab8500-usb.0"),
	REGULATOR_SUPPLY("v-uart", "uart0"),
	REGULATOR_SUPPLY("v-uart", "uart1"),
	REGULATOR_SUPPLY("v-uart", "uart2"),
	REGULATOR_SUPPLY("v-ape", "nmk-ske-keypad.0"),
	REGULATOR_SUPPLY("v-hsi", "ste_hsi.0"),
};

static struct regulator_consumer_supply db8500_vsmps2_consumers[] = {
	REGULATOR_SUPPLY("musb_1v8", "ab8540-usb.0"),
	REGULATOR_SUPPLY("musb_1v8", "ab9540-usb.0"),
	REGULATOR_SUPPLY("musb_1v8", "ab8500-usb.0"),
	/* AV8100 regulator */
	REGULATOR_SUPPLY("hdmi_1v8", "0-0070"),
};

static struct regulator_consumer_supply db8500_b2r2_mcde_consumers[] = {
	REGULATOR_SUPPLY("vsupply", "b2r2_core"),
	REGULATOR_SUPPLY("vsupply", "b2r2_1_core"),
	REGULATOR_SUPPLY("vsupply", "mcde"),
	REGULATOR_SUPPLY("vsupply", "dsilink.0"),
	REGULATOR_SUPPLY("vsupply", "dsilink.1"),
	REGULATOR_SUPPLY("vsupply", "dsilink.2"),
};

/* SVA MMDSP regulator switch */
static struct regulator_consumer_supply db8500_svammdsp_consumers[] = {
	REGULATOR_SUPPLY("sva-mmdsp", "cm_control"),
};

/* SVA pipe regulator switch */
static struct regulator_consumer_supply db8500_svapipe_consumers[] = {
	REGULATOR_SUPPLY("sva-pipe", "cm_control"),
	REGULATOR_SUPPLY("v-hva", NULL),
	REGULATOR_SUPPLY("v-g1", NULL),
};

/* SIA MMDSP regulator switch */
static struct regulator_consumer_supply db8500_siammdsp_consumers[] = {
	REGULATOR_SUPPLY("sia-mmdsp", "cm_control"),
};

/* SIA pipe regulator switch */
static struct regulator_consumer_supply db8500_siapipe_consumers[] = {
	REGULATOR_SUPPLY("sia-pipe", "cm_control"),
};

static struct regulator_consumer_supply db8500_sga_consumers[] = {
	REGULATOR_SUPPLY("v-mali", NULL),
};

static struct regulator_consumer_supply db8500_vpll_consumers[] = {
	REGULATOR_SUPPLY("v-vpll", NULL),
};

/* ESRAM1 and 2 regulator switch */
static struct regulator_consumer_supply db8500_esram12_consumers[] = {
	REGULATOR_SUPPLY("esram12", "cm_control"),
};

/* ESRAM3 and 4 regulator switch */
static struct regulator_consumer_supply db8500_esram34_consumers[] = {
	REGULATOR_SUPPLY("v-esram34", "mcde"),
	REGULATOR_SUPPLY("esram34", "cm_control"),
	REGULATOR_SUPPLY("lcla_esram", "dma40.0"),
};

static struct regulator_init_data db9540_regulators[DB8500_NUM_REGULATORS] = {
	[DB8500_REGULATOR_VAPE] = {
		.constraints = {
			.name = "db8500-vape",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_vape_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_vape_consumers),
	},
	[DB8500_REGULATOR_VARM] = {
		.constraints = {
			.name = "db8500-varm",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VMODEM] = {
		.constraints = {
			.name = "db8500-vmodem",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VPLL] = {
		.constraints = {
			.name = "db8500-vpll",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_vpll_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_vpll_consumers),
	},
	[DB8500_REGULATOR_VSMPS1] = {
		.constraints = {
			.name = "db8500-vsmps1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VSMPS2] = {
		.constraints = {
			.name = "db8500-vsmps2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_vsmps2_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_vsmps2_consumers),
	},
	[DB8500_REGULATOR_VSMPS3] = {
		.constraints = {
			.name = "db8500-vsmps3",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_VRF1] = {
		.constraints = {
			.name = "db8500-vrf1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SVAMMDSP] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sva-mmdsp",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_svammdsp_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_svammdsp_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SVAMMDSPRET] = {
		.constraints = {
			/* "ret" means "retention" */
			.name = "db8500-sva-mmdsp-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SVAPIPE] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sva-pipe",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_svapipe_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_svapipe_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SIAMMDSP] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sia-mmdsp",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_siammdsp_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_siammdsp_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SIAMMDSPRET] = {
		.constraints = {
			.name = "db8500-sia-mmdsp-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_SIAPIPE] = {
		/* dependency to u8500-vape is handled outside regulator framework */
		.constraints = {
			.name = "db8500-sia-pipe",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_siapipe_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_siapipe_consumers),
	},
	[DB8500_REGULATOR_SWITCH_SGA] = {
		.supply_regulator = "db8500-vape",
		.constraints = {
			.name = "db8500-sga",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_sga_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_sga_consumers),

	},
	[DB8500_REGULATOR_SWITCH_B2R2_MCDE] = {
		.supply_regulator = "db8500-vape",
		.constraints = {
			.name = "db8500-b2r2-mcde",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_b2r2_mcde_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_b2r2_mcde_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM12] = {
		/*
		 * esram12 is set in retention and supplied by Vsafe when Vape is off,
		 * no need to hold Vape
		 */
		.constraints = {
			.name = "db8500-esram12",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_esram12_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_esram12_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM12RET] = {
		.constraints = {
			.name = "db8500-esram12-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
	[DB8500_REGULATOR_SWITCH_ESRAM34] = {
		/*
		 * esram34 is set in retention and supplied by Vsafe when Vape is off,
		 * no need to hold Vape
		 */
		.constraints = {
			.name = "db8500-esram34",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.consumer_supplies = db8500_esram34_consumers,
		.num_consumer_supplies = ARRAY_SIZE(db8500_esram34_consumers),
	},
	[DB8500_REGULATOR_SWITCH_ESRAM34RET] = {
		.constraints = {
			.name = "db8500-esram34-ret",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
	},
};

/*
 * Thermal Sensor
 */
static struct dbx500_temp_ops dbx540_temp_ops = {
	.config_hotdog = config_hotdog,
	.config_hotmon = config_hotmon,
	.start_temp_sense = start_temp_sense,
	.stop_temp_sense = stop_temp_sense,
	.thsensor_get_temp = thsensor_get_temp,
};

static struct dbx500_temp_pdata dbx540_temp_pdata = {
	.ops = &dbx540_temp_ops,
	.monitoring_active = true,
};


static struct resource ux540_thsens_resources[] = {
	{
		.name = "IRQ_HOTMON_LOW",
		.start  = IRQ_PRCMU_HOTMON_LOW,
		.end    = IRQ_PRCMU_HOTMON_LOW,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name = "IRQ_HOTMON_HIGH",
		.start  = IRQ_PRCMU_HOTMON_HIGH,
		.end    = IRQ_PRCMU_HOTMON_HIGH,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct ux500_pasr_data db9540_pasr_pdata[] = {
	{
		.name = "DDR0CS0",
		.apply_mask = dbx500_prcmu_config_pasr_mask,
		.base_addr = U9540_DDR0_CS0_BASE_ADDR,
		.mailbox = MB0H_CONFIG_PASR_DDR0_CS0,
	},
	{
		.name = "DDR0CS1",
		.apply_mask = dbx500_prcmu_config_pasr_mask,
		.base_addr = U9540_DDR0_CS1_BASE_ADDR,
		.mailbox = MB0H_CONFIG_PASR_DDR0_CS1,
	},
	{
		.name = "DDR1CS0",
		.apply_mask = dbx500_prcmu_config_pasr_mask,
		.base_addr = U9540_DDR1_CS0_BASE_ADDR,
		.mailbox = MB0H_CONFIG_PASR_DDR1_CS0,
	},
	{
		.name = "DDR1CS1",
		.apply_mask = dbx500_prcmu_config_pasr_mask,
		.base_addr = U9540_DDR1_CS1_BASE_ADDR,
		.mailbox = MB0H_CONFIG_PASR_DDR1_CS1,
	},
	{
		/*  End marker */
		.base_addr = 0xFFFFFFFF
	},
};

static struct mfd_cell db8500_prcmu_devs[] = {
	{
		.name = "cpufreq-ux500",
		.id = -1,
	},
	{
		.name = "db8500-prcmu-regulators",
		.platform_data = &db9540_regulators,
		.pdata_size = sizeof(db9540_regulators),
	},
	{
		.name = "ux500_wdt",
		.id = -1,
	},
	{
		.name = "dbx500_temp",
		.platform_data = &dbx540_temp_pdata,
		.pdata_size = sizeof(dbx540_temp_pdata),
		.resources       = ux540_thsens_resources,
		.num_resources  = ARRAY_SIZE(ux540_thsens_resources),
	},
	{
		.name = "dbx500-prcmu",
		.platform_data = &probe_data,
		.pdata_size = sizeof(probe_data),
	},
	{
		.name = "ux500-pasr",
		.platform_data = &db9540_pasr_pdata,
		.pdata_size = sizeof(db9540_pasr_pdata),
	},
};

/**
 * prcmu_fw_init - arch init call for the Linux PRCMU fw init logic
 *
 */
static int __init dbx540_prcmu_probe(struct platform_device *pdev)
{
	int err;
	int i;

	struct dbx540_prcmu_pdata *pdata =
		(struct dbx540_prcmu_pdata *)dev_get_platdata(&pdev->dev);
	init_prcm_registers();

	/*  init notifiers */
	for (i = 0; i < UPAP_NFYID_MAX; i++)
		ATOMIC_INIT_NOTIFIER_HEAD(&prcmu_nfy_notifiers[i]);

	/* Clean up the mailbox interrupts after pre-kernel code. */
	err = db8500_prcmu_clean_mailbox_irq();
	if (err < 0) {
		pr_err("prcmu: Failed to allocate IRQ_DB8500_PRCMU1.\n");
		err = -EBUSY;
		goto no_irq_return;
	}
	freq_table = pdata->cpufreq;
	enable_ape_opp_100_voltage = pdata->enable_ape_opp_100_voltage;

	for (i = 0; i < ARRAY_SIZE(db8500_prcmu_devs); i++) {
		if (!strcmp(db8500_prcmu_devs[i].name, "cpufreq-ux500")) {
			db8500_prcmu_devs[i].platform_data = pdata->cpufreq;
			db8500_prcmu_devs[i].pdata_size = pdata->cpufreq_size;
			break;
		}
	}

	update_freq_table(freq_table);

	err = mfd_add_devices(&pdev->dev, 0, db8500_prcmu_devs,
			ARRAY_SIZE(db8500_prcmu_devs), NULL,
			0);

	if (err)
		pr_err("prcmu: Failed to add subdevices\n");
	else
		pr_info("DBX540 PRCMU initialized\n");

	/*
	 * Temporary U9540 bringup code - Enable all clock gates.
	 * Write 1 to all bits of PRCM_YYCLKEN0_MGT_SET and
	 * PRCM_YYCLKEN1_MGT_SET registers.
	 */
	writel(~0, _PRCMU_BASE + 0x510); /* PRCM_YYCLKEN0_MGT_SET */
	writel(~0, _PRCMU_BASE + 0x514); /* PRCM_YYCLKEN1_MGT_SET */

	/*
	 * set a flag to indicate that prcmu drv is well initialised and that
	 * prcmu driver services can be called
	 */
	prcmu_driver_initialised = 1;
	cpu1_unplug_ongoing = 0;

no_irq_return:
	return err;
}

static struct platform_driver dbx540_prcmu_driver = {
	.driver = {
		.name = "dbx540-prcmu",
		.owner = THIS_MODULE,
	},
};

static int __init dbx540_prcmu_init(void)
{
	return platform_driver_probe(&dbx540_prcmu_driver, dbx540_prcmu_probe);
}

arch_initcall(dbx540_prcmu_init);

MODULE_DESCRIPTION("DBX540 PRCMU Unit driver");
MODULE_LICENSE("GPL v2");
