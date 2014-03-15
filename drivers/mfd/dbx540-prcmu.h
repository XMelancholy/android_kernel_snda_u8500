/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Michel Jaouen <michel.jaouen@stericsson.com>
 *
 * PRCMU f/w APIs
 */
#ifndef __MFD_DBX540_PRCMU_H
#define __MFD_DBX540_PRCMU_H

#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/cpufreq.h>

#define KHZ_TO_HZ(A) (A*1000)
#define HZ_TO_KHZ(A) (A/1000)

enum ap_pwrst_trans_status_9540 {
	EXECUTETODEEPIDLE	= 0xE8,
	EXECUTETODEEPSLEEP	= 0xF8
};

#define PRCMU_DDR_SLEEP_STRAT_DDRCTRL_NB 2
#define PRCMU_DDR_SLEEP_STRAT_LP_MODE_NB 3

enum ddr_sleep_strat_ap_pwrst {
	DDR_SLEEP_STRAT_AP_IDLE_INDEX,
	DDR_SLEEP_STRAT_AP_DEEPIDLE_INDEX,
	DDR_SLEEP_STRAT_AP_SLEEP_INDEX,
};

enum ddr_ctrl_lp_mode {
	DDRCTRLSTATE_OFFHIGHLAT = 0,
	DDRCTRLSTATE_OFFLOWLAT,
	DDRCTRLSTATE_ON,
};

enum ddr_ctrl_nb {
	DDR_SLEEP_STRAT_DDRCTRL0,
	DDR_SLEEP_STRAT_DDRCTRL1
};

#ifdef CONFIG_MFD_DBX540_PRCMU
struct prcmu_fops_register_data *dbx540_prcmu_early_init(
		struct prcmu_tcdm_map *map);
int prcmu_set_vsafe_opp(u8 opp);
int prcmu_get_vsafe_opp(void);
int prcmu_set_ddr_sleep_strat_policy(u8 ddr_ctrl_nb, u8 lp_mode,
						u8 ddr_ctrl_mode);
void prcmu_set_sdmmc_psw(bool status);
void prcmu_c2c_request_notif_up(void);
void prcmu_c2c_request_reset(void);

void prcmu_reset_hva(void);
void prcmu_reset_hx170(void);
void prcmu_pullup_tdo(bool enable);

int prcmu_register_modem(char *name);
int prcmu_unregister_modem(char *name);

#else /* !CONFIG_MFD_DBX540_PRCMU */
static inline struct prcmu_fops_register_data *dbx540_prcmu_early_init(
		struct prcmu_tcdm_map *map)
{
	return NULL;
}

static inline int prcmu_set_vsafe_opp(u8 opp)
{
	return -EINVAL;
}

static inline int prcmu_get_vsafe_opp(void)
{
	return -EINVAL;
}

static inline int prcmu_set_ddr_sleep_strat_policy(u8 ddr_ctrl_nb, u8 lp_mode,
						u8 ddr_ctrl_mode)
{
	return -EINVAL;
}

static inline void db8540_prcmu_set_sdmmc_psw(bool status) {}
static inline void prcmu_c2c_request_notif_up(void) {}
static inline void prcmu_c2c_request_reset(void) {}
static inline void prcmu_reset_hva(void) {}
static inline void prcmu_reset_hx170(void) {}
static inline void prcmu_pullup_tdo(bool enable) {}

#endif /* !CONFIG_MFD_DBX540_PRCMU */

#endif /* __MFD_DBX540_PRCMU_H */
