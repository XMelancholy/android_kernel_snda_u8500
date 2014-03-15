/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Kumar Sanghvi <kumar.sanghvi@stericsson.com>
 *
 * PRCMU f/w APIs
 */
#ifndef __MFD_DB8500_PRCMU_H
#define __MFD_DB8500_PRCMU_H

#include <linux/interrupt.h>
#include <linux/bitops.h>

#define DB8500_PRCM_GPIOCR 0x138
#define DB8500_PRCM_GPIOCR_DBG_UARTMOD_CMD0	BIT(0)
#define DB8500_PRCM_GPIOCR_DBG_STM_APE_CMD	BIT(9)
#define DB8500_PRCM_GPIOCR_DBG_STM_MOD_CMD1	BIT(11)
#define DB8500_PRCM_GPIOCR_SPI2_SELECT		BIT(23)

/**
 * enum ret_state - general purpose On/Off/Retention states
 *
 */
enum ret_state {
	OFFST = 0,
	ONST  = 1,
	RETST = 2
};

/**
 * enum clk_arm - ARM Cortex A9 clock schemes
 * @A9_OFF:
 * @A9_BOOT:
 * @A9_OPPT1:
 * @A9_OPPT2:
 * @A9_EXTCLK:
 */
enum clk_arm {
	A9_OFF,
	A9_BOOT,
	A9_OPPT1,
	A9_OPPT2,
	A9_EXTCLK
};

/**
 * enum clk_gen - GEN#0/GEN#1 clock schemes
 * @GEN_OFF:
 * @GEN_BOOT:
 * @GEN_OPPT1:
 */
enum clk_gen {
	GEN_OFF,
	GEN_BOOT,
	GEN_OPPT1,
};

/* some information between arm and xp70 */

/**
 * enum romcode_write - Romcode message written by A9 AND read by XP70
 * @RDY_2_DS: Value set when ApDeepSleep state can be executed by XP70
 * @RDY_2_XP70_RST: Value set when 0x0F has been successfully polled by the
 *                 romcode. The xp70 will go into self-reset
 */
enum romcode_write {
	RDY_2_DS = 0x09,
	RDY_2_XP70_RST = 0x10
};

/**
 * enum romcode_read - Romcode message written by XP70 and read by A9
 * @INIT: Init value when romcode field is not used
 * @FS_2_DS: Value set when power state is going from ApExecute to
 *          ApDeepSleep
 * @END_DS: Value set when ApDeepSleep power state is reached coming from
 *         ApExecute state
 * @DS_TO_FS: Value set when power state is going from ApDeepSleep to
 *           ApExecute
 * @END_FS: Value set when ApExecute power state is reached coming from
 *         ApDeepSleep state
 * @SWR: Value set when power state is going to ApReset
 * @END_SWR: Value set when the xp70 finished executing ApReset actions and
 *          waits for romcode acknowledgment to go to self-reset
 */
enum romcode_read {
	INIT = 0x00,
	FS_2_DS = 0x0A,
	END_DS = 0x0B,
	DS_TO_FS = 0x0C,
	END_FS = 0x0D,
	SWR = 0x0E,
	END_SWR = 0x0F
};

/**
 * enum ap_pwrst - current power states defined in PRCMU firmware
 * @NO_PWRST: Current power state init
 * @AP_BOOT: Current power state is apBoot
 * @AP_EXECUTE: Current power state is apExecute
 * @AP_DEEP_SLEEP: Current power state is apDeepSleep
 * @AP_SLEEP: Current power state is apSleep
 * @AP_IDLE: Current power state is apIdle
 * @AP_RESET: Current power state is apReset
 */
enum ap_pwrst {
	NO_PWRST = 0x00,
	AP_BOOT = 0x01,
	AP_EXECUTE = 0x02,
	AP_DEEP_SLEEP = 0x03,
	AP_SLEEP = 0x04,
	AP_IDLE = 0x05,
	AP_RESET = 0x06
};

/**
 * enum hw_acc_state - State definition for hardware accelerator
 * @HW_NO_CHANGE: The hardware accelerator state must remain unchanged
 * @HW_OFF: The hardware accelerator must be switched off
 * @HW_OFF_RAMRET: The hardware accelerator must be switched off with its
 *               internal RAM in retention
 * @HW_ON: The hwa hardware accelerator hwa must be switched on
 *
 * NOTE! Deprecated, to be removed when all users switched over to use the
 * regulator API.
 */
enum hw_acc_state {
	HW_NO_CHANGE = 0x00,
	HW_OFF = 0x01,
	HW_OFF_RAMRET = 0x02,
	HW_ON = 0x04
};

/**
 * enum dvfs_stat - DVFS status messages definition
 * @DVFS_GO: A state transition DVFS is on going
 * @DVFS_ARM100OPPOK: The state transition DVFS has been completed for 100OPP
 * @DVFS_ARM50OPPOK: The state transition DVFS has been completed for 50OPP
 * @DVFS_ARMEXTCLKOK: The state transition DVFS has been completed for EXTCLK
 * @DVFS_NOCHGTCLKOK: The state transition DVFS has been completed for
 *                   NOCHGCLK
 * @DVFS_INITSTATUS: Value init
 */
enum dvfs_stat {
	DVFS_GO = 0xFF,
	DVFS_ARM100OPPOK = 0xFE,
	DVFS_ARM50OPPOK = 0xFD,
	DVFS_ARMEXTCLKOK = 0xFC,
	DVFS_NOCHGTCLKOK = 0xFB,
	DVFS_INITSTATUS = 0x00
};

/**
 * enum sva_mmdsp_stat - SVA MMDSP status messages
 * @SVA_MMDSP_GO: SVAMMDSP interrupt has happened
 * @SVA_MMDSP_INIT: Status init
 */
enum sva_mmdsp_stat {
	SVA_MMDSP_GO = 0xFF,
	SVA_MMDSP_INIT = 0x00
};

/**
 * enum sia_mmdsp_stat - SIA MMDSP status messages
 * @SIA_MMDSP_GO: SIAMMDSP interrupt has happened
 * @SIA_MMDSP_INIT: Status init
 */
enum sia_mmdsp_stat {
	SIA_MMDSP_GO = 0xFF,
	SIA_MMDSP_INIT = 0x00
};

/**
 * enum  mbox_to_arm_err - Error messages definition
 * @INIT_ERR: Init value
 * @PLLARMLOCKP_ERR: PLLARM has not been correctly locked in given time
 * @PLLDDRLOCKP_ERR: PLLDDR has not been correctly locked in the given time
 * @PLLSOC0LOCKP_ERR: PLLSOC0 has not been correctly locked in the given time
 * @PLLSOC1LOCKP_ERR: PLLSOC1 has not been correctly locked in the given time
 * @ARMWFI_ERR: The ARM WFI has not been correctly executed in the given time
 * @SYSCLKOK_ERR: The SYSCLK is not available in the given time
 * @BOOT_ERR: Romcode has not validated the XP70 self reset in the given time
 * @ROMCODESAVECONTEXT: The Romcode didn.t correctly save it secure context
 * @VARMHIGHSPEEDVALTO_ERR: The ARM high speed supply value transfered
 *          through I2C has not been correctly executed in the given time
 * @VARMHIGHSPEEDACCESS_ERR: The command value of VarmHighSpeedVal transfered
 *             through I2C has not been correctly executed in the given time
 * @VARMLOWSPEEDVALTO_ERR:The ARM low speed supply value transfered through
 *                     I2C has not been correctly executed in the given time
 * @VARMLOWSPEEDACCESS_ERR: The command value of VarmLowSpeedVal transfered
 *             through I2C has not been correctly executed in the given time
 * @VARMRETENTIONVALTO_ERR: The ARM retention supply value transfered through
 *                     I2C has not been correctly executed in the given time
 * @VARMRETENTIONACCESS_ERR: The command value of VarmRetentionVal transfered
 *             through I2C has not been correctly executed in the given time
 * @VAPEHIGHSPEEDVALTO_ERR: The APE highspeed supply value transfered through
 *                     I2C has not been correctly executed in the given time
 * @VSAFEHPVALTO_ERR: The SAFE high power supply value transfered through I2C
 *                         has not been correctly executed in the given time
 * @VMODSEL1VALTO_ERR: The MODEM sel1 supply value transfered through I2C has
 *                             not been correctly executed in the given time
 * @VMODSEL2VALTO_ERR: The MODEM sel2 supply value transfered through I2C has
 *                             not been correctly executed in the given time
 * @VARMOFFACCESS_ERR: The command value of Varm ON/OFF transfered through
 *                     I2C has not been correctly executed in the given time
 * @VAPEOFFACCESS_ERR: The command value of Vape ON/OFF transfered through
 *                     I2C has not been correctly executed in the given time
 * @VARMRETACCES_ERR: The command value of Varm retention ON/OFF transfered
 *             through I2C has not been correctly executed in the given time
 * @CURAPPWRSTISNOTBOOT:Generated when Arm want to do power state transition
 *             ApBoot to ApExecute but the power current state is not Apboot
 * @CURAPPWRSTISNOTEXECUTE: Generated when Arm want to do power state
 *              transition from ApExecute to others power state but the
 *              power current state is not ApExecute
 * @CURAPPWRSTISNOTSLEEPMODE: Generated when wake up events are transmitted
 *             but the power current state is not ApDeepSleep/ApSleep/ApIdle
 * @CURAPPWRSTISNOTCORRECTDBG:  Generated when wake up events are transmitted
 *              but the power current state is not correct
 * @ARMREGU1VALTO_ERR:The ArmRegu1 value transferred through I2C has not
 *                    been correctly executed in the given time
 * @ARMREGU2VALTO_ERR: The ArmRegu2 value transferred through I2C has not
 *                    been correctly executed in the given time
 * @VAPEREGUVALTO_ERR: The VApeRegu value transfered through I2C has not
 *                    been correctly executed in the given time
 * @VSMPS3REGUVALTO_ERR: The VSmps3Regu value transfered through I2C has not
 *                      been correctly executed in the given time
 * @VMODREGUVALTO_ERR: The VModemRegu value transfered through I2C has not
 *                    been correctly executed in the given time
 */
enum mbox_to_arm_err {
	INIT_ERR = 0x00,
	PLLARMLOCKP_ERR = 0x01,
	PLLDDRLOCKP_ERR = 0x02,
	PLLSOC0LOCKP_ERR = 0x03,
	PLLSOC1LOCKP_ERR = 0x04,
	ARMWFI_ERR = 0x05,
	SYSCLKOK_ERR = 0x06,
	BOOT_ERR = 0x07,
	ROMCODESAVECONTEXT = 0x08,
	VARMHIGHSPEEDVALTO_ERR = 0x10,
	VARMHIGHSPEEDACCESS_ERR = 0x11,
	VARMLOWSPEEDVALTO_ERR = 0x12,
	VARMLOWSPEEDACCESS_ERR = 0x13,
	VARMRETENTIONVALTO_ERR = 0x14,
	VARMRETENTIONACCESS_ERR = 0x15,
	VAPEHIGHSPEEDVALTO_ERR = 0x16,
	VSAFEHPVALTO_ERR = 0x17,
	VMODSEL1VALTO_ERR = 0x18,
	VMODSEL2VALTO_ERR = 0x19,
	VARMOFFACCESS_ERR = 0x1A,
	VAPEOFFACCESS_ERR = 0x1B,
	VARMRETACCES_ERR = 0x1C,
	CURAPPWRSTISNOTBOOT = 0x20,
	CURAPPWRSTISNOTEXECUTE = 0x21,
	CURAPPWRSTISNOTSLEEPMODE = 0x22,
	CURAPPWRSTISNOTCORRECTDBG = 0x23,
	ARMREGU1VALTO_ERR = 0x24,
	ARMREGU2VALTO_ERR = 0x25,
	VAPEREGUVALTO_ERR = 0x26,
	VSMPS3REGUVALTO_ERR = 0x27,
	VMODREGUVALTO_ERR = 0x28
};

enum hw_acc {
	SVAMMDSP = 0,
	SVAPIPE = 1,
	SIAMMDSP = 2,
	SIAPIPE = 3,
	SGA = 4,
	B2R2MCDE = 5,
	ESRAM12 = 6,
	ESRAM34 = 7,
};

enum cs_pwrmgt {
	PWRDNCS0  = 0,
	WKUPCS0   = 1,
	PWRDNCS1  = 2,
	WKUPCS1   = 3
};

/* Defs related to autonomous power management */

/**
 * enum sia_sva_pwr_policy - Power policy
 * @NO_CHGT:	No change
 * @DSPOFF_HWPOFF:
 * @DSPOFFRAMRET_HWPOFF:
 * @DSPCLKOFF_HWPOFF:
 * @DSPCLKOFF_HWPCLKOFF:
 *
 */
enum sia_sva_pwr_policy {
	NO_CHGT			= 0x0,
	DSPOFF_HWPOFF		= 0x1,
	DSPOFFRAMRET_HWPOFF	= 0x2,
	DSPCLKOFF_HWPOFF	= 0x3,
	DSPCLKOFF_HWPCLKOFF	= 0x4,
};

/**
 * enum auto_enable - Auto Power enable
 * @AUTO_OFF:
 * @AUTO_ON:
 *
 */
enum auto_enable {
	AUTO_OFF	= 0x0,
	AUTO_ON		= 0x1,
};

struct prcmu_fops_register_data *db8500_prcmu_early_init(
		struct prcmu_tcdm_map *map);
int prcmu_set_rc_a2p(enum romcode_write);
enum romcode_read prcmu_get_rc_p2a(void);
enum ap_pwrst prcmu_get_xp70_current_state(void);
int prcmu_release_usb_wakeup_state(void);
int db8500_prcmu_set_display_clocks(void);
int db8500_prcmu_disable_dsipll(void);
int db8500_prcmu_enable_dsipll(void);

#endif /* __MFD_DB8500_PRCMU_H */
