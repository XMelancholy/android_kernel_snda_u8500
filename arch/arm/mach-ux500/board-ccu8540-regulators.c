/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License Terms: GNU General Public License v2
 *
 * Authors: Alexandre Torgue <alexandre.torgue@stericsson.com>
 *
 *
 * CCU8540 board specific initialization for regulators
 */

#include <linux/kernel.h>

#include <asm/mach-types.h>

#include <mach/id.h>
#include "board-ccu8540-regulators.h"

static struct regulator_consumer_supply ab8540_vaux1_consumers[] = {
	/* Main display, u8500 R3 uib */
	REGULATOR_SUPPLY("vddi", "mcde_disp_sony_acx424akp.0"),
	/* Main display, u8500 uib and ST uib */
	REGULATOR_SUPPLY("vdd1", "samsung_s6d16d0.0"),
	/* Secondary display, ST uib */
	REGULATOR_SUPPLY("vdd1", "samsung_s6d16d0.1"),
	/* Main display on tablet UIB */
	REGULATOR_SUPPLY("vdd1", "toshiba_dsi2lvds.0"),
	/* Main display, U9540_S_V2 UIB */
	REGULATOR_SUPPLY("vddi", "himax_hx8392.0"),
	/* SFH7741 proximity sensor */
	REGULATOR_SUPPLY("vcc", "gpio-keys.0"),
	/* BH1780GLS ambient light sensor */
	REGULATOR_SUPPLY("vcc", "2-0029"),
	/* lsm303dlh accelerometer */
	REGULATOR_SUPPLY("vdd", "2-0018"),
	/* lsm303dlhc accelerometer */
	REGULATOR_SUPPLY("vdd", "2-0019"),
	/* lsm303dlh magnetometer */
	REGULATOR_SUPPLY("vdd", "2-001e"),
	/* Rohm BU21013 Touchscreen devices */
	REGULATOR_SUPPLY("avdd", "3-005c"),
	REGULATOR_SUPPLY("avdd", "3-005d"),
	/* Synaptics RMI4 Touchscreen device */
	REGULATOR_SUPPLY("vdd", "3-004b"),
	/* L3G4200D Gyroscope device */
	REGULATOR_SUPPLY("vdd", "2-006a"),
	REGULATOR_SUPPLY("vdd", "2-0068"),
	/* Ambient light sensor device */
	REGULATOR_SUPPLY("vdd", "3-0029"),
	/* Pressure sensor device */
	REGULATOR_SUPPLY("vdd", "2-005c"),
	/* Cypress TrueTouch Touchscreen device */
	REGULATOR_SUPPLY("vcpin", "spi8.0"),
	/* Camera device */
	REGULATOR_SUPPLY("vaux12v5", "mmio_camera"),
	REGULATOR_SUPPLY("vaux12v5", "mmio_camera_raw"),
	REGULATOR_SUPPLY("vaux12v5", "mmio_camera_yuv"),
};

static struct regulator_consumer_supply ab8540_vaux2_consumers[] = {
	/* On-board eMMC power */
	REGULATOR_SUPPLY("vmmc", "sdi4"),
	/* AB8500 audio codec */
	REGULATOR_SUPPLY("vcc-N2158", "ab850x-codec.0"),
	/* AB8500 accessory detect 1 */
	REGULATOR_SUPPLY("vcc-N2158", "ab8500-acc-det.0"),
	/* AB8500 Tv-out device */
	REGULATOR_SUPPLY("vcc-N2158", "mcde_tv_ab8500.4"),
	/* AV8100 HDMI device */
	REGULATOR_SUPPLY("vcc-N2158", "av8100_hdmi.3"),
};

static struct regulator_consumer_supply ab8540_vaux3_consumers[] = {
	REGULATOR_SUPPLY("v-SD-STM", "stm"),
	/* External MMC slot power */
	REGULATOR_SUPPLY("vmmc", "sdi0"),
};
static struct regulator_consumer_supply ab8540_vaux4_consumers[] = {
	/* NFC and standalone secure element device */
	REGULATOR_SUPPLY("vnfc-se", "st21nfca"),
};

static struct regulator_consumer_supply ab8540_vtvout_consumers[] = {
	/* TV-out DENC supply */
	REGULATOR_SUPPLY("vtvout", "ab8500-denc.0"),
	/* Internal general-purpose ADC */
	REGULATOR_SUPPLY("vddadc", "ab8500-gpadc.0"),
	/* ADC for charger */
	REGULATOR_SUPPLY("vddadc", "ab8500-charger.0"),
	/* ADC for external AC charger */
	REGULATOR_SUPPLY("vddadc", "1-002c"),
	/* AB8500 Tv-out device */
	REGULATOR_SUPPLY("vtvout", "mcde_tv_ab8500.4"),
};

static struct regulator_consumer_supply ab8540_vaudio_consumers[] = {
	/* AB8500 audio codec device */
	REGULATOR_SUPPLY("v-audio", NULL),
};

static struct regulator_consumer_supply ab8540_vamic1_consumers[] = {
	/* AB8500 audio codec device */
	REGULATOR_SUPPLY("v-amic1", NULL),
};

static struct regulator_consumer_supply ab8540_vamic2_consumers[] = {
	/* AB8500 audio codec device */
	REGULATOR_SUPPLY("v-amic2", NULL),
};

static struct regulator_consumer_supply ab8540_vdmic_consumers[] = {
	/* AB8500 audio codec device */
	REGULATOR_SUPPLY("v-dmic", NULL),
};

static struct regulator_consumer_supply ab8540_vintcore_consumers[] = {
	/* SoC core supply, no device */
	REGULATOR_SUPPLY("v-intcore", NULL),
	/* USB Transceiver */
	REGULATOR_SUPPLY("vddulpivio18", "ab8540-usb.0"),
};

static struct regulator_consumer_supply ab8540_vana_consumers[] = {
	/* DB8500 DSI */
	REGULATOR_SUPPLY("vdddsi1v2", "mcde"),
	REGULATOR_SUPPLY("vdddsi1v2", "b2r2_core"),
	REGULATOR_SUPPLY("vdddsi1v2", "b2r2_1_core"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.0"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.1"),
	REGULATOR_SUPPLY("vdddsi1v2", "dsilink.2"),
	/* DB8500 CSI */
	REGULATOR_SUPPLY("vddcsi1v2", "mmio_camera"),
	REGULATOR_SUPPLY("vddcsi1v2", "mmio_camera_raw"),
	REGULATOR_SUPPLY("vddcsi1v2", "mmio_camera_yuv"),
};

static struct regulator_consumer_supply ab8540_vsdio_consumers[] = {
	/* SDMMC0 internal L/S */
	REGULATOR_SUPPLY("vmmc_io", "sdi0"),
};

static struct regulator_consumer_supply ab8540_sysclkreq_2_consumers[] = {
	/* CG2900 device */
	REGULATOR_SUPPLY("gbf_1v8", "cg2900-uart.0"),
};

static struct regulator_consumer_supply ab8540_sysclkreq_4_consumers[] = {
	/* CW1200 device */
	REGULATOR_SUPPLY("wlan_1v8", "cw1200_wlan.0"),
};

/* ab8540 regulator register initialization */
struct ab8500_regulator_reg_init
ab8540_regulator_reg_init[AB8540_NUM_REGULATOR_REGISTERS] = {
	/*
	 * VarmRequestCtrl          = force in HP whatever VxRequest
	 * VapeRequestCtrl          = force in HP whatever VxRequest
	 * Vsmps1RequestCtrl	    = force in HP whatever VxRequest
	 * Vsmps2RequestCtrl	    = force in HP whatever VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUREQUESTCTRL1,       0xff, 0xff),
	/*
	 * Vsmps3RequestCtrl	    = force in HP whatever VxRequest
	 * VanaRequestCtrl          = force in HP whatever VxRequest
	 * VpllRequestCtrl          = force in HP whatever VxRequest
	 * VextSupply1RequestCtrl   = force in HP whatever VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUREQUESTCTRL2,       0xff, 0xff),
	/*
	 * VextSupply2RequestCtrl   = force in HP whatever VxRequest
	 * VextSupply3RequestCtrl   = force in HP whatever VxRequest
	 * Vaux1RequestCtrl         = force in HP whatever VxRequest
	 * Vaux2RequestCtrl         = force in HP whatever VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUREQUESTCTRL3,       0xff, 0xff),
	/*
	 * Vaux3RequestCtrl         = force in HP whatever VxRequest
	 * SwHPReq                  = Control through SWValid disabled
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUREQUESTCTRL4,       0x07, 0x03),
	/*
	 * Vsmps1HwHPReq1Valid	    = doesn't valid HW config1
	 * Vsmps2HwHPReq1Valid      = doesn't valid HW config1
	 * Vsmps3HwHPReq1Valid      = doesn't valid HW config1
	 * VanaHwHPReq1Valid        = doesn't valid HW config1
	 * Vaux1HwHPreq1Valid       = doesn't valid HW config1
	 * Vaux2HwHPReq1Valid       = doesn't valid HW config1
	 * Vaux3HwHPReqValid        = doesn't valid HW config1
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUHWHPREQ1VALID1,     0xff, 0x00),
	/*
	 * VextSupply1HwHPReq1Valid = doesn't valid HW config1
	 * VextSupply2HwHPReq1Valid = doesn't valid HW config1
	 * VextSupply3HwHPReq1Valid = doesn't valid HW config1
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUHWHPREQ1VALID2,     0x07, 0x00),
	/*
	 * Vsmps1HwHPReq2Valid	    = doesn't valid HW config2
	 * Vsmps2HwHPReq2Valid      = doesn't valid HW config2
	 * Vsmps3HwHPReq2Valid      = doesn't valid HW config2
	 * VanaHwHPReq2Valid        = doesn't valid HW config2
	 * Vaux1HwHPreq2Valid       = doesn't valid HW config2
	 * Vaux2HwHPReq2Valid       = doesn't valid HW config2
	 * Vaux3HwHPReq2Valid       = doesn't valid HW config2
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUHWHPREQ2VALID1,     0xff, 0x00),
	/*
	 * VextSupply1HwHPReq2Valid = doesn't valid HW config2
	 * VextSupply2HwHPReq2Valid = doesn't valid HW config2
	 * VextSupply3HwHPReq2Valid = doesn't valid HW config2
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUHWHPREQ2VALID2,     0x07, 0x04),
	/*
	 * VapeSwHPReqValid         = not controlled by SW
	 * VarmSwHPReqValid         = not controlled by SW
	 * Vsmps1SwHPReqValid       = not controlled by SW
	 * Vsmps2SwHPReqValid       = not controlled by SW
	 * Vsmps3SwHPReqValid       = not controlled by SW
	 * VanaSwHPReqValid         = not controlled by SW
	 * VpllSwHPReqValid         = not controlled by SW
	 * Vaux1SwHPReqValid        = not controlled by SW
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUSWHPREQVALID1,      0xff, 0x00),
	/*
	 * Vaux2SwHPReqValid        = not controlled by SW
	 * Vaux3SwHPReqValid        = not controlled by SW
	 * VextSupply1SwHPReqValid  = not controlled by SW
	 * VextSupply2SwHPReqValid  = not controlled by SW
	 * VextSupply3SwHPReqValid  = not controlled by SW
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUSWHPREQVALID2,      0x1f, 0x00),
	/*
	 * SysClkReq1Valid1         = SysClkReq1 controlled
	 * SysClkReq2Valid1         = disabled
	 * SysClkReq3Valid1         = SysClkReq3 controlled
	 * SysClkReq4Valid1         = disabled
	 * SysClkReq5Valid1         = SysClkReq5 controlled
	 * SysClkReq6Valid1         = disabled
	 * SysClkReq7Valid1         = disabled
	 * SysClkReq8Valid1         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUSYSCLKREQVALID1,    0xff, 0x2a),
	/*
	 * SysClkReq2Valid2         = disabled
	 * SysClkReq3Valid2         = disabled
	 * SysClkReq4Valid2         = disabled
	 * SysClkReq5Valid2         = disabled
	 * SysClkReq6Valid2         = disabled
	 * SysClkReq7Valid2         = disabled
	 * SysClkReq8Valid2         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUSYSCLKREQVALID2,    0xff, 0x00),
	/*
	 * sysclkreq1 requests VCLKB to be ON
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUVCLKBREQVALID,      0x0f, 0x0E),
	/*
	 * sysclkreq1 requests VRF1 to be ON
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUVRF1REQVALID,      0x0f, 0x0E),
	/*
	 * VTVoutEna                = disabled
	 * Vintcore12Ena            = disabled
	 * Vintcore12Sel            = 1.25 V
	 * Vintcore12LP             = inactive (HP)
	 * VTVoutLP                 = inactive (HP)
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUMISC1,              0xfe, 0x10),
	/*
	 * VaudioEna                = disabled
	 * VdmicEna                 = disabled
	 * Vamic1Ena                = disabled
	 * Vamic2Ena                = disabled
	 * Vamic12LP                = inactive
	 * VdmicSel(0)		    = inactive
	 * VdmicSel(1)		    = inactive
	 */
	INIT_REGULATOR_REGISTER(AB8540_VAUDIOSUPPLY,           0xfe, 0x00),
	/*
	 * Vsmps1Regu               = force in HP
	 * Vsmps1SelCtrl            = Vsmps1 voltage defined by Vsmsp1Sel2
	 */
	INIT_REGULATOR_REGISTER(AB8540_VSMPS1REGU,             0x3f, 0x05),
	/*
	 * Vsmps2Regu               = force in HP
	 * Vsmps2SelCtrl            = Vsmps2 voltage defined by Vsmsp2Sel2
	 */
	INIT_REGULATOR_REGISTER(AB8540_VSMPS2REGU,             0x3f, 0x05),
	/*
	 * VPll                     = force in HP
	 * VanaRegu                 = force in HP
	 */
	INIT_REGULATOR_REGISTER(AB8540_VPLLVANAREGU,           0x0f, 0x05),
	/*
	 * VextSupply1Regu          = force in HP
	 * VextSupply2Regu          = force in HP
	 * VextSupply3Regu          = force in HP
	 * ExtSupply2Bypass         = ExtSupply12LPn ball is 0 when Ena is 0
	 * ExtSupply3Bypass         = ExtSupply3LPn ball is 0 when Ena is 0
	 */
	INIT_REGULATOR_REGISTER(AB8540_EXTSUPPLYREGU,          0xff, 0x15),
	/*
	 * Vaux1Regu                = force in HP
	 * Vaux2Regu                = force in HP
	 */
	INIT_REGULATOR_REGISTER(AB8540_VAUX12REGU,             0x0f, 0x05),
	/*
	 * Vrf1Regu                 = HW control
	 * Vaux3Regu                = force in HP
	 */
	INIT_REGULATOR_REGISTER(AB8540_VRF1VAUX3REGU,          0x0f, 0x09),
	/*
	 * Vsmps1Sel2               = 1.2 V
	 */
	INIT_REGULATOR_REGISTER(AB8540_VSMPS1SEL2,             0x3f, 0x28),
	/*
	 * Vaux1Sel                 = 2.8 V
	 */
	INIT_REGULATOR_REGISTER(AB8540_VAUX1SEL,               0x0f, 0x0c),
	/*
	 * Vaux2Sel                 = 2.9 V
	 */
	INIT_REGULATOR_REGISTER(AB8540_VAUX2SEL,               0x0f, 0x0d),
	/*
	 * Vaux3Sel                 = 2.79 V
	 */
	INIT_REGULATOR_REGISTER(AB8540_VRF1VAUX3SEL,           0x77, 0x07),
	/*
	 * VextSupply12LP           = disabled (no LP)
	 */
	INIT_REGULATOR_REGISTER(AB8540_REGUCTRL2SPARE,         0x01, 0x00),
	/*
	 * VSimSycClkReq1Valid
	 * VSimSycClkReq2Valid
	 * VSimSycClkReq3Valid
	 * VSimSycClkReq4Valid
	 * VSimSycClkReq5Valid
	 * VSimSycClkReq6Valid
	 * VSimSycClkReq7Valid
	 * VSimSycClkReq8Valid
	 */
	INIT_REGULATOR_REGISTER(AB8540_VSIMSYSCLKCTRL,         0xff, 0x00),
	/*
	 * VCLKBRequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8540_VCLKBREQCTRL,           0x03, 0x02),
	/*
	 * VCLKBRegu: on in HP mode
	 */
	INIT_REGULATOR_REGISTER(AB8540_VCLKBREGU,              0x03, 0x01),
	/*
	 * VRF1RequestCtrl
	 */
	INIT_REGULATOR_REGISTER(AB8540_VRF1REQCTRL,            0x03, 0x02),
	/*
	 *
	 * On in HP at 1.2 V
	 *
	 * */
	INIT_REGULATOR_REGISTER(AB8540_VHSIC,                  0x3f, 0x1b),
	/*
	 * ON in HP at 1.8V
	 *
	 */
	INIT_REGULATOR_REGISTER(AB8540_VSDIO,                  0x3f, 0x1c),
	/*
	 *  VanaSel                 = 1.2 V
	 */
	INIT_REGULATOR_REGISTER(AB8540_VANAVPLLSEL,            0x07, 0x00),
};

struct regulator_init_data ab8540_regulators[AB8540_NUM_REGULATORS] = {
	/* supplies to the display/camera */
	[AB8540_LDO_AUX1] = {
		.constraints = {
			.name = "V-DISPLAY",
			.min_uV = 2800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.boot_on = 1, /* display is on at boot */
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vaux1_consumers),
		.consumer_supplies = ab8540_vaux1_consumers,
	},
	/* supplies to the on-board eMMC */
	[AB8540_LDO_AUX2] = {
		.constraints = {
			.name = "V-eMMC1",
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vaux2_consumers),
		.consumer_supplies = ab8540_vaux2_consumers,
	},
	/* supply for VAUX3, supplies to SDcard slots */
	[AB8540_LDO_AUX3] = {
		.constraints = {
			.name = "V-MMC-SD",
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vaux3_consumers),
		.consumer_supplies = ab8540_vaux3_consumers,
	},
	/* supply for VAUX4, supplies to NFC and standalone secure element */
	[AB8540_LDO_AUX4] = {
		.constraints = {
			.name = "V-NFC-SE",
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vaux4_consumers),
		.consumer_supplies = ab8540_vaux4_consumers,
	},
	/* supply for tvout, gpadc, TVOUT LDO */
	[AB8540_LDO_TVOUT] = {
		.constraints = {
			.name = "V-TVOUT",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vtvout_consumers),
		.consumer_supplies = ab8540_vtvout_consumers,
	},
	/* supply for ab8500-vaudio, VAUDIO LDO */
	[AB8540_LDO_AUDIO] = {
		.constraints = {
			.name = "V-AUD",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vaudio_consumers),
		.consumer_supplies = ab8540_vaudio_consumers,
	},
	/* supply for v-anamic1 VAMic1-LDO */
	[AB8540_LDO_ANAMIC1] = {
		.constraints = {
			.name = "V-AMIC1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vamic1_consumers),
		.consumer_supplies = ab8540_vamic1_consumers,
	},
	/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
	[AB8540_LDO_ANAMIC2] = {
		.constraints = {
			.name = "V-AMIC2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vamic2_consumers),
		.consumer_supplies = ab8540_vamic2_consumers,
	},
	/* supply for v-dmic, VDMIC LDO */
	[AB8540_LDO_DMIC] = {
		.constraints = {
			.name = "V-DMIC",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vdmic_consumers),
		.consumer_supplies = ab8540_vdmic_consumers,
	},
	/* supply for v-intcore12, VINTCORE12 LDO */
	[AB8540_LDO_INTCORE] = {
		.constraints = {
			.name = "V-INTCORE",
			.min_uV = 1250000,
			.max_uV = 1350000,
			.input_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE |
					  REGULATOR_CHANGE_DRMS,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					    REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vintcore_consumers),
		.consumer_supplies = ab8540_vintcore_consumers,
	},
	/* supply for U8500 CSI-DSI, VANA LDO */
	[AB8540_LDO_ANA] = {
		.constraints = {
			.name = "V-CSI-DSI",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vana_consumers),
		.consumer_supplies = ab8540_vana_consumers,
	},
	/* supply for v-mmcio, VSDIO LDO */
	[AB8540_LDO_SDIO] = {
		.constraints = {
			.name = "V-SDIO",
			.min_uV = 1050000,
			.max_uV = 3050000,
			.input_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS |
					  REGULATOR_CHANGE_MODE |
					  REGULATOR_CHANGE_DRMS,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8540_vsdio_consumers),
		.consumer_supplies = ab8540_vsdio_consumers,
	},
	/* sysclkreq 2 pin */
	[AB8540_SYSCLKREQ_2] = {
		.constraints = {
			.name = "V-SYSCLKREQ-2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8540_sysclkreq_2_consumers),
		.consumer_supplies = ab8540_sysclkreq_2_consumers,
	},
	/* sysclkreq 4 pin */
	[AB8540_SYSCLKREQ_4] = {
		.constraints = {
			.name = "V-SYSCLKREQ-4",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies =
			ARRAY_SIZE(ab8540_sysclkreq_4_consumers),
		.consumer_supplies = ab8540_sysclkreq_4_consumers,
	},
};

/*
 * AB8540 external regulators
 */
static struct regulator_init_data ab8540_ext_regulators[] = {

	[AB8500_EXT_SUPPLY1] = {
		.constraints = {
			.name = "ab8500-ext-supply1",
			.min_uV = 4500000,
			.max_uV = 4500000,
			.initial_mode = REGULATOR_MODE_IDLE,
			.boot_on = 1,
			.always_on = 1,
		},
	},
	/*not already hardwired */
	[AB8500_EXT_SUPPLY2] = {
		.constraints = {
			.name = "ab8500-ext-supply2",
		},
	},

	[AB8500_EXT_SUPPLY3] = {
		.constraints = {
			.name = "ab8500-ext-supply3",
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
		},
	},
};

struct ab8500_regulator_platform_data ab8540_regulator_plat_data = {
	.reg_init               = ab8540_regulator_reg_init,
	.num_reg_init           = ARRAY_SIZE(ab8540_regulator_reg_init),
	.regulator              = ab8540_regulators,
	 .num_regulator          = ARRAY_SIZE(ab8540_regulators),
	.ext_regulator          = ab8540_ext_regulators,
	.num_ext_regulator      = ARRAY_SIZE(ab8540_ext_regulators),
};
