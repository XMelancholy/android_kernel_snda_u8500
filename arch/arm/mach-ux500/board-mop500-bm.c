/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License terms:  GNU General Public License (GPL), version 2
 *
 * U8500 board specific charger and battery initialization parameters.
 *
 * Author: Johan Palsson <johan.palsson@stericsson.com> for ST-Ericsson.
 * Author: Johan Gardsmark <johan.gardsmark@stericsson.com> for ST-Ericsson.
 *
 */
#include <linux/interrupt.h>
#include <linux/pm2301_charger.h>
#include <linux/power_supply.h>
#include <linux/bug.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ab8500-bm.h>
#include <linux/mfd/abx500/ab8500-pwmleds.h>
#include "board-mop500-bm.h"
#include "board-ccu9540.h"

/*
 * This array maps the raw hex value to charger output current used by the
 * AB8500 values
 */
static int ab8500_charge_output_curr_map[] = {
	100,	200,	300,	400,	500,	600,	700,	800,
	900,	1000,	1100,	1200,	1300,	1400,	1500,	1500,
};

static int ab8540_charge_output_curr_map[] = {
	0,      0,      0,      75,     100,	125,	150,	175,
	200,	225,	250,	275,	300,	325,	350,	375,
	400,	425,	450,	475,	500,	525,	550,	575,
	600,	625,	650,	675,	700,	725,	750,	775,
	800,	825,	850,	875,	900,	925,	950,	975,
	1000,	1025,	1050,	1075,	1100,	1125,	1150,	1175,
	1200,	1225,	1250,	1275,	1300,	1325,	1350,	1375,
	1400,	1425,	1450,	1500,	1600,	1700,	1900,	2000,
};

/*
 * This array maps the raw hex value to VBUS input current used by the AB8500
 * Values
 */
static int ab8500_charge_input_curr_map[] = {
	50,     98,     193,	290,	380,	450,	500,	600,
	700,	800,	900,	1000,	1100,	1300,	1400,	1500,
};

static int ab8540_charge_input_curr_map[] = {
	25,     50,     75,     100,	125,	150,	175,	200,
	225,	250,	275,	300,	325,	350,	375,	400,
	425,	450,	475,	500,	525,	550,	575,	600,
	625,	650,	675,	700,	725,	750,	775,	800,
	825,	850,	875,	900,	925,	950,	975,	1000,
	1025,	1050,	1075,	1100,	1125,	1150,	1175,	1200,
	1225,	1250,	1275,	1300,	1325,	1350,	1375,	1400,
	1425,	1450,	1475,	1500,	1500,	1500,	1500,	1500,
};

/*
 * These are the defined batteries that uses a NTC and ID resistor placed
 * inside of the battery pack.
 * Note that the res_to_temp table must be strictly sorted by falling resistance
 * values to work.
 */
static struct abx500_res_to_temp temp_tbl_A[] = {
	{-5, 53407},
	{ 0, 48594},
	{ 5, 43804},
	{10, 39188},
	{15, 34870},
	{20, 30933},
	{25, 27422},
	{30, 24347},
	{35, 21694},
	{40, 19431},
	{45, 17517},
	{50, 15908},
	{55, 14561},
	{60, 13437},
	{65, 12500},
};
static struct abx500_res_to_temp temp_tbl_B[] = {
	{-5, 165418},
	{ 0, 159024},
	{ 5, 151921},
	{10, 144300},
	{15, 136424},
	{20, 128565},
	{25, 120978},
	{30, 113875},
	{35, 107397},
	{40, 101629},
	{45,  96592},
	{50,  92253},
	{55,  88569},
	{60,  85461},
	{65,  82869},
};
/* Discharge curve for 10mA load for Lowe batteries */
static struct abx500_v_to_cap cap_tbl_type1[] = {
	{4171,  100},
	{4114,   95},
	{4009,   83},
	{3947,   74},
	{3907,   67},
	{3863,   59},
	{3830,   56},
	{3813,   53},
	{3791,   46},
	{3771,   33},
	{3754,   25},
	{3735,   20},
	{3717,   17},
	{3681,   13},
	{3664,    8},
	{3651,    6},
	{3635,    5},
	{3560,    3},
	{3408,    1},
	{3247,    0},
};
static struct abx500_v_to_cap cap_tbl_A[] = {
	{4179,	100},
	{4117,	 95},
	{4073,	 90},
	{4028,	 85},
	{3988,	 80},
	{3956,	 75},
	{3925,	 70},
	{3897,	 65},
	{3869,	 60},
	{3827,	 55},
	{3806,	 50},
	{3791,	 45},
	{3780,	 40},
	{3773,	 35},
	{3766,	 30},
	{3754,	 25},
	{3734,	 20},
	{3695,	 15},
	{3677,	 10},
	{3637,	  5},
	{3251,	  0},
};
static struct abx500_v_to_cap cap_tbl_B[] = {
	{4161,	100},
	{4124,	 98},
	{4044,	 90},
	{4003,	 85},
	{3966,	 80},
	{3933,	 75},
	{3888,	 67},
	{3849,	 60},
	{3813,	 55},
	{3787,	 47},
	{3772,	 30},
	{3751,	 25},
	{3718,	 20},
	{3681,	 16},
	{3660,	 14},
	{3589,	 10},
	{3546,	  7},
	{3495,	  4},
	{3404,	  2},
	{3250,	  0},
};
static struct abx500_v_to_cap cap_tbl[] = {
	{4186,	100},
	{4163,	 99},
	{4114,	 95},
	{4068,	 90},
	{3990,	 80},
	{3926,	 70},
	{3898,	 65},
	{3866,	 60},
	{3833,	 55},
	{3812,	 50},
	{3787,	 40},
	{3768,	 30},
	{3747,	 25},
	{3730,	 20},
	{3705,	 15},
	{3699,	 14},
	{3684,	 12},
	{3672,	  9},
	{3657,	  7},
	{3638,	  6},
	{3556,	  4},
	{3424,	  2},
	{3317,	  1},
	{3094,	  0},
};
static struct abx500_v_to_cap cap_s1e[] = {
		{4171, 100},
		{4123,  98},
		{4077,  95},
		{3992,  86},
		{3934,  80},
		{3879,  75},
		{3838,  70},
		{3812,  65},
		{3786,  55},
		{3774,  40},
		{3769,  45},
		{3753,  30},
		{3741,  20},
		{3714,  13},
		{3690,  10},
		{3685,   8},
		{3681,   6},
		{3642,   4},
		{3515,   2},
		{3492,   1},
		{3305,   0},
};

/*
 * Note that the res_to_temp table must be strictly sorted by falling
 * resistance values to work.
 */
static struct abx500_res_to_temp temp_tbl[] = {
	{-5, 214834},
	{ 0, 162943},
	{ 5, 124820},
	{10,  96520},
	{15,  75306},
	{20,  59254},
	{25,  47000},
	{30,  37566},
	{35,  30245},
	{40,  24520},
	{45,  20010},
	{50,  16432},
	{55,  13576},
	{60,  11280},
	{65,   9425},
};

/*
 * Note that the table must be strictly sorted by falling
 * temperature values to work.
 */
static struct batres_vs_temp bat_therm_on_temp_to_batres_tbl[] = {
	{ 40, 120},
	{ 30, 135},
	{ 20, 165},
	{ 10, 230},
	{ 00, 325},
	{-10, 445},
	{-20, 595},
};

static struct batres_vs_temp temp_to_batres_tbl[] = {
	{ 60, 300},
	{ 30, 300},
	{ 20, 300},
	{ 10, 300},
	{ 00, 300},
	{-10, 300},
	{-20, 300},
};

static const struct abx500_battery_type bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.charge_full_design = 612,
		.nominal_voltage = 3700,
		.termination_vol = 4050,
		.termination_curr = 200,
		.recharge_cap = 95,
		.normal_cur_lvl = 400,
		.normal_vol_lvl = 4100,
		.maint_a_cur_lvl = 400,
		.maint_a_vol_lvl = 4050,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 400,
		.maint_b_vol_lvl = 4000,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 53407,
		.resis_low = 12500,
		.charge_full_design = 900,
		.nominal_voltage = 3600,
		.termination_vol = 4150,
		.termination_curr = 80,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATCTRL,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_A),
		.r_to_t_tbl = temp_tbl_A,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_A),
		.v_to_cap_tbl = cap_tbl_A,
		.n_batres_tbl_elements = ARRAY_SIZE(bat_therm_on_temp_to_batres_tbl),
		.batres_tbl = bat_therm_on_temp_to_batres_tbl,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 165418,
		.resis_low = 82869,
		.charge_full_design = 900,
		.nominal_voltage = 3600,
		.termination_vol = 4150,
		.termination_curr = 80,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATCTRL,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl_B),
		.r_to_t_tbl = temp_tbl_B,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_B),
		.v_to_cap_tbl = cap_tbl_B,
		.n_batres_tbl_elements = ARRAY_SIZE(bat_therm_on_temp_to_batres_tbl),
		.batres_tbl = bat_therm_on_temp_to_batres_tbl,
	},
/*
 * These are the batteries that doesn't have an internal NTC resistor to measure
 * its temperature. The temperature in this case is measure with a NTC placed
 * near the battery but on the PCB.
 */
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 76000,
		.resis_low = 53000,
		.charge_full_design = 900,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 30000,
		.resis_low = 10000,
		.battery_resistance = 160,
		.charge_full_design = 1880, //950,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_cur_lvl = 900, //700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(/*cap_tbl*/cap_s1e),
		.v_to_cap_tbl = /*cap_tbl*/cap_s1e,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 95000,
		.resis_low = 76001,
		.charge_full_design = 950,
		.nominal_voltage = 3700,
		.termination_vol = 4150,
		.termination_curr = 100,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
};

static const struct abx500_battery_type ab9540_bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.charge_full_design = 612,
		.nominal_voltage = 3700,
		.termination_vol = 4050,
		.termination_curr = 200,
		.recharge_cap = 95,
		.normal_cur_lvl = 400,
		.normal_vol_lvl = 4100,
		.maint_a_cur_lvl = 400,
		.maint_a_vol_lvl = 4050,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 400,
		.maint_b_vol_lvl = 4000,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.resis_high = 54500,
		.resis_low = 12500,
		.battery_resistance = 300,
		.charge_full_design = 1500,
		.nominal_voltage = 3600,
		.termination_vol = 4150,
		.termination_curr = 80,
		.recharge_cap = 95,
		.normal_cur_lvl = 700,
		.normal_vol_lvl = 4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4025,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
		.adc_therm = ABx500_ADC_THERM_BATTEMP,
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_type1),
		.v_to_cap_tbl = cap_tbl_type1,
		.n_batres_tbl_elements = ARRAY_SIZE(temp_to_batres_tbl),
		.batres_tbl = temp_to_batres_tbl,
	},
};

static char *ab8500_charger_supplied_to[] = {
	"abx500_chargalg",
	"ab8500_fg",
	"ab8500_btemp",
};

static char *pm2xxx_charger_supplied_to[] = {
	"abx500_chargalg",
	"ab8500_fg",
	"ab8500_btemp",
};

static char *ab8500_btemp_supplied_to[] = {
	"abx500_chargalg",
	"ab8500_fg",
};

static char *ab8500_fg_supplied_to[] = {
	"abx500_chargalg",
	"ab8500_usb",
};

static char *ab8500_chargalg_supplied_to[] = {
	"ab8500_fg",
};

struct abx500_charger_platform_data ab8500_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
	.autopower_cfg		= false,
	.ac_enabled		= true,
	.usb_enabled		= true,
};

struct abx500_charger_platform_data ab9540_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
	.autopower_cfg		= false,
	.ac_enabled		= false,
	.usb_enabled		= true,
};

struct abx500_charger_platform_data ab8540_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
	.autopower_cfg		= false,
	.ac_enabled		= false,
	.usb_enabled		= true,
	.usb_power_path		= true,
};

struct pm2xxx_charger_platform_data ccu9540_pm2xxx_charger_plat_data = {
	.supplied_to = pm2xxx_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm2xxx_charger_supplied_to),
	.label		= "pm2301",
	.gpio_irq_number = GPIO_171,
	.irq_type	= IRQF_SHARED | IRQF_NO_SUSPEND,
	/* AB GPIO 60 located at offset 54 */
	.lpn_gpio   = AB8500_PIN_GPIO(GPIO_60),
};

struct pm2xxx_charger_platform_data ccu8540_pm2xxx_charger_plat_data = {
	.supplied_to = pm2xxx_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm2xxx_charger_supplied_to),
	.label		= "pm2301",
	.gpio_irq_number = GPIO_38,
	.irq_type	= IRQF_SHARED | IRQF_NO_SUSPEND,
	.lpn_gpio   = AB8500_PIN_GPIO(52),
};

struct abx500_btemp_platform_data ab8500_btemp_plat_data = {
	.supplied_to = ab8500_btemp_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_btemp_supplied_to),
};

struct abx500_fg_platform_data ab8500_fg_plat_data = {
	.supplied_to = ab8500_fg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_fg_supplied_to),
};

struct abx500_chargalg_platform_data ab8500_chargalg_plat_data = {
	.supplied_to = ab8500_chargalg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_chargalg_supplied_to),
};

static struct ab8500_led_pwm leds_pwm_data[] = {
	[0] = {
		.pwm_id = 1,
		.blink_en = 1,
	},
	[1] = {
		.pwm_id = 2,
		.blink_en = 0,
	},
	[2] = {
		.pwm_id = 3,
		.blink_en = 0,
	},
};

struct ab8500_pwmled_platform_data ab8500_pwmled_plat_data = {
	.num_pwm = 3,
	.leds = leds_pwm_data,
};

static const struct abx500_bm_capacity_levels cap_levels = {
	.critical	= 2,
	.low		= 10,
	.normal		= 70,
	.high		= 95,
	.full		= 100,
};

static const struct abx500_fg_parameters fg = {
	.recovery_sleep_timer = 10,
	.recovery_total_time = 100,
	.init_timer = 1,
	.init_discard_time = 5,
	.init_total_time = 40,
	.high_curr_time = 60,
	.accu_charging = 30,
	.accu_high_curr = 30,
	.high_curr_threshold = 50,
	.lowbat_threshold = 3100,
	.battok_falling_th_sel0 = 2860,
	.battok_raising_th_sel1 = 2860,
	.maint_thres = 95,
	.user_cap_limit = 15,
	.pcut_enable = 1,
	.pcut_max_time = 111,
	.pcut_flag_time = 96,
	.pcut_max_restart = 15,
	.pcut_debunce_time = 2,
};

static const struct abx500_maxim_parameters ab8500_maxi_params = {
	.ena_maxi = true,
	.chg_curr = 910,
	.wait_cycles = 10,
	.charger_curr_step = 100,
};

static const struct abx500_maxim_parameters abx540_maxi_params = {
	.ena_maxi = true,
	.chg_curr = 3000,
	.wait_cycles = 10,
	.charger_curr_step = 200,
};

static const struct abx500_bm_charger_parameters chg = {
	.usb_volt_max		= 5500,
	.usb_curr_max		= 1500,
	.ac_volt_max		= 7500,
	.ac_curr_max		= 1500,
};

struct abx500_bm_data ab8500_bm_data = {
	.temp_under		= 3,
	.temp_low		= 8,
	.temp_high		= 55,
	.temp_over		= 60,
	.main_safety_tmr_h	= 6,
	.temp_interval_chg	= 20,
	.temp_interval_nochg	= 120,
	.usb_safety_tmr_h	= 6,
	.bkup_bat_v		= BUP_VCH_SEL_3P1V,
	.bkup_bat_i		= BUP_ICH_SEL_150UA,
	.no_maintenance		= true,
	.capacity_scaling	= false,
	.adc_therm		= ABx500_ADC_THERM_BATTEMP,
	.chg_unknown_bat	= false,
	.enable_overshoot	= false,
	.fg_res			= 115,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 4,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 3,
	.gnd_lift_resistance	= 34,
	.maxi			= &ab8500_maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.chg_output_curr	= ab8500_charge_output_curr_map,
	.n_chg_out_curr		= ARRAY_SIZE(ab8500_charge_output_curr_map),
	.chg_input_curr		= ab8500_charge_input_curr_map,
	.n_chg_in_curr		= ARRAY_SIZE(ab8500_charge_input_curr_map),
};

struct abx500_bm_data ab9540_bm_data = {
	.temp_under		= 3,
	.temp_low		= 8,
	.temp_high		= 43,
	.temp_over		= 48,
	.main_safety_tmr_h	= 4,
	.temp_interval_chg	= 20,
	.temp_interval_nochg	= 120,
	.usb_safety_tmr_h	= 4,
	.bkup_bat_v		= BUP_VCH_SEL_2P6V,
	.bkup_bat_i		= BUP_ICH_SEL_150UA,
	.no_maintenance		= false,
	.capacity_scaling	= false,
	.adc_therm		= ABx500_ADC_THERM_BATTEMP,
	.chg_unknown_bat	= false,
	.enable_overshoot	= false,
	.fg_res			= 100,
	.cap_levels		= &cap_levels,
	.bat_type		= ab9540_bat_type,
	.n_btypes		= ARRAY_SIZE(ab9540_bat_type),
	.batt_id		= 0,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 3,
	.gnd_lift_resistance	= 0,
	.maxi			= &abx540_maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.chg_output_curr	= ab8500_charge_output_curr_map,
	.n_chg_out_curr		= ARRAY_SIZE(ab8500_charge_output_curr_map),
	.chg_input_curr		= ab8500_charge_input_curr_map,
	.n_chg_in_curr		= ARRAY_SIZE(ab8500_charge_input_curr_map),
};

struct abx500_bm_data ab8540_bm_data = {
	.temp_under		= 3,
	.temp_low		= 8,
	.temp_high		= 43,
	.temp_over		= 48,
	.main_safety_tmr_h	= 4,
	.temp_interval_chg	= 20,
	.temp_interval_nochg	= 120,
	.usb_safety_tmr_h	= 4,
	.bkup_bat_v		= BUP_VCH_SEL_2P6V,
	.bkup_bat_i		= BUP_ICH_SEL_150UA,
	.no_maintenance		= false,
	.capacity_scaling	= false,
	.adc_therm		= ABx500_ADC_THERM_BATCTRL,
	.chg_unknown_bat	= false,
	.enable_overshoot	= false,
	.fg_res			= 100,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 0,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 3,
	.gnd_lift_resistance	= 0,
	.maxi			= &abx540_maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.chg_output_curr	= ab8540_charge_output_curr_map,
	.n_chg_out_curr		= ARRAY_SIZE(ab8540_charge_output_curr_map),
	.chg_input_curr		= ab8540_charge_input_curr_map,
	.n_chg_in_curr		= ARRAY_SIZE(ab8540_charge_input_curr_map),
};

static const struct pm2xxx_bm_charger_parameters pm2xxx_chg = {
	.ac_volt_max		= 7500,
	.ac_curr_max		= 3000,
};

struct pm2xxx_bm_data pm2xxx_bm_data = {
	.enable_overshoot	= false,
	.chg_params		= &pm2xxx_chg,
};
