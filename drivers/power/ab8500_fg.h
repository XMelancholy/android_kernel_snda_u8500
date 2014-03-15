/*
 * Copyright (C) ST-Ericsson AB 2012
 *
 * Main and Back-up battery management driver.
 *
 * Note: Backup battery management is required in case of Li-Ion battery and not
 * for capacitive battery. HREF boards have capacitive battery and hence backup
 * battery management is not used and the supported code is available in this
 * driver.
 *
 * License Terms: GNU General Public License v2
 * Author: Johan Palsson <johan.palsson@stericsson.com>
 * Author: Karl Komierowski <karl.komierowski@stericsson.com>
 */

#define MILLI_TO_MICRO			1000
#define FG_LSB_IN_MA			1627
#define QLSB_NANO_AMP_HOURS_X10		1129
#define INS_CURR_TIMEOUT		(3 * HZ)

#define SEC_TO_SAMPLE(S)		(S * 4)

#define NBR_AVG_SAMPLES			20

#define LOW_BAT_CHECK_INTERVAL		(HZ / 16) /* 62.5 ms */

#define VALID_CAPACITY_SEC		(45 * 60) /* 45 minutes */
#define BATT_OK_MIN			2360 /* mV */
#define BATT_OK_INCREMENT		50 /* mV */
#define BATT_OK_MAX_NR_INCREMENTS	0xE

/* FG constants */
#define BATT_OVV			0x01

/**
 * struct ab8500_fg_interrupts - ab8500 fg interupts
 * @name:	name of the interrupt
 * @isr		function pointer to the isr
 */
struct ab8500_fg_interrupts {
	char *name;
	irqreturn_t (*isr)(int irq, void *data);
};

enum ab8500_fg_discharge_state {
	AB8500_FG_DISCHARGE_INIT,
	AB8500_FG_DISCHARGE_INITMEASURING,
	AB8500_FG_DISCHARGE_INIT_RECOVERY,
	AB8500_FG_DISCHARGE_RECOVERY,
	AB8500_FG_DISCHARGE_READOUT_INIT,
	AB8500_FG_DISCHARGE_READOUT,
	AB8500_FG_DISCHARGE_WAKEUP,
};

enum ab8500_fg_charge_state {
	AB8500_FG_CHARGE_INIT,
	AB8500_FG_CHARGE_READOUT,
};

enum ab8500_fg_calibration_state {
	AB8500_FG_CALIB_INIT,
	AB8500_FG_CALIB_WAIT,
	AB8500_FG_CALIB_END,
};

struct ab8500_fg_avg_cap {
	int avg;
	int samples[NBR_AVG_SAMPLES];
	__kernel_time_t time_stamps[NBR_AVG_SAMPLES];
	int pos;
	int nbr_samples;
	int sum;
};

struct ab8500_fg_cap_scaling {
	bool enable;
	int cap_to_scale[2];
	int disable_cap_level;
	int scaled_cap;
};

struct ab8500_fg_battery_capacity {
	int max_mah_design;
	int max_mah;
	int mah;
	int permille;
	int level;
	int prev_mah;
	int prev_percent;
	int prev_level;
	int user_mah;
	struct ab8500_fg_cap_scaling cap_scale;
};

struct ab8500_fg_flags {
	bool fg_enabled;
	bool conv_done;
	bool charging;
	bool fully_charged;
	bool force_full;
	bool low_bat_delay;
	bool low_bat;
	bool bat_ovv;
	bool batt_unknown;
	bool calibrate;
	bool user_cap;
	bool batt_id_received;
};

struct inst_curr_result_list {
	struct list_head list;
	int *result;
};

/**
 * struct ab8500_fg_test - ab8500 FG device information in test mode
 * @enable:			true if fg in test mode else false
 * @cc_int_offset:		offset for internal calibration
 * @cc_soft_offset:		offset for software calibration
 * @cc_sample_conv:		sample read
 * @cc_sample_conv_calib_uA:	sample converted in uA
 * @cceoc_complete:		pointer to the struct completion, to indicate
 *				the completion of internal calibration and
 *				one sample reading
 * @nconv_accu_complete:	pointer to the struct completion, to indicate
 *				the completion of sample to accumulate
 * @cc_int_calib_complete:	pointer to the struct completion, to indicate
 *				the completion of internal calibration
 * @lock:			Mutex for locking the CC
 */
struct ab8500_fg_test {
	bool enable;
	u8 cc_int_offset;
	u8 cc_soft_offset;
	u16 cc_sample_conv;
	int cc_sample_conv_calib_uA;
	struct completion cceoc_complete;
	struct completion nconv_accu_complete;
	struct completion cc_int_calib_complete;
	struct mutex lock;
};

/**
 * struct ab8500_fg - ab8500 FG device information
 * @dev:		Pointer to the structure device
 * @node:		a list of AB8500 FGs, hence prepared for reentrance
 * @irq			holds the CCEOC interrupt number
 * @vbat:		Battery voltage in mV
 * @vbat_nom:		Nominal battery voltage in mV
 * @inst_curr:		Instantenous battery current in mA
 * @avg_curr:		Average battery current in mA
 * @bat_temp		battery temperature
 * @fg_samples:		Number of samples used in the FG accumulation
 * @accu_charge:	Accumulated charge from the last conversion
 * @recovery_cnt:	Counter for recovery mode
 * @high_curr_cnt:	Counter for high current mode
 * @init_cnt:		Counter for init mode
 * @low_bat_cnt		Counter for number of consecutive low battery measures
 * @nbr_cceoc_irq_cnt	Counter for number of CCEOC irqs received since enabled
 * @recovery_needed:	Indicate if recovery is needed
 * @high_curr_mode:	Indicate if we're in high current mode
 * @init_capacity:	Indicate if initial capacity measuring should be done
 * @turn_off_fg:	True if fg was off before current measurement
 * @calib_state		State during offset calibration
 * @discharge_state:	Current discharge state
 * @charge_state:	Current charge state
 * @ab8500_fg_started	Completion struct used for the instant current start
 * @ab8500_fg_complete	Completion struct used for the instant current reading
 * @flags:		Structure for information about events triggered
 * @bat_cap:		Structure for battery capacity specific parameters
 * @avg_cap:		Average capacity filter
 * @parent:		Pointer to the struct ab8500
 * @gpadc:		Pointer to the struct gpadc
 * @pdata:		Pointer to the ab8500_fg platform data
 * @bat:		Pointer to the ab8500_bm platform data
 * @fg_psy:		Structure that holds the FG specific battery properties
 * @fg_wq:		Work queue for running the FG algorithm
 * @fg_periodic_work:	Work to run the FG algorithm periodically
 * @fg_low_bat_work:	Work to check low bat condition
 * @fg_reinit_work	Work used to reset and reinitialise the FG algorithm
 * @fg_work:		Work to run the FG algorithm instantly
 * @fg_acc_cur_work:	Work to read the FG accumulator
 * @fg_check_hw_failure_work:	Work for checking HW state
 * @cc_lock:		Mutex for locking the CC
 * @fg_kobject:		Structure of type kobject
 * @test:		Structure of type ab8500_fg_test for test purpose
 * @fg_inst_wake_lock»  Instant current wake_lock
 */
struct ab8500_fg {
	struct device *dev;
	struct list_head node;
	int irq;
	int vbat;
	int vbat_nom;
	int inst_curr;
	int avg_curr;
	int bat_temp;
	int fg_samples;
	int accu_charge;
	int recovery_cnt;
	int high_curr_cnt;
	int init_cnt;
	int low_bat_cnt;
	int nbr_cceoc_irq_cnt;
	bool recovery_needed;
	bool high_curr_mode;
	bool init_capacity;
	bool turn_off_fg;
	enum ab8500_fg_calibration_state calib_state;
	enum ab8500_fg_discharge_state discharge_state;
	enum ab8500_fg_charge_state charge_state;
	struct completion ab8500_fg_started;
	struct completion ab8500_fg_complete;
	struct ab8500_fg_flags flags;
	struct ab8500_fg_battery_capacity bat_cap;
	struct ab8500_fg_avg_cap avg_cap;
	struct ab8500 *parent;
	struct ab8500_gpadc *gpadc;
	struct abx500_fg_platform_data *pdata;
	struct abx500_bm_data *bat;
	struct power_supply fg_psy;
	struct workqueue_struct *fg_wq;
	struct delayed_work fg_periodic_work;
	struct delayed_work fg_low_bat_work;
	struct delayed_work fg_reinit_work;
	struct work_struct fg_work;
	struct work_struct fg_acc_cur_work;
	struct delayed_work fg_check_hw_failure_work;
	struct mutex cc_lock;
	struct kobject fg_kobject;
	struct ab8500_fg_test test;
	struct wake_lock fg_inst_wake_lock;
};

extern char *discharge_state[];
extern char *charge_state[];

int ab8500_fg_coulomb_counter(struct ab8500_fg *di, bool enable);
void ab8500_fg_charge_state_to(struct ab8500_fg *di,
		enum ab8500_fg_charge_state new_state);
void ab8500_fg_discharge_state_to(struct ab8500_fg *di,
		enum ab8500_fg_charge_state new_state);
/* test initialization */
#ifdef CONFIG_AB8500_BM_DEEP_DEBUG
void ab8500_fg_test_init(struct ab8500_fg *di);
#else
void ab8500_fg_test_init(struct ab8500_fg *di) {return; }
#endif
