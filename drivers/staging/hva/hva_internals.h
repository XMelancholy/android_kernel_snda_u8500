/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * ST-Ericsson HVA core
 * Author: <thomas.costis@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __HVA_INTERNALS_H__
#define __HVA_INTERNALS_H__

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/irqreturn.h>

#include <video/hva_common.h>

/* HW fifo size is limited to 8 tasks */
#define HW_HVA_FIFO_SIZE 8
/* Timeout for one job to be completed in ms */
#define HVA_TASK_TIMEOUT_IN_MS 300
/*
 * Delay of inactivity in ms after which we can suspend HVA
 * Avoid power on/off between 2 consecutive jobs
 */
#define HVA_AUTOSUSPEND_DELAY_MS 3
/* Max number of stored instance stats */
#define MAX_NB_DEBUGFS_INSTANCES 3

enum hva_state {
	HVA_OK = 0,
	HVA_STUCKED,
	HVA_RET_ERROR
};


/**
 * HVA struct for debugging instances
 */
struct stats {
	unsigned int   tag;
	unsigned long  min_duration, max_duration;
	unsigned long  sum_duration, session_duration;
	unsigned long  last_task_hw_start_ts, last_task_hw_stop_ts;
	unsigned int   total_task_nb;
	unsigned long  first_task_drv_start_ts;
};

/**
 * @dev:  The device handle
 * @data: Used to store a reference to hva_core
 * @stats: Stat data (dumped through debugfs)
 */
struct hva_instance {
	struct device   *dev;
	void            *data;
	struct stats    stats;
};

/**
 * HVA task
 * @id: Task id
 * @client_id: Client id
 * @cmd: Command sent to HW (defined in hif_cmd_type struct)
 * @task_desc: Task descriptor pointer
 * @status_reg: Status register upadted after task completion
 * @completed: Task completion status
 * @used: Mark task status in task array
 */
struct hva_task {
	__u16             id;
	__u8              client_id;
	__u8              cmd;
	__u32             task_desc;
	__u32             status_reg;
	struct completion completed;
	bool              used;
};

/**
 * Main struct hva
 * @dev: device pointer
 * @miscdev: misc device struct
 * @name[]: Name of the device
 * @chip_id: HW version
 * @irq_its: End of task IRQ
 * @irq_err: Error IRQ
 * @hva_hw_cb: End of task callback
 * @hva_hw_err_cb: Error callback
 * @hw_base_addr: HVA base address
 * @regs_size: HVA total register range
 * @task_lock: Lock on task (semaphore is mandatory: no mutex)
 * @task: Task array
 * @hw_fifo_lock: Lock on task array
 * @hw_lock: HW registers access protection
 * @instance_nb: Number of instances/clients
 * @hw_state: HW state (defined in enum hva_state)
 * @nb_of_tasks: Number of task currently in HW (debug purpose)
 * @max_nb_of_tasks: Max number of task for one instance
 * @clk: HVA clock
 * @regulator: HVA regulator
 * @stats: Array of hva_stats for previous instances
 * @index_stats: Index in array of hva_stats
 * @debugfs_root_dir: Root dir of hva in debugfs
 */
struct hva_core {
	struct device     *dev;
	struct miscdevice miscdev;
	char              name[16];
	__u32             chip_id;

	__u32             irq_its, irq_err;
	void              (*hva_hw_cb)(struct hva_core *, __u32);
	void              (*hva_hw_err_cb)(struct hva_core *);
	void __iomem      *hw_base_addr;
	int               regs_size;

	struct semaphore  task_lock;
	struct hva_task   task[HW_HVA_FIFO_SIZE];
	struct semaphore  hw_fifo_lock;

	spinlock_t        hw_lock;

	int               instance_nb;
	enum hva_state    hw_state;

	int               nb_of_tasks;
	int               max_nb_of_tasks;

	struct clk        *clk;
	struct regulator  *regulator;

	struct stats      stats[MAX_NB_DEBUGFS_INSTANCES];
	int               index_stats;

	struct dentry     *debugfs_root_dir;
};

void hva_opp_on(struct hva_core *core, struct file *filp);
void hva_opp_off(struct hva_core *core, struct file *filp);
void hva_clock_on(struct hva_core *core);
void hva_clock_off(struct hva_core *core);
int hva_hw_reset(struct hva_core *core);
int hva_write_cfg_registers(struct hva_core *core);

int hva_get_reg(struct hva_core *core, struct hva_reg *reg);
unsigned long int hva_check_hardware_id(struct hva_core *core);
int hva_check_task_errors(struct hva_instance *instance, struct hva_task *task,
		unsigned int *status, unsigned int *info);
void hva_trigger_task(struct hva_core *core, struct hva_task *task);
irqreturn_t hva_irq_its_handler(int irq, void *x);
irqreturn_t hva_irq_err_handler(int irq, void *x);

int hva_debug_init(struct hva_core *core);
void hva_debug_exit(struct hva_core *core);

#endif
