/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Etienne Carriere <etienne.carriere@stericsson.com>
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef _AP9540_C2C_H_
#define _AP9540_C2C_H_

/* C2C registers mapping */
#define C2COFF_REVISION	0x00
#define C2COFF_SYSCONFIG	0x04
#define C2COFF_SYSSTATUS	0x08
#define C2COFF_PORTCONFIG	0x0C
#define C2COFF_MIRRORMODE	0x10
#define C2COFF_IRQ_RAW_STATUS_0	0x14
#define C2COFF_IRQ_RAW_STATUS_1	0x18
#define C2COFF_IRQ_ENABLE_STATUS_0	0x1C
#define C2COFF_IRQ_ENABLE_STATUS_1	0x20
#define C2COFF_IRQ_ENABLE_SET_0	0x24
#define C2COFF_IRQ_ENABLE_SET_1	0x28
#define C2COFF_IRQ_ENABLE_CLEAR_0	0x2C
#define C2COFF_IRQ_ENABLE_CLEAR_1	0x30
#define C2COFF_IRQ_EOI	0x34
#define C2COFF_FCLK_FREQ	0x40
#define C2COFF_RX_MAX_FREQ	0x44
#define C2COFF_TX_MAX_FREQ	0x48
#define C2COFF_RX_MAX_FREQ_ACK	0x4C
#define C2COFF_WAKE_REQ	0x50
#define C2COFF_WAKE_ACK	0x54
#define C2COFF_STANDBY	0x60
#define C2COFF_STANDBY_IN	0x64
#define C2COFF_WAIT	0x68
#define C2COFF_GENI_CONTROL	0x70
#define C2COFF_GENI_MASK	0x74
#define C2COFF_GENO_STATUS	0x80
#define C2COFF_GENO_INTERRUPT	0x84
#define C2COFF_GENO_LEVEL	0x88

/* Some usefull PRCM registers related to C2C */
#define PRCMOFF_A9_C2C_GENO_MASK_SET	0x2078
#define PRCMOFF_A9_C2C_GENO_MASK_CLR	0x207C
#define PRCMOFF_A9_C2C_GENO_MASK_VAL	0x2080
#define PRCMOFF_C2C_SSCM_GENI_SET	0x2054
#define PRCMOFF_C2C_SSCM_GENI_CLR	0x2058
#define PRCMOFF_C2C_SSCM_GENI_VAL	0x205C
#define PRCMOFF_C2C_SSCM_GENO	0x2060

/* c2c_genio structure hold what IRQ handler require to work
 *
 * @mask: allocated genio bit mask, 0 until allocated
 * @pending: non-0 if pending operation on bit until C2C is 'on'
 * @poll_timeout: default=0, set to non-0 once polling timeout detected
 * @prcm: base addr of PRCM unit registers
 * @event_cb: callback routine addr
 * @event_data: callback data pointer
 * @trigged: GENO IRQ trigged (no used for fast-hanshake)
 * @level: last read GENO level (no used for fast-hanshake)
 * @irq: registered irq handler, -1 is no irq requested
 * @bit: storage of bit pos (ease debug)
 * @hs_cnt: handshake counter
 * @poll_timeout_nfy: errhandler() already called for this GENIO timeout
 * @rsv: padding to 64byte long
 *
 * Description
 * Each cell is 64byte long: the size of a 2 ARM cache lines.
 * Base address of the table must be 32byte align (page aligned)
 * This structure aims at accelerating IRQ handling and preventing
 * (as possible) SCU update of L1, (fast handshake support).
 * The structure also embedded fields dedicated to GENO event
 * that participate in slow GPIO event notification.
*/
struct c2c_genio {
	u32 mask;
	int pending;
	int poll_timeout;
	void __iomem *prcm;
	void (*event_cb)(void *data);
	void *event_data;
	int trigged;
	int level;
	int irq;
	int bit;
	uint hs_cnt;
	int poll_timeout_nfy;
	u32 rsv[4];
};

/* struct ap9540_c2c - AP9540 linux C2C-GENIO device data
 *
 * @genio: 32-cell array of struct c2c_genio GENI/GENO mngt.
 * @reset_flag: set once genio_reset is called.
 * @setter_mask: fast-setter: bit suporting fast handshake
 * @getter_mask: fast-getter: bit suporting fast handshake
 * @irq1_mask: slow GENO IRQs through C2C IP irq1
 * @pwr_is_on: 0 if C2C power state is unknown, 1 is known ON
 * @pwr_last_req: last power state requested from API genio_power_req()
 * @powerup_timer: C2C powerup timeout timer
 * @powerup_timeout_nfy: powerup timeout already nofified.
 * @powerup_timeout: timeout flag: set if power-up timeout detected
 * @powerup_timeout_ms: timeout value, in millisecond
 * @powerup_timeout_armed: prevent multiple arming
 * @init_flags: bit flags for C2C init steps, protected by lock.
 * @ipc_ready_cb: CAIF dedicated: callback for IPC_READY event
 * @caif_ready_cb: CAIF dedicated: callback for CAIF_READY events
 * @errhandler: generic majure failure callback
 * @prcm_base: base addr of PRCM_C2C_xxx registers
 * @c2c_base: base addr of c2c HW registers
 * @wumod_gpio: GPIO WU_MOD
 * @irq0: handler for linux IRQ "C2C IP irq0"
 * @irq1: handler for linux IRQ "C2C IP irq1"
 * @workq: private workqueue
 * @ipc_ready_work: queued work for IPC_READY handling
 * @caif_ready_work: queued work for CAIF_READY handling
 * @protection_work: queued work for C2C wake-up timeout protection
 * @waitq: wait queue (various purpose)
 * @protection_event: to signal event info to protection work
 * @lock: mutex lock, protects structure access
 * @dev: ptr to struct device
 * @pdev: ptr to struct platform_device
 * @dbgdir: ptr to debugfs dentry
 * @no_power_down: do not allow C2C to power down.
 *
 * Description
 * ap9540_c2c structure is used as plaftform driver private data.
 * Its address is aligned on 32byte so that c2c_genio cells are cache aligned.
 * Address of ap9540_c2c structure or target bit c2c_genio cell base address
 * is passed as private data handler though registers callbacks.
 *
 * Most fields of ap9540_c2c and c2c_genio strutures are 32-bit boolean
 * fields, that prevents from locking requirements. When set, 1 is writen,
 * else 0 is writen. This prevents locking check during fast processings:
 * fast handshake on GENO bit detection, fasthandshake in genio_set_bit()
 * API, genio_power_req() API. These boolean 32bit cells are:
 * - ap9540_c2c.reset_flag
 * - ap9540_c2c.power_is_on
 * - ap9540_c2c.pwr_last_req
 * - ap9540_c2c.powerup_timeout
 * - ap9540_c2c.protection_event
 * - c2c_genio.pending
 * - c2c_genio.poll_timeout
 *
 * ap9540_c2c field 'lock' is used as a single mutex lock to prevent
 * concurrent accesses to following data:
 * - some C2C IP HW registers (GENO_INTERRUPT, GENO_LEVEL)
 * - ap9540_c2c.init_flags
 * - ap9540_c2c.irq1_mask
 */

struct ap9540_c2c {
	struct c2c_genio genio[32];
	int reset_flag;
	int pwr_is_on;
	int pwr_last_req;
	u32 setter_mask;
	u32 getter_mask;
	u32 irq1_mask;
	int protection_event;
	struct timer_list powerup_timer;
	int powerup_timeout;
	int powerup_timeout_nfy;
	int powerup_timeout_ms;
	int powerup_timeout_armed;
	uint init_flags;
	void (*ipc_ready_cb)(void);
	void (*caif_ready_cb)(bool);
	void (*errhandler)(int);
	void __iomem *prcm_base;
	void __iomem *c2c_base;
	int wumod_gpio;
	int irq0;
	int irq1;
	struct work_struct ipc_ready_work;
	struct work_struct caif_ready_work;
	struct work_struct protection_work;
	wait_queue_head_t waitq;
	struct mutex lock;
	struct device *dev;
	struct platform_device *pdev;
	struct dentry *dbgdir;
	int no_power_down;
};

/* C2C inits states flags: bit flag */
#define GENIO_INIT_RESET			(1<<0)
#define GENIO_INIT_IPC_READY_REQ	(1<<1)
#define GENIO_INIT_IPC_READY		(1<<2)
#define GENIO_INIT_IPC_READY_ACK	(1<<3)
#define GENIO_INIT_BIT_ALLOC		(1<<4)
#define GENIO_INIT_CAIF_READY		(1<<5)
#define GENIO_INIT_CAIF_READY_REQ	(1<<6)

#ifdef CONFIG_C2C_DEEP_DEBUG
int ap9540_c2c_deep_debug_exit(void);
int ap9540_c2c_deep_debug_init(struct ap9540_c2c *c2c_dev);
#else
int ap9540_c2c_deep_debug_exit(void) { return 0; }
int ap9540_c2c_deep_debug_init(struct ap9540_c2c *c2c_dev) {return 0; }
#endif

#endif
