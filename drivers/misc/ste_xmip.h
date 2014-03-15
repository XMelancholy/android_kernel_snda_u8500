/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Duan Jingbo <jingbo.duan@stericsson.com>
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef _STE_XMIP_H_
#define _STE_XMIP_H_

#define XMIP_IPCHW_LINE_NB 32

/* Some usefull PRCM registers related to XMIP */
#define PRCMOFF_A9_XMIP_MASK_SET	0x2078
#define PRCMOFF_A9_XMIP_MASK_CLR	0x207C
#define PRCMOFF_A9_XMIP_MASK_VAL	0x2080
#define PRCMOFF_XMIP_APPMOD_SET		0x2054
#define PRCMOFF_XMIP_APPMOD_CLR		0x2058
#define PRCMOFF_XMIP_APPMOD_VAL		0x205C
#define PRCMOFF_XMIP_MODAPP_GENO	0x2060

#define PRCMOFF_LINE_VALUE_8		0x24D8
#define PRCMOFF_LINE_VALUE		0x170

/*
 * Request to be notified on Resout0N, Resout2N and ModApp IPC_READY.
 * ModApp[31:30] are expected to carry no wake-up events.
 * All other ModApp can be used for fast-handshake protocol so PRCMU
 * fmw can detect rising edges to wakeup AP if sleeping.
 * READY_FOR_IPC is rising/falling edge detection from PRCMU fmw
 * MODDADD 28 is reserved: not a "fast-handshake" MODAPP.
 * Theses values are valid only for READY_FOR_IPC_BIT = 29.
 */
#define PRCM_DEFAULT_EDGE_SENS_L8	0x55555555
#define PRCM_DEFAULT_EDGE_SENS_H8	0xFB555555
#define PRCM_DEFAULT_MODAPP_NOTIF	0xF0000000

#define APPMOD_CLEAR_ALL		0xFFFFFFFF
#define MODAPP_MASK_ALL			0xFFFFFFFF

#define PRCM_LINE_VALUE_RESOUT2_MASK	(1 << 21)
/*
 * xmip_ipchw structure hold what IRQ handler require to work
 *
 * @mask: allocated genio bit mask, 0 until allocated
 * @poll_timeout: default=0, set to non-0 once polling timeout detected
 * @poll_timeout_nfy: prevent multiple timeout notifications
 * @prcm: base addr of PRCM unit registers
 * @event_cb: callback routine addr
 * @event_data: callback data pointer
 * @trigged: GENO IRQ trigged (no used for fast-hanshake)
 * @level: last read MODAPP level (no used for fast-hanshake)
 * @irq: registered irq handler, -1 is no irq requested
 * @bit: storage of bit pos (ease debug)
 * @hs_cnt: handshake counter
 * @rsv: padding to 64byte long
 *
 * Description
 * Each cell is 64byte long: the size of a single ARM cache line.
 * Base address of the table must be 64byte align (page aligned)
 * This structure aims at accelerating IRQ handling and preventing
 * (as possible) SCU update of L1, (fast handshake support).
 * The structure also embedded fields dedicated to XMIP MODAPP event
 * that participate in slow GPIO event notification.
 */
struct xmip_ipchw {
	u32 mask;
	int poll_timeout;
	int poll_timeout_nfy;
	void __iomem *prcm;
	void (*event_cb)(void *data);
	void *event_data;
	int trigged;
	int level;
	int irq;
	int bit;
	uint hs_cnt;
	u32 rsv[5];
};

/*
 * struct ste_xmip - db8540 linux XMIP-MODAPP/APPMOD device data
 *
 * @ipchw: 32-cell array of struct XMIP MODAPP/APPMOD mngt.
 * @reset_flag: set once xmip_reset is called.
 * @setter_mask: fast-setter: bit suporting fast handshake
 * @getter_mask: fast-getter: bit suporting fast handshake
 * @protection_event: to signal event info to protection work
 * @init_flags: bit flags for XMIP init steps, protected by lock.
 * @ipc_ready_state: not 0 if IPC_READY is High, 0 if IPC_READY is low
 * @ipc_ready_cb: CAIF dedicated: callback for IPC_READY event
 * @caif_ready_cb: CAIF dedicated: callback for CAIF_READY events
 * @errhandler: generic majure failure callback
 * @base: base addr of PRCM_XMIP_xxx registers
 * @ipc_ready_work: queued work for IPC_READY handling
 * @protection_work: queued work for C2C wake-up timeout protection
 * @waitq: wait queue (various purpose)
 * @lock: mutex lock, protects structure access
 * @dev: ptr to struct device
 * @pdev: ptr to struct platform_device
 * @dbgdir: ptr to debugfs dentry
 *
 * Description
 *
 * ste_xmip structure is used as plaftform driver private data.
 * Its address is aligned on 64byte so that xmip_ipchw cells are cache aligned.
 * Address of ste_xmip structure or target bit xmip_ipchw cell base address
 * is passed as private data handler though registers callbacks.
 *
 * Most fields of ste_xmip and xmip_ipchw strutures are 32-bit boolean
 * fields, that prevents from locking requirements. When set, 1 is writen,
 * else 0 is writen. This prevents locking check during fast processings:
 * fast handshake on MODAPP/APPMOD bit detection, fasthandshake
 * in xmip_set_bit() API. These boolean 32bit cells are:
 * - ste_xmip.reset_flag
 * - ste_xmip.protection_event
 * - xmip_ipchw.pending
 * - xmip_ipchw.poll_timeout
 *
 * ste_xmip field 'lock' is used as a single mutex lock to prevent
 * concurrent accesses to following data:
 * - ste_xmip.init_flags
 */

struct ste_xmip {
	struct xmip_ipchw ipchw[XMIP_IPCHW_LINE_NB];
	int reset_flag;
	u32 setter_mask;
	u32 getter_mask;
	int protection_event;
	uint init_flags;
	int ipc_ready_state;
	void (*ipc_ready_cb)(void);
	void (*caif_ready_cb)(bool);
	void (*errhandler)(int);
	void __iomem *base;
	struct work_struct ipc_ready_work;
	struct work_struct protection_work;
	wait_queue_head_t waitq;
	struct mutex lock;
	struct device *dev;
	struct platform_device *pdev;
	struct dentry *dbgdir;
};

/* Ensure struct ste_xmip is aligned on 64 bytes boundary */
#define XMIP_STRUCT_ALIGN	0x3f

/* XMIP inits states flags: bit flag */
#define XMIP_INIT_RESET		(1<<0)
#define XMIP_INIT_BIT_ALLOC	(1<<4)

#endif
