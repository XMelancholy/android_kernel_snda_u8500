/*
 * Copyright (C) ST-Ericsson AB 2011
 * Author: Sjur Brendeland / sjur.brandeland@stericsson.com
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __INC_GENIO_H
#define __INC_GENIO_H
#include <linux/types.h>

/**
 * DOC: C2C GENI/GENO interface.
 *
 * This API defines the API for the C2C driver and the GENI/GENO registers.
 */

/**
 * enum GENIO_BITS - Definition of special GENIO BITS.
 * @READY_FOR_IPC_BIT:  Remote side is ready for IPC.
 * This GENI/GENO bit is triggered when ring-buffer protocol
 * is enabled from the remote end (modem)
 *
 * @READY_FOR_CAIF_BIT: Remote side is ready for CAIF.
 * This GENI/GENO bit is triggered when CAIF protocol
 * is enabled from the remote end (modem)
 */
enum GENIO_BITS {
	READY_FOR_CAIF_BIT = 28,
	READY_FOR_IPC_BIT = 29
};

/**
 * enum genio_hwerror - IDs used when HW error is detected.
 * @GENIO_IPC_HW_CRASH: HW used for IPC has unexpectedly crashed.
 * @GENIO_IPC_HW_RESET: HW used for IPC has unexpectedly reset.
 */
enum genio_hwerror {
	GENIO_IPC_HW_CRASH,
	GENIO_IPC_HW_RESET,
};

/**
 * genio_subscribe - Subscribe for notifications on bit-change of GENI/O bits.
 *
 * @bit:	The GENI/O bit where we want to be called back when it changes.
 *
 * @bit_set_cb:	Callback function to be called when the requested GENI/O bit
 *		is set by the external device (modem).
 *
 * @data: Client data to be provided in the callback function.
 *
 * Install a callback function for a GENI/O bit. Returns negative upon error.
 *
 * The genio driver is expected to handle the 4-state handshake for geni/geno
 * update, for the bit this function subscribe to. This function may block,
 * and cannot be called from IRQ context.
 *
 * Returns zero on success, and negative upon error.
 *
 * Precondition: This function is called after genio_set_shm_addr() and
 * genio_bit_alloc(). @bit must be defined as as getter in genio_bit_alloc().
 *
 * Callback context:
 *		The "bit_set_cb" callback must be called from the
 *		IRQ context. The callback function is not allowed to block
 *		or spend much CPU time in the callback, It must defer
 *		work to soft-IRQ or work queues.
 */
int genio_subscribe(int bit, void (*bit_set_cb)(void *data), void *data);

/**
 * genio_unsubscribe - Unsubscribe for callback  GENI/O bit.
 * @bit:       The GENI/O bit to we want release.
 *
 * This function may block. It returns zero on success, and negative upon error.
 *
 * Precondition: @bit must be defined as as getter in genio_bit_alloc().
 */
int genio_unsubscribe(int bit);

/**
 * genio_bit_alloc - Allocate the usage of GENI/O bits.
 *
 * @setter_mask:	Bit-mask defining the bits that can be set by
 *			genio_set_bit()
 * @getter_mask:	Bit-mask defining the bits that can be subscribed by
 *			genio_subscribe().
 *
 * The @getter_mask defines the bit for RX direction, i.e. bits that can
 * be subscribed by the function genio_subscribe().
 * The @setter_mask defines the bit for TX direction, i.e. bits that can
 * be set by the function genio_set_bit().
 * This function may block.
 *
 * Returns zero on success, and negative upon error.
 *
 * Precondition:
 * This function cannot be called before ipc_ready_cb() has been called,
 * and must be called prior to any call on genio_subscribe() or genio_set_bit().
 *
 */
int genio_bit_alloc(u32 setter_mask, u32 getter_mask);

/**
 * genio_set_shm_addr - Inform remote device about the shared memory address.
 * @addr:		The shared memory address.
 * @ipc_ready_cb:	The callback function indicating IPC is ready.
 *
 * Description:
 * Implements the setting of shared memory address involving the
 * READY_FOR_IPC GENIO bits, and READY_FOR_IPC handshaking.
 * When handshaking is done ipc_ready_cb is called.
 * The usage of this bit is during start/restart.
 * This function may block.
 *
 * When the ipc_ready_cb() with @ready=%true, Stream channels can be opened.
 *
 * Sequence:
 * (1) Write the address to the GENO register,
 *
 * (2) wait for interrupt on IPC_READY (set to one in GENI register)
 *
 * (3) write zero to the GENO register
 *
 * (4) wait for interrupt on IPC_READY (set to zero)
 *
 * (5) call the callback function for IPC_READY
 *
 * Returns zero on success, and negative upon error.
 *
 * Precondition:
 * This function must be called initially upon start, or after remote device
 * has been reset.
 */
int genio_set_shm_addr(u32 addr, void (*ipc_ready_cb) (void));

/**
 * genio_subscribe_caif_ready - Notify that CAIF channels can be
 * opened.
 *
 * @caif_ready_cb: is called with @ready = %true in the start up sequence,
 * when the remote side is ready for CAIF enumeration. Upon reset,
 * caif_ready_cb() will be called with @ready = %false.
 *
 * The %READY_FOR_CAIF_BIT is set to %one as long as the
 * modem is able to run CAIF traffic. Upon modem restart/crash it will
 * be set back to %zero.
 * This function may block.
 *
 * Returns zero on success, and negative upon error.
 */
int genio_subscribe_caif_ready(void (*caif_ready_cb) (bool ready));

/**
 * genio_register_errhandler - Register an error handler.
 *
 * @errhandler: error handler called from driver upon severe errors
 *		that requires reset of the remote device.
 */
void genio_register_errhandler(void (*errhandler)(int errno));

/**
 * genio_reset() - Reset the C2C driver
 *
 * Reset the C2C Driver due to remote device restart.
 * This shall reset state back to initial state, and should only
 * be used when remote device (modem) has reset.
 *
 * All settings, subscriptions and state information in the driver must
 * be reset. GENIO client must do all subscriptions again.
 * This function may block.
 *
 * Returns zero on success, and negative upon error.
 */
int genio_reset(void);

/**
 * genio_set_bit() -	Set a single GENI/O bit.
 *
 * @bit:	The GENI/O bit to set
 *
 * This function is used to signal over GENI/GENO the driver must
 * perform the 4-state protocol to signal the change to remote device.
 * This function is non-blocking, and can be called from Soft-IRQ context.
 *
 * Returns zero on success, and negative upon error.
 *
 * Precondition: @bit must be defined as as setter in genio_bit_alloc().
 */
int genio_set_bit(int bit);

/**
 * genio_power_req() -	Request power on on remote C2C block.
 *
 * @state:	1 - power-on request , 0 - power-off request
 *
 * This function will request power on of remote C2C block.
 * This function is non-blocking, and can be called from Soft-IRQ context.
 *
 * Returns zero on success, and negative upon error.
 */
int genio_power_req(int state);

/**
 * genio_drvname() - get driver name string.
 *
 * If no registered driver or registered driver does not provide
 * any drvname, genio_drvname insured null string ("\0") is returned.
 */
char * genio_drvname(void);

/**
 * A GENIO based Modem IPC driver need to register its API handlers
 * to the genio_wrap driver using this structure.
 * Each handler can be set to NULL if not supported by driver.
 *
 * @drvname: string to identify registered driver
 * @subscribe: driver handler for API genio_subscribe
 * @unsubscribe: driver handler for API genio_unsubscribe
 * @bit_alloc: driver handler for API genio_bit_alloc
 * @set_shm_addr: driver handler for API genio_set_shm_addr
 * @subscribe_caif_ready: driver handler for API genio_subscribe_caif_ready
 * @register_errhandler: driver handler for API genio_register_errhandler
 * @reset: driver handler for API genio_reset
 * @set_bit: driver handler for API genio_set_bit
 * @power_req: driver handler for API genio_power_req
 */
struct genio_wrap {
	char *drvname;
	int (*subscribe)(int bit, void (*cb)(void *data), void *data);
	int (*unsubscribe)(int bit);
	int (*bit_alloc)(u32 setter_mask, u32 getter_mask);
	int (*set_shm_addr)(u32 addr, void (*ipc_ready_cb) (void));
	int (*subscribe_caif_ready)(void (*caif_ready_cb) (bool ready));
	void (*register_errhandler)(void (*errhandler)(int errno));
	int (*reset)(void);
	int (*set_bit)(int bit);
	int (*power_req)(int state);
};

/**
 * genio_register_driver() - Register callbacks for a GENIO based IPC driver.
 */
int genio_register_driver(struct genio_wrap *in);

/**
 * genio_unregister_driver() - Unregister GENIO based IPC driver from wrapper.
 */
void genio_unregister_driver(struct genio_wrap *in);

#endif /*INC_GENIO_H*/
