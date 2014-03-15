/*
 * hva_common.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author: <thomas.costis@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef __HVA_COMMON_H__
#define __HVA_COMMON_H__

#define HVA_DEVNAME "hva"

/* Control user request */
enum hva_flag {
	HVA_FLAG_REPORT_WHEN_DONE   =  0x1,
	HVA_FLAG_REPORT_PERFORMANCE =  0x2
};

enum hif_cmd_type {
	H264_BASELINE_DEC       = 0x0,
	H264_HIGHPROFILE_DEC    = 0x01,
	H264_ENC                = 0x02,
	JPEG_ENC                = 0x03,
	/* SW synchro task (reserved in HW) */
	SYNC_HVC                = 0x4,
	SYNC_HEC_HVC            = 0x5,
	SYNC_HJE                = 0x6,
	/* RESERVED            = 0x7 */
	REMOVE_CLIENT           = 0x08,
	FREEZE_CLIENT           = 0x09,
	START_CLIENT            = 0x0A,
	FREEZE_ALL              = 0x0B,
	START_ALL               = 0x0C,
	REMOVE_ALL              = 0x0D
};

/* Status from HW */
enum hva_status_error {
	HVA_ERROR_NONE              = 0x0,
	HVA_IT_FIFO_STS_FULL        = 0x1,
	HVA_IT_HIF_LMI_ERROR        = 0x2,
	HVA_IT_HIF_EMI_ERROR        = 0x4,
	HVA_IT_HIF_HEC_MIF_ERROR    = 0x8,
	HVA_TASK_LIST_FULL          = 0x10,
	HVA_UNKNOWN_CMD             = 0x20,
	HVA_READ_ERR                = 0x40
};

/* Specific codec status */
enum hva_h264dec_status {
	TASK_OK                         = 0x0,
	PICTURE_LOSS                    = 0x01,
	HEC_IB_OVERFLOW_ERROR           = 0x02,
	HEC_BITSTREAM_UNDERFLOW_ERROR   = 0x04,
	HEC_READ_PLUG_ERROR             = 0x10,
	HEC_WRITE_PLUG_ERROR            = 0x20,
	/* Added for H264 decode synchro purpose (ARM side) */
	VOID_FRAME                      = 0x100,
	ERT_NONE                        = 0x101
};

enum hva_h264enc_status {
	H264ENC_ENCODE_OK                       = 0x0,
	H264ENC_ABORT_PIC_OVER_BITSREAM_SIZE    = 0x2,
	H264ENC_ABORT_MB_OVER_SLICE_SIZE        = 0x3,
	H264ENC_FRAME_SKIPPED                   = 0x4
};

enum hva_jpegenc_status {
	JPEGENC_ENCODE_OK                       = 0x0,
	JPEGENC_ABORT_PIC_OVER_BITSREAM_SIZE    = 0x1
};

/**
 * HVA register struct
 */
struct hva_reg {
	unsigned int offset; /* Offset / hva base addr */
	unsigned int val;    /* Register value */
};

/**
 * HVA user request
 */
struct hva_user_req {
	enum hif_cmd_type cmd;             /* Command sent to HVA */

	unsigned int      task_desc;       /* Pointer to task descriptor */
	unsigned int      client_handler;
	unsigned int      client_id;
	unsigned int      user_task_id;
	unsigned int      status;          /* Task status */
	unsigned int      info;            /* Task additional info */
	unsigned int      hw_start_ts, hw_stop_ts;

	/**
	 * @brief Callback function, will be called when request is finished.
	 * It will be called from another thread. May be NULL.
	 */
	void (*callback)(void *callback_data);
};

#define HVA_IOC_MAGIC  'h'
#define HVA_IOC        _IOW(HVA_IOC_MAGIC, 0, struct hva_user_req)
#define HVA_VERSION    _IOR(HVA_IOC_MAGIC, 1, unsigned int)
#define HVA_GET_REG    _IOWR(HVA_IOC_MAGIC, 2, struct hva_reg)

#endif /* __HVA_COMMON_H__ */


