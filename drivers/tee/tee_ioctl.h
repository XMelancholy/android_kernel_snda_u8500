/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/types.h>
#include <linux/fs.h>

#ifndef TEE_IOCTL_H
#define TEE_IOCTL_H

/**
 * tee_ioctl - Handle TEE ioctl
 * @filp: Pointer to the file
 * @cmd: The TEE ioctl command
 * @arg: The ioctl arguments
 *
 * Returns zero in case of success and a negative error code in case of failure
 */
long tee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif
