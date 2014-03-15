/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/tee.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "tee_mem.h"

long tee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOSYS;

	switch (cmd) {
	case TEE_RESOLVE_IOC:
	{
		struct tee_sharedmemory req;
		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct tee_sharedmemory))) {
			ret = -EFAULT;
			break;
		} else
			req.hwmem_gname = tee_mem_uva2gid(req.buffer);

		if (copy_to_user((void __user *)arg, &req,
				 sizeof(struct tee_sharedmemory)))
			ret = -EFAULT;
		else
			ret = 0;
	}
	break;
	}

	return ret;
}
