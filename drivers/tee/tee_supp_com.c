/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * Author: Marten Olsson <marten.xm.olsson@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/tee.h>
#include <linux/slab.h>
#include <linux/hwmem.h>
#include <linux/semaphore.h>
#include "tee_rpc.h"
#include "tee_mem.h"


#define TEEDS_NAME "teesupp"
#define TEEDS_PFX "TEE-supp: "

#define TEE_RPC_BUFFER 0x00000001
#define TEE_RPC_VALUE  0x00000002

static DEFINE_SEMAPHORE(datatouser);
static DEFINE_SEMAPHORE(datafromuser);
static DEFINE_MUTEX(outsync);
static DEFINE_MUTEX(insync);

static struct tee_rpc_bf rpc_buffers[TEE_RPC_NBR_BUFF];
static u8 *ku_inbuffer;
static u8 *ku_outbuffer;

/* device functions */
static int tee_supp_open(struct inode *inode, struct file *file);
static int tee_supp_release(struct inode *inode, struct file *file);
static int tee_supp_read(struct file *filp, char __user *buffer,
		    size_t length, loff_t *offset);
static int tee_supp_write(struct file *filp, const char __user *buffer,
		     size_t length, loff_t *offset);

static void send_request(uint32_t cmd, const void *payload,
			 const uint32_t payloadlength)
{
	struct tee_supp_comm *clientdata = 0;

	mutex_lock(&outsync);

	ku_outbuffer = kmalloc(sizeof(struct tee_supp_comm)+payloadlength,
			       GFP_KERNEL);

	if (ku_outbuffer == NULL) {
		pr_err(TEEDS_PFX "[%s] allocation failed", __func__);
		mutex_unlock(&outsync);
		return;
	}

	clientdata = (struct tee_supp_comm *)ku_outbuffer;

	clientdata->cmd = cmd;
	clientdata->payloadlength = payloadlength;

	memcpy(&(clientdata->payload), payload, payloadlength);

	mutex_unlock(&outsync);

	up(&datatouser);
}

static void get_response(uint32_t *cmd, void **payload,
			 uint32_t *payloadlength)
{
	struct tee_supp_comm *clientdata = 0;

	*payload = NULL;
	*payloadlength = 0;
	*cmd = 0;

	down(&datafromuser);

	mutex_lock(&insync);

	if (ku_inbuffer != NULL) {
		clientdata = (struct tee_supp_comm *)ku_inbuffer;

		*cmd = clientdata->cmd;

		if (clientdata->payload != 0) {

			*payload = kmalloc(clientdata->payloadlength,
					   GFP_KERNEL);

			if (*payload == NULL) {
				pr_err(TEEDS_PFX "[%s] allocation failed",
				       __func__);
			} else {
				*payloadlength = clientdata->payloadlength;
				memcpy(*payload, &(clientdata->payload),
				       clientdata->payloadlength);
			}
		} else {
			pr_err(TEEDS_PFX "No response payload!\n");
		}

		kfree(ku_inbuffer);
		ku_inbuffer = NULL;
	} else {
		pr_err(TEEDS_PFX "No data!\n");
	}

	mutex_unlock(&insync);
}

static void tee_rpc_gid2pa(struct tee_rpc_invoke *inv)
{
	uint32_t i;
	struct tee_rpc_cmd *itr = (struct tee_rpc_cmd *)(inv + 1);
	struct tee_mem *mem;
	struct tee_rpc_mem *rmem;

	for (i = 0; i < inv->nbr_bf; i++) {
		if (itr->type == TEE_RPC_BUFFER && itr->buffer) {
			mem = tee_mem_register((s32)itr->buffer);
			rmem = (struct tee_rpc_mem *)tee_mem_get_paddr(mem);
			itr->buffer = (void *)rmem;
			tee_mem_free(mem);
		}
		itr++;
	}
}

static enum teec_rpc_result rpc_callback(
	uint32_t id, uint32_t p0, uint32_t p1, uint32_t p2)
{
	enum teec_rpc_result res = TEEC_RPC_FAIL;

	switch (id) {
	case TEE_RPC_ICMD_ALLOCATE:
	{
		struct tee_rpc_alloc *alloc;
		struct tee_mem *mem;
		struct tee_rpc_mem *rmem;
		phys_addr_t pa;

		if (p0 >= TEE_RPC_NBR_BUFF)
			break;

		alloc = (struct tee_rpc_alloc *)rpc_buffers[p0].data;

		mem = tee_mem_alloc(alloc->size + sizeof(struct tee_rpc_mem));
		if (!mem)
			break;

		rmem = (struct tee_rpc_mem *)tee_mem_get_vaddr(mem);
		if (!rmem) {
			tee_mem_free(mem);
			break;
		}

		rmem->tee_mem = mem;
		rmem->gid = tee_mem_get_gid(mem);
		rmem->size = alloc->size;

		pa = tee_mem_get_paddr(mem);

		alloc->data = (void *)(pa);
		res = TEEC_RPC_OK;

		break;
	}
	case TEE_RPC_ICMD_FREE:
	{
		tee_mem_free((struct tee_mem *)p0);
		res = TEEC_RPC_OK;
		break;
	}
	case TEE_RPC_ICMD_INVOKE:
	{
		struct tee_rpc_invoke *inv;
		void *out;
		uint32_t cmd;
		uint32_t in_size;
		uint32_t out_size;

		if (p1 >= TEE_RPC_NBR_BUFF)
			break;

		inv = (struct tee_rpc_invoke *)rpc_buffers[p1].data;
		in_size = inv->nbr_bf * sizeof(struct tee_rpc_cmd) +
			sizeof(struct tee_rpc_invoke);

		send_request(p0, (void *)inv, in_size);
		get_response(&cmd, &out, &out_size);

		if (in_size != out_size)
			break;
		memcpy(inv, out, out_size);
		kfree(out);

		/* translate from gid to pa */
		tee_rpc_gid2pa(inv);
		res = TEEC_RPC_OK;

		break;
	}
	default:
		/* not supported */
		break;
	}

	return res;
}

static int tee_supp_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int tee_supp_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int tee_supp_read(struct file *filp, char __user *buffer,
		    size_t length, loff_t *offset)
{
	int ret = -ERESTARTSYS;
	struct tee_supp_comm *clientdata = 0;

	if (down_interruptible(&datatouser))
		return -ERESTARTSYS;

	mutex_lock(&outsync);
	clientdata = (struct tee_supp_comm *)ku_outbuffer;

	if (length < (clientdata->payloadlength +
		      sizeof(struct tee_supp_comm))) {
		ret = -EINVAL;
		goto exit;
	}

	if (copy_to_user(buffer, ku_outbuffer,
			 (clientdata->payloadlength +
			  sizeof(struct tee_supp_comm)))) {
		pr_err(TEEDS_PFX "[%s] error, copy_to_user failed!\n",
		       __func__);
		ret = -EINVAL;
	} else
		ret = (clientdata->payloadlength +
		       sizeof(struct tee_supp_comm));

exit:
	kfree(ku_outbuffer);
	ku_outbuffer = NULL;

	mutex_unlock(&outsync);
	return ret;
}

static int tee_supp_write(struct file *filp, const char __user *buffer,
			  size_t length, loff_t *offset)
{
	int ret = 0;

	mutex_lock(&insync);
	if (length > 0) {
		ku_inbuffer = kmalloc(length, GFP_KERNEL);

		if (ku_inbuffer == NULL) {
			pr_err(TEEDS_PFX "[%s] allocation failed", __func__);
			ret = -ENOMEM;
			goto exit;
		}
		if (copy_from_user(ku_inbuffer, buffer, length)) {
			pr_err(TEEDS_PFX "[%s] error, tee_session "
			       "copy_from_user failed\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
	}
exit:
	mutex_unlock(&insync);

	if (ret == 0) {
		up(&datafromuser);
		ret = length;
	}

	return ret;
}

static const struct file_operations tee_supp_fops = {
	.owner = THIS_MODULE,
	.read = tee_supp_read,
	.write = tee_supp_write,
	.open = tee_supp_open,
	.release = tee_supp_release,
};

static struct miscdevice tee_supp_dev = {
	MISC_DYNAMIC_MINOR,
	TEEDS_NAME,
	&tee_supp_fops
};

int tee_supp_init(void)
{
	int err = misc_register(&tee_supp_dev);

	if (err)
		pr_err(TEEDS_PFX "[%s] error %d adding character device "
		       "TEE\n", __func__, err);

	/* register callback */
	tee_register_rpc(rpc_callback, rpc_buffers, TEE_RPC_NBR_BUFF);

	down(&datatouser);
	down(&datafromuser);
	return err;
}

void tee_supp_exit(void)
{
	misc_deregister(&tee_supp_dev);
}

