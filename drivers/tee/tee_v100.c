/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/tee.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmem.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/pgtable.h>

#include "tee_ioctl.h"
#include "tee_supp_com.h"
#include "tee_mem.h"

/* tee tee to get same prefix as from the device. */
#define TEED_PFX "tee tee: "

#define TEED_STATE_OPEN_DEV 0
#define TEED_STATE_OPEN_SESSION 1

#define TEE_STE_OPEN_SESSION    0x11000008
#define TEE_STE_CLOSE_SESSION   0x11000009
#define TEE_STE_INVOKE          0x1100000a
#define TEE_STE_REGISTER_RPC    0x1100000b
#define TEE_STE_SET_SEC_DDR     0x1100000c

#define SEC_ROM_NO_FLAG_MASK 0x0000

static struct mutex sync;
static struct platform_device *pdev;

struct tee_open_session_arg {
	uint32_t res;
	uint32_t origin;
	uint32_t sess;
	struct ta_signed_header_t *ta;
	struct tee_uuid *uuid;
	uint32_t param_types;
	struct tee_value params[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	struct tee_identity client_id;
	uint32_t params_flags[TEEC_CONFIG_PAYLOAD_REF_COUNT];
};

struct tee_invoke_command_arg {
	uint32_t res;
	uint32_t origin;
	uint32_t sess;
	uint32_t cmd;
	uint32_t param_types;
	struct tee_value params[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	struct tee_identity client_id;
	uint32_t params_flags[TEEC_CONFIG_PAYLOAD_REF_COUNT];
};

static u32 call_sec_rom_bridge(u32 service_id, u32 cfg, ...)
{
	typedef u32 (*bridge_func)(u32, u32, va_list);
	bridge_func hw_sec_rom_pub_bridge;
	va_list ap;
	u32 ret;
	struct tee_platform_data *tee_pdata;

	tee_pdata = (struct tee_platform_data *)pdev->dev.platform_data;
	hw_sec_rom_pub_bridge = (bridge_func)(tee_pdata->bridge_func_addr);

	va_start(ap, cfg);
	ret = hw_sec_rom_pub_bridge(service_id, cfg, ap);
	va_end(ap);

	return ret;
}

static void tee_convert_op(
	struct tee_session *session,
	struct tee_value params[TEEC_CONFIG_PAYLOAD_REF_COUNT],
	uint32_t *out_types)
{
	uint32_t n;
	struct tee_operation *in = session->uc.op;
	uint32_t tmp = 0;

	if (in == NULL)
		return;

	*out_types = 0;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		tmp = TEEC_PARAM_TYPE_GET(in->types, n);
		switch (tmp) {
		case TEEC_MEMREF_TEMP_INPUT:
		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			*out_types |= (tmp << (n*4));
			params[n].b = (uint32_t)in->param[n].tmpref.size;
			if (session->login == TEEC_LOGIN_KERNEL)
				params[n].a = (uint32_t)virt_to_phys(
					(void *)in->param[n].tmpref.buffer);
			else
				params[n].a =
					(uint32_t)in->param[n].tmpref.buffer;
			break;

		case TEEC_VALUE_INPUT:
		case TEEC_VALUE_OUTPUT:
		case TEEC_VALUE_INOUT:
			*out_types |= (tmp << (n*4));
			params[n] = in->param[n].value;
			break;

		case TEEC_MEMREF_WHOLE:
		{
			uint32_t type = 0;

			if (in->shm[n].flags == TEEC_MEM_INPUT)
				type = TEEC_MEMREF_TEMP_INPUT;
			else if (in->shm[n].flags == TEEC_MEM_OUTPUT)
				type = TEEC_MEMREF_TEMP_OUTPUT;
			else if (in->shm[n].flags ==
				 (TEEC_MEM_INPUT | TEEC_MEM_OUTPUT))
				type = TEEC_MEMREF_TEMP_INOUT;

			*out_types |= (type << (n*4));
			params[n].b = (uint32_t)in->param[n].memref.size;
			if (session->login == TEEC_LOGIN_KERNEL)
				params[n].a = (uint32_t)virt_to_phys(
					(void *)
					((uint32_t)
					 in->param[n].memref.parent->buffer +
					 in->param[n].memref.offset));
			else
				params[n].a =
					(uint32_t)in->param[n].tmpref.buffer;
			break;
		}
		case TEEC_MEMREF_PARTIAL_INPUT:
		case TEEC_MEMREF_PARTIAL_OUTPUT:
		case TEEC_MEMREF_PARTIAL_INOUT:
			*out_types |= ((tmp & TEEC_MEMREF_TEMP_INOUT)
				       << (n*4));
			params[n].b = (uint32_t)in->param[n].memref.size;
			if (session->login == TEEC_LOGIN_KERNEL)
				params[n].a = (uint32_t)virt_to_phys(
					(void *)
					((uint32_t)
					 in->param[n].memref.parent->buffer +
					 in->param[n].memref.offset));
			else
				params[n].a =
					(uint32_t)in->param[n].tmpref.buffer;
			break;

		default:
			break;
		}
	}
}

static void update_outdata(
	struct tee_session *session,
	struct tee_value params[TEEC_CONFIG_PAYLOAD_REF_COUNT],
	uint32_t in_types)
{
	uint8_t n;
	uint32_t type;
	struct tee_operation *out = session->uc.op;

	for (n = 0; n < TEEC_CONFIG_PAYLOAD_REF_COUNT; n++) {
		type = (in_types >> (n*4)) & 0xf;

		switch (type) {
		case TEEC_VALUE_OUTPUT:
		case TEEC_VALUE_INOUT:
			out->param[n].value = params[n];
			break;

		case TEEC_MEMREF_TEMP_OUTPUT:
		case TEEC_MEMREF_TEMP_INOUT:
			out->param[n].tmpref.size = params[n].b;
			break;

		case TEEC_MEMREF_WHOLE:
		case TEEC_MEMREF_PARTIAL_OUTPUT:
		case TEEC_MEMREF_PARTIAL_INOUT:
			out->param[n].memref.size = params[n].b;
			break;

		default:
			break;
		}
	}
}

int tee_mem_init(uint32_t addr, uint32_t size)
{
	/* register ddr region */
	call_sec_rom_bridge(TEE_STE_SET_SEC_DDR, SEC_ROM_NO_FLAG_MASK,
			    addr, size);

	return 0;
}

int tee_register_rpc(void *fnk, void *bf, uint32_t nbr_bf)
{
	void *ph = (void *)virt_to_phys(bf);

	return call_sec_rom_bridge(TEE_STE_REGISTER_RPC, SEC_ROM_NO_FLAG_MASK,
				   fnk, ph, nbr_bf);
}

static int call_sec_world(struct tee_session *ts, int sec_cmd)
{
	switch (sec_cmd) {
	case TEED_OPEN_SESSION:
	{
		struct tee_open_session_arg arg;

		memset(&arg, 0, sizeof(arg));
		if (ts->uc.data != NULL)
			arg.ta = (void *)virt_to_phys((void *)ts->uc.data);
		if (ts->uc.uuid != NULL)
			arg.uuid = (void *)virt_to_phys((void *)ts->uc.uuid);
		arg.client_id.login = ts->login;

		if (ts->uc.op) {
			tee_convert_op(ts, arg.params, &arg.param_types);
			memcpy(arg.params_flags, ts->uc.op->param_flags,
			       sizeof(arg.params_flags));
		}


		call_sec_rom_bridge(TEE_STE_OPEN_SESSION, SEC_ROM_NO_FLAG_MASK,
				    virt_to_phys((void *)&arg));

		update_outdata(ts, arg.params, arg.param_types);

		/* store session id */
		ts->id = arg.sess;
		ts->uc.err = arg.res;
		ts->uc.origin = arg.origin;
		break;
	}
	case TEED_INVOKE:
	{
		struct tee_invoke_command_arg arg;

		memset(&arg, 0, sizeof(arg));

		arg.sess = ts->id;
		arg.cmd = ts->uc.cmd;

		if (ts->uc.op) {
			tee_convert_op(ts, arg.params, &arg.param_types);
			memcpy(arg.params_flags, ts->uc.op->param_flags,
			       sizeof(arg.params_flags));
		}

		call_sec_rom_bridge(TEE_STE_INVOKE, SEC_ROM_NO_FLAG_MASK,
				    virt_to_phys(&arg));

		update_outdata(ts, arg.params, arg.param_types);

		ts->uc.err = arg.res;
		ts->uc.origin = arg.origin;
		break;
	}
	case TEED_CLOSE_SESSION:
	{
		call_sec_rom_bridge(TEE_STE_CLOSE_SESSION,
				    SEC_ROM_NO_FLAG_MASK, ts->id);
		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}



static inline void set_emsg(struct tee_session *ts, u32 msg, int line)
{
	dev_err(&pdev->dev, "msg: 0x%08x at line: %d\n", msg, line);
	ts->uc.err = msg;
	ts->uc.origin = TEED_ORIGIN_DRIVER;
}

static void reset_session(struct tee_session *ts)
{
	int i;

	ts->state = TEED_STATE_OPEN_DEV;
	ts->uc.err = TEED_SUCCESS;
	ts->uc.origin = TEED_ORIGIN_DRIVER;
	ts->id = 0;
	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; i++)
		ts->vaddr[i] = NULL;
	ts->uc.data = NULL;
	ts->uc.uuid = NULL;
	ts->uc.cmd = 0;
	ts->uc.driver_cmd = TEED_OPEN_SESSION;
	ts->uc.data_size = 0;
	ts->uc.op = NULL;
}

static int copy_ta(struct tee_session *ts,
		   struct tee_cmd *ku_buffer)
{
	ts->uc.data = kmalloc(ku_buffer->data_size, GFP_KERNEL);
	if (ts->uc.data == NULL) {
		dev_err(&pdev->dev, "[%s] error, out of memory (ta)\n",
			__func__);
		set_emsg(ts, TEED_ERROR_OUT_OF_MEMORY, __LINE__);
		return -ENOMEM;
	}

	ts->uc.data_size = ku_buffer->data_size;

	if (copy_from_user(ts->uc.data, ku_buffer->data, ku_buffer->data_size))
		return -EINVAL;

	return 0;
}

static int copy_uuid(struct tee_session *ts,
		     struct tee_cmd *ku_buffer)
{
	ts->uc.uuid = kmalloc(sizeof(struct tee_uuid), GFP_KERNEL);

	if (ts->uc.uuid == NULL) {
		dev_err(&pdev->dev, "[%s] error, out of memory (uuid)\n",
			__func__);
		set_emsg(ts, TEED_ERROR_OUT_OF_MEMORY, __LINE__);
		return -ENOMEM;
	}

	if (copy_from_user(ts->uc.uuid, ku_buffer->uuid,
			   sizeof(struct tee_uuid)))
		return -EINVAL;

	return 0;
}

static inline void free_operation(struct tee_session *ts,
				  struct tee_mem **mem)
{
	int i;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		if (mem[i]) {
			tee_mem_free(mem[i]);
			mem[i] = NULL;
			ts->uc.op->param[i].tmpref.buffer = NULL;
		}

		if (ts->vaddr[i])
			ts->vaddr[i] = NULL;
	}

	kfree(ts->uc.op);
	ts->uc.op = NULL;
}

static int tee_update_size(size_t size,
			   struct tee_tmpref __user *out)
{
	unsigned long bytes_left = put_user(size, &out->size);

	if (bytes_left != 0) {
		dev_err(&pdev->dev,
			"[%s] err cp (%d) to user space (%lu left).\n",
			__func__, size, bytes_left);
		return -EINVAL;
	}

	return 0;
}

static int tee_update_buffer(struct tee_tmpref *in,
			     struct tee_tmpref __user *out)
{
	unsigned long bytes_left = copy_to_user(
		out->buffer, in->buffer, in->size);

	if (bytes_left != 0) {
		dev_err(&pdev->dev,
			"[%s] err cp (%x to %x) to user space (%lu left of %d).\n",
			__func__, (u32)in->buffer,
			(u32)out->buffer, bytes_left, in->size);
		return -EINVAL;
	}

	/* update size */
	bytes_left = put_user(in->size, &out->size);

	if (bytes_left != 0) {
		dev_err(&pdev->dev,
			"[%s] err cp (%d) to user space (%lu left).\n",
			__func__, in->size,
			bytes_left);
		return -EINVAL;
	}

	return 0;
}

static int tee_update_value(struct tee_value *in,
			    struct tee_value __user *out)
{
	/* update value a */
	unsigned long bytes_left = put_user(in->a, &out->a);

	if (bytes_left != 0) {
		dev_err(&pdev->dev,
			"[%s] err cp a (%d) to user space (%lu left).\n",
			__func__, in->a, bytes_left);
		return -EINVAL;
	}

	/* update value b */
	bytes_left = put_user(in->b, &out->b);

	if (bytes_left != 0) {
		dev_err(&pdev->dev,
			"[%s] err cp b(%d) to user space (%lu left).\n",
			__func__, in->b, bytes_left);
		return -EINVAL;
	}

	return 0;
}

static int copy_memref_to_user(struct tee_session *ts,
			       struct tee_operation *kbuf_op,
			       struct tee_operation __user *ubuf_op,
			       int memref)
{
	int ret = 0;

	switch (TEEC_PARAM_TYPE_GET(ts->uc.op->types, memref)) {
	case TEEC_NONE:
	case TEEC_VALUE_INPUT:
	case TEEC_MEMREF_TEMP_INPUT:
	case TEEC_MEMREF_PARTIAL_INPUT:
		/* nothing to copy */
		ret = 0;
		break;

	case TEEC_VALUE_OUTPUT:
	case TEEC_VALUE_INOUT:

		ret = tee_update_value(
			&ts->uc.op->param[memref].value,
			&ubuf_op->param[memref].value);
		break;

	case TEEC_MEMREF_TEMP_OUTPUT:
	case TEEC_MEMREF_TEMP_INOUT:
	{
		struct tee_tmpref in;

		in.buffer = ts->vaddr[memref];
		in.size = ts->uc.op->param[memref].tmpref.size;

		ret = tee_update_buffer(
			&in, &ubuf_op->param[memref].tmpref);
		break;
	}
	case TEEC_MEMREF_WHOLE:
	{
		struct tee_sharedmemory *shm = &ts->uc.op->shm[memref];

		if (shm->hwmem_gname <= 0) {
			struct tee_tmpref in;

			in.buffer = ts->vaddr[memref];
			in.size = ts->uc.op->param[memref].memref.size;

			ret = tee_update_buffer(
				&in, &ubuf_op->param[memref].tmpref);
		} else {
			ret = tee_update_size(
				shm->size,
				&ubuf_op->param[memref].tmpref);
		}
		break;
	}
	case TEEC_MEMREF_PARTIAL_OUTPUT:
	case TEEC_MEMREF_PARTIAL_INOUT:
	{
		u32 offset = kbuf_op->param[memref].memref.offset;
		size_t size = ts->uc.op->param[memref].memref.size;
		struct tee_sharedmemory *shm = &ts->uc.op->shm[memref];

		/* ensure we do not exceed the shared buffer length */
		if ((offset + size) > shm->size) {
			ret = -EINVAL;
			break;
		}

		if (shm->hwmem_gname <= 0) {
			struct tee_tmpref in;

			in.buffer = ts->vaddr[memref] + offset;
			in.size = size;

			ret = tee_update_buffer(
				&in, &ubuf_op->param[memref].tmpref);
		} else {
			ret = tee_update_size(
				size, &ubuf_op->param[memref].tmpref);
		}
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tee_cpy_memref(struct tee_session *ts,
			  struct tee_mem **mem,
			  void *buffer,
			  u32 size,
			  int memref)
{
	int ret = 0;

	ts->vaddr[memref] = 0;

	/* Size 0 is OK to use. */
	if (size == 0) {
		ts->uc.op->param[memref].tmpref.buffer = NULL;
		ts->vaddr[memref] = NULL;
		ret = 0;
		goto exit;
	}

	/*
	 * Allocate consecutive memory
	 */
	*mem = tee_mem_alloc(size);
	if (!*mem) {
		dev_err(&pdev->dev,
			"[%s] couldn't alloc tee memory (memref: %d)\n",
			__func__, memref);
		goto exit;
	}

	/*
	 * Since phys_to_virt is not working for tee memory we are storing
	 * the virtual addresses in separate array in tee_session and we keep
	 * the address of the physical pointers in the memref buffer.
	 */
	ts->uc.op->param[memref].tmpref.buffer =
		(void *)tee_mem_get_paddr(*mem);
	ts->vaddr[memref] = tee_mem_get_vaddr(*mem);

	/* Buffer unmapped/freed in invoke_command if this function fails. */
	if (!ts->uc.op->param[memref].tmpref.buffer || !ts->vaddr[memref]) {
		dev_err(&pdev->dev, "[%s] out of memory (memref: %d)\n",
			__func__, memref);
		ret = -ENOMEM;
		goto exit;
	}

	if (copy_from_user(ts->vaddr[memref], buffer, size)) {
		ret = -EINVAL;
		goto exit;
	}

exit:
	return ret;
}

static int tee_resolve_shm(struct tee_sharedmemory *shm,
			   void **pbuff, u32 *desc)
{
	int ret = 0;
	void *base;
	struct tee_mem *mem = tee_mem_register(shm->hwmem_gname);

	if (!mem) {
		dev_err(&pdev->dev, "[%s] couldn't register buffer\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	*pbuff = (void *)tee_mem_get_paddr(mem);
	if (!*pbuff) {
		dev_err(&pdev->dev, "[%s] incorrect physical address\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	/* If base differs from buffer we have an offset in shm */
	base = tee_mem_get_base(shm->buffer);
	if (base != shm->buffer)
		*pbuff = (uint8_t *)*pbuff + ((u32)shm->buffer - (u32)base);

	*desc = tee_mem_get_desc(mem);
exit:
	/*
	 * Always release the buffer, the kernel do not own it.
	 */
	tee_mem_free(mem);

	return ret;
}

static int copy_memref_to_kernel(struct tee_session *ts,
				 struct tee_cmd *ku_buffer,
				 struct tee_mem **mem,
				 int memref)
{
	int ret = -EINVAL;

	ts->uc.op->param_flags[memref] = 0;

	switch (TEEC_PARAM_TYPE_GET(ts->uc.op->types, memref)) {
	case TEEC_NONE:
	case TEEC_VALUE_INPUT:
	case TEEC_VALUE_OUTPUT:
	case TEEC_VALUE_INOUT:
		/* nothing to copy or allocate */
		return 0;
	case TEEC_MEMREF_TEMP_INPUT:
	case TEEC_MEMREF_TEMP_OUTPUT:
	case TEEC_MEMREF_TEMP_INOUT:
	{
		ret = tee_cpy_memref(
			ts, &mem[memref],
			ku_buffer->op->param[memref].tmpref.buffer,
			ku_buffer->op->param[memref].tmpref.size,
			memref);

		break;
	}
	case TEEC_MEMREF_WHOLE:
	{
		struct tee_sharedmemory *shm = &ts->uc.op->shm[memref];

		if (copy_from_user(
			    shm, ku_buffer->op->param[memref].memref.parent,
			    sizeof(struct tee_sharedmemory)))
			return -EINVAL;

		if (shm->hwmem_gname <= 0) {
			ret = tee_cpy_memref(ts, &mem[memref], shm->buffer,
					     shm->size, memref);
		} else {
			ts->uc.op->param[memref].tmpref.size = shm->size;
			ret = tee_resolve_shm(
				shm, &ts->uc.op->param[memref].tmpref.buffer,
				&ts->uc.op->param_flags[memref]);
		}
		break;
	}
	case TEEC_MEMREF_PARTIAL_INPUT:
	case TEEC_MEMREF_PARTIAL_OUTPUT:
	case TEEC_MEMREF_PARTIAL_INOUT:
	{
		struct tee_sharedmemory *shm = &ts->uc.op->shm[memref];
		u32 offset = ku_buffer->op->param[memref].memref.offset;
		u32 size = ku_buffer->op->param[memref].memref.size;
		void *pbuf;

		if (copy_from_user(
			    shm, ku_buffer->op->param[memref].memref.parent,
			    sizeof(struct tee_sharedmemory)))
			return -EINVAL;

		if (shm->hwmem_gname <= 0) {
			ret = tee_cpy_memref(
				ts, &mem[memref],
				(uint8_t *)shm->buffer + offset, size, memref);
		} else {
			ts->uc.op->param[memref].tmpref.size = size;
			ret = tee_resolve_shm(
				shm, &pbuf, &ts->uc.op->param_flags[memref]);
			ts->uc.op->param[memref].tmpref.buffer =
				pbuf + offset;
		}
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int copy_op(struct tee_session *ts,
		   struct tee_cmd *ku_buffer,
		   struct tee_mem **mem)
{
	int i;

	ts->uc.op = kmalloc(sizeof(struct tee_operation), GFP_KERNEL);

	if (!ts->uc.op) {
		dev_err(&pdev->dev, "[%s] error, out of memory (op)\n",
			__func__);
		set_emsg(ts, TEED_ERROR_OUT_OF_MEMORY, __LINE__);

		return -ENOMEM;
	}
	*ts->uc.op = *ku_buffer->op;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		int ret = copy_memref_to_kernel(ts, ku_buffer, mem, i);
		if (ret) {
			dev_err(&pdev->dev,
				"[%s] failed copy memref[%d] to kernel\n",
				__func__, i);
			return ret;
		}
	}

	return 0;
}

static int open_tee_device(struct tee_session *ts,
			   struct tee_cmd *ku_buffer,
			   struct tee_cmd __user *u_buffer)
{
	int i;
	int ret;
	struct tee_mem *mem[TEEC_CONFIG_PAYLOAD_REF_COUNT] = {0};

	if (ku_buffer->driver_cmd != TEED_OPEN_SESSION) {
		set_emsg(ts, TEED_ERROR_BAD_STATE, __LINE__);
		ret = -EINVAL;
		goto err;
	}

	if (ku_buffer->data == NULL && ku_buffer->uuid == NULL) {
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		ret = -EINVAL;
		goto err;
	}

	if (ku_buffer->data) {
		ret = copy_ta(ts, ku_buffer);
		if (ret)
			goto err;
	}

	if (ku_buffer->uuid) {
		ret = copy_uuid(ts, ku_buffer);
		if (ret)
			goto err;
	}

	if (ku_buffer->op) {
		ret = copy_op(ts, ku_buffer, mem);
		if (ret)
			goto err;
	}

	ts->id = 0;
	ts->login = TEEC_LOGIN_PUBLIC;

	/* Call secure world */
	if (call_sec_world(ts, TEED_OPEN_SESSION)) {
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		ret = -EINVAL;
		goto err;
	}

	if (ts->uc.err != TEED_SUCCESS)
		dev_err(&pdev->dev,
			"[%s], call_sec_world, err [%x], org [%x]\n",
			__func__, ts->uc.err, ts->uc.origin);

	if (ts->uc.op) {
		struct tee_operation *uop;
		if (get_user(uop, &u_buffer->op)) {
			dev_err(&pdev->dev, "[%s] failed get operation\n" ,
				__func__);
			goto err;
		}

		for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
			ret = copy_memref_to_user(ts, ku_buffer->op, uop, i);
			if (ret) {
				dev_err(&pdev->dev,
					"[%s] failed copy memref[%d] to user\n",
					__func__, i);
				goto err;
			}
		}
	}

err:
	free_operation(ts, mem);
	if (!ret)
		ts->state = TEED_STATE_OPEN_SESSION;

	return ret;
}

static int invoke_command(struct tee_session *ts,
			  struct tee_cmd *ku_buffer,
			  struct tee_cmd __user *u_buffer)
{
	int i;
	int ret = 0;
	struct tee_mem *mem[TEEC_CONFIG_PAYLOAD_REF_COUNT] = {0};

	if (ku_buffer->op) {
		ret = copy_op(ts, ku_buffer, mem);
		if (ret)
			goto err;
	}

	ts->uc.cmd = ku_buffer->cmd;

	if (call_sec_world(ts, TEED_INVOKE)) {
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		ret = -EINVAL;
		goto err;
	}

	if (ts->uc.err != TEED_SUCCESS)
		dev_err(&pdev->dev,
			"[%s], call_sec_world, err [%x], org [%x]\n",
			__func__, ts->uc.err, ts->uc.origin);

	if (ts->uc.op) {
		struct tee_operation *uop;
		if (get_user(uop, &u_buffer->op)) {
			dev_err(&pdev->dev, "[%s] failed get operation\n",
				__func__);
			goto err;
		}

		for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
			ret = copy_memref_to_user(ts, ku_buffer->op, uop, i);
			if (ret) {
				dev_err(&pdev->dev,
					"[%s] failed copy memref[%d] to user\n",
					__func__, i);
				goto err;
			}
		}
	}
err:
	free_operation(ts, mem);

	return ret;
}

static int tee_open(struct inode *inode, struct file *filp)
{
	struct tee_session *ts;
	filp->private_data = kmalloc(sizeof(struct tee_session),
				     GFP_KERNEL);

	if (filp->private_data == NULL) {
		dev_err(&pdev->dev, "[%s] allocation failed\n", __func__);
		return -ENOMEM;
	}

	ts = (struct tee_session *)(filp->private_data);
	reset_session(ts);

	return 0;
}

static int tee_release(struct inode *inode, struct file *filp)
{
	struct tee_session *ts = filp->private_data;

	/* Close session to secure world if a session is open */
	if (ts->state == TEED_STATE_OPEN_SESSION)
		/*
		 * Ignore error code, if there's an error it's an internal
		 * error that would only confuse the client.
		 */
		(void)teec_close_session(ts);

	kfree(filp->private_data);
	filp->private_data = NULL;

	return 0;
}

/*
 * Called when a process, which already opened the dev file, attempts
 * to read from it. This function gets the current status of the session.
 */
static int tee_read(struct file *filp, char __user *buffer,
		    size_t length, loff_t *offset)
{
	struct tee_read buf;
	struct tee_session *ts;

	if (length != sizeof(struct tee_read)) {
		dev_err(&pdev->dev, "[%s] error, incorrect input length\n",
			__func__);
		return -EINVAL;
	}

	ts = (struct tee_session *)(filp->private_data);

	if (ts == NULL) {
		dev_err(&pdev->dev,
			"[%s] error, private_data not initialized\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&sync);

	buf.err = ts->uc.err;
	buf.origin = ts->uc.origin;

	mutex_unlock(&sync);

	if (copy_to_user(buffer, &buf, length)) {
		dev_err(&pdev->dev, "[%s] error, copy_to_user failed!\n",
			__func__);
		return -EINVAL;
	}

	return length;
}

/*
 * Called when a process writes to a dev file.
 */
static int tee_write(struct file *filp, const char __user *buffer,
		     size_t length, loff_t *offset)
{
	struct tee_cmd ku_buffer;
	struct tee_session *ts;
	int ret = 0;

	if (length != sizeof(struct tee_cmd)) {
		dev_err(&pdev->dev, "[%s] error, incorrect input length\n",
			__func__);
		return -EINVAL;
	}
	if (copy_from_user(&ku_buffer, buffer, length)) {
		dev_err(&pdev->dev,
			"[%s] error, tee_session copy_from_user failed\n",
			__func__);
		return -EINVAL;
	}

	ts = (struct tee_session *)(filp->private_data);

	if (ts == NULL) {
		dev_err(&pdev->dev,
			"[%s] error, private_data not initialized\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&sync);

	switch (ts->state) {
	case TEED_STATE_OPEN_DEV:
		switch (ku_buffer.driver_cmd) {
		case TEED_OPEN_SESSION:
			ret = open_tee_device(ts, &ku_buffer,
					      (struct tee_cmd *)buffer);
			break;
		default:
			set_emsg(ts, TEED_ERROR_BAD_PARAMETERS, __LINE__);
			ret = -EINVAL;
		}
		break;

	case TEED_STATE_OPEN_SESSION:
		switch (ku_buffer.driver_cmd) {
		case TEED_INVOKE:
			ret = invoke_command(ts, &ku_buffer,
					     (struct tee_cmd *)buffer);
			break;

		case TEED_CLOSE_SESSION:
			if (call_sec_world(ts, TEED_CLOSE_SESSION)) {
				set_emsg(ts, TEED_ERROR_COMMUNICATION,
					 __LINE__);
				ret = -EINVAL;
			}

			kfree(ts->uc.data);
			ts->uc.data = NULL;

			kfree(ts->uc.uuid);
			ts->uc.uuid = NULL;

			reset_session(ts);
			break;

		default:
			set_emsg(ts, TEED_ERROR_BAD_PARAMETERS, __LINE__);
			ret = -EINVAL;
		}
		break;
	default:
		dev_err(&pdev->dev, "[%s] unknown state\n", __func__);
		set_emsg(ts, TEED_ERROR_BAD_STATE, __LINE__);
		ret = -EINVAL;
	}

	/*
	 * We expect that ret has value zero when reaching the end here.
	 * If it has any other value some error must have occured.
	 */
	if (!ret)
		ret = length;
	else {
		dev_err(&pdev->dev, "[%s], forcing error to -EINVAL\n",
			__func__);
		ret = -EINVAL;
	}

	mutex_unlock(&sync);

	return ret;
}

static int teec_initialize_context_local(const char *name,
					 struct tee_context *context)
{
	return TEED_SUCCESS;
}

static int teec_finalize_context_local(struct tee_context *context)
{
	return TEED_SUCCESS;
}

static int teec_open_session_local(struct tee_context *context,
		      struct tee_session *session,
		      const struct tee_uuid *destination,
		      unsigned int connection_method,
		      void *connection_data, struct tee_operation *operation,
		      unsigned int *error_origin)
{
	int res = TEED_SUCCESS;

	mutex_lock(&sync);

	if (session == NULL || destination == NULL) {
		dev_err(&pdev->dev, "[%s] session or destination == NULL\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_BAD_PARAMETERS;
		goto exit;
	}

	reset_session(session);

	/*
	 * Open a session towards an application already loaded inside
	 * the TEE.
	 */
	session->uc.uuid = kmalloc(sizeof(struct tee_uuid), GFP_KERNEL);
	if (session->uc.uuid == NULL) {
		dev_err(&pdev->dev, "[%s] error, out of memory (uuid)\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_OUT_OF_MEMORY;
		goto exit;
	}
	memcpy(session->uc.uuid, destination, sizeof(struct tee_uuid));

	session->uc.data = NULL;
	session->id = 0;
	session->uc.op = operation;
	session->login = TEEC_LOGIN_KERNEL;

	/*
	 * Call secure world
	 */
	if (call_sec_world(session, TEED_OPEN_SESSION)) {
		dev_err(&pdev->dev, "[%s] error, call_sec_world failed\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_GENERIC;
	}
	if (session->uc.err != TEED_SUCCESS) {
		dev_err(&pdev->dev, "[%s] error, call_sec_world failed\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = session->uc.origin;
		res = session->uc.err;
	}

	session->uc.op = NULL;

exit:
	mutex_unlock(&sync);
	return res;
}

static int teec_close_session_local(struct tee_session *session)
{
	int res = TEED_SUCCESS;

	mutex_lock(&sync);

	if (session == NULL) {
		dev_err(&pdev->dev, "[%s] error, session == NULL\n", __func__);
		res = TEED_ERROR_BAD_PARAMETERS;
		goto exit;
	}

	if (call_sec_world(session, TEED_CLOSE_SESSION)) {
		dev_err(&pdev->dev, "[%s] error, call_sec_world failed\n",
			__func__);
		res = TEED_ERROR_GENERIC;
		goto exit;
	}

exit:
	if (session != NULL) {
		kfree(session->uc.uuid);
		session->uc.uuid = NULL;
	}

	mutex_unlock(&sync);
	return res;
}

static int teec_invoke_command_local(
	struct tee_session *session, unsigned int command_id,
	struct tee_operation *operation,
	unsigned int *error_origin)
{
	int res = TEED_SUCCESS;

	mutex_lock(&sync);

	if (session == NULL || operation == NULL || error_origin == NULL) {
		dev_err(&pdev->dev, "[%s] error, input parameters == NULL\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_BAD_PARAMETERS;
		goto exit;
	}

	session->uc.op = operation;
	session->uc.cmd = command_id;

	/*
	 * Call secure world
	 */
	if (call_sec_world(session, TEED_INVOKE)) {
		dev_err(&pdev->dev, "[%s] error, call_sec_world failed\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_GENERIC;
	}
	if (session->uc.err != TEED_SUCCESS) {
		dev_err(&pdev->dev, "[%s] error, call_sec_world failed\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = session->uc.origin;
		res = session->uc.err;
	}

	session->uc.op = NULL;

exit:
	mutex_unlock(&sync);
	return res;
}

static int teec_allocate_shared_memory_local(struct tee_context *context,
				    struct tee_sharedmemory *shared_memory)
{
	int res = TEED_SUCCESS;

	if (shared_memory == NULL) {
		res = TEED_ERROR_BAD_PARAMETERS;
		goto exit;
	}

	shared_memory->buffer = kmalloc(shared_memory->size,
					GFP_KERNEL);

	if (shared_memory->buffer == NULL) {
		res = TEED_ERROR_OUT_OF_MEMORY;
		goto exit;
	}

exit:
	return res;
}

static void teec_release_shared_memory_local(
				   struct tee_sharedmemory *shared_memory)
{
	kfree(shared_memory->buffer);
}

static char *tee_opts;

static int tee_setup(char *str)
{
	if (!str)
		return 0;

	tee_opts = str;

	return 1;
}
__setup("mem_issw=", tee_setup);

static int tee_exit(void)
{
	tee_supp_exit();
	return 0;
}

static const struct tee_export_operations eops = {
	.initialize = teec_initialize_context_local,
	.finalize = teec_finalize_context_local,
	.open_session = teec_open_session_local,
	.close_session = teec_close_session_local,
	.invoke = teec_invoke_command_local,
	.allocate_shared_memory = teec_allocate_shared_memory_local,
	.release_shared_memory = teec_release_shared_memory_local
};

static const struct file_operations fops = {
	.read = tee_read,
	.write = tee_write,
	.open = tee_open,
	.release = tee_release,
	.unlocked_ioctl = tee_ioctl
};

static const struct tee_internal_operations tops = {
	.init = tee_init_100,
	.exit = tee_exit,
	.setup = tee_setup
};

int tee_init_100(struct platform_device *pd)
{
	int err = 0;
	u32 size = 0;
	u32 addr = 0;
	struct tee_platform_data *tee_pdata;

	if (!pd) {
		pr_err(TEED_PFX "[%s] error, no platform device\n", __func__);
		return -EINVAL;
	}

	pdev = pd;
	tee_pdata = (struct tee_platform_data *)pdev->dev.platform_data;
	tee_pdata->eops = eops;
	tee_pdata->fops = fops;
	tee_pdata->tops = tops;

	if (!tee_opts) {
		dev_err(&pdev->dev,
			"[%s] error, command line option mem_issw is missing or incorrect\n",
			__func__);
		return -EFAULT;
	}


	size = memparse(tee_opts, &tee_opts);

	if (*tee_opts != '@') {
		err = -EINVAL;
		goto exit;
	}

	addr = memparse(tee_opts + 1, &tee_opts);

	if (size == 0 || addr == 0) {
		err = -EINVAL;
		goto exit;
	}

	err = tee_mem_init(addr, size);
	if (err)
		goto exit;

	mutex_init(&sync);

	err = tee_supp_init();

exit:
	if (err)
		dev_err(&pdev->dev, "[%s] error, [%d]", __func__, err);

	return err;
}
