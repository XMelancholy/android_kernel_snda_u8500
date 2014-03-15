/*
 * TEE driver implementation that matches Global Platform Specification API
 * 0.17.
 *
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/tee.h>
#include <linux/slab.h>
#include <linux/hwmem.h>

/* tee tee to get same prefix as from the device. */
#define TEED_PFX "tee tee: "

#define TEED_STATE_OPEN_DEV 0
#define TEED_STATE_OPEN_SESSION 1

#define ISSWAPI_EXECUTE_TA 0x11000001
#define ISSWAPI_CLOSE_TA   0x11000002
#define SEC_ROM_NO_FLAG_MASK 0x0000

static struct mutex sync;
static struct platform_device *pdev;

struct tee_sharedmemory_legacy {
	void *buffer;
	size_t size;
	__u32 flags;
};

struct tee_operation_legacy {
	struct tee_sharedmemory_legacy shm[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	__u32 flags;
};

static void copy_tee_op_to_legacy(struct tee_operation_legacy *dest,
				  const struct tee_operation *src)
{
	size_t i;

	if (!src || !dest) {
		pr_err("[%s]: ERROR, NULL parameter as inparameter src: 0x%08x, dest: 0x%08x\n",
		       __func__, (__u32)src, (__u32)dest);
		return;
	}

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; i++) {
		dest->shm[i].buffer = src->shm[i].buffer;
		dest->shm[i].size = src->shm[i].size;
		dest->shm[i].flags = src->shm[i].flags;
	}
	dest->flags = src->flags;
}

static void copy_tee_op_from_legacy(struct tee_operation *dest,
				    const struct tee_operation_legacy *src)
{
	size_t i;

	if (!src || !dest) {
		pr_err("[%s]: ERROR, NULL parameter as inparameter src: 0x%08x, dest: 0x%08x\n",
		       __func__, (__u32)src, (__u32)dest);
		return;
	}

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; i++) {
		dest->shm[i].buffer = src->shm[i].buffer;
		dest->shm[i].size = src->shm[i].size;
		dest->shm[i].flags = src->shm[i].flags;
	}
	dest->flags = src->flags;
}

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

static int call_sec_world(struct tee_session *ts, int sec_cmd)
{
	struct tee_operation_legacy op_legacy;
	memset(&op_legacy, 0, sizeof(struct tee_operation_legacy));

	/*
	 * ts->ta and ts->uuid is set to NULL when opening the device, hence it
	 * should be safe to just do the call here.
	 */
	switch (sec_cmd) {
	case TEED_INVOKE:
	copy_tee_op_to_legacy(&op_legacy, ts->uc.op);
	if (!ts->uc.uuid) {
		call_sec_rom_bridge(ISSWAPI_EXECUTE_TA,
				SEC_ROM_NO_FLAG_MASK,
				virt_to_phys(&ts->id),
				NULL,
				((struct ta_addr *)ts->uc.data)->paddr,
				ts->uc.cmd,
				virt_to_phys((void *)(&op_legacy)),
				virt_to_phys((void *)(&ts->uc.err)),
				virt_to_phys((void *)(&ts->uc.origin)));
	} else {
		call_sec_rom_bridge(ISSWAPI_EXECUTE_TA,
				SEC_ROM_NO_FLAG_MASK,
				virt_to_phys(&ts->id),
				virt_to_phys(ts->uc.uuid),
				NULL,
				ts->uc.cmd,
				virt_to_phys((void *)(&op_legacy)),
				virt_to_phys((void *)(&ts->uc.err)),
				virt_to_phys((void *)(&ts->uc.origin)));
	}
	copy_tee_op_from_legacy(ts->uc.op, &op_legacy);
	break;

	case TEED_CLOSE_SESSION:
	call_sec_rom_bridge(ISSWAPI_CLOSE_TA,
			    SEC_ROM_NO_FLAG_MASK,
			    ts->id,
			    NULL,
			    NULL,
			    virt_to_phys((void *)(&ts->uc.err)));

	/*
	 * Since the TEE Client API does NOT take care of the return value, we
	 * print a warning here if something went wrong in secure world.
	 */
	if (ts->uc.err != TEED_SUCCESS)
		pr_warning("[%s] failed in secure world\n", __func__);

	break;
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
	int ret = -EINVAL;
	size_t mem_chunks_length = 1;
	struct hwmem_mem_chunk mem_chunks;
	struct ta_addr *ta_addr;

	ts->uc.data_size = ku_buffer->data_size;

	if (ts->uc.data_size == 0)
		return 0;

	ta_addr = kmalloc(sizeof(struct ta_addr), GFP_KERNEL);
	if (!ta_addr)
		return -ENOMEM;

	ta_addr->paddr = NULL;
	ta_addr->vaddr = NULL;

	ta_addr->alloc = hwmem_alloc(ts->uc.data_size,
				     (HWMEM_ALLOC_HINT_WRITE_COMBINE |
				      HWMEM_ALLOC_HINT_CACHED |
				      HWMEM_ALLOC_HINT_CACHE_WB |
				      HWMEM_ALLOC_HINT_CACHE_AOW |
				      HWMEM_ALLOC_HINT_INNER_AND_OUTER_CACHE),
				     (HWMEM_ACCESS_READ | HWMEM_ACCESS_WRITE |
				      HWMEM_ACCESS_IMPORT),
				     HWMEM_MEM_CONTIGUOUS_SYS);

	if (IS_ERR(ta_addr->alloc)) {
		set_emsg(ts, TEED_ERROR_OUT_OF_MEMORY, __LINE__);
		dev_err(&pdev->dev, "[%s] couldn't alloc hwmem for TA\n",
			__func__);
		kfree(ta_addr);
		return PTR_ERR(ta_addr->alloc);
	}

	ret = hwmem_pin(ta_addr->alloc, &mem_chunks, &mem_chunks_length);
	if (ret) {
		dev_err(&pdev->dev, "[%s] couldn't pin TA buffer\n",
			__func__);
		kfree(ta_addr);
		return ret;
	}

	ta_addr->paddr = (void *)mem_chunks.paddr;
	ta_addr->vaddr = hwmem_kmap(ta_addr->alloc);
	ts->uc.data = ta_addr;

	if (copy_from_user(ta_addr->vaddr, ku_buffer->data,
			   ts->uc.data_size)) {
		dev_err(&pdev->dev, "[%s] error, copy_from_user failed\n",
			__func__);
	}

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
			   sizeof(struct tee_uuid))) {
		dev_err(&pdev->dev, "[%s] error, copy_from_user failed\n",
			__func__);
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		return -EIO;
	}

	return 0;
}

static inline void free_ta(struct ta_addr *ta)
{
	if (ta->alloc) {
		hwmem_kunmap(ta->alloc);
		hwmem_unpin(ta->alloc);
		hwmem_release(ta->alloc);
	}

	ta->alloc = NULL;
	ta->paddr = NULL;
	ta->vaddr = NULL;

	kfree(ta);
	ta = NULL;
}

static inline void free_operation(struct tee_session *ts,
				  struct hwmem_alloc **alloc,
				  int memrefs_allocated)
{
	int i;

	for (i = 0; i < memrefs_allocated; ++i) {
		if (ts->uc.op->shm[i].buffer) {
			hwmem_kunmap(alloc[i]);
			hwmem_unpin(alloc[i]);
			hwmem_release(alloc[i]);
			ts->uc.op->shm[i].buffer = NULL;
		}

		if (ts->vaddr[i])
			ts->vaddr[i] = NULL;
	}

	kfree(ts->uc.op);
	ts->uc.op = NULL;
}

static inline void memrefs_phys_to_virt(struct tee_session *ts)
{
	int i;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		if (ts->uc.op->flags & (1 << i)) {
			ts->uc.op->shm[i].buffer =
				phys_to_virt((unsigned long)
					     ts->uc.op->shm[i].buffer);
		}
	}
}

static inline void memrefs_virt_to_phys(struct tee_session *ts)
{
	int i;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		if (ts->uc.op->flags & (1 << i)) {
			ts->uc.op->shm[i].buffer =
				(void *)virt_to_phys(ts->uc.op->shm[i].buffer);
		}
	}
}

static int copy_memref_to_user(struct tee_session *ts,
			       struct tee_operation __user *ubuf_op,
			       int memref)
{
	unsigned long bytes_left;

	bytes_left = copy_to_user(ubuf_op->shm[memref].buffer,
				  ts->vaddr[memref],
				  ts->uc.op->shm[memref].size);

	if (bytes_left != 0) {
		dev_err(&pdev->dev, "[%s] failed to copy result to user space (%lu bytes left of buffer).\n",
			__func__, bytes_left);
		return bytes_left;
	}

	bytes_left = put_user(ts->uc.op->shm[memref].size,
			      &ubuf_op->shm[memref].size);

	if (bytes_left != 0) {
		dev_err(&pdev->dev, "[%s] failed to copy result to user space (%lu bytes left of size).\n",
			__func__, bytes_left);
		return -EINVAL;
	}

	bytes_left = put_user(ts->uc.op->shm[memref].flags,
			      &ubuf_op->shm[memref].flags);
	if (bytes_left != 0) {
		dev_err(&pdev->dev, "[%s] failed to copy result to user space (%lu bytes left of flags).\n",
			__func__, bytes_left);
		return -EINVAL;
	}

	return 0;
}

static int copy_memref_to_kernel(struct tee_session *ts,
				 struct tee_cmd *ku_buffer,
				 struct hwmem_alloc **alloc,
				 int memref)
{
	int ret = -EINVAL;
	size_t mem_chunks_length = 1;
	struct hwmem_mem_chunk mem_chunks;

	if (ku_buffer->op->shm[memref].size != 0) {
		alloc[memref] = hwmem_alloc(
				    ku_buffer->op->shm[memref].size,
				    (HWMEM_ALLOC_HINT_WRITE_COMBINE |
				     HWMEM_ALLOC_HINT_CACHED |
				     HWMEM_ALLOC_HINT_CACHE_WB |
				     HWMEM_ALLOC_HINT_CACHE_AOW |
				     HWMEM_ALLOC_HINT_INNER_AND_OUTER_CACHE),
				    (HWMEM_ACCESS_READ | HWMEM_ACCESS_WRITE |
				     HWMEM_ACCESS_IMPORT),
				    HWMEM_MEM_CONTIGUOUS_SYS);

		if (IS_ERR(alloc[memref])) {
			dev_err(&pdev->dev, "[%s] couldn't alloc hwmem_alloc (memref: %d)\n",
				__func__, memref);
			return PTR_ERR(alloc[memref]);
		}

		ret = hwmem_pin(alloc[memref], &mem_chunks, &mem_chunks_length);
		if (ret) {
			dev_err(&pdev->dev, "[%s] couldn't pin buffer (memref: %d)\n",
				__func__, memref);
			return ret;
		}

		/*
		 * Since phys_to_virt is not working for hwmem memory we are
		 * storing the virtual addresses in separate array in
		 * tee_session and we keep the address of the physical pointers
		 * in the memref buffer.
		 */
		ts->uc.op->shm[memref].buffer = (void *)mem_chunks.paddr;
		ts->vaddr[memref] = hwmem_kmap(alloc[memref]);

		/*
		 * Buffer unmapped/freed in invoke_command if this function
		 * fails.
		 */
		if (!ts->uc.op->shm[memref].buffer || !ts->vaddr[memref]) {
			dev_err(&pdev->dev, "[%s] out of memory (memref: %d)\n",
				__func__, memref);
			return -ENOMEM;
		}

		if (ku_buffer->op->shm[memref].flags & TEEC_MEM_INPUT) {
			if (copy_from_user(ts->vaddr[memref],
					   ku_buffer->op->shm[memref].buffer,
					   ku_buffer->op->shm[memref].size)) {
				dev_err(&pdev->dev, "[%s] error, copy_from_user failed\n",
					__func__);
				set_emsg(ts, TEED_ERROR_COMMUNICATION,
					 __LINE__);
				return -EIO;
			}
		}
	} else {
		ts->uc.op->shm[memref].buffer = NULL;
		ts->vaddr[memref] = NULL;
	}

	ts->uc.op->shm[memref].size = ku_buffer->op->shm[memref].size;
	ts->uc.op->shm[memref].flags = ku_buffer->op->shm[memref].flags;

	return 0;
}

static int open_tee_device(struct tee_session *ts,
			   struct tee_cmd *ku_buffer)
{
	int ret;

	if (ku_buffer->driver_cmd != TEED_OPEN_SESSION) {
		set_emsg(ts, TEED_ERROR_BAD_STATE, __LINE__);
		return -EINVAL;
	}

	if (ku_buffer->data && ku_buffer->data_size > 0)
		ret = copy_ta(ts, ku_buffer);
	else if (ku_buffer->uuid)
		ret = copy_uuid(ts, ku_buffer);
	else {
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		return -EINVAL;
	}

	ts->id = 0;
	ts->state = TEED_STATE_OPEN_SESSION;
	return ret;
}

static int invoke_command(struct tee_session *ts,
			  struct tee_cmd *ku_buffer,
			  struct tee_cmd __user *u_buffer)
{
	int i;
	int ret = 0;
	/* To keep track of which memrefs to free when failure occurs. */
	int memrefs_allocated = 0;
	struct hwmem_alloc *alloc[TEEC_CONFIG_PAYLOAD_REF_COUNT];

	ts->uc.op = kmalloc(sizeof(struct tee_operation), GFP_KERNEL);

	if (!ts->uc.op) {
		if (ts->uc.op == NULL) {
			dev_err(&pdev->dev, "[%s] error, out of memory (op)\n",
				__func__);
			set_emsg(ts, TEED_ERROR_OUT_OF_MEMORY, __LINE__);
			return -ENOMEM;
		}
	}

	ts->uc.op->flags = ku_buffer->op->flags;
	ts->uc.cmd = ku_buffer->cmd;

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		ts->uc.op->shm[i].buffer = NULL;
		memrefs_allocated++;

		/* We only want to copy memrefs in use to kernel space. */
		if (ku_buffer->op->flags & (1 << i)) {
			ret = copy_memref_to_kernel(ts, ku_buffer, alloc, i);
			if (ret) {
				dev_err(&pdev->dev, "[%s] failed copy memref[%d] to kernel",
					__func__, i);
				goto err;
			}
		} else {
			ts->uc.op->shm[i].size = 0;
			ts->uc.op->shm[i].flags = 0;
		}
	}

	if (call_sec_world(ts, TEED_INVOKE)) {
		set_emsg(ts, TEED_ERROR_COMMUNICATION, __LINE__);
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < TEEC_CONFIG_PAYLOAD_REF_COUNT; ++i) {
		if ((ku_buffer->op->flags & (1 << i)) &&
		    (ku_buffer->op->shm[i].flags & TEEC_MEM_OUTPUT) &&
		    (ts->vaddr[i] != NULL)) {
			ret = copy_memref_to_user(ts, u_buffer->op, i);
			if (ret) {
				dev_err(&pdev->dev, "[%s] failed copy memref[%d] to user",
					__func__, i);
				goto err;
			}
		}
	}
err:
	free_operation(ts, alloc, memrefs_allocated);

	return ret;
}

static int tee_open(struct inode *inode, struct file *filp)
{
	struct tee_session *ts;
	filp->private_data = kmalloc(sizeof(struct tee_session),
				     GFP_KERNEL);

	if (filp->private_data == NULL) {
		dev_err(&pdev->dev, "[%s] allocation failed", __func__);
		return -ENOMEM;
	}

	ts = (struct tee_session *)(filp->private_data);
	reset_session(ts);

	return 0;
}

static int tee_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	filp->private_data = NULL;

	return 0;
}


/*
 * Called when a process, which already opened the dev file, attempts
 * to read from it. This function gets the current status of the session.
 */
static int tee_read(struct file *filp, char __user *buffer, size_t length,
		    loff_t *offset)
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
		dev_err(&pdev->dev, "[%s] error, private_data not initialized\n",
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
		dev_err(&pdev->dev, "[%s] error, tee_cmd copy_from_user failed\n",
			__func__);
		return -EINVAL;
	}

	ts = (struct tee_session *)(filp->private_data);

	if (ts == NULL) {
		dev_err(&pdev->dev, "[%s] error, private_data not initialized\n",
			__func__);
		return -EINVAL;
	}

	mutex_lock(&sync);

	switch (ts->state) {
	case TEED_STATE_OPEN_DEV:
		ret = open_tee_device(ts, &ku_buffer);
		break;

	case TEED_STATE_OPEN_SESSION:
		switch (ku_buffer.driver_cmd) {
		case TEED_INVOKE:
			ret = invoke_command(ts, &ku_buffer,
					     (struct tee_cmd *)buffer);
			break;

		case TEED_CLOSE_SESSION:
			/* no caching implemented yet... */
			if (call_sec_world(ts, TEED_CLOSE_SESSION)) {
				set_emsg(ts, TEED_ERROR_COMMUNICATION,
					 __LINE__);
				ret = -EINVAL;
			}

			if (ts->uc.data)
				free_ta(ts->uc.data);

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
	if (!ret) {
		ret = length;
	} else {
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

static int teec_open_session_local(
		  struct tee_context *context,
		  struct tee_session *session,
		  const struct tee_uuid *destination,
		  unsigned int connection_method,
		  void *connection_data, struct tee_operation *operation,
		  unsigned int *error_origin)
{
	int res = TEED_SUCCESS;

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

exit:
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
	int i;

	mutex_lock(&sync);

	if (session == NULL || operation == NULL || error_origin == NULL) {
		dev_err(&pdev->dev, "[%s] error, input parameters == NULL\n",
			__func__);
		if (error_origin != NULL)
			*error_origin = TEED_ORIGIN_DRIVER;
		res = TEED_ERROR_BAD_PARAMETERS;
		goto exit;
	}

	for (i = 0; i < 4; ++i) {
		/* We only want to translate memrefs in use. */
		if (operation->flags & (1 << i)) {
			operation->shm[i].buffer =
				(void *)virt_to_phys(
					operation->shm[i].buffer);
		}
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

	memrefs_phys_to_virt(session);
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

static int tee_exit(void)
{
	dev_err(&pdev->dev, "[%s] NOT IMPLEMENTED!\n", __func__);
	return 0;
}

static int tee_setup(char *str)
{
	(void)str;
	dev_err(&pdev->dev, "[%s] NOT IMPLEMENTED!\n", __func__);
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
	.release = tee_release
};

static const struct tee_internal_operations tops = {
	.init = tee_init_017,
	.exit = tee_exit,
	.setup = tee_setup
};

int tee_init_017(struct platform_device *pd)
{
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

	mutex_init(&sync);
	return 0;
}
