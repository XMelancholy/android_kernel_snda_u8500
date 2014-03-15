/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * ST-Ericsson HVA core
 * Author: <thomas.costis@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include "hva_internals.h"

static void hva_stats_update(struct hva_instance *instance,
		unsigned long start_ts, unsigned long stop_ts)
{
	struct stats *s = &instance->stats;
	unsigned long duration = stop_ts - start_ts;

	if (s->min_duration > duration)
		s->min_duration = duration;
	if (s->max_duration < duration)
		s->max_duration = duration;

	if (s->total_task_nb > 0) {
		/* Deals with multiple in fifo -> sum_duration is ok */
		if (s->last_task_hw_stop_ts > start_ts)
			duration = stop_ts - s->last_task_hw_stop_ts;
	}

	s->sum_duration += duration;

	s->last_task_hw_stop_ts = stop_ts;
	s->last_task_hw_start_ts = start_ts;

	instance->stats.total_task_nb++;
}

/**
 * Sets hva_task from user request
 * Returns task index in task array, if available
 * else return error code (<0)
 * cmd = task_id | client_id | hva_cmd
 * Lock task_lock should be held to protect
 * any prehemption between begining of hva_request
 * till hardware triggered. This is to ensure that
 * tasks are given to HVA hardware keeping calling order,
 * as some dependencies between tasks can be there
 */
static int hva_acquire_task(struct hva_instance *instance,
		struct hva_task **ptask, struct hva_user_req *user_req)
{
	struct hva_core *core = (struct hva_core *)instance->data;
	struct hva_task *task;
	int i;

	/*
	 * As HVA hardware task_id container is only 16 bits we cannot use it
	 * to get back context on irq to acknowledge task completion,
	 * -> so a global array of hva tasks.
	 */
	for (i = 0; i < HW_HVA_FIFO_SIZE; i++) {
		if (!core->task[i].used)
			break;
	}

	/*
	 * At least one task should be available
	 * (as protected with hw_fifo_lock semaphore)
	 */
	if (i == HW_HVA_FIFO_SIZE) {
		/*
		 * Abnormal, but exits gracefully,
		 * completion timeout will unblocked
		 * stucked tasks
		 */
		dev_err(core->dev, "No more tasks available");

		up(&core->hw_fifo_lock);
		return -EAGAIN;
	}

	task = &core->task[i];
	task->used = true;
	task->id = i;
	task->client_id = user_req->client_id;
	task->cmd = user_req->cmd;
	task->task_desc = user_req->task_desc;
	task->status_reg = 0;
	init_completion(&task->completed);
	*ptask = task;

	dev_dbg(core->dev, "task[%d] acquired for instance %p\n",
		task->id, instance);
	return i;
}

/**
 * Mark task as unused and release semaphore on task array
 */
static void hva_release_task(struct hva_core *core, struct hva_task *task)
{
	down(&core->task_lock);
	task->used = false;
	up(&core->task_lock);
	dev_dbg(core->dev, "task[%d] released\n", task->id);
}

static int hva_request(struct file *filp, struct hva_user_req *user_req)
{
	struct hva_instance *instance =
	    (struct hva_instance *)filp->private_data;
	struct hva_core *core = (struct hva_core *)instance->data;
	struct hva_task *task;
	struct timeval start_time, stop_time;
	unsigned long start_ts, stop_ts;
	unsigned int status, info;
	int ret;

	dev_dbg(core->dev, "> hva_request instance %p\n", instance);

	/* Ensures that not more HW_HVA_FIFO_SIZE requests are done */
	dev_dbg(core->dev, "Down hardware fifo lock (instance:%p)\n",
		instance);
	ret = down_timeout(&core->hw_fifo_lock,
			msecs_to_jiffies(HVA_TASK_TIMEOUT_IN_MS));
	if (ret == -ETIME) {
		dev_err(core->dev, "Waiting for task semaphore timeout!");
		ret = -EAGAIN;
		goto bail;
	} else if (ret < 0) {
		dev_info(core->dev, "Waiting for task semaphore interrupted!");
		goto bail;
	}

	/*
	 * Ensures that tasks are processed in FIFO
	 * (first ioctl request is first in hardware)
	 */
	ret = down_timeout(&core->task_lock,
			msecs_to_jiffies(HVA_TASK_TIMEOUT_IN_MS));
	if (ret == -ETIME) {
		dev_err(core->dev, "Waiting for task order semaphore timeout!");
		ret = -EAGAIN;
		goto up_hw_fifo;
	} else if (ret < 0) {
		dev_info(core->dev, "Waiting for task order semaphore interrupted!");
		goto up_hw_fifo;
	}

	/* Do not allow any other tasks until power_off/reset has been done */
	if (core->hw_state == HVA_STUCKED) {
		dev_err(core->dev, "HVA stucked (waiting for reset)");
		ret = -EFAULT;
		goto up_task;
	}

	ret = hva_acquire_task(instance, &task, user_req);
	if (ret < 0) {
		dev_err(core->dev, "%s hva_acquire_task returns %d\n",
				__func__, ret);
		goto up_task;
	}

	dev_dbg(core->dev, "Sending task[%d] cmd: 0x%x\n",
		task->id, task->cmd);
	if (pm_runtime_get_sync(core->dev) < 0) {
		dev_err(core->dev, "%s pm_runtime_get_sync failed\n",
			 __func__);
		ret = -EAGAIN;
		/* Avoid deadlock in hva_release_task (down semaphore) */
		up(&core->task_lock);
		goto release_task;
	}

	hva_clock_on(core);

	do_gettimeofday(&start_time);

	hva_trigger_task(core, task);
	core->nb_of_tasks++;
	if (core->nb_of_tasks > core->max_nb_of_tasks) {
		core->max_nb_of_tasks = core->nb_of_tasks;
		dev_info(core->dev, "%d tasks in hardware fifo\n",
				core->max_nb_of_tasks);
	}

	dev_dbg(core->dev, "Wait for task[%d] user_task_id:%d\n",
		task->id,  user_req->user_task_id);

	/* Task is trigerred, we can unblock */
	up(&core->task_lock);

	/* Waiting for task completion */
	if (!wait_for_completion_timeout
	    (&task->completed, msecs_to_jiffies(HVA_TASK_TIMEOUT_IN_MS))) {
		dev_info(core->dev, "Wait for task[%d] -> TIMEOUT\n",
			 task->id);
		/* Time out detected */
		core->hw_state = HVA_STUCKED;
	}

	ret = hva_check_task_errors(instance, task,
			&status, &info);

	/* Release task as soon as possible after wait */
	hva_release_task(core, task);

	dev_dbg(core->dev, "Up hardware fifo lock (task[%d])\n",
		task->id);
	up(&core->hw_fifo_lock);

	hva_clock_off(core);

	pm_runtime_mark_last_busy(core->dev);
	pm_runtime_put_autosuspend(core->dev);

	do_gettimeofday(&stop_time);
	start_ts =
		start_time.tv_sec * USEC_PER_SEC + start_time.tv_usec;
	stop_ts =
		stop_time.tv_sec * USEC_PER_SEC + stop_time.tv_usec;
	hva_stats_update(instance, start_ts, stop_ts);

	/* User request update */
	user_req->status = status;
	user_req->info = info;
	user_req->hw_start_ts = start_ts;
	user_req->hw_stop_ts  = stop_ts;

	dev_dbg(core->dev, "< hva_request instance %p\n", instance);
	return ret;

release_task:
	hva_release_task(core, task);
up_task:
	up(&core->task_lock);
up_hw_fifo:
	up(&core->hw_fifo_lock);
bail:
	dev_dbg(core->dev, "< hva_request instance %p on error (%d)\n",
			instance, ret);
	return ret;
}

/**
 * HVA currently has 2 IRQ callbacks:
 *  - end of task (handling task completion is kept in the same file)
 *  - HW error
 */
static void hva_hw_callback(struct hva_core *core, __u32 sts_reg)
{
	__u16 task_idx = (__u16)(sts_reg >> 16) & 0x7;

	BUG_ON(!core->task[task_idx].used);
	core->task[task_idx].status_reg = sts_reg;
	complete(&core->task[task_idx].completed);
	dev_dbg(core->dev, "%s task[%d] completed\n",
			__func__, task_idx);
	core->nb_of_tasks--;
}

static void hva_hw_err_callback(struct hva_core *core)
{
	core->hw_state = HVA_RET_ERROR;
}

/**
 * File operations
 */
static int hva_open(struct inode *inode, struct file *filp)
{
	struct hva_core *core = container_of(filp->private_data,
					     struct hva_core, miscdev);
	struct hva_instance *instance;
	struct timeval start_time;

	/* Alloc control struct */
	instance = kzalloc(sizeof(struct hva_instance), GFP_KERNEL);
	if (!instance) {
		dev_err(core->dev, "HVA instance alloc failed\n");
		return -EINVAL;
	}

	filp->private_data = instance;

	instance->data = (void *)core;
	instance->dev = core->dev;
	instance->stats.tag = (unsigned int)filp;
	instance->stats.min_duration = ULONG_MAX;
	do_gettimeofday(&start_time);
	instance->stats.session_duration =
	    start_time.tv_sec * USEC_PER_SEC + start_time.tv_usec;


	if (core->instance_nb == 0) {
		core->nb_of_tasks = 0;
		core->max_nb_of_tasks = 0;
	}

	core->instance_nb++;

	dev_info(core->dev, "%s %p\n", __func__, instance);

	hva_opp_on(core, filp);
	return 0;
}

static int hva_release(struct inode *inode, struct file *filp)
{
	struct hva_instance *instance =
	    (struct hva_instance *)filp->private_data;
	struct hva_core *core = (struct hva_core *)instance->data;
	struct timeval stop_time;

	dev_info(core->dev, "%s %p\n", __func__, instance);

	hva_opp_off(core, filp);

	do_gettimeofday(&stop_time);
	instance->stats.session_duration =
	    stop_time.tv_sec * USEC_PER_SEC + stop_time.tv_usec -
	    instance->stats.session_duration;

	memcpy(&core->stats[core->index_stats], &instance->stats,
	       sizeof(struct stats));
	core->index_stats++;
	if (core->index_stats == MAX_NB_DEBUGFS_INSTANCES)
		core->index_stats = 0;
	kfree(instance);
	filp->private_data = NULL;

	core->instance_nb--;

	return 0;
}

/*
 * Returns 0 if OK else negative error code
 */
static long hva_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct hva_instance *instance =
	    (struct hva_instance *)filp->private_data;
	struct hva_core *core = (struct hva_core *)instance->data;

	dev_dbg(core->dev, "%s\n", __func__);

	switch (cmd) {
	case HVA_IOC:
		{
			struct hva_user_req user_req;
			if (copy_from_user(&user_req, (void *)arg,
					   sizeof(struct hva_user_req))) {
				dev_err(core->dev, "Copy from user failed");
				return -EAGAIN;
			}
			ret = hva_request(filp, &user_req);

			if (copy_to_user((void *)arg, &user_req,
					 sizeof(struct hva_user_req))) {
				dev_err(core->dev, "Copy to user failed");
				return -EAGAIN;
			}
		}
		break;

	case HVA_VERSION:
		{
			if (copy_to_user((unsigned int *)arg,
					 &core->chip_id, sizeof(unsigned int)))
				ret = -EFAULT;
		}
		break;

	case HVA_GET_REG:
		{
			struct hva_reg reg;

			if (copy_from_user(&reg, (void *)arg,
					   sizeof(struct hva_reg))) {
				dev_warn(core->dev,
					 "%s: copy_from_user failed\n",
					 __func__);
				return -EFAULT;
			}
			hva_get_reg(core, &reg);

			if (copy_to_user((struct hva_reg *)arg,
					 &reg, sizeof(struct hva_reg)))
				return -EFAULT;
		}
		break;
	default:
		dev_warn(core->dev, "%s: Unknown cmd %d\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations hva_fops = {
	.open = hva_open,
	.release = hva_release,
	.unlocked_ioctl = hva_ioctl,
	.owner = THIS_MODULE,
};

static int hva_runtime_suspend(struct device *dev)
{
	struct hva_core *core = dev_get_drvdata(dev);
	BUG_ON(core == NULL);
	dev_dbg(core->dev, "%s\n", __func__);

	/* Disabling IRQ to avoid spurious IRQ */
	disable_irq(core->irq_its);
	disable_irq(core->irq_err);
	dev_dbg(core->dev, "Request power OFF\n");
	regulator_disable(core->regulator);
	core->hw_state = HVA_OK;
	return 0;
}

static int hva_runtime_resume(struct device *dev)
{
	struct hva_core *core = dev_get_drvdata(dev);

	BUG_ON(core == NULL);
	dev_dbg(core->dev, "%s\n", __func__);

	dev_dbg(core->dev, "Request power ON\n");
	regulator_enable(core->regulator);
	enable_irq(core->irq_its);
	enable_irq(core->irq_err);

	return 0;
}

static const struct dev_pm_ops hva_pm_ops = {
	.runtime_suspend = hva_runtime_suspend,
	.runtime_resume = hva_runtime_resume,
};

/*
 * hva_core_probe() - This routine loads the hva core driver
 *
 * @pdev: platform device.
 */
static int __devinit hva_probe(struct platform_device *pdev)
{
	struct resource *iomem;
	struct hva_core *core;
	int ret = 0;
	int minor;

	BUG_ON(pdev == NULL);

	core = devm_kzalloc(&pdev->dev, sizeof(*core), GFP_KERNEL);
	if (!core) {
		dev_err(&pdev->dev, "HVA core alloc failed\n");
		ret = -EINVAL;
		goto hva_bail;
	}

	platform_set_drvdata(pdev, core);
	core->dev = &pdev->dev;
	snprintf(core->name, sizeof(core->name), HVA_DEVNAME);

	core->clk = clk_get(&pdev->dev, "hva");
	if (IS_ERR(core->clk)) {
		ret = PTR_ERR(core->clk);
		dev_err(core->dev, "Unable to get hva clock\n");
		goto hva_bail;
	}

	dev_dbg(core->dev, "Prepare hva clock\n");
	ret = clk_prepare(core->clk);
	if (ret) {
		dev_err(core->dev, "Failed preparing hva clock\n");
		goto hva_prepare_clk_failed;
	}

	core->regulator = regulator_get(&pdev->dev, "v-hva");
	if (IS_ERR(core->regulator)) {
		ret = PTR_ERR(core->regulator);
		dev_err(core->dev, "regulator_get v-hva failed (dev_name=%s)\n",
			dev_name(&pdev->dev));
		goto hva_clk_put;
	}

	/* get a memory region for mmio */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(core->dev, "Reserving mmio region failed\n");
		ret = -ENODEV;
		goto hva_clk_put;
	}

	dev_dbg(core->dev, "Probe hva at address %x size:%d\n",
		iomem->start, iomem->end - iomem->start + 1);
	/* Remap hva registers in kernel space */
	core->hw_base_addr =
	    ioremap_nocache(iomem->start, iomem->end - iomem->start + 1);
	if (!core->hw_base_addr) {
		dev_err(core->dev, "ioremap hva registers region failed\n");
		ret = -ENOMEM;
		goto hva_clk_put;
	}
	core->regs_size = iomem->end - iomem->start + 1;

	/*
	 * Semaphore to ensure that tasks are processed
	 * in FIFO (first ioctl request is first in hardware)
	 * Reuse it to protect task globals
	 */
	sema_init(&core->task_lock, 1);

	/* Lock for hardware registers access */
	spin_lock_init(&core->hw_lock);

	/*
	 * Semaphore lock to avoid submitting more than the
	 * maximal task number (HW_HVA_FIFO_SIZE) to HW
	 */
	sema_init(&core->hw_fifo_lock, HW_HVA_FIFO_SIZE);

	/* Interrupts callback */
	core->hva_hw_cb = hva_hw_callback;
	core->hva_hw_err_cb = hva_hw_err_callback;

	/*
	 * Retrieve irq number from board resources
	 */
	core->irq_its = platform_get_irq(pdev, 0);
	if (core->irq_its <= 0) {
		dev_err(core->dev, "No irq defined\n");
		goto hva_unmap_hw_base_addr;
	}
	ret = request_irq(core->irq_its, hva_irq_its_handler,
			  IRQF_DISABLED | IRQF_SHARED, core->name,
			  (void *)core);
	if (ret) {
		dev_err(core->dev, "Request ITS IRQ 0x%x failed\n",
			core->irq_its);
		goto hva_unmap_hw_base_addr;
	}
	disable_irq(core->irq_its);

	/* HVA IRQ ERR */
	core->irq_err = platform_get_irq(pdev, 1);
	if (core->irq_err <= 0) {
		dev_err(core->dev, "No irq defined\n");
		goto hva_free_its_irq;
	}

	ret = request_irq(core->irq_err, hva_irq_err_handler,
			  IRQF_DISABLED | IRQF_SHARED, core->name,
			  (void *)core);
	if (ret) {
		dev_err(core->dev, "Request ERR IRQ 0x%x failed\n",
			core->irq_err);
		goto hva_free_its_irq;
	}
	disable_irq(core->irq_err);

	core->miscdev.parent = core->dev;
	core->miscdev.minor = MISC_DYNAMIC_MINOR;
	core->miscdev.name = HVA_DEVNAME;
	core->miscdev.fops = &hva_fops;
	minor = misc_register(&core->miscdev);
	if (minor) {
		dev_warn(core->dev, "hva_module_init failed\n");
		goto hva_free_err_irq;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, HVA_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	/* check hardware ID */
	core->chip_id = hva_check_hardware_id(core);
	if (!core->chip_id) {
		dev_err(core->dev, "Unknown chip\n");
		goto hva_free_err_irq;
	}

	/* Debugfs init */
	ret = hva_debug_init(core);
	if (ret < 0) {
		dev_err(core->dev, "hva_debug_init failed\n");
		goto hva_free_err_irq;
	}

	dev_info(&pdev->dev, "Init done.\n");
	return ret;

 hva_free_err_irq:
	free_irq(core->irq_err, 0);
 hva_free_its_irq:
	free_irq(core->irq_its, 0);
 hva_unmap_hw_base_addr:
	if (core->hw_base_addr)
		iounmap(core->hw_base_addr);
	core->hw_base_addr = NULL;
 hva_clk_put:
	clk_unprepare(core->clk);
 hva_prepare_clk_failed:
	clk_put(core->clk);
 hva_bail:
	dev_info(core->dev, "Init done with errors.\n");
	return ret;
}

/*
 * hva_remove() - Module exit function for the hva driver
 */
static int __devexit hva_remove(struct platform_device *pdev)
{
	struct hva_core *core;

	BUG_ON(pdev == NULL);

	core = dev_get_drvdata(&pdev->dev);
	BUG_ON(core == NULL);

	disable_irq(core->irq_its);
	disable_irq(core->irq_err);

	pm_runtime_put_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	free_irq(core->irq_its, 0);
	free_irq(core->irq_err, 0);

	hva_debug_exit(core);
	clk_unprepare(core->clk);

	clk_put(core->clk);
	regulator_put(core->regulator);

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static struct platform_driver platform_hva_driver = {
	.remove = hva_remove,
	.probe = hva_probe,
	.driver = {
		   .name = HVA_DEVNAME,
		   .pm = &hva_pm_ops,
		   },
};

static int __init hva_init(void)
{
	return platform_driver_register(&platform_hva_driver);
}

module_init(hva_init);

static void __exit hva_exit(void)
{
	platform_driver_unregister(&platform_hva_driver);
}

module_exit(hva_exit);

MODULE_AUTHOR("Thomas Costis <thomas.costis@st.com>");
MODULE_AUTHOR("Hugues Fruchet <hugues.fruchet@st.com>");
MODULE_AUTHOR("Benjamin Gaignard <benjamin.gaignard@st.com>");
MODULE_AUTHOR("Jean-Marc Gentit <jean-marc.gentit@st.com>");
MODULE_DESCRIPTION("STE HVA driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
