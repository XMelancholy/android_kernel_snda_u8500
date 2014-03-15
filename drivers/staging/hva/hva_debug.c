/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * ST-Ericsson HVA debug
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <video/hva_regs.h>
#include "hva_internals.h"

static int hva_debug_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int hva_debug_regs_read(struct file *file, char __user *user_buf,
			       size_t size, loff_t *ppos)
{
	struct hva_core *core = (struct hva_core *)file->private_data;
	unsigned int buffer_size = 5000;
	unsigned int len = 0, count;
	char *buf;

	buf = kzalloc(buffer_size, GFP_KERNEL);
	if (!buf) {
		dev_err(core->dev, "Can not allocate memory\n");
		return 0;
	}

	len += snprintf(buf + len, buffer_size - len, "HVA device\n");
	len += snprintf(buf + len, buffer_size - len,
			"%20s %s\n%20s 0x%p\n%20s 0x%x\n%20s 0x%x, 0x%x\n",
			"Device name  :", core->name,
			"Base address :", core->hw_base_addr,
			"Chip ID      :", core->chip_id,
			"HVA irq      :", core->irq_its, core->irq_err);

	if (pm_runtime_get_sync(core->dev) < 0) {
		dev_err(core->dev, "%s pm_runtime_get_sync failed\n",
				__func__);
		goto regulator_enable_failed;
	}

	hva_clock_on(core);
	len += snprintf(buf + len, buffer_size - len,
			"\n%30s\n%23s  %10s %10s\n",
			"HVA registers ", "Register", "Address", "Value");
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_MIF_CFG",
			(void *)HVA_HIF_REG_MIF_CFG,
			readl(core->hw_base_addr + HVA_HIF_REG_MIF_CFG));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HEC_MIF_CFG",
			(void *)HVA_HIF_REG_HEC_MIF_CFG,
			readl(core->hw_base_addr + HVA_HIF_REG_HEC_MIF_CFG));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_CFL",
			(void *)HVA_HIF_REG_CFL,
			readl(core->hw_base_addr + HVA_HIF_REG_CFL));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_SFL",
			(void *)HVA_HIF_REG_SFL,
			readl(core->hw_base_addr + HVA_HIF_REG_SFL));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_LMI_ERR",
			(void *)HVA_HIF_REG_LMI_ERR,
			readl(core->hw_base_addr + HVA_HIF_REG_LMI_ERR));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_EMI_ERR",
			(void *)HVA_HIF_REG_EMI_ERR,
			readl(core->hw_base_addr + HVA_HIF_REG_EMI_ERR));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HEC_MIF_ERR",
			(void *)HVA_HIF_REG_HEC_MIF_ERR,
			readl(core->hw_base_addr + HVA_HIF_REG_HEC_MIF_ERR));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HEC_STS",
			(void *)HVA_HIF_REG_HEC_STS,
			readl(core->hw_base_addr + HVA_HIF_REG_HEC_STS));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HVC_STS",
			(void *)HVA_HIF_REG_HVC_STS,
			readl(core->hw_base_addr + HVA_HIF_REG_HVC_STS));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HJE_STS",
			(void *)HVA_HIF_REG_HJE_STS,
			readl(core->hw_base_addr + HVA_HIF_REG_HJE_STS));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_CNT",
			(void *)HVA_HIF_REG_CNT,
			readl(core->hw_base_addr + HVA_HIF_REG_CNT));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_HEC_CHKSYN_DIS",
			(void *)HVA_HIF_REG_HEC_CHKSYN_DIS,
			readl(core->hw_base_addr + HVA_HIF_REG_HEC_CHKSYN_DIS));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_CLK_GATING",
			(void *)HVA_HIF_REG_CLK_GATING,
			readl(core->hw_base_addr + HVA_HIF_REG_CLK_GATING));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_VERSION",
			(void *)HVA_HIF_REG_VERSION,
			readl(core->hw_base_addr + HVA_HIF_REG_VERSION));
	len += snprintf(buf + len, buffer_size - len,
			"%25s @%p  : Ox%08x\n", "HVA_HIF_REG_BSM",
			(void *)HVA_HIF_REG_BSM,
			readl(core->hw_base_addr + HVA_HIF_REG_BSM));

	hva_clock_off(core);
	pm_runtime_put_autosuspend(core->dev);

	count = simple_read_from_buffer(user_buf, size, ppos, buf, len);
	kfree(buf);

	return count;

regulator_enable_failed:
	kfree(buf);
	return 0;
}

static int hva_debug_stats_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int hva_debug_stats_read(struct file *file, char __user *user_buf,
				size_t size, loff_t *ppos)
{
	struct hva_core *core = (struct hva_core *)file->private_data;
	unsigned int buffer_size = 5000;
	unsigned int len = 0, count;
	struct stats *stats;
	int i;
	char *buf;

	buf = kzalloc(buffer_size, GFP_KERNEL);
	if (!buf) {
		dev_err(core->dev, "Can not allocate memory\n");
		return 0;
	}

	len += snprintf(buf + len, buffer_size - len,
			"hva_core:\n\thw_state:         %u\n",
			core->hw_state);

	for (i = 0; i < MAX_NB_DEBUGFS_INSTANCES; i++) {
		stats = &core->stats[i];
		len += snprintf(buf + len, buffer_size - len,
				"\nInstance: 0x%x\n", stats->tag);
		len += snprintf(buf + len, buffer_size - len,
				"%2sNb tasks done %d\n", "",
				stats->total_task_nb);
		if (stats->sum_duration > 0)
			len += snprintf(buf + len, buffer_size - len,
					"%2sTotal elapsed time: %lu ms -> Tasks/sec: %lu\n",
					"", stats->sum_duration / 1000,
					(1000000 * stats->total_task_nb) /
					stats->sum_duration);
		len += snprintf(buf + len, buffer_size - len,
				"%2sLast task start: %lu stop: %lu\n",
				"", stats->last_task_hw_start_ts,
				stats->last_task_hw_stop_ts);
		len += snprintf(buf + len, buffer_size - len,
				"%2sTask min duration %lu us\n"
				"%2s     max duration %lu us\n",
				"", stats->min_duration,
				"", stats->max_duration);
		if (stats->total_task_nb > 0)
			len += snprintf(buf + len, buffer_size - len,
					"%2s     avg duration %lu us\n"
					"%2sHW duration %lu ms\n",
					"", stats->sum_duration /
					stats->total_task_nb,
					"", stats->sum_duration / 1000);
		len += snprintf(buf + len, buffer_size - len,
				"%2sSession duration %lu ms\n",
				"", stats->session_duration / 1000);
		if (stats->session_duration > 0)
			len += snprintf(buf + len, buffer_size - len,
					"%2sLoad on device %lu %%\n",
					"", (100 * stats->sum_duration) /
					stats->session_duration);
	}

	count = simple_read_from_buffer(user_buf, size, ppos, buf, len);
	kfree(buf);
	return count;
}

static const struct file_operations regs_fops = {
	.open = hva_debug_regs_open,
	.read = hva_debug_regs_read,
	.owner = THIS_MODULE,
};

static const struct file_operations stats_fops = {
	.open = hva_debug_stats_open,
	.read = hva_debug_stats_read,
	.owner = THIS_MODULE,
};

int hva_debug_init(struct hva_core *core)
{
	core->debugfs_root_dir = debugfs_create_dir("hva", NULL);
	if (!core->debugfs_root_dir) {
		dev_warn(core->dev, "%s: could not create hva dir in debugfs\n",
			 __func__);
		return -ENODEV;
	}

	(void)debugfs_create_file("regs", 0444, core->debugfs_root_dir, core,
				  &regs_fops);
	(void)debugfs_create_file("stats", 0444, core->debugfs_root_dir, core,
				  &stats_fops);

	return 0;
}

void hva_debug_exit(struct hva_core *core)
{
	if (core->debugfs_root_dir) {
		debugfs_remove_recursive(core->debugfs_root_dir);
		core->debugfs_root_dir = NULL;
	}
}
