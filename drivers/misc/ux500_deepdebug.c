/*
 * Copyright (C) ST-Ericssson SA 2012
 *
 * Deep Debug Core feature
 * This debug framework is used by HATS to access
 * peripheral registers and driver debug specific services
 * Each driver could register:
 *   - list of HW registers
 *   - list of debug services
 * This framework exposes them through debugfs interface
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <linux/ux500_deepdebug.h>

static struct dentry *deep_debug_dir;

/*
 * Registers access management
 */
static int deep_debug_registers_print(struct seq_file *s, void *p)
{
	struct ddbg_target *ddbg = (void *) s->private;
	int count = 0;
	int ret, i;

	if (ddbg == NULL) {
		pr_err("deep debug: invalid ddbg_target structure\n");
		ret = -EFAULT;
		goto out;
	}

	for (i = 0; i < ddbg->nb_regs; i++) {
		ret = seq_printf(s, "%s:0x%04X:%i\n", ddbg->reg[i].name,
			ddbg->phyaddr + ddbg->reg[i].offset, ddbg->reg[i].perm);
		if (ret < 0)
			goto out;
		else
			count += ret;
	}

	return count;

out:
	return ret;
}

static int deep_debug_registers_open(struct inode *inode, struct file *file)
{
	return single_open(file, deep_debug_registers_print, inode->i_private);
}

static const struct file_operations deep_debug_registers_fops = {
	.open = deep_debug_registers_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int deep_debug_select_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ddbg_target *ddbg;
	int ret, i;
	unsigned long user_val;

	ddbg = ((struct seq_file *)(file->private_data))->private;
	if (ddbg == NULL) {
		pr_err("deep debug: invalid ddbg_target structure\n");
		ret = -EFAULT;
		goto out;
	}
	ddbg->curr = NULL;

	ret = kstrtoul_from_user(user_buf, count, 0, &user_val);
	if (ret)
		goto out;

	for (i = 0; i < ddbg->nb_regs; i++) {
		if (user_val == ddbg->phyaddr + ddbg->reg[i].offset)
			break;
	}

	if (i < ddbg->nb_regs)
		ddbg->curr = &ddbg->reg[i];
	else {
		pr_err("deep debug: invalid register address\n");
		ret = -EINVAL;
		goto out;
	}

	return count;

out:
	return ret;
}

static int deep_debug_select_print(struct seq_file *s, void *p)
{
	struct ddbg_target *ddbg = (void *) s->private;
	int ret;

	if ((ddbg == NULL) || (ddbg->curr == NULL)) {
		pr_err("deep debug: no selected register\n");
		ret = -EFAULT;
	} else {
		ret = seq_printf(s, "%s:0x%04X:%i\n", ddbg->curr->name,
			ddbg->phyaddr + ddbg->curr->offset, ddbg->curr->perm);
	}

	return ret;
}

static int deep_debug_select_open(struct inode *inode, struct file *file)
{
	return single_open(file, deep_debug_select_print, inode->i_private);
}

static const struct file_operations deep_debug_select_fops = {
	.open = deep_debug_select_open,
	.write = deep_debug_select_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int deep_debug_value_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct ddbg_target *ddbg;
	int ret;
	unsigned long user_val;

	ddbg = ((struct seq_file *)(file->private_data))->private;
	if ((ddbg == NULL) || (ddbg->curr == NULL)) {
		pr_err("deep debug: no selected register\n");
		ret = -EFAULT;
		goto out;
	}

	ret = kstrtoul_from_user(user_buf, count, 0, &user_val);
	if (ret)
		goto out;

	if (ddbg->curr->perm != DDBG_RW) {
		pr_err("deep debug: read-only register\n");
		ret = -EINVAL;
		goto out;
	}

	ret = ddbg->write_reg(ddbg, user_val);
	if (ret) {
		pr_err("deep debug: can't target write register\n");
		goto out;
	}

	return count;

out:
	return ret;
}

static int deep_debug_value_print(struct seq_file *s, void *p)
{
	struct ddbg_target *ddbg = (void *) s->private;
	u32 val;
	int ret, count;

	if ((ddbg == NULL) || (ddbg->curr == NULL)) {
		pr_err("deep debug: no selected register\n");
		ret = -EFAULT;
		goto out;
	}

	ret = ddbg->read_reg(ddbg, &val);
	if (ret) {
		pr_err("deep debug: can't target read register\n");
		goto out;
	}

	count = seq_printf(s, "0x%08X\n", val);

	return count;

out:
	return ret;
}

static int deep_debug_value_open(struct inode *inode, struct file *file)
{
	return single_open(file, deep_debug_value_print, inode->i_private);
}

static const struct file_operations deep_debug_value_fops = {
	.open = deep_debug_value_open,
	.write = deep_debug_value_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

int deep_debug_regaccess_register(struct ddbg_target *ddbg)
{
	int i;
	int ret = 0;

	if (!deep_debug_dir) {
		ret = -ENODEV;
		goto out;
	}

	if ((ddbg->reg == NULL) || (ddbg->name == NULL) ||
		(ddbg->read_reg == NULL) || (ddbg->write_reg == NULL)) {
			pr_err("deep debug: invalid ddbg_target structure\n");
			ret = -EINVAL;
			goto out;
	}

	ddbg->curr = NULL;
	/* only 0x10000 registers supported */
	i = 0;
	while (ddbg->reg[i].name) {
		if (++i > DDBG_MAX_REG)
			break;
	}
	BUG_ON(i > DDBG_MAX_REG);

	ddbg->nb_regs = i;

	ddbg->dir = debugfs_create_dir(ddbg->name, deep_debug_dir);
	if (ddbg->dir == NULL) {
		pr_err("deep debug: failed to create ddbg folder.\n");
		ret = -ENXIO;
		goto out;
	}

	ddbg->reg_file = debugfs_create_file("registers", S_IRUGO,
				ddbg->dir, ddbg, &deep_debug_registers_fops);
	ddbg->sel_file = debugfs_create_file("select", S_IRUGO | S_IWUSR,
				ddbg->dir, ddbg, &deep_debug_select_fops);
	ddbg->val_file = debugfs_create_file("value", S_IRUGO | S_IWUSR,
				ddbg->dir, ddbg, &deep_debug_value_fops);
	if ((ddbg->reg_file == NULL) || (ddbg->val_file == NULL) ||
			(ddbg->sel_file == NULL)) {
		debugfs_remove_recursive(ddbg->dir);
		pr_err("deep debug: failed to create ddbg entries.\n");
		ret = -ENXIO;
		goto out;
	}

out:
	return ret;
}
EXPORT_SYMBOL(deep_debug_regaccess_register);

int deep_debug_regaccess_unregister(struct ddbg_target *ddbg)
{
	int ret = 0;

	if (ddbg->reg == NULL) {
		pr_err("deep debug: Invalid ddbg_target structure\n");
		ret = -EINVAL;
		goto out;
	}

	if (ddbg->dir)
		debugfs_remove_recursive(ddbg->dir);

out:
	return ret;
}
EXPORT_SYMBOL(deep_debug_regaccess_unregister);

/*
 * Services access management
 */

int deep_debug_service_access_register(struct ddbg_service *ddbg)
{
	int ret = 0;

	if (!deep_debug_dir) {
		ret = -ENODEV;
		goto out;
	}

	if (!ddbg->probe) {
		pr_err("deep debug: invalid ddbg_target structure\n");
		ret = -EINVAL;
		goto out;
	}

	/* call probe function to register service debugfs */
	ddbg->probe(ddbg, deep_debug_dir);

out:
	return ret;
}
EXPORT_SYMBOL(deep_debug_service_access_register);

int deep_debug_service_access_unregister(struct ddbg_service *ddbg)
{

	return 0;
}
EXPORT_SYMBOL(deep_debug_service_access_unregister);

/*
 * deep debug module init/cleanup
 */
static int __init deep_debug_init(void)
{
	int ret = 0;
	deep_debug_dir = debugfs_create_dir("deep_debug", NULL);
	if (!deep_debug_dir) {
		pr_err("deep debug: failed to create deep_debug dir\n");
		ret = -ENXIO;
		goto out;
	}

out:
	return ret;
}
arch_initcall(deep_debug_init);
