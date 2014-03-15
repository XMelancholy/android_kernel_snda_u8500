/*
 * Copyright (C) ST-Ericsson SA 2011
 * Author: Julien Delacou <julien.delacou@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/pasr.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <mach/pasr.h>

#define INFO_SZ    64

static char area[INFO_SZ];
module_param_string(area, area, INFO_SZ, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(area, "Memory area to test PASR");

struct pasr_dbg {
	unsigned int nr_sections;
	unsigned long area_size;
	phys_addr_t area_start;
	phys_addr_t *addr;
	u8 mask;
	struct dentry *pasr_dir;
};

static struct pasr_dbg *pasr_dbg;

static int pasr_parse_area_param(char *p)
{
	pasr_dbg->area_size = memparse(p, &p);

	if (*p != '@')
		return -EINVAL;

	pasr_dbg->area_start = memparse(p + 1, &p);

	pr_info("%s(): area start:%#x\n",
			__func__, pasr_dbg->area_start);

	/* check arguments consistency */
	if ((pasr_dbg->area_size % 8) != 0
	 || (pasr_dbg->area_start == 0))
		return -EINVAL;

	/* compute number of sections available in the given area */
	pasr_dbg->nr_sections = pasr_dbg->area_size / PASR_SECTION_SZ;
	if (pasr_dbg->nr_sections == 0
	 || pasr_dbg->nr_sections > PASR_MAX_SECTION_NR_PER_DIE)
		return -EINVAL;

	pr_debug("%s(): %d sections found\n", __func__, pasr_dbg->nr_sections);

	return 0;
}

static int pasr_init_addr(void)
{
	int i;

	pasr_dbg->addr = kzalloc(sizeof(pasr_dbg->addr), GFP_KERNEL);
	if (!pasr_dbg->addr)
		return -ENOMEM;

	for (i = 0; i < pasr_dbg->nr_sections; i++)
		pasr_dbg->addr[i] = pasr_dbg->area_start +
			(i * PASR_SECTION_SZ);

	return 0;
}

static void pasr_apply_mask(u8 mask)
{
	int i;
	int b1, b2;

	/* lookup for each updated bit... */
	for (i = 0; i < pasr_dbg->nr_sections; i++) {
		b1 = (mask & (1 << i)) >> i;
		b2 = (pasr_dbg->mask & (1 << i)) >> i;
		if (b1 != b2) {
			/* ...and apply pasr accordingly */
			if (b1 == PASR_NO_REFRESH)
				pasr_put(pasr_dbg->addr[i], PASR_SECTION_SZ);
			else
				pasr_get(pasr_dbg->addr[i], PASR_SECTION_SZ);
		}
	}

	pasr_dbg->mask = mask;
}

static ssize_t pasr_dbg_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int err;
	u8 mask;

	err = kstrtou8_from_user(user_buf, count, 0, &mask);
	if (err)
		return -EINVAL;

	if (!err && (pasr_dbg->mask^mask))
		pasr_apply_mask(mask);

	return count;
}

static int pasr_dbg_read(struct seq_file *s, void *d)
{
	int i;
	char *buf;
	char *p;

	buf = kzalloc(INFO_SZ * pasr_dbg->nr_sections, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	p = buf;

	for (i = 0; i < pasr_dbg->nr_sections; i++) {
		p += snprintf(p, INFO_SZ, "section %d: %s\n"
				, i
				, ((pasr_dbg->mask & (1 << i)) >> i) ==
				PASR_REFRESH ? "refresh" : "no refresh");
		strncat(buf, p, sizeof(buf));
	}

	i = seq_printf(s, "%#x\n%s", pasr_dbg->mask, buf);
	kfree(buf);

	return i;
}

static int pasr_dbg_open(struct inode *inode,
						 struct file *file)
{
	return single_open(file, pasr_dbg_read, inode->i_private);
}

static const struct file_operations pasr_dbg_fops = {
	.open		= pasr_dbg_open,
	.write		= pasr_dbg_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int __init pasr_init_dbg(void)
{
	int err = 0;

	pasr_dbg = kzalloc(sizeof(struct pasr_dbg), GFP_KERNEL);
	if (!pasr_dbg)
		return -ENOMEM;

	err = pasr_parse_area_param(area);
	if (err)
		goto fail_1;

	err = pasr_init_addr();
	if (err)
		goto fail_1;

	pasr_dbg->pasr_dir = debugfs_create_dir("pasr", NULL);
	if (!pasr_dbg->pasr_dir)
		goto fail_2;

	if (IS_ERR_OR_NULL(debugfs_create_file("mask",
					S_IWUGO | S_IRUGO, pasr_dbg->pasr_dir,
					(void *) &pasr_dbg->mask,
					&pasr_dbg_fops)))
		goto fail_2;

	/* default value, all refreshed */
	pasr_dbg->mask = 0x00;

	return 0;
fail_2:
	pr_debug("%s(): failed to create debugfs (%d)\n", __func__, err);
	debugfs_remove_recursive(pasr_dbg->pasr_dir);
	kfree(pasr_dbg->addr);
fail_1:
	pr_debug("%s(): failed to initialize pasr_dbg (%d)\n", __func__, err);
	kfree(pasr_dbg);
	return err;
}
module_init(pasr_init_dbg);

static void __exit pasr_exit_dbg(void)
{
	debugfs_remove_recursive(pasr_dbg->pasr_dir);
	kfree(pasr_dbg->addr);
	kfree(pasr_dbg);
}
module_exit(pasr_exit_dbg);


