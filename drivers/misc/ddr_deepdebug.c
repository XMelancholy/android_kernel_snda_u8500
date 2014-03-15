/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 *
 * Authors: David Paris <david.paris@stericsson.com>
 *	    Yves Magnaud <yves.magnaud@stericsson.com>
 *
 * Add specific debufs for deepdebug
 * Allows to set 100% cpu with continuous ddr test
 * Use in hats running mode
 *
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/ux500_deepdebug.h>
#include <linux/uaccess.h>

#define X500_DDR_NAME_STRING "ddr"

static u8 memtest_val;
static u8 *ext_ram_addr;
static struct workqueue_struct *memorystress_wq;
static struct work_struct memstress_handler;
static u8 memorytest_start;

static void memorystress_work(struct work_struct *work)
{
	u8 data_read;
	int i;

	if (ext_ram_addr) {
		printk(KERN_INFO "Memory stress started\n");
		while (memorytest_start) {
			for (i = 0; i < 1024; i++)
				ext_ram_addr[i] = (u8)i;

			for (i = 0; i < 1024; i++) {
				data_read = ext_ram_addr[i];
				if (data_read != (u8)i)
					printk(KERN_ERR
						"Memory stress failed, data_read=%d, i=%d\n",
						data_read, (u8)i);
			}
		}
	}
}

int ddr_stress_memory_access_state(bool on_off)
{
	static dma_addr_t device_ptr;

	if (on_off) {
		if (memorytest_start != 1) {
			/* create thread for work */
			memorystress_wq = create_workqueue("memstress_wq");

			if (memorystress_wq == NULL)
				return -ENOMEM;

			ext_ram_addr = dma_alloc_coherent(NULL,
					256 * sizeof(uint32_t), &device_ptr,
					GFP_KERNEL | GFP_DMA);

			if (ext_ram_addr) {
				memorytest_start = 1;
				queue_work(memorystress_wq, &memstress_handler);
			}
		}

	} else {

		if (memorytest_start != 0) {
			memorytest_start = 0;
			destroy_workqueue(memorystress_wq);

			dma_free_coherent(NULL, 256 * sizeof(uint32_t),
					ext_ram_addr, device_ptr);
			printk("Memory stress stopped\n");
		}
	}

	return memorytest_start;
}

static ssize_t x500_ddr_memtest_store(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char var[7];

	if (count > 6)
		return -EFAULT;

	if (copy_from_user(var, user_buf, count))
		return -EFAULT;

	var[count] = '\0';

	/* To launch memory stress tests */
	if (strcmp(var, "start\n") == 0)
		memtest_val = ddr_stress_memory_access_state(1);
	else if (strcmp(var, "stop\n") == 0)
		memtest_val = ddr_stress_memory_access_state(0);

	return count;
}

static ssize_t memtest_show(struct seq_file *s, void *p)
{
	if (memtest_val == 0)
		return seq_printf(s, "Memory test stopped\n");
	else
		return seq_printf(s, "Memory test is running\n");
}

static int x500_ddr_memtest_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, memtest_show, inode->i_private);
}

static const struct file_operations x500_ddr_memtest_fops = {
	.open = x500_ddr_memtest_open,
	.read = seq_read,
	.write = x500_ddr_memtest_store,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static struct dentry *ddr_dir;

int x500_ddr_deepdebug_probe(struct ddbg_service *data,
		struct dentry *parent)
{
	struct dentry *file;
	int ret = -ENOMEM;

	ddr_dir = debugfs_create_dir(X500_DDR_NAME_STRING,
		parent);
	if (!ddr_dir)
		goto err;

	file = debugfs_create_file("memtest", (S_IRUGO | S_IWUGO),
		ddr_dir, data, &x500_ddr_memtest_fops);
	if (!file)
		goto err;

	return 0;

err:
	if (ddr_dir)
		debugfs_remove_recursive(ddr_dir);
	pr_err("failed to create debugfs entries.\n");

	return ret;
}

static struct ddbg_service x500_ddr_ddbg_services = {
	.name = X500_DDR_NAME_STRING,
	.probe = x500_ddr_deepdebug_probe,
};

/*
 * Initialization
 */

static int __init ddr_deepdebug_init(void)
{
	INIT_WORK(&memstress_handler, memorystress_work);
	deep_debug_service_access_register(&x500_ddr_ddbg_services);
	return 0;
}

device_initcall(ddr_deepdebug_init);
