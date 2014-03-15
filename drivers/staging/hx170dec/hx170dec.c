/*
 * hx170dec.c
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author: <benjamin.gaignard@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>

/* power and clock management */
#include <linux/regulator/driver.h>
#include <linux/clk.h>
#include <linux/mfd/dbx500-prcmu.h>

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/time.h>

#include <../drivers/mfd/dbx540-prcmu-regs.h>
#include "hx170dec.h"


/* define irq registers offset and masks */
#define HX170_DEC_IRQ_OFFSET	(1*4)
#define HX170_PP_IRQ_OFFSET	(60*4)
#define HX170_DEC_IRQ_MASK	0x100
#define HX170_PP_IRQ_MASK	0x100
#define HX170_PP_PIPELINE_E_MASK	0x2

/* macros for register ID */
#define PRODUCT_NUMBER(x) ((x >> 16) & 0xFFFF)
#define PRODUCT_MAJOR(x)  ((x >> 12) & 0x000F)
#define PRODUCT_MINOR(x)  ((x >> 4)  & 0x00FF)
#define PRODUCT_TYPE(x)   ((x >> 3)  & 0x0001)
#define PRODUCT_BUILD(x)  ((x >> 0)  & 0x0003)

#define DEVICE_NAME "g1"
#define CLOCK_NAME  "g1"
/*
 * struct hx170_chip
 * @dev: The device
 * @regs: registers base address
 * @regs_size: registers memory size
 * @irq: the interrupt source number
 * @chip_id: the chip ID
 * @protect_mutex: a mutex protection for multi-instance
 * @pp_interrupt: a completion signal to wait for post proc interrupt
 * @dec_interrupt: a completion signal to wait for decoder interrupt
 * @clk: device clock
 * @debug_dir: debugfs directory
 * @dump_regs: debugfs file to dump registers
 * @dump_stats: debugfs file to dump stats
 */

#define G1_AUTOSUSPEND_DELAY_MS     3
#define PERF_CIRCULAR_ARRAY_SIZE    10
struct hx170_perf {
	unsigned long max;
	unsigned long min;
	unsigned long sum;
	unsigned long n;
	unsigned long circular_idx;
	unsigned long duration[PERF_CIRCULAR_ARRAY_SIZE];
};

struct hx170_chip {
	struct device *dev;
	struct miscdevice miscdev;
	void __iomem *regs;
	int regs_size;
	int irq;
	unsigned long int chip_id;
	struct mutex protect_mutex;
	struct completion pp_interrupt;
	struct completion dec_interrupt;
	struct clk *clk;
	struct regulator *regulator;
	/* debugfs */
	struct dentry *debug_dir;
	struct dentry *dump_regs;
	struct dentry *dump_stats;
	/* profiling */
	struct hx170_perf *perf;
};

/*
 * power and clock management functions
 */
static void hx170_dec_power_on(struct hx170_chip *hx170)
{
	dev_dbg(hx170->dev, "request power ON\n");
	regulator_enable(hx170->regulator);
}

static void hx170_dec_power_off(struct hx170_chip *hx170)
{
	dev_dbg(hx170->dev, "request power OFF\n");
	regulator_disable(hx170->regulator);
}

static void hx170_dec_clock_on(struct hx170_chip *hx170)
{
	dev_dbg(hx170->dev, "request clock ON\n");
	clk_enable(hx170->clk);
}

static void hx170_dec_clock_off(struct hx170_chip *hx170)
{
	dev_dbg(hx170->dev, "request clock OFF\n");
	clk_disable(hx170->clk);
}

/*
 * debugfs file operation functions
 */
static int hx170_dec_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static void hx170_performance_init(struct hx170_perf *perf)
{
	perf->min = (unsigned long) -1;
	perf->max = 0;
	perf->sum = 0;
	perf->n = 0;
	perf->circular_idx = 0;
}

static void hx170_performance_record(struct hx170_perf *perf,
					unsigned long duration)
{
	if (duration < perf->min)
		perf->min = duration;
	if (duration > perf->max)
		perf->max = duration;
	perf->sum += duration;
	perf->n++;
	perf->duration[perf->circular_idx++] = duration;
	if (perf->circular_idx == PERF_CIRCULAR_ARRAY_SIZE)
		perf->circular_idx = 0;
}

static ssize_t hx170_dec_debugfs_registers_dump(struct file *file,
						char __user *user_buf,
						size_t size, loff_t *ppos)
{
	struct hx170_chip *hx170 = file->private_data;
	unsigned long int *regs;
	char *buf;
	unsigned int buffer_size =
	    sizeof("12345678 swreg100: 0x12345678\n") * 102;
	unsigned int len = 0, count = 0;
	int i;

	regs = kzalloc(hx170->regs_size, GFP_KERNEL);
	if (!regs) {
		dev_err(hx170->dev, "can't allocate memory\n");
		return 0;
	}

	buf = kzalloc(buffer_size, GFP_KERNEL);
	if (!buf) {
		kfree(regs);
		dev_err(hx170->dev, "can't allocate memory\n");
		return 0;
	}

	/* before dump registers make sure that power and clock are on */
	hx170_dec_power_on(hx170);
	hx170_dec_clock_on(hx170);
	mutex_lock(&hx170->protect_mutex);
	/* dump registers */
	for (i = 0; i < hx170->regs_size; i += 4)
		regs[i / 4] = ioread32(hx170->regs + i);

	mutex_unlock(&hx170->protect_mutex);
	/* release clock and power */
	hx170_dec_clock_off(hx170);
	hx170_dec_power_off(hx170);

	/* write them in humain readable way */
	for (i = 0; i < hx170->regs_size; i += 4) {
		len +=
		    snprintf(buf + len, buffer_size - len,
			     "%08x swreg%03d: 0x%08lx\n", i, i / 4,
			     regs[i / 4]);
		dev_dbg(hx170->dev, "swreg%03d: 0x%08lx\n",
			i / 4, regs[i / 4]);
	}

	count = simple_read_from_buffer(user_buf, size, ppos, buf, len);
	kfree(regs);
	kfree(buf);

	return count;
}

static ssize_t hx170_dec_debugfs_stats_dump(struct file *file,
						char __user *user_buf,
						size_t size, loff_t *ppos)
{
	struct hx170_chip *hx170 = file->private_data;
	struct hx170_perf *perf = hx170->perf;
	char *buf;
	unsigned int buffer_size =
	    sizeof("Measured on 123456789 samples\n")*4 +
	    sizeof("  Duration[1234]=0123456789 us\n")*PERF_CIRCULAR_ARRAY_SIZE;
	unsigned int len = 0, count = 0;

	buf = kzalloc(buffer_size, GFP_KERNEL);
	if (!buf) {
		dev_err(hx170->dev, "can't allocate memory\n");
		return 0;
	}

	len += snprintf(buf + len, buffer_size - len,
		"Measured on %4ld samples\n",
		perf->n);
	if (perf->n) {

		int	i;
		int	absidx = perf->n-1;
		int	idx = perf->circular_idx-1;

		len += snprintf(buf + len, buffer_size - len,
			"Min duration %lu us\n", perf->min);
		len += snprintf(buf + len, buffer_size - len,
			"Max duration %ld us\n", perf->max);
		len += snprintf(buf + len, buffer_size - len,
			"Avg duration %ld us\n", perf->sum/perf->n);

		for (i = 0; i < PERF_CIRCULAR_ARRAY_SIZE; i++) {
			len += snprintf(buf + len, buffer_size - len,
				"  Duration[%4d]=%ld us\n",
				absidx, perf->duration[idx]);
			absidx--;
			idx--;
			if (idx < 0)
				idx = PERF_CIRCULAR_ARRAY_SIZE-1;
			if (absidx < 0)
				break;
		}
	}

	count = simple_read_from_buffer(user_buf, size, ppos, buf, len);
	kfree(buf);

	hx170_performance_init(hx170->perf);

	return count;
}

static const struct file_operations hx170_dec_debugfs_fops = {
	.open = hx170_dec_debugfs_open,
	.read = hx170_dec_debugfs_registers_dump,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static const struct file_operations hx170_dec_stats_fops = {
	.open = hx170_dec_debugfs_open,
	.read = hx170_dec_debugfs_stats_dump,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static void hx170_dec_opp_on(struct file *filp)
{
	char	opp_dev_name[64];
	struct hx170_chip *hx170 = container_of(filp->private_data,
			struct hx170_chip, miscdev);

	snprintf(opp_dev_name, 64, DEVICE_NAME"_%p_", filp);
	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
		opp_dev_name, 100);
	prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP,
		opp_dev_name, 100);
	dev_dbg(hx170->dev, "add opp requirement for %s\n",
		opp_dev_name);

}

static void hx170_dec_opp_off(struct file *filp)
{
	char	opp_dev_name[64];

	struct hx170_chip *hx170 = container_of(filp->private_data,
			struct hx170_chip, miscdev);


	snprintf(opp_dev_name, 64, DEVICE_NAME"_%p_", filp);

	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP,
		opp_dev_name);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP,
		opp_dev_name);
	dev_dbg(hx170->dev, "removed opp requirement for %s\n",
		opp_dev_name);
}

/*
 * file operations functions
 */
static int hx170_dec_open(struct inode *inode, struct file *filp)
{
	struct hx170_chip *hx170 = container_of(filp->private_data,
			struct hx170_chip, miscdev);
	dev_dbg(hx170->dev, "open\n");
	hx170_dec_opp_on(filp);
	return 0;
}

static int hx170_dec_release(struct inode *inode, struct file *filp)
{

	struct hx170_chip *hx170 = container_of(filp->private_data,
			struct hx170_chip, miscdev);
	dev_dbg(hx170->dev, "close\n");
	hx170_dec_opp_off(filp);
	return 0;
}

/*
 * hx170_dec_read_only_register
 * @reg_offset : the offset of reg to check
 * return true if the register is a read only register
 */
static bool hx170_dec_read_only_register(int reg_offset)
{
	switch (reg_offset) {
	case 0x000:
	case 0x0C8:
	case 0x0D0:
	case 0x0D4:
	case 0x0D8:
	case 0x0E0:
	case 0x0E4:
	case 0x0E8:
	case 0x12C:
	case 0x130:
	case 0x134:
	case 0x138:
		return true;
	}
	if (reg_offset >= 0x17C)
		return true;

	return false;
}

static int hx170_ioctl_get_regsize(struct hx170_chip *hx170, unsigned long arg)
{
	u32 *p = (void *)arg;
	*p = hx170->regs_size;
	return 0;
}

static int hx170_ioctl_decode(struct hx170_chip *hx170, unsigned long arg)
{
	int	i;
	int	ret = 0;
	int	pending_pp = 0;
	int	pending_dec = 0;
	struct timeval start_time, stop_time;

	struct hx170_reglist_t reglist;

	if (copy_from_user((unsigned char *)&reglist, (unsigned char *)arg,
			   sizeof(struct hx170_reglist_t))) {
		dev_err(hx170->dev, "copy from user failed");
		return  -EFAULT;
	}

	/* enable power */
	if (pm_runtime_get_sync(hx170->dev) < 0) {
		dev_err(hx170->dev, "%s pm_runtime_get_sync failed\n",
				__func__);
		return -EFAULT;
	}

	/* enable clock */
	hx170_dec_clock_on(hx170);

	/* get exclusive access to the hardware */
	mutex_lock(&hx170->protect_mutex);

	/* enabling Irq */
	enable_irq(hx170->irq);

	dev_dbg(hx170->dev, "decode: %d reg to write\n", reglist.n);

	/* write data into hx170 registers */
	for (i = 0; i < reglist.n; i++) {
		/* do not write read-only registers */
		if (!hx170_dec_read_only_register((reglist.offset[i]))) {

			/* detect start of pp and wait for it  */
			if ((reglist.offset[i] == HX170_PP_IRQ_OFFSET)
				&& (reglist.data[i] & 1)) {
				pending_pp = 1;
				do_gettimeofday(&start_time);
				dev_dbg(hx170->dev, "fired pp\n");
			}
			/* detect start of decoder and wait for it  */
			if ((reglist.offset[i] == HX170_DEC_IRQ_OFFSET) &&
				(reglist.data[i] & 1)) {
				pending_dec = 1;
				do_gettimeofday(&start_time);
				dev_dbg(hx170->dev, "fired dec\n");
			}
			iowrite32(reglist.data[i],
			hx170->regs + reglist.offset[i]);
			/*dev_dbg(hx170->dev, "ioctl write reg[%d] : %d\n"
			,reglist.offset[i]/4,reglist.data[i]);*/
		}
	}

	/* now Here we wait for HW completion if needed */
	if (pending_pp && !wait_for_completion_timeout
		    (&hx170->pp_interrupt, msecs_to_jiffies(2000))) {
		dev_warn(hx170->dev, "post proc timeout\n");
	}

	if (pending_dec && !wait_for_completion_timeout
		    (&hx170->dec_interrupt, msecs_to_jiffies(2000))) {
		dev_warn(hx170->dev, "decoder timeout\n");
	}

	/* to get performance information */
	if (pending_dec || pending_pp) {
		long	duration;
		do_gettimeofday(&stop_time);
		duration = stop_time.tv_usec-start_time.tv_usec;
		if (duration < 0)
			duration = 1000000+duration;
		hx170_performance_record(hx170->perf,
			(unsigned long) duration);
	}

	/* read back all register */

	for (i = 0; i < hx170->regs_size/4; i++) {
		reglist.data[i] = ioread32(hx170->regs + 4*i);
		reglist.offset[i] = 4*i;

		if (i  == HX170_DEC_IRQ_OFFSET/4) {
			dev_dbg(hx170->dev,
			"get decoder control reg[%d] : %x\n",
			i, reglist.data[i]);
		}
		if (i == HX170_PP_IRQ_OFFSET/4) {
			/* turn pipeline OFF so it doesn't interfer
			 * with other instances
			 */
			reglist.data[i] &= ~HX170_PP_PIPELINE_E_MASK;
			iowrite32(reglist.data[i],
				  (hx170->regs + HX170_PP_IRQ_OFFSET));
			dev_dbg(hx170->dev,
			"get postproc control reg[%d] : %x\n",
			i, reglist.data[i]);
		}

		/*	dev_dbg(hx170->dev, "ioctl read reg[%d] : %x\n",
		reglist.offset[i] / 4, reglist.data[i]);*/
	}

	if (copy_to_user((void *)arg, &reglist,
				sizeof(struct hx170_reglist_t))) {
		dev_dbg(hx170->dev, "copy_to_user failed");
		ret = -EFAULT;
	}

	/* disable irq */
	disable_irq(hx170->irq);

	/* release hardware access */
	mutex_unlock(&hx170->protect_mutex);

	/* disable clock */
	hx170_dec_clock_off(hx170);

	/* disable power */
	pm_runtime_mark_last_busy(hx170->dev);
	pm_runtime_put_autosuspend(hx170->dev);

	return ret;
}

static long hx170_dec_ioctl(struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct hx170_chip *hx170 = container_of(file->private_data,
			struct hx170_chip, miscdev);
	int ret;

	switch (cmd) {

	case HX170_IOCTL_GET_REGSIZE:
		ret = hx170_ioctl_get_regsize(hx170, arg);
		break;

	case HX170_IOCTL_DECODE:
		ret = hx170_ioctl_decode(hx170, arg);
		break;

	default:
		dev_dbg(hx170->dev, "Wrong IOCTL command\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations hx170_dec_fops = {
	.open = hx170_dec_open,
	.release = hx170_dec_release,
	.unlocked_ioctl = hx170_dec_ioctl,
	.owner = THIS_MODULE,
};

/*
 * hx170_interrupt - Interrupt handler
 */
static irqreturn_t hx170_dec_interrupt(int irq, void *dev)
{
	struct hx170_chip *hx170 = dev;
	unsigned int irq_dec;
	unsigned int irq_pp;

	/* there is two possible irq source decoder and post-processor */
	irq_dec = ioread32(hx170->regs + HX170_DEC_IRQ_OFFSET);
	irq_pp = ioread32(hx170->regs + HX170_PP_IRQ_OFFSET);

	if (irq_dec & HX170_DEC_IRQ_MASK) {
		/* clear decoder irq */
		iowrite32(irq_dec & (~HX170_DEC_IRQ_MASK),
			  (hx170->regs + HX170_DEC_IRQ_OFFSET));
		complete(&hx170->dec_interrupt);
		dev_dbg(hx170->dev, "get an interrupt from decoder\n");
	}

	if (irq_pp & HX170_PP_IRQ_MASK) {
		/* clear post-processor irq */
		iowrite32(irq_pp & (~HX170_PP_IRQ_MASK),
			  (hx170->regs + HX170_PP_IRQ_OFFSET));
		complete(&hx170->pp_interrupt);
		dev_dbg(hx170->dev, "get an interrupt from post-processor\n");

	}

	return IRQ_HANDLED;
}

/*
 * check hardware ID
 * return the ID if the value is correct
 * else return 0
 */
static unsigned long int hx170_dec_check_hardware_id(struct hx170_chip *hx170)
{
	unsigned long int id;
	unsigned long ret = 0;

	/* enable power and clock before read register ID */
	hx170_dec_power_on(hx170);
	hx170_dec_clock_on(hx170);

	id = ioread32(hx170->regs);

	dev_dbg(hx170->dev, "ID register 0x%lx\n", id);

	switch (PRODUCT_NUMBER(id)) {
	case 0x8190:
	case 0x8170:
	case 0x9170:
	case 0x9190:
	case 0x6731:
		ret = PRODUCT_NUMBER(id);
		break;
	default:
		dev_err(hx170->dev, "bad hardware ID 0x%lx\n",
			PRODUCT_NUMBER(id));
		break;
	}

	/* disable clock and power before leave */
	hx170_dec_clock_off(hx170);
	hx170_dec_power_off(hx170);

	return ret;
}

static int __devinit hx170_dec_probe(struct platform_device *pdev)
{
	struct hx170_chip *hx170;
	struct resource *iomem;
	int minor;
	int ret = 0;

	hx170 = kzalloc(sizeof(struct hx170_chip), GFP_KERNEL);
	if (hx170 == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory\n");
		ret = -ENOMEM;
		goto bail;
	}

	hx170->perf = kzalloc(sizeof(struct hx170_perf), GFP_KERNEL);
	if (hx170->perf == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory\n");
		ret = -ENOMEM;
		goto out_release_hx170;
	}


	platform_set_drvdata(pdev, hx170);
	hx170->dev = &pdev->dev;

	/* get a memory region for mmio */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(&pdev->dev, "couldn't reserve mmio region\n");
		ret = -ENODEV;
		goto out_release_perf;
	}

	dev_dbg(&pdev->dev, "Probe hx170 at address %x\n", iomem->start);
	/* remap hx170 registers in kernel space */
	hx170->regs =
	    ioremap_nocache(iomem->start, iomem->end - iomem->start + 1);
	if (!hx170->regs) {
		dev_err(&pdev->dev, "can't ioremap hx170 registers region\n");
		ret = -ENOMEM;
		goto out_release_perf;
	}
	hx170->regs_size = iomem->end - iomem->start + 1;

	/* retrieve irq number from board resources */
	hx170->irq = platform_get_irq(pdev, 0);
	if (hx170->irq <= 0) {
		dev_err(&pdev->dev, "No irq defined\n");
		goto out_iounmap;
	}
	dev_dbg(&pdev->dev, "hx170 request irq %d\n", hx170->irq);
	/* request irq */
	if (request_irq
	    (hx170->irq, hx170_dec_interrupt, IRQF_DISABLED | IRQF_SHARED,
	     "hx170", (void *)hx170)) {
		dev_err(&pdev->dev, "can't register IRQ 0x%x\n", hx170->irq);
		goto out_iounmap;
	}

	disable_irq(hx170->irq);

	/* get clock */
	hx170->clk = clk_get(hx170->dev, CLOCK_NAME);
	if (IS_ERR(hx170->clk)) {
		dev_err(&pdev->dev, "can't get clock\n");
		goto out_free_irq;
	}

	ret = clk_prepare(hx170->clk);
	if (ret) {
		dev_err(&pdev->dev, "can't prepare clock\n");
		goto out_put_clk;
	}

	hx170->miscdev.minor = MISC_DYNAMIC_MINOR;
	hx170->miscdev.name = "hx170dec";
	hx170->miscdev.fops = &hx170_dec_fops;

	minor = misc_register(&hx170->miscdev);
	if (minor) {
		dev_err(&pdev->dev, "can't register /dev/hx170dec file\n");
		goto out_release_clock;
	}

	/* initialisation of the protection mutex */
	mutex_init(&hx170->protect_mutex);

	/* initialisation of completion signal */
	init_completion(&hx170->dec_interrupt);
	init_completion(&hx170->pp_interrupt);

	/* create debugfs file */
	hx170->debug_dir = debugfs_create_dir("hx170", NULL);
	hx170->dump_regs =
	    debugfs_create_file("dump_regs", S_IRUSR, hx170->debug_dir, hx170,
				&hx170_dec_debugfs_fops);

	hx170->dump_stats =
	debugfs_create_file("dump_stats", S_IRUSR, hx170->debug_dir, hx170,
			&hx170_dec_stats_fops);


	/* get regulator */
	hx170->regulator = regulator_get(hx170->dev, "v-g1");
	if (IS_ERR(hx170->regulator)) {
		dev_err(&pdev->dev, "can't get regulator\n");
		goto out_release_clock;
	}

	/* check hardware ID */
	hx170->chip_id = hx170_dec_check_hardware_id(hx170);
	if (!hx170->chip_id) {
		dev_err(&pdev->dev, "bad chip ID\n");
		goto out_release_power;
	}

	/*
	 * init pm_runtime used for power management
	 * power down delay is set to 3 ms
	 */
	pm_runtime_set_autosuspend_delay(&pdev->dev, G1_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	/* clear performance  */
	hx170_performance_init(hx170->perf);

	dev_info(&pdev->dev, "found hx170 device\n");
	return ret;

out_release_power:
	regulator_put(hx170->regulator);

out_release_clock:
	clk_unprepare(hx170->clk);

out_put_clk:
	clk_put(hx170->clk);

out_free_irq:
	free_irq(hx170->irq, &hx170_dec_interrupt);

out_iounmap:
	iounmap(hx170->regs);

out_release_perf:
	kfree(hx170->perf);

out_release_hx170:
	kfree(hx170);

bail:
	return ret;
}

static int __devexit hx170_dec_remove(struct platform_device *pdev)
{
	struct hx170_chip *hx170 = platform_get_drvdata(pdev);

	/* remove debugfs file */
	debugfs_remove(hx170->dump_regs);
	debugfs_remove(hx170->dump_stats);
	debugfs_remove(hx170->debug_dir);

	/* disable interrupt */
	disable_irq(hx170->irq);

	pm_runtime_put_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	/* clean hx170 */
	clk_unprepare(hx170->clk);
	clk_put(hx170->clk);

	free_irq(hx170->irq, &hx170_dec_interrupt);
	iounmap(hx170->regs);
	kfree(hx170->perf);
	kfree(hx170);
	return 0;
}

static int g1_runtime_suspend(struct device *dev)
{
	struct hx170_chip *hx170 = dev_get_drvdata(dev);
	hx170_dec_power_off(hx170);
	return 0;
}

static int g1_runtime_resume(struct device *dev)
{
	struct hx170_chip *hx170 = dev_get_drvdata(dev);
	hx170_dec_power_on(hx170);
	return 0;
}

static const struct dev_pm_ops g1_pm_ops = {
	.runtime_suspend = g1_runtime_suspend,
	.runtime_resume = g1_runtime_resume,
};

#define DEV_PM_OPS	(&g1_pm_ops)

static struct platform_driver hx170_dec_driver = {
	.probe = hx170_dec_probe,
	.remove = __devexit_p(hx170_dec_remove),
	.driver = {
		   .name = DEVICE_NAME,
		   .pm   = DEV_PM_OPS,
		   }
};

static int __init hx170_dec_init(void)
{
	return platform_driver_register(&hx170_dec_driver);
}

module_init(hx170_dec_init);

static void __exit hx170_dec_exit(void)
{
	platform_driver_unregister(&hx170_dec_driver);
}

module_exit(hx170_dec_exit);

MODULE_AUTHOR("Benjamin Gaignard <benjamin.gaignard@stericsson.com>");
MODULE_AUTHOR("Jean-Marc Gentit <jean-marc.gentit@stericsson.com>");
MODULE_DESCRIPTION("video decoder driver for hx170");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
