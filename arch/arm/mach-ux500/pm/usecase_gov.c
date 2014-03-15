/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Alexandre Torgue <alexandre.torgue@stericsson.com> for ST-Ericsson
 * Author: Vincent Guittot <vincent.guittot@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/io.h>
#include <linux/earlysuspend.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/kernel_stat.h>
#include <linux/ktime.h>
#include <linux/cpufreq.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <mach/usecase_gov.h>

#define CPULOAD_MEAS_DELAY	3000 /* 3 secondes of delta */

/* debug */
static unsigned long debug;

#define hp_printk \
	if (debug) \
		printk \

/* cpu load monitor struct */
struct hotplug_cpu_info {
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_idle;
};

static DEFINE_PER_CPU(struct hotplug_cpu_info, hotplug_info);

/* instant load */
static unsigned long max_instant_load = 85;

/* Number of interrupts per second before exiting auto mode */
static u32 exit_irq_per_s = 1000;
static u64 old_num_irqs;

static DEFINE_MUTEX(usecase_mutex);
static DEFINE_MUTEX(state_mutex);
static enum ux500_uc current_uc = UX500_UC_NORMAL;
static enum ux500_uc enabled_uc = UX500_UC_NORMAL;
static bool is_work_scheduled;
static bool is_early_suspend;
static bool uc_master_enable = true;

static struct usecase_config *usecase_conf;

/* daemon */
static struct delayed_work work_usecase;
static struct early_suspend usecase_early_suspend;

static unsigned int system_min_freq;
static unsigned int system_max_freq;

extern int cpufreq_update_freq(int cpu, unsigned int min, unsigned int max);
extern int cpuidle_set_multiplier(unsigned int value);
extern int cpuidle_force_state(unsigned int state);

static unsigned long determine_cpu_load(void)
{
	int i;
	unsigned long total_load = 0;

	/* get cpu load of each cpu */
	for_each_online_cpu(i) {
		unsigned int load;
		unsigned int idle_time, wall_time;
		cputime64_t cur_wall_time, cur_idle_time;
		struct hotplug_cpu_info *info;

		info = &per_cpu(hotplug_info, i);

		/* update both cur_idle_time and cur_wall_time */
		cur_idle_time = get_cpu_idle_time_us(i, &cur_wall_time);

		/* how much wall time has passed since last iteration? */
		wall_time = (unsigned int) (cur_wall_time -
					    info->prev_cpu_wall);
		info->prev_cpu_wall = cur_wall_time;

		/* how much idle time has passed since last iteration? */
		idle_time = (unsigned int) (cur_idle_time -
					    info->prev_cpu_idle);
		info->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		/* load is the percentage of time not spent in idle */
		load = 100 * (wall_time - idle_time) / wall_time;

		hp_printk("cpu %d load %u, ", i, load);

		total_load += load;
	}

	return total_load / num_online_cpus();
}

static void init_cpu_load_trend(void)
{
	int i;

	for_each_possible_cpu(i) {
		struct hotplug_cpu_info *info;

		info = &per_cpu(hotplug_info, i);

		info->prev_cpu_idle = get_cpu_idle_time_us(i,
						&(info->prev_cpu_wall));
	}
}

static u32 get_num_interrupts_per_s(void)
{
	int cpu;
	int i;
	u64 num_irqs = 0;
	ktime_t now;
	static ktime_t last;
	unsigned int delta;
	u32 irqs = 0;

	now = ktime_get();

	for_each_possible_cpu(cpu) {
		for (i = 0; i < NR_IRQS; i++)
			num_irqs += kstat_irqs_cpu(i, cpu);
	}
	pr_debug("%s: total num irqs: %lld, previous %lld\n",
					__func__, num_irqs, old_num_irqs);

	if (old_num_irqs > 0) {
		delta = (u32)ktime_to_ms(ktime_sub(now, last)) / 1000;
		if (!delta)
			delta = 1;
		irqs = ((u32)(num_irqs - old_num_irqs)) / delta;
	}

	old_num_irqs = num_irqs;
	last = now;

	pr_debug("delta irqs per sec:%d\n", irqs);

	return irqs;
}

static int set_cpufreq(int cpu, int min_freq, int max_freq)
{
	int ret;
	struct cpufreq_policy policy;

	pr_debug("set cpu freq: min %d max: %d\n", min_freq, max_freq);

	ret = cpufreq_get_policy(&policy, cpu);
	if (ret < 0) {
		pr_err("usecase-gov: failed to read policy\n");
		return ret;
	}

	if (policy.min > max_freq) {
		ret = cpufreq_update_freq(cpu, min_freq, policy.max);
		if (ret)
			pr_err("usecase-gov: update min cpufreq failed (1)\n");
	}
	if (policy.max < min_freq) {
		ret = cpufreq_update_freq(cpu, policy.min, max_freq);
		if (ret)
			pr_err("usecase-gov: update max cpufreq failed (2)\n");
	}

	ret = cpufreq_update_freq(cpu, min_freq, max_freq);
	if (ret)
		pr_err("usecase-gov: update min-max cpufreq failed\n");

	return ret;
}

static void set_cpu_config(enum ux500_uc new_uc)
{
	int cpu;
	int min_freq, max_freq;

	pr_debug("%s: new_usecase=%d, current_usecase=%d\n",
		__func__, new_uc, current_uc);

	if (new_uc == current_uc)
		return;

	/* Cpu hotplug */
	if (!(usecase_conf[new_uc].second_cpu_online) &&
	    (num_online_cpus() > 1))
		cpu_down(1);
	else if ((usecase_conf[new_uc].second_cpu_online) &&
		 (num_online_cpus() < 2))
		cpu_up(1);

	if (usecase_conf[new_uc].max_arm)
		max_freq = usecase_conf[new_uc].max_arm;
	else
		max_freq = system_max_freq;

	if (usecase_conf[new_uc].min_arm)
		min_freq = usecase_conf[new_uc].min_arm;
	else
		min_freq = system_min_freq;

	for_each_online_cpu(cpu)
		set_cpufreq(cpu,
			    min_freq,
			    max_freq);

	/*
	 * Kinda doing the job twice, but this is needed for reference keeping
	 */
	if (usecase_conf[new_uc].min_arm)
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "usecase",
					     usecase_conf[new_uc].min_arm);
	else
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,
					     "usecase",
					     PRCMU_QOS_DEFAULT_VALUE);

	/* Cpu idle */
	cpuidle_set_multiplier(usecase_conf[new_uc].cpuidle_multiplier);

	/* L2 prefetch */
	if (usecase_conf[new_uc].l2_prefetch_en)
		outer_prefetch_enable();
	else
		outer_prefetch_disable();

	/* Force cpuidle state */
	cpuidle_force_state(usecase_conf[new_uc].cpuidle_force_state);

	/* QOS override */
	prcmu_qos_voice_call_override(usecase_conf[new_uc].vc_override);

	current_uc = new_uc;
}

void usecase_update_governor_state(void)
{
	bool cancel_work = false;

	/*
	 * usecase_mutex will have to be unlocked to ensure safe exit of
	 * delayed_usecase_work(). Protect this function with its own mutex
	 * from being executed by multiple threads at that point.
	 */
	mutex_lock(&state_mutex);
	mutex_lock(&usecase_mutex);

	if (uc_master_enable && (enabled_uc != UX500_UC_NORMAL)) {
		/*
		 * Usecases are enabled. If we are in early suspend put
		 * governor to work.
		 */
		if (is_early_suspend && !is_work_scheduled) {
			schedule_delayed_work_on(0, &work_usecase,
				msecs_to_jiffies(CPULOAD_MEAS_DELAY));
			is_work_scheduled = true;
		} else if (!is_early_suspend && is_work_scheduled) {
			/* Exiting from early suspend. */
			cancel_work = true;
		}

	} else if (is_work_scheduled) {
		/* No usecase enabled or governor is not enabled. */
		cancel_work =  true;
	}

	if (cancel_work) {
		/*
		 * usecase_mutex is used by delayed_usecase_work() so it must
		 * be unlocked before we call to cacnel the work.
		 */
		mutex_unlock(&usecase_mutex);
		cancel_delayed_work_sync(&work_usecase);
		mutex_lock(&usecase_mutex);

		is_work_scheduled = false;

		/* Set the default settings before exiting. */
		set_cpu_config(UX500_UC_NORMAL);
	}

	mutex_unlock(&usecase_mutex);
	mutex_unlock(&state_mutex);
}

/*
 * Start load measurment every 6 s in order detrmine if can unplug one CPU.
 * In order to not corrupt measurment, the first load average is not done
 * here call in  early suspend.
 */
static void usecase_earlysuspend_callback(struct early_suspend *h)
{
	init_cpu_load_trend();

	is_early_suspend = true;

	usecase_update_governor_state();
}

/* Stop measurement, call LCD early resume */
static void usecase_lateresume_callback(struct early_suspend *h)
{
	is_early_suspend = false;

	usecase_update_governor_state();
}

static void delayed_usecase_work(struct work_struct *work)
{
	unsigned long load;
	bool inc_perf = false;
	u32 irqs_per_s;

	/* determine instant load */
	load = determine_cpu_load();
	hp_printk("cpu instant load = %lu max %lu\n", load, max_instant_load);

	irqs_per_s = get_num_interrupts_per_s();

	/* Dont let configuration change in the middle of our calculations. */
	mutex_lock(&usecase_mutex);

	/* detect "instant" load increase */
	if (load > max_instant_load || irqs_per_s > exit_irq_per_s)
		inc_perf = true;

	/*
	 * set_cpu_config() will not update the config unless it has been
	 * changed.
	 */
	if (inc_perf)
		set_cpu_config(UX500_UC_NORMAL);
	else
		set_cpu_config(enabled_uc);

	mutex_unlock(&usecase_mutex);

	/* reprogramm scheduled work */
	schedule_delayed_work_on(0, &work_usecase,
				msecs_to_jiffies(CPULOAD_MEAS_DELAY));
}

static struct dentry *usecase_dir;

#ifdef CONFIG_DEBUG_FS
#define define_set(_name) \
static ssize_t set_##_name(struct file *file, \
				 const char __user *user_buf, \
				 size_t count, loff_t *ppos) \
{ \
	int err; \
	long unsigned i; \
 \
	err = kstrtoul_from_user(user_buf, count, 0, &i); \
 \
	if (err) \
		return err; \
 \
	_name = i; \
	hp_printk("New value : %lu\n", _name); \
 \
	return count; \
}

define_set(max_instant_load);
define_set(debug);

#define define_print(_name) \
static ssize_t print_##_name(struct seq_file *s, void *p) \
{ \
	return seq_printf(s, "%lu\n", _name); \
}

define_print(max_instant_load);
define_print(debug);

#define define_open(_name) \
static ssize_t open_##_name(struct inode *inode, struct file *file) \
{ \
	return single_open(file, print_##_name, inode->i_private); \
}

define_open(max_instant_load);
define_open(debug);

#define define_dbg_file(_name) \
static const struct file_operations fops_##_name = { \
	.open = open_##_name, \
	.write = set_##_name, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release, \
	.owner = THIS_MODULE, \
}; \
static struct dentry *file_##_name;

define_dbg_file(max_instant_load);
define_dbg_file(debug);

struct dbg_file {
	struct dentry **file;
	const struct file_operations *fops;
	const char *name;
};

#define define_dbg_entry(_name) \
{ \
	.file = &file_##_name, \
	.fops = &fops_##_name, \
	.name = #_name \
}

static struct dbg_file debug_entry[] = {
	define_dbg_entry(max_instant_load),
	define_dbg_entry(debug),
};

static int setup_debugfs(void)
{
	int i;
	usecase_dir = debugfs_create_dir("usecase", NULL);

	if (IS_ERR_OR_NULL(usecase_dir))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(debug_entry); i++) {
		if (IS_ERR_OR_NULL(debugfs_create_file(debug_entry[i].name,
						       S_IWUGO | S_IRUGO,
						       usecase_dir,
						       NULL,
						       debug_entry[i].fops)))
			goto fail;
	}

	if (IS_ERR_OR_NULL(debugfs_create_u32("exit_irq_per_s",
					      S_IWUGO | S_IRUGO, usecase_dir,
					      &exit_irq_per_s)))
		goto fail;
	return 0;
fail:
	debugfs_remove_recursive(usecase_dir);
	return -EINVAL;
}
#else
static int setup_debugfs(void)
{
	return 0;
}
#endif

struct usecase_dev_attr {
	struct device_attribute dev_attr;
	u32 index;
};

/* One for each usecase + current + enable */
#define UX500_NUM_SYSFS_NODES (UX500_UC_MAX + 2)
#define UX500_CURRENT_NODE_INDEX (UX500_NUM_SYSFS_NODES - 1)
#define UX500_ENABLE_NODE_INDEX (UX500_NUM_SYSFS_NODES - 2)

static struct usecase_dev_attr usecase_dc_attr[UX500_NUM_SYSFS_NODES];

static struct attribute *dbs_attributes[UX500_NUM_SYSFS_NODES + 1] = {NULL};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "usecase",
};

static ssize_t show_current(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "min_arm: %d\n"
		"max_arm: %d\n"
		"cpuidle_multiplier: %ld\n"
		"second_cpu_online: %s\n"
		"l2_prefetch_en: %s\n"
		"cpuidle_force_state: %d\n"
		"vc_override: %s\n",
		usecase_conf[current_uc].min_arm,
		usecase_conf[current_uc].max_arm,
		usecase_conf[current_uc].cpuidle_multiplier,
		usecase_conf[current_uc].second_cpu_online ? "true" : "false",
		usecase_conf[current_uc].l2_prefetch_en ? "true" : "false",
		usecase_conf[current_uc].cpuidle_force_state,
		usecase_conf[current_uc].vc_override ? "true" : "false");
}

static ssize_t show_enable(struct device *device,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", uc_master_enable);
}

static ssize_t store_enable(struct device *device,
					struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	uc_master_enable = (bool) input;

	usecase_update_governor_state();

	return count;
}

static ssize_t show_dc_attr(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct usecase_dev_attr *uattr =
		container_of(attr, struct usecase_dev_attr, dev_attr);

	return sprintf(buf, "%u\n",
				(uattr->index == enabled_uc));
}

static ssize_t store_dc_attr(struct device *device,
					struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	struct usecase_dev_attr *uattr =
		container_of(attr, struct usecase_dev_attr, dev_attr);

	ret = sscanf(buf, "%u", &input);

	/* Normal mode cant be changed. */
	if ((ret != 1) || (uattr->index == UX500_UC_NORMAL))
		return -EINVAL;

	if (input) {
		/* Check if any usecase is already enabled */
		if (enabled_uc != UX500_UC_NORMAL) {
			pr_err("usecase governor: %s already enabled\n",
					usecase_conf[enabled_uc].name);
			return -EINVAL;
		}

		enabled_uc = uattr->index;
		pr_info("usecase governor: %s enabled\n",
				usecase_conf[enabled_uc].name);
	} else {
		if (uattr->index == enabled_uc) {
			pr_info("usecase governor: %s disabled\n",
					usecase_conf[enabled_uc].name);
			enabled_uc = UX500_UC_NORMAL;
		} else {
			pr_err("usecase governor: %s not enabled\n",
				usecase_conf[uattr->index].name);
			return -EINVAL;
		}
	}

	if (uattr->index == UX500_UC_VC)
		prcmu_vc((bool)input);

	usecase_update_governor_state();

	return count;
}

static int usecase_sysfs_init(void)
{
	int err;
	int i;

	/* Last two nodes are not based on usecase configurations */
	for (i = 0; i < (UX500_NUM_SYSFS_NODES - 2); i++) {
		usecase_dc_attr[i].dev_attr.attr.name = usecase_conf[i].name;
		usecase_dc_attr[i].dev_attr.attr.mode = 0644;
		usecase_dc_attr[i].dev_attr.show = show_dc_attr;
		usecase_dc_attr[i].dev_attr.store = store_dc_attr;
		usecase_dc_attr[i].index = i;

		dbs_attributes[i] = &(usecase_dc_attr[i].dev_attr.attr);
	}

	/* sysfs current */
	usecase_dc_attr[UX500_CURRENT_NODE_INDEX].dev_attr.attr.name =
		"current";
	usecase_dc_attr[UX500_CURRENT_NODE_INDEX].dev_attr.attr.mode =
		0644;
	usecase_dc_attr[UX500_CURRENT_NODE_INDEX].dev_attr.show =
		show_current;
	usecase_dc_attr[UX500_CURRENT_NODE_INDEX].dev_attr.store =
		NULL;
	usecase_dc_attr[UX500_CURRENT_NODE_INDEX].index =
		0;
	dbs_attributes[UX500_CURRENT_NODE_INDEX] =
		&(usecase_dc_attr[UX500_CURRENT_NODE_INDEX].dev_attr.attr);

	/* sysfs enable */
	usecase_dc_attr[UX500_ENABLE_NODE_INDEX].dev_attr.attr.name =
		"enable";
	usecase_dc_attr[UX500_ENABLE_NODE_INDEX].dev_attr.attr.mode =
		0644;
	usecase_dc_attr[UX500_ENABLE_NODE_INDEX].dev_attr.show =
		show_enable;
	usecase_dc_attr[UX500_ENABLE_NODE_INDEX].dev_attr.store =
		store_enable;
	usecase_dc_attr[UX500_ENABLE_NODE_INDEX].index =
		0;
	dbs_attributes[UX500_ENABLE_NODE_INDEX] =
		&(usecase_dc_attr[UX500_ENABLE_NODE_INDEX].dev_attr.attr);

	err = sysfs_create_group(&cpu_subsys.dev_root->kobj,
				 &dbs_attr_group);
	if (err)
		pr_err("usecase governor: sysfs_create_group"
				" failed with error = %d\n", err);

	return err;
}

/*  initialize devices */
static int __init probe_usecase_devices(struct platform_device *pdev)
{
	int err;
	struct cpufreq_frequency_table *table;
	unsigned int min_freq = UINT_MAX;
	unsigned int max_freq = 0;
	int i;

	usecase_conf = dev_get_platdata(&pdev->dev);

	table = cpufreq_frequency_get_table(0);

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (min_freq > table[i].frequency)
			min_freq = table[i].frequency;
		if (max_freq < table[i].frequency)
			max_freq = table[i].frequency;
	}

	system_min_freq = min_freq;
	system_max_freq = max_freq;

	/*  add early_suspend callback */
	usecase_early_suspend.level = 200;
	usecase_early_suspend.suspend = usecase_earlysuspend_callback;
	usecase_early_suspend.resume = usecase_lateresume_callback;
	register_early_suspend(&usecase_early_suspend);

	/* register delayed queuework */
	INIT_DELAYED_WORK_DEFERRABLE(&work_usecase,
				     delayed_usecase_work);

	init_cpu_load_trend();

	err = setup_debugfs();
	if (err)
		goto error;
	err = usecase_sysfs_init();
	if (err)
		goto error2;

	prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, "usecase",
				  PRCMU_QOS_DEFAULT_VALUE);

	pr_info("usecase governor initialized\n");

	return 0;
error2:
	debugfs_remove_recursive(usecase_dir);
error:
	unregister_early_suspend(&usecase_early_suspend);
	return err;
}
static struct platform_driver dbx500_usecase_gov = {
	.driver = {
		.name = "dbx500-usecase-gov",
		.owner = THIS_MODULE,
	},
};

static int __init init_usecase_devices(void)
{
	return platform_driver_probe(&dbx500_usecase_gov,
					probe_usecase_devices);
}

late_initcall(init_usecase_devices);
