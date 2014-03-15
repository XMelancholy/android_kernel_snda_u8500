/*
 * Copyright (C) ST-Ericsson AB 2010
 * Author:	Sjur Brendeland / sjur.brandeland@stericsson.com
 * License terms: GNU General Public License (GPL) version 2
 */

/* This is a test stub for C2C GENIO */
#define pr_fmt(fmt) "c2c_loop: %s():" fmt, __func__

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <net/pkt_sched.h>
#include <linux/if_arp.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <net/caif/caif_device.h>
#include <net/caif/caif_layer.h>
#include <net/caif/caif_layer.h>
#include <net/caif/cfcnfg.h>
#include <net/caif/cfctrl.h>
#include <linux/interrupt.h>
#include <linux/c2c_genio.h>
#include <linux/xshm/xshm_ipctoc.h>
#include <linux/xshm/xshm_pdev.h>

MODULE_LICENSE("GPL");

struct genio {
	void (*bit_set_cb)(void *data);
	void *data;
	int bit;
	int value;
	struct list_head node;
} clients[32];

static bool c2c_ready_for_caif;
static bool c2c_ready_for_ipc;
static u32 setter;
static u32 getter;
static void (*caif_ready_cb) (bool ready);
static void (*ipc_ready_cb) (void);

struct tasklet_struct task;
struct hrtimer hrtimer;
static spinlock_t c2clock;
static int use_timers = 1;

int genio_subscribe(int bit, void (*bit_set_cb)(void *data), void *data)
{
	pr_debug("enter: bit:%d\n", bit);
	if (bit < 0 || bit > 31)
		return -EINVAL;

	if (clients[bit].bit_set_cb != NULL) {
		pr_err("GENIO bit already subscribed\n");
	}

	clients[bit].bit = bit;
	clients[bit].bit_set_cb = bit_set_cb;
	clients[bit].data = data;
	return 0;
}
EXPORT_SYMBOL(genio_subscribe);

int genio_unsubscribe(int bit)
{
	pr_debug("enter: bit:%d\n", bit);
	clients[bit].bit_set_cb = NULL;
	clients[bit].data = NULL;

	return 0;
}
EXPORT_SYMBOL(genio_unsubscribe);

int genio_set_shm_addr(u32 addr, void (*ipc_ready) (void))
{
	pr_debug("genio_write:0x%04x\n", addr);
	ipc_ready_cb = ipc_ready;
	if (c2c_ready_for_ipc)
		ipc_ready_cb();
	return 0;
}
EXPORT_SYMBOL(genio_set_shm_addr);

int genio_subscribe_caif_ready(void (*caif_ready) (bool ready))
{
	caif_ready_cb = caif_ready;
	return 0;
}
EXPORT_SYMBOL(genio_subscribe_caif_ready);


static void c2c_work(void)
{
	struct genio *e;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&c2clock, flags);
	for (i = 0; i < 28; i++) {
		e = &clients[i];
		if (e->value == 0)
			continue;
		e->value = 0;

		if ( ((1 << (e->bit)) & getter) == 0)
			pr_warn("GETTER MASK WRONG? bit:%x getter:%x\n", e->bit, getter);
		if (clients[e->bit].bit_set_cb == NULL)
			pr_debug("no client for bit:%d\n", e->bit);
		else {
			void (*callback)(void *data);
			void *data;
			pr_devel("callback for bit:%d\n", e->bit);
			callback = clients[e->bit].bit_set_cb;
			data = clients[e->bit].data;
			if (data != NULL && callback != NULL) {
				spin_unlock_irqrestore(&c2clock, flags);
				callback(data);
				spin_lock_irqsave(&c2clock, flags);
			}
		}
	}

	spin_unlock_irqrestore(&c2clock, flags);
}

static void genio_task(unsigned long data)
{
	c2c_work();
}

int genio_set_bit(int bit)
{
	struct genio *e;
	unsigned long flags;
	//pr_devel("setbit:%d\n", bit);

	if ( ((1 << bit) & setter) == 0)
		pr_warn("set of GENIO bit:%x don't match setter:%x\n", bit, setter);

	if (WARN_ON(bit < 0 || bit > 31))
		return -EINVAL;

	spin_lock_irqsave(&c2clock, flags);
	e = &clients[bit];
	if (e->value == 0) {
		e->value = 1;
	}

	if (use_timers) {
		if (!hrtimer_active(&hrtimer)) {
			hrtimer_cancel(&hrtimer);
			hrtimer_start(&hrtimer, ns_to_ktime(10), HRTIMER_MODE_REL);
		}
	} else
		tasklet_hi_schedule(&task);

	spin_unlock_irqrestore(&c2clock, flags);
	return 0;
}
EXPORT_SYMBOL(genio_set_bit);

static enum hrtimer_restart tout(struct hrtimer *hrt)
{
	c2c_work();
	return HRTIMER_NORESTART;
}

int genio_bit_alloc(u32 setter_mask, u32 getter_mask)
{
	pr_devel("setter_mask:%x getter_mask:%x\n", setter_mask, getter_mask);
	setter = setter_mask;
	getter = getter_mask;
	return 0;
}
EXPORT_SYMBOL(genio_bit_alloc);

int genio_reset(void)
{
	setter = 0;
	getter = 0;
	c2c_ready_for_ipc = 0;
	c2c_ready_for_caif = 0;
	caif_ready_cb = 0;
	ipc_ready_cb = 0;
	return 0;
}
EXPORT_SYMBOL(genio_reset);

static ssize_t ipc_ready_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d", c2c_ready_for_ipc);
}

static ssize_t ipc_ready_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	int err;

	pr_debug("set value:%s\n", buf);
	err = strict_strtoul(buf, 10, &val);
	if (err < 0) {
		pr_devel("BAD VALUE FOR IPC_READY\n");
		return -EINVAL;
	}

	if (val == 0) {
		getter = 0;
		setter = 0;
	}

	if (val != c2c_ready_for_ipc) {
		c2c_ready_for_ipc = val;
		pr_devel("SET IPC READY (%d)\n", c2c_ready_for_ipc);
		if (ipc_ready_cb == NULL) {
			pr_devel("client for ipc_ready not set\n");
			return -EINVAL;
		}
//		if (c2c_ready_for_ipc)
			ipc_ready_cb();

#if 0
		/* For test pusposes overwrite ready_for_ipc */
		ready_for_ipc = val;
#endif
	}
	return count;
}
static ssize_t caif_ready_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", c2c_ready_for_caif);
}

static ssize_t caif_ready_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val;
	int err;

	err = strict_strtoul(buf, 10, &val);
	if (err < 0) {
		pr_devel("BAD VALUE FOR CAIF_READY\n");
		return -EINVAL;
	}

	if (val != c2c_ready_for_caif) {
		c2c_ready_for_caif = val;
		pr_devel("SET CAIF READY (%d)\n", (int) val);
		if (caif_ready_cb == NULL) {
			pr_devel("client for caif_ready not set\n");
			return -EINVAL;
		}
		caif_ready_cb(c2c_ready_for_caif);
	}
	return count;
}

static struct kobj_attribute ipc_ready_attr =
	__ATTR(ipc_ready, 0644, ipc_ready_show, ipc_ready_store);

static struct kobj_attribute caif_ready_attr =
	__ATTR(caif_ready, 0644, caif_ready_show, caif_ready_store);

void genio_register_errhandler(void (*errhandler)(int errno))
{

}
EXPORT_SYMBOL(genio_register_errhandler);
int genio_power_req(int state)
{
	pr_debug("power:%s\n", state ? "on" : "off");
	return 0;
}
EXPORT_SYMBOL(genio_power_req);

int __init genio_init(void)
{
	int err;
	spin_lock_init(&c2clock);
	tasklet_init(&task, genio_task, 0L);


	hrtimer_init(&hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer.function = tout;

	pr_devel("Start c2c test\n");
	err = sysfs_create_file(kernel_kobj, &ipc_ready_attr.attr);
	pr_devel("Created sysfs for ipc_ready (err:%d)\n", err);
	err = sysfs_create_file(kernel_kobj, &caif_ready_attr.attr);
	pr_devel("Created sysfs for caif_ready (err:%d)\n", err);
	return 0;
}

void __exit genio_exit(void)
{
	int i;
	unsigned long flags;
	pr_devel("genio_exit\n");

	spin_lock_irqsave(&c2clock, flags);
	for (i = 0; i < 32; i++) {
		clients[i].value = 0;
		clients[i].data = NULL;
	}
	hrtimer_cancel(&hrtimer);
	tasklet_kill(&task);
	spin_unlock_irqrestore(&c2clock, flags);

	sysfs_remove_file(kernel_kobj,  &ipc_ready_attr.attr);
	sysfs_remove_file(kernel_kobj,  &caif_ready_attr.attr);
	pr_devel("genio_exit complete\n");
}
module_init(genio_init);
module_exit(genio_exit);
