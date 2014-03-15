/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License terms: GNU General Public License (GPL), version 2
 * Author: Duan Jingbo <jingbo.duan@stericsson.com>
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/c2c_genio.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/notifier.h>
#include "ste_xmip.h"

/* Global to access XMIP platform device data */
static struct ste_xmip *xmip_dev;

static inline void xmip_mask_modapp(u32 mark, void __iomem *base)
{
	writel(mark, base + PRCMOFF_A9_XMIP_MASK_SET);
}

static inline void xmip_unmask_modapp(u32 mark, void __iomem *base)
{
	writel(mark, base + PRCMOFF_A9_XMIP_MASK_CLR);
}

static inline void xmip_set_appmod(u32 mark, void __iomem *base)
{
	writel(mark, base + PRCMOFF_XMIP_APPMOD_SET);
}

static inline void xmip_clear_appmod(u32 mark, void __iomem *base)
{
	writel(mark, base + PRCMOFF_XMIP_APPMOD_CLR);
}

static inline u32 xmip_read_modapp(void __iomem *base)
{
	return readl(base + PRCMOFF_XMIP_MODAPP_GENO);
}

/* MODAPP IRQs fast handshake (through MODAPP line to A9 GIC) */
static int config_ipchw_bit(int bit,
		irqreturn_t (*hdl)(int irq, void *data))
{
	struct xmip_ipchw *ipchw = &xmip_dev->ipchw[bit];
	char name[32];
	int ret;
	u32 tmp;

	if (xmip_dev == NULL)
		return -ENODEV;

	tmp = xmip_dev->setter_mask | xmip_dev->getter_mask;
	if (((1 << bit) & tmp) == 0) {
		dev_err(xmip_dev->dev, "bit%d not alloc in setter_mask\n", bit);
		return -EINVAL;
	}
	if (ipchw->mask) {
		dev_err(xmip_dev->dev, "bit%d already subscribed to\n", bit);
		return -EINVAL;
	}
	ret = 0;
	memset(ipchw, 0, sizeof(*ipchw));
	ipchw->mask = 1 << bit;
	ipchw->bit = bit;
	ipchw->irq = -1;
	ipchw->prcm = xmip_dev->base;

	if (ipchw->mask & xmip_dev->setter_mask)
		return 0;

	sprintf(name, "XMIP_MODAPP%d", bit);
	ipchw->irq = platform_get_irq_byname(xmip_dev->pdev, name);
	if (ipchw->irq < 0) {
		dev_err(xmip_dev->dev, "Failed to get irq %s\n", name);
		ret = -EINVAL;
		goto err;
	}
	ret = request_irq(ipchw->irq, hdl, IRQF_NO_SUSPEND,
			dev_name(xmip_dev->dev), ipchw);
	if (ret) {
		dev_err(xmip_dev->dev, "Failed to request IRQ %s\n", name);
		goto err;
	}
	return 0;
err:
	if (ipchw)
		memset(ipchw, 0, sizeof(*ipchw));
	return ret;
}

/* Hardcoded fastpolling timeout threshold: about 1ms */
#define FASTPOLL_TO	0x100000LL
#define FASTPOLL_THR	(~0LL - FASTPOLL_TO)

static void fastpoll_timeout_init(unsigned long long *sclk_end)
{
	unsigned long long sclk = sched_clock();

	*sclk_end = FASTPOLL_TO;
	if (likely(sclk < FASTPOLL_THR))
		*sclk_end += sclk;
	else
		*sclk_end -= FASTPOLL_THR - sclk;
}

static bool fastpoll_timeout_check(unsigned long long *sclk_end)
{
	unsigned long long sclk = sched_clock();

	if (unlikely((*sclk_end <= FASTPOLL_TO) && (sclk > FASTPOLL_THR)))
		return false;
	if (unlikely(sclk > *sclk_end))
		return true;
	return false;
}

/* IRQ hander for incoming fast handshake notification */
static irqreturn_t fast_getter_hdl(int irq, void *data)
{
	/* Polling protection: here use sched_clock, with 1ms timeout */
	struct xmip_ipchw *ipchw = (struct xmip_ipchw *) data;
	unsigned long long sclk_end;

	if (likely(ipchw->event_cb))
		ipchw->event_cb(ipchw->event_data);

	xmip_set_appmod(ipchw->mask, ipchw->prcm);
	fastpoll_timeout_init(&sclk_end);
	while ((xmip_read_modapp(ipchw->prcm) & ipchw->mask) != 0) {
		if (unlikely(fastpoll_timeout_check(&sclk_end)))
			goto poll_err;
	}
	xmip_clear_appmod(ipchw->mask, ipchw->prcm);
	ipchw->hs_cnt++;
	return IRQ_HANDLED;
poll_err:
	ipchw->poll_timeout = 1;
	xmip_dev->protection_event = 1;
	wake_up_interruptible_all(&xmip_dev->waitq);
	return IRQ_HANDLED;
}

/* XMIP driver handler for 'genio_wrap' subscribe() API */
static int xmip_subscribe(int bit, void (*bit_set_cb)(void *data), void *data)
{
	struct xmip_ipchw *ipchw = &xmip_dev->ipchw[bit];

	dev_dbg(xmip_dev->dev, "subscribe to bit %d\n", bit);

	if ((xmip_dev->getter_mask & (1 << bit)) == 0) {
		dev_err(xmip_dev->dev, "bit%d not alloc in getter_mask\n", bit);
		return -EINVAL;
	}
	if (ipchw->mask == 0) {
		dev_err(xmip_dev->dev, "bit%d IRQ subscrption error\n", bit);
		return -EINVAL;
	}
	if (ipchw->event_cb != 0) {
		dev_err(xmip_dev->dev, "bit%d already subscribed to\n", bit);
		return -EINVAL;
	}
	ipchw->event_cb = bit_set_cb;
	ipchw->event_data = data;

	/* insure callback resources are set before IT can trig */
	smp_wmb();
	xmip_unmask_modapp(ipchw->mask, ipchw->prcm);
	return 0;
}

/* XMIP driver handler for 'genio_wrap' unsubscribe() API */
static int xmip_unsubscribe(int bit)
{
	struct xmip_ipchw *ipchw = &xmip_dev->ipchw[bit];

	dev_dbg(xmip_dev->dev, "unsubscribe from bit %d\n", bit);

	if ((xmip_dev->getter_mask & (1 << bit)) == 0) {
		dev_err(xmip_dev->dev, "bit%d not alloc in getter_mask\n", bit);
		return -EINVAL;
	}

	xmip_mask_modapp(ipchw->mask, ipchw->prcm);
	ipchw->event_cb = NULL;
	smp_wmb(); /* Update event cb before event data */
	ipchw->event_data = NULL;
	return 0;
}

/* XMIP driver handler for 'genio_wrap' set_bit() API */
static int xmip_set_bit(int bit)
{
	struct xmip_ipchw *ipchw = &xmip_dev->ipchw[bit];
	unsigned long long sclk_end;

	if (unlikely((xmip_dev->setter_mask & (1 << bit)) == 0)) {
		dev_err(xmip_dev->dev, "bit%d not alloc in setter_mask\n", bit);
		return -EINVAL;
	}

	fastpoll_timeout_init(&sclk_end);

	while ((xmip_read_modapp(ipchw->prcm) & ipchw->mask) != 0) {
		if (unlikely(fastpoll_timeout_check(&sclk_end))) {
			if ((xmip_read_modapp(ipchw->prcm) & ipchw->mask) != 0)
				goto poll_err;
			else
				break;
		}
	}
	xmip_set_appmod(ipchw->mask, ipchw->prcm);
	while ((xmip_read_modapp(ipchw->prcm) & ipchw->mask) == 0) {
		if (unlikely(fastpoll_timeout_check(&sclk_end))) {
			if ((xmip_read_modapp(ipchw->prcm) & ipchw->mask) == 0)
				goto poll_err;
			else
				break;
		}
	}
	xmip_clear_appmod(ipchw->mask, ipchw->prcm);
	ipchw->hs_cnt++;
	return 0;

poll_err:
	ipchw->poll_timeout = 1;
	xmip_dev->protection_event = 1;
	wake_up_interruptible_all(&xmip_dev->waitq);
	return -1;
}

/* XMIP driver handler for 'genio_wrap' bit_alloc() API */
static int xmip_bit_alloc(u32 setter_mask, u32 getter_mask)
{
	struct xmip_ipchw *ipchw;
	int bit, ret = 0;

	if (xmip_dev == NULL)
		return -ENODEV;

	if (xmip_dev->init_flags & XMIP_INIT_BIT_ALLOC) {
		dev_err(xmip_dev->dev, "bit_alloc already called\n");
		return -EINVAL;
	}
	if ((setter_mask & getter_mask) != 0) {
		dev_err(xmip_dev->dev, "setter and getter masks overlap\n");
		return -EINVAL;
	}

	xmip_dev->setter_mask = setter_mask;
	xmip_dev->getter_mask = getter_mask;

	xmip_mask_modapp(setter_mask | getter_mask, xmip_dev->base);

	ipchw = xmip_dev->ipchw;
	for (bit = 0; bit < XMIP_IPCHW_LINE_NB; bit++) {
		if ((1 << bit) & xmip_dev->getter_mask)
			ret = config_ipchw_bit(bit, fast_getter_hdl);
		else if ((1 << bit) & xmip_dev->setter_mask)
			ret = config_ipchw_bit(bit, NULL);
		if (ret)
			return ret;
		ipchw++;
	}

	mutex_lock(&xmip_dev->lock);
	xmip_dev->init_flags |= XMIP_INIT_BIT_ALLOC;
	mutex_unlock(&xmip_dev->lock);
	return 0;
}

static void protection_nfy_tout(struct ste_xmip *xmip,
		struct xmip_ipchw *ipchw)
{
	if (xmip->errhandler == NULL) {
		dev_err(xmip->dev, "%d poll tout, no handler\n", ipchw->bit);
		return;
	}
	if (ipchw->poll_timeout_nfy == 0) {
		ipchw->poll_timeout_nfy = 1;
		xmip->errhandler(-ETIMEDOUT);
	} else if (ipchw->poll_timeout_nfy < 3) {
		ipchw->poll_timeout_nfy++;
	}
	if (ipchw->poll_timeout_nfy < 2)
		dev_err(xmip->dev, "%d poll tout (%d)\n", ipchw->bit,
				ipchw->poll_timeout_nfy);
}

/*
 * protection_work
 *
 * Sleep on waitqueue xmip_dev->waitq on event prot_event!=0
 * If xmip->reset_flag => end work
 * If ipchw[i]->poll_timeout => errhandler().
 */
static void protection_work(struct work_struct *work)
{
	struct ste_xmip *xmip;
	int ret, i;

	xmip = container_of(work, struct ste_xmip, protection_work);
	ret = 0;
	while (1) {
		/* Sleep until some event occur */
		ret = wait_event_interruptible(xmip->waitq,
				(xmip->protection_event != 0));
		if (ret) {
			dev_err(xmip->dev, "unexpected wake of 'prot_work'\n");
			break;
		}
		xmip->protection_event = 0;

		/* ipchw reset */
		if (xmip->reset_flag)
			break;

		/* polling timeout */
		for (i = 0; i < XMIP_IPCHW_LINE_NB; i++) {
			if (xmip->ipchw[i].poll_timeout)
				protection_nfy_tout(xmip, &xmip->ipchw[i]);
		}
	}
	if ((ret < 0) && xmip->errhandler)
		xmip->errhandler(ret);
}

/* Work that manages IPC_READY handshake */
static void ipc_ready_work(struct work_struct *work)
{
	struct ste_xmip *xmip = container_of(work,
		struct ste_xmip, ipc_ready_work);
	u32 mask = 1 << READY_FOR_IPC_BIT;
	int ret = 0;

	if (xmip->reset_flag)
		return;

	/* Case IPC_READY already set to 1 */
	if ((xmip_read_modapp(xmip->base) & mask))
		xmip_dev->ipc_ready_state = 1;

	ret = wait_event_interruptible(xmip->waitq, (xmip->reset_flag ||
		(xmip_dev->ipc_ready_state != 0)));
	if ((ret == -ERESTARTSYS) || xmip->reset_flag)
		goto err;
	dev_dbg(xmip->dev, "ipc_ready raise detected\n");

	xmip_clear_appmod(APPMOD_CLEAR_ALL, xmip->base);

	ret = wait_event_interruptible(xmip->waitq, (xmip->reset_flag ||
			(xmip_dev->ipc_ready_state == 0)));
	if ((ret == -ERESTARTSYS) || xmip->reset_flag)
		goto err;
	dev_info(xmip->dev, "ipc_ready handshake completed\n");

	if (xmip->ipc_ready_cb)
		xmip->ipc_ready_cb();

	dev_dbg(xmip->dev, "ipc_ready completed\n");
	return;
err:
	dev_err(xmip->dev, "ipc_ready failure\n");
	if ((ret < 0) && xmip->errhandler)
		xmip->errhandler(ret);
}

/* XMIP driver handler for 'genio_wrap' set_shm_addr() API */
static int xmip_set_shm_addr(u32 addr, void (*ipc_ready_cb) (void))
{
	if (xmip_dev == NULL)
		return -ENODEV;

	dev_dbg(xmip_dev->dev, "xmip_set_shm_addr start\n");
	xmip_dev->ipc_ready_cb = ipc_ready_cb;

	prcmu_xmip_modapp_notif(PRCM_DEFAULT_MODAPP_NOTIF,
			PRCM_DEFAULT_EDGE_SENS_L8,
			PRCM_DEFAULT_EDGE_SENS_H8);

	/* Write boot addr */
	xmip_clear_appmod(APPMOD_CLEAR_ALL, xmip_dev->base);
	xmip_set_appmod(addr, xmip_dev->base);

	schedule_work(&xmip_dev->ipc_ready_work);
	schedule_work(&xmip_dev->protection_work);
	return 0;
}

/* XMIP driver handler for 'genio_wrap' subscribe_caif_ready() API */
static int xmip_subscribe_caif_ready(void (*caif_ready_cb)(bool ready))
{
	if (xmip_dev == NULL)
		return -ENODEV;

	dev_dbg(xmip_dev->dev, "caif_ready called\n");
	xmip_dev->caif_ready_cb = caif_ready_cb;
	return 0;
}

/* XMIP driver handler for 'genio_wrap' register_errhandler() API */
static void xmip_register_errhandler(void (*errhandler)(int errno))
{
	if (xmip_dev == NULL)
		return;
	dev_dbg(xmip_dev->dev, "registering errhandler\n");
	xmip_dev->errhandler = errhandler;
}

/* XMIP driver handler for 'genio_wrap' reset() API */
static int xmip_reset(void)
{
	struct xmip_ipchw *ipchw;
	int i;

	if (xmip_dev == NULL)
		return 0;
	dev_dbg(xmip_dev->dev, "processing reset\n");
	mutex_lock(&xmip_dev->lock);
	xmip_mask_modapp(MODAPP_MASK_ALL, xmip_dev->base);
	xmip_dev->init_flags = 0;
	xmip_dev->reset_flag = 1;
	ipchw = xmip_dev->ipchw;
	for (i =  0; i < XMIP_IPCHW_LINE_NB; i++, ipchw++) {
		if (ipchw->mask && ipchw->irq)
			free_irq(ipchw->irq, ipchw);
		memset(ipchw, 0, sizeof(*ipchw));
	}
	xmip_dev->setter_mask = 0;
	xmip_dev->getter_mask = 0;
	xmip_dev->ipc_ready_cb = NULL;
	xmip_dev->caif_ready_cb = NULL;
	xmip_dev->errhandler = NULL;

	xmip_dev->protection_event = 1;
	wake_up_interruptible_all(&xmip_dev->waitq);
	flush_scheduled_work();

	xmip_dev->reset_flag = 0;
	mutex_unlock(&xmip_dev->lock);
	return 0;
}

static int xmip_ipc_ready_notif(struct notifier_block *nb,
		unsigned long event, void *data)
{
	u32 cur;
	struct upap_modapp_evt *evt = (struct upap_modapp_evt *)data;

	if (evt->modapp_itstatus & (1 << READY_FOR_IPC_BIT)) {
		xmip_dev->ipc_ready_state = (evt->modapp_linelevel
			& (1 << READY_FOR_IPC_BIT)) ? 1 : 0;

		dev_dbg(xmip_dev->dev, "IPC_READY bit switched to %d\n",
				xmip_dev->ipc_ready_state);
		wake_up_interruptible_all(&xmip_dev->waitq);

		cur = (readl(xmip_dev->base + PRCMOFF_LINE_VALUE_8)
			& (1 << READY_FOR_IPC_BIT)) ? 1 : 0;
		if (cur != xmip_dev->ipc_ready_state)
			dev_err(xmip_dev->dev,
				"ERROR: IPC_READY toggled during detection\n");
	}
	return 0;
}

static int xmip_caif_ready_notif(struct notifier_block *nb,
		unsigned long event, void *data)
{
	u32 cur;
	struct upap_resoutxnlevel_evt *evt =
		(struct upap_resoutxnlevel_evt *)data;

	dev_dbg(xmip_dev->dev, "CAIF_READY bit switched to %d\n",
		evt->resoutxn_level);
	if (xmip_dev->caif_ready_cb)
		xmip_dev->caif_ready_cb(evt->resoutxn_level);

	/* CAIF_READY from RESOUT2N not from ModApp[CAIF_READY] */
	cur = (readl(xmip_dev->dev + PRCMOFF_LINE_VALUE)
		& PRCM_LINE_VALUE_RESOUT2_MASK) ? 1 : 0;
	if (cur != evt->resoutxn_level) {
		dev_info(xmip_dev->dev, "CAIF_READY toggled during" \
				"detection\n");
		if (xmip_dev->caif_ready_cb)
			xmip_dev->caif_ready_cb(cur);
	}
	return 0;
}

static struct notifier_block xmip_modapp_nb = {
	.notifier_call = xmip_ipc_ready_notif,
};

static struct notifier_block resout2n_nb = {
	.notifier_call = xmip_caif_ready_notif,
};

/* APIs to be registered to geion_wrapper */
static struct genio_wrap ste_xmip_genio_apis = {
	.drvname = "xmip",
	.reset = xmip_reset,
	.set_shm_addr = xmip_set_shm_addr,
	.subscribe_caif_ready = xmip_subscribe_caif_ready,
	.register_errhandler = xmip_register_errhandler,
	.bit_alloc = xmip_bit_alloc,
	.subscribe = xmip_subscribe,
	.unsubscribe = xmip_unsubscribe,
	.set_bit = xmip_set_bit,
};

#ifdef CONFIG_DEBUG_FS
static int dbg_status_print(struct seq_file *s, void *p)
{
	struct ste_xmip *xmip = (struct ste_xmip *) s->private;
	struct xmip_ipchw *ipchw;
	int i = 0, j = 0;

	if (xmip == NULL) {
		seq_printf(s, "No xmip device avaiable\n");
		return 0;
	}
	seq_printf(s, "Subscribed MODAPP/APPMODs:\n");
	seq_printf(s, "- setter 0x%08X    getter 0x%08X\n",
		xmip->setter_mask, xmip->getter_mask);

	for (ipchw = xmip->ipchw; i < XMIP_IPCHW_LINE_NB; i++, ipchw++) {
		if (ipchw->mask == 0)
			continue;
		seq_printf(s, "- bit %02d : %s, timeout:%d, cb:%p, cnt:%d\n",
			i, (ipchw->mask & xmip->setter_mask) ? "Fast-Setter" :
			(ipchw->mask & xmip->getter_mask) ? "Fast-Getter" :
			"unknown", ipchw->poll_timeout,
			ipchw->event_cb, ipchw->hs_cnt);
		j++;
	}
	if (j == 0)
		seq_printf(s, "- no configured bit.\n");
	seq_printf(s, "\n");

	seq_printf(s, "Misc: init=%04X reset=%d prot_evt=%d ",
			xmip->init_flags, xmip->reset_flag,
			xmip->protection_event);
	seq_printf(s, "\n");
	seq_printf(s, "PRCM Registers\n");
	seq_printf(s, "-a9_xmip_mask 0x%08X appmod_val 0x%08X modapp 0x%08X\n",
		readl(xmip_dev->base + PRCMOFF_A9_XMIP_MASK_VAL),
		readl(xmip_dev->base + PRCMOFF_XMIP_APPMOD_VAL),
		readl(xmip_dev->base + PRCMOFF_XMIP_MODAPP_GENO));

	return 0;
}

static int dbg_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_status_print, inode->i_private);
}

static const struct file_operations dbg_status_fops = {
	.open = dbg_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

const char xmip_dbg_test_help[] =
	"Supported commands one can write to this interface:\n"
	"\n"
	"errhandler                => genio_errhandler(callback)\n"
	"ipc_ready ADDR            => xmip_set_shm_addr(addr, callback)\n"
	"caif_ready                => xmip_subscribe_caif_ready(callback)\n"
	"bit_alloc SETTER GETTER   => xmip_bit_alloc(setter, getter)\n"
	"reset                     => xmip_reset()\n"
	"\n"
	"set_bit BIT               => xmip_set_bit(bit)\n"
	"subscribe BIT             => xmip_subscribe(bit, callback)\n"
	"\n"
	"set appmod MASK             => set target PRCM appmod line to 1\n"
	"clr appmod MASK             => set target PRCM appmod line to 1\n"
	"\n"
	"All \"callback\"s subscribed here only generate printk() logs.\n";

static int dbg_test_print(struct seq_file *s, void *p)
{
	return seq_printf(s, "%s", xmip_dbg_test_help);
}

static void dbg_test_ipc_ready_hdl(void)
{
	dev_warn(xmip_dev->dev, "XMIP-test: IPC_READY notification\n");
}
static void dbg_test_caif_ready_hdl(bool ready)
{
	dev_warn(xmip_dev->dev, "XMIP-test: CAIF_READY with %s notification\n",
			(ready) ? "True" : "FALSE");
}
static void dbg_test_errhandler(int err)
{
	dev_warn(xmip_dev->dev, "XMIP-test: errhandler(%d)  %s\n", err,
			(err == -EINVAL) ? "inval" :
			(err == -ENODEV) ? "nodev" :
			(err == -ETIMEDOUT) ? "timedout" :
			(err == -ERESTARTSYS) ? "restartsys" : "");
}
static void dbg_test_getter_hld(void *data)
{
	dev_warn(xmip_dev->dev, "XMIP-test: bit_set callback for bit %d\n",
			(int) data);
}

static ssize_t dbg_test_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buffer[32], *buf = (char *) buffer, *buf2;
	int bsize, val1, val2, ret = 0;

	/* Get userspace string and assure termination */
	bsize = min(count, (sizeof(buffer) - 1));
	if (copy_from_user(buf, user_buf, bsize))
		return -EFAULT;
	buf[bsize] = 0;

	if (strncmp(buf, "errhandler", 10) == 0) {
		xmip_register_errhandler(dbg_test_errhandler);
	} else if (strncmp(buf, "ipc_ready", 9) == 0) {
		buf += 9;
		val1 = simple_strtoul(buf, &buf, 16);
		ret = xmip_set_shm_addr(val1, dbg_test_ipc_ready_hdl);
	} else if (strncmp(buf, "caif_ready", 10) == 0) {
		ret = xmip_subscribe_caif_ready(dbg_test_caif_ready_hdl);
	} else if (strncmp(buf, "bit_alloc", 9) == 0) {
		buf += 9;
		val1 = simple_strtoul(buf, &buf, 16);
		val2 = simple_strtoul(buf, &buf, 16);
		ret = xmip_bit_alloc(val1, val2);
	} else if (strncmp(buf, "set_bit", 7) == 0) {
		buf += 7;
		val1 = simple_strtoul(buf, &buf, 10);
		ret = xmip_set_bit(val1);
	} else if (strncmp(buf, "reset", 5) == 0) {
		xmip_reset();
	} else if (strncmp(buf, "subscribe", 9) == 0) {
		buf += 9;
		val1 = simple_strtoul(buf, &buf, 10);
		ret = xmip_subscribe(val1, dbg_test_getter_hld, (void *) val1);
	} else if ((strncmp(buf, "set appmod", 10) == 0) ||
			(strncmp(buf, "clr appmod", 10) == 0)) {
		buf2 = buf + 10;
		val1 = simple_strtoul(buf2, &buf2, 16);
		if (strncmp(buf, "set", 3) == 0)
			xmip_set_appmod(val1, xmip_dev->base);
		else
			xmip_clear_appmod(val1, xmip_dev->base);
	} else {
		ret = -EINVAL;
	}

	if (ret < 0)
		return ret;
	return bsize;
}

static int dbg_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_test_print, inode->i_private);
}

static const struct file_operations dbg_test_fops = {
	.open = dbg_test_open,
	.read = seq_read,
	.write = dbg_test_write,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int ste_xmip_debugfs_create(struct ste_xmip *xmip)
{
	int ret = -ENOMEM;

	xmip->dbgdir = debugfs_create_dir("xmip", NULL);
	if (xmip->dbgdir == NULL)
		return -ENOMEM;
	if (!debugfs_create_file("status", S_IRUGO,
				xmip->dbgdir, xmip, &dbg_status_fops))
		goto err;
	if (!debugfs_create_file("test", S_IRUGO | S_IWUSR,
				xmip->dbgdir, xmip, &dbg_test_fops))
		goto err;
	return 0;
err:
	if (xmip->dbgdir) {
		debugfs_remove_recursive(xmip->dbgdir);
		xmip->dbgdir = NULL;
	}
	return ret;
}
#else
static int ste_xmip_debugfs_create(struct ste_xmip *xmip)
{
	return 0;
}
#endif

static void ste_xmip_clear(struct ste_xmip *xmip)
{
	/*
	 * need to abort all pending works !
	 * set 'reset_flag', wake waitqueue and flush target workqueue.
	 */
	xmip->reset_flag = 1;
	xmip->protection_event = 1;
	wake_up_interruptible_all(&xmip->waitq);
	flush_scheduled_work();

	genio_unregister_driver(&ste_xmip_genio_apis);
	prcmu_unregister_modem("xmip");
	upap_unregister_notifier(UPAP_NFYID_XMIP_MODAPP, &xmip_modapp_nb);
	upap_unregister_notifier(UPAP_NFYID_XMIP_RESOUT2N, &resout2n_nb);

	if (xmip->dbgdir)
		debugfs_remove_recursive(xmip->dbgdir);

	xmip->base = NULL;
}

/* ste_xmip_remove - free XMIP ressources */
static int __devexit ste_xmip_remove(struct platform_device *pdev)
{
	struct ste_xmip *xmip = platform_get_drvdata(pdev);
	if (xmip == NULL)
		xmip = xmip_dev;
	if (xmip == NULL)
		return 0;

	mutex_lock(&xmip_dev->lock);
	ste_xmip_clear(xmip);
	mutex_unlock(&xmip_dev->lock);
	xmip_dev = NULL;
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/* ste_xmip_probe - probe XMIP ressources */
static int ste_xmip_probe(struct platform_device *pdev)
{
	struct ste_xmip *xmip, *xmip_alloc;
	struct resource *res;
	int ret = 0, i;
	int align = XMIP_STRUCT_ALIGN;

	if (xmip_dev)
		return -EBUSY;

	if (READY_FOR_IPC_BIT != 29) {
		dev_err(&pdev->dev, "%d unsupported READY_FOR_IPC_BIT value.\n",
				READY_FOR_IPC_BIT);
		return -EINVAL;
	}

	/* alocate xmip structure, with 64byte address alignement */
	xmip = devm_kzalloc(&pdev->dev, sizeof(*xmip) + align, GFP_KERNEL);
	if (xmip == NULL)
		return -ENOMEM;

	xmip_alloc = (struct ste_xmip *)PTR_ALIGN(xmip, align);
	xmip = (struct ste_xmip *) xmip_alloc;

	/* init xmip structure */
	xmip->dev = &pdev->dev;
	xmip->pdev = pdev;

	mutex_init(&xmip->lock);

	/* init modapp irq, -1 is no irq requested */
	for (i = XMIP_IPCHW_LINE_NB - 1; i >= 0; i--)
		xmip->ipchw[i].irq = -1;

	INIT_WORK(&xmip->ipc_ready_work, ipc_ready_work);
	INIT_WORK(&xmip->protection_work, protection_work);

	init_waitqueue_head(&xmip->waitq);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "PRCM_REGS");
	if (!res) {
		ret = -ENODEV;
		goto err;
	}
	xmip->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (xmip->base == NULL) {
		ret = -EIO;
		goto err;
	}

	ret = ste_xmip_debugfs_create(xmip);
	if (ret)
		goto err;

	/* GIC do not treat modapplines as IRQs */
	xmip_mask_modapp(MODAPP_MASK_ALL, xmip->base);

	ret = genio_register_driver(&ste_xmip_genio_apis);
	if (ret)
		goto err;

	/* register PRCMU fmw driver services */
	ret = prcmu_register_modem("xmip");
	if (ret)
		goto err;

	ret = upap_register_notifier(UPAP_NFYID_XMIP_MODAPP, &xmip_modapp_nb);
	if (ret)
		goto err;

	ret = upap_register_notifier(UPAP_NFYID_XMIP_RESOUT2N, &resout2n_nb);
	if (ret)
		goto err;

	xmip_dev = xmip;
	platform_set_drvdata(pdev, xmip);

	dev_info(xmip_dev->dev, "probed\n");
	return ret;
err:
	if (xmip)
		ste_xmip_clear(xmip);

	return ret;
}

static struct platform_driver ste_xmip_driver = {
	.driver = {
		.name = "ste-xmip",
		.owner = THIS_MODULE,
	},
	.probe	= ste_xmip_probe,
	.remove	= __devexit_p(ste_xmip_remove)
};

static int __init ste_xmip_init(void)
{
	int ret;
	ret = platform_driver_register(&ste_xmip_driver);
	if (ret)
		printk(KERN_ERR "XMIP plat-drv registation failed %d.\n", ret);
	return ret;
}

static void __exit ste_xmip_exit(void)
{
	platform_driver_unregister(&ste_xmip_driver);
}

module_init(ste_xmip_init);
module_exit(ste_xmip_exit);

MODULE_AUTHOR("ST-Ericsson");
MODULE_DESCRIPTION("XMIP IPC driver for STE Modem/APE IPC control");
MODULE_LICENSE("GPL v2");
