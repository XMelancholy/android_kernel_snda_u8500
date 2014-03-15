/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Etienne Carriere <etienne.carriere@stericsson.com>
 * License terms:  GNU General Public License (GPL), version 2
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
#include <linux/platform_data/ap9540_c2c_plat.h>
#include <linux/notifier.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/db8500-regs.h>

#include "ap9540_c2c.h"

/* global to access c2c platform device data */
struct ap9540_c2c *c2c_dev;
static int c2c_dev_align; /*  quick assurance of 64byte alignement */

/*
 * utilitaries: marcos for PRCM registers on GENI/GENO lines
 * m is a bit mask, base is PRCM register base address
 */
#define MASK_GENO(m, base)	writel(m, base + PRCMOFF_A9_C2C_GENO_MASK_SET)
#define UNMASK_GENO(m, base)	writel(m, base + PRCMOFF_A9_C2C_GENO_MASK_CLR)
#define READ_GENO(base)	(readl(base + PRCMOFF_C2C_SSCM_GENO))
#define SET_GENI(m, base)	writel(m, base + PRCMOFF_C2C_SSCM_GENI_SET)
#define CLEAR_GENI(m, base)	writel(m, base + PRCMOFF_C2C_SSCM_GENI_CLR)

/* power stuf */
static inline void request_c2c_wakeup(bool on)
{
	if (unlikely(!on && c2c_dev->no_power_down))
		return;
	if (!on)
		c2c_dev->pwr_is_on = 0;
	gpio_set_value(c2c_dev->wumod_gpio, (on) ? 1 : 0);
}

static int c2c_powerup_notif(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if (c2c_dev == NULL)
		return -ENODEV;
	c2c_dev->pwr_is_on = 1;
	wake_up_interruptible_all(&c2c_dev->waitq);
	return 0;
}
static struct notifier_block c2c_powerup_nb = {
	.notifier_call = c2c_powerup_notif,
};

static void request_c2c_wakeup_notif(void)
{
	prcmu_c2c_request_notif_up();
}

static int c2c_power_req(int state)
{
	int i;

	c2c_dev->pwr_last_req = state;
	if (state == 0) {
		if (unlikely(!(c2c_dev->init_flags & GENIO_INIT_CAIF_READY)))
			return 0;
		for (i = 0; i < 32; i++) {
			if (unlikely(c2c_dev->genio[i].pending != 0))
				goto off_race;
		}
		if (c2c_dev->powerup_timeout_armed) {
			del_timer(&c2c_dev->powerup_timer);
			c2c_dev->powerup_timeout_armed = 0;
		}
	}
	request_c2c_wakeup(state != 0);
	return 0;
off_race:
	dev_warn(c2c_dev->dev, "power_req(off) while pending "
		"set_bit operations on GENI bits\n");
	return -EPERM;
}

static int c2c_reset_notif(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if (c2c_dev == NULL)
		return -ENODEV;

	dev_warn(c2c_dev->dev, "C2C HW reset from reset request or crash\n");
	if (c2c_dev->errhandler)
		c2c_dev->errhandler(-EIO);
	return 0;
}
static struct notifier_block c2c_reset_nb = {
	.notifier_call = c2c_reset_notif,
};

/* GENO IRQs fast handshake (through GENO line to A9 GIC) */

static int get_linux_geno_a9_irq(int bit,
		irqreturn_t (*hdl)(int irq, void *data))
{
	struct c2c_genio *genio = &c2c_dev->genio[bit];
	char name[32];
	int ret;
	u32 tmp;

	if (c2c_dev == NULL)
		return -ENODEV;
	if ((c2c_dev->init_flags & GENIO_INIT_IPC_READY) == 0) {
		dev_err(c2c_dev->dev, "IPC_READY for received yet\n");
		return -EINVAL;
	}
	tmp = c2c_dev->setter_mask | c2c_dev->getter_mask;
	if (((1 << bit) & tmp) == 0) {
		dev_err(c2c_dev->dev, "bit %d not alloc in setter_mask\n", bit);
		return -EINVAL;
	}
	if (genio->mask) {
		dev_err(c2c_dev->dev, "bit %d already subscribed to\n", bit);
		return -1; /* FIXME: find better errno */
	}
	ret = 0;
	memset(genio, 0, sizeof(*genio));
	genio->mask = 1 << bit;
	genio->bit = bit;
	genio->irq = -1;
	genio->prcm = c2c_dev->prcm_base;

	sprintf(name, "C2C_GENO%d", bit);
	genio->irq = platform_get_irq_byname(c2c_dev->pdev, name);
	if (genio->irq < 0) {
		dev_err(c2c_dev->dev, "Failed to get irq %s\n", name);
		ret = -EINVAL;
		goto err;
	}
	ret = devm_request_irq(c2c_dev->dev, genio->irq, hdl, IRQF_NO_SUSPEND,
			name, genio);
	if (ret) {
		dev_err(c2c_dev->dev, "Failed to request IRQ %s\n", name);
		goto err;
	}
	return 0;
err:
	if (genio)
		memset(genio, 0, sizeof(*genio));
	return ret;
}

/* hardcoded fastpolling timeout threshold: about 1ms */
#define FASTPOLL_TO	0x100000LL
#define FASTPOLL_THR	(0xFFFFFFFFFFFFFFFFLL - FASTPOLL_TO)

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

static irqreturn_t fast_setter_hdl(int irq, void *data)
{
	struct c2c_genio *genio = (struct c2c_genio *) data;

	MASK_GENO(genio->mask, genio->prcm);
	/* 'pending=0' must be loaded before 'pwr_is_on' is set in order to
	 * prevent false detection of 'pending handshake while C2C is on'.
	 */
	genio->pending = 0;
	smp_wmb();
	c2c_dev->pwr_is_on = 1;
	CLEAR_GENI(genio->mask, genio->prcm);
#ifdef CONFIG_DEBUG_FS
	genio->hs_cnt++;
#endif
	return IRQ_HANDLED;
}

static irqreturn_t fast_getter_hdl(int irq, void *data)
{
	/* polling protection: here use sched_clock, with 1ms timeout */
	struct c2c_genio *genio = (struct c2c_genio *) data;
	unsigned long long sclk_end;

	if (likely(genio->event_cb))
		genio->event_cb(genio->event_data);

	SET_GENI(genio->mask, genio->prcm);
	fastpoll_timeout_init(&sclk_end);
	while ((READ_GENO(genio->prcm) & genio->mask) != 0) {
		if (unlikely(fastpoll_timeout_check(&sclk_end)))
			goto poll_err;
	}
	CLEAR_GENI(genio->mask, genio->prcm);
#ifdef CONFIG_DEBUG_FS
	genio->hs_cnt++;
#endif
	return IRQ_HANDLED;
poll_err:
	genio->poll_timeout = 1;
	c2c_dev->protection_event = 1;
	wake_up_interruptible_all(&c2c_dev->waitq);
	return IRQ_HANDLED;
}

static int c2c_subscribe(int bit, void (*bit_set_cb)(void *data), void *data)
{
	struct c2c_genio *genio = &c2c_dev->genio[bit];

	dev_dbg(c2c_dev->dev, "subscribe to bit %d\n", bit);

	if ((c2c_dev->getter_mask & (1 << bit)) == 0) {
		dev_err(c2c_dev->dev, "bit %d not alloc in getter_mask\n", bit);
		return -EINVAL;
	}
	if (genio->mask == 0) {
		dev_err(c2c_dev->dev, "bit %d IRQ subscrption error\n", bit);
		return -EINVAL;
	}
	if (genio->event_cb != 0) {
		dev_err(c2c_dev->dev, "bit %d already subscribed to\n", bit);
		return -EINVAL;
	}
	genio->event_cb = bit_set_cb;
	genio->event_data = data;

	smp_wmb();
	UNMASK_GENO(genio->mask, genio->prcm);
	return 0;
}

static int c2c_unsubscribe(int bit)
{
	struct c2c_genio *genio = &c2c_dev->genio[bit];

	dev_dbg(c2c_dev->dev, "unsubscribe from bit %d\n", bit);

	if ((c2c_dev->getter_mask & (1 << bit)) == 0) {
		dev_err(c2c_dev->dev, "bit %d not alloc in getter_mask\n", bit);
		return -EINVAL;
	}

	/* do not mask IT: we always handshake getter GENOs */
	genio->event_cb = NULL;
	smp_wmb();
	genio->event_data = NULL;

	return 0;
}

static int c2c_set_bit(int bit)
{
	struct c2c_genio *genio = &c2c_dev->genio[bit];
	unsigned long long sclk_end;

	if (unlikely((c2c_dev->setter_mask & (1 << bit)) == 0)) {
		dev_err(c2c_dev->dev, "bit %d not alloc in setter_mask\n", bit);
		return -EINVAL;
	}

	if (likely(c2c_dev->pwr_is_on)) {
		fastpoll_timeout_init(&sclk_end);

		while ((READ_GENO(genio->prcm) & genio->mask) != 0) {
			if (unlikely(fastpoll_timeout_check(&sclk_end))) {
				if ((READ_GENO(genio->prcm) & genio->mask) != 0)
					goto poll_err;
				else
					break;
			}
		}
		SET_GENI(genio->mask, genio->prcm);
		while ((READ_GENO(genio->prcm) & genio->mask) == 0) {
			if (unlikely(fastpoll_timeout_check(&sclk_end))) {
				if ((READ_GENO(genio->prcm) & genio->mask) == 0)
					goto poll_err;
				else
					break;
			}
		}
		CLEAR_GENI(genio->mask, genio->prcm);
#ifdef CONFIG_DEBUG_FS
		genio->hs_cnt++;
#endif
	} else {
		MASK_GENO(genio->mask, genio->prcm);
		genio->pending = 1;
		smp_wmb();
		SET_GENI(genio->mask, genio->prcm);
		UNMASK_GENO(genio->mask, genio->prcm);
		if (unlikely(!c2c_dev->pwr_last_req)) {
			dev_err(c2c_dev->dev, "set_bit: force power-on req\n");
			request_c2c_wakeup(true);
		}
		c2c_dev->protection_event = 1;
		wake_up_interruptible_all(&c2c_dev->waitq);
	}
	return 0;

poll_err:
	genio->poll_timeout = 1;
	c2c_dev->protection_event = 1;
	wake_up_interruptible_all(&c2c_dev->waitq);
	return -1;
}

static int c2c_bit_alloc(u32 setter_mask, u32 getter_mask)
{
	struct c2c_genio *genio;
	int bit, ret = 0;

	if (c2c_dev == NULL)
		return -ENODEV;
	if (!(c2c_dev->init_flags & GENIO_INIT_IPC_READY_ACK)) {
		dev_err(c2c_dev->dev, "ipc_ready not reached yet\n");
		return -EINVAL;
	}
	if (c2c_dev->init_flags & GENIO_INIT_BIT_ALLOC) {
		dev_err(c2c_dev->dev, "bit_alloc already called\n");
		return -EINVAL;
	}
	if ((setter_mask & getter_mask) != 0) {
		dev_err(c2c_dev->dev, "setter and getter masks overlap\n");
		return -EINVAL;
	}

	c2c_dev->setter_mask = setter_mask;
	c2c_dev->getter_mask = getter_mask;

	/* no event expected from setter handshake */
	MASK_GENO(setter_mask, c2c_dev->prcm_base);
	/* handshake getters even if not yet subscribed to */
	UNMASK_GENO(getter_mask, c2c_dev->prcm_base);

	genio = c2c_dev->genio;
	for (bit = 0; bit < 32; bit++) {
		if ((1 << bit) & c2c_dev->setter_mask)
			ret = get_linux_geno_a9_irq(bit, fast_setter_hdl);
		if ((1 << bit) & c2c_dev->getter_mask)
			ret = get_linux_geno_a9_irq(bit, fast_getter_hdl);
		if (ret)
			return ret;
		genio++;
	}

	mutex_lock(&c2c_dev->lock);
	c2c_dev->init_flags |= GENIO_INIT_BIT_ALLOC;
	mutex_unlock(&c2c_dev->lock);
	return 0;
}

/*
 * Proctection timeout mechanism
 * if no pending set bit found => no issue (clear prot_timeoutarmed)
 * else call errhandler for -ETIMEDOUT.
 * If pwr_is_on and last pwr_req==power_off => power C2C Off
 */
static void powerup_timer_elapsed(unsigned long data)
{
	struct ap9540_c2c *c2c = (struct ap9540_c2c *) data;
	struct c2c_genio *genio;
	int err, i;

	dev_dbg(c2c->dev, "Powerup protection timeout expired\n");
	for (i = 0, err = 0, genio = c2c->genio; i < 32; i++) {
		if (genio->pending)
			err++;
		genio++;
	}
	if (err) {
		dev_err(c2c->dev, "power-up timeout (%d pending set)\n", err);
		c2c->powerup_timeout = 1;
		c2c_dev->protection_event = 1;
		wake_up_interruptible_all(&c2c_dev->waitq);
	}
	c2c_dev->powerup_timeout_armed = 0;
	if (c2c_dev->pwr_last_req == 0) {
		dev_warn(c2c->dev, "should restore last power-off resquest!\n");
	}
}

static void protection_nfy_powerup_tout(struct ap9540_c2c *c2c, int pendings)
{
	if (c2c->errhandler == NULL) {
		dev_err(c2c->dev, "powerup tout, no errhandler, %d pendings\n",
				pendings);
		return;
	}
	if (c2c->powerup_timeout_nfy == 0) {
		c2c->powerup_timeout_nfy = 1;
		c2c->errhandler(-ETIMEDOUT);
	} else if (c2c->powerup_timeout_nfy < 3) {
		c2c->powerup_timeout_nfy++;
	}
	if (c2c->powerup_timeout_nfy < 2)
		dev_err(c2c->dev, "powerup tout (%d pendings)\n", pendings);
}

static void protection_nfy_tout(struct ap9540_c2c *c2c, struct c2c_genio *genio)
{
	if (c2c->errhandler == NULL) {
		dev_err(c2c->dev, "%d poll tout, no handler\n", genio->bit);
		return;
	}
	if (genio->poll_timeout_nfy == 0) {
		genio->poll_timeout_nfy = 1;
		c2c->errhandler(-ETIMEDOUT);
	} else if (genio->poll_timeout_nfy < 3) {
		genio->poll_timeout_nfy++;
	}
	if (genio->poll_timeout_nfy < 2)
		dev_err(c2c->dev, "%d poll tout (%d)\n", genio->bit,
				genio->poll_timeout_nfy);
}


/*
 * protection_work
 *
 *  Sleep on waitqueue c2c_dev->waitq on event prot_event!=0
 *  If c2c->reset_flag => end work
 *  If genio[i]->poll_timeout => errhandler() and end work.
 *  If c2c->pending_armed => possible pending operations
 *     if (genio[i].pending!=0) arm wakeup timeout
 *  If c2c->pws_is_on => release pending operation timeout (wakeup timeout)
 */
static void protection_work(struct work_struct *work)
{
	struct ap9540_c2c *c2c;
	struct c2c_genio *genio;
	unsigned long exp;
	int ret, i, n;

	c2c = container_of(work, struct ap9540_c2c, protection_work);
	ret = 0;
	while (1) {
		/* sleep until some event occur */
		ret = wait_event_interruptible(c2c->waitq,
				(c2c->protection_event != 0));
		if (ret == -ERESTARTSYS) {
			dev_err(c2c->dev, "unexpected wake of 'prot_work'\n");
			break;
		}
		c2c->protection_event = 0;
		/* genio reset */
		if (c2c->reset_flag) {
			ret = 0;
			break;
		}
		/* polling timeout */
		ret = 0;
		for (i = 0, genio = c2c->genio; i < 32; i++, genio++) {
			if (genio->poll_timeout)
				protection_nfy_tout(c2c, genio);
		}
		/* pending set operation */
		for (i = 0, n = 0, genio = c2c->genio; i < 32; i++, genio++) {
			if (genio->pending)
				n++;
		}
		if (c2c->powerup_timeout)
			protection_nfy_powerup_tout(c2c, n);

		if (n) {
			if (c2c->pwr_is_on) {
				dev_err(c2c->dev, "%d pending op(s) !!!\n", n);
			} else if (c2c->powerup_timeout_armed == 0) {
				c2c->powerup_timeout_armed = 1;
				exp = round_jiffies((c2c->powerup_timeout_ms
						* HZ) / 1000);
				exp += jiffies;
				c2c->powerup_timer.expires = exp;
				add_timer(&c2c->powerup_timer);
			}
		}
		/* C2C wakeup acknowledge */
		if (c2c->pwr_is_on) {
			del_timer(&c2c->powerup_timer);
			c2c->powerup_timeout_armed = 0;
		}
	}
	if ((ret < 0) && c2c->errhandler)
		c2c->errhandler(ret);
}

/* GENO IRQs from C2C IP irq[1] (slow notification) */

static irqreturn_t c2c_irq1_top(int irq, void *data)
{
	struct ap9540_c2c *c2c = (struct ap9540_c2c *) data;
	struct c2c_genio *genio;
	u32 status;

	status = readl(c2c->c2c_base + C2COFF_IRQ_ENABLE_STATUS_1);
	if (c2c->reset_flag) {
		writel(status, c2c->c2c_base + C2COFF_IRQ_ENABLE_STATUS_1);
		writel(status, c2c->c2c_base + C2COFF_IRQ_ENABLE_CLEAR_1);
		return IRQ_HANDLED;
	}
	if (status & (~c2c->irq1_mask)) { /* unexpeted irq(s) */
		writel(status & (~c2c->irq1_mask),
				c2c->c2c_base + C2COFF_IRQ_ENABLE_STATUS_1);
		status &= ~c2c->irq1_mask;
	}
	if ((status & c2c->irq1_mask) == 0)
		return IRQ_NONE;

	genio = &c2c_dev->genio[31];
	while (status & c2c->irq1_mask) {
		if (status & genio->mask) {
			genio->trigged = 1;
			writel(genio->mask, c2c->c2c_base +
					C2COFF_IRQ_ENABLE_STATUS_1);
			genio->level = (genio->mask &
					READ_GENO(genio->prcm)) ? 1 : 0;
			status &= ~genio->mask;
		}
		genio--;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t c2c_irq1_bottom(int irq, void *data)
{
	struct c2c_genio *genio;
	int i;

	genio = ((struct ap9540_c2c *) data)->genio;
	for (i = 0; i < 32; i++, genio++) {
		if (genio->trigged) {
			if (genio->event_cb)
				genio->event_cb(genio->event_data);
			genio->trigged = 0;
		}
	}
	return IRQ_HANDLED;
}

/* ipc_ready management */

static void ipc_ready_hdl(void *data)
{
	struct c2c_genio *genio = (struct c2c_genio *) data;

	mutex_lock(&c2c_dev->lock);
	if (genio->level)
		c2c_dev->init_flags |= GENIO_INIT_IPC_READY;
	else
		c2c_dev->init_flags |= GENIO_INIT_IPC_READY_ACK;
	mutex_unlock(&c2c_dev->lock);

	wake_up_interruptible_all(&c2c_dev->waitq);
}

static void ipc_ready_work(struct work_struct *work)
{
	struct ap9540_c2c *c2c = container_of(work,
		struct ap9540_c2c, ipc_ready_work);
	u32	mask = 1 << READY_FOR_IPC_BIT, tmp;
	int ret = 0;

	if (c2c->reset_flag)
		return;

	dev_dbg(c2c->dev, "ipc_ready work\n");

	request_c2c_wakeup(true);
	request_c2c_wakeup_notif();
	ret = wait_event_interruptible(c2c->waitq,
			(c2c->pwr_is_on || c2c->reset_flag));
	if ((ret == -ERESTARTSYS) || c2c->reset_flag)
		goto err;
	dev_info(c2c->dev, "ipc_ready C2C powerup\n");

	/* config C2C IRQ[1] for GENO[IPC_READY] events */
	mutex_lock(&c2c->lock);
	tmp = readl(c2c->c2c_base + C2COFF_GENO_INTERRUPT) & (~mask);
	writel(tmp, c2c->c2c_base + C2COFF_GENO_INTERRUPT);
	mutex_unlock(&c2c->lock);
	writel(mask, c2c->c2c_base + C2COFF_IRQ_ENABLE_SET_1);

	/* case GENO already set to 1 */
	if ((readl(c2c->c2c_base + C2COFF_GENO_STATUS) & mask)) {
		mutex_lock(&c2c->lock);
		c2c->init_flags |= GENIO_INIT_IPC_READY;
		mutex_unlock(&c2c->lock);
	}

	ret = wait_event_interruptible(c2c->waitq, (c2c->reset_flag ||
		(c2c->init_flags & GENIO_INIT_IPC_READY)));
	if ((ret == -ERESTARTSYS) || c2c->reset_flag)
		goto err;
	dev_dbg(c2c->dev, "ipc_ready raise detected\n");

	CLEAR_GENI(0xFFFFFFFF, c2c->prcm_base);

	ret = wait_event_interruptible(c2c->waitq, (c2c->reset_flag ||
			(c2c->init_flags & GENIO_INIT_IPC_READY_ACK)));
	if ((ret == -ERESTARTSYS) || c2c->reset_flag)
		goto err;
	dev_info(c2c->dev, "ipc_ready handshake completed\n");

	writel(mask, c2c->c2c_base + C2COFF_IRQ_ENABLE_CLEAR_1);

	if (c2c->ipc_ready_cb)
		c2c->ipc_ready_cb();

	dev_dbg(c2c->dev, "ipc_ready completed\n");
	return;
err:
	dev_err(c2c->dev, "ipc_ready failure\n");
	if ((ret < 0) && c2c->errhandler)
		c2c->errhandler(ret);
}

static int c2c_set_shm_addr(u32 addr, void (*ipc_ready_cb) (void))
{
	struct c2c_genio *genio;

	if (c2c_dev == NULL)
		return -ENODEV;
	if (c2c_dev->init_flags & GENIO_INIT_IPC_READY_REQ)
		return -EINVAL;

	genio = &c2c_dev->genio[READY_FOR_IPC_BIT];

	dev_dbg(c2c_dev->dev, "genio_set_shm_addr start\n");
	c2c_dev->ipc_ready_cb = ipc_ready_cb;
	c2c_dev->init_flags = GENIO_INIT_IPC_READY_REQ;

	/* write boot addr */
	CLEAR_GENI(0xFFFFFFFF, c2c_dev->prcm_base);
	SET_GENI(addr, c2c_dev->prcm_base);

	/* prepare IRQ handling */
	memset(genio, 0, sizeof(*genio));
	genio->mask = 1 << READY_FOR_IPC_BIT;
	genio->prcm = c2c_dev->prcm_base;
	genio->event_cb = ipc_ready_hdl;
	genio->event_data = genio;
	genio->irq = -1;
	mutex_lock(&c2c_dev->lock);
	c2c_dev->irq1_mask |= 1 << READY_FOR_IPC_BIT;
	mutex_unlock(&c2c_dev->lock);

	schedule_work(&c2c_dev->ipc_ready_work);
	schedule_work(&c2c_dev->protection_work);
	return 0;
}

/* caif_ready management */

static void caif_ready_hdl(void *data)
{
	struct c2c_genio *genio = (struct c2c_genio *) data;
	u32 tmp = 0, mask;

	dev_info(c2c_dev->dev, "CAIF_READY bit switched to %d\n", genio->level);

	if (genio->level) {
		mutex_lock(&c2c_dev->lock);
		mask = 1 << READY_FOR_CAIF_BIT;
		/* reconfig C2C to detect only CAIF_READY falling edge */
		tmp = readl(c2c_dev->c2c_base + C2COFF_GENO_INTERRUPT);
		writel(tmp | mask, c2c_dev->c2c_base + C2COFF_GENO_INTERRUPT);
		tmp = readl(c2c_dev->c2c_base + C2COFF_GENO_LEVEL);
		writel(tmp & ~mask, c2c_dev->c2c_base + C2COFF_GENO_LEVEL);
		tmp = READ_GENO(c2c_dev->prcm_base) & mask;
		smp_wmb(); /* updated IP before WU_MOD can go down */
		/* allow WU_MOD to fall (and C2C to sleep) */
		c2c_dev->init_flags |= GENIO_INIT_CAIF_READY;
		mutex_unlock(&c2c_dev->lock);
	}
	if (c2c_dev->caif_ready_cb)
		c2c_dev->caif_ready_cb((genio->level != 0));

	/* case CAIF_READY felt before IRQ was configured */
	if (genio->level && (tmp == 0)) {
		dev_info(c2c_dev->dev, "CAIF_READY bit switched back to 0\n");
		if (c2c_dev->caif_ready_cb)
			c2c_dev->caif_ready_cb((0));
	}
}

static int caif_ready_subscribe(struct ap9540_c2c *c2c)
{
	struct c2c_genio *genio;
	u32	mask = 1 << READY_FOR_CAIF_BIT, tmp;
	int ret = 0;

	if (c2c->pwr_is_on == 0) {
		request_c2c_wakeup(true);
		request_c2c_wakeup_notif();
	}
	ret = wait_event_interruptible(c2c->waitq,
			(c2c->pwr_is_on || c2c->reset_flag));
	if ((ret == -ERESTARTSYS) || (c2c->reset_flag))
		goto err;
	dev_dbg(c2c->dev, "caif_ready: C2C wakeup\n");

	/* config C2C IRQ[1] for GENO[CAIF_READY] events */
	genio = &c2c->genio[READY_FOR_CAIF_BIT];
	memset(genio, 0, sizeof(*genio));
	genio->mask = mask;
	genio->prcm = c2c->prcm_base;
	genio->event_cb = caif_ready_hdl;
	genio->event_data = genio;
	genio->irq = -1;

	mutex_lock(&c2c->lock);
	c2c->irq1_mask |= mask;
	tmp = readl(c2c->c2c_base + C2COFF_GENO_INTERRUPT) & (~mask);
	writel(tmp, c2c->c2c_base + C2COFF_GENO_INTERRUPT);
	mutex_unlock(&c2c->lock);
	writel(mask, c2c->c2c_base + C2COFF_IRQ_ENABLE_SET_1);

	/* case CAIF_READY is already set */
	if ((READ_GENO(c2c->prcm_base) & mask) && c2c->caif_ready_cb) {
		mutex_lock(&c2c->lock);
		c2c->init_flags |= GENIO_INIT_CAIF_READY;
		mutex_unlock(&c2c->lock);
		c2c->caif_ready_cb(true);
	}

	mutex_lock(&c2c->lock);
	c2c->init_flags |= GENIO_INIT_CAIF_READY_REQ;
	mutex_unlock(&c2c->lock);

	dev_dbg(c2c->dev, "caif_ready subscription completed\n");
	return 0;
err:
	dev_err(c2c->dev, "caif_ready subscription failure\n");
	if ((ret < 0) && c2c->errhandler)
		c2c->errhandler(ret);
	return ret;
}

static void caif_ready_work(struct work_struct *work)
{
	struct ap9540_c2c *c2c = container_of(work,
		struct ap9540_c2c, caif_ready_work);

	if (c2c->reset_flag)
		return;
	caif_ready_subscribe(c2c);
}

static int c2c_subscribe_caif_ready(void (*caif_ready_cb)(bool ready))
{
	if (c2c_dev == NULL)
		return -ENODEV;
	if (c2c_dev->init_flags & GENIO_INIT_CAIF_READY)
		return -EINVAL;

	dev_dbg(c2c_dev->dev, "caif_ready called\n");
	c2c_dev->caif_ready_cb = caif_ready_cb;
	schedule_work(&c2c_dev->caif_ready_work);
	return 0;
}

static void c2c_register_errhandler(void (*errhandler)(int errno))
{
	if (c2c_dev == NULL)
		return;
	dev_dbg(c2c_dev->dev, "registering errhandler\n");
	c2c_dev->errhandler = errhandler;
}

static int c2c_reset(void)
{
	struct c2c_genio *genio;
	int i;

	if (c2c_dev == NULL)
		return 0;

	dev_info(c2c_dev->dev, "processing reset\n");
	mutex_lock(&c2c_dev->lock);

	MASK_GENO(0xFFFFFFFF, c2c_dev->prcm_base);

	prcmu_c2c_request_reset();

	c2c_dev->init_flags = 0;
	c2c_dev->reset_flag = 1;
	genio = c2c_dev->genio;
	for (i =  0; i < 32; i++, genio++) {
		if (genio->mask && genio->irq)
			free_irq(genio->irq, genio);
		memset(genio, 0, sizeof(*genio));
	}
	c2c_dev->setter_mask = 0;
	c2c_dev->getter_mask = 0;
	c2c_dev->irq1_mask = 0;
	request_c2c_wakeup(false);
	c2c_dev->pwr_is_on = 0;
	c2c_dev->pwr_last_req = 0;
	del_timer(&c2c_dev->powerup_timer);
	c2c_dev->powerup_timeout_armed = 0;
	c2c_dev->ipc_ready_cb = NULL;
	c2c_dev->caif_ready_cb = NULL;
	c2c_dev->errhandler = NULL;

	c2c_dev->protection_event = 1;
	wake_up_interruptible_all(&c2c_dev->waitq);
	flush_scheduled_work();

	c2c_dev->reset_flag = 0;

	mutex_unlock(&c2c_dev->lock);
	return 0;
}

/* APIs to be registered to geion_wrapper */
static struct genio_wrap ap9540_c2c_genio_apis = {
	.drvname = "c2c",
	.set_shm_addr = c2c_set_shm_addr,
	.subscribe_caif_ready = c2c_subscribe_caif_ready,
	.register_errhandler = c2c_register_errhandler,
	.bit_alloc = c2c_bit_alloc,
	.subscribe = c2c_subscribe,
	.unsubscribe = c2c_unsubscribe,
	.set_bit = c2c_set_bit,
	.power_req = c2c_power_req,
	.reset = c2c_reset,
};

#ifdef CONFIG_DEBUG_FS

/* FIXME: there must be some linux std to do that */
#define TMP_SKIP_SPACES(buf) { \
	while ((*buf != 0) && (*buf == ' ' || *buf == '\t')) \
		buf++; \
	if (*buf == 0) \
		return -EINVAL; \
	}

static int dbg_status_print(struct seq_file *s, void *p)
{
	struct ap9540_c2c *c2c = (struct ap9540_c2c *) s->private;
	struct c2c_genio *genio;
	int i , j;

	if (c2c == NULL) {
		seq_printf(s, "No C2C device avaiable\n");
		return 0;
	}
	seq_printf(s, "Subscribed GENI/GENOs:\n");
	seq_printf(s, "- setter 0x%08X    getter 0x%08X   irq1 0x%08X\n",
			c2c->setter_mask, c2c->getter_mask, c2c->irq1_mask);
	;
	for (i = 0, j = 0, genio = c2c->genio; i < 31; i++, genio++) {
		if (genio->mask == 0)
			continue;
		seq_printf(s, "- bit %02d : %s, timeout:%d, pending:%d, "
				"event_cb:%p, cnt:%d\n",
			i, (genio->mask & c2c->setter_mask) ? "Fast-Setter" :
			(genio->mask & c2c->getter_mask) ? "Fast-Getter" :
			(genio->mask & c2c->irq1_mask) ? "Irq1-Getter" :
			"unknown", genio->poll_timeout, genio->pending,
			genio->event_cb, genio->hs_cnt);
		j++;
	}
	if (j == 0)
		seq_printf(s, "- no pending set bit, no subscribed bit.\n");
	seq_printf(s, "\n");
	seq_printf(s, "Powerup timeout: trigged=%s armed=%s, pending=%s, "
			"timer-ms=%d\n", (c2c->powerup_timeout) ? "Yes" : "No",
			(c2c->powerup_timeout_armed) ? "Yes" : "No",
			(timer_pending(&c2c->powerup_timer)) ? "Yes" : "No",
			c2c->powerup_timeout_ms);
	seq_printf(s, "Misc: init=%04X reset=%d prot_evt=%d pwr_is_on=%d "
			"pwr_last_req=%d\n", c2c->init_flags, c2c->reset_flag,
			c2c->protection_event, c2c->pwr_is_on,
			c2c->pwr_last_req);
	seq_printf(s, "\n");
	seq_printf(s, "C2C Registers:\n");
	if (c2c->pwr_is_on) {
		seq_printf(s, "-  wake_req %d  wake_ack %d  standby %d  "
				"standby_in %d  wait %d\n",
			readl(c2c_dev->c2c_base + C2COFF_WAKE_REQ),
			readl(c2c_dev->c2c_base + C2COFF_WAKE_ACK),
			readl(c2c_dev->c2c_base + C2COFF_STANDBY),
			readl(c2c_dev->c2c_base + C2COFF_STANDBY_IN),
			readl(c2c_dev->c2c_base + C2COFF_WAIT));
		seq_printf(s, "- fclk_freq %-4d    rx_max %-4d  "
				"tx_max %-4d   rx_max_ack %-4d\n",
			readl(c2c_dev->c2c_base + C2COFF_FCLK_FREQ),
			readl(c2c_dev->c2c_base + C2COFF_RX_MAX_FREQ),
			readl(c2c_dev->c2c_base + C2COFF_TX_MAX_FREQ),
			readl(c2c_dev->c2c_base + C2COFF_RX_MAX_FREQ_ACK));
		seq_printf(s, "- portconfig   0x%04X       mirrormode %d\n",
			readl(c2c_dev->c2c_base + C2COFF_PORTCONFIG),
			readl(c2c_dev->c2c_base + C2COFF_MIRRORMODE));
		seq_printf(s, "- irq_raw_st_0 0x%08X   irq_raw_st_1 0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_IRQ_RAW_STATUS_0),
			readl(c2c_dev->c2c_base + C2COFF_IRQ_RAW_STATUS_1));
		seq_printf(s, "- irq_status_0 0x%08X   irq_status_1 0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_STATUS_0),
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_STATUS_1));
		seq_printf(s, "- irq_set_0    0x%08X   irq_set_1    0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_SET_0),
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_SET_1));
		seq_printf(s, "- irq_clear_0  0x%08X   irq_clear_1  0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_CLEAR_0),
			readl(c2c_dev->c2c_base + C2COFF_IRQ_ENABLE_CLEAR_1));
		seq_printf(s, "- geni_control 0x%08X   geni_mask    0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_GENI_CONTROL),
			readl(c2c_dev->c2c_base + C2COFF_GENI_MASK));
		seq_printf(s, "- geno_status  0x%08X   geno_interr. 0x%08X   "
			"geno_level 0x%08X\n",
			readl(c2c_dev->c2c_base + C2COFF_GENO_STATUS),
			readl(c2c_dev->c2c_base + C2COFF_GENO_INTERRUPT),
			readl(c2c_dev->c2c_base + C2COFF_GENO_LEVEL));
	} else {
		seq_printf(s, "- can't access C2C IP: not sure it is 'On'\n");
	}

	seq_printf(s, "\n");
	seq_printf(s, "PRCM Registers\n");
	seq_printf(s, "- a9_geno_mask 0x%08X   geni_val 0x%08X   geno 0x%08X\n",
		readl(c2c_dev->prcm_base + PRCMOFF_A9_C2C_GENO_MASK_VAL),
		readl(c2c_dev->prcm_base + PRCMOFF_C2C_SSCM_GENI_VAL),
		readl(c2c_dev->prcm_base + PRCMOFF_C2C_SSCM_GENO));

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


static int ap9540_c2c_debugfs(struct ap9540_c2c *c2c)
{
	c2c->dbgdir = debugfs_create_dir("c2c", NULL);
	if (c2c->dbgdir == NULL)
		goto err;
	if (!debugfs_create_u32("powerup_timeout_ms", S_IRUSR | S_IWUSR,
				c2c->dbgdir, (u32 *) &c2c->powerup_timeout_ms))
		goto err;
	if (!debugfs_create_file("status", S_IRUGO,
				c2c->dbgdir, c2c, &dbg_status_fops))
		goto err;
	if (!debugfs_create_u32("no_power_down", S_IRUGO | S_IWUSR,
				c2c->dbgdir, (u32 *) &c2c->no_power_down))
		goto err;
	return 0;
err:
	if (c2c->dbgdir) {
		debugfs_remove_recursive(c2c->dbgdir);
		c2c->dbgdir = NULL;
	}
	return -1;
}
#else
static int ap9540_c2c_debugfs(struct ap9540_c2c *c2c)
{
	return 0;
}
#endif

static void ap9540_c2c_clear(struct ap9540_c2c *c2c)
{
	genio_unregister_driver(&ap9540_c2c_genio_apis);
	del_timer(&c2c->powerup_timer);
	c2c->reset_flag = 1;
	c2c->protection_event = 1;
	wake_up_interruptible_all(&c2c->waitq);
	flush_scheduled_work();
	if (c2c->wumod_gpio >= 0)
		gpio_free(c2c->wumod_gpio);
	c2c->wumod_gpio = -1;
	if (c2c->irq1 >= 0)
		free_irq(c2c->irq1, c2c);
	c2c->irq1 = -1;
	if (c2c->dbgdir)
		debugfs_remove_recursive(c2c->dbgdir);
	if (c2c->c2c_base)
		iounmap(c2c->c2c_base);
	if (c2c->prcm_base)
		iounmap(c2c->prcm_base);
	c2c->c2c_base = NULL;
	c2c->prcm_base = NULL;

	prcmu_unregister_modem("c2c");
	upap_unregister_notifier(UPAP_NFYID_C2C_NOTIF, &c2c_powerup_nb);

	ap9540_c2c_deep_debug_exit();
}

/* ap9540_c2c_remove - free C2C ressources */
static int __devexit ap9540_c2c_remove(struct platform_device *pdev)
{
	struct ap9540_c2c *c2c = platform_get_drvdata(pdev);
	if (c2c == NULL)
		c2c = c2c_dev;
	if (c2c == NULL)
		return 0;

	mutex_lock(&c2c_dev->lock);
	ap9540_c2c_clear(c2c);
	mutex_unlock(&c2c_dev->lock);
	kfree(((void *) c2c) - c2c_dev_align);
	c2c_dev_align = 0;
	c2c_dev = NULL;
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/* ap9540_c2c_probe - probe C2C ressources */
static int ap9540_c2c_probe(struct platform_device *pdev)
{
	struct c2c_platform_data *pdata = pdev->dev.platform_data;
	struct ap9540_c2c *c2c, *c2c_alloc;
	struct resource *res;
	int ret = 0, i;
	int align = 0x3f;

	if (c2c_dev)
		return -EBUSY;

	/* alocate c2c structure, with 64byte address alignement */
	c2c = kzalloc(sizeof(*c2c) + align, GFP_KERNEL);
	if (c2c == NULL)
		return -ENOMEM;

	c2c_alloc =\
		(struct ap9540_c2c *) PTR_ALIGN((struct ap9540_c2c *)c2c, align);
	c2c_dev_align = (int)(c2c_alloc - c2c);
	c2c = (struct ap9540_c2c *) c2c_alloc;

	/* init c2c structure */
	c2c->dev = &pdev->dev;
	c2c->pdev = pdev;

	mutex_init(&c2c->lock);
	for (i = 31; i >= 0; i--)
		c2c->genio[i].irq = -1;
	c2c->irq1 = -1;
	c2c->wumod_gpio = -1;

	INIT_WORK(&c2c->ipc_ready_work, ipc_ready_work);
	INIT_WORK(&c2c->caif_ready_work, caif_ready_work);
	INIT_WORK(&c2c->protection_work, protection_work);

	init_timer(&c2c->powerup_timer);
	c2c->powerup_timeout_ms = 1000;
	c2c->powerup_timer.function = powerup_timer_elapsed;
	c2c->powerup_timer.data = (unsigned long) c2c;

	init_waitqueue_head(&c2c->waitq);

	/* PRCMU and C2C registers base adresses */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "C2C_REGS");
	if (!res) {
		ret = -ENODEV;
		goto err;
	}
	c2c->c2c_base = ioremap(res->start,
			res->end - res->start + 1);
	if (c2c->c2c_base == NULL) {
		ret = -EIO;
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "PRCMU_REGS");
	if (!res) {
		ret = -ENODEV;
		goto err;
	}
	c2c->prcm_base = ioremap(res->start,
			res->end - res->start + 1);
	if (c2c->prcm_base == NULL) {
		ret = -EIO;
		goto err;
	}

	/* GPIO WU_MOD */
	if (!pdata) {
		dev_err(&pdev->dev, "invalid c2c platform data\n");
		goto err;
	}
	c2c->wumod_gpio = pdata->wumod_gpio;
	ret = gpio_request(c2c->wumod_gpio, "WU_MOD");
	if (ret < 0) {
		c2c->wumod_gpio = -1;
		goto err;
	}
	ret = gpio_direction_output(c2c->wumod_gpio, 1);
	if (ret < 0)
		goto err;

	/* C2C IRQ[1], required for IPC_READY/CAIF_READY */
	c2c->irq1 = platform_get_irq_byname(c2c->pdev, "C2C_IRQ1");
	if (c2c->irq1 < 0) {
		ret = -ENODEV;
		goto err;
	}
	ret = devm_request_threaded_irq(c2c->dev, c2c->irq1, c2c_irq1_top,
			c2c_irq1_bottom, IRQF_NO_SUSPEND, "C2C_IRQ1", c2c);
	if (ret)
		goto err;

	ret = ap9540_c2c_debugfs(c2c);
	if (ret)
		goto err;

	ret = ap9540_c2c_deep_debug_init(c2c);
	if (ret)
		goto err;

	/* GIC do not treat GENO lines as IRQs */
	MASK_GENO(0xFFFFFFFF, c2c->prcm_base);

	/* register GENIO driver to genio wrapper */
	ret = genio_register_driver(&ap9540_c2c_genio_apis);
	if (ret)
		goto err;

	/* Register PRCMU fmw driver services */
	prcmu_register_modem("c2c");
	upap_register_notifier(UPAP_NFYID_C2C_NOTIF, &c2c_powerup_nb);
	/*
	 * TODO: register to the several c2c reset/crash notification
	 * once prcmu driver is ready. Single handler: c2c_reset_nb.
	 */
	c2c->no_power_down = 1; /* FIXME: to be removed useless */

	c2c_dev = c2c;
	platform_set_drvdata(pdev, c2c);

	dev_info(c2c_dev->dev, "probed\n");
	return ret;

err:
	if (c2c) {
		ap9540_c2c_clear(c2c);
		kfree(((void *) c2c) - c2c_dev_align);
		c2c_dev_align = 0;
	}
	return ret;
}

static struct platform_driver ap9540_c2c_driver = {
	.driver = {
		.name = "c2c",
		.owner = THIS_MODULE,
	},
	.probe	= ap9540_c2c_probe,
	.remove	= __devexit_p(ap9540_c2c_remove)
};

static int __init ap9540_c2c_init(void)
{
	int ret;
	ret = platform_driver_register(&ap9540_c2c_driver);
	if (ret)
		printk(KERN_ERR "C2C plat-drv registation failed %d.\n", ret);
	return ret;
}

static void __exit ap9540_c2c_exit(void)
{
	platform_driver_unregister(&ap9540_c2c_driver);
}

/*arch_initcall(ap9540_c2c_init);*/
module_init(ap9540_c2c_init);
module_exit(ap9540_c2c_exit);

MODULE_AUTHOR("ST-Ericsson");
MODULE_DESCRIPTION("C2C driver for AP9540 Modem boot and IPC");
MODULE_LICENSE("GPL v2");
