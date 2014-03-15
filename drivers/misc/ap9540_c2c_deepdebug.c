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
#include <linux/mfd/db8500-prcmu.h>
#include <linux/ux500_deepdebug.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/db8500-regs.h>

#include "ap9540_c2c.h"

static struct ddbg_register ddbg_c2c_registers[] = {
	DDBG_REG("C2C_REVISION", C2COFF_REVISION, DDBG_RO),
	DDBG_REG("C2C_SYSCONFIG", C2COFF_SYSCONFIG, DDBG_RW),
	DDBG_REG("C2C_SYSSTATUS", C2COFF_SYSSTATUS, DDBG_RO),
	DDBG_REG("C2C_PORTCONFIG", C2COFF_PORTCONFIG, DDBG_RW),
	DDBG_REG("C2C_MIRRORMODE", C2COFF_MIRRORMODE, DDBG_RW),
	DDBG_REG("C2C_IRQ_RAW_STATUS_0", C2COFF_IRQ_RAW_STATUS_0, DDBG_RW),
	DDBG_REG("C2C_IRQ_RAW_STATUS_1", C2COFF_IRQ_RAW_STATUS_1, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_STS_0", C2COFF_IRQ_ENABLE_STATUS_0, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_STS_1", C2COFF_IRQ_ENABLE_STATUS_1, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_SET_0", C2COFF_IRQ_ENABLE_SET_0, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_SET_1", C2COFF_IRQ_ENABLE_SET_1, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_CLEAR_0", C2COFF_IRQ_ENABLE_CLEAR_0, DDBG_RW),
	DDBG_REG("C2C_IRQ_ENABLE_CLEAR_1", C2COFF_IRQ_ENABLE_CLEAR_1, DDBG_RW),
	DDBG_REG("C2C_IRQ_EOI", C2COFF_IRQ_EOI, DDBG_RW),
	DDBG_REG("C2C_FCLK_FREQ", C2COFF_FCLK_FREQ, DDBG_RW),
	DDBG_REG("C2C_RX_MAX_FREQ", C2COFF_RX_MAX_FREQ, DDBG_RW),
	DDBG_REG("C2C_TX_MAX_FREQ", C2COFF_TX_MAX_FREQ, DDBG_RW),
	DDBG_REG("C2C_RX_MAX_FREQ_ACK", C2COFF_RX_MAX_FREQ_ACK, DDBG_RW),
	DDBG_REG("C2C_WAKE_REQ", C2COFF_WAKE_REQ, DDBG_RW),
	DDBG_REG("C2C_WAKE_ACK", C2COFF_WAKE_ACK, DDBG_RW),
	DDBG_REG("C2C_STANDBY", C2COFF_STANDBY, DDBG_RW),
	DDBG_REG("C2C_STANDBY_IN", C2COFF_STANDBY_IN, DDBG_RW),
	DDBG_REG("C2C_WAIT", C2COFF_WAIT, DDBG_RW),
	DDBG_REG("C2C_GENI_CONTROL", C2COFF_GENI_CONTROL, DDBG_RW),
	DDBG_REG("C2C_GENI_MASK", C2COFF_GENI_MASK, DDBG_RW),
	DDBG_REG("C2C_GENO_STATUS", C2COFF_GENO_STATUS, DDBG_RW),
	DDBG_REG("C2C_GENO_INTERRUPT", C2COFF_GENO_INTERRUPT, DDBG_RW),
	DDBG_REG("C2C_GENO_LEVEL", C2COFF_GENO_LEVEL, DDBG_RW),
	DDBG_REG_NULL,
};

static int ddbg_c2c_write(struct ddbg_target *ddbg, u32 val)
{
	struct ap9540_c2c *c2c_dev;
	int ret = 0;

	if (!ddbg) {
		ret = -EINVAL;
		goto out;
	}

	c2c_dev = (struct ap9540_c2c *) ddbg->dev_data;
	if (!c2c_dev) {
		ret = -ENXIO;
		goto out;
	}

	if (!c2c_dev->pwr_is_on) {
		ret = -EIO;
		goto out;
	}

	mutex_lock(&c2c_dev->lock);
	writel(val, c2c_dev->c2c_base + ddbg->curr->offset);
	mutex_unlock(&c2c_dev->lock);

out:
	return ret;
}

static int ddbg_c2c_read(struct ddbg_target *ddbg, u32 *val)
{
	struct ap9540_c2c *c2c_dev;
	int ret = 0;

	if (!ddbg) {
		ret = -EINVAL;
		goto out;
	}

	c2c_dev = (struct ap9540_c2c *) ddbg->dev_data;
	if (!c2c_dev) {
		ret = -ENXIO;
		goto out;
	}

	if (!c2c_dev->pwr_is_on) {
		ret = -EIO;
		goto out;
	}

	mutex_lock(&c2c_dev->lock);
	*val = readl(c2c_dev->c2c_base + ddbg->curr->offset);
	mutex_unlock(&c2c_dev->lock);


out:
	return ret;
}

static struct ddbg_target ddbg_c2c = {
	.name = "c2c",
	.phyaddr = U8500_C2C_BASE,
	.reg = ddbg_c2c_registers,
	.read_reg = ddbg_c2c_read,
	.write_reg = ddbg_c2c_write,
};

int ap9540_c2c_deep_debug_exit(void)
{
	return deep_debug_regaccess_unregister(&ddbg_c2c);
}

int ap9540_c2c_deep_debug_init(struct ap9540_c2c *c2c_dev)
{
	if (!c2c_dev)
		return -EINVAL;

	ddbg_c2c.dev_data = c2c_dev;
	return deep_debug_regaccess_register(&ddbg_c2c);
}
