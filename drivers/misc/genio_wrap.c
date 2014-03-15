/*
 * Copyright (C) ST-Ericssson Le Mans SA 2012
 * Author: Etienne Carriere <etienne.carriere@stericsson.com>
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Wrapper for modem IPC based on C2C GENI/GENO like drivers: GENIO drivers.
 * Supports only one instance of HW 'GENIO' driver.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/c2c_genio.h>
#include <linux/mfd/dbx500-prcmu.h>

static struct genio_wrap *genio_dev;
static DEFINE_MUTEX(lock);
static const char nodrvname[]="";

char * genio_drvname(void)
{
	if (unlikely(genio_dev == NULL))
		return (char *) nodrvname;
	return (genio_dev->drvname) ? genio_dev->drvname : (char *) nodrvname;
}
EXPORT_SYMBOL_GPL(genio_drvname);

int genio_subscribe(int bit, void (*bit_set_cb)(void *data), void *data)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->subscribe)
		return (genio_dev->subscribe)(bit, bit_set_cb, data);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_subscribe);

int genio_unsubscribe(int bit)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->unsubscribe)
		return (genio_dev->unsubscribe)(bit);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_unsubscribe);

int genio_bit_alloc(u32 setter_mask, u32 getter_mask)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->bit_alloc)
		return (genio_dev->bit_alloc)(setter_mask, getter_mask);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_bit_alloc);

int genio_set_shm_addr(u32 addr, void (*ipc_ready_cb) (void))
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->set_shm_addr)
		return (genio_dev->set_shm_addr)(addr, ipc_ready_cb);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_set_shm_addr);

int genio_subscribe_caif_ready(void (*caif_ready_cb) (bool ready))
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->subscribe_caif_ready)
		return (genio_dev->subscribe_caif_ready)(caif_ready_cb);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_subscribe_caif_ready);

void genio_register_errhandler(void (*errhandler)(int errno))
{
	if (unlikely(genio_dev == NULL))
		return;
	if (genio_dev->register_errhandler)
		(genio_dev->register_errhandler)(errhandler);
}
EXPORT_SYMBOL_GPL(genio_register_errhandler);

int genio_reset(void)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->reset)
		return (genio_dev->reset)();
	return 0;
}
EXPORT_SYMBOL_GPL(genio_reset);

int genio_set_bit(int bit)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->set_bit)
		return (genio_dev->set_bit)(bit);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_set_bit);

int genio_power_req(int state)
{
	if (unlikely(genio_dev == NULL))
		return -ENODEV;
	if (genio_dev->power_req)
		return (genio_dev->power_req)(state);
	return 0;
}
EXPORT_SYMBOL_GPL(genio_power_req);

int genio_register_driver(struct genio_wrap *drv)
{
	int ret = 0;
	mutex_lock(&lock);
	if (genio_dev) {
		pr_err("%s: genio_wrap supports only 1 instance !\n", __func__);
		WARN_ON(true);
		ret = -EBUSY;
	} else {
		genio_dev = drv;
	}
	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(genio_register_driver);

void genio_unregister_driver(struct genio_wrap *drv)
{
	mutex_lock(&lock);
	if (genio_dev && (genio_dev != drv)) {
		pr_err("%s: refuse to unregister this instance\n", __func__);
		WARN_ON(true);
	} else {
		prcmu_unregister_modem(genio_dev->drvname);
		genio_dev = NULL;
	}
	mutex_unlock(&lock);
}
EXPORT_SYMBOL(genio_unregister_driver);
