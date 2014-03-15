/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author:	Daniel Martensson <Daniel.Martensson@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/mutex.h>

#include <net/caif/caif_hsi.h>

#include <linux/hsi/hsi.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Martensson<daniel.martensson@stericsson.com>");
MODULE_DESCRIPTION("CAIF HSI V3 glue");

#define NR_OF_CAIF_HSI_CHANNELS	2

struct cfhsi_v3 {
	struct list_head list;
	struct cfhsi_ops ops;
	struct platform_device pdev;
	struct hsi_msg *tx_msg;
	struct hsi_msg *rx_msg;
	bool ac_wake_high;
	bool ca_wake_high;
};

static LIST_HEAD(cfhsi_ops_list);
static DEFINE_MUTEX(cfhsi_mutex);
static struct hsi_client *cfhsi_client;

static void cfhsi_v3_rx_start_stop_cb(struct hsi_client *client,
							unsigned long event);
static int cfhsi_v3_flush_fifo(struct cfhsi_ops *ops);

static int cfhsi_v3_up(struct cfhsi_ops *ops)
{
	struct cfhsi_v3 *cfhsi;
	int res = 0;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev,"%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	/*
	 * Design issue:
	 * Code is executed NR_OF_CAIF_HSI_CHANNELS (2) times, for
	 * each cfhsi.? device. There is only one hsi_client. Therefore,
	 * the pointer initialized if equal to NULL.
	 */
	if (!cfhsi_client->ehandler) {
		res = hsi_register_port_event(cfhsi_client,
						cfhsi_v3_rx_start_stop_cb);
		if (res) {
			dev_err(&cfhsi->pdev.dev, "%s: failed to register "
					"ehandler: %d\n", __func__, res);
			return res;
		}
	}

	(void)cfhsi_v3_flush_fifo(ops);

	return res;
}

static int cfhsi_v3_down(struct cfhsi_ops *ops)
{
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev,"%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	hsi_unregister_port_event(cfhsi_client);
	cfhsi_client->ehandler = NULL;

	return 0;
}

static int cfhsi_v3_tx(u8 *ptr, int len, struct cfhsi_ops *ops)
{
	int res;
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev,"%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	/* Check length and alignment. */
	BUG_ON(((int)ptr)%4);
	BUG_ON(len%4);

	sg_init_one(cfhsi->tx_msg->sgt.sgl, (const void *)ptr,
		    (unsigned int)len);

	/* Write on HSI device. */
	res = hsi_async_write(cfhsi_client, cfhsi->tx_msg);

	return res;
}

static int cfhsi_v3_rx(u8 *ptr, int len, struct cfhsi_ops *ops)
{
	int res;
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev,"%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	/* Check length and alignment. */
	BUG_ON(((int)ptr)%4);
	BUG_ON(len%4);

	sg_init_one(cfhsi->rx_msg->sgt.sgl, (const void *)ptr,
		    (unsigned int)len);

	/* Read from HSI device. */
	res = hsi_async_read(cfhsi_client, cfhsi->rx_msg);

	return res;
}

static int cfhsi_v3_wake_up(struct cfhsi_ops *ops)
{
	int res;
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	res = hsi_setup(cfhsi_client);
	if (res) {
		dev_err(&cfhsi->pdev.dev, "%s: failed to setup HSI link: %d\n",
				__func__, res);
		return res;
	}

	/* Set AC_WAKE high */
	if (cfhsi->ac_wake_high == false) {
		res = hsi_start_tx(cfhsi_client);
		if (res) {
			dev_err(&cfhsi->pdev.dev, "%s: failed to start tx: "
					"%d\n", __func__, res);
			return res;
		}
		cfhsi->ac_wake_high = true;
	}

	return res;
}

static int cfhsi_v3_wake_down(struct cfhsi_ops *ops)
{
	int res = 0;
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	/* Set AC_WAKE down */
	if (true == cfhsi->ac_wake_high) {
		res = hsi_stop_tx(cfhsi_client);
		if (res) {
			dev_err(&cfhsi->pdev.dev, "%s: failed to stop tx: %d\n",
				__func__, res);
			return res;
		}
		cfhsi->ac_wake_high = false;
	}

	return res;
}

static int cfhsi_v3_get_peer_wake(struct cfhsi_ops *ops, bool *status)
{
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	/* status may be incorrect if ca_wake missed */
	*status = cfhsi->ca_wake_high;
	return 0;
}

static int cfhsi_v3_fifo_occupancy(struct cfhsi_ops *ops, size_t *occupancy)
{
	*occupancy = 0;
	return 0;
}

static int cfhsi_v3_rx_cancel(struct cfhsi_ops *ops)
{
	return cfhsi_v3_flush_fifo(ops);
}

static int cfhsi_v3_flush_fifo(struct cfhsi_ops *ops)
{
	struct cfhsi_v3 *cfhsi;

	cfhsi = container_of(ops, struct cfhsi_v3, ops);

	if (!cfhsi_client) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI glue layer driver not "
				"registered\n", __func__);
		return -ENODEV;
	}

	return hsi_flush(cfhsi_client);
}

void cfhsi_v3_release(struct device *dev)
{
	dev_dbg(dev, "%s:%d %s called\n", __FILE__, __LINE__, __func__);
}

static void cfhsi_v3_destructor(struct hsi_msg *msg)
{
	struct cfhsi_v3 *cfhsi = (struct cfhsi_v3 *)msg->context;

	if (!cfhsi) {
		pr_err("%s: CAIF HSI glue layer driver not registered\n",
				__func__);
		return;
	}

	dev_dbg(&cfhsi->pdev.dev, "%s:%d %s called\n", __FILE__, __LINE__,
			__func__);
}

static void cfhsi_v3_read_cb(struct hsi_msg *msg)
{
	struct cfhsi_v3 *cfhsi = (struct cfhsi_v3 *)msg->context;

	if (!cfhsi) {
		pr_err("%s: CAIF HSI glue layer driver not registered\n",
				__func__);
		return;
	}

	if (!cfhsi->ops.cb_ops) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI driver not registered",
				__func__);
		return;
	}

	if (!cfhsi->ops.cb_ops->rx_done_cb) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI rx_done_cb function "
				"not registered", __func__);
		return;
	}

	cfhsi->ops.cb_ops->rx_done_cb(cfhsi->ops.cb_ops);
}

static void cfhsi_v3_write_cb(struct hsi_msg *msg)
{
	struct cfhsi_v3 *cfhsi = (struct cfhsi_v3 *)msg->context;

	if (!cfhsi) {
		pr_err("%s: CAIF HSI glue layer driver not registered\n",
				__func__);
		return;
	}

	if (!cfhsi->ops.cb_ops) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI driver not registered",
				__func__);
		return;
	}

	if (!cfhsi->ops.cb_ops->tx_done_cb) {
		dev_err(&cfhsi->pdev.dev, "%s: CAIF HSI tx_done_cb function"
				"not registered", __func__);
		return;

	}

	cfhsi->ops.cb_ops->tx_done_cb(cfhsi->ops.cb_ops);
}

static void cfhsi_v3_rx_start_stop_cb(struct hsi_client *client,
							unsigned long event)
{
	struct cfhsi_v3 *cfhsi;

	mutex_lock(&cfhsi_mutex);

	/*
	 * CAIF-HSI design issue: there is only one "struct hsi_client"
	 * corresponding to "cfhsi_v3_driver", but there are
	 * NR_OF_CAIF_HSI_CHANNELS (2) devices:
	 * cfhsi.0 and cfhsi.1 and corresponding "struct cfhsi_ops".
	 * When CA_WAKE event occurs, the HSI controller layer doesn't know
	 * which cfhsi.? device is used.
	 * CA_WAKE callback will be always executed for cfhsi.0
	 * (terminated in CAIF).
	 */
	cfhsi = list_first_entry(&cfhsi_ops_list, struct cfhsi_v3, list);

	switch (event) {
	case HSI_EVENT_START_RX:
		cfhsi->ca_wake_high = true;
		cfhsi->ops.cb_ops->wake_up_cb(cfhsi->ops.cb_ops);
		break;
	case HSI_EVENT_STOP_RX:
		cfhsi->ca_wake_high = false;
		cfhsi->ops.cb_ops->wake_down_cb(cfhsi->ops.cb_ops);
		break;
	}

	mutex_unlock(&cfhsi_mutex);
}

static int cfhsi_v3_probe(struct device *dev)
{
	int res;
	int i;
	struct cfhsi_v3 *cfhsi;

	if (cfhsi_client) {
		pr_err("%s: CAIF HSI glue layer driver already registered\n",
				__func__);
		return -EBUSY;
	}
	cfhsi_client = to_hsi_client(dev);

	res = hsi_claim_port(cfhsi_client, 0);
	if (res) {
		pr_err("%s: failed to claimed HSI port: %d.\n", __func__, res);
		goto err_hsi_claim;
	}

	res = hsi_setup(cfhsi_client);
	if (res) {
		pr_err("%s: failed to setup HSI link: %d.\n", __func__, res);
		goto err_hsi_setup;
	}

	/* Make sure that AC_WAKE is high (No power management). */
	res = hsi_start_tx(cfhsi_client);
	if (res) {
		pr_err("%s: failed to start tx: %d.\n", __func__, res);
		goto err_hsi_setup;
	}

	mutex_lock(&cfhsi_mutex);
	/* Connect channels to CAIF HSI devices. */
	for (i = 0; i < NR_OF_CAIF_HSI_CHANNELS; i++) {
		cfhsi = kzalloc(sizeof(struct cfhsi_v3), GFP_KERNEL);
		if (!cfhsi) {
			res = -ENOMEM;
			goto err_caif_dev_reg;
		}

		/*
		 * Initialize ACWAKE/CAWAKE signals:
		 * CA_WAKE event may be missed when the driver is unregistered.
		 * Assuming that CA_WAKE is HIGH after successful modem boot by
		 * FLB protocol.
		 */
		cfhsi->ac_wake_high = false;
		cfhsi->ca_wake_high = true;

		/* Assign HSI client to this CAIF HSI device. */
		cfhsi->ops.cfhsi_tx = cfhsi_v3_tx;
		cfhsi->ops.cfhsi_rx = cfhsi_v3_rx;

		cfhsi->ops.cfhsi_up = cfhsi_v3_up;
		cfhsi->ops.cfhsi_down = cfhsi_v3_down;
		cfhsi->ops.cfhsi_wake_up = cfhsi_v3_wake_up;
		cfhsi->ops.cfhsi_wake_down = cfhsi_v3_wake_down;
		cfhsi->ops.cfhsi_get_peer_wake = cfhsi_v3_get_peer_wake;
		cfhsi->ops.cfhsi_fifo_occupancy = cfhsi_v3_fifo_occupancy;
		cfhsi->ops.cfhsi_rx_cancel = cfhsi_v3_rx_cancel;

		/* Allocate HSI messages. */
		cfhsi->tx_msg = hsi_alloc_msg(1, GFP_KERNEL);
		cfhsi->rx_msg = hsi_alloc_msg(1, GFP_KERNEL);
		if (!cfhsi->tx_msg || !cfhsi->rx_msg) {
			res = -ENOMEM;
			goto err_caif_dev_reg;
		}

		/* Set up TX message. */
		cfhsi->tx_msg->cl = cfhsi_client;
		cfhsi->tx_msg->context = (void *)cfhsi;
		cfhsi->tx_msg->complete = cfhsi_v3_write_cb;
		cfhsi->tx_msg->destructor = cfhsi_v3_destructor;
		cfhsi->tx_msg->channel = i;
		cfhsi->tx_msg->ttype = HSI_MSG_WRITE;
		cfhsi->tx_msg->break_frame = 0; /* No break frame. */

		/* Set up RX message. */
		cfhsi->rx_msg->cl = cfhsi_client;
		cfhsi->rx_msg->context = (void *)cfhsi;
		cfhsi->rx_msg->complete = cfhsi_v3_read_cb;
		cfhsi->rx_msg->destructor = cfhsi_v3_destructor;
		cfhsi->rx_msg->channel = i;
		cfhsi->rx_msg->ttype = HSI_MSG_READ;
		cfhsi->rx_msg->break_frame = 0; /* No break frame. */

		/* Initialize CAIF HSI platform device. */
		cfhsi->pdev.name = "cfhsi";
		cfhsi->pdev.dev.platform_data = &cfhsi->ops;
		cfhsi->pdev.dev.release = cfhsi_v3_release;
		/* Use channel number as id. */
		cfhsi->pdev.id = i;
		/* Register platform device. */
		res = platform_device_register(&cfhsi->pdev);
		if (res) {
			pr_err("%s: failed to register CAIF HSI device: "
					"%d.\n", __func__, res);
			goto err_caif_dev_reg;
		}

		/* Add HSI device to device list. */
		list_add_tail(&cfhsi->list, &cfhsi_ops_list);
	}
	mutex_unlock(&cfhsi_mutex);

	return res;

err_caif_dev_reg:
	mutex_unlock(&cfhsi_mutex);
err_hsi_setup:
	hsi_release_port(cfhsi_client);
err_hsi_claim:
	cfhsi_client = NULL;

	return res;
}

static int cfhsi_v3_remove(struct device *dev)
{
	struct cfhsi_v3 *cfhsi;
	struct list_head *list_node;
	struct list_head *n;

	if (!cfhsi_client) {
		dev_err(dev, "%s: CAIF HSI glue layer driver not registered\n",
				__func__);
		return -ENODEV;
	}

	hsi_stop_tx(cfhsi_client);
	hsi_release_port(cfhsi_client);
	hsi_unregister_port_event(cfhsi_client);

	mutex_lock(&cfhsi_mutex);
	list_for_each_safe(list_node, n, &cfhsi_ops_list) {
		cfhsi = list_entry(list_node, struct cfhsi_v3, list);
		/* Remove from list. */
		list_del(list_node);
		/* Our HSI device is gone, unregister CAIF HSI device. */
		platform_device_del(&cfhsi->pdev);
		hsi_free_msg(cfhsi->tx_msg);
		hsi_free_msg(cfhsi->rx_msg);
		/* Free memory. */
		kfree(cfhsi);
	}
	mutex_unlock(&cfhsi_mutex);

	cfhsi_client = NULL;

	return 0;
}

static int cfhsi_v3_suspend(struct device *dev, pm_message_t mesg)
{
	/* Not handled. */
	dev_dbg(dev, "%s.\n", __func__);
	return 0;
}

static int cfhsi_v3_resume(struct device *dev)
{
	/* Not handled. */
	dev_dbg(dev, "%s.\n", __func__);
	return 0;
}

static struct hsi_client_driver cfhsi_v3_driver = {
	.driver = {
		.name = "cfhsi_v3_driver",
		.owner	= THIS_MODULE,
		.probe = cfhsi_v3_probe,
		.remove = __devexit_p(cfhsi_v3_remove),
		.suspend = cfhsi_v3_suspend,
		.resume = cfhsi_v3_resume,
	},
};

static int __init cfhsi_v3_init(void)
{
	int res;

	/* Register protocol driver for HSI interface. */
	res =  hsi_register_client_driver(&cfhsi_v3_driver);
	if (res)
		pr_err("Failed to register CAIF HSI V3 driver.\n");

	return res;
}

static void __exit cfhsi_v3_exit(void)
{
	/* Unregister driver. */
	hsi_unregister_client_driver(&cfhsi_v3_driver);
}

module_init(cfhsi_v3_init);
module_exit(cfhsi_v3_exit);

