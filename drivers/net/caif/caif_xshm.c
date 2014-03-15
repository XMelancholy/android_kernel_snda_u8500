/*
 * Copyright (C) ST-Ericsson AB 2010
 * Contact: Sjur Brendeland / sjur.brandeland@stericsson.com
 * Authors: Sjur Brendeland / sjur.brandeland@stericsson.com
 *	   Daniel Martensson / daniel.martensson@stericsson.com
 * License terms: GNU General Public License (GPL) version 2
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s() :" fmt, __func__
#include <linux/module.h>
#include <linux/pkt_sched.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/netdevice.h>
#include <net/rtnetlink.h>
#include <linux/if_arp.h>
#include <net/caif/caif_device.h>
#include <net/caif/caif_layer.h>
#include <linux/xshm/xshm_pdev.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Martensson <daniel.martensson@stericsson.com>");
MODULE_AUTHOR("Sjur Brendeland <sjur.brandeland@stericsson.com>");
MODULE_DESCRIPTION("CAIF SHM driver");

#define CONNECT_TIMEOUT (3 * HZ)
#define CAIF_NEEDED_HEADROOM	32
#define CAIF_FLOW_ON		1
#define CAIF_FLOW_OFF		0

#define LOW_XOFF_WATERMARK	50
#define HIGH_XOFF_WATERMARK	70
#define STUFF_MARK		30

/* When to kick remote side, based on free bufs in percentage */
#define RELEASE_KICK_THR	90

struct cfshm_ringbuf {
	__le32	*rip;
	__le32	*wip;
	u32	size;
	__le32	*bufsize;
};

struct cfshm_pck_desc {
	/* Offset from start of channel to CAIF frame. */
	u32 offset;
	u32 length;
} __packed;

struct cfshm_frame {
	/* Number of bytes of padding before the CAIF frame. */
	u8 hdr_ofs;
} __packed;

#define SHM_HDR_LEN sizeof(struct cfshm_frame)

struct cfshm_shmbuffer {
/* Static part: */
	u8 *addr;
	u32 index;
	u32 len;
/* Dynamic part: */
	u32 frames;
	/* Offset from start of buffer to CAIF frame. */
	u32 frm_ofs;
};

enum CFSHM_STATE {
	CFSHM_CLOSED = 1,
	CFSHM_OPENING,
	CFSHM_OPEN
};

struct cfshm {
	/*****************/
	/* Dynamic part: */
	/*****************/
	/* caif_dev_common must always be first in the structure*/
	struct caif_dev_common cfdev;
	struct xshm_dev *xshm;
	struct napi_struct napi;
	struct net_device *ndev;
	enum CFSHM_STATE state;

	struct sk_buff_head sk_qhead;
	struct list_head node;
	wait_queue_head_t netmgmt_wq;
	struct tasklet_hrtimer aggregation_timer;

	struct cfshm_shmbuffer **rx_bufs;
	struct cfshm_shmbuffer **tx_bufs;
	u32 tx_flow_on;

	/****************/
	/* Static part: */
	/****************/
	u32 high_xoff_water;
	u32 low_xoff_water;

	spinlock_t lock_rx;
	struct cfshm_ringbuf rx;
	u8 *rx_ringbuf;
	u32 rx_frms_pr_buf;
	u32 rx_alignment;

	spinlock_t lock_tx;
	struct cfshm_ringbuf tx;
	u8 *tx_ringbuf;
	u32 tx_frms_pr_buf;
	u32 tx_alignment;
	u32 release_kick_thr;
	struct tasklet_struct tx_release_tasklet;
	struct platform_device *pdev;
};

static int aggregation_timeout = 1;
module_param(aggregation_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(aggregation_timeout, "Aggregation timeout on HSI, ms.");

static LIST_HEAD(cfshm_list);
static spinlock_t cfshm_list_lock;

/* Ringbuffer primitives */
static unsigned int ringbuf_used(struct cfshm_ringbuf *rb)
{
	if (le32_to_cpu(*rb->wip) >= le32_to_cpu(*rb->rip))
		return le32_to_cpu(*rb->wip) - le32_to_cpu(*rb->rip);
	else
		return rb->size - le32_to_cpu(*rb->rip) + le32_to_cpu(*rb->wip);
}

static int ringbuf_get_writepos(struct cfshm_ringbuf *rb)
{
	if ((le32_to_cpu(*rb->wip) + 1) % rb->size == le32_to_cpu(*rb->rip))
		return -1;
	else
		return le32_to_cpu(*rb->wip);
}

static int ringbuf_get_readpos(struct cfshm_ringbuf *rb)
{
	if (le32_to_cpu(*rb->wip) == le32_to_cpu(*rb->rip))
		return -1;
	else
		return le32_to_cpu(*rb->rip);
}

static int ringbuf_upd_writeptr(struct cfshm_ringbuf *rb)
{
	if (!WARN_ON((le32_to_cpu(*rb->wip) + 1) % rb->size ==
				le32_to_cpu(*rb->rip))) {
		/* Do write barrier before updating index */
		wmb();
		*rb->wip = cpu_to_le32((le32_to_cpu(*rb->wip) + 1) % rb->size);
	}
	return le32_to_cpu(*rb->wip);
}

static void ringbuf_upd_readptr(struct cfshm_ringbuf *rb)
{
	if (!WARN_ON(le32_to_cpu(*rb->wip) == le32_to_cpu(*rb->rip))) {
		/* Do write barrier before updating index */
		wmb();
		*rb->rip = cpu_to_le32((le32_to_cpu(*rb->rip) + 1) % rb->size);
	}
}

static struct cfshm_shmbuffer *cfshm_get_rx_buf(struct cfshm *cfshm)
{
	struct cfshm_shmbuffer *pbuf = NULL;
	int idx = ringbuf_get_readpos(&cfshm->rx);

	if (idx < 0)
		goto out;
	pbuf = cfshm->rx_bufs[idx];
out:
	return pbuf;
}

static struct cfshm_shmbuffer *cfshm_new_rx_buf(struct cfshm *cfshm)
{
	struct cfshm_shmbuffer *pbuf = cfshm_get_rx_buf(cfshm);

	WARN_ON(!spin_is_locked(&cfshm->lock_rx));
	if (pbuf)
		pbuf->frames = 0;
	return pbuf;
}

static struct cfshm_shmbuffer *cfshm_get_tx_buf(struct cfshm *cfshm)
{
	int idx = ringbuf_get_writepos(&cfshm->tx);

	if (idx < 0)
		return NULL;
	return cfshm->tx_bufs[idx];
}

inline struct cfshm_shmbuffer *cfshm_bump_buf(struct cfshm *cfshm,
			struct cfshm_shmbuffer *pbuf)
{
	u32 desc_size;
	struct cfshm_shmbuffer *newpbuf = pbuf;

	WARN_ON(!spin_is_locked(&cfshm->lock_tx));
	if (pbuf) {
		cfshm->xshm->cfg.tx.buf_size[pbuf->index] =
			cpu_to_le32(pbuf->frm_ofs);
		ringbuf_upd_writeptr(&cfshm->tx);
		newpbuf = cfshm_get_tx_buf(cfshm);
		/* Reset buffer parameters. */
		desc_size = (cfshm->tx_frms_pr_buf + 1) *
			sizeof(struct cfshm_pck_desc);
		pbuf->frm_ofs = desc_size + (desc_size % cfshm->rx_alignment);
		pbuf->frames = 0;
	}
	return newpbuf;
}

static struct cfshm_shmbuffer *cfshm_rx_func(struct cfshm *cfshm, int quota, int *cnt)
{
	struct cfshm_shmbuffer *pbuf;
	struct sk_buff *skb;
	int ret;

	spin_lock_bh(&cfshm->lock_rx);
	pbuf = cfshm_get_rx_buf(cfshm);
	while (pbuf) {
		/* Retrieve pointer to start of the packet descriptor area. */
		struct cfshm_pck_desc *pck_desc =
			((struct cfshm_pck_desc *) pbuf->addr) + pbuf->frames;
		u32 offset;

		/* Loop until descriptor contains zero offset */
		while ((offset = pck_desc->offset)) {
			unsigned int caif_len;
			struct cfshm_frame *frm;
			u32 length = pck_desc->length;
			u8 hdr_ofs;
			frm = (struct cfshm_frame *)(pbuf->addr + offset);
			hdr_ofs = frm->hdr_ofs;
			caif_len = length - SHM_HDR_LEN - hdr_ofs;

			pr_devel("copy data buf:%d frm:%d offs:%d @%x len:%d\n",
					pbuf->index, pbuf->frames, offset,
					(u32) (SHM_HDR_LEN + hdr_ofs + offset +
						pbuf->addr - cfshm->rx_ringbuf),
					length);

			/* Check whether number of frames is below limit */
			if (pbuf->frames > cfshm->rx_frms_pr_buf) {
				pr_warn("Too many frames in buffer.\n");
				++cfshm->ndev->stats.rx_frame_errors;
				goto desc_err;
			}

			/* Check whether offset is below low limits */
			if (pbuf->addr + offset
					<= (u8 *)(pck_desc + 1)) {
				pr_warn("Offset in desc. below buffer area.\n");
				++cfshm->ndev->stats.rx_frame_errors;
				goto desc_err;
			}

			/* Check whether offset above upper limit */
			if (offset + length > pbuf->len) {
				pr_warn("Offset outside buffer area:\n");
				++cfshm->ndev->stats.rx_frame_errors;
				goto desc_err;
			}

			skb = netdev_alloc_skb(cfshm->ndev, caif_len + 1);
			if (skb == NULL) {
				pr_debug("Couldn't allocate SKB\n");
				++cfshm->ndev->stats.rx_dropped;
				goto out;
			}

			memcpy(skb_put(skb, caif_len),
					SHM_HDR_LEN + hdr_ofs +
					offset + pbuf->addr,
					caif_len);

			skb->protocol = htons(ETH_P_CAIF);
			skb_reset_mac_header(skb);
			skb->dev = cfshm->ndev;

			/* Push received packet up the stack. */
			ret = netif_receive_skb(skb);

			if (!ret) {
				cfshm->ndev->stats.rx_packets++;
				cfshm->ndev->stats.rx_bytes += length;
			} else
				++cfshm->ndev->stats.rx_dropped;
			/* Move to next packet descriptor. */
			pck_desc++;

			(*cnt)++;
			pbuf->frames++;
			if (--quota <= 0) {
				pr_devel("Quota exeeded (pbuf:%p)\n", pbuf);
				goto out;
			}
		}
desc_err:
		pbuf->frames = 0;
		ringbuf_upd_readptr(&cfshm->rx);
		pbuf = cfshm_new_rx_buf(cfshm);
	}
out:
	/* Generate read interrupt only if threshold is reached */
	if (ringbuf_used(&cfshm->rx) >= cfshm->release_kick_thr)
		cfshm->xshm->ipc_rx_release(cfshm->xshm, false);
	spin_unlock_bh(&cfshm->lock_rx);
	return pbuf;
}

/* Note: called in hard irq context. */
static int cfshm_rx_cb(void *drv)
{
	struct cfshm *cfshm = drv;

	if (unlikely(*cfshm->xshm->cfg.rx.state == cpu_to_le32(XSHM_CLOSED)))
		return -ESHUTDOWN;

	napi_schedule(&cfshm->napi);
	return 0;
}

static int cfshm_rx_poll(struct napi_struct *napi, int quota)
{
	struct cfshm *cfshm = container_of(napi, struct cfshm, napi);
	int rxcnt = 0;

	pr_devel("enter\n");

	cfshm_rx_func(cfshm, quota, &rxcnt);

	if (!rxcnt) {
		/* First notify complete */
		napi_complete(&cfshm->napi);

		/* Then double-check ringbuffer and reschedule if new
		 * data is available. */
		if (ringbuf_used(&cfshm->rx))
			napi_schedule(&cfshm->napi);
	}

	return rxcnt;
}

/* TX datapath implementation */
static int cfshm_xmit_skb(struct cfshm *cfshm, struct sk_buff *skb,
					struct cfshm_shmbuffer *pbuf)
{
	struct cfshm_pck_desc *pck_desc;
	unsigned int frmlen;
	struct cfshm_frame *frm;
	u8 hdr_ofs, pad_len;
	struct caif_payload_info *info = (struct caif_payload_info *)&skb->cb;

	WARN_ON(!spin_is_locked(&cfshm->lock_tx));

	if (unlikely(pbuf->frames >= cfshm->tx_frms_pr_buf)) {
		pr_devel("-ENOSPC exeeded frames: %d >= %d\n",
				pbuf->frames, cfshm->tx_frms_pr_buf);
		return -ENOSPC;
	}

	/*
	 * Align the address of the entire CAIF frame (incl padding),
	 * so the modem can do efficient DMA of this frame
	 * FIXME: Alignment is power of to, so it could use binary ops.
	 */
	pbuf->frm_ofs = roundup(pbuf->frm_ofs, cfshm->tx_alignment);

	/* Make the payload (IP packet) inside the frame aligned */
	hdr_ofs = ((unsigned long) &pbuf->frm_ofs) + SHM_HDR_LEN +
		info->hdr_len;

	frm = (struct cfshm_frame *)
		(pbuf->addr + pbuf->frm_ofs);

	pad_len = ALIGN(hdr_ofs, cfshm->tx_alignment) - hdr_ofs;

	frmlen = SHM_HDR_LEN + pad_len + skb->len;

	/*
	 * Verify that packet, header and additional padding
	 * can be written within the buffer frame area.
	 */
	if (pbuf->len < pbuf->frm_ofs + frmlen) {
		pr_devel("-ENOSPC exeeded offset %d < %d\n",
				pbuf->len, pbuf->frm_ofs + frmlen);
		return -ENOSPC;
	}

	/* Copy in CAIF frame. */
	frm->hdr_ofs = pad_len;
	skb_copy_bits(skb, 0, pbuf->addr +
			pbuf->frm_ofs + SHM_HDR_LEN +
			pad_len, skb->len);

	pr_devel("copy data buf:%d frm:%d offs:%d @%d len:%d\n",
			pbuf->index, pbuf->frames,
			pbuf->frm_ofs,
			(u32) (pbuf->addr + pbuf->frm_ofs +
				SHM_HDR_LEN + pad_len - cfshm->tx_ringbuf),
			skb->len);

	cfshm->ndev->stats.tx_packets++;
	cfshm->ndev->stats.tx_bytes += frmlen;
	/* Fill in the shared memory packet descriptor area. */
	pck_desc = (struct cfshm_pck_desc *) (pbuf->addr);
	/* Forward to current frame. */
	pck_desc += pbuf->frames;
	pck_desc->offset = pbuf->frm_ofs;
	pck_desc->length = frmlen;
	/* Terminate packet descriptor area. */
	pck_desc++;
	pck_desc->offset = 0;
	pck_desc->length = 0;
	/* Update buffer parameters. */
	pbuf->frames++;
	pbuf->frm_ofs += frmlen;

	return 0;
}

static bool cfshm_is_prio_frame(struct sk_buff *skb)
{
	switch (skb->priority) {
	case TC_PRIO_INTERACTIVE_BULK:
	case TC_PRIO_INTERACTIVE:
	case TC_PRIO_CONTROL ... TC_PRIO_MAX:
		return true;
	default:
		return false;
	}
}

static void cfshm_xmit_queued(struct cfshm *cfshm, bool force)
{
	struct cfshm_shmbuffer *pbuf = NULL;
	int usedbufs;
	bool kick = false;
	struct sk_buff *skb;
	int err;

	WARN_ON(!spin_is_locked(&cfshm->lock_tx));

	pbuf = cfshm_get_tx_buf(cfshm);
	while (pbuf != NULL) {
		skb = skb_peek(&cfshm->sk_qhead);
		if (skb == NULL)
			break;
		err = cfshm_xmit_skb(cfshm, skb, pbuf);
		if (unlikely(err == -ENOSPC)) {
			pr_devel("No more space in buffer\n");
			pbuf = cfshm_bump_buf(cfshm, pbuf);
			continue;
		}
		skb = skb_dequeue(&cfshm->sk_qhead);
		/* We're always in NET_*_SOFTIRQ */
		dev_kfree_skb(skb);
	}

	usedbufs = ringbuf_used(&cfshm->tx);

	if (usedbufs > cfshm->high_xoff_water && cfshm->tx_flow_on) {
		pr_debug("Flow off\n");
		cfshm->tx_flow_on = false;
		cfshm->cfdev.flowctrl(cfshm->ndev, CAIF_FLOW_OFF);
		kick = true;
	} else if (usedbufs <= cfshm->low_xoff_water && !cfshm->tx_flow_on) {
		pr_debug("Flow on\n");
		cfshm->tx_flow_on = true;
		cfshm->cfdev.flowctrl(cfshm->ndev, CAIF_FLOW_ON);
	}

	if (force) {
		/* Send prioritized traffic immediately */
		pr_devel("TX prio frame\n");
		if (pbuf && pbuf->frames)
			cfshm_bump_buf(cfshm, pbuf);
		/* Delete aggregation timer. */
		hrtimer_cancel(&cfshm->aggregation_timer.timer);
		kick = true;
	} else {
		/* Accumulate bulk traffic, start timer if not already active */
		pr_devel("Accumulate frame\n");
		if (!hrtimer_active(&cfshm->aggregation_timer.timer))
			tasklet_hrtimer_start(&cfshm->aggregation_timer,
				ktime_set(0, aggregation_timeout *
					NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
	}

	if (kick)
		cfshm->xshm->ipc_tx(cfshm->xshm);
}

static enum hrtimer_restart cfshm_aggregation_tout(struct hrtimer *timer)
{
	struct cfshm *cfshm =
		container_of(timer, struct cfshm, aggregation_timer.timer);

	if (unlikely(*cfshm->xshm->cfg.rx.state == cpu_to_le32(XSHM_CLOSED)))
		return HRTIMER_NORESTART;

	pr_devel("Aggregation timer kicked\n");

	spin_lock_bh(&cfshm->lock_tx);
	cfshm_xmit_queued(cfshm, true);
	spin_unlock_bh(&cfshm->lock_tx);
	return HRTIMER_NORESTART;
}

static void cfshm_tx_release_tasklet(unsigned long drv)
{
	struct cfshm *cfshm = (struct cfshm *)drv;

	spin_lock_bh(&cfshm->lock_tx);
	cfshm_xmit_queued(cfshm, false);
	spin_unlock_bh(&cfshm->lock_tx);
}

/* Note: called in hard irq context */
static int cfshm_tx_release_cb(void *drv)
{
	struct cfshm *cfshm = drv;
	tasklet_schedule(&cfshm->tx_release_tasklet);
	return 0;
}

static int cfshm_start_xmit(struct sk_buff *skb, struct net_device *shm_netdev)
{
	struct cfshm *cfshm = netdev_priv(shm_netdev);

	spin_lock_bh(&cfshm->lock_tx);

	skb_queue_tail(&cfshm->sk_qhead, skb);
	cfshm_xmit_queued(cfshm, cfshm_is_prio_frame(skb));

	spin_unlock_bh(&cfshm->lock_tx);

	return 0;
}

static int cfshm_netdev_open(struct net_device *netdev)
{
	struct cfshm *cfshm = netdev_priv(netdev);
	int ret, err = 0;

	cfshm->state = CFSHM_OPENING;
	if (cfshm->xshm != NULL && cfshm->xshm->open != NULL)
		err = cfshm->xshm->open(cfshm->xshm);
	if (err)
		goto error;

	rtnl_unlock();  /* Release RTNL lock during connect wait */
	ret = wait_event_interruptible_timeout(cfshm->netmgmt_wq,
			cfshm->state != CFSHM_OPENING,
			CONNECT_TIMEOUT);
	rtnl_lock();

	if (ret == 0) {
		pr_debug("connect timeout\n");
		err = -ETIMEDOUT;
		goto error;
	}

	if (cfshm->state !=  CFSHM_OPEN) {
		pr_debug("connect failed\n");
		err = -ECONNREFUSED;
		goto error;
	}

	tasklet_hrtimer_init(&cfshm->aggregation_timer,
		cfshm_aggregation_tout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	napi_enable(&cfshm->napi);
	return 0;
error:
	if (cfshm->xshm != NULL && cfshm->xshm->close != NULL)
		cfshm->xshm->close(cfshm->xshm);
	return err;
}

static int cfshm_netdev_close(struct net_device *netdev)
{
	struct cfshm *cfshm = netdev_priv(netdev);

	spin_lock_bh(&cfshm->lock_tx);
	tasklet_hrtimer_cancel(&cfshm->aggregation_timer);
	spin_unlock_bh(&cfshm->lock_tx);

	napi_disable(&cfshm->napi);

	if (cfshm->xshm != NULL && cfshm->xshm->close != NULL)
		cfshm->xshm->close(cfshm->xshm);

	return 0;
}

static int cfshm_open_cb(void *drv)
{
	struct cfshm *cfshm = drv;

	WARN_ON_ONCE(in_irq());

	cfshm->state = CFSHM_OPEN;
	netif_carrier_on(cfshm->ndev);
	wake_up_interruptible(&cfshm->netmgmt_wq);
	return 0;
}

static void cfshm_close_cb(void *drv)
{
	struct cfshm *cfshm = drv;

	WARN_ON_ONCE(in_irq());

	cfshm->state = CFSHM_CLOSED;
	netif_carrier_off(cfshm->ndev);
	wake_up_interruptible(&cfshm->netmgmt_wq);
}

static const struct net_device_ops netdev_ops = {
	.ndo_open = cfshm_netdev_open,
	.ndo_stop = cfshm_netdev_close,
	.ndo_start_xmit = cfshm_start_xmit,
};

static void cfshm_netdev_setup(struct net_device *pshm_netdev)
{
	struct cfshm *cfshm;

	cfshm = netdev_priv(pshm_netdev);
	pshm_netdev->netdev_ops = &netdev_ops;
	pshm_netdev->type = ARPHRD_CAIF;
	pshm_netdev->hard_header_len = CAIF_NEEDED_HEADROOM;
	pshm_netdev->tx_queue_len = 0;
	pshm_netdev->destructor = free_netdev;

	/* Initialize structures in a clean state. */
	memset(cfshm, 0, sizeof(struct cfshm));
}

static void cfshm_deinit_bufs(struct cfshm *cfshm)
{
	int j;

	if (cfshm == NULL)
		return;

	for (j = 0; j < cfshm->xshm->cfg.rx.buffers; j++)
		kfree(cfshm->rx_bufs[j]);
	kfree(cfshm->rx_bufs);

	for (j = 0; j < cfshm->xshm->cfg.tx.buffers; j++)
		kfree(cfshm->tx_bufs[j]);
	kfree(cfshm->tx_bufs);
}

static int cfshm_probe(struct platform_device *pdev)
{
	int err, j;
	struct xshm_dev *xshm = pdev->dev.platform_data;
	struct cfshm *cfshm = NULL;
	struct net_device *netdev;
	u32 buf_size;

	if (xshm == NULL)
		return -EINVAL;
	if (xshm->cfg.tx.addr == NULL || xshm->cfg.rx.addr == NULL) {
		pr_debug("Shared Memory are not configured\n");
		return -EINVAL;
	}

	if (xshm->cfg.tx.ch_size / xshm->cfg.tx.buffers <
			xshm->cfg.tx.packets * sizeof(struct cfshm_pck_desc) +
				xshm->cfg.tx.mtu) {
		pr_warn("Bad packet TX-channel size");
		return -EINVAL;
	}

	if (xshm->cfg.rx.ch_size / xshm->cfg.rx.buffers <
			sizeof(struct cfshm_pck_desc) + xshm->cfg.rx.mtu) {
		pr_warn("Bad packet RX-channel size");
		return -EINVAL;
	}

	if (xshm->cfg.rx.buffers < 2 || xshm->cfg.tx.buffers < 2) {
		pr_warn("Too few buffers in channel");
		return -EINVAL;
	}

	err = -ENOMEM;
	netdev = alloc_netdev(sizeof(struct cfshm), xshm->cfg.name,
			cfshm_netdev_setup);

	if (netdev == NULL)
		goto error;

	cfshm = netdev_priv(netdev);
	cfshm->pdev = pdev;
	cfshm->state = CFSHM_CLOSED;
	init_waitqueue_head(&cfshm->netmgmt_wq);

	cfshm->xshm = xshm;
	xshm->driver_data = cfshm;
	cfshm->ndev = netdev;
	netdev->mtu = xshm->cfg.tx.mtu;
	cfshm->high_xoff_water =
		(xshm->cfg.rx.buffers * HIGH_XOFF_WATERMARK) / 100;
	cfshm->low_xoff_water =
		(xshm->cfg.rx.buffers * LOW_XOFF_WATERMARK) / 100;

	cfshm->tx_frms_pr_buf = xshm->cfg.tx.packets;
	cfshm->rx_frms_pr_buf = xshm->cfg.rx.packets;
	cfshm->rx_alignment = xshm->cfg.rx.alignment;
	cfshm->tx_alignment = xshm->cfg.tx.alignment;

	if (xshm->cfg.latency)
		cfshm->cfdev.link_select = CAIF_LINK_LOW_LATENCY;
	else
		cfshm->cfdev.link_select = CAIF_LINK_HIGH_BANDW;

	cfshm->tx.rip = xshm->cfg.tx.read;
	cfshm->tx.wip = xshm->cfg.tx.write;
	cfshm->tx.bufsize = xshm->cfg.tx.buf_size;
	cfshm->tx.size = xshm->cfg.tx.buffers;

	cfshm->rx.rip = xshm->cfg.rx.read;
	cfshm->rx.wip = xshm->cfg.rx.write;
	cfshm->rx.bufsize = xshm->cfg.rx.buf_size;
	cfshm->rx.size = xshm->cfg.rx.buffers;
	pr_devel("RX ri:%d wi:%d size:%d\n",
		le32_to_cpu(*cfshm->rx.rip),
			le32_to_cpu(*cfshm->rx.wip), cfshm->rx.size);
	pr_devel("TX ri:%d wi:%d size:%d\n",
		le32_to_cpu(*cfshm->tx.rip),
			le32_to_cpu(*cfshm->tx.wip), cfshm->rx.size);
	pr_devel("frms_pr_buf:%d %d\n", cfshm->rx_frms_pr_buf,
			cfshm->tx_frms_pr_buf);

	spin_lock_init(&cfshm->lock_tx);
	spin_lock_init(&cfshm->lock_rx);
	netif_carrier_off(netdev);
	skb_queue_head_init(&cfshm->sk_qhead);
	tasklet_init(&cfshm->tx_release_tasklet,
		cfshm_tx_release_tasklet,
		(unsigned long)cfshm);

	pr_devel("SHM DEVICE[%p] PROBED BY DRIVER, NEW SHM DRIVER"
			" INSTANCE AT cfshm =0x%p\n",
			cfshm->xshm, cfshm);

	cfshm->tx_ringbuf = xshm->cfg.tx.addr;
	cfshm->rx_ringbuf = xshm->cfg.rx.addr;

	pr_devel("TX-BASE:%p RX-BASE:%p\n",
			cfshm->tx_ringbuf,
			cfshm->rx_ringbuf);

	cfshm->tx_bufs = kzalloc(sizeof(struct cfshm_shmbuffer *) *
			xshm->cfg.tx.buffers, GFP_KERNEL);
	if (cfshm->tx_bufs == NULL)
		goto error;
	buf_size = xshm->cfg.tx.ch_size / xshm->cfg.tx.buffers;

	pr_devel("TX: buffers:%d buf_size:%d frms:%d mtu:%d\n",
			xshm->cfg.tx.buffers, buf_size,
			cfshm->tx_frms_pr_buf, netdev->mtu);

	for (j = 0; j < xshm->cfg.tx.buffers; j++) {
		u32 desc_size;
		struct cfshm_shmbuffer *tx_buf =
				kzalloc(sizeof(struct cfshm_shmbuffer), GFP_KERNEL);

		if (tx_buf == NULL) {
			pr_warn("ERROR, Could not"
					" allocate dynamic mem. for tx_buf, "
					" Bailing out ...\n");
			goto error;
		}

		tx_buf->index = j;

		tx_buf->addr = cfshm->tx_ringbuf + (buf_size * j);
		tx_buf->len = buf_size;
		tx_buf->frames = 0;
		desc_size = (cfshm->tx_frms_pr_buf + 1) *
				sizeof(struct cfshm_pck_desc);

		tx_buf->frm_ofs = desc_size + (desc_size % cfshm->tx_alignment);

		cfshm->tx_bufs[j] = tx_buf;

		pr_devel("tx_buf[%d] addr:%p len:%d\n",
				tx_buf->index,
				tx_buf->addr,
				tx_buf->len);
	}

	cfshm->rx_bufs = kzalloc(sizeof(struct cfshm_shmbuffer *) *
				xshm->cfg.rx.buffers, GFP_KERNEL);
	if (cfshm->rx_bufs == NULL)
		goto error;
	buf_size = xshm->cfg.tx.ch_size / xshm->cfg.tx.buffers;
	pr_devel("RX: buffers:%d buf_size:%d frms:%d mtu:%d\n",
			xshm->cfg.rx.buffers, buf_size,
			cfshm->rx_frms_pr_buf, netdev->mtu);

	for (j = 0; j < xshm->cfg.rx.buffers; j++) {
		struct cfshm_shmbuffer *rx_buf =
				kzalloc(sizeof(struct cfshm_shmbuffer), GFP_KERNEL);

		if (rx_buf == NULL) {
			pr_warn("ERROR, Could not"
					" allocate dynamic mem.for rx_buf, "
					" Bailing out ...\n");
			goto error;
		}

		rx_buf->index = j;

		rx_buf->addr = cfshm->rx_ringbuf + (buf_size * j);
		rx_buf->len = buf_size;
		cfshm->rx_bufs[j] = rx_buf;
		pr_devel("rx_buf[%d] addr:%p len:%d\n",
				rx_buf->index,
				rx_buf->addr,
				rx_buf->len);
	}

	/* Calculate threshold when to kick remote when a buffer is released */
	cfshm->release_kick_thr = min(cfshm->rx.size - 1,
				(cfshm->rx.size * RELEASE_KICK_THR) / 100);

	cfshm->tx_flow_on = 1;
	cfshm->xshm->ipc_rx_cb = cfshm_rx_cb;
	cfshm->xshm->ipc_tx_release_cb = cfshm_tx_release_cb;
	cfshm->xshm->open_cb = cfshm_open_cb;
	cfshm->xshm->close_cb = cfshm_close_cb;

	netif_napi_add(netdev, &cfshm->napi, cfshm_rx_poll,
			2 * cfshm->rx_frms_pr_buf);

	err = register_netdev(netdev);
	if (err) {
		pr_warn("ERROR[%d], SHM could not, "
			"register with NW FRMWK Bailing out ...\n", err);
		goto error;
	}

	/* Add CAIF SHM device to list. */
	spin_lock(&cfshm_list_lock);
	list_add_tail(&cfshm->node, &cfshm_list);
	spin_unlock(&cfshm_list_lock);

	return err;
error:
	cfshm_deinit_bufs(cfshm);
	free_netdev(netdev);
	return err;
}

static int cfshm_remove(struct platform_device *pdev)
{
	struct xshm_dev *xshm;
	struct cfshm *cfshm;

	xshm = pdev->dev.platform_data;

	if (xshm == NULL || xshm->driver_data == NULL)
		return 0;

	cfshm = xshm->driver_data;

	spin_lock(&cfshm_list_lock);
	list_del(&cfshm->node);
	spin_unlock(&cfshm_list_lock);

	tasklet_kill(&cfshm->tx_release_tasklet);
	cfshm_deinit_bufs(cfshm);

	unregister_netdev(cfshm->ndev);

	xshm->ipc_rx_cb = NULL;
	xshm->ipc_tx_release_cb = NULL;
	xshm->open_cb = NULL;
	xshm->close_cb = NULL;
	xshm->driver_data = NULL;

	return 0;
}

static struct platform_driver cfshm_plat_drv = {
	.probe = cfshm_probe,
	.remove = cfshm_remove,
	.driver = {
		.name = "xshmp",
		.owner = THIS_MODULE,
	},
};

static void __exit cfshm_exit_module(void)
{
	platform_driver_unregister(&cfshm_plat_drv);
}

static int __init cfshm_init_module(void)
{
	int err;

	spin_lock_init(&cfshm_list_lock);

	err = platform_driver_register(&cfshm_plat_drv);
	if (err) {
		printk(KERN_ERR "Could not register platform SHM driver: %d.\n",
			err);
		goto err_dev_register;
	}
	return err;

 err_dev_register:
	return err;
}

module_init(cfshm_init_module);
module_exit(cfshm_exit_module);
