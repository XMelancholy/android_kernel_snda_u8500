/*
 * ste_hsi_deepdebug.c
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author:  Cedric Madianga <cedric.madianga> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/ux500_deepdebug.h>
#include <linux/hsi/hsi.h>
#include <linux/regulator/consumer.h>

#include <mach/hsi.h>

#include "ste_hsi.h"

/* HSIT registers list */
static struct ddbg_register ddbg_hsit_registers[] = {
	DDBG_REG("STE_HSI_TX_ID", STE_HSI_TX_ID, DDBG_RW),
	DDBG_REG("STE_HSI_TX_MODE", STE_HSI_TX_MODE, DDBG_RW),
	DDBG_REG("STE_HSI_TX_STATE", STE_HSI_TX_STATE, DDBG_RW),
	DDBG_REG("STE_HSI_TX_IOSTATE", STE_HSI_TX_IOSTATE, DDBG_RO),
	DDBG_REG("STE_HSI_TX_BUFSTATE", STE_HSI_TX_BUFSTATE, DDBG_RW),
	DDBG_REG("STE_HSI_TX_DIVISOR", STE_HSI_TX_DIVISOR, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BREAK", STE_HSI_TX_BREAK, DDBG_RW),
	DDBG_REG("STE_HSI_TX_CHANNELS", STE_HSI_TX_CHANNELS, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FLUSHBITS", STE_HSI_TX_FLUSHBITS, DDBG_RW),
	DDBG_REG("STE_HSI_TX_PRIORITY", STE_HSI_TX_PRIORITY, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BURSTLEN", STE_HSI_TX_BURSTLEN, DDBG_RW),
	DDBG_REG("STE_HSI_TX_PREAMBLE", STE_HSI_TX_PREAMBLE, DDBG_RW),
	DDBG_REG("STE_HSI_TX_DATASWAP", STE_HSI_TX_DATASWAP, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN0", STE_HSI_TX_FRAMELENX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN1", STE_HSI_TX_FRAMELENX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN2", STE_HSI_TX_FRAMELENX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN3", STE_HSI_TX_FRAMELENX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN4", STE_HSI_TX_FRAMELENX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN5", STE_HSI_TX_FRAMELENX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN6", STE_HSI_TX_FRAMELENX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_FRAMELEN7", STE_HSI_TX_FRAMELENX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER0", STE_HSI_TX_BUFFERX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER1", STE_HSI_TX_BUFFERX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER2", STE_HSI_TX_BUFFERX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER3", STE_HSI_TX_BUFFERX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER4", STE_HSI_TX_BUFFERX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER5", STE_HSI_TX_BUFFERX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER6", STE_HSI_TX_BUFFERX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BUFFER7", STE_HSI_TX_BUFFERX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE0", STE_HSI_TX_BASEX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE1", STE_HSI_TX_BASEX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE2", STE_HSI_TX_BASEX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE3", STE_HSI_TX_BASEX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE4", STE_HSI_TX_BASEX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE5", STE_HSI_TX_BASEX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE6", STE_HSI_TX_BASEX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_BASE7", STE_HSI_TX_BASEX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN0", STE_HSI_TX_SPANX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN1", STE_HSI_TX_SPANX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN2", STE_HSI_TX_SPANX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN3", STE_HSI_TX_SPANX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN4", STE_HSI_TX_SPANX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN5", STE_HSI_TX_SPANX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN6", STE_HSI_TX_SPANX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_SPAN7", STE_HSI_TX_SPANX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE0", STE_HSI_TX_GAUGEX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE1", STE_HSI_TX_GAUGEX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE2", STE_HSI_TX_GAUGEX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE3", STE_HSI_TX_GAUGEX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE4", STE_HSI_TX_GAUGEX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE5", STE_HSI_TX_GAUGEX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE6", STE_HSI_TX_GAUGEX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_GAUGE7", STE_HSI_TX_GAUGEX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK0", STE_HSI_TX_WATERMARKX, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK1", STE_HSI_TX_WATERMARKX+4, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK2", STE_HSI_TX_WATERMARKX+8, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK3", STE_HSI_TX_WATERMARKX+12, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK4", STE_HSI_TX_WATERMARKX+16, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK5", STE_HSI_TX_WATERMARKX+20, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK6", STE_HSI_TX_WATERMARKX+24, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARK7", STE_HSI_TX_WATERMARKX+28, DDBG_RW),
	DDBG_REG("STE_HSI_TX_DMAEN", STE_HSI_TX_DMAEN, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARKMIS", STE_HSI_TX_WATERMARKMIS, DDBG_RO),
	DDBG_REG("STE_HSI_TX_WATERMARKIM", STE_HSI_TX_WATERMARKIM, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARKIC", STE_HSI_TX_WATERMARKIC, DDBG_RW),
	DDBG_REG("STE_HSI_TX_WATERMARKID", STE_HSI_TX_WATERMARKID, DDBG_RW),
	DDBG_REG("STE_HSI_TX_ITCR", STE_HSI_TX_ITCR, DDBG_RW),
	DDBG_REG("STE_HSI_TX_ITIP", STE_HSI_TX_ITIP, DDBG_RW),
	DDBG_REG("STE_HSI_TX_ITOP1", STE_HSI_TX_ITOP1, DDBG_RW),
	DDBG_REG("STE_HSI_TX_ITOP2", STE_HSI_TX_ITOP2, DDBG_RW),
	DDBG_REG("STE_HSI_TX_PERIPHID0", STE_HSI_TX_PERIPHID0, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PERIPHID1", STE_HSI_TX_PERIPHID1, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PERIPHID2", STE_HSI_TX_PERIPHID2, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PERIPHID3", STE_HSI_TX_PERIPHID3, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PCELLID0", STE_HSI_TX_PCELLID0, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PCELLID1", STE_HSI_TX_PCELLID1, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PCELLID2", STE_HSI_TX_PCELLID2, DDBG_RO),
	DDBG_REG("STE_HSI_TX_PCELLID3", STE_HSI_TX_PCELLID3, DDBG_RO),
	DDBG_REG_NULL,
};

/* HSIR registers list */
static struct ddbg_register ddbg_hsir_registers[] = {
	DDBG_REG("STE_HSI_RX_ID", STE_HSI_RX_ID, DDBG_RO),
	DDBG_REG("STE_HSI_RX_MODE", STE_HSI_RX_MODE, DDBG_RW),
	DDBG_REG("STE_HSI_RX_STATE", STE_HSI_RX_STATE, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFSTATE", STE_HSI_RX_BUFSTATE, DDBG_RW),
	DDBG_REG("STE_HSI_RX_THRESHOLD", STE_HSI_RX_THRESHOLD, DDBG_RW),
	DDBG_REG("STE_HSI_RX_DETECTOR", STE_HSI_RX_DETECTOR, DDBG_RW),
	DDBG_REG("STE_HSI_RX_EXCEP", STE_HSI_RX_EXCEP, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ACK", STE_HSI_RX_ACK, DDBG_RW),
	DDBG_REG("STE_HSI_RX_CHANNELS", STE_HSI_RX_CHANNELS, DDBG_RW),
	DDBG_REG("STE_HSI_RX_REALTIME", STE_HSI_RX_REALTIME, DDBG_RW),
	DDBG_REG("STE_HSI_RX_OVERRUN", STE_HSI_RX_OVERRUN, DDBG_RW),
	DDBG_REG("STE_HSI_RX_OVERRUNACK", STE_HSI_RX_OVERRUNACK, DDBG_RW),
	DDBG_REG("STE_HSI_RX_PREAMBLE", STE_HSI_RX_PREAMBLE, DDBG_RW),
	DDBG_REG("STE_HSI_RX_PIPEGAUGE", STE_HSI_RX_PIPEGAUGE, DDBG_RW),
	DDBG_REG("STE_HSI_RX_STATICCONFID", STE_HSI_RX_STATICCONFID, DDBG_RO),
	DDBG_REG("STE_HSI_RX_BUFFER0", STE_HSI_RX_BUFFERX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER1", STE_HSI_RX_BUFFERX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER2", STE_HSI_RX_BUFFERX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER3", STE_HSI_RX_BUFFERX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER4", STE_HSI_RX_BUFFERX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER5", STE_HSI_RX_BUFFERX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER6", STE_HSI_RX_BUFFERX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BUFFER7", STE_HSI_RX_BUFFERX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN0", STE_HSI_RX_FRAMELENX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN1", STE_HSI_RX_FRAMELENX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN2", STE_HSI_RX_FRAMELENX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN3", STE_HSI_RX_FRAMELENX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN4", STE_HSI_RX_FRAMELENX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN5", STE_HSI_RX_FRAMELENX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN6", STE_HSI_RX_FRAMELENX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMELEN7", STE_HSI_RX_FRAMELENX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE0", STE_HSI_RX_BASEX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE1", STE_HSI_RX_BASEX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE2", STE_HSI_RX_BASEX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE3", STE_HSI_RX_BASEX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE4", STE_HSI_RX_BASEX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE5", STE_HSI_RX_BASEX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE6", STE_HSI_RX_BASEX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_BASE7", STE_HSI_RX_BASEX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN0", STE_HSI_RX_SPANX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN1", STE_HSI_RX_SPANX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN2", STE_HSI_RX_SPANX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN3", STE_HSI_RX_SPANX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN4", STE_HSI_RX_SPANX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN5", STE_HSI_RX_SPANX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN6", STE_HSI_RX_SPANX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_SPAN7", STE_HSI_RX_SPANX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE0", STE_HSI_RX_GAUGEX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE1", STE_HSI_RX_GAUGEX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE2", STE_HSI_RX_GAUGEX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE3", STE_HSI_RX_GAUGEX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE4", STE_HSI_RX_GAUGEX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE5", STE_HSI_RX_GAUGEX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE6", STE_HSI_RX_GAUGEX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_GAUGE7", STE_HSI_RX_GAUGEX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK0", STE_HSI_RX_WATERMARKX, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK1", STE_HSI_RX_WATERMARKX+4, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK2", STE_HSI_RX_WATERMARKX+8, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK3", STE_HSI_RX_WATERMARKX+12, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK4", STE_HSI_RX_WATERMARKX+16, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK5", STE_HSI_RX_WATERMARKX+20, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK6", STE_HSI_RX_WATERMARKX+24, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARK7", STE_HSI_RX_WATERMARKX+28, DDBG_RW),
	DDBG_REG("STE_HSI_RX_FRAMEBURSTCNT", STE_HSI_RX_FRAMEBURSTCNT, DDBG_RW),
	DDBG_REG("STE_HSI_RX_DMAEN", STE_HSI_RX_DMAEN, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARKMIS", STE_HSI_RX_WATERMARKMIS, DDBG_RO),
	DDBG_REG("STE_HSI_RX_WATERMARKIM", STE_HSI_RX_WATERMARKIM, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARKIC", STE_HSI_RX_WATERMARKIC, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARKID", STE_HSI_RX_WATERMARKID, DDBG_RW),
	DDBG_REG("STE_HSI_RX_OVERRUNMIS", STE_HSI_RX_OVERRUNMIS, DDBG_RO),
	DDBG_REG("STE_HSI_RX_OVERRUNIM", STE_HSI_RX_OVERRUNIM, DDBG_RW),
	DDBG_REG("STE_HSI_RX_EXCEPMIS", STE_HSI_RX_EXCEPMIS, DDBG_RO),
	DDBG_REG("STE_HSI_RX_EXCEPIM", STE_HSI_RX_EXCEPIM, DDBG_RW),
	DDBG_REG("STE_HSI_RX_WATERMARKIS", STE_HSI_RX_WATERMARKIS, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ITCR", STE_HSI_RX_ITCR, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ITIP", STE_HSI_RX_ITIP, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ITOP1", STE_HSI_RX_ITOP1, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ITOP2", STE_HSI_RX_ITOP2, DDBG_RW),
	DDBG_REG("STE_HSI_RX_ITOP3", STE_HSI_RX_ITOP3, DDBG_RW),
	DDBG_REG("STE_HSI_RX_PERIPHID0", STE_HSI_RX_PERIPHID0, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PERIPHID1", STE_HSI_RX_PERIPHID1, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PERIPHID2", STE_HSI_RX_PERIPHID2, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PERIPHID3", STE_HSI_RX_PERIPHID3, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PCELLID0", STE_HSI_RX_PCELLID0, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PCELLID1", STE_HSI_RX_PCELLID1, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PCELLID2", STE_HSI_RX_PCELLID2, DDBG_RO),
	DDBG_REG("STE_HSI_RX_PCELLID3", STE_HSI_RX_PCELLID3, DDBG_RO),
	DDBG_REG_NULL,
};

static int ste_hsi_ddbg_hsit_write(struct ddbg_target *ddbg, u32 val)
{
	int err;
	struct hsi_controller *hsi_ddbg;
	struct ste_hsi_controller *ste_hsi_ddbg;

	if (!ddbg)
		return -EINVAL;

	hsi_ddbg = (struct hsi_controller *) ddbg->dev_data;
	if (!hsi_ddbg)
		return -ENXIO;

	ste_hsi_ddbg = hsi_controller_drvdata(hsi_ddbg);
	if (!ste_hsi_ddbg)
		return -ENXIO;

	if (ste_hsi_ddbg->regulator)
		regulator_enable(ste_hsi_ddbg->regulator);

	err = ste_hsi_clock_enable(hsi_ddbg);
	if (unlikely(err))
		goto out;

	spin_lock_bh(&ste_hsi_ddbg->lock);
	writel(val, ste_hsi_ddbg->tx_base + ddbg->curr->offset);
	spin_unlock_bh(&ste_hsi_ddbg->lock);

	ste_hsi_clock_disable(hsi_ddbg);

out:
	if (ste_hsi_ddbg->regulator)
		regulator_disable(ste_hsi_ddbg->regulator);

	return err;
}

static int ste_hsi_ddbg_hsit_read(struct ddbg_target *ddbg, u32 *val)
{
	int err;
	struct hsi_controller *hsi_ddbg;
	struct ste_hsi_controller *ste_hsi_ddbg;

	if (!ddbg)
		return -EINVAL;

	hsi_ddbg = (struct hsi_controller *) ddbg->dev_data;
	if (!hsi_ddbg)
		return -ENXIO;

	ste_hsi_ddbg = hsi_controller_drvdata(hsi_ddbg);
	if (!ste_hsi_ddbg)
		return -ENXIO;

	if (ste_hsi_ddbg->regulator)
		regulator_enable(ste_hsi_ddbg->regulator);

	err = ste_hsi_clock_enable(hsi_ddbg);
	if (unlikely(err))
		goto out;

	spin_lock_bh(&ste_hsi_ddbg->lock);
	*val = readl(ste_hsi_ddbg->tx_base +  ddbg->curr->offset);
	spin_unlock_bh(&ste_hsi_ddbg->lock);

	ste_hsi_clock_disable(hsi_ddbg);

out:
	if (ste_hsi_ddbg->regulator)
		regulator_disable(ste_hsi_ddbg->regulator);

	return err;
}

static struct ddbg_target ddbg_hsit = {
	.name = "hsit",
	.phyaddr = U8500_HSIT_BASE,
	.reg = ddbg_hsit_registers,
	.read_reg = ste_hsi_ddbg_hsit_read,
	.write_reg = ste_hsi_ddbg_hsit_write,
};

static int ste_hsi_ddbg_hsir_write(struct ddbg_target *ddbg, u32 val)
{
	int err;
	struct hsi_controller *hsi_ddbg;
	struct ste_hsi_controller *ste_hsi_ddbg;

	if (!ddbg)
		return -EINVAL;

	hsi_ddbg = (struct hsi_controller *) ddbg->dev_data;
	if (!hsi_ddbg)
		return -ENXIO;

	ste_hsi_ddbg = hsi_controller_drvdata(hsi_ddbg);
	if (!ste_hsi_ddbg)
		return -ENXIO;

	if (ste_hsi_ddbg->regulator)
		regulator_enable(ste_hsi_ddbg->regulator);

	err = ste_hsi_clock_enable(hsi_ddbg);
	if (unlikely(err))
		goto out;

	spin_lock_bh(&ste_hsi_ddbg->lock);
	writel(val, ste_hsi_ddbg->rx_base +  ddbg->curr->offset);
	spin_unlock_bh(&ste_hsi_ddbg->lock);

	ste_hsi_clock_disable(hsi_ddbg);

out:
	if (ste_hsi_ddbg->regulator)
		regulator_disable(ste_hsi_ddbg->regulator);

	return err;
}

static int ste_hsi_ddbg_hsir_read(struct ddbg_target *ddbg, u32 *val)
{
	int err;
	struct hsi_controller *hsi_ddbg;
	struct ste_hsi_controller *ste_hsi_ddbg;

	if (!ddbg)
		return -EINVAL;

	hsi_ddbg = (struct hsi_controller *) ddbg->dev_data;
	if (!hsi_ddbg)
		return -ENXIO;

	ste_hsi_ddbg = hsi_controller_drvdata(hsi_ddbg);
	if (!ste_hsi_ddbg)
		return -ENXIO;

	if (ste_hsi_ddbg->regulator)
		regulator_enable(ste_hsi_ddbg->regulator);

	err = ste_hsi_clock_enable(hsi_ddbg);
	if (unlikely(err))
		goto out;

	spin_lock_bh(&ste_hsi_ddbg->lock);
	*val = readl(ste_hsi_ddbg->rx_base +  ddbg->curr->offset);
	spin_unlock_bh(&ste_hsi_ddbg->lock);

	ste_hsi_clock_disable(hsi_ddbg);

out:
	if (ste_hsi_ddbg->regulator)
		regulator_disable(ste_hsi_ddbg->regulator);

	return err;
}

static struct ddbg_target ddbg_hsir = {
	.name = "hsir",
	.phyaddr = U8500_HSIR_BASE,
	.reg = ddbg_hsir_registers,
	.read_reg = ste_hsi_ddbg_hsir_read,
	.write_reg = ste_hsi_ddbg_hsir_write,
};

int ste_hsi_deep_debug_init(struct hsi_controller *hsi)
{
	int ret;

	if (!hsi) {
		ret = -EINVAL;
		goto out;
	}
	ddbg_hsit.dev_data = hsi;
	ddbg_hsir.dev_data = hsi;

	ret = deep_debug_regaccess_register(&ddbg_hsit);
	if (ret)
		pr_err("%s: hsit deep debug not registered\n", __func__);

	ret = deep_debug_regaccess_register(&ddbg_hsir);
	if (ret)
		pr_err("%s: hsir deep debug not registered\n", __func__);
out:
	return ret;
}

int ste_hsi_deep_debug_exit(void)
{
	int ret;

	ret = deep_debug_regaccess_unregister(&ddbg_hsit);
	if (ret)
		pr_err("%s: hsit deep debug not unregistered\n", __func__);

	deep_debug_regaccess_unregister(&ddbg_hsir);
	if (ret)
		pr_err("%s: hsir deep debug not unregistered\n", __func__);

	return ret;
}
