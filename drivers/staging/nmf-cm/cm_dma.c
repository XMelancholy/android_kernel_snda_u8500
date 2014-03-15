/*
 * Copyright (C) ST-Ericsson SA 2010
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <mach/db8500-regs.h>

#include "cm_dma.h"

#define CMDMA_LIDX (2)
#define CMDMA_REG_LCLA (0x024)

static void __iomem *virtbase = NULL;

static int cmdma_write_cyclic_list_mem2per(
    unsigned int from_addr,
    unsigned int to_addr,
    unsigned int segments,
    unsigned int segmentsize,
    unsigned int LOS);

static int cmdma_write_cyclic_list_per2mem(
    unsigned int from_addr,
    unsigned int to_addr,
    unsigned int segments,
    unsigned int segmentsize,
    unsigned int LOS);

static bool cmdma_setup_relink_area_called = false;
static bool cscall_cmdma_setup_relink_area_called = false;

#define MSP4_ADDRESS                0x80116000

int cmdma_setup_relink_area( unsigned int mem_addr,
    unsigned int per_addr,
    unsigned int segments,
    unsigned int segmentsize,
    unsigned int LOS,
    enum cmdma_type type)
{
	if  (per_addr != MSP4_ADDRESS)
		/* audiocodec case */
		cmdma_setup_relink_area_called = true;

	else
		/* cscall_mpc case, using MSP4 */
		cscall_cmdma_setup_relink_area_called = true;

    switch (type) {

    case CMDMA_MEM_2_PER:
        return cmdma_write_cyclic_list_mem2per(
            mem_addr,
            per_addr,
            segments,
            segmentsize,
            LOS);

    case CMDMA_PER_2_MEM:
        return cmdma_write_cyclic_list_per2mem(
            per_addr,
            mem_addr,
            segments,
            segmentsize,
            LOS);

    default :
        return -EINVAL;
    }
 }

 static unsigned int cmdma_getlcla( void) {

    if(!virtbase)
        virtbase = ioremap(U8500_DMA_BASE, CMDMA_REG_LCLA + sizeof(int) );

    return readl(virtbase + CMDMA_REG_LCLA);
 }

 static void cmdma_write_relink_params_mem2per (
    int * relink,
    unsigned int LOS,
    unsigned int nb_element,
    unsigned int src_addr,
    unsigned int dst_addr,
    unsigned int burst_size) {

    relink[0] =  (((long)(nb_element & 0xFFFF)) << 16) |
        (src_addr & 0xFFFF);

    relink[1] = (((src_addr >> 16)  & 0xFFFFUL) << 16) |
        (0x1200UL | (LOS << 1) | (burst_size<<10));

    relink[2] = ((nb_element & 0xFFFF) << 16)  |
        (dst_addr & 0xFFFF);

    relink[3] = (((dst_addr >> 16)  & 0xFFFFUL) << 16 ) |
        0x8201UL | ((LOS+1) << 1) | (burst_size<<10);
}

static void cmdma_write_relink_params_per2mem (
    int * relink,
    unsigned int LOS,
    unsigned int nb_element,
    unsigned int src_addr,
    unsigned int dst_addr,
    unsigned int burst_size) {

    relink[0] =  (((long)(nb_element & 0xFFFF)) << 16) |
        (src_addr & 0xFFFF);

    relink[1] = (((src_addr >> 16)  & 0xFFFFUL) << 16) |
        (0x8201UL | (LOS << 1) | (burst_size<<10));

    relink[2] = ((nb_element & 0xFFFF) << 16)  |
        (dst_addr & 0xFFFF);

    relink[3] = (((dst_addr >> 16)  & 0xFFFFUL) << 16 ) |
        0x1200UL | ((LOS+1) << 1) | (burst_size<<10);
}

static int cmdma_write_cyclic_list_mem2per(
    unsigned int from_addr,
    unsigned int to_addr,
    unsigned int segments,
    unsigned int segmentsize,
    unsigned int LOS) {

    unsigned int i,j;
    int *relink;

    j = LOS;

    for ( i = 0; i < segments; i++) {
        relink = ioremap_nocache (cmdma_getlcla() + 1024 * CMDMA_LIDX + 8 * j, 4 * sizeof(int));

        if (i == (segments-1))
                j = LOS;
            else
                j += 2;

        cmdma_write_relink_params_mem2per (
            relink,
            j,
            segmentsize / 4,
            from_addr,
            to_addr,
            0x2);

	iounmap(relink);

        from_addr += segmentsize;
	}

    return 0;
}

static int cmdma_write_cyclic_list_per2mem(
    unsigned int from_addr,
    unsigned int to_addr,
    unsigned int segments,
    unsigned int segmentsize,
    unsigned int LOS) {

    unsigned int i,j;
    int *relink;
    j = LOS;

    for ( i = 0; i < segments; i++) {
        relink = ioremap_nocache (cmdma_getlcla() + 1024 * CMDMA_LIDX + 8 * j, 4 * sizeof(int));

        if (i == (segments-1))
            j = LOS;
        else
            j += 2;

        cmdma_write_relink_params_per2mem (
            relink,
            j,
            segmentsize / 4,
            from_addr,
            to_addr,
            0x2);

	iounmap(relink);

        to_addr += segmentsize;
    }

    return 0;
}

static void __iomem *dmabase = 0;
int cmdma_init(void)
{
	dmabase = ioremap_nocache(U8500_DMA_BASE, PAGE_SIZE);
	if (dmabase == NULL)
		return -ENOMEM;
	else
		return 0;
}

void cmdma_destroy(void)
{
	iounmap(dmabase);
}

#define SSLNK_CHAN_2 (0x40C + 0x20 * 2)
#define SDLNK_CHAN_2 (0x41C + 0x20 * 2)

void cmdma_stop_dma(void)
{
	unsigned int tmp_register_sslnk;
	unsigned int tmp_register_sdlnk;

	if (cmdma_setup_relink_area_called) {
		cmdma_setup_relink_area_called = false;
		pr_err("cmdma_stop_dma (audiocodec channel)");

		/* Checks if in disable state or not
		(should already be set by audiocodec component) */
		tmp_register_sslnk = readl(dmabase + SSLNK_CHAN_2);
		if (tmp_register_sslnk & (0x3 << 28))
			printk(KERN_ERR "CM: ERROR - RX DMA was running\n");

		/* Checks if in disable state or not
		(should already be set by audiocodec component) */
		tmp_register_sdlnk = readl(dmabase + SDLNK_CHAN_2);
		if  (tmp_register_sdlnk & (0x3 << 28))
			printk(KERN_ERR "CM: ERROR - TX DMA was running\n");

		/* Transitions to disable state MSP1 and MSP3
		Rx logical channels */
		/* Sets to 0 MSP1 and MSP3 event lines */
		tmp_register_sslnk &= ~(3 << 28);
		/* Sets to 2 (for Suspend_Req) MSP1 and MSP3 event lines */
		tmp_register_sslnk |= (2 << 28);
		/* Pushes it in DMA register */
		writel(tmp_register_sslnk, dmabase + SSLNK_CHAN_2);
		/* Polls on Disable state (HW transition) */
		while (readl(dmabase + SSLNK_CHAN_2) & (0x3 << 28));

		/* Transitions to disable state MSP1 and MSP3
		Tx logical channels */
		/* Sets to 0 MSP1 and MSP3 event lines */
		tmp_register_sdlnk &= ~(3 << 28);
		/* Sets to 2 (for Suspend_Req) MSP1 and MSP3 event lines */
		tmp_register_sdlnk |= (2 << 28);
		/* Pushes it in DMA register */
		writel(tmp_register_sdlnk, dmabase + SDLNK_CHAN_2);
		/* Polls on Disable state (HW transition) */
		while (readl(dmabase + SDLNK_CHAN_2) & (0x3 << 28));
	}

	if (cscall_cmdma_setup_relink_area_called) {
		cscall_cmdma_setup_relink_area_called = false;
		pr_err("cmdma_stop_dma (cscall_mpc channel)");

		/* Checks if in disable state or not
		(should already be set by cscall_mpc component) */
		tmp_register_sslnk = readl(dmabase + SSLNK_CHAN_2);
		if (tmp_register_sslnk & (0x3 << 8))
			printk(KERN_ERR "CM: ERROR - RX DMA was running for MSP4\n");

		/* Checks if in disable state or not
		(should already be set by cscall_mpc component) */
		tmp_register_sdlnk = readl(dmabase + SDLNK_CHAN_2);
		if (tmp_register_sdlnk & (0x3 << 10))
			printk(KERN_ERR "CM: ERROR - TX DMA was running for MSP4\n");

		/* Transitions to disable state MSP4 Rx logical channel */
		/* Sets to 0 MSP4 Rx event line */
		tmp_register_sslnk &= ~(3 << 8);
		/* Sets to 2 (for Suspend_Req) MSP4 event line */
		tmp_register_sslnk |= (2 << 8);
		/* Pushes it in DMA register */
		writel(tmp_register_sslnk, dmabase + SSLNK_CHAN_2);
		/* Polls on Disable state (HW transition) */
		while (readl(dmabase + SSLNK_CHAN_2) & (0x3 << 8));

		/* Transitions to disable state MSP4 Tx logical channel */
		/* Sets to 0 MSP4 Tx event line */
		tmp_register_sdlnk &= ~(3 << 10);
		/* Sets to 2 (for Suspend_Req) MSP4 Tx event line */
		tmp_register_sdlnk |= (2 << 10);
		/* Pushes it in DMA register */
		writel(tmp_register_sdlnk, dmabase + SDLNK_CHAN_2);
		/* Polls on Disable state (HW transition) */
		while (readl(dmabase + SDLNK_CHAN_2) & (0x3 << 10));
	}
}
