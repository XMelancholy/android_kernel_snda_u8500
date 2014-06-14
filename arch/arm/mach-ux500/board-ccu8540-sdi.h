/*
 * board-ccu8540-sdi.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author:  Xiaomei ZHANG <xiaomei.zhang@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 *
 * Specific initialization U8540 CCU boards.
 */
#ifndef	__BOARD_CCU8540_SDI_H
#define	__BOARD_CCU8540_SDI_H

/* CCU8540 GPIO for MMC Card detect */
#define CCU8540_SDMMC_CD_GPIO		230

void __init ccu8540_sdi_init(struct device *parent);
#endif