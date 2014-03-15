/*
 * hva_regs.h
 *
 * Copyright (C) ST-Ericsson SA 2012
 * Author <thomas.costis@stericsson.com> for ST-Ericsson.
 * License terms  GNU General Public License (GPL), version 2
 */

/*!
 * @brief HVA Registers
 * This file contains the HVA register definition
 */


#ifndef HVA_REGS_DEF_H
#define HVA_REGS_DEF_H

#define HVA_HIF_REG_RST                 0x0100U
#define HVA_HIF_REG_RST_ACK             0x0104U
#define HVA_HIF_REG_MIF_CFG             0x0108U
#define HVA_HIF_REG_HEC_MIF_CFG         0x010CU
#define HVA_HIF_REG_CFL                 0x0110U
#define HVA_HIF_FIFO_CMD                0x0114U
#define HVA_HIF_FIFO_STS                0x0118U
#define HVA_HIF_REG_SFL                 0x011CU
#define HVA_HIF_REG_IT_ACK              0x0120U
#define HVA_HIF_REG_ERR_IT_ACK          0x0124U
#define HVA_HIF_REG_LMI_ERR             0x0128U
#define HVA_HIF_REG_EMI_ERR             0x012CU
#define HVA_HIF_REG_HEC_MIF_ERR         0x0130U
#define HVA_HIF_REG_HEC_STS             0x0134U
#define HVA_HIF_REG_HVC_STS             0x0138U
#define HVA_HIF_REG_HJE_STS             0x013CU
#define HVA_HIF_REG_CNT                 0x0140U
#define HVA_HIF_REG_HEC_CHKSYN_DIS      0x0144U
#define HVA_HIF_REG_CLK_GATING          0x0148U
#define HVA_HIF_REG_VERSION             0x014CU
#define HVA_HIF_REG_BSM                 0x0150U

/* HVA Versions */
#define HVA_VERSION_MPE41       0x10D
#define HVA_VERSION_MPE42       0x1AC
#define HVA_VERSION_9540        0x1FA
#define HVA_VERSION_9600        0x101
#define HVA_VERSION_8540        0x251
#define HVA_VERSION_8540V2      0x111
#define HVA_VERSION_5500V2      0x235
#define HVA_VERSION_SVP_9540    0x11FA
#define HVA_VERSION_SVP_9600    0x1101
#define HVA_VERSION_SVP_8540    0x1251
#define HVA_VERSION_SVP_8540V2  0x1111

#endif  /*HVA_REGS_DEF_H*/

