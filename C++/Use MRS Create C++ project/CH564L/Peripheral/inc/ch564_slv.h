/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_slv.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      SLV firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_SLV_H
#define __CH564_SLV_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

    typedef enum
    {
        slv_data,
        slv_cmd,
        slv_timeout
    } SLV_STA;

#define SLV_CFG(cfglist, en) (BITS_CFG(R8_SLV_CONFIG, cfglist, en))
#define SLV_SEND_DATA(data) (R8_SLV_DOUT = (data))
#define SLV_SEND_STA(status) (R8_SLV_STATUS = (status))
#define SLV_GET_IF(RB_IF_SLV) (R8_INT_FLAG_SLV & (RB_IF_SLV))
#define SLV_CLEAR_IF(RB_IF_SLV) (R8_INT_FLAG_SLV |= (RB_IF_SLV))
#define SLV_GET_DATA() (R8_INT_SLV_DIN)
#define SLV_DMA_CFG(cfglist, en) (BITS_CFG(R8_DMA_EN_SLV, cfglist, en))
#define SLV_SET_MODE_CTRL(cfglist, en) (BITS_CFG(R8_DMA_MODE_CTRL_SLV, cfglist, en))
#define SLV_SET_MODE_EN(cfglist, en) (BITS_CFG(R8_DMA_MODE_EN_SLV, cfglist, en))

#define SLV_DMA_GET_IF(slv_dma_if) (R8_DMA_INT_FLAG_SLV & (slv_dma_if))
#define SLV_DMA_CLEAR_IF(slv_dma_if) (R8_DMA_INT_FLAG_SLV |= (slv_dma_if))
#define SLV_DMA_START_ADDR_RD(address) (R32_RD_DMA_START_ADDR_SLV = (uint32_t)(address))

#define SLV_DMA_END_ADDR_RD(address) (R32_RD_DMA_END_ADDR_SLV = (uint32_t)(address))

#define SLV_DMA_START_ADDR_WR(address) (R32_WR_DMA_START_ADDR_SLV = (uint32_t)(address))

#define SLV_DMA_END_ADDR_WR(address) (R32_WR_DMA_END_ADDR_SLV = (uint32_t)(address))

#define SLV_DMA_GET_NOW_ADDR() (R32_DMA_END_NOW_SLV)

#define SLV_SET_DMA_CMD0(cmd) (R8_DMA_CMD0_SLV = (cmd))

#define SLV_SET_DMA_CMD1(cmd) (R8_DMA_CMD1_SLV = (cmd))
#define SLV_SET_RST_CMD(cmd) (R8_SLV_RESET_CMD = (cmd))

#define SLV_SET_SOCKET(cfglist, en) (BITS_CFG(R32_SLV_SOCKET, cfglist, en))

#define SLV_SET_BASE_ADDR_BEG(address) (R32_SLV_BASE_REG = (address))
#define SLV_GET_OTHER_DATA() (R8_OTHER_DATA)
#define SLV_GET_OTHER_DATA_LEN() (R8_OTHER_DATA_LEN)
#define SLV_GET_DMA_DEC_LEN() (R16_DMA_DEC_LEN)
#define SLV_GET_DMA_DEC_OFFSET() (R16_DMA_DEC_OFFSET)

SLV_STA SLV_Read(uint8_t *dataAddress, uint16_t dataSize, uint16_t timeout);
ErrorStatus SLV_SendDATA(uint8_t *data, uint16_t datasize);
ErrorStatus SLV_SendSTA(uint8_t *sta, uint16_t datasize);

#ifdef __cplusplus
}
#endif

#endif
