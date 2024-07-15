/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_spi.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      SPI firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_SPI_H
#define __CH564_SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

/**
 * @brief  SPI0 interrupt bit define
 */
#define SPI0_IT_FST_BYTE RB_SPI_IE_FST_BYTE
#define SPI0_IT_FIFO_OV RB_SPI_IE_FIFO_OV
#define SPI0_IT_DMA_END RB_SPI_IE_DMA_END
#define SPI0_IT_FIFO_HF RB_SPI_IE_FIFO_HF
#define SPI0_IT_BYTE_END RB_SPI_IE_BYTE_END
#define SPI0_IT_CNT_END RB_SPI_IE_CNT_END

#define SPI_MAX_DELAY 0xffff

/**
 * @brief  Configuration data mode
 */
typedef enum
{
    Mode0_HighBitINFront,
    Mode3_HighBitINFront,
} ModeBitOrderTypeDef;

/**
 * @brief  Configuration SPI slave mode
 */
typedef enum
{
    Mode_DataStream = 0,
    Mose_FirstCmd,
} Slave_ModeTypeDef;

/**************** SPI0 */
void SPI0_MasterInit(uint32_t clockRate);
void SPI0_DataMode(ModeBitOrderTypeDef m);

void SPI0_MasterSendByte(uint8_t d);
uint8_t SPI0_MasterRecvByte(void);

void SPI0_MasterTrans(uint8_t *pbuf, uint16_t len);
void SPI0_MasterRecv(uint8_t *pbuf, uint16_t len);

void SPI0_MasterDMATrans(uint8_t *pbuf, uint32_t len);
void SPI0_MasterDMARecv(uint8_t *pbuf, uint32_t len);
void SPI0_MasterTransRecv(uint8_t *ptbuf, uint8_t *prbuf, uint16_t len);

void SPI0_SlaveInit(uint32_t clockRate);
#define SetFirst0Data(d) (R8_SPI0_SLAVE_PRE = (d))
void SPI0_SlaveSendByte(uint8_t d);
uint8_t SPI0_SlaveRecvByte(void);

uint8_t SPI0_SlaveTrans(uint8_t *pbuf, uint16_t len,uint16_t timeouts);
uint8_t SPI0_SlaveRecv(uint8_t *pbuf, uint16_t len,uint16_t timeouts);

// refer to SPI0 interrupt bit define
#define SPI0_MODE_CFG(cfglist, en) BITS_CFG(R8_SPI0_CTRL_MOD, cfglist, en)
#define SPI0_ITCfg(cfglist, en) BITS_CFG(R8_SPI0_INTER_EN, cfglist, en)
#define SPI0_SET_CLOCK_DIV(div) (R8_SPI0_CLOCK_DIV = (div))
#define SPI0_GetITFlag(f) (R8_SPI0_INT_FLAG & (f))
#define SPI0_ClearITFlag(f) (R8_SPI0_INT_FLAG = (f))
#define SPI0_SET_RST(dat) (R8_SPI0_RESET_CMD = (dat))
#define SPI0_GET_RST() (R8_SPI0_RESET_CMD)
#define SPI0_GET_BUSY() (R8_SPI0_BUSY)
#define SPI0_GET_BUFFER() (R8_SPI0_BUFFER)
#define SPI0_SET_BUFFER(dat) (R8_SPI0_BUFFER = (dat))
#define SPI0_CLEAR_FIFO() (R8_SPI0_CTRL_MOD |= RB_SPI_ALL_CLEAR);
#define SPI0_GET_FIFO() (R8_SPI0_FIFO)
#define SPI0_SET_FIFO(dat) (R8_SPI0_FIFO = (dat))
#define SPI0_SET_FIFO_CNT(cnt) (R8_SPI0_FIFO_COUNT = (cnt))
#define SPI0_GET_FIFO_CNT() (R8_SPI0_FIFO_COUNT)
#define SPI0_SET_TOTAL_CNT(cnt) (R16_SPI0_TOTAL_CNT = (cnt) )
#define SPI0_GET_TOTAL_CNT() (R16_SPI0_TOTAL_CNT)

#define SPI0_SET_DMA_MODE(cfglist, en) BITS_CFG(R8_SPI0_CTRL_DMA, cfglist, en)
#define SPI0_SET_DMA_RANGE(start, end)                                                                                 \
({                                                                                                                 \
    R32_SPI0_DMA_BEG = (uint32_t)(start) & MASK_SPI0_DMA_ADDR;                                                                 \
    R32_SPI0_DMA_END = (uint32_t)(end) & MASK_SPI0_DMA_ADDR;                                                                    \
})

/**************** SPI1 */
void SPI1_MasterInit(uint32_t clockRate);
void SPI1_DataMode(ModeBitOrderTypeDef m);

void SPI1_MasterSendByte(uint8_t d);
uint8_t SPI1_MasterRecvByte(void);

void SPI1_MasterTrans(uint8_t *pbuf, uint16_t len);
void SPI1_MasterRecv(uint8_t *pbuf, uint16_t len);

void SPI1_SlaveInit(uint32_t clockRate);
#define SetFirst1Data(d) (R8_SPI1_SLAVE_PRE = (d))
void SPI1_SlaveSendByte(uint8_t d);
uint8_t SPI1_SlaveRecvByte(void);

uint8_t SPI1_SlaveTrans(uint8_t *pbuf, uint16_t len,uint16_t timeouts);
uint8_t SPI1_SlaveRecv(uint8_t *pbuf, uint16_t len,uint16_t timeouts);

// refer to SPI1 interrupt bit define
#define SPI1_MODE_CFG(cfglist, en) BITS_CFG(R8_SPI1_CTRL_MOD, cfglist, en)
#define SPI1_ITCfg(cfglist, en) BITS_CFG(R8_SPI1_INTER_EN, cfglist, en)
#define SPI1_SET_CLOCK_DIV(div) (R8_SPI1_CLOCK_DIV = (div))
#define SPI1_GetITFlag(f) (R8_SPI1_INT_FLAG & (f))
#define SPI1_ClearITFlag(f) (R8_SPI1_INT_FLAG = (f))
#define SPI1_GET_BUFFER() (R8_SPI1_BUFFER)
#define SPI1_SET_BUFFER(dat) (R8_SPI1_BUFFER = (dat))
#define SPI1_CLEAR_FIFO() (R8_SPI1_CTRL_MOD |= RB_SPI_ALL_CLEAR);
#define SPI1_GET_FIFO() (R8_SPI1_FIFO)
#define SPI1_SET_FIFO(dat) (R8_SPI1_FIFO = (dat))
#define SPI1_SET_FIFO_CNT(cnt) (R8_SPI1_FIFO_COUNT = (cnt))
#define SPI1_GET_FIFO_CNT() (R8_SPI1_FIFO_COUNT)
#define SPI1_SET_TOTAL_CNT(cnt) (R16_SPI1_TOTAL_CNT = (cnt))
#define SPI1_GET_TOTAL_CNT() (R16_SPI1_TOTAL_CNT)

#define SPI1_SET_DMA_MODE(cfglist, en) BITS_CFG(R8_SPI1_CTRL_DMA, (cfglist), (en))

#ifdef __cplusplus
}
#endif

#endif
