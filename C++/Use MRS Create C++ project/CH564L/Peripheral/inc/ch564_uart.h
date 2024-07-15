/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_uart.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      UART firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_UART_H
#define __CH564_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

/**
 * @brief	Line Error Status Definition
 */
#define STA_ERR_BREAK RB_LSR_BREAK_ERR // Data Interval Error
#define STA_ERR_FRAME RB_LSR_FRAME_ERR // DataFrame error
#define STA_ERR_PAR RB_LSR_PAR_ERR     // Parity bit error
#define STA_ERR_FIFOOV RB_LSR_OVER_ERR // Receive Data Overflow

#define STA_TXFIFO_EMP RB_LSR_TX_FIFO_EMP // The current send FIFO is empty, you can continue to fill the send data
#define STA_TXALL_EMP RB_LSR_TX_ALL_EMP   // All currently sent data has been sent
#define STA_RECV_DATA RB_LSR_DATA_RDY     // Data is currently received

    /**
     * @brief  Serial port byte trigger configuration
     */
    typedef enum
    {
        UART_1BYTE_TRIG = 0, // 1 byte trigger
        UART_2BYTE_TRIG = 1, // 2 byte trigger
        UART_4BYTE_TRIG = 2, // 4 byte trigger
        UART_7BYTE_TRIG = 3, // 7 byte trigger

    } UARTByteTRIGTypeDef;

    /****************** UART0 */
    void UART0_DefInit(void);                      /* Serial port default initialization configuration */
    void UART0_BaudRateCfg(uint32_t baudrate);     /* Serial port baud rate configuration */
    void UART0_ByteTrigCfg(UARTByteTRIGTypeDef b); /* Serial byte trigger interrupt configuration */
    void UART0_INTCfg(uint8_t s, uint8_t i);       /* Serial port interrupt configuration */
    void UART0_Reset(void);                        /* Serial port software reset */

#define UART0_SET_DLV(dlv) ({ R8_UART0_DIV = (dlv); })

#define UART0_CLR_RXFIFO() (R8_UART0_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear the current receive FIFO */
#define UART0_CLR_TXFIFO() (R8_UART0_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current transmit FIFO */

#define UART0_GetITFlag() (R8_UART0_IIR & (RB_IIR_NO_INT | RB_IIR_INT_MASK)) /* Get the current interrupt flag */

#define UART0_SET_FCR(cfglist, en) BITS_CFG(R8_UART0_FCR, (cfglist), (en))
#define UART0_SET_LCR(cfglist, en) BITS_CFG(R8_UART0_LCR, (cfglist), (en))
#define UART0_SET_MCR(cfglist, en) BITS_CFG(R8_UART0_MCR, (cfglist), (en))

// please refer to LINE error and status define
#define UART0_GetLinSTA() (R8_UART0_LSR) /* Get the current communication status */
#define UART0_GetMSRSTA() (R8_UART0_MSR) /* Get the current flow control status, only applicable to UART0 */

#define UART0_DMACFG(cfglist, en) BITS_CFG(R8_UART0_DMA_CTRL, (cfglist), (en))
#define UART0_DMA_SET_RD_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART0_DMA_RD_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                           \
        R32_UART0_DMA_RD_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                                 \
    })
#define UART0_DMA_GET_RD_CURRENT_ADDR() (R32_UART0_DMA_RD_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_GET_RD_BEG_ADDR() (R32_UART0_DMA_RD_START_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_GET_RD_END_ADDR() (R32_UART0_DMA_RD_END_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_SET_WR_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART0_DMA_WR_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                          \
        R32_UART0_DMA_WR_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                              \
    })
#define UART0_DMA_GET_WR_CURRENT_ADDR() (R32_UART0_DMA_WR_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_GET_WR_BEG_ADDR() (R32_UART0_DMA_WR_START_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_GET_WR_END_ADDR() (R32_UART0_DMA_WR_END_ADDR & MASK_UART_DMA_ADDR)
#define UART0_DMA_GET_IT_FLAG(dmaif) (R8_UART0_DMA_IF & (dmaif))

#define UART0_SendByte(b) (R8_UART0_THR = (b))       /* Serial port single byte transmission */
    void UART0_SendString(uint8_t *buf, uint16_t l); /* Serial multi-byte transmission */
    void UART0_SendString_DMA(uint8_t *buf, uint32_t lenth);
#define UART0_RecvByte() (R8_UART0_RBR)      /* Serial port read single byte */
    uint16_t UART0_RecvString(uint8_t *buf); /* Serial port read multibyte */

    /****************** UART1 */
    void UART1_DefInit(void);                      /* Serial port default initialization configuration */
    void UART1_BaudRateCfg(uint32_t baudrate);     /* Serial port baud rate configuration */
    void UART1_ByteTrigCfg(UARTByteTRIGTypeDef b); /* Serial byte trigger interrupt configuration */
    void UART1_INTCfg(uint8_t s, uint8_t i);       /* Serial port interrupt configuration */
    void UART1_Reset(void);                        /* Serial port software reset */

#define UART1_SET_DLV(dlv) ({ R8_UART1_DIV = dlv; })

#define UART1_CLR_RXFIFO() (R8_UART1_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear the current receive FIFO */
#define UART1_CLR_TXFIFO() (R8_UART1_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current transmit FIFO */

#define UART1_GetITFlag() (R8_UART1_IIR & (RB_IIR_NO_INT | RB_IIR_INT_MASK)) /* Get the current interrupt flag */

#define UART1_SET_FCR(cfglist, en) BITS_CFG(R8_UART1_FCR, (cfglist), (en))
#define UART1_SET_LCR(cfglist, en) BITS_CFG(R8_UART1_LCR, (cfglist), (en))
#define UART1_SET_MCR(cfglist, en) BITS_CFG(R8_UART1_MCR, (cfglist), (en))

// please refer to LINE error and status define
#define UART1_GetLinSTA() (R8_UART1_LSR) /* Get the current communication status */
#define UART1_GetMSRSTA() (R8_UART1_MSR) /* Get the current flow control status, only applicable to UART1 */

#define UART1_DMACFG(cfglist, en) BITS_CFG(R8_UART1_DMA_CTRL, (cfglist), (en))
#define UART1_DMA_SET_RD_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART1_DMA_RD_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                           \
        R32_UART1_DMA_RD_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                                 \
    })
#define UART1_DMA_GET_RD_CURRENT_ADDR() (R32_UART1_DMA_RD_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_GET_RD_BEG_ADDR() (R32_UART1_DMA_RD_START_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_GET_RD_END_ADDR() (R32_UART1_DMA_RD_END_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_SET_WR_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART1_DMA_WR_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                          \
        R32_UART1_DMA_WR_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                              \
    })
#define UART1_DMA_GET_WR_CURRENT_ADDR() (R32_UART1_DMA_WR_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_GET_WR_BEG_ADDR() (R32_UART1_DMA_WR_START_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_GET_WR_END_ADDR() (R32_UART1_DMA_WR_END_ADDR & MASK_UART_DMA_ADDR)
#define UART1_DMA_GET_IT_FLAG(dmaif) (R8_UART1_DMA_IF & (dmaif))

#define UART1_SendByte(b) (R8_UART1_THR = (b))       /* Serial port single byte transmission */
    void UART1_SendString(uint8_t *buf, uint16_t l); /* Serial multi-byte transmission */
    void UART1_SendString_DMA(uint8_t *buf, uint32_t lenth);
#define UART1_RecvByte() (R8_UART1_RBR)      /* Serial port read single byte */
    uint16_t UART1_RecvString(uint8_t *buf); /* Serial port read multibyte */

    /****************** UART2 */
    void UART2_DefInit(void);                      /* Serial port default initialization configuration */
    void UART2_BaudRateCfg(uint32_t baudrate);     /* Serial port baud rate configuration */
    void UART2_ByteTrigCfg(UARTByteTRIGTypeDef b); /* Serial byte trigger interrupt configuration */
    void UART2_INTCfg(uint8_t s, uint8_t i);       /* Serial port interrupt configuration */
    void UART2_Reset(void);                        /* Serial port software reset */

#define UART2_SET_DLV(dlv) ({ R8_UART2_DIV = (dlv); })

#define UART2_CLR_RXFIFO() (R8_UART2_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear the current receive FIFO */
#define UART2_CLR_TXFIFO() (R8_UART2_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current transmit FIFO */

#define UART2_GetITFlag() (R8_UART2_IIR & (RB_IIR_NO_INT | RB_IIR_INT_MASK)) /* Get the current interrupt flag */

#define UART2_SET_FCR(cfglist, en) BITS_CFG(R8_UART2_FCR, (cfglist), (en))
#define UART2_SET_LCR(cfglist, en) BITS_CFG(R8_UART2_LCR, (cfglist), (en))
#define UART2_SET_MCR(cfglist, en) BITS_CFG(R8_UART2_MCR, (cfglist), (en))

// please refer to LINE error and status define
#define UART2_GetLinSTA() (R8_UART2_LSR) /* Get the current communication status */
#define UART2_GetMSRSTA() (R8_UART2_MSR) /* Get the current flow control status, only applicable to UART2 */

#define UART2_DMACFG(cfglist, en) BITS_CFG(R8_UART2_DMA_CTRL, (cfglist), (en))
#define UART2_DMA_SET_RD_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART2_DMA_RD_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                          \
        R32_UART2_DMA_RD_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                              \
    })
#define UART2_DMA_GET_RD_CURRENT_ADDR() (R32_UART2_DMA_RD_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_GET_RD_BEG_ADDR() (R32_UART2_DMA_RD_START_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_GET_RD_END_ADDR() (R32_UART2_DMA_RD_END_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_SET_WR_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART2_DMA_WR_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                          \
        R32_UART2_DMA_WR_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                              \
    })
#define UART2_DMA_GET_WR_CURRENT_ADDR() (R32_UART2_DMA_WR_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_GET_WR_BEG_ADDR() (R32_UART2_DMA_WR_START_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_GET_WR_END_ADDR() (R32_UART2_DMA_WR_END_ADDR & MASK_UART_DMA_ADDR)
#define UART2_DMA_GET_IT_FLAG(dmaif) (R8_UART2_DMA_IF & (dmaif))

#define UART2_SendByte(b) (R8_UART2_THR = (b))       /* Serial port single byte transmission */
    void UART2_SendString(uint8_t *buf, uint16_t l); /* Serial multi-byte transmission */
    void UART2_SendString_DMA(uint8_t *buf, uint32_t lenth);
#define UART2_RecvByte() (R8_UART2_RBR)      /* Serial port read single byte */
    uint16_t UART2_RecvString(uint8_t *buf); /* Serial port read multibyte */

    /****************** UART3 */
    void UART3_DefInit(void);                      /* Serial port default initialization configuration */
    void UART3_BaudRateCfg(uint32_t baudrate);     /* Serial port baud rate configuration */
    void UART3_ByteTrigCfg(UARTByteTRIGTypeDef b); /* Serial byte trigger interrupt configuration */
    void UART3_INTCfg(uint8_t s, uint8_t i);       /* Serial port interrupt configuration */
    void UART3_Reset(void);                        /* Serial port software reset */

#define UART3_SET_DLV(dlv) ({ R8_UART3_DIV = dlv; })

#define UART3_CLR_RXFIFO() (R8_UART3_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear the current receive FIFO */
#define UART3_CLR_TXFIFO() (R8_UART3_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current transmit FIFO */

#define UART3_GetITFlag() (R8_UART3_IIR & (RB_IIR_NO_INT | RB_IIR_INT_MASK)) /* Get the current interrupt flag */

#define UART3_SET_FCR(cfglist, en) BITS_CFG(R8_UART3_FCR, (cfglist), (en))
#define UART3_SET_LCR(cfglist, en) BITS_CFG(R8_UART3_LCR, (cfglist), (en))
#define UART3_SET_MCR(cfglist, en) BITS_CFG(R8_UART3_MCR, (cfglist), (en))

// please refer to LINE error and status define
#define UART3_GetLinSTA() (R8_UART3_LSR) /* Get the current communication status */
#define UART3_GetMSRSTA() (R8_UART3_MSR) /* Get the current flow control status, only applicable to UART3 */

#define UART3_DMACFG(cfglist, en) BITS_CFG(R8_UART3_DMA_CTRL, (cfglist), (en))
#define UART3_DMA_SET_RD_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART3_DMA_RD_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                                      \
        R32_UART3_DMA_RD_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                                          \
    })
#define UART3_DMA_GET_RD_CURRENT_ADDR() (R32_UART3_DMA_RD_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_GET_RD_BEG_ADDR() (R32_UART3_DMA_RD_START_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_GET_RD_END_ADDR() (R32_UART3_DMA_RD_END_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_SET_WR_RANGE(start, end)                                                                             \
    ({                                                                                                                 \
        R32_UART3_DMA_WR_START_ADDR = (uint32_t)(start) & MASK_UART_DMA_ADDR;                                                      \
        R32_UART3_DMA_WR_END_ADDR = (uint32_t)(end) & MASK_UART_DMA_ADDR;                                                          \
    })
#define UART3_DMA_GET_WR_CURRENT_ADDR() (R32_UART3_DMA_WR_NOW_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_GET_WR_BEG_ADDR() (R32_UART3_DMA_WR_START_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_GET_WR_END_ADDR() (R32_UART3_DMA_WR_END_ADDR & MASK_UART_DMA_ADDR)
#define UART3_DMA_GET_IT_FLAG(dmaif) (R8_UART3_DMA_IF & (dmaif))

#define UART3_SendByte(b) (R8_UART3_THR = (b))       /* Serial port single byte transmission */
    void UART3_SendString(uint8_t *buf, uint16_t l); /* Serial multi-byte transmission */
    void UART3_SendString_DMA(uint8_t *buf, uint32_t lenth);
#define UART3_RecvByte() (R8_UART3_RBR)      /* Serial port read single byte */
    uint16_t UART3_RecvString(uint8_t *buf); /* Serial port read multibyte */

#ifdef __cplusplus
}
#endif

#endif // __CH56x_UART_H__
