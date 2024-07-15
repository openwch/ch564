/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_uart.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : This file provides all the UART firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_uart.h"

static uint8_t Best_DIV;
/******************************************************************************
 * @fn     Less_Loss_DIV_Calcu
 *
 * @brief  Caculate the most fit DIV value
 *
 * @return none
 */
void Less_Loss_DIV_Calcu(uint64_t targetbaud)
{
    uint64_t extranum, result_keeper = 1;
    extranum = targetbaud;
    for (unsigned int i = 1; i < 128; i++)
    {
        if (!((SystemCoreClock * 2 / 16 / i) * 2 / targetbaud))
            break;
        long tmpextra = (SystemCoreClock * 2 / 16 / i) % targetbaud;
        tmpextra = tmpextra > targetbaud / 2 ? targetbaud - tmpextra : tmpextra;

        if (tmpextra < extranum)
        {
            result_keeper = i;
            extranum = tmpextra;
        }
    }
    Best_DIV = result_keeper;
}
/******************************************************************************
 * @fn     UART0_DefInit
 *
 * @brief  Serial port default initialization configuration: FIFO enabled, trigger point byte count, serial port data
 * length setting, baud rate and frequency division coefficient
 *
 * @return none
 */
void UART0_DefInit(void)
{

    UART0_BaudRateCfg(115200);
    R8_UART0_FCR = RB_FCR_FIFO_TRIG | RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR |
                   RB_FCR_FIFO_EN; // FIFO open, trigger point 14 bytes
    R8_UART0_LCR = RB_LCR_WORD_SZ;
    R8_UART0_IER = RB_IER_TXD_EN;
    R8_UART0_MCR = RB_MCR_OUT1;
}

/*******************************************************************************
 * @fn     UART1_DefInit
 *
 * @brief  Serial port default initialization configuration: FIFO enabled, trigger point byte count, serial port data
 *length setting, baud rate and frequency division coefficient
 *
 * @return none
 **/
void UART1_DefInit(void)
{
    UART1_BaudRateCfg(115200);
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    // FIFO open, trigger point 14 bytes
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R8_UART1_MCR = RB_MCR_OUT1;
}

/*******************************************************************************
 * @fn     UART2_DefInit
 *
 * @brief  Serial port default initialization configuration: FIFO enabled, trigger point byte count, serial port data
 * length setting, baud rate and frequency division coefficient
 *
 * @return none
 */
void UART2_DefInit(void)
{

    UART2_BaudRateCfg(115200);
    R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    ; // FIFO open, trigger point 14 bytes
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    R8_UART2_IER = RB_IER_TXD_EN;
    R8_UART2_MCR = RB_MCR_OUT1;
}

/*******************************************************************************
 * @fn     UART3_DefInit
 *
 * @brief  Serial port default initialization configuration: FIFO enabled, trigger point byte count, serial port data
 * length setting, baud rate and frequency division coefficient
 *
 * @return none
 */
void UART3_DefInit(void)
{
    UART3_BaudRateCfg(115200);
    R8_UART3_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    ; // FIFO open, trigger point 14 bytes
    R8_UART3_LCR = RB_LCR_WORD_SZ;
    R8_UART3_IER = RB_IER_TXD_EN;
    R8_UART3_MCR = RB_MCR_OUT1;
}

/*******************************************************************************
 * @fn     UART0_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return none
 */
void UART0_BaudRateCfg(uint32_t baudrate)
{
    uint64_t x;
    Less_Loss_DIV_Calcu(baudrate);
    x = 10 * (SystemCoreClock / Best_DIV) / 8 / baudrate;
    x += 5;
    x /= 10;
    x = x == 0 ? 1 : x;
    R8_UART0_LCR |= RB_LCR_DLAB;
    UART0_SET_DLV(Best_DIV);
    R8_UART0_DLM = x >> 8;
    R8_UART0_DLL = x;
    R8_UART0_LCR &= ~RB_LCR_DLAB;
}

/*******************************************************************************
 * @fn     UART1_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return none
 */
void UART1_BaudRateCfg(uint32_t baudrate)
{
    uint64_t x;
    Less_Loss_DIV_Calcu(baudrate);
    x = 10 * (SystemCoreClock / Best_DIV) / 8 / baudrate;
    x += 5;
    x /= 10;
    x = x == 0 ? 1 : x;
    R8_UART1_LCR |= RB_LCR_DLAB;
    UART1_SET_DLV(Best_DIV);
    R8_UART1_DLM = x >> 8;
    R8_UART1_DLL = x;
    R8_UART1_LCR &= ~RB_LCR_DLAB;
}

/*******************************************************************************
 * @fn     UART2_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return none
 */
void UART2_BaudRateCfg(uint32_t baudrate)
{
    uint64_t x;
    Less_Loss_DIV_Calcu(baudrate);
    x = 10 * (SystemCoreClock / Best_DIV) / 8 / baudrate;
    x += 5;
    x /= 10;
    x = x == 0 ? 1 : x;
    R8_UART2_LCR |= RB_LCR_DLAB;
    UART2_SET_DLV(Best_DIV);
    R8_UART2_DLM = x >> 8;
    R8_UART2_DLL = x;
    R8_UART2_LCR &= ~RB_LCR_DLAB;
}

/*******************************************************************************
 * @fn     UART3_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return none
 */
void UART3_BaudRateCfg(uint32_t baudrate)
{
    uint64_t x;
    Less_Loss_DIV_Calcu(baudrate);
    x = 10 * (SystemCoreClock / Best_DIV) / 8 / baudrate;
    x += 5;
    x /= 10;
    x = x == 0 ? 1 : x;
    R8_UART3_LCR |= RB_LCR_DLAB;
    UART3_SET_DLV(Best_DIV);
    R8_UART3_DLM = x >> 8;
    R8_UART3_DLL = x;
    R8_UART3_LCR &= ~RB_LCR_DLAB;
}

/*******************************************************************************
 * @fn     UART0_ByteTrigCfg
 *
 * @brief  Serial byte trigger interrupt configuration
 *
 * @param  UARTByteTRIG - trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return none
 */
void UART0_ByteTrigCfg(UARTByteTRIGTypeDef UARTByteTRIG)
{
    R8_UART0_FCR = (R8_UART0_FCR & ~RB_FCR_FIFO_TRIG) | (UARTByteTRIG << 6);
}

/*******************************************************************************
 * @fn     UART1_ByteTrigCfg
 *
 * @brief  Serial byte trigger interrupt configuration
 *
 * @param  UARTByteTRIG - trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return none
 **/
void UART1_ByteTrigCfg(UARTByteTRIGTypeDef UARTByteTRIG)
{
    R8_UART1_FCR = (R8_UART1_FCR & ~RB_FCR_FIFO_TRIG) | (UARTByteTRIG << 6);
}

/*******************************************************************************
 * @fn     UART2_ByteTrigCfg
 *
 * @brief  Serial byte trigger interrupt configuration
 *
 * @param  UARTByteTRIG - trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return none
 */
void UART2_ByteTrigCfg(UARTByteTRIGTypeDef UARTByteTRIG)
{
    R8_UART2_FCR = (R8_UART2_FCR & ~RB_FCR_FIFO_TRIG) | (UARTByteTRIG << 6);
}

/*******************************************************************************
 * @fn     UART3_ByteTrigCfg
 *
 * @brief  Serial byte trigger interrupt configuration
 *
 * @param  UARTByteTRIG - trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return none
 ***/
void UART3_ByteTrigCfg(UARTByteTRIGTypeDef UARTByteTRIG)
{
    R8_UART3_FCR = (R8_UART3_FCR & ~RB_FCR_FIFO_TRIG) | (UARTByteTRIG << 6);
}

/*******************************************************************************
 * @fn     UART0_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param   NewSTA - interrupt control status
 *					ENABLE  - Enable the corresponding interrupt
 *					DISABLE - Disable the corresponding interrupt
 * @param   RB_IER - interrupt type
 *					RB_IER_MODEM_CHG  - Modem input status change interrupt enable bit (supported on UART0 only)
 *					RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *					RB_IER_THR_EMPTY  - Send Holding Register Empty Interrupt
 *					RB_IER_RECV_RDY   - receive data interrupt
 * @return none
 **/
void UART0_INTCfg(FunctionalState NewSTA, uint8_t RB_IER)
{
    if (NewSTA)
    {
        R8_UART0_IER |= RB_IER;

        R8_UART0_MCR |= RB_MCR_OUT2;
    }
    else
    {
        R8_UART0_IER &= ~RB_IER;
    }
}

/*******************************************************************************
 * @fn     UART1_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param   NewSTA - interrupt control status
 *					ENABLE  - Enable the corresponding interrupt
 *					DISABLE - Disable the corresponding interrupt
 * @param   RB_IER - interrupt type
 *					RB_IER_MODEM_CHG  - Modem input status change interrupt enable bit (supported on UART0 only)
 *					RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *					RB_IER_THR_EMPTY  - Send Holding Register Empty Interrupt
 *					RB_IER_RECV_RDY   - receive data interrupt
 * @return none
 **/
void UART1_INTCfg(FunctionalState NewSTA, uint8_t RB_IER)
{
    if (NewSTA)
    {
        R8_UART1_IER |= RB_IER;
        R8_UART1_MCR |= RB_MCR_OUT2;
    }
    else
    {
        R8_UART1_IER &= ~RB_IER;
    }
}

/*******************************************************************************
 * @fn     UART2_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param   NewSTA - interrupt control status
 *					ENABLE  - Enable the corresponding interrupt
 *					DISABLE - Disable the corresponding interrupt
 * @param   RB_IER - interrupt type
 *					RB_IER_MODEM_CHG  - Modem input status change interrupt enable bit (supported on UART0 only)
 *					RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *					RB_IER_THR_EMPTY  - Send Holding Register Empty Interrupt
 *					RB_IER_RECV_RDY   - receive data interrupt
 * @return none
 **/
void UART2_INTCfg(FunctionalState NewSTA, uint8_t RB_IER)
{
    if (NewSTA)
    {
        R8_UART2_IER |= RB_IER;
        R8_UART2_MCR |= RB_MCR_OUT2;
    }
    else
    {
        R8_UART2_IER &= ~RB_IER;
    }
}

/*******************************************************************************
 * @fn     UART3_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param   NewSTA - interrupt control status
 *					ENABLE  - Enable the corresponding interrupt
 *					DISABLE - Disable the corresponding interrupt
 * @param   RB_IER - interrupt type
 *					RB_IER_MODEM_CHG  - Modem input status change interrupt enable bit (supported on UART0 only)
 *					RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *					RB_IER_THR_EMPTY  - Send Holding Register Empty Interrupt
 *					RB_IER_RECV_RDY   - receive data interrupt
 * @return none
 **/
void UART3_INTCfg(FunctionalState NewSTA, uint8_t RB_IER)
{
    if (NewSTA)
    {
        R8_UART3_IER |= RB_IER;
        R8_UART3_MCR |= RB_MCR_OUT2;
    }
    else
    {
        R8_UART3_IER &= ~RB_IER;
    }
}

/*******************************************************************************
 * @fn     UART0_Reset
 *
 * @brief  Serial port software reset
 *
 * @return none
 **/
void UART0_Reset(void)
{
    R8_UART0_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART1_Reset
 *
 * @brief  Serial port software reset
 *
 * @return none
 **/
void UART1_Reset(void)
{
    R8_UART1_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART2_Reset
 *
 * @brief  Serial port software reset
 *
 * @return none
 **/
void UART2_Reset(void)
{
    R8_UART2_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART3_Reset
 *
 * @brief  Serial port software reset
 *
 * @return none
 **/
void UART3_Reset(void)
{
    R8_UART3_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART0_SendString
 *
 * @brief  Serial multi-byte transmission
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART0_SendString(uint8_t *buf, uint16_t length)
{
    uint16_t len = length;

    while (len)
    {
        if ((R8_UART0_LSR & RB_LSR_TX_FIFO_EMP))
        {
            R8_UART0_THR = *buf++;
            len--;
        }
    }
}

/*******************************************************************************
 * @fn     UART1_SendString
 *
 * @brief  Serial multi-byte transmission
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART1_SendString(uint8_t *buf, uint16_t length)
{
    uint16_t len = length;

    while (len)
    {
        if ((R8_UART1_LSR & RB_LSR_TX_FIFO_EMP))
        {
            R8_UART1_THR = *buf++;
            len--;
        }
    }
}

/*******************************************************************************
 * @fn     UART2_SendString
 *
 * @brief  Serial multi-byte transmission
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART2_SendString(uint8_t *buf, uint16_t length)
{
    uint16_t len = length;

    while (len)
    {
        if ((R8_UART2_LSR & RB_LSR_TX_FIFO_EMP))
        {
            R8_UART2_THR = *buf++;
            len--;
        }
    }
}

/*******************************************************************************
 * @fn     UART3_SendString
 *
 * @brief  Serial multi-byte transmission
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART3_SendString(uint8_t *buf, uint16_t length)
{
    uint16_t len = length;

    while (len)
    {
        if ((R8_UART3_LSR & RB_LSR_TX_FIFO_EMP))
        {
            R8_UART3_THR = *buf++;
            len--;
        }
    }
}

/*******************************************************************************
 * @fn     UART0_RecvString
 *
 * @brief  Serial port read multibyte
 *
 * @param  buf - The first address of the read data storage buffer
 *
 * @return read data length
 */
uint16_t UART0_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    if (!((R8_UART0_LSR) & (RB_LSR_OVER_ERR | RB_LSR_PAR_ERR | RB_LSR_FRAME_ERR | RB_LSR_BREAK_ERR)))
    {
        while ((R8_UART0_LSR & RB_LSR_DATA_RDY) == 0)
            ;
        do
        {
            *buf++ = R8_UART0_RBR;
            len++;
        } while ((R8_UART0_LSR & RB_LSR_DATA_RDY));
    }

    return (len);
}

/*******************************************************************************
 * @fn     UART1_RecvString
 *
 * @brief  Serial port read multibyte
 *
 * @param  buf - The first address of the read data storage buffer
 *
 * @return read data length
 */

uint16_t UART1_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    if (!((R8_UART1_LSR) & (RB_LSR_OVER_ERR | RB_LSR_PAR_ERR | RB_LSR_FRAME_ERR | RB_LSR_BREAK_ERR)))
    {
        while ((R8_UART1_LSR & RB_LSR_DATA_RDY) == 0)
            ;
        do
        {
            *buf++ = R8_UART1_RBR;
            len++;
        } while ((R8_UART1_LSR & RB_LSR_DATA_RDY));
    }

    return (len);
}

/*******************************************************************************
 * @fn     UART2_RecvString
 *
 * @brief  Serial port read multibyte
 *
 * @param  buf - The first address of the read data storage buffer
 *
 * @return read data length
 */

uint16_t UART2_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    if (!((R8_UART2_LSR) & (RB_LSR_OVER_ERR | RB_LSR_PAR_ERR | RB_LSR_FRAME_ERR | RB_LSR_BREAK_ERR)))
    {
        while ((R8_UART2_LSR & RB_LSR_DATA_RDY) == 0)
            ;
        do
        {
            *buf++ = R8_UART2_RBR;
            len++;
        } while ((R8_UART2_LSR & RB_LSR_DATA_RDY));
    }

    return (len);
}

/*******************************************************************************
 * @fn     UART3_RecvString
 *
 * @brief  Serial port read multibyte
 *
 * @param  buf - The first address of the read data storage buffer
 *
 * @return read data length
 */

uint16_t UART3_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    if (!((R8_UART3_LSR) & (RB_LSR_OVER_ERR | RB_LSR_PAR_ERR | RB_LSR_FRAME_ERR | RB_LSR_BREAK_ERR)))
    {
        while ((R8_UART3_LSR & RB_LSR_DATA_RDY) == 0)
            ;
        do
        {
            *buf++ = R8_UART3_RBR;
            len++;
        } while ((R8_UART3_LSR & RB_LSR_DATA_RDY));
    }
    return (len);
}

/*******************************************************************************
 * @fn     UART0_Send_DMA
 *
 * @brief  Serial multi-byte transmission via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART0_Send_DMA(uint8_t *buf, uint32_t lenth)
{
    UART0_DMA_SET_RD_RANGE(buf, buf + lenth);
    UART0_DMACFG(RB_DMA_RD_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART1_Send_DMA
 *
 * @brief  Serial multi-byte transmission via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART1_Send_DMA(uint8_t *buf, uint32_t lenth)
{
    UART1_DMA_SET_RD_RANGE(buf, buf + lenth);
    UART1_DMACFG(RB_DMA_RD_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART2_Send_DMA
 *
 * @brief  Serial multi-byte transmission via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART2_Send_DMA(uint8_t *buf, uint32_t lenth)
{
    UART2_DMA_SET_RD_RANGE(buf, buf + lenth);
    UART2_DMACFG(RB_DMA_RD_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART3_Send_DMA
 *
 * @brief  Serial multi-byte transmission via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART3_Send_DMA(uint8_t *buf, uint32_t lenth)
{
    UART3_DMA_SET_RD_RANGE(buf, buf + lenth);
    UART3_DMACFG(RB_DMA_RD_EN, ENABLE);
}

/*******************************************************************************
 * @fn     UART0_Recv_DMA
 *
 * @brief  Serial multi-byte receive via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART0_Recv_DMA(uint8_t *buf, uint32_t lenth)
{
    UART0_DMA_SET_WR_RANGE(buf, buf + lenth);
    UART0_DMACFG(RB_DMA_WR_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART1_Recv_DMA
 *
 * @brief  Serial multi-byte receive via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART1_Recv_DMA(uint8_t *buf, uint32_t lenth)
{
    UART1_DMA_SET_WR_RANGE(buf, buf + lenth);
    UART1_DMACFG(RB_DMA_WR_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART2_Recv_DMA
 *
 * @brief  Serial multi-byte receive via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART2_Recv_DMA(uint8_t *buf, uint32_t lenth)
{
    UART2_DMA_SET_WR_RANGE(buf, buf + lenth);
    UART2_DMACFG(RB_DMA_WR_EN, ENABLE);
}
/*******************************************************************************
 * @fn     UART3_Recv_DMA
 *
 * @brief  Serial multi-byte receive via DMA
 *
 * @param  buf - The first address of the data content to be sent
 *         length - length of data to be sent
 * @return none
 */
void UART3_Recv_DMA(uint8_t *buf, uint32_t lenth)
{
    UART3_DMA_SET_WR_RANGE(buf, buf + lenth);
    UART3_DMACFG(RB_DMA_WR_EN, ENABLE);
}
