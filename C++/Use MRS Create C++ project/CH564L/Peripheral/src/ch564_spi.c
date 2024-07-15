/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_spi.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the SPI firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_spi.h"

static void delay()
{
    for (uint16_t i = 0; i < (SystemCoreClock / 1000000) * 1000; i++)
        ;
}

/*******************************************************************************
 * @fn      SPI0_MasterDefInit
 *
 * @brief   Host mode default initialization
 *
 * @return  none
 */
void SPI0_MasterInit(uint32_t clockRate)
{
    SPI0_MODE_CFG(RB_SPI_MODE_SLAVE, DISABLE);
    SPI0_MODE_CFG(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_ALL_CLEAR | RB_SPI_FIFO_DIR, ENABLE);
    SPI0_MODE_CFG(RB_SPI_MISO_OE | RB_SPI_ALL_CLEAR, DISABLE);
    SPI0_SET_CLOCK_DIV((SystemCoreClock / clockRate) < 2 ? (2) : (SystemCoreClock / clockRate));
    SPI0_SET_DMA_MODE(0xff, DISABLE);
}

/*******************************************************************************
 * @fn      SPI0_DataMode
 *
 * @brief   Set data flow mode
 *
 * @param   m - data flow mode
 *
 * @return  none
 */
void SPI0_DataMode(ModeBitOrderTypeDef m)
{
    switch (m)
    {
    case Mode0_HighBitINFront: // Mode 0, high bit first
        SPI0_MODE_CFG(RB_SPI_MST_SCK_MOD, DISABLE);
        break;
    case Mode3_HighBitINFront: // Mode 3, high bit first
        SPI0_MODE_CFG(RB_SPI_MST_SCK_MOD, ENABLE);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 * @fn      SPI0_MasterSendByte
 *
 * @brief   Send a single byte (buffer)
 *
 * @param   d - send bytes
 *
 * @return  none
 */
void SPI0_MasterSendByte(uint8_t d)
{
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE);
    SPI0_SET_TOTAL_CNT(1);
    SPI0_SET_FIFO(d);

    while (SPI0_GET_TOTAL_CNT() != 0)
        ;
}

/*******************************************************************************
 * @fn       SPI0_MasterRecvByte
 *
 * @brief    Receive a single byte (buffer)
 *
 * @return   bytes received
 */
uint8_t SPI0_MasterRecvByte(void)
{
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE);
    SPI0_SET_TOTAL_CNT(1);
    SPI0_SET_BUFFER(0xff);
    while (!SPI0_GET_FIFO_CNT())
        ;
    return (SPI0_GET_FIFO());
}

/*******************************************************************************
 * @fn      SPI0_MasterTrans
 *
 * @brief   Continuously send multiple bytes using FIFO
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 */
void SPI0_MasterTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    SPI0_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI0_ClearITFlag(RB_SPI_IF_CNT_END);
    while (sendlen)
    {
        if (SPI0_GET_FIFO_CNT() < SPI0_FIFO_SIZE)
        {
            SPI0_SET_FIFO(*pbuf);
            pbuf++;
            sendlen--;
        }
    }
    while (SPI0_GET_TOTAL_CNT() != 0) // Wait for all the data in the FIFO to be sent
        ;
}

/*******************************************************************************
 * @fn      SPI0_MasterRecv
 *
 * @brief   Receive multiple bytes continuously using FIFO
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 **/
void SPI0_MasterRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t readlen;

    readlen = len;
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set data direction to input
    SPI0_SET_TOTAL_CNT(len); // Set the length of the data to be received, the FIFO direction will start the
                             // transmission if the input length is not 0
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END;
    SPI0_SET_BUFFER(0xff);
    while (readlen)
    {
        if (SPI0_GET_FIFO_CNT())
        {
            *pbuf = SPI0_GET_FIFO();
            SPI0_SET_BUFFER(0xff);
            pbuf++;
            readlen--;
        }
    }
}

/*******************************************************************************
 * @fn      SPI0_MasterTransRecv
 *
 * @brief   Continuously send/receive multiple bytes
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 */
void SPI0_MasterTransRecv(uint8_t *ptbuf, uint8_t *prbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    SPI0_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI0_ClearITFlag(RB_SPI_IF_CNT_END);

    while (sendlen)
    {
        if (SPI0_GET_FIFO_CNT() == 0)
        {
            SPI0_SET_FIFO(*ptbuf);
            while (SPI0_GET_FIFO_CNT() != 0);
            ptbuf++;
            *prbuf = SPI0_GET_BUFFER();
            prbuf++;
            sendlen--;
        }
    }
//    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set data direction to input
//    SPI0_SET_FIFO_CNT(16);
}

/*******************************************************************************
 * @fn      SPI0_MasterDMATrans
 *
 * @brief   Continuously send data in DMA mode
 *
 * @param   pbuf: The starting address of the data to be sent
 *
 * @return  none
 */
void SPI0_MasterDMATrans(uint8_t *pbuf, uint32_t len)
{
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE);

    SPI0_SET_DMA_RANGE(pbuf, pbuf + len);
    SPI0_SET_TOTAL_CNT(len);
    SPI0_SET_DMA_MODE(RB_SPI_DMA_ENABLE, ENABLE);
    while (SPI0_GET_TOTAL_CNT())
        ;
    SPI0_SET_DMA_MODE(RB_SPI_DMA_ENABLE, DISABLE);
}

/*******************************************************************************
 * @fn      SPI0_MasterDMARecv
 *
 * @brief   Receive data continuously in DMA mode
 *
 * @param   pbuf: The starting address for storing the data to be received
 *
 * @return  none
 **/
void SPI0_MasterDMARecv(uint8_t *pbuf, uint32_t len)
{
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE);
    SPI0_SET_DMA_RANGE(pbuf, pbuf + len);
    SPI0_SET_TOTAL_CNT(len);
    SPI0_SET_DMA_MODE(RB_SPI_DMA_ENABLE, ENABLE);
    while (SPI0_GET_TOTAL_CNT())
        ;
    SPI0_SET_DMA_MODE(RB_SPI_DMA_ENABLE, DISABLE);
}

/*******************************************************************************
 * @fn      SPI0_SlaveInit
 *
 * @brief   Device mode default initialization
 *
 * @return  none
 */
void SPI0_SlaveInit(uint32_t clockRate)
{
    SPI0_MODE_CFG(RB_SPI_MODE_SLAVE | RB_SPI_ALL_CLEAR, ENABLE);
    SPI0_MODE_CFG(RB_SPI_MOSI_OE | RB_SPI_ALL_CLEAR | RB_SPI_SCK_OE, DISABLE);
    SPI0_MODE_CFG(RB_SPI_MISO_OE, ENABLE);
    
    SPI0_SET_DMA_MODE(0xff, DISABLE);
}

/*******************************************************************************
 * @fn      SPI0_SlaveRecvByte
 *
 * @brief   Slave mode, receive one byte of data
 *
 * @return  received data
 */
uint8_t SPI0_SlaveRecvByte(void)
{
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set to input mode, receive data
    while (!SPI0_GET_FIFO_CNT())
        ;
    return SPI0_GET_FIFO();
}

/*******************************************************************************
 * @fn      SPI0_SlaveRecvByte
 *
 * @brief   Slave mode, send one byte of data
 *
 * @return  received data
 **/
void SPI0_SlaveSendByte(uint8_t d)
{
    SPI0_SET_FIFO(d);
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set to output mode, send data
    SPI0_SET_TOTAL_CNT(1);
    while (SPI0_GET_FIFO_CNT())
        ;
}

/*******************************************************************************
 * @fn      SPI0_SlaveRecv
 *
 * @brief   Slave mode, receive multi-byte data
 *
 * @param   pbuf: Receive data storage starting address
 *
 * @return  0/1 0 means receive failed,1 means receive success.
 **/
uint8_t SPI0_SlaveRecv(uint8_t *pbuf, uint16_t len, uint16_t timeouts)
{
    uint16_t revlen;

    revlen = len;
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set to input mode, receive data
    SPI0_SET_TOTAL_CNT(revlen);             // Assign a value to the SPI send and receive data total length register
    while (revlen && timeouts)
    {
        if (!(timeouts & SPI_MAX_DELAY))
        {
            delay();
            timeouts--;
        }
        if (SPI0_GET_FIFO_CNT()) // Byte count in the current FIFO
        {
            *pbuf = SPI0_GET_FIFO();
            pbuf++;
            revlen--;
        }
    }
    if (!revlen)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*******************************************************************************
 * @fn      SPI0_SlaveTrans
 *
 * @brief   Slave mode, send multi-byte data
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  0/1 0 means receive failed,1 means receive success.
 */
uint8_t SPI0_SlaveTrans(uint8_t *pbuf, uint16_t len, uint16_t timeouts)
{
    uint16_t sendlen;

    sendlen = len;
    SPI0_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI0_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    while (sendlen)
    {
        if (!(timeouts & SPI_MAX_DELAY))
        {
            delay();
            timeouts--;
        }
        if (SPI0_GET_FIFO_CNT() < SPI0_FIFO_SIZE) // Compare the byte count size in the current FIFO
        {
            SPI0_SET_FIFO(*pbuf);
            pbuf++;
            sendlen--;
        }
    }
    if (!sendlen)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*******************************************************************************
 * @fn      SPI1_MasterDefInit
 *
 * @brief   Host mode default initialization
 *
 * @return  none
 */
void SPI1_MasterInit(uint32_t clockRate)
{
    SPI1_MODE_CFG(RB_SPI_MODE_SLAVE, DISABLE);
    SPI1_MODE_CFG(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_ALL_CLEAR, ENABLE);
    SPI1_MODE_CFG(RB_SPI_MISO_OE | RB_SPI_ALL_CLEAR, DISABLE);
    SPI1_SET_CLOCK_DIV((SystemCoreClock / clockRate) < 2 ? (2) : (SystemCoreClock / clockRate));
    SPI1_SET_DMA_MODE(0xff, DISABLE);
}

/*******************************************************************************
 * @fn      SPI1_DataMode
 *
 * @brief   Set data flow mode
 *
 * @param   m - data flow mode
 *
 * @return  none
 */
void SPI1_DataMode(ModeBitOrderTypeDef m)
{
    switch (m)
    {
    case Mode0_HighBitINFront: // Mode 0, high bit first
        SPI1_MODE_CFG(RB_SPI_MST_SCK_MOD, DISABLE);
        break;
    case Mode3_HighBitINFront: // Mode 3, high bit first
        SPI1_MODE_CFG(RB_SPI_MST_SCK_MOD, ENABLE);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 * @fn      SPI1_MasterSendByte
 *
 * @brief   Send a single byte (buffer)
 *
 * @param   d - send bytes
 *
 * @return  none
 */
void SPI1_MasterSendByte(uint8_t d)
{
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE);
    SPI1_SET_TOTAL_CNT(1);
    SPI1_SET_FIFO(d);

    while (SPI1_GET_TOTAL_CNT() != 0)
        ;
}

/*******************************************************************************
 * @fn       SPI1_MasterRecvByte
 *
 * @brief    Receive a single byte (buffer)
 *
 * @return   bytes received
 */
uint8_t SPI1_MasterRecvByte(void)
{
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE);
    SPI1_SET_TOTAL_CNT(1);
    SPI1_SET_FIFO(0xff);
    while (!SPI1_GET_FIFO_CNT())
        ;
    return (SPI1_GET_FIFO());
}

/*******************************************************************************
 * @fn      SPI1_MasterTrans
 *
 * @brief   Continuously send multiple bytes using FIFO
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 */
void SPI1_MasterTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    SPI1_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI1_ClearITFlag(RB_SPI_IF_CNT_END);
    while (sendlen)
    {
        if (SPI1_GET_FIFO_CNT() < SPI1_FIFO_SIZE)
        {
            SPI1_SET_FIFO(*pbuf);
            pbuf++;
            sendlen--;
        }
    }
    while (SPI1_GET_TOTAL_CNT() != 0) // Wait for all the data in the FIFO to be sent
        ;
}

/*******************************************************************************
 * @fn      SPI1_MasterRecv
 *
 * @brief   Receive multiple bytes continuously using FIFO
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 **/
void SPI1_MasterRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t readlen;

    readlen = len;
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set data direction to input
    SPI1_SET_TOTAL_CNT(len); // Set the length of the data to be received, the FIFO direction will start the
                             // transmission if the input length is not 0
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    SPI1_SET_FIFO(0xff);
    while (readlen)
    {
        if (SPI1_GET_FIFO_CNT())
        {
            *pbuf = SPI1_GET_FIFO();
            SPI1_SET_FIFO(0xff);
            pbuf++;
            readlen--;
        }
    }
}

/*******************************************************************************
 * @fn      SPI1_MasterTransRecv
 *
 * @brief   Continuously send/receive multiple bytes
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 */
void SPI1_MasterTransRecv(uint8_t *ptbuf, uint8_t *prbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    SPI1_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI1_ClearITFlag(RB_SPI_IF_CNT_END);
    while (sendlen)
    {
        if (SPI1_GET_FIFO_CNT() == 0)
        {
            SPI1_SET_FIFO(*ptbuf);
            while (SPI1_GET_FIFO_CNT() != 0);
            ptbuf++;
            *prbuf = SPI1_GET_BUFFER();
            prbuf++;
            sendlen--;
        }
    }
}

/*******************************************************************************
 * @fn      SPI1_SlaveInit
 *
 * @brief   Device mode default initialization
 *
 * @return  none
 */
void SPI1_SlaveInit(uint32_t clockRate)
{
    SPI1_MODE_CFG(RB_SPI_MODE_SLAVE | RB_SPI_ALL_CLEAR, ENABLE);
    SPI1_MODE_CFG(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_ALL_CLEAR, DISABLE);
    SPI1_MODE_CFG(RB_SPI_MISO_OE, ENABLE);

    SPI1_SET_DMA_MODE(0xff, DISABLE);
}

/*******************************************************************************
 * @fn      SPI1_SlaveRecvByte
 *
 * @brief   Slave mode, receive one byte of data
 *
 * @return  received data
 */
uint8_t SPI1_SlaveRecvByte(void)
{
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set to input mode, receive data
    while (!SPI1_GET_FIFO_CNT())
        ;
    return SPI1_GET_FIFO();
}

/*******************************************************************************
 * @fn      SPI1_SlaveRecvByte
 *
 * @brief   Slave mode, send one byte of data
 *
 * @return  received data
 **/
void SPI1_SlaveSendByte(uint8_t d)
{
    SPI1_SET_FIFO(d);
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set to output mode, send data
    SPI1_SET_TOTAL_CNT(1);
    while (SPI1_GET_FIFO_CNT())
        ;
}

/*******************************************************************************
 * @fn      SPI1_SlaveRecv
 *
 * @brief   Slave mode, receive multi-byte data
 *
 * @param   pbuf: Receive data storage starting address
 *
 * @return  0/1 0 means receive failed,1 means receive success.
 **/
uint8_t SPI1_SlaveRecv(uint8_t *pbuf, uint16_t len, uint16_t timeouts)
{
    uint16_t revlen;

    revlen = len;
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, ENABLE); // Set to input mode, receive data
    SPI1_SET_TOTAL_CNT(revlen);             // Assign a value to the SPI send and receive data total length register
    while (revlen && timeouts)
    {
        if (!(timeouts & SPI_MAX_DELAY))
        {
            delay();
            timeouts--;
        }
        if (SPI1_GET_FIFO_CNT()) // Byte count in the current FIFO
        {
            *pbuf = SPI1_GET_FIFO();
            pbuf++;
            revlen--;
        }
    }
    if (!revlen)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*******************************************************************************
 * @fn      SPI1_SlaveTrans
 *
 * @brief   Slave mode, send multi-byte data
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  0/1 0 means receive failed,1 means receive success.
 */
uint8_t SPI1_SlaveTrans(uint8_t *pbuf, uint16_t len, uint16_t timeouts)
{
    uint16_t sendlen;

    sendlen = len;
    SPI1_MODE_CFG(RB_SPI_FIFO_DIR, DISABLE); // Set data direction to output
    SPI1_SET_TOTAL_CNT(sendlen);             // Set the length of the data to be sent
    while (sendlen)
    {
        if (!(timeouts & SPI_MAX_DELAY))
        {
            delay();
            timeouts--;
        }
        if (SPI1_GET_FIFO_CNT() < SPI1_FIFO_SIZE) // Compare the byte count size in the current FIFO
        {
            SPI1_SET_FIFO(*pbuf);
            pbuf++;
            sendlen--;
        }
    }
    if (!sendlen)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
