/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_slv.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the SLV firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_slv.h"

/**********************************************************************
 * @fn      SLV_Read
 * @brief   The function SLV_Read reads data from a slave device with a specified data size and timeout.
 *
 * @param   dataAddress A pointer to the memory location where the received data will be stored.
 *          dataSize    The dataSize parameter represents the number of bytes to be read from the SLV
 *                    (slave) device.
 *          timeout     The "timeout" parameter is the maximum amount of time (in milliseconds) that the
 *                    function will wait for data to be received before returning with a timeout error.
 *
 * @return  a value of type SLV_STA.
 *              --slv_data      data read right now is a data
 *              --slv_cmd       data read right now is a command
 *              --slv_timeout   read data timeout
 */
SLV_STA SLV_Read(uint8_t *dataAddress, uint16_t dataSize, uint16_t timeout)
{
    SLV_STA SLV_TYPE;

    SLV_TYPE = slv_data;

    while (dataSize--)
    {
        uint16_t t = timeout;
        while (t--)
        {
            if (SLV_GET_IF(RB_IF_SLV_WR))
            {
                if (SLV_GET_IF(RB_IF_SLV_CMD))
                    SLV_TYPE = slv_cmd;
                else
                    SLV_TYPE = slv_data;
            }

            *(dataAddress++) = SLV_GET_DATA();
        }
        if (t == 0)
        {
            SLV_TYPE = slv_timeout;
            break;
        }
    }

    return SLV_TYPE;
}

/**********************************************************************
 * @fn      SLV_SendDATA
 *
 * @brief   The function SLV_SendDATA sends data over a communication interface and returns the status of the
 *        operation.
 *
 * @param data      The "data" parameter is a pointer to an array of uint8_t (unsigned 8-bit integer)
 *                values. It represents the data that needs to be sent.
 *        datasize  The parameter "datasize" is the size of the data array that is being sent. It
 *                represents the number of elements in the array.
 *
 * @return  ErrorStatus value. If the timeout value reaches 0 before the condition "R8_INT_FLAG_SLV &
 *        RB_IF_SLV_RD" is true, the function will return NoREADY. Otherwise, it will return READY.
 */
ErrorStatus SLV_SendDATA(uint8_t *data, uint16_t datasize)
{
    uint16_t timeout;

    while (datasize--)
    {
        timeout = 100;
        SLV_SEND_DATA(*(data++));
        while (timeout--)
        {
            if (SLV_GET_IF(RB_IF_SLV_RD))
            {
                break;
            }
        }
        if (timeout == 0)
        {
            return NoREADY;
        }
    }
    return READY;
}

/**********************************************************************
 * @fn      SLV_SendSTA
 *
 * @brief   The function SLV_SendSTA sends a series of data bytes to a slave device and returns a status
 *        indicating if the operation was successful or not.
 *
 * @param   sta           A pointer to an array of uint8_t values that represent the data to be sent.
 *          datasize      datasize is the number of bytes in the sta array that need to be sent.
 *
 * @return  ErrorStatus.
 */
ErrorStatus SLV_SendSTA(uint8_t *sta, uint16_t datasize)
{
    uint16_t timeout;

    while (datasize--)
    {
        timeout = 100;
        SLV_SEND_STA(*(sta++));
        while (timeout--)
        {
            if (SLV_GET_IF(RB_IF_SLV_RD))
            {
                break;
            }
        }
        if (timeout == 0)
        {
            return NoREADY;
        }
    }
    return READY;
}
