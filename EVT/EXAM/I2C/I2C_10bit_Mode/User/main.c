/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 10-bit address mode, master/slave mode, transceiver routine:
 I2C_SCL(PB13)\I2C_SDA(PB15).
 This routine demonstrates that Master sends and Slave receives.
 The two boards download the Master and Slave programs respectively, and power on at the same time.
     Hardware connection:
               PB13 -- PB13
               PB15 -- PB15
 *
 */

#include "debug.h"

/* I2C Mode Definition */
#define HOST_MODE 0
#define SLAVE_MODE 1

/* I2C Communication Mode Selection */
// #define I2C_MODE      HOST_MODE
#define I2C_MODE SLAVE_MODE

/* Global define */
#define Size 6
#define RXAdderss 0x02
#define TxAdderss 0x02

vu8 TxData[Size] = {0x03, 0x04, 0x05, 0x04, 0x05, 0x04};
vu8 RxData[5][Size];

/*********************************************************************
 * @fn      IIC_Init
 *
 * @brief   Initializes the IIC peripheral.
 *
 * @return  none
 */
void IIC_Init(u32 bound, u16 address)
{
    I2C_InitTypeDef I2C_InitTSturcture = {0};

    /* You do not need to initialize gpio for i2c, the hardware will automatically initialize gpio for it*/
    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_10bit;
    I2C_Init(I2C, &I2C_InitTSturcture);

    I2C_Cmd(I2C, ENABLE);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint32_t ChipID;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("IIC 10Bit Mode Routine\r\n");

    printf("Start\r\n");

    u8 i = 0;
    u8 j = 0;
    u8 p = 0;

#if (I2C_MODE == HOST_MODE)
    printf("IIC Host mode\r\n");
    IIC_Init(80000, TxAdderss);

    for (j = 0; j < 5; j++)
    {
        while (I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY) != RESET)
            ;

        I2C_GenerateSTART(I2C, ENABLE);

        while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT))
            ;
        I2C_Send7bitAddress(I2C, 0xF0, I2C_Direction_Transmitter);

        while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_ADDRESS10))
            ;
        I2C_Send7bitAddress(I2C, 0x02, I2C_Direction_Transmitter);

        while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
            ;

        for (i = 0; i < 6; i++)
        {
            while (I2C_GetFlagStatus(I2C, I2C_FLAG_TXE) == RESET)
                ;
            Delay_Ms(10);
            I2C_SendData(I2C, TxData[i]);
        }
        while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
            ;
        I2C_GenerateSTOP(I2C, ENABLE);
        Delay_Ms(200);
    }

#elif (I2C_MODE == SLAVE_MODE)
    printf("IIC Slave mode\r\n");
    IIC_Init(80000, RXAdderss);

    for (p = 0; p < 5; p++)
    {
        i = 0;
        while (!I2C_CheckEvent(I2C, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED))
            ;

        while (i < 6)
        {
            while (I2C_GetFlagStatus(I2C, I2C_FLAG_RXNE) == RESET)
                ;
            RxData[p][i] = I2C_ReceiveData(I2C);
            i++;
        }
        while (I2C_GetFlagStatus(I2C, I2C_FLAG_STOPF) == RESET)
            ;
        I2C->CTLR1 &= I2C->CTLR1;
    }
    printf("RxData:\r\n");
    for (p = 0; p < 5; p++)
    {
        for (i = 0; i < 6; i++)
        {
            printf("%02x ", RxData[p][i]);
        }
        printf("\r\n ");
    }

#endif

    while (1)
        ;
}