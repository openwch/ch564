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
 *polling transceiver mode, master/slave transceiver routine:
 *Master:USART1_Tx(PB1)\USART1_Rx(PB0).
 *Slave:USART2_Tx(PD29)\USART2_Rx(PD28).
 *
 *This example demonstrates sending from UART1 and receiving from USART2.
 *
 *    Hardware connection:PB1 -- PD28
 *                        PB0 -- PD29
 *
 */

#include "debug.h"
#include "string.h"

/* Global define */
#define TxSize (size(TxBuffer))
#define size(a) (sizeof(a) / sizeof(*(a)))

/* Global Variable */
u8 TxBuffer[] = "#Buffer Send from USART1 by polling!";
u8 RxBuffer[TxSize] = {0};
u8 TxCnt = 0, RxCnt = 0;

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
    printf("Uart Polling \r\n");

    GPIO_PinRemapConfig(GPIO_PartialRemap1_UART1, ENABLE);
    UART1_DefInit();
    GPIOB_ModeCfg(GPIO_Pin_1, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_Floating);

    UART2_DefInit();
    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);

    uint16_t len = 0;

    while (len < TxSize)
    {
        while (!(R8_UART1_LSR & RB_LSR_TX_FIFO_EMP))
        {
        }
        R8_UART1_THR = TxBuffer[len];

        while (!(R8_UART2_LSR & RB_LSR_DATA_RDY))
        {
        }
        RxBuffer[len] = R8_UART2_RBR;
        len++;
    }

    printf("receive: %s \n", RxBuffer);
    while (1)
        ;
}
