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
 *USART DMA, master/slave mode transceiver routine:
 *Master:USART1_Tx(PB1)\USART1_Rx(PB0).
 *Slave:USART2_Tx(PD29)\USART2_Rx(PD28).
 *
 *This example demonstrates UART1 and UART2 use DMA data to send and receive.
 *
 *    Hardware connection:PB1 -- PD28
 *                        PB0 -- PD29
 *
 */

#include "debug.h"
#include "string.h"

/* Global define */
#define TxSize1 (size(TxBuffer1))
#define TxSize2 (size(TxBuffer2))
#define size(a) (sizeof(a) / sizeof(*(a)))

/* Global Variable */
u8 TxBuffer1[] = "*Buffer1 Send from USART1 using DMA!"; /* Send by UART1 */
u8 TxBuffer2[] = "#Buffer2 Send from USART2 using DMA!"; /* Send by UART2 */
u8 RxBuffer1[TxSize1] = {0};                             /* USART1 Using  */
u8 RxBuffer2[TxSize2] = {0};                             /* USART2 Using  */

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
    printf("UART Polling\r\n");

    GPIO_PinRemapConfig(GPIO_PartialRemap1_UART1, ENABLE);
    UART1_DefInit();
    GPIOB_ModeCfg(GPIO_Pin_1, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_Floating);

    UART2_DefInit();
    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);

    UART1_Send_DMA(TxBuffer1, TxSize1);
    UART2_Send_DMA(TxBuffer2, TxSize2);
    UART1_Recv_DMA(RxBuffer1, TxSize1);
    UART2_Recv_DMA(RxBuffer2, TxSize2);

    while (R32_UART1_DMA_WR_NOW_ADDR != R32_UART1_DMA_WR_END_ADDR ||
           R32_UART2_DMA_WR_NOW_ADDR != R32_UART2_DMA_WR_END_ADDR)
    {
    }
    printf("End of transfer\n");

    printf("1:%s \n", RxBuffer1);
    printf("2:%s \n", RxBuffer2);

    while (1)
        ;
}
