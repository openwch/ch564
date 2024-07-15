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
 *USARTinterrupt routine:
 *Master:USART1_Tx(PB1)\USART1_Rx(PB0).
 *Slave:USART2_Tx(PD29)\USART2_Rx(PD28).
 *
 *This example demonstrates that UART1 and UART2 use query to send and interrupt to receive.
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
u8 TxBuffer1[] = "*Buffer1 Send from UART1 using Interrupt!"; /* Send by UART1 */
u8 TxBuffer2[] = "#Buffer2 Send from UART2 using Interrupt!"; /* Send by UART2 */
u8 RxBuffer1[TxSize1] = {0};                                  /* UART1 Using */
u8 RxBuffer2[TxSize2] = {0};                                  /* UART2 Using  */

volatile u8 TxCnt1 = 0, RxCnt1 = 0;
volatile u8 TxCnt2 = 0, RxCnt2 = 0;

volatile u8 Rxfinish1 = 0, Rxfinish2 = 0;

void UART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void UART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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
    printf("Uart Interrupt\r\n");

    GPIO_PinRemapConfig(GPIO_PartialRemap1_UART1, ENABLE);
    UART1_DefInit();
    GPIOB_ModeCfg(GPIO_Pin_1, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_Floating);

    UART2_DefInit();
    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);

    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_THR_EMPTY);
    NVIC_EnableIRQ(UART1_IRQn);

    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_THR_EMPTY);
    NVIC_EnableIRQ(UART2_IRQn);

    while (!(Rxfinish1 && Rxfinish2))
        ;
    printf("1:%s \n", RxBuffer1);
    printf("2:%s \n", RxBuffer2);
    while (1)
        ;
}

void UART1_IRQHandler()
{
    vu8 u1_flag = UART1_GetITFlag();
    if (u1_flag == UART_II_THR_EMPTY)
    {
        UART1_SendByte(TxBuffer1[TxCnt1++]);
        if (TxCnt1 == TxSize1)
        {
            UART1_INTCfg(DISABLE, RB_IER_THR_EMPTY);
        }
    }
    else if (u1_flag == UART_II_RECV_RDY)
    {
        RxBuffer1[RxCnt1++] = UART1_RecvByte();
    }
    else if (u1_flag == UART_II_RECV_TOUT)
    {
        if (UART1_GetLinSTA() & RB_LSR_DATA_RDY)
        {
            RxBuffer1[RxCnt1++] = UART1_RecvByte();
        }
    }
    if (RxCnt1 == TxSize2)
    {
        Rxfinish1 = 1;
    }
}

void UART2_IRQHandler()
{
    vu8 u2_flag = UART2_GetITFlag();
    if (u2_flag == UART_II_THR_EMPTY)
    {
        UART2_SendByte(TxBuffer2[TxCnt2++]);
        if (TxCnt2 == TxSize2)
        {
            UART2_INTCfg(DISABLE, RB_IER_THR_EMPTY);
        }
    }
    else if (u2_flag == UART_II_RECV_RDY)
    {
        RxBuffer2[RxCnt2++] = UART2_RecvByte();
    }
    else if (u2_flag == UART_II_RECV_TOUT)
    {
        if (UART2_GetLinSTA() & RB_LSR_DATA_RDY)
        {
            RxBuffer2[RxCnt2++] = UART2_RecvByte();
        }
    }
    if (RxCnt2 == TxSize1)
    {
        Rxfinish2 = 1;
    }
}
