/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@NoteMy
 *UART Echo routine:
 *UART backhaul.
 *
 */

#include "debug.h"
#include "string.h"

extern uint32_t temp;

char string[40]=  "", *p = string;

void __attribute__((interrupt("WCH-Interrupt-fast"))) UART2_IRQHandler() {
    if (UART2_GetITFlag() == UART_II_RECV_RDY) {
        *p++ = UART2_RecvByte();
    } else if (UART2_GetITFlag() == UART_II_RECV_TOUT) {
        while(UART2_GetLinSTA() & RB_LSR_DATA_RDY) {
            *p++ = UART2_RecvByte();
        }
        UART2_SendString(string, strlen(string));
        p = string;
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("Uart2 Interrupt Routine\r\n");

    printf("Start\r\n");

    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);
    UART2_DefInit();
    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY);
    NVIC_EnableIRQ(UART2_IRQn);
    while (1)
    {
        UART2_SendString(string, strlen(string));
        Delay_Ms(1000);
    }
}
