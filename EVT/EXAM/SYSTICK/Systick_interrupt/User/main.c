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
 *Systick interrupt rountine
 *
 */
#include "debug.h"

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
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("Systick Interrupt Routine\r\n");
    printf("Start\r\n");

    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = SystemCoreClock - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xFu;

    NVIC_EnableIRQ(SysTick_IRQn);

    while (1)
    {
    }
}

void __attribute__((interrupt("WCH-Interrupt-fast"))) SysTick_Handler(void)
{
    printf("Systick\r\n");
    SysTick->SR = 0;
}
