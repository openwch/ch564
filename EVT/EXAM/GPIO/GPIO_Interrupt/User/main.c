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
 *GPIO interrupt routine:
 *When the pin(PA1) detect a falling signal,chip will send massage via usart0.
 *
 */

#include "debug.h"

void PA_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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
    __attribute__((unused)) FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("GPIO Interrupt Routine\r\n");

    printf("Start\r\n");

    NVIC_SetPriority(PA_IRQn, 0x1 << 4);
    NVIC_EnableIRQ(PA_IRQn);

    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_8, GPIO_ITMode_FallEdge);

    while (1)
    {
    }
}

void PA_IRQHandler(void)
{
    if (GPIOA_ReadITFLAGBit(GPIO_Pin_8))
    {
        GPIOA_ClearITFlagbit(GPIO_Pin_8);
        NVIC_ClearPendingIRQ(PA_IRQn);
        printf("There's a falling edge detect at PA8\r\n");
    }
}
