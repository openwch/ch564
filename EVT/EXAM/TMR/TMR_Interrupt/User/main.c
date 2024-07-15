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
 *PWM routine:
 *Configuring the timer to PWM mode and outputting a PWM waveform.
 *
 */

#include "debug.h"
#include "string.h"

void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM0_IRQHandler()
{
    TMR0_ClearITFlag(RB_TMR_IF_CYC_END);
    printf("In the interrupt IRQ function\r\n");
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
    printf("PWM Routine\r\n");

    printf("Start\r\n");
    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock / 10000);
    TMR0_ITCfg(RB_TMR_IF_CYC_END, ENABLE);
    NVIC_EnableIRQ(TIM0_IRQn);
    TMR0_Enable();

    while (1)
    {
    }
}
