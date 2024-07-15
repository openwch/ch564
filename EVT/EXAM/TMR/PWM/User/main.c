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

    GPIOB_ModeCfg(GPIO_Pin_0,GPIO_ModeOut_PP);

    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock/1000);
    TMR0_PWMInit(low_on_high,PWM_Times_1);
    TMR0_PWMActDataWidth(SystemCoreClock/1000/2);
    TMR0_Enable();

    while (1)
    {
        
    }
}
