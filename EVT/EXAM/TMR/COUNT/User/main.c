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
 *TIM Extern clock routine:
 *TIM's clock will be change once external clock pin receiving the pulse.
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
    printf("TIM Counter Mode Routine\r\n");

    printf("Start\r\n");

    GPIOB_ModeCfg(GPIO_Pin_6,GPIO_ModeIN_PU);
    TMR0_DeInit();
    TMR3_EXTSignalCounterInit(65535,Edge_To_Edge,clock16);

    TMR0_Enable();

    while (1)
    {
        printf("%d\r\n",TMR3_GetCurrentCount());
        Delay_Ms(1000);
    }
}
