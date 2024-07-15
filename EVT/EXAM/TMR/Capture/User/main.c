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
 *TIM Capture routine:
 *Configure the TIM to capture mode,capture the palse edge and print the pulse edges' time.
 *
 */

#include "debug.h"
#include "string.h"

uint8_t pos = 0;
uint32_t datas[3] = {0};

void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM0_IRQHandler()
{
    if (TMR0_GetITFlag(RB_TMR_IF_DATA_ACT))
    {
        TMR0_ClearITFlag(RB_TMR_IF_DATA_ACT);
        datas[pos] = TMR0_CAPGetData();

        datas[pos] & (0x01ul << 27) ? printf("High: %ld\r\n", 0x07ffffff & datas[pos])
                                    : printf("Low: %ld\r\n", datas[pos]);
        pos = (pos + 1) % 3;
    }
    if (TMR0_GetITFlag(RB_TMR_IF_CYC_END))
    {
        TMR0_ClearITFlag(RB_TMR_IF_CYC_END);
        datas[pos] = TMR0_CAPGetData();
        printf("overtime\n");
    }
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
    printf("Capture Routine\r\n");

    printf("Start\r\n");

    GPIOB_ModeCfg(GPIO_Pin_1, GPIO_ModeIN_Floating);
    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock / 1000);
    TMR0_CapInit(Edge_To_Edge, clock8);
    TMR0_ITCfg(ENABLE, RB_TMR_IE_DATA_ACT | RB_TMR_IE_CYC_END);
    TMR0_Enable();

    NVIC_EnableIRQ(TIM0_IRQn);

    while (1)
    {
        Delay_Ms(1000);
    }
}
