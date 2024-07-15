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
 *TIM Capture via DMA routine:
 *Configure the TIM to capture mode,capture the palse edge and print the pulse edges' time witch is Handling by DMA.
 *
 */

#include "debug.h"
#include "string.h"

uint8_t pos = 0;
uint32_t datas[4] = {0};

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
    printf("Capture DMA Routine\r\n");

    printf("Start\r\n");

    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_Floating);
    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock / 1000);
    TMR0_CapInit(Edge_To_Edge,clock8);
    TMR0_DMACfg(ENABLE, (uint32_t)datas, (uint32_t)(datas + sizeof(datas) / 4), Mode_LOOP);
    TMR0_Enable();
    while (1)
    {
        for (uint8_t i = 0; i < sizeof(datas) / 4; i++)
        {
            datas[i] & (0x01ul << 27) ? printf("High: %ld\r\n", 0x07ffffff & datas[i]) : printf("Low: %ld\r\n", datas[i]);
        }
        Delay_Ms(1000);
    }
}
