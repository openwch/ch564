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
 *Deep sleep and wake up routine:
 *The chip will fall asleep.When the pin(PA13) detect a rising signa,the chip will be wake.
 *
 */

#include "debug.h"

#define USE_WFI 0 
#define USE_WFE 1

#define MODE USE_WFE

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
    Delay_Ms(2000);
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOD_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    RCC_UNLOCK_SAFE_ACCESS();
    EXTEN->CTLR0 &= ~RB_SW_CFG; // disable SWD
    RCC_LOCK_SAFE_ACCESS();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused)) FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("GPIO Interrupt Routine\r\n");

#if (MODE == USE_WFI)
    NVIC_SetPriority(PA_IRQn, 0x1 << 4);
    NVIC_EnableIRQ(PA_IRQn);
#endif
    GPIOA_ITModeCfg(GPIO_Pin_13, GPIO_ITMode_RiseEdge);
    RCC_SlpWakeCtrl(RB_SLP_PA_WAKE, ENABLE);
    printf("Fell deep sleep\r\n");
    LDO_DORMENCY_EN(DISABLE);
#if (MODE == USE_WFI)
    PWR_DeepSleep(PWR_STOPEntry_WFI);
#else
    PWR_DeepSleep(PWR_STOPEntry_WFE);
#endif
    printf("Woke up\r\n");
    while (1)
    {
        printf("Run in main\r\n");
        Delay_Ms(1000);
    }
}

#if (MODE == USE_WFI)
void PA_IRQHandler(void)
{
    if (GPIOA_ReadITFLAGBit(GPIO_Pin_13))
    {
        GPIOA_ClearITFlagbit(GPIO_Pin_13);
        printf("There's a rising edge detect at PA13\r\n");
    }
}
#endif