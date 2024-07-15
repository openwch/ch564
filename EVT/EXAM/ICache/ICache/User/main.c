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
 *Putting similar programs into different code spaces can reveal differences in program speed, 
 *which can be seen in the role ICache plays at this time.
 *
 */

#include "debug.h"

extern char _cache_beg[];
extern char _cache_end[];
#define RAM_STARTUP_ADDR_cache_beg ((uint32_t)_cache_beg)
#define RAM_STARTUP_ADDR_cache_end ((uint32_t)_cache_end)

void __attribute__((section(".non_0_wait"))) TogglePin_inFlash()
{
    printf("TogglePin_inFlash\r\n");
    for (uint16_t i = 0; i < 100; i++)
    {
        GPIOB_InverseBits(GPIO_Pin_17);
    }
    Delay_Ms(1000);
}

void __attribute__((section(".non_0_wait"))) TogglePin_inFlash1()
{
    printf("TogglePin_inFlash\r\n");
    for (uint16_t i = 0; i < 100; i++)
    {
        GPIOB_InverseBits(GPIO_Pin_17);
    }
    Delay_Ms(1000);
}

void __attribute__((section(".cache"))) TogglePin_inCache()
{
    printf("TogglePin_inCache\r\n");
    for (uint16_t i = 0; i < 100; i++)
    {
        GPIOB_InverseBits(GPIO_Pin_17);
    }
    Delay_Ms(1000);
}

void __attribute__((section(".cache"))) TogglePin_inCache1()
{
    printf("TogglePin_inCache\r\n");
    for (uint16_t i = 0; i < 100; i++)
    {
        GPIOB_InverseBits(GPIO_Pin_17);
    }
    Delay_Ms(1000);
}

void TogglePin_inRam()
{
    printf("TogglePin_inRam\r\n");
    for (uint16_t i = 0; i < 100; i++)
    {
        GPIOB_InverseBits(GPIO_Pin_17);
    }
    Delay_Ms(1000);
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
    Delay_Ms(200);
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("GPIO Interrupt Routine\r\n");

    printf("Start\r\n");

    GPIOB_ModeCfg(GPIO_Pin_17, GPIO_ModeOut_PP);

    printf("RAM_STARTUP_ADDR_cache_beg-%08x\r\n", RAM_STARTUP_ADDR_cache_beg);
    printf("RAM_STARTUP_ADDR_cache_end-%08x\r\n", RAM_STARTUP_ADDR_cache_end);

    while (1)
    {
        TogglePin_inCache1();
        TogglePin_inFlash();
        TogglePin_inFlash1();
        TogglePin_inCache();
        TogglePin_inRam();
    }
}
