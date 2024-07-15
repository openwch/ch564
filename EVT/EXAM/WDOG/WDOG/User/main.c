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
 *Window watch dog routine:
 *Turns on the watchdog peripheral and resets the system if the dog has not been fed for a period of time.
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
    printf("WDOG Routine\r\n");

    printf("Start\r\n");

    FEED_DOG();
    WDOG_ENABLE();

    while (1)
    {
        if (GPIOB_ReadPortPin(GPIO_Pin_3) == RESET)
        {
            FEED_DOG();
            printf("Dog had been fed\r\n");
            Delay_Ms(20);
        }
        else
        {
            printf("Did not feed the dog\r\n");
            Delay_Ms(200);
        }
    }
}
