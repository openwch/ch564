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
 *@NoteMy
 *Polling Read ADC Data Routine:
 *Polling reads ADC data and prints via UART0.
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
    printf("ADC Routine\r\n");
    printf("Start\r\n");

    /* This part of the code is a loop that continuously reads ADC data and prints it via UART0. Here is a breakdown of
     what each step does: */
    volatile uint16_t temp, i = 0;

    ADC_SET_DIV(0x40);
    ADC_SET_SAMPLE_CYCLE(0x00);
    ADC_SelectChannel(ADC_Channel0);

    ADC_SET_SAMPLE_TIME(0x100000);
    ADC_SET_12BITRESOLUTION(ENABLE);
    ADC_CMD(ENABLE);

    while (1)
    {
        ADC_MEASURE();
        temp = ADC_GET_FIFO();
        printf("ADC_VAL: %x\r\n", temp);
        Delay_Ms(1000);
        i++;
    }
}
