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
 *Read ADC Data via DMA Routine:
 *Repeatedly read the ADC conversion data through DMA and print the data once for each successful conversion.
 *
 */

#include "debug.h"
#include "string.h"

__attribute__((aligned(4))) uint16_t ADC_RAW[60] = {0};
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
    printf("ADC DMA Routine\r\n");

    printf("Start\r\n");

    /* This code snippet is a main program that reads ADC data via DMA (Direct Memory Access) routine on a
    microcontroller. Here is a breakdown of what each function call is doing: */
    ADC_IT_CONFIG(RB_ADC_IE_DMA_END, ENABLE);
    ADC_SET_12BITRESOLUTION(ENABLE);
    ADC_SET_DIV(0x40);
    ADC_SET_SAMPLE_CYCLE(0xff);
    ADC_SelectChannel(ADC_Channel0);
    ADC_SET_SAMPLE_TIME(0x10000);
    ADC_DMA_SET_RANGE((uint32_t)&ADC_RAW[0], (uint32_t)&ADC_RAW[59]);
    ADC_DMA_CMD(RB_ADC_DMA_ENABLE | RB_ADC_DMA_BURST, ENABLE);
    ADC_CMD(ENABLE);
    ADC_DMA_CMD(RB_ADC_MAN_SAMPLE, ENABLE);

    while (1)
    {
        if (ADC_GET_IT(RB_ADC_IF_DMA_END))
        {
            ADC_CLEAR_IT(RB_ADC_IF_DMA_END);
            for (uint16_t ad = 0; ad < 60; ad++)
            {
                printf("addr %x %#x\r\n", &ADC_RAW[ad], (uint8_t)ADC_RAW[ad]);
            }
            ADC_DMA_SET_RANGE((uint32_t)&ADC_RAW[0], (uint32_t)&ADC_RAW[59]);
            ADC_DMA_CMD(RB_ADC_DMA_ENABLE, ENABLE);
        }
    }
}
