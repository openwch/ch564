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
 *PWM_DMA routine:
 *Configure the timer to PWM mode, change the duty cycle of the PWM via DMA and output a changing PWM waveform.
 *
 */


#include "debug.h"
#include "math.h"
#include "string.h"
#include "malloc.h"

#define SINE_ESOLUTION 128

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
    printf("PWM DMA Routine\r\n");

    printf("Start\r\n");

    uint32_t *sine = malloc(SINE_ESOLUTION*4);
    for (uint8_t i = 0; i < SINE_ESOLUTION; i++)
    {
        *(sine+i) = (long)((sin(2.0*3.1415926 / (float)SINE_ESOLUTION * (float)i)+1)*SystemCoreClock/1000/2);
        printf("%d ",*(sine + i));
    }
    
    GPIOB_ModeCfg(GPIO_Pin_0,GPIO_ModeOut_PP);

    TMR0_DeInit();
    TMR0_TimerInit(SystemCoreClock/1000);
    TMR0_PWMInit(low_on_high,PWM_Times_1);
    TMR0_DMACfg(ENABLE,(uint32_t)sine,(uint32_t)(sine+SINE_ESOLUTION),Mode_LOOP);
    TMR0_Enable();

    while (1)
    {
        
    }
}
