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
 *GPIO interrupt routine:
 *When the pin(PA13) detect a rising signal,chip will send massage via usart0.
 *
 */

#include "debug.h"

#define USE_WFI 0
#define USE_WFE 1

#define MODE USE_WFE

void PA_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      Peripherals_Clock_OFF_Init
 *
 * @brief   Before going to sleep, try to turn off useless peripheral clocks
 *          to reduce power consumption.
 *
 * @return  none
 */
void Peripherals_Clock_OFF_Init(void)
{
    RCC_SlpClkOff(&R8_SLP_CLK_OFF0,
                  RB_SLP_CLK_TMR0 | RB_SLP_CLK_TMR1 | RB_SLP_CLK_TMR2 | RB_SLP_CLK_TMR3 | RB_SLP_CLK_SPI0 |
                      RB_SLP_CLK_SPI1 | RB_SLP_CLK_UART1,
                  ENABLE);
    RCC_SlpClkOff(
        &R8_SLP_CLK_OFF1,
        RB_SLP_CLK_UTMI | RB_SLP_CLK_I2C | RB_SLP_CLK_USBPD | RB_SLP_CLK_ADC | RB_SLP_CLK_USB | RB_SLP_CLK_ETH, ENABLE);
    RCC_SlpClkOff(&R8_SLP_CTRL_PLL, RB_SLP_CLK_UART2 | RB_SLP_CLK_UART3, ENABLE);
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
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOD_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PD);
    Peripherals_Clock_OFF_Init();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("GPIO Interrupt Routine\r\n");
    printf("Start\r\n");
#if (MODE == USE_WFI)
    NVIC_SetPriority(PA_IRQn, 0x1 << 4);
    NVIC_EnableIRQ(PA_IRQn);
#endif
    GPIOA_ITModeCfg(GPIO_Pin_13, GPIO_ITMode_RiseEdge);
    RCC_SlpWakeCtrl(RB_SLP_PA_WAKE, ENABLE);
    printf("Fall asleep\r\n");
#if (MODE == USE_WFI)
    PWR_Sleep(PWR_STOPEntry_WFI);
#else
    PWR_Sleep(PWR_STOPEntry_WFE);
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
