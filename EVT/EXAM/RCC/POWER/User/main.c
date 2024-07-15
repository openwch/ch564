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
 *Peripheral Power Control Routines:
 *The corresponding peripheral power supply is manipulated by inputting commands via UART1.
 *
 */

#include "debug.h"
#include "string.h"

uint8_t rData[10] = {0};

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
    printf("Peripherals' Power Control Routine\r\n");

    printf("Start\r\n");

    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_Floating);
    GPIOB_ModeCfg(GPIO_Pin_1, GPIO_ModeOut_PP);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_UART1, ENABLE);
    UART1_DefInit();
    while (1)
    {

        /* This part of the code is responsible for receiving a string of characters via UART1 and then processing the
        received data to control the power supply of various peripherals based on the input command. */
        UART1_RecvString(rData);
        if (rData[0])
        {
            switch (rData[0]-'0')
            {
                case 1:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_TMR0, ENABLE);
                    printf("TIM0 Sleeped\r\n");
                    break;

                case 2:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_TMR1, ENABLE);
                    printf("TIM1 Sleeped\r\n");
                    break;

                case 3:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_TMR2, ENABLE);
                    printf("TIM2 Sleeped\r\n");
                    break;

                case 4:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_TMR3, ENABLE);
                    printf("TIM3 Sleeped\r\n");
                    break;

                case 5:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_SPI0, ENABLE);
                    printf("SPI0 Sleeped\r\n");
                    break;

                case 6:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_SPI1, ENABLE);
                    printf("SPI1 Sleeped\r\n");
                    break;

                case 7:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_UART0, ENABLE);
                    printf("UART0 Sleeped\r\n");
                    break;

                case 8:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF0, RB_SLP_CLK_UART1, ENABLE);
                    printf("UART1 Sleeped\r\n");
                    break;

                case 9:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_UTMI, ENABLE);
                    printf("UTMI Sleeped\r\n");
                    break;

                case 10:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_I2C, ENABLE);
                    printf("I2C Sleeped\r\n");
                    break;

                case 11:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_USBPD, ENABLE);
                    printf("UDP Sleeped\r\n");
                    break;

                case 12:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_ADC, ENABLE);
                    printf("ADC Sleeped\r\n");
                    break;

                case 13:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_GPIO, ENABLE);
                    printf("GPIO Sleeped\r\n");
                    break;

                case 14:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_USB, ENABLE);
                    printf("USB Sleeped\r\n");
                    break;

                case 15:
                    RCC_SlpClkOff(&R8_SLP_CLK_OFF1, RB_SLP_CLK_ETH, ENABLE);
                    printf("ETH Sleeped\r\n");
                    break;

                case 16:
                    RCC_SlpClkOff(&R8_SLP_CTRL_PLL, RB_SLP_CLK_UART2, ENABLE);
                    printf("UART2 Sleeped\r\n");
                    break;

                case 17:
                    RCC_SlpClkOff(&R8_SLP_CTRL_PLL, RB_SLP_CLK_UART3, ENABLE);
                    printf("UART3 Sleeped\r\n");
                    break;

                default:
                    break;
            }
            printf("R8_SLP_CLK_OFF0: %d \r\n",R8_SLP_CLK_OFF0);
            rData[0] = 0;
        }
    }
}
