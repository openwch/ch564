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
 *SPI HOST Routine:
 *Sending data by spi and receiving data from it then print the received result.
 *SPI0_SCK(PB13)\SPI0_MISO(PB15)\SPI0_MOSI(PB14)\SPI0_SCS(PB12).
 *
 */

#include "debug.h"

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
    printf("SPI0 FullDuplex\r\n");

    u8 spi_Rx[100] = {0};
    u8 spi_Tx[sizeof(spi_Rx)] = {0};

    for (u16 i = 0; i < sizeof(spi_Rx); i++)
    {
        spi_Tx[i] = i;
    }

    GPIOB_ModeCfg(MOSI | SCK0 | SCS, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(MISO, GPIO_ModeIN_Floating);
    SPI0_MasterInit(1000000);

    SPI0_MasterTransRecv(spi_Tx, spi_Rx, sizeof(spi_Rx));

    printf("The data received is:\n");
    for (uint16_t i = 0; i < sizeof(spi_Rx); i++)
    {
        printf("%x ", spi_Rx[i]);
    }
    printf("\n");

    while (1)
        ;
}
