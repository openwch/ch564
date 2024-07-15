
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
 *SPI DMA, master/slave mode transceiver routine:
 *Master:SPI0_SCK(PB13)\SPI0_MISO(PB15)\SPI0_MOSI(PB14)\SPI0_SCS(PB12).
 *Slave:SPI0_SCK(PB13)\SPI0_MISO(PB15)\SPI0_MOSI(PB14)\SPI0_SCS(PB12).
 *
 *This example demonstrates that Master and Slave use DAM half-duplex transmission.
 *Note: The two boards download the Master and Slave programs respectively, and power
 *on at the same time.
 *Hardware connection:PB12 -- PB12
 *                     PB13 -- PB13
 *                     PB14 -- PB14
 *                     PB15 -- PB15
 *
 */

#include "debug.h"
#include "string.h"

/* SPI Mode Definition */
#define HOST_MODE 0
#define SLAVE_MODE 1

/* SPI Communication Mode Selection */
#define SPI_MODE HOST_MODE
// #define SPI_MODE      SLAVE_MODE

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
    printf("SPI0 DMA Routine\r\n");

    uint8_t spi_Tx[256];
    uint8_t spi_Rx[256];
    uint16_t usize = sizeof(spi_Tx);
    for (uint16_t i = 0; i < usize; i++)
    {
        spi_Tx[i] = i;
    }

#if (SPI_MODE == HOST_MODE)
    GPIOB_ModeCfg(MOSI | SCK0 | SCS, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(MISO, GPIO_ModeIN_Floating);
    SPI0_MasterInit(2000000);
    printf("SPI Master\r\n");
    Delay_Ms(1000);

    GPIOB_ResetBits(SCS);
    SPI0_DMATrans((uint8_t *)spi_Tx, usize);
    SPI0_DMARecv((uint8_t *)spi_Rx, usize);
    GPIOB_SetBits(SCS);

#elif (SPI_MODE == SLAVE_MODE)
    GPIOB_ModeCfg(MOSI | SCK0 | SCS, GPIO_ModeIN_Floating);
    GPIOB_ModeCfg(MISO, GPIO_ModeOut_PP);
    SPI0_SlaveInit(2000000);
    printf("SPI Slave\r\n");

    SPI0_MasterDMARecv((uint8_t *)spi_Rx, usize);
    SPI0_MasterDMATrans((uint8_t *)spi_Tx, usize);

#endif

    printf("The data received is:\n");
    for (uint16_t i = 0; i < sizeof(spi_Rx); i++)
    {
        printf("%x ", spi_Rx[i]);
    }
    printf("\n");

    while (1)
        ;
}
