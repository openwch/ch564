/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/16
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/* @Note
 * UDisk Example:
 * This program provides examples of UDisk.Supports external SPI Flash and internal
 * Flash, selected by STORAGE_MEDIUM at SW_UDISK.h.
 *
 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
 *  */

#include "ch564_usbhs_device.h"
#include "debug.h"
#include "Internal_Flash.h"
#include "SPI_FLASH.h"
#include "SW_UDISK.h"
#include "ISP564.h"


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
    USART_Printf_Init( 115200 );

    printf( "SystemClk:%d\r\n", SystemCoreClock );
    GetCHIPID(&ChipID);
    printf( "ChipID:%08x\r\n", ChipID );
#if (STORAGE_MEDIUM == MEDIUM_SPI_FLASH)
    printf( "USBHS UDisk Demo\r\nStorage Medium: SPI FLASH \r\n" );
    /* SPI flash init */
    FLASH_Port_Init( );
    /* FLASH ID check */
    FLASH_IC_Check( );

#elif (STORAGE_MEDIUM == MEDIUM_INTERAL_FLASH)
    printf( "USBHS UDisk Demo\r\nStorage Medium: INTERNAL FLASH \r\n" );
    Flash_Sector_Count = IFLASH_UDISK_SIZE  / DEF_UDISK_SECTOR_SIZE;
    Flash_Sector_Size = DEF_UDISK_SECTOR_SIZE;
#endif

    /* Enable Udisk */
    Udisk_Capability = Flash_Sector_Count;
    Udisk_Status |= DEF_UDISK_EN_FLAG;

    USBHS_Device_Init(ENABLE);
    NVIC_EnableIRQ( USBHS_DEV_IRQn );

    while (1)
    {
        ;
    }
}