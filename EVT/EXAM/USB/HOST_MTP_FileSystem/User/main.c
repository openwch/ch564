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

/*
 * @Note
 * This example demonstrates the enumeration process of a USB host to a device that
 * supports MTP and PTP protocols, and reads its files.
 *
 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/

#include "usb_host_config.h"

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
    printf( "SystemClk:%d\r\n",SystemCoreClock) ;
    GetCHIPID(&ChipID);
    printf( "ChipID:%08x\r\n", ChipID );
    printf( "CH372Device Running On USBHS Controller\n" );

    printf( "USBHS Host Init\r\n" );
    USBHS_Host_Init( ENABLE );
    memset( &RootHubDev[ DEF_USBHS_PORT_INDEX ].bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
    memset( &HostCtl[ DEF_USBHS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL ].InterfaceNum, 0, DEF_ONE_USB_SUP_DEV_TOTAL * sizeof( HOST_CTL ) );

    while( 1 )
    {
        USBH_MainDeal( );
    }
}
