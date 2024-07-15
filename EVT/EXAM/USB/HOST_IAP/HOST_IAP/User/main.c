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
 *@Note
  USB host IAP demo code.
  USBHS HOST USB drive enumeration and operation, read the APP.BIN file in the root directory inside the USB drive, write to location 0x00007000, after a successful check
  Write the flag data, then jump to the user code. The code will run IAP first to check the flag data, if the preparation data is normal, it will jump to the user code,
  if not, it will stay in IAP again.
  Attention: User code needs to be pure in front of flash 80K(0x00014000).after a successful check,The system needs to be reset,then execute the code IAP_Initialization(),jump to APP Code
    Support: FAT12/FAT16/FAT32
  If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/

#include "usb_host_iap.h"
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
    USART_Printf_Init( 115200 );

    printf( "SystemClk:%d.\r\n",SystemCoreClock );
    GetCHIPID(&ChipID);
    printf( "ChipID:%08x\r\n", ChipID );
    printf( "USBHS HOST, UDisk IAP.\r\n" );

    /* IAP initialization */
    IAP_Initialization( );

    while(1)
    {
        IAP_Main_Deal( );
    }
}
