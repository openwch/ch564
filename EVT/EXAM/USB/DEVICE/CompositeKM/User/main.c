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

#include "ch564_usbhs_device.h"
#include <usbd_composite_km.h>

/*
 * @Note
 * Composite Keyboard and Mouse Example:
 * This example uses PB12-PB15 and PA8-PA11 to simulate keyboard key pressing and mouse
 * movement respectively, active low.
 * At the same time, it also uses USART2 to receive the specified data sent from
 * the host to simulate the pressing and releasing of the following specific keyboard
 * keys. Data is sent in hexadecimal format and 1 byte at a time.
 * 'W' -> 0x1A
 * 'A' -> 0x04
 * 'S' -> 0x16
 * 'D' -> 0x07
 *
 *  If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
 */

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
    printf( "Compatibility HID Running On USBHS Controller\n" );

    /* Initialize USART2 for receiving the specified keyboard data */
    USART2_Init( 115200 );
    printf( "USART2 Init OK!\r\n" );


    RCC_SlpWakeCtrl(RB_SLP_PA_WAKE | RB_SLP_PB_WAKE,ENABLE);
    RCC_SlpWakeCtrl(RB_SLP_USB_WAKE,ENABLE);
    /* Initialize GPIO for keyboard scan */
    KB_Scan_Init( );
    KB_Sleep_Wakeup_Cfg( );
    printf( "KB Scan Init OK!\r\n" );

    /* Initialize GPIO for mouse scan */
    MS_Scan_Init( );
    MS_Sleep_Wakeup_Cfg( );
    printf( "MS Scan Init OK!\r\n" );

    /* Initialize timer for Keyboard and mouse scan timing */
    TMR3_DeInit();
    TMR3_TimerInit(SystemCoreClock / 10000 - 1 );
    TMR3_ITCfg(ENABLE, RB_TMR_IE_CYC_END);
    TMR3_Enable();
    NVIC_EnableIRQ(TIM3_IRQn);
    printf( "TIM3 Init OK!\r\n" );

    /* Initialize USBHS interface to communicate with the host  */
    USBHS_Device_Init(ENABLE);
    NVIC_EnableIRQ( USBHS_DEV_IRQn );

    while (1)
    {
        if( USBHS_DevEnumStatus )
        {
            /* Handle keyboard scan data */
            KB_Scan_Handle( );

            /* Handle keyboard lighting */
            KB_LED_Handle( );

            /* Handle mouse scan data */
            MS_Scan_Handle( );

            /* Handle USART2 receiving data */
            USART2_Receive_Handle( );
        }
    }
}
