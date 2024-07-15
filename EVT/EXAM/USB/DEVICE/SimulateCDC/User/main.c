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
 *Example routine to emulate a simulate USB-CDC Device, USE USART2(PD28/PD29);
 *Please note: This code uses the default serial port 1 for debugging,
 *if you need to modify the debugging serial port, please do not use USART2
 *
 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/

#include "UART.h"
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

    printf( "SystemClk:%d\r\n", SystemCoreClock );
    GetCHIPID(&ChipID);
    printf( "ChipID:%08x\r\n", ChipID );
    printf( "Simulate USB-CDC Device running on USBHS Controller\r\n" );

    /* Tim2 init */
    TMR2_DeInit();
    TMR2_TimerInit(SystemCoreClock/10000);
    TMR2_ITCfg(RB_TMR_IF_CYC_END,ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
    TMR2_Enable();

    /* Usart2 init */
    UART2_Init( 1, DEF_UARTx_BAUDRATE, DEF_UARTx_STOPBIT, DEF_UARTx_PARITY );

    USBHS_Device_Init(ENABLE);
    NVIC_EnableIRQ( USBHS_DEV_IRQn );

    while(1)
    {
        UART2_DataRx_Deal( );
        UART2_DataTx_Deal( );
    }
}
