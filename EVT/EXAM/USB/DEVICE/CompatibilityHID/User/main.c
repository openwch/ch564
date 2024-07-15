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
 * Compatibility HID Example:
 * This program provides examples of the pass-through of USB-HID data and serial port
 *  data based on compatibility HID device. And the data returned by Get_Report request is
 *  the data sent by the last Set_Report request.Speed of UART1/2 is 115200bps.
 *
 * Interrupt Transfers:
 *   UART2_RX   ---> Endpoint2
 *   Endpoint1  ---> UART2_TX
 *
 *   Note that the first byte is the valid data length and the remaining bytes are
 *   the transmission data for interrupt Transfers.
 *
 * Control Transfers:
 *   Set_Report ---> UART1_TX
 *   Get_Report <--- last Set_Report packet
 *

 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
 *  */

#include "ch564_usbhs_device.h"
#include "usbd_compatibility_hid.h"

/*********************************************************************
 * @fn      Var_Init
 *
 * @brief   Software parameter initialization
 *
 * @return  none
 */
void Var_Init(void)
{
    uint16_t i;
    RingBuffer_Comm.LoadPtr = 0;
    RingBuffer_Comm.StopFlag = 0;
    RingBuffer_Comm.DealPtr = 0;
    RingBuffer_Comm.RemainPack = 0;
    for(i=0; i<DEF_Ring_Buffer_Max_Blks; i++)
    {
        RingBuffer_Comm.PackLen[i] = 0;
    }
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
    USART_Printf_Init( 115200 );

    printf( "SystemClk:%d\r\n",SystemCoreClock) ;
    GetCHIPID(&ChipID);
    printf( "ChipID:%08x\r\n", ChipID );
    printf( "Compatibility HID Running On USBHS Controller\n" );

    /* Variables init */
    Var_Init( );

    /* uart2 init */
    USART2_Init( 115200 );

    /* timer 2 init */
    TMR2_TimerInit(SystemCoreClock / 10000 - 1 );
    TMR2_ITCfg(ENABLE, RB_TMR_IE_CYC_END);
    NVIC_EnableIRQ(TIM2_IRQn);
    TMR2_Enable( );

    /* usbhs init */
    USBHS_Device_Init(ENABLE);
    NVIC_EnableIRQ( USBHS_DEV_IRQn );

    while (1)
    {
        if (USBHS_DevEnumStatus)
        {
            UART2_Rx_Service();
            UART2_Tx_Service();
            HID_Set_Report_Deal();
        }
    }
}
