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
 *GPIO toggle routine:
 *toggle the PA17 per second
 *
 */

#include "debug.h"
#include "iap.h"
#include "ch564_usbhs_device.h"

extern u8 End_Flag;

void IAP_2_APP()
{
	printf( "Jump APP\r\n" );

	NVIC_EnableIRQ( Software_IRQn );
	Delay_Ms( 20 );
	NVIC_SetPendingIRQ( Software_IRQn );
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
	Delay_Ms( 200 );
	USART_Printf_Init( 115200 );
	printf( "SystemClk:%d\r\n" , SystemCoreClock );
	__attribute__((unused))     FLASH_Status res = GetCHIPID( &ChipID );
	printf( "ChipID:%08x\r\n" , ChipID );
	printf( "IAP\r\n" );

	if (  R8_GLOB_RESET_KEEP == 0xaa ) {
		IAP_2_APP();
		while(1);
	}

	GPIOA_ModeCfg( GPIO_Pin_0 , GPIO_ModeIN_PU );

	if (( GPIOA_ReadPortPin(GPIO_Pin_0) == SET ) )
	{
		IAP_2_APP();
		while(1);
	}

	USBHS_Device_Init( ENABLE );
	NVIC_EnableIRQ( USBHS_DEV_IRQn );
	USART3_CFG( 57600 );

	while (1)
	{
		if( (UART3_GetLinSTA() & RB_LSR_DATA_RDY) != RESET)
		{
			UART_Rx_Deal();
		}
		if (End_Flag)
		{
			R8_GLOB_RESET_KEEP = 0xaa;
			RCC_GlobleRstCFG(RB_GLOB_FORCE_RST, ENABLE);
			while(1);
		}
	}
}
