/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch564.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : CH564 Device Peripheral Access Layer System Source File.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch564.h"

/*
 * Uncomment the line corresponding to the desired System clock (SYSCLK)
 * frequency (after reset the HSI is used as SYSCLK source).
 */

//#define SYSCLK_FREQ_120MHz_HSI 120000000
//#define SYSCLK_FREQ_80MHz_HSI 80000000
//#define SYSCLK_FREQ_60MHz_HSI 60000000
//#define SYSCLK_FREQ_40MHz_HSI 40000000
//#define SYSCLK_FREQ_20MHz_HSI HSI_VALUE
#define SYSCLK_FREQ_120MHz_HSE 120000000
//#define SYSCLK_FREQ_80MHz_HSE 80000000
//#define SYSCLK_FREQ_60MHz_HSE 60000000
//#define SYSCLK_FREQ_40MHz_HSE 40000000
//#define SYSCLK_FREQ_25MHz_HSE HSE_VALUE

/* Clock Definitions */
#ifdef SYSCLK_FREQ_120MHz_HSI
uint32_t SystemCoreClock = SYSCLK_FREQ_120MHz_HSI; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_80MHz_HSI
uint32_t SystemCoreClock = SYSCLK_FREQ_80MHz_HSI; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_60MHz_HSI
uint32_t SystemCoreClock = SYSCLK_FREQ_60MHz_HSI; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_40MHz_HSI
uint32_t SystemCoreClock = SYSCLK_FREQ_40MHz_HSI; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_20MHz_HSI
uint32_t SystemCoreClock = SYSCLK_FREQ_20MHz_HSI; /* System Clock Frequency (Core Clock) */

#elif defined SYSCLK_FREQ_120MHz_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_120MHz_HSE; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_80MHz_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_80MHz_HSE; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_60MHz_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_60MHz_HSE; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_40MHz_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_40MHz_HSE; /* System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_25MHz_HSE
uint32_t SystemCoreClock = SYSCLK_FREQ_25MHz_HSE; /* System Clock Frequency (Core Clock) */
#endif

/* system_private_function_proto_types */
static void SetSysClock(void);

#ifdef SYSCLK_FREQ_120MHz_HSI
static void SetSysClockTo120_HSI(void);
#elif defined SYSCLK_FREQ_80MHz_HSI
static void SetSysClockTo80_HSI(void);
#elif defined SYSCLK_FREQ_60MHz_HSI
static void SetSysClockTo60_HSI(void);
#elif defined SYSCLK_FREQ_40MHz_HSI
static void SetSysClockTo40_HSI(void);
#elif defined SYSCLK_FREQ_20MHz_HSI
static void SetSysClockTo20_HSI(void);
#elif defined SYSCLK_FREQ_120MHz_HSE
static void SetSysClockTo120_HSE(void);
#elif defined SYSCLK_FREQ_80MHz_HSE
static void SetSysClockTo80_HSE(void);
#elif defined SYSCLK_FREQ_60MHz_HSE
static void SetSysClockTo60_HSE(void);
#elif defined SYSCLK_FREQ_40MHz_HSE
static void SetSysClockTo40_HSE(void);
#elif defined SYSCLK_FREQ_25MHz_HSE
static void SetSysClockTo25_HSE(void);
#endif

/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash
 * Interface, update the SystemCoreClock variable.
 *
 * @return  none
 */
void SystemInit(void)
{
	if ( SystemCoreClock >= 60000000 )
	{
		RCC_UNLOCK_SAFE_ACCESS();
		BITS_CFG( R32_EXTEN_CTLR0 , RB_FLASH_PRE_EN , ENABLE );
		RCC_LOCK_SAFE_ACCESS();
	}
	else
	{
		RCC_UNLOCK_SAFE_ACCESS();
		BITS_CFG( R32_EXTEN_CTLR0 , RB_FLASH_PRE_EN , DISABLE );
		RCC_LOCK_SAFE_ACCESS();
	}

	SystemCoreClockUpdate();
	Delay_Init();
	HSI_ON();

	/* Close ETH PHY */
	RCC_SlpWakeCtrl( RB_SLP_ETH_PWR_DN , DISABLE );
	Delay_Us( PLL_STARTUP_TIME );
	ETH->PHY_CR |= ( 1 << 31 );
	ETH->PHY_CR &= ~( 1 << 30 );
	ETH->PHY_CR |= ( 1 << 30 );
	Delay_Us( HSI_STARTUP_TIME );
	RCC_SlpWakeCtrl( RB_SLP_ETH_PWR_DN , ENABLE );

	CLKSEL_HSI()
	;
	SYSCLK_SOURCE_SELECT( SYSCLK_SOURCE_HSI_HSE );
	USB_PLL_OFF()
	;
	SetSysClock();
}

/*********************************************************************
 * @fn      SystemCoreClockUpdate
 *
 * @brief   Update SystemCoreClock variable according to Clock Register Values.
 *
 * @return  none
 */
void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0;

	if ( R32_EXTEN_CTLR0 & RB_SW )
	{
		if ( R32_EXTEN_CTLR1 & RB_CLKSEL )
		{
			tmp = HSE_Value;
		}
		else
		{
			tmp = HSI_Value;
		}
	}
	else
	{
		switch ( R32_EXTEN_CTLR0 & RB_USBPLLSRC )
		{
			case 0x60:
				tmp = HSI_Value;
				break;
			case 0x20:
				tmp = HSE_Value;
				break;
			default:
				tmp = HSE_Value * 20 / 25;
				break;
		}

		switch ( R32_EXTEN_CTLR0 & RB_USBPLLCLK )
		{
			case 0x0:
				tmp *= 24;
				break;
			case 0x4000:
				tmp *= 20;
				break;
			case 0x8000:
				tmp *= 16;
				break;
			case 0xC000:
				tmp *= 15;
				break;
			default:
				break;
		}
		tmp /= ( R8_PLL_OUT_DIV >> 4 ) + 1;
	}

	SystemCoreClock = tmp;
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   Configures the System clock frequency, HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClock(void)
{
	SystemCoreClockUpdate();
	Delay_Init();
	GPIO_IPD_Unused();

#ifdef SYSCLK_FREQ_120MHz_HSI
	SetSysClockTo120_HSI();
#elif defined SYSCLK_FREQ_80MHz_HSI
	SetSysClockTo80_HSI();
#elif defined SYSCLK_FREQ_60MHz_HSI
	SetSysClockTo60_HSI();
#elif defined SYSCLK_FREQ_40MHz_HSI
	SetSysClockTo40_HSI();
#elif defined SYSCLK_FREQ_20MHz_HSI
	SetSysClockTo20_HSI();
#elif defined SYSCLK_FREQ_120MHz_HSE
	SetSysClockTo120_HSE();
#elif defined SYSCLK_FREQ_80MHz_HSE
	SetSysClockTo80_HSE();
#elif defined SYSCLK_FREQ_60MHz_HSE
	SetSysClockTo60_HSE();
#elif defined SYSCLK_FREQ_40MHz_HSE
	SetSysClockTo40_HSE();
#elif defined SYSCLK_FREQ_25MHz_HSE
	SetSysClockTo25_HSE();
#endif
}

#ifdef SYSCLK_FREQ_120MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo120_HSI
 *
 * @brief   Sets System clock frequency to 120MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo120_HSI(void)
{
	RCC_SET_PLL_SYS_OUT_DIV( 0x3 );
	USB_PLL_MUL_SELECT( USB_PLL_MUL_24 );
	USB_PLL_SOURCE_SELECT( USB_PLL_SOURCE_HSI );
	USB_PLL_ON()
	;
	Delay_Us( PLL_STARTUP_TIME );
	SYSCLK_SOURCE_SELECT( SYSCLK_SOURCE_USBPLL );
}

#elif defined SYSCLK_FREQ_80MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo80_HSI
 *
 * @brief   Sets System clock frequency to 80MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo80_HSI(void)
{
	RCC_SET_PLL_SYS_OUT_DIV(0x5);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_HSI);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_60MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo60_HSI
 *
 * @brief   Sets System clock frequency to 60MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo60_HSI(void)
{
	RCC_SET_PLL_SYS_OUT_DIV(0x7);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_HSI);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_40MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo8_HSI
 *
 * @brief   Sets System clock frequency to 40MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo40_HSI(void)
{
	RCC_SET_PLL_SYS_OUT_DIV( 0xB );
	USB_PLL_MUL_SELECT( USB_PLL_MUL_24 );
	USB_PLL_SOURCE_SELECT( USB_PLL_SOURCE_HSI );
	USB_PLL_ON()
	;
	Delay_Us( PLL_STARTUP_TIME );
	SYSCLK_SOURCE_SELECT( SYSCLK_SOURCE_USBPLL );
}

#elif defined SYSCLK_FREQ_20MHz_HSI

/*********************************************************************
 * @fn      SetSysClockTo20_HSI
 *
 * @brief   Sets System clock frequency to 20MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo20_HSI(void)
{
	CLKSEL_HSI();
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_HSI_HSE);
}

#elif defined SYSCLK_FREQ_120MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo120_HSE
 *
 * @brief   Sets System clock frequency to 24MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo120_HSE(void)
{
	HSE_ON();
	Delay_Us(HSE_STARTUP_TIME);
	RCC_SlpWakeCtrl(RB_SLP_ETH_PWR_DN, DISABLE);
	RCC_SET_PLL_SYS_OUT_DIV(0x3);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_ETH_PLL_OUT);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_80MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo80_HSE
 *
 * @brief   Sets System clock frequency to 80MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo80_HSE(void)
{
	HSE_ON();
	Delay_Us(HSE_STARTUP_TIME);
	RCC_SlpWakeCtrl(RB_SLP_ETH_PWR_DN, DISABLE);
	RCC_SET_PLL_SYS_OUT_DIV(0x5);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_ETH_PLL_OUT);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_60MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo60_HSE
 *
 * @brief   Sets System clock frequency to 60MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo60_HSE(void)
{
	HSE_ON();
	Delay_Us(HSE_STARTUP_TIME);
	RCC_SlpWakeCtrl(RB_SLP_ETH_PWR_DN, DISABLE);
	RCC_SET_PLL_SYS_OUT_DIV(0x7);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_ETH_PLL_OUT);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_40MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo40_HSE
 *
 * @brief   Sets System clock frequency to 40MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo40_HSE(void)
{
	HSE_ON();
	Delay_Us(HSE_STARTUP_TIME);
	RCC_SlpWakeCtrl(RB_SLP_ETH_PWR_DN, DISABLE);
	RCC_SET_PLL_SYS_OUT_DIV(0xB);
	USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE_ETH_PLL_OUT);
	USB_PLL_MUL_SELECT(USB_PLL_MUL_24);
	USB_PLL_ON();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_USBPLL);
}

#elif defined SYSCLK_FREQ_25MHz_HSE

/*********************************************************************
 * @fn      SetSysClockTo25_HSE
 *
 * @brief   Sets System clock frequency to 25MHz and configure HCLK prescalers.
 *
 * @return  none
 */
static void SetSysClockTo25_HSE(void)
{
	HSE_ON();
	Delay_Us(HSE_STARTUP_TIME);
	CLKSEL_HSE();
	SystemCoreClock = HSE_VALUE;
	Delay_Init();
	Delay_Us(PLL_STARTUP_TIME);
	SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE_HSI_HSE);

}

#endif
