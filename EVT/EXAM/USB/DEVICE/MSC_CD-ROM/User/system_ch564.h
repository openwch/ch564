/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch564.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/16
 * Description        : CH564 Device Peripheral Access Layer System Header File.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __SYSTEM_CH564_H
#define __SYSTEM_CH564_H

#ifdef __cplusplus
extern "C" {
#endif 

extern uint32_t SystemCoreClock; /* System Clock Frequency (Core Clock) */

/* System_Exported_Functions */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif

