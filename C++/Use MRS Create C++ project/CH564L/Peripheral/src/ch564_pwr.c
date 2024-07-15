/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_pwr.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the PWR firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_pwr.h"

/*******************************************************************************
 * @fn       LowPower_Idle
 *
 * @brief    Low power consumption - Idle mode
 *
 * @return   None
 */
void PWR_Sleep(void)
{

    PFIC->SCTLR &= ~1 << 2; // Set the SleepDeep field of the core PFIC SCTLR register to 0
    __WFE();                // Execute __WFI() after setting the wake-up condition
}

/*******************************************************************************
 * @fn       LowPower_Halt
 *
 * @brief    Low power consumption - Halt mode
 *
 * @return   None
 */
void PWR_DeepSleep(void)
{

    PFIC->SCTLR |= 1 << 2; // Set the SleepDeep field of the core PFIC SCTLR register to 1
    __WFE();               // Execute __WFI() after setting the wake-up condition
}
