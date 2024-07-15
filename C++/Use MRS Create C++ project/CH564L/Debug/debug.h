/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include "ch564.h"
#include "stdio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* UART Printf Definition */
#define DEBUG_UART0 1
#define DEBUG_UART1 2
#define DEBUG_UART2 3

/* DEBUG UATR Definition */
#ifndef DEBUG
#define DEBUG DEBUG_UART0
#endif

void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

#if (DEBUG)
#ifndef PRINT
#define PRINT(format, ...) printf(format, ##__VA_ARGS__)
#endif
#else
#define PRINT(X...)
#endif

#ifdef __cplusplus
}
#endif

#endif
