/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_xbus.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      XBUS firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_XBUS_H
#define __CH564_XBUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

typedef enum {
    NoOutput = 0x0, AddrNum_6bit, AddrNum_12bit, AddrNum_ALL
} XbusOutputADDrBit;

typedef enum {
    Setuptime_1clk, Setuptime_2clk,
} XbusSetupTime;

#define SET_XBUS_CYCLE(val)                                                                                            \
(                                                                                                                  \
    R8_XBUS_CYCLE = XBUS_CYCLE_VALUE_MASK & (val)                                                                  \
)

void XbusInit(XbusOutputADDrBit AddrBit, FunctionalState Bit32En,
        FunctionalState Stat);
void XbusHoldInit(XbusSetupTime setuptm, uint8_t holdtm);

#ifdef __cplusplus
}
#endif

#endif /* INC_CH564_XBUS_H_ */
