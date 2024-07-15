/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_xbus.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the XBUS firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_xbus.h"

/**********************************************************************
 * @fn      XbusInit
 *
 * @brief             The XbusInit function initializes the Xbus configuration by setting the address bit, enabling or
 *                  disabling 32-bit mode, and enabling or disabling the Xbus.
 *
 * @param   AddrBit   The AddrBit parameter is of type XbusOutputADDrBit, which is likely an enumeration or
 *                  a typedef for a specific type. It represents the address bit configuration for the Xbus module.
 *                  NoOutput      No Xbus address output
 *                    - AddrNum_6bit  PA[5:0] part of address output
 *                    - AddrNum_12bit PA[11:0] part of address output
 *                    - AddrNum_ALL   PA[19:0] part of address output
 *          Bit32En   The Bit32En parameter is used to enable or disable the 32-bit mode of the Xbus. If
 *                  Bit32En is set to ENABLE, the 32-bit mode is enabled. If Bit32En is set to DISABLE, the 32-bit mode
 *                  is disabled.
 *          Stat      The "Stat" parameter is used to enable or disable the Xbus. If "Stat" is set to ENABLE,
 *                  the Xbus will be enabled. If "Stat" is set to DISABLE, the Xbus will be disabled.
 */
void XbusInit(XbusOutputADDrBit AddrBit, FunctionalState Bit32En, FunctionalState Stat)
{

    RCC_UNLOCK_SAFE_ACCESS();
    R8_XBUS_CONFIG = AddrBit << 2;
    RCC_UNLOCK_SAFE_ACCESS();
    R8_XBUS_CONFIG |= (Bit32En == ENABLE ? 0 : RB_XBUS_EN_32BIT);
    RCC_UNLOCK_SAFE_ACCESS();
    R8_XBUS_CONFIG |= (Stat == ENABLE ? 0 : RB_XBUS_ENABLE);
    RCC_LOCK_SAFE_ACCESS(); /* lock, to prevent unexpected writing */
}

/**********************************************************************
 * @fn XbusHoldInit
 *
 * @brief           The function XbusHoldInit initializes the Xbus setup hold time and sets the hold time value based on
 *                the input parameters.
 *
 * @param setuptm   The parameter "setuptm" is of type XbusSetupTime, which is an enumeration type. It
 *                represents the setup time for the XbusHoldInit function. The possible values for setuptm are:
 *                  - Setuptime_1clk        1 clock cycle
 *                  - Setuptime_2clk        2 clock cycle
 *        holdtm    The holdtm parameter is a uint8_t variable that represents the hold time for the Xbus
 *                setup. It is used to set the R8_XBUS_SETUP_HOLD register.
 */
void XbusHoldInit(XbusSetupTime setuptm, uint8_t holdtm)
{
    holdtm = holdtm > 0x1f ? 0x1f : holdtm;
    R8_XBUS_SETUP_HOLD = holdtm;
    R8_XBUS_SETUP_HOLD |= setuptm == Setuptime_1clk ? 0 : 0x80;
}
