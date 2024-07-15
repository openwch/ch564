/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_pwr.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      PWR firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_PWR_H
#define __CH564_PWR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

#define WDOG_ENABLE()                                                                                                  \
    {                                                                                                                  \
        R8_SAFE_ACCESS_SIG = 0x57;                                                                                     \
        R8_SAFE_ACCESS_SIG = 0xA8;                                                                                     \
        R8_GLOB_RST_CFG |= (0x40 | RB_GLOB_WDOG_EN);                                                                   \
        R8_SAFE_ACCESS_SIG = 0x00;                                                                                     \
    }
#define WDOG_DISABLE()                                                                                                 \
    {                                                                                                                  \
        R8_SAFE_ACCESS_SIG = 0x57;                                                                                     \
        R8_SAFE_ACCESS_SIG = 0xA8;                                                                                     \
        R8_GLOB_RST_CFG = 0x40;                                                                                        \
        R8_SAFE_ACCESS_SIG = 0x00;                                                                                     \
    }
#define FEED_DOG() (R8_WDOG_CLEAR = 0)
#define VIO_PWN_INT_CMD(cmd)                                                                                           \
    {                                                                                                                  \
        cmd == ENABLE ? (R32_EXTEN_CTLR1 |= RB_VIO_PWN_INT_EN) : (R32_EXTEN_CTLR1 &= ~RB_VIO_PWN_INT_EN);              \
    }
#define VIO_PWN_RST_CMD(cmd)                                                                                           \
    {                                                                                                                  \
        cmd == ENABLE ? (R32_EXTEN_CTLR1 |= RB_VIO_PWN_RST_EN) : (R32_EXTEN_CTLR1 &= ~RB_VIO_PWN_RST_EN);              \
    }
#define VIO_PWN_IO_CMD(cmd)                                                                                            \
    {                                                                                                                  \
        cmd == ENABLE ? (R32_EXTEN_CTLR1 |= RB_VIO_PWN_IO_EN) : (R32_EXTEN_CTLR1 &= ~RB_VIO_PWN_IO_EN);                \
    }
#define LDO_DORMENCY_EN(cmd)                                                                                           \
    {                                                                                                                  \
        cmd == ENABLE ? (R32_EXTEN_CTLR1 |= RB_LDO_SLP_EN) : (R32_EXTEN_CTLR1 &= ~RB_LDO_SLP_EN);                      \
    }

void PWR_Sleep(void);
void PWR_DeepSleep(void);

#ifdef __cplusplus
}
#endif

#endif
