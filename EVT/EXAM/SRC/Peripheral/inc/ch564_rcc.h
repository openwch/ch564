/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_rcc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : This file contains all the functions prototypes for the
 *                      RCC firmware library.
 *********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH564_RCC_H
#define __CH564_RCC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

#define RCC_UNLOCK_SAFE_ACCESS()                                                                                       \
({                                                                                                                 \
    R8_SAFE_ACCESS_SIG = 0x57;                                                                                     \
    R8_SAFE_ACCESS_SIG = 0xA8;                                                                                     \
})

#define RCC_LOCK_SAFE_ACCESS() ({ R8_SAFE_ACCESS_SIG = 0x0; })
#define RCC_GET_ID_SAFELY() (R8_SAFE_ACCESS_ID)
#define RCC_CLEAR_WDOG() ({ R8_WDOG_CLEAR = 0; })

#define HSI_ON() (R32_EXTEN_CTLR1 |= RB_HSION)
#define HSE_ON() (R32_EXTEN_CTLR1 |= RB_HSEON)
#define HSE_GET_STTATEUS() ((R32_EXTEN_CTLR1 & 0x8000) != 0 ? 1 : 0)
#define HSI_OFF() (R32_EXTEN_CTLR1 &= ~RB_HSION)
#define HSE_OFF() (R32_EXTEN_CTLR1 &= ~RB_HSEON)
#define USB_PLL_ON()                                                                                                   \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 |= (RB_USBPLLON);                                                                              \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}
#define USB_PLL_OFF()                                                                                                  \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 &= ~(RB_USBPLLON);                                                                             \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}
#define USB_PLL_MUL_15 0x0000c000
#define USB_PLL_MUL_16 0x00008000
#define USB_PLL_MUL_20 0x00004000
#define USB_PLL_MUL_24 0x00000000
#define USB_PLL_MUL_SELECT(USB_PLL_MUL)                                                                                \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 &= ~USB_PLL_MUL_15;                                                                            \
    R32_EXTEN_CTLR0 |= (USB_PLL_MUL);                                                                              \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}
#define USB_PLL_SOURCE_HSI 0x00000060
#define USB_PLL_SOURCE_HSE 0x00000020
#define USB_PLL_SOURCE_ETH_PLL_OUT 0x00000040
#define USB_PLL_SOURCE_SELECT(USB_PLL_SOURCE)                                                                          \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 &= ~USB_PLL_SOURCE_HSI;                                                                        \
    R32_EXTEN_CTLR0 |= (USB_PLL_SOURCE);                                                                           \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}

#define CLKSEL_HSI()                                                                                                   \
{                                                                                                                  \
    R32_EXTEN_CTLR1 &= ~(RB_CLKSEL);                                                                               \
}
#define CLKSEL_HSE()                                                                                                   \
{                                                                                                                  \
    R32_EXTEN_CTLR1 |= (RB_CLKSEL);                                                                                \
}
#define USB_PLL_ON()                                                                                                   \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 |= (RB_USBPLLON);                                                                              \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}
#define USB_PLL_OFF()                                                                                                  \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R32_EXTEN_CTLR0 &= ~(RB_USBPLLON);                                                                             \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}
#define SYSCLK_SOURCE_USBPLL 0
#define SYSCLK_SOURCE_HSI_HSE 1
#define SYSCLK_SOURCE_SELECT(SYSCLK_SOURCE)                                                                            \
{                                                                                                                  \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    ((SYSCLK_SOURCE) == SYSCLK_SOURCE_HSI_HSE) ? (R32_EXTEN_CTLR0 |= (RB_SW)) : (R32_EXTEN_CTLR0 &= ~(RB_SW));     \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
}

#define RCC_GET_GLOB_RST_KEEP() (R8_GLOB_RESET_KEEP;)
#define RCC_SET_GLOB_RST_KEEP(val) (R8_GLOB_RESET_KEEP = (val);)
#define RCC_SET_PLL_SYS_OUT_DIV(val)                                                                                   \
({                                                                                                                 \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    R8_PLL_OUT_DIV = 0x04 | ((val) << 4);                                                                          \
    RCC_LOCK_SAFE_ACCESS();                                                                                        \
})
#define RCC_FLASH_CLK_PRE_DIV(sta)                                                                                     \
({                                                                                                                 \
    RCC_UNLOCK_SAFE_ACCESS();                                                                                      \
    ((sta) == ENABLE) ? (R32_EXTEN_CTLR0 |= 0x00200000) : (R32_EXTEN_CTLR0 &= ~0x00200000) RCC_LOCK_SAFE_ACCESS(); \
})

typedef enum
{
    Code16k_Data128k = 0x0,
    Code48k_Data96k = 0x1,
    Code80k_Data64k = 0x2
} GlobMem_Cfg;

void RCC_SetGlobalMemCFG(GlobMem_Cfg Cfg);
void RCC_LockPort(uint8_t globport, FunctionalState NewSTA);
void RCC_GlobleRstCFG(uint8_t cfg, FunctionalState NewSTA);
void RCC_SlpClkOff(volatile uint8_t *reg, uint8_t slpclk, FunctionalState NewSTA);
void RCC_SlpWakeCtrl(uint8_t slpwake, FunctionalState NewSTA);

#ifdef __cplusplus
}
#endif

#endif
