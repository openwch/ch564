/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_rcc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : This file provides all the RCC firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_rcc.h"

/*********************************************************************
 * @fn      RCC_SetGlobleMem
 *
 * @brief   Config the different memory assignment
 *
 * @param   Cfg - Three choice of memory assignment
 *              Code16k_Data128k - assign 16k memory for code 128k memory for data
 *              Code48k_Data96k - assign 48k memory for code 96k memory for data
 *              Code80k_Data64k - assign 80k memory for code 64k memory for data
 *
 * @return  none
 */
void RCC_SetGlobalMemCFG(GlobMem_Cfg Cfg)
{
    RCC_UNLOCK_SAFE_ACCESS();
    R8_GLOB_MEM_CFG = Cfg;
    RCC_LOCK_SAFE_ACCESS();
}

/*********************************************************************
 * @fn      RCC_LockPort
 *
 * @brief   Choose a port and decide whether lock or not
 *
 * @param   globport - choose port
 *              - RB_GLOB_LOCK_PA
 *              - RB_GLOB_LOCK_PB
 *              - RB_GLOB_LOCK_PD
 *          NewSTA - Enable or disable
 *              - ENABLE
 *              - DISABLE
 * @return  none
 */
void RCC_LockPort(uint8_t globport, FunctionalState NewSTA)
{
    uint8_t temp = R8_GLOB_LOCK_PORT;
    NewSTA == ENABLE ? (temp |= globport) : (temp &= ~globport);
    R8_GLOB_LOCK_PORT = 0x3f & temp;
}

/*********************************************************************
 * @fn      RCC_GlobleRstCFG
 *
 * @brief   Choose Reset function
 *
 * @param   globrst - choose port
 *              - RB_GLOB_FORCE_RST
 *              - RB_GLOB_WDOG_EN
 *          NewSTA - Enable or disable
 *              - ENABLE
 *              - DISABLE
 * @return  none
 */
void RCC_GlobleRstCFG(uint8_t globrst, FunctionalState NewSTA)
{
    uint8_t temp = R8_GLOB_RST_CFG;
    NewSTA == ENABLE ? (temp = 0x40 | globrst) : (temp = (0x0F & (~globrst))|0x40);
    RCC_UNLOCK_SAFE_ACCESS();
    R8_GLOB_RST_CFG = temp;
    RCC_LOCK_SAFE_ACCESS();
}

/*********************************************************************
 * @fn      RCC_SlpClkOff
 *
 * @brief   Choose peripherals' clock to be on or off
 *
 * @param   reg - register pointer to write
 *              - R8_SLP_CLK_OFF0
 *              - R8_SLP_CLK_OFF1
 *              - R8_SLP_CTRL_PLL
 *          slpclk - choose periph clock
 *              - RB_SLP_CLK_TMR0
 *              - RB_SLP_CLK_TMR1
 *              - RB_SLP_CLK_TMR2
 *              - RB_SLP_CLK_TMR3
 *              - RB_SLP_CLK_SPI0
 *              - RB_SLP_CLK_SPI1
 *              - RB_SLP_CLK_UART0
 *              - RB_SLP_CLK_UART1
 *
 *              - RB_SLP_CLK_UTMI
 *              - RB_SLP_CLK_I2C
 *              - RB_SLP_CLK_UDP
 *              - RB_SLP_CLK_ADC
 *              - RB_SLP_CLK_GPIO
 *              - RB_SLP_CLK_USB
 *              - RB_SLP_CLK_ETH
 *
 *              - RB_SLP_CTRL_PLL_UART2
 *              - RB_SLP_CTRL_PLL_UART3
 *          NewSTA - Enable or disable
 *              - ENABLE
 *              - DISABLE
 * @return  none
 */
void RCC_SlpClkOff(volatile uint8_t *reg, uint8_t slpclk, FunctionalState NewSTA)
{
    if (reg != &R8_SLP_CLK_OFF0 && reg != &R8_SLP_CLK_OFF1 && reg != &R8_SLP_CTRL_PLL)
        return;
    RCC_UNLOCK_SAFE_ACCESS();
    NewSTA == ENABLE ? (*reg |= slpclk) : (*reg &= ~slpclk);
    RCC_LOCK_SAFE_ACCESS();
}

/*********************************************************************
 * @fn      RCC_SlpWakeCtrl
 *
 * @brief   Choose Reset function
 *
 * @param   slpwake - choose periph to wake the device
 *              - RB_SLP_PA_WAKE
 *              - RB_SLP_PB_WAKE
 *              - RB_SLP_PD_WAKE
 *              - RB_SLP_USB_WAKE
 *              - RB_SLP_AP_WAK_USB
 *              - RB_SLP_WOL_WAKE
 *              - RB_SLP_ETH_PWR_DN
 *          NewSTA - Enable or disable
 *              - ENABLE
 *              - DISABLE
 * @return  none
 */
void RCC_SlpWakeCtrl(uint8_t slpwake, FunctionalState NewSTA)
{
    RCC_UNLOCK_SAFE_ACCESS();
    NewSTA == ENABLE ? (R8_SLP_WAKE_CTRL |= slpwake) : (R8_SLP_WAKE_CTRL &= ~slpwake);
    RCC_LOCK_SAFE_ACCESS();
}
