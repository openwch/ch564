/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_tim.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the TIM firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_tim.h"

/*******************************************************************************
 * @fn      TMR0_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   t - the Most End Value counting to
 *
 * @return  none
 */
void TMR0_TimerInit(uint32_t t)
{
    R32_TMR0_CNT_END = t;
}

/*******************************************************************************
 * @fn      TMR1_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   t - the Most End Value counting to
 *
 * @return  none
 */
void TMR1_TimerInit(uint32_t t)
{
    R32_TMR1_CNT_END = t;
}

/*******************************************************************************
 * @fn      TMR2_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   t - the Most End Value counting to
 *
 * @return  none
 */
void TMR2_TimerInit(uint32_t t)
{
    R32_TMR2_CNT_END = t;
}

/*******************************************************************************
 * @fn      TMR3_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   t - the Most End Value counting to
 *
 * @return  none
 */
void TMR3_TimerInit(uint32_t t)
{
    R32_TMR3_CNT_END = t;
}

/*******************************************************************************
 * @fn      TMR3_EXTSignalCounterInit
 *
 * @brief   external signal count
 *
 * @param   c
 *
 * @return  none
 */
void TMR3_EXTSignalCounterInit(uint32_t c, CapModeTypeDef capedge, CapWidthTypedef capwidth)
{
    R32_TMR3_CNT_END = c;
    R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR3_CTRL_MOD = RB_TMR3_MODE_COUNT;
    R8_TMR3_CTRL_MOD &= ~(0x03 << 6);
    R8_TMR3_CTRL_MOD |= (capedge << 6);
    R8_TMR3_CTRL_MOD &= ~(0x01 << 4);
    R8_TMR3_CTRL_MOD |= (capwidth << 4);
}

/*******************************************************************************
 * @fn      TMR0_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   pr-
 *          ts-
 *
 * @return  none
 */
void TMR0_PWMInit(PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (pr << 4) | (ts << 6);
    R8_TMR0_CTRL_MOD = tmp;
}

/********** *********************************************************************
 * @fn      TMR1_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   pr-
 *          ts-
 *
 * @return  none
 */
void TMR1_PWMInit(PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (pr << 4) | (ts << 6);
    R8_TMR1_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR2_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   pr-
 *          ts-
 *
 * @return  none
 */
void TMR2_PWMInit(PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (pr << 4) | (ts << 6);
    R8_TMR2_CTRL_MOD = tmp;
}

/********** *********************************************************************
 * @fn      TMR3_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   pr-
 *          ts-
 *
 * @return  none
 */
void TMR3_PWMInit(PWM_PolarTypeDef pr, PWM_RepeatTsTypeDef ts)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (pr << 4) | (ts << 6);
    R8_TMR3_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR0_CapInit
 *
 * @brief   cap
 *
 * @param   CapModeTypeDef
 *
 * @return  none
 */
void TMR0_CapInit(CapModeTypeDef cap, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (cap << 6) | (widt << 4);
    R8_TMR0_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR1_CapInit
 *
 * @brief   cap
 *
 * @param   CapModeTypeDef
 *
 * @return  none
 */
void TMR1_CapInit(CapModeTypeDef cap, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (cap << 6) | (widt << 4);
    R8_TMR1_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR2_CapInit
 *
 * @brief   cap
 *
 * @param   CapModeTypeDef
 *
 * @return  none
 */
void TMR2_CapInit(CapModeTypeDef cap, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (cap << 6) | (widt << 4);
    R8_TMR2_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR3_CapInit
 *
 * @brief   cap
 *
 * @param   CapModeTypeDef
 *
 * @return  none
 */
void TMR3_CapInit(CapModeTypeDef cap, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (cap << 6) | (widt << 4);
    R8_TMR3_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn       TMR0_DMACfd
 *
 * @brief    TMR DMA Configuration
 *
 * @param    ENABLE/DISABLE
 *                 startAddr
 *                 endAddr
 *                 DMAModeTypeDef
 * @return  none
 **/
void TMR0_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m)
{
    if (s == DISABLE)
    {
        R8_TMR0_CTRL_DMA = 0;
    }
    else
    {
        TMR0_DMA_SET_RANGE(startAddr, endAddr);
        if (m & Mode_LOOP)
            R8_TMR0_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (m & Mode_Burst)
            R8_TMR0_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR0_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}

/*******************************************************************************
 * @fn       TMR1_DMACfd
 *
 * @brief    TMR DMA Configuration
 *
 * @param    ENABLE/DISABLE
 *                 startAddr
 *                 endAddr
 *                 DMAModeTypeDef
 * @return  none
 **/
void TMR1_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m)
{
    if (s == DISABLE)
    {
        R8_TMR1_CTRL_DMA = 0;
    }
    else
    {
        TMR1_DMA_SET_RANGE(startAddr, endAddr);
        if (m & Mode_LOOP)
            R8_TMR1_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (m & Mode_Burst)
            R8_TMR1_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR1_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}

/*******************************************************************************
 * @fn      TMR2_DMACfd
 *
 * @brief   TMR DMA Configuration
 *
 * @param   ENABLE/DISABLE
 *                 startAddr
 *                 endAddr
 *                 DMAModeTypeDef
 * @return  none
 **/

void TMR2_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m)
{
    if (s == DISABLE)
    {
        R8_TMR2_CTRL_DMA = 0;
    }
    else
    {
        TMR2_DMA_SET_RANGE(startAddr, endAddr);
        if (m & Mode_LOOP)
            R8_TMR2_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (m & Mode_Burst)
            R8_TMR2_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR2_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}
