/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_tim.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
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
 * @param   arr - the Most End Value counting to
 *
 * @return  none
 */
void TMR0_TimerInit(uint32_t arr)
{
    R32_TMR0_CNT_END = arr;
}

/*******************************************************************************
 * @fn      TMR1_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   arr - the Most End Value counting to
 *
 * @return  none
 */
void TMR1_TimerInit(uint32_t arr)
{
    R32_TMR1_CNT_END = arr;
}

/*******************************************************************************
 * @fn      TMR2_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   arr - the Most End Value counting to
 *
 * @return  none
 */
void TMR2_TimerInit(uint32_t arr)
{
    R32_TMR2_CNT_END = arr;
}

/*******************************************************************************
 * @fn      TMR3_TimerInit
 *
 * @brief   Counting Function on TIM PeriPheral
 *
 * @param   arr - the Most End Value counting to
 *
 * @return  none
 */
void TMR3_TimerInit(uint32_t arr)
{
    R32_TMR3_CNT_END = arr;
}

/*******************************************************************************
 * @fn      TMR3_EXTSignalCounterInit
 *
 * @brief   external signal count
 *
 * @param   arr - the most end value contting to
 *          capedge - capture edge
 *              CAP_NULL
 *              Edge_To_Edge
 *              FallEdge_To_FallEdge
 *              RiseEdge_To_RiseEdge
 *          capwidth - the shortest width can be captured
 *              clock16 = 0,
 *              clock8
 *
 * @return  none
 */
void TMR3_EXTSignalCounterInit(uint32_t arr, CapModeTypeDef capedge, CapWidthTypedef capwidth)
{
    R32_TMR3_CNT_END = arr;
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
 * @param   polarities - PWM output polarity
 *              high_on_low
 *              low_on_high
 *          repeattime - Number of repetitions of PWM
 *              PWM_Times_1 
 *              PWM_Times_4 
 *              PWM_Times_8 
 *              PWM_Times_16
 *
 * @return  none
 */
void TMR0_PWMInit(PWM_PolarTypeDef polarities, PWM_RepeatTsTypeDef repeattime)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (polarities << 4) | (repeattime << 6);
    R8_TMR0_CTRL_MOD = tmp;
}

/********** *********************************************************************
 * @fn      TMR1_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   polarities - PWM output polarity
 *              high_on_low
 *              low_on_high
 *          repeattime - Number of repetitions of PWM
 *              PWM_Times_1 
 *              PWM_Times_4 
 *              PWM_Times_8 
 *              PWM_Times_16
 *
 * @return  none
 */
void TMR1_PWMInit(PWM_PolarTypeDef polarities, PWM_RepeatTsTypeDef repeattime)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (polarities << 4) | (repeattime << 6);
    R8_TMR1_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR2_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   polarities - PWM output polarity
 *              high_on_low
 *              low_on_high
 *          repeattime - Number of repetitions of PWM
 *              PWM_Times_1 
 *              PWM_Times_4 
 *              PWM_Times_8 
 *              PWM_Times_16
 *
 * @return  none
 */
void TMR2_PWMInit(PWM_PolarTypeDef polarities, PWM_RepeatTsTypeDef repeattime)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (polarities << 4) | (repeattime << 6);
    R8_TMR2_CTRL_MOD = tmp;
}

/********** *********************************************************************
 * @fn      TMR3_PWMInit
 *
 * @brief   PWM Output Init
 *
 * @param   polarities - PWM output polarity
 *              high_on_low
 *              low_on_high
 *          repeattime - Number of repetitions of PWM
 *              PWM_Times_1 
 *              PWM_Times_4 
 *              PWM_Times_8 
 *              PWM_Times_16
 *
 * @return  none
 */
void TMR3_PWMInit(PWM_PolarTypeDef polarities, PWM_RepeatTsTypeDef repeattime)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_OUT_EN | (polarities << 4) | (repeattime << 6);
    R8_TMR3_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR0_CapInit
 *
 * @brief   Timer capture function initialization
 *
 * @param   capedge - capture edge
 *              CAP_NULL
 *              Edge_To_Edge
 *              FallEdge_To_FallEdge
 *              RiseEdge_To_RiseEdge
 *          capwidth - the shortest width can be captured
 *              clock16 = 0,
 *              clock8
 *
 * @return  none
 */
void TMR0_CapInit(CapModeTypeDef capedge, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (capedge << 6) | (widt << 4);
    R8_TMR0_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR1_CapInit
 *
 * @brief   Timer capture function initialization
 *
 * @param   capedge - capture edge
 *              CAP_NULL
 *              Edge_To_Edge
 *              FallEdge_To_FallEdge
 *              RiseEdge_To_RiseEdge
 *          capwidth - the shortest width can be captured
 *              clock16 = 0,
 *              clock8
 *
 * @return  none
 */
void TMR1_CapInit(CapModeTypeDef capedge, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (capedge << 6) | (widt << 4);
    R8_TMR1_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR2_CapInit
 *
 * @brief   Timer capture function initialization
 *
 * @param   capedge - capture edge
 *              CAP_NULL
 *              Edge_To_Edge
 *              FallEdge_To_FallEdge
 *              RiseEdge_To_RiseEdge
 *          capwidth - the shortest width can be captured
 *              clock16 = 0,
 *              clock8
 *
 * @return  none
 */
void TMR2_CapInit(CapModeTypeDef capedge, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (capedge << 6) | (widt << 4);
    R8_TMR2_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR3_CapInit
 *
 * @brief   Timer capture function initialization
 *
 * @param   capedge - capture edge
 *              CAP_NULL
 *              Edge_To_Edge
 *              FallEdge_To_FallEdge
 *              RiseEdge_To_RiseEdge
 *          capwidth - the shortest width can be captured
 *              clock16 = 0,
 *              clock8
 *
 * @return  none
 */
void TMR3_CapInit(CapModeTypeDef capedge, CapWidthTypedef widt)
{
    uint8_t tmp = 0;
    tmp = RB_TMR_MODE_IN | (capedge << 6) | (widt << 4);
    R8_TMR3_CTRL_MOD = tmp;
}

/*******************************************************************************
 * @fn      TMR0_DMACfg
 *
 * @brief   TMR DMA Configuration
 *
 * @param   NewSTA
 *              - ENABLE/DISABLE
 *          startAddr
 *              - DMA start address
 *          endAddr
 *              - DMA end address
 *          DMAMode
 *              - DMA mode
 * @return  none
 **/
void TMR0_DMACfg(FunctionalState NewSTA, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef DMAMode)
{
    if (NewSTA == DISABLE)
    {
        R8_TMR0_CTRL_DMA = 0;
    }
    else
    {
        TMR0_DMA_SET_RANGE(startAddr, endAddr);
        if (DMAMode & Mode_LOOP)
            R8_TMR0_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (DMAMode & Mode_Burst)
            R8_TMR0_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR0_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}

/*******************************************************************************
 * @fn      TMR1_DMACfg
 *
 * @brief   TMR DMA Configuration
 *
 * @param   NewSTA
 *              - ENABLE/DISABLE
 *          startAddr
 *              - DMA start address
 *          endAddr
 *              - DMA end address
 *          DMAMode
 *              - DMA mode
 * @return  none
 **/
void TMR1_DMACfg(FunctionalState NewSTA, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef DMAMode)
{
    if (NewSTA == DISABLE)
    {
        R8_TMR1_CTRL_DMA = 0;
    }
    else
    {
        TMR1_DMA_SET_RANGE(startAddr, endAddr);
        if (DMAMode & Mode_LOOP)
            R8_TMR1_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (DMAMode & Mode_Burst)
            R8_TMR1_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR1_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}

/*******************************************************************************
 * @fn      TMR2_DMACfg
 *
 * @brief   TMR DMA Configuration
 *
 * @param   NewSTA
 *              - ENABLE/DISABLE
 *          startAddr
 *              - DMA start address
 *          endAddr
 *              - DMA end address
 *          DMAMode
 *              - DMA mode
 * @return  none
 **/

void TMR2_DMACfg(FunctionalState NewSTA, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef DMAMode)
{
    if (NewSTA == DISABLE)
    {
        R8_TMR2_CTRL_DMA = 0;
    }
    else
    {
        TMR2_DMA_SET_RANGE(startAddr, endAddr);
        if (DMAMode & Mode_LOOP)
            R8_TMR2_CTRL_DMA |= RB_TMR_DMA_LOOP;
        if (DMAMode & Mode_Burst)
            R8_TMR2_CTRL_DMA |= RB_TMR_DMA_BURST;
        R8_TMR2_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}
