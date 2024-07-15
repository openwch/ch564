/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_adc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      ADC firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_ADC_H
#define __CH564_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

typedef enum
{
    ADC_Channel0 = 0x00,
    ADC_Channel1,
    ADC_Channel2,
    ADC_Channel0_1,
    ADC_Channel3,
    ADC_Channel4,
    ADC_Channel5,
    ADC_Channel6,
    ADC_ChannelREF,
    ADC_ChannelCN
} ADCChannelTypedef;
/***********************************************************************************
 * @fn    ADC_CMD
 * 
 * @brief ADC Enable/Disable
 * 
 * @param en
 *          - ENABLE
 *          - DISABLE 
 */
#define ADC_CMD(en)                                                                                                    \
    {                                                                                                                  \
        (en) == ENABLE ? (R8_ADC_CTRL_MOD |= RB_ADC_POWER_ON) : (R8_ADC_CTRL_MOD &= ~RB_ADC_POWER_ON);                 \
    }
/***********************************************************************************
 * @fn    ADC_SET_SAMPLE_WIDTH_2CLK
 * 
 * @brief ADC Sample time 2clk enable
 * 
 * @param en
 *          - ENABLE
 *          - DISABLE 
 * @return  None 
 */
#define ADC_SET_SAMPLE_WIDTH_2CLK(en)                                                                                  \
    {                                                                                                                  \
        (en) == ENABLE ? (R8_ADC_CTRL_MOD |= RB_ADC_SAMPLE_WID) : (R8_ADC_CTRL_MOD &= RB_ADC_SAMPLE_WID);              \
    }
/***********************************************************************************
 * @fn    ADC_SET_SAMPLE_CYCLE
 * 
 * @brief Config ADC sample cycle.
 * 
 * @param val
 *          - val = 0:Manual Control 
 *          - val = 0b000001 - 0b111111:Sampling every val clock 
 * @return  None 
 */
#define ADC_SET_SAMPLE_CYCLE(val)                                                                                      \
    ({                                                                                                                 \
        R8_ADC_CTRL_MOD &= ~RB_ADC_CYCLE_CLK;                                                                          \
        R8_ADC_CTRL_MOD |= (val) & RB_ADC_CYCLE_CLK;                                                                   \
        R32_ADC_CTRL &= ~MASK_ADC_CYCLE_BIT_4_6;                                                                       \
        R32_ADC_CTRL |= (((val) >> 4) << 25) & MASK_ADC_CYCLE_BIT_4_6;                                                 \
    })
/***********************************************************************************
 * @fn    ADC_DMA_CMD
 * 
 * @brief Config the ADC DMA control and etc.
 * 
 * @param RB_ADC_IE
 *          - RB_ADC_IE_ADC_CMP
 *          - RB_ADC_DMA_ENABLE
 *          - RB_ADC_DMA_BURST 
 *          - RB_ADC_DMA_LOOP  
 *          - RB_ADC_CHAN_OE   
 *          - RB_ADC_MAN_SAMPLE
 *          
 *        en
 *          - ENABLE
 *          - DISABLE
 * @return  None 
 */
#define ADC_DMA_CMD(RB_ADC_DMA, en)                                                                                    \
    ({ (en) == ENABLE ? (R8_ADC_CTRL_DMA |= (RB_ADC_DMA)) : (R8_ADC_CTRL_DMA &= ~(RB_ADC_DMA)); })
/***********************************************************************************
 * @fn    ADC_IT_CONFIG
 * 
 * @brief ADC interrupt enable
 * 
 * @param RB_ADC_IE
 *          - RB_ADC_IE_ADC_CMP
 *          - RB_ADC_IE_ADC_END
 *          - RB_ADC_IE_FIFO_HF
 *          - RB_ADC_IE_DMA_END
 *          - RB_ADC_IE_FIFO_OV
 *          - RB_ADC_IE_DMA_ERR
 *          - RB_ADC_CMP_MOD_EQ
 *          - RB_ADC_CMP_MOD_GT
 *        en
 *          - ENABLE
 *          - DISABLE
 * @return  None 
 */
#define ADC_IT_CONFIG(RB_ADC_IE, en)                                                                                   \
    ({ (en) == ENABLE ? (R8_ADC_INTER_EN |= (RB_ADC_IE)) : (R8_ADC_INTER_EN &= ~(RB_ADC_IE)); })
/***********************************************************************************
 * @fn    ADC_SET_12BITRESOLUTION
 * 
 * @brief ADC 12bit resolution enable
 * 
 * @param en
 *          - ENABLE
 *          - DISABLE 
 * @return  None 
 */
#define ADC_SET_12BITRESOLUTION(en)                                                                                    \
    ({ (en) == ENABLE ? (R32_ADC_CTRL |= MASK_ADC_BIT_MODE) : (R32_ADC_CTRL &= ~MASK_ADC_BIT_MODE); })
/***********************************************************************************
 * @fn    ADC_SET_SAMPLE_TIME
 * 
 * @brief Config ADC sample calibration time.
 * 
 * @param val
 *          - ADC sample calibration time
 * @return  None 
 */
#define ADC_SET_SAMPLE_TIME(val)                                                                                       \
    ({                                                                                                                 \
        R32_ADC_CTRL &= ~MASK_ADC_SMAPLE_TIME;                                                                         \
        R32_ADC_CTRL |= MASK_ADC_SMAPLE_TIME & ((val) << 4);                                                           \
    })
/***********************************************************************************
 * @fn    ADC_DMA_SET_RANGE
 * 
 * @brief Config ADC DMA transport range
 * 
 * @param startAddress
 *          -  ADC DMA Handling Start Address
 *        endAddress
 *          -  ADC DMA Handling End Address
 * @return  None 
 */
#define ADC_DMA_SET_RANGE(startAddress, endAddress)                                                                    \
    ({                                                                                                                 \
        R32_ADC_DMA_BEG = (uint32_t)(startAddress) & MASK_ADC_DMA_ADDR;                                                \
        R32_ADC_DMA_END = (uint32_t)(endAddress) & MASK_ADC_DMA_ADDR;                                                  \
    })
/***********************************************************************************
 * @fn    ADC_DMA_GET_CURRENT
 * 
 * @brief Get ADC DMA current transport address
 * 
 * @return R32_ADC_DMA_NOW
 */
#define ADC_DMA_GET_CURRENT() (R32_ADC_DMA_NOW & MASK_ADC_DMA_ADDR)
/***********************************************************************************
 * @fn    ADC_DMA_GET_BEGIN
 * 
 * @brief Get ADC DMA start transport address
 * 
 * @return R32_ADC_DMA_BEG
 */
#define ADC_DMA_GET_BEGIN() (R32_ADC_DMA_BEG & MASK_ADC_DMA_ADDR)
/***********************************************************************************
 * @fn    ADC_DMA_GET_END
 * 
 * @brief Get ADC DMA end transport address
 * 
 * @return R32_ADC_DMA_END
 */
#define ADC_DMA_GET_END() (R32_ADC_DMA_END & MASK_ADC_DMA_ADDR)
/***********************************************************************************
 * @fn    ADC_GET_FIFO
 * 
 * @brief Get ADC's FIFO content
 * 
 * @return R16_ADC_FIFO
 */
#define ADC_GET_FIFO() (R16_ADC_FIFO)
/***********************************************************************************
 * @fn    ADC_SET_COMPARE_VAL
 * 
 * @brief Config ADC comparison reference value
 * 
 * @param val
 *          - ADC comparison reference value
 * @return  None 
 */
#define ADC_SET_COMPARE_VAL(val) ({ R16_ADC_CMP_VALUE = ADC_CMP_VALUE & (val); })
/***********************************************************************************
 * @fn    ADC_GET_FIFO_CNT
 * 
 * @brief Get ADC's FIFO count
 * 
 * @return R8_ADC_FIFO_COUNT
 */
#define ADC_GET_FIFO_CNT() (R8_ADC_FIFO_COUNT)
/***********************************************************************************
 * @fn    ADC_GET_VAL
 * 
 * @brief Get ADC's converted value
 * 
 * @return R16_ADC_DATA
 */
#define ADC_GET_VAL() (R16_ADC_DATA)
/***********************************************************************************
 * @fn    ADC_SET_DIV
 * 
 * @brief Config ADC crossover coefficients
 * 
 * @param val
 *          - ADC crossover coefficients
 * @return  None 
 */
#define ADC_SET_DIV(value) ({ R8_ADC_CLOCK_DIV = (value); })
/***********************************************************************************
 * @fn    ADC_CLEAR_IT
 * 
 * @brief Config ADC crossover coefficients
 * 
 * @param RB_ADC_IF
 *          - RB_ADC_IF_ADC_CMP
 *          - RB_ADC_IF_ADC_END
 *          - RB_ADC_IF_FIFO_HF
 *          - RB_ADC_IF_DMA_END
 *          - RB_ADC_IF_FIFO_OV
 *          - RB_ADC_IF_DMA_ERR
 *          - RB_ADC_EOC_FLAG  
 *          - RB_ADC_CHAN_INDEX
 *        en 
 *          - ENABLE
 *          - DISABLE
 */
#define ADC_CLEAR_IT(RB_ADC_IF) ({ R8_ADC_INT_FLAG |= (RB_ADC_IF); })
/***********************************************************************************
 * @fn    ADC_GET_IT
 * 
 * @brief Config ADC crossover coefficients
 * 
 * @param RB_ADC_IF
 *          - RB_ADC_IF_ADC_CMP
 *          - RB_ADC_IF_ADC_END
 *          - RB_ADC_IF_FIFO_HF
 *          - RB_ADC_IF_DMA_END
 *          - RB_ADC_IF_FIFO_OV
 *          - RB_ADC_IF_DMA_ERR
 *          - RB_ADC_EOC_FLAG  
 *          - RB_ADC_CHAN_INDEX
 * @return  0:No Interrupt or interrupt flag
 *          
 */
#define ADC_GET_IT(RB_ADC_IF) (R8_ADC_INT_FLAG & (RB_ADC_IF))
/***********************************************************************************
 * @fn    ADC_MEASURE
 * 
 * @brief Manually initiated measurements
 * 
 * @return  None 
 */
#define ADC_MEASURE() ({ R8_ADC_CTRL_DMA |= RB_ADC_MAN_SAMPLE; })

void ADC_SelectChannel(ADCChannelTypedef adcChannel);

#ifdef __cplusplus
}
#endif

#endif
