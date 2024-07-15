/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_adc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the ADC firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_adc.h"

/*********************************************************************
 * @fn               ADC_SelectChannel
 *
 * @brief            The function sets the ADC channel for conversion.
 *
 * @param adcChannel The adcChannel parameter is of type ADCChannelTypedef, which is likely an
 *                 enumeration or a typedef for an integer value representing the desired ADC channel.
 *
 * @return           none
 */
void ADC_SelectChannel(ADCChannelTypedef adcChannel)
{
    if (adcChannel <= ADC_Channel0_1)
    {
        R32_ADC_CTRL &= ~MASK_ADC_CTL_MOD1;
        R8_ADC_CTRL_MOD &= ~RB_ADC_CHAN_MOD;
        R8_ADC_CTRL_MOD |= adcChannel << 4;
    }
    else
    {
        R32_ADC_CTRL &= ~MASK_ADC_CTL_MOD1;
        R32_ADC_CTRL |= adcChannel - 1;
    }
}
