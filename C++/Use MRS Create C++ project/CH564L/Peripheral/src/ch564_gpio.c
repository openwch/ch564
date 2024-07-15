/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_gpio.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the GPIO firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_gpio.h"

/*******************************************************************************
 * @fn       GPIOA_ModeCfg
 *
 * @brief    GPIOA port pin mode configuration
 *
 * @param    pin -  PA0-PA15
 *		  			GPIO_Pin_0 - GPIO_Pin_15
 *		     mode -
 *		  			GPIO_ModeIN_Floating       -  Floating input/high impedance input
 *		  			GPIO_ModeIN_PU             -  input with pull-up resistor
 *		  			GPIO_ModeIN_PD             -  input with pull-down resistor
 *		  			GPIO_ModeOut_OP            -  Drain output
 *		  			GPIO_ModeOut_PP            -  Push-pull output
 *
 * @return   none
 */

void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{

    switch (mode)
    {
    case GPIO_ModeIN_Floating:
        R32_PA_PD &= ~pin;
        R32_PA_PU &= ~pin;
        R32_PA_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PU:
        R32_PA_PD &= ~pin;
        R32_PA_PU |= pin;
        R32_PA_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PD:
        R32_PA_PD |= pin;
        R32_PA_PU &= ~pin;
        R32_PA_DIR &= ~pin;
        break;

    case GPIO_ModeOut_OP:
        R32_PA_PD |= pin;
        R32_PA_DIR |= pin;
        break;

    case GPIO_ModeOut_PP:
        R32_PA_DIR |= pin;
        R32_PA_PU &= ~pin;
        R32_PA_PD &= ~pin;
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn       GPIOB_ModeCfg
 *
 * @brief    GPIOB port pin mode configuration
 *
 * @param    pin -  PB0-PB15
 *		  			GPIO_Pin_0 - GPIO_Pin_15
 *		     mode -
 *		  			GPIO_ModeIN_Floating       -  Floating input/high impedance input
 *		  			GPIO_ModeIN_PU             -  input with pull-up resistor
 *		  			GPIO_ModeIN_PD             -  input with pull-down resistor
 *		  			GPIO_ModeOut_OP            -  Drain output
 *		  			GPIO_ModeOut_PP            -  Push-pull output
 *
 * @return   none
 */

void GPIOB_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
    switch (mode)
    {
    case GPIO_ModeIN_Floating:
        R32_PB_PD &= ~pin;
        R32_PB_PU &= ~pin;
        R32_PB_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PU:
        R32_PB_PD &= ~pin;
        R32_PB_PU |= pin;
        R32_PB_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PD:
        R32_PB_PD |= pin;
        R32_PB_PU &= ~pin;
        R32_PB_DIR &= ~pin;
        break;

    case GPIO_ModeOut_OP:
        R32_PB_PD |= pin;
        R32_PB_DIR |= pin;
        break;

    case GPIO_ModeOut_PP:
        R32_PB_DIR |= pin;
        R32_PB_PU &= ~pin;
        R32_PB_PD &= ~pin;
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn       GPIOD_ModeCfg
 *
 * @brief    GPIOD port pin mode configuration
 *
 * @param    pin -  PD0-PD15
 *					GPIO_Pin_0 - GPIO_Pin_15
 *		     mode -
 *					GPIO_ModeIN_Floating       -  Floating input/high impedance input
 *					GPIO_ModeIN_PU             -  input with pull-up resistor
 *					GPIO_ModeIN_PD             -  input with pull-down resistor
 *					GPIO_ModeOut_OP            -  Drain output
 *					GPIO_ModeOut_PP            -  Push-pull output
 *
 * @return   none
 */

void GPIOD_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
    switch (mode)
    {
    case GPIO_ModeIN_Floating:
        R32_PD_PD &= ~pin;
        R32_PD_PU &= ~pin;
        R32_PD_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PU:
        R32_PD_PD &= ~pin;
        R32_PD_PU |= pin;
        R32_PD_DIR &= ~pin;
        break;

    case GPIO_ModeIN_PD:
        R32_PD_PD |= pin;
        R32_PD_PU &= ~pin;
        R32_PD_DIR &= ~pin;
        break;

    case GPIO_ModeOut_OP:
        R32_PD_PD |= pin;
        R32_PD_DIR |= pin;
        break;

    case GPIO_ModeOut_PP:
        R32_PD_DIR |= pin;
        R32_PD_PU &= ~pin;
        R32_PD_PD &= ~pin;
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn       GPIOA_ITModeCfg
 *
 * @brief    GPIOA pin interrupt mode configuration
 *
 * @param    pin - PAx
 *		     mode -
 *					GPIO_ITMode_LowLevel   -  Low level trigger
 *					GPIO_ITMode_HighLevel  -  High level trigger
 *					GPIO_ITMode_FallEdge   -  Falling edge trigger
 *					GPIO_ITMode_RiseEdge   -  Rising edge trigger
 *
 * @return   none
 */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
    switch (mode)
    {
    case GPIO_ITMode_FallEdge:
        R32_INT_MODE_PA |= pin;
        R32_INT_POLAR_PA &= ~pin;
        R32_INT_ENABLE_PA |= pin;
        break;

    case GPIO_ITMode_RiseEdge:
        R32_INT_MODE_PA |= pin;
        R32_INT_POLAR_PA |= pin;
        R32_INT_ENABLE_PA |= pin;
        break;

    case GPIO_ITMode_HighLevel:
        R32_INT_MODE_PA &= ~pin;
        R32_INT_POLAR_PA |= pin;
        R32_INT_ENABLE_PA |= pin;
        break;

    case GPIO_ITMode_LowLevel:
        R32_INT_MODE_PA &= ~pin;
        R32_INT_POLAR_PA &= ~pin;
        R32_INT_ENABLE_PA |= pin;
        break;

    case GPIO_ITMode_None:
        R32_INT_ENABLE_PA |= pin;
        R32_INT_ENABLE_PA &= ~pin;
        break;

    default:
        break;
    }
    R32_INT_STATUS_PA = pin;
}

/*******************************************************************************
 * @fn       GPIOB_ITModeCfg
 *
 * @brief    GPIOB pin interrupt mode configuration
 *
 * @param    pin - PBx
 *		     mode -
 *		    			GPIO_ITMode_LowLevel   -  Low level trigger
 *		    			GPIO_ITMode_HighLevel  -  High level trigger
 *		    			GPIO_ITMode_FallEdge   -  Falling edge trigger
 *		    			GPIO_ITMode_RiseEdge   -  Rising edge trigger
 *
 * @return   none
 */
void GPIOB_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
    switch (mode)
    {
    case GPIO_ITMode_FallEdge:
        R32_INT_MODE_PB |= pin;
        R32_INT_POLAR_PB &= ~pin;
        R32_INT_ENABLE_PB |= pin;
        break;

    case GPIO_ITMode_RiseEdge:
        R32_INT_MODE_PB |= pin;
        R32_INT_POLAR_PB |= pin;
        R32_INT_ENABLE_PB |= pin;
        break;

    case GPIO_ITMode_HighLevel:
        R32_INT_MODE_PB &= ~pin;
        R32_INT_POLAR_PB |= pin;
        R32_INT_ENABLE_PB |= pin;
        break;

    case GPIO_ITMode_LowLevel:
        R32_INT_MODE_PB &= ~pin;
        R32_INT_POLAR_PB &= ~pin;
        R32_INT_ENABLE_PB |= pin;
        break;

    case GPIO_ITMode_None:
        R32_INT_ENABLE_PB |= pin;
        R32_INT_ENABLE_PB &= ~pin;
        break;

    default:
        break;
    }
    R32_INT_STATUS_PB = pin;
}

/*******************************************************************************
 * @fn       GPIOD_ITModeCfg
 *
 * @brief    GPIOD pin interrupt mode configuration
 *
 * @param    pin - PDx
 *		     mode -
 *		    			GPIO_ITMode_LowLevel   -  Low level trigger
 *		    			GPIO_ITMode_HighLevel  -  High level trigger
 *		    			GPIO_ITMode_FallEdge   -  Falling edge trigger
 *		    			GPIO_ITMode_RiseEdge   -  Rising edge trigger
 *
 * @return   none
 */
void GPIOD_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
    switch (mode)
    {
    case GPIO_ITMode_FallEdge:
        R32_INT_MODE_PD |= pin;
        R32_INT_POLAR_PD &= ~pin;
        R32_INT_ENABLE_PD |= pin;
        break;

    case GPIO_ITMode_RiseEdge:
        R32_INT_MODE_PD |= pin;
        R32_INT_POLAR_PD |= pin;
        R32_INT_ENABLE_PD |= pin;
        break;

    case GPIO_ITMode_HighLevel:
        R32_INT_MODE_PD &= ~pin;
        R32_INT_POLAR_PD |= pin;
        R32_INT_ENABLE_PD |= pin;
        break;

    case GPIO_ITMode_LowLevel:
        R32_INT_MODE_PD &= ~pin;
        R32_INT_POLAR_PD &= ~pin;
        R32_INT_ENABLE_PD |= pin;
        break;

    case GPIO_ITMode_None:
        R32_INT_ENABLE_PD |= pin;
        R32_INT_ENABLE_PD &= ~pin;
        break;

    default:
        break;
    }
    R32_INT_STATUS_PD = pin;
}

/*******************************************************************************
 * @fn      GPIO_PinRemapConfig
 *
 * @brief   Remap GPIO function
 *
 * @param   GPIO_Remap - GPIO_Remap_x
 *		    NewSTA - ENABLE
 *                 - DISABLE
 *
 * @return  None
 */
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewSTA)
{
    uint32_t tempr = R32_AFIO_PCFR1;
    /*GPIO_Remap fomat:
     bit[31]:    Choose register R32_AFIO_PCFR1(0x0) or R32_AFIO_PCFR2(0x80000000) to be write
     bit[24:20]: Position of bits low anchor
     bit[19:16]: Size of bits
     bit[15:0]:  Specific value of remap
     */
    if (GPIO_Remap & 0x80000000)
    {
        tempr = R32_AFIO_PCFR2;
    }
    /*Clear bits*/
    tempr &= ~((~(0xffffffff << ((GPIO_Remap >> 16) & 0xf))) << ((GPIO_Remap >> 20) & 0x1f));
    /*Write bits*/
    if (NewSTA == ENABLE)
    {
        tempr |= (GPIO_Remap & (~(0xffffffff << ((GPIO_Remap >> 16) & 0xf)))) << ((GPIO_Remap >> 20) & 0x1f);
    }
    if (GPIO_Remap & 0x80000000)
        R32_AFIO_PCFR2 = tempr;
    else
        R32_AFIO_PCFR1 = tempr;
}
