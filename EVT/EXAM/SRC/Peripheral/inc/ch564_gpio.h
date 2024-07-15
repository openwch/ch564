/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_GPIO_H
#define __CH564_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

#define GPIO_Pin_0                       (0x00000001)/*!< Pin 0 selected */
#define GPIO_Pin_1                       (0x00000002)/*!< Pin 1 selected */
#define GPIO_Pin_2                       (0x00000004)/*!< Pin 2 selected */
#define GPIO_Pin_3                       (0x00000008)/*!< Pin 3 selected */
#define GPIO_Pin_4                       (0x00000010)/*!< Pin 4 selected */
#define GPIO_Pin_5                       (0x00000020)/*!< Pin 5 selected */
#define GPIO_Pin_6                       (0x00000040)/*!< Pin 6 selected */
#define GPIO_Pin_7                       (0x00000080)/*!< Pin 7 selected */
#define GPIO_Pin_8                       (0x00000100)/*!< Pin 8 selected */
#define GPIO_Pin_9                       (0x00000200)/*!< Pin 9 selected */
#define GPIO_Pin_10                      (0x00000400)/*!< Pin 10 selected */
#define GPIO_Pin_11                      (0x00000800)/*!< Pin 11 selected */
#define GPIO_Pin_12                      (0x00001000)/*!< Pin 12 selected */
#define GPIO_Pin_13                      (0x00002000)/*!< Pin 13 selected */
#define GPIO_Pin_14                      (0x00004000)/*!< Pin 14 selected */
#define GPIO_Pin_15                      (0x00008000)/*!< Pin 15 selected */
#define GPIO_Pin_16                      (0x00010000)/*!< Pin 16 selected */
#define GPIO_Pin_17                      (0x00020000)/*!< Pin 17 selected */
#define GPIO_Pin_18                      (0x00040000)/*!< Pin 18 selected */
#define GPIO_Pin_19                      (0x00080000)/*!< Pin 19 selected */
#define GPIO_Pin_20                      (0x00100000)/*!< Pin 20 selected */
#define GPIO_Pin_21                      (0x00200000)/*!< Pin 21 selected */
#define GPIO_Pin_22                      (0x00400000)/*!< Pin 22 selected */
#define GPIO_Pin_23                      (0x00800000)/*!< Pin 23 selected */
#define GPIO_Pin_24                      (0x01000000)/*!< Pin 24 selected */
#define GPIO_Pin_25                      (0x02000000)/*!< Pin 25 selected */
#define GPIO_Pin_26                      (0x04000000)/*!< Pin 26 selected */
#define GPIO_Pin_27                      (0x08000000)/*!< Pin 27 selected */
#define GPIO_Pin_28                      (0x10000000)/*!< Pin 28 selected */
#define GPIO_Pin_29                      (0x20000000)/*!< Pin 29 selected */
#define GPIO_Pin_30                      (0x40000000)/*!< Pin 30 selected */
#define GPIO_Pin_31                      (0x80000000)/*!< Pin 31 selected */
#define GPIO_Pin_All                     (0xFFFFFFFF)/*!< All pins selected */

#define GPIO_NoRemap_SPI0                (0x00020000)
#define GPIO_PartialRemap1_SPI0          (0x00020001)
#define GPIO_FullRemap_SPI0              (0x00020003)

#define GPIO_NoRemap_UART0               (0x00220000)
#define GPIO_PartialRemap2_UART0         (0x00220002)
#define GPIO_FullRemap_UART0             (0x00220003)

#define GPIO_NoRemap_UART1               (0x00420000)
#define GPIO_PartialRemap1_UART1         (0x00420001)
#define GPIO_FullRemap_UART1             (0x00420003)

#define GPIO_NoRemap_UART2               (0x00620000)
#define GPIO_PartialRemap1_UART2         (0x00620001)
#define GPIO_PartialRemap2_UART2         (0x00620002)
#define GPIO_FullRemap_UART2             (0x00620003)

#define GPIO_NoRemap_UART3               (0x00820000)
#define GPIO_PartialRemap1_UART3         (0x00820001)
#define GPIO_FullRemap_UART3             (0x00820003)

#define GPIO_NoRemap_UART0_MODEM         (0x00a20000)
#define GPIO_PartialRemap1_UART0_MODEM   (0x00a20001)
#define GPIO_PartialRemap2_UART0_MODEM   (0x00a20002)
#define GPIO_FullRemap_UART0_MODEM       (0x00a20003)

#define GPIO_NoRemap_UART1_MODEM         (0x00c20000)
#define GPIO_PartialRemap1_UART1_MODEM   (0x00c20001)
#define GPIO_PartialRemap2_UART1_MODEM   (0x00c20002)
#define GPIO_FullRemap_UART1_MODEM       (0x00c20003)

#define GPIO_NoRemap_UART2_MODEM         (0x00e20000)
#define GPIO_PartialRemap2_UART2_MODEM   (0x00e20002)
#define GPIO_FullRemap_UART2_MODEM       (0x00e20003)

#define GPIO_NoRemap_I2C                 (0x01020000)
#define GPIO_PartialRemap1_I2C           (0x01020001)

#define GPIO_NoRemap_SLV_INTERUPT        (0x01220000)
#define GPIO_PartialRemap1_SLV_INTERUPT  (0x01220001)
#define GPIO_PartialRemap2_SLV_INTERUPT  (0x01220002)
#define GPIO_FullRemap_SLV_INTERUPT      (0x01220003)

#define GPIO_NoRemap_SLV_CS              (0x01420000)
#define GPIO_PartialRemap1_SLV_CS        (0x01420001)

#define GPIO_NoRemap_SLV_ADDR            (0x01620000)
#define GPIO_PartialRemap1_SLV_ADDR      (0x01620001)
#define GPIO_PartialRemap2_SLV_ADDR      (0x01620002)

#define GPIO_NoRemap_SLV_ADDR1           (0x01820000)
#define GPIO_PartialRemap2_SLV_ADDR1     (0x01820002)
#define GPIO_FullRemap_SLV_ADDR1         (0x01820003)

#define GPIO_NoRemap_SLV_DATA            (0x01a20000)
#define GPIO_PartialRemap1_SLV_DATA      (0x01a20001)

#define GPIO_NolRemap_SLV_RW             (0x01c20000)
#define GPIO_PartialRemap1_SLV_RW        (0x01c20001)

#define GPIO_NoRemap_LINK_LED            (0x01e20000)
#define GPIO_PartialRemap1_LINK_LED      (0x01e20001)
#define GPIO_PartialRemap2_LINK_LED      (0x01e20002)
#define GPIO_FullRemap_LINK_LED          (0x01e20003)

#define GPIO_NoRemap_ACT_LED             (0x80020000)
#define GPIO_PartialRemap1_ACT_LED       (0x80020001)
#define GPIO_PartialRemap2_ACT_LED       (0x80020002)
#define GPIO_FullRemap_ACT_LED           (0x80020003)

#define GPIO_NoRemap_RST                 (0x80220000)
#define GPIO_PartialRemap1_RST           (0x80220001)
#define GPIO_PartialRemap2_RST           (0x80220002)
#define GPIO_FullRemap_RST               (0x80220003)

#define GPIO_NoRemap_TIMER0              (0x80410000)
#define GPIO_FullRemap_TIMER0            (0x80410001)

#define GPIO_NoRemap_TIMER1              (0x80510000)
#define GPIO_FullRemap_TIMER1            (0x80510001)

#define GPIO_NoRemap_BUSY                (0x80610000)
#define GPIO_FullRemap_BUSY              (0x80610001)

#define GPIO_NoRemap_SPI1                (0x80820000)
#define GPIO_FullRemap_SPI1              (0x80820003)

#define GPIO_NoRemap_TNOW0               (0x80a20000)
#define GPIO_FullRemap_TNOW0             (0x80a20003)

#define GPIO_NoRemap_TNOW1               (0x80c20000)
#define GPIO_FullRemap_TNOW1             (0x80c20003)

#define GPIO_NoRemap_TNOW2               (0x80e20000)
#define GPIO_FullRemap_TNOW2             (0x80e20003)

#define GPIO_NoRemap_TNOW3               (0x81020000)
#define GPIO_FullRemap_TNOW3             (0x81020003)

#define GPIO_NoRemap_UART3_MODEM         (0x81220000)
#define GPIO_FullRemap_UART3_MODEM       (0x81220003)

/**
 * @brief  GPIO mode structure configuration
 */
typedef enum
{
    GPIO_ModeIN_Floating = 0,
    GPIO_ModeIN_PU,
    GPIO_ModeIN_PD,
    GPIO_ModeOut_PP,
    GPIO_ModeOut_OP
} GPIOModeTypeDef;

/**
 * @brief  GPIO interrupt structure configuration
 */
typedef enum
{
    GPIO_ITMode_LowLevel = 0,  // Low level trigger
    GPIO_ITMode_HighLevel, // High level trigger
    GPIO_ITMode_FallEdge,  // Falling edge trigger
    GPIO_ITMode_RiseEdge,  // Rising edge trigger
    GPIO_ITMode_None
} GPIOITModeTpDef;

/**
 * @brief  GPIO MCO structure configuration
 */
typedef enum
{
    MCO_125 = 0,
    MCO_25 = 4,
    MCO_2d5 = 0xC,
} MCOMode;

void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode); /* GPIOA port pin mode configuration */
void GPIOB_ModeCfg(uint32_t pin, GPIOModeTypeDef mode); /* GPIOB port pin mode configuration */
void GPIOD_ModeCfg(uint32_t pin, GPIOModeTypeDef mode); /* GPIOB port pin mode configuration */
#define GPIOA_ResetBits(pin) (R32_PA_CLR |= pin)            /* GPIOA port pin output set low */
#define GPIOA_SetBits(pin) (R32_PA_OUT |= pin)              /* GPIOA port pin output set high */
#define GPIOB_ResetBits(pin) (R32_PB_CLR |= pin)            /* GPIOB port pin output set low */
#define GPIOB_SetBits(pin) (R32_PB_OUT |= pin)              /* GPIOB port pin output set high */
#define GPIOD_ResetBits(pin) (R32_PD_OUT &= ~pin)           /* GPIOA port pin output set low */
#define GPIOD_SetBits(pin) (R32_PD_OUT |= pin)              /* GPIOA port pin output set high */
#define GPIOA_InverseBits(pin) (R32_PA_OUT ^= pin)          /* GPIOA port pin output level flip */
#define GPIOB_InverseBits(pin) (R32_PB_OUT ^= pin)          /* GPIOB port pin output level flip */
#define GPIOD_InverseBits(pin) (R32_PD_OUT ^= pin)          /* GPIOB port pin output level flip */
#define GPIOA_ReadPort() (R32_PA_PIN) /* The 32-bit data returned by the GPIOA port, the lower 16 bits are valid */
#define GPIOB_ReadPort() (R32_PB_PIN) /* The 32-bit data returned by the GPIOB port, the lower 24 bits are valid */
#define GPIOD_ReadPort() (R32_PD_PIN) /* The 32-bit data returned by the GPIOB port, the lower 24 bits are valid */
#define GPIOA_ReadPortPin(pin) (R32_PA_PIN & pin) /* GPIOA port pin status, 0-pin low level, (!0)-pin high level */
#define GPIOB_ReadPortPin(pin) (R32_PB_PIN & pin) /* GPIOB port pin status, 0-pin low level, (!0)-pin high level */
#define GPIOD_ReadPortPin(pin) (R32_PD_PIN & pin) /* GPIOB port pin status, 0-pin low level, (!0)-pin high level */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode); /* GPIOA pin interrupt mode configuration */
void GPIOB_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode); /* GPIOB pin interrupt mode configuration */
void GPIOD_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode); /* GPIOB pin interrupt mode configuration */
#define GPIOA_ReadITFlagPort() (R32_INT_STATUS_PA)            /* Read GPIOA port interrupt flag status */
#define GPIOB_ReadITFlagPort() (R32_INT_STATUS_PB)            /* Read GPIOB port interrupt flag status */
#define GPIOD_ReadITFlagPort() (R32_INT_STATUS_PD)            /* Read GPIOD port interrupt flag status */

/*************************************Read Interrupt Bit Flag************************************/
#define GPIOA_ReadITFLAGBit(pin) (R32_INT_STATUS_PA & pin)
#define GPIOB_ReadITFLAGBit(pin) (R32_INT_STATUS_PB & pin)
#define GPIOD_ReadITFLAGBit(pin) (R32_INT_STATUS_PD & pin)

/*************************************Clear Interrupt Bit Flag************************************/
#define GPIOA_ClearITFlagbit(pin) (R32_INT_STATUS_PA |= pin)
#define GPIOB_ClearITFlagbit(pin) (R32_INT_STATUS_PB |= pin)
#define GPIOD_ClearITFlagbit(pin) (R32_INT_STATUS_PD |= pin)

void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewSTA);
void GPIO_IPD_Unused(void);

#ifdef __cplusplus
}
#endif

#endif
