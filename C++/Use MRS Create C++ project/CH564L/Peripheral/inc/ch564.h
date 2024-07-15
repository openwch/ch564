
/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2023/11/11
 * Description        : CH564 Device Peripheral Access Layer Header File.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_H
#define __CH564_H

#ifdef __cplusplus
extern "C"
{
#endif

#define HSE_VALUE                       ((uint32_t)25000000)                                                          /* Value of the External oscillator in Hz */

/* In the following line adjust the External High Speed oscillator (HSE) Startup Timeout value */
#define HSE_STARTUP_TIMEOUT             ((uint16_t)0x1000)                                                            /* Time out for HSE start up */

#define HSI_VALUE                       20000000                                                                      /* Value of the Internal oscillator in Hz */

/* CH564 Standard Peripheral Library version number */
#define __CH564_STDPERIPH_VERSION_MAIN  (0x01)                                                                        /* [15:8] main version */
#define __CH564_STDPERIPH_VERSION_SUB   (0x00)                                                                        /* [7:0] sub version */
#define __CH564_STDPERIPH_VERSION       ((__CH564_STDPERIPH_VERSION_MAIN << 8) | (__CH564_STDPERIPH_VERSION_SUB << 0))

/* ********************************************************************************************************************* */
/* Base types & constants */

#ifndef TRUE
#define TRUE                    1
#define FALSE                   0
#endif
#ifndef NULL
#define NULL                    0
#endif

#ifndef VOID
#define VOID                    void
#endif
#ifndef CONST
#define CONST                   const
#endif
#ifndef BOOL
typedef unsigned char           BOOL;
#endif
#ifndef BOOLEAN
typedef unsigned char           BOOLEAN;
#endif
#ifndef CHAR
typedef char                    CHAR;
#endif
#ifndef INT8
typedef char                    INT8;
#endif
#ifndef INT16
typedef short                   INT16;
#endif
#ifndef INT32
typedef long                    INT32;
#endif
#ifndef UINT8
typedef unsigned char           UINT8;
#endif
#ifndef UINT16
typedef unsigned short          UINT16;
#endif
#ifndef UINT32
typedef unsigned long           UINT32;
#endif
#ifndef UINT8V
typedef unsigned char volatile  UINT8V;
#endif
#ifndef UINT16V
typedef unsigned short volatile UINT16V;
#endif
#ifndef UINT32V
typedef unsigned long volatile  UINT32V;
#endif

#ifndef PVOID
typedef void                    *PVOID;
#endif
#ifndef PCHAR
typedef char                    *PCHAR;
#endif
#ifndef PCHAR
typedef const char              *PCCHAR;
#endif
#ifndef PINT8
typedef char                    *PINT8;
#endif
#ifndef PINT16
typedef short                   *PINT16;
#endif
#ifndef PINT32
typedef long                    *PINT32;
#endif
#ifndef PUINT8
typedef unsigned char           *PUINT8;
#endif
#ifndef PUINT16
typedef unsigned short          *PUINT16;
#endif
#ifndef PUINT32
typedef unsigned long           *PUINT32;
#endif
#ifndef PUINT8V
typedef volatile unsigned char  *PUINT8V;
#endif
#ifndef PUINT16V
typedef volatile unsigned short *PUINT16V;
#endif
#ifndef PUINT32V
typedef volatile unsigned long  *PUINT32V;
#endif

/* ********************************************************************************************************************* */
/* Base macros */

#ifndef min
#define min(a,b)                (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a,b)                (((a) > (b)) ? (a) : (b))
#endif

#if		DEBUG
#define PRINT(X...) printf(X)
#else
#define PRINT(X...)
#endif

/* Calculate the byte offset of a field in a structure of type */
#define FIELD_OFFSET(Type, Field)    ((UINT16)&(((Type *)0)->Field))

/* Calculate the size of a field in a structure of type */
#define FIELD_SIZE(Type, Field)      (sizeof(((Type *)0)->Field))

/* An expression that yields the type of a field in a struct */
#define FIELD_TYPE(Type, Field)      (((Type *)0)->Field)

/* Return the number of elements in a statically sized array */
#define NUMBER_OF(Array)             (sizeof(Array)/sizeof((Array)[0]))
#define NUMBER_OF_FIELD(Type, Field) (NUMBER_OF(FIELD_TYPE(Type, Field)))

/* Interrupt Number Definition, according to the selected device */
typedef enum IRQn
{
    /******  RISC-V Processor Exceptions Numbers *******************************************************/
    NonMaskableInt_IRQn = 2, /* 2 Non Maskable Interrupt*/
    HardFault_IRQn = 3,      /* 3 Exception Interrupt*/
    SysTick_IRQn = 12,       /* 12 System timer Interrupt*/
    Software_IRQn = 14,      /* 14 software Interrupt*/

    /******  RISC-V specific Interrupt Numbers *********************************************************/
    IIC_EV_IRQn = 16,
    IIC_ER_IRQn = 17,
    ETH_IRQn = 18,
    USBPD_IRQn = 19,
    TIM0_IRQn = 20,
    TIM1_IRQn = 21,
    TIM2_IRQn = 22,
    TIM3_IRQn = 23,
    SPI0_IRQn = 24,
    SPI1_IRQn = 25,
    UART0_IRQn = 26,
    UART1_IRQn = 27,
    PA_IRQn = 28,
    PB_IRQn = 29,
    PD_IRQn = 30,
    ADC_IRQn = 31,
    SLV_IRQn = 32,
    USBHS_HOST_IRQn = 33,
    USBHS_DEV_IRQn = 34,
    UART2_IRQn = 35,
    UART3_IRQn = 36,
    ETHWakeUp_IRQn = 37,
    USBHSWakeUp_IRQn = 38,
    UPDWakeUp_IRQn = 39,
} IRQn_Type;

#include <stdint.h>
#include "core_riscv.h"
#include "system_ch564.h"

/* Reset and Clock Control */
typedef struct
{
    __IO uint8_t SAFE_ACCESS_SIG;
    uint8_t RESERVED0;
    __IO uint8_t SAFE_ACCESS_ID;
    __IO uint8_t WDOG_CLEAR;
    __IO uint8_t GLOB_MEM_CFG;
    __IO uint8_t GLOB_LOCK_PORT;
    __IO uint8_t GLOB_RST_CFG;
    __IO uint8_t GLOB_RESET_KEEP;
    uint8_t RESERVED1;
    __IO uint8_t PLL_OUT_DIV;
    uint16_t RESERVED2;
    __IO uint8_t SLP_CLK_OFF0;
    __IO uint8_t SLP_CLK_OFF1;
    __IO uint8_t SLP_WAKE_CTRL;
    __IO uint8_t SLP_CTRL_PLL;
} RCC_Typedef;
/* General Purpose Input Output */
typedef struct
{
    __IO uint32_t DIR;
    __IO uint32_t PIN;
    __IO uint32_t OUT;
    __IO uint32_t CLR;
    __IO uint32_t PU;
    __IO uint32_t PD;
} GPIO_Typedef;
/* Alternate Function IO */
typedef struct
{
    __IO uint32_t PCFR1;
    __IO uint32_t PCFR2;
} AFIO_Typedef;
/* External Interrupts */
typedef struct
{
    __IO uint32_t STATUS;
    __IO uint32_t ENABLE;
    __IO uint32_t MODE;
    __IO uint32_t POLAR;
} EXTI_Typedef;
/* Timer */
typedef struct
{
    __IO uint8_t CTRL_MOD;
    __IO uint8_t CTRL_DMA;
    __IO uint8_t INTER_EN;
    uint8_t RESERVED0[3];
    __IO uint8_t INT_FLAG;
    __IO uint8_t FIFO_COUNT;
    __IO uint32_t COUNT;
    __IO uint32_t CNT_END;
    __IO uint32_t FIFO;
    union
    {
        __IO uint32_t TMR0_DMA_NOW;
        __IO uint32_t TMR1_DMA_NOW;
        __IO uint32_t TMR2_DMA_NOW;
        uint32_t RESERVED1;
    };
    union
    {
        __IO uint32_t TMR0_DMA_BEG;
        __IO uint32_t TMR1_DMA_BEG;
        __IO uint32_t TMR2_DMA_BEG;
        uint32_t RESERVED2;
    };
    union
    {
        __IO uint32_t TMR0_DMA_END;
        __IO uint32_t TMR1_DMA_END;
        __IO uint32_t TMR2_DMA_END;
        uint32_t RESERVED3;
    };

} TIM_Typedef;
/* Analog-to-Digital Converter */
typedef struct
{
    __IO uint8_t CTRL_MOD;
    __IO uint8_t CTRL_DMA;
    __IO uint8_t INTER_EN;
    __IO uint8_t CLOCK_DIV;
    __IO uint16_t DATA;
    __IO uint8_t INT_FLAG;
    __IO uint8_t FIFO_COUNT;
    __IO uint32_t CTRL;
    __IO uint16_t CMP_VALUE;
    uint16_t RESERVED0;
    __IO uint16_t FIFO;
    uint16_t RESERVED1;
    __IO uint32_t DMA_NOW;
    __IO uint32_t DMA_BEG;
    __IO uint32_t DMA_END;
} ADC_Typedef;
/* Universal Synchronous/Asynchronous Receiver/Transmitter */
typedef struct
{
    union
    {
        __IO uint8_t RBR;
        __IO uint8_t THR;
        __IO uint8_t DLL;
        __IO uint8_t ADR;
    };
    union
    {
        __IO uint8_t IER;
        __IO uint8_t DLM;
    };
    union
    {
        __IO uint8_t IIR;
        __IO uint8_t FCR;
    };
    __IO uint8_t LCR;
    __IO uint8_t MCR;
    __IO uint8_t LSR;
    __IO uint8_t MSR;
    __IO uint8_t DIV;
    __IO uint8_t DMA_CTRL;
    __IO uint8_t DMA_IF;
    uint16_t RESERVED0;
    __IO uint32_t DMA_WR_NOW_ADDR;
    __IO uint32_t DMA_WR_START_ADDR;
    __IO uint32_t DMA_WR_END_ADDR;
    __IO uint32_t DMA_RD_NOW_ADDR;
    __IO uint32_t DMA_RD_START_ADDR;
    __IO uint32_t DMA_RD_END_ADDR;
} UART_Typedef;

/* Inter Integrated Circuit Interface */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t RESERVED1;
    __IO uint16_t OADDR1;
    uint16_t RESERVED2;
    __IO uint16_t OADDR2;
    uint16_t RESERVED3;
    __IO uint16_t DATAR;
    uint16_t RESERVED4;
    __IO uint16_t STAR1;
    uint16_t RESERVED5;
    __IO uint16_t STAR2;
    uint16_t RESERVED6;
    __IO uint16_t CKCFGR;
    uint16_t RESERVED7;
    __IO uint16_t RTR;
} I2C_Typedef;
/* Serial Peripheral Interface */
typedef struct
{
    union
    {
        __IO uint32_t CONTROL_R32;
        struct
        {
            __IO uint8_t CTRL_MOD;
            __IO uint8_t CTRL_DMA;
            __IO uint8_t INTER_EN;
            union
            {
                __IO uint8_t CLOCK_DIV;
                __IO uint8_t SLAVE_PRE;
            };
        };
    };
    union
    {
        __IO uint32_t STATUS_R32;
        struct
        {
            __IO uint8_t BUFFER;
            __IO uint8_t RUN_FLAG;
            __IO uint8_t INT_FLAG;
            __IO uint8_t FIFO_COUNT;
        };
    };
    __IO uint8_t RESET_CMD;
    __IO uint8_t BUSY;
    uint16_t RESERVED0;
    __IO uint16_t TOTAL_CNT;
    uint16_t RESERVED1;
    union
    {
        __IO uint32_t FIFO_R32;
        struct
        {
            __IO uint8_t FIFO;
            uint16_t RESERVED2;
            __IO uint8_t FIFO_COUNT1;
        };
    };
    union
    {
        __IO uint32_t SPI0_DMA_NOW;
        uint32_t RESERVED3;
    };
    union
    {
        __IO uint32_t SPI0_DMA_BEG;
        uint32_t RESERVED4;
    };
    union
    {
        __IO uint32_t SPI0_DMA_END;
        uint32_t RESERVED5;
    };
} SPI_Typedef;
/* passive parallel port (computing) */
typedef struct
{
    __IO uint8_t CONFIG;
    uint8_t RESERVED0;
    __IO uint8_t DOUT;
    __IO uint8_t STATUS;
    uint8_t RESERVED1[42];
    __IO uint8_t INT_FLAG_SLV;
    __IO uint8_t INT_SLV_DIN;
    uint8_t RESERVED2[316];
    __IO uint8_t DMA_EN_SLV;
    __IO uint8_t DMA_MODE_CTRL_SLV;
    __IO uint8_t DMA_MODE_EN_SLV;
    uint8_t RESERVED3;
    __IO uint8_t DMA_INT_FLAG_SLV;
    uint8_t RESERVED4[3];
    __IO uint32_t WR_DMA_START_ADDR_SLV;
    __IO uint32_t WR_DMA_END_ADDR_SLV;
    uint32_t RESERVED5;
    __IO uint32_t RD_DMA_START_ADDR_SLV;
    __IO uint32_t RD_DMA_END_ADDR_SLV;
    __IO uint32_t DMA_END_NOW_SLV;
    __IO uint8_t DMA_CMD0_SLV;
    __IO uint8_t DMA_CMD1_SLV;
    uint16_t RESERVED6;
    __IO uint8_t SLV_RESET_CMD;
    __IO uint8_t SLV_BUSY;
    uint8_t RESERVED7[10];
    __IO uint8_t OTHER_DATA;
    uint8_t RESERVED9[3];
    __IO uint16_t DMA_DEC_LEN;
    __IO uint16_t DMA_DEC_OFFSET;
} SLV_Typedef;
/* XBUS */
typedef struct
{
    __IO uint8_t CONFIG;
    uint8_t RESERVED0;
    __IO uint8_t CYCLE;
    __IO uint8_t SETUP_HOLD;
} XBUS_Typedef;

/* Ethernet MAC */
typedef struct
{
    __IO uint32_t MACCR;
    __IO uint32_t MACFFR;
    __IO uint32_t MACHTHR;
    __IO uint32_t MACHTLR;
    __IO uint32_t MACMIIAR;
    __IO uint32_t MACMIIDR;
    __IO uint32_t MACFCR;
    __IO uint32_t MACVLANTR;
    uint32_t RESERVED0[2];
    __IO uint32_t MACRWUFFR;
    __IO uint32_t MACPMTCSR;
    uint32_t RESERVED1[2];
    __IO uint32_t MACSR;
    __IO uint32_t MACIMR;
    __IO uint32_t MACA0HR;
    __IO uint32_t MACA0LR;
    __IO uint32_t MACA1HR;
    __IO uint32_t MACA1LR;
    __IO uint32_t MACA2HR;
    __IO uint32_t MACA2LR;
    __IO uint32_t MACA3HR;
    __IO uint32_t MACA3LR;
    uint32_t RESERVED2[8];
    __IO uint32_t PHY_CR;
    __IO uint32_t CHKSUM_CR;
    __IO uint32_t IP_PDR;
    __IO uint32_t CHKSUM_HR;
    __IO uint32_t CHKSUM_PR;
    uint32_t RESERVED3[27];
    __IO uint32_t MMCCR;
    __IO uint32_t MMCRIR;
    __IO uint32_t MMCTIR;
    __IO uint32_t MMCRIMR;
    __IO uint32_t MMCTIMR;
    uint32_t RESERVED4[14];
    __IO uint32_t MMCTGFSCCR;
    __IO uint32_t MMCTGFMSCCR;
    uint32_t RESERVED5[5];
    __IO uint32_t MMCTGFCR;
    uint32_t RESERVED6[10];
    __IO uint32_t MMCRFCECR;
    __IO uint32_t MMCRFAECR;
    __IO uint32_t MMCRAFCR;
    __IO uint32_t ETH_STATE;
    uint32_t RESERVED7[8];
    __IO uint32_t MMCRGUFCR;
    uint32_t RESERVED8[334];
    __IO uint32_t PTPTSCR;
    __IO uint32_t PTPSSIR;
    __IO uint32_t PTPTSHR;
    __IO uint32_t PTPTSLR;
    __IO uint32_t PTPTSHUR;
    __IO uint32_t PTPTSLUR;
    __IO uint32_t PTPTSAR;
    __IO uint32_t PTPTTHR;
    __IO uint32_t PTPTTLR;
    uint32_t RESERVED9[567];
    __IO uint32_t DMABMR;
    __IO uint32_t DMATPDR;
    __IO uint32_t DMARPDR;
    __IO uint32_t DMARDLAR;
    __IO uint32_t DMATDLAR;
    __IO uint32_t DMASR;
    __IO uint32_t DMAOMR;
    __IO uint32_t DMAIER;
    __IO uint32_t DMAMFBOCR;
    uint32_t RESERVED10[9];
    __IO uint32_t DMACHTDR;
    __IO uint32_t DMACHRDR;
    __IO uint32_t DMACHTBAR;
    __IO uint32_t DMACHRBAR;
} ETH_Typedef;
/* USB Power Delivery */
typedef struct
{
    union
    {
        __IO uint32_t CONFIG_R32;
        struct
        {
            __IO uint16_t CONFIG;
            __IO uint16_t BMC_CLK_CNT;
        };
    };
    union
    {
        __IO uint32_t CONTROL_R32;
        struct
        {
            union
            {
                __IO uint16_t CONTROL_R16;
                struct
                {
                    __IO uint8_t CONTROL;
                    __IO uint8_t TX_SEL;
                };
            };
            __IO uint16_t BMC_TX_SZ;
        };
    };
    union
    {
        __IO uint32_t STATUS_R32;
        struct
        {
            union
            {
                __IO uint16_t STATUS_16;
                struct
                {
                    __IO uint8_t DATA_BUF;
                    __IO uint8_t STATUS;
                };
            };
            __IO uint16_t BMC_BYTE_CNT;
        };
    };
    union
    {
        __IO uint32_t PORT_R32;
        struct
        {
            __IO uint16_t PORT_CC1;
            __IO uint16_t PORT_CC2;
        };
    };
    union
    {
        __IO uint32_t DMA_R32;
        struct
        {
            __IO uint16_t DMA;
            uint16_t RESERVED0;
        };
    };
} USBPD_Typedef;
/* USB High-Speed Device */
typedef struct
{
  __IO uint8_t  CONTROL;
  __IO uint8_t  BASE_MODE;
  __IO uint8_t  INT_EN;
  __IO uint8_t  DEV_AD;
  __IO uint8_t  WAKE_CR;
  __IO uint8_t  TEST_MODE;
  __IO uint16_t LPM_DATA;
  __IO uint8_t  INT_FG;
  __IO uint8_t  INT_ST;
  __IO uint8_t  MIS_ST;
  __IO uint8_t  reserve1;
  __IO uint16_t FRAME_NO;
  __IO uint16_t USB_BUS;
  __IO uint16_t UEP_TX_EN;
  __IO uint16_t UEP_RX_EN;
  __IO uint16_t UEP_T_TOG_AUTO;
  __IO uint16_t UEP_R_TOG_AUTO;
  __IO uint16_t UEP_T_BURST;
  __IO uint16_t UEP_R_BURST;
  __IO uint32_t UEP_AF_MODE;
  __IO uint32_t UEP0_DMA;
  __IO uint32_t UEP1_RX_DMA;
  __IO uint32_t UEP2_RX_DMA;
  __IO uint32_t UEP3_RX_DMA;
  __IO uint32_t UEP4_RX_DMA;
  __IO uint32_t UEP5_RX_DMA;
  __IO uint32_t UEP6_RX_DMA;
  __IO uint32_t UEP7_RX_DMA;
  __IO uint32_t UEP1_TX_DMA;
  __IO uint32_t UEP2_TX_DMA;
  __IO uint32_t UEP3_TX_DMA;
  __IO uint32_t UEP4_TX_DMA;
  __IO uint32_t UEP5_TX_DMA;
  __IO uint32_t UEP6_TX_DMA;
  __IO uint32_t UEP7_TX_DMA;
  __IO uint32_t UEP0_MAX_LEN;
  __IO uint32_t UEP1_MAX_LEN;
  __IO uint32_t UEP2_MAX_LEN;
  __IO uint32_t UEP3_MAX_LEN;
  __IO uint32_t UEP4_MAX_LEN;
  __IO uint32_t UEP5_MAX_LEN;
  __IO uint32_t UEP6_MAX_LEN;
  __IO uint32_t UEP7_MAX_LEN;
  __IO uint16_t UEP0_RX_LEN;
  __IO uint16_t UEP0_RX_SIZE;
  __IO uint16_t UEP1_RX_LEN;
  __IO uint16_t UEP1_RX_SIZE;
  __IO uint16_t UEP2_RX_LEN;
  __IO uint16_t UEP2_RX_SIZE;
  __IO uint16_t UEP3_RX_LEN;
  __IO uint16_t UEP3_RX_SIZE;
  __IO uint16_t UEP4_RX_LEN;
  __IO uint16_t UEP4_RX_SIZE;
  __IO uint16_t UEP5_RX_LEN;
  __IO uint16_t UEP5_RX_SIZE;
  __IO uint16_t UEP6_RX_LEN;
  __IO uint16_t UEP6_RX_SIZE;
  __IO uint16_t UEP7_RX_LEN;
  __IO uint16_t UEP7_RX_SIZE;
  __IO uint16_t UEP0_TX_LEN;
  __IO uint8_t  UEP0_TX_CTRL;
  __IO uint8_t  UEP0_RX_CTRL;
  __IO uint16_t UEP1_TX_LEN;
  __IO uint8_t  UEP1_TX_CTRL;
  __IO uint8_t  UEP1_RX_CTRL;
  __IO uint16_t UEP2_TX_LEN;
  __IO uint8_t  UEP2_TX_CTRL;
  __IO uint8_t  UEP2_RX_CTRL;
  __IO uint16_t UEP3_TX_LEN;
  __IO uint8_t  UEP3_TX_CTRL;
  __IO uint8_t  UEP3_RX_CTRL;
  __IO uint16_t UEP4_TX_LEN;
  __IO uint8_t  UEP4_TX_CTRL;
  __IO uint8_t  UEP4_RX_CTRL;
  __IO uint16_t UEP5_TX_LEN;
  __IO uint8_t  UEP5_TX_CTRL;
  __IO uint8_t  UEP5_RX_CTRL;
  __IO uint16_t UEP6_TX_LEN;
  __IO uint8_t  UEP6_TX_CTRL;
  __IO uint8_t  UEP6_RX_CTRL;
  __IO uint16_t UEP7_TX_LEN;
  __IO uint8_t  UEP7_TX_CTRL;
  __IO uint8_t  UEP7_RX_CTRL;
  __IO uint16_t UEP_TX_ISO;
  __IO uint16_t UEP_RX_ISO;
} USBHSD_TypeDef;

typedef struct
{
  __IO uint8_t   CFG;
  __IO uint8_t   RESERVE1;
  __IO uint8_t   INT_EN;
  __IO uint8_t   DEV_AD;
  __IO uint32_t  CONTROL;
  __IO uint8_t   INT_FG;
  __IO uint8_t   INT_ST;
  __IO uint8_t   MIS_ST;
  __IO uint8_t   RESERVE2;
  __IO uint32_t  LPM;
  __IO uint32_t  SPLIT;
  __IO uint32_t  FRAME;
  __IO uint32_t  HOST_TX_LEN;
  __IO uint32_t  RX_LEN;
  __IO uint32_t  HOST_RX_MAX_LEN;
  __IO uint32_t  HOST_RX_DMA;
  __IO uint32_t  HOST_TX_DMA;
  __IO uint32_t  PORT_CTRL;
  __IO uint8_t   PORT_CFG;
  __IO uint8_t   RESERVE3;
  __IO uint8_t   PORT_INT_EN;
  __IO uint8_t   PORT_TEST_CT;
  __IO uint16_t  PORT_STATUS;
  __IO uint8_t   PORT_STATUS_CHG;
  __IO uint32_t  RESERVE4;
  __IO uint32_t  ROOT_BC_CR;
} USBHSH_TypeDef;
typedef struct
{
  __IO uint32_t  CAL_CR;
} USBHSI_TypeDef;
/* Expansion Registers */
typedef struct
{
    __IO uint32_t CTLR0;
    uint32_t RESERVED0[3];
    __IO uint32_t CTLR1;
    uint32_t RESERVED1[5];
    __IO uint32_t CTLR2;
} EXTEN_Typedef;
/* Flash Registers */
typedef struct
{
    __IO uint8_t DATA;
    uint8_t RESERVED0;
    __IO uint8_t CTRL;
    __IO uint16_t FLASHA_KEY_BUF;
} FLASH_Typedef;
/* Debug Registers */
typedef struct
{
    __IO uint32_t CR;
} DBG_Typedef;
#pragma pack()

#define RCC_BASE        0x40400000
#define GPIOA_BASE      0x40400080
#define GPIOB_BASE      0x404000A0
#define GPIOD_BASE      0x404000C0
#define AFIO_BASE       0x40400100
#define EXTIA_BASE      0x40400050
#define EXTIB_BASE      0x40400060
#define EXTID_BASE      0x40400070
#define TIM0_BASE       0x40408000
#define TIM1_BASE       0x40408400
#define TIM2_BASE       0x40408800
#define TIM3_BASE       0x40408C00
#define ADC_BASE        0x4040A000
#define UART0_BASE      0x4040D000
#define UART1_BASE      0x4040D800
#define UART2_BASE      0x40409000
#define UART3_BASE      0x40409800
#define I2C_BASE        0x4040F000
#define SPI0_BASE       0x4040C000
#define SPI1_BASE       0x4040C800
#define SLV_BASE        0x40400014
#define XBUS_BASE       0x40400010
#define ETH_BASE        0x40406000
#define USBPD_BASE      0x4040E000
#define USBHSD_BASE     0x40404000
#define USBHSH_BASE     0x40404800
#define USB_HSI_BASE    0x40005000
#define EXTEN_BASE      0x40400144
#define FLASH_BASE      0x40400018
#define DBG_BASE        0x7C0

#define RCC         ((RCC_Typedef *)RCC_BASE)
#define GPIOA       ((GPIO_Typedef *)GPIOA_BASE)
#define GPIOB       ((GPIO_Typedef *)GPIOB_BASE)
#define GPIOD       ((GPIO_Typedef *)GPIOD_BASE)
#define AFIO        ((AFIO_Typedef *)AFIO_BASE)
#define EXTIA       ((EXTI_Typedef *)EXTIA_BASE)
#define EXTIB       ((EXTI_Typedef *)EXTIB_BASE)
#define EXTID       ((EXTI_Typedef *)EXTID_BASE)
#define TIM0        ((TIM_Typedef *)TIM0_BASE)
#define TIM1        ((TIM_Typedef *)TIM1_BASE)
#define TIM2        ((TIM_Typedef *)TIM2_BASE)
#define TIM3        ((TIM_Typedef *)TIM3_BASE)
#define ADC         ((ADC_Typedef *)ADC_BASE)
#define UART0       ((UART_Typedef *)UART0_BASE)
#define UART1       ((UART_Typedef *)UART1_BASE)
#define UART2       ((UART_Typedef *)UART2_BASE)
#define UART3       ((UART_Typedef *)UART3_BASE)
#define I2C         ((I2C_Typedef *)I2C_BASE)
#define SPI0        ((SPI_Typedef *)SPI0_BASE)
#define SPI1        ((SPI_Typedef *)SPI1_BASE)
#define SLV         ((SLV_Typedef *)SLV_BASE)
#define XBUS        ((XBUS_Typedef *)XBUS_BASE)
#define ETH         ((ETH_Typedef *)ETH_BASE)
#define USBPD       ((USBPD_Typedef *)USBPD_BASE)
#define USBHSD      ((USBHSD_TypeDef *) USBHSD_BASE)
#define USBHSH      ((USBHSH_TypeDef *) USBHSH_BASE)
#define USB_HSI     ((USBHSI_TypeDef *) USB_HSI_BASE)
#define EXTEN       ((EXTEN_Typedef *)EXTEN_BASE)
#define FLASH       ((FLASH_Typedef *)FLASH_BASE)
#define DBG         ((DBG_Typedef *)DBG_BASE)

/* Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSI_Value                       HSI_VALUE
#define HSE_Value                       HSE_VALUE

/* ********************************************************************************************************************* */
/* Base types & constants */

#ifndef TRUE
#define TRUE                            1
#define FALSE                           0
#endif
#ifndef NULL
#define NULL                            0
#endif

/******************************************************************************/

// Address Space
//   CODE:   00000000H - 000FFFFFH, 1MB
//    SFR:   00400000H - 0040FFFFH, 64KB
//   DATA:   00808000H - 0081FFFFH, 96KB/64KB/32KB
//   XBUS:   00C00000H - 00CFFFFFH, 1MB
//  OTHER:   00E00000H - FFFFFFFFH, undefined
//
//    SFR:   00400000H - 0040FFFFH, 64KB
//      SYS:    +1000H - 1FFFH, include base config, interrupt, GPIO, etc...
//      USB:    +4000H - 5FFFH
//      ETH:    +6000H - 7FFFH
//      TMR0:   +8000H - 83FFH
//      TMR1:   +8400H - 87FFH
//      TMR2:   +8800H - 8BFFH
//      TMR3:   +8C00H - 8FFFH
//      ADC:    +A000H - AFFFH
//      SPI0:   +C000H - C7FFH
//      SPI1:   +C800H - CFFFH
//      UART0:  +D000H - D7FFH
//      UART1:  +D800H - DFFFH

// Register Bit Attribute / Bit Access Type
//   RF:    Read only for Fixed value
//   RO:    Read Only (internal change)
//   RZ:    Read only with auto clear Zero
//   WO:    Write Only (read zero or different)
//   WA:    Write only under safe Accessing mode (read zero or different)
//   WZ:    Write only with auto clear Zero
//   RW:    Read / Write
//   RWA:   Read / Write under safe Accessing mode
//   RW1:   Read / Write 1 to Clear

/* Register name rule:
R32_* for 32 bits register (UINT32,ULONG)
R16_* for 16 bits register (UINT16,USHORT)
R8_*  for  8 bits register (UINT8,UCHAR)
RB_*  for bit or bit mask of 8 bit register
BA_*  for base address point
Others for register address offset */

/* ********************************************************************************************************************* */
/* System: safe accessing register */
#define R32_SAFE_ACCESS                                 (*((volatile uint32_t *)0x40400000))                        // RW, safe accessing
#define R8_SAFE_ACCESS_SIG                              (*((volatile uint8_t *)0x40400000))                         // WO, safe accessing sign register, must write 0x57 then 0xA8 to enter safe accessing mode
#define RB_SAFE_ACC_MODE                                0x03                                                        // RO, current safe accessing mode: 11=safe/unlocked (SAM), other=locked (00..01..10..11)
#define RB_SAFE_ACC_TIMER                               0x70                                                        // RO, safe accessing timer bit mask (16*clock number)
#define R8_SAFE_ACCESS_ID                               (*((volatile uint8_t *)0x40400002))                         // RF, safe accessing ID register, always 0x01
#define R8_WDOG_CLEAR                                   (*((volatile uint8_t *)0x40400003))                         // WO, clear watch-dog
        
/* System: global configuration register */     
#define R32_GLOBAL_CONFIG                               (*((volatile uint32_t *)0x40400004))                        // RW, global configuration
#define R8_GLOB_MEM_CFG                                 (*((volatile uint8_t *)0x40400004))                         // RWA, global memory configuration, SAM and bit7:6 must write 1:0
#define RB_GLOB_MEM_CFG                                 0x03                                                        // RWA, global memory config: 10=C96K/D32K, 11=C32K/D96K, 00/01=C64K/D64K
#define RB_GLOB_CFG_FLAG                                0x80                                                        // RO, global config flag
#define R8_GLOB_LOCK_PORT                               (*((volatile uint8_t *)0x40400005))                         // RWA, lock port configuration, SAM and bit7:6 must write 0:0
#define RB_GLOB_LOCK_PA                                 0x01                                                        // RWA, lock GPIO PA
#define RB_GLOB_LOCK_PB                                 0x02                                                        // RWA, lock GPIO PB
#define RB_GLOB_LOCK_PD                                 0x08                                                        // RWA, lock GPIO PD
#define R8_GLOB_RST_CFG                                 (*((volatile uint8_t *)0x40400006))                         // RWA, global reset configuration, SAM and bit7:6 must write 0:1
#define RB_GLOB_FORCE_RST                               0x01                                                        // WA/WZ, force global reset, high action, auto clear
#define RB_GLOB_WDOG_EN                                 0x02                                                        // RWA, watch-dog enable
#define R8_GLOB_RESET_KEEP                              (*((volatile uint8_t *)0x40400007))                         // RW, value keeper during global reset
        
/* System: PLL configuration register */        
#define R32_PLL_CONFIG                                  (*((volatile uint32_t *)0x40400008))                        // RWA, PLL configuration, SAM
#define R8_PLL_OUT_DIV                                  (*((volatile uint8_t *)0x40400009))                         // RWA, PLL output clock divider, SAM and bit7:6 must write 1:0
#define RB_PLL_ETH_DIV                                  0x0F                                                        // RWA, PLL ethernet clock divider
#define RB_PLL_SYS_DIV                                  0x30                                                        // RWA, PLL system clock divider

/* System: sleep control register */
#define R32_SLEEP_CONTROL                               (*((volatile uint32_t *)0x4040000C))                        // RWA, sleep control, SAM
#define R8_SLP_CLK_OFF0                                 (*((volatile uint8_t *)0x4040000C))                         // RWA, sleep clock off control byte 0, SAM
#define RB_SLP_CLK_TMR0                                 0x01                                                        // RWA, sleep TMR0 clock
#define RB_SLP_CLK_TMR1                                 0x02                                                        // RWA, sleep TMR1 clock
#define RB_SLP_CLK_TMR2                                 0x04                                                        // RWA, sleep TMR2 clock
#define RB_SLP_CLK_TMR3                                 0x08                                                        // RWA, sleep TMR3 clock
#define RB_SLP_CLK_SPI0                                 0x10                                                        // RWA, sleep SPI0 clock
#define RB_SLP_CLK_SPI1                                 0x20                                                        // RWA, sleep SPI1 clock
#define RB_SLP_CLK_UART0                                0x40                                                        // RWA, sleep UART0 clock
#define RB_SLP_CLK_UART1                                0x80                                                        // RWA, sleep UART1 clock
#define R8_SLP_CLK_OFF1                                 (*((volatile uint8_t *)0x4040000D))                         // RWA, sleep clock off control byte 1, SAM
#define RB_SLP_CLK_UTMI                                 0x02                                                        // RWA, sleep UTMI clock
#define RB_SLP_CLK_I2C                                  0x04                                                        // RWA, sleep I2C clock
#define RB_SLP_CLK_USBPD                                0x08                                                        // RWA, sleep USBPD clock
#define RB_SLP_CLK_ADC                                  0x10                                                        // RWA, sleep ADC clock
#define RB_SLP_CLK_GPIO                                 0x20                                                        // RWA, sleep GPIO clock
#define RB_SLP_CLK_USB                                  0x40                                                        // RWA, sleep USB clock
#define RB_SLP_CLK_ETH                                  0x80                                                        // RWA, sleep Ethernet clock
#define R8_SLP_WAKE_CTRL                                (*((volatile uint8_t *)0x4040000E))                         // RWA, wake control, SAM
#define RB_SLP_PA_WAKE                                  0x01                                                        // RWA，enable PA waking
#define RB_SLP_PB_WAKE                                  0x02                                                        // RWA，enable PB waking
#define RB_SLP_USBPD_WAKE                               0x04                                                        // RWA，enable USBPD waking
#define RB_SLP_PD_WAKE                                  0x08                                                        // RWA，enable PD waking
#define RB_SLP_USB_WAKE                                 0x10                                                        // RWA，enable USB waking
#define RB_SLP_WOL_WAKE                                 0x40                                                        // RWA，enable Ethernet WOL waking
#define RB_SLP_ETH_PWR_DN                               0x80                                                        // RWA，Ethernet module power control
#define R8_SLP_CTRL_PLL                                 (*((volatile uint8_t *)0x4040000F))                         // WA, PLL sleep control, SAM and write 0x6A to sleep CPU or write 0x95 to sleep PLL
#define R8_SLP_STATUS                                   (*((volatile uint8_t *)0x4040000F))                         // RO, sleep status
#define RB_SLP_WOL_STATUS                               0x01                                                        // RO, current ethernet WOL status
#define RB_SLP_CLK_UART2                                0x02                                                        // RWA, sleep UART2 clock
#define RB_SLP_CLK_UART3                                0x04                                                        // RWA, sleep UART3 clock
/* System: external bus configuration register */                       
#define R32_EXT_BUS_CFG                                 (*((volatile uint32_t *)0x40400010))                        // RW, external bus configuration
#define R8_XBUS_CONFIG                                  (*((volatile uint8_t *)0x40400010))                         // RW, external bus configuration
#define RB_XBUS_ENABLE                                  0x01                                                        // RWA, external bus enable
#define RB_XBUS_EN_32BIT                                0x02                                                        // RW, enable 16bit or 32bit external bus
#define RB_XBUS_ADDR_OE                                 0x0C                                                        // RWA, bus address output enable: 00=none,01=PA[5:0],10=PA[11:0],11=PA[19:0]
#define R8_XBUS_CYCLE                                   (*((volatile uint8_t *)0x40400012))                         // RW, external bus total cycle (clock number), only low 5 bit
#define R8_XBUS_SETUP_HOLD                              (*((volatile uint8_t *)0x40400013))                         // RW, external bus setup and hold config
#define RB_XBUS_HOLD                                    0x1F                                                        // RW, external bus hold time bit mask (clock number)
#define RB_XBUS_SETUP                                   0x80                                                        // RW, external bus setup time: 0=1 clock, 1=2 clocks

/* System: parallel slave configuration register */                     
#define R32_PARA_SLV_CFG                                (*((volatile uint32_t *)0x40400014))                        // RW, parallel slave configuration
#define R8_SLV_CONFIG                                   (*((volatile uint8_t *)0x40400014))                         // RWA, parallel slave configuration, SAM
#define RB_SLV_ENABLE                                   0x01                                                        // RWA, parallel slave enable
#define RB_SLV_IE_CMD                                   0x02                                                        // RWA, enable interrupt for slave writing command event
#define RB_SLV_IE_WR                                    0x04                                                        // RWA, enable interrupt for slave writing event
#define RB_SLV_IE_RD                                    0x08                                                        // RWA, enable interrupt for slave reading event
#define R8_SLV_DOUT                                     (*((volatile uint8_t *)0x40400016))                         // RW, parallel slave data to output
#define R8_SLV_STATUS                                   (*((volatile uint8_t *)0x40400017))                         // RW, parallel slave status to output, only low 7 bit

/* System: miscell control register */                      
#define R32_MISCELL_CTRL                                (*((volatile uint32_t *)0x4040001C))                        // RW, miscell control
#define R8_MISC_CTRL_ETH                                (*((volatile uint8_t *)0x4040001C))                         // RW, miscell control for ethernet
#define RB_MISC_ETH_RX                                  0x01                                                        // RW, miscell control ethernet rx LED enable
#define RB_MISC_ETH_TX                                  0x02                                                        // RW, miscell control ethernet tx LED enable
#define RB_MISC_ETH_LED                                 0x04                                                        // RW, miscell control ethernet link LED enable
#define RB_MISC_ETH_RST                                 0x80                                                        // RW, miscell control ethernet transceiver reset
#define R8_MISC_CTRL_USB                                (*((volatile uint8_t *)0x4040001D))                         // RW, miscell control for USB
#define RB_MISC_USB_VBUS                                0x01                                                        // RW, miscell control USB OTG VBUS pin enable
#define RB_MISC_USB_ID_EN                               0x02                                                        // RW, miscell control USB OTG ID pin enable
#define RB_MISC_USB_ID_ST                               0x08                                                        // RW, miscell control USB OTG ID setting value
#define R8_MISC_CTRL_OTHER                              (*((volatile uint8_t *)0x4040001E))                         // RW, miscell control for others
#define RB_MISC_PECL_EN                                 0x01                                                        // RW, miscell control PECL input enable

/* Interrupt vector register */                     
#define R32_INT_VEC_CTRL                                (*((volatile uint32_t *)0x40400020))                        // RWA, interrupt vector control, SAM
#define R8_INT_VEC_IRQ                                  (*((volatile uint8_t *)0x40400020))                         // RWA, IRQ normal interrupt vector control, SAM
#define RB_IV_IRQ_TMR0                                  0x01                                                        // RWA, TMR0 IRQ vector enable
#define RB_IV_IRQ_SPI0                                  0x02                                                        // RWA, SPI0 IRQ vector enable
#define RB_IV_IRQ_PB                                    0x04                                                        // RWA, GPIO PB IRQ vector enable
#define R8_INT_VEC_FIQ                                  (*((volatile uint8_t *)0x40400021))                         // RWA, FIQ fast interrupt vector control, SAM
#define RB_IV_FIQ_TMR0                                  0x01                                                        // RWA, TMR0 FIQ vector enable
#define RB_IV_FIQ_SPI0                                  0x02                                                        // RWA, SPI0 FIQ vector enable
#define RB_IV_FIQ_PB                                    0x04                                                        // RWA, GPIO PB FIQ vector enable
#define R32_INT_VEC_TMR0                                (*((volatile uint32_t *)0x40400024))                        // RWA, interrupt vector for TMR0, SAM
#define R32_INT_VEC_SPI0                                (*((volatile uint32_t *)0x40400028))                        // RWA, interrupt vector for SPI0, SAM
#define R32_INT_VEC_PB                                  (*((volatile uint32_t *)0x4040002C))                        // RWA, interrupt vector for GPIO PB, SAM

/* Interrupt flag register */                       
#define R32_INT_FLAG                                    (*((volatile uint32_t *)0x40400040))                        // RW, interrupt flag
#define R8_INT_FLAG_0                                   (*((volatile uint8_t *)0x40400040))                         // RO, interrupt flag byte 0
#define RB_IF_TMR0                                      0x01                                                        // RO, interrupt flag of TMR0
#define RB_IF_TMR1                                      0x02                                                        // RO, interrupt flag of TMR1
#define RB_IF_TMR2                                      0x04                                                        // RO, interrupt flag of TMR2
#define RB_IF_TMR3                                      0x08                                                        // RO, interrupt flag of TMR3
#define RB_IF_SPI0                                      0x10                                                        // RO, interrupt flag of SPI0
#define RB_IF_SPI1                                      0x20                                                        // RO, interrupt flag of SPI1
#define RB_IF_UART0                                     0x40                                                        // RO, interrupt flag of UART0
#define RB_IF_UART1                                     0x80                                                        // RO, interrupt flag of UART1
#define R8_INT_FLAG_1                                   (*((volatile uint8_t *)0x40400041))                         // RO, interrupt flag byte 1
#define RB_IF_PA                                        0x01                                                        // RO, interrupt flag of GPIO PA
#define RB_IF_PB                                        0x02                                                        // RO, interrupt flag of GPIO PB
#define RB_INT_WOL_STATUS                               0x04                                                        // RO, current ethernet WOL status
#define RB_IF_PD                                        0x08                                                        // RO, interrupt flag of GPIO PD
#define RB_IF_ADC                                       0x10                                                        // RO, interrupt flag of ADC
#define RB_IF_SLV                                       0x20                                                        // RO, interrupt flag of parallel slave
#define RB_IF_USB                                       0x40                                                        // RO, interrupt flag of USB
#define RB_IF_ETH                                       0x80                                                        // RO, interrupt flag of ethernet
#define R8_INT_FLAG_SLV                                 (*((volatile uint8_t *)0x40400042))                         // RW1, parallel slave interrupt flag
#define RB_IF_SLV_CMD0                                  0x10                                                        // RO, parallel slave command raw flag
#define RB_IF_SLV_CMD                                   0x20                                                        // RO, parallel slave command synchro flag
#define RB_IF_SLV_WR                                    0x40                                                        // RW1, interrupt flag of parallel slave writing event
#define RB_IF_SLV_RD                                    0x80                                                        // RW1, interrupt flag of parallel slave reading event
#define R8_INT_SLV_DIN                                  (*((volatile uint8_t *)0x40400043))                         // RO, parallel slave data input
/*SLV DMA enable register*/
#define R8_DMA_EN_SLV                                   (*((volatile uint8_t *)0x40400180))                         //SLV DMA enable register
#define RB_DMA_EN_SLV                                   0x01                                                        //SLV DMA mode enable control bit
#define RB_DMA_RESP_IE_SLV                              0x02                                                        //SLV DMA error transmission interrupts enable control bit
#define RB_DMA_END_IE_SLV                               0x04                                                        //SLV DMA error transmission end enable control bit
#define RB_DMA_LOOP_EN_SLV                              0x08                                                        //SLV DMA cyclic address mode control bits
#define RB_DMA_FIFO_OV_IE_SLV                           0x10                                                        //The FIFO full state interrupts the enable bit
#define RB_DMA_OTHER_IE_SLV                             0x20                                                        //when in compatibility mode, Transmission of non-DMA datastate interrupts the enable bit
#define RB_DMA_WR_8BIT                                  0x40                                                        //selection of transfer data length for DMA write operations
/*SLV DMA mode control register*/                   
#define R8_DMA_MODE_CTRL_SLV                            (*((volatile uint8_t *)0x40400181))                         //SLV DMA mode control register
#define CMD0_WR_MODE                                    0x40                                                        //DMA transfer command word 0 read and write type
#define CMD1_WR_MODE                                    0x80                                                        //DMA transfer command word 1 read and write type
/*SLV DMA mode enable register*/                    
#define R8_DMA_MODE_EN_SLV                              (*((volatile uint8_t *)0x40400182))                         //SLV DMA mode enable register
#define CMD0_EN                                         0x01                                                        //DMA transfer command word 0 control bit
#define CMD1_EN                                         0x02                                                        //DMA transfer command word 1 control bit
#define DMA_CMD_MODE1                                   0x08                                                        //DMA transfer command word mode1
/*SLV DMA interrupt flag register*/                 
#define R8_DMA_INT_FLAG_SLV                             (*((volatile uint8_t *)0x40400184))                         //SLV DMA interrupt flag register
#define WR_DMA_END_IF                                   0x01                                                        //DMA receive mode transmission end flag bit
#define WR_DMA_RESP_IF                                  0x02                                                        //DMA receive mode transmission error flag bit
#define RD_DMA_END_IF                                   0x04                                                        //DMA occurrence mode transmission end flag bit
#define RD_DMA_RESP_IF                                  0x08                                                        //DMA occurrence mode transmission error flag bit
#define WR_DMA_OV_IF                                    0x10                                                        //input FIFO full flag
#define WR_OTHER_IF                                     0x20                                                        //data are written to the flag bit
#define DATA_IN_FIFO_EMPTY                              0x40                                                        //Whether DATA_IN_FIFO is empty
/*DMA receive mode start address register*/                 
#define R32_WR_DMA_START_ADDR_SLV                       (*((volatile uint32_t *)0x40400188))                        //DMA receive mode start address register
#define MASK_WR_DMA_START_ADDR_SLV                      0x0003ffff                                                  //DMA receive mode start address
/*DMA receive mode end address register*/                   
#define R32_WR_DMA_END_ADDR_SLV                         (*((volatile uint32_t *)0x4040018c))                        //DMA receive mode end address register
#define MASK_WR_DMA_START_ADDR_SLV                      0x0003ffff                                                  //DMA receive mode end address
/*DMA send mode start address register*/                    
#define R32_RD_DMA_START_ADDR_SLV                       (*((volatile uint32_t *)0x40400194))                        //DMA send mode start address register
#define MASK_RD_DMA_START_ADDR_SLV                      0x0003ffff                                                  //DMA send mode start address
/*DMA send mode end address register*/                  
#define R32_RD_DMA_END_ADDR_SLV                         (*((volatile uint32_t *)0x40400198))                        //DMA send mode end address register
#define MASK_RD_DMA_START_ADDR_SLV                      0x0003ffff                                                  //DMA send mode end address
/*DMA current address register*/                    
#define R32_DMA_END_NOW_SLV                             (*((volatile uint32_t *)0x4040019c))                        //DMA current address register
#define MASK_DMA_NOW_ADDR_SLV                           0x0003ffff                                                  //DMA current address
/*DMA mode command word0 register*/                 
#define R8_DMA_CMD0_SLV                                 (*((volatile uint8_t *)0x404001a0))                         //DMA mode command word0 register
/*DMA mode command word1 register*/                 
#define R8_DMA_CMD1_SLV                                 (*((volatile uint8_t *)0x404001a1))                         //DMA mode command word1 register
/*DMA reset the command word register*/                 
#define R8_SLV_RESET_CMD                                (*((volatile uint8_t *)0x404001a4))                         //DMA reset the command word register
/*SLV busy flag register*/                  
#define R8_SLV_BUSY                                     (*((volatile uint8_t *)0x404001a5))                         //SLV busy flag register
#define RB_SLV_BUSY                                     0x01                                                        //SLV busy flag
/*Non-DMA writes to the data register*/                 
#define R8_OTHER_DATA                                   (*((volatile uint8_t *)0x404001b0))                         //Non-DMA writes to the data register
/*DMA parses length register*/                  
#define R16_DMA_DEC_LEN                                 (*((volatile uint16_t *)0x404001b4))                        //DMA parses length register
/*DMA resolves the offset address register*/                    
#define R16_DMA_DEC_OFFSET                              (*((volatile uint16_t *)0x404001b6))                        //DMA resolves the offset address register


/* GPIO PA interrupt register */
#define R32_INT_STATUS_PA                               (*((volatile uint32_t *)0x40400050))                        // RW1, GPIO PA interrupt flag
#define R8_INT_STATUS_PA_1                              (*((volatile uint8_t *)0x40400051))                         // RW1, GPIO PA interrupt flag byte 1
#define R8_INT_STATUS_PA_2                              (*((volatile uint8_t *)0x40400052))                         // RW1, GPIO PA interrupt flag byte 2
#define R32_INT_ENABLE_PA                               (*((volatile uint32_t *)0x40400054))                        // RW, GPIO PA interrupt enable
#define R8_INT_ENABLE_PA_1                              (*((volatile uint8_t *)0x40400055))                         // RW, GPIO PA interrupt enable byte 1
#define R8_INT_ENABLE_PA_2                              (*((volatile uint8_t *)0x40400056))                         // RW, GPIO PA interrupt enable byte 2
#define R32_INT_MODE_PA                                 (*((volatile uint32_t *)0x40400058))                        // RW, GPIO PA interrupt mode: 0=level action, 1=edge action
#define R8_INT_MODE_PA_1                                (*((volatile uint8_t *)0x40400059))                         // RW, GPIO PA interrupt mode byte 1
#define R8_INT_MODE_PA_2                                (*((volatile uint8_t *)0x4040005A))                         // RW, GPIO PA interrupt mode byte 2
#define R32_INT_POLAR_PA                                (*((volatile uint32_t *)0x4040005C))                        // RW, GPIO PA interrupt polarity: 0=normal/low level/fall edge, 1=invert/high level/rise edge
#define R8_INT_POLAR_PA_1                               (*((volatile uint8_t *)0x4040005D))                         // RW, GPIO PA interrupt polarity byte 1
#define R8_INT_POLAR_PA_2                               (*((volatile uint8_t *)0x4040005E))                         // RW, GPIO PA interrupt polarity byte 2

/* GPIO PB interrupt register */                        
#define R32_INT_STATUS_PB                               (*((volatile uint32_t *)0x40400060))                        // RW1, GPIO PB interrupt flag
#define R8_INT_STATUS_PB_0                              (*((volatile uint8_t *)0x40400060))                         // RW1, GPIO PB interrupt flag byte 0
#define R8_INT_STATUS_PB_1                              (*((volatile uint8_t *)0x40400061))                         // RW1, GPIO PB interrupt flag byte 1
#define R8_INT_STATUS_PB_2                              (*((volatile uint8_t *)0x40400062))                         // RW1, GPIO PB interrupt flag byte 2
#define R32_INT_ENABLE_PB                               (*((volatile uint32_t *)0x40400064))                        // RW, GPIO PB interrupt enable
#define R8_INT_ENABLE_PB_0                              (*((volatile uint8_t *)0x40400064))                         // RW, GPIO PB interrupt enable byte 0
#define R8_INT_ENABLE_PB_1                              (*((volatile uint8_t *)0x40400065))                         // RW, GPIO PB interrupt enable byte 1
#define R8_INT_ENABLE_PB_2                              (*((volatile uint8_t *)0x40400066))                         // RW, GPIO PB interrupt enable byte 2
#define R32_INT_MODE_PB                                 (*((volatile uint32_t *)0x40400068))                        // RW, GPIO PB interrupt mode: 0=level action, 1=edge action
#define R8_INT_MODE_PB_0                                (*((volatile uint8_t *)0x40400068))                         // RW, GPIO PB interrupt mode byte 0
#define R8_INT_MODE_PB_1                                (*((volatile uint8_t *)0x40400069))                         // RW, GPIO PB interrupt mode byte 1
#define R8_INT_MODE_PB_2                                (*((volatile uint8_t *)0x4040006A))                         // RW, GPIO PB interrupt mode byte 2
#define R32_INT_POLAR_PB                                (*((volatile uint32_t *)0x4040006C))                        // RW, GPIO PB interrupt polarity: 0=normal/low level/fall edge, 1=invert/high level/rise edge
#define R8_INT_POLAR_PB_0                               (*((volatile uint8_t *)0x4040006C))                         // RW, GPIO PB interrupt polarity byte 0
#define R8_INT_POLAR_PB_1                               (*((volatile uint8_t *)0x4040006D))                         // RW, GPIO PB interrupt polarity byte 1
#define R8_INT_POLAR_PB_2                               (*((volatile uint8_t *)0x4040006E))                         // RW, GPIO PB interrupt polarity byte 2

/* GPIO PD interrupt register */                        
#define R32_INT_STATUS_PD                               (*((volatile uint32_t *)0x40400070))                        // RW1, GPIO PD interrupt flag
#define R8_INT_STATUS_PD_0                              (*((volatile uint8_t *)0x40400070))                         // RW1, GPIO PD interrupt flag byte 0
#define R8_INT_STATUS_PD_3                              (*((volatile uint8_t *)0x40400073))                         // RW1, GPIO PD interrupt flag byte 3
#define R32_INT_ENABLE_PD                               (*((volatile uint32_t *)0x40400074))                        // RW, GPIO PD interrupt enable
#define R8_INT_ENABLE_PD_0                              (*((volatile uint8_t *)0x40400074))                         // RW, GPIO PD interrupt enable byte 0
#define R8_INT_ENABLE_PD_3                              (*((volatile uint8_t *)0x40400077))                         // RW, GPIO PD interrupt enable byte 3
#define R32_INT_MODE_PD                                 (*((volatile uint32_t *)0x40400078))                        // RW, GPIO PD interrupt mode: 0=level action, 1=edge action
#define R8_INT_MODE_PD_0                                (*((volatile uint8_t *)0x40400078))                         // RW, GPIO PD interrupt mode byte 0
#define R8_INT_MODE_PD_3                                (*((volatile uint8_t *)0x4040007B))                         // RW, GPIO PD interrupt mode byte 3
#define R32_INT_POLAR_PD                                (*((volatile uint32_t *)0x4040007C))                        // RW, GPIO PD interrupt polarity: 0=normal/low level/fall edge, 1=invert/high level/rise edge
#define R8_INT_POLAR_PD_0                               (*((volatile uint8_t *)0x4040007C))                         // RW, GPIO PD interrupt polarity byte 0
#define R8_INT_POLAR_PD_3                               (*((volatile uint8_t *)0x4040007F))                         // RW, GPIO PD interrupt polarity byte 3

/* GPIO interrupt register address offset and bit define */                     
#define BA_INT_PA                                       ((volatile uint8_t *)0x40400050)                            // point GPIO PA interrupt base address
#define BA_INT_PB                                       ((volatile uint8_t *)0x40400060)                            // point GPIO PB interrupt base address
#define BA_INT_PD                                       ((volatile uint8_t *)0x40400070)                            // point GPIO PD interrupt base address
#define INT_GPIO_STATUS                                 0x00
#define INT_GPIO_STATUS_0                               0x00
#define INT_GPIO_STATUS_1                               0x01
#define INT_GPIO_STATUS_2                               0x02
#define INT_GPIO_ENABLE                                 0x04
#define INT_GPIO_ENABLE_0                               0x04
#define INT_GPIO_ENABLE_1                               0x05
#define INT_GPIO_ENABLE_2                               0x06
#define INT_GPIO_MODE                                   0x08
#define INT_GPIO_MODE_0                                 0x08
#define INT_GPIO_MODE_1                                 0x09
#define INT_GPIO_MODE_2                                 0x0A
#define INT_GPIO_POLAR                                  0x0C
#define INT_GPIO_POLAR_0                                0x0C
#define INT_GPIO_POLAR_1                                0x0D
#define INT_GPIO_POLAR_2                                0x0E

/* GPIO PA register */
#define R32_PA_DIR                                      (*((volatile uint32_t *)0x40400080))                        // RW, GPIO PA I/O direction: 0=in, 1=out
#define R8_PA_DIR_0                                     (*((volatile uint8_t *)0x40400080))                         // RW, GPIO PA I/O direction byte 0
#define R8_PA_DIR_1                                     (*((volatile uint8_t *)0x40400081))                         // RW, GPIO PA I/O direction byte 1
#define R8_PA_DIR_2                                     (*((volatile uint8_t *)0x40400082))                         // RW, GPIO PA I/O direction byte 2
#define R32_PA_PIN                                      (*((volatile uint32_t *)0x40400084))                        // RO, GPIO PA input
#define R8_PA_PIN_0                                     (*((volatile uint8_t *)0x40400084))                         // RO, GPIO PA input byte 0
#define R8_PA_PIN_1                                     (*((volatile uint8_t *)0x40400085))                         // RO, GPIO PA input byte 1
#define R8_PA_PIN_2                                     (*((volatile uint8_t *)0x40400086))                         // RO, GPIO PA input byte 2
#define R32_PA_OUT                                      (*((volatile uint32_t *)0x40400088))                        // RW, GPIO PA output
#define R8_PA_OUT_0                                     (*((volatile uint8_t *)0x40400088))                         // RW, GPIO PA output byte 0
#define R8_PA_OUT_1                                     (*((volatile uint8_t *)0x40400089))                         // RW, GPIO PA output byte 1
#define R8_PA_OUT_2                                     (*((volatile uint8_t *)0x4040008A))                         // RW, GPIO PA output byte 2
#define R32_PA_CLR                                      (*((volatile uint32_t *)0x4040008C))                        // WZ, GPIO PA clear output: 0=keep, 1=clear
#define R8_PA_CLR_0                                     (*((volatile uint8_t *)0x4040008C))                         // WZ, GPIO PA clear output byte 0
#define R8_PA_CLR_1                                     (*((volatile uint8_t *)0x4040008D))                         // WZ, GPIO PA clear output byte 1
#define R8_PA_CLR_2                                     (*((volatile uint8_t *)0x4040008E))                         // WZ, GPIO PA clear output byte 2
#define R32_PA_PU                                       (*((volatile uint32_t *)0x40400090))                        // RW, GPIO PA pullup resistance enable
#define R8_PA_PU_0                                      (*((volatile uint8_t *)0x40400090))                         // RW, GPIO PA pullup resistance enable byte 0
#define R8_PA_PU_1                                      (*((volatile uint8_t *)0x40400091))                         // RW, GPIO PA pullup resistance enable byte 1
#define R8_PA_PU_2                                      (*((volatile uint8_t *)0x40400092))                         // RW, GPIO PA pullup resistance enable byte 2
#define R32_PA_PD                                       (*((volatile uint32_t *)0x40400094))                        // RW, GPIO PA output open-drain & input pulldown resistance enable
#define R8_PA_PD_0                                      (*((volatile uint8_t *)0x40400094))                         // RW, GPIO PA output open-drain & input pulldown resistance enable byte 0
#define R8_PA_PD_1                                      (*((volatile uint8_t *)0x40400095))                         // RW, GPIO PA output open-drain & input pulldown resistance enable byte 1
#define R8_PA_PD_2                                      (*((volatile uint8_t *)0x40400096))                         // RW, GPIO PA output open-drain & input pulldown resistance enable byte 2

/* GPIO PB register */                      
#define R32_PB_DIR                                      (*((volatile uint32_t *)0x404000A0))                        // RW, GPIO PB I/O direction: 0=in, 1=out
#define R8_PB_DIR_0                                     (*((volatile uint8_t *)0x404000A0))                         // RW, GPIO PB I/O direction byte 0
#define R8_PB_DIR_1                                     (*((volatile uint8_t *)0x404000A1))                         // RW, GPIO PB I/O direction byte 1
#define R8_PB_DIR_2                                     (*((volatile uint8_t *)0x404000A2))                         // RW, GPIO PB I/O direction byte 2
#define R32_PB_PIN                                      (*((volatile uint32_t *)0x404000A4))                        // RO, GPIO PB input
#define R8_PB_PIN_0                                     (*((volatile uint8_t *)0x404000A4))                         // RO, GPIO PB input byte 0
#define R8_PB_PIN_1                                     (*((volatile uint8_t *)0x404000A5))                         // RO, GPIO PB input byte 1
#define R8_PB_PIN_2                                     (*((volatile uint8_t *)0x404000A6))                         // RO, GPIO PB input byte 2
#define R32_PB_OUT                                      (*((volatile uint32_t *)0x404000A8))                        // RW, GPIO PB output
#define R8_PB_OUT_0                                     (*((volatile uint8_t *)0x404000A8))                         // RW, GPIO PB output byte 0
#define R8_PB_OUT_1                                     (*((volatile uint8_t *)0x404000A9))                         // RW, GPIO PB output byte 1
#define R8_PB_OUT_2                                     (*((volatile uint8_t *)0x404000AA))                         // RW, GPIO PB output byte 2
#define R32_PB_CLR                                      (*((volatile uint32_t *)0x404000AC))                        // WZ, GPIO PB clear output: 0=keep, 1=clear
#define R8_PB_CLR_0                                     (*((volatile uint8_t *)0x404000AC))                         // WZ, GPIO PB clear output byte 0
#define R8_PB_CLR_1                                     (*((volatile uint8_t *)0x404000AD))                         // WZ, GPIO PB clear output byte 1
#define R8_PB_CLR_2                                     (*((volatile uint8_t *)0x404000AE))                         // WZ, GPIO PB clear output byte 2
#define R32_PB_PU                                       (*((volatile uint32_t *)0x404000B0))                        // RW, GPIO PB pullup resistance enable
#define R8_PB_PU_0                                      (*((volatile uint8_t *)0x404000B0))                         // RW, GPIO PB pullup resistance enable byte 0
#define R8_PB_PU_1                                      (*((volatile uint8_t *)0x404000B1))                         // RW, GPIO PB pullup resistance enable byte 1
#define R8_PB_PU_2                                      (*((volatile uint8_t *)0x404000B2))                         // RW, GPIO PB pullup resistance enable byte 2
#define R32_PB_PD                                       (*((volatile uint32_t *)0x404000B4))                        // RW, GPIO PB output open-drain & input pulldown resistance enable
#define R8_PB_PD_0                                      (*((volatile uint8_t *)0x404000B4))                         // RW, GPIO PB output open-drain & input pulldown resistance enable byte 0
#define R8_PB_PD_1                                      (*((volatile uint8_t *)0x404000B5))                         // RW, GPIO PB output open-drain & input pulldown resistance enable byte 1
#define R8_PB_PD_2                                      (*((volatile uint8_t *)0x404000B6))                         // RW, GPIO PB output open-drain & input pulldown resistance enable byte 2

/* GPIO PD register */                      
#define R32_PD_DIR                                      (*((volatile uint32_t *)0x404000C0))                        // RW, GPIO PD I/O direction: 0=in, 1=out
#define R8_PD_DIR_0                                     (*((volatile uint8_t *)0x404000C0))                         // RW, GPIO PD I/O direction byte 0
#define R8_PD_DIR_1                                     (*((volatile uint8_t *)0x404000C1))                         // RW, GPIO PD I/O direction byte 1
#define R8_PD_DIR_2                                     (*((volatile uint8_t *)0x404000C2))                         // RW, GPIO PD I/O direction byte 2
#define R8_PD_DIR_3                                     (*((volatile uint8_t *)0x404000C3))                         // RW, GPIO PD I/O direction byte 3
#define R32_PD_PIN                                      (*((volatile uint32_t *)0x404000C4))                        // RO, GPIO PD input
#define R8_PD_PIN_0                                     (*((volatile uint8_t *)0x404000C4))                         // RO, GPIO PD input byte 0
#define R8_PD_PIN_1                                     (*((volatile uint8_t *)0x404000C5))                         // RO, GPIO PD input byte 1
#define R8_PD_PIN_2                                     (*((volatile uint8_t *)0x404000C6))                         // RO, GPIO PD input byte 2
#define R8_PD_PIN_3                                     (*((volatile uint8_t *)0x404000C7))                         // RO, GPIO PD input byte 3
#define R32_PD_OUT                                      (*((volatile uint32_t *)0x404000C8))                        // RW, GPIO PD output
#define R8_PD_OUT_0                                     (*((volatile uint8_t *)0x404000C8))                         // RW, GPIO PD output byte 0
#define R8_PD_OUT_1                                     (*((volatile uint8_t *)0x404000C9))                         // RW, GPIO PD output byte 1
#define R8_PD_OUT_2                                     (*((volatile uint8_t *)0x404000CA))                         // RW, GPIO PD output byte 2
#define R8_PD_OUT_3                                     (*((volatile uint8_t *)0x404000CB))                         // RW, GPIO PD output byte 3
#define R32_PD_PU                                       (*((volatile uint32_t *)0x404000D0))                        // RW, GPIO PD pullup resistance enable
#define R8_PD_PU_0                                      (*((volatile uint8_t *)0x404000D0))                         // RW, GPIO PD pullup resistance enable 0
#define R8_PD_PU_1                                      (*((volatile uint8_t *)0x404000D1))                         // RW, GPIO PD pullup resistance enable 1
#define R8_PD_PU_2                                      (*((volatile uint8_t *)0x404000D2))                         // RW, GPIO PD pullup resistance enable 2
#define R8_PD_PU_3                                      (*((volatile uint8_t *)0x404000D3))                         // RW, GPIO PD pullup resistance enable 3
#define R32_PD_PD                                       (*((volatile uint32_t *)0x404000D4))                        // RW, GPIO PD pulldown resistance enable
#define R8_PD_PD_0                                      (*((volatile uint8_t *)0x404000D4))                         // RW, GPIO PD pulldown resistance enable 0
#define R8_PD_PD_1                                      (*((volatile uint8_t *)0x404000D5))                         // RW, GPIO PD pulldown resistance enable 1
#define R8_PD_PD_2                                      (*((volatile uint8_t *)0x404000D6))                         // RW, GPIO PD pulldown resistance enable 2
#define R8_PD_PD_3                                      (*((volatile uint8_t *)0x404000D7))                         // RW, GPIO PD pulldown resistance enable 3

/* GPIO register address offset and bit define */
#define BA_PA                                           ((volatile uint8_t *)0x40400080)                            // point GPIO PA base address
#define BA_PB                                           ((volatile uint8_t *)0x404000A0)                            // point GPIO PB base address
#define BA_PD                                           ((volatile uint8_t *)0x404000C0)                            // point GPIO PD base address
#define GPIO_DIR                                        0x00
#define GPIO_DIR_0                                      0x00
#define GPIO_DIR_1                                      0x01
#define GPIO_DIR_2                                      0x02
#define GPIO_DIR_3                                      0x03
#define GPIO_PIN                                        0x04
#define GPIO_PIN_0                                      0x04
#define GPIO_PIN_1                                      0x05
#define GPIO_PIN_2                                      0x06
#define GPIO_PIN_3                                      0x07
#define GPIO_OUT                                        0x08
#define GPIO_OUT_0                                      0x08
#define GPIO_OUT_1                                      0x09
#define GPIO_OUT_2                                      0x0A
#define GPIO_OUT_3                                      0x0B
#define GPIO_CLR                                        0x0C
#define GPIO_CLR_0                                      0x0C
#define GPIO_CLR_1                                      0x0D
#define GPIO_CLR_2                                      0x0E
#define GPIO_CLR_3                                      0x0F
#define GPIO_PU                                         0x10
#define GPIO_PU_0                                       0x10
#define GPIO_PU_1                                       0x11
#define GPIO_PU_2                                       0x12
#define GPIO_PU_3                                       0x13
#define GPIO_PD                                         0x14
#define GPIO_PD_0                                       0x14
#define GPIO_PD_1                                       0x15
#define GPIO_PD_2                                       0x16
#define GPIO_PD_3                                       0x17
#define RB_PD_15_8                                      0x01                                                        // RW, GPIO PD[15:8] pullup resistance enable together
#define RB_PD_23_16                                     0x01                                                        // RW, GPIO PD[23:16] pullup resistance enable together
#define RB_PD_31_24                                     0x01                                                        // RW, GPIO PD[31:24] pullup resistance enable together

/*AF remap and debug I/O configuration register (AFIO_PCFR1)*/
#define R32_AFIO_PCFR1                                  (*((volatile uint32_t *)0x40400140))                        //AF remap and debug I/O configuration register (AFIO_PCFR1)
#define MASK_LINK_LED_RM                                0xc0000000                                                  //LINK LED remapping
#define MASK_SLV_RW_RM                                  0x30000000                                                  //SLV remapping
#define MASK_SLV_DATA_RM                                0x0c000000                                                  //SLV data remapping
#define MASK_SLV_ADDR1_RM                               0x03000000                                                  //SLV address1 remapping
#define MASK_SLV_ADDR_RM                                0x00c00000                                                  //SLV command data selection input pin remapping
#define MASK_SLV_CS_RM                                  0x00300000                                                  //SLV piece selection pin remapping
#define MASK_SLV_INTERRUPT_RM                           0x000c0000                                                  //SLV interrupt pin remapping
#define MASK_I2C_RM                                     0x00030000                                                  //I2C remapping
#define MASK_UART2_MODEM_RM                             0x0000c000                                                  //UART2 MODEM remapping
#define MASK_UART1_MODEM_RM                             0x00003000                                                  //UART1 MODEM remapping
#define MASK_UART0_MODEM_RM                             0x00000c00                                                  //UART0 MODEM remapping
#define MASK_UART3_RM                                   0x00000300                                                  //UART3 remapping
#define MASK_UART2_RM                                   0x000000c0                                                  //UART2 remapping
#define MASK_UART1_RM                                   0x00000030                                                  //UART1 remapping
#define MASK_UART0_RM                                   0x0000000c                                                  //UART0 remapping
#define MASK_SPI0_RM                                    0x00000003                                                  //SPI0 remapping
/*AF remap and debug I/O configuration register (AFIO_PCFR2)*/                  
#define R32_AFIO_PCFR2                                  (*((volatile uint32_t *)0x40400160))                        //AF remap and debug I/O configuration register (AFIO_PCFR2)
#define UART3_MODEM                                     0x000c0000                                                  //UART3 MODEM remapping
#define TNOW3_RM                                        0x00030000                                                  //TNOW3 remapping
#define TNOW2_RM                                        0x0000c000                                                  //TNOW2 remapping
#define TNOW1_RM                                        0x00003000                                                  //TNOW1 remapping
#define TNOW0_RM                                        0x00000c00                                                  //TNOW0 remapping
#define SPI1_RM                                         0x00000300                                                  //UART3 remapping
#define BUSY_RM                                         0x00000040                                                  //BUSY remapping
#define TIMER1_RM                                       0x00000020                                                  //TIMER1 remapping
#define TIMER0_RM                                       0x00000010                                                  //TIMER0 remapping
#define RST_RM                                          0x0000000c                                                  //reset pin remapping
#define ACT_LED_RM                                      0x00000003                                                  //active led remapping
                    
/*Configure the extended control register 0*/                   
#define R32_EXTEN_CTLR0                                 (*((volatile uint32_t *)0x40400144))                        //Configure the extended control register 0
#define RB_CORE_PROT_STATUS                             0x40000000                                                  //Core protected moed status bit
#define RB_CORE_HALT_INT_EN                             0x02000000                                                  //Kernel error interrupt enabled
#define RB_RST_DLEAY_EN                                 0x01000000                                                  //reset time extension control bit
#define RB_XI_STATUS                                    0x00800000                                                  //XI STATUS
#define RB_USBPLLON                                     0x00400000                                                  //USBPLL clock enable bit
#define RB_FLASH_PRE_EN                                 0x00200000                                                  //flash clock pre-divisor enable
#define RB_ETHRST                                       0x00080000                                                  //ethernet reset control
#define RB_DIG_ETH_PHY_RST                              0x00020000                                                  //ethernet phy digital module reset control
#define RB_ANA_ETH_PHY_RST                              0x00010000                                                  //ethernet phy analog module reset control
#define RB_USBPLLCLK                                    0x0000c000                                                  //USBPLL input clock frequency selection
#define RB_RST_CMD_EN                                   0x00002000                                                  //global reset enabled
#define RB_BUSY_EN                                      0x00001000                                                  //the busy signal output of spi and slv enabled
#define RB_TNOW3_EN                                     0x00000800                                                  //the TNOW signal output of the UART3 is enabled
#define RB_TNOW2_EN                                     0x00000400                                                  //the TNOW signal output of the UART2 is enabled
#define RB_TNOW1_EN                                     0x00000200                                                  //the TNOW signal output of the UART1 is enabled
#define RB_TNOW0_EN                                     0x00000100                                                  //the TNOW signal output of the UART0 is enabled
#define RB_SW                                           0x00000080                                                  //select the systemitem clock source
#define RB_USBPLLSRC                                    0x00000060                                                  //USBPLL reference clock source
#define RB_SW_CFG                                       0x00000001                                                  //Serial wire JTAG configuration
/*Configure the extended control register 1*/                   
#define R32_EXTEN_CTLR1                                 (*((volatile uint32_t *)0x40400154))                        //Configure the extended control register 1
#define RB_VIO_PWN_INT_EN                               0x00800000                                                  //Enables an outage when the VIO power is down
#define RB_VIO_PWN_RST_EN                               0x00400000                                                  //Enable system reset when the VIO power is down
#define RB_VIO_PWN_IO_EN                                0x00200000                                                  //Enable the IO port of the peripheral to input a high levelwhen the VIO power is down
#define RB_LDO_SLP_EN                                   0x00100000                                                  //Enable the LDO to go into sleep mode when the VIO power isdown
#define RB_CLKSEL                                       0x00080000                                                  //Clock signal selection
#define RB_HSE_STATUS                                   0x00008000                                                  //External crystal current level status
#define RB_VIO_RDY                                      0x00000080                                                  //VIO status
#define RB_HSION                                        0x00000040                                                  //Internal High Speed clock enable
#define RB_HSEON                                        0x00000020                                                  //External High Speed clock enable
/*Configure the extended control register 2*/                   
#define R32_EXTEN_CTLR2                                 (*((volatile uint32_t *)0x4040016c))                        //Configure the extended control register 2
#define RB_XIXO_GPIO_EN                                 0x02000000                                                  //XI/XO pin open-drain_ouput control bit
#define RB_XO_OE                                        0x01000000                                                  //XO ouput enable bit
#define RB_XI_OE                                        0x00800000                                                  //XI ouput enable bit
#define RB_USBPD_IN_HVT1                                0x00000002                                                  //PD PIN PB19 high threshold input mode
#define RB_USBPD_IN_HVT0                                0x00000001                                                  //PD PIN PB18 high threshold input mode
                    
/*flash data manipulation register */                   
#define R8_SPI_FLASH_DATA                               (*((volatile uint8_t *)0x40400018))                         //flash data manipulation register
/*Flash control register*/                  
#define R8_SPI_FLASH_CTRL                               (*((volatile uint8_t *)0x4040001a))                         //Flash control register
#define RB_FLASH_RD_EN                                  0x04                                                        //The software reads the FLASH function enable bit
#define RB_FLASH_OE                                     0x02                                                        //Flash output enable
#define RB_FLASH_CS                                     0x01                                                        //Flash slice bit selection was enabled
/*CodeFlash key buffer register*/                   
#define R16_FLASHA_KEY_BUF                              (*((volatile uint16_t *)0x40400168))                        //CodeFlash key buffer register
#define R16_FLASHB_KEY_BUF                              (*((volatile uint16_t *)0x4040016A))                        //CodeFlash key buffer register

/* GPIO alias name */
#define TWP0                                            (1 << 4)                                                    // PA4
#define TWP1                                            (1 << 5)                                                    // PA5
#define TACK                                            (1 << 6)                                                    // PA6
#define TDO                                             (1 << 7)                                                    // PA7
#define TRST                                            (1 << 9)                                                    // PA9
#define TDI                                             (1 << 13)                                                   // PA13
#define TCK                                             (1 << 14)                                                   // PA14
#define TMS                                             (1 << 15)                                                   // PA15
#define UID                                             (1 << 8)                                                    // PA8
#define SLVI                                            (1 << 9)                                                    // PA9
#define PIN_PARA_A0                                     (1 << 10)                                                   // PA10
#define PIN_PARA_PCS                                    (1 << 11)                                                   // PA11
#define TNOW0                                           (1 << 7)                                                    // PA7
#define DTR0                                            (1 << 7)                                                    // PA7
#define RTS0                                            (1 << 8)                                                    // PA8
#define CTS0                                            (1 << 12)                                                   // PA12
#define DSR0                                            (1 << 13)                                                   // PA13
#define RI0                                             (1 << 14)                                                   // PA14
#define DCD0                                            (1 << 15)                                                   // PA15
#define CTS1                                            (1 << 16)                                                   // PA16
#define RTS1                                            (1 << 17)                                                   // PA17
#define TNOW1                                           (1 << 17)                                                   // PA17
#define ELED                                            (1 << 13)                                                   // PA13
#define ELINK                                           (1 << 18)                                                   // PA18
#define VBUS                                            (1 << 19)                                                   // PA19
#define PRD                                             (1 << 20)                                                   // PA20
#define PIN_PARA_RD                                     (1 << 20)                   
#define PWR_314                                         (1 << 21)                                                   // PA21
#define PIN_PARA_WR                                     (1 << 21)                   
#define CTS0X                                           (1 << 0)                                                    // PB0
#define DSR0X                                           (1 << 1)                                                    // PB1
#define RI0X                                            (1 << 2)                                                    // PB2
#define DCD0X                                           (1 << 3)                                                    // PB3
#define DTR0X                                           (1 << 4)                                                    // PB4
#define RTS0X                                           (1 << 5)                                                    // PB5
#define TRAN0                                           (1 << 0)                                                    // PB0
#define RECV0                                           (1 << 1)                                                    // PB1
#define TRAN1                                           (1 << 2)                                                    // PB2
#define RECV1                                           (1 << 3)                                                    // PB3
#define PWM0                                            (1 << 0)                                                    // PB0
#define CAT0                                            (1 << 1)                                                    // PB1
/* note: PB1/CAT0 will input from PECL if RB_MISC_PECL_EN=1 and TMR0.RB_TMR_OUT_EN=1 */                 
#define PWM1                                            (1 << 2)                                                    // PB2
#define CAT1                                            (1 << 3)                                                    // PB3
#define PWM2                                            (1 << 4)                                                    // PB4
#define CAT2                                            (1 << 5)                                                    // PB5
#define PWM3                                            (1 << 6)                                                    // PB6
#define CAT3                                            (1 << 6)                                                    // PB6
#define RXD0                                            (1 << 8)                                                    // PB8
#define TXD0                                            (1 << 9)                                                    // PB9
#define RXD1                                            (1 << 10)                                                   // PB10
/* note: PB10/RXD1 will input from PECL if RB_MISC_PECL_EN=1 and TMR0.RB_TMR_OUT_EN=0 */                    
#define RXTX1                                           (1 << 10)                                                   // PB10
#define TXD1                                            (1 << 11)                                                   // PB11
                    
#define SCS                                             (1 << 12)                                                   // PB12
#define SCK0                                            (1 << 13)                                                   // PB13
#define MOSI                                            (1 << 14)                                                   // PB14
#define MISO                                            (1 << 15)                                                   // PB15
#define SDX0                                            (1 << 15)                                                   // PB15
#define ADCS                                            (1 << 16)                                                   // PB16
#define SCK1                                            (1 << 17)                                                   // PB17
#define SDO                                             (1 << 18)                                                   // PB18
#define SDI                                             (1 << 19)                                                   // PB19
#define SDX1                                            (1 << 19)                                                   // PB19
#define RXD2                                            (1 << 28)                                                   // PD28
#define TXD2                                            (1 << 29)                                                   // PD29
#define RXD3                                            (1 << 22)                                                   // PD22
#define TXD3                                            (1 << 23)

/* ADC register */
#define R32_ADC_CONTROL                                 (*((volatile uint32_t *)0x4040A000))                        // RW, ADC control
#define R8_ADC_CTRL_MOD                                 (*((volatile uint8_t *)0x4040A000))                         // RW, ADC mode control
#define R8_ADC_CTRL_DMA                                 (*((volatile uint8_t *)0x4040A001))                         // RW, ADC DMA control and etc.
#define R8_ADC_INTER_EN                                 (*((volatile uint8_t *)0x4040A002))                         // RW, ADC interrupt enable
#define R8_ADC_CLOCK_DIV                                (*((volatile uint8_t *)0x4040A003))                         // RW, ADC clock divisor
#define R32_ADC_STATUS                                  (*((volatile uint32_t *)0x4040A004))                        // RW, ADC status
#define R16_ADC_DATA                                    (*((volatile uint16_t *)0x4040A004))                        // RO, ADC result data
#define R8_ADC_INT_FLAG                                 (*((volatile uint8_t *)0x4040A006))                         // RW1, ADC interrupt flag
#define R8_ADC_FIFO_COUNT                               (*((volatile uint8_t *)0x4040A007))                         // RO, ADC FIFO count status
#define R32_ADC_CTRL                                    (*((volatile uint8_t *)0x4040A008))                         // RO, ADC control register
#define R16_ADC_CMP_VALUE                               (*((volatile uint16_t *)0x4040A00C))                        // RW, ADC comparison reference value
#define R16_ADC_FIFO                                    (*((volatile uint16_t *)0x4040A010))                        // RO, ADC FIFO register
#define R32_ADC_DMA_NOW                                 (*((volatile uint32_t *)0x4040A014))                        // RW, ADC DMA current address
#define R32_ADC_DMA_BEG                                 (*((volatile uint32_t *)0x4040A018))                        // RW, ADC DMA begin address
#define R32_ADC_DMA_END                                 (*((volatile uint32_t *)0x4040A01C))                        // RW, ADC DMA end address

/* ADC register address offset and bit define */
#define ADC_FIFO_SIZE                                   8                                                           // ADC FIFO size (depth)
#define BA_ADC                                          ((volatile uint8_t *)0x4040A000)                            // point ADC base address
#define ADC_CTRL_MOD                                    0
#define RB_ADC_CYCLE_CLK                                0x0F                                                        // RW, ADC cycle bit mask (clock number): 0=manual sample, other=set cycle for auto sample
#define RB_ADC_CHAN_MOD                                 0x30                                                        // RW, ADC channel control mode: 00=0#, 01=1#, 10=2#, 11=auto flip 0#/1#
#define RB_ADC_SAMPLE_WID                               0x40                                                        // RW, ADC sample pulse width: 0=1 clock, 1=2 clock
#define RB_ADC_POWER_ON                                 0x80                                                        // RW, ADC module enable
#define ADC_CTRL_DMA                                    1
#define RB_ADC_DMA_ENABLE                               0x01                                                        // RW, ADC DMA enable
#define RB_ADC_DMA_BURST                                0x02                                                        // RW, ADC DMA burst enable
#define RB_ADC_DMA_LOOP                                 0x04                                                        // RW, ADC DMA address loop enable
#define RB_ADC_CHAN_OE                                  0x40                                                        // WO, ADC channel control output enable
#define RB_ADC_MAN_SAMPLE                               0x80                                                        // RW, ADC manual sample control, high action
#define ADC_INTER_EN                                    2
#define RB_ADC_IE_ADC_CMP                               0x01                                                        // RW, enable interrupt for current ADC comparison action
#define RB_ADC_IE_ADC_END                               0x02                                                        // RW, enable interrupt for current ADC end
#define RB_ADC_IE_FIFO_HF                               0x04                                                        // RW, enable interrupt for ADC FIFO half
#define RB_ADC_IE_DMA_END                               0x08                                                        // RW, enable interrupt for ADC DMA completion
#define RB_ADC_IE_FIFO_OV                               0x10                                                        // RW, enable interrupt for ADC FIFO overflow
#define RB_ADC_IE_DMA_ERR                               0x20                                                        // RW, enable interrupt for ADC DMA respond error
#define RB_ADC_CMP_MOD_EQ                               0x40                                                        // RW, ADC equal comparison enable: 0=exclude equal, 1=include equal
#define RB_ADC_CMP_MOD_GT                               0x80                                                        // RW, ADC comparison mode: 0=less action, 1=great action
#define ADC_CLOCK_DIV                                   3
#define ADC_DATA                                        4
#define ADC_INT_FLAG                                    6
#define RB_ADC_IF_ADC_CMP                               0x01                                                        // RW1, interrupt flag for current ADC comparison action
#define RB_ADC_IF_ADC_END                               0x02                                                        // RW1, interrupt flag for current ADC end
#define RB_ADC_IF_FIFO_HF                               0x04                                                        // RW1, interrupt flag for ADC FIFO half
#define RB_ADC_IF_DMA_END                               0x08                                                        // RW1, interrupt flag for ADC DMA completion
#define RB_ADC_IF_FIFO_OV                               0x10                                                        // RW1, interrupt flag for ADC FIFO overflow
#define RB_ADC_IF_DMA_ERR                               0x20                                                        // RW1, interrupt flag for ADC DMA respond error
#define RB_ADC_EOC_FLAG                                 0x40                                                        // RO, current ADC converter end indicator
#define RB_ADC_CHAN_INDEX                               0x80                                                        // RO, current ADC channel number for auto flip: 0=0#/2#, 1=1#
#define ADC_FIFO_COUNT                                  7
#define ADC_CTRL                                        8
#define MASK_ADC_CTL_MOD1                               0x0000000f                                                  //Corresponding channel
#define MASK_ADC_SMAPLE_TIME                            0x01fffff0                                                  //ADC sampling and calibration time
#define MASK_ADC_CYCLE_BIT_4_6                          0x0e000000                                                  //Position 4-6 of the automatic sampling period
#define MASK_ADC_BIT_MODE                               0x10000000                                                  //ADC resolution selection bit
#define ADC_CMP_VALUE                                   0x0C
#define ADC_FIFO                                        0x10
#define MASK_ADC_DMA_ADDR                               0x0003ffff

/* UART0 register */
#define R8_UART0_RBR                                    (*((volatile uint8_t *)0x4040D000))                         // RO, UART0 receiver buffer, receiving byte
#define R8_UART0_THR                                    (*((volatile uint8_t *)0x4040D000))                         // WO, UART0 transmitter holding, transmittal byte
#define R8_UART0_IER                                    (*((volatile uint8_t *)0x4040D001))                         // RW, UART0 interrupt enable
#define R8_UART0_IIR                                    (*((volatile uint8_t *)0x4040D002))                         // RO, UART0 interrupt identification
#define R8_UART0_FCR                                    (*((volatile uint8_t *)0x4040D002))                         // WO, UART0 FIFO control
#define R8_UART0_LCR                                    (*((volatile uint8_t *)0x4040D003))                         // RW, UART0 line control
#define R8_UART0_MCR                                    (*((volatile uint8_t *)0x4040D004))                         // RW, UART0 modem control
#define R8_UART0_LSR                                    (*((volatile uint8_t *)0x4040D005))                         // RO, UART0 line status
#define R8_UART0_MSR                                    (*((volatile uint8_t *)0x4040D006))                         // RO, UART0 modem status
#define R8_UART0_DIV                                    (*((volatile uint8_t *)0x4040D007))                         // RW, UART0 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
#define R8_UART0_DMA_CTRL                               (*((volatile uint8_t *)0x4040d008))                         // RW, DMA Control register
#define R8_UART0_DMA_IF                                 (*((volatile uint8_t *)0x4040d009))                         // RW, DMA status register
#define R32_UART0_DMA_WR_NOW_ADDR                       (*((volatile uint32_t *)0x4040d00c))                        //DMA receive mode current address register
#define R32_UART0_DMA_WR_START_ADDR                     (*((volatile uint32_t *)0x4040d010))                        //DMA receive mode start address register
#define R32_UART0_DMA_WR_END_ADDR                       (*((volatile uint32_t *)0x4040d014))                        //DMA received mode end address register
#define R32_UART0_DMA_RD_NOW_ADDR                       (*((volatile uint32_t *)0x4040d018))                        //DMA send mode current address register
#define R32_UART0_DMA_RD_START_ADDR                     (*((volatile uint32_t *)0x4040d01c))                        //DMA send mode start address register
#define R32_UART0_DMA_RD_END_ADDR                       (*((volatile uint32_t *)0x4040d020))                        //DMA send mode end address register
#define R8_UART0_DLL                                    (*((volatile uint8_t *)0x4040D000))                         // RW, UART0 divisor latch LSB byte
#define R8_UART0_DLM                                    (*((volatile uint8_t *)0x4040D001))                         // RW, UART0 divisor latch MSB byte

/* UART1 register */
#define R8_UART1_RBR                                    (*((volatile uint8_t *)0x4040D800))                         // RO, UART1 receiver buffer, receiving byte
#define R8_UART1_THR                                    (*((volatile uint8_t *)0x4040D800))                         // WO, UART1 transmitter holding, transmittal byte
#define R8_UART1_IER                                    (*((volatile uint8_t *)0x4040D801))                         // RW, UART1 interrupt enable
#define R8_UART1_IIR                                    (*((volatile uint8_t *)0x4040D802))                         // RO, UART1 interrupt identification
#define R8_UART1_FCR                                    (*((volatile uint8_t *)0x4040D802))                         // WO, UART1 FIFO control
#define R8_UART1_LCR                                    (*((volatile uint8_t *)0x4040D803))                         // RW, UART1 line control
#define R8_UART1_MCR                                    (*((volatile uint8_t *)0x4040D804))                         // RW, UART1 modem control
#define R8_UART1_LSR                                    (*((volatile uint8_t *)0x4040D805))                         // RO, UART1 line status
#define R8_UART1_MSR                                    (*((volatile uint8_t *)0x4040D806))                         // RO, UART1 modem status
#define R8_UART1_DIV                                    (*((volatile uint8_t *)0x4040D807))                         // RW, UART1 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
#define R8_UART1_DMA_CTRL                               (*((volatile uint8_t *)0x4040d808))                         // RW, DMA Control register
#define R8_UART1_DMA_IF                                 (*((volatile uint8_t *)0x4040d809))                         // RW, DMA status register
#define R32_UART1_DMA_WR_NOW_ADDR                       (*((volatile uint32_t *)0x4040d80c))                        //DMA receive mode current address register
#define R32_UART1_DMA_WR_START_ADDR                     (*((volatile uint32_t *)0x4040d810))                        //DMA receive mode start address register
#define R32_UART1_DMA_WR_END_ADDR                       (*((volatile uint32_t *)0x4040d814))                        //DMA received mode end address register
#define R32_UART1_DMA_RD_NOW_ADDR                       (*((volatile uint32_t *)0x4040d818))                        //DMA send mode current address register
#define R32_UART1_DMA_RD_START_ADDR                     (*((volatile uint32_t *)0x4040d81c))                        //DMA send mode start address register
#define R32_UART1_DMA_RD_END_ADDR                       (*((volatile uint32_t *)0x4040d820))                        //DMA send mode end address register
#define R8_UART1_DLL                                    (*((volatile uint8_t *)0x4040D800))                         // RW, UART1 divisor latch LSB byte
#define R8_UART1_DLM                                    (*((volatile uint8_t *)0x4040D801))                         // RW, UART1 divisor latch MSB byte

/* UART2 register */
#define R8_UART2_RBR                                    (*((volatile uint8_t *)0x40409000))                         // RO, UART2 receiver buffer, receiving byte
#define R8_UART2_THR                                    (*((volatile uint8_t *)0x40409000))                         // WO, UART2 transmitter holding, transmittal byte
#define R8_UART2_IER                                    (*((volatile uint8_t *)0x40409001))                         // RW, UART2 interrupt enable
#define R8_UART2_IIR                                    (*((volatile uint8_t *)0x40409002))                         // RO, UART2 interrupt identification
#define R8_UART2_FCR                                    (*((volatile uint8_t *)0x40409002))                         // WO, UART2 FIFO control
#define R8_UART2_LCR                                    (*((volatile uint8_t *)0x40409003))                         // RW, UART2 line control
#define R8_UART2_MCR                                    (*((volatile uint8_t *)0x40409004))                         // RW, UART2 modem control
#define R8_UART2_LSR                                    (*((volatile uint8_t *)0x40409005))                         // RO, UART2 line status
#define R8_UART2_MSR                                    (*((volatile uint8_t *)0x40409006))                         // RO, UART2 modem status
#define R8_UART2_DIV                                    (*((volatile uint8_t *)0x40409007))                         // RW, UART2 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
#define R8_UART2_DMA_CTRL                               (*((volatile uint8_t *)0x40409008))                         // RW, DMA Control register
#define R8_UART2_DMA_IF                                 (*((volatile uint8_t *)0x40409009))                         // RW, DMA status register
#define R32_UART2_DMA_WR_NOW_ADDR                       (*((volatile uint32_t *)0x4040900c))                        //DMA receive mode current address register
#define R32_UART2_DMA_WR_START_ADDR                     (*((volatile uint32_t *)0x40409010))                        //DMA receive mode start address register
#define R32_UART2_DMA_WR_END_ADDR                       (*((volatile uint32_t *)0x40409014))                        //DMA received mode end address register
#define R32_UART2_DMA_RD_NOW_ADDR                       (*((volatile uint32_t *)0x40409018))                        //DMA send mode current address register
#define R32_UART2_DMA_RD_START_ADDR                     (*((volatile uint32_t *)0x4040901c))                        //DMA send mode start address register
#define R32_UART2_DMA_RD_END_ADDR                       (*((volatile uint32_t *)0x40409020))                        //DMA send mode end address register
#define R8_UART2_DLL                                    (*((volatile uint8_t *)0x40409000))                         // RW, UART2 divisor latch LSB byte
#define R8_UART2_DLM                                    (*((volatile uint8_t *)0x40409001))                         // RW, UART2 divisor latch MSB byte


/* UART3 register */
#define R8_UART3_RBR                                    (*((volatile uint8_t *)0x40409800))                         // RO, UART3 receiver buffer, receiving byte
#define R8_UART3_THR                                    (*((volatile uint8_t *)0x40409800))                         // WO, UART3 transmitter holding, transmittal byte
#define R8_UART3_IER                                    (*((volatile uint8_t *)0x40409801))                         // RW, UART3 interrupt enable
#define R8_UART3_IIR                                    (*((volatile uint8_t *)0x40409802))                         // RO, UART3 interrupt identification
#define R8_UART3_FCR                                    (*((volatile uint8_t *)0x40409802))                         // WO, UART3 FIFO control
#define R8_UART3_LCR                                    (*((volatile uint8_t *)0x40409803))                         // RW, UART3 line control
#define R8_UART3_MCR                                    (*((volatile uint8_t *)0x40409804))                         // RW, UART3 modem control
#define R8_UART3_LSR                                    (*((volatile uint8_t *)0x40409805))                         // RO, UART3 line status
#define R8_UART3_MSR                                    (*((volatile uint8_t *)0x40409806))                         // RO, UART3 modem status
#define R8_UART3_DIV                                    (*((volatile uint8_t *)0x40409807))                         // RW, UART3 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
#define R8_UART3_DMA_CTRL                               (*((volatile uint8_t *)0x40409808))                         // RW, DMA Control register
#define R8_UART3_DMA_IF                                 (*((volatile uint8_t *)0x40409809))                         // RW, DMA status register
#define R32_UART3_DMA_WR_NOW_ADDR                       (*((volatile uint32_t *)0x4040980c))                        //DMA receive mode current address register
#define R32_UART3_DMA_WR_START_ADDR                     (*((volatile uint32_t *)0x40409810))                        //DMA receive mode start address register
#define R32_UART3_DMA_WR_END_ADDR                       (*((volatile uint32_t *)0x40409814))                        //DMA received mode end address register
#define R32_UART3_DMA_RD_NOW_ADDR                       (*((volatile uint32_t *)0x40409818))                        //DMA send mode current address register
#define R32_UART3_DMA_RD_START_ADDR                     (*((volatile uint32_t *)0x4040981c))                        //DMA send mode start address register
#define R32_UART3_DMA_RD_END_ADDR                       (*((volatile uint32_t *)0x40409820))                        //DMA send mode end address register
#define R8_UART3_DLL                                    (*((volatile uint8_t *)0x40409800))                         // RW, UART3 divisor latch LSB byte
#define R8_UART3_DLM                                    (*((volatile uint8_t *)0x40409801))                         // RW, UART3 divisor latch MSB byte


/* UART register address offset and bit define */
#define UART_FIFO_SIZE                                  16                                                          // UART0 FIFO size (depth)
#define UART_RECV_RDY_SZ                                14                                                          // the max FIFO trigger level for UART0 receiver data available
#define BA_UART0                                        ((volatile uint8_t *)0x4040D000)                            // point UART0 base address
#define BA_UART1                                        ((volatile uint8_t *)0x4040D800)                            // point UART1 base address
#define BA_UART2                                        ((volatile uint8_t *)0x40409000)                            // point UART2 base address
#define BA_UART3                                        ((volatile uint8_t *)0x40409800)                            // point UART3 base address
#define UART_RBR                                        0
#define UART_THR                                        0
#define UART_IER                                        1
#define RB_IER_RECV_RDY                                 0x01                                                        // RW, UART interrupt enable for receiver data ready
#define RB_IER_THR_EMPTY                                0x02                                                        // RW, UART interrupt enable for THR empty
#define RB_IER_LINE_STAT                                0x04                                                        // RW, UART interrupt enable for receiver line status
#define RB_IER_MODEM_CHG                                0x08                                                        // RW, UART interrupt enable for modem status change
#define RB_IER_MODEM_IO                                 0x10                                                        // RW, UART0 modem pin selection: 0=from/to GPIO PA, 1=from/to GPIO PB
#define RB_IER_IRDA_MOD                                 0x10                                                        // RW, UART1 IrDA mode enable
#define RB_IER_MOUT_EN                                  0x20                                                        // RW, UART modem output pin enable
#define RB_IER_TXD_EN                                   0x40                                                        // RW, UART TXD pin enable
#define RB_IER_RESET                                    0x80                                                        // WZ, UART software reset control, high action, auto clear
#define UART_IIR                                        2                                   
#define RB_IIR_NO_INT                                   0x01                                                        // RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
#define RB_IIR_INT_MASK                                 0x0F                                                        // RO, UART interrupt flag bit mask
#define RB_IIR_FIFO_ID                                  0xC0                                                        // RO, UART FIFO enabled flag
#define UART_FCR                                        2                                   
#define RB_FCR_FIFO_EN                                  0x01                                                        // WO, UART FIFO enable
#define RB_FCR_RX_FIFO_CLR                              0x02                                                        // WZ, clear UART receiver FIFO, high action, auto clear
#define RB_FCR_TX_FIFO_CLR                              0x04                                                        // WZ, clear UART transmitter FIFO, high action, auto clear
#define RB_FCR_FIFO_TRIG                                0xC0                                                        // WO, UART0/1 receiver FIFO trigger level: 00-1byte, 01-4/8bytes, 10-8/16bytes, 11-14/28bytes
#define UART_LCR                                        3                                   
#define RB_LCR_WORD_SZ                                  0x03                                                        // RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
#define RB_LCR_STOP_BIT                                 0x04                                                        // RW, UART stop bit length: 0-1bit, 1-2bit
#define RB_LCR_PAR_EN                                   0x08                                                        // RW, UART parity enable
#define RB_LCR_PAR_MOD                                  0x30                                                        // RW, UART pariry mode: 00-odd, 01-even, 10-mark, 11-space
#define RB_LCR_BREAK_EN                                 0x40                                                        // RW, UART break control enable
#define RB_LCR_DLAB                                     0x80                                                        // RW, UART divisor latch access bit
#define UART_MCR                                        4                                   
#define RB_MCR_DTR                                      0x01                                                        // RW, UART control DTR
#define RB_MCR_RTS                                      0x02                                                        // RW, UART control RTS
#define RB_MCR_OUT1                                     0x04                                                        // RW, UART control OUT1
#define RB_MCR_OUT2                                     0x08                                                        // RW, UART control OUT2
#define RB_MCR_LOOP                                     0x10                                                        // RW, UART0 enable local loop back
#define RB_MCR_RXTX                                     0x10                                                        // RW, UART1 enable RXTX for half-duplex (TXD auto three-state output on RXD pin)
#define RB_MCR_AU_FLOW_EN                               0x20                                                        // RW, UART enable autoflow control
#define RB_MCR_TNOW                                     0x40                                                        // RW, UART enable TNOW output on DTR or RTS pin
#define RB_MCR_HALF                                     0x80                                                        // RW, UART enable half-duplex
#define UART_LSR                                        5                                   
#define RB_LSR_DATA_RDY                                 0x01                                                        // RO, UART receiver fifo data ready status
#define RB_LSR_OVER_ERR                                 0x02                                                        // RZ, UART receiver overrun error
#define RB_LSR_PAR_ERR                                  0x04                                                        // RZ, UART receiver parity error
#define RB_LSR_FRAME_ERR                                0x08                                                        // RZ, UART receiver frame error
#define RB_LSR_BREAK_ERR                                0x10                                                        // RZ, UART receiver break error
#define RB_LSR_TX_FIFO_EMP                              0x20                                                        // RO, UART transmitter fifo empty status
#define RB_LSR_TX_ALL_EMP                               0x40                                                        // RO, UART transmitter all empty status
#define RB_LSR_ERR_RX_FIFO                              0x80                                                        // RO, error in UART receiver fifo
#define UART_MSR                                        6                                   
#define RB_MSR_CTS_CHG                                  0x01                                                        // RZ, UART CTS changed status, high action
#define RB_MSR_DSR_CHG                                  0x02                                                        // RZ, UART DSR changed status, high action
#define RB_MSR_RI_CHG                                   0x04                                                        // RZ, UART RI changed status, high action
#define RB_MSR_DCD_CHG                                  0x08                                                        // RZ, UART DCD changed status, high action
#define RB_MSR_CTS                                      0x10                                                        // RO, UART CTS action status
#define RB_MSR_DSR                                      0x20                                                        // RO, UART DSR action status
#define RB_MSR_RI                                       0x40                                                        // RO, UART RI action status
#define RB_MSR_DCD                                      0x80                                                        // RO, UART DCD action status
#define UART_ADR                                        7                                   
#define UART_DIV                                        7                                   
#define UART0_DMA_CTRL                                  8                                   
#define RB_DMA_WR_EN                                    0x01                                                        //DMA enabled when receiving data
#define RB_DMA_WR_LOOP_EN                               0x02                                                        //DMA loop mode enabled when receiving data
#define RB_DMA_WR_END_EN                                0x04                                                        //the end of DMA interrupt when receiving data is enabled
#define RB_DMA_WR_ERR_EN                                0x08                                                        //DMA data transfer error interrupt when receiving data isenabled
#define RB_DMA_RD_EN                                    0x10                                                        //DMA enabled when sending data
#define RB_DMA_RD_LOOP_EN                               0x20                                                        //DMA loop mode enabled when sending data
#define RB_DMA_RD_END_EN                                0x40                                                        //the end of DMA interrupt when sending data is enabled
#define RB_DMA_RD_ERR_EN                                0x80                                                        //DMA data transfer error interrupt when sending data isenabled
#define UART0_DMA_IF                                    9                                   
#define RB_DMA_IF_TX_ERR                                0x08                                                        //DMA transfer error flag while sending data
#define RB_DMA_IF_TX_END                                0x04                                                        //DMA end flag when data is sended
#define RB_DMA_IF_RX_ERR                                0x02                                                        //DMA transfer error flag while receiving data
#define RB_DMA_IF_RX_END                                0x01                                                        //DMA end flag when data is received
#define UART_DLL                                        0                                   
#define UART_DLM                                        1                                   
#define MASK_UART_DMA_ADDR                              0x0003ffff                                  
                                    
/* UART interrupt identification values for IIR bits 3:0 */                                 
#define UART_II_SLV_ADDR                                0x0E                                                        // RO, UART1 slave address match
#define UART_II_LINE_STAT                               0x06                                                        // RO, UART interrupt by receiver line status
#define UART_II_RECV_RDY                                0x04                                                        // RO, UART interrupt by receiver data available
#define UART_II_RECV_TOUT                               0x0C                                                        // RO, UART interrupt by receiver fifo timeout
#define UART_II_THR_EMPTY                               0x02                                                        // RO, UART interrupt by THR empty
#define UART_II_MODEM_CHG                               0x00                                                        // RO, UART interrupt by modem status change
#define UART_II_NO_INTER                                0x01                                                        // RO, no UART interrupt is pending

/* SPI0 register */
#define R32_SPI0_CONTROL                                (*((volatile uint32_t *)0x4040C000))                        // RW, SPI0 control
#define R8_SPI0_CTRL_MOD                                (*((volatile uint8_t *)0x4040C000))                         // RW, SPI0 mode control
#define R8_SPI0_CTRL_DMA                                (*((volatile uint8_t *)0x4040C001))                         // RW, SPI0 DMA control
#define R8_SPI0_INTER_EN                                (*((volatile uint8_t *)0x4040C002))                         // RW, SPI0 interrupt enable
#define R8_SPI0_CLOCK_DIV                               (*((volatile uint8_t *)0x4040C003))                         // RW, SPI0 master clock divisor
#define R8_SPI0_SLAVE_PRE                               (*((volatile uint8_t *)0x4040C003))                         // RW, SPI0 slave preset value
#define R32_SPI0_STATUS                                 (*((volatile uint32_t *)0x4040C004))                        // RW, SPI0 status
#define R8_SPI0_BUFFER                                  (*((volatile uint8_t *)0x4040C004))                         // RW, SPI0 data buffer
#define R8_SPI0_RUN_FLAG                                (*((volatile uint8_t *)0x4040C005))                         // RO, SPI0 work flag
#define R8_SPI0_INT_FLAG                                (*((volatile uint8_t *)0x4040C006))                         // RW1, SPI0 interrupt flag
#define R8_SPI0_FIFO_COUNT                              (*((volatile uint8_t *)0x4040C007))                         // RO, SPI0 FIFO count status
#define R8_SPI0_RESET_CMD                               (*((volatile uint8_t *)0x4040c008))                         //SPI0 reset command word register
#define R16_SPI0_TOTAL_CNT                              (*((volatile uint16_t *)0x4040C00C))                        // RW, SPI0 total byte count, only low 12 bit
#define R32_SPI0_FIFO                                   (*((volatile uint32_t *)0x4040C010))                        // RW, SPI0 FIFO register
#define R8_SPI0_FIFO                                    (*((volatile uint8_t *)0x4040C010))                         // RO/WO, SPI0 FIFO register
#define R8_SPI0_FIFO_COUNT1                             (*((volatile uint8_t *)0x4040C013))                         // RO, SPI0 FIFO count status
#define R32_SPI0_DMA_NOW                                (*((volatile uint32_t *)0x4040C014))                        // RW, SPI0 DMA current address
#define R32_SPI0_DMA_BEG                                (*((volatile uint32_t *)0x4040C018))                        // RW, SPI0 DMA begin address
#define R32_SPI0_DMA_END                                (*((volatile uint32_t *)0x4040C01C))                        // RW, SPI0 DMA end address

/* SPI1 register */                     
#define R32_SPI1_CONTROL                                (*((volatile uint32_t *)0x4040C800))                        // RW, SPI1 control
#define R8_SPI1_CTRL_MOD                                (*((volatile uint8_t *)0x4040C800))                         // RW, SPI1 mode control
#define R8_SPI1_CTRL_DMA                                (*((volatile uint8_t *)0x4040C801))                         // RW, SPI1 DMA control
#define R8_SPI1_INTER_EN                                (*((volatile uint8_t *)0x4040C802))                         // RW, SPI1 interrupt enable
#define R8_SPI1_CLOCK_DIV                               (*((volatile uint8_t *)0x4040C803))                         // RW, SPI1 master clock divisor
#define R32_SPI1_STATUS                                 (*((volatile uint32_t *)0x4040C804))                        // RW, SPI1 status
#define R8_SPI1_BUFFER                                  (*((volatile uint8_t *)0x4040C804))                         // RW, SPI1 data buffer
#define R8_SPI1_RUN_FLAG                                (*((volatile uint8_t *)0x4040C805))                         // RO, SPI1 work flag
#define R8_SPI1_INT_FLAG                                (*((volatile uint8_t *)0x4040C806))                         // RW1, SPI1 interrupt flag
#define R8_SPI1_FIFO_COUNT                              (*((volatile uint8_t *)0x4040C807))                         // RO, SPI1 FIFO count status
#define R16_SPI1_TOTAL_CNT                              (*((volatile uint16_t *)0x4040C80C))                        // RW, SPI1 total byte count, only low 12 bit
#define R32_SPI1_FIFO                                   (*((volatile uint32_t *)0x4040C810))                        // RW, SPI1 FIFO register
#define R8_SPI1_FIFO                                    (*((volatile uint8_t *)0x4040C810))                         // RO/WO, SPI1 FIFO register
#define R8_SPI1_FIFO_COUNT1                             (*((volatile uint8_t *)0x4040C813))                         // RO, SPI1 FIFO count status

/* SPI register address offset and bit define */
#define SPI0_FIFO_SIZE                                  32                                                          // SPI0 FIFO size (depth)
#define SPI1_FIFO_SIZE                                  16                                                          // SPI1 FIFO size (depth)
#define BA_SPI0                                         ((volatile uint8_t *)0x4040C000)                            // point SPI0 base address
#define BA_SPI1                                         ((volatile uint8_t *)0x4040C800)                            // point SPI1 base address
#define SPI_CTRL_MOD                                    0   
#define RB_SPI_MODE_SLAVE                               0x01                                                        // RW, SPI slave mode: 0=master/host, 1=slave/device
#define RB_SPI_ALL_CLEAR                                0x02                                                        // RW, force clear SPI FIFO and count
#define RB_SPI_2WIRE_MOD                                0x04                                                        // RW, SPI enable 2 wire mode: 0=3wire(SCK,MOSI,MISO), 1=2wire(SCK,MISO=SDX)
#define RB_SPI_MST_SCK_MOD                              0x08                                                        // RW, SPI master clock mode: 0=mode 0, 1=mode 3
#define RB_SPI_SLV_CMD_MOD                              0x08                                                        // RW, SPI slave command mode: 0=byte stream, 1=first byte command
#define RB_SPI_FIFO_DIR                                 0x10                                                        // RW, SPI FIFO direction: 0=out(write @master mode), 1=in(read @master mode)
#define RB_SPI_SCK_OE                                   0x20                                                        // RW, SPI SCK output enable
#define RB_SPI_MOSI_OE                                  0x40                                                        // RW, SPI MOSI output enable
#define RB_SPI1_SDO_OE                                  0x40                                                        // RW, SPI1 SDO output enable
#define RB_SPI_MISO_OE                                  0x80                                                        // RW, SPI MISO output enable
#define RB_SPI1_SDI_OE                                  0x80                                                        // RW, SPI1 SDI output enable, SPI1 enable 2 wire mode: 0=3wire(SCK1,SDO,SDI), 1=2wire(SCK1,SDX1)
#define SPI_CTRL_DMA                                    1                                   
#define RB_SPI_DMA_ENABLE                               0x01                                                        // RW, SPI DMA enable
#define RB_SPI_DMA_BURST                                0x02                                                        // RW, SPI DMA burst enable
#define RB_SPI_DMA_LOOP                                 0x04                                                        // RW, SPI DMA address loop enable
#define RB_SPI_HS_HOST                                  0x80                                                        // RW, High speed host receive mode control bit
#define SPI_INTER_EN                                    2                                   
#define RB_SPI_IE_CNT_END                               0x01                                                        // RW, enable interrupt for SPI total byte count end
#define RB_SPI_IE_BYTE_END                              0x02                                                        // RW, enable interrupt for SPI byte exchanged
#define RB_SPI_IE_FIFO_HF                               0x04                                                        // RW, enable interrupt for SPI FIFO half
#define RB_SPI_IE_DMA_END                               0x08                                                        // RW, enable interrupt for SPI DMA completion
#define RB_SPI_IE_FIFO_OV                               0x10                                                        // RW, enable interrupt for SPI FIFO overflow
#define RB_SPI_IE_DMA_ERR                               0x20                                                        // RW, enable interrupt for SPI DMA respond error
#define RB_SPI_IE_FST_BYTE                              0x80                                                        // RW, enable interrupt for SPI slave mode first byte received
#define SPI_CLOCK_DIV                                   3                                   
#define SPI_SLAVE_PRESET                                3                                   
#define SPI_BUFFER                                      4                                   
#define SPI_RUN_FLAG                                    5                                   
#define RB_SPI_SLV_CMD_ACT                              0x10                                                        // RO, SPI slave command flag
#define RB_SPI_FIFO_READY                               0x20                                                        // RO, SPI FIFO ready status
#define RB_SPI_SLV_CS_LOAD                              0x40                                                        // RO, SPI slave chip-select loading status
#define RB_SPI_SLV_SELECT                               0x80                                                        // RO, SPI slave selection status
#define SPI_INT_FLAG                                    6                                   
#define RB_SPI_IF_CNT_END                               0x01                                                        // RW1, interrupt flag for SPI total byte count end
#define RB_SPI_IF_BYTE_END                              0x02                                                        // RW1, interrupt flag for SPI byte exchanged
#define RB_SPI_IF_FIFO_HF                               0x04                                                        // RW1, interrupt flag for SPI FIFO half
#define RB_SPI_IF_DMA_END                               0x08                                                        // RW1, interrupt flag for SPI DMA completion
#define RB_SPI_IF_FIFO_OV                               0x10                                                        // RW1, interrupt flag for SPI FIFO overflow
#define RB_SPI_IF_DMA_ERR                               0x20                                                        // RW1, interrupt flag for SPI DMA respond error
#define RB_SPI_FREE                                     0x40                                                        // RO, current SPI free status
#define RB_SPI_IF_FST_BYTE                              0x80                                                        // RW1, interrupt flag for SPI slave mode first byte received
#define SPI_FIFO_COUNT                                  7                                   
#define SPI_TOTAL_CNT                                   0x0C                                    
#define SPI_FIFO                                        0x10                                    
#define SPI0_DMA_NOW                                    20                                  
#define MASK_SPI0_DMA_ADDR                              0x0003ffff                                                  //SPI DMA current address

/* Timer0 register */
#define R32_TMR0_CONTROL                                (*((volatile uint32_t *)0x40408000))                        // RW, TMR0 control
#define R8_TMR0_CTRL_MOD                                (*((volatile uint8_t *)0x40408000))                         // RW, TMR0 mode control
#define R8_TMR0_CTRL_DMA                                (*((volatile uint8_t *)0x40408001))                         // RW, TMR0 DMA control
#define R8_TMR0_INTER_EN                                (*((volatile uint8_t *)0x40408002))                         // RW, TMR0 interrupt enable
#define R32_TMR0_STATUS                                 (*((volatile uint32_t *)0x40408004))                        // RW, TMR0 status
#define R8_TMR0_INT_FLAG                                (*((volatile uint8_t *)0x40408006))                         // RW1, TMR0 interrupt flag
#define R8_TMR0_FIFO_COUNT                              (*((volatile uint8_t *)0x40408007))                         // RO, TMR0 FIFO count status
#define R32_TMR0_COUNT                                  (*((volatile uint32_t *)0x40408008))                        // RO, TMR0 current count
#define R16_TMR0_COUNT                                  (*((volatile uint16_t *)0x40408008))                        // RO, TMR0 current count
#define R8_TMR0_COUNT                                   (*((volatile uint8_t *)0x40408008))                         // RO, TMR0 current count
#define R32_TMR0_CNT_END                                (*((volatile uint32_t *)0x4040800C))                        // RW, TMR0 end count value, only low 28 bit
#define R32_TMR0_FIFO                                   (*((volatile uint32_t *)0x40408010))                        // RO/WO, TMR0 FIFO register, only low 28 bit
#define R16_TMR0_FIFO                                   (*((volatile uint16_t *)0x40408010))                        // RO/WO, TMR0 FIFO register
#define R8_TMR0_FIFO                                    (*((volatile uint8_t *)0x40408010))                         // RO/WO, TMR0 FIFO register
#define R32_TMR0_DMA_NOW                                (*((volatile uint32_t *)0x40408014))                        // RW, TMR0 DMA current address
#define R32_TMR0_DMA_BEG                                (*((volatile uint32_t *)0x40408018))                        // RW, TMR0 DMA begin address
#define R32_TMR0_DMA_END                                (*((volatile uint32_t *)0x4040801C))                        // RW, TMR0 DMA end address

/* Timer1 register */                       
#define R32_TMR1_CONTROL                                (*((volatile uint32_t *)0x40408400))                        // RW, TMR1 control
#define R8_TMR1_CTRL_MOD                                (*((volatile uint8_t *)0x40408400))                         // RW, TMR1 mode control
#define R8_TMR1_CTRL_DMA                                (*((volatile uint8_t *)0x40408401))                         // RW, TMR1 DMA control
#define R8_TMR1_INTER_EN                                (*((volatile uint8_t *)0x40408402))                         // RW, TMR1 interrupt enable
#define R8_TMR1_NRZI_CK_DIV                             (*((volatile uint8_t *)0x40408403))                         // RW, TMR1 NRZI clock divisor, only low 4 bit, from 0 to 15
#define R32_TMR1_STATUS                                 (*((volatile uint32_t *)0x40408404))                        // RW, TMR1 status
#define R8_TMR1_NRZI_STATUS                             (*((volatile uint8_t *)0x40408404))                         // RO, TMR1 NRZI status
#define R8_TMR1_INT_FLAG                                (*((volatile uint8_t *)0x40408406))                         // RW1, TMR1 interrupt flag
#define R8_TMR1_FIFO_COUNT                              (*((volatile uint8_t *)0x40408407))                         // RO, TMR1 FIFO count status
#define R32_TMR1_COUNT                                  (*((volatile uint32_t *)0x40408408))                        // RO, TMR1 current count
#define R16_TMR1_COUNT                                  (*((volatile uint16_t *)0x40408408))                        // RO, TMR1 current count
#define R8_TMR1_COUNT                                   (*((volatile uint8_t *)0x40408408))                         // RO, TMR1 current count
#define R32_TMR1_CNT_END                                (*((volatile uint32_t *)0x4040840C))                        // RW, TMR1 end count value, only low 28 bit
#define R32_TMR1_FIFO                                   (*((volatile uint32_t *)0x40408410))                        // RO/WO, TMR1 FIFO register, only low 28 bit
#define R16_TMR1_FIFO                                   (*((volatile uint16_t *)0x40408410))                        // RO/WO, TMR1 FIFO register
#define R8_TMR1_FIFO                                    (*((volatile uint8_t *)0x40408410))                         // RO/WO, TMR1 FIFO register
#define R32_TMR1_DMA_NOW                                (*((volatile uint32_t *)0x40408414))                        // RW, TMR1 DMA current address
#define R32_TMR1_DMA_BEG                                (*((volatile uint32_t *)0x40408418))                        // RW, TMR1 DMA begin address
#define R32_TMR1_DMA_END                                (*((volatile uint32_t *)0x4040841C))                        // RW, TMR1 DMA end address

/* Timer2 register */                       
#define R32_TMR2_CONTROL                                (*((volatile uint32_t *)0x40408800))                        // RW, TMR2 control
#define R8_TMR2_CTRL_MOD                                (*((volatile uint8_t *)0x40408800))                         // RW, TMR2 mode control
#define R8_TMR2_CTRL_DMA                                (*((volatile uint8_t *)0x40408801))                         // RW, TMR2 DMA control
#define R8_TMR2_INTER_EN                                (*((volatile uint8_t *)0x40408802))                         // RW, TMR2 interrupt enable
#define R32_TMR2_STATUS                                 (*((volatile uint32_t *)0x40408804))                        // RW, TMR2 status
#define R8_TMR2_INT_FLAG                                (*((volatile uint8_t *)0x40408806))                         // RW1, TMR2 interrupt flag
#define R8_TMR2_FIFO_COUNT                              (*((volatile uint8_t *)0x40408807))                         // RO, TMR2 FIFO count status
#define R32_TMR2_COUNT                                  (*((volatile uint32_t *)0x40408808))                        // RO, TMR2 current count
#define R16_TMR2_COUNT                                  (*((volatile uint16_t *)0x40408808))                        // RO, TMR2 current count
#define R8_TMR2_COUNT                                   (*((volatile uint8_t *)0x40408808))                         // RO, TMR2 current count
#define R32_TMR2_CNT_END                                (*((volatile uint32_t *)0x4040880C))                        // RW, TMR2 end count value, only low 28 bit
#define R32_TMR2_FIFO                                   (*((volatile uint32_t *)0x40408810))                        // RO/WO, TMR2 FIFO register, only low 28 bit
#define R16_TMR2_FIFO                                   (*((volatile uint16_t *)0x40408810))                        // RO/WO, TMR2 FIFO register
#define R8_TMR2_FIFO                                    (*((volatile uint8_t *)0x40408810))                         // RO/WO, TMR2 FIFO register
#define R32_TMR2_DMA_NOW                                (*((volatile uint32_t *)0x40408814))                        // RW, TMR2 DMA current address
#define R32_TMR2_DMA_BEG                                (*((volatile uint32_t *)0x40408818))                        // RW, TMR2 DMA begin address
#define R32_TMR2_DMA_END                                (*((volatile uint32_t *)0x4040881C))                        // RW, TMR2 DMA end address

/* Timer3 register */                       
#define R32_TMR3_CONTROL                                (*((volatile uint32_t *)0x40408C00))                        // RW, TMR3 control
#define R8_TMR3_CTRL_MOD                                (*((volatile uint8_t *)0x40408C00))                         // RW, TMR3 mode control
#define R8_TMR3_INTER_EN                                (*((volatile uint8_t *)0x40408C02))                         // RW, TMR3 interrupt enable
#define R32_TMR3_STATUS                                 (*((volatile uint32_t *)0x40408C04))                        // RW, TMR3 status
#define R8_TMR3_INT_FLAG                                (*((volatile uint8_t *)0x40408C06))                         // RW1, TMR3 interrupt flag
#define R8_TMR3_FIFO_COUNT                              (*((volatile uint8_t *)0x40408C07))                         // RO, TMR3 FIFO count status
#define R32_TMR3_COUNT                                  (*((volatile uint32_t *)0x40408C08))                        // RO, TMR3 current count
#define R16_TMR3_COUNT                                  (*((volatile uint16_t *)0x40408C08))                        // RO, TMR3 current count
#define R8_TMR3_COUNT                                   (*((volatile uint8_t *)0x40408C08))                         // RO, TMR3 current count
#define R32_TMR3_CNT_END                                (*((volatile uint32_t *)0x40408C0C))                        // RW, TMR3 end count value, only low 28 bit
#define R32_TMR3_FIFO                                   (*((volatile uint32_t *)0x40408C10))                        // RO/WO, TMR3 FIFO register, only low 28 bit
#define R16_TMR3_FIFO                                   (*((volatile uint16_t *)0x40408C10))                        // RO/WO, TMR3 FIFO register
#define R8_TMR3_FIFO                                    (*((volatile uint8_t *)0x40408C10))                         // RO/WO, TMR3 FIFO register

/* Timer register address offset and bit define */
#define TMR_FIFO_SIZE                                   8                                                           // timer FIFO size (depth)
#define BA_TMR0                                         ((volatile uint8_t *)0x40408000)                            // point TMR0 base address
#define BA_TMR1                                         ((volatile uint8_t *)0x40408400)                            // point TMR1 base address
#define BA_TMR2                                         ((volatile uint8_t *)0x40408800)                            // point TMR2 base address
#define BA_TMR3                                         ((volatile uint8_t *)0x40408C00)                            // point TMR3 base address
#define TMR_CTRL_MOD                                    0
#define RB_TMR_MODE_IN                                  0x01                                                        // RW, timer in mode: 0=timer/PWM/count/NRZI encode, 1=catcher/NRZI decode
#define RB_TMR_ALL_CLEAR                                0x02                                                        // RW, force clear timer FIFO and count
#define RB_TMR_COUNT_EN                                 0x04                                                        // RW, timer count enable
#define RB_TMR_OUT_EN                                   0x08                                                        // RW, timer output enable
#define RB_TMR_OUT_POLAR                                0x10                                                        // RW, timer PWM/NRZI encode output polarity: 0=high action, 1=low action
#define RB_TMR_CAT_WIDTH                                0x10                                                        // RW, timer catcher input pulse min width selection: 0=16*clock, 1=8*clock
#define RB_TMR_MODE_NRZI                                0x20                                                        // RW, TMR0/TMR1 NRZI mode: 0=timer/PWM/catcher, 1=NRZI encode/decode
#define RB_TMR3_MODE_COUNT                              0x20                                                        // RW, TMR3 count mode: 0=timer/PWM/catcher/NRZI, 1=count
#define RB_TMR_PWM_REPEAT                               0xC0                                                        // RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16
#define RB_TMR_CATCH_EDGE                               0xC0                                                        // RW, timer catcher edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
#define TMR_CTRL_DMA                                    1
#define RB_TMR_DMA_ENABLE                               0x01                                                        // RW, timer DMA enable
#define RB_TMR_DMA_BURST                                0x02                                                        // RW, timer DMA burst enable
#define RB_TMR_DMA_LOOP                                 0x04                                                        // RW, timer DMA address loop enable
#define TMR_INTER_EN                                    2
#define RB_TMR_IE_CYC_END                               0x01                                                        // RW, enable interrupt for timer catcher count timeout or PWM cycle end
#define RB_TMR_IE_DATA_ACT                              0x02                                                        // RW, enable interrupt for timer catcher input action or PWM trigger or NRZI recv packet end/tran packet end
#define RB_TMR_IE_FIFO_HF                               0x04                                                        // RW, enable interrupt for timer FIFO half
#define RB_TMR_IE_DMA_END                               0x08                                                        // RW, enable interrupt for timer DMA completion
#define RB_TMR_IE_FIFO_OV                               0x10                                                        // RW, enable interrupt for timer FIFO overflow
#define RB_TMR_IE_DMA_ERR                               0x20                                                        // RW, enable interrupt for timer DMA respond error
#define RB_TMR3_FORCE_EN                                0x80                                                        // RW, TMR3 force together timer0/1/2 count enable, independent of RB_TMR_COUNT_EN
#define TMR_NRZI_CK_DIV                                 3
#define TMR_NRZI_STATUS                                 4
#define RB_TMR_RECV_FREE                                0x01                                                        // RO, timer NRZI receiver free status, 0->1 then RB_TMR_IF_DATA_ACT for recv
#define RB_TMR_RECV_ERR                                 0x02                                                        // RO, timer NRZI receiving error status, 0->1 then RB_TMR_IF_NRZI_AUX for recv
#define RB_TMR_TRAN_END                                 0x10                                                        // RO, timer NRZI transmittal end status, 0->1 then RB_TMR_IF_DATA_ACT for tran
#define RB_TMR_TRAN_DOE                                 0x20                                                        // RO, timer NRZI transmitter encode output enable status, 0->1 then RB_TMR_IF_NRZI_AUX for tran
#define TMR_INT_FLAG                                    6
#define RB_TMR_IF_CYC_END                               0x01                                                        // RW1, interrupt flag for timer catcher count timeout or PWM cycle end
#define RB_TMR_IF_DATA_ACT                              0x02                                                        // RW1, interrupt flag for timer catcher input action or PWM trigger or NRZI recv packet end/tran packet end
#define RB_TMR_IF_FIFO_HF                               0x04                                                        // RW1, interrupt flag for timer FIFO half
#define RB_TMR_IF_DMA_END                               0x08                                                        // RW1, interrupt flag for timer DMA completion
#define RB_TMR_IF_FIFO_OV                               0x10                                                        // RW1, interrupt flag for timer FIFO overflow
#define RB_TMR_IF_DMA_ERR                               0x20                                                        // RW1, interrupt flag for timer DMA respond error
#define TMR_FIFO_COUNT                                  7
#define TMR_COUNT                                       0x08
#define TMR_CNT_END                                     0x0C
#define TMR_FIFO                                        0x10
#define TMR_DMA_NOW                                     0x14
#define TMR_DMA_BEG                                     0x18
#define TMR_DMA_END                                     0x1C
#define MASK_TMR_DMA_ADDR                               0x0003ffff



/* Address space define */
#define BA_CODE                                         ((uint32_t *)0x00000000)                                                         // point code base address
#define SZ_CODE                                         0x00100000                                                                    // code size
#define BA_SFR                                          ((uint32_t *)0x40400000)                                                         // point SFR base address
#define SZ_SFR                                          0x00010000                                                                    // SFR size
#ifdef MEM_DATA
#if MEM_DATA == 32
#define BA_RAM                                          ((uint32_t *)0x00818000)                                                         // point RAM base address
#define SZ_RAM                                          0x00008000                                                                    // RAM size
#endif
#if MEM_DATA == 64
#define BA_RAM                                          ((uint32_t *)0x00810000)                                                         // point RAM base address
#define SZ_RAM                                          0x00010000                                                                    // RAM size
#endif
#if MEM_DATA == 96
#define BA_RAM                                          ((uint32_t *)0x00808000)                                                         // point RAM base address
#define SZ_RAM                                          0x00018000                                                                    // RAM size
#endif
#endif
#ifdef SZ_RAM
#else
#define BA_RAM                          ((uint32_t *)(( R8_GLOB_MEM_CFG & RB_GLOB_MEM_CFG ) \
 (( R8_GLOB_MEM_CFG & RB_GLOB_MEM_CFG ) == 0x02 ? 0x20818000 : 0x20810000 )))                                     // point RAM base address
#define SZ_RAM                          ((R8_GLOB_MEM_CFG & RB_GLOB_MEM_CFG)             \
 ((R8_GLOB_MEM_CFG & RB_GLOB_MEM_CFG) == 0x02 ? 0x00008000 : 0x00010000)) // RAM size
#endif
#define BA_XBUS                         ((uint32_t *)0x60C00000)                                                         // point XBUS base address
#define SZ_XBUS                         0x00100000                                                                    // XBUS size

#include "ch564_conf.h"

#define BITS_CFG(REGISTER,bit,en) ((en) == ENABLE ? (REGISTER |= (uint32_t)(bit)) : (REGISTER &= (uint32_t)~(bit)))


#ifdef __cplusplus
}
#endif

#endif
