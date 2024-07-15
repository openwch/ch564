/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <debug.h>

static uint8_t p_us = 0;
static uint16_t p_ms = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = (SystemCoreClock / 8) / 1000000;
    p_ms = (uint16_t)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  None
 */
void Delay_Us(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_us;

    SysTick->CMP = i;
    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while ((SysTick->SR & (1 << 0)) != (1 << 0))
        ;
    SysTick->CTLR &= ~(1 << 0);
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  None
 */
void Delay_Ms(uint32_t n)
{
    uint32_t i;

    SysTick->SR &= ~(1 << 0);
    i = (uint32_t)n * p_ms;

    SysTick->CMP = i;
    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 5) | (1 << 0);

    while ((SysTick->SR & (1 << 0)) != (1 << 0))
        ;
    SysTick->CTLR &= ~(1 << 0);
}

/**
 * The function mInitSTDIO initializes the UART communication with a specified baudrate.
 *
 * @param baudrate The "baudrate" parameter is the desired communication speed in bits per second
 * (bps). It determines how fast data is transmitted and received over the serial communication
 * interface.
 */
void USART_Printf_Init(uint32_t baudrate)
{
    uint64_t x, x2;

    x = 10 * SystemCoreClock * 2 / 16 / baudrate; /* 115200bps */
    x2 = x % 10;
    x /= 10;
    if (x2 >= 5)
        x++;

#if DEBUG == DEBUG_UART0
    R8_UART0_LCR = 0x80; /* Set DLAB */
    R8_UART0_DIV = 1;    /* prescaler */
    R8_UART0_DLM = x >> 8;
    R8_UART0_DLL = x & 0xff;

    R8_UART0_LCR = RB_LCR_WORD_SZ; /* Set byte length to 8 */
    R8_UART0_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR |
                   RB_FCR_FIFO_EN; /* Set FIFO trigger point to 14, clear transmit and receive FIFOs, FIFO enable */
    R8_UART0_IER = RB_IER_TXD_EN;  /* TXD enable */
    R32_PB_PD &= ~RXD0;            /* disable pulldown for RXD0, keep pullup */
    R32_PB_DIR |= TXD0;            /* TXD0 output enable */
#elif DEBUG == DEBUG_UART1
    R8_UART0_LCR = 0x80; /* Set DLAB */
    R8_UART1_DIV = 1;    /* prescaler */
    R8_UART1_DLM = x >> 8;
    R8_UART1_DLL = x & 0xff;

    R8_UART1_LCR = RB_LCR_WORD_SZ; /* Set byte length to 8 */
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR |
                   RB_FCR_FIFO_EN; /* Set FIFO trigger point to 14, clear transmit and receive FIFOs, FIFO enable */
    R8_UART1_IER = RB_IER_TXD_EN;  /* TXD enable */
    R32_PB_PD &= ~RXD1;            /* disable pulldown for RXD1, keep pullup */
    R32_PB_DIR |= TXD1;            /* TXD1 output enable */

#elif DEBUG == DEBUG_UART2
    R8_UART2_LCR = 0x80; /* Set DLAB */
    R8_UART2_DIV = 1;    /* prescaler */
    R8_UART2_DLM = x >> 8;
    R8_UART2_DLL = x & 0xff;

    R8_UART2_LCR = RB_LCR_WORD_SZ; /* Set byte length to 8 */
    R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR |
                   RB_FCR_FIFO_EN; /* Set FIFO trigger point to 14, clear transmit and receive FIFOs, FIFO enable */
    R8_UART2_IER = RB_IER_TXD_EN;  /* TXD enable */
    R32_PD_PD &= ~RXD2;            /* disable pulldown for RXD2, keep pullup */
    R32_PD_DIR |= TXD2;            /* TXD2 output enable */
#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length.
 *
 * @return  size - Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size)
{
    int i;

    for (i = 0; i < size; i++)
    {
#if (DEBUG) == DEBUG_UART0
        while ((R8_UART0_LSR & RB_LSR_TX_FIFO_EMP) == 0)
            ;
        /* Waiting for data sending until complated */
        R8_UART0_THR = *(buf++); /* Send data */
#elif (DEBUG) == DEBUG_UART1
        while ((R8_UART1_LSR & RB_LSR_TX_FIFO_EMP) == 0)
            ;                    /* Waiting for data sending until complated */
        R8_UART1_THR = *(buf++); /* Send data */
#elif (DEBUG) == DEBUG_UART2
        while ((R8_UART2_LSR & RB_LSR_TX_FIFO_EMP) == 0)
            ;                    /* Waiting for data sending until complated */
        R8_UART2_THR = *(buf++); /* Send data */
#endif
    }

    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
        return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}

void _fini(){}
void _init(){}
