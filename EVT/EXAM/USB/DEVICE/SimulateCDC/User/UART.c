/********************************** (C) COPYRIGHT *******************************
* File Name          : UART.C
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/05/16
* Description        : uart serial port related initialization and processing
*******************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "UART.h"

/*******************************************************************************/
/* Variable Definition */
/* Global */

/* The following are serial port transmit and receive related variables and buffers */
volatile UART_CTL Uart;

__attribute__ ((aligned(4))) uint8_t  UART2_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ];  /* Serial port 2 transmit data buffer */
__attribute__ ((aligned(4))) uint8_t  UART2_Rx_Buf[ DEF_UARTx_RX_BUF_LEN ];  /* Serial port 2 receive data buffer */
volatile uint32_t UARTx_Rx_DMACurCount;                       /* Serial port 1 receive dma current counter */
volatile uint32_t UARTx_Rx_DMALastCount;                      /* Serial port 1 receive dma last value counter  */

/*********************************************************************
 * @fn      UART2_ParaInit
 *
 * @brief   Uart2 parameters initialization
 *          mode = 0 : Used in usb modify initialization
 *          mode = 1 : Used in default initializations
 * @return  none
 */
void UART2_ParaInit( uint8_t mode )
{
    uint8_t i;

    Uart.Rx_LoadPtr = 0x00;
    Uart.Rx_DealPtr = 0x00;
    Uart.Rx_RemainLen = 0x00;
    Uart.Rx_TimeOut = 0x00;
    Uart.Rx_TimeOutMax = 30;

    Uart.Tx_LoadNum = 0x00;
    Uart.Tx_DealNum = 0x00;
    Uart.Tx_RemainNum = 0x00;
    for( i = 0; i < DEF_UARTx_TX_BUF_NUM_MAX; i++ )
    {
        Uart.Tx_PackLen[ i ] = 0x00;
    }
    Uart.Tx_Flag = 0x00;
    Uart.Tx_CurPackLen = 0x00;
    Uart.Tx_CurPackPtr = 0x00;

    Uart.USB_Up_IngFlag = 0x00;
    Uart.USB_Up_TimeOut = 0x00;
    Uart.USB_Up_Pack0_Flag = 0x00;
    Uart.USB_Down_StopFlag = 0x00;
    UARTx_Rx_DMACurCount = 0x00;
    UARTx_Rx_DMALastCount = 0x00;

    if( mode )
    {
        Uart.Com_Cfg[ 0 ] = (uint8_t)( DEF_UARTx_BAUDRATE );
        Uart.Com_Cfg[ 1 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 8 );
        Uart.Com_Cfg[ 2 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 16 );
        Uart.Com_Cfg[ 3 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 24 );
        Uart.Com_Cfg[ 4 ] = DEF_UARTx_STOPBIT;
        Uart.Com_Cfg[ 5 ] = DEF_UARTx_PARITY;
        Uart.Com_Cfg[ 6 ] = DEF_UARTx_DATABIT;
        Uart.Com_Cfg[ 7 ] = DEF_UARTx_RX_TIMEOUT;
    }
}

/*********************************************************************
 * @fn      UART2_CfgInit
 *
 * @brief   Uart2 configuration
 *          baudrate : baudrate number
 *          stopbits : Stop bit
 *          parity   : parity
 * @return  none
 */
void UART2_CfgInit( uint32_t baudrate, uint8_t stopbits, uint8_t parity )
{
    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);
    UART2_BaudRateCfg(baudrate);
    R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    if(stopbits == 1)
       R8_UART2_LCR |= RB_LCR_STOP_BIT ;
    /* Check digit (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space); */
    if(parity == 1)
        R8_UART2_LCR |= (RB_LCR_PAR_EN) ;
    else if(parity == 2)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x10) ;
    else if(parity == 3)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x20) ;
    else if(parity == 4)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x30) ;

    R8_UART2_IER = RB_IER_TXD_EN;
}

/*********************************************************************
 * @fn      UART2_Init
 *
 * @brief   Uart2 total initialization
 *          mode     : See the useage of UART2_ParaInit( mode )
 *          baudrate : Serial port 2 default baud rate
 *          stopbits : Serial port 2 default stop bits
 *          parity   : Serial port 2 default parity
 *
 * @return  none
 */
void UART2_Init( uint8_t mode, uint32_t baudrate, uint8_t stopbits, uint8_t parity )
{
    GPIOD_ModeCfg(GPIO_Pin_28, GPIO_ModeIN_Floating);
    GPIOD_ModeCfg(GPIO_Pin_29, GPIO_ModeOut_PP);

    R8_UART2_LCR = RB_LCR_DLAB;
    UART2_SET_DLV(1);
    UART2_BaudRateCfg(baudrate);
    R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    ; // FIFO open, trigger point 14 bytes
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    R8_UART2_IER = RB_IER_TXD_EN;
    R8_UART2_MCR = RB_MCR_OUT1;

    if(stopbits == 1)
       R8_UART2_LCR |= RB_LCR_STOP_BIT ;
    /* Check digit (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space); */
    if(parity == 1)
        R8_UART2_LCR |= (RB_LCR_PAR_EN) ;
    else if(parity == 2)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x10) ;
    else if(parity == 3)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x20) ;
    else if(parity == 4)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x30) ;

    UART2_DMACFG(RB_DMA_RD_END_EN | RB_DMA_WR_EN | RB_DMA_WR_LOOP_EN, ENABLE);
    UART2_DMA_SET_WR_RANGE(UART2_Rx_Buf,UART2_Rx_Buf+DEF_UARTx_RX_BUF_LEN);

    UART2_ParaInit( mode );
}

/*********************************************************************
 * @fn      UART2_USB_Init
 *
 * @brief   Uart2 initialization in usb interrupt
 *
 * @return  none
 */
void UART2_USB_Init( void )
{
    uint32_t baudrate;
    uint8_t  stopbits;
    uint8_t  parity;

    baudrate = ( uint32_t )( Uart.Com_Cfg[ 3 ] << 24 ) + ( uint32_t )( Uart.Com_Cfg[ 2 ] << 16 );
    baudrate += ( uint32_t )( Uart.Com_Cfg[ 1 ] << 8 ) + ( uint32_t )( Uart.Com_Cfg[ 0 ] );
    stopbits = Uart.Com_Cfg[ 4 ];
    parity = Uart.Com_Cfg[ 5 ];

    UART2_Init( 0, baudrate, stopbits, parity );

    /* restart usb receive  */
    USBHSD->UEP2_RX_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
    USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
    USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
}

/*********************************************************************
 * @fn      UART2_DataTx_Deal
 *
 * @brief   Uart2 data transmission processing
 *
 * @return  none
 */
void UART2_DataTx_Deal( void )
{
    uint16_t  count;

    /* uart1 transmission processing */
    if( Uart.Tx_Flag )
    {
        /* Query whether the DMA transmission of the serial port is completed */
        if( R8_UART2_DMA_IF & 0x04 )
        {
            R8_UART2_DMA_IF = 0x04;

            Uart.Tx_Flag = 0x00;

            NVIC_DisableIRQ(USBHS_DEV_IRQn);
            NVIC_DisableIRQ(USBHS_DEV_IRQn);

            /* Calculate the variables of last data */
            count = Uart.Tx_CurPackLen - (R32_UART2_DMA_RD_END_ADDR - R32_UART2_DMA_RD_NOW_ADDR);
            Uart.Tx_CurPackLen -= count;
            Uart.Tx_CurPackPtr += count;
            if( Uart.Tx_CurPackLen == 0x00 )
            {
                Uart.Tx_PackLen[ Uart.Tx_DealNum ] = 0x0000;
                Uart.Tx_DealNum++;
                if( Uart.Tx_DealNum >= DEF_UARTx_TX_BUF_NUM_MAX )
                {
                    Uart.Tx_DealNum = 0x00;
                }
                Uart.Tx_RemainNum--;
            }

            /* If the current serial port has suspended the downlink, restart the driver downlink */
            if( ( Uart.USB_Down_StopFlag == 0x01 ) && ( Uart.Tx_RemainNum < 2 ) )
            {
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
                USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
                Uart.USB_Down_StopFlag = 0x00;
            }

            NVIC_EnableIRQ( USBHS_DEV_IRQn );
        }
    }
    else
    {
        /* Load data from the serial port send buffer to send  */
        if( Uart.Tx_RemainNum )
        {
            /* Determine whether to load from the last unsent buffer or from a new buffer */
            if( Uart.Tx_CurPackLen == 0x00 )
            {
                Uart.Tx_CurPackLen = Uart.Tx_PackLen[ Uart.Tx_DealNum ];
                Uart.Tx_CurPackPtr = ( Uart.Tx_DealNum * DEF_USB_HS_PACK_LEN );
            }
            /* Configure DMA and send */
            R8_UART2_DMA_IF = 0x04;                                                         
            R8_UART2_DMA_CTRL &= ~RB_DMA_RD_EN;
            R32_UART2_DMA_RD_START_ADDR = (uint32_t)&UART2_Tx_Buf[ Uart.Tx_CurPackPtr ];     
            R32_UART2_DMA_RD_END_ADDR = (uint32_t)&UART2_Tx_Buf[ Uart.Tx_CurPackPtr + Uart.Tx_CurPackLen ];    
            R8_UART2_DMA_CTRL |= RB_DMA_RD_EN;
            Uart.Tx_Flag = 0x01;
        }
    }
}

/*********************************************************************
 * @fn      UART2_DataRx_Deal
 *
 * @brief   Uart2 data receiving processing
 *
 * @return  none
 */
void UART2_DataRx_Deal( void )
{
    uint16_t temp16;
    uint32_t remain_len;
    uint16_t packlen;

    /* Serial port 1 data DMA receive processing */
    NVIC_DisableIRQ( USBHS_DEV_IRQn );
    NVIC_DisableIRQ( USBHS_DEV_IRQn );
    UARTx_Rx_DMACurCount = (R32_UART2_DMA_WR_END_ADDR - R32_UART2_DMA_WR_NOW_ADDR);
    if( UARTx_Rx_DMALastCount != UARTx_Rx_DMACurCount )
    {
        if( UARTx_Rx_DMALastCount > UARTx_Rx_DMACurCount )
        {
            temp16 = UARTx_Rx_DMALastCount - UARTx_Rx_DMACurCount;
        }
        else
        {
            temp16 = DEF_UARTx_RX_BUF_LEN - UARTx_Rx_DMACurCount;
            temp16 += UARTx_Rx_DMALastCount;
        }
        UARTx_Rx_DMALastCount = UARTx_Rx_DMACurCount;
        if( ( Uart.Rx_RemainLen + temp16 ) > DEF_UARTx_RX_BUF_LEN )
        {
            /* Overflow handling */
            /* Save frame error status */
            printf("U0_O:%08lx\n",(uint32_t)Uart.Rx_RemainLen);
        }
        else
        {
            Uart.Rx_RemainLen += temp16;
        }

        /* Setting reception status */
        Uart.Rx_TimeOut = 0x00;
    }
    NVIC_EnableIRQ( USBHS_DEV_IRQn );

    /*****************************************************************/
    /* Serial port 1 data processing via USB upload and reception */
    if( Uart.Rx_RemainLen )
    {
        if( Uart.USB_Up_IngFlag == 0 )
        {
            /* Calculate the length of this upload */
            remain_len = Uart.Rx_RemainLen;
            packlen = 0x00;
            if( remain_len >= DEF_USBD_HS_PACK_SIZE )
            {
                packlen = DEF_USBD_HS_PACK_SIZE;
            }
            else
            {
                if( Uart.Rx_TimeOut >= Uart.Rx_TimeOutMax )
                {
                    packlen = remain_len;
                }
            }
            if( packlen > ( DEF_UARTx_RX_BUF_LEN - Uart.Rx_DealPtr ) )
            {
                packlen = ( DEF_UARTx_RX_BUF_LEN - Uart.Rx_DealPtr );
            }

            /* Upload serial data via usb */
            if( packlen )
            {
                NVIC_DisableIRQ( USBHS_DEV_IRQn );
                NVIC_DisableIRQ( USBHS_DEV_IRQn );
                Uart.USB_Up_IngFlag = 0x01;
                Uart.USB_Up_TimeOut = 0x00;
                USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t *)&UART2_Rx_Buf[ Uart.Rx_DealPtr ];
                USBHSD->UEP2_TX_LEN  = packlen;
                USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                NVIC_EnableIRQ( USBHS_DEV_IRQn );

                /* Calculate the variables of interest */
                Uart.Rx_RemainLen -= packlen;
                Uart.Rx_DealPtr += packlen;
                if( Uart.Rx_DealPtr >= DEF_UARTx_RX_BUF_LEN )
                {
                    Uart.Rx_DealPtr = 0x00;
                }

                /* Start 0-length packet timeout timer */
                if( packlen == DEF_USBD_HS_PACK_SIZE )
                {
                    Uart.USB_Up_Pack0_Flag = 0x01;
                }
            }
        }
        else
        {
            /* Set the upload success flag directly if the upload is not successful after the timeout */
            if( Uart.USB_Up_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT )
            {
                Uart.USB_Up_IngFlag = 0x00;
                USBHS_Endp_Busy[ DEF_UEP2 ] = 0;
            }
        }
    }

    /*****************************************************************/
    /* Determine if a 0-length packet needs to be uploaded (required for CDC mode) */
    if( Uart.USB_Up_Pack0_Flag )
    {
        if( Uart.USB_Up_IngFlag == 0 )
        {
            if( Uart.USB_Up_TimeOut >= ( DEF_UARTx_RX_TIMEOUT * 20 ) )
            {
                NVIC_DisableIRQ( USBHS_DEV_IRQn );
                NVIC_DisableIRQ( USBHS_DEV_IRQn );
                Uart.USB_Up_IngFlag = 0x01;
                Uart.USB_Up_TimeOut = 0x00;
                USBHSD->UEP2_TX_LEN  = 0x00;
                USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                Uart.USB_Up_IngFlag = 0;
                NVIC_EnableIRQ( USBHS_DEV_IRQn );
                Uart.USB_Up_Pack0_Flag = 0x00;
            }
        }
    }
}
