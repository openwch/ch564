/********************************** (C) COPYRIGHT *******************************
* File Name          : ch564_usbhs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/05/16
* Description        : This file provides all the USBHS firmware functions.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <ch564_usbhs_host.h>
#include "usb_host_config.h"

__attribute__((aligned(4))) uint8_t  USBHS_RX_Buf[ USBHS_MAX_PACKET_SIZE ];     // IN, must even address
__attribute__((aligned(4))) uint8_t  USBHS_TX_Buf[ USBHS_MAX_PACKET_SIZE ];     // OUT, must even address

/* Get Device Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t  SetupGetDevDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00
};

/* Get Configuration Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetCfgDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00
};

/* Get String Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetStrDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_STRING, 0x09, 0x04, 0x04, 0x00
};

/* Set USB Address Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetAddr[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_ADDRESS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Set USB Configuration Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetConfig[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Clear Endpoint STALL Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupClearEndpStall[ ] =
{
    USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*********************************************************************
 * @fn      USBHS_Host_Init
 *
 * @brief   USB host mode initialized.
 *
 * @param   sta - ENABLE or DISABLE
 *
 * @return  none
 */
void USBHS_Host_Init( FunctionalState sta )
{
    if( sta )
    {
        USBHSH->CFG = LINK_RESET | PHY_SUSPENDM ;
        USBHSH->HOST_TX_DMA = (uint32_t)&USBHS_TX_Buf;
        USBHSH->HOST_RX_DMA = (uint32_t)&USBHS_RX_Buf;
        USBHSH->HOST_RX_MAX_LEN  = 512;
        USBHSH->PORT_CFG = PORT_15K_RPD | PORT_HOST_MODE;
        USBHSH->FRAME |= SOF_CNT_EN;
        USBHSH->CFG = ROOT_SOF_EN | DEV_DMA_EN | PHY_SUSPENDM ;
    }
    else
    {
        USBHSH->CFG = LINK_RESET | SIE_RESET | USB_ALL_CLR;
    }
}

/*********************************************************************
 * @fn      USBHSH_CheckRootHubPortEnable
 *
 * @brief   Check the enable status of the USB port.
 *          Note: This bit is automatically cleared when the device is disconnected.
 *
 * @return  The current enable status of the port.
 */
uint8_t USBHSH_CheckRootHubPortEnable( void )
{
    return ( USBHSH->CFG & ROOT_SOF_EN );
}

/*********************************************************************
 * @fn      USBHSH_CheckRootHubPortStatus
 *
 * @brief   Check status of USB port.
 *
 * @para    dev_sta: The status of the root device connected to this port.
 *
 * @return  The current status of the port.
 */
uint8_t USBHSH_CheckRootHubPortStatus( uint8_t status )
{
    /* Detect USB devices plugged or unplugged */
    if( USBHSH->PORT_STATUS_CHG & PORT_CONNECT_IF )
    {
        USBHSH->PORT_STATUS_CHG = PORT_CONNECT_IF;

        if( USBHSH->PORT_STATUS & PORT_CONNECT ) // Detect that the USB device has been connected to the port
        {
            if( ( status == ROOT_DEV_DISCONNECT ) || ( ( status != ROOT_DEV_FAILED ) && ( USBHSH_CheckRootHubPortEnable( ) == 0x00 ) ) )
            {
                return ROOT_DEV_CONNECTED;
            }
            else
            {
                return ROOT_DEV_FAILED;
            }
        }
        else
        {
            return ROOT_DEV_DISCONNECT;
        }
    }
    else
    {
        return ROOT_DEV_FAILED;
    }
}

/*********************************************************************
 * @fn      USBHSH_CheckRootHubPortSpeed
 *
 * @brief   Check the speed of the USB port.
 *
 * @return  The current speed of the port.
 */
uint8_t USBHSH_CheckRootHubPortSpeed( void )
{
    uint8_t speed;

    speed = USBHSH->PORT_STATUS & PORT_SPD_MASK;

    if( speed == PORT_LOW_SPD )
    {
        return USB_LOW_SPEED;
    }
    else if( speed == PORT_FULL_SPD )
    {
        return USB_FULL_SPEED;
    }
    else if( speed == PORT_HIGH_SPD )
    {
        return USB_HIGH_SPEED;
    }

    return USB_SPEED_CHECK_ERR;
}

/*********************************************************************
 * @fn      USBHSH_SetSelfAddr
 *
 * @brief   Set the USB device address.
 *
 * @para    addr: USB device address.
 *
 * @return  none
 */
void USBHSH_SetSelfAddr( uint8_t addr )
{
    USBHSH->DEV_AD = addr & 0x7f;
}

/*********************************************************************
 * @fn      USBHSH_ResetRootHubPort
 *
 * @brief   Reset USB port.
 *
 * @para    mod: Reset host port operating mode.
 *               0 -> reset and wait end
 *               1 -> begin reset
 *               2 -> end reset
 *
 * @return  none
 */
void USBHSH_ResetRootHubPort( uint8_t mode )
{
    USBHSH_SetSelfAddr( 0x00 );

    if( mode <= 1 )
    {
        USBHSH->PORT_CTRL |= SET_PORT_RESET;
    }
    if( mode == 0 )
    {
        Delay_Ms( DEF_BUS_RESET_TIME ); // Reset time from 10mS to 20mS
    }
    if( mode != 1 )
    {
        USBHSH->PORT_CTRL &= ~SET_PORT_RESET;
    }
    Delay_Ms( 2 );

    if( USBHSH->PORT_STATUS_CHG & PORT_CONNECT_IF )
    {
        if( USBHSH->PORT_STATUS & PORT_CONNECT )
        {
            USBHSH->PORT_STATUS_CHG = PORT_CONNECT_IF;
        }
    }
}

/*********************************************************************
 * @fn      USBHSH_EnableRootHubPort
 *
 * @brief   Enable USB host port.
 *
 * @para    *pspeed: USB speed.
 *
 * @return  Operation result of the enabled port.
 */
uint8_t USBHSH_EnableRootHubPort( uint8_t *pspeed )
{
    if( USBHSH->PORT_STATUS & PORT_CONNECT )
    {
        if( USBHSH_CheckRootHubPortEnable( ) == 0x00 )
        {
            *pspeed = USBHSH_CheckRootHubPortSpeed( );
        }
        USBHSH->CFG |= ROOT_SOF_EN;

        return ERR_SUCCESS;
    }

    return ERR_USB_DISCON;
}

/*********************************************************************
 * @fn      USBHSH_Transact
 *
 * @brief   Perform USB transaction.
 *
 * @para    endp_pid: Token PID.
 *          endp_tog: Toggle
 *          timeout: Timeout time.
 *
 * @return  USB transfer result.
 */
uint8_t USBHSH_Transact( uint8_t endp_pid_number, uint16_t endp_tog, uint32_t timeout )
{
    uint8_t   r, trans_retry;
    uint16_t  i;

    uint8_t endp_number = endp_pid_number&0xf0;
    uint8_t endp_pid = endp_pid_number&0x0f;
    if( endp_pid == USB_PID_IN )
    {
        if( (endp_tog & 0x80) || (endp_tog & TOG_DATA1) )
        {
            endp_tog = TOG_DATA1;
        }
        else
        {
            endp_tog = 0;
        }

    }
    else if( endp_pid == USB_PID_OUT )
    {
        if( (endp_tog & 0x40) || (endp_tog & TOG_DATA1) )
        {
            endp_tog = TOG_DATA1;
        }
        else
        {
            endp_tog = 0;
        }
    }
    else
    {
        endp_tog = 0;
    }

    trans_retry = 0;
    do
    {
        USBHSH->CONTROL = HOST_ACTION | endp_pid | endp_tog | endp_number; // in(0x69)

        USBHSH->INT_FG = RTX_ACT_IF;
        for( i = DEF_WAIT_USB_TRANSFER_CNT; ( i != 0 ) && ( ( USBHSH->INT_FG & RTX_ACT_IF ) == 0 ); i-- )
        {
            Delay_Us( 1 );
        }

        USBHSH->CONTROL = ( USBHSH->CONTROL & ~ USB_PID_MASK );

        if( ( USBHSH->INT_FG & RTX_ACT_IF ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }

        if( USBHSH->PORT_STATUS_CHG & PORT_CONNECT_IF )
        {
            USBHSH->PORT_STATUS_CHG = PORT_CONNECT_IF;
            Delay_Us( 200 );
            if( USBHSH->PORT_STATUS & PORT_CONNECT )
            {
                if( USBHSH->CFG & ROOT_SOF_EN )
                {
                    return ERR_USB_CONNECT;
                }
            }
            else
            {
                return ERR_USB_DISCON;
            }
        }
        else if( USBHSH->INT_FG & RTX_ACT_IF ) // The packet transmission was successful
        {
            r = USBHSH->INT_ST & USB_PID_MASK;
            if( endp_pid  == USB_PID_IN )
            {
                if( (r == USB_PID_DATA0 && endp_tog == TOG_DATA0) || (r == USB_PID_DATA1 && endp_tog == TOG_DATA1) )
                {
                    return ERR_SUCCESS; // Packet token match
                }
            }
            else
            {
                if( ( r == USB_PID_ACK ) || ( r == USB_PID_NYET ) )
                {
                    return ERR_SUCCESS;
                }
            }
            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }

            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFFFFFF )
                {
                    timeout--;
                }
                --trans_retry;
            }
            else switch( endp_pid )
            {
                case USB_PID_SETUP:

                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) || ( r == USB_PID_DATA1 ) )
                    {
                        ;
                    }
                    else if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        else
        {
            USBHSH->INT_FG = RB_WKUP_IF | RB_RESUME_ACT_IF | RB_USB_ACT_IF | RB_SOF_ACT_IF | RB_TX_HALT_IF | RB_FIFO_OV_IF;
        }
        Delay_Us( 15 );
    } while( ++trans_retry < 10 );

    return ERR_USB_TRANSFER;
}

/*********************************************************************
 * @fn      USBHSH_CtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @brief   USB host control transfer.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          pbuf: Data buffer
 *          plen: Data length
 *
 * @return  USB control transfer result.
 */
uint8_t USBHSH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    uint16_t rem_len, rx_len, rx_cnt, tx_cnt;

    Delay_Us( 100 );
    if( plen )
    {
        *plen = 0;
    }
    USBHSH->HOST_TX_LEN = sizeof( USB_SETUP_REQ );
    s = USBHSH_Transact( ( USB_PID_SETUP  ) | 0x00, 0x00, DEF_CTRL_TRANS_TIMEOVER_CNT ); // SETUP stage
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    USBHSH->CONTROL = (USBHSH->CONTROL &~ TOG_MASK ) | TOG_DATA1;
    rem_len = pUSBHS_SetupRequest->wLength;

    if( rem_len && pbuf ) //data stage
    {
        if( pUSBHS_SetupRequest->bRequestType & USB_REQ_TYP_IN ) //device to host
        {
            /* Receive data */
            while( rem_len )
            {
                Delay_Us( 100 );
                s = USBHSH_Transact( USB_PID_IN , USBHSH->CONTROL, DEF_CTRL_TRANS_TIMEOVER_CNT );
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                USBHSH->CONTROL ^= TOG_DATA1;

                rx_len = ( USBHSH->RX_LEN < rem_len )? USBHSH->RX_LEN : rem_len;
                rem_len -= rx_len;
                if( plen )
                {
                    *plen += rx_len;
                }
                for( rx_cnt = 0; rx_cnt != rx_len; rx_cnt++ )
                {
                    *pbuf = USBHS_RX_Buf[ rx_cnt ];
                    pbuf++;
                }
                if( ( USBHSH->RX_LEN == 0 ) || ( USBHSH->RX_LEN & ( ep0_size - 1 ) ) )
                {
                    break;
                }
            }
            USBHSH->HOST_TX_LEN = 0;
        }
        else
        {                                                           // host to device
            /* Send data */
            while( rem_len )
            {
                Delay_Us( 100 );
                USBHSH->HOST_TX_LEN = ( rem_len >= ep0_size )? ep0_size : rem_len;
                for( tx_cnt = 0; tx_cnt != USBHSH->HOST_TX_LEN; tx_cnt++ )
                {
                    USBHS_TX_Buf[ tx_cnt ] = *pbuf;
                    pbuf++;
                }
                s = USBHSH_Transact( USB_PID_OUT | 0x00, USBHSH->CONTROL, DEF_CTRL_TRANS_TIMEOVER_CNT );
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                USBHSH->CONTROL ^= TOG_DATA1;

                rem_len -= USBHSH->HOST_TX_LEN;
                if( plen )
                {
                    *plen += USBHSH->HOST_TX_LEN;
                }
            }
        }
    }

    Delay_Us( 100 );
    s = USBHSH_Transact( ( USBHSH->HOST_TX_LEN )? ( USB_PID_IN | 0x00 ) : ( USB_PID_OUT | 0x00 ), TOG_DATA1, DEF_CTRL_TRANS_TIMEOVER_CNT );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    if( USBHSH->HOST_TX_LEN == 0 )
    {
        return ERR_SUCCESS;    //status stage is out, send a zero-length packet.
    }

    if( USBHSH->RX_LEN == 0 )
    {
        return ERR_SUCCESS;    //status stage is in, a zero-length packet is returned indicating success.
    }

    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      USBHSH_GetDeviceDescr
 *
 * @brief   Get the device descriptor of the USB device.
 *
 * @para    pep0_size: Device endpoint 0 size
 *          pbuf: Data buffer
 *
 * @return  The result of getting the device descriptor.
 */
uint8_t USBHSH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    *pep0_size = DEFAULT_ENDP0_SIZE;
    memcpy( pUSBHS_SetupRequest, SetupGetDevDesc, sizeof( USB_SETUP_REQ ) );
    s = USBHSH_CtrlTransfer( *pep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    *pep0_size = ( (PUSB_DEV_DESCR)pbuf )->bMaxPacketSize0;
    if( len < ( (PUSB_SETUP_REQ)SetupGetDevDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBHSH_GetConfigDescr
 *
 * @brief   Get the configuration descriptor of the USB device.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          pbuf: Data buffer
 *          buf_len: Data buffer length
 *          pcfg_len: The length of the device configuration descriptor
 *
 * @return  The result of getting the configuration descriptor.
 */
uint8_t USBHSH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len )
{
    uint8_t  s;

    /* Get the string descriptor of the first 4 bytes */
    memcpy( pUSBHS_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    s = USBHSH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( *pcfg_len < ( (PUSB_SETUP_REQ)SetupGetCfgDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }

    /* Get the complete string descriptor */
    *pcfg_len = ((PUSB_CFG_DESCR)pbuf)->wTotalLength;
    if( *pcfg_len > buf_len )
    {
        *pcfg_len = buf_len;
    }
    memcpy( pUSBHS_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wLength = *pcfg_len;
    s = USBHSH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    return s;
}

/*********************************************************************
 * @fn      USBH_GetStrDescr
 *
 * @brief   Get the string descriptor of the USB device.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          str_num: Index of string descriptor
 *          pbuf: Data buffer
 *
 * @return  The result of getting the string descriptor.
 */
uint8_t USBHSH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    /* Get the string descriptor of the first 4 bytes */
    memcpy( pUSBHS_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    s = USBHSH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    /* Get the complete string descriptor */
    len = pbuf[ 0 ];
    memcpy( pUSBHS_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    pUSBHS_SetupRequest->wLength = len;
    s = USBHSH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBH_SetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          addr: Device address
 *
 * @return  The result of setting device address.
 */
uint8_t USBHSH_SetUsbAddress( uint8_t ep0_size, uint8_t addr )
{
    uint8_t  s;

    memcpy( pUSBHS_SetupRequest, SetupSetAddr, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wValue = (uint16_t)addr;
    s = USBHSH_CtrlTransfer( ep0_size, NULL, NULL );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    USBHSH_SetSelfAddr( addr );
    Delay_Ms( DEF_BUS_RESET_TIME >> 1 ); // Wait for the USB device to complete its operation.
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBH_SetUsbConfig
 *
 * @brief   Set USB configuration.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          cfg: Device configuration value
 *
 * @return  The result of setting device configuration.
 */
uint8_t USBHSH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val )
{
    memcpy( pUSBHS_SetupRequest, SetupSetConfig, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wValue = (uint16_t)cfg_val;
    return USBHSH_CtrlTransfer( ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      USBH_ClearEndpStall
 *
 * @brief   Clear endpoint stall.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          endp_num: Endpoint number.
 *
 * @return  The result of clearing endpoint stall.
 */
uint8_t USBHSH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num )
{
    memcpy( pUSBHS_SetupRequest, SetupClearEndpStall, sizeof( USB_SETUP_REQ ) );
    pUSBHS_SetupRequest->wIndex = (uint16_t)endp_num;
    return ( USBHSH_CtrlTransfer( ep0_size, NULL, NULL ) );
}

/*********************************************************************
 * @fn      USBHSH_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @para    endp_num: Endpoint number
 *          endp_tog: Endpoint toggle
 *          pbuf: Data Buffer
 *          plen: Data length
 *
 * @return  The result of getting data.
 */
uint8_t USBHSH_GetEndpData( uint8_t endp_num, uint16_t *pendp_tog, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;

    s = USBHSH_Transact( USB_PID_IN | (endp_num << 4), *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *plen = USBHSH->RX_LEN;
        memcpy( pbuf, USBHS_RX_Buf, *plen );
        *pendp_tog ^= TOG_DATA1;
    }

    return s;
}

/*********************************************************************
 * @fn      USBHSH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endp_num: Endpoint number
 *          endp_tog: Endpoint toggle
 *          pbuf: Data Buffer
 *          len: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USBHSH_SendEndpData( uint8_t endp_num, uint16_t *pendp_tog, uint8_t *pbuf, uint16_t len )
{
    uint8_t  s;

    memcpy( USBHS_TX_Buf, pbuf, len );
    USBHSH->HOST_TX_LEN = len;
    s = USBHSH_Transact( USB_PID_OUT  | (endp_num << 4), *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *pendp_tog ^= TOG_DATA1;
    }

    return s;
}
