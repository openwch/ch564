/********************************** (C) COPYRIGHT ************* ******************
* File Name          : eth_driver.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/05/05
* Description        : This file contains the headers of the ETH Driver.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __ETH_DRIVER__
#define __ETH_DRIVER__

#ifdef __cplusplus
 extern "C" {
#endif 

#include "debug.h"
#define PHY_ADDRESS                             1

#define ROM_CFG_USERADR_ID                      0X30CC

#define ETH_TXBUFNB                             2
#define ETH_RXBUFNB                             4
#define ETH_RX_BUF_SZE                          ETH_MAX_PACKET_SIZE
#define ETH_TX_BUF_SZE                          ETH_MAX_PACKET_SIZE

extern ETH_DMADESCTypeDef *DMATxDescToSet;
extern ETH_DMADESCTypeDef *DMARxDescToGet;

#define ETH_LED_CTRL    (*((volatile uint32_t *)0x40400158))

void ETH_PHYLink( void );
void WCHNET_ETHIsr(void);
void WCHNET_MainTask( void );
void ETH_Init( uint8_t *macAddr );
uint32_t MACRAW_Tx(uint8_t *buff, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif
