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

#include "ch564_usbhs_device.h"
#include "iap.h"

/******************************************************************************/
/* Variable Definition */
/* test mode */
volatile uint8_t USBHS_Test_Flag;
__attribute__ ((aligned(4))) uint8_t IFTest_Buf[ 53 ] =
{
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xFE, //26
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF, //37
		0x7F,
		0xBF,
		0xDF,
		0xEF,
		0xF7,
		0xFB,
		0xFD, //44
		0xFC,
		0x7E,
		0xBF,
		0xDF,
		0xEF,
		0xF7,
		0xFB,
		0xFD,
		0x7E
//53
		};

/* Global */
u8 Endp1Busy = 0;
u8 EP1_OUT_Flag = 0;
u8 EP2_OUT_Flag = 0;
u8 Endp3Busy = 0;
u8 Flag_LED = 0;
u8 EP1_Rx_Cnt, EP2_Rx_Cnt;
const uint8_t *pUSBHS_Descr;

/* Setup Request */
volatile uint8_t USBHS_SetupReqCode;
volatile uint8_t USBHS_SetupReqType;
volatile uint16_t USBHS_SetupReqValue;
volatile uint16_t USBHS_SetupReqIndex;
volatile uint16_t USBHS_SetupReqLen;

/* USB Device Status */
volatile uint8_t USBHS_DevConfig;
volatile uint8_t USBHS_DevAddr;
volatile uint16_t USBHS_DevMaxPackLen;
volatile uint8_t USBHS_DevSpeed;
volatile uint8_t USBHS_DevSleepStatus;
volatile uint8_t USBHS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) UINT8 USBHS_EP0_Buf[ USBHS_UEP0_SIZE ];
__attribute__ ((aligned(4))) UINT8 Ep1Buffer[ USBHS_MAX_PACK_SIZE * 2 ];
__attribute__ ((aligned(4))) UINT8 Ep2Buffer[ USBHS_MAX_PACK_SIZE * 2 ];
__attribute__ ((aligned(4))) UINT8 Ep3Buffer[ USBHS_MAX_PACK_SIZE * 2 ];

/* Endpoint tx busy flag */
volatile uint8_t USBHS_Endp_Busy[ DEF_UEP_NUM ];

/* Ring buffer */
RING_BUFF_COMM RingBuffer_Comm;
__attribute__ ((aligned(4))) uint8_t Data_Buffer[ DEF_RING_BUFFER_SIZE ];

void DevEPhs_OUT_Deal(UINT8 l);

/******************************************************************************/
/* Interrupt Service Routine Declaration*/
void USBHS_DEV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USB_TestMode_Deal
 *
 * @brief   Eye Diagram Test Function Processing.
 *
 * @return  none
 *
 */
void USB_TestMode_Deal(void)
{
#if TEST_ENABLE==0x01
	/* start test */
	USBHS_Test_Flag &= ~0x80;

	if ( USBHS_SetupReqIndex == 0x0100 )
	{
		/* Test_J */
		USBHSD->TEST_MODE &= ~TEST_MASK;
		USBHSD->TEST_MODE |= RB_TEST_J;
	}
	else if ( USBHS_SetupReqIndex == 0x0200 )
	{
		/* Test_K */
		USBHSD->TEST_MODE &= ~TEST_MASK;
		USBHSD->TEST_MODE |= RB_TEST_K;
	}
	else if ( USBHS_SetupReqIndex == 0x0300 )
	{
		/* Test_SE0_NAK */
		USBHSD->TEST_MODE &= ~TEST_MASK;
		USBHSD->TEST_MODE |= RB_TEST_SE0NAK;
	}
	else if ( USBHS_SetupReqIndex == 0x0400 )
	{
		/* Test_Packet */
		USBHSD->TEST_MODE &= ~TEST_MASK;

		USBHSD->UEP4_TX_DMA = (uint32_t) ( &IFTest_Buf[ 0 ] );
		USBHSD->UEP4_TX_LEN = 53;
		USBHSD->UEP4_TX_CTRL = USBHS_UEP_R_RES_ACK;
		USBHSD->TEST_MODE |= RB_TEST_PKT;
	}
	USBHSD->TEST_MODE |= RB_TEST_EN;
#endif
}

/*********************************************************************
 * @fn      USBHS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHS_Device_Endp_Init(void)
{
	USBHSD->UEP_TX_EN = RB_EP0_EN | RB_EP1_EN | RB_EP2_EN | RB_EP3_EN | RB_EP4_EN;
	USBHSD->UEP_RX_EN = RB_EP0_EN | RB_EP1_EN | RB_EP2_EN | RB_EP3_EN | RB_EP4_EN;

	USBHSD->UEP0_MAX_LEN = 64;
	USBHSD->UEP1_MAX_LEN = 64;
	USBHSD->UEP2_MAX_LEN = 64;
	USBHSD->UEP3_MAX_LEN = 64;
	USBHSD->UEP4_MAX_LEN = 64;
	USBHSD->UEP5_MAX_LEN = 512;
	USBHSD->UEP6_MAX_LEN = 512;
	USBHSD->UEP7_MAX_LEN = 512;

	USBHSD->UEP0_DMA = (UINT32) (UINT8 *) USBHS_EP0_Buf;
	USBHSD->UEP1_RX_DMA = (UINT32) (UINT8 *) Ep1Buffer;
	USBHSD->UEP1_TX_DMA = (UINT32) (UINT8 *) ( &Ep1Buffer[ 64 ] );
	USBHSD->UEP2_RX_DMA = (UINT32) (UINT8 *) Ep2Buffer;
	USBHSD->UEP2_TX_DMA = (UINT32) (UINT8 *) ( &Ep2Buffer[ 64 ] );
	USBHSD->UEP3_RX_DMA = (UINT32) (UINT8 *) Ep3Buffer;
	USBHSD->UEP3_TX_DMA = (UINT32) (UINT8 *) ( &Ep3Buffer[ 64 ] );

	USBHSD->UEP0_TX_LEN = 0;
	USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

	USBHSD->UEP1_TX_LEN = 0;
	USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_RES_ACK;

	USBHSD->UEP2_TX_LEN = 0;
	USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

	USBHSD->UEP3_TX_LEN = 0;
	USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK;

	USBHSD->UEP4_TX_LEN = 0;
	USBHSD->UEP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP4_RX_CTRL = USBHS_UEP_R_RES_ACK;
}

/*********************************************************************
 * @fn      USBHS_Device_Init
 *
 * @brief   Initializes USB high-speed device.
 *
 * @return  none
 */
void USBHS_Device_Init(FunctionalState sta)
{
	if ( sta )
	{
		USBHSD->CONTROL = LINK_RESET | PHY_SUSPENDM;
		USBHSD->INT_EN = BUS_REST_IE | BUS_SUSP_IE | BUS_SLEEP_IE | LPM_ACT_IE | RTX_ACT_IE | LINK_RDY_IE;
		USBHS_Device_Endp_Init();
		USBHSD->BASE_MODE = EXP_FS_SPD;
		USBHSD->CONTROL = DEV_EN | DEV_DMA_EN | DEV_LPM_EN | PHY_SUSPENDM;
		NVIC_EnableIRQ( USBHS_DEV_IRQn );
	}
	else
	{
		USBHSD->CONTROL |= SIE_RESET;
		USBHSD->CONTROL &= ~SIE_RESET;
		NVIC_DisableIRQ( USBHS_DEV_IRQn );
	}
}

/*********************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   This function handles USBHS exception.
 *
 * @return  none
 */
void USBHS_DEV_IRQHandler(void)
{
	uint8_t intflag, intst, errflag;
	uint16_t len;
	uint8_t endp_num;

	intflag = USBHSD->INT_FG;
	intst = USBHSD->INT_ST;

	if ( intflag & RTX_ACT_IF )
	{
		endp_num = intst & RB_UIS_EP_ID_MASK;
		if ( !( USBHSD->INT_ST & RB_UIS_EP_DIR ) ) // SETUP/OUT Transaction
		{
			switch ( endp_num )
			{
				case DEF_UEP0:
					if ( USBHSD->UEP0_RX_CTRL & USBHS_RB_SETUP_IS )
					{

						/* Store All Setup Values */
						USBHS_SetupReqType = pUSBHS_SetupReqPak->bRequestType;
						USBHS_SetupReqCode = pUSBHS_SetupReqPak->bRequest;
						USBHS_SetupReqLen = pUSBHS_SetupReqPak->wLength;
						USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
						USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;

						len = 0;
						errflag = 0;
						if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
						{
							/* usb non-standard request processing */
							errflag = 0xFF;
						}
						else
						{
							/* usb standard request processing */
							switch ( USBHS_SetupReqCode )
							{
								/* get device/configuration/string/report/... descriptors */
								case USB_GET_DESCRIPTOR:
									switch ( (uint8_t) ( USBHS_SetupReqValue >> 8 ) )
									{
										/* get usb device descriptor */
										case USB_DESCR_TYP_DEVICE:
											pUSBHS_Descr = MyDevDescr;
											len = DEF_USBD_DEVICE_DESC_LEN;
											break;

											/* get usb configuration descriptor */
										case USB_DESCR_TYP_CONFIG:
											/* Query current usb speed */
											if ( USBHSD->BASE_MODE == EXP_HS_SPD )
											{
												/* High speed mode */
												USBHS_DevSpeed = USBHS_SPEED_HIGH;
												USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
											}
											else
											{
												/* Full speed mode */
												USBHS_DevSpeed = USBHS_SPEED_FULL;
												USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
											}

											/* Load usb configuration descriptor by speed */
											if ( USBHS_DevSpeed == USBHS_SPEED_HIGH )
											{
												/* High speed mode */
												pUSBHS_Descr = MyCfgDescr_HS;
												len = DEF_USBD_CONFIG_HS_DESC_LEN;
											}
											else
											{
												/* Full speed mode */
												pUSBHS_Descr = MyCfgDescr_FS;
												len = DEF_USBD_CONFIG_FS_DESC_LEN;
											}
											break;

											/* get usb string descriptor */
										case USB_DESCR_TYP_STRING:
											switch ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) )
											{
												/* Descriptor 0, Language descriptor */
												case DEF_STRING_DESC_LANG:
													pUSBHS_Descr = MyLangDescr;
													len = DEF_USBD_LANG_DESC_LEN;
													break;

													/* Descriptor 1, Manufacturers String descriptor */
												case DEF_STRING_DESC_MANU:
													pUSBHS_Descr = MyManuInfo;
													len = DEF_USBD_MANU_DESC_LEN;
													break;

													/* Descriptor 2, Product String descriptor */
												case DEF_STRING_DESC_PROD:
													pUSBHS_Descr = MyProdInfo;
													len = DEF_USBD_PROD_DESC_LEN;
													break;

													/* Descriptor 3, Serial-number String descriptor */
												case DEF_STRING_DESC_SERN:
													pUSBHS_Descr = MySerNumInfo;
													len = DEF_USBD_SN_DESC_LEN;
													break;

												default:
													errflag = 0xFF;
													break;
											}
											break;

											/* get usb device qualify descriptor */
										case USB_DESCR_TYP_QUALIF:
											pUSBHS_Descr = MyQuaDesc;
											len = DEF_USBD_QUALFY_DESC_LEN;
											break;

											/* get usb BOS descriptor */
										case USB_DESCR_TYP_BOS:
											/* USB 2.00 DO NOT support BOS descriptor */
											errflag = 0xFF;
											break;

											/* get usb other-speed descriptor */
										case USB_DESCR_TYP_SPEED:
											if ( USBHS_DevSpeed == USBHS_SPEED_HIGH )
											{
												/* High speed mode */
												memcpy( &TAB_USB_HS_OSC_DESC[ 2 ] , &MyCfgDescr_FS[ 2 ] ,
														DEF_USBD_CONFIG_FS_DESC_LEN - 2 );
												pUSBHS_Descr = (uint8_t *) &TAB_USB_HS_OSC_DESC[ 0 ];
												len = DEF_USBD_CONFIG_FS_DESC_LEN;
											}
											else if ( USBHS_DevSpeed == USBHS_SPEED_FULL )
											{
												/* Full speed mode */
												memcpy( &TAB_USB_FS_OSC_DESC[ 2 ] , &MyCfgDescr_HS[ 2 ] ,
														DEF_USBD_CONFIG_HS_DESC_LEN - 2 );
												pUSBHS_Descr = (uint8_t *) &TAB_USB_FS_OSC_DESC[ 0 ];
												len = DEF_USBD_CONFIG_HS_DESC_LEN;
											}
											else
											{
												errflag = 0xFF;
											}
											break;

										default:
											errflag = 0xFF;
											break;
									}

									/* Copy Descriptors to Endp0 DMA buffer */
									if ( USBHS_SetupReqLen > len )
									{
										USBHS_SetupReqLen = len;
									}
									len =
											( USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
									memcpy( USBHS_EP0_Buf , pUSBHS_Descr , len );
									pUSBHS_Descr += len;
									break;

									/* Set usb address */
								case USB_SET_ADDRESS:
									USBHS_DevAddr = (uint16_t) ( USBHS_SetupReqValue & 0xFF );
									break;

									/* Get usb configuration now set */
								case USB_GET_CONFIGURATION:
									USBHS_EP0_Buf[ 0 ] = USBHS_DevConfig;
									if ( USBHS_SetupReqLen > 1 )
									{
										USBHS_SetupReqLen = 1;
									}
									break;

									/* Set usb configuration to use */
								case USB_SET_CONFIGURATION:
									USBHS_DevConfig = (uint8_t) ( USBHS_SetupReqValue & 0xFF );
									USBHS_DevEnumStatus = 0x01;
									break;

									/* Clear or disable one usb feature */
								case USB_CLEAR_FEATURE:
									if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
									{
										/* clear one device feature */
										if ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) == 0x01 )
										{
											/* clear usb sleep status, device not prepare to sleep */
											USBHS_DevSleepStatus &= ~0x01;
										}
										else
										{
											errflag = 0xFF;
										}
									}
									else if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
									{
										/* Set End-point Feature */
										if ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
										{
											/* Clear End-point Feature */
											switch ( (uint8_t) ( USBHS_SetupReqIndex & 0xFF ) )
											{
												case ( DEF_UEP2 | DEF_UEP_OUT ):
													/* Set End-point 2 OUT ACK */
													USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
													break;

												case ( DEF_UEP2 | DEF_UEP_IN ):
													/* Set End-point 2 IN NAK */
													USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
													break;

												case ( DEF_UEP3 | DEF_UEP_OUT ):
													/* Set End-point 3 OUT ACK */
													USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK;
													break;

												case ( DEF_UEP4 | DEF_UEP_IN ):
													/* Set End-point 4 IN NAK */
													USBHSD->UEP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
													break;

												case ( DEF_UEP5 | DEF_UEP_OUT ):
													/* Set End-point 5 OUT ACK */
													USBHSD->UEP5_RX_CTRL = USBHS_UEP_R_RES_ACK;
													break;

												case ( DEF_UEP6 | DEF_UEP_IN ):
													/* Set End-point 6 IN NAK */
													USBHSD->UEP6_TX_CTRL = USBHS_UEP_T_RES_NAK;
													break;

												default:
													errflag = 0xFF;
													break;
											}
										}
										else
										{
											errflag = 0xFF;
										}

									}
									else
									{
										errflag = 0xFF;
									}
									break;

									/* set or enable one usb feature */
								case USB_SET_FEATURE:
									if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
									{
										/* Set Device Feature */
										if ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
										{
											if ( MyCfgDescr_FS[ 7 ] & 0x20 )
											{
												/* Set Wake-up flag, device prepare to sleep */
												USBHS_DevSleepStatus |= 0x01;
											}
											else
											{
												errflag = 0xFF;
											}
										}
										else if ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) == 0x02 )
										{
											/* test mode deal */
											if ( ( USBHS_SetupReqIndex == 0x0100 ) || ( USBHS_SetupReqIndex == 0x0200 )
													|| ( USBHS_SetupReqIndex == 0x0300 )
													|| ( USBHS_SetupReqIndex == 0x0400 ) )
											{
												/* Set the flag and wait for the status to be uploaded before proceeding with the actual operation */
												USBHS_Test_Flag |= 0x80;
											}
										}
										else
										{
											errflag = 0xFF;
										}
									}
									else if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
									{
										/* Set End-point Feature */
										if ( (uint8_t) ( USBHS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
										{
											/* Set end-points status stall */
											switch ( (uint8_t) ( USBHS_SetupReqIndex & 0xFF ) )
											{
												case ( DEF_UEP1 | DEF_UEP_OUT ):
													/* Set End-point 1 OUT STALL */
													USBHSD->UEP1_RX_CTRL = ( USBHSD->UEP1_RX_CTRL
															& ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
													break;

												case ( DEF_UEP1 | DEF_UEP_IN ):
													/* Set End-point 1 IN STALL */
													USBHSD->UEP1_TX_CTRL = ( USBHSD->UEP1_TX_CTRL
															& ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
													break;

												case ( DEF_UEP3 | DEF_UEP_OUT ):
													/* Set End-point 3 OUT STALL */
													USBHSD->UEP3_RX_CTRL = ( USBHSD->UEP3_RX_CTRL
															& ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
													break;

												case ( DEF_UEP4 | DEF_UEP_IN ):
													/* Set End-point 4 IN STALL */
													USBHSD->UEP4_TX_CTRL = ( USBHSD->UEP4_TX_CTRL
															& ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
													break;

												case ( DEF_UEP5 | DEF_UEP_OUT ):
													/* Set End-point 5 OUT STALL */
													USBHSD->UEP5_RX_CTRL = ( USBHSD->UEP5_RX_CTRL
															& ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
													break;

												case ( DEF_UEP6 | DEF_UEP_IN ):
													/* Set End-point 6 IN STALL */
													USBHSD->UEP6_TX_CTRL = ( USBHSD->UEP6_TX_CTRL
															& ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
													break;

												default:
													errflag = 0xFF;
													break;
											}
										}
									}
									break;

									/* This request allows the host to select another setting for the specified interface  */
								case USB_GET_INTERFACE:
									USBHS_EP0_Buf[ 0 ] = 0x00;
									if ( USBHS_SetupReqLen > 1 )
									{
										USBHS_SetupReqLen = 1;
									}
									break;

								case USB_SET_INTERFACE:
									break;

									/* host get status of specified device/interface/end-points */
								case USB_GET_STATUS:
									USBHS_EP0_Buf[ 0 ] = 0x00;
									USBHS_EP0_Buf[ 1 ] = 0x00;
									if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
									{
										switch ( (uint8_t) ( USBHS_SetupReqIndex & 0xFF ) )
										{
											case ( DEF_UEP1 | DEF_UEP_OUT ):
												if ( ( ( USBHSD->UEP1_RX_CTRL ) & USBHS_UEP_R_RES_MASK )
														== USBHS_UEP_R_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}

											case ( DEF_UEP1 | DEF_UEP_IN ):
												if ( ( ( USBHSD->UEP1_TX_CTRL ) & USBHS_UEP_T_RES_MASK )
														== USBHS_UEP_T_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											case ( DEF_UEP2 | DEF_UEP_IN ):
												if ( ( ( USBHSD->UEP2_TX_CTRL ) & USBHS_UEP_T_RES_MASK )
														== USBHS_UEP_T_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											case ( DEF_UEP3 | DEF_UEP_OUT ):
												if ( ( ( USBHSD->UEP3_RX_CTRL ) & USBHS_UEP_R_RES_MASK )
														== USBHS_UEP_R_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											case ( DEF_UEP4 | DEF_UEP_IN ):
												if ( ( ( USBHSD->UEP4_TX_CTRL ) & USBHS_UEP_T_RES_MASK )
														== USBHS_UEP_T_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											case ( DEF_UEP5 | DEF_UEP_OUT ):
												if ( ( ( USBHSD->UEP5_RX_CTRL ) & USBHS_UEP_R_RES_MASK )
														== USBHS_UEP_R_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											case ( DEF_UEP6 | DEF_UEP_IN ):
												if ( ( ( USBHSD->UEP6_TX_CTRL ) & USBHS_UEP_T_RES_MASK )
														== USBHS_UEP_T_RES_STALL )
												{
													USBHS_EP0_Buf[ 0 ] = 0x01;
												}
												break;

											default:
												errflag = 0xFF;
												break;
										}
									}
									else if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
									{
										if ( USBHS_DevSleepStatus & 0x01 )
										{
											USBHS_EP0_Buf[ 0 ] = 0x02;
										}
									}

									if ( USBHS_SetupReqLen > 2 )
									{
										USBHS_SetupReqLen = 2;
									}
									break;

								default:
									errflag = 0xFF;
									break;
							}
						}

						/* errflag = 0xFF means a request not support or some errors occurred, else correct */
						if ( errflag == 0xFF )
						{
							/* if one request not support, return stall */
							USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
							USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
						}
						else
						{
							/* end-point 0 data Tx/Rx */
							if ( USBHS_SetupReqType & DEF_UEP_IN )
							{
								/* tx */
								len =
										( USBHS_SetupReqLen > DEF_USBD_UEP0_SIZE ) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
								USBHS_SetupReqLen -= len;
								USBHSD->UEP0_TX_LEN = len;
								USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
							}
							else
							{
								/* rx */
								if ( USBHS_SetupReqLen == 0 )
								{
									USBHSD->UEP0_TX_LEN = 0;
									USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
								}
								else
								{
									USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
								}
							}
						}
					}
					/* end-point 0 data out interrupt */
					else
					{
						USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_NAK; // clear
						len = USBHSD->UEP0_RX_LEN;

						/* if any processing about rx, set it here */
						if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
						{
							USBHS_SetupReqLen = 0;
							/* Non-standard request end-point 0 Data download */
							if ( USBHS_SetupReqCode == CDC_SET_LINE_CODING )
							{

							}
						}
						else
						{
							/* Standard request end-point 0 Data download */
						}

						if ( USBHS_SetupReqLen == 0 )
						{
							USBHSD->UEP0_TX_LEN = 0;
							USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
						}
					}
					USBHSD->UEP0_RX_CTRL &= ~USBHS_UEP_ENDP_R_DONE;
					break;
					/* end-point 1 data out interrupt */
				case DEF_UEP1:
					EP1_Rx_Cnt = USBHSD->UEP1_RX_LEN;
					EP1_OUT_Flag = 1;
					USBHSD->UEP1_RX_CTRL = ( USBHSD->UEP1_RX_CTRL & ( ~USBHS_UEP_R_RES_MASK ) ) | USBHS_UEP_R_RES_NAK;
					USBHSD->UEP1_RX_CTRL &= ~USBHS_UEP_ENDP_R_DONE;
					break;

					/* end-point 2 data out interrupt */
				case DEF_UEP2:

					USBHSD->UEP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
					EP2_Rx_Cnt = USBHSD->UEP2_RX_LEN;
					EP2_OUT_Flag = 1;

					DevEPhs_OUT_Deal( EP2_Rx_Cnt );

					USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_ENDP_R_DONE;
					USBHSD->UEP2_RX_CTRL |= USBHS_UEP_T_RES_ACK;
					break;

					/* end-point 5 data out interrupt */
				case DEF_UEP5:

					USBHSD->UEP5_RX_CTRL &= ~USBHS_UEP_ENDP_R_DONE;
					break;

				default:
					errflag = 0xFF;
					break;
			}
		}

		else
		{
			/* data-in stage processing */
			switch ( endp_num )
			{
				/* end-point 0 data in interrupt */
				case DEF_UEP0:
					if ( USBHS_SetupReqLen == 0 )
					{
						USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
					}
					if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
					{
						/* Non-standard request endpoint 0 Data upload */
					}
					else
					{
						/* Standard request endpoint 0 Data upload */
						switch ( USBHS_SetupReqCode )
						{
							case USB_GET_DESCRIPTOR:
								len = USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
								memcpy( USBHS_EP0_Buf , pUSBHS_Descr , len );
								USBHS_SetupReqLen -= len;
								pUSBHS_Descr += len;
								USBHSD->UEP0_TX_LEN = len;
								USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
								USBHSD->UEP0_TX_CTRL = ( USBHSD->UEP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK )
										| USBHS_UEP_T_RES_ACK; // clear
								break;

							case USB_SET_ADDRESS:
								USBHSD->DEV_AD = USBHS_DevAddr;
								break;

							default:
								USBHSD->UEP0_TX_LEN = 0;
								break;
						}
					}

					/* test mode */
					if ( USBHS_Test_Flag & 0x80 )
					{
						USB_TestMode_Deal();
					}
					USBHSD->UEP0_TX_CTRL &= ~USBHS_UEP_ENDP_T_DONE;
					break;

					/* end-point 1 data in interrupt */
				case DEF_UEP1:
					USBHSD->UEP1_TX_LEN = 0;
					USBHSD->UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
					Endp1Busy = 0;
					USBHSD->UEP1_TX_CTRL = ( USBHSD->UEP1_TX_CTRL & ( ~USBHS_UEP_T_RES_MASK ) ) | USBHS_UEP_T_RES_NAK;
					USBHSD->UEP1_TX_CTRL &= ~USBHS_UEP_ENDP_T_DONE;
					break;

					/* end-point 2 data in interrupt */
				case DEF_UEP2:
					len = USBHSD->UEP2_TX_LEN;

					USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;

					USBHSD->UEP2_TX_CTRL = ( USBHSD->UEP2_TX_CTRL & ( ~USBHS_UEP_T_RES_MASK ) ) | USBHS_UEP_T_RES_NAK;
					USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_ENDP_T_DONE;
					break;

					/* end-point 3 data in interrupt */
				case DEF_UEP3:
					USBHSD->UEP3_TX_CTRL &= ~USBHS_UEP_ENDP_T_DONE;
					break;

				default:
					break;
			}
		}
	}
	else if ( USBHSD->INT_FG & LINK_RDY_IF )
	{

#ifdef  SUPPORT_USB_HSI

		USB_HSI->CAL_CR |= HSI_CAL_EN | HSI_CAL_VLD;
		USB_HSI->CAL_CR &= ~HSI_CAL_RST;

#endif
		USBHSD->INT_FG = LINK_RDY_IF;

	}
	else if ( intflag & BUS_SUSP_IF )
	{
		USBHSD->INT_FG = BUS_SUSP_IF;
		/* usb suspend interrupt processing */
		if ( USBHSD->MIS_ST & RB_UMS_SUSPEND )
		{
			USBHS_DevSleepStatus |= 0x02;
			if ( USBHS_DevSleepStatus == 0x03 )
			{
				/* Handling usb sleep here */
			}
		}
		else
		{
			USBHS_DevSleepStatus &= ~0x02;
		}
	}
	else if ( intflag & BUS_REST_IF )
	{
		/* usb reset interrupt processing */
		USBHS_DevConfig = 0;
		USBHS_DevAddr = 0;
		USBHS_DevSleepStatus = 0;
		USBHS_DevEnumStatus = 0;

		USBHSD->DEV_AD = 0;
		USBHS_Device_Endp_Init();
		USBHSD->INT_FG = BUS_REST_IF;
	}
	else
	{
		/* other interrupts */
		USBHSD->INT_FG = intflag;
	}
}

/*********************************************************************
 * @fn      void DevEPhs_IN_Deal
 *
 * @brief   Device endpoint2 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEPhs_IN_Deal(UINT8 l)
{
	USBHSD->UEP2_TX_LEN = l;
	USBHSD->UEP2_TX_CTRL = ( USBHSD->UEP2_TX_CTRL & ( ~USBHS_UEP_T_RES_MASK ) ) | USBHS_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEPhs_OUT_Deal
 *
 * @brief   Deal device Endpoint 2 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEPhs_OUT_Deal(UINT8 l)
{
	UINT8 s, EP2_Tx_Cnt;
	memcpy( EP2_Rx_Buffer , Ep2Buffer , l );
	s = RecData_Deal();
	if ( s != ERR_End )
	{
		Ep2Buffer[ 64 ] = 0x00;
		if ( s == ERR_ERROR ) Ep2Buffer[ 65 ] = 0x01;
		else Ep2Buffer[ 65 ] = 0x00;
		EP2_Tx_Cnt = 2;
		DevEPhs_IN_Deal( EP2_Tx_Cnt );
	}
}
