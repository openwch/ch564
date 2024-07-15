/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_usb.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      USB firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_USB_H
#define __CH564_USB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"


/* USB standard device request code */
#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS              0x00
#define USB_CLEAR_FEATURE           0x01
#define USB_SET_FEATURE             0x03
#define USB_SET_ADDRESS             0x05
#define USB_GET_DESCRIPTOR          0x06
#define USB_SET_DESCRIPTOR          0x07
#define USB_GET_CONFIGURATION       0x08
#define USB_SET_CONFIGURATION       0x09
#define USB_GET_INTERFACE           0x0A
#define USB_SET_INTERFACE           0x0B
#define USB_SYNCH_FRAME             0x0C
#endif

#define DEF_STRING_DESC_LANG        0x00
#define DEF_STRING_DESC_MANU        0x01
#define DEF_STRING_DESC_PROD        0x02
#define DEF_STRING_DESC_SERN        0x03

/* USB hub class request code */
#ifndef HUB_GET_DESCRIPTOR
#define HUB_GET_STATUS              0x00
#define HUB_CLEAR_FEATURE           0x01
#define HUB_GET_STATE               0x02
#define HUB_SET_FEATURE             0x03
#define HUB_GET_DESCRIPTOR          0x06
#define HUB_SET_DESCRIPTOR          0x07
#endif

/* USB HID class request code */
#ifndef HID_GET_REPORT
#define HID_GET_REPORT              0x01
#define HID_GET_IDLE                0x02
#define HID_GET_PROTOCOL            0x03
#define HID_SET_REPORT              0x09
#define HID_SET_IDLE                0x0A
#define HID_SET_PROTOCOL            0x0B
#endif

/* USB CDC Class request code */
#ifndef CDC_GET_LINE_CODING
#define CDC_GET_LINE_CODING         0x21                                      /* This request allows the host to find out the currently configured line coding */
#define CDC_SET_LINE_CODING         0x20                                      /* Configures DTE rate, stop-bits, parity, and number-of-character */
#define CDC_SET_LINE_CTLSTE         0x22                                      /* This request generates RS-232/V.24 style control signals */
#define CDC_SEND_BREAK              0x23                                      /* Sends special carrier modulation used to specify RS-232 style break */
#endif

/* Bit Define for USB Request Type */
#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN              0x80
#define USB_REQ_TYP_OUT             0x00
#define USB_REQ_TYP_READ            0x80
#define USB_REQ_TYP_WRITE           0x00
#define USB_REQ_TYP_MASK            0x60
#define USB_REQ_TYP_STANDARD        0x00
#define USB_REQ_TYP_CLASS           0x20
#define USB_REQ_TYP_VENDOR          0x40
#define USB_REQ_TYP_RESERVED        0x60
#define USB_REQ_RECIP_MASK          0x1F
#define USB_REQ_RECIP_DEVICE        0x00
#define USB_REQ_RECIP_INTERF        0x01
#define USB_REQ_RECIP_ENDP          0x02
#define USB_REQ_RECIP_OTHER         0x03
#define USB_REQ_FEAT_REMOTE_WAKEUP  0x01
#define USB_REQ_FEAT_ENDP_HALT      0x00
#endif

/* USB Descriptor Type */
#ifndef USB_DESCR_TYP_DEVICE
#define USB_DESCR_TYP_DEVICE        0x01
#define USB_DESCR_TYP_CONFIG        0x02
#define USB_DESCR_TYP_STRING        0x03
#define USB_DESCR_TYP_INTERF        0x04
#define USB_DESCR_TYP_ENDP          0x05
#define USB_DESCR_TYP_QUALIF        0x06
#define USB_DESCR_TYP_SPEED         0x07
#define USB_DESCR_TYP_OTG           0x09
#define USB_DESCR_TYP_BOS           0X0F
#define USB_DESCR_TYP_HID           0x21
#define USB_DESCR_TYP_REPORT        0x22
#define USB_DESCR_TYP_PHYSIC        0x23
#define USB_DESCR_TYP_CS_INTF       0x24
#define USB_DESCR_TYP_CS_ENDP       0x25
#define USB_DESCR_TYP_HUB           0x29
#endif

/* USB Device Class */
#ifndef USB_DEV_CLASS_HUB
#define USB_DEV_CLASS_RESERVED      0x00
#define USB_DEV_CLASS_AUDIO         0x01
#define USB_DEV_CLASS_COMMUNIC      0x02
#define USB_DEV_CLASS_HID           0x03
#define USB_DEV_CLASS_MONITOR       0x04
#define USB_DEV_CLASS_PHYSIC_IF     0x05
#define USB_DEV_CLASS_POWER         0x06
#define USB_DEV_CLASS_IMAGE         0x06
#define USB_DEV_CLASS_PRINTER       0x07
#define USB_DEV_CLASS_STORAGE       0x08
#define USB_DEV_CLASS_HUB           0x09
#define USB_DEV_CLASS_VEN_SPEC      0xFF
#endif

/* USB Hub Class Request */
#ifndef HUB_GET_HUB_DESCRIPTOR
#define HUB_CLEAR_HUB_FEATURE       0x20
#define HUB_CLEAR_PORT_FEATURE      0x23
#define HUB_GET_BUS_STATE           0xA3
#define HUB_GET_HUB_DESCRIPTOR      0xA0
#define HUB_GET_HUB_STATUS          0xA0
#define HUB_GET_PORT_STATUS         0xA3
#define HUB_SET_HUB_DESCRIPTOR      0x20
#define HUB_SET_HUB_FEATURE         0x20
#define HUB_SET_PORT_FEATURE        0x23
#endif

/* Hub Class Feature Selectors */
#ifndef HUB_PORT_RESET
#define HUB_C_HUB_LOCAL_POWER       0
#define HUB_C_HUB_OVER_CURRENT      1
#define HUB_PORT_CONNECTION         0
#define HUB_PORT_ENABLE             1
#define HUB_PORT_SUSPEND            2
#define HUB_PORT_OVER_CURRENT       3
#define HUB_PORT_RESET              4
#define HUB_PORT_POWER              8
#define HUB_PORT_LOW_SPEED          9
#define HUB_C_PORT_CONNECTION       16
#define HUB_C_PORT_ENABLE           17
#define HUB_C_PORT_SUSPEND          18
#define HUB_C_PORT_OVER_CURRENT     19
#define HUB_C_PORT_RESET            20
#endif

/* USB UDisk */
#ifndef USB_BO_CBW_SIZE
#define USB_BO_CBW_SIZE             0x1F
#define USB_BO_CSW_SIZE             0x0D
#endif
#ifndef USB_BO_CBW_SIG0
#define USB_BO_CBW_SIG0             0x55
#define USB_BO_CBW_SIG1             0x53
#define USB_BO_CBW_SIG2             0x42
#define USB_BO_CBW_SIG3             0x43
#define USB_BO_CSW_SIG0             0x55
#define USB_BO_CSW_SIG1             0x53
#define USB_BO_CSW_SIG2             0x42
#define USB_BO_CSW_SIG3             0x53
#endif

/*******************************************************************************/
/* USBHS Related Register Macro Definition */

/* USBHS Device Register Definition */
/* Bit definition for USB_CTRL register */
#define DEV_LPM_EN                               0x80                                       /* LPM enable */
#define DEV_EN                                   0x20                                       /* USB device enabled */
#define DEV_DMA_EN                               0x10                                       /* DMA transfer enabled */
#define PHY_SUSPENDM                             0x08                                       /* USB PHY suspend */
#define USB_ALL_CLR                              0x04                                       /* clear all interrupt flags */
#define SIE_RESET                                0x02                                       /* USB protocol processor reset */
#define LINK_RESET                               0x01

/* Bit definition for usb_BASE_MODE register */
#define EXP_SPD_MASK                             0x03                                       /* bit[0:1] controls the desired device speed */
#define EXP_FS_SPD                               0x00                                       /* Full-speed mode */
#define EXP_HS_SPD                               0x01                                       /* High-speed mode */
#define EXP_LOW_SPD                              0x02                                       /* Low-speed mode */

/* Bit definition for USB_INT_EN register */
#define FIFO_OVER_IE                             0x80                                       /* USB Overflow interrupt enable */
#define LINK_RDY_IE                              0x40                                       /* USB connection interrupt enable */
#define RX_SOF_IE                                0x20                                       /* Receive SOF packet interrupt enable */
#define RTX_ACT_IE                               0x10                                       /* USB transfer end interrupt enabled */
#define LPM_ACT_IE                               0x08                                       /* LMP transfer end interrupt enabled */
#define BUS_SLEEP_IE                             0x04                                       /* USB bus sleep interrupt enabled */
#define BUS_SUSP_IE                              0x02                                       /* USB bus pause interrupt enabled */
#define BUS_REST_IE                              0x01                                       /* USB bus reset interrupt enabled */

/* Bit definition for USB_DEV_AD register */
#define MASK_USB_ADDR                            0x7f

/* Bit definition for USB_WAKE_CR register */
#define RB_RMT_WAKE                              0x01                                       /* remote wake up */

/* Bit definition for USB_TEST_MODE register */
#define RB_TEST_EN                               0x80                                       /* test mode enable */
#define RB_TEST_SE0NAK                           0x08                                       /* test mode,output SEO */
#define RB_TEST_PKT                              0x04                                       /* test mode,output a packet */
#define RB_TEST_K                                0x02                                       /* test mode,output K */
#define RB_TEST_J                                0x01                                       /* test mode,output J */

/* Bit definition for USB_LPM_DATA register */
#define LPM_BUSY                                 0x8000
#define LPM_DATA                                 0x07ff                                     /* read-only power management data */

/* Bit definition for USB_INT_FG register */
#define FIFO_OVER_IF                             0x80                                       /* read-write USB Overflow interrupt flag */
#define LINK_RDY_IF                              0x40                                       /* read-write USB connection interrupt flag */
#define RX_SOF_IF                                0x20                                       /* read-write Receive SOF packet interrupt flag */
#define RTX_ACT_IF                               0x10                                       /* read-only USB transmission end interrupt flag */
#define LPM_ACT_IF                               0x08                                       /* read-write LPM transmission end interrupt flag */
#define BUS_SLEEP_IF                             0x04                                       /* read-write USB bus sleep interrupt flag */
#define BUS_SUSP_IF                              0x02                                       /* read-write USB bus suspend interrupt flag */
#define BUS_REST_IF                              0x01                                       /* read-write USB bus reset interrupt flag */

/* Bit definition for USB_INT_ST register */
#define RB_UIS_EP_DIR                            0x10                                       /* Endpoint data transmission direction */
#define RB_UIS_EP_ID_MASK                        0x07                                       /* The endpoint number at which the data transfer occurs */

/* Bit definition for USB_MIS_ST register */
#define RB_UMS_HS_MOD                            0x80                                       /* whether the host is high-speed */
#define RB_UMS_SUSP_REQ                          0x10                                       /* USB suspends the request */
#define RB_UMS_FREE                              0x08                                       /* USB free status */
#define RB_UMS_SLEEP                             0x04                                       /* USB sleep status */
#define RB_UMS_SUSPEND                           0x02                                       /* USB suspend status */
#define RB_UMS_READY                             0x01                                       /* USB connection status */

/* Bit definition for USB_FRAMME_NO register */
#define MICRO_FRAME                              0xe000                                     /* Received micro frame number */
#define FRAME_NO                                 0x07ff                                     /* Received frame number */

/* Bit definition for USB_BUS register */
#define USB_DM_ST                                0x0008                                     /* read-only UDM status */
#define USB_DP_ST                                0x0004                                     /* read-only UDP status */
#define USB_WAKEUP                               0x0001                                     /* read-only USB wakeup */

/* Bit definition for DEV_UEP_TX_EN & DEV_UEP_RX_EN register */
#define RB_EP0_EN                                0x0001
#define RB_EP1_EN                                0x0002
#define RB_EP2_EN                                0x0004
#define RB_EP3_EN                                0x0008
#define RB_EP4_EN                                0x0010
#define RB_EP5_EN                                0x0020
#define RB_EP6_EN                                0x0040
#define RB_EP7_EN                                0x0080
#define RB_EP8_EN                                0x0100
#define RB_EP9_EN                                0x0200
#define RB_EP10_EN                               0x0400
#define RB_EP11_EN                               0x0800
#define RB_EP12_EN                               0x1000
#define RB_EP13_EN                               0x2000
#define RB_EP14_EN                               0x4000
#define RB_EP15_EN                               0x8000

/* Bit definition for DEV_UEP_T_TOG_AUTO register */
#define EP0_T_TOG_AUTO                           0x01
#define EP1_T_TOG_AUTO                           0x02
#define EP2_T_TOG_AUTO                           0x04
#define EP3_T_TOG_AUTO                           0x08
#define EP4_T_TOG_AUTO                           0x10
#define EP5_T_TOG_AUTO                           0x20
#define EP6_T_TOG_AUTO                           0x40
#define EP7_T_TOG_AUTO                           0x80

/* Bit definition for DEV_UEP_R_TOG_AUTO register */
#define EP0_R_TOG_AUTO                           0x01
#define EP1_R_TOG_AUTO                           0x02
#define EP2_R_TOG_AUTO                           0x04
#define EP3_R_TOG_AUTO                           0x08
#define EP4_R_TOG_AUTO                           0x10
#define EP5_R_TOG_AUTO                           0x20
#define EP6_R_TOG_AUTO                           0x40
#define EP7_R_TOG_AUTO                           0x80

/* Bit definition for DEV_UEP_T_BURST register */
#define EP0_T_BURST_EN                           0x01
#define EP1_T_BURST_EN                           0x02
#define EP2_T_BURST_EN                           0x04
#define EP3_T_BURST_EN                           0x08
#define EP4_T_BURST_EN                           0x10
#define EP5_T_BURST_EN                           0x20
#define EP6_T_BURST_EN                           0x40
#define EP7_T_BURST_EN                           0x80

/* Bit definition for DEV_UEP_T_BURST_MODE register */
#define EP0_T_BURST_MODE                         0x01
#define EP1_T_BURST_MODE                         0x02
#define EP2_T_BURST_MODE                         0x04
#define EP3_T_BURST_MODE                         0x08
#define EP4_T_BURST_MODE                         0x10
#define EP5_T_BURST_MODE                         0x20
#define EP6_T_BURST_MODE                         0x40
#define EP7_T_BURST_MODE                         0x80

/* Bit definition for DEV_UEP_R_BURST register */
#define EP0_R_BURST_EN                           0x01
#define EP1_R_BURST_EN                           0x02
#define EP2_R_BURST_EN                           0x04
#define EP3_R_BURST_EN                           0x08
#define EP4_R_BURST_EN                           0x10
#define EP5_R_BURST_EN                           0x20
#define EP6_R_BURST_EN                           0x40
#define EP7_R_BURST_EN                           0x80

/* Bit definition for DEV_UEP_R_RES_MODE register */
#define EP0_R_RES_MODE                           0x01
#define EP1_R_RES_MODE                           0x02
#define EP2_R_RES_MODE                           0x04
#define EP3_R_RES_MODE                           0x08
#define EP4_R_RES_MODE                           0x10
#define EP5_R_RES_MODE                           0x20
#define EP6_R_RES_MODE                           0x40
#define EP7_R_RES_MODE                           0x80

/* Bit definition for DEV_UEP_AF_MODE register */
#define EP1_T_AF                                 0x02
#define EP2_T_AF                                 0x04
#define EP3_T_AF                                 0x08
#define EP4_T_AF                                 0x10
#define EP5_T_AF                                 0x20
#define EP6_T_AF                                 0x40
#define EP7_T_AF                                 0x80

/* Bit definition for UEPx_TX_CTRL register */
#define USBHS_UEP_T_RES_MASK                     0x03                                       /* Response control mask for endpoint 0 transmission */
#define USBHS_UEP_T_RES_NAK                      0x00                                       /* UEP0_TX_CTRL[0:1] = 00, reply NAK to host */
#define USBHS_UEP_T_RES_STALL                    0x01                                       /* UEP0_TX_CTRL[0:1] = 01, reply STALL to host */
#define USBHS_UEP_T_RES_ACK                      0x02                                       /* UEP0_TX_CTRL[0:1] = 10, reply ACK to host */
#define USBHS_UEP_T_RES_NYET                     0x03                                       /* UEP0_TX_CTRL[0:1] = 11, reply NYET to host */
#define USBHS_UEP_T_TOG_MASK                     0x0C                                       /* Synchronization trigger bit mask */
#define USBHS_UEP_T_TOG_DATA0                    0x00                                       /* UEP0_TX_CTRL[2:3] = 00, represents DATA0 */
#define USBHS_UEP_T_TOG_DATA1                    0x04                                       /* UEP0_TX_CTRL[2:3] = 01, represents DATA1 */
#define USBHS_UEP_T_TOG_DATA2                    0x08                                       /* UEP0_TX_CTRL[2:3] = 10, represents DATA2 */
#define USBHS_UEP_T_TOG_MDATA                    0x0C                                       /* UEP0_TX_CTRL[2:3] = 11, represents MDATA */
#define USBHS_UEP_ENDP_T_DONE                    0x80                                       /* Writing 0 clears the interrupt */

/* Bit definition for UEPx_RX_CTRL register */
#define USBHS_UEP_R_RES_MASK                     0x03                                       /* Response control mask for endpoint 0 transmission */
#define USBHS_UEP_R_RES_NAK                      0x00                                       /* UEP0_TX_CTRL[0:1] = 00, reply NAK to host */
#define USBHS_UEP_R_RES_STALL                    0x01                                       /* UEP0_TX_CTRL[0:1] = 01, reply STALL to host */
#define USBHS_UEP_R_RES_ACK                      0x02                                       /* UEP0_TX_CTRL[0:1] = 10, reply ACK to host */
#define USBHS_UEP_R_RES_NYET                     0x03                                       /* UEP0_TX_CTRL[0:1] = 11, reply NYET to host */
#define USBHS_UEP_R_TOG_MASK                     0x0C                                       /* Synchronization trigger bit mask */
#define USBHS_UEP_R_TOG_DATA0                    0x00                                       /* UEP0_TX_CTRL[2:3] = 00, represents DATA0 */
#define USBHS_UEP_R_TOG_DATA1                    0x04                                       /* UEP0_TX_CTRL[2:3] = 01, represents DATA1 */
#define USBHS_UEP_R_TOG_DATA2                    0x08                                       /* UEP0_TX_CTRL[2:3] = 10, represents DATA2 */
#define USBHS_UEP_R_TOG_MDATA                    0x0C                                       /* UEP0_TX_CTRL[2:3] = 11, represents MDATA */
#define USBHS_UEP_ENDP_T_DONE                    0x80                                       /* Writing 0 clears the interrupt */
#define USBHS_UEP_ENDP_R_DONE                    0x80                                       /* Writing 0 clears the interrupt */
#define USBHS_RB_SETUP_IS                        0x08                                       /* Indicates whether the reception of endpoint 0 is a Setup transaction */
#define USBHS_ENDP_R_TOG_MATCH                   0x10

/* Bit definition for DEV_UEP_T_ISO register */
#define EP1_T_ISO                                0x02
#define EP2_T_ISO                                0x04
#define EP3_T_ISO                                0x08
#define EP4_T_ISO                                0x10
#define EP5_T_ISO                                0x20
#define EP6_T_ISO                                0x40
#define EP7_T_ISO                                0x80

/* Bit definition for DEV_UEP_R_ISO register */
#define EP1_R_ISO                                0x02
#define EP2_R_ISO                                0x04
#define EP3_R_ISO                                0x08
#define EP4_R_ISO                                0x10
#define EP5_R_ISO                                0x20
#define EP6_R_ISO                                0x40
#define EP7_R_ISO                                0x80

/* USBHS Host Register Definition */
/* Bit definition for UHOST_CTRL register */
#define root_LPM_EN         (1<<7)
#define ROOT_FORCE_FS       (1<<6)
#define ROOT_SOF_EN         (1<<5)
#define ROOT_DMA_EN         (1<<4)
#define ROOT_PHY_SUSPENDM   (1<<3)
#define ROOT_ALL_CLR        (1<<2)
#define ROOT_SIE_RESET      (1<<1)
#define ROOT_LINK_RESET     (1<<0)

/* Bit definition for UH_INT_EN register */
#define FIFO_OV_IE          (1<<7)
#define TX_HALT_IE          (1<<6)
#define SOF_ACT_IE          (1<<5)
#define USB_ACT_IE          (1<<4)
#define RESUME_ACT_IE       (1<<3)
#define WKUP_ACT_IE         (1<<2)

/* Bit definition for UH_CONTROL register */
#define RX_NO_RES           (1<<23)
#define TX_NO_RES           (1<<22)
#define RX_NO_DATA          (1<<21)
#define TX_NO_DATA          (1<<20)
#define TX_LOW_SPD          (1<<19)
#define SPLIT_VALID         (1<<18)
#define LPM_VALID           (1<<17)
#define HOST_ACTION         (1<<16)
#define BUF_MODE            (1<<10)
#define TOG_MASK            (3<<8)
#define TOG_MDATA           (3<<8)
#define TOG_DATA2           (2<<8)
#define TOG_DATA1           (1<<8)
#define TOG_DATA0           (0<<8)

/* Bit definition for UH_INT_FLAG register */
#define RB_FIFO_OV_IF       (1<<7)
#define RB_TX_HALT_IF       (1<<6)
#define RB_SOF_ACT_IF       (1<<5)
#define RB_USB_ACT_IF       (1<<4)
#define RB_RESUME_ACT_IF    (1<<3)
#define RB_WKUP_IF          (1<<2)

/* Bit definition for UH_INT_ST register */
#define PORT_RX_RESUME      (1<<4)
#define USB_PID_MASK        0x0f
#define USB_PID_TOUT        0x0
#define USB_PID_ACK         0x2
#define USB_PID_NAK         0xa
#define USB_PID_STALL       0xe
#define USB_PID_NYET        0x6
#define USB_PID_DATA0       0x3
#define USB_PID_DATA1       0xb
#define USB_PID_DATA2       0x7
#define USB_PID_MDATA       0xf

#define USB_PID_PRE         0xc
#define USB_PID_ERR         0xc
#define USB_PID_SPLIT       0x8
#define USB_PID_PING        0x4
#define USB_PID_SOF         0x5
#define USB_PID_SETUP       0xd
#define USB_PID_IN          0x9
#define USB_PID_OUT         0x1

/* Bit definition for UH_MIS_ST register */
#define RB_BUS_SE0          (1<<7)
#define RB_BUS_J            (1<<6)
#define RB_LINESTATE_MASK   (0x3<<4)
#define RB_USB_WAKEUP       (1<<3)
#define RB_SOF_ST           (1<<2)
#define RB_SOF_PRE          (1<<1)
#define RB_SOF_FREE         (1<<0)

/* Bit definition for UH_FRAME register */
#define SOF_CNT_CLR         (1<<25)
#define SOF_CNT_EN          (1<<24)

/* Bit definition for PORT_CTRL register */
#define BUS_RST_LONG        (1<<16)
#define PORT_SLEEP_BESL     (0xf<<12)
#define CLR_PORT_SLEEP      (1<<8)
#define CLR_PORT_CONNECT    (1<<5)
#define CLR_PORT_EN         (1<<4)
#define SET_PORT_SLEEP      (1<<3)
#define CLR_PORT_SUSP       (1<<2)
#define SET_PORT_SUSP       (1<<1)
#define SET_PORT_RESET      (1<<0)

/* Bit definition for PORT_CFG register */
#define PORT_15K_RPD        (1<<7)
#define PORT_HOST_MODE      (1<<0)//1: HOST function
#define PORT_DEVICE_MODE    (0<<0)//0: DEVICE function

/* Bit definition for PORT_INT_EN register */
#define PORT_SLP_IE         (1<<5)
#define PORT_RESET_IE       (1<<4)
#define PORT_SUSP_IE        (1<<2)
#define PORT_EN_IE          (1<<1)
#define PORT_CONNECT_IE     (1<<0)

/* Bit definition for PORT_TEST_CT register */
#define TEST_FORCE_EN       (1<<2)
#define TEST_K              (1<<1)
#define TEST_J              (1<<0)

/* Bit definition for PORT_STATUS register */
#define PORT_TEST           (1<<11)
#define PORT_SPD_MASK       (3<<9)
#define PORT_HIGH_SPD       (1<<10)
#define PORT_LOW_SPD        (1<<9)
#define PORT_FULL_SPD       (0<<9)
#define PORT_SLP            (1<<5)
#define PORT_RESETTING      (1<<4)
#define PORT_OVC            (1<<3)
#define PORT_SUSP           (1<<2)
#define PORT_EN             (1<<1)
#define PORT_CONNECT        (1<<0)

/* Bit definition for PORT_STATUS_CHG register */
#define PORT_SLP_IF         (1<<5)
#define PORT_RESET_IF       (1<<4)
#define PORT_SUSP_IF        (1<<2)
#define PORT_EN_IF          (1<<1)
#define PORT_CONNECT_IF     (1<<0)

/* Bit definition for ROOT_BC_CR register */
#define UDM_VSRC_ACT        (1<<10)
#define UDM_BC_CMPE         (1<<9)
#define UDP_BC_CMPE         (1<<8)
#define BC_AUTO_MODE        (1<<6)
#define UDM_BC_VSRC         (1<<5)
#define UDP_BC_VSRC         (1<<4)
#define UDM_BC_CMPO         (1<<1)
#define UDP_BC_CMPO         (1<<0)

/* Bit definition for HSI_CAL_CR register */
#define CLK_SEL             (1<<21)
#define SOF_FREE            (1<<3)
#define SFT_RST             (1<<2)
#define CAL_EN              (1<<1)
#define CAL_RST             (1<<0)

/*******************************************************************************/
/* Struct Definition */

/* USB Setup Request */
typedef struct __attribute__((packed)) _USB_SETUP_REQ
{
    uint8_t  bRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_SETUP_REQ, *PUSB_SETUP_REQ;

/* USB Device Descriptor */
typedef struct __attribute__((packed)) _USB_DEVICE_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} USB_DEV_DESCR, *PUSB_DEV_DESCR;

/* USB Configuration Descriptor */
typedef struct __attribute__((packed)) _USB_CONFIG_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  MaxPower;
} USB_CFG_DESCR, *PUSB_CFG_DESCR;

/* USB Interface Descriptor */
typedef struct __attribute__((packed)) _USB_INTERF_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} USB_ITF_DESCR, *PUSB_ITF_DESCR;

/* USB Endpoint Descriptor */
typedef struct __attribute__((packed)) _USB_ENDPOINT_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint8_t  wMaxPacketSizeL;
    uint8_t  wMaxPacketSizeH;
    uint8_t  bInterval;
} USB_ENDP_DESCR, *PUSB_ENDP_DESCR;

/* USB Configuration Descriptor Set */
typedef struct __attribute__((packed)) _USB_CONFIG_DESCR_LONG
{
    USB_CFG_DESCR  cfg_descr;
    USB_ITF_DESCR  itf_descr;
    USB_ENDP_DESCR endp_descr[ 1 ];
} USB_CFG_DESCR_LONG, *PUSB_CFG_DESCR_LONG;

/* USB HUB Descriptor */
typedef struct __attribute__((packed)) _USB_HUB_DESCR
{
    uint8_t  bDescLength;
    uint8_t  bDescriptorType;
    uint8_t  bNbrPorts;
    uint8_t  wHubCharacteristicsL;
    uint8_t  wHubCharacteristicsH;
    uint8_t  bPwrOn2PwrGood;
    uint8_t  bHubContrCurrent;
    uint8_t  DeviceRemovable;
    uint8_t  PortPwrCtrlMask;
} USB_HUB_DESCR, *PUSB_HUB_DESCR;

/* USB HID Descriptor */
typedef struct __attribute__((packed)) _USB_HID_DESCR
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bDescriptorTypeX;
    uint8_t  wDescriptorLengthL;
    uint8_t  wDescriptorLengthH;
} USB_HID_DESCR, *PUSB_HID_DESCR;

/* USB UDisk */
typedef struct __attribute__((packed)) _UDISK_BOC_CBW
{
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCBW_DataLen;
    uint8_t  mCBW_Flag;
    uint8_t  mCBW_LUN;
    uint8_t  mCBW_CB_Len;
    uint8_t  mCBW_CB_Buf[ 16 ];
} UDISK_BOC_CBW, *PXUDISK_BOC_CBW;

/* USB UDisk */
typedef struct __attribute__((packed)) _UDISK_BOC_CSW
{
    uint32_t mCBW_Sig;
    uint32_t mCBW_Tag;
    uint32_t mCSW_Residue;
    uint8_t  mCSW_Status;
} UDISK_BOC_CSW, *PXUDISK_BOC_CSW;

#ifdef __cplusplus
}
#endif

#endif
