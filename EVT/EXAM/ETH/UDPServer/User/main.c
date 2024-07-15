/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
UDP Server example, demonstrating that the UDP Server receives data and loops back.
 */
#include "string.h"
#include "eth_driver.h"

#define UDP_RECE_BUF_LEN                        1472
u8 MACAddr[6] = {0x84, 0xc2, 0xe4, 0x01, 0x02, 0x03};                                  //MAC address
u8 IPAddr[4] = {192, 168, 1, 10};               //IP address
u8 GWIPAddr[4] = {192, 168, 1, 1};              //Gateway IP address
u8 IPMask[4] = {255, 255, 255, 0};              //subnet mask
u16 srcport = 1000;                             //source port

u8 SocketId;
u8 SocketRecvBuf[UDP_RECE_BUF_LEN];             //socket receive buffer
/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   check if error.
 *
 * @param   iError - error constants.
 *
 * @return  none
 */
void mStopIfError(u8 iError)
{
    if (iError == WCHNET_ERR_SUCCESS)
        return;
    printf("Error: %02X\r\n", (u16) iError);
}

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   Initializes TIM2.
 *
 * @return  none
 */
void TIM2_Init(void)
{
    TMR2_ClrCurrentCount();
    R32_TMR2_CNT_END = SystemCoreClock / 1000 * WCHNETTIMERPERIOD;
    R8_TMR2_CTRL_MOD = RB_TMR_COUNT_EN;
    TMR2_ITCfg(ENABLE, RB_TMR_IE_CYC_END);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/*********************************************************************
 * @fn      WCHNET_UdpServerRecv
 *
 * @brief   UDP Receive data function
 *
 *@param    socinf - socket information.
 *          ipaddr - The IP address from which the data was sent
 *          port - source port
 *          buf - pointer to the data buffer
 *          len - received data length
 * @return  none
 */
void WCHNET_UdpServerRecv(struct _SOCK_INF *socinf, u32 ipaddr, u16 port, u8 *buf, u32 len)
{
    u8 ip_addr[4], i;

    printf("Remote IP:");
    for (i = 0; i < 4; i++) {
        ip_addr[i] = ipaddr & 0xff;
        printf("%d ", ip_addr[i]);
        ipaddr = ipaddr >> 8;
    }

    printf("srcport = %d len = %d socketid = %d\r\n", port, len,
            socinf->SockIndex);

    WCHNET_SocketUdpSendTo(socinf->SockIndex, buf, &len, ip_addr, port);
}

/*********************************************************************
 * @fn      WCHNET_CreateUdpSocket
 *
 * @brief   Create UDP Socket
 *
 * @return  none
 */
void WCHNET_CreateUdpSocket(void)
{
    u8 i;
    SOCK_INF TmpSocketInf;

    memset((void *) &TmpSocketInf, 0, sizeof(SOCK_INF));
    TmpSocketInf.SourPort = srcport;
    TmpSocketInf.ProtoType = PROTO_TYPE_UDP;
    TmpSocketInf.RecvStartPoint = (u32) SocketRecvBuf;
    TmpSocketInf.RecvBufLen = UDP_RECE_BUF_LEN;
    TmpSocketInf.AppCallBack = WCHNET_UdpServerRecv;
    i = WCHNET_SocketCreat(&SocketId, &TmpSocketInf);
    printf("SocketId %d\r\n", SocketId);
    mStopIfError(i);
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(u8 socketid, u8 intstat)
{
    if (intstat & SINT_STAT_RECV)                               //receive data
    {
    }
    if (intstat & SINT_STAT_CONNECT)                            //connect successfully
    {
        printf("TCP Connect Success\r\n");
    }
    if (intstat & SINT_STAT_DISCONNECT)                         //disconnect
    {
        printf("TCP Disconnect\r\n");
    }
    if (intstat & SINT_STAT_TIM_OUT)                            //timeout disconnect
    {
        printf("TCP Timeout\r\n");
    }
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global Interrupt Handle
 *
 * @return  none
 */
void WCHNET_HandleGlobalInt(void)
{
    u8 intstat;
    u16 i;
    u8 socketint;

    intstat = WCHNET_GetGlobalInt();                              //get global interrupt flag
    if (intstat & GINT_STAT_UNREACH)                              //Unreachable interrupt
    {
        printf("GINT_STAT_UNREACH\r\n");
    }
    if (intstat & GINT_STAT_IP_CONFLI)                            //IP conflict
    {
        printf("GINT_STAT_IP_CONFLI\r\n");
    }
    if (intstat & GINT_STAT_PHY_CHANGE)                           //PHY status change
    {
        i = WCHNET_GetPHYStatus();
        if (i & PHY_Linked_Status)
            printf("PHY Link Success\r\n");
    }
    if (intstat & GINT_STAT_SOCKET) {                             //socket related interrupt
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
            socketint = WCHNET_GetSocketInt(i);
            if (socketint)
                WCHNET_HandleSockInt(i, socketint);
        }
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program
 *
 * @return  none
 */
int main(void)
{
    u8 i;
    uint32_t ChipID;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);                                    //USART initialize
    printf("UDPServer Test\r\n");
    printf("SystemClk:%d\r\n", SystemCoreClock);
    GetCHIPID(&ChipID);
    printf("ChipID:%#x\r\n", ChipID);
    printf("net version:%x\n", WCHNET_GetVer());
    if ( WCHNET_LIB_VER != WCHNET_GetVer()) {
        printf("version error.\n");
    }
    GetMACAddress(MACAddr);                                   //get the chip MAC address
    printf("mac addr:");
    for(i = 0; i < 6; i++)
        printf("%x ",MACAddr[i]);
    printf("\n");
    TIM2_Init();
    i = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr);           //Ethernet library initialize
    mStopIfError(i);
    if (i == WCHNET_ERR_SUCCESS)
        printf("WCHNET_LibInit Success\r\n");
    WCHNET_CreateUdpSocket();                                     //Create  UDP Socket

    while(1)
    {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if(WCHNET_QueryGlobalInt())
        {
            WCHNET_HandleGlobalInt();
        }
    }
}
