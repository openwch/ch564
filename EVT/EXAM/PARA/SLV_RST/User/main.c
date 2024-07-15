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
 * @Note
 * Para slave routine:
 * Send and transmit data by Para slave 
 * If para received the data match with RST register,the chip will reset
 * The hardware connections are as follows:
 * D0-D7     PD0-PD7
 * INT       PA9
 * A0        PA10
 * CS        PA11
 * RD        PA20
 * WR        PA21
 */
#include "debug.h"

uint8_t __attribute__((__aligned__(4))) Rxbuffer[64];
uint8_t __attribute__((__aligned__(4))) Txbuffer[64];
uint8_t RxCMD = 0;

/*********************************************************************
 * @fn      ParaSlvInit
 *
 * @brief   Para port iniitailize programm
 *
 * @return  none
 */
void ParaSlvInit(void)
{

    /* The code snippet provided is a function named `ParaSlvQryTest` that appears to be related to handling
    communication with a parallel slave device. Here is a breakdown of what the function is doing: */
    SLV_CFG(RB_SLV_ENABLE | RB_SLV_IE_CMD | RB_SLV_IE_RD | RB_SLV_IE_WR, ENABLE);

    GPIOA_ModeCfg(PIN_PARA_A0 | PIN_PARA_PCS | PIN_PARA_RD | PIN_PARA_WR, GPIO_ModeIN_Floating);

    GPIOA_ModeCfg(SLVI, GPIO_ModeOut_PP);
    GPIOA_ResetBits(SLVI);

    SLV_SET_RST_CMD(0xaa);

    RCC_UNLOCK_SAFE_ACCESS();
    BITS_CFG(R32_EXTEN_CTLR0,RB_RST_CMD_EN,ENABLE);
    RCC_LOCK_SAFE_ACCESS();

    GPIOA_ResetBits(SLVI);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    uint32_t ChipID;
    uint16_t pos = 0;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused)) FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("Para Slave Routine\r\n");

    printf("Start\r\n");
    for (int var = 0; var < sizeof(Txbuffer); ++var)
    {
        Txbuffer[var] = var;
    }
    ParaSlvInit();
    while (1)
    {
        if (SLV_GET_IF(RB_IF_SLV_WR))
        {
            SLV_CLEAR_IF(RB_IF_SLV_WR);
            if (SLV_GET_IF(RB_IF_SLV_CMD))
            {
                RxCMD = SLV_GET_DATA();
            }else
            {

                Rxbuffer[pos] = SLV_GET_DATA();
                pos ++ ;
            }

        }
        if(pos >= 64){
            printf("CMD: %x \n",RxCMD);printf("Data: \r\n");
            for (int var = 0; var < sizeof(Rxbuffer); ++var)
            {
                printf("%x ",Rxbuffer[var]);
            }
            printf("\n");
            pos = 0;
        }
    }
}
