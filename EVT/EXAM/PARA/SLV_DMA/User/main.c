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
 * The hardware connections are as follows:
 * D0-D7     PD0-PD7
 * INT       PA9
 * A0        PA10
 * CS        PA11
 * RD        PA20
 * WR        PA21
 */
#include "debug.h"

uint8_t __attribute__((__aligned__(4))) Rxbuffer[4096];
uint8_t __attribute__((__aligned__(4))) Txbuffer[4096];
uint8_t RxCMD = 0;

void __attribute__((interrupt("WCH-Interrupt-fast"))) SLV_IRQHandler() {
    if (SLV_DMA_GET_IF(WR_DMA_END_IF)) {
        SLV_DMA_CLEAR_IF(WR_DMA_END_IF);
        for (int var = 0; var < sizeof(Rxbuffer); ++var) {
            printf("%02x  ", Rxbuffer[var]);
        }
        printf("\n");
    }
    if (SLV_DMA_GET_IF(RD_DMA_END_IF)) {
        SLV_DMA_CLEAR_IF(RD_DMA_END_IF);
        printf("Send finished\r\n");
        printf("\n");
    }
}

/*********************************************************************
 * @fn      ParaSlvDMAInit
 *
 * @brief   Para port iniitailize programm
 *
 * @return  none
 */
void ParaSlvDMAInit(void) {

    /* The code snippet provided is a function named `ParaSlvQryTest` that appears to be related to handling
     communication with a parallel slave device. Here is a breakdown of what the function is doing: */

    GPIOA_ModeCfg(PIN_PARA_A0 | PIN_PARA_PCS | PIN_PARA_RD | PIN_PARA_WR,
            GPIO_ModeIN_Floating);

    GPIOA_ModeCfg(SLVI, GPIO_ModeOut_PP);
    GPIOA_ResetBits(SLVI);

    SLV_SET_DMA_CMD0(0xaa);
    SLV_SET_DMA_CMD1(0x55);

    SLV_DMA_START_ADDR_RD(Txbuffer);
    SLV_DMA_END_ADDR_RD(Txbuffer + 4096);

    SLV_DMA_START_ADDR_WR(Rxbuffer);
    SLV_DMA_END_ADDR_WR(Rxbuffer + 4096);

    SLV_SET_MODE_CTRL(CMD1_WR_MODE, ENABLE);

    SLV_SET_MODE_EN(CMD0_EN | DMA_CMD_MODE1 | CMD1_EN, ENABLE);

    SLV_DMA_CFG(RB_DMA_EN_SLV | RB_DMA_END_IE_SLV, ENABLE);

    SLV_CFG(RB_SLV_ENABLE, ENABLE);

    printf("R8_DMA_MODE_EN_SLV %x R8_DMA_MODE_CTRL_SLV %x R8_DMA_EN_SLV %x \n",
    R8_DMA_MODE_EN_SLV,
    R8_DMA_MODE_CTRL_SLV, R8_DMA_EN_SLV);

    NVIC_EnableIRQ(SLV_IRQn);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {
    uint32_t ChipID;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n", SystemCoreClock);
    __attribute__((unused))   FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("Para Slave Routine\r\n");

    printf("Start\r\n");
    for (int var = 0; var < sizeof(Txbuffer); ++var) {
        Txbuffer[var] = var;
    }
    ParaSlvDMAInit();
    while (1)
    {
    }
}
