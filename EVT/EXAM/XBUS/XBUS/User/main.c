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
 *XBUS Function Routine:
 *Configure XBUS and read and print external RAM data
 *
 */

#include "debug.h"

uint16_t my_buffer[0x2000];
volatile uint8_t *pXbusPt;
/*********************************************************************
 * @fn      XbusTest
 *
 * @brief   Use xbus to operate the sram
 *
 * @return  none
 */
void XbusTest(void)
{
    uint8_t mFstDat, mTstDat;
    uint8_t mHoldClk; /* hold clk */
    uint32_t i, j;

    PRINT("Xbus Test\n");
    R32_PA_DIR = (1 << 21) | (1 << 20) | 0x7FFF; /* Set the pin mode */
    mFstDat = 0x55;
    PRINT("setup clk=1:\n");
    for (mHoldClk = 0; mHoldClk <= RB_XBUS_HOLD; mHoldClk++)
    {
        R8_SAFE_ACCESS_SIG = 0x57; /* unlock step 1 */
        R8_SAFE_ACCESS_SIG = 0xA8; /* unlock step 2 */

        R8_XBUS_CONFIG = RB_XBUS_ENABLE | RB_XBUS_ADDR_OE; 
        R8_XBUS_SETUP_HOLD = mHoldClk;                     

        R8_SAFE_ACCESS_SIG = 0x00; /* lock, to prevent unexpected writing */

        j = 1024 * 32;                   
        pXbusPt = (uint8_t *)0x00C00000; 
        mTstDat = mFstDat;

        PRINT("    mHoldClk=0x%x, FstDat=0x%x\n", (uint16_t)mHoldClk, (uint16_t)mFstDat);
        PRINT("        write: ");
        for (i = 0; i < j; i++)
        {
            if (i < 16)
                PRINT("%x ", (uint16_t)mTstDat);
            *pXbusPt = mTstDat;
            pXbusPt++;
            mTstDat ^= 0xFF;
        }
        PRINT("\n");
        PRINT("        read:  ");
        pXbusPt = (uint8_t *)0x00C00000; 
        mTstDat = mFstDat;               
        for (i = 0; i < j; i++)
        {
            if (i < 16)
                PRINT("%x  ", (uint16_t)*pXbusPt);
            if (*pXbusPt != mTstDat)
            {
                PRINT("        err: i=0x%lX, mTstDat=%x\n", (uint32_t)i, (uint16_t)mTstDat);
                break;
            }
            pXbusPt++;
            mTstDat ^= 0xFF;
        }
        PRINT("\n        over\n");
        mFstDat ^= 0xFF;
    }
    PRINT("setup clk=2:\n");
    for (mHoldClk = 0; mHoldClk <= RB_XBUS_HOLD; mHoldClk++)
    {
        R8_SAFE_ACCESS_SIG = 0x57; /* unlock step 1 */
        R8_SAFE_ACCESS_SIG = 0xA8; /* unlock step 2 */

        R8_XBUS_CONFIG = RB_XBUS_ENABLE | RB_XBUS_ADDR_OE; 
        R8_XBUS_SETUP_HOLD = mHoldClk;                     

        R8_SAFE_ACCESS_SIG = 0x00; /* lock, to prevent unexpected writing */

        j = 1024 * 32;                   
        pXbusPt = (uint8_t *)0x00C00000; 
        mTstDat = mFstDat;               
        PRINT("    mHoldClk=0x%x, FstDat=0x%x\n", (uint16_t)mHoldClk, (uint16_t)mFstDat);
        PRINT("        write: ");
        for (i = 0; i < j; i++)
        {
            if (i < 16)
                PRINT("%x ", (uint16_t)mTstDat); 
            *pXbusPt = mTstDat;
            pXbusPt++;
            mTstDat ^= 0xFF;
        }
        PRINT("\n");
        PRINT("        read:  ");
        pXbusPt = (uint8_t *)0x00C00000; 
        mTstDat = mFstDat;               
        for (i = 0; i < j; i++)
        {
            if (i < 16)
                PRINT("%x  ", (uint16_t)*pXbusPt);
            if (*pXbusPt != mTstDat)
            {
                PRINT("        err: i=0x%lX, mTstDat=%x\n", (uint32_t)i, (uint16_t)mTstDat);
                break;
            }
            pXbusPt++;
            mTstDat ^= 0xFF;
        }
        PRINT("\n        over\n");
        mFstDat ^= 0xFF;
    }
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

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%08x\r\n", ChipID);
    printf("xBus Routine\r\n");

    printf("Start\r\n");

    XbusTest();

    while (1)
    {
    }
}
