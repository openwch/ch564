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
 *@NoteMy
 *Test Flash routines:
 *Verify FLASH availability by reading, writing, and checksumming.
 *
 */

#include "debug.h"
#include "stdarg.h"
#include "stdlib.h"
#include "string.h"
#define LEN 1024

#define READPROTECT 0

uint8_t datas[LEN] = {0};
uint8_t temp[LEN] = {0};

/*********************************************************************
 * @fn      EnableCodeProtection
 *
 * @brief   Enables the code protection.
 *
 * @return  FLASH_Status - The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_UNLOCK_ERROR.
 */
FLASH_Status EnableCodeProtection(void)
{
   __attribute__((unused))  FLASH_Status res = FLASH_COMPLETE;

    if(EXTEN->CTLR0 & RB_CORE_PROT_STATUS)
    {
        printf("MCU in code protected state\r\n");
    }
    else
    {
        FLASH_Unlock();
        res = FLASH_EnableCodeProtection();
        FLASH_Lock();
    }

    return res;
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

    FLASH_Status sta;
    uint8_t uniqID[8];
    uint8_t macID[6];

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("Flash Routine\r\n");

    __attribute__((unused))  FLASH_Status res = GetCHIPID(&ChipID);
    printf("ChipID:%#x\r\n", ChipID);

    printf("Start\r\n");

    for (int var = 0; var < LEN; ++var)
    {
        datas[var] = var;
    }

    GET_UNIQUE_ID(uniqID);
    printf("uniqID ");
    for (int var = 0; var < 8; ++var)
    {
        printf("%#04x ", uniqID[var]);
    }
    printf("\n");

    GetMACAddress(macID);
    printf("macID ");
    for (int var = 0; var < 6; ++var)
    {
        printf("%#04x ", macID[var]);
    }
    printf("\n");

    for (int var = 0; var < LEN; ++var)
    {
        printf("%x ", datas[var]);
    }
    printf("\n ");
    FLASH_Unlock();
    sta = FLASH_ROMA_ERASE(0x10000, 0x1000);
    FLASH_Lock();

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_ERASE failed!! Err Code %x\r\n", sta);

    sta = FLASH_ROMA_READ(0x10000, temp, LEN);

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_READ failed!! Err Code %x\r\n", sta);

    for (int var = 0; var < LEN; ++var)
    {
        if (temp[var] != 0xf5)
        {
            printf("Operation Erase Error!! Err Code %x\r\n");
            break;
        }
    }

    FLASH_Unlock();
    sta = FLASH_ROMA_WRITE(0x10000, datas, LEN);
    FLASH_Lock();
    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_WRITE failed!! Err Code %x\r\n", sta);

    sta = FLASH_ROMA_READ(0x10000, temp, LEN);

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_READ failed!! Err Code %x\r\n", sta);

    if (memcmp(datas, temp, LEN))
    {
        printf("Operation VERIFY failed!!\r\n");
    }

    sta = FLASH_ROMA_VERIFY(0x10000, datas, LEN);

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_VERIFY failed!! Err Code %x\r\n", sta);

    printf("ROMA: ");
    for (int var = 0; var < LEN; ++var)
    {
        printf("%x ", datas[var]);
    }
    printf("\n");

    FLASH_Unlock();
    sta = EEPROM_ERASE(0x75000, 0x1000);
    FLASH_Lock();

    if (sta != FLASH_COMPLETE)
        printf("Operation EEPROM_ERASE failed!! Err Code %x\r\n", sta);

    sta = EEPROM_READ(0x75000, temp, LEN);

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_READ failed!! Err Code %x\r\n", sta);

    for (int var = 0; var < LEN; ++var)
    {
        if (temp[var] != 0xf5)
        {
            printf("Operation Erase Error!! Err Code %x\r\n");
            break;
        }
    }

    FLASH_Unlock();
    sta = EEPROM_WRITE(0x75000, datas, LEN);
    FLASH_Lock();

    if (sta != FLASH_COMPLETE)
        printf("Operation EEPROM_WRITE failed!! Err Code %x\r\n", sta);

    sta = EEPROM_READ(0x75000, temp, LEN);

    if (sta != FLASH_COMPLETE)
        printf("Operation FLASH_ROMA_READ failed!! Err Code %x\r\n", sta);

    if (memcmp(datas, temp, LEN))
    {
        printf("Operation VERIFY failed!!\r\n");
    }

    printf("EEPROM: ");
    for (int var = 0; var < LEN; ++var)
    {
        printf("%x ", datas[var]);
    }
    printf("\n");

#if READPROTECT
    sta = EnableCodeProtection();
    if (sta != FLASH_COMPLETE)
        printf("Set read protect failed!! Err Code %x\r\n", sta);
#endif

    while (1)
    {
    }
}
