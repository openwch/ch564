/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_flash.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file contains all the functions prototypes for the
 *                      FLASH firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH564_FLASH_H
#define __CH564_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ch564.h"

typedef enum
{
    None = 0,
    Show = 1,
    ZoneA = 2,
    ZoneB = 4,
    LockB = 8,
    RandomX = 16
} FlashCFG;

typedef enum
{
    WriteEnable = 0x60,
    PageProgram = 0x02,
    SectorErase = 0x20,
    BlockErase32K = 0x52,
    BlockErase64K = 0xD8,
    SecurityErase = 0x44,
    SecurityProgram = 0x42,
    SecurityRead = 0x48,
    ChipErase = 0xC7,
    FastRead = 0x0b,
    Read = 0x03
} FlashCMD;

#define FLASH_SECUR_UNLOCK(zone,key) ((zone) & ZoneA) ? (R16_FLASHA_KEY_BUF = (key)):(R16_FLASHB_KEY_BUF = (key))

void FlashErase(uint32_t address, uint32_t datasize, FlashCFG config);
void FlashWriteData(uint32_t address, uint8_t *dataAddress, uint32_t datasize, FlashCFG config, uint16_t Keycode);
void FlashSecurityWritefunc(uint32_t address, uint8_t *dataAddress, uint32_t datasize);
void FlashSecurityReadData(uint32_t address, uint32_t datasize, uint8_t *databuff);
void FlashReadDataFast(uint32_t address, uint32_t datasize, uint8_t *databuff);
void FlashReadData(uint32_t address, uint32_t datasize, uint8_t *databuff);
void FlashWritefunc(uint32_t address, uint8_t *dataAddress,uint32_t datasize);

#ifdef __cplusplus
}
#endif

#endif
