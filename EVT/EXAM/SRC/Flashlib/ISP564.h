/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ISP564.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/05
 * Description        : This file contains all the functions prototypes for the
 *                      FLASH firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __ISP564_H
#define __ISP564_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

/* FLASH Status */
typedef enum
{
    FLASH_COMPLETE,
    FLASH_TIMEOUT,
    FLASH_VERIFY_ERROR,
    FLASH_ADR_RANGE_ERROR,
    FLASH_UNLOCK_ERROR,
}FLASH_Status;

/*********************************************************************
 * @fn      FLASH_Unlock
 *
 * @brief   Unlocks the FLASH Program and Erase Controller.
 *
 * @return  none
 */
extern void FLASH_Unlock(void);

/*********************************************************************
 * @fn      FLASH_Lock
 *
 * @brief   Locks the FLASH Program and Erase Controller.
 *
 * @return  none
 */
extern void FLASH_Lock(void);

/*********************************************************************
 * @fn      GetMACAddress
 *
 * @brief   Get MAC address(6Bytes)
 *
 * @param   Buffer - Pointer to the buffer where data should be stored,
 *        Must be aligned to 4 bytes.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT.
 */
extern FLASH_Status GetMACAddress( void *Buffer );

/*********************************************************************
 * @fn      GET_UNIQUE_ID
 *
 * @brief   Get unique ID(8Bytes)
 *
 * @param   Buffer - Pointer to the buffer where data should be stored,
 *        Must be aligned to 4 bytes.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT.
 */
extern FLASH_Status GET_UNIQUE_ID( void *Buffer );

/*********************************************************************
 * @fn      GetCHIPID
 *
 * @brief   Get chip ID(4Bytes)
 *
 * @param   Buffer - Pointer to the buffer where data should be stored,
 *        Must be aligned to 4 bytes.
 *            ChipID List-
 *              CH564L-0x564005x8
 *              CH564Q-0x564105x8
 *              CH564D-0x564305x8
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT.
 */
extern FLASH_Status GetCHIPID( void *Buffer );

/*********************************************************************
 * @fn      Get_Flash_Size
 *
 * @brief   Get FLASH Size(1Bytes)
 *
 * @param   Buffer - Pointer to the buffer where data should be stored.
 *            0 - FLASH-256K
 *                ROMA(UserFLASH)
 *                  - Size(192K)
 *                  - Address range(0x0 -- 0x2FFFF)
 *                EEPROM(DataFLASH)
 *                  - Size(32K)
 *                  - Address range(0x30000 -- 0x37FFF)
 *            1 - FLASH-512K
 *                ROMA(UserFLASH)
 *                  - Size(448K)
 *                  - Address range(0x0 -- 0x6FFFF)
 *                EEPROM(DataFLASH)
 *                  - Size(32K)
 *                  - Address range(0x70000 -- 0x77FFF)
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT.
 */
extern FLASH_Status Get_Flash_Size( void *Buffer );

/*********************************************************************
 * @fn      FLASH_EnableCodeProtection
 *
 * @brief   Enables the code protection.
 *
 * @return  FLASH Status - The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
extern FLASH_Status FLASH_EnableCodeProtection( void );

/*********************************************************************
 * @fn      FLASH_ROM_PWR_UP
 *
 * @brief   The function `FLASH_ROM_PWR_DOWN` sends a command to put
 *        the SPI flash memory into power down mode.
 *
 * @return  none
 */
extern void FLASH_ROM_PWR_DOWN( void );

/*********************************************************************
 * @fn      FLASH_ROM_PWR_UP
 *
 * @brief   The function `FLASH_ROM_PWR_UP` sets up the SPI flash
 *        control register to power up the flash memory
 *
 * @return  none
 */
extern void FLASH_ROM_PWR_UP( void );

/*********************************************************************
 * @fn      EEPROM_READ
 *
 * @brief   (DataFLASH) - The EEPROM_READ function reads data from a specified address
 *        in flash memory with error handling for address range checks.
 *
 * @param   StartAddr - Read the starting address of the DataFLASH.
 *          Buffer - Read the value of the DataFLASH.
 *          Length - Read the length of the DataFLASH.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR.
 */
extern FLASH_Status EEPROM_READ( uint32_t StartAddr, void *Buffer, uint32_t Length );

/*********************************************************************
 * @fn      EEPROM_ERASE
 *
 * @brief   (DataFLASH) - The function EEPROM_ERASE checks the flash size and address
 *        range before erasing a specified portion of flash memory.
 *
 * @param   StartAddr - Erases the starting address of the DataFLASH(StartAddr%4096 == 0).
 *          Length - Erases the length of the DataFLASH(Length%4096 == 0).
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR,FLASH_UNLOCK_ERROR.
 */
extern FLASH_Status EEPROM_ERASE( uint32_t StartAddr, uint32_t Length );

/*********************************************************************
 * @fn      EEPROM_WRITE
 *
 * @brief   (DataFLASH) - The function EEPROM_WRITE writes data to EEPROM memory
 *        based on specified address and length, performing address
 *        range and unlock checks.
 *
 * @param   StartAddr - Writes the starting address of the DataFLASH.
 *          Buffer - Writes the value of the DataFLASH.
 *          Length - Writes the length of the DataFLASH.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR,FLASH_UNLOCK_ERROR.
 */
extern FLASH_Status EEPROM_WRITE( uint32_t StartAddr, void *Buffer, uint32_t Length );

/*********************************************************************
 * @fn      FLASH_ROMA_ERASE
 *
 * @brief   (UserFLASH) - The function `FLASH_ROMA_ERASE` checks the flash size and
 *        address range before erasing a specified portion of flash
 *        memory.
 *
 * @param   StartAddr - Erases the starting address of the UserFLASH(StartAddr%4096 == 0).
 *          Length - Erases the length of the UserFLASH(Length%4096 == 0).
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR,FLASH_UNLOCK_ERROR.
 */
extern FLASH_Status FLASH_ROMA_ERASE( uint32_t StartAddr, uint32_t Length );

/*********************************************************************
 * @fn      FLASH_ROMA_WRITE
 *
 * @brief   (UserFLASH) - The function FLASH_ROMA_WRITE writes data to a specific
 *        flash memory address after performing size and unlock checks.
 *
 * @param   StartAddr - Writes the starting address of the UserFLASH.
 *          Buffer - Writes the value of the UserFLASH.
 *          Length - Writes the length of the UserFLASH.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR,FLASH_UNLOCK_ERROR.
 */
extern FLASH_Status FLASH_ROMA_WRITE( uint32_t StartAddr, void *Buffer, uint32_t Length );

/*********************************************************************
 * @fn      FLASH_ROMA_VERIFY
 *
 * @brief   (UserFLASH) - The function `FLASH_ROMA_VERIFY` verifies the contents of
 *        a specified flash memory region against a provided buffer.
 *
 * @param   StartAddr - Verify the starting address of the UserFLASH.
 *          Buffer - Verify the value of the UserFLASH.
 *          Length - Verify the length of the UserFLASH.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR,FLASH_VERIFY_ERROR.
 */
extern FLASH_Status FLASH_ROMA_VERIFY( uint32_t StartAddr, void *Buffer, uint32_t Length );

/*********************************************************************
 * @fn      FLASH_ROMA_READ
 *
 * @brief   (UserFLASH) - The function `FLASH_ROMA_READ` reads data from a specific
 *        flash memory address with error handling for different flash
 *        size
 *
 * @param   StartAddr - Read the starting address of the UserFLASH.
 *          Buffer - Read the value of the UserFLASH.
 *          Length - Read the length of the UserFLASH.
 *
 * @return  FLASH_Status -The returned value can be: FLASH_COMPLETE,
 *        FLASH_TIMEOUT,FLASH_ADR_RANGE_ERROR.
 */
extern FLASH_Status FLASH_ROMA_READ( uint32_t StartAddr, void *Buffer, uint32_t Length );



#ifdef __cplusplus
}
#endif

#endif
