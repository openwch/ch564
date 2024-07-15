/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch564_flash.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/11/11
 * Description        : This file provides all the FLASH firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch564_flash.h"

/*********************************************************************
 * @fn      DelayForSPIGeneration
 *
 * @brief   Delay 2 us
 *
 * @return  none
 */
static inline void DelayForSPIGeneration()
{
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
}

/*********************************************************************
 * @fn      Flash_Get_SR1
 *
 * @brief   The Flash_Get_SR1 function can read SR1 Register to make sure the flash is avilable
 *
 * @return  Sr
 */
static uint8_t Flash_Get_SR1()
{
    uint8_t Sr = 0;
    DelayForSPIGeneration();
    R8_SPI_FLASH_CTRL = 0x07;
    DelayForSPIGeneration();
    R8_SPI_FLASH_DATA = 0x05;
    DelayForSPIGeneration();
    Sr = R8_SPI_FLASH_DATA;
    DelayForSPIGeneration();
    Sr = (u8)(R8_SPI_FLASH_DATA & 0xff);
    DelayForSPIGeneration();
    R8_SPI_FLASH_CTRL = 0;

    return Sr;
}

/*********************************************************************
 * @fn      Flash_CMD
 *
 * @brief   The Flash_CMD function performs different operations on a flash memory based on the given command
 *          and address.
 *
 * @param   cmd   The parameter "cmd" is of type FlashCMD, which is an enumeration type. It represents the
 *              command to be executed on the flash memory. The possible values for "cmd" are FastRead and other
 *              commands specific to the flash memory being used.
 * @param   addr  The `addr` parameter is the address of the memory location in the flash memory where the
 *              operation (read or write) will be performed.
 *
 * @return  none
 */
static void Flash_CMD(FlashCMD cmd, uint32_t addr)
{
    while (Flash_Get_SR1() & 0x01)
        ;
    if (cmd == FastRead || cmd == Read || cmd == SecurityRead)
    {
        R8_SPI_FLASH_CTRL = 0x07;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = cmd;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = (addr >> 16) & 0xff;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = (addr >> 8) & 0xff;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = addr & 0xff;
        DelayForSPIGeneration();
    }
    else
    {
        R8_SPI_FLASH_CTRL = 0x03;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = 0x06;
        DelayForSPIGeneration();
        R8_SPI_FLASH_CTRL = 0x00;
        DelayForSPIGeneration();

        R8_SPI_FLASH_CTRL = 0x03;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = cmd;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = (addr >> 16) & 0xff;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = (addr >> 8) & 0xff;
        DelayForSPIGeneration();
        R8_SPI_FLASH_DATA = addr & 0xff;
        DelayForSPIGeneration();
    }
}

/*********************************************************************
 * @fn      Flash_END
 *
 * @brief   The Flash_END function disables the ROM SPI and introduces a delay.
 *
 * @return  none
 */
static void Flash_END()
{
    R8_SPI_FLASH_CTRL = 0;
}

/*********************************************************************
 * @fn      Lock_ZoneB
 *
 * @brief   The function Lock_ZoneB locks a specific zone in flash memory by setting a key code, sending a
 *                command, and writing a buffer of values.
 *
 * @param   KeyCode   The KeyCode parameter is of type uint8_t, which means it is an 8-bit unsigned
 *                  integer. It is used to store the key code that will be written to the R16_FLASHB_KEY_BUF variable.
 *
 * @return  none
 */
static void Lock_ZoneB(uint8_t KeyCode)
{
    R16_FLASHA_KEY_BUF = KeyCode;
    Flash_CMD(SecurityProgram, 0x30fc);
    u8 LockBuffer[4] = {0x45, 0x76, 0x01, 0x32};
    for (u8 j = 0; j < 4; j++)
    {
        R8_SPI_FLASH_DATA = LockBuffer[j];
        DelayForSPIGeneration();
    }
    Flash_END();
}

/*********************************************************************
 * @fn      FlashSecureErase
 *
 * @brief   The FlashSecureErase function performs a secure erase operation on a specified address in flash
 *        memory.
 *
 * @param   address   The address parameter in the FlashSecureErase function is the memory address where
 *                the secure erase operation will be performed.
 *
 * @return  none
 */
static void FlashSecureErase(u32 address)
{
    Flash_CMD(SecurityErase, address);
    Flash_END();
}

/*********************************************************************
 * @fn      FlashEraseSector
 *
 * @brief   The function FlashEraseSector erases a 4K block of flash memory at the specified address.
 *
 * @param   address   The address parameter is the starting address of the block that needs to be erased.
 *
 * @return  none
 */
static void FlashEraseSector(u32 address) // 4K
{
    Flash_CMD(SectorErase, address);
    Flash_END();
}

/*********************************************************************
 * @fn       FlashEraseBlock
 *
 * @brief    The function FlashEraseBlock erases a 32K block of flash memory at the specified address.
 *
 * @param    address   The address parameter is the starting address of the block that needs to be erased.
 *
 * @return   none
 */
static void FlashEraseBlock(u32 address) // 32K
{
    Flash_CMD(BlockErase32K, address);
    Flash_END();
}

/*********************************************************************
 * @fn       FlashEraseBigBlock
 *
 * @brief    The function FlashEraseBigBlock erases a 64K block of flash memory at the specified address.
 *
 * @param    address   The address parameter is the starting address of the block that needs to be erased.
 *
 * @return   none
 */
static void FlashEraseBigBlock(u32 address) // 64K
{
    Flash_CMD(BlockErase64K, address);
    Flash_END();
}

/*********************************************************************
 * @fn       FlashEraseChip
 *
 * @brief    The FlashEraseChip function erases the entire flash memory chip at the specified address.
 *
 * @param    address   The address parameter in the FlashEraseChip function is the starting address of the
 *                   flash memory chip that needs to be erased.
 *                   It's actually useless
 *
 * @return   none
 */
void FlashEraseChip(u32 address)
{
    Flash_CMD(ChipErase, address);
    Flash_END();
}

/*********************************************************************
 * @fn      ChosenFunc
 *
 * @brief   The function ChosenFunc takes an address and a type as input and performs different actions based on
 *        the type, such as erasing a sector, block, or big block in flash memory.
 *
 * @param   addr  The `addr` parameter is an unsigned 32-bit integer that represents the address of the
 *              sector, block, or big block to be erased in the flash memory.
 *          ty    The parameter "ty" is of type uint8_t, which means it is an 8-bit unsigned integer. It is
 *              used as a switch case to determine which operation to perform based on its value.
 *
 * @return  none
 */
static void ChosenFunc(uint32_t addr, uint8_t ty)
{
    switch (ty)
    {
    case 0:
        FlashEraseSector(addr);
        printf("Erase sector %#x\r\n", addr);
        break;
    case 1:
        FlashEraseBlock(addr);
        printf("Erase block %#x\r\n", addr);
        break;
    case 2:
        FlashEraseBigBlock(addr);
        printf("Erase bigblock %#x\r\n", addr);
        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      FlashErase
 *
 * @brief   The function FlashErase erases a range of flash memory by dividing it into smaller chunks and
 *        calling a chosen function for each chunk.
 *
 * @param   address   The `address` parameter represents the starting address in the flash memory from
 *                  where the erasing operation should begin. It is of type `uint32_t`, which means it is an unsigned
 *                  32-bit integer.
 *          datasize  The `datasize` parameter represents the size of the data that needs to be erased in
 *                  the flash memory.
 *
 * @return  none
 */

void FlashErase(uint32_t address, uint32_t datasize, FlashCFG config)
{
    if (config & (ZoneA | ZoneB))
    {
        FlashSecureErase(address);
    }
    else
    {
        uint32_t AddressCtr = (address + address + datasize) / 2, AddressLft = 0, AddressRit = 0,
                 FlashScale[] = {0x1000, 0x4000, 0x10000};
        uint8_t centerScale = 0, sideScale = 0;
        while (centerScale < (sizeof(FlashScale) / sizeof(uint32_t) + 1))
        {
            if (!(address < FlashScale[centerScale] * (AddressCtr / FlashScale[centerScale]) &&
                  FlashScale[centerScale] * (AddressCtr / FlashScale[centerScale] + 1) < address + datasize))
            {
                if (centerScale != 0)
                    centerScale--;
                break;
            }
            centerScale++;
        }
        ChosenFunc(AddressCtr, centerScale);

        AddressLft = FlashScale[centerScale] * (AddressCtr / FlashScale[centerScale]);
        AddressRit = FlashScale[centerScale] * (AddressCtr / FlashScale[centerScale] + 1);
        sideScale = centerScale;
        while (sideScale > 0)
        {
            if (AddressLft < FlashScale[sideScale] ||
                AddressLft - FlashScale[sideScale] < address * (address / FlashScale[sideScale]))
            {
                sideScale--;
            }
            else
            {
                AddressLft -= FlashScale[sideScale];
                ChosenFunc(AddressLft, sideScale);
            }
        }
        while (AddressLft > address)
        {
            AddressLft -= FlashScale[sideScale];
            ChosenFunc(AddressLft, sideScale);
        }
        sideScale = centerScale;
        while (sideScale > 0)
        {
            if (AddressRit + FlashScale[sideScale] > (address + datasize))
            {
                sideScale--;
            }
            else
            {
                AddressRit += FlashScale[sideScale];
                ChosenFunc(AddressRit - FlashScale[sideScale], sideScale);
            }
        }
        while (AddressRit < address + datasize)
        {
            AddressRit += FlashScale[sideScale];
            ChosenFunc(AddressRit - FlashScale[sideScale], sideScale);
        }
    }
}

/*******************************************************
 * @fn       FlashSecurityWritefunc
 *
 * @brief    The function FlashWritefunc writes data to a flash memory starting at a specified address.
 *
 * @param    address       The starting address in the flash memory where the data will be written.
 *           dataAddress   A pointer to the start address of the data that needs to be written to the flash
 *                       memory.
 *           datasize      The parameter "datasize" represents the size of the data that needs to be written to
 *                       the flash memory.
 *
 * @return   none
 */

void FlashSecurityWritefunc(uint32_t address, uint8_t *dataAddress, uint32_t datasize)
{

    uint32_t offset = 0;
    offset = address % 256;

    Flash_CMD(SecurityProgram, address);
    if (datasize > (256 - offset))
    {
        for (uint16_t i = 0; i < 256 - offset; i++)
        {
            R8_SPI_FLASH_DATA = *(dataAddress++);
            DelayForSPIGeneration();
        }
    }
    else
    {
        for (uint16_t i = 0; i < datasize; i++)
        {
            R8_SPI_FLASH_DATA = *(dataAddress++);
            DelayForSPIGeneration();
        }
    }
    Flash_END();
}

/*******************************************************
 * @fn       FlashWritefunc
 *
 * @brief    The function FlashWritefunc writes data to a flash memory starting at a specified address.
 *
 * @param    address       The starting address in the flash memory where the data will be written.
 *           dataAddress   A pointer to the start address of the data that needs to be written to the flash
 *                       memory.
 *           datasize      The parameter "datasize" represents the size of the data that needs to be written to
 *                       the flash memory.
 *
 * @return   none
 */

 void FlashWritefunc(uint32_t address, uint8_t *dataAddress, uint32_t datasize)
{

    uint32_t offset = 0;
    offset = address % 256;
    if (datasize <= (256 - offset))
    {
        Flash_CMD(PageProgram, address);
        for (uint16_t i = 0; i < datasize; i++)
        {
            R8_SPI_FLASH_DATA = *(dataAddress++);
            DelayForSPIGeneration();
        }
        Flash_END();
    }
    else
    {
        Flash_CMD(PageProgram, address);
        for (uint16_t i = 0; i < 256 - offset; i++)
        {
            R8_SPI_FLASH_DATA = *(dataAddress++);
            DelayForSPIGeneration();
        }
        Flash_END();

        for (uint16_t j = 0; j < (datasize - 256 + offset) / 256; j++)
        {
            Flash_CMD(PageProgram, address + 256 - offset + j * 256);
            for (uint16_t i = 0; i < 256; i++)
            {
                R8_SPI_FLASH_DATA = *(dataAddress++);
                DelayForSPIGeneration();
            }
            Flash_END();
        }
        Flash_CMD(PageProgram, ((address + datasize) / 256 * 256));

        for (uint16_t i = 0; i < (datasize + datasize) % 256; i++)
        {
            R8_SPI_FLASH_DATA = *(dataAddress++);
            DelayForSPIGeneration();
        }
        Flash_END();
    }
}

/*********************************************************************
 * @fn      FlashReadDataFast
 *
 * @brief   The function FlashReadDataFast reads data from a flash memory chip using SPI communication.
 *
 * @param   address  The address parameter is the starting address from where the data needs to be read
 *                 from the flash memory.
 *          datasize The `datasize` parameter represents the size of the data to be read from the flash
 *                 memory.
 *          databuff A pointer to the buffer where the read data will be stored.
 *
 * @return  none
 */
void FlashReadDataFast(uint32_t address, uint32_t datasize, uint8_t *databuff)
{
    __attribute__((__unused__)) uint8_t tmp = 0;

        Flash_CMD(FastRead, address);

        tmp = R8_SPI_FLASH_DATA;
        DelayForSPIGeneration();
        tmp = R8_SPI_FLASH_DATA;
        DelayForSPIGeneration();

        for (int j = 0; j < datasize; j++)
        {
            *databuff ++ = R8_SPI_FLASH_DATA;
            DelayForSPIGeneration();
        }
        Flash_END();
}

/*********************************************************************
 * @fn      FlashReadData
 *
 * @brief   The function FlashReadData reads data from a flash memory chip using SPI communication.
 *
 * @param   address  The address parameter is the starting address from where the data needs to be read
 *                 from the flash memory.
 *          datasize The `datasize` parameter represents the size of the data to be read from the flash
 *                 memory.
 *          databuff A pointer to the buffer where the read data will be stored.
 *
 * @return  none
 */
void FlashReadData(uint32_t address, uint32_t datasize, uint8_t *databuff)
{
    __attribute__((__unused__)) uint8_t tmp = 0;

    Flash_CMD(Read, address);

    tmp = R8_SPI_FLASH_DATA;
    DelayForSPIGeneration();
    
    for (int j = 0; j < datasize; j++)
    {
        *databuff ++ = R8_SPI_FLASH_DATA;
        DelayForSPIGeneration();
    }
    Flash_END();
}
/*********************************************************************
 * @fn      FlashSecurityReadData
 *
 * @brief   The function FlashReadData reads data from a flash memory chip using SPI communication.
 *
 * @param   address  The address parameter is the starting address from where the data needs to be read
 *                 from the flash memory.
 *          datasize The `datasize` parameter represents the size of the data to be read from the flash
 *                 memory.
 *          databuff A pointer to the buffer where the read data will be stored.
 *
 * @return  none
 */
void FlashSecurityReadData(uint32_t address, uint32_t datasize, uint8_t *databuff)
{
    __attribute__((__unused__)) uint8_t tmp = 0;
        Flash_CMD(SecurityRead, address);

        tmp = R8_SPI_FLASH_DATA;
        DelayForSPIGeneration();
        tmp = R8_SPI_FLASH_DATA;
        DelayForSPIGeneration();

        for (int j = 0; j < datasize; ++j)
        {
            *(databuff + j) = (u8)(R8_SPI_FLASH_DATA & 0xff);
            DelayForSPIGeneration();
        }
        Flash_END();
}

/*********************************************************
 * @fn       FlashWriteData
 *
 * @brief    The function FlashWriteData writes data to a specified address in flash memory, with optional
 *           security features enabled based on the provided configuration.
 *
 * @param    address     The address parameter is the starting address in the flash memory where the data will
 *                     be written to.
 *           dataAddress A pointer to the start address of the data to be written to flash memory.
 *           datasize    The parameter "datasize" is of type uint16_t and represents the size of the data to
 *                     be written to the flash memory.
 *           config      The "config" parameter is a bitmask that specifies the configuration options for the
 *                     flash write operation. It can have the following values:
 *           Keycode     The Keycode parameter is a 16-bit value used for authentication or authorization
 *                     purposes. It is typically used to ensure that only authorized users can perform certain
 *                     operations, such as erasing or writing data to the flash memory.
 * @warning  If you use this function to write the sucury zone(0x2000 to 0x20ff or 0x3000 to 0x30ff),the
 *         function will unlock the flash for only once, so make sure that the beginning address is between sucury zone
 * @return   none
 */
void FlashWriteData(uint32_t address, uint8_t *dataAddress, uint32_t datasize, FlashCFG config, uint16_t Keycode)
{

    if (config & (ZoneA | ZoneB))
    {
        FLASH_SECUR_UNLOCK(config&(ZoneA|ZoneB),Keycode);
        FlashErase(address, datasize, config);
        FLASH_SECUR_UNLOCK(config&(ZoneA|ZoneB),Keycode);
        FlashSecurityWritefunc(address, dataAddress, datasize);
        if (config == (ZoneB | LockB))
        {
            Lock_ZoneB(Keycode);
        }
    }
    else
    {
        FlashErase(address, datasize, config);
        FlashWritefunc(address, dataAddress, datasize);
    }
}
