/********************************** (C) COPYRIGHT *******************************
 * File Name          : SPI_FLAH.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/05/16
 * Description        : SPI FLASHоƬ��������ļ�
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include <SPI_FLASH.h>
#include <SW_UDISK.h>
#include "ch564_spi.h"

/******************************************************************************/
/* ������������ */
volatile uint8_t   Flash_Type = 0x00;                                           /* FLASHоƬ����: 0: W25XXXϵ�� */
volatile uint32_t  Flash_ID = 0x00;                                             /* FLASHоƬID�� */
volatile uint32_t  Flash_Sector_Count = 0x00;                                   /* FLASHоƬ������ */
volatile uint16_t  Flash_Sector_Size = 0x00;                                    /* FLASHоƬ������С */

/*******************************************************************************
* Function Name  : FLASH_Port_Init
* Description    : FLASHоƬ����������ź�Ӳ����ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_Port_Init( void )
{
    GPIOB_ModeCfg(MOSI | SCK0 | SCS, GPIO_ModeOut_PP);
    GPIOB_ModeCfg(MISO, GPIO_ModeIN_Floating);
    SPI0_MasterInit(2400000);
}

/*********************************************************************
 * @fn      SPI0_ReadWriteByte
 *
 * @brief   SPI1 read or write one byte.
 *
 * @param   TxData - write one byte data.
 *
 * @return  Read one byte data.
 */
uint8_t SPI0_ReadWriteByte(uint8_t TxData)
{
    uint8_t recdata = 0;
    SPI0_MasterTransRecv(&TxData, &recdata, 1);
    return recdata;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : SPI����һ���ֽ�����
* Input          : byte: Ҫ���͵��ֽ�
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t SPI_FLASH_SendByte( uint8_t byte )
{
    return SPI0_ReadWriteByte( byte );
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : SPI����һ���ֽ�����
* Input          : None
* Output         : None
* Return         : ���ؽ��յ��ֽ�����
*******************************************************************************/
uint8_t SPI_FLASH_ReadByte( void )
{
    return SPI0_ReadWriteByte( 0xFF );
}

/*******************************************************************************
* Function Name  : FLASH_ReadID
* Description    : ��ȡFLASHоƬID
* Input          : None
* Output         : None
* Return         : ����4���ֽ�,����ֽ�Ϊ0x00,
*                  �θ��ֽ�ΪManufacturer ID(0xEF),
*                  �ε��ֽ�ΪMemory Type ID
*                  ����ֽ�ΪCapacity ID
*                  W25X40BL����: 0xEF��0x30��0x13
*                  W25X10BL����: 0xEF��0x30��0x11
*******************************************************************************/
uint32_t FLASH_ReadID( void )
{
    uint32_t dat;
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_JEDEC_ID );
    dat = ( uint32_t )SPI_FLASH_SendByte( DEF_DUMMY_BYTE ) << 16;
    dat |= ( uint32_t )SPI_FLASH_SendByte( DEF_DUMMY_BYTE ) << 8;
    dat |= SPI_FLASH_SendByte( DEF_DUMMY_BYTE );
    PIN_FLASH_CS_HIGH( );
    return( dat );
}

/*******************************************************************************
* Function Name  : FLASH_WriteEnable
* Description    : FLASHоƬ����д����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_WriteEnable( void )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_WREN );
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : FLASH_WriteDisable
* Description    : FLASHоƬ��ֹд����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_WriteDisable( void )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_WRDI );
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : FLASH_ReadStatusReg
* Description    : FLASHоƬ��ȡ״̬�Ĵ��� 
* Input          : None
* Output         : None
* Return         : ���ؼĴ���ֵ
*******************************************************************************/
uint8_t FLASH_ReadStatusReg( void )
{
    uint8_t  status;

    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_RDSR );
    status = SPI_FLASH_ReadByte( );
    PIN_FLASH_CS_HIGH( );
    return( status );
}

/*******************************************************************************
* Function Name  : FLASH_Erase_Sector
* Description    : FLASH�������ݲ���
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/ 
void FLASH_Erase_Sector( uint32_t address )
{
    uint8_t  temp;
    FLASH_WriteEnable( );
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_SECTOR_ERASE );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
    PIN_FLASH_CS_HIGH( );
    do
    {
        temp = FLASH_ReadStatusReg( );    
    }while( temp & 0x01 );
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block_Start
* Description    : FLASH�����ݶ�ȡ��ʼ
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/  
void FLASH_RD_Block_Start( uint32_t address )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_READ );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block
* Description    : FLASH�����ݶ�ȡ
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/  
void FLASH_RD_Block( uint8_t *pbuf, uint32_t len )
{
    while( len-- )                                                             
    {
        *pbuf++ = SPI_FLASH_ReadByte( );
    }
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block_End
* Description    : FLASH�����ݶ�ȡ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_RD_Block_End( void )
{
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : W25XXX_WR_Page
* Description    : д��һ������
*                  ע����ǰ����֧�ֿ�ҳд����      
* Input          : address----׼��д���׵�ַ
*                  len--------׼��д�����ݳ���
*                  *pbuf------׼��д��Ļ�������ַ 
* Output         : None
* Return         : None
*******************************************************************************/
void W25XXX_WR_Page( uint8_t *pbuf, uint32_t address, uint32_t len )
{
    uint8_t  temp;
    FLASH_WriteEnable( );
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_BYTE_PROG );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
    if( len > SPI_FLASH_PerWritePageSize )
    {
        len = SPI_FLASH_PerWritePageSize;
    }
    while( len-- )
    {
        SPI_FLASH_SendByte( *pbuf++ );
    }
    PIN_FLASH_CS_HIGH( );
    do
    {
        temp = FLASH_ReadStatusReg( );    
    }while( temp & 0x01 );
}

/*******************************************************************************
* Function Name  : W25XXX_WR_Block
* Description    : д��һ������
* Input          : address----׼��д���׵�ַ
*                  len--------׼��д�����ݳ���
*                  *pbuf------׼��д��Ļ�������ַ
* Output         : None
* Return         : None
*******************************************************************************/
void W25XXX_WR_Block( uint8_t *pbuf, uint32_t address, uint32_t len )
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = address % SPI_FLASH_PageSize;
    count = SPI_FLASH_PageSize - Addr;
    NumOfPage =  len / SPI_FLASH_PageSize;
    NumOfSingle = len % SPI_FLASH_PageSize;

    if( Addr == 0 )
    {
        if( NumOfPage == 0 )
        {
            W25XXX_WR_Page( pbuf, address, len );
        }
        else
        {
            while( NumOfPage-- )
            {
                W25XXX_WR_Page( pbuf, address, SPI_FLASH_PageSize );
                address +=  SPI_FLASH_PageSize;
                pbuf += SPI_FLASH_PageSize;
            }
            W25XXX_WR_Page( pbuf, address, NumOfSingle );
        }
    }
    else
    {
        if( NumOfPage == 0 )
        {
            if( NumOfSingle > count )
            {
                temp = NumOfSingle - count;
                W25XXX_WR_Page( pbuf, address, count );
                address +=  count;
                pbuf += count;
                W25XXX_WR_Page( pbuf, address, temp );
            }
            else
            {
                W25XXX_WR_Page( pbuf, address, len );
            }
        }
        else
        {
            len -= count;
            NumOfPage =  len / SPI_FLASH_PageSize;
            NumOfSingle = len % SPI_FLASH_PageSize;
            W25XXX_WR_Page( pbuf, address, count );
            address +=  count;
            pbuf += count;
            while( NumOfPage-- )
            {
                W25XXX_WR_Page( pbuf, address, SPI_FLASH_PageSize );
                address += SPI_FLASH_PageSize;
                pbuf += SPI_FLASH_PageSize;
            }
            if( NumOfSingle != 0 )
            {
                W25XXX_WR_Page( pbuf, address, NumOfSingle );
            }
        }
    }
}



/*******************************************************************************
* Function Name  : FLASH_IC_Check
* Description    : FLASHоƬ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IC_Check( void )
{
    uint32_t count;

    /* ��ȡFLASHоƬID�� */    
    Flash_ID = FLASH_ReadID( );
    printf("Flash_ID: %08x\n",(uint32_t)Flash_ID);

    /* ����оƬ�ͺ�,�ж�������С */
    Flash_Type = 0x00;                                                                
    Flash_Sector_Count = 0x00;
    Flash_Sector_Size = 0x00;

    switch( Flash_ID )
    {
        /* W25XXXϵ��оƬ */
        case W25X10_FLASH_ID:                                                   /* 0xEF3011-----1M bit */
            count = 1;
            break;

        case W25X20_FLASH_ID:                                                   /* 0xEF3012-----2M bit */
            count = 2;
            break;

        case W25X40_FLASH_ID:                                                   /* 0xEF3013-----4M bit */
            count = 4;
            break;

        case W25X80_FLASH_ID:                                                   /* 0xEF4014-----8M bit */
            count = 8;
            break;

        case W25Q16_FLASH_ID1:                                                  /* 0xEF3015-----16M bit */
        case W25Q16_FLASH_ID2:                                                  /* 0xEF4015-----16M bit */
            count = 16;
            break;

        case W25Q32_FLASH_ID1:                                                  /* 0xEF4016-----32M bit */
        case W25Q32_FLASH_ID2:                                                  /* 0xEF6016-----32M bit */
            count = 32;
            break;

        case W25Q64_FLASH_ID1:                                                  /* 0xEF4017-----64M bit */
        case W25Q64_FLASH_ID2:                                                  /* 0xEF6017-----64M bit */
            count = 64;
            break;

        case W25Q128_FLASH_ID1:                                                 /* 0xEF4018-----128M bit */
        case W25Q128_FLASH_ID2:                                                 /* 0xEF6018-----128M bit */
            count = 128;
            break;

        case W25Q256_FLASH_ID1:                                                 /* 0xEF4019-----256M bit */
        case W25Q256_FLASH_ID2:                                                 /* 0xEF6019-----256M bit */
            count = 256;
            break;
         default:
             if( ( Flash_ID != 0xFFFFFFFF ) || ( Flash_ID != 0x00000000 ) )
             {
                count = 16;
             }
             else
             {
                 count = 0x00;
            }
            break;
    }
    count = ( (uint32_t)count * 1024 ) * ( (uint32_t)1024 / 8 );

    /* ����FLASHоƬ������Ŀ,������С����Ϣ */
    if( count )
    {
        Flash_Sector_Count = count / DEF_UDISK_SECTOR_SIZE;
        Flash_Sector_Size = DEF_UDISK_SECTOR_SIZE;
    }
}

