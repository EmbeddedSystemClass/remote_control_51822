/*
 * drv_Qspi.c
 *
 *  Created on: 2015年7月30日
 *      Author: Arvin
 * @version V1.0.0
 * @date    25-June-2015
 * @brief   This file includes a standard driver for the N25Q128A QSPI
 *          memory mounted on STM32746G-Discovery board.
 @verbatim
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 [..]
 (#) This driver is used to drive the N25Q128A QSPI external


 (#) This driver need a specific component driver (N25Q128A) to be included with.

 (#) Initialization steps:
 (++) Initialize the QPSI external memory using the drv_QspiInit() function. This
 function includes the MSP layer hardware resources initialization and the
 QSPI interface with the external memory.

 (#) QSPI memory operations
 (++) QSPI memory can be accessed with read/write operations once it is
 initialized.
 Read/write operation can be performed with AHB access using the functions
 drv_QspiRead()/drv_QspiWrite().
 (++) The function drv_QspiGetInfo() returns the configuration of the QSPI memory.
 (see the QSPI memory data sheet)
 (++) Perform erase block operation using the function drv_QspiErase_Block() and by
 specifying the block address. You can perform an erase operation of the whole
 chip by calling the function drv_QspiErase_Chip().
 (++) The function drv_QspiGetStatus() returns the current status of the QSPI memory.
 (see the QSPI memory data sheet)

 */

/* Includes ------------------------------------------------------------------*/
#include "drv_FlashSpi.h"
#ifdef NOR_FLASH_PLUG_IN
#include <string.h>
#include "../drv_SpiComDef.h"
static SPI_HandleTypeDef hFlashSpi;

#define EN_FLASH_SPI_CMU()      	 HAL_GPIO_WritePin(SPI_ROM_NSS_PORT,SPI_ROM_NSS_PIN,GPIO_PIN_RESET)
#define DISABLE_FLASH_SPI_CMU()      HAL_GPIO_WritePin(SPI_ROM_NSS_PORT,SPI_ROM_NSS_PIN,GPIO_PIN_SET)
#define SPI_CMU_TIME_OUT      		 500 			//ms
#define W25Q128_ID					 0x00EF4018
static HAL_StatusTypeDef
drv_FlashEraseSector (u32 add);
static HAL_StatusTypeDef
drv_FlashWriteEn (void);




/**
 * @brief  Initializes the QSPI interface.
 * @retval QSPI memory status
 */
HAL_StatusTypeDef drv_FlashSpiInit (void)
{
	u32 id=0;
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_COMInitTypeDef spiFlashInit;



  GPIO_InitStruct.Pin = SPI_ROM_NSS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init (SPI_ROM_NSS_PORT, &GPIO_InitStruct);


  spiFlashInit.CLKPhase=SPI_PHASE_1EDGE;
  spiFlashInit.CLKPolarity=SPI_POLARITY_LOW;
  spiFlashInit.DataSize=SPI_DATASIZE_8BIT;
  spiFlashInit.Mode=SPI_MODE_MASTER;
  Spi3Drv.Init(&spiFlashInit);

  id=drv_FlashGetId();
  if(id==W25Q128_ID)
    {
      return HAL_OK;
    }
  else
    {
      return HAL_ERROR;
    }
}

/**
 * @brief  ENABLE program flash.
 * @param  None
 * @retval None
 */
static HAL_StatusTypeDef drv_FlashWriteEn ()
{
  u8 addBuf = WRITE_ENABLE_CMD;
  EN_FLASH_SPI_CMU();

  if (Spi3Drv.Write (&addBuf, 1) != HAL_OK)
    {
      return HAL_ERROR;
    }

  DISABLE_FLASH_SPI_CMU();
  return HAL_OK;
}
/**
 * @brief  erase a 4kb sector
 * @param  add: pointer to the first address of sector .
 * @param  NumByteToWrite: Number of bytes to write.
 * @retval None
 */
static HAL_StatusTypeDef
drv_FlashEraseSector (u32 add)
{
  u8 addBuf[3] =
    { 0 };

  if (drv_FlashWriteEn () != HAL_OK) /* 发送写使能命令 */
    {
      return HAL_ERROR;
    }
  EN_FLASH_SPI_CMU(); /* 使能片选 */
  addBuf[0] = SUBSECTOR_ERASE_CMD;/* 发送PAGE prog命令(地址自动增加编程) */
  if (Spi3Drv.Write (&addBuf[0], 1) != HAL_OK)
    {
      return HAL_ERROR;
    }
  addBuf[0] = (add & 0xFF0000) >> 16;
  addBuf[1] = (add & 0xFF00) >> 8;
  addBuf[2] = add & 0xFF;
  if (Spi3Drv.Write (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
    {
      return HAL_ERROR;
    }

  DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */

  do
    {
      vTaskDelay (100);/*等待100ms 确定器件的4kb 擦出完成*/
    }
  while (drv_FlashIsBusy ());
  return HAL_OK;
}

/*
 * W25Q128 id=0xEF17
 *
 * */
u32
drv_FlashGetId ()
{
  u32 flashId = 0;
  u8 addBuf[3] =
    { 0 };
  EN_FLASH_SPI_CMU(); /* 使能片选 */
  Spi3Drv.WriteRead (0x9F);
  if (Spi3Drv.Read (addBuf, 3) != HAL_OK)/* 读一个字节并存储到pBuf，读完后指针自加1 */
    {
      return HAL_ERROR;
    }
  flashId = addBuf[0];
  flashId = (flashId << 8) | addBuf[1];
  flashId = (flashId << 8) | addBuf[2];

  DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */
  return flashId;
}

/**
 * @brief  Check the flash is busy.
 * @param  n/a
 * @retval   flash busy? TRUE:FALSE
 */

BOOL
drv_FlashIsBusy ()
{
  EN_FLASH_SPI_CMU(); /* 使能片选 */
  Spi3Drv.WriteRead (READ_STATUS_REG_CMD);
  if (Spi3Drv.WriteRead (DUMMY_BYTE) & W25Q128F_STATUS1_BUSY)
    {
      DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */
      return TRUE;
    }
  DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */
  return FALSE;
}

/**
 * @brief  drv_FlashWritePage 按整页（256 bytes）的方式写入数据,擦出按照扇区的方式
 * @param  pBuffer: pointer to the buffer containing the data to be written to the Flash.
 * @param  WriteAddr: Flash's internal address to write to.
 * @param  length: Number of bytes to write.
 * @retval None
 */
HAL_StatusTypeDef
drv_FlashWritePage (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length)
{
  u32 i = 0, j = 0;
  u8 addBuf[3] =
    { 0 };
  if ((length % N25Q128A_SUBSECTOR_SIZE) != 0) /* 整个扇区都改写 */
    {
      return HAL_ERROR;/*此驱动用于FATFS文件系统，定义扇区大小为4096*/
    }
  /*擦出扇区*/
  for (i = 0; i < (length >>12) ; i++)/*有几个扇区*/
    {
      if (drv_FlashEraseSector (writeAddr+i*4096) != HAL_OK) /* 擦出扇区 */
	{
	  return HAL_ERROR;
	}
    }
      for (j = 0; j < length / 256; j++)
	{
	  if (drv_FlashWriteEn () != HAL_OK) /* 发送写使能命令 */
	    {
	      return HAL_ERROR;
	    }

	  EN_FLASH_SPI_CMU(); /* 使能片选 */
	  addBuf[0] = PAGE_PROG_CMD;/* 发送PAGE prog命令(地址自动增加编程) */
	  if (Spi3Drv.Write  (&addBuf[0], 1) != HAL_OK)
	    {
	      return HAL_ERROR;
	    }
	  addBuf[0] = (writeAddr & 0xFF0000) >> 16;
	  addBuf[1] = (writeAddr & 0xFF00) >> 8;
	  addBuf[2] = writeAddr & 0xFF;
	  if (Spi3Drv.Write  (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
	    {
	      return HAL_ERROR;
	    }

	  if (Spi3Drv.Write  ((pBuffer + j * 256), 256) != HAL_OK)/* 发送数据 */
	    {
	      return HAL_ERROR;
	    }

	  DISABLE_FLASH_SPI_CMU(); /* 禁止片选 */

	  while (drv_FlashIsBusy ())
	    {
	      vTaskDelay (2);/*等待1ms 确认写操作完成*/
	    }

	  writeAddr += 256;

	}


  /* 进入写保护状态 */
  EN_FLASH_SPI_CMU();
  addBuf[0] = WRITE_DISABLE_CMD;
  Spi3Drv.Write  (&addBuf[0], 1);
  DISABLE_FLASH_SPI_CMU();

  while (drv_FlashIsBusy ())
    {
      vTaskDelay (1);/*等待1ms 确定器件的4kb 擦出完成*/
    }

  return HAL_OK;
}

/**
 * @brief  Reads a block of data from the Flash.
 * @param  pBuffer: pointer to the buffer that receives the data read from the Flash.
 * @param  readAddr: Flash internal address to read from.
 * @param  length: number of bytes to read from the Flash.
 * @retval None
 */
HAL_StatusTypeDef
drv_FlashRead (uint8_t * pBuffer, uint32_t readAddr, uint32_t length)
{
  u8 addBuf[3] =
    { 0 };
  /* 如果读取的数据长度为0或者超出串行Flash地址空间，则直接返回 */
  if ((length == 0) || (readAddr + length) > N25Q128A_FLASH_SIZE >> 3)
    {
      return HAL_ERROR;
    }
  EN_FLASH_SPI_CMU(); /* 使能片选 */
  addBuf[0] = READ_CMD;
  if (Spi3Drv.Write  (&addBuf[0], 1) != HAL_OK) /* 发送读命令 */
    {
      return HAL_ERROR;
    }
  addBuf[0] = (readAddr & 0xFF0000) >> 16;
  addBuf[1] = (readAddr & 0xFF00) >> 8;
  addBuf[2] = readAddr & 0xFF;
  if (Spi3Drv.Write  (addBuf, 3) != HAL_OK)/* 发送扇区地址的高8bit */
    {
      return HAL_ERROR;
    }
  if (Spi3Drv.Read (pBuffer, length) != HAL_OK)/* 读一个字节并存储到pBuf，读完后指针自加1 */
    {
      return HAL_ERROR;
    }
  DISABLE_FLASH_SPI_CMU();
  return HAL_OK;
}
#endif

