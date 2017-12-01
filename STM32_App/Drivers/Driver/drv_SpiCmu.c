/******************************************************************************
 *
 * COPYRIGHT:
 *   Copyright (c)  2005-2050   GnsTime  Inc.    All rights reserved.
 *
 *   This is unpublished proprietary source code of GnsTime Inc.
 *   The copyright notice above does not evidence any actual or intended
 *   publication of such source code.
 *
 * FILE NAME:
 *   drv_SpiCmu.c
 * DESCRIPTION:
 *
 * HISTORY:
 *   2015年10月15日        Arvin         Create/Update
 *
 *****************************************************************************/

#include "drv_SpiComDef.h"

SPI_HandleTypeDef hEPgSpi;
static SemaphoreHandle_t xSpiEpgMutex=NULL;
static bool isInit = FALSE;

static bool drv_Spi3IsInit (void);
static HAL_StatusTypeDef drv_Spi3GetAccess (void);
static HAL_StatusTypeDef drv_Spi3ReleaseAccess (void);
static HAL_StatusTypeDef drv_Spi3Init (SPI_COMInitTypeDef *spiInit);
static HAL_StatusTypeDef drv_Spi3WriteReg (uint8_t *txBuf, uint16_t length);
static HAL_StatusTypeDef drv_Spi3ReadReg (uint8_t *rxBuf, uint16_t length);
static uint8_t drv_Spi3WriteRead (uint8_t Byte);

SPI_COM_DrvTypeDef Spi3Drv =
{ 
    drv_Spi3Init, 
    drv_Spi3WriteReg, 
    drv_Spi3ReadReg, 
    drv_Spi3WriteRead,
    drv_Spi3GetAccess, 
    drv_Spi3ReleaseAccess, 
    drv_Spi3IsInit 
};

static bool drv_Spi3IsInit(void)
{
    return isInit;
}

static HAL_StatusTypeDef drv_Spi3GetAccess (void)
{
  if ( pdTRUE == xSemaphoreTake(xSpiEpgMutex, portMAX_DELAY))
    {
      return HAL_OK;
    }
  return HAL_ERROR;
}

static HAL_StatusTypeDef drv_Spi3ReleaseAccess(void)
{
  if ( pdTRUE == xSemaphoreGive(xSpiEpgMutex))
    {
      return HAL_OK;
    }
  return HAL_ERROR;
}

static HAL_StatusTypeDef drv_Spi3Init (SPI_COMInitTypeDef *spiInit)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if (xSpiEpgMutex == NULL)
    {
        xSpiEpgMutex = xSemaphoreCreateMutex();
    }
    
    if (xSpiEpgMutex == NULL)
    {
        return HAL_ERROR;
    }

    /* Peripheral clock enable */
    SPI_ROM_CLK_EN();

    GPIO_InitStruct.Pin = SPI_ROM_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = SPI_ROM_MI_MO_AF;
    HAL_GPIO_Init (SPI_ROM_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_ROM_MISO_PIN;
    HAL_GPIO_Init (SPI_ROM_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_ROM_MOSI_PIN;
    HAL_GPIO_Init (SPI_ROM_MOSI_PORT, &GPIO_InitStruct);

    hEPgSpi.Instance = SPI_ROM_SPI_INS;
    hEPgSpi.Init.Mode = spiInit->Mode;
    hEPgSpi.Init.Direction = SPI_DIRECTION_2LINES;
    hEPgSpi.Init.DataSize = spiInit->DataSize;
    hEPgSpi.Init.CLKPolarity = spiInit->CLKPolarity; //SPI_POLARITY_LOW;
    hEPgSpi.Init.CLKPhase = spiInit->CLKPhase; //SPI_PHASE_2EDGE;
    hEPgSpi.Init.NSS = SPI_NSS_SOFT;
    hEPgSpi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hEPgSpi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hEPgSpi.Init.TIMode = SPI_TIMODE_DISABLED;
    hEPgSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hEPgSpi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init (&hEPgSpi) != HAL_OK)
    {
        isInit = FALSE;
        return HAL_ERROR;
    }
    
    isInit = TRUE;
    return HAL_OK;
}

 
/**
 * @brief  Sends a Byte through the SPI interface a
 * @param  txBuf: Byte send.  length send length
 * @retval  the send result  HLA_OK or HAL_ERROR
 */
HAL_StatusTypeDef drv_Spi3WriteReg (uint8_t *txBuf, uint16_t length)
{
    uint8_t rx[length];
    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_TransmitReceive (&hEPgSpi, txBuf, rx, length, SPI_COM_TIME_OUT) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}
/**
 * @brief  Get the Byte received
 *         from the SPI bus.
 * @param  rxBuf: Bytes received.  length :want to read
 * @retval  the send result  HLA_OK or HAL_ERROR
 * @retval The received byte value
 */
HAL_StatusTypeDef drv_Spi3ReadReg (uint8_t *rxBuf, uint16_t length)
{
    uint8_t gClk = DUMMY_BYTE;
    
    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_TransmitReceive (&hEPgSpi, &gClk, rxBuf, length,SPI_COM_TIME_OUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  Sends a Byte through the SPI interface and return the Byte received
 *         from the SPI bus.
 * @param  Byte: Byte send.
 * @retval The received byte value
 */
static uint8_t drv_Spi3WriteRead (uint8_t Byte)
{
    uint8_t receivedbyte = 0;

    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_TransmitReceive (&hEPgSpi, (uint8_t*) &Byte,(uint8_t*) &receivedbyte, 1, SPI_COM_TIME_OUT)
    != HAL_OK)
    {
        receivedbyte = 0;
    }

    return receivedbyte;
}
