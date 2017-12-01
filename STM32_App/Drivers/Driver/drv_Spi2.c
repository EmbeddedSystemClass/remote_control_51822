/**
  ****************************************************************************************
  * @file    drv_Spi2.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-16
  * @brief   Spi2 driver,used for communication with nRF51822
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/


#include "drv_SpiComDef.h"


static bool drv_Spi2IsInit (void);
static HAL_StatusTypeDef drv_Spi2GetAccess (void);
static HAL_StatusTypeDef drv_Spi2ReleaseAccess (void);
static HAL_StatusTypeDef drv_Spi2Init (SPI_COMInitTypeDef *spiInit);
static HAL_StatusTypeDef drv_Spi2WriteReg (uint8_t *txBuf, uint16_t length);
static HAL_StatusTypeDef drv_Spi2ReadReg (uint8_t *rxBuf, uint16_t length);
static uint8_t drv_Spi2WriteRead (uint8_t Byte);
static SPI_HandleTypeDef spi2Handle;
static SemaphoreHandle_t xSpiEpgMutex=NULL;
static bool isInit = FALSE;


SPI_COM_DrvTypeDef Spi2Drv = 
{
    drv_Spi2Init,
    drv_Spi2WriteReg,
    drv_Spi2ReadReg,
    drv_Spi2WriteRead,
    drv_Spi2GetAccess,
    drv_Spi2ReleaseAccess,
    drv_Spi2IsInit
};

static bool drv_Spi2IsInit ()
{
    return isInit;
}

static HAL_StatusTypeDef drv_Spi2GetAccess()
{
    if(pdTRUE == xSemaphoreTake(xSpiEpgMutex, portMAX_DELAY))
    {
        return HAL_OK;
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef drv_Spi2ReleaseAccess()
{
    if(pdTRUE == xSemaphoreGive(xSpiEpgMutex))
    {
        return HAL_OK;
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef drv_Spi2Init(SPI_COMInitTypeDef *spiInit)
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
    SPI_BLE_CLK_ENABLE();
    SPI_BLE_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = SPI_BLE_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = SPI_BLE_MI_MO_AF;
    HAL_GPIO_Init(SPI_BLE_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_BLE_MISO_PIN;
    HAL_GPIO_Init(SPI_BLE_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_BLE_MOSI_PIN;
    HAL_GPIO_Init(SPI_BLE_MOSI_PORT, &GPIO_InitStruct);

    spi2Handle.Instance = SPI_BLE_SPI_INS;
    spi2Handle.Init.Mode = spiInit->Mode;
    spi2Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi2Handle.Init.DataSize = spiInit->DataSize;
    spi2Handle.Init.CLKPolarity = spiInit->CLKPolarity;
    spi2Handle.Init.CLKPhase = spiInit->CLKPhase; //SPI_PHASE_2EDGE;
    spi2Handle.Init.NSS = SPI_NSS_SOFT;
    spi2Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    spi2Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi2Handle.Init.TIMode = SPI_TIMODE_DISABLED;
    spi2Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;

    if (HAL_SPI_Init(&spi2Handle) != HAL_OK)
    {
        isInit = FALSE;
        return HAL_ERROR;
    }

    __HAL_SPI_ENABLE(&spi2Handle);
    
    isInit = TRUE;
    return HAL_OK;
}

/**
 * @brief  Sends a Byte through the SPI interface a
 * @param  txBuf: Byte send.  length send length
 * @retval  the send result  HLA_OK or HAL_ERROR
 */
HAL_StatusTypeDef drv_Spi2WriteReg(uint8_t *txBuf, uint16_t length)
{
    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_Transmit (&spi2Handle, txBuf, length,
    SPI_COM_TIME_OUT) != HAL_OK)
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
HAL_StatusTypeDef drv_Spi2ReadReg(uint8_t *rxBuf, uint16_t length)
{
    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_Receive (&spi2Handle, rxBuf, length, SPI_COM_TIME_OUT) != HAL_OK)
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
static uint8_t drv_Spi2WriteRead (uint8_t Byte)
{
    uint8_t receivedbyte = 0;

    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_TransmitReceive (&spi2Handle, (uint8_t*) &Byte,(uint8_t*) &receivedbyte, 1, SPI_COM_TIME_OUT)
        != HAL_OK)
    {
        receivedbyte = 0;
    }

    return receivedbyte;
}









/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

