/**
  ****************************************************************************************
  * @file    ble_communication.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-16
  * @brief   communication with nRF51822
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "ble_communication.h"

static void ble_gpio_init(void);
static SPI_COM_DrvTypeDef *ble_spi = &Spi2Drv;
void BLE_Reset(void);

/**
  * @brief  ble init
  * @param  None
  * @retval status
  */
HAL_StatusTypeDef ble_init(void)
{
    HAL_StatusTypeDef       status = HAL_ERROR;
    SPI_COMInitTypeDef      spiInit;

    /* nRF51822 Control GPIO(SYNC/REQN/RESET) config */
    ble_gpio_init();

    /* OutPut LSE for nRF51822 */

    ble_reset();

    /* SPI config */
    spiInit.CLKPhase = SPI_PHASE_1EDGE;
    spiInit.CLKPolarity = SPI_POLARITY_LOW;
    spiInit.DataSize = SPI_DATASIZE_8BIT;
    spiInit.Mode = SPI_MODE_MASTER;
    status = ble_spi->Init(&spiInit);

    return status;
}

/**
  * @brief  ble_send_data
  * @param[in]  uint8_t *data,uint16_t len
  * @retval status
  */
HAL_StatusTypeDef ble_send_data(uint8_t *data,uint16_t len)
{
    HAL_StatusTypeDef   status = HAL_ERROR;

    if(ble_spi->GetAccess() == HAL_OK)
    {
        HAL_GPIO_WritePin(SPI_BLE_NSS_PORT,SPI_BLE_NSS_PIN,GPIO_PIN_RESET);
        status = ble_spi->Write(data,len);
        HAL_GPIO_WritePin(SPI_BLE_NSS_PORT,SPI_BLE_NSS_PIN,GPIO_PIN_SET);
        ble_spi->ReleaseAccess();
    }
    return status;
}
/**
  * @brief  ble_receive_data
  * @param[out]  CMU_DATA_T *p_data
  * @retval status
  */
HAL_StatusTypeDef ble_receive_data(CMU_DATA_T *p_data)
{
    HAL_StatusTypeDef       status = HAL_ERROR;
    uint8_t                 data[MAX_BLE_SPI_LEN] = {0};
	uint8_t                 FrameHeaderH = 0;
	uint8_t                 FrameHeaderL = 0;
    uint8_t                 i = 0;

    if(ble_spi->GetAccess () == HAL_OK)
    {
        HAL_GPIO_WritePin(SPI_BLE_NSS_PORT,SPI_BLE_NSS_PIN,GPIO_PIN_RESET);
        status = ble_spi->Read(data, 5);        /*先读5个头*/

        FrameHeaderH = data[0];
        FrameHeaderL = data[1];  //提取数据帧头

        if((FrameHeaderH == FRAME_HEADER_H) && (FrameHeaderL == FRAME_HEADER_L)) //数据帧头正确
        {
            p_data->frameType = data[2];
            p_data->dataLen = (uint16_t)((uint16_t)(data[3]<<8) + data[4]);

            if(p_data->dataLen > MAX_BLE_SPI_LEN) //RXMSG_DATA_LEN_MAX
            {
                status = HAL_ERROR;
            }
            else
            {
                status =  ble_spi->Read(data+5, p_data->dataLen);/*数据*/
            }

            if(p_data->dataLen >= 6) //每帧数据至少有6个字节
            {
                p_data->devSN = (uint32_t)((uint32_t)(data[5]<<24) + (uint32_t)(data[6]<<16) +
                                (uint32_t)(data[7]<<8) + (uint32_t)(data[8]));
                p_data->crc = (uint16_t)(data[p_data->dataLen+5-2]<<8 + data[p_data->dataLen+5-1]);
                p_data->dataLen = p_data->dataLen-4;  //去除所有帧头+SN后数据长度
                for(i=0;i<p_data->dataLen;i++)//去除所有帧头+SN后数据
                {
                    p_data->dataBuf[i] = data[i+9];
                }
            }
        }
        else    //数据帧头错误
        {
            p_data->frameType = CMU_FRAME_ID_ERR;
        }

        HAL_GPIO_WritePin(SPI_BLE_NSS_PORT,SPI_BLE_NSS_PIN,GPIO_PIN_SET);
        ble_spi->ReleaseAccess ();
    }
    return status;
}



/**
  * @brief  reset nRF51822
  * @param  None
  * @retval None
  */
void ble_reset(void)
{
    HAL_GPIO_WritePin (PT_GPIOBTRESET, PT_GPIO_PIN_BTRESET, GPIO_PIN_RESET);
    vTaskDelay(1);
    HAL_GPIO_WritePin (PT_GPIOBTRESET, PT_GPIO_PIN_BTRESET, GPIO_PIN_SET);
    vTaskDelay(1);
}

/**
  * @brief  ble gpio init
  * @param  None
  * @retval None
  */
static void ble_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* SPI NSS Pin config */
    HAL_GPIO_WritePin(SPI_BLE_NSS_PORT,SPI_BLE_NSS_PIN,GPIO_PIN_SET);
    SPI_BLE_NSS_PORT_CLK();

    GPIO_InitStructure.Pin = SPI_BLE_NSS_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (SPI_BLE_NSS_PORT, &GPIO_InitStructure);

    /* BT Reset pin config */
    PT_RCC_AHBPeriph_GPIO_BTRESET();

    HAL_GPIO_WritePin(PT_GPIOBTRESET, PT_GPIO_PIN_BTRESET, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = PT_GPIO_PIN_BTRESET;       //nRF51822 Reset
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PT_GPIOBTRESET, &GPIO_InitStructure);

    /* BT REQN Pin config */
    PT_RCC_AHB_GPIO_BTREQN();
    HAL_NVIC_DisableIRQ (BLE_BTREQN_EXIT_IT_LINE);

    GPIO_InitStructure.Pin = BLE_BTREQN_EXIT_IO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(BLE_BTREQN_EXIT_IO_PORT, &GPIO_InitStructure);

    __HAL_GPIO_EXTI_CLEAR_IT(BLE_BTREQN_EXIT_IO_PIN);
    /* Enable and set EXTI line4 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(BLE_BTREQN_EXIT_IT_LINE, EXTI0_IRQn_LINE_PRI, 0);
    HAL_NVIC_EnableIRQ(BLE_BTREQN_EXIT_IT_LINE);
}








/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


