/**
  ****************************************************************************************
  * @file    drv_Spi2.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-16
  * @brief   header of ble_communication.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_COMMUNICATION_H__
#define __BLE_COMMUNICATION_H__


#include "drv_SpiComDef.h"

#define MAX_BLE_SPI_LEN                         255 //STM32与nRF51822之间SPI通讯，一包数据最大长度

/* type define for spi communication data */
typedef struct
{
    uint8_t         frameType;
    uint16_t        dataLen;
    uint8_t         dataBuf[MAX_BLE_SPI_LEN];
    uint32_t        devSN;
    uint16_t        crc;
}CMU_DATA_T;

/* spi communication frame ID define */
#define FRAME_HEADER_H                              0xAA
#define FRAME_HEADER_L                              0x54
#define CMU_FRAME_ID_ERR                            0x00
#define CMU_FRAME_ID_START_SCAN                     0x01
#define CMU_FRAME_ID_BLE_CONNECT_STATUS             0x02
#define CMU_FRAME_ID_SET_TIME                       0x03
#define CMU_FRAME_ID_SET_TIME_RSP                   0x04
#define CMU_FRAME_ID_SET_MONITOR_TEMPLATE           0x05
#define CMU_FRAME_ID_SET_MONITOR_TEMPLATE_RSP       0x06
#define CMU_FRAME_ID_START_DATA_SYNC                0x07
#define CMU_FRAME_ID_DATA_SYNC_ACK_LEN              0x08
#define CMU_FRAME_ID_DATA_SYNC                      0x09
#define CMU_FRAME_ID_BLE_INFO_REQ                   0x0A
#define CMU_FRAME_ID_BLE_INFO_GET                   0x0B
#define CMU_FRAME_ID_BLE_UNBIND_PEERS               0x0C

/* function declare */
HAL_StatusTypeDef ble_init(void);
void ble_reset(void);
HAL_StatusTypeDef ble_send_data(uint8_t *data,uint16_t len);
HAL_StatusTypeDef ble_receive_data(CMU_DATA_T *p_data);


#endif /* end of __BLE_COMMUNICATION_H__ */

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




