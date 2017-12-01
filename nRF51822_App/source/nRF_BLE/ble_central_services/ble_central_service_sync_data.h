/**
  ****************************************************************************************
  * @file    ble_central_service_sync_data.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-21
  * @brief   header of ble_central_service_sync_data.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_SERVICE_SYNC_DATA_H_
#define __BLE_CENTRAL_SERVICE_SYNC_DATA_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"


/* function declare */
uint32_t ble_central_service_sync_data_init(sync_data_service_t *p_sync_data_service);
void reset_ble_central_sync_data_service(sync_data_service_t *p_sync_data_service);
void ble_sync_data_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt);
void ble_sync_data_ble_evt_handler(ble_evt_t * p_ble_evt);
void ble_central_start_sync_data(DeviceInfomation_t *p_dev);
void ble_central_ack_sync_data_len(DeviceInfomation_t *p_dev,uint16_t len);


#endif /* __BLE_CENTRAL_SERVICE_SYNC_DATA_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



