/**
  ****************************************************************************************
  * @file    ble_central_service_devinfo_manage.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-15
  * @brief   header of ble_central_service_devinfo_manage.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_SERVICE_DEVINFO_MANAGE_H_
#define __BLE_CENTRAL_SERVICE_DEVINFO_MANAGE_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"



/* function declare */
uint32_t ble_central_service_devinfo_manage_init(devinfo_manage_service_t *p_devinfo_manage_service);
void reset_ble_central_devinfo_manage_service(devinfo_manage_service_t *p_devinfo_manage_service);
void ble_devinfo_manage_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt);
void ble_devinfo_manage_ble_evt_handler(ble_evt_t * p_ble_evt);
void ble_central_synctime_write(DeviceInfomation_t *p_dev, time_t p_time);


#endif /* __BLE_CENTRAL_SERVICE_DEVINFO_MANAGE_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



