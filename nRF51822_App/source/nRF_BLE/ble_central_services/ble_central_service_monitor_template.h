/**
  ****************************************************************************************
  * @file    ble_central_service_monitor_template.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-21
  * @brief   header of ble_central_service_monitor_template.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_SERVICE_MONITOR_TEMPLATE_H_
#define __BLE_CENTRAL_SERVICE_MONITOR_TEMPLATE_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"



/* function declare */
uint32_t ble_central_service_monitor_template_init(monitor_template_service_t *p_monitor_template_service);
void ble_central_monitor_template_write(DeviceInfomation_t *p_dev);
void reset_ble_central_monitor_template_service(monitor_template_service_t *p_monitor_template_service);
void ble_monitor_template_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt);
void ble_monitor_template_ble_evt_handler(ble_evt_t * p_ble_evt);


#endif /* __BLE_CENTRAL_SERVICE_MONITOR_TEMPLATE_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



