/**
  ****************************************************************************************
  * @file    ble_top_implementation.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   header of ble_top_implementation.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_TOP_IMPLEMENTATION_H_
#define __BLE_TOP_IMPLEMENTATION_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"




/* defines for ble pareameters */
#define CENTRAL_LINK_COUNT               1                                /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
                           


void ble_init(void);
void ble_task_handler(void *p_event_data,uint16_t event_size);
void ble_scan_control_timer_init(void);
void ble_connect_bonding_status_polling_timer_init(void);
void ble_reconnect_timer_init(void);
void start_ble_reconnect_timer(DeviceInfomation_t *p_dev);
void stop_ble_reconnect_timer(DeviceInfomation_t *p_dev);                                 
void start_character_find_status_manage_timer(DeviceInfomation_t *p_dev);
void stop_character_find_status_manage_timer(DeviceInfomation_t *p_dev);
void start_connect_bonding_status_polling_timer_timer(DeviceInfomation_t *p_dev);
void stop_connect_bonding_status_polling_timer_timer(DeviceInfomation_t *p_dev);                                 
uint32_t ble_connect_bonding_tatus_execute(CONNECT_BONDING_STATUS_T status);





#endif /* __BLE_TOP_IMPLEMENTATION_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/
