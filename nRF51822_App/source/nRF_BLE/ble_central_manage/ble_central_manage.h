/**
  ****************************************************************************************
  * @file    ble_central_manage.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   header of ble_central_manage.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_CENTRAL_MANAGE_H__
#define __BLE_CENTRAL_MANAGE_H__

#include "include.h"


/* Central related define  */

#define APP_TIMER_PRESCALER         (0)                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     (10)                                          /**< Size of timer operation queues. */

#define SCAN_INTERVAL               (uint16_t) MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 (uint16_t) MSEC_TO_UNITS(80, UNIT_0_625_MS)  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                (0)                                           /**< disables timeout. */

#define MIN_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                             /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */


/* self define service UUID */
#define YWK_DEVICE_CONFIRM_UUID     0x2311      //云卫康设备认证服务UUID，在广播包中包含改服务UUID的即认为是云卫康设备

/* service and character find complete management define(max 32 characters) */                                               
#define YWK_CHARACTER_PWD_WRITE                                             ((uint32_t)(1 << 0))
#define YWK_CHARACTER_PWD_RESULT                                            ((uint32_t)(1 << 1))
#define YWK_CHARACTER_SYC_TIME                                              ((uint32_t)(1 << 2))
#define YWK_CHARACTER_NAME_MODIFY                                           ((uint32_t)(1 << 3))
#define YWK_CHARACTER_VERSION_GET                                           ((uint32_t)(1 << 4))
#define YWK_CHARACTER_BAT_LEVEL_GET                                         ((uint32_t)(1 << 5))
#define YWK_CHARACTER_MONITOR_TEMPLATE_SET                                  ((uint32_t)(1 << 6))
#define YWK_CHARACTER_MONITOR_TEMPLATE_RESULT                               ((uint32_t)(1 << 7))
#define YWK_CHARACTER_SYNC_DATA_SWITCH                                      ((uint32_t)(1 << 8))
#define YWK_CHARACTER_SYNC_DATA                                             ((uint32_t)(1 << 9))
#define YWK_CHARACTER_SYNC_DATA_DONE                                        ((uint32_t)(1 << 10))

#define YWK_CHARACTER_NONE                      ((uint32_t)(0x0)) 
#define YWK_CHARACTER_ALL                       (YWK_CHARACTER_PWD_WRITE | YWK_CHARACTER_PWD_RESULT | YWK_CHARACTER_SYC_TIME |  \
                                                YWK_CHARACTER_NAME_MODIFY | YWK_CHARACTER_VERSION_GET | YWK_CHARACTER_BAT_LEVEL_GET | \
                                                YWK_CHARACTER_MONITOR_TEMPLATE_SET | YWK_CHARACTER_MONITOR_TEMPLATE_RESULT | \
                                                YWK_CHARACTER_SYNC_DATA_SWITCH | YWK_CHARACTER_SYNC_DATA | YWK_CHARACTER_SYNC_DATA_DONE \
                                                )


ret_code_t scan_start(void);
ret_code_t scan_stop(void);
ret_code_t ble_central_connect(ble_gap_addr_t *peerAddr);
ret_code_t ble_central_connect_cancel(void);
ret_code_t ble_central_disconnect(uint16_t conn_handle);
void on_ble_central_evt(const ble_evt_t * const p_ble_evt);





#endif // __BLE_CENTRAL_MANAGE_H__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




