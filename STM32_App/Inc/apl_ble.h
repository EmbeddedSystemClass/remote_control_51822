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
 *   apl_B2E.h
 * DESCRIPTION:
 *
 * HISTORY:
 *   2016年1月12日        Arvin         Create/Update
 *
*****************************************************************************/
#ifndef __APL_BLE_H__
#define __APL_BLE_H__

#include "includes.h"

//#define BLE_SCAN_START_EVENT                    (1)
//#define BLE_SCAN_RESULT_EVENT                   (2)
//#define BLE_CONNECT_EVENT                       (3)
//#define BLE_DISCONNECT_EVENT                    (4)
//#define BLE_SYNC_EVENT                          (5)
//#define BLE_GET_STATUS_EVENT                    (6)

/* type define for ble event */
typedef enum
{
    BLE_DEFAULT_EVENT = 0,
    BLE_SPI_READ_EVENT,
    BLE_START_SCAN_EVENT,
    BLE_CONNECT_EVENT,
    BLE_UNBIND_PEERS,
    BLE_SET_TIME_EVENT,
    BLE_SET_MONITOR_TEMPLATE,
    BLE_START_DATA_SYNC,
    BLE_DATA_SYNC,
    BLE_DEV_INFO_SEND_EVENT
}BLE_EVENT_T;

/* type define for ble message */
typedef struct
{
    BLE_EVENT_T event;
    uint8_t     *buf;
    uint8_t     len;
    uint32_t    sn;
}BLE_MSG_T;

/* type define for ble connect process status control */
typedef enum
{
    BLE_PROCESS_INIT = 0,
    BLE_PROCESS_CONNECT,
    BLE_PROCESS_SET_TIME,
    BLE_PROCESS_SET_MONITOR_TEMPLATE,
    BLE_PROCESS_START_DATA_SYNC
}BLE_PROCESS_STATUS_T;

extern QueueHandle_t xQueue_Ble;
extern DEV_BLE_INFO_T  gBLEDevInfo;


void CreateBleTask(void);
HAL_StatusTypeDef ble_start_scan(void);
HAL_StatusTypeDef ble_set_time(uint32_t sn,uint32_t tim);
HAL_StatusTypeDef ble_unbind_peers(uint32_t sn,uint8_t cmd);
HAL_StatusTypeDef ble_set_monitor_template(uint32_t sn, MONITOR_TEMPLATE_T *monitorTemplate);
HAL_StatusTypeDef ble_start_data_sync(uint32_t sn);
HAL_StatusTypeDef ble_data_sync_len_ack(uint32_t sn,uint16_t len);
HAL_StatusTypeDef ble_data_ble_info_send(uint32_t sn,uint8_t *p_mac,uint8_t len);

#endif /* __APL_BLE_H */

