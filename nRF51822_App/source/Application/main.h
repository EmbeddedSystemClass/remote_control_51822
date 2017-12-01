/**
  ****************************************************************************************
  * @file    platform.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   header of main.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN__
#define __MAIN__

#include "include.h"



/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
  uint8_t * p_data; /**< Pointer to data. */
  uint16_t data_len; /**< Length of data. */
} data_t;

/* type struct define for monitor template */
typedef struct
{
    uint16_t len;
    uint8_t  p_contex[MONITOR_TEMPLATE_DATA_MAX_LEN];
}MONITOR_TEMPLATE_T;

/* type struct define */
typedef struct
{
	uint8_t                             DevSupportCap;
	uint8_t                             HWREV[3];
	uint8_t                             STM32APPREV[3];
    uint8_t                             STMBootVersion[3];
    uint8_t                             BLEAPPREV[3];
    uint8_t                             BLEBOOTREV[3];
    uint8_t                             BLESDREV[3];
    uint8_t                             protocolVersion;
    uint8_t                             batLevel;
	uint8_t                             Flag_ReceviedDevInfor; 
    int8_t                              rssi;
    uint32_t                            sn;
    ble_gap_addr_t                      MACaddr;
    bool                                isValid;
    uint16_t                            conn_handle;
    uint32_t                            char_find_manage;
    bonding_service_t                   bonding_service;
    devinfo_manage_service_t            devinfo_manage_service;
    monitor_template_service_t          monitor_template_service;
    sync_data_service_t                 sync_data_service;
    bool                                isNRFBusy;
    MONITOR_TEMPLATE_T                  monitor_template;
}DeviceInfomation_t;

/*type struct define for ble scan list */
typedef struct
{
    int8_t              rssi;
    uint8_t             sn[4];
    ble_gap_addr_t      MACaddr;
    bool                isValid;
}BLE_SCAN_LIST_T;

/* type struct define for connect and bonding status */
typedef enum
{
    STATUS_NONE = 0,
    STATUS_WRITE_PIN,
    STATUS_WRITE_PIN_WAIT,
    STATUS_WRITE_TIME,
    STATUS_WRITE_TIME_WAIT,
    STATUS_WRITE_MONITOR_TEMPLATE,
    STATUS_WRITE_MONITOR_TEMPLATE_WAIT,
    STATUS_START_SYNC_DATA,
    STATUS_START_SYNC_DATA_WAIT,
    STATUS_CONNECT_BONDING_COMPLATE
}CONNECT_BONDING_STATUS_T;

/******************* event define for task handler **********************************/
/* ble handler task */
typedef enum
{
    EVENT_APP_BLE_DEFAULT =0,
    EVENT_APP_BLE_START_SCAN,
    EVENT_APP_BLE_STOP_SCAN,
    EVENT_APP_BLE_CONNECT,
    EVENT_APP_BLE_DISCONNECT,
    EVENT_APP_BLE_PASSKEY_WRITE,
    EVENT_APP_BLE_SYNC_TIME,
    EVENT_APP_BLE_MONITOR_TEMPLATE_WRITE,
    EVENT_APP_BLE_SERVICE_CHAR_FIND_COMPLATE,
    EVENT_APP_BLE_BONDING_OK,
    EVENT_APP_BLE_BONDING_ERR
}BLE_EVENT_ID_T;

typedef struct
{
    BLE_EVENT_ID_T      eventID;
}BLE_MSG_T;

/* app device manage task */
typedef enum
{
    EVENT_APP_DEVICE_MANAGE_DEFAULT = 0,
    EVENT_APP_DEVICE_MANAGE_GET_BATLEVEL_VERSION
}APP_DEVICE_MANAGE_EVENT_ID_T;

typedef struct
{
    APP_DEVICE_MANAGE_EVENT_ID_T        eventID;
    uint8_t                             len;
    uint8_t                             *p_data;
    uint16_t                            conn_handle;
}APP_DEVICE_MANAGE_MSG_T;

/* app data sync task */
typedef enum
{
    EVENT_APP_DATA_SYNC_DEFAULT = 0,
    EVENT_APP_DATA_SYNC_START_SYNC_DATA,
    EVENT_APP_DATA_SYNC_RX_DATA,
    EVENT_APP_DATA_SYNC_ACK_LEN
}APP_DATA_SYNC_EVENT_ID_T;

typedef struct
{
    APP_DATA_SYNC_EVENT_ID_T            eventID;
    uint16_t                            len;
    uint8_t                             *p_data;
    uint16_t                            conn_handle;    
}APP_DATA_SYNC_MSG_T;



/* gloable variables declare */
extern DeviceInfomation_t  			g_DeviceInformation;                //硬件设备信息
extern BLE_SCAN_LIST_T              gScanList[];  
extern ble_db_discovery_t           g_ble_db_discovery[]; /**< list of DB structures used by the database discovery module. */
extern CONNECT_BONDING_STATUS_T     g_connect_bonding_status;
extern uint8_t                      g_buf_bat_version[VERSION_INFO_BUF_LEN];
extern uint32_t                     g_setTimeValue;                 //设置时间

#endif // __MAIN__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




