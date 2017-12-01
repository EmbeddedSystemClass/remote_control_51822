/**
  ****************************************************************************************
  * @file    app_cmu_manage.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-20
  * @brief   header of app_cmu_manage.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _APP_CMU_MANAGE_H__
#define _APP_CMU_MANAGE_H__

#include "include.h"

/* type define for CMU SPI EVENT */
typedef enum
{
    EVENT_APP_CMU_DEFAULT = 0,
    EVENT_APP_CMU_SPI_RX,
}APP_CMU_MANAGE_EVENT_ID_T;

/* type define for CMU message */
typedef struct
{
    APP_CMU_MANAGE_EVENT_ID_T           eventID;
    uint8_t                             len;
    uint8_t                             *p_data;
}APP_CMU_MANAGE_MSG_T;

/* type define for spi communication data */
typedef struct
{
    uint8_t         frameType;
    uint16_t        dataLen;
    uint8_t         *p_data;
    uint32_t        devSN;
    uint16_t        crc;
}CMU_DATA_T;

/* spi communication frame ID define */
#define FRAME_HEADER_H                              0xAA
#define FRAME_HEADER_L                              0x55
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
    




/* extern variables declare */
extern uint8_t              g_data_cmubuf[MAX_SPI_LEN];


/* function declare */
void app_cmu_manage_task_handler(void *p_event_data,uint16_t event_size);
ret_code_t app_cmu_ble_connect_status_send(uint32_t sn,uint8_t *macAddr,bool isConnected);
ret_code_t app_cmu_time_set_status_rsp(uint32_t sn,bool isOK);
ret_code_t app_cmu_monitor_template_set_status_rsp(uint32_t sn,bool isOK);
ret_code_t app_cmu_data_sync(uint32_t sn,uint8_t *buf,uint8_t len);
ret_code_t app_cmu_ble_info_request(void);


#endif // _APP_CMU_MANAGE_H__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




