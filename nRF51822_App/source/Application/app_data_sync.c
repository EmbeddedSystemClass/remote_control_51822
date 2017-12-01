/**
  ****************************************************************************************
  * @file    app_data_sync.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-2
  * @brief   the sync data for app,include:
  *             - start the data sync
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "app_data_sync.h"


uint8_t              g_data_syncbuf[MAX_LEN_ONE_PACKET] = {0};
uint16_t             current_data_len = 0;
uint16_t             need_rx_data_len = 0;
uint16_t             rx_data_cnt = 0;
bool                 flag_get_rx_data_len = true; 


/**
  * @brief  app_data_sync_task_handler
  * @param  *p_event_data,event_size
  * @retval None
  */
void app_data_sync_task_handler(void *p_event_data,uint16_t event_size)
{
    APP_DATA_SYNC_MSG_T         *dataSyncEventMsgValue = (APP_DATA_SYNC_MSG_T *)p_event_data;
    
    /* 接收到消息，对消息事件进行处理 */
    switch(dataSyncEventMsgValue->eventID)
    {
        case EVENT_APP_DATA_SYNC_START_SYNC_DATA:
        {
            #ifdef DEBUG_BLE_SYNC_DATA
                printf("start sync data\r\n");
            #endif
            nrf_delay_ms(500);
            ble_central_start_sync_data(&g_DeviceInformation);        
        }
        break;
        case EVENT_APP_DATA_SYNC_RX_DATA:
        {
//            uint8_t i=0;
//            printf("Rx Data:%d\r\n",dataSyncEventMsgValue->len);
//            for(i=0;i<dataSyncEventMsgValue->len;i++)
//            {
//                printf("0x%02x,",dataSyncEventMsgValue->p_data[i]);
//            }
//            printf("\r\n");

            /* 将接收到的同步数据发送至STM32 */
            if(g_DeviceInformation.conn_handle == dataSyncEventMsgValue->conn_handle)
            {
                #ifdef DEBUG_BLE_SYNC_DATA
                    printf("send sync data to stm32\r\n");
                #endif                
                app_cmu_data_sync(g_DeviceInformation.sn,dataSyncEventMsgValue->p_data,dataSyncEventMsgValue->len);
            } 
        }
        break;
        case EVENT_APP_DATA_SYNC_ACK_LEN:
        {
            /* 根据conn_handle判断应答哪个设备*/
            if(dataSyncEventMsgValue->conn_handle == g_DeviceInformation.conn_handle)
            {
                #ifdef DEBUG_BLE_SYNC_DATA
                    printf("ACK Len:%d\r\n",dataSyncEventMsgValue->len);
                #endif                  
                ble_central_ack_sync_data_len(&g_DeviceInformation,dataSyncEventMsgValue->len);
            }
        }
        break;
        default:break;
    }
}



/**
  * @brief  reset the data sync variables
  * @param  None
  * @retval None
  */
void reset_data_sync(void)
{
    current_data_len = 0;
    need_rx_data_len = 0;
    rx_data_cnt = 0;
    memset(g_data_syncbuf,0,MAX_LEN_ONE_PACKET);
    flag_get_rx_data_len = true;
}







/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



