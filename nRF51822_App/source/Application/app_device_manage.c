/**
  ****************************************************************************************
  * @file    app_device_manage.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-28
  * @brief   the device manage,include:
  *             - analysis the battery level,HW & SW version of the peer
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "app_device_manage.h"



/**
  * @brief  app_device_manage_task_handler
  * @param  *p_event_data,event_size
  * @retval None
  */
void app_device_manage_task_handler(void *p_event_data,uint16_t event_size)
{
    APP_DEVICE_MANAGE_MSG_T         *deviceManageEventMsgValue = (APP_DEVICE_MANAGE_MSG_T *)p_event_data;
    
    /* 接收到消息，对消息事件进行处理 */
    switch(deviceManageEventMsgValue->eventID)
    {
        case EVENT_APP_DEVICE_MANAGE_GET_BATLEVEL_VERSION:
        {
            //解析收到的电池电量、版本信息            
            if((deviceManageEventMsgValue->len) < VERSION_INFO_BUF_LEN)
            {
                /*格式：电量值（1字节）+ 
                        STM32bootloader固件版本号（3字节）+
                        nRF51822Bootloader固件版本号（3字节）+
                        nRF51822App固件版本号（3字节）+
                        nRF51822SD固件版本号（3字节）
                        (备注：STM32APP固件版本号在广播信息中)
                */
//                printf("RX:%d\r\n",deviceManageEventMsgValue->len);
//                uint8_t i=0;
//                for(i=0;i<deviceManageEventMsgValue->len;i++)
//                {
//                    printf("0x%x,",deviceManageEventMsgValue->p_data[i]);
//                }
//                printf("\r\n");
                /* 通过conn_handler判断是哪个设备的信息 */
                if(deviceManageEventMsgValue->conn_handle == g_DeviceInformation.conn_handle)
                {
                    g_DeviceInformation.batLevel = deviceManageEventMsgValue->p_data[0];
                    memcpy(g_DeviceInformation.STMBootVersion,&deviceManageEventMsgValue->p_data[1],3);
                    memcpy(g_DeviceInformation.BLEBOOTREV,&deviceManageEventMsgValue->p_data[4],3);
                    memcpy(g_DeviceInformation.BLEAPPREV,&deviceManageEventMsgValue->p_data[7],3);
                    memcpy(g_DeviceInformation.BLESDREV,&deviceManageEventMsgValue->p_data[10],3);
                }
            }
        }
        break;
        default:break;
    }
}






/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

