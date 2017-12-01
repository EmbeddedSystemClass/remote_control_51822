/**
  ****************************************************************************************
  * @file    app_cmu_manage.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-20
  * @brief   the device manage,include:
  *             - manage the communication with STM32 by spi interface
  *             - 白了青丝，少年已不在
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "app_cmu_manage.h"


/* private function declare */
static void app_cmu_parse(uint8_t *data,uint8_t len);


/* variables declare */
uint8_t              g_data_cmubuf[MAX_SPI_LEN];    //the buffer for SPI communication 


/**
  * @brief  app_cmu_manage_task_handler
  * @param  *p_event_data,event_size
  * @retval None
  */
void app_cmu_manage_task_handler(void *p_event_data,uint16_t event_size)
{
    APP_CMU_MANAGE_MSG_T         *RxCMUManageMsgValue = (APP_CMU_MANAGE_MSG_T *)p_event_data;
    
    /* 接收到消息，对消息事件进行处理 */
    switch(RxCMUManageMsgValue->eventID)
    {
        case EVENT_APP_CMU_SPI_RX:  // SPI接收到数据
        {
//            uint8_t i = 0;
//            printf("Rx_len_%d:",RxCMUManageMsgValue->len);
//            for(i=0;i<RxCMUManageMsgValue->len;i++)
//            {
//                printf("0x%x,",RxCMUManageMsgValue->p_data[i]);
//            }
//            printf("\r\n");
            app_cmu_parse(RxCMUManageMsgValue->p_data,RxCMUManageMsgValue->len);
        }
        break;
        default:break;
    }
}
/**
 * @brief send the ble connected status to STM32
 * @param[in]  uint32_t sn,bool isConnected
 * @retval None
 */
ret_code_t app_cmu_ble_connect_status_send(uint32_t sn,uint8_t *macAddr,bool isConnected)
{
    uint8_t             data[20] = {0};
    uint8_t             i = 0;
    ret_code_t          status = NRF_ERROR_NULL;
    
    data[i++] = 0xAA;
    data[i++] = 0x54;
    data[i++] = CMU_FRAME_ID_BLE_CONNECT_STATUS;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);
    
    if(isConnected == true)
    {
        data[i++] = 0x01;
    }
    else
    {
        data[i++] = 0x00;
    }
    
    data[i++] = macAddr[0];
    data[i++] = macAddr[1];
    data[i++] = macAddr[2];
    data[i++] = macAddr[3];
    data[i++] = macAddr[4];
    data[i++] = macAddr[5];
    
    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用
    
    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);
    
    status = bsp_spi_slave_transmit(data,i);  

    return status;
}
/**
 * @brief send the ble information request to STM32
 * @param[in]  None
 * @retval status
 */
ret_code_t app_cmu_ble_info_request(void)
{
    uint8_t             data[20] = {0};
    uint8_t             i = 0;
    ret_code_t          status = NRF_ERROR_NULL;
    
    data[i++] = 0xAA;
    data[i++] = 0x54;
    data[i++] = CMU_FRAME_ID_BLE_INFO_REQ;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = 0;
    data[i++] = 0;
    data[i++] = 0;
    data[i++] = 0;  //SN为0
        
    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用
    
    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);
    
    status = bsp_spi_slave_transmit(data,i);  

    return status;
}
/**
 * @brief send the time set status to STM32
 * @param[in]  uint32_t sn,bool isOK
 * @retval None
 */
ret_code_t app_cmu_time_set_status_rsp(uint32_t sn,bool isOK)
{
    uint8_t             data[20] = {0};
    uint8_t             i = 0;
    ret_code_t          status = NRF_ERROR_NULL;
    
    data[i++] = 0xAA;
    data[i++] = 0x54;
    data[i++] = CMU_FRAME_ID_SET_TIME_RSP;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);
    
    if(isOK == true)
    {
        data[i++] = 0x01;
    }
    else
    {
        data[i++] = 0x00;
    }
    
    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用
    
    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);
    
    status = bsp_spi_slave_transmit(data,i);  

    return status;
}

/**
 * @brief send the monitor template set status to STM32
 * @param[in]  uint32_t sn,bool isOK
 * @retval None
 */
ret_code_t app_cmu_monitor_template_set_status_rsp(uint32_t sn,bool isOK)
{
    uint8_t             data[20] = {0};
    uint8_t             i = 0;
    ret_code_t          status = NRF_ERROR_NULL;
    
    data[i++] = 0xAA;
    data[i++] = 0x54;
    data[i++] = CMU_FRAME_ID_SET_MONITOR_TEMPLATE_RSP;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);
    
    if(isOK == true)
    {
        data[i++] = 0x01;
    }
    else
    {
        data[i++] = 0x00;
    }
    
    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用
    
    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);
    
    status = bsp_spi_slave_transmit(data,i);  

    return status;
}

/**
 * @brief send the sync data to STM32
 * @param[in]  uint32_t sn,uint8_t *buf,uint8_t len
 * @retval None
 */
ret_code_t app_cmu_data_sync(uint32_t sn,uint8_t *buf,uint8_t len)
{
    uint8_t             data[MAX_SPI_LEN] = {0};
    uint8_t             i = 0;
    uint8_t             j = 0;
    ret_code_t          status = NRF_ERROR_NULL;
    
    data[i++] = 0xAA;
    data[i++] = 0x54;
    data[i++] = CMU_FRAME_ID_DATA_SYNC;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);
    
    for(j=0;j<len;j++)
    {
        data[i++] = buf[j];
    }
    
    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用
    
    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);
    
    status = bsp_spi_slave_transmit(data,i);  

    return status;
}

/**
 * @brief Parses spi communication data
 * @param[in]  uint8_t *data,uint8_t len
 * @retval None
 */
static void app_cmu_parse(uint8_t *data,uint8_t len)
{
	uint8_t                 FrameHeaderH = 0;
	uint8_t                 FrameHeaderL = 0;
    uint8_t                 buf[MAX_SPI_LEN] = {0};
    CMU_DATA_T              cmuData;
    uint8_t                 i = 0;
    BLE_MSG_T               bleEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;    
    APP_DATA_SYNC_MSG_T     msg;
    
    
//    printf("Rx_len_%d:",len);
//    for(i=0;i<len;i++)
//    {
//        printf("0x%x,",data[i]);
//    }
//    printf("\r\n");

    memset(&cmuData,0,sizeof(CMU_DATA_T));
    cmuData.p_data = buf;
    FrameHeaderH = data[0];
    FrameHeaderL = data[1];  //提取数据帧头
    
    if((FrameHeaderH == FRAME_HEADER_H) && (FrameHeaderL == FRAME_HEADER_L)) //数据帧头正确
    {
        cmuData.frameType = data[2];
        cmuData.dataLen = (uint16_t)((uint16_t)(data[3]<<8) + data[4]);
        if(cmuData.dataLen >= 6)  //每一帧数据至少有6个字节数据
        {
            cmuData.devSN = (uint32_t)((uint32_t)(data[5]<<24) + (uint32_t)(data[6]<<16) + 
                            (uint32_t)(data[7]<<8) + (uint32_t)(data[8]));
            cmuData.crc = (uint16_t)((uint16_t)(data[cmuData.dataLen+5-2]<<8) + data[cmuData.dataLen+5-1]);
            cmuData.dataLen = cmuData.dataLen-4;  //去除所有帧头+SN后数据长度
            for(i=0;i<cmuData.dataLen;i++)//去除所有帧头+SN后数据
            {
                cmuData.p_data[i] = data[i+9];
            }
        }
    }
    else    //数据帧头错误 
    {
        cmuData.frameType = CMU_FRAME_ID_ERR;
    }

    if(cmuData.frameType != CMU_FRAME_ID_ERR)
    {
        printf("Type:0x%x\r\n",cmuData.frameType);
        printf("devSN:%d\r\n",cmuData.devSN);
        printf("len:%d\r\n",cmuData.dataLen);
        for(i=0;i<cmuData.dataLen;i++)
        {
            printf("0x%x,",cmuData.p_data[i]);
        }
        printf("\r\n");
    }
    
    switch(cmuData.frameType) //根据数据帧类型解析
    {
        case CMU_FRAME_ID_START_SCAN:
        {
            bleEventMsgValue.eventID = EVENT_APP_BLE_START_SCAN;
            err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
            APP_ERROR_CHECK(err_code);             
        }
        break;
        case CMU_FRAME_ID_SET_TIME:
        {
            g_setTimeValue = (uint32_t)((uint32_t)(cmuData.p_data[0]<<24) + (uint32_t)(cmuData.p_data[1]<<16) + 
                                        (uint32_t)(cmuData.p_data[2]<<8) + (uint32_t)(cmuData.p_data[3]));
            
//            bleEventMsgValue.eventID = EVENT_APP_BLE_SYNC_TIME;
//            err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
//            APP_ERROR_CHECK(err_code);
            g_connect_bonding_status = STATUS_WRITE_TIME;  //修改状态机，而不是直接发送事件
        }
        break;
        case CMU_FRAME_ID_SET_MONITOR_TEMPLATE:
        {
            if(cmuData.devSN == g_DeviceInformation.sn)
            {
                g_DeviceInformation.monitor_template.len = (cmuData.dataLen-2);   //减去2个字节CRC
                memcpy(g_DeviceInformation.monitor_template.p_contex,cmuData.p_data,g_DeviceInformation.monitor_template.len);
                
                g_connect_bonding_status = STATUS_WRITE_MONITOR_TEMPLATE;  //修改状态机，而不是直接发送事件
            }                
        }
        break;
        case CMU_FRAME_ID_START_DATA_SYNC:
        {
            if(cmuData.devSN == g_DeviceInformation.sn)
            {
                g_connect_bonding_status = STATUS_START_SYNC_DATA;  //修改状态机，而不是直接发送事件
            }          
        }
        break;
        case CMU_FRAME_ID_DATA_SYNC_ACK_LEN:
        {
            if(cmuData.devSN == g_DeviceInformation.sn)
            {
                msg.conn_handle = g_DeviceInformation.conn_handle;
                msg.eventID = EVENT_APP_DATA_SYNC_ACK_LEN;
                msg.len = (uint16_t)((uint16_t)(cmuData.p_data[0] << 8) + (cmuData.p_data[1]));
                msg.p_data = NULL;
                
                app_sched_event_put(&msg,sizeof(msg),app_data_sync_task_handler);                    
            }
        }
        break;
        case CMU_FRAME_ID_BLE_INFO_GET:  //上电时获取STM32发送的蓝牙相关信息
        {
            if(cmuData.devSN != 0) //STM32端已绑定了蓝牙设备，自动重连
            {
                g_DeviceInformation.sn = cmuData.devSN;
                
                memcpy(g_DeviceInformation.MACaddr.addr,cmuData.p_data,BLE_GAP_ADDR_LEN);
                g_DeviceInformation.MACaddr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;  //直接连接方式，MAC地址类型需要与Peer设备保持一直
                                    
                /* start the ble reconnect control timer */
                start_ble_reconnect_timer(&g_DeviceInformation);                
            }
        }
        break;
        case CMU_FRAME_ID_BLE_UNBIND_PEERS:
        {
            if(cmuData.p_data[0] == 0x01) //解除绑定命令
            {
                if(cmuData.devSN == g_DeviceInformation.sn)  //找到需要解绑的设备
                {
                    /* 停止自动重连定时器 */
                    stop_ble_reconnect_timer(&g_DeviceInformation);  
                    
                    ble_central_connect_cancel();
                    
                    /* 断开蓝牙连接 */
                    bleEventMsgValue.eventID = EVENT_APP_BLE_DISCONNECT;
                    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
        break;
        default:break;
    }
}




/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

