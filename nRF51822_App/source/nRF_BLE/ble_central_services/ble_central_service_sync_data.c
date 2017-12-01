/**
  ****************************************************************************************
  * @file    ble_central_service_sync_data.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-21
  * @brief   manage the data sync function for the peers,include：
  *             - the switch to start data sync
  *             - get the data from the peer device
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ble_central_service_sync_data.h"


/* UUID define for sync data service and character */
#define SYN_DATA_SERVICE_UUID                       0x0CE6
#define SYN_DATA_SWITCH_CHAR_UUID                   0x0CE7
#define SYN_DATA_SYNC_CHAR_UUID                     0x0CE8
#define SYN_DATA_SYNC_DONE_UUID                     0x0CE9

static const ble_uuid128_t SYN_DATA_UUID_128B = { 0x33, 0x6F, 0xA4, 0xBD, 0x31, 0x0B, 0xC4, 0x2D, 
                                                  0x35, 0xE5, 0x2D, 0x48, 0xE6, 0x0C, 0xD6, 0xD5};

#define SYNC_TIME_DATA_LEN      7
#define TX_BUFFER_MASK       0x07                  /**< TX Buffer mask, must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE       (TX_BUFFER_MASK + 1)  /**< Size of the send buffer, which is 1 higher than the mask. */
#define WRITE_MESSAGE_LENGTH BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

/**@brief Structure for holding the data that will be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    TX_REQUEST_T type;         /**< Type of message. (read or write). */
    union
    {
        uint16_t       read_handle;  /**< Read request handle. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;


static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index        = 0;        /**< Current index in the transmit buffer containing the next message to be transmitted. */

/* private function declare */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool notification_enable); 
static void tx_buffer_process(void);
static void on_hvx (const ble_evt_t * p_ble_evt);
static void on_write_rsp (const ble_evt_t * p_ble_evt);
static void on_read_rsp (const ble_evt_t * p_ble_evt);
static void sync_write_to_client(uint16_t conn_handle,uint16_t char_handle, uint8_t *pData,uint8_t length);

/**
  * @brief  init the device information manage service
  * @param  *p_devinfo_manage_service
  * @retval status
  */
uint32_t ble_central_service_sync_data_init(sync_data_service_t *p_sync_data_service)
{
    ble_uuid_t          bas_uuid;
    uint32_t            err_code = NRF_ERROR_NULL;
    
    /* Reset the all the char of sync data service */
    reset_ble_central_sync_data_service(p_sync_data_service);
    
    err_code = sd_ble_uuid_vs_add (&SYN_DATA_UUID_128B, &bas_uuid.type);
    bas_uuid.uuid = SYN_DATA_SERVICE_UUID;
    APP_ERROR_CHECK(err_code);

    return ble_db_discovery_evt_register(&bas_uuid);    
}
/**
  * @brief  ble central start data sync
  * @param  *p_dev
  * @retval None
  */
void ble_central_start_sync_data(DeviceInfomation_t *p_dev)
{
    uint8_t cmd = 0x01;
    
    if(p_dev->conn_handle != BLE_CONN_HANDLE_INVALID) //设备仍然连接中
    {
        /* write the start data sync cmd to the peer */
        sync_write_to_client(p_dev->conn_handle,p_dev->sync_data_service.syncDataSwitchCharW.char_handle,&cmd,1);
    }
}
/**
  * @brief  ble central ack the synced data length to the peer
  * @param  *p_dev,len
  * @retval None
  */
void ble_central_ack_sync_data_len(DeviceInfomation_t *p_dev,uint16_t len)
{
    uint8_t buf[2] = {0};
    
    buf[0] = (uint8_t)(len >> 8);
    buf[1] = (uint8_t)(len);
    
    if(p_dev->conn_handle != BLE_CONN_HANDLE_INVALID) //设备仍然连接中
    {
        /* ack the synced len to the peer */
        sync_write_to_client(p_dev->conn_handle,p_dev->sync_data_service.syncDataDoneCharW.char_handle,buf,2);
    }    
}

/**
  * @brief  reset the sync data service
  * @param  *p_sync_data_service
  * @retval None
  */
void reset_ble_central_sync_data_service(sync_data_service_t *p_sync_data_service)
{
    /* Reset all the characteristic of sync data service */
    p_sync_data_service->syncDataCharR.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_sync_data_service->syncDataCharR.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_sync_data_service->syncDataDoneCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_sync_data_service->syncDataDoneCharW.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_sync_data_service->syncDataSwitchCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_sync_data_service->syncDataSwitchCharW.char_handle = BLE_CONN_HANDLE_INVALID;
}

/**
  * @brief  the data sync service discovery event handler
  * @param  *p_dev_info,* p_evt
  * @retval None
  */  
void ble_sync_data_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt)
{
    // Check if the device information manage Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == SYN_DATA_SERVICE_UUID &&
        p_evt->conn_handle == p_dev_info->conn_handle  //设备仍然处于连接中
    ) 
    {
        // Find the CCCD Handle of the characteristic.
        uint8_t i;
        
        #ifdef DEBUG_BLE_SYNC_DATA
            printf("\tFind char for data sync:%d\r\n",(p_evt->params.discovered_db.char_count));
        #endif
        
        for(i = 0; i < (p_evt->params.discovered_db.char_count); i++)
        {
            // Find the data sync switch characteristic. Store CCCD handle and break.
            if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                SYN_DATA_SWITCH_CHAR_UUID)
            {
                p_dev_info->sync_data_service.syncDataSwitchCharW.cccd_handle =
                                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->sync_data_service.syncDataSwitchCharW.char_handle =
                                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                
                p_dev_info->char_find_manage |= YWK_CHARACTER_SYNC_DATA_SWITCH;
                #ifdef DEBUG_BLE_SYNC_DATA
                    printf("\t\tSync Data Switch Char find OK\r\n");
                #endif
            }            
            // Find the data sync channnel characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                SYN_DATA_SYNC_CHAR_UUID)
            {
                p_dev_info->sync_data_service.syncDataCharR.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->sync_data_service.syncDataCharR.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                
                p_dev_info->char_find_manage |= YWK_CHARACTER_SYNC_DATA;
                
                /*  Set to notify feature           */
                cccd_configure (
                                p_evt->conn_handle,
                                p_dev_info->sync_data_service.syncDataCharR.cccd_handle,
                                true);  
                
                #ifdef DEBUG_BLE_SYNC_DATA
                    printf("\t\tSync Data channel Char find OK\r\n");
                #endif                           
            }            
            // Find the data sync done characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                SYN_DATA_SYNC_DONE_UUID)
            {
                p_dev_info->sync_data_service.syncDataDoneCharW.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->sync_data_service.syncDataDoneCharW.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                
                p_dev_info->char_find_manage |= YWK_CHARACTER_SYNC_DATA_DONE;
                               
                #ifdef DEBUG_BLE_SYNC_DATA
                    printf("\t\tSync Data Done Char find OK\r\n");
                #endif                 
            }
        }
    }
    #ifdef DEBUG_BLE_SYNC_DATA
    else if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
    {
        if(p_evt->params.discovered_db.srv_uuid.uuid == SYN_DATA_SERVICE_UUID)
        {
            printf("sync data service not found\r\n");
        }     
    }
    #endif
}
/**
  * @brief  the data sync ble event handler
  * @param  * p_evt
  * @retval None
  */  
void ble_sync_data_ble_evt_handler(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
        {
            on_hvx(p_ble_evt);
        }
        break;
        case BLE_GATTC_EVT_WRITE_RSP:
        {
            on_write_rsp(p_ble_evt);
        }
        break;
        case BLE_GATTC_EVT_READ_RSP:
        {
            on_read_rsp(p_ble_evt);
        }
        break;
        default:break;
    }
}

/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool notification_enable)
{
    //LOG("CCCD. CCCD Handle = %d, Connection Handle = %d\r\n",handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = 2;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;
    
    tx_buffer_process();
    
    return NRF_SUCCESS;
} 
/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            #ifdef DEBUG_BLE_SYNC_DATA
                printf("Sync Data SD Read/Write API returns Success..\r\n");
            #endif
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
            
            if(m_tx_buffer[m_tx_index].conn_handle == g_DeviceInformation.conn_handle)
            {
                g_DeviceInformation.isNRFBusy = false;
            }            
        }
        else if(err_code == NRF_ERROR_BUSY)
        {
            if(m_tx_buffer[m_tx_index].conn_handle == g_DeviceInformation.conn_handle)
            {
                g_DeviceInformation.isNRFBusy = true;
            }
        }
        else
        {
            #ifdef DEBUG_BLE_SYNC_DATA
                printf("Sync Data SD Read/Write error = 0x%x\r\n", err_code);
            #endif
        }
    }
}
/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will handle the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the Battery Level measurement from the peer. If
 *            so, this function will decode the battery level measurement and send it to the
 *            PWDication.
 *
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx (const ble_evt_t * p_ble_evt)
{
    APP_DATA_SYNC_MSG_T     syncDataEventMsgValue; 
    uint32_t                err_code = NRF_ERROR_NULL;
    
    // Check if this notification is a data sync notification.
    if((p_ble_evt->evt.gattc_evt.params.hvx.handle
            == g_DeviceInformation.sync_data_service.syncDataCharR.char_handle) &&
        (p_ble_evt->evt.gap_evt.conn_handle == g_DeviceInformation.conn_handle)     //仍然在连接中
    )
    {    
        #ifdef DEBUG_BLE_SYNC_DATA
//            uint8_t     i=0;
//        
//            printf("rx data:");
//            for(i=0;i<p_ble_evt->evt.gattc_evt.params.hvx.len;i++)
//            {
//                printf("0x%02x,",p_ble_evt->evt.gattc_evt.params.hvx.data[i]);
//            }
//            printf("\r\n");
        #endif
        #ifdef DEBUG_BLE_SYNC_DATA
            printf("flag_get_rx_data_len:%d\r\n",flag_get_rx_data_len);
        #endif
        if(flag_get_rx_data_len == true)
        {
            current_data_len = (uint16_t)(p_ble_evt->evt.gattc_evt.params.hvx.data[0]*256 + 
                                     p_ble_evt->evt.gattc_evt.params.hvx.data[1]);
            need_rx_data_len = current_data_len + 15;
            if((current_data_len <=MAX_LEN_ONE_PACKET) && (p_ble_evt->evt.gattc_evt.params.hvx.data[6] == 0x02) &&
               (p_ble_evt->evt.gattc_evt.params.hvx.data[14] == 0x08) //临时措施，解决数据同步第一帧出错问题
            )
            {
                flag_get_rx_data_len = false;
            }
            rx_data_cnt = 0;
            memset(g_data_syncbuf,0,MAX_LEN_ONE_PACKET);
        }
                
        if(need_rx_data_len < MAX_LEN_ONE_PACKET)
        {
            memcpy(g_data_syncbuf+rx_data_cnt,p_ble_evt->evt.gattc_evt.params.hvx.data,p_ble_evt->evt.gattc_evt.params.hvx.len);
            rx_data_cnt = rx_data_cnt + p_ble_evt->evt.gattc_evt.params.hvx.len;
            #ifdef DEBUG_BLE_SYNC_DATA
                printf("rx_data_cnt:%d,need_rx_data_len:%d\r\n",rx_data_cnt,need_rx_data_len);
            #endif
            if(rx_data_cnt >= need_rx_data_len)
            {
                flag_get_rx_data_len = true;
                
                syncDataEventMsgValue.eventID = EVENT_APP_DATA_SYNC_RX_DATA;
                syncDataEventMsgValue.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                syncDataEventMsgValue.len = need_rx_data_len;
                syncDataEventMsgValue.p_data = g_data_syncbuf;

                err_code = app_sched_event_put(&syncDataEventMsgValue,sizeof(syncDataEventMsgValue),app_data_sync_task_handler);
                APP_ERROR_CHECK(err_code); 
            }
        }
        else  //数据包长度大于每次接受最大长度，重新接收
        {
            flag_get_rx_data_len = true;
        }
    }
}

/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_evt Pointer to the SoftDevice event.
 */
static void on_write_rsp (const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_write_rsp_t * p_response;
    
    p_response = &p_ble_evt->evt.gattc_evt.params.write_rsp;

    if(
        (p_response->handle ==  g_DeviceInformation.sync_data_service.syncDataDoneCharW.char_handle) && 
        (p_response->len != 0) //response data
    )
    {
//        #ifdef DEBUG_BLE_SYNC_DATA
//            printf("[PWD]: PWD write result:Leng_%d,0x%X,0x%X ,0x%X ,0x%X \r\n ", 
//                                    p_response->len, 
//                                    p_response->data[0], 
//                                    p_response->data[1], 
//                                    p_response->data[2], 
//                                    p_response->data[3]);
//        #endif
    }
    tx_buffer_process();
}
/**@brief     Function for handling read response events.
 *
 * @details   This function will validate the read response and raise the appropriate
 *            event to the PWDication.
 *
 * @param[in] p_ble_evt Pointer to the SoftDevice event.
 */
static void on_read_rsp (const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;
    if (p_response->handle == g_DeviceInformation.sync_data_service.syncDataCharR.char_handle)
    {

//        #ifdef DEBUG_BLE_SYNC_DATA
//              printf("[PWD]: PWD Read result:Leng_%d,0x%X,0x%X,0x%X,0x%X\r\n", p_response->len, 
//                        p_response->data[0], p_response->data[1], 
//                        p_response->data[2], p_response->data[3]);
//        #endif        
    }
    tx_buffer_process();
}

/**
  * @brief  Function for writing data to the client
  * @param  None
  * @retval None
  */
static void sync_write_to_client(uint16_t conn_handle,uint16_t char_handle, uint8_t *pData,uint8_t length)
{
    tx_message_t * p_msg;
    
    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;
    
    p_msg->req.write_req.gattc_params.handle = char_handle;
    p_msg->req.write_req.gattc_params.len = length;
    p_msg->req.write_req.gattc_params.p_value = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    memcpy(p_msg->req.write_req.gattc_value, pData, length);
    p_msg->conn_handle = conn_handle;
    p_msg->type = WRITE_REQ;
    tx_buffer_process();
}
/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

