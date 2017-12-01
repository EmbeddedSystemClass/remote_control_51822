/**
  ****************************************************************************************
  * @file    ble_central_service_devinfo_manage.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-15
  * @brief   manage the information for the peers,include：
  *             - write the current time to the peers
  *             - modify the device name of the peers
  *             - read the soft&hw version from the peers
  *             - get the battery level of the peers
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ble_central_service_devinfo_manage.h"


/* uuid define for the peer device information manage service and char */
#define BLE_UUID_DEVICE_INFORMATION_MANAGE_SERVICE				0x2FC0
#define BLE_UUID_DEVICE_NAME_CHAR                     			0x2FC1
#define BLE_UUID_DEVICE_VERSION_CHAR                  			0x2FC2
#define BLE_UUID_DEVICE_BATTERY_LEVEL_CHAR                   	0x2FC3
#define BLE_UUID_DEVICE_CURRENT_TIME_CHAR                    	0x2FC4

static const ble_uuid128_t DEVINFO_MANAGE_UUID_128B = { 0x48, 0x73, 0x35, 0x03, 0xE7, 0xD5, 0x0C, 0xBC, 
                                                        0x50, 0x5F, 0x89, 0x9C, 0xC0, 0x2F, 0x8A, 0xE0 };

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
static void write_to_client(uint16_t conn_handle,uint16_t char_handle, uint8_t *pData,uint8_t length);

/**
  * @brief  init the device information manage service
  * @param  *p_devinfo_manage_service
  * @retval status
  */
uint32_t ble_central_service_devinfo_manage_init(devinfo_manage_service_t *p_devinfo_manage_service)
{
    ble_uuid_t          bas_uuid;
    uint32_t            err_code = NRF_ERROR_NULL;
    
    /* Reset the all the char of device information manage service */
    reset_ble_central_devinfo_manage_service(p_devinfo_manage_service);
    
    err_code = sd_ble_uuid_vs_add (&DEVINFO_MANAGE_UUID_128B, &bas_uuid.type);
    bas_uuid.uuid = BLE_UUID_DEVICE_INFORMATION_MANAGE_SERVICE;
    APP_ERROR_CHECK(err_code);

    return ble_db_discovery_evt_register(&bas_uuid);    
}

/**
  * @brief  ble central device write pin code to the cleint
  * @param  *p_dev
  * @retval None
  */
void ble_central_synctime_write(DeviceInfomation_t *p_dev, time_t p_time)
{
    uint8_t temp[SYNC_TIME_DATA_LEN] = {0};
    time_t nowTime = p_time;
    
    struct tm * timeNow = localtime(&nowTime);
    
    temp[0] = timeNow->tm_year/100;
    temp[1] = timeNow->tm_year%100;
    temp[2] = timeNow->tm_mon + 1;
    temp[3] = timeNow->tm_mday;
    temp[4] = (timeNow->tm_hour+8)%24;
    temp[5] = timeNow->tm_min;
    temp[6] = timeNow->tm_sec;
    
    write_to_client(p_dev->conn_handle,p_dev->devinfo_manage_service.syncTimeCharW.char_handle,temp,SYNC_TIME_DATA_LEN);
}

/**
  * @brief  reset the device information manage service
  * @param  *p_devinfo_manage_service
  * @retval None
  */
void reset_ble_central_devinfo_manage_service(devinfo_manage_service_t *p_devinfo_manage_service)
{
    /* Reset all the characteristic of device information manage service */
    p_devinfo_manage_service->batLevelGetCharR.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_devinfo_manage_service->batLevelGetCharR.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_devinfo_manage_service->nameModifyCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_devinfo_manage_service->nameModifyCharW.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_devinfo_manage_service->syncTimeCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_devinfo_manage_service->syncTimeCharW.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_devinfo_manage_service->versionGetCharR.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_devinfo_manage_service->versionGetCharR.char_handle = BLE_CONN_HANDLE_INVALID;
}

/**
  * @brief  the device information manage service discovery event handler
  * @param  *p_dev_info,* p_evt
  * @retval None
  */  
void ble_devinfo_manage_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt)
{
    // Check if the device information manage Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_DEVICE_INFORMATION_MANAGE_SERVICE &&
        p_evt->conn_handle == p_dev_info->conn_handle  //设备仍然处于连接中
    ) 
    {
        // Find the CCCD Handle of the characteristic.
        uint8_t i;
        
        #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
            printf("\tFind char for devinfo manage:%d\r\n",(p_evt->params.discovered_db.char_count));
        #endif
        
        for(i = 0; i < (p_evt->params.discovered_db.char_count); i++)
        {
            // Find the time sync characteristic. Store CCCD handle and break.
            if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_DEVICE_CURRENT_TIME_CHAR)
            {
                p_dev_info->devinfo_manage_service.syncTimeCharW.cccd_handle =
                                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->devinfo_manage_service.syncTimeCharW.char_handle =
                                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_SYC_TIME;
                #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                    printf("\t\tTime sync Char find OK\r\n");
                #endif
            }            
            // Find the modify device name characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                BLE_UUID_DEVICE_NAME_CHAR)
            {
                p_dev_info->devinfo_manage_service.nameModifyCharW.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->devinfo_manage_service.nameModifyCharW.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_NAME_MODIFY;
                
                #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                    printf("\t\tName modify result Char find OK\r\n");
                #endif                           
            }            
            // Find the read soft&hw version characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                BLE_UUID_DEVICE_VERSION_CHAR)
            {
                p_dev_info->devinfo_manage_service.versionGetCharR.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->devinfo_manage_service.versionGetCharR.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_VERSION_GET;
                               
                /*  Set to notify feature           */
                cccd_configure (
                                p_evt->conn_handle,
                                p_dev_info->devinfo_manage_service.versionGetCharR.cccd_handle,
                                true); 

                #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                    printf("\t\tGet version Char find OK\r\n");
                #endif                 
            }
            // Find the get battery level characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                BLE_UUID_DEVICE_BATTERY_LEVEL_CHAR)
            {
                p_dev_info->devinfo_manage_service.batLevelGetCharR.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->devinfo_manage_service.batLevelGetCharR.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_BAT_LEVEL_GET;
                               
                /*  Set to notify feature           */
                cccd_configure (
                                p_evt->conn_handle,
                                p_dev_info->devinfo_manage_service.batLevelGetCharR.cccd_handle,
                                true); 

                #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                    printf("\t\tGet batLevel Char find OK\r\n");
                #endif                 
            }            
        }
    }
    #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
    else if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
    {
        if(p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_DEVICE_INFORMATION_MANAGE_SERVICE)
        {
            printf("devinfo manage service not found\r\n");
        }     
    }
    #endif
}

/**
  * @brief  the device information manage ble event handler
  * @param  * p_evt
  * @retval None
  */  
void ble_devinfo_manage_ble_evt_handler(ble_evt_t * p_ble_evt)
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
            #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                printf("devinfo manage SD Read/Write API returns Success..\r\n");
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
            #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
                printf("devinfo manage SD Read/Write error = 0x%x\r\n", err_code);
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
    APP_DEVICE_MANAGE_MSG_T     appDeviceManageEventMsgValue;
    uint32_t                    err_code = NRF_ERROR_NULL;

    
    // Check if this notification is a battery level notification.
    if((p_ble_evt->evt.gattc_evt.params.hvx.handle
            == g_DeviceInformation.devinfo_manage_service.batLevelGetCharR.char_handle) &&
        (p_ble_evt->evt.gap_evt.conn_handle == g_DeviceInformation.conn_handle)  //处于连接状态
    )
        
    {        
        /*  get the battery level result       */
        #ifdef DEBUG_BLE_PEER_DEVINFO_MANAGE
//            printf("get bat level:%d,Len:%d\r\n",p_ble_evt->evt.gattc_evt.params.hvx.data[0],
//                                                 p_ble_evt->evt.gattc_evt.params.hvx.len
//                    );
        #endif
        
        if(p_ble_evt->evt.gattc_evt.params.hvx.len < VERSION_INFO_BUF_LEN)
        {
            memcpy(g_buf_bat_version,&p_ble_evt->evt.gattc_evt.params.hvx.data,p_ble_evt->evt.gattc_evt.params.hvx.len);
            
            /* 发送电量及HW SW版本解析事件 */
            appDeviceManageEventMsgValue.eventID = EVENT_APP_DEVICE_MANAGE_GET_BATLEVEL_VERSION;
            appDeviceManageEventMsgValue.len = p_ble_evt->evt.gattc_evt.params.hvx.len;
            appDeviceManageEventMsgValue.p_data = g_buf_bat_version;
            appDeviceManageEventMsgValue.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                                   
            err_code = app_sched_event_put(&appDeviceManageEventMsgValue,sizeof(appDeviceManageEventMsgValue),
                                            app_device_manage_task_handler);
            APP_ERROR_CHECK(err_code);
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
        (p_response->handle ==  g_DeviceInformation.devinfo_manage_service.nameModifyCharW.char_handle) && 
        (p_response->len != 0) //response data
    )
    {
//        #ifdef DEBUG_BLE_BONDING
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
    if (p_response->handle == g_DeviceInformation.devinfo_manage_service.batLevelGetCharR.char_handle)
    {

//        #ifdef DEBUG_BLE_BONDING
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
static void write_to_client(uint16_t conn_handle,uint16_t char_handle, uint8_t *pData,uint8_t length)
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


