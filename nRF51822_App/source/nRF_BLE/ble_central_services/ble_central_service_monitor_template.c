/**
  ****************************************************************************************
  * @file    ble_central_service_monitor_template.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-21
  * @brief   manage the monitor template for the peers,include：
  *             - write the monitor template to the peers
  *             - get the monitor template write result of the peers
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ble_central_service_monitor_template.h"

/* UUID define for the peer monitor template service and characteristic */
#define  BLE_UUID_MONITOR_TEMPLATE_SERVICE                  0xDA6A
#define  BLE_UUID_MONITOR_TEMPLATE_SET_CHAR                 0xDA6B
#define  BLE_UUID_MONITOR_TEMPLATE_RESULT_CHAR       		0xDA6C

static const ble_uuid128_t MONITOR_TEMPLATE_UUID_128B =	{ 0x92, 0xE5, 0x2B, 0xE5, 0xF8, 0xD8, 0x83, 0x01, 
                                                          0x61, 0x8B, 0x0E, 0x01, 0x6A, 0xDA, 0x2A, 0x4E};
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
  * @brief  init the monitor template service
  * @param  *p_monitor_template_service
  * @retval status
  */
uint32_t ble_central_service_monitor_template_init(monitor_template_service_t *p_monitor_template_service)
{
    ble_uuid_t          bas_uuid;
    uint32_t            err_code = NRF_ERROR_NULL;
    
    /* Reset the all the char of monitor template service */
    reset_ble_central_monitor_template_service(p_monitor_template_service);
    
    err_code = sd_ble_uuid_vs_add(&MONITOR_TEMPLATE_UUID_128B, &bas_uuid.type);
    bas_uuid.uuid = BLE_UUID_MONITOR_TEMPLATE_SERVICE;    
    APP_ERROR_CHECK(err_code);

    return ble_db_discovery_evt_register(&bas_uuid);    
}

/**
  * @brief  ble central device write monitor template to the cleint
  * @param  *p_dev
  * @retval None
  */
void ble_central_monitor_template_write(DeviceInfomation_t *p_dev)
{
    uint8_t         i=0;
    uint8_t         buf[MONITOR_TEMPLATE_DATA_MAX_LEN+2] = {0};  //2为监护方案长度
    uint8_t         len = 0;
    uint8_t         send_len = 0;
    uint8_t         *buf_send;

    #ifdef DEBUG_BLE_MONITOR_TEMPLATE
        printf("monitor template write\r\n");
    #endif
    
    if(p_dev->conn_handle != BLE_CONN_HANDLE_INVALID)  //仍然连接中
    {
        if(p_dev->monitor_template.len < MONITOR_TEMPLATE_DATA_MAX_LEN) //监护方案数据长度有效
        {
            buf[0] = (uint8_t)(p_dev->monitor_template.len >> 8);
            buf[1] = (uint8_t)(p_dev->monitor_template.len);    //前两字节为监护方案长度
            
            for(i=0;i<(p_dev->monitor_template.len);i++)
            {
                buf[i+2] = p_dev->monitor_template.p_contex[i];
            }
        }
        len = (uint8_t)(p_dev->monitor_template.len + 2);
        buf_send = buf;
                    
        /* 分段写方案数据至设备端(这种处理方式不是很好，后面需要改改) */   
        while(len)
        {
            if(len > 20)
            {
                send_len = 20;
            }
            else
            {
                send_len = len;
            }
            write_to_client(p_dev->conn_handle,p_dev->monitor_template_service.monitorTemplateSetCharW.char_handle,
                            buf_send,send_len);
            buf_send += send_len;
            len -= send_len;        
        }
        nrf_delay_ms(500);
        /* 应答STM32方案写成功 */
        app_cmu_monitor_template_set_status_rsp(p_dev->sn,true);
    }
}

/**
  * @brief  reset the device monitor template service
  * @param  *p_devinfo_manage_service
  * @retval None
  */
void reset_ble_central_monitor_template_service(monitor_template_service_t *p_monitor_template_service)
{
    /* Reset all the characteristic of monitor template service */
    p_monitor_template_service->monitorTemplateSetCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_monitor_template_service->monitorTemplateSetCharW.char_handle = BLE_CONN_HANDLE_INVALID;
    
    p_monitor_template_service->monitorTemplateSetResultCharR.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_monitor_template_service->monitorTemplateSetResultCharR.char_handle = BLE_CONN_HANDLE_INVALID;
}

/**
  * @brief  the monitor template service discovery event handler
  * @param  *p_dev_info,* p_evt
  * @retval None
  */  
void ble_monitor_template_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt)
{
    // Check if the device information manage Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_MONITOR_TEMPLATE_SERVICE &&
        p_evt->conn_handle == p_dev_info->conn_handle  //设备仍然处于连接中
    ) 
    {
        // Find the CCCD Handle of the characteristic.
        uint8_t i;
        
        #ifdef DEBUG_BLE_MONITOR_TEMPLATE
            printf("\tFind char for monitor template:%d\r\n",(p_evt->params.discovered_db.char_count));
        #endif
        
        for(i = 0; i < (p_evt->params.discovered_db.char_count); i++)
        {
            // Find the monitor template write character. Store CCCD handle and break.
            if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_MONITOR_TEMPLATE_SET_CHAR)
            {
                p_dev_info->monitor_template_service.monitorTemplateSetCharW.cccd_handle =
                                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->monitor_template_service.monitorTemplateSetCharW.char_handle =
                                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_MONITOR_TEMPLATE_SET;
                
                #ifdef DEBUG_BLE_MONITOR_TEMPLATE
                    printf("\t\tMonitor Template Char find OK\r\n");
                #endif
            }            
            // Find the monitor template set result characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                BLE_UUID_MONITOR_TEMPLATE_RESULT_CHAR)
            {
                p_dev_info->monitor_template_service.monitorTemplateSetResultCharR.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->monitor_template_service.monitorTemplateSetResultCharR.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_MONITOR_TEMPLATE_RESULT;
                
                #ifdef DEBUG_BLE_MONITOR_TEMPLATE
                    printf("\t\tMonitor Template result Char find OK\r\n");
                #endif 

                /*  Set to notify feature           */
                cccd_configure (
                                p_evt->conn_handle,
                                p_dev_info->monitor_template_service.monitorTemplateSetResultCharR.cccd_handle,
                                true);               
            }                       
        }
    }
    #ifdef DEBUG_BLE_MONITOR_TEMPLATE
    else if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
    {
        if(p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_MONITOR_TEMPLATE_SERVICE)
        {
            printf("monitor template service not found\r\n");
        }    
    }
    #endif
}

/**
  * @brief  the monitor template ble event handler
  * @param  * p_evt
  * @retval None
  */  
void ble_monitor_template_ble_evt_handler(ble_evt_t * p_ble_evt)
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
            #ifdef DEBUG_BLE_MONITOR_TEMPLATE
                printf("Monitor Template SD Read/Write API returns Success..\r\n");
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
            #ifdef DEBUG_BLE_MONITOR_TEMPLATE
                printf("Monitor Template SD Read/Write error = 0x%x\r\n", err_code);
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
    // Check if this notification is a battery level notification.
    if(p_ble_evt->evt.gattc_evt.params.hvx.handle
            == g_DeviceInformation.monitor_template_service.monitorTemplateSetResultCharR.char_handle)
    {        
        /*  get the battery level result       */
        #ifdef DEBUG_BLE_MONITOR_TEMPLATE
            printf("get monitor template result:%d,Len:%d\r\n",p_ble_evt->evt.gattc_evt.params.hvx.data[0],
                                                 p_ble_evt->evt.gattc_evt.params.hvx.len
                    );
        #endif
        
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
        (p_response->handle ==  g_DeviceInformation.monitor_template_service.monitorTemplateSetCharW.char_handle) && 
        (p_response->len != 0) //response data
    )
    {
        #ifdef DEBUG_BLE_MONITOR_TEMPLATE
//            printf("[PWD]: PWD write result:Leng_%d,0x%X,0x%X ,0x%X ,0x%X \r\n ", 
//                                    p_response->len, 
//                                    p_response->data[0], 
//                                    p_response->data[1], 
//                                    p_response->data[2], 
//                                    p_response->data[3]);
        #endif
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
    if (p_response->handle == g_DeviceInformation.monitor_template_service.monitorTemplateSetResultCharR.char_handle)
    {

//        #ifdef DEBUG_BLE_MONITOR_TEMPLATE
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

