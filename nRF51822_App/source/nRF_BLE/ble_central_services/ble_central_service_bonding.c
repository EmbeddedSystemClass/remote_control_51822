/**
  ****************************************************************************************
  * @file    ble_central_service_bonding.c
  * @author  Jason
  * @version V1.0.0
  * @date    2017-2-9
  * @brief   bonding service for central
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "ble_central_service_bonding.h"
  
/* passkey uuid define for bonding */
#define  BLE_UUID_PASSKEY_AUTH_SERVICE   	0x8939
#define  BLE_UUID_PASSKEY_WRITE_CHAR   	    0x8940
#define  BLE_UUID_AUTH_RESULT_CHAR       	0x8941 	

/* passkey pin code length define */
#define BAND_SN_LENGTH                      0x04
#define BAND_OK                             1
#define BAND_PWD_ERR                        0

static const ble_uuid128_t PWD_UUID_128B = { 0x22, 0x1A, 0xAC, 0xD7, 0x5B, 0xCA, 0xA0, 0x5A, 0x23, 0xD6, 0xFA, 0xAD,
                                             0x39, 0x89, 0x88, 0xC3 };

                                             /**
* @note Add code here to seperate the tx buffer 
*/

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
static void write_to_client(DeviceInfomation_t *p_dev, uint8_t *pData,uint8_t length);

                                             
                                             
/**
  * @brief  init the bonding service
  * @param  *p_bondinf_service
  * @retval status
  */
uint32_t ble_central_service_bonding_init(bonding_service_t *p_bondinf_service)
{
    ble_uuid_t          bas_uuid;
    uint32_t            err_code = NRF_ERROR_NULL;
    
    /* Reset the bonding service pwd write and result char */
    reset_ble_central_bonding_service(p_bondinf_service);
    
    err_code = sd_ble_uuid_vs_add (&PWD_UUID_128B, &bas_uuid.type);
    bas_uuid.uuid = BLE_UUID_PASSKEY_AUTH_SERVICE;
    APP_ERROR_CHECK(err_code);

    return ble_db_discovery_evt_register (&bas_uuid);    
}
/**
  * @brief  ble central device write pin code to the cleint
  * @param  *p_dev
  * @retval None
  */
void ble_central_passkey_write(DeviceInfomation_t *p_dev)
{
    uint8_t     PinCodeInCharFormat[BAND_SN_LENGTH] = {0};
    uint32_t    uSnConvertTmp = 0;
    
    if(p_dev->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        /*  convert Sn to char type         */
        uSnConvertTmp = p_dev->sn;
        
        PinCodeInCharFormat[3] = 0x30 + uSnConvertTmp % 10;
        PinCodeInCharFormat[2] = 0x30 + (uSnConvertTmp / 10) % 10;
        PinCodeInCharFormat[1] = 0x30 + (uSnConvertTmp / 100) % 10;
        PinCodeInCharFormat[0] = 0x30 + (uSnConvertTmp / 1000) % 10;
        
        /* send pin code to client */
        write_to_client(p_dev,PinCodeInCharFormat,BAND_SN_LENGTH);
        #ifdef DEBUG_BLE_BONDING
            printf("[PWD]: Write PWD for Bonding. \r\n");
        #endif        
    }
    #ifdef DEBUG_BLE_BONDING
    else
    {
        printf("[PWD]: connHandle INVALID. \r\n");
    }
    #endif
}

/**
  * @brief  reset the bonding service
  * @param  *p_bondinf_service
  * @retval None
  */
void reset_ble_central_bonding_service(bonding_service_t *p_bondinf_service)
{
    /* Reset the bonding service pwd write and result char */
    p_bondinf_service->pwdResultCharR.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_bondinf_service->pwdResultCharR.char_handle  = BLE_CONN_HANDLE_INVALID;
    
    p_bondinf_service->pwdWriteCharW.cccd_handle = BLE_CONN_HANDLE_INVALID;
    p_bondinf_service->pwdWriteCharW.char_handle = BLE_CONN_HANDLE_INVALID;    
}

/**
  * @brief  the bonding service discovery event handler
  * @param  * p_evt
  * @retval None
  */  
void ble_bonding_db_discovery_evt_handler(DeviceInfomation_t *p_dev_info, ble_db_discovery_evt_t * p_evt)
{
    // Check if the Heart Rate Service was discovered.
    if(p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_PASSKEY_AUTH_SERVICE &&
        p_evt->conn_handle == p_dev_info->conn_handle  //设备仍然处于连接中
    ) 
    {
        // Find the CCCD Handle of the characteristic.
        uint8_t i;
        
        #ifdef DEBUG_BLE_BONDING
            printf("\tFind char for bonding:%d\r\n",(p_evt->params.discovered_db.char_count));
        #endif
        
        for(i = 0; i < (p_evt->params.discovered_db.char_count); i++)
        {
            // Find passkey write characteristic. Store CCCD handle and break.
            if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_PASSKEY_WRITE_CHAR)
            {
                p_dev_info->bonding_service.pwdWriteCharW.cccd_handle =
                                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->bonding_service.pwdWriteCharW.char_handle =
                                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_PWD_WRITE;
                #ifdef DEBUG_BLE_BONDING
                    printf("\t\tPWD Write Char find OK\r\n");
                #endif
            }            
            // Find passkey confirm result characteristic. Store CCCD handle and break.
            else if(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == 
                BLE_UUID_AUTH_RESULT_CHAR)
            {
                p_dev_info->bonding_service.pwdResultCharR.cccd_handle =
                                p_evt->params.discovered_db.charateristics[i].cccd_handle;
                p_dev_info->bonding_service.pwdResultCharR.char_handle =
                                p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                p_dev_info->char_find_manage |= YWK_CHARACTER_PWD_RESULT;
                
                #ifdef DEBUG_BLE_BONDING
                    printf("\t\tPWD result Char find OK\r\n");
                #endif                
                /*  Set to notify feature           */
                cccd_configure (
                                p_evt->conn_handle,
                                p_dev_info->bonding_service.pwdResultCharR.cccd_handle,
                                true);               
            }            
        }
    }
    #ifdef DEBUG_BLE_BONDING
    else if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
    {
        if(p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_PASSKEY_AUTH_SERVICE)
        {
            printf("Bond service not found\r\n");
        }
    }   
    #endif
}
/**
  * @brief  the bonding service ble event handler
  * @param  * p_evt
  * @retval None
  */  
void ble_bonding_ble_evt_handler(ble_evt_t * p_ble_evt)
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
            #ifdef DEBUG_BLE_BONDING
                printf("Bond SD Read/Write API returns Success..\r\n");
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
            #ifdef DEBUG_BLE_BONDING
                printf("Bond SD Read/Write error = 0x%x\r\n", err_code);
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
    BLE_MSG_T               bleEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;
    
    // Check if this notification is a password confirm result notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle
            == g_DeviceInformation.bonding_service.pwdResultCharR.char_handle)
    {        
        /*  Check band result       */
        if (p_ble_evt->evt.gattc_evt.params.hvx.data[0] == BAND_OK)
        {
            #ifdef DEBUG_BLE_BONDING
                printf("[PWD]: bonding Ok \r\n");
            #endif

            bleEventMsgValue.eventID = EVENT_APP_BLE_BONDING_OK;
            err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
            APP_ERROR_CHECK(err_code);                  
        }
        else
        {
            #ifdef DEBUG_BLE_BONDING
                printf("[PWD]: bonding fault \r\n");
            #endif

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
        (p_response->handle ==  g_DeviceInformation.bonding_service.pwdWriteCharW.char_handle) && 
        (p_response->len != 0) //response data
    )
    {
        #ifdef DEBUG_BLE_BONDING
            printf("[PWD]: PWD write result:Leng_%d,0x%X,0x%X ,0x%X ,0x%X \r\n ", 
                                    p_response->len, 
                                    p_response->data[0], 
                                    p_response->data[1], 
                                    p_response->data[2], 
                                    p_response->data[3]);
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
    if (p_response->handle == g_DeviceInformation.bonding_service.pwdResultCharR.char_handle)
    {

        #ifdef DEBUG_BLE_BONDING
              printf("[PWD]: PWD Read result:Leng_%d,0x%X,0x%X,0x%X,0x%X\r\n", p_response->len, 
                        p_response->data[0], p_response->data[1], 
                        p_response->data[2], p_response->data[3]);
        #endif        
    }
    tx_buffer_process();
}

/**
  * @brief  Function for writing data to the client
  * @param  None
  * @retval None
  */
static void write_to_client(DeviceInfomation_t *p_dev, uint8_t *pData,uint8_t length)
{
    tx_message_t * p_msg;
    
    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;
    
    p_msg->req.write_req.gattc_params.handle = p_dev->bonding_service.pwdWriteCharW.char_handle;
    p_msg->req.write_req.gattc_params.len = length;
    p_msg->req.write_req.gattc_params.p_value = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    memcpy(p_msg->req.write_req.gattc_value, pData, length);
    p_msg->conn_handle = p_dev->conn_handle;
    p_msg->type = WRITE_REQ;
    tx_buffer_process();
}
  
/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


