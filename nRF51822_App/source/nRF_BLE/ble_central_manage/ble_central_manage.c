/**
  ****************************************************************************************
  * @file    ble_central_manage.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-29
  * @brief   ble central manage
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_central_manage.h"



/* private variables declare -------------------------------------------------*/

/* private function decalre --------------------------------------------------*/
static void scan_advertise_data_report(const ble_gap_evt_adv_report_t *adv_report);
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata);
static void reset_ble_central_all_service(void);

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};
/**
 * @brief Parameters used for connnecting.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t) MIN_CONNECTION_INTERVAL, 
    (uint16_t) MAX_CONNECTION_INTERVAL, 
    0,
    (uint16_t) SUPERVISION_TIMEOUT 
};


/**
  * @brief  Function for initiating scanning.
  * @note   Function for initiating scanning.
  * @param  None
  * @retval None
  */
ret_code_t scan_start(void)
{
    ret_code_t      err_code = NRF_SUCCESS;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
    return err_code;
}

/**
  * @brief  Function for initiating stop scanning.
  * @note   Function for initiating stop scanning.
  * @param  None
  * @retval None
  */
ret_code_t scan_stop(void)
{
    ret_code_t      err_code = NRF_SUCCESS;
    
	err_code = sd_ble_gap_scan_stop();
    
    return err_code;
}
/**
  * @brief  Function for central device connect the periphral device 
  * @param  *peerAddr, mac address
  * @retval ret_code_t
  */
ret_code_t ble_central_connect(ble_gap_addr_t *peerAddr)
{
    ret_code_t      err_code = NRF_SUCCESS;
    
    err_code = sd_ble_gap_connect(peerAddr, &m_scan_params, &m_connection_param);
    
    return err_code;
}
/**
  * @brief  Function for central device to cancel connect the periphral device 
  * @param  None
  * @retval ret_code_t
  */
ret_code_t ble_central_connect_cancel(void)
{
    ret_code_t      err_code = NRF_SUCCESS;
    
    err_code = sd_ble_gap_connect_cancel();
    
    return err_code;
}
/**
  * @brief  Function for central device disconnect the periphral device 
  * @param  conn_handle
  * @retval ret_code_t
  */
ret_code_t ble_central_disconnect(uint16_t conn_handle)
{
    ret_code_t      err_code = NRF_SUCCESS;
    
    err_code = sd_ble_gap_disconnect(conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    
    return err_code;
}

/**
  * @brief  Function for handling BLE Stack events concerning central applications.
  * @note   This function keeps the connection handles of central applications up-to-date. It
  *         parses scanning reports, initiating a connection attempt to peripherals when a target UUID
  *         is found, and manages connection parameter update requests.
  *         Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
  *         should be dispatched to the target application before invoking this function.
  * @param  p_ble_evt   Bluetooth stack event.
  * @retval None
  */
void on_ble_central_evt(const ble_evt_t * const p_ble_evt)
{
    const ble_gap_evt_t   		* const p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t                    err_code;

    switch (p_ble_evt->header.evt_id)
    {
        /** Upon connection, check which peripheral has connected, initiate DB
         *  discovery,resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
			#ifdef DEBUG_BLE_CONNECT
				printf("CENTRAL: connected\r\n");
			#endif
  
			/* 开始查找当前连接中peripheral端所包含的服务 */
			#ifdef DEBUG_BLE_CONNECT
                printf("CENTRAL: start to find service on conn_handle 0x%x\r\n", p_gap_evt->conn_handle);
			#endif
            
            g_DeviceInformation.conn_handle = p_gap_evt->conn_handle;
                     
            /* Reset all the service and char */
            reset_ble_central_all_service();

			APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT);
			err_code = ble_db_discovery_start(&g_ble_db_discovery[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
			APP_ERROR_CHECK(err_code);
            
            /* stop the ble reconnect control timer */
            stop_ble_reconnect_timer(&g_DeviceInformation);             
            
            /* start the time to poll the character find status */
            start_character_find_status_manage_timer(&g_DeviceInformation);
            
            /** check if we should be looking for more peripherals to connect to. */
            if (ble_conn_state_n_centrals() <= CENTRAL_LINK_COUNT)
            {

            }
            else
            {
				// 已达到连接设备数量上限
            }
        } break; // BLE_GAP_EVT_CONNECTED

        /** Upon disconnection, reset the connection handle of the peer which disconnected, and start scanning again. */
        case BLE_GAP_EVT_DISCONNECTED:
        {
			#ifdef DEBUG_BLE_CONNECT
				printf("CENTRAL: disconnected (reason: %d)\r\n",p_gap_evt->params.disconnected.reason);
			#endif
            memset(&g_ble_db_discovery[p_gap_evt->conn_handle],0,sizeof(ble_db_discovery_t));
            g_DeviceInformation.conn_handle = BLE_CONN_HANDLE_INVALID;
            g_DeviceInformation.char_find_manage = YWK_CHARACTER_NONE;
            
            /* reset the g_connect_bonding_status to default status */
            g_connect_bonding_status = STATUS_NONE;
            
            /* Reset all the service and char */
            reset_ble_central_all_service();
            
            /* Reset the data sync variables */
            reset_data_sync();
            
            /* stop the time to poll the character find status */
            stop_character_find_status_manage_timer(&g_DeviceInformation);
            stop_connect_bonding_status_polling_timer_timer(&g_DeviceInformation);
            app_cmu_ble_connect_status_send(g_DeviceInformation.sn,g_DeviceInformation.MACaddr.addr,false);
            
            /* start the ble reconnect control timer */
            start_ble_reconnect_timer(&g_DeviceInformation);
            
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
			scan_advertise_data_report(&p_gap_evt->params.adv_report);

        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
				#ifdef DEBUG_BLE_CONNECT
					printf("CENTRAL: Connection Request timed out.\r\n");
				#endif
            }
        } break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
			#ifdef DEBUG_BLE_CONNECT
				printf("CENTRAL: GATT Client Timeout.\r\n");
			#endif
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
			#ifdef DEBUG_BLE_CONNECT
				printf("CENTRAL: GATT Server Timeout.\r\n");
			#endif
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}
/**
  * @brief  Function for handling advertise data report
  * @note   Central端处理扫描到的广播数据报告
  * @param  ble_gap_evt_adv_report_t *adv_report
  * @retval None
  */
static void scan_advertise_data_report(const ble_gap_evt_adv_report_t *adv_report)
{
	uint8_t				i = 0;
    data_t              sn;
    data_t              adv_data;
    data_t              advUUID;
    uint32_t            SN_Dec = 0;         //十进制形式的序列号
    uint16_t            UUID=0;             //提取广播中的服务ID，以识别是云卫康设备
    
    
	#ifdef DEBUG_BLE_CONNECT
//		printf("peer addr:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,\r\n",adv_report->peer_addr.addr[0],\
//											adv_report->peer_addr.addr[1],adv_report->peer_addr.addr[2],\
//											adv_report->peer_addr.addr[3],adv_report->peer_addr.addr[4],\
//											adv_report->peer_addr.addr[5]);
//		printf("Adv Data:");
//		for(i=0;i<adv_report->dlen;i++)
//		{
//			printf("0x%x, ",adv_report->data[i]);
//		}
//        printf("rssi:%d",adv_report->rssi);
//		printf("\r\n");
	#endif
    
    if(adv_report->rssi > DEFAULT_REF_RSSI)
    {
        
        for(i=0;i<MAX_SCAN_LIST_NUM;i++)
        {
            if(gScanList[i].isValid == true) //找到可以存放的位置
            {
                break;
            }
        }
                
        // Initialize advertisement report for parsing.
        adv_data.p_data = (uint8_t *) adv_report->data;
        adv_data.data_len = adv_report->dlen;
        
        /* 判断是云卫康的设备 */            
        adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,&adv_data,&advUUID);
        UUID = (uint16_t)((advUUID.p_data[1]<<8) + advUUID.p_data[0]);
        if(UUID == YWK_DEVICE_CONFIRM_UUID)  //是云卫康设备
        {    
            /* 获取设备SN */
            adv_report_parse(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,&adv_data,&sn);
            
            SN_Dec = (uint32_t)((sn.p_data[0]<<24)+(sn.p_data[1]<<16)+(sn.p_data[2]<<8)+sn.p_data[3]<<0);
            #ifdef DEBUG_BLE_CONNECT
                printf("SN:%d,RSSI:%d,i:%d\r\n",SN_Dec,adv_report->rssi,i);
            #endif
            if(SN_Dec != 0)  //手环序列号不为0才处理
            {
                memcpy(gScanList[i].sn,sn.p_data,4);
                memcpy(gScanList[i].MACaddr.addr,adv_report->peer_addr.addr,BLE_GAP_ADDR_LEN);
                gScanList[i].MACaddr.addr_type = adv_report->peer_addr.addr_type;
                gScanList[i].isValid = false;
                gScanList[i].rssi = adv_report->rssi;
            }
        }
    }
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse (uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**
 * @brief reset all service and characteristic
 *
 * @param[in] none
 *
 * @retval none.
 */
static void reset_ble_central_all_service(void)
{
    reset_ble_central_bonding_service(&g_DeviceInformation.bonding_service);
    reset_ble_central_devinfo_manage_service(&g_DeviceInformation.devinfo_manage_service);
    reset_ble_central_monitor_template_service(&g_DeviceInformation.monitor_template_service);
    reset_ble_central_sync_data_service(&g_DeviceInformation.sync_data_service);
}
/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

