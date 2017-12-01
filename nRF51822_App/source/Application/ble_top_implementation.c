/**
  ****************************************************************************************
  * @file    ble_top_implementation.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   ble communication top level implementation
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
#include "ble_top_implementation.h"





/* peripheral related. */
#define DEVICE_NAME                      "NordicLESCApp"                            /**< Name of device used for advertising. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 40                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       60                                        /**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */




/* private variables define */
APP_TIMER_DEF(m_ble_scanCTL_timer_id);                              /**<ble scan control timer. */
APP_TIMER_DEF(m_ble_char_find_manage_timer_id);                     /**<timer to poll the status of character find manage. */
APP_TIMER_DEF(m_ble_connect_bonding_status_poll_timer_id);          /**<timer to poll the status of connect and bonding. */
APP_TIMER_DEF(m_ble_reconnect_timer_id);                            /**<timer for ble reconnect control. */


/* private function declare */
static void ble_stack_init(void);
static void sys_evt_dispatch(uint32_t sys_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void db_discovery_init(void);
static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
static void gap_params_init(void);
static void conn_params_init(void);
static void conn_params_error_handler(uint32_t nrf_error);
static void advertising_init(void);
//static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static ret_code_t start_ble_scan(void);
static ret_code_t stop_ble_scan(void);
static void services_init(void);
static uint32_t ble_central_service_init(void);
static void vTimerStopBleScanCB(void * p_context);
static void vTimerCharFindStatusPollCB(void *p_context);
static void vTimerConnectBondingStatusPollCB(void *p_context);
static void vTimerBleReconnectCB(void * p_context);
static void reset_scan_list(void);
static void reset_target_Connnect_DevInfo(void);
static ret_code_t ble_central_connect_target(ble_gap_addr_t *peerAddr);
static ret_code_t ble_central_disconnect_target(uint16_t conn_handle);

/**
  * @brief  ble_init
  * @param  None
  * @retval None
  */
void ble_init(void)
{	
    g_DeviceInformation.conn_handle = BLE_CONN_HANDLE_INVALID;
    g_DeviceInformation.char_find_manage = YWK_CHARACTER_NONE;
    g_DeviceInformation.isNRFBusy = false;
       
	/* ble stack init */
	ble_stack_init();	
	db_discovery_init();
	gap_params_init();
	conn_params_init();
	services_init();
    ble_central_service_init();
	advertising_init();	
}
/**
  * @brief  ble_task_handler
  * @param  *p_event_data,event_size
  * @retval None
  */
void ble_task_handler(void *p_event_data,uint16_t event_size)
{
    BLE_MSG_T               *bleRxEventMsgValue = (BLE_MSG_T *)p_event_data;
    BLE_MSG_T               bleEventMsgValue; 
    uint32_t                err_code = NRF_ERROR_NULL;
    uint8_t                 i = 0;
   

     
    /* 接收到消息，对消息事件进行处理 */
    switch(bleRxEventMsgValue->eventID)
    {
        case EVENT_APP_BLE_START_SCAN:
        {
            #ifdef DEBUG_BLE_SCAN
                printf("start scan\r\n");
            #endif
            /* start ble scan */
            start_ble_scan();
            
            /* 清空扫描列表 */
            reset_scan_list();
            
            /* stop the timer of scan control */
            err_code = app_timer_stop(m_ble_scanCTL_timer_id);
            APP_ERROR_CHECK(err_code);
                       
            /* start the timer of scan control */
            err_code = app_timer_start(m_ble_scanCTL_timer_id,STOP_SCAN_TIME,NULL);
            APP_ERROR_CHECK(err_code);  
        }
        break;
        case EVENT_APP_BLE_STOP_SCAN:
        {
            #ifdef DEBUG_BLE_SCAN
                printf("stop scan\r\n");
            #endif 
            
            /* stop ble scan */
            stop_ble_scan();
            
            /* reset the target connnect device info */
            reset_target_Connnect_DevInfo();
            
            /* 找出扫描列表中最大信号质量强度对应的手环 */
            for(i=0;i<MAX_SCAN_LIST_NUM;i++)
            {
                if(gScanList[i].isValid == false)  //说明在扫描事件中已将填充对应的扫描列表位置
                {
                    if(g_DeviceInformation.rssi < gScanList[i].rssi)
                    {                      
                        g_DeviceInformation.rssi = gScanList[i].rssi;
                        g_DeviceInformation.sn = (uint32_t)((uint32_t)(gScanList[i].sn[0]<<24) + (uint32_t)(gScanList[i].sn[1]<<16) +
                                                            (uint32_t)(gScanList[i].sn[2]<<8) + (uint32_t)(gScanList[i].sn[3]));
                        memcpy(&g_DeviceInformation.MACaddr,&gScanList[i].MACaddr,sizeof(ble_gap_addr_t));
                        g_DeviceInformation.isValid = true;
                    }
                }                                
            }
            
            if(g_DeviceInformation.isValid == true)  //说明找到了可以连接的设备
            {
                #ifdef DEBUG_BLE_BONDING
                    printf("target SN:%d,Mac:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,type:%d\r\n",g_DeviceInformation.sn,
                                            g_DeviceInformation.MACaddr.addr[0],
                                            g_DeviceInformation.MACaddr.addr[1],
                                            g_DeviceInformation.MACaddr.addr[2],
                                            g_DeviceInformation.MACaddr.addr[3],
                                            g_DeviceInformation.MACaddr.addr[4],
                                            g_DeviceInformation.MACaddr.addr[5],
                                            g_DeviceInformation.MACaddr.addr_type
                            );
                #endif
                /* 发送连接蓝牙连接事件 */
                bleEventMsgValue.eventID = EVENT_APP_BLE_CONNECT;
                err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                APP_ERROR_CHECK(err_code); 
            }
            else  //未找到可以连接的设备
            {
                //@todo
                #ifdef DEBUG_BLE_BONDING
                    printf("not find the device to connnect\r\n");
                #endif
            }
        }
        break;
        case EVENT_APP_BLE_CONNECT:
        {
            /* 连接设备 */
            err_code = ble_central_connect_target(&g_DeviceInformation.MACaddr);
            APP_ERROR_CHECK(err_code); 
        }
        break;
        case EVENT_APP_BLE_DISCONNECT:
        {
            /* 主动断开设备连接 */
            err_code = ble_central_disconnect_target(g_DeviceInformation.conn_handle);
            APP_ERROR_CHECK(err_code);
        }
        break;
        case EVENT_APP_BLE_PASSKEY_WRITE:
        {
            /* 写PIN码 */
            ble_central_passkey_write(&g_DeviceInformation);
        }
        break;
        case EVENT_APP_BLE_BONDING_OK:
        {
            /* passkey confirm ok */
            app_cmu_ble_connect_status_send(g_DeviceInformation.sn,g_DeviceInformation.MACaddr.addr,true);
        }
        break;
        case EVENT_APP_BLE_SYNC_TIME:
        {
            /* 同步peer设备时间 */
            if(g_setTimeValue != 0)
            {
                ble_central_synctime_write(&g_DeviceInformation,g_setTimeValue);
                app_cmu_time_set_status_rsp(g_DeviceInformation.sn,true);
            }
        }
        break;
        case EVENT_APP_BLE_MONITOR_TEMPLATE_WRITE:
        {            
            /* 设置监护方案 */            
            ble_central_monitor_template_write(&g_DeviceInformation);
        }
        break;
        case EVENT_APP_BLE_SERVICE_CHAR_FIND_COMPLATE:
        {
            #ifdef DEBUG_BLE_BONDING
                printf("EVENT_APP_BLE_SERVICE_CHAR_FIND_COMPLATE\r\n");
            #endif
                       
            /* set the g_connect_bonding_status to STATUS_WRITE_PIN */
            g_connect_bonding_status = STATUS_WRITE_PIN;
            
            /* start the timer to poll the status of connect and bonding */
            stop_connect_bonding_status_polling_timer_timer(&g_DeviceInformation);
            start_connect_bonding_status_polling_timer_timer(&g_DeviceInformation);
        }
        break;
        default:break;
    }
}

/**
  * @brief  allocate an app timer to control the ble scan
  * @param  None
  * @retval None
  */
void ble_scan_control_timer_init(void)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    err_code = app_timer_create(&m_ble_scanCTL_timer_id,APP_TIMER_MODE_SINGLE_SHOT,vTimerStopBleScanCB);
    APP_ERROR_CHECK(err_code);    
}
/**
  * @brief  start the time of polling the status of character find management
  * @param  *p_dev
  * @retval None
  */
void start_character_find_status_manage_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
     
    /* start the timer of polling the status of character find management */
    err_code = app_timer_start(m_ble_char_find_manage_timer_id,FIND_CHAR_STATUS_POLL_TIME,p_dev);
    APP_ERROR_CHECK(err_code); 
}
/**
  * @brief  stop the time of polling the status of character find management
  * @param  *p_dev
  * @retval None
  */
void stop_character_find_status_manage_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    /* stop the timer of character find status polling */
    err_code = app_timer_stop(m_ble_char_find_manage_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**
  * @brief  allocate an app timer to reconnect the peers when ble disconnect
  * @param  None
  * @retval None
  */
void ble_reconnect_timer_init(void)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    err_code = app_timer_create(&m_ble_reconnect_timer_id,APP_TIMER_MODE_REPEATED,vTimerBleReconnectCB); 
    APP_ERROR_CHECK(err_code);    
}
/**
  * @brief  start the time of ble reconnect
  * @param  *p_dev
  * @retval None
  */
void start_ble_reconnect_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
     
    /* start the timer of polling the status of character find management */
    err_code = app_timer_start(m_ble_reconnect_timer_id,BLE_RECONNECT_TIME,p_dev);
    APP_ERROR_CHECK(err_code); 
}
/**
  * @brief  stop the time of ble reconnect
  * @param  *p_dev
  * @retval None
  */
void stop_ble_reconnect_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    /* stop the timer of character find status polling */
    err_code = app_timer_stop(m_ble_reconnect_timer_id);
    APP_ERROR_CHECK(err_code);
}


/**
  * @brief  allocate an app timer to poll the status of connect and bonding progress
  * @param  None
  * @retval None
  */
void ble_connect_bonding_status_polling_timer_init(void)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    err_code = app_timer_create(&m_ble_connect_bonding_status_poll_timer_id,APP_TIMER_MODE_REPEATED,vTimerConnectBondingStatusPollCB); 
    APP_ERROR_CHECK(err_code);    
}
/**
  * @brief  start the time of polling the status of connect and bonding
  * @param  *p_dev
  * @retval None
  */
void start_connect_bonding_status_polling_timer_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
     
    /* start the timer of polling the status of character find management */
    err_code = app_timer_start(m_ble_connect_bonding_status_poll_timer_id,CONNECT_BONDING_STATUS_POLL_TIME,p_dev);
    APP_ERROR_CHECK(err_code); 
}
/**
  * @brief  stop the time of polling the status of connect and bonding
  * @param  *p_dev
  * @retval None
  */
void stop_connect_bonding_status_polling_timer_timer(DeviceInfomation_t *p_dev)
{
    ret_code_t  err_code = NRF_ERROR_NULL;
    
    /* stop the timer of character find status polling */
    err_code = app_timer_stop(m_ble_connect_bonding_status_poll_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**
  * @brief  ble_connect_bonding_tatus_execute
  * @note   the execute of connect and bonding status
  * @param  CONNECT_BONDING_STATUS_T status
  * @retval None
  */
uint32_t ble_connect_bonding_tatus_execute(CONNECT_BONDING_STATUS_T status)
{
    BLE_MSG_T               bleEventMsgValue;
    APP_DATA_SYNC_MSG_T     syncDataEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;
    
    if(g_DeviceInformation.isNRFBusy == false)
    {        
        /* polling the status of connect and bonding progress */
        switch(status)
        {
            case STATUS_WRITE_PIN:
            {
                bleEventMsgValue.eventID = EVENT_APP_BLE_PASSKEY_WRITE;
                
                err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                APP_ERROR_CHECK(err_code);
            }
            break;
            case STATUS_WRITE_TIME:
            {
                bleEventMsgValue.eventID = EVENT_APP_BLE_SYNC_TIME;
                
                err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                APP_ERROR_CHECK(err_code);    
            }
            break;
            case STATUS_WRITE_MONITOR_TEMPLATE:
            {
                bleEventMsgValue.eventID = EVENT_APP_BLE_MONITOR_TEMPLATE_WRITE;
                
                err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                APP_ERROR_CHECK(err_code); 
            }
            break;
            case STATUS_START_SYNC_DATA:
            {
                syncDataEventMsgValue.eventID = EVENT_APP_DATA_SYNC_START_SYNC_DATA;

                err_code = app_sched_event_put(&syncDataEventMsgValue,sizeof(syncDataEventMsgValue),app_data_sync_task_handler);
                APP_ERROR_CHECK(err_code); 
            }
            break;
            default:break;            
        }
    }
    else  //nRF busy
    {
        err_code = NRF_ERROR_BUSY;
    }
    return err_code;
}

/**@brief Function for start ble scanning.
 */
static ret_code_t start_ble_scan(void)
{
    ret_code_t      err_code = NRF_SUCCESS;
    uint32_t        count = 0;

    //check if there are no flash operations in progress
    err_code = fs_queued_op_count_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count == 0)
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Running speed and cadence UUIDs.
        scan_start();  
    }
    
    return err_code;
}
/**
  * @brief  Function for stopping ble scan. 
  * @param  None
  * @retval ret_code_t
  */
static ret_code_t stop_ble_scan(void)
{
    ret_code_t      err_code = NRF_SUCCESS;  

    err_code = scan_stop();
    
    return err_code;
}
/**
  * @brief  Function for central device to connect target device
  * @param  *peerAddr,the mac address of target device
  * @retval ret_code_t
  */
static ret_code_t ble_central_connect_target(ble_gap_addr_t *peerAddr)
{
    ret_code_t      err_code = NRF_SUCCESS;  

    err_code = ble_central_connect(peerAddr);
    
    return err_code;    
}
/**
  * @brief  Function for central device to disconnect target device
  * @param  conn_handle
  * @retval ret_code_t
  */
static ret_code_t ble_central_disconnect_target(uint16_t conn_handle)
{
    ret_code_t      err_code = NRF_SUCCESS;
    
    err_code = ble_central_disconnect(conn_handle);
    
    return err_code;
}
/**
  * @brief  Function for initializing the BLE stack. 
  * @note   Initializes the SoftDevice and the BLE event interrupts.
  * @param  None
  * @retval None
  */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module,Use system event for scheduler 
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
//static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
//{
//    switch (ble_adv_evt)
//    {
//        case BLE_ADV_EVT_FAST:
//            bsp_led_toggle(LED2);
//            break;

//        case BLE_ADV_EVT_IDLE:
//        {
//            ret_code_t err_code;
//            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//            APP_ERROR_CHECK(err_code);
//        } break;

//        default:
//            // No implementation needed.
//            break;
//    }
//}

/**
  * @brief  Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
  * @note   This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
  * @param  p_ble_evt   Bluetooth stack event.
  * @retval None
  */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	uint16_t conn_handle;
    uint16_t role;
	
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
	
	// The connection handle should really be retrievable for any event type.
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    role        = ble_conn_state_role(conn_handle);
	
    // Based on the role this device plays in the connection, dispatch to the right applications.
    if (role == BLE_GAP_ROLE_PERIPH)
    {
		ble_advertising_on_ble_evt(p_ble_evt);
		ble_conn_params_on_ble_evt(p_ble_evt);
	}
	else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
	{
        /** on_ble_central_evt will update the connection handles, so we want to execute it
         * after dispatching to the central applications upon disconnection. */
        on_ble_central_evt(p_ble_evt);
        
        if (conn_handle < CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT)
        {
            ble_db_discovery_on_ble_evt(&g_ble_db_discovery[conn_handle], p_ble_evt);
            ble_bonding_ble_evt_handler(p_ble_evt);
            ble_devinfo_manage_ble_evt_handler(p_ble_evt);
            ble_monitor_template_ble_evt_handler(p_ble_evt);
            ble_sync_data_ble_evt_handler(p_ble_evt);
        }
	}  
}
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
//    // Dispatch the system event to the fstorage module, where it will be
//    // dispatched to the Flash Data Storage (FDS) module.
//    fs_sys_event_handler(sys_evt);

//    // Dispatch to the Advertising module last, since it will check if there are any
//    // pending flash operations in fstorage. Let fstorage process system events first,
//    // so that it can report correctly to the Advertising module.
//    ble_advertising_on_sys_evt(sys_evt);
}

/**
  * @brief  Function for the GAP initialization.
  * @note   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
  *         device including the device name, appearance, and the preferred connection parameters.
  * @param  None
  * @retval None
  */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
  * @brief  Function for initializing the Connection Parameters module.
  * @note   Connection Parameters
  * @param  None
  * @retval None
  */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling database discovery events.
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 * @param[in] p_event  Pointer to the database discovery event.
 * @retval None
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_bonding_db_discovery_evt_handler(&g_DeviceInformation, p_evt);
    ble_devinfo_manage_db_discovery_evt_handler(&g_DeviceInformation, p_evt);
    ble_monitor_template_db_discovery_evt_handler(&g_DeviceInformation, p_evt);
    ble_sync_data_db_discovery_evt_handler(&g_DeviceInformation, p_evt);
}

/**
  * @brief  Database discovery initialization.
  * @note   Database discovery initialization. 
  * @param  None
  * @retval None
  */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
    
    /* allocate a timer to poll the status of character find management */
    err_code = app_timer_create(&m_ble_char_find_manage_timer_id,APP_TIMER_MODE_REPEATED,vTimerCharFindStatusPollCB);
    APP_ERROR_CHECK(err_code);
}

/**
  * @brief  Function for initializing the Advertising functionality.
  * @note   Function for initializing the Advertising functionality.
  * @param  None
  * @retval None
  */
static void advertising_init(void)
{
//    uint32_t               				err_code;
//    ble_advdata_t          				advdata;
//	ble_advdata_t              			scanrsp;
//    ble_adv_modes_config_t 				options;
//	ble_uuid_t 							scanrsp_uuids[] = {BLE_UUID_PASSKEY_AUTH_SERVICE,BLE_UUID_TYPE_BLE};
//    ble_uuid_t							adv_uuids[] = {CHECK_UP_UUID_SERVICE,BLE_UUID_TYPE_BLE};

//    // Build advertising data struct to pass into @ref ble_advertising_init.
//    memset(&advdata, 0, sizeof(advdata));

//    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance      = true;
//    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
//    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    advdata.uuids_complete.p_uuids  = adv_uuids;


//	memset(&scanrsp, 0, sizeof(scanrsp));
//    scanrsp.uuids_complete.uuid_cnt = sizeof(scanrsp_uuids) / sizeof(scanrsp_uuids[0]);
//    scanrsp.uuids_complete.p_uuids  = scanrsp_uuids;
//	
//    memset(&options, 0, sizeof(options));
//    options.ble_adv_fast_enabled  = true;
//    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
//    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

//    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
//    APP_ERROR_CHECK(err_code);

}
/**
  * @brief  Function for initializing the service
  * @note   Function for initializing the service
  * @param  None
  * @retval None
  */
static void services_init(void)
{

}
/**
  * @brief  Function for initializing the ble central service
  * @param  None
  * @retval status
  */
static uint32_t ble_central_service_init(void)
{
    uint32_t    ret = NRF_ERROR_INVALID_PARAM;
    
    ret = ble_central_service_bonding_init(&g_DeviceInformation.bonding_service);
    APP_ERROR_CHECK(ret);
    
    ret = ble_central_service_monitor_template_init(&g_DeviceInformation.monitor_template_service);
    APP_ERROR_CHECK(ret);    
    
    ret = ble_central_service_devinfo_manage_init(&g_DeviceInformation.devinfo_manage_service);
    APP_ERROR_CHECK(ret); 

    ret = ble_central_service_sync_data_init(&g_DeviceInformation.sync_data_service);
    APP_ERROR_CHECK(ret);
    
    return ret;
}
/**
  * @brief  vTimerStopBleScanCB
  * @note   stop ble scan timer call back
  * @param  void * p_context
  * @retval None
  */
static void vTimerStopBleScanCB(void * p_context)
{
    BLE_MSG_T               bleEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;
    
    bleEventMsgValue.eventID = EVENT_APP_BLE_STOP_SCAN;
    
    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
    APP_ERROR_CHECK(err_code);    
}
/**
  * @brief  vTimerCharFindStatusPollCB
  * @note   polling the status of character find manage
  * @param  void * p_context
  * @retval None
  */
static void vTimerCharFindStatusPollCB(void *p_context)
{
    ret_code_t              err_code = NRF_ERROR_NULL;
    DeviceInfomation_t      *dev = (DeviceInfomation_t *)p_context;
    BLE_MSG_T               bleEventMsgValue;
    dev = dev;
        
    if(g_DeviceInformation.char_find_manage == YWK_CHARACTER_ALL)  //所有的character已发现完成
    {
        /* stop the timer of character find status polling */
        err_code = app_timer_stop(m_ble_char_find_manage_timer_id);
        APP_ERROR_CHECK(err_code);
        
        #ifdef DEBUG_BLE_CONNECT
            printf("YWK character find complete:0x%x\r\n",g_DeviceInformation.char_find_manage);
        #endif
 
        bleEventMsgValue.eventID = EVENT_APP_BLE_SERVICE_CHAR_FIND_COMPLATE;
        
        err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
        APP_ERROR_CHECK(err_code);                 
    }
}

/**
  * @brief  vTimerConnectBondingStatusPollCB
  * @note   polling the status of connect and bonding status
  * @param  void * p_context
  * @retval None
  */
static void vTimerConnectBondingStatusPollCB(void *p_context)
{
    BLE_MSG_T               bleEventMsgValue;
    APP_DATA_SYNC_MSG_T     syncDataEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;
    
    if(g_DeviceInformation.conn_handle != BLE_CONN_HANDLE_INVALID) //蓝牙连接中
    {
        if(g_DeviceInformation.isNRFBusy == false)
        {        
            /* polling the status of connect and bonding progress */
            switch(g_connect_bonding_status)
            {
                case STATUS_WRITE_PIN:
                {
                    bleEventMsgValue.eventID = EVENT_APP_BLE_PASSKEY_WRITE;
                    
                    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                    APP_ERROR_CHECK(err_code);
                    
                    g_connect_bonding_status = STATUS_NONE;
                }
                break;
                case STATUS_WRITE_TIME:
                {
                    bleEventMsgValue.eventID = EVENT_APP_BLE_SYNC_TIME;
                    
                    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                    APP_ERROR_CHECK(err_code); 
                    
                    g_connect_bonding_status = STATUS_NONE;      
                }
                break;
                case STATUS_WRITE_MONITOR_TEMPLATE:
                {
                    bleEventMsgValue.eventID = EVENT_APP_BLE_MONITOR_TEMPLATE_WRITE;
                    
                    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
                    APP_ERROR_CHECK(err_code); 
                    
                    g_connect_bonding_status = STATUS_NONE;
                }
                break;
                case STATUS_START_SYNC_DATA:
                {
                    syncDataEventMsgValue.eventID = EVENT_APP_DATA_SYNC_START_SYNC_DATA;

                    err_code = app_sched_event_put(&syncDataEventMsgValue,sizeof(syncDataEventMsgValue),app_data_sync_task_handler);
                    APP_ERROR_CHECK(err_code);                 
                    
                    g_connect_bonding_status = STATUS_CONNECT_BONDING_COMPLATE;
                    
                    /* stop the time when connect and bonding progress complate */
                    stop_connect_bonding_status_polling_timer_timer(&g_DeviceInformation); 
                }
                break;
                default:break;            
            }
        }
    }
    else //蓝牙已断开
    {
        /* stop the time when connect and bonding progress complate */
        stop_connect_bonding_status_polling_timer_timer(&g_DeviceInformation);         
    }
}

/**
  * @brief  vTimerBleReconnectCB
  * @note   call back for ble reconnect
  * @param  void * p_context
  * @retval None
  */
static void vTimerBleReconnectCB(void * p_context)
{
    BLE_MSG_T               bleEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;   

    #ifdef DEBUG_BLE_CONNECT
        printf("ble reconnected\r\n");
    #endif    
    /* 发送连接蓝牙连接事件 */
    bleEventMsgValue.eventID = EVENT_APP_BLE_CONNECT;
    err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
    APP_ERROR_CHECK(err_code);     
}

/**
  * @brief  reset_scan_list
  * @param  None
  * @retval None
  */
static void reset_scan_list(void)
{
    uint8_t i = 0;
    
    for(i=0;i<MAX_SCAN_LIST_NUM;i++)
    {
        memset(&gScanList[i],0,sizeof(BLE_SCAN_LIST_T));
        gScanList[i].isValid = true;
        gScanList[i].rssi = DEFAULT_REF_RSSI;
    }
}
/**
  * @brief  reset_target_Connnect_DevInfo
  * @param  None
  * @retval None
  */
static void reset_target_Connnect_DevInfo(void)
{
    memset(&g_DeviceInformation,0,sizeof(DeviceInfomation_t));
    g_DeviceInformation.isValid = false;
    g_DeviceInformation.rssi = DEFAULT_REF_RSSI;
}

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

