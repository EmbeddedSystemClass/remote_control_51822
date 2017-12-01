/******************************************************************************
 *
 * COPYRIGHT:
 *   Copyright (c)  2005-2050   GnsTime  Inc.    All rights reserved.
 *
 *   This is unpublished proprietary source code of GnsTime Inc.
 *   The copyright notice above does not evidence any actual or intended
 *   publication of such source code.
 *
 * FILE NAME:
 *   apl_B2E.c
 * DESCRIPTION:
 *
 * HISTORY:
 *   2016年1月12日        Arvin         Create/Update
 *
 *****************************************************************************/
#include "apl_ble.h"

/* global variable declare */
DEV_BLE_INFO_T     gBLEDevInfo;

/* private function declare */
static uint8_t syncDataBuf[MAX_BLE_SPI_LEN] = {0};
FIL dataSyncFile;

/* private function declare */
static void ble_connect_process(BLE_PROCESS_STATUS_T status);
static void ble_receive_data_handle(CMU_DATA_T *data);
static HAL_StatusTypeDef ble_data_sync_data_handle(DEV_BLE_INFO_T *bleDevInfo,uint8_t *data,uint8_t len);

QueueHandle_t xQueue_Ble;
SemaphoreHandle_t xMutex_Ble;
TimerHandle_t xTimers_Ble;
TaskHandle_t xBleTaskHandle = NULL;

void vBleTimerCallback( TimerHandle_t pxTimer )
{
//    uint32_t u32TxedMessage = 0;

//    u32TxedMessage = BLE_GET_STATUS_EVENT;
//    xQueueSend(xQueue_Ble, &u32TxedMessage, ( TickType_t ) 100 );
}

static void vBleTask (void * argument)
{

    BLE_MSG_T       bleRxedMessage;
    CMU_DATA_T      cmuData;
    uint16_t        curLen = 0;
    uint32_t        remainLen = 0;

    /* watch dog register */
    WDT_Register(WDT_BLE_TASK_ID);

    /* ble init */
    ble_init();

    /* set default ble message event */
    bleRxedMessage.event = BLE_DEFAULT_EVENT;
    gBLEDevInfo.pFile = &dataSyncFile;

    /* for test */
    gBLEDevInfo.isStoreFileCreat = false;

    while (1)
    {
        if(pdTRUE == xQueueReceive(xQueue_Ble, &bleRxedMessage, ( TickType_t ) 10/portTICK_PERIOD_MS) )
        {
            /* handle with the ble received event */
            switch(bleRxedMessage.event)
            {
                case BLE_SPI_READ_EVENT:
                {
                    /* read data from spi */
                    ble_receive_data(&cmuData);
                    /* handle with spi data */
                    ble_receive_data_handle(&cmuData);
                }
                break;
                case BLE_START_SCAN_EVENT:
                {
                    /* start ble scan */
                    ble_start_scan();
                }
                break;
                case BLE_UNBIND_PEERS:
                {
                    /* unbind the ble peers */
                    ble_unbind_peers(bleRxedMessage.sn,0x01);
                    bleRxedMessage.sn = 0;
                    printf("SN:%d\r\n",gBLEDevInfo.sn);
                }
                break;
                case BLE_SET_TIME_EVENT:
                {
                    ble_connect_process(BLE_PROCESS_SET_TIME);
                }
                break;
                case BLE_SET_MONITOR_TEMPLATE:
                {
                    ble_connect_process(BLE_PROCESS_SET_MONITOR_TEMPLATE);
                }
                break;
                case BLE_START_DATA_SYNC:
                {
                    ble_connect_process(BLE_PROCESS_START_DATA_SYNC);
                }
                break;
                case BLE_DATA_SYNC:  //数据同步数据接收
                {
                    if(bleRxedMessage.sn == gBLEDevInfo.sn)  //所接收到的数据为当前绑定手环
                    {
                        /* store the data to flash */
                        ble_data_sync_data_handle(&gBLEDevInfo,bleRxedMessage.buf,bleRxedMessage.len);

                        curLen = bleRxedMessage.buf[0]*255 + bleRxedMessage.buf[1];
                        remainLen = (uint32_t)((uint32_t)(bleRxedMessage.buf[2]<<24) + (uint32_t)(bleRxedMessage.buf[3]<<16) +
                                                (uint32_t)(bleRxedMessage.buf[4]<<8)+ (uint32_t)(bleRxedMessage.buf[5]));
                        #ifdef BLE_DATA_SYNC_DEBUG
                            printf("curLen:%d,remainLen:%d\r\n",curLen,remainLen);
                        #endif
                        /* response ack len to the peers */
                        ble_data_sync_len_ack(bleRxedMessage.sn,curLen);

                        /* send data to the service */
                        if(curLen >= (remainLen+2))
                        {
                            #ifdef BLE_DATA_SYNC_DEBUG
                                printf("send to the service\r\n");
                            #endif
                            gBLEDevInfo.isStoreFileCreat = false;

                            /* 关闭文件 */
                            f_close(gBLEDevInfo.pFile);

                            apl_service_event_t apl_service_event_tx;
                            apl_service_event_tx.apl_event_id = SERVER_UPLOAD_FILE_EVENT;
                            apl_service_event_tx.u32Value = gBLEDevInfo.sn;
                    		xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                        }
                    }
                }
                break;
                case BLE_DEV_INFO_SEND_EVENT:  //发送蓝牙基本信息至51822
                {
                    ble_data_ble_info_send(gBLEDevInfo.sn,gBLEDevInfo.macAddr,BLE_ADDR_LEN);
                }
                break;
                default:break;
            }
        }   /* End of if(pdTRUE == xQueueReceive... */
        else
        {
            WDT_Feed(WDT_BLE_TASK_ID);
        }
    }
}

void CreateBleTask (void)
{
	// Create a queue capable of containing 10 BLE_MSG_T values.
	xQueue_Ble = xQueueCreate(10, sizeof(BLE_MSG_T));
	usr_res_assert(xQueue_Ble != NULL);

    // Create a queue capable of containing 10 uint32_t values.
	xMutex_Ble = xSemaphoreCreateMutex();
	usr_res_assert(xMutex_Ble != NULL);

    xTimers_Ble = xTimerCreate("BleTimer",3000/portTICK_PERIOD_MS, pdTRUE, NULL, vBleTimerCallback);
    usr_res_assert(xTimers_Ble != NULL);

    /* Create that task that handles the console itself. */
    xTaskCreate(vBleTask, "vBleTask", configMINIMAL_STACK_SIZE * 20,NULL, tskBLE_PRIORITY, &xBleTaskHandle);
    configASSERT( xBleTaskHandle);
}
/**
 * @brief ble_start_scan
 * @param[in]  None
 * @retval status
 */
HAL_StatusTypeDef ble_start_scan(void)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_START_SCAN;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = 0;  //sn[0];
    data[i++] = 0;  //sn[1];
    data[i++] = 0;  //sn[2];
    data[i++] = 0;  //sn[3]; 开启扫描，SN设置为0

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}
/**
 * @brief ble_unbind_peers
 * @param[in]  uint32_t sn,uint8_t cmd: 1, unbind the peers.
 * @retval status
 */
HAL_StatusTypeDef ble_unbind_peers(uint32_t sn,uint8_t cmd)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_BLE_UNBIND_PEERS;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);
    
    data[i++] = cmd;

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}

/**
 * @brief ble_set_time
 * @param[in]  uint32_t sn,uint32_t tim
 * @retval status
 */
HAL_StatusTypeDef ble_set_time(uint32_t sn,uint32_t tim)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_SET_TIME;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);

    data[i++] = (uint8_t)(tim >> 24);
    data[i++] = (uint8_t)(tim >> 16);
    data[i++] = (uint8_t)(tim >> 8);
    data[i++] = (uint8_t)(tim);

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}

/**
 * @brief ble_set_monitor_template
 * @param[in]  uint32_t sn,MONITOR_TEMPLATE_T *monitorTemplate
 * @retval status
 */
HAL_StatusTypeDef ble_set_monitor_template(uint32_t sn, MONITOR_TEMPLATE_T *monitorTemplate)
{
    uint8_t                     data[MAX_MONITOR_TEMPLATE_LEN] = {0};
    uint8_t                     i = 0;
    uint8_t                     j = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_SET_MONITOR_TEMPLATE;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);

    for(j=0;j<monitorTemplate->len;j++)
    {
        data[i++] = monitorTemplate->pbuf[j];
    }

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}

/**
 * @brief ble_start_data_sync
 * @param[in]  uint32_t sn
 * @retval status
 */
HAL_StatusTypeDef ble_start_data_sync(uint32_t sn)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_START_DATA_SYNC;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}
/**
 * @brief ble_start_scan
 * @param[in]  uint32_t sn,uint16_t len
 * @retval status
 */
HAL_StatusTypeDef ble_data_sync_len_ack(uint32_t sn,uint16_t len)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_DATA_SYNC_ACK_LEN;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);

    data[i++] = (uint8_t)(len >> 8);
    data[i++] = (uint8_t)(len);

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}

/**
 * @brief send ble information to 51822
 * @param[in]  uint32_t sn,uint8_t *p_mac,uint8_t len
 * @retval status
 */
HAL_StatusTypeDef ble_data_ble_info_send(uint32_t sn,uint8_t *p_mac,uint8_t len)
{
    uint8_t                     data[20] = {0};
    uint8_t                     i = 0;
    uint8_t                     j = 0;
    HAL_StatusTypeDef           status = HAL_ERROR;

    data[i++] = 0xAA;
    data[i++] = 0x55;
    data[i++] = CMU_FRAME_ID_BLE_INFO_GET;
    data[i++] = 0;
    data[i++] = 0;  //数据包长度，后面添加
    data[i++] = (uint8_t)(sn>>24);
    data[i++] = (uint8_t)(sn>>16);
    data[i++] = (uint8_t)(sn>>8);
    data[i++] = (uint8_t)(sn);

    if(len == BLE_ADDR_LEN)
    {
        for(j=0;j<len;j++)
        {
            data[i++] = p_mac[j];
        }
    }

    data[i++] = 0;
    data[i++] = 0;  //CRC,暂时未用

    data[3] = (uint8_t)((i-5)>>8);   //数据长度减去帧头
    data[4] = (uint8_t)(i-5);

    status = ble_send_data(data,i);

    return status;
}

/**
 * @brief ble_connect_process
 * @note 采用状态机控制蓝牙连接进程
 * @param[in]  None
 * @retval None
 */
static void ble_connect_process(BLE_PROCESS_STATUS_T status)
{
    uint32_t                    tim = 0;


    if((gBLEDevInfo.isBleConnect == true) &&
        (gBLEDevInfo.sn != 0)
    ) //蓝牙连接情况下才处理以下操作
    {
        switch(status)
        {
            case BLE_PROCESS_SET_TIME:
            {
                #ifdef BLE_CONNECT_DEBUG
                    printf("[BLE]:set time\r\n");
                #endif
                /* get the current time */
                get_time_inUnix(&tim);
                ble_set_time(gBLEDevInfo.sn,tim);
            }
            break;
            case BLE_PROCESS_SET_MONITOR_TEMPLATE:
            {
                #ifdef BLE_CONNECT_DEBUG
                    printf("[BLE]:set monitor template\r\n");
                #endif
                if(gBLEDevInfo.monitorTemplate.len != 0)
                {
                    ble_set_monitor_template(gBLEDevInfo.sn,&gBLEDevInfo.monitorTemplate);
                }
            }
            break;
            case BLE_PROCESS_START_DATA_SYNC:
            {
                #ifdef BLE_CONNECT_DEBUG
                    printf("[BLE]:start data sync\r\n");
                #endif
                ble_start_data_sync(gBLEDevInfo.sn);
            }
            break;
            default:break;
        }
    }
    else  //蓝牙已断开
    {
        //@todo
    }
}
/**
 * @brief ble_receive_data_handle
 * @param[in]  CMU_DATA_T *data
 * @retval None
 */
static void ble_receive_data_handle(CMU_DATA_T *data)
{
    #ifdef BLE_CONNECT_DEBUG
        uint8_t             i = 0;
    #endif
    BLE_MSG_T           bleTxedMessage;

//    printf("Type:0x%x\r\n",data->frameType);
//    printf("devSN:%d\r\n",data->devSN);
//    printf("len:%d\r\n",data->dataLen);
//    for(i=0;i<data->dataLen;i++)
//    {
//        printf("0x%x,",data->dataBuf[i]);
//    }
//    printf("\r\n");

    bleTxedMessage.event = BLE_DEFAULT_EVENT;

    switch(data->frameType)
    {
        case CMU_FRAME_ID_BLE_CONNECT_STATUS:  //蓝牙连接状态
        {
            gBLEDevInfo.sn = data->devSN;

            if(data->dataBuf[0] == 0x00) //蓝牙断开
            {
                #ifdef BLE_CONNECT_DEBUG
                    printf("ble disconnect,SN:%d\r\n",data->devSN);
                #endif
                gBLEDevInfo.isBleConnect = false;
            }
            else if(data->dataBuf[0] == 0x01) //蓝牙连接成功
            {
                #ifdef BLE_CONNECT_DEBUG
                    printf("ble connect,SN:%d\r\n",data->devSN);
                #endif
                gBLEDevInfo.isBleConnect = true;
                if(0 != memcmp(gBLEDevInfo.macAddr, data->dataBuf + 1,BLE_ADDR_LEN))
                {
                    printf("memcpy,SERVER_ADDBAND_EVENT");
                    gBLEDevInfo.macAddr[0] = data->dataBuf[1];
                    gBLEDevInfo.macAddr[1] = data->dataBuf[2];
                    gBLEDevInfo.macAddr[2] = data->dataBuf[3];
                    gBLEDevInfo.macAddr[3] = data->dataBuf[4];
                    gBLEDevInfo.macAddr[4] = data->dataBuf[5];
                    gBLEDevInfo.macAddr[5] = data->dataBuf[6];
                    
                    apl_service_event_t apl_service_event_tx;
                    apl_service_event_tx.apl_event_id = SERVER_ADDBAND_EVENT;
                    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                }

                /* 发送设置时间事件 */
                bleTxedMessage.event = BLE_SET_TIME_EVENT;
                xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
            }

            #ifdef BLE_CONNECT_DEBUG
                printf("Mac Addr:");
                for(i=0;i<6;i++)
                {
                    printf("0x%x,",gBLEDevInfo.macAddr[i]);
                }
                printf("\r\n");
            #endif
        }
        break;
        case CMU_FRAME_ID_SET_TIME_RSP:
        {
            if(gBLEDevInfo.sn == data->devSN) //当前连接设备
            {
                if(data->dataBuf[0] == 0x01) //时间设置成功
                {
                    /* 发送设置方案事件 */
                    bleTxedMessage.event = BLE_SET_MONITOR_TEMPLATE;
                    xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
                }
                else
                {
                    //@todo:时间设置错误时
                }
            }
        }
        break;
        case CMU_FRAME_ID_SET_MONITOR_TEMPLATE_RSP:
        {
            if(gBLEDevInfo.sn == data->devSN)  //当前连接设备
            {
                if(data->dataBuf[0] == 0x01) //方案设置成功
                {
                    /* 发送开始数据同步事件 */
                    bleTxedMessage.event = BLE_START_DATA_SYNC;
                    xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
                }
                else
                {
                    //@todo:方案设置错误时
                }
            }
        }
        break;
        case CMU_FRAME_ID_DATA_SYNC:
        {
            if(gBLEDevInfo.sn == data->devSN)  //当前连接设备
            {
                bleTxedMessage.event = BLE_DATA_SYNC;  //接收数据同步数据
                bleTxedMessage.len = (data->dataLen - 2); //减掉2字节CRC
                bleTxedMessage.sn = data->devSN;
                bleTxedMessage.buf = syncDataBuf;
                memcpy(bleTxedMessage.buf,data->dataBuf,bleTxedMessage.len);
                xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
            }
        }
        break;
        default:break;
    }
}
/**
 * @brief ble_data_sync_data_handle
 * @param[in]  DEV_BLE_INFO_T *bleDevInfo,uint8_t *data,uint8_t len
 * @retval None
 */
static HAL_StatusTypeDef ble_data_sync_data_handle(DEV_BLE_INFO_T *bleDevInfo,uint8_t *data,uint8_t len)
{
    HAL_StatusTypeDef               status = HAL_ERROR;
    char                            fileName[30] = {'\0'};
    FRESULT                         res;
    bool                            isStoreFileCreat = false;
    uint32_t                        byteswritten = 0;

    /*uint8_t i=0;
    for(i=0;i<len;i++)
    {
        printf("0x%x,",data[i]);
    }
    printf("\r\n");*/

    isStoreFileCreat = bleDevInfo->isStoreFileCreat;
    if(isStoreFileCreat == false)   //没有创建对应的文件
    {
        /* 创建文件 */
        sprintf (fileName, "DAT%d.txt", bleDevInfo->sn);
        res = f_open(bleDevInfo->pFile,fileName,FA_OPEN_ALWAYS | FA_WRITE);
        if(res == FR_OK)
        {
            bleDevInfo->isStoreFileCreat = true;
            isStoreFileCreat = true;
        }
        #ifdef BLE_DATA_SYNC_DEBUG
        else
        {
            printf("creat file err:%d\r\n",res);
        }
        #endif
    }

    if(isStoreFileCreat == true)  //已经创建了对应的文件
    {
        /* 写入数据至文件中 */
        res = f_write(bleDevInfo->pFile,data,len,(void *)&byteswritten);
        #ifdef BLE_DATA_SYNC_DEBUG
        if((res != FR_OK) || (byteswritten != len))
        {
            printf("write file err:%d\r\n",res);
        }
        #endif
    }

    if(res == FR_OK)
    {
        status = HAL_OK;
    }

    return status;
}








