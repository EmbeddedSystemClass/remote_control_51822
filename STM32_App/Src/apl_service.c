#include "apl_service.h"
#include "includes.h"

extern char ucBandMonitorTemplateArr[BOX_CONNECT_MAX_NUM][MAX_MONITOR_TEMPLATE_LEN];

QueueHandle_t xQueue_AppServer = NULL;
TimerHandle_t xTimers_UploadFile = NULL;
TimerHandle_t xTimers_CheckServerState = NULL;
TimerHandle_t xTimersCoAP = NULL;
SemaphoreHandle_t xSemaphore_Server = NULL;
QueueHandle_t xQueue_CoAPRxed = NULL;
char u8ServerReturnData[1024] = {'\0'};
uint32_t u32CoAPCnt = 0;


void vTimerCallback_UploadFile( TimerHandle_t pxTimer )
{
	apl_service_event_t apl_service_event_tx;

    xTimerStop(xTimers_UploadFile, 0);
	apl_service_event_tx.apl_event_id = SERVER_UPLOAD_FILE_EVENT;
    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
}

void vTimerCallback_GetServerState( TimerHandle_t pxTimer )
{
	apl_service_event_t apl_service_event_tx;
	apl_service_event_tx.apl_event_id = SERVER_CHECK_SIGNAL_EVENT;
    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
}

void vTimerCallback_CoAP( TimerHandle_t pxTimer )
{
    uint32_t u32TxedMessage = COAP_DOWORK_EVENT;

	u32CoAPCnt++;
    xQueueSend(xQueue_CoAP, &u32TxedMessage, 5/portTICK_PERIOD_MS);
}


void StartCheckServerStateTimer(void)
{
    xTimerStart(xTimers_CheckServerState, 0);
}

void StopCheckServerStateTimer(void)
{
    xTimerStop(xTimers_CheckServerState, 0);
}

void StartCoAPTimer(void)
{
    xTimerStart(xTimersCoAP, 0);
}

void StopCoAPTimer(void)
{
    xTimerStop(xTimersCoAP, 0);
}

uint8_t CoAP_WorkMemory[4096]; //Working memory of CoAPs internal memory allocator

void init_done(void)
{
	CoAP_Init(CoAP_WorkMemory, sizeof(CoAP_WorkMemory));
	CoAP_ESP8266_CreateInterfaceSocket(0, &Esp8266_conn, 5683, CoAP_onNewPacketHandler, CoAP_ESP8266_SendDatagram);
	coap_mem_determinateStaticMem();
	coap_mem_stats();

    xQueue_CoAPRxed = xQueueCreate( 3, sizeof( uint32_t ) );
    configASSERT( xQueue_CoAPRxed );
}

int8_t Apl_TransmitBandSycDataToServer(uint32_t u32BandSn)
{
	int8_t ret = -1;
	char cFileName[50];
	
	memset(cFileName,'\0',sizeof(cFileName));
    sprintf(cFileName,"DAT%d.txt",u32BandSn);
	
	if(MID_OK == mid_TransmitBandSycDataToServer(cFileName, u32BandSn))
	{
		ret = 0;
	}

    return (ret);
}

int8_t Apl_DelBandSycDataFile(uint32_t u32BandSn)
{
	int8_t ret = -1;
	char cFileName[50];
	
	memset(cFileName,'\0',sizeof(cFileName));
    sprintf(cFileName,"DAT%d.txt",u32BandSn);

	mid_DeleteFile(cFileName);

    return (ret);
}



void vAppServerTaskCode( void * pvParameters )
{
	apl_service_event_t             apl_service_event_rx;
	apl_service_event_t             apl_service_event_tx;
//    static uint8_t                  u8UploadRetryCnt = 0;
    BLE_MSG_T                       bleTxedMessage;

    MX_FATFS_Init();
    drv_SimUART_TaskCreate();
    if(-1 == WDT_Register(WDT_UPLOAD_TASK_ID))
    {
        printf("WDT_Register,Error\r\n");
    }

    sim800c.pfHwInit();
    init_done();
    printf("init_done,End\r\n");
    apl_service_event_tx.apl_event_id = SERVER_INIT_EVENT;
    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);

    // Infinite Loop
    while(1)
    {
        if(pdTRUE == xQueueReceive( xQueue_AppServer, &apl_service_event_rx, ( TickType_t ) 100/portTICK_PERIOD_MS) )
		{
			switch(apl_service_event_rx.apl_event_id)
            {
                case SERVER_INIT_EVENT:
//                    printf("\r\n SERVER_INIT_EVENT\r\n");
                    if(pdTRUE == xSemaphoreTake(xSemaphore_Server, portMAX_DELAY))
                    {
                        if(HAL_OK != drvSim_Init())
                        {
                            vTaskDelay(10/portTICK_PERIOD_MS);
                            apl_service_event_tx.apl_event_id = SERVER_INIT_EVENT;
                            xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                        }
                        else
                        {
                            printf("\r\nThe server is on-line\r\n");
                            StartCoAPTimer();
                            mid_Coap_Init();

                            //midCoAP_GetServerAvailable(&box_info.state);
							box_info.server_setup = SERVER_SETUP_INIT;
							apl_service_event_tx.apl_event_id = SERVER_GETCONFIG_EVENT;
                    		xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                        }

                        xSemaphoreGive(xSemaphore_Server);
                    }
                    break;

                case SERVER_GETCONFIG_EVENT:
					if(MID_OK == mid_ServerInit())
					{
                        printf("\r\nmid_ServerInit,OK\r\n");
                        printf("BoxInfo u8SupportBandMaxNum : %d, %d\r\n", box_info.u8SupportBandMaxNum,box_info.u8BandNum);
                        printf("BoxInfo IPAddr : %s\r\n", box_info.ucIpAddr);
                        printf("BoxInfo IPPort : %s\r\n", box_info.ucPort);
                        for(uint8_t i = 0;i < box_info.u8BandNum;i ++)
                        {
                            printf("i = %d:\r\n", i);
                           	printf("SN: %d\r\n", (*(box_info.ptdev_band_info + i))->sn);
                            printf("monitorTemplate: %s\r\n", (*(box_info.ptdev_band_info + i))->monitorTemplate.pbuf);
							printf("monitorTemplateLen: %d\r\n", (*(box_info.ptdev_band_info + i))->monitorTemplate.len);
                        }

                        xSemaphoreGive(xSemaphore_Key);
						
 						/* send ble information to 51822 */
                        bleTxedMessage.event = BLE_DEV_INFO_SEND_EVENT;
                        xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS); 					
					}
                    else
                    {
                        apl_service_event_tx.apl_event_id = SERVER_GETCONFIG_EVENT;
                    	if(pdTRUE != xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS))
                    	{
                            printf("xQueueSend,Failed\r\n");
                        }
                    }
                    break;

                case SERVER_CHECK_SIGNAL_EVENT:
                    printf("\r\n SERVER_CHECK_SIGNAL_EVENT\r\n");
                    if(pdTRUE == xSemaphoreTake(xSemaphore_Server, 100))
                    {
                        drvSim_GetTcpStatus();
                        if(sim800c_stat.bOnLine)
                        {
                            drvSim_GetSigStrength(&sim800c_stat.u8SQ, &sim800c_stat.i8SQdBm);
                            printf("sim800c.u8SQ = %d\r\n", sim800c_stat.u8SQ);
                            if(sim800c_stat.u8SQ > 10)
                            {
                                //

                            }
                        }
                        else
                        {
                            printf("The server is off-line\r\n");

                            apl_service_event_tx.apl_event_id = SERVER_INIT_EVENT;
                            xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                        }

                        xSemaphoreGive(xSemaphore_Server);
                    }
                    break;

                case SERVER_DISCONNECT_EVENT:
                    printf("\r\n SERVER_DISCONNECT_EVENT\r\n");
                    if(pdTRUE == xSemaphoreTake(xSemaphore_Server, 100))
                    {
                        printf("The server is off-line\r\n");
                        sim800c_stat.bOnLine = FALSE;
                        cdma_stat = CDMA_OFF;
						box_info.server_setup = SERVER_SETUP_INIT;

                        apl_service_event_tx.apl_event_id = SERVER_INIT_EVENT;
                        xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS);
                        xSemaphoreGive(xSemaphore_Server);
                    }
                    break;

                case SERVER_UPLOAD_FILE_EVENT:
                    printf("\r\n[SYN] SERVER_UPLOAD_FILE_EVENT\r\n");
                    WDT_Feed(WDT_UPLOAD_TASK_ID);
                    if(pdTRUE == xSemaphoreTake(xSemaphore_Server, portMAX_DELAY))
                    {
                      	if(0 ==  Apl_TransmitBandSycDataToServer(apl_service_event_rx.u32Value))
                      	{
							printf("Apl_TransmitBandSycDataToServer,OK\r\n");
							// Apl_DelBandSycDataFile(apl_service_event_rx.u32Value);
						}
						else
						{
							printf("Apl_TransmitBandSycDataToServer,Failed\r\n");
						}

                        xSemaphoreGive(xSemaphore_Server);
                    }
                    break;

               case SERVER_ADDBAND_EVENT:
                    if(MID_OK == midCoAP_PutNewBandInfo(0))
                    {
                        printf("Put new band info,OK\r\n");
						uint8_t i = 0;
						if(MID_OK == midCoAP_GetMonitorTemplate((char *)(ucBandMonitorTemplateArr + i), (*(box_info.ptdev_band_info + i)) ->sn))
						{
							(*(box_info.ptdev_band_info + i))->monitorTemplate.pbuf = (uint8_t *)ucBandMonitorTemplateArr + i;
							(*(box_info.ptdev_band_info + i))->monitorTemplate.len = strlen((char *)(ucBandMonitorTemplateArr + i));

							BLE_MSG_T  bleTxedMessage;
							bleTxedMessage.event = BLE_SET_MONITOR_TEMPLATE;
							xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
						}
                    }
                    else
                    {
                        printf("Put new band info,ERROR\r\n");
                    }
                    break;

                case APP_2GBOX_MCU_DOWNLOAD_FILE_EVENT:
                    break;

                case APP_2GBOX_MCU_FW_UPDATE_EVENT:
                    break;

                default:
                    break;
            }
		}
        else
        {
            WDT_Feed(WDT_UPLOAD_TASK_ID);
        }
    }
}


void CreateServerTask(void)
{
    TaskHandle_t xHandle = NULL;

    // Create a queue capable of containing 10 uint32_t values.
    xQueue_AppServer = xQueueCreate( 30, sizeof( apl_service_event_t ) );
    configASSERT( xQueue_AppServer );

    if(pdPASS == xTaskCreate( vAppServerTaskCode,"ServerTaskCode",configMINIMAL_STACK_SIZE * 40,NULL,tskSERVER_PRIORITY,&xHandle))
    {
        configASSERT( xHandle );
    }

    xTimers_UploadFile = xTimerCreate("Timers_UploadFile",
                                    10000/portTICK_PERIOD_MS,
                                    pdFALSE,
                                    ( void * ) 0,
                                    vTimerCallback_UploadFile);
    configASSERT( xTimers_UploadFile );

    xTimers_CheckServerState = xTimerCreate("Timers_CheckServerState",
                                    3000/portTICK_PERIOD_MS,
                                    pdTRUE,
                                    ( void * ) 0,
                                    vTimerCallback_GetServerState);
    configASSERT( xTimers_CheckServerState );

    xSemaphore_Server = xSemaphoreCreateMutex();
    configASSERT( xSemaphore_Server );

    xTimersCoAP = xTimerCreate("xTimersCoAP",
                            500/portTICK_PERIOD_MS,
                            pdTRUE,
                            ( void * ) 0,
                            vTimerCallback_CoAP);
    configASSERT( xTimersCoAP );

}

