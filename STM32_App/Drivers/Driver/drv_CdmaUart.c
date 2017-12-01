/*
 * drv_Uart.c
 *
 *  Created on: 2015年5月4日
 *      Author: yyy
 */
#include "drv_CdmaUart.h"
#include "includes.h"

QueueHandle_t xQueue_SIM_UART_RX = NULL;
QueueHandle_t xQueue_SIM_UART_RX_Data = NULL;
SemaphoreHandle_t xMutex_InterfaceTx = NULL;
SemaphoreHandle_t xMutex_InterfaceRx = NULL;
SemaphoreHandle_t xSem_InterfaceAtRx = NULL;
EventGroupHandle_t xEventGroup_sim = NULL;
TaskHandle_t xUart2GRxTaskHandle = NULL;

uint8_t u8SimUartRxByte = 0;
uint8_t u8PreSimRxArr[MAX_LEN_SIM_UART] = {0};
uint8_t u8SimRxArr[MAX_LEN_SIM_UART] = {0};
uint8_t u8SimRespArr[MAX_LEN_SIM_UART] = {0};
uint8_t u8SimAtRespArr[50] = {'\0'};
uint8_t u8SimRxedDataArr[800] = {0};

uint8_t u8OKArr[] = {0x0D, 0x0A, 0x4F, 0x4B, 0x0D, 0x0A};
uint8_t u8ERRORArr[] = {0x0D, 0x0A, 0x45, 0x52, 0x52, 0x4F, 0x52, 0x0D, 0x0A};
char *pucIpReceiveArr = "^IPRECV";
char *pucIpStateArr = "^IPSTAT";
char *pucDialedStateArr = "^IPCALL";
char *pucIpOpenArr = "^IPOPEN";
char pucIpRxedDataArr[] = "^IPRECV:1,182.139.182.209,30019,0,";


HAL_StatusTypeDef drv_SimUART_Init(void);
HAL_StatusTypeDef drv_SimUartSendData(char *pucTxBuf, uint16_t length);
HAL_StatusTypeDef drv_SimUartSendData_Raw(char *pucTxBuf, uint16_t length);
HAL_StatusTypeDef drv_SimUartReceiveData(char *pucRxBuf, uint16_t *pu16Len,uint32_t u32DelayTimes);

net_init_t sim800c =
{
    drv_SimUART_Init,
    drv_SimHwInit,
    drv_SimSWInit,
    drv_SimHwON_OFF,
    drv_SimHwON_OFF,
    drv_SimHwReset,
    drv_SimUartSendData,
    drv_SimUartSendData_Raw,
    drv_SimUartReceiveData
};

sim800c_stat_t sim800c_stat;

uint16_t u16SimUartRxCnt = 0;

void vAppSIM900RxTaskCode( void * pvParameters )
{
    uint32_t ulRXdata = 0;
    sim_rx_data_t sim_tx_data;

    while(1)
    {
        if(pdPASS == xTaskNotifyWait(0,0xFFFFFFFF,&ulRXdata,portMAX_DELAY))
        {
            /*printf("Rxed data: %d\r\n",ulRXdata);
            for(uint8_t i = 0;i < ulRXdata;i++)
            {
                printf("%02x, ",u8SimRxArr[i]);
            }
            printf("\r\n");*/
            // printf("Uart Rxed String: %s",u8SimRxArr);

            if(0 == memcmp(u8SimRxArr + 2,pucIpReceiveArr,6))
            {
                if(pdTRUE == xSemaphoreTake(xMutex_InterfaceRx, (TickType_t)10))
                {
                    memset(u8SimRespArr,'\0',sizeof(u8SimRespArr));
                    memcpy(u8SimRespArr,u8SimRxArr,ulRXdata);
                    xSemaphoreGive(xMutex_InterfaceRx);
                    sim_tx_data.pu8Data = u8SimRespArr;
                    sim_tx_data.u16Len = ulRXdata;
                    sim_tx_data.u8Cmd = 0;
                    xQueueSend(xQueue_SIM_UART_RX_Data, &sim_tx_data, 10);
                }
            }
            else if((0 == memcmp(u8SimRxArr + 2,pucIpStateArr,6))
                ||(0 == memcmp(u8SimRxArr + 2,pucIpOpenArr,6)))
            {
                if(pdTRUE == xSemaphoreTake(xMutex_InterfaceRx, (TickType_t)10))
                {
                    memset(u8SimRespArr,'\0',sizeof(u8SimRespArr));
                    memcpy(u8SimRespArr,u8SimRxArr,ulRXdata);
                    xSemaphoreGive(xMutex_InterfaceRx);
                    sim_tx_data.pu8Data = u8SimRespArr;
                    sim_tx_data.u16Len = ulRXdata;
                    sim_tx_data.u8Cmd = 0;
                    xQueueSend(xQueue_SIM_UART_RX_Data, &sim_tx_data, 10);
                }
            }
            else if(0 == memcmp(u8SimRxArr + 2,pucDialedStateArr,7))
            {
                if(cdma_stat == CDMA_CONNECTED)
                {
                    if(pdTRUE == xSemaphoreTake(xMutex_InterfaceRx, (TickType_t)10))
                    {
                        memset(u8SimRespArr,'\0',sizeof(u8SimRespArr));
                        memcpy(u8SimRespArr,u8SimRxArr,ulRXdata);
                        xSemaphoreGive(xMutex_InterfaceRx);
                        sim_tx_data.pu8Data = u8SimRespArr;
                        sim_tx_data.u16Len = ulRXdata;
                        sim_tx_data.u8Cmd = 0;
                        xQueueSend(xQueue_SIM_UART_RX_Data, &sim_tx_data, 10);
                    }
                }
                else
                {
                    memcpy(u8SimAtRespArr,u8SimRxArr,ulRXdata);
                    sim_tx_data.pu8Data = u8SimAtRespArr;
                    sim_tx_data.u16Len = ulRXdata;
                    sim_tx_data.u8Cmd = 0;
                    xQueueSend(xQueue_SIM_UART_RX, &sim_tx_data, 10);
                }
            }
            else
            {
                memcpy(u8SimAtRespArr,u8SimRxArr,ulRXdata);
                sim_tx_data.pu8Data = u8SimAtRespArr;
                sim_tx_data.u16Len = ulRXdata;
                sim_tx_data.u8Cmd = 0;
                xQueueSend(xQueue_SIM_UART_RX, &sim_tx_data, 10);
            }
        }
    }
}

void vAppSIM900RxDataTaskCode ( void * pvParameters )
{
    sim_rx_data_t sim_rx_data;
	apl_service_event_t apl_service_event_tx;
    char ucRespAddr[20] = {"\0"};
    char * pucSimRx = NULL;
    uint8_t u8Tmp = 0;
    uint16_t u16RxedDataLen = 0;

    while(1)
    {
        if(pdTRUE == xQueueReceive(xQueue_SIM_UART_RX_Data, &sim_rx_data, portMAX_DELAY))
        {
            // printf("RxDataTask,Rxed string: %s\r\n",sim_rx_data.pu8Data);

            pucSimRx = strstr((char *)(u8SimRxArr + 2),"^IPSTAT:1,");
            if(NULL != pucSimRx)
            {
                sscanf(pucSimRx,"%*[^,],%s",ucRespAddr);
                u8Tmp = atoi(ucRespAddr);

                // Disconnectted
                if(u8Tmp)
                {
                    apl_service_event_tx.apl_event_id = SERVER_DISCONNECT_EVENT;
                    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 10);
                }
            }

            pucSimRx = strstr((char *)(u8SimRxArr + 2),"^IPCALL:");
            if(NULL != pucSimRx)
            {
                sscanf(pucSimRx,"%*[^:]:%s",ucRespAddr);
                u8Tmp = atoi(ucRespAddr);

                // Disconnectted
                if(!u8Tmp)
                {
                    apl_service_event_tx.apl_event_id = SERVER_DISCONNECT_EVENT;
                    xQueueSend(xQueue_AppServer, &apl_service_event_tx, 10);
                }
            }

            pucSimRx = strstr((char *)(u8SimRxArr + 2),pucIpReceiveArr);
            if(NULL != pucSimRx)
            {
                u16RxedDataLen = strlen(pucSimRx) - strlen(pucIpRxedDataArr);
                memset(u8SimRxedDataArr,'\0',sizeof(u8SimRxedDataArr));
                M2Mstoh((char *)u8SimRxedDataArr, (pucSimRx + sizeof(pucIpRxedDataArr) - 2), u16RxedDataLen);

                udp_recv_cb(NULL, (char *)u8SimRxedDataArr, (strlen(pucSimRx) - strlen(pucIpRxedDataArr))/2);
            }
        }
    }
}

HAL_StatusTypeDef drv_SimUART_StartRx (void)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    status = HAL_UART_Receive_IT(&huart6, &u8SimUartRxByte, 1);

    return (status);
}

/* USART2 init function */
HAL_StatusTypeDef drv_SimUART_TaskCreate (void)
{
    HAL_StatusTypeDef status = HAL_OK;
    TaskHandle_t xUart2GRxDataTaskHandle = NULL;

    xMutex_InterfaceTx = xSemaphoreCreateMutex();
    usr_res_assert(xMutex_InterfaceTx != NULL);

    xMutex_InterfaceRx = xSemaphoreCreateMutex();
    usr_res_assert(xMutex_InterfaceRx != NULL);

    xSem_InterfaceAtRx = xSemaphoreCreateBinary();
    usr_res_assert(xSem_InterfaceAtRx != NULL);

    xQueue_SIM_UART_RX = xQueueCreate( 1, sizeof(sim_rx_data_t) );
    configASSERT( xQueue_SIM_UART_RX );

    xQueue_SIM_UART_RX_Data = xQueueCreate( 1, sizeof(sim_rx_data_t) );
    configASSERT( xQueue_SIM_UART_RX_Data );

    xTaskCreate( vAppSIM900RxTaskCode,"SIM900RxTask",configMINIMAL_STACK_SIZE,NULL,SIM900_RX_THREAD_PRIO,&xUart2GRxTaskHandle);
    configASSERT( xUart2GRxTaskHandle );

    xTaskCreate( vAppSIM900RxDataTaskCode,"SIM900RxDataTask",configMINIMAL_STACK_SIZE * 5,NULL,
                SIM900_RX_THREAD_PRIO-1,&xUart2GRxDataTaskHandle);
    configASSERT( xUart2GRxDataTaskHandle );

    return (status);
}

HAL_StatusTypeDef drv_SimUART_Init (void)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    MX_USART6_UART_Init();

    status = drv_SimUART_StartRx();

    return (status);
}

HAL_StatusTypeDef drv_SimUartSendData(char *pucTxBuf, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    // Check parameters
    usr_para_assert(pucTxBuf != NULL);

    if(pdTRUE == xSemaphoreTake(xMutex_InterfaceTx, 100))
    {
		HAL_GPIO_WritePin(CDMA_UART_RTS_GPIO_Port, CDMA_UART_RTS_Pin, GPIO_PIN_RESET);
        status = HAL_UART_Transmit(&huart6, (uint8_t *)pucTxBuf, length, 100);
//		HAL_GPIO_WritePin(CDMA_UART_RTS_GPIO_Port, CDMA_UART_RTS_Pin, GPIO_PIN_SET);
        xSemaphoreGive(xMutex_InterfaceTx);
    }
    else
    {
        printf("No Semaphore\r\n");
    }

    return (status);
}

HAL_StatusTypeDef drv_SimUartSendData_Raw(char *pucTxBuf, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    return (status);
}

HAL_StatusTypeDef drv_SimUartReceiveData(char *pucRxBuf, uint16_t *pu16Len,uint32_t u32DelayTimes)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    sim_rx_data_t sim_rx_data;

    // Check parameters
    usr_para_assert(pucRxBuf != NULL);
    usr_para_assert(pu16Len != NULL);

    if(pdTRUE == xQueueReceive(xQueue_SIM_UART_RX, &sim_rx_data, u32DelayTimes))
    {
        memcpy(pucRxBuf, sim_rx_data.pu8Data, sim_rx_data.u16Len);
        *pu16Len = sim_rx_data.u16Len;

        status = HAL_OK;
    }

    return (status);
}


