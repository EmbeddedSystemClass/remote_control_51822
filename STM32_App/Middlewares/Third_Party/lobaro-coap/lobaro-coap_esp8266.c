/*******************************************************************************
 * Copyright (c)  2015  Dipl.-Ing. Tobias Rohde, http://www.lobaro.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/
/*
 * lobaro-coap-interface.c
 *
 *  Created on: 13.11.2015
 *      Author: Tobias
 */

#include "coap.h"
#include "ip_addr.h"
#include "lobaro-coap_esp8266.h"
#include "net_Endpoint.h"
#include "includes.h"

remot_info remote_info;

CoAP_ESP8266_States_t CoAP_ESP8266_States = {.TxSocketIdle=true, .StationConStatus=0xff};
NetEp_t ServerEp;
struct espconn Esp8266_conn;
CoAP_Res_t CoAP_Res;
TaskHandle_t xCoAPTaskHandle = NULL;
QueueHandle_t xQueue_CoAP = NULL;


//implement internaly used functions (see also lobaro-coap/interface/coap_interface.h)
void hal_uart_puts(char *s) {
	printf("%s",s);
}
void hal_uart_putc(char c){
	printf("%c",c);
}

//1Hz Clock used by timeout logic
uint32_t hal_rtc_1Hz_Cnt(void){
    return (u32CoAPCnt);
}

//Non volatile memory e.g. flash/sd-card/eeprom
//used to store observers during deepsleep of server
uint8_t* hal_nonVolatile_GetBufPtr(){
	return NULL; //not implemented yet on esp8266
}

bool hal_nonVolatile_WriteBuf(uint8_t* data, uint32_t len){
	return false; //not implemented yet on esp8266
}

//---------------------------------
void udp_sent_cb(void *arg) {
	// struct espconn *pesp_conn = arg;
    CoAP_ESP8266_States.TxSocketIdle = true;
    printf("send OK!\r\n");
}

//implement network functions around coap if socket
void udp_recv_cb(void *arg, char *pdata, unsigned short len) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    NetPacket_t     Packet;
    NetSocket_t*    pSocket=NULL;
    SocketHandle_t handle = (SocketHandle_t) pesp_conn;
    int32_t i = 0;

    static uint32_t u32CntRxedUart = 0;

    printf("udp_recv_cb: %d\r\n",u32CntRxedUart++);
//get socket by handle
    pSocket = RetrieveSocket(handle);
    if(pSocket == NULL){
        printf("Corresponding Socket not found!\r\n");
        return;
    }

//packet data
    Packet.pData = (uint8_t *)pdata;
    Packet.size = len;

//Sender
    Packet.Sender.NetPort = ServerEp.NetPort;
    Packet.Sender.NetType = IPV4;
    for(i=0; i<4; i++)
    {
        Packet.Sender.NetAddr.IPv4.u8[i] = ServerEp.NetAddr.IPv4.u8[i];
    }

//Receiver
    Packet.Receiver.NetPort = pSocket->EpLocal.NetPort;
    Packet.Receiver.NetType = IPV4;
    for(i=0; i<4; i++)
    {
        Packet.Receiver.NetAddr.IPv4.u8[i] = pesp_conn->proto.udp->local_ip[i];
    }

//meta info
    Packet.MetaInfo.Type = META_INFO_NONE;

    //call the consumer of this socket
    //the packet is only valid during runtime of consuming function!
    //-> so it has to copy relevant data if needed
    // or parse it to a higher level and store this result!
    pSocket->RxCB(pSocket->ifID, &Packet);

	return;
}

NetSocket_t* CoAP_ESP8266_CreateInterfaceSocket(uint8_t ifID, struct espconn* pEsp8266_conn, uint16_t LocalPort, NetReceiveCallback_fn Callback, NetTransmit_fn SendPacket)
{
	NetSocket_t* pSocket;

	pSocket=RetrieveSocket2(ifID);
	if(pSocket != NULL) {
		printf("CoAP_ESP8266_CreateInterfaceSocket(): interface ID already in use!\r\n");
		return NULL;
	}

	if(Callback == NULL || SendPacket == NULL) {
		printf("CoAP_ESP8266_CreateInterfaceSocket(): packet rx & tx functions must be provided!\r\n");
		return NULL;
	}

	pSocket = AllocSocket();
	if(pSocket == NULL){
		printf("CoAP_ESP8266_CreateInterfaceSocket(): failed socket allocation\r\n");
		return NULL;
	}

    ServerEp.NetType = IPV4;
    ServerEp.NetAddr.IPv4.u8[0] = 182;
    ServerEp.NetAddr.IPv4.u8[1] = 139;
    ServerEp.NetAddr.IPv4.u8[2] = 182;
    ServerEp.NetAddr.IPv4.u8[3] = 209;
    ServerEp.NetPort = 5683;

//	pSocket->Handle = (void*) (pEsp8266_conn); //external  to CoAP Stack
	pSocket->ifID = ifID; //internal  to CoAP Stack

//user callback registration
	pSocket->RxCB = Callback;
	pSocket->Tx = SendPacket;
	pSocket->Alive = true;

	printf("- CoAP_ESP8266_CreateInterfaceSocket(): listening... IfID: %d  Port: %d\r\n",ifID, LocalPort);

	return pSocket;
}


bool CoAP_ESP8266_DeleteInterfaceSocket(uint8_t ifID)
{
	NetSocket_t* pSocket = RetrieveSocket2(ifID);

//	if(pSocket)
//	{
//		struct espconn *conn= (struct espconn *)pSocket->Handle;
//		espconn_delete(conn);
//		printf("CoAP_ESP8266_DeleteInterfaceSocket(): unlisten OK! IfID: %d\r\n", ifID);
//		return true;
//	}

	printf("CoAP_ESP8266_DeleteInterfaceSocket(): Socket not found! IfID: %d\r\n", ifID);
	return false;
}

bool CoAP_ESP8266_SendDatagram(uint8_t ifID, NetPacket_t* pckt)
{
    bool bRet = TRUE;
    char remote_port[10] = {'\0'};
    char remote_ipaddr[16] = {'\0'};
    //bool bTxedCompleted = FALSE;
    //uint16_t u16ToTxedLen = 0;
    //uint16_t u16TxedLen = 0;

    printf("CoAP_ESP8266_SendDatagram,%d\r\n",pckt->size);

    sprintf(remote_ipaddr,"%d.%d.%d.%d",pckt->Receiver.NetAddr.IPv4.u8[0],pckt->Receiver.NetAddr.IPv4.u8[1],
            pckt->Receiver.NetAddr.IPv4.u8[2],pckt->Receiver.NetAddr.IPv4.u8[3]);
    sprintf(remote_port,"%d",pckt->Receiver.NetPort);
    if(HAL_OK != drvSim_SendUdpDataToServer(pckt->pData, pckt->size, remote_ipaddr, remote_port))
    {
        bRet = FALSE;
    }

    /*u16ToTxedLen = pckt->size;
    u16TxedLen = 0;
    do
    {
        if(u16ToTxedLen > 256 )
        {
            drvSim_SendUdpDataToServer((pckt->pData + u16TxedLen), 256, remote_ipaddr, remote_port);
            u16ToTxedLen = (u16ToTxedLen > 256) ? (u16ToTxedLen - 256) : 0;
            u16TxedLen += 256;
        }
        else
        {
            drvSim_SendUdpDataToServer((pckt->pData + u16TxedLen), u16ToTxedLen, remote_ipaddr, remote_port);
            bTxedCompleted = TRUE;
        }
    }while(bTxedCompleted == FALSE);*/

    return (bRet);
}

CoAP_Result_t CoAP_RespHandler(CoAP_Message_t* pRespMsg, NetEp_t* Sender)
{
    uint32_t ulVar = COAP_RXED_EVENT;

    // printf("\r\nCoAP_RespHandler,%d,%s\r\n",pRespMsg->PayloadLength,pRespMsg->Payload);
    memset(u8ServerReturnData, 0, sizeof(u8ServerReturnData));
    memcpy(u8ServerReturnData,pRespMsg->Payload,pRespMsg->PayloadLength);
    xQueueSend(xQueue_CoAPRxed, &ulVar, 5/portTICK_PERIOD_MS);

    return (COAP_OK);
}

CoAP_Result_t CoAP_RespTimeoutHandler(CoAP_Message_t* pRespMsg, NetEp_t* Sender)
{
    uint32_t ulVar = COAP_TIMEOUT_EVENT;

    xQueueSend(xQueue_CoAPRxed, &ulVar, 5/portTICK_PERIOD_MS);

    return (COAP_OK);
}

MID_STATUS_TYPE_T midCoAP_BuildBlockOptionPkg(char *pucArrPkgCnt, uint8_t *pu8ValidLength, uint32_t index, bool bFollowing)
{
	char ucArrPkgCntTmp[3];
	uint32_t u32Index_tmp = 0;

	/* Check paramaters */
	usr_para_assert(pucArrPkgCnt != NULL);
	usr_para_assert(pu8ValidLength != NULL);

	memset(ucArrPkgCntTmp, '\0', sizeof(ucArrPkgCntTmp));
	u32Index_tmp = (bFollowing == TRUE) ? (0x08 | 0x04) : (0x00 | 0x04);
	u32Index_tmp |= (index << 4);

	if(index <= 0x0f)
	{
		ucArrPkgCntTmp[0]     = (uint8_t)u32Index_tmp;
		*pu8ValidLength = 1;
	}
	else if(index <= 0x0fff)
	{
		ucArrPkgCntTmp[0]     = (uint8_t)(u32Index_tmp >> 8);
		ucArrPkgCntTmp[1]     = (uint8_t)u32Index_tmp;
		*pu8ValidLength = 2;
	}
	else if(index <= 0x0fffff)
	{
		ucArrPkgCntTmp[0]     = (uint8_t)(u32Index_tmp >> 16);
		ucArrPkgCntTmp[1]     = (uint8_t)(u32Index_tmp >> 8);
		ucArrPkgCntTmp[2]     = (uint8_t)u32Index_tmp;
		*pu8ValidLength = 3;
	}

	memcpy(pucArrPkgCnt,ucArrPkgCntTmp,sizeof(ucArrPkgCntTmp));

	return (MID_OK);
}

MID_STATUS_TYPE_T midCoAP_AbstractBandSnAndMacFromString(char *pcDstBandSnArr,
														char *pcDstBandMacArr,
                                                        char *pucString)
{
	MID_STATUS_TYPE_T status = MID_ERROR;
	char ucTmpArr[30];
	char *pucStr1 = NULL;
	char *pucStr2 = NULL;

	/* Check paramaters */
	usr_para_assert(pucString != NULL);
	usr_para_assert(pcDstBandMacArr != NULL);
	usr_para_assert(pcDstBandSnArr != NULL);

	memset(ucTmpArr, '\0',sizeof(ucTmpArr));
	pucStr1 = strstr(pucString, "SN:");
	pucStr2 = strstr(pucString, "MAC:");
	if((NULL != pucStr1) && (NULL != pucStr2))
	{
		sscanf(pucStr1,"%*[^:]:%[^,]s", pcDstBandSnArr);
		M2Mstoh(ucTmpArr, pucStr2 + strlen("MAC:"), BLE_ADDR_LEN * 2);
		memcpy(pcDstBandMacArr, ucTmpArr, BLE_ADDR_LEN);

		status = MID_OK;
	}

	return (status);
}


MID_STATUS_TYPE_T midCoAP_ParseBandSnAndMacFromString(uint8_t *pu8ValidSnNum, 
																	const char *pscString)
{
	uint8_t u8Cnt = 0;
	uint8_t u8BandMacAddr[BLE_ADDR_LEN];
	char cPayloadArr[BOX_CONNECT_MAX_NUM * 30];
	char cBandSnTmp[BOX_SN_MAX_LEN];
	char *pcStr = NULL;

	/* Check paramaters */
	usr_para_assert(pscString != NULL);
	usr_para_assert(pu8ValidSnNum != NULL);

	memset(u8BandMacAddr, 0, BOX_CONNECT_MAX_NUM);
	memset(cPayloadArr, '\0', BOX_CONNECT_MAX_NUM);
	memset(cBandSnTmp, '\0', BOX_SN_MAX_LEN);
	sscanf(pscString, "%*[^=]=%[^#]s",cPayloadArr);
	if((NULL != strstr(cPayloadArr, "SN:")) && (NULL != strstr(cPayloadArr, "MAC:")))
	{
		pcStr = cPayloadArr;
		for(uint8_t i = 0;i < BOX_CONNECT_MAX_NUM;i ++)
		{
			memset(cBandSnTmp, '\0', BOX_SN_MAX_LEN);
			printf("ptr = %s\r\n", pcStr);
			if(NULL != strstr(pcStr,"####"))
			{
				if(MID_OK == midCoAP_AbstractBandSnAndMacFromString(cBandSnTmp, 
													(char *)u8BandMacAddr, 
													pcStr))
				{
					(*(box_info.ptdev_band_info + i))->sn = strtoul(cBandSnTmp, NULL, 10);
					memcpy((*(box_info.ptdev_band_info + i))->macAddr, u8BandMacAddr, BLE_ADDR_LEN);
					pcStr += strlen(cBandSnTmp) + 20;	// BLE_ADDR_LEN * 2 + 8;
					u8Cnt++;
				}
				else
				{
					break;
				}
			}
			else
			{
				if(MID_OK == midCoAP_AbstractBandSnAndMacFromString(cBandSnTmp, 
													(char *)u8BandMacAddr, 
													pcStr))
				{
					(*(box_info.ptdev_band_info + i))->sn = strtoul(cBandSnTmp, NULL, 10);
					memcpy((*(box_info.ptdev_band_info + i))->macAddr, u8BandMacAddr, BLE_ADDR_LEN);

					u8Cnt++;
				}
													
				break;
			}
		}
	}

	*pu8ValidSnNum = u8Cnt;

	return (MID_OK);
}

MID_STATUS_TYPE_T midCoAP_PostBlockWiseTransmit(char* UriString, uint8_t *pucSrcData,
											  uint16_t u16PkgSize,     uint32_t index, bool bFollowing, uint32_t u32TxedTotelLen)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
    char ucArrPkgCnt[4];
	uint8_t u8ValidLength = 0;
	

    memset(ucArrPkgCnt, '\0', sizeof(ucArrPkgCnt));
	midCoAP_BuildBlockOptionPkg(ucArrPkgCnt, &u8ValidLength, index, bFollowing);
    CoAP_StartNewPostRequest(UriString, ucArrPkgCnt, u8ValidLength, 0, &ServerEp, pucSrcData, u16PkgSize, u16PkgSize + 10, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
        	if(FALSE == bFollowing)
        	{
        		char * pcString = NULL;
				pcString = strstr(u8ServerReturnData,"assistants_bands_syncData=");
				if(NULL != pcString)
	            {
	            	uint32_t u32RxedLen = 0;

					u32RxedLen = strtoul(pcString + strlen("assistants_bands_syncData="), NULL, 10);
	            	if(u32RxedLen == u32TxedTotelLen)
	            	{
						status = MID_OK;
	            	}
					else
					{
						status = MID_ERROR;
					}
	            }
	            else
	            {
					status = MID_ERROR;
	            }
			}
			else
			{
				 status = MID_OK;
			}
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}


MID_STATUS_TYPE_T midCoAP_GetServerAvailable(uint8_t *pu8DstState)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
	char cUriArr[50];
	uint8_t u8BoxSnTmp[BOX_SN_MAX_LEN];

    /* Check parameters */
    usr_para_assert(pu8DstState != NULL);

	memcpy(u8BoxSnTmp, box_info.ucBoxSN,strlen(box_info.ucBoxSN));
	sprintf(cUriArr, "servers_status?assistantSn=%s", u8BoxSnTmp);
    CoAP_StartNewGetRequest(cUriArr, 0, &ServerEp, NULL, 0, 0, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            if(NULL != strstr(u8ServerReturnData,"servers_status=ok"))
            {
                *pu8DstState = 1;
            }
            else
            {
                *pu8DstState = 0;
            }

            status = MID_OK;
        }
        else
        {
            status = MID_ERROR;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_PutAssistantInfo(void)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint8_t ucPayload[100];
    uint16_t u16PayloadLen = 0;
    uint32_t u32RexdMessage = 0;
	char cUriArr[50];
	uint8_t u8BoxSnTmp[BOX_SN_MAX_LEN];
    char cBoxHwVerTmp[15];
    char cBoxFwVerTmp[15];

    memcpy(u8BoxSnTmp, box_info.ucBoxSN,strlen(box_info.ucBoxSN));
    memcpy(cBoxHwVerTmp, u8BoxHwVer,strlen(u8BoxHwVer));
    memcpy(cBoxFwVerTmp, u8BoxFwVer,strlen(u8BoxFwVer));
    memset(ucPayload, '\0', sizeof(ucPayload));
	sprintf(cUriArr, "assistants_info?assistantSn=%s", u8BoxSnTmp);
    u16PayloadLen += sprintf((char *)ucPayload, "%s####%s",cBoxHwVerTmp,cBoxFwVerTmp);
    CoAP_StartNewPutRequest(cUriArr, NULL, 0, &ServerEp, ucPayload, u16PayloadLen, u16PayloadLen + 10, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            if(NULL != strstr(u8ServerReturnData,"assistants_info=ok"))
            {
                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_GetRealServerIP(char * pucDstIpAddr, char * pucDstIpPort)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
    char *pucResp = NULL;
	uint8_t u8BoxSnTmp[BOX_SN_MAX_LEN];
    char ucIpAddrTmp[BOX_IPADDR_MAX_LEN];
    char ucPortTmp[BOX_IPPORT_MAX_LEN];
	char cUriArr[50];

    /* Check parameters */
    usr_para_assert(pucDstIpAddr != NULL);
    usr_para_assert(pucDstIpPort != NULL);

	memset(u8BoxSnTmp, '\0', sizeof(u8BoxSnTmp));
	memcpy(u8BoxSnTmp,box_info.ucBoxSN, BOX_SN_MAX_LEN);
	sprintf(cUriArr, "assistants_ipaddr_port?assistantSn=%s", u8BoxSnTmp);
    CoAP_StartNewGetRequest(cUriArr, 0, &ServerEp, NULL, 0, 0, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            pucResp = strstr((char *)u8ServerReturnData,"assistants_ipaddr_port=");
            if(NULL != pucResp)
            {
                memset(ucIpAddrTmp, '\0', BOX_IPADDR_MAX_LEN);
                memset(ucPortTmp, '\0', BOX_IPPORT_MAX_LEN);
                sscanf(pucResp,"%*[^=]=%[^:]s",ucIpAddrTmp);
                sscanf(pucResp,"%*[^:]:%s",ucPortTmp);
                strcpy(pucDstIpAddr, ucIpAddrTmp);
                strcpy(pucDstIpPort, ucPortTmp);

                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_GetSysTime(uint32_t *pu32Time)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
	uint8_t u8BoxSnTmp[BOX_SN_MAX_LEN];
    char ucSysTime[20];
	char cUriArr[50];

	memset(u8BoxSnTmp, '\0', sizeof(u8BoxSnTmp));
	memcpy(u8BoxSnTmp,box_info.ucBoxSN, BOX_SN_MAX_LEN);
	sprintf(cUriArr, "assistants_time?assistantSn=%s", u8BoxSnTmp);
    CoAP_StartNewGetRequest(cUriArr, 0, &ServerEp, NULL, 0, 0, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            if(NULL != strstr(u8ServerReturnData,"assistants_time="))
            {
                if(pu32Time != NULL)
                {
                    memset(ucSysTime, '\0', sizeof(ucSysTime));
                    sscanf(u8ServerReturnData,"%*[^=]=%s", ucSysTime);
                    *pu32Time = atoi(ucSysTime);
                }

                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_GetParedBandInfo(uint8_t *pu8BandNum)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
	uint8_t u8ValidBandNum = 0;
	char *pucStr = NULL;
    uint8_t u8BoxSnTmp[BOX_SN_MAX_LEN];
    char cUriString[50];

    /* Check parameters */
    usr_para_assert(pu8BandNum != NULL);

    memset(u8BoxSnTmp, '\0', sizeof(u8BoxSnTmp));
    memset(cUriString, '\0', sizeof(cUriString));
    memcpy(u8BoxSnTmp, box_info.ucBoxSN, strlen(box_info.ucBoxSN));
    sprintf(cUriString, "assistants_bands_info?assistantSn=%s", u8BoxSnTmp);
   	CoAP_StartNewGetRequest(cUriString, 0, &ServerEp, NULL, 0, 0, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
        	pucStr = strstr(u8ServerReturnData,"assistants_bands_info=");
            if(NULL != pucStr)
            {
				midCoAP_ParseBandSnAndMacFromString(&u8ValidBandNum, pucStr);
				if(u8ValidBandNum)
				{
                    *pu8BandNum = u8ValidBandNum;
				}
				else
				{
					*pu8BandNum = 0;
				}

                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_PutNewBandInfo(uint8_t index)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint8_t ucPayload[50];
    uint16_t u16PayloadLen = 0;
    uint32_t u32RexdMessage = 0;
    char ucBoxSnTmp[15];
    char ucUriTmp[50];
    char ucBandMacTmp[13];
    uint32_t u32BandSn = 0;

    memset(ucBoxSnTmp, '\0', sizeof(ucBoxSnTmp));
    memset(ucUriTmp, '\0', sizeof(ucUriTmp));
    memset(ucPayload, '\0', sizeof(ucPayload));
    memset(ucBandMacTmp, '\0', sizeof(ucBandMacTmp));

    memcpy(ucBoxSnTmp, box_info.ucBoxSN,strlen(box_info.ucBoxSN));
    u32BandSn = (*(box_info.ptdev_band_info + index))->sn;
    M2Mhtos(ucBandMacTmp, (char *)(*(box_info.ptdev_band_info + index))->macAddr, BLE_ADDR_LEN);
    sprintf((char *)ucUriTmp, "assistants_bands_info?assistantSn=%s",ucBoxSnTmp);
    u16PayloadLen += sprintf((char *)ucPayload, "%d####%s",u32BandSn, ucBandMacTmp);
    CoAP_StartNewPutRequest(ucUriTmp, NULL, 0, &ServerEp, ucPayload, u16PayloadLen, u16PayloadLen + 10, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            if(NULL != strstr(u8ServerReturnData,"assistants_bands_info=ok"))
            {
                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}


MID_STATUS_TYPE_T midCoAP_GetMonitorTemplate(char *pucDstMonitorTemplate, uint32_t u32BandSn)
{
    MID_STATUS_TYPE_T status = MID_ERROR;
    uint32_t u32RexdMessage = 0;
    uint8_t ucPayload[15];
    uint16_t u16PayloadLen = 0;
    char *pucResp = NULL;
	char cBoxSnTmp[BOX_SN_MAX_LEN];
	char cUriArr[100];

	/* Check parameters */
	usr_para_assert(pucDstMonitorTemplate != NULL);

	memcpy(cBoxSnTmp, box_info.ucBoxSN, BOX_SN_MAX_LEN);
    memset(ucPayload, '\0', sizeof(ucPayload));
	memset(cUriArr, '\0', sizeof(cUriArr));
	sprintf(cUriArr, "assistants_bands_monitor_template?assistantSn=%s&bandSn=%d", cBoxSnTmp, u32BandSn);
    u16PayloadLen += sprintf((char *)ucPayload, "%d",u32BandSn);
    CoAP_StartNewGetRequest(cUriArr, 0, &ServerEp, ucPayload, u16PayloadLen, u16PayloadLen + 10, CoAP_RespHandler);
    if(pdTRUE == xQueueReceive(xQueue_CoAPRxed, &u32RexdMessage, portMAX_DELAY))
    {
        if(u32RexdMessage == COAP_RXED_EVENT)
        {
            pucResp = strstr((char *)u8ServerReturnData,"assistants_bands_monitor_template=");
            if(NULL != pucResp)
            {
                strcpy(pucDstMonitorTemplate, (char *)(pucResp + strlen("assistants_bands_monitor_template=")));
                status = MID_OK;
            }
            else
            {
                status = MID_ERROR;
            }
        }
        else
        {
            status = MID_TIMEOUT;
        }
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_TransmitDataToServer(uint8_t *pucSrcData, uint16_t u16PkgSize, uint32_t index, bool bFollowing)
{
    MID_STATUS_TYPE_T status = MID_ERROR;

    if(MID_OK == midCoAP_PostBlockWiseTransmit("template_data", pucSrcData, u16PkgSize, index, bFollowing, 0))
    {
        status = MID_OK;
    }

    return (status);
}

MID_STATUS_TYPE_T midCoAP_TransmitMonitorDataToServer(char *pcUriString, uint8_t *pucSrcData, 
															uint16_t u16PkgSize, uint32_t index, bool bFollowing, uint32_t u32TxedTotelLen)
{
	uint8_t u8RetryCnt = 0;
	uint8_t status = 0;
	
	do{
	    if(MID_OK == midCoAP_PostBlockWiseTransmit(pcUriString, pucSrcData, u16PkgSize, index, bFollowing, u32TxedTotelLen))
	    {
			status = 1;
	    }
		else
		{
			u8RetryCnt++;
		}
	}while((status == 0) && (u8RetryCnt < 5));

	if(status == 1)
	{
		return MID_OK;
	}
	else
	{
		return MID_ERROR;
	}

	/*if(MID_OK == midCoAP_PostBlockWiseTransmit(pcUriString, pucSrcData, u16PkgSize, index, bFollowing, u32TxedTotelLen))
	    {
			return MID_OK;
	    }
		else
		{
			return MID_ERROR;
		}*/
}


CoAP_HandlerResult_t CoAP_ResourceObserverNotifier(CoAP_Observer_t* pListObservers, CoAP_Message_t* pResp)
{
    printf("\r\n***CoAP_ResourceObserverNotifier***\r\n");

    return (HANDLER_OK);
}

CoAP_Observer_t CoAP_Observer;
void mid_Coap_Init(void)
{
    CoAP_Observer.Ep.NetAddr.IPv4.u8[0] = ServerEp.NetAddr.IPv4.u8[0];
    CoAP_Observer.Ep.NetAddr.IPv4.u8[1] = ServerEp.NetAddr.IPv4.u8[1];
    CoAP_Observer.Ep.NetAddr.IPv4.u8[2] = ServerEp.NetAddr.IPv4.u8[2];
    CoAP_Observer.Ep.NetAddr.IPv4.u8[3] = ServerEp.NetAddr.IPv4.u8[3];
    CoAP_Observer.Ep.NetType = ServerEp.NetType;
    CoAP_Observer.Ep.NetPort = ServerEp.NetPort;
    CoAP_Observer.IfID = 0;
    CoAP_Observer.next = NULL;
    CoAP_Observer.Token = 0x55AA;
    CoAP_Observer.pOptList = NULL;

    CoAP_Res.next = NULL;
    CoAP_Res.Handler = 0;
    CoAP_Res.pListObservers = &CoAP_Observer;
    CoAP_Res.Notifier = CoAP_ResourceObserverNotifier;

    CoAP_Interactions_Instance.ifID = 0;
    CoAP_Interactions_Instance.next = NULL;
    CoAP_Interactions_Instance.Role = COAP_ROLE_NOTIFICATION; // COAP_ROLE_CLIENT; //COAP_ROLE_NOTIFICATION;
    CoAP_Interactions_Instance.pObserver->Ep.NetType = ServerEp.NetType;
    CoAP_Interactions_Instance.pObserver->Ep.NetPort = ServerEp.NetPort;
    for(int i = 0;i < 4;i ++)
    {
        CoAP_Interactions_Instance.pObserver->Ep.NetAddr.IPv4.u8[i] = ServerEp.NetAddr.IPv4.u8[i];
    }

}

void vCoAPTask(void * pvParameters)
{
    uint32_t u32RxData = 0;

    WDT_Register(WDT_COAP_TASK_ID);

    /* Infinit loop */
    while(1)
    {
        if(pdTRUE == xQueueReceive(xQueue_CoAP, &u32RxData, 200/portTICK_PERIOD_MS))
        {
            switch(u32RxData)
            {
                case COAP_DOWORK_EVENT:
                    WDT_Feed(WDT_COAP_TASK_ID);
                    CoAP_doWork();
                    break;

                default:
                    break;
            }
        }
        else
        {
            WDT_Feed(WDT_COAP_TASK_ID);
        }
    }
}


void CreatCoAPTask(void)
{
    xTaskCreate( vCoAPTask, "CoAPTask", configMINIMAL_STACK_SIZE * 10, NULL, tskCOAP_PRIORITY, &xCoAPTaskHandle );
    configASSERT( xCoAPTaskHandle );

    // Create a queue capable of containing 10 uint32_t values.
	xQueue_CoAP = xQueueCreate( 5, sizeof( uint32_t ) );
	configASSERT( xQueue_CoAP );
}

