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
#ifndef COAP_ESP8266_INTERFACE_H
#define COAP_ESP8266_INTERFACE_H

#include <stdbool.h>
#include "stm32f2xx_hal.h"
#include "net_packet.h"
#include "net_Socket.h"
#include "coap_resource.h"
#include "includes.h"
#include "TypePorting.h"

//-----------------------------------------------------------------------
// Please use at least ESP8266_NONOS_SDK_v1.4.1_15_10_27 !!!!
// v1.4.0 -> 1.4.1 : http://bbs.espressif.com/viewtopic.php?f=46&t=1268
//-----------------------------------------------------------------------
typedef void (* espconn_connect_callback)(void *arg);
typedef void (* espconn_reconnect_callback)(void *arg, int8_t err);

typedef struct _esp_tcp {
    int remote_port;
    int local_port;
    uint8_t local_ip[4];
    uint8_t remote_ip[4];
    espconn_connect_callback connect_callback;
    espconn_reconnect_callback reconnect_callback;
    espconn_connect_callback disconnect_callback;
	espconn_connect_callback write_finish_fn;
} esp_tcp;

typedef struct _esp_udp {
    int remote_port;
    int local_port;
    uint8_t local_ip[4];
	uint8_t remote_ip[4];
} esp_udp;

/** A callback prototype to inform about events for a espconn */
typedef void (* espconn_recv_callback)(void *arg, char *pdata, unsigned short len);
typedef void (* espconn_sent_callback)(void *arg);

/** Protocol family and type of the espconn */
enum espconn_type {
    ESPCONN_INVALID    = 0,
    /* ESPCONN_TCP Group */
    ESPCONN_TCP        = 0x10,
    /* ESPCONN_UDP Group */
    ESPCONN_UDP        = 0x20,
};

/** Current state of the espconn. Non-TCP espconn are always in state ESPCONN_NONE! */
enum espconn_state {
    ESPCONN_NONE,
    ESPCONN_WAIT,
    ESPCONN_LISTEN,
    ESPCONN_CONNECT,
    ESPCONN_WRITE,
    ESPCONN_READ,
    ESPCONN_CLOSE
};

/** A espconn descriptor */
struct espconn {
    /** type of the espconn (TCP, UDP) */
    enum espconn_type type;
    /** current state of the espconn */
    enum espconn_state state;
    union {
        esp_tcp *tcp;
        esp_udp *udp;
    } proto;
    /** A callback function that is informed about events for this espconn */
    espconn_recv_callback recv_callback;
    espconn_sent_callback sent_callback;
    uint8_t link_cnt;
    void *reverse;
};

typedef struct remot_info{
	enum espconn_state state;
	int remote_port;
	uint8_t remote_ip[4];
}remot_info;

typedef struct {

	bool TxSocketIdle;
	uint8_t StationConStatus;

}CoAP_ESP8266_States_t;


#define COAP_DOWORK_EVENT                   (1)



extern CoAP_ESP8266_States_t CoAP_ESP8266_States;
extern NetEp_t ServerEp;
extern struct espconn Esp8266_conn;
extern CoAP_Res_t CoAP_Res;
extern QueueHandle_t xQueue_CoAP;


bool CoAP_ESP8266_SendDatagram(uint8_t ifID, NetPacket_t* pckt);
NetSocket_t* CoAP_ESP8266_CreateInterfaceSocket(uint8_t ifID, struct espconn* pEsp8266_conn, uint16_t LocalPort, NetReceiveCallback_fn Callback, NetTransmit_fn SendPacket);
bool  CoAP_ESP8266_ConfigDevice(void);
void udp_recv_cb(void *arg, char *pdata, unsigned short len);

#define USE_HARDCODED_CREDENTIALS (0) //other option: set via coap on soft-ap interface
#define EXTERNAL_AP_SSID "YOUR-SSID"
#define EXTERNAL_AP_PW "YOUR-WIFI-SECRET"

//enable always the softap even if connected to external router/ap
//soft ap config: ssid = "Lobaro-CoAP (ESP8266)", pw= "lobaro!!", AP-IP: 192.168.4.1
//note: soft-ap will not work properly until esp8266 is trying to connect to external router/ap
//see here why: http://bbs.espressif.com/viewtopic.php?t=324
#define SOFTAP_ALLWAYS_ON (0) //0=only soft-ap is connection to ap failed 1=soft-ap allways on

#define MAX_CON_RETRIES_BEFORE_ACTIVATING_SOFT_AP (2) //turn on the softAP after x-connection retries to external AP/Router

CoAP_Result_t CoAP_RespTimeoutHandler(CoAP_Message_t* pRespMsg, NetEp_t* Sender);
MID_STATUS_TYPE_T midCoAP_GetServerAvailable(uint8_t *pu8DstState);
MID_STATUS_TYPE_T midCoAP_PutAssistantInfo(void);
MID_STATUS_TYPE_T midCoAP_GetRealServerIP(char * pucDstIpAddr, char * pucDstIpPort);
MID_STATUS_TYPE_T midCoAP_GetSysTime(uint32_t *pu32Time);
MID_STATUS_TYPE_T midCoAP_GetParedBandInfo(uint8_t *pu8BandNum);
MID_STATUS_TYPE_T midCoAP_GetMonitorTemplate(char *pucDstMonitorTemplate, uint32_t u32BandSn);
MID_STATUS_TYPE_T midCoAP_PutBlockWiseTransmit(char* UriString, uint8_t *pucSrcData, uint16_t u16PkgSize,
                                             uint32_t index, bool bFollowing);
MID_STATUS_TYPE_T midCoAP_PutNewBandInfo(uint8_t index);
MID_STATUS_TYPE_T midCoAP_ParseBandSnAndMacFromString(uint8_t *pu8ValidSnNum, const char *pscString);

// Sync data cmd
MID_STATUS_TYPE_T midCoAP_TransmitMonitorDataToServer(char *pcUriString, uint8_t *pucSrcData, 
															uint16_t u16PkgSize, uint32_t index, 
															bool bFollowing, uint32_t u32TxedTotelLen);
MID_STATUS_TYPE_T midCoAP_PostSyncDataPkgStart(char *pcUriString, char *pcPayload,uint16_t u16PayloadLen);
MID_STATUS_TYPE_T midCoAP_PostSyncDataPkgStop(char *pcUriString, uint16_t u16crc);

MID_STATUS_TYPE_T midCoAP_TransmitDataToServer(uint8_t *pucSrcData, uint16_t u16PkgSize, uint32_t index, bool bFollowing);

void CreatCoAPTask(void);


#endif

