#ifndef __SERVICE_H
#define __SERVICE_H

#include "includes.h"

typedef enum {
	SERVER_INIT_EVENT,
	SERVER_APP_INIT_EVENT,
	SERVER_DISCONNECT_EVENT,
	SERVER_UPLOAD_FILE_EVENT,
	SERVER_CHECK_SIGNAL_EVENT,
	SERVER_GETCONFIG_EVENT,
	SERVER_ADDBAND_EVENT,
	APP_2GBOX_MCU_DOWNLOAD_FILE_EVENT,
	APP_2GBOX_BLE_DOWNLOAD_FILE_EVENT,
	APP_2GBOX_MCU_FW_UPDATE_EVENT,
	APP_2GBOX_BLE_FW_UPDATE_EVENT
}apl_service_en;

typedef struct {
	apl_service_en apl_event_id;
	uint32_t u32Value;
}apl_service_event_t;

#define COAP_RXED_EVENT                         (1 << 0)
#define COAP_TIMEOUT_EVENT                      (1 << 1)


extern QueueHandle_t xQueue_AppServer;
extern SemaphoreHandle_t xSemaphore_GSM;
extern SemaphoreHandle_t xSemaphore_PENDING;
extern QueueHandle_t xQueue_CoAPRxed;

extern uint32_t u32CoAPCnt;
extern char u8ServerReturnData[1024];
int8_t BuildDownAddr(char *pcString);
void mid_ServerStart (void);
void StartCheckServerStateTimer(void);
void StopCheckServerStateTimer(void);

void CreateServerTask(void);


#endif /* End of #ifndef __SERVICE_H */
