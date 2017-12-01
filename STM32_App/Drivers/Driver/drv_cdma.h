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
 *   drv_Sim900Gprs.h
 * DESCRIPTION:
 *
 * HISTORY:
 *   2016年1月11日        Arvin         Create/Update
 *
*****************************************************************************/
#ifndef DRIVERS_SIM900_DRV_SIM900_H_
#define DRIVERS_SIM900_DRV_SIM900_H_

#include  "includes.h"

#define SIM900_DEBUG    0

typedef enum{
    CDMA_OFF = 0,
    CDMA_ON,
    CDMA_SET_ECHO,
    CDMA_SET_UART,
    CDMA_SET_ACCOUNT,
    CDMA_BUILD_CONN,
    CDMA_WAIT_CONN,
    CDMA_CONNECTED,
    CDMA_DISCONN
}cdma_stat_e;

extern cdma_stat_e cdma_stat;

#if SIM900_DEBUG
#define DBG_PRINTF   printf
#else
#define DBG_PRINTF(...)
#endif

HAL_StatusTypeDef drvSim_SyncATState(void);
HAL_StatusTypeDef drv_SimHwInit (void);
HAL_StatusTypeDef drv_SimHwON_OFF (void);
HAL_StatusTypeDef drv_SimHwReset(void);
HAL_StatusTypeDef drv_SimSWInit(void);
HAL_StatusTypeDef drv_SimSetAccoutPass(void);
HAL_StatusTypeDef drv_SimStartDial(void);
HAL_StatusTypeDef drv_SimConnectServer(uint8_t u8Socket, uint32_t u32LocalPort, char *pucIpAddr, char *pucPort, bool bUdpServer);
HAL_StatusTypeDef drvSim_GetTcpStatus(void);
HAL_StatusTypeDef drvSim_SendTcpDataToServer(uint8_t * pucBuffer, int16_t u16Size);
HAL_StatusTypeDef drvSim_SendTcpDataToServer_Ext(uint8_t * pucBuffer, int16_t u16Size);
HAL_StatusTypeDef drvSim_GetTcpDataFromServer(uint8_t * pucBuffer, uint16_t *pu16Size);
HAL_StatusTypeDef drvSim_GetSigStrength(uint8_t *pu8Rssi, int8_t *pi8Rssi);
HAL_StatusTypeDef drvSim_EntryRawMode(void);
HAL_StatusTypeDef drvSim_ExitRawMode(void);
HAL_StatusTypeDef drvSim_SendUdpDataToServer(uint8_t * pucBuffer, int16_t u16Size,
                                          char * pucServerIpAddr, char *pucPort);
HAL_StatusTypeDef drvSim_LookUpDns(char *pucUrl);
HAL_StatusTypeDef drv_SimQueryVersion(char * pucVersion);
HAL_StatusTypeDef drv_SimQueryEsn(char * pucESN);
HAL_StatusTypeDef drv_SimQueryEsn(char * pucESN);

HAL_StatusTypeDef drv_SimHwReset(void);
void vAppSIM900RxDataTaskCode ( void * pvParameters );
HAL_StatusTypeDef drvSim_Init(void);


#endif /* DRIVERS_SIM900_DRV_SIM900_H_ */

