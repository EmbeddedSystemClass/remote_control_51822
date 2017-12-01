#ifndef _MID_NET_H_
#define _MID_NET_H_

#include "stm32f2xx_hal.h"
#include "main.h"
#include "apl_monitor_template.h"


#define MAX_LEN_PER_PKG								(256)

MID_STATUS_TYPE_T mid_TransmitFileToServer(char *pucFileName);
MID_STATUS_TYPE_T mid_TransmitFileToServer_test(char *pucFileName);
MID_STATUS_TYPE_T mid_TransmitBandSycDataToServer(char *pucFileName, uint32_t u32BandSn);
MID_STATUS_TYPE_T mid_GetMonitorTemplate(void);
MID_STATUS_TYPE_T mid_ServerInit(void);

#endif          /* End of #ifndef _MID_NET_H_*/

