/*
 * TypePorting.h
 *
 *  Created on: 2015��2��9��
 *      Author: Arvin.Liao
 */

#ifndef _TYPEPORTING_H_
#define _TYPEPORTING_H_

#include "stm32f2xx.h"
#include "includes.h"
#include "ff.h"			        /* Declarations of FatFs API */

/** @brief BLE address length. */
#define BLE_ADDR_LEN            6

typedef union {
	uint8_t 	arr[4];
	uint32_t 	l;
}uint32_or_uint8_u;

typedef enum
{
    MID_OK          = 0x00,
    MID_ERROR       = 0x01,
    MID_BUSY        = 0x02,
    MID_TIMEOUT     = 0x03,
    MID_PAUSE       = 0X04,
    MID_PARA_ERROR  = 0X05
} MID_STATUS_TYPE_T;

/* type define for monitor template */
typedef struct
{
    uint8_t *pbuf;
    uint8_t len;
}MONITOR_TEMPLATE_T;

/* type define for device ble information */
typedef struct 
{
    uint32_t                        sn;
    uint8_t                         macAddr[BLE_ADDR_LEN]; 
    bool                            isBleConnect;
    bool                            isStoreFileCreat;
    MONITOR_TEMPLATE_T              monitorTemplate;
    FIL                             *pFile;
}DEV_BLE_INFO_T;

#endif /* BRANUCH_SRC_TYPEPORTING_H_ */
