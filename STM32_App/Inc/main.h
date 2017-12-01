/*
 * main.h
 *
 *  Created on: 2015年6月18日
 *      Author: Arvin
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f2xx_hal.h"
#include "includes.h"

#define BOX_SN_MAX_LEN                          (11)
#define BOX_IPADDR_MAX_LEN                      (16)
#define BOX_IPPORT_MAX_LEN                      (6)
#define BOX_CONNECT_MAX_NUM                      (1)

typedef enum{
		SERVER_SETUP_INIT,
		SERVER_SETUP_AVAILABLE,
		SERVER_SETUP_BASEINFO,
		SERVER_SETUP_CONFIGIP,
		SERVER_SETUP_TIME,
		SERVER_SETUP_BANDINFO,
		SERVER_SETUP_MONITORTEMP,
		SERVER_SETUP
}server_setup_t;

typedef struct{
    uint8_t state;
    char ucBoxSN[BOX_SN_MAX_LEN];
    char ucIpAddr[BOX_IPADDR_MAX_LEN];
    char ucPort[BOX_IPPORT_MAX_LEN];
	server_setup_t server_setup;
	uint8_t u8SupportBandMaxNum;
	uint8_t u8BandNum;
	DEV_BLE_INFO_T *ptdev_band_info[BOX_CONNECT_MAX_NUM];
}box_info_t;

extern box_info_t box_info;
extern uint32_t u32Tim6IntCnt;
extern const char * u8BoxHwVer;
extern const char * u8BoxFwVer;

extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);

void TIM6_Start(void);
void TIM6_Stop(void);
void TIM7_Start(void);
void TIM7_Stop(void);

int GetBoxSN(char *pucDstBoxSn, uint32_t u32Addr);



#endif /* _MAIN_H_ */

