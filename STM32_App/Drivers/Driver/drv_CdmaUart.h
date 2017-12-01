/*
 * drv_Uart.h
 *
 *  Created on: 2015��5��4��
 *      Author: yyy
 */

#ifndef __DRV_SIM_UART_H_
#define __DRV_SIM_UART_H_

#include "stm32f2xx_hal.h"
#include "includes.h"

#define MAX_LEN_SIM_UART                                (1600)

#define SIM_UART_CMD_RX                                 (1)

#define OK                  (1<<0)
#define ERROR           	(1<<1)


typedef struct {
    uint8_t     u8Cmd;
    uint16_t    u16Len;
    uint8_t     *pu8Data;
}sim_rx_data_t;

typedef struct {
    HAL_StatusTypeDef (*pfInterfaceInit)(void);
    HAL_StatusTypeDef (*pfHwInit)(void);
    HAL_StatusTypeDef (*pfFwInit)(void);
    HAL_StatusTypeDef (*pfHwON)(void);
    HAL_StatusTypeDef (*pfHwOFF)(void);
    HAL_StatusTypeDef (*pfHwReset)(void);
    HAL_StatusTypeDef (*pfInterfaceTransmit)(char *, uint16_t);       // (uint8_t * pucSrcBuffer, uint16_t u16Size)
    HAL_StatusTypeDef (*pfInterfaceTransmit_Raw)(char *, uint16_t); // (uint8_t * pucSrcBuffer, uint16_t u16Size)
    HAL_StatusTypeDef (*pfInterfaceReceive)(char *, uint16_t *,uint32_t); // (uint8_t * pucDstBuffer, uint16_t * u16Size,uint32_t u32DelayTimes)
}net_init_t;

typedef struct {
    bool bOnLine;
    uint8_t u8SQ;
    int8_t i8SQdBm;
}sim800c_stat_t;

extern uint8_t u8SimUartRxByte;
extern uint8_t u8PreSimRxArr[MAX_LEN_SIM_UART];
extern uint8_t u8SimRxArr[MAX_LEN_SIM_UART];

extern uint8_t u8UartNoEcho;
extern net_init_t sim800c;
extern sim800c_stat_t sim800c_stat;
extern uint16_t u16SimUartRxCnt;

extern QueueHandle_t xQueue_SIM_UART_RX;
extern QueueHandle_t xQueue_SIM_UART_RX_Data;
extern SemaphoreHandle_t xMutex_InterfaceTx;
extern SemaphoreHandle_t xSem_InterfaceAtRx;
extern EventGroupHandle_t xEventGroup_sim;
extern TaskHandle_t xUart2GRxTaskHandle;


HAL_StatusTypeDef drv_SimUART_StartRx (void);
HAL_StatusTypeDef drv_SimUART_TaskCreate (void);

#endif /* End of #ifndef __DRV_SIM_UART_H_ */

