/*
 * drv_Uart.h
 *
 *  Created on: 2015��5��4��
 *      Author: yyy
 */

#ifndef _DRV_CMUUART_H_
#define _DRV_CMUUART_H_
/* Scheduler includes. */

#include "includes.h"

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)

/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported functions ------------------------------------------------------- */
typedef struct
{
    HAL_StatusTypeDef       (*Init)(UART_HandleTypeDef *uartHandle);
    HAL_StatusTypeDef       (*Write)( uint8_t *, uint16_t );
    char       			     (*ReadCharBlock)(void);
    char       			     (*ReadAchar)(void);
    HAL_StatusTypeDef	      (*Read)( uint8_t *, uint32_t );
    HAL_StatusTypeDef	     (*GetAccess)(void);
    HAL_StatusTypeDef       (*ReleaseAccess)(void);
    bool		 	    (*IsInit)(void);
} SHELL_DrvTypeDef;

extern SHELL_DrvTypeDef  shellDrv;

extern SemaphoreHandle_t xRawModeTxSem;

void ShellCmuUartTxCpltCallBack(void);
void ShellCmuUartRxCpltCallBack(void);

#endif /* ECGPB_DRIVERS_DRV_CMUUART_H_ */
