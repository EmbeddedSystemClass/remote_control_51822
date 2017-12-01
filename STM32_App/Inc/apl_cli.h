/*
 * apl_CliUart.h
 *
 *  Created on: 2015��5��4��
 *      Author: yyy
 */

#ifndef ECGPB_SRC_APL_COM_APL_CLIUART_H_
#define ECGPB_SRC_APL_COM_APL_CLIUART_H_

#include "includes.h"

/* Example includes. */
#include "mid_CLI.h"
/* Demo application includes. */

#define cmdMAX_MUTEX_WAIT		( ( ( TickType_t ) 300 ) / ( portTICK_PERIOD_MS ) )

typedef struct 
{
	char ch;
} serial_msg;

typedef struct {
	char(*getc)(void);
	void (*putc)(char str);
	int (*puts)(const char *msg);
	int (*gets)(void);
	int (*printf)(const char *format, ...);
} SERIAL;

extern SemaphoreHandle_t xSemaphore_FW;

//void apl_CliStart (uint16_t usStackSize, UBaseType_t uxPriority);
void GetEndOfOutputMessage(char * pbuf);
void CreateCliTask(void);

#endif /* ECGPB_SRC_APL_COM_APL_CLIUART_H_ */
