/*
 FreeRTOS V8.1.2 - Copyright (C) 2014 Real Time Engineers Ltd.
 All rights reserved

 VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that has become a de facto standard.             *
 *                                                                       *
 *    Help yourself get started quickly and support the FreeRTOS         *
 *    project by purchasing a FreeRTOS tutorial book, reference          *
 *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
 *                                                                       *
 *    Thank you!                                                         *
 *                                                                       *
 ***************************************************************************

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available from the following
 link: http://www.freertos.org/a00114.html

 1 tab == 4 spaces!

 ***************************************************************************
 *                                                                       *
 *    Having a problem?  Start by reading the FAQ "My application does   *
 *    not run, what could be wrong?"                                     *
 *                                                                       *
 *    http://www.FreeRTOS.org/FAQHelp.html                               *
 *                                                                       *
 ***************************************************************************

 http://www.FreeRTOS.org - Documentation, books, training, latest versions,
 license and Real Time Engineers Ltd. contact details.

 http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 including FreeRTOS+Trace - an indispensable productivity tool, a DOS
 compatible FAT file system, and our tiny thread aware UDP/IP stack.

 http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
 Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
 licenses offer ticketed support, indemnification and middleware.

 http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 engineered and independently SIL3 certified version for use in safety and
 mission critical applications that require provable dependability.

 1 tab == 4 spaces!
 */
/*
 * NOTE:  This file uses a third party USB CDC driver.
 */
#include "drv_FlashSpi.h"
/* Standard includes. */
#include "includes.h"
#include "apl_cli.h"
#include "CLI_CmdFile.h"
#define USE_FILE_SYS_CLI   /*支持文件系统命令的CLI_CMD*/

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE			50/*输入命令的缓存BUF长度*/

#define cmdQUEUE_LENGTH			    25

/* DEL acts as a backspace. */
#define cmdASCII_DEL				( 0x7F )

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */

static void prvUARTCommandConsoleTask (void *pvParameters);
static void apl_FormatLogo (char * pbuf);
/*-----------------------------------------------------------*/

static char * pcNewLine = "\r\n";
SemaphoreHandle_t xSemaphore_FW = NULL;
/*-----------------------------------------------------------*/

void apl_CliStart (uint16_t usStackSize, UBaseType_t uxPriority)
{

    portBASE_TYPE x = pdFALSE;
    mid_CliInit ();
    
    xSemaphore_FW = xSemaphoreCreateBinary();
    usr_res_assert(xSemaphore_FW != NULL);
    
    /* Create that task that handles the console itself. */
    x = xTaskCreate(prvUARTCommandConsoleTask, /* The task that implements the command console. */
        "CLI", /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
        usStackSize, /* The size of the stack allocated to the task. */
        NULL, /* The parameter is not used, so NULL is passed. */
        uxPriority, /* The priority allocated to the task. */
        NULL); /* A handle is not required, so just pass NULL. */
    while (x != pdTRUE);

    vRegisterFileSysCLICommands();
}

static void run_test(char *cInputString,
                    char *pcOutputString,
                    char *cLastInputString,
                    uint8_t *ucInputIndex)
{
    BaseType_t xReturned;

    do{
        /* Get the next output string from the command interpreter. */
        xReturned = MidCLI_ProcessCommand (cInputString,
                                            pcOutputString,
                                            configCOMMAND_INT_MAX_OUTPUT_SIZE);

        /* Write the generated string to the UART.
        * TODO: The length of the buffer that CMD process must be confirm
        * */
        mid_CliSendData ((uint8_t *) pcOutputString,strlen (pcOutputString));
        vTaskDelay (100);/*确保DMA发送时，TXBUF的内容不被改变*/
    }while (xReturned != pdFALSE);

    /* All the strings generated by the input command have been
    sent.  Clear the input string ready to receive the next command.
    Remember the command that was just processed first in case it is
    to be processed again. */
    strcpy (cLastInputString, cInputString);
    *ucInputIndex = 0;
    memset (cInputString, 0x00, cmdMAX_INPUT_SIZE);

    //vSerialPutString
    GetEndOfOutputMessage (pcOutputString);
    mid_CliSendData ((uint8_t *) pcOutputString, strlen (pcOutputString));
}

static void prvUARTCommandConsoleTask (void *pvParameters)
{
    uint8_t cRxedChar;
    uint8_t ucInputIndex = 0;
    char *pcOutputString;
    static char cInputString[cmdMAX_INPUT_SIZE];
    static char cLastInputString[cmdMAX_INPUT_SIZE];
    
    (void) pvParameters;
    pcOutputString = MidCLI_GetOutputBuffer ();
    /* Obtain the address of the output buffer.  Note there is no mutual
    exclusion on this buffer as it is assumed only one command console interface
    will be used at any one time. */

//    mid_FileSysInit();
//    MX_FATFS_Init();

    /* Send the welcome message. */
    apl_FormatLogo (pcOutputString);
    pcOutputString = MidCLI_GetOutputBuffer();
    mid_CliSendData((uint8_t *) pcOutputString, strlen (pcOutputString));
    vTaskDelay(150);
    GetEndOfOutputMessage (pcOutputString);
    mid_CliSendData ((uint8_t *) pcOutputString, strlen (pcOutputString));

    for (;;)
    {
        /* Wait for the next character.  The while loop is used in case
        INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
        be a genuine block time rather than an infinite block time. */
        cRxedChar = mid_CliGetChar ();

        /* Ensure exclusive access to the UART Tx. */
        if (cRxedChar != NULL)
        {
            /* Echo the character back. */
            //xSerialPutChar( xPort, cRxedChar, portMAX_DELAY );
            mid_CliSendData (&cRxedChar, sizeof(cRxedChar));

            /* Was it the end of the line? */
            if (cRxedChar == '\n' || cRxedChar == '\r')
            {
                /* Just to space the output from the input. */
                mid_CliSendData((uint8_t *)pcNewLine, strlen(pcNewLine));

                /* See if the command is empty, indicating that the last command
                is to be executed again. */
                if (ucInputIndex == 0)
                {
                    /* Copy the last command back into the input string. */
                    strcpy (cInputString, cLastInputString);
                }
                
                run_test(cInputString,pcOutputString,cLastInputString,&ucInputIndex);
                /* Pass the received command to the command interpreter.  The
                command interpreter is called repeatedly until it returns
                pdFALSE	(indicating there is no more output) as it might
                generate more than one string. */
				
            }
            else
            {
                if (cRxedChar == '\r')
                {
                /* Ignore the character. */
                }
                else if ((cRxedChar == '\b') || (cRxedChar == cmdASCII_DEL))
                {
                    /* Backspace was pressed.  Erase the last character in the
                    string - if any. */
                    if (ucInputIndex > 0)
                    {
                        ucInputIndex--;
                        cInputString[ucInputIndex] = '\0';
                    }
                }
                else
                {
                    /* A character was entered.  Add it to the string entered so
                    far.  When a \n is entered the complete	string will be
                    passed to the command interpreter. */
                    if ((cRxedChar >= ' ') && (cRxedChar <= '~'))
                    {
                        if (ucInputIndex < cmdMAX_INPUT_SIZE)
                        {
                            cInputString[ucInputIndex] = cRxedChar;
                            ucInputIndex++;
                        }
                    }
                }
            }
        }
    }
}
/*-----------------------------------------------------------*/
void vOutputString (const char * const pcMessage)
{
    
}
/*-----------------------------------------------------------*/

void GetEndOfOutputMessage (char * pbuf)
{
    char *path = NULL;
    
    path = cli_CmdGetPath();
    sprintf(pbuf,"\r\n%s>",path);
    *(pbuf+2) += 0x11;
}

/*
 *********************************************************************************************************
 *	函 数 名: PrintfLogo
 *	功能说明: 打印例程名称和例程发布日期, 接上串口线后，打开PC机的超级终端软件可以观察结果
 *	形    参：无
 *	返 回 值: 无
 *********************************************************************************************************
 */
static void apl_FormatLogo (char * pbuf)
{
    pbuf += sprintf(pbuf,"\r\n*************************************************************\r\n");
    pbuf += sprintf(pbuf,"*                        GNSS TIME                          *\r\n"); 
    pbuf += sprintf(pbuf,"*                        %s                        *\r\n",__DATE__);
    pbuf += sprintf(pbuf,"* 	             HW: %s,FW: %s 	       	    *\r\n", u8BoxHwVer,u8BoxFwVer);
    pbuf += sprintf(pbuf,"*             Copyright (c)  2005-2050  ICT  Inc.           *\r\n");
    pbuf += sprintf(pbuf,"*                   All rights reserved                     *\r\n");
    pbuf += sprintf(pbuf,"*************************************************************\r\n");
    pbuf += sprintf(pbuf,"\r\nType Help to view a list of registered commands.\r\n");
}

void CreateCliTask(void)
{
//    apl_CliStart (configMINIMAL_STACK_SIZE * 40, osPriorityNormal);
    apl_CliStart( configMINIMAL_STACK_SIZE * 40, tskCLI_PRIORITY);
}
