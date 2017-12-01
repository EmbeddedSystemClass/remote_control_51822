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

/******************************************************************************
 *
 * See the following URL for information on the commands defined in this file:
 * http://www.FreeRTOS.org/FreeRTOS-Plus/FreeRTOS_Plus_UDP/Embedded_Ethernet_Examples/Ethernet_Related_CLI_Commands.shtml
 *
 ******************************************************************************/

/* FreeRTOS includes. */
#include "CLI_CmdFile.h"
#include "includes.h"
#include "ymodem.h"


uint8_t yModeBuf[1024];
uint8_t fileName[FILE_NAME_LENGTH_YM];

char pathCurrent[50];

/* FatFs API的返回值 */
static const char * FR_Table[] =
  { "FR_OK", /* (0) Succeeded 成功*/
  "FR_DISK_ERR", /* ：底层硬件错误(1) A hard error occurred in the low level disk I/O layer */
  "FR_INT_ERR", /* ：断言失败(2) Assertion failed */
  "FR_NOT_READY", /* ：物理驱动没有工作(3) The physical drive cannot work */
  "FR_NO_FILE", /*：文件不存在 (4) Could not find the file */
  "FR_NO_PATH", /* ：路径不存在(5) Could not find the path */
  "FR_INVALID_NAME", /* ：无效文件名(6) The path name format is invalid */
  "FR_DENIED", /* (7) ：由于禁止访问或者目录已满访问被拒绝Access denied due to prohibited access or directory full */
  "FR_EXIST", /* ：文件已经存在(8) Access denied due to prohibited access */
  "FR_INVALID_OBJECT", /* ：文件或者目录对象无效(9) The file/directory object is invalid */
  "FR_WRITE_PROTECTED", /* ：物理驱动被写保护(10) The physical drive is write protected */
  "FR_INVALID_DRIVE", /* ：逻辑驱动号无效(11) The logical drive number is invalid */
  "FR_NOT_ENABLED", /* ：卷中无工作区(12) The volume has no work area */
  "FR_NO_FILESYSTEM", /* ：没有有效的FAT卷(13) There is no valid FAT volume */
  "FR_MKFS_ABORTED", /*：由于参数错误f_mkfs()被终止 (14) The f_mkfs() aborted due to any parameter error */
  "FR_TIMEOUT", /* ：在规定的时间内无法获得访问卷的许可(15) Could not get a grant to access the volume within defined period */
  "FR_LOCKED", /* ：由于文件共享策略操作被拒绝(16) The operation is rejected according to the file sharing policy */
  "FR_NOT_ENOUGH_CORE", /* ：无法分配长文件名工作区(17) LFN working buffer could not be allocated */
  "FR_TOO_MANY_OPEN_FILES", /* ：当前打开的文件数大于_FS_SHARE(18) Number of open files > _FS_SHARE */
  "FR_INVALID_PARAMETER" /* ：参数无效(19) Given parameter is invalid */
  };
/*
 * The function that registers the commands that are defined within this file.
 */

/*
 * Implements the task-stats command.
 */
static BaseType_t prFileDirCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvrFileCDCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileMkdirCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileDelCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileChmodCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileDownCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileUpCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvFileMkFsCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvSendAtCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvPwrOffCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvBleTestCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvSendCommand (char *pcWriteBuffer, size_t xWriteBufferLen,const char *pcCommandString);


/* Structure that defines the "run-time-stats" command line command.   This
 generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t xFileDirStats =
{
    "dir", /* The command string to type. */
    "\r\ndir:\r\n Displays File system information\r\n",
    prFileDirCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command.  This generates
 a table that gives information on each task in the system. */
static const CLI_Command_Definition_t xFileCdStats =
{
    "cd",                                     /* The command string to type. */
    "\r\ncd:\r\n Entry file dir\r\n",
    prvrFileCDCommand,                        /* The function to run. */
    1                                         /* No parameters are expected. */
};

/* Structure that defines the "echo_3_parameters" command line command.  This
 takes exactly three parameters that the command simply echos back one at a
 time. */
static const CLI_Command_Definition_t xFileMkDirStats =
{
    "mkdir",
    "\r\nmkdir:\r\n Creat file dir\r\n",
    prvFileMkdirCommand,        /* The function to run. */
    1                           /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_3_parameters" command line command.  This
 takes exactly three parameters that the command simply echos back one at a
 time. */
static const CLI_Command_Definition_t xFileDelStats =
{
    "del",
    "\r\ndel\r\n delete file or empty dir \r\n",
    prvFileDelCommand,          /* The function to run. */
    1                           /* Three parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t xFileChmodStats =
{
    "chmod",
    "\r\nchmod\r\n delete file or empty dir \r\n",
    prvFileChmodCommand,        /* The function to run. */
    1                           /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "run-time-stats" command line command. */
static const CLI_Command_Definition_t xFileDownStats =
{
    "download",                 /* The command string to type. */
    "\r\ndownload:\r\n Download a file via serial port\r\n",
    prvFileDownCommand,         /* The function to run. */
    0                           /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xFileUpStats =
{
    "upload",                   /* The command string to type. */
    "\r\nupload:\r\n Upload a file via serial port\r\n",
    prvFileUpCommand,           /* The function to run. */
    1                           /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xFileMkFsStats =
{
    "format",                   /* The command string to type. */
    "\r\nformat:\r\n format the driver\r\n",
    prvFileMkFsCommand,         /* The function to run. */
    0                           /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xSimAtStats =
{
    "at",                   /* The command string to type. */
    "\r\nat:\r\n Send the AT command to the module\r\n",
    prvSendAtCommand,         /* The function to run. */
    0                           /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xPwrOffStats =
{
    "pwroff",                   /* The command string to type. */
    "\r\npwroff:\r\n Power off the system\r\n",
    prvPwrOffCommand,         /* The function to run. */
    0                           /* No parameters are expected. */
};
/* Structure that defines the "ble test" command line command. */
static const CLI_Command_Definition_t xBleTestStats =
{
    "ble",                   /* The command string to type. */
    "\r\nble:\r\n ble test\r\n",
    prvBleTestCommand,          /* The function to run. */
    1                           /* No parameters are expected. */
};

/* Structure that defines the "ble test" command line command. */
static const CLI_Command_Definition_t xSendStats =
{
    "send",                   /* The command string to type. */
    "\r\nsend:\r\n send a file to server\r\n",
    prvSendCommand,          /* The function to run. */
    0                           /* No parameters are expected. */
};


/*-----------------------------------------------------------*/

char * cli_CmdGetPath(void)
{
    f_getcwd (pathCurrent, 50);
    return pathCurrent;
}


void vRegisterFileSysCLICommands (void)
{
    /* Register all the command line commands defined immediately above. */
    MidCLI_RegisterCommand (&xFileDirStats);
    MidCLI_RegisterCommand (&xFileCdStats);
    MidCLI_RegisterCommand (&xFileMkDirStats);
    MidCLI_RegisterCommand (&xFileDelStats);
    MidCLI_RegisterCommand (&xFileChmodStats);
    MidCLI_RegisterCommand (&xFileMkFsStats);
    MidCLI_RegisterCommand (&xFileDownStats);
    MidCLI_RegisterCommand (&xFileUpStats);
    MidCLI_RegisterCommand (&xSimAtStats);
    MidCLI_RegisterCommand (&xPwrOffStats);
	MidCLI_RegisterCommand (&xBleTestStats);
    MidCLI_RegisterCommand (&xSendStats);
}
/*-----------------------------------------------------------*/

static BaseType_t prFileDirCommand (char *pcWriteBuffer,
                                size_t xWriteBufferLen,
                                const char *pcCommandString)
{
	const char * const pcDir = "<DIR>";
	const char * const pcFile = "<FIL>";
	uint32_t cnt = 0;
	FRESULT res;
	char lfname[_MAX_LFN + 1];

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	/* Generate a table of task stats. */

	DIR DirInf;/*由于MAX_SS配置该大，此结构体中包含了4KB的数据区，需要较多的内存堆，否则在MEMSET时，出现HARDFAULT*/
	FILINFO fileinf;
	char *fn;   /* This function assumes non-Unicode configuration */

     memset(pcWriteBuffer,0,50);
     f_getcwd (pcWriteBuffer, 20);
     res = f_opendir (&DirInf, (const TCHAR*)pathCurrent); // 如果不带参数，则从当前目录开始

	pcWriteBuffer += sprintf (pcWriteBuffer, "Current Path:%s", pathCurrent);
	pcWriteBuffer += sprintf (pcWriteBuffer, "    File/Directory Info\r\n");
	if (res != FR_OK)
	{
		sprintf (pcWriteBuffer, "Dir Open Error! %s \r\n", FR_Table[res]);
		return pdFALSE;
	}
	else
    {
		fileinf.fsize = sizeof(lfname);
		for (cnt = 0;; cnt++)
		{   //是否读完根据当前扇区sect的值，因为目录项读完后设为0.
			res = f_readdir (&DirInf, &fileinf); //读取目录项，索引会自动下移。
			if (res != FR_OK || fileinf.fname[0] == 0)
			{
				pcWriteBuffer += sprintf (pcWriteBuffer, "Read over! %s \r\n",FR_Table[res]);
				break;
			}

			if (fileinf.fname[0] == '.')
			{
				continue;
			}

			if ((fileinf.fattrib & AM_DIR) == AM_DIR)
			{
				pcWriteBuffer += sprintf (pcWriteBuffer, "\t%s", pcDir);
			}
			else
			{
				pcWriteBuffer += sprintf (pcWriteBuffer, "\t%s", pcFile);
			}
			
#if _USE_LFN
			fn = *fileinf.lfname ? fileinf.lfname : fileinf.fname;
#else
			fn = fileinf.fname;
#endif

			pcWriteBuffer += sprintf (pcWriteBuffer, "\t%s\t%d Bytes\r\n",fn, fileinf.fsize);
			if (mid_CliWritePointCheck (pcWriteBuffer))
			{
				return pdFALSE;
			}
		}

    }
  	f_closedir(&DirInf);
	
  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;

    /* Get volume information and free clusters of drive 1 */
    res = f_getfree (pathCurrent, &fre_clust, &fs);
    if (res != FR_OK)
    {
        pcWriteBuffer += sprintf(pcWriteBuffer, "Get Free Space Error! %s \r\n",FR_Table[res]);
        return pdFALSE;
    }

  /* Get total sectors and free sectors */
  tot_sect = (fs->n_fatent - 2) * fs->csize;
  fre_sect = fre_clust * fs->csize;

  /* Print the free space (assuming 512 bytes/sector) */
  pcWriteBuffer += sprintf (pcWriteBuffer,"\r\n%10lu KiB total drive space. %10lu KiB available.\r\n", tot_sect / 2, fre_sect / 2);

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/
/*改变驱动器的当前目录*/
static BaseType_t prvrFileCDCommand (char *pcWriteBuffer,
                                 size_t xWriteBufferLen,
		                         const char *pcCommandString)
{

    FRESULT res;
    const char *pcParameter;

    BaseType_t xParameterStringLength;
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    /* Obtain the parameter string. */
    pcParameter = MidCLI_GetParameter (pcCommandString, /* The command string itself. */
                    			     1, /* Return the next parameter. */
                    			     &xParameterStringLength /* Store the parameter string length. */
                    			     );
    if (*pcParameter == NULL)
    {
        sprintf (pcWriteBuffer, "Please input path! like /dir1  \r\n");
        return pdFALSE;
    }

    res = f_chdir (pcParameter);     //这个函数没有对磁盘内容作任何改变
    if (res != FR_OK)
    {
        pcWriteBuffer+=sprintf (pcWriteBuffer, "Change Dir  Error! %s \r\n", FR_Table[res]);
    }
    else
    {
        pcWriteBuffer+=sprintf (pcWriteBuffer, "Dir Changed!\r\n");
    }

    pcWriteBuffer+=sprintf (pcWriteBuffer, "Current Path: ");
    f_getcwd (pcWriteBuffer, 10);
    pcWriteBuffer += 10;
    strcpy (pcWriteBuffer, "\r\n");

    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvFileMkdirCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  const char *pcParameter;
  BaseType_t xParameterStringLength;
  /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
  (void) xWriteBufferLen;
  configASSERT(pcWriteBuffer);
  /* Generate a table of task stats. */
  FRESULT res;

  /* Obtain the parameter string. */
  pcParameter = MidCLI_GetParameter (pcCommandString, /* The command string itself. */
				     1, /* Return the next parameter. */
				     &xParameterStringLength /* Store the parameter string length. */
				     );

	memset(pcWriteBuffer,0,100);
	strcpy(pcWriteBuffer,pathCurrent);
	strcat(pcWriteBuffer,pcParameter);
  	res = f_mkdir (pcWriteBuffer);

  	if (res != FR_OK)
    {
      sprintf(pcWriteBuffer,"Make Dir  Error,Input path like: sub1  or sub1/sub2 or sub1/sub2/sub3. ! %s   Parameter: %s\r\n",
	  			FR_Table[res], pcParameter);
    }
  	else
    {
      	strcpy (pcWriteBuffer, "Dir Created!\r\n");
    }

  return pdFALSE;
}

/*       初始化时间
 *       char fmt[] = "%Y-%m-%d-%H:%M:%S";
 *       char buf[] = "2000-01-01-00:00:00";
 *       struct tm tb;
 *       if (strptime(buf, fmt, &tb) != NULL) {
 *       	fprintf(stdout "ok");
 *       }*/
static BaseType_t
prvFileDelCommand (char *pcWriteBuffer, size_t xWriteBufferLen,
		   const char *pcCommandString)
{
  const char *pcParameter;
  BaseType_t xParameterStringLength;
  /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
  (void) xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  /* Obtain the parameter string. */
  pcParameter = MidCLI_GetParameter (pcCommandString, /* The command string itself. */
				     1, /* Return the next parameter. */
				     &xParameterStringLength /* Store the parameter string length. */
				     /* f_unlink("SD.txt");*/);
  memset(pcWriteBuffer,0,100);
  strcpy(pcWriteBuffer,pathCurrent);
  strcat(pcWriteBuffer,pcParameter);

    if(0 == mid_DeleteFile(pcWriteBuffer))
    {
        strcpy (pcWriteBuffer, "File Deleted!\r\n");
    }
    else
    {
        sprintf (pcWriteBuffer, "Delete File Error!\r\n");
    }

    return pdFALSE;
}

static BaseType_t
prvFileMkFsCommand (char *pcWriteBuffer, size_t xWriteBufferLen,
		    const char *pcCommandString)
{

  /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */

    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    FRESULT res;

    res = f_mkfs (pathCurrent, 0, 1);/*每个簇的大小为1个扇区大小*/
    if (res != FR_OK)
    {
        sprintf (pcWriteBuffer, "Format Driver Error! %s   Parameter: %s\r\n",FR_Table[res], pathCurrent);
    }
    else
    {
        sprintf (pcWriteBuffer, "Format %s  OK!\r\n", pathCurrent);
    }

    return pdFALSE;
}

static BaseType_t prvSendAtCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

  /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */

    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    char ucEsn[50];
    char ucVersion[50];

    drv_SimQueryEsn(ucEsn);
    drv_SimQueryVersion(ucVersion);
    sprintf (pcWriteBuffer, "ok\r\n");

    return pdFALSE;
}

static BaseType_t prvPwrOffCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

  /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */

    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    
    drvSys_PwrOff();
    sprintf (pcWriteBuffer, "ok\r\n");

    return pdFALSE;
}

static BaseType_t prvFileChmodCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  (void) xWriteBufferLen;
  (void) pcCommandString;
  strcpy (pcWriteBuffer, "ChMod Not implement!\r\n");

  return pdFALSE;
}
/******************************************************************************
 * FUNCTION NAME:
 *      prvFileDownCommand
 * DESCRIPTION:
 *      下传文件到嵌入式存储设备.
 * PARAMETERS:
 *      N/A
 * RETURN:
 *      N/A
 * NOTES:
 *      N/A
 * HISTORY:
 *      2014.3.17        Arvin.Liao        Create/Update
 *****************************************************************************/

static BaseType_t prvFileDownCommand (char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    FIL *indexFp;
    uint8_t Number[10] = "          ";
    int32_t Size = 0;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    (void) pcWriteBuffer;
    configASSERT(pcWriteBuffer);

    strcpy (pcWriteBuffer,"Waiting for the file to be sent ... (press 'a' to abort)\n\r");
    mid_CliSendData ((uint8_t *) pcWriteBuffer, strlen (pcWriteBuffer));
    indexFp = mid_FileSysGetFIL (pathCurrent);/*TODO:自动判断，根据当前文件路径*/
    Size = Ymodem_Receive(yModeBuf, indexFp);

    if (Size > 0)
    {
        Int2Str (Number, Size);
        sprintf (pcWriteBuffer,"\r\n Programming Completed Successfully!\r\n Name: %s Size: %s Bytes\r\n",
                                                    fileName, Number);
    }
    else if (Size == -1)
    {
        sprintf (pcWriteBuffer,"\n\n\rThe image size is higher than the allowed space memory!");

    }
    else if (Size == -2)
    {
        sprintf (pcWriteBuffer, "\n\n\rVerification failed!\n\r");
    }
    else if (Size == -3)
    {
        sprintf (pcWriteBuffer, "\r\n\nAborted by user.\n\r");
    }
    else
    {
        sprintf (pcWriteBuffer, "\n\rFailed to receive the file!\n\r");
    }

    return pdFALSE;
}
/******************************************************************************
 * FUNCTION NAME:
 *      prvFileUpCommand
 * DESCRIPTION:
 *      通过ymodem协议进行文件上传
 * PARAMETERS:
 *      N/A
 * RETURN:
 *      N/A
 * NOTES:
 *      N/A
 * HISTORY:
 *      2014.3.17        Arvin.Liao        Create/Update
 *****************************************************************************/
static BaseType_t
prvFileUpCommand (char *pcWriteBuffer, size_t xWriteBufferLen,
		  const char *pcCommandString)
{
    uint8_t status = 0;
    FIL *indexFp;
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    FRESULT res;
    char filepath[50] = { 0 };

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    (void) pcWriteBuffer;

    /* Obtain the parameter string. */
    pcParameter = MidCLI_GetParameter (pcCommandString, /* The command string itself. */
    			     1, /* Return the next parameter. */
    			     &xParameterStringLength /* Store the parameter string length. */
    			     );
    strcpy (filepath, pathCurrent);
    strcat (filepath, pcParameter);
    //printf("%s",pcParameter);

    strcpy (pcWriteBuffer, "\n\rSelect Receive File\n\r");
    mid_CliSendData ((uint8_t *) pcWriteBuffer, strlen (pcWriteBuffer));

    indexFp = mid_FileSysGetFIL (pathCurrent);
    res = f_open (indexFp, filepath, FA_READ);
    if (res != FR_OK)
    {
      sprintf (pcWriteBuffer, "Open File  Error! %s ,File info:%s\r\n",FR_Table[res], filepath);
      return pdFALSE;
    }

    /* Transmit the flash image through ymodem protocol */
    if (mid_CliGetChar () == CRC16)
    {
        status = Ymodem_Transmit (indexFp, (const uint8_t*) pcParameter,indexFp->fsize);
        if (status != 0)
        {
            strcpy (pcWriteBuffer,"\n\rError Occurred while Transmitting File\n\r");
        }
        else
        {
            pcWriteBuffer+=sprintf (pcWriteBuffer, "\n\rFile: %s  Size:%d bytes uploaded successfully \n\r",
                                pcParameter,indexFp->fsize);
        }
    }
    else
    {
        strcpy (pcWriteBuffer, "\n\rError Input!\n\r");
    }

    f_close (indexFp);

    return pdFALSE;
}

static BaseType_t prvBleTestCommand (char *pcWriteBuffer, size_t xWriteBufferLen,
		   const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    /* Obtain the parameter string. */
    pcParameter = MidCLI_GetParameter (pcCommandString, /* The command string itself. */
                                     1, /* Return the next parameter. */
                                     &xParameterStringLength /* Store the parameter string length. */
                                     /* f_unlink("SD.txt");*/);
    memset(pcWriteBuffer,'\0',100);
    strcpy(pcWriteBuffer,pcParameter);
    
    
    /* for ble test */
    BLE_MSG_T           bleTxedMessage;
    
    if(strcmp("scan",pcWriteBuffer) == 0)
    {
        bleTxedMessage.event = BLE_START_SCAN_EVENT;
        xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);
    }
    else if(strcmp("unbind",pcWriteBuffer) == 0)
    {
        bleTxedMessage.event = BLE_UNBIND_PEERS;
        bleTxedMessage.sn = gBLEDevInfo.sn;
        xQueueSend(xQueue_Ble, &bleTxedMessage, 10/portTICK_PERIOD_MS);    
    }
    
    return pdFALSE;
}

static BaseType_t prvSendCommand (char *pcWriteBuffer, size_t xWriteBufferLen,const char *pcCommandString)
{
   const char *pcParameter;
   BaseType_t xParameterStringLength;
   
   /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
   (void) xWriteBufferLen;
   configASSERT(pcWriteBuffer);
	apl_service_event_t apl_service_event_tx;
	
	apl_service_event_tx.apl_event_id = SERVER_UPLOAD_FILE_EVENT;
	apl_service_event_tx.u32Value = (*(box_info.ptdev_band_info + 0))->sn;
	xQueueSend(xQueue_AppServer, &apl_service_event_tx, 5/portTICK_PERIOD_MS); 

   sprintf( pcWriteBuffer,"Send events,OK\r\n");

   return pdFALSE;
}


