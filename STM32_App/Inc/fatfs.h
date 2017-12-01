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
 *   mid_FileSys.h
 * DESCRIPTION:
 *   
 * HISTORY:
 *   2015年8月10日        Arvin         Create/Update
 *
*****************************************************************************/
#ifndef ECGSTP7_MIDDLEWARES_FILESSYS_MID_FILESYS_H_
#define ECGSTP7_MIDDLEWARES_FILESSYS_MID_FILESYS_H_

#include "ff.h"


#define FILE_NAME_LENGTH   10

typedef enum
{
    NORFLASH,
    SDCARD,
    SDRAM
}FILE_DRIVER_T;

char * mid_FileGetDrvPath (FILE_DRIVER_T drvType);
FIL * mid_FileSysGetFILByType (FILE_DRIVER_T drvType);
HAL_StatusTypeDef mid_FileSysInit (void);
FIL * mid_FileSysGetFIL (char *path);
FRESULT mid_FileSysMount(TCHAR const *path);
FRESULT mid_FileSysUnMount(TCHAR const *path);
int8_t mid_DeleteFile(const TCHAR* path);
void MX_FATFS_Init(void);
void MX_FATFS_Init_test(void);
#endif /* ECGSTP7_MIDDLEWARES_FILESSYS_MID_FILESYS_H_ */
