/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "fatfs.h"
#include "includes.h"

#define FILE_COPY_MAX_LEN               (512)

extern Diskio_drvTypeDef NorFlash_Driver;
FATFS qSPIFatFs; /* File system objects logical drives */
static FIL NorFlashFile; /* File objects */
TCHAR norFlashPath[4]; /* RAM disk and SD card logical drives paths */

void MX_FATFS_Init(void)
{
    FRESULT res = FR_OK;
    DIR dir;

    /*## FatFS: Link the USER driver ###########################*/
    if ((FATFS_LinkDriver (&NorFlash_Driver, norFlashPath) == 0))
    {
        /*##-2- Register the file system object to the FatFs module ##############*/
        if(f_mount(&qSPIFatFs, (TCHAR const*) norFlashPath, 0) != FR_OK)
        {
        }
        else
        {
            #ifdef FATFS_DEBUG
                printf("norFlashPath = %s\r\n",norFlashPath);
            #endif
           	res = f_opendir(&dir, norFlashPath);
          	if(res != FR_OK)
            {
                res = f_mkfs ((TCHAR const*) norFlashPath, 0, 0);
                if ((res != FR_OK))
                {
                    #ifdef FATFS_DEBUG
                        printf("ExtFlash,f_mkfs,Error,%d\r\n",res);
                    #endif
                }
            }
            #ifdef FATFS_DEBUG
           	else
            {
                printf("\r\nHas been MKFS!!!\r\n");
            }
            #endif
        }

    }
}

void mid_FileSysFormat (char *driverPath)
{
    FRESULT res1;
    /*##-3- Create a FAT file system (format) on the logical drives ########*/
    /* WARNING: Formatting the uSD card will delete all content on the device */
    res1 = f_mkfs ((TCHAR const*) driverPath, 0, 0);
    if ((res1 != FR_OK))
    {
      /* FatFs Format Error */
      usr_self_check_failed((uint8_t *)__FILE__, __LINE__);
    }
}

/**
 * @brief  根据存储介质类型返回当前路径
 * @param  drvType  存储介质类型
 * @retval FIL * 当前存储介质类型所对应的路径
 */
TCHAR * mid_FileGetDrvPath (FILE_DRIVER_T drvType)
{
    TCHAR *returnFil = NULL;

    switch (drvType)
    {
        case NORFLASH:
            returnFil = norFlashPath;
            break;

        default:
            break;
    }

    return returnFil;
}
/**
 * @brief   返回当前路径所对应的文件结构
 * @param  path  当前的文件路径
 * @retval FIL * 当前路径所对应的文件结构指针
 */
FIL * mid_FileSysGetFIL (char *path)
{
    FIL *returnFil = NULL;

    if (memcmp (path, norFlashPath, 2) == 0)
    {
        returnFil = &NorFlashFile;
    }

    return returnFil;
}

/**
 * @brief  根据存储介质类型返回当前路径所对应的文件结构
 * @param  drvType  存储介质类型
 * @retval FIL * 当前存储介质类型所对应的文件结构指针
 */
FIL *mid_FileSysGetFILByType (FILE_DRIVER_T drvType)
{
    FIL *returnFil = NULL;

    switch (drvType)
    {
        case NORFLASH:
            returnFil = &NorFlashFile;
            break;

        default:
            returnFil = NULL;
            break;
    }

    return returnFil;
}

bool mid_FileSysWrite (FIL *file, const char* fileName, bool newFile, char *pdata, uint16_t dataByteSize)
{
  FRESULT res1;
  uint32_t byteswritten1;
  BYTE mode = newFile ? FA_CREATE_ALWAYS | FA_WRITE : FA_WRITE;
  res1 = f_open (file, fileName, mode);

  if (res1 != FR_OK)
    {
      /* Open for write Error */
      return FALSE;
    }
  else
    {
      /* Write data to the text files ###############################*/
      res1 = f_write (file, pdata, dataByteSize, (void *) &byteswritten1);

      if ((byteswritten1 == 0) || (res1 != FR_OK))
	{
	  /*file Write or EOF Error */
	  return FALSE;
	}
      else
	{
	  /* Close the open text files ################################*/
	  f_close (file);
	}
    }
  return TRUE;
}

bool mid_FileSysRead (FIL *file, const char* fileName, char *pdata, uint16_t dataByteSize)
{
  FRESULT res1;
  uint32_t bytesread1;
  /*Open the text files object with read access ##############*/
  res1 = f_open (file, fileName, FA_READ);

  if ((res1 != FR_OK))
    {
      /* file Open for read Error */
      return FALSE;
    }
  else
    {
      /*##-8- Read data from the text files ##########################*/
      res1 = f_read (file, pdata, dataByteSize, (UINT*) &bytesread1);

      if ((res1 != FR_OK))
	{
	  /*  file Read or EOF Error */
	  return FALSE;
	}
      else
	{
	  /*##-9- Close the open text files ############################*/
	  f_close (file);

	}
    }
  return TRUE;
}

int8_t mid_DeleteFile(const TCHAR* path)
{
    FRESULT res;
    int8_t ret = 0;

    /* Check parameters */
    usr_para_assert(path != NULL);

    printf("\r\n To delete file: %s\r\n",path);
    res = f_unlink(path);
    if(FR_OK != res)
    {
//        printf("Unlink File,Failed,res = %d!!!\r\n",res);
        ret = -1;
    }
    else
    {
        ret = 0;
    }

    return (ret);
}

void MX_FATFS_Init_test(void)
{
    FRESULT res = FR_OK;
    char * Path = "stm32.txt";
    uint8_t tmp[256] = {0};
    uint8_t tmp_rd[256] = {0};
    uint32_t wr_len = 0;
    uint32_t rd_len = 0;
    char text_name[256];

    memset(tmp,0,sizeof(tmp));
    memset(tmp_rd,0,sizeof(tmp_rd));
    printf("Now Init the SRAM......\r\n");
    /*## FatFS: Link the SRAMDISK driver ###########################*/
    if ((FATFS_LinkDriver (&NorFlash_Driver, norFlashPath) == 0))
    {
        res = f_mount(&qSPIFatFs, (TCHAR const*) norFlashPath, 0);
        if(res == FR_OK)
        {
            printf("UserFlash,f_mount,OK,%s\r\n",norFlashPath);
            res = f_mkfs(norFlashPath, 0, 0);
            if(res == FR_OK)
            {
                printf("UserFlash,f_mkfs,OK\r\n");

                memset(text_name,0,sizeof(text_name));
                strcpy(text_name, Path);
                res = f_open(&NorFlashFile,text_name,FA_CREATE_ALWAYS|FA_WRITE);
                if(res != FR_OK)
                {
                    printf("UserFlash,Open file,Error,%d\r\n",res);
                }

                for(int i = 0;i < 256;i ++)
                {
                    tmp[i] = i;
                }

                for(int i = 0;i < 256;i ++)
                {
                    printf("0x%02x, ",tmp[i]);
                }
                printf("\r\n");

                res = f_write(&NorFlashFile, tmp, 256, &wr_len);
                if(res != FR_OK)
                {
                    printf("UserFlash,Write file,Error,%d\r\n",res);
                }
                else
                {
                    printf("UserFlash,Write file,OK,%d\r\n",wr_len);
                }

                f_close(&NorFlashFile);

                res = f_open(&NorFlashFile,text_name,FA_WRITE|FA_READ);
                if(res != FR_OK)
                {
                    printf("UserFlash,Open file,Error,%d\r\n",res);
                }

                res = f_read(&NorFlashFile, tmp_rd, 256, &rd_len);
                if(res != FR_OK)
                {
                    printf("UserFlash,Read file,Error,%d\r\n",res);
                }
                else
                {
                    printf("UserFlash,Read file,OK,%d\r\n",rd_len);
                }

                for(int i = 0;i < 256;i ++)
                {
                    printf("0x%02x, ",tmp_rd[i]);
                }
                printf("\r\n");
            }
            else
            {
                printf("UserFlash,f_mkfs,Error,%d\r\n",res);
            }
        }
        else
        {
            printf("UserFlash,f_mount,ERROR,%d\r\n",res);
        }
    }
    /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
