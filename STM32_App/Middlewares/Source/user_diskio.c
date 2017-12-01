/**
 ******************************************************************************
 * @file    user_diskio.c
 * @brief   This file includes a diskio driver skeleton to be completed by the user.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "drv_FlashSpi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Block Size in Bytes*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
//#define FLASH_FATFS_START_ADDR                    0x0003E000      /* Fatfs START ADDR */
#define FLASH_FATFS_START_ADDR                    0      /* Fatfs START ADDR */
/* Private function prototypes -----------------------------------------------*/

DSTATUS USER_initialize (BYTE);
DSTATUS USER_status (BYTE);
DRESULT USER_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT USER_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE, BYTE, void*);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef NorFlash_Driver =
{ 
    USER_initialize, 
    USER_status, 
    USER_read,
    #if  _USE_WRITE
    USER_write,
    #endif  /* _USE_WRITE == 1 */  
    #if  _USE_IOCTL == 1
    USER_ioctl,
    #endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes a Drive
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS USER_initialize (BYTE pdrv /* Physical drive nmuber to identify the drive */
)
{
    Stat = STA_NOINIT;

    if(drv_FlashSpiInit ()==HAL_OK)
    {
        Stat &= ~STA_NOINIT;
    }

    return Stat;
}

/**
 * @brief  Gets Disk Status
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS USER_status (BYTE pdrv /* Physical drive nmuber to identify the drive */
)
{
    Stat = STA_NOINIT;
    
    Stat &= ~STA_NOINIT;
    
    return Stat;
}


/**
 * @brief  Reads Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  *buff: Data buffer to store read data
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to read (1..128)
 * @retval DRESULT: Operation result
 */
DRESULT USER_read (BYTE lun, /* Physical drive nmuber to identify the drive */
                   BYTE *buff, /* Data buffer to store read data */
                   DWORD sector, /* Sector address in LBA   扇区地址：实际地址乘以扇区大小*/
                   UINT count /* Number of sectors to read */
                   )
{
    DRESULT status = RES_ERROR;
    uint32_t read_addr = 0;
    
//    printf("USER_read,sector = 0x%08x,count = %d\r\n",sector,count);
    if (drv_FlashGetAccess () == HAL_OK)
    {
        read_addr = sector * N25Q128A_SECTOR_SIZE + FLASH_FATFS_START_ADDR;
        if(drv_FlashRead (buff, read_addr, (uint32_t)(count * N25Q128A_SECTOR_SIZE)) != HAL_OK)
        {
            status = RES_ERROR;
        }
        else
        {
            status = RES_OK;
        }
        
        drv_FlashReleaseAccess ();
    }

    return status;
}


/**
 * @brief  Writes Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  *buff: Data to be written
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to write (1..128)
 * @retval DRESULT: Operation result
 */
#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, /* Physical drive nmuber to identify the drive */
	    const BYTE *buff, /* Data to be written */
	    DWORD sector, /* Sector address in LBA */
	    UINT count /* Number of sectors to write */
	    )
{
    DRESULT status=RES_ERROR;
    uint32_t write_addr = 0;
    
//    printf("USER_write,sector = 0x%08x,count = %d\r\n",sector,count);
    if (drv_FlashGetAccess () == HAL_OK)
    {
        write_addr = sector * N25Q128A_SECTOR_SIZE + FLASH_FATFS_START_ADDR;
        if (drv_FlashWritePage ((uint8_t *) buff, write_addr, count << 12) != HAL_OK)
        {
            status = RES_ERROR;
        }
        else
        {
            status = RES_OK;
        }
        
        drv_FlashReleaseAccess ();
    }

    return status;
}
#endif /* _USE_WRITE == 1 */

/**
 * @brief  I/O control operation
 * @param  pdrv: Physical drive number (0..)
 * @param  cmd: Control code
 * @param  *buff: Buffer to send/receive control data
 * @retval DRESULT: Operation result
 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, /* Physical drive nmuber (0..) */
                    BYTE cmd, /* Control code */
                    void *buff /* Buffer to send/receive control data */
                    )
{
    DRESULT res = RES_ERROR;
    
//    printf("USER_ioctl,cmd = %d,Stat = %d\r\n",cmd,Stat);
    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    switch (cmd)
    {
        /* Make sure that no pending write process */
        case CTRL_SYNC:
            res = RES_OK;
            break;

        /* Get number of sectors on the disk (DWORD) */
        case GET_SECTOR_COUNT:
            *(DWORD*) buff = (N25Q128A_FLASH_SIZE - FLASH_FATFS_START_ADDR)/N25Q128A_SECTOR_SIZE;
            res = RES_OK;
            break;

        /* Get R/W sector size (WORD) */
        case GET_SECTOR_SIZE:
            *(WORD*) buff = N25Q128A_SECTOR_SIZE;
            res = RES_OK;
            break;

        /* Get erase block size in unit of sector (DWORD) */
        case GET_BLOCK_SIZE:
            *(WORD*) buff = N25Q128A_SECTOR_SIZE;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
            break;
    }

    return res;
}

#endif /* _USE_IOCTL == 1 */

uint8_t test_data_buff[4096];
uint8_t test_data_buff_read[4096];
void tese_nor(void)
{
    printf("tese_nor\r\n");
    NorFlash_Driver.disk_initialize(1);
    
    if(NorFlash_Driver.disk_status(1)!=STA_NOINIT)
    {
        vTaskDelay(1000);
    }
    
    for(uint16_t i=0;i<4096;i++)
    {
        test_data_buff[i] =i+1;
        test_data_buff_read[i] =0;
    }
    
    if(NorFlash_Driver.disk_write(1,test_data_buff,128,1)!=RES_OK)
    {
        printf("nor flash write error\r\n");
    }
    
    if(NorFlash_Driver.disk_read(1,test_data_buff,128,1)!=RES_OK)
    {
        printf("nor flash rESD error\r\n");
    }
}
