/*
 * drv_Qspi.h
 *
 *  Created on: 2015年7月30日
 *      Author: Arvin
 */

#ifndef ECGSTP7_DRIVERS_STORAGE_DRV_QSPI_H_
#define ECGSTP7_DRIVERS_STORAGE_DRV_QSPI_H_

#ifdef __cplusplus
extern "C"
  {
#endif

/* Includes ------------------------------------------------------------------*/
#include "drv_IOInit.h"
#include "includes.h"

#include "drv_n25q128a.h"

typedef enum {
    FlashOperationSuccess,
    FlashWriteRegFailed,
    FlashTimeOut,
    FlashIsBusy,
    FlashQuadNotEnable,
    FlashAddressInvalid
}ReturnMsg;

/** @addtogroup STM32746G_DISCOVERY_QSPI
 * @{
 */
HAL_StatusTypeDef drv_FlashGetAccess (void);
HAL_StatusTypeDef drv_FlashReleaseAccess (void);
HAL_StatusTypeDef drv_FlashSpiInit (void);
HAL_StatusTypeDef drv_FlashWritePage (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length);
HAL_StatusTypeDef drv_FlashEraseSector (uint32_t add);
HAL_StatusTypeDef drv_FlashRead (uint8_t * pBuffer, uint32_t readAddr, uint32_t length);
HAL_StatusTypeDef drv_FlashWritePageForNoFafts (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length);
HAL_StatusTypeDef drv_FlashReadSector (uint8_t * pBuffer, uint32_t readAddr, uint32_t length);
HAL_StatusTypeDef drv_FlashWriteSector (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length);
bool drv_FlashIsBusy (void);
uint32_t drv_FlashGetId (void);
ReturnMsg CMD_SECTOR( uint32_t flash_address );
ReturnMsg CMD_PageProg( uint32_t flash_address, void *source_address, uint32_t byte_length );
ReturnMsg CMD_READ_PAGE( uint32_t flash_address, uint8_t *target_address, uint32_t byte_length );

#endif /* ECGSTP7_DRIVERS_STORAGE_DRV_QSPI_H_ */
