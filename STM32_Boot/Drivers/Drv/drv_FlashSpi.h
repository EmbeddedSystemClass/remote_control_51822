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
//#include "../drv_IOInit.h"

#ifdef NOR_FLASH_PLUG_IN
#include "drv_n25q128a.h"

/** @addtogroup STM32746G_DISCOVERY_QSPI
 * @{
 */
HAL_StatusTypeDef drv_FlashSpiInit (void);
HAL_StatusTypeDef drv_FlashWritePage (uint8_t * pBuffer, uint32_t writeAddr, uint32_t length);

HAL_StatusTypeDef drv_FlashRead (uint8_t * pBuffer, uint32_t readAddr, uint32_t length);
BOOL drv_FlashIsBusy (void);
u32 drv_FlashGetId (void);
#endif
#endif /* ECGSTP7_DRIVERS_STORAGE_DRV_QSPI_H_ */
