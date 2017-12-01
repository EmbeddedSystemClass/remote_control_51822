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
 *   drv_SpiCmuEPPG.h
 * DESCRIPTION:
 *   
 * HISTORY:
 *   2015年10月15日        Arvin         Create/Update
 *
*****************************************************************************/
#ifndef ECGPB_DRIVERS_DRV_SPICMUEPPG_H_
#define ECGPB_DRIVERS_DRV_SPICMUEPPG_H_

#include "includes.h"
#include <stdbool.h>
#include "drv_IOInit.h"

#define SPI_COM_TIME_OUT      		1000 			//ms
typedef struct
{
    uint32_t Mode;               /*!< Specifies the SPI operating mode.
                                      This parameter can be a value of @ref SPI_mode */

    uint32_t DataSize;          /*!< Specifies the SPI Directional mode state.
                                      This parameter can be a value of @ref SPI_Direction_mode */


    uint32_t CLKPolarity;        /*!< Specifies the serial clock steady state.
                                      This parameter can be a value of @ref SPI_Clock_Polarity */

    uint32_t CLKPhase;           /*!< Specifies the clock active edge for the bit capture.
                                      This parameter can be a value of @ref SPI_Clock_Phase */
} SPI_COMInitTypeDef;


typedef struct
{
  HAL_StatusTypeDef       (*Init)(SPI_COMInitTypeDef*);
  HAL_StatusTypeDef       (*Write)( uint8_t *, uint16_t );
  HAL_StatusTypeDef       (*Read)( uint8_t *, uint16_t );
  uint8_t       			     (*WriteRead)( uint8_t );
  HAL_StatusTypeDef	     (*GetAccess)(void);
  HAL_StatusTypeDef       (*ReleaseAccess)(void);
  bool		 	    (*IsInit)(void);
} SPI_COM_DrvTypeDef;


extern SPI_COM_DrvTypeDef Spi3Drv;
extern SPI_COM_DrvTypeDef Spi2Drv;
#endif /* ECGPB_DRIVERS_DRV_SPICMUEPPG_H_ */
