/**
  ****************************************************************************************
  * @file    bsp_spis.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-14
  * @brief   header of bsp_spis.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SPIS_H__
#define __BSP_SPIS_H__

#include "include.h"
#include "nrf_drv_spis.h"


/* function declare */
void bsp_spi_slave_config(void);
ret_code_t bsp_spi_slave_transmit(uint8_t *data,uint8_t len);



#endif /* end of __BSP_SPIS_H__ */

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




