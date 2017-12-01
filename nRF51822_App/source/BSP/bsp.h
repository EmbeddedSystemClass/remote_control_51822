/**
  ****************************************************************************************
  * @file    platform.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   platform define 
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_H__
#define BSP_H__

#include "include.h"


/* type struct define */
typedef enum
{
    MODE_INTERRUPT = 0,
    MODE_SCHEDULER
}TIME_MODE_T;


void bsp_init(void);

#endif // BSP_H__

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




