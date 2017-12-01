/**
  ****************************************************************************************
  * @file    bsp_led.h
  * @author  Jason
  * @version V1.0.0
  * @date    2016-12-12
  * @brief   bsp led config
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "platform.h"
#include "bsp.h"
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_gpio.h"



/* type struct define */
typedef enum
{
    LED1 = 0,
    LED2,
    LED3,
    LED4
}LED_T;



/* function declare */
void bsp_led_init(LED_T led);
void bsp_led_on(LED_T led);
void bsp_led_off(LED_T led);
void bsp_led_toggle(LED_T led);

#endif /* end of __BSP_LED_H__ */

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/




