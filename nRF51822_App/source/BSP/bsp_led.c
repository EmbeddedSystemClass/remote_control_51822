/**
  ****************************************************************************************
  * @file    bsp_uart.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   bsp uart
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
#include "include.h"


/**
  * @brief  bsp_led_init 
  * @note   led init  
  * @param  led
  * @retval None
  */
void bsp_led_init(LED_T led)
{
    switch(led)
    {
        case LED1:
        {
            nrf_gpio_cfg_output(LED_1);
        }
        break;
        case LED2:
        {
            nrf_gpio_cfg_output(LED_2);
        }
        break;
        case LED3:
        {
            nrf_gpio_cfg_output(LED_3);
        }
        break;
        case LED4:
        {
            nrf_gpio_cfg_output(LED_4);
        }
        break;
        default:break;
    }
}

/**
  * @brief  bsp_led_off 
  * @note   led off 
  * @param  led
  * @retval None
  */
void bsp_led_off(LED_T led)
{
    switch(led)
    {
        case LED1:
        {
            nrf_gpio_pin_set(LED_1);
        }
        break;
        case LED2:
        {
            nrf_gpio_pin_set(LED_2);
        }
        break;
        case LED3:
        {
            nrf_gpio_pin_set(LED_3);
        }
        break;
        case LED4:
        {
            nrf_gpio_pin_set(LED_4);
        }
        break;
        default:break;
    }    
}

/**
  * @brief  bsp_led_on
  * @note   led on
  * @param  led
  * @retval None
  */
void bsp_led_on(LED_T led)
{
    switch(led)
    {
        case LED1:
        {
            nrf_gpio_pin_clear(LED_1);
        }
        break;
        case LED2:
        {
            nrf_gpio_pin_clear(LED_2);
        }
        break;
        case LED3:
        {
            nrf_gpio_pin_clear(LED_3);
        }
        break;
        case LED4:
        {
            nrf_gpio_pin_clear(LED_4);
        }
        break;
        default:break;
    }    
}
/**
  * @brief  bsp_led_toggle 
  * @note   led toggle
  * @param  led
  * @retval None
  */
void bsp_led_toggle(LED_T led)
{
    switch(led)
    {
        case LED1:
        {
            nrf_gpio_pin_toggle(LED_1);
        }
        break;
        case LED2:
        {
            nrf_gpio_pin_toggle(LED_2);
        }
        break;
        case LED3:
        {
            nrf_gpio_pin_toggle(LED_3);
        }
        break;
        case LED4:
        {
            nrf_gpio_pin_toggle(LED_4);
        }
        break;
        default:break;
    }    
}





/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

