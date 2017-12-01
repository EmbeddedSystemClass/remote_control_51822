/**
  ****************************************************************************************
  * @file    bsp.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-11-25
  * @brief   bsp
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Private variables ---------------------------------------------------------*/
static void timers_init(TIME_MODE_T mode);


const char Dev_Msg[] =
	"/*********************  This is to be Done ********************/\r\n"
    "/*                                                            */\r\n"
    "/*                                                            */\r\n"
    "/*                  BSP Init Complate,Start...                */\r\n"
#ifdef SHELL_ENABLE
    "/*                  Pls enter help for more usages            */\r\n"
#endif
	"\r\n";
/**
  * @brief  bsp init 
  * @note   This function is called  automatically at the beginning of program 
  * @param  None
  * @retval None
  */
void bsp_init(void)
{
	/* uart init */
	uart_init();
    bsp_spi_slave_config();
    
    /* app timers init */
    timers_init(MODE_INTERRUPT);
    
    /* shell_init */
    shell_init();
	    
    printf("%s",Dev_Msg);
}
/**
  * @brief  Function for initializing the timer module.
  * @param  None
  * @retval None
  */
static void timers_init(TIME_MODE_T mode)
{
    if(mode == MODE_INTERRUPT)
    {
        /* app timer init, calling it directly when timeout(interrupt mode) */
        APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    }
    else if(mode == MODE_SCHEDULER)
    {
        // Initialize timer module, making it use the scheduler.
        APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);        
    }
    
    /* 分配timer控制扫描时间 */
    ble_scan_control_timer_init();
    
    /* 分配timer用于查询绑定连接过程状态 */
    ble_connect_bonding_status_polling_timer_init();
    
    /* 分配timer用于蓝牙断线重连 */
    ble_reconnect_timer_init();
}


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/

