/**
  ****************************************************************************************
  * @file    app_data_sync.h
  * @author  Jason
  * @version V1.0.0
  * @date    2017-3-2
  * @brief   header of app_data_sync.c
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_DATA_SYNC_H_
#define __APP_DATA_SYNC_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"


extern uint8_t              g_data_syncbuf[MAX_LEN_ONE_PACKET];
extern uint16_t             current_data_len;
extern uint16_t             need_rx_data_len;
extern uint16_t             rx_data_cnt;
extern bool                 flag_get_rx_data_len; 

/* function declare */
void reset_data_sync(void);
void app_data_sync_task_handler(void *p_event_data,uint16_t event_size);


#endif /* __APP_DATA_SYNC_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



