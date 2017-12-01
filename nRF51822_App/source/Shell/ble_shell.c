/**
  ****************************************************************************************
  * @file    SHELL_BLE_ENABLE.c
  * @author  Jason
  * @version V1.0.0
  * @date    2016-12-30
  * @brief   shell control for ble event
  ****************************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 Chengdu CloudCare Healthcare Co., Ltd.</center></h2>
  *
  ****************************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "shell.h"          //Shell

/*---------------------* 
*     Shell版本判断
*----------------------*/
#ifdef SHELL_VER
  #if (SHELL_VER < 2)
    #error "shell版本太低"
  #endif
#else
    #error "未找到Shell文件，或Shell版本信息"
#endif


#ifdef SHELL_BLE_ENABLE
  extern void Shell_BLE_Service(void);
#else
  void Shell_BLE_Service(void){;}
#endif

#ifdef SHELL_BLE_ENABLE
/*---------------------* 
*       
*----------------------*/
//命令帮助文件
const char BLE_HelpMsg[] =
	"[BLE contorls]\r\n"
	" ble help\t\t- help.\r\n"
	" ble start scan\t\t- start ble scan.\r\n"
    " ble stop scan\t\t- stop ble scan.\r\n"
    " ble disconnect\t\t- disconnect the peer device.\r\n"
	"\r\n";
	
  
/**
  * @brief  Shell_BLE_Service 
  * @note   BLE shell service
  * @param  None
  * @retval None
  */
void Shell_BLE_Service(void)
{
    uint8_t                 *ptRxd;         //用于接收指令处理  
    uint8_t                 i;
    BLE_MSG_T               bleEventMsgValue;
    uint32_t                err_code = NRF_ERROR_NULL;

    //指令初级过滤  
    //--------------------------------------------------------------------------  
    //格式:<->[cmd bytes]<CRLF>  即"-[cmd bytes]\r\n"  
    //指令必须以"-"开始, 以"\r\n"结束 
    i = shell_rx_rdy;
    if( (i < 2) || ('\r' != shell_rx_buff[i-2]) || ('\n' != shell_rx_buff[i-1]))return;
    
    //长度和前缀过滤 
    ptRxd = (uint8_t *)shell_rx_buff;
    if( (' ' != shell_rx_buff[3]) || ('b' != shell_rx_buff[0]) || (i < 6) || 
        ('l' != shell_rx_buff[1]) || ('e' != shell_rx_buff[2]) )  return;
       
    //处理指令
    //--------------------------------------------------------------------------
    ptRxd += 4;
    if(StrComp(ptRxd,"start scan"))    //按包读取指令
    {
        bleEventMsgValue.eventID = EVENT_APP_BLE_START_SCAN;
        err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
        APP_ERROR_CHECK(err_code);         
    }
    else if(StrComp(ptRxd,"stop scan")) //停止蓝牙扫描
    {
        bleEventMsgValue.eventID = EVENT_APP_BLE_STOP_SCAN;
        err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
        APP_ERROR_CHECK(err_code);          
    }
    else if(StrComp(ptRxd ,"disconnect")) //断开蓝牙连接
    {
        bleEventMsgValue.eventID = EVENT_APP_BLE_DISCONNECT;
        err_code = app_sched_event_put(&bleEventMsgValue,sizeof(bleEventMsgValue),ble_task_handler);
        APP_ERROR_CHECK(err_code);     
    }
    else if(StrComp(ptRxd,"help\r\n"))      //
    {
        printf("%s",BLE_HelpMsg);
    }
    else return;
    
    //退出处理
    //--------------------------------------------------------------------------
    shell_rx_rdy = 0;  //shell_rx_rdy为0,表示指令已被处理完毕,否者下个Shell服务继续调用
}
#endif

/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/



