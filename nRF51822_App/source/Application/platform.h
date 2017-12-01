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
#ifndef __PLATFORM_H_
#define __PLATFORM_H_

/* Includes ------------------------------------------------------------------*/


/* debug switch --------------------------------------------------------------*/
#define HARDFAULT_HANDLER_ENABLED	1
#define DEBUG_BLE_CONNECT
#define DEBUG_BLE_EVENT
#define DEBUG_BLE_SCAN
#define DEBUG_BLE_BONDING
#define DEBUG_BLE_PEER_DEVINFO_MANAGE
#define DEBUG_BLE_MONITOR_TEMPLATE
#define DEBUG_BLE_SYNC_DATA
#define DEBUG_SPI_RX

/* shell debug switch --------------------------------------------------------*/
#define SHELL_ENABLE
#define SHELL_BLE_ENABLE


/* gpio and buffer define for uart */
#define APP_UART_ENABLED	1
#define RETARGET_ENABLED	1
#ifndef UART_TX_BUF_SIZE
    #define UART_TX_BUF_SIZE 1024         /**< UART TX buffer size. */
#endif
#ifndef UART_RX_BUF_SIZE
    #define UART_RX_BUF_SIZE 1            /**< UART RX buffer size. */
#endif

#define TEST_PIN                    29
#define RX_PIN_NUMBER  				18    // UART RX pin number.
#define TX_PIN_NUMBER  				20    // UART TX pin number.
#define CTS_PIN_NUMBER 				10    // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 				1     // Not used if HWFC is set to false

/*spi pin define */
#define SPIS_MISO_PIN               8     // SPI MISO signal. 
#define SPIS_CSN_PIN                3     // SPI CSN signal. 
#define SPIS_MOSI_PIN               9     // SPI MOSI signal. 
#define SPIS_SCK_PIN                15    // SPI SCK signal. 

#define BT_REQN                     2     // notice the spi master to read data out

/* gpio define for leds */
#define LED_1          21
#define LED_2          22
#define LED_3          23
#define LED_4          24

/* 停止扫描定时器定义 */
#define STOP_SCAN_TIME                          APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)  //扫描5s后停止扫描
#define FIND_CHAR_STATUS_POLL_TIME              APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)   //character发现状态查询时间
#define CONNECT_BONDING_STATUS_POLL_TIME        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)   //connect and bonding状态查询时间
#define BLE_RECONNECT_TIME                      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  //reconnect control time
#define DEFAULT_REF_RSSI    -70  //默认参考蓝牙信号强度值，大于改信号质量的设备才进入扫描队列
#define MAX_SCAN_LIST_NUM   10   //最大支持扫描列表数量
#define MONITOR_TEMPLATE_DATA_MAX_LEN 		80  //监护方案最大长度
#define VERSION_INFO_BUF_LEN                20  //蓝牙Central端接收设备电量及版本信息长度
#define MAX_LEN_ONE_PACKET                  250 //数据同步过程中，一包数据最大长度
#define MAX_SPI_LEN                         255 //STM32与nRF51822之间SPI通讯，一包数据最大长度

/*  Define the max event size and event queue size      */
#define SCHED_MAX_EVENT_DATA_SIZE           128
#define SCHED_QUEUE_SIZE                    40  /**< Maximum number of events in the scheduler queue. */


#endif /* __PLATFORM_H_ */


/************************ (C) COPYRIGHT Chengdu CloudCare Healthcare Co., Ltd. *****END OF FILE****/


