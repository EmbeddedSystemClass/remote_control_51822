#ifndef _INCLUDES_H_
#define _INCLUDES_H_

// Include std header files
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <stdlib.h>
#include <time.h>


// Include OS header files
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

// Include CoAP header files
#include "coap.h"
#include "lobaro-coap_esp8266.h"

// Include File system header files
#include "fatfs.h"
#include "ff.h"			        /* Declarations of FatFs API */
#include "ff_gen_drv.h"
#include "diskio.h"		        /* Declarations of disk I/O functions */

// Include system header files
#include "stm32f2xx_hal.h"
#include "TypePorting.h"
#include "ymodem.h"
#include "dataconvert.h"

// Include driver header files
#include "drv_IOInit.h"
#include "drv_led.h"
#include "drv_SpiComDef.h"
#include "drv_FlashSpi.h"
#include "drv_cdma.h"
#include "drv_rtc.h"
#include "drv_CdmaUart.h"
#include "drv_sys.h"
#include "lobaro-coap_esp8266.h"
#include "crc16.h"

// Include middleware header files
#include "mid_cli.h"
#include "mid_net.h"
#include "main.h"


// Include application header files
#include "apl_cli.h"
#include "apl_watchdog.h"
#include "apl_key.h"
#include "apl_service.h"
#include "apl_monitor_template.h"
#include "apl_ble.h"
#include "apl_bat.h"
#include "ble_communication.h"

#ifndef TRUE
    #define TRUE  1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

/* Define the debug option */
#define Debug

#ifdef Debug

//#define FATFS_DEBUG
//#define BAT_DEBUG
//#define WDG_ENABLE
#define SYN2G_ENABLE
#define BLE_DATA_SYNC_DEBUG
#define BLE_CONNECT_DEBUG
#endif      // End of #ifdef Debug

#define tskBATTERY_PRIORITY			( ( UBaseType_t ) 1U )
#define tskKEY_PRIORITY			    ( ( UBaseType_t ) 2U )
#define tskCLI_PRIORITY			    ( ( UBaseType_t ) 5U )
#define tskBLE_PRIORITY		        ( ( UBaseType_t ) 6U )
#define tskSERVER_PRIORITY		    ( ( UBaseType_t ) 7U )
#define tskCOAP_PRIORITY			( ( UBaseType_t ) 8U )
#define SIM900_RX_THREAD_PRIO		( ( UBaseType_t ) 9U )
#define tskMONITOR_PRIORITY			( ( UBaseType_t ) 10U )


/* Definition for LEDs */
#define LED0                                (0)
#define LED1                                (1)
#define LED2                                (2)
#define LED3                                (3)
#define LED4                                (4)

/* exit priority define */
#define EXTI0_IRQn_LINE_PRI    		        (6)



#define USR_ASSERT
#define USR_CHECK

/* Exported macro ------------------------------------------------------------*/
#ifdef USR_ASSERT
    #define usr_para_assert(expr) ((expr) ? (void)0 : usr_assert_failed((uint8_t *)__FILE__, __LINE__))
    /* Exported functions ------------------------------------------------------- */
    void usr_assert_failed(uint8_t* file, uint32_t line);
#else
    #define usr_para_assert(expr) ((void)0)
#endif 	/* End of #ifdef USR_ASSERT */


/* Exported macro ------------------------------------------------------------*/
#ifdef USR_CHECK
    #define usr_res_assert(expr) ((expr) ? (void)0 : usr_res_assert_failed((uint8_t *)__FILE__, __LINE__))
    /* Exported functions ------------------------------------------------------- */
    void usr_res_assert_failed(uint8_t* file, uint32_t line);
#else
    #define usr_res_assert(expr) ((void)0)
#endif 	/* End of #ifdef USR_ASSERT */


#define usr_self_check(expr) ((expr) ? (void)0 : usr_self_check_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void usr_self_check_failed(uint8_t* file, uint32_t line);


#endif /* End of #ifndef __INCLUDES_H */

