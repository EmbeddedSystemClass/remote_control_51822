#ifndef _APL_WATCHDOG_H_
#define _APL_WATCHDOG_H_

#include "stm32f2xx.h"

#define WDT_TASK_MAX                    (15)

#define WDT_BAT_TASK_ID                 (0)
#define WDT_UPLOAD_TASK_ID              (1)
#define WDT_KEY_TASK_ID                 (2)
#define WDT_BLE_TASK_ID                 (3)
#define WDT_COAP_TASK_ID                 (4)


typedef struct
{
    uint32_t     CurCnt;
    uint32_t     PreCnt;
    int16_t      RunState;
    int16_t      taskID;
}WatchDog_t;

void CreateMonitorTask( void );
int8_t WDG_Init(void);

int8_t WDT_Register(uint16_t taskid);
int8_t WDT_Feed(uint16_t taskid);

#endif  // End of #ifndef _APL_WATCHDOG_H_
