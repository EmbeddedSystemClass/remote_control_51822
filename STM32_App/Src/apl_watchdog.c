#include "apl_watchdog.h"
#include "includes.h"

TaskHandle_t xMonitorTaskHandle = NULL;
IWDG_HandleTypeDef IwdgHandle;
//QueueHandle_t xQueue;
SemaphoreHandle_t xSemaphore_wdt = NULL;
WatchDog_t tWatchDog[WDT_TASK_MAX] = {0};


int8_t WDG_Init(void)
{
    memset(&tWatchDog, 0, sizeof(WatchDog_t));

    IwdgHandle.Instance = IWDG;
    IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
    IwdgHandle.Init.Reload    = 4095;
    if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int8_t WDG_Feed(void)
{
    int8_t sta = 0;

    #ifdef WDG_ENABLE
//    printf("WDG_Feed\r\n");
    if(HAL_OK == HAL_IWDG_Refresh(&IwdgHandle))
    {
        sta = 0;
    }
    else
    {
        sta = -1;
    }
    #endif

    return (sta);
}

int8_t WDT_Register(uint16_t taskid)
{
    int8_t ret = 0;

    #ifdef WDG_ENABLE
    if(taskid < WDT_TASK_MAX)
    {
        if(pdTRUE == xSemaphoreTake(xSemaphore_wdt, 10/portTICK_PERIOD_MS))
        {
            if(0 == tWatchDog[taskid].RunState)
            {
                tWatchDog[taskid].RunState = 1;
                ret = 0;
            }
            else
            {
                ret = -1;
            }

            xSemaphoreGive(xSemaphore_wdt);
        }
        else
        {
            ret = -1;
        }
    }
    else
    {
        ret = -1;
    }
    #endif

    return ret;
}

int8_t WDT_UnRegister(uint16_t taskid)
{
    int8_t ret = 0;

    #ifdef WDG_ENABLE
    if(taskid < WDT_TASK_MAX)
    {
        if(pdTRUE == xSemaphoreTake(xSemaphore_wdt, 10/portTICK_PERIOD_MS))
        {
            if(1 == tWatchDog[taskid].RunState)
            {
                tWatchDog[taskid].RunState = 0;
                ret = 0;
            }
            xSemaphoreGive(xSemaphore_wdt);
        }
    }
    #endif

    return ret;
}

int8_t WDT_Feed(uint16_t taskid)
{
    int8_t ret = 0;

    #ifdef WDG_ENABLE
//    printf("\r\nWDT_Feed,%d\r\n",taskid);
    if(taskid < WDT_TASK_MAX)
    {
        if(pdTRUE == xSemaphoreTake(xSemaphore_wdt, 10/portTICK_PERIOD_MS))
        {
            tWatchDog[taskid].CurCnt++;
            xSemaphoreGive(xSemaphore_wdt);
        }
        else
        {
            ret = -1;
        }
    }
    else
    {
        ret = -1;
    }
    #endif

    return ret;
}

int8_t MonitorCheck(uint16_t taskid )
{
    int8_t sta = 0;

    #ifdef WDG_ENABLE
    if(1 == tWatchDog[taskid].RunState)
    {
//        printf("tWatchDog[%d].runsta = %d,tWatchDog[taskid].PreCnt = %d,tWatchDog[taskid].CurCnt = %d\r\n",
//                taskid,tWatchDog[taskid].RunState,tWatchDog[taskid].PreCnt,tWatchDog[taskid].CurCnt);
        if(tWatchDog[taskid].PreCnt == tWatchDog[taskid].CurCnt)
        {
            sta = -1;
        }
        else
        {
            tWatchDog[taskid].PreCnt = tWatchDog[taskid].CurCnt;
        }
    }
    #endif

    return (sta);
}

int8_t WDT_Check(void)
{
    int8_t ret = 0;

    #ifdef WDG_ENABLE
    if(pdTRUE == xSemaphoreTake(xSemaphore_wdt, 10/portTICK_PERIOD_MS))
    {
        for(uint8_t i = 0;i < WDT_TASK_MAX;i ++)
        {
            if(-1 == MonitorCheck(i))
            {
                ret = -1;
                break;
            }
        }
        xSemaphoreGive(xSemaphore_wdt);
    }
    #endif

    return ret;
}


void vMonitorTask(void * pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10000/portTICK_PERIOD_MS;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount ();

    #ifdef WDG_ENABLE
    WDG_Init();
    #endif

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        #ifdef WDG_ENABLE
        if(0 == WDT_Check())
        {
            if(0 == WDG_Feed())
            {
//                printf("FEED OK\r\n");
            }
        }
        #endif
    }
}

// Function that creates a task.
void CreateMonitorTask( void )
{
    // Create a queue capable of containing 10 uint32_t values.
	xSemaphore_wdt = xSemaphoreCreateMutex();
    configASSERT( xSemaphore_wdt );

    xTaskCreate( vMonitorTask, "MonitorTask", configMINIMAL_STACK_SIZE * 5, NULL, tskMONITOR_PRIORITY, &xMonitorTaskHandle );
    usr_para_assert( xMonitorTaskHandle!= NULL );
}

